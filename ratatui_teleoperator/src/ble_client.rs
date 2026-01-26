use bluer::{Adapter, Device, Session, Uuid, AdapterEvent};
use futures::{pin_mut, StreamExt};
use serde::{Deserialize, Serialize};
use std::time::Duration;
use tokio::time::sleep;
use tokio::sync::mpsc;

const TARGET_DEVICE_NAME: &str = "RustyRover"; 
const SERVICE_UUID: Uuid = Uuid::from_u128(0x1deaebe7_ce65_4d57_8933_1bdc2065f37b);
const CHARACTERISTIC_UUID: Uuid = Uuid::from_u128(0x9dd2899d_f3c9_47ee_992a_aad14b2cdaaf);

// Data to send over Bluetooth
#[derive(Serialize, Deserialize, Debug)]
pub struct RobotCommand {
    #[serde(rename = "type")]
    pub cmd_type: String,
    pub data: String,
}

#[derive(Debug)]
pub enum ToBle {
    Connect,
    SendJson(RobotCommand),
}

// Events from BLE Worker -> TUI
#[derive(Debug)]
pub enum FromBle {
    StatusChange(String),
    DataReceived(String),
}

// --- 3. THE WORKER ---

pub async fn run_ble_worker(
    mut cmd_rx: mpsc::UnboundedReceiver<ToBle>, 
    event_tx: mpsc::UnboundedSender<FromBle>
) {
    let _ = event_tx.send(FromBle::StatusChange("Initializing...".into()));

    // Create session to BlueZ
    let session = match Session::new().await {
        Ok(s) => s,
        Err(e) => {
            let _ = event_tx.send(FromBle::StatusChange(format!("BlueZ Error: {}", e)));
            return;
        }
    };

    // Connecting to BLE adapter
    let adapter = match session.default_adapter().await {
        Ok(a) => a,
        Err(e) => {
            let _ = event_tx.send(FromBle::StatusChange(format!("Adapter Error: {}", e)));
            return;
        }
    };

    let _ = adapter.set_powered(true).await;
    let _ = event_tx.send(FromBle::StatusChange("Ready to Connect".into()));

    loop {
        if let Some(cmd) = cmd_rx.recv().await {
            match cmd {
                ToBle::Connect => {
                    let _ = event_tx.send(FromBle::StatusChange("Scanning...".into()));
                    
                    match find_and_connect(&adapter, &event_tx).await {
                        Ok(_device) => {
                             let _ = event_tx.send(FromBle::StatusChange("Connected!".into()));
                             comm_loop(&mut cmd_rx, &event_tx).await;
                        }
                        Err(e) => {
                            let _ = event_tx.send(FromBle::StatusChange(format!("Connection Failed: {}", e)));
                        }
                    }
                }
                _ => {}
            }
        }
    }
}

// Helper to scan and connect
async fn find_and_connect(
    adapter: &Adapter, 
    event_tx: &mpsc::UnboundedSender<FromBle>
) -> Result<Device, String> {
    let discovery = adapter.discover_devices().await.map_err(|e| e.to_string())?;
    
    pin_mut!(discovery);

    let _ = event_tx.send(FromBle::StatusChange("Scanning for devices...".into()));

    loop {
        tokio::select! {

            Some(evt) = discovery.next() => {
                match evt {
                    AdapterEvent::DeviceAdded(addr) => {
                        match adapter.device(addr) {
                            Ok(device) => {
                                let name_opt = device.name().await.ok().flatten();
                                
                                if let Some(name) = name_opt {
                                    if name == TARGET_DEVICE_NAME {
                                        let _ = event_tx.send(FromBle::StatusChange(format!("Found {}! Connecting...", name)));
                                    
                                        drop(discovery);

                                        let mut retries = 3;
                                        while retries > 0 {
                                            match device.connect().await {
                                                Ok(_) => {
                                                    return Ok(device);
                                                }
                                                Err(e) => {
                                                    let _ = event_tx.send(FromBle::StatusChange(format!("Connect failed: {}. Retrying...", e)));
                                                    retries -= 1;
                                                    sleep(Duration::from_millis(500)).await;
                                                }
                                            }
                                        }
                                        return Err("Failed to connect after 3 attempts".into());
                                    }
                                }
                            }
                            Err(_e) => {
                                // Failed to get device details, ignore
                            }
                        }
                    }
                    _ => {
                        // Ignore other events (DeviceRemoved, etc.)
                    }
                }
            }
        }
    }
}

// Main Read/Write loop
async fn comm_loop(
    cmd_rx: &mut mpsc::UnboundedReceiver<ToBle>,
    event_tx: &mpsc::UnboundedSender<FromBle>
) {    
    loop {
        tokio::select! {
            cmd = cmd_rx.recv() => {
                match cmd {
                    Some(ToBle::SendJson(data_struct)) => {
                        if let Ok(json_str) = serde_json::to_string(&data_struct) {
                            let _ = event_tx.send(FromBle::DataReceived(format!("Sent: {}", json_str)));
                        }
                    }
                    _ => {}
                }
            }
        }
    }
}