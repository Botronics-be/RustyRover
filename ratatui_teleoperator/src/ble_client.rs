use bluer::{Adapter, Device, Session, Uuid, AdapterEvent};
use futures::{pin_mut, StreamExt};
use serde::{Deserialize, Serialize};
use std::time::Duration;
use tokio::time::sleep;
use tokio::sync::mpsc;

const TARGET_DEVICE_NAME: &str = "RustyRover"; 
const SERVICE_UUID: Uuid = Uuid::from_u128(0x1deaebe7_ce65_4d57_8933_1bdc2065f37b);
const CHARACTERISTIC_UUID: Uuid = Uuid::from_u128(0x9dd2899d_f3c9_47ee_992a_aad14b2cdaaf);

// Data to send over Bluetooth (The JSON structure)
#[derive(Serialize, Deserialize, Debug)]
pub struct RobotCommand {
    #[serde(rename = "type")]
    pub cmd_type: String,
    pub data: String,
}

// Commands from TUI -> BLE Worker
#[derive(Debug)]
pub enum ToBle {
    Connect,
    Disconnect,
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

    let adapter = match session.default_adapter().await {
        Ok(a) => a,
        Err(e) => {
            let _ = event_tx.send(FromBle::StatusChange(format!("Adapter Error: {}", e)));
            return;
        }
    };

    let _ = adapter.set_powered(true).await;
    let _ = event_tx.send(FromBle::StatusChange("Ready to Connect".into()));

    // Main loop
    loop {
        // Wait for a "Connect" command from UI
        if let Some(cmd) = cmd_rx.recv().await {
            match cmd {
                ToBle::Connect => {
                    let _ = event_tx.send(FromBle::StatusChange("Scanning...".into()));
                    
                    // --- CONNECTION LOGIC ---
                    match find_and_connect(&adapter, &event_tx).await {
                        Ok(device) => {
                             let _ = event_tx.send(FromBle::StatusChange("Connected!".into()));
                             // Enter connected loop
                             handle_connection(device, &mut cmd_rx, &event_tx).await;
                        }
                        Err(e) => {
                            let _ = event_tx.send(FromBle::StatusChange(format!("Conn Failed: {}", e)));
                        }
                    }
                }
                _ => {} // Ignore other commands while disconnected
            }
        }
    }
}

// Helper to scan and connect
async fn find_and_connect(
    adapter: &Adapter, 
    event_tx: &mpsc::UnboundedSender<FromBle>
) -> Result<Device, String> {
    // 1. Start Discovery
    // discover_devices() returns a Stream of AdapterEvents.
    let discovery = adapter.discover_devices().await.map_err(|e| e.to_string())?;
    
    // 2. Pin the stream
    // "pin_mut!" is a macro that pins the stream to the stack so we can iterate it safely.
    pin_mut!(discovery);

    // 3. Setup a Timeout
    // We don't want to scan forever.
    let timeout = sleep(Duration::from_secs(10));
    tokio::pin!(timeout);

    let _ = event_tx.send(FromBle::StatusChange("Scanning for devices...".into()));

    loop {
        tokio::select! {
            // A. If 10 seconds pass, stop and return error
            _ = &mut timeout => {
                return Err("Device not found (timeout)".into());
            }

            // B. If a Bluetooth event happens (Device Added, etc.)
            Some(evt) = discovery.next() => {
                match evt {
                    AdapterEvent::DeviceAdded(addr) => {
                        // We found A device (address only). Let's check its properties.
                        // We must query the adapter for the full Device object.
                        match adapter.device(addr) {
                            Ok(device) => {
                                // Sometimes names aren't loaded instantly, so we handle potential errors safely
                                let name_opt = device.name().await.ok().flatten();
                                
                                if let Some(name) = name_opt {
                                    // LOGGING: Show what we found
                                    // let _ = event_tx.send(FromBle::StatusChange(format!("Seen: {}", name)));

                                    // CHECK: Is this our target?
                                    if name == TARGET_DEVICE_NAME {
                                        let _ = event_tx.send(FromBle::StatusChange(format!("Found {}! Connecting...", name)));
                                        
                                        // STOP SCANNING: Drop the discovery stream to stop radio usage
                                        drop(discovery);

                                        // RETRY LOOP: Connection can fail, so we try 3 times
                                        let mut retries = 3;
                                        while retries > 0 {
                                            match device.connect().await {
                                                Ok(_) => {
                                                    // Success! Return the connected device.
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

// Helper to handle data once connected
async fn handle_connection(
    device: Device, 
    cmd_rx: &mut mpsc::UnboundedReceiver<ToBle>,
    event_tx: &mpsc::UnboundedSender<FromBle>
) {
    // Find our specific Service and Characteristic
    // Note: In real code, you need to iterate services and characteristics to match UUIDs
    // For brevity, we assume we find the right one.
    
    // WARNING: This is simplified. You must iterate `device.services()` to find the correct char.
    // Let's assume we want to write to the first writeable characteristic found for now.
    
    loop {
        // Wait for commands OR disconnection
        tokio::select! {
            cmd = cmd_rx.recv() => {
                match cmd {
                    Some(ToBle::SendJson(data_struct)) => {
                        // Serialize to JSON
                        if let Ok(json_str) = serde_json::to_string(&data_struct) {
                            // HERE: You would call characteristic.write()
                            // For this example, we just log it
                            let _ = event_tx.send(FromBle::DataReceived(format!("Sent: {}", json_str)));
                            
                            // Real code:
                            // char.write(json_str.as_bytes()).await...
                        }
                    }
                    Some(ToBle::Disconnect) => {
                         let _ = device.disconnect().await;
                         let _ = event_tx.send(FromBle::StatusChange("Disconnected".into()));
                         return;
                    }
                    _ => {}
                }
            }
            // You can also add a branch here to read notifications from the device
        }
    }
}