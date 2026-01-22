use rclrs::*;
use std::{thread, sync::Arc};
use std_msgs::msg::String as StringMsg;
use bluer::{
    adv::Advertisement,
    gatt::local::{
        Application, Characteristic,
        CharacteristicWrite, Service, CharacteristicWriteMethod
    },
    Uuid,
};
use serde::Deserialize;
use tokio::sync::mpsc;

// -----------------------------------------------------------------------------
// 1. Domain Definition (Scalable Message Types)
// -----------------------------------------------------------------------------

// We use Serde's "tag" to automatically dispatch based on the "type" field.
// { "type": "HELLO", "data": "World" } -> BleCommand::Hello("World")
// { "type": "STOP" } -> BleCommand::Stop
#[derive(Deserialize, Debug)]
#[serde(tag = "type", content = "data")] 
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
enum BleCommand {
    Hello(String),
    Stop, 
    SetSpeed(f64),
}

const SERVICE_UUID: Uuid = Uuid::from_u128(0x1deaebe7_ce65_4d57_8933_1bdc2065f37b);
const CHARACTERISTIC_UUID: Uuid = Uuid::from_u128(0x9dd2899d_f3c9_47ee_992a_aad14b2cdaaf);

struct RosBridge {
    logger: Logger,
    hello_publisher: Publisher<StringMsg>,
    // Add other publishers here (e.g., cmd_vel_publisher)
}

impl RosBridge {
    fn new(node: &Node) -> Result<Self, RclrsError> {
        Ok(Self {
            logger: node.logger().clone(),
            hello_publisher: node.create_publisher::<StringMsg>("from_ble_topic")?,
        })
    }

    // This is the only place you need to touch when adding logic for new commands
    fn dispatch(&self, cmd: BleCommand) {
        match cmd {
            BleCommand::Hello(data) => {
                log_info!(&self.logger, "Processing Hello: {}", data);
                let _ = self.hello_publisher.publish(StringMsg { data });
            }
            BleCommand::Stop => {
                log_warn!(&self.logger, "Emergency Stop requested!");
                // self.stop_publisher.publish(...)
            }
            BleCommand::SetSpeed(val) => {
                log_info!(&self.logger, "Setting speed to: {}", val);
            }
        }
    }
}

async fn run_bluetooth_server(tx: mpsc::Sender<BleCommand>) -> bluer::Result<()> {
    let session = bluer::Session::new().await?;
    let adapter = session.default_adapter().await?;
    adapter.set_powered(true).await?;

    println!("Advertising on Bluetooth adapter {} with address {}", adapter.name(), adapter.address().await?);

    let le_advertisement = Advertisement {
        service_uuids: vec![SERVICE_UUID].into_iter().collect(),
        discoverable: Some(true),
        local_name: Some("RustyRover".to_string()),
        ..Default::default()
    };
    let _adv_handle = adapter.advertise(le_advertisement).await?;

    // The generic write handler
    let char_handle = Characteristic {
        uuid: CHARACTERISTIC_UUID,
        write: Some(CharacteristicWrite {
            write: true,
            write_without_response: true,
            method: CharacteristicWriteMethod::Fun(Box::new(move |new_value, _req| {
                let tx = tx.clone();
                Box::pin(async move {
                    let s = std::str::from_utf8(&new_value).unwrap_or("");
                    
                    // The parsing logic is now handled entirely by Serde
                    match serde_json::from_str::<BleCommand>(s) {
                        Ok(cmd) => {
                            // Send valid command to the ROS bridge
                            if let Err(_) = tx.send(cmd).await {
                                eprintln!("Receiver dropped, cannot send command");
                            }
                        },
                        Err(e) => eprintln!("Failed to parse BLE command: {} | Raw: {}", e, s),
                    }
                    Ok(())
                })
            })),
            ..Default::default()
        }),
        ..Default::default()
    };

    let app = Application {
        services: vec![Service {
            uuid: SERVICE_UUID,
            primary: true,
            characteristics: vec![char_handle],
            ..Default::default()
        }],
        ..Default::default()
    };

    let _app_handle = adapter.serve_gatt_application(app).await?;
    
    // Keep async task alive
    std::future::pending::<()>().await;
    Ok(())
}

// -----------------------------------------------------------------------------
// 4. Main Entry Point
// -----------------------------------------------------------------------------

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("bluetooth_server_node")?;

    // 1. Create the ROS Logic Bridge
    let bridge = Arc::new(RosBridge::new(&node)?);

    // 2. Create a channel to communicate between BLE (Tokio) and ROS Logic
    let (tx, mut rx) = mpsc::channel::<BleCommand>(32);

    // 3. Spawn the Tokio Runtime in a separate thread
    let node_clone = node.clone();
    thread::spawn(move || {
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async {
            
            // Task A: Run the BLE Server (Producer)
            let ble_handle = tokio::spawn(async move {
                if let Err(e) = run_bluetooth_server(tx).await {
                    log_error!(node_clone.logger(), "BLE Server Error: {}", e);
                }
            });

            // Task B: Run the Command Dispatcher (Consumer)
            // This reads from the channel and calls ROS publishers
            let bridge_clone = bridge.clone();
            let dispatcher_handle = tokio::spawn(async move {
                while let Some(cmd) = rx.recv().await {
                    bridge_clone.dispatch(cmd);
                }
            });

            // Wait for both (usually they run forever)
            let _ = tokio::join!(ble_handle, dispatcher_handle);
        });
    });

    log_info!(node.logger(), "Bluetooth Node Started. Waiting for connections...");
    
    // 4. Spin ROS (Required if you have subscribers or timers, otherwise just keeps process alive)
    executor.spin(SpinOptions::default()).first_error()
}