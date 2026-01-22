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

const SERVICE_UUID: Uuid = Uuid::from_u128(0x1deaebe7_ce65_4d57_8933_1bdc2065f37b);
const CHARACTERISTIC_UUID: Uuid = Uuid::from_u128(0x9dd2899d_f3c9_47ee_992a_aad14b2cdaaf);

#[derive(Deserialize, Debug)]
#[serde(tag = "type", content = "data")] 
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
enum BleCommand {
    Hello(String),
}

struct RosBridge {
    logger: Logger,
    hello_publisher: Publisher<StringMsg>,
}

impl RosBridge {
    fn new(node: &Node) -> Result<Self, RclrsError> {
        Ok(Self {
            logger: node.logger().clone(),
            hello_publisher: node.create_publisher::<StringMsg>("from_ble_topic")?,
        })
    }

    fn dispatch(&self, cmd: BleCommand) {
        match cmd {
            BleCommand::Hello(data) => {
                log_info!(&self.logger, "Processing Hello: {}", data);
                let _ = self.hello_publisher.publish(StringMsg { data });
            }
        }
    }
}

async fn run_bluetooth_server(node: Node, tx: mpsc::Sender<BleCommand>) -> bluer::Result<()> {
    // Receives the command via ble and sends them through the mpsc channel to the dispatcher
    let session = bluer::Session::new().await?;
    let adapter = session.default_adapter().await?;
    adapter.set_powered(true).await?;

    let le_advertisement = Advertisement {
        service_uuids: vec![SERVICE_UUID].into_iter().collect(),
        discoverable: Some(true),
        local_name: Some("RustyRover".to_string()),
        ..Default::default()
    };
    let _adv_handle = adapter.advertise(le_advertisement).await?;

    let char_handle = Characteristic {
        uuid: CHARACTERISTIC_UUID,
        write: Some(CharacteristicWrite {
            write: true,
            write_without_response: true,
            method: CharacteristicWriteMethod::Fun(Box::new(move |new_value, _req| {
                let node_clone = node.clone();
                let tx = tx.clone();
                Box::pin(async move {
                    let s = std::str::from_utf8(&new_value).unwrap_or("");
                    match serde_json::from_str::<BleCommand>(s) {
                        Ok(cmd) => {
                            if let Err(_) = tx.send(cmd).await {
                                log_error!(node_clone.logger(), "Receiver dropped, cannot send command");
                            }
                        },
                        Err(e) => log_error!(node_clone.logger(), "Failed to parse BLE command: {} | Raw: {}", e, s),
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

    std::future::pending::<()>().await;
    Ok(())
}

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("bluetooth_server_node")?;
    log_info!(node.logger(), "Bluetooth server node starting ...");

    let bridge = Arc::new(RosBridge::new(&node)?);

    // Works like a UART channel a tx sends to a rx receiver
    let (tx, mut rx) = mpsc::channel::<BleCommand>(32);

    let node_clone = node.clone();
    thread::spawn(move || {
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async {
            
            let ble_handle = tokio::spawn(async move {
                let ble_node_clone = node_clone.clone();
                if let Err(e) = run_bluetooth_server(node_clone, tx).await {
                    log_error!(ble_node_clone.logger(), "BLE Server Error: {}", e);
                }
            });

            let bridge_clone = bridge.clone();
            let dispatcher_handle = tokio::spawn(async move {
                while let Some(cmd) = rx.recv().await {
                    bridge_clone.dispatch(cmd);
                }
            });

            let _ = tokio::join!(ble_handle, dispatcher_handle);
        });
    });
    
    executor.spin(SpinOptions::default()).first_error()
}