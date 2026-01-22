use rclrs::*;
use std::{thread, time::Duration};
use std::sync::Arc;
use std_msgs::msg::String as StringMsg;

use bluer::{
    adv::Advertisement,
    gatt::local::{
        Application, Characteristic, CharacteristicRead,
        CharacteristicWrite, Service,
    }, Uuid,
};
use bluer::gatt::local::CharacteristicWriteMethod;
use serde::Deserialize;

// Service and characteristic UUID used to identify the ble server
const SERVICE_UUID: Uuid = Uuid::from_u128(0x12345678_1234_5678_1234_567812345678);
const CHARACTERISTIC_UUID: Uuid = Uuid::from_u128(0x87654321_4321_6789_4321_678943216789);

// Default BLE message structure
#[derive(Deserialize, Debug)]
struct Message {
    #[serde(rename = "type")]
    msg_type: String,
    data: String,
}

pub struct BleServerNode {
    hello_publisher: Publisher<StringMsg>,
}

impl BleServerNode {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {        
        let node = executor.create_node("bluetooth_server_node")?;
        log_info!(node.logger(), "Bluetooth_server_node Startup");

        let hello_publisher = node.create_publisher::<StringMsg>("from_ble_topic")?;

        // Create a separate thread to hold the async bluer Server logic
        thread::spawn(move || {
            let rt = tokio::runtime::Runtime::new().unwrap();
            
            rt.block_on(async {
                if let Err(e) = Self::run_bluetooth_server(node.clone()).await {
                    log_error!(node.logger(),"Bluetooth error: {}", e);
                }
            });
        });

        Ok(Self { hello_publisher })
    }

    async fn run_bluetooth_server(node: Node) -> bluer::Result<()> {
        log_info!(node.logger(), "Initializing Bluetooth Session...");

        // Creating session and powering the adapter on
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

        log_info!(node.logger(), "BLE Server available !!!");

        let char_handle = Characteristic {
            uuid: CHARACTERISTIC_UUID,
            write: Some(CharacteristicWrite {
                write: true,
                write_without_response: true,
                method: CharacteristicWriteMethod::Fun(Box::new(move |new_value, _req| {

                    let node_clone = node.clone();

                    Box::pin(async move {
                        let s = std::str::from_utf8(&new_value).unwrap_or("");;

                        match serde_json::from_str::<Message>(s) {
                            Ok(msg) => {
                                match msg.msg_type.as_str() {
                                    "HELLO" => Self::hello_cmd_callback(node_clone.clone(), msg.data),
                                    _ =>  log_warn!(node_clone.logger(),"Unknown command"),
                                }
                            }
                            Err(e) => log_error!(node_clone.logger(),"Failed to parse JSON: {}", e),
                        }
                        Ok(())
                    })
                })),
                ..Default::default()
            }),
            ..Default::default()
        };

        let service_handle = Service {
            uuid: SERVICE_UUID,
            primary: true,
            characteristics: vec![char_handle],
            ..Default::default()
        };

        let app = Application {
            services: vec![service_handle],
            ..Default::default()
        };
        let app_handle = adapter.serve_gatt_application(app).await?;

        std::future::pending::<()>().await;

        Ok(())
    }

    fn hello_cmd_callback(node: Node, msg_data: String) {
        log_info!(node.logger(),"Received hello msg with data: {}", msg_data);
        hello_publisher.publish(StringMsg {data: msg_data.clone(),})
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = BleServerNode::new(&executor)?;

    executor.spin(SpinOptions::default()).first_error()
}