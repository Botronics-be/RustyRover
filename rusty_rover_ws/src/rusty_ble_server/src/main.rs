use rclrs::*;
use std::{thread, sync::{Arc, Mutex}, time::Duration};
use geometry_msgs::msg::TwistStamped as TwistStamped;
use std_msgs::msg::Header;
use geometry_msgs::msg::Twist;
use geometry_msgs::msg::Vector3;
use builtin_interfaces::msg::Time;
use bluer::{
    adv::Advertisement,
    gatt::local::{
        Application, Characteristic, CharacteristicRead,
        CharacteristicWrite, Service, CharacteristicWriteMethod
    },
    Uuid,
};
use serde::Deserialize;
use tokio::sync::mpsc;

#[derive(Debug, Deserialize)]
struct TeleopCmd {
    linear_x: f64,
    angular_z: f64,
}

const SERVICE_UUID: Uuid = Uuid::from_u128(0x1deaebe7_ce65_4d57_8933_1bdc2065f37b);
const CHARACTERISTIC_UUID: Uuid = Uuid::from_u128(0x9dd2899d_f3c9_47ee_992a_aad14b2cdaaf);

#[derive(Deserialize, Debug)]
#[serde(tag = "type", content = "data")] 
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
enum BleCommand {
    Connected(String),
    Teleop(String),
}

struct RosBridge {
    node: Node,
    cmd_vel_publisher: Publisher<TwistStamped>,
    status: String,
}

impl RosBridge {
    fn new(node: &Node) -> Result<Self, RclrsError> {
        Ok(Self {
            node: node.clone(),
            status: "Idle".to_string(),
            cmd_vel_publisher: node.create_publisher::<TwistStamped>("cmd_vel")?,
        })
    }

    fn dispatch(&mut self, cmd: BleCommand) {
        match cmd {
            BleCommand::Connected(data) => {self.handle_connected_cmd(data);}
            BleCommand::Teleop(data) => {self.handle_teleop_cmd(data);}
        }
    }

    fn handle_connected_cmd(&mut self, data: String){
        log_info!(self.node.logger(), "Received CONNECTED from : {}", data);
    }

    fn handle_teleop_cmd(&mut self, data: String){
        self.status = "Teleoperating".to_string();
        let data_cmd: TeleopCmd = serde_json::from_str(&data).unwrap();
        log_info!(
            self.node.logger().throttle(Duration::from_secs(1)), 
            "Received TELEOP from client: x:{}, z:{}",
            data_cmd.linear_x,
            data_cmd.angular_z
        );

        let now = self.node.get_clock().now();

        let nanoseconds = now.nsec;
        let seconds = nanoseconds / 1_000_000_000;

        let msg: TwistStamped = TwistStamped {
            header: Header {
                frame_id: "base_link".to_string(),
                stamp: Time {
                    sec: seconds as i32,
                    nanosec: nanoseconds as u32,
                },
            },
            twist: Twist {
                linear: Vector3 {
                    x: data_cmd.linear_x,
                    y: 0.0,
                    z: 0.0,
                },
                angular: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: data_cmd.angular_z,
                },
            },
        };

        let _ = self.cmd_vel_publisher.publish(msg);
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
        read: Some(CharacteristicRead {
            read: true,
            fun: Box::new(move |_req| {
                Box::pin(async move {
                    let value = "Hello from Rust!".as_bytes().to_vec();
                    println!("Read request received, sending: {:?}", value);
                    Ok(value)
                })
            }),
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

    let bridge = Arc::new(Mutex::new(RosBridge::new(&node)?));

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
                    if let Ok(mut bridge_guard) = bridge_clone.lock() {
                        bridge_guard.dispatch(cmd);
                    }
                }
            });

            let _ = tokio::join!(ble_handle, dispatcher_handle);
        });
    });
    
    executor.spin(SpinOptions::default()).first_error()
}