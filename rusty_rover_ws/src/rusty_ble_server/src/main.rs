use rclrs::*;
use std::{thread, time::Duration};

pub struct BleServerNode {
}

impl BleServerNode {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("bluetooth_server_node")?;

        Ok(Self { })
    }

}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = BleServerNode::new(&executor)?;

    executor.spin(SpinOptions::default()).first_error()
}