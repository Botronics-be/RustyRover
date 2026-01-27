use anyhow::{Error, Result};
use rclrs::*;
use sensor_msgs::msg::Image;
use std_srvs::srv::*;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

// We keep the atomic flag for simple cross-callback communication
static SHOULD_REPUBLISH: AtomicBool = AtomicBool::new(false);

struct ToggleImagePublisher {
    node: Node,
    publisher: Arc<Publisher<Image>>,
}

impl ToggleImagePublisher {
    fn new(node: &Node) -> Result<Self, RclrsError> {
        Ok(Self {
            node: node.clone(),
            publisher: Arc::new(node.create_publisher::<Image>("/image_viewer")?),
        })
    }

    // Static method for service
    fn handle_service(request: SetBool_Request, _info: ServiceInfo) -> SetBool_Response {
        SHOULD_REPUBLISH.store(request.data, Ordering::SeqCst);
        let status = if request.data { "ENABLED" } else { "DISABLED" };
        
        SetBool_Response {
            success: true,
            message: format!("Republishing set to {}", status),
        }
    }

    // Method for handling images
    fn handle_image(&self, msg: Image) {
        if SHOULD_REPUBLISH.load(Ordering::SeqCst) {
            let _ = self.publisher.publish(&msg);
        }
    }
}

fn main() -> Result<(), Error> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("toggle_image_publisher")?;

    // Initialize the struct logic
    let gatekeeper = Arc::new(ToggleImagePublisher::new(&node)?);

    // Service: Use the static method from the struct
    let _server = node.create_service::<SetBool, _>(
        "toggle_republish", 
        ToggleImagePublisher::handle_service
    )?;

    // Subscription: Use a closure to call the struct method
    let toggle_image_publisher_clone = Arc::clone(&gatekeeper);
    let _sub = node.create_subscription::<Image, _>(
        "/camera/image_raw",
        move |msg: Image| {
            toggle_image_publisher_clone.handle_image(msg);
        },
    )?;

    println!("ToggleImagePublisher node with struct-style ready.");
    
    // Using the exact syntax that worked for you
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}