use anyhow::{Error, Result};
use rclrs::*;
use sensor_msgs::msg::Image;
use std_srvs::srv::*;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

static SHOULD_REPUBLISH: AtomicBool = AtomicBool::new(false);

struct ToggleImagePublisher {
    publisher: Publisher<Image>,
    subscription: Subscription<Image>,
    service: Service<SetBool>,
}

impl ToggleImagePublisher {
    fn new(node: &Node) -> Result<Arc<Self>, RclrsError> {
        let publisher = node.create_publisher::<Image>("/image_viewer")?;

        let service = node.create_service::<SetBool, _>(
            "toggle_republish",
            Self::handle_service,
        )?;
        
        let publisher_clone = publisher.clone();
        let subscription = node.create_subscription::<Image, _>(
            "/camera/image_raw",
            move |msg: Image| {
                if SHOULD_REPUBLISH.load(Ordering::SeqCst) {
                    let _ = publisher_clone.publish(&msg);
                }
            },
        )?;

        Ok(Arc::new(Self {
            publisher: publisher,
            subscription: subscription,
            service: service,
        }))
    }

    fn handle_service(request: SetBool_Request, _info: ServiceInfo) -> SetBool_Response {
        SHOULD_REPUBLISH.store(request.data, Ordering::SeqCst);
        let status = if request.data { "ENABLED" } else { "DISABLED" };
        SetBool_Response {
            success: true,
            message: format!("Republishing set to {}", status),
        }
    }
}

fn main() -> Result<(), Error> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("toggle_image_publisher")?;

    // This one line handles the publisher, service, and subscriber setup
    let toggle_image_publisher = ToggleImagePublisher::new(&node)?;

    println!("ToggleImagePublisher node is running...");
    
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}