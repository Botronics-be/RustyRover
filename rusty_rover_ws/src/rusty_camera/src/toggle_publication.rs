use anyhow::{Error, Result};
use rclrs::*;
use sensor_msgs::msg::Image;
use std_srvs::srv::{SetBool, SetBool_Request, SetBool_Response, Trigger, Trigger_Request, Trigger_Response};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::path::Path; 

static SHOULD_REPUBLISH: AtomicBool = AtomicBool::new(false);

struct ToggleImageApp {
    _publisher: Publisher<Image>,
    _subscription: Subscription<Image>,
    _toggle_service: Service<SetBool>,
    _capture_service: Service<Trigger>,
    last_frame: Arc<Mutex<Option<Image>>>,
}

impl ToggleImageApp {
    fn new(node: &Node) -> Result<Self, RclrsError> {
        let publisher = node.create_publisher::<Image>("/image_viewer")?;
        
        let last_frame: Arc<Mutex<Option<Image>>> = Arc::new(Mutex::new(None));
        
        let last_frame_for_sub = Arc::clone(&last_frame);
        let last_frame_for_srv = Arc::clone(&last_frame);

        let capture_service = node.create_service::<Trigger, _>(
            "capture_frame",
            move |_request: Trigger_Request, _info: ServiceInfo| -> Trigger_Response {
                let frame_lock = last_frame_for_srv.lock().unwrap();
                
                if let Some(frame) = &*frame_lock {

                    let file_path = "/home/david/Documents/Project/Otto/RustyRover/rusty_rover_ws/captured_frame.png";
                    
                    println!("Service: Processing frame {}x{} (Encoding: {})", frame.width, frame.height, frame.encoding);

                    // Determine how to save based on encoding
                    let save_result = match frame.encoding.as_str() {
                        "rgb8" => {
                            image::save_buffer(
                                &Path::new(file_path),
                                &frame.data,
                                frame.width,
                                frame.height,
                                image::ColorType::Rgb8
                            )
                        },
                        "bgr8" => {
                            let mut rgb_data = frame.data.clone();
                            for chunk in rgb_data.chunks_exact_mut(3) {
                                chunk.swap(0, 2); // Swap Blue (0) and Red (2)
                            }
                            image::save_buffer(
                                &Path::new(file_path),
                                &rgb_data,
                                frame.width,
                                frame.height,
                                image::ColorType::Rgb8
                            )
                        },
                        "mono8" => {
                            image::save_buffer(
                                &Path::new(file_path),
                                &frame.data,
                                frame.width,
                                frame.height,
                                image::ColorType::L8
                            )
                        },
                        _ => {
                            // Fallback: try saving as RGB and hope for the best
                            image::save_buffer(
                                &Path::new(file_path),
                                &frame.data,
                                frame.width,
                                frame.height,
                                image::ColorType::Rgb8
                            )
                        }
                    };

                    match save_result {
                        Ok(_) => Trigger_Response {
                            success: true,
                            message: format!("Saved to {}", file_path),
                        },
                        Err(e) => Trigger_Response {
                            success: false,
                            message: format!("Failed to save image: {}", e),
                        },
                    }
                } else {
                    Trigger_Response {
                        success: false,
                        message: "No frame received yet.".to_string(),
                    }
                }
            },
        )?;

        let toggle_service = node.create_service::<SetBool, _>(
            "toggle_republish",
            move |request: SetBool_Request, _info: ServiceInfo| -> SetBool_Response {
                SHOULD_REPUBLISH.store(request.data, Ordering::SeqCst);
                println!("Service: Republishing set to {}", request.data);
                SetBool_Response {
                    success: true,
                    message: format!("Republishing: {}", request.data),
                }
            },
        )?;


        let publisher_clone = publisher.clone();
        let subscription = node.create_subscription::<Image, _>(
            "/camera/image_raw",
            move |msg: Image| {
                let mut frame_lock = last_frame_for_sub.lock().unwrap();
                *frame_lock = Some(msg.clone());

                if SHOULD_REPUBLISH.load(Ordering::SeqCst) {
                    let _ = publisher_clone.publish(&msg);
                }
            },
        )?;

        Ok(Self {
            _publisher: publisher,
            _subscription: subscription,
            _toggle_service: toggle_service,
            _capture_service: capture_service,
            last_frame,
        })
    }
}

fn main() -> Result<(), Error> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("toggle_image_publisher")?;
    let _app = ToggleImageApp::new(&node)?;

    println!("Node running.");
    println!(" - Call /capture_frame to save 'captured_frame.png' to your workspace root.");
    
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}