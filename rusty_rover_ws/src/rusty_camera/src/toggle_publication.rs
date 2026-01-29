use anyhow::{Error, Result};
use rclrs::*;
use sensor_msgs::msg::Image;
use std_srvs::srv::{Trigger, Trigger_Request, Trigger_Response}; // Removed SetBool
use std::sync::{Arc, Mutex};
use std::path::{Path, PathBuf};
use std::fs; 

struct CaptureServiceNode {
    // We only need to listen now, not republish
    _subscription: Subscription<Image>,
    _capture_service: Service<Trigger>,
    
    storage_path: PathBuf,
    last_frame: Arc<Mutex<Option<Image>>>,
}

impl CaptureServiceNode {
    fn new(node: &Node) -> Result<Self, RclrsError> {
        
        let home_dir = std::env::var("HOME").unwrap_or_else(|_| ".".to_string());
        let storage_path = Path::new(&home_dir).join("rusty_images");
        
        if let Err(e) = fs::create_dir_all(&storage_path) {
            eprintln!("Warning: Could not create directory {:?}: {}", storage_path, e);
        } else {
            println!("Storage ready: Images will be saved to {:?}", storage_path);
        }

        
        let last_frame: Arc<Mutex<Option<Image>>> = Arc::new(Mutex::new(None));
        let last_frame_for_sub = Arc::clone(&last_frame);
        let last_frame_for_srv = Arc::clone(&last_frame);
        
        
        let storage_path_for_srv = storage_path.clone();

       
        let capture_service = node.create_service::<Trigger, _>(
            "capture_frame",
            move |_request: Trigger_Request, _info: ServiceInfo| -> Trigger_Response {
                let frame_lock = last_frame_for_srv.lock().unwrap();
                
                if let Some(frame) = &*frame_lock {

                    let filename = format!(
                        "frame_{}.png", 
                        frame.header.stamp.sec
                    );
                    
                    let file_path = storage_path_for_srv.join(&filename);
                    println!("Service: Saving to {:?}", file_path);

                    // Save result depending on encoding
                    let save_result = match frame.encoding.as_str() {
                        "rgb8" => {
                            image::save_buffer(
                                &file_path,
                                &frame.data,
                                frame.width,
                                frame.height,
                                image::ColorType::Rgb8
                            )
                        },
                        "bgr8" => {
                            let mut rgb_data = frame.data.clone();
                            for chunk in rgb_data.chunks_exact_mut(3) {
                                chunk.swap(0, 2); 
                            }
                            image::save_buffer(
                                &file_path,
                                &rgb_data,
                                frame.width,
                                frame.height,
                                image::ColorType::Rgb8
                            )
                        },
                        "mono8" => {
                            image::save_buffer(
                                &file_path,
                                &frame.data,
                                frame.width,
                                frame.height,
                                image::ColorType::L8
                            )
                        },
                        _ => {
                           
                            image::save_buffer(
                                &file_path,
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
                            message: format!("Saved to {}", file_path.display()),
                        },
                        Err(e) => Trigger_Response {
                            success: false,
                            message: format!("Failed to save: {}", e),
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

        let subscription = node.create_subscription::<Image, _>(
            "/camera/image_raw",
            move |msg: Image| {
                let mut frame_lock = last_frame_for_sub.lock().unwrap();
                *frame_lock = Some(msg.clone());
            },
        )?;

        Ok(Self {
            _subscription: subscription,
            _capture_service: capture_service,
            last_frame,
            storage_path,
        })
    }
}

fn main() -> Result<(), Error> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("capture_service_node")?;
    let _app = CaptureServiceNode::new(&node)?;

    println!("Capture Node Running.");
    println!(" - Saving to: ~/rusty_images/");
    println!(" - Service: /capture_frame");
    
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}