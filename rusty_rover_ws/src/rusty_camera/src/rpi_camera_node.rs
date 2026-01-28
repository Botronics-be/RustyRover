use anyhow::{Context as AnyhowContext, Error, Result};
use opencv::{prelude::*, videoio, core, imgproc};
use rclrs::*;
use sensor_msgs::msg::Image;
use std::sync::Mutex;
use std::time::Duration;

// Change return type to anyhow::Result to allow mixing different error types
fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("worker_demo")?;

    // Specified the message type for the publisher
    let publisher = node.create_publisher::<Image>("output_topic")?;
    let worker = node.create_worker(String::new());
    
    // Map OpenCV error to anyhow or use .context()
    let cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)
        .map_err(|e| anyhow::anyhow!("OpenCV Init Error: {:?}", e))?;

    if !videoio::VideoCapture::is_opened(&cam).map_err(|e| anyhow::anyhow!("{:?}", e))? {
        return Err(anyhow::anyhow!("Could not open video device"));
    }

    let cam_mutex = Mutex::new(cam);

    let _timer = worker.create_timer_repeating(
        Duration::from_millis(66),
        move |_data: &mut String| {
            let mut cam = cam_mutex.lock().expect("Failed to lock camera");
            let mut frame = core::Mat::default();
            
            if cam.read(&mut frame).unwrap_or(false) && !frame.empty() {
                if let Ok(msg) = cv_mat_to_ros_msg(&frame) {
                    let _ = publisher.publish(msg);
                }
            }
        }
    )?;

    println!("Beginning camera stream on /output_topic...");
    
    // executor.spin returns a Result, so we handle it
    executor.spin(SpinOptions::default()).first_error()?;

    Ok(())
}

fn cv_mat_to_ros_msg(frame: &core::Mat) -> Result<Image> {
    let mut rgb_frame = core::Mat::default();
    
    // 1. Convert BGR to RGB
    imgproc::cvt_color(
        frame, 
        &mut rgb_frame, 
        imgproc::COLOR_BGR2RGB, 
        0, 
        core::AlgorithmHint::ALGO_HINT_APPROX
    ).map_err(|e| anyhow::anyhow!("Color conversion error: {:?}", e))?;

    // 2. Explicitly get dimensions
    let height = rgb_frame.rows(); // Use rows() for height
    let width = rgb_frame.cols();  // Use cols() for width
    let channels = 3; 
    let step = (width * channels) as u32;

    // 3. Get raw data
    let data = rgb_frame.data_bytes()
        .map_err(|e| anyhow::anyhow!("Data conversion error: {:?}", e))?
        .to_vec();

    // 4. Set the timestamp (Optional but highly recommended for visualization)
    let mut header = std_msgs::msg::Header::default();
    header.frame_id = "camera_frame".to_string();
    // In a real node, you'd set header.stamp here

    Ok(Image {
        header,
        height: height as u32,
        width: width as u32,
        encoding: "rgb8".to_string(),
        is_bigendian: 0,
        step,
        data,
    })
}