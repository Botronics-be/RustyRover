use rclrs::*;
use tokio::sync::mpsc::unbounded_channel;
use std::time::{Duration, Instant};
use anyhow::{Error, Result};
use geometry_msgs::msg::{TwistStamped, Twist, Vector3};
use std_msgs::msg::Header;
use builtin_interfaces::msg::Time;
use rusty_msgs::action::{SpinAction, SpinAction_Feedback, SpinAction_Result};

async fn spinner_action(node: Node, handle: RequestedGoal<SpinAction>) -> TerminatedGoal {
    let spinning_time = handle.goal().spinning_time; // Get the fibonacci order inside the requested goal
    log_info!(node.logger(), "Received spin goal with order: {} from client", spinning_time);

    // Reject a forbidden goal
    if spinning_time < 0 {
        log_error!(node.logger(), "Rejecting goal, can't spin for a negative time");
        return handle.reject(); 
    }

    let result = SpinAction_Result::default(); // Initialize the result variable
    let elapsed_time = Instant::now(); // Initialize the feedback

    // Notifying the acceptance of the goal and starting the execution phase
    let executing = match handle.accept().begin() {
        BeginAcceptedGoal::Execute(executing) => executing,
        BeginAcceptedGoal::Cancel(cancelling) => {
            return cancelling.cancelled_with(result);
        }
    };

    let (sender, mut receiver) = unbounded_channel();

    
    // Execution thread actually sending the spinning command
    let execution_node = node.clone();
    std::thread::spawn(move || {
        let cmd_vel_publisher = execution_node
        .create_publisher::<TwistStamped>("/diff_cont/cmd_vel")
        .expect("Failed to create publisher inside thread");

        while elapsed_time.elapsed() < Duration::from_millis(spinning_time as u64){

            let now = execution_node.get_clock().now();

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
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    angular: Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.25,
                    },
                },
            };
            if let Err(_) = sender.send(elapsed_time.elapsed().as_millis() as i64) {
                return;
            }

            let _ = cmd_vel_publisher.publish(msg);
            std::thread::sleep(Duration::from_millis(33));
        }
    });

    // Consuming the successive results comming from the execution thread
    loop {
        match executing.unless_cancel_requested(receiver.recv()).await {
            Ok(Some(elapsed)) => {
                // Still executing, push the current result as feedback
                executing.publish_feedback(SpinAction_Feedback {
                    elapsed_time: elapsed
                });
            }
            Ok(None) => {
                // The end of the sequence is reached, return result
                log_info!(node.logger(), "Sequence end reached, action succeeded");
                return executing.succeeded_with(SpinAction_Result {
                    is_succeeded: true
                });
            }
            Err(_) => {
                // Cancel received, end the current execution
                log_warn!(node.logger(), "Goal cancelled");
                let cancelling = executing.begin_cancelling();
                return cancelling.cancelled_with(result);
            }
        }
    }
}

fn main() -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    let node = executor.create_node("spinner_node")?;
    log_info!(node.logger(), "Action server node starting ...");

    let action_node = node.clone();
    let _action = node.create_action_server(
    &"action_name",
    move |handle| {
        spinner_action(action_node.clone(), handle)
    }).unwrap();

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}