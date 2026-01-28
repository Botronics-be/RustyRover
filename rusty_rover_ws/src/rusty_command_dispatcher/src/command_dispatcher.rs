use rclrs::*;
use std_msgs::msg::Header;
use geometry_msgs::msg::{TwistStamped, Twist, Vector3};
use rusty_msgs::msg::{SpinCmd as SpinCmdMsg, Teleop as TeleopMsg};
use rusty_msgs::action::{SpinAction_Goal, SpinAction};
use builtin_interfaces::msg::Time;
use futures::StreamExt;
use anyhow::Result;

struct TeleopData {node: Node, cmd_vel_publisher: Publisher<TwistStamped>}

pub struct CommandDispatcher {
    node: Node,
    _teleop_command_subscriber: Subscription<TeleopMsg>,
    _spin_command_subscriber: Subscription<SpinCmdMsg>,
    cmd_vel_publisher: Publisher<TwistStamped>,
}

impl CommandDispatcher {

    fn new(executor: &Executor) -> Result<Self>{
        let node = executor.create_node("command_dispatcher")?;

        let cmd_vel_publisher = node.create_publisher::<TwistStamped>("/diff_cont/cmd_vel")?;
        let spin_client = node.create_action_client::<SpinAction>(&"spin").unwrap();

        let cmd_vel_pub_clone = cmd_vel_publisher.clone();
        let node_clone = node.clone();
        let _teleop_command_subscriber = node.create_subscription::<TeleopMsg, _>(
            "/cmd/teleop",
            move |msg: TeleopMsg|{
                let now = node_clone.get_clock().now();

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
                            x: msg.linear_x,
                            y: 0.0,
                            z: 0.0,
                        },
                        angular: Vector3 {
                            x: 0.0,
                            y: 0.0,
                            z: msg.angular_z,
                        },
                    },
                };

                let _ = cmd_vel_pub_clone.publish(msg);

            }
        )?;

        let spin_node = node.clone();
        let spin_client_clone = spin_client.clone();        
        let _spin_command_subscriber = node.create_subscription::<SpinCmdMsg, _>(
            "/cmd/spin",
            move |msg: SpinCmdMsg| {
                let client = spin_client_clone.clone();
                let log_node = spin_node.clone();
                let spinning_time = msg.spinning_time;

                std::thread::spawn(move || {
                    
                    futures::executor::block_on(async {
                        log_info!(log_node.logger(), "Sending spin goal...");
                        
                        let goal = SpinAction_Goal { spinning_time: spinning_time };
                        let request_future = client.request_goal(goal);

                        let goal_handle = match request_future.await {
                            Some(handle) => handle,
                            None => {
                                log_error!(log_node.logger(), "Failed to send goal");
                                return;
                            }
                        };

                        let mut stream = goal_handle.stream();

                        while let Some(event) = stream.next().await {
                            match event {
                                GoalEvent::Feedback(feedback) => {
                                    log_debug!(log_node.logger(), "Feedback: Elapsed time: {:?}", feedback.elapsed_time);
                                }
                                GoalEvent::Result(result) => {
                                    log_info!(log_node.logger(), "Action finished. Status: {:?}", result);
                                    return;
                                }
                                _ => {}
                            }
                        }
                    });
                });
            }
        )?;

        Ok(CommandDispatcher {
            node,
            _spin_command_subscriber,
            _teleop_command_subscriber,
            cmd_vel_publisher,
        })
    }

}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env().unwrap().create_basic_executor();
    let _dispatcher = CommandDispatcher::new(&executor).unwrap();

    executor.spin(SpinOptions::default()).first_error()
}
