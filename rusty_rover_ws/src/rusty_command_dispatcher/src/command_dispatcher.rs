use rclrs::*;
use geometry_msgs::msg::TwistStamped as TwistStamped;
use std_msgs::msg::Header;
use geometry_msgs::msg::{TwistStamped, Twist, Vector3};
use builtin_interfaces::msg::Time;
use anyhow::Result;

struct TeleopData {node: Node, cmd_vel_publisher: Publisher<TwistStamped>}

pub struct CommandDispatcher {
    node: Node,
    _teleop_command_subscriber: Subscription<TeleopMsg>,
    cmd_vel_publisher: Publisher<TwistStamped>,
}

impl CommandDispatcher {

    fn new(executor: &Executor) -> Result<Self>{
        let node = executor.create_node("command_dispatcher")?;

        let cmd_vel_publisher = node.create_publisher::<TwistStamped>("/diff_cont/cmd_vel")?;

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

        Ok(CommandDispatcher {
            node,
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
