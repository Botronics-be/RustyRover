use rclrs::*;
use rusty_msgs::msg::{SpinCmd as SpinCmdMsg, Teleop as TeleopMsg};
use rusty_msgs::action::{SpinAction_Goal, SpinAction};
use futures::StreamExt;
use anyhow::Result;

pub struct CommandDispatcher {
    node: Node,
    _teleop_command_subscriber: Subscription<TeleopMsg>,
    _spin_command_subscriber: Subscription<SpinCmdMsg>,
    pilot_cmd_publisher: Publisher<TeleopMsg>,
}

impl CommandDispatcher {

    fn new(executor: &Executor) -> Result<Self>{
        let node = executor.create_node("command_dispatcher")?;

        let pilot_cmd_publisher = node.create_publisher::<TeleopMsg>("/pilot/teleop")?;
        let spin_client = node.create_action_client::<SpinAction>(&"spin").unwrap();

        let pilot_cmd_pub_clone = pilot_cmd_publisher.clone();
        let _teleop_command_subscriber = node.create_subscription::<TeleopMsg, _>(
            "/cmd/teleop",
            move |msg: TeleopMsg|{            
                let _ = pilot_cmd_pub_clone.publish(msg);
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
            pilot_cmd_publisher,
        })
    }

}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env().unwrap().create_basic_executor();
    let _dispatcher = CommandDispatcher::new(&executor).unwrap();

    executor.spin(SpinOptions::default()).first_error()
}
