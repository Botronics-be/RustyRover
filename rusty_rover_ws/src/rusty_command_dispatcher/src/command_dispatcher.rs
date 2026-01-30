use rclrs::*;
use rusty_msgs::msg::{SpinCmd as SpinCmdMsg, Teleop as TeleopMsg};
use rusty_msgs::action::{SpinAction_Goal, SpinAction};
use std::sync::{Arc, Mutex};
use std_srvs::srv::{Trigger, Trigger_Request, Trigger_Response};
use std_msgs::msg::{Empty as EmptyMsg, String as StringMsg};
use std::time::{Duration, Instant};
use futures::StreamExt;
use anyhow::Result;

pub struct CommandDispatcher {
    _node: Node,
    _state: Arc<Mutex<RobotState>>,
    _teleop_command_subscriber: Subscription<TeleopMsg>,
    _spin_command_subscriber: Subscription<SpinCmdMsg>,
    _capture_image_subscriber: Subscription<EmptyMsg>,
    _status_publisher: Publisher<StringMsg>,
    _status_timer: Arc<TimerState<Arc<WorkerState<std::string::String>>>>,
}

struct RobotState {
    status: String,
    last_command_received: Instant,
}

impl CommandDispatcher {

    fn new(executor: &Executor) -> Result<Self>{
        let node = executor.create_node("command_dispatcher")?;
        log_info!(node.logger(), "Command dispatcher node starting ...");

        let shared_state = Arc::new(Mutex::new(RobotState {
            status: "Idle".to_string(),
            last_command_received: Instant::now(),
        }));

        let pilot_cmd_publisher = node.create_publisher::<TeleopMsg>("/pilot/teleop")?;
        let spin_client = node.create_action_client::<SpinAction>(&"spin").unwrap();
        let capture_image_client = node.create_client::<Trigger>(&"capture_frame").unwrap();
        let status_publisher = node.create_publisher::<StringMsg>("/status")?;

        let pilot_cmd_pub_clone = pilot_cmd_publisher.clone();
        let teleop_robot_state = shared_state.clone();
        let _teleop_command_subscriber = node.create_subscription::<TeleopMsg, _>(
            "/cmd/teleop",
            move |msg: TeleopMsg|{         
                let mut state = teleop_robot_state.lock().unwrap();
                state.status = "Teleoperating".to_string();
                state.last_command_received = Instant::now();

                drop(state);
                   
                let _ = pilot_cmd_pub_clone.publish(msg);
            }
        )?;

        let capture_image_node = node.clone();
        let capture_image_client_clone = capture_image_client.clone();
        let _capture_image_subscriber = node.create_subscription::<EmptyMsg, _>(
            "/cmd/capture_image",
            move |_msg: EmptyMsg| {
                
                let client = capture_image_client_clone.clone();
                let log_node = capture_image_node.clone();

                std::thread::spawn(move || {
                    futures::executor::block_on(async {
                        log_info!(log_node.logger(), "Check for service...");
                        client.notify_on_service_ready().await.unwrap();

                        let request = Trigger_Request { 
                            structure_needs_at_least_one_member: 0 
                        };
                        
                        let _response: Trigger_Response = client.call(&request).unwrap().await.unwrap();
                    });
                });
                
            }
        )?;

        let spin_node = node.clone();
        let spin_client_clone = spin_client.clone();
        let spin_state = shared_state.clone();

        let _spin_command_subscriber = node.create_subscription::<SpinCmdMsg, _>(
            "/cmd/spin",
            move |msg: SpinCmdMsg| {
                let client = spin_client_clone.clone();
                let log_node = spin_node.clone();
                let spinning_time = msg.spinning_time;

                let thread_state = spin_state.clone();
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
                                    let mut state = thread_state.lock().unwrap();

                                    state.status = "Spinning".to_string();
                                    state.last_command_received = Instant::now();

                                    drop(state);

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

        let timer_state = shared_state.clone();
        let timer_status_publisher = status_publisher.clone();
        let worker = node.create_worker(String::new());

        let _status_timer = worker.create_timer_repeating(
            Duration::from_millis(500), 
            move || {
                let mut state = timer_state.lock().unwrap();

                if state.last_command_received.elapsed() > Duration::from_secs(1) {
                    state.status = "Idle".to_string();
                                        
                }
                let _ = timer_status_publisher.publish(StringMsg{data: state.status.to_string()});
                drop(state);
            })?;

        Ok(CommandDispatcher {
            _node: node,
            _state: shared_state,
            _spin_command_subscriber,
            _capture_image_subscriber,
            _teleop_command_subscriber,
            _status_publisher: status_publisher,
            _status_timer,
        })
    }

}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env().unwrap().create_basic_executor();
    let _dispatcher = CommandDispatcher::new(&executor).unwrap();

    executor.spin(SpinOptions::default()).first_error()
}
