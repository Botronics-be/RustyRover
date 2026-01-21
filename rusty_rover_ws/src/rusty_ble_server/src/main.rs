use rclrs::*;
use std::{thread, time::Duration};
use std_msgs::msg::String as StringMsg;

pub struct Middleman {
    worker: Worker<Option<String>>,
    publisher: Publisher<StringMsg>,
}

impl Middleman {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("simple_subscription")?;
        let worker = node.create_worker(None);
        let publisher = node.create_publisher("republishing_hello")?;
        worker.create_subscription::<StringMsg, _>(
            "publish_hello",
            move |data: &mut Option<String>, msg: StringMsg| {
                *data = Some(msg.data);
            },
        )?;

        Ok(Self { worker, publisher })
    }

    fn republish(&self) -> Result<(), RclrsError> {
        let publisher = self.publisher.clone();
        drop(
            self.worker
                .run(move |data: &mut Option<String>| -> Result<(), RclrsError> {
                    data.clone()
                        .map(|message| {
                            publisher.publish(StringMsg {
                                data: message.clone(),
                            })
                        })
                        .ok_or(RclrsError::RclError {
                            code: RclReturnCode::SubscriptionTakeFailed,
                            msg: None,
                        })??;
                    Ok(())
                }),
        );
        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = Middleman::new(&executor)?;

    thread::spawn(move || {
        loop {
            thread::sleep(Duration::from_secs(1));
            if let Err(e) = node.republish() {
                eprintln!("Republish error: {}", e);
            }
        }
    });

    executor.spin(SpinOptions::default()).first_error()
}