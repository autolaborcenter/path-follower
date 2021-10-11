use driver::{Driver, SupersivorEventForSingle::*, SupervisorForSingle};
use nalgebra::{Isometry2, Vector2};
use path_follower::controller::Controller;
use pm1_control_model::Physical;
use pm1_sdk::{odometry::Odometry, PM1Event, PM1};
use pose_filter::{InterpolationAndPredictionFilter, PoseFilter, PoseType};
use rtk_ins570::{ins570::*, RTK};
use std::{
    sync::{mpsc::*, Arc, Mutex},
    thread,
    time::{Duration, Instant},
};

enum Message {
    Pose(Isometry2<f32>),
    List,
    Record(String),
    Follow(String),
    Stop,
    Exit,
}

fn main() {
    // 从参数中解析路径
    let dir = match std::env::args().skip(1).next() {
        Some(dir) => dir,
        None => {
            eprintln!("Please pass the path directory by arg.");
            return;
        }
    };
    // 构造任务控制器
    let mut controller = match Controller::new(dir.as_str()) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("{} is not a valid path: {}", dir, e);
            return;
        }
    };
    // 构造程序消息队列
    let (sender, receiver) = sync_channel::<Message>(0);
    // 插值滤波器
    let filter = Arc::new(Mutex::new(InterpolationAndPredictionFilter::new()));

    // 控制台
    let console = sender.clone();
    std::thread::spawn(move || loop {
        let mut line = String::new();
        if std::io::stdin().read_line(&mut line).is_err() {
            let _ = console.send(Message::Exit);
            break;
        }
        let words = line.split_whitespace().collect::<Vec<_>>();
        if words.is_empty() {
            continue;
        }
        match words[0] {
            "list" => {
                if words.len() == 1 {
                    let _ = console.send(Message::List);
                }
            }
            "record" => {
                if words.len() == 2 {
                    let _ = console.send(Message::Record(words[1].into()));
                }
            }
            "follow" => {
                if words.len() == 2 {
                    let _ = console.send(Message::Follow(words[1].into()));
                }
            }
            "stop" => {
                if words.len() == 1 {
                    let _ = console.send(Message::Stop);
                }
            }
            _ => {}
        };
    });

    {
        let sender = sender.clone();
        let filter = filter.clone();
        thread::spawn(move || {
            SupervisorForSingle::<RTK>::new().join(|e| {
                match e {
                    Connected(_, _) => println!("Connected."),
                    ConnectFailed => {
                        println!("Failed.");
                        thread::sleep(Duration::from_secs(1));
                    }
                    Disconnected => {
                        println!("Disconnected.");
                        thread::sleep(Duration::from_secs(1));
                    }
                    Event(_, Some((time, event))) => match event {
                        Solution::Uninitialized(_) => {}
                        Solution::Data(data) => {
                            let SolutionData { state, enu, dir } = data;
                            let SolutionState {
                                state_pos,
                                satellites: _,
                                state_dir,
                            } = state;
                            if state_pos >= 40 && state_dir >= 30 {
                                let pose = filter.lock().unwrap().update(
                                    PoseType::Absolute,
                                    time,
                                    Isometry2::new(
                                        Vector2::new(enu.e as f32, enu.n as f32),
                                        dir as f32,
                                    ),
                                );
                                let _ = sender.send(Message::Pose(pose));
                            }
                        }
                    },
                    Event(_, None) => {}
                };
                true
            });
        });
    }

    let target = Arc::new(Mutex::new((Instant::now(), Physical::RELEASED)));
    {
        let target = target.clone();
        thread::spawn(move || {
            SupervisorForSingle::<PM1>::new().join(|e| {
                match e {
                    Connected(_, driver) => println!("Connected: {}", driver.status()),
                    ConnectFailed => {
                        println!("Failed.");
                        thread::sleep(Duration::from_secs(1));
                    }
                    Disconnected => {
                        println!("Disconnected.");
                        thread::sleep(Duration::from_secs(1));
                    }
                    Event(chassis, event) => {
                        chassis.send(*target.lock().unwrap());
                        if let Some((time, PM1Event::Odometry(Odometry { s: _, a: _, pose }))) =
                            event
                        {
                            let pose =
                                filter
                                    .lock()
                                    .unwrap()
                                    .update(PoseType::Relative, time, pose);
                            let _ = sender.send(Message::Pose(pose));
                        }
                    }
                };
                true
            });
        });
    }

    for msg in receiver {
        match msg {
            Message::Pose(pose) => {
                if let Some(proportion) = controller.put_pose(&pose) {
                    *target.lock().unwrap() = (
                        Instant::now(),
                        Physical {
                            speed: 0.25,
                            rudder: -2.0 * (proportion - 0.5),
                        },
                    );
                }
            }
            Message::List => {
                for name in controller.list() {
                    println!("{}", name);
                }
            }
            Message::Record(name) => match controller.record_to(name.as_str()) {
                Ok(_) => println!("Done"),
                Err(e) => eprintln!("Failed: {}", e),
            },
            Message::Follow(name) => match controller.follow(name.as_str()) {
                Ok(_) => println!("Done"),
                Err(e) => eprintln!("Failed: {}", e),
            },
            Message::Stop => {
                controller.stop_task();
                println!("Done");
            }
            Message::Exit => break,
        }
    }
}
