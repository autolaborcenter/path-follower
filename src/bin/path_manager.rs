use nalgebra::Isometry2;
use path_follower::{controller::Controller, launch_rtk};
use pm1_control_model::Physical;
use pm1_sdk::{
    find_pm1,
    pm1::{odometry::Odometry, PM1Event},
};
use pose_filter::{InterpolationAndPredictionFilter, PoseFilter, PoseType};
use std::sync::{mpsc::*, Arc, Mutex};

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
        std::thread::spawn(move || {
            for (time, pose) in launch_rtk() {
                let pose = filter
                    .lock()
                    .unwrap()
                    .update(PoseType::Absolute, time, pose);
                let _ = sender.send(Message::Pose(pose));
            }
        });
    }

    let handle = if let Some(pm1) = find_pm1!() {
        let sender = sender;
        let filter = filter;
        let handle = pm1.get_handle();
        std::thread::spawn(move || {
            for (time, event) in pm1 {
                if let PM1Event::Odometry(Odometry { s: _, a: _, pose }) = event {
                    let pose = filter
                        .lock()
                        .unwrap()
                        .update(PoseType::Relative, time, pose);
                    let _ = sender.send(Message::Pose(pose));
                }
            }
        });
        handle
    } else {
        return;
    };

    for msg in receiver {
        match msg {
            Message::Pose(pose) => {
                if let Some(proportion) = controller.put_pose(&pose) {
                    handle.set_target(Physical {
                        speed: 0.1,
                        rudder: 2.0 * (proportion - 0.5),
                    });
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
