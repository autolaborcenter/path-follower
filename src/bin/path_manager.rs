use nalgebra::Isometry2;
use path_follower::{locator, Controller};
use std::sync::mpsc::*;

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

    let module = sender;
    std::thread::spawn(move || {
        let locator = locator::Locator::new();
        for pose in locator {
            let _ = module.send(Message::Pose(pose));
        }
    });

    for msg in receiver {
        match msg {
            Message::Pose(pose) => controller.put_pose(&pose),
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
