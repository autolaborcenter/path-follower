use path_follower::Controller;

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

    loop {
        let mut line = String::new();
        if std::io::stdin().read_line(&mut line).is_err() {
            break;
        }
        let words = line.split_whitespace().collect::<Vec<_>>();
        if words.is_empty() {
            continue;
        }
        match words[0] {
            "list" => {
                if words.len() == 1 {
                    for file in controller.list() {
                        println!("{}", file);
                    }
                }
            }
            "record" => {
                if words.len() == 2 {
                    match controller.record_to(words[1]) {
                        Ok(_) => println!("Done."),
                        Err(e) => eprintln!("failed: {}", e),
                    };
                }
            }
            "stop" => {
                if words.len() == 1 {
                    controller.stop_task();
                    println!("Done.")
                }
            }
            _ => {}
        }
    }
}
