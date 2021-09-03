use internal::{follow, manage, parse_isometry2};
use internal::follow::{FollowTask, Progress};
use internal::manage::PathFile;

const HELP: &str = "commands: `<file> << <pose>`, `delete <file>`, `list`, `follow <file> [progress][%]`, `search <pose>`";

fn main() {
    let dir = match parse_dir_from_args() {
        Ok(s) => s,
        Err(e) => {
            eprintln!("{}", e);
            return;
        }
    };
    println!("path dir: {:?}", std::fs::canonicalize(&dir).unwrap());
    println!("{}", HELP);

    let mut file_to_save: Option<PathFile> = None;
    let mut follow_task: Option<FollowTask> = None;
    loop {
        let mut line = String::new();
        if std::io::stdin().read_line(&mut line).is_err() { break; }
        let words: Vec<_> = line.split_whitespace().collect();

        if words.is_empty() { continue; }
        // 保存路径点
        if words.len() == 3 && words[1] == "<<" {
            match parse_isometry2(words[2]) {
                Some(pose) => {
                    if !PathFile::name_equals(&file_to_save, words[0]) {
                        match PathFile::new(dir.as_str(), words[0]) {
                            Ok(file) => file_to_save = Some(file),
                            Err(e) => {
                                eprintln!("Failed to open \"{}.path\", because {}", words[0], e);
                                continue;
                            }
                        }
                    }
                    file_to_save.as_mut().unwrap().append(&pose);
                    continue;
                }
                None => {}
            };
        }
        match words[0] {
            // 删除路径文件
            "delete" => if words.len() == 2 {
                match std::fs::remove_file(format!("{}/{}.path", dir, words[1])) {
                    Ok(_) => {
                        println!("\"{}.path\" is deleted.", words[1]);
                        if PathFile::name_equals(&file_to_save, words[1]) { file_to_save = None; }
                    }
                    Err(e) => { eprintln!("Failed to delete \"{}.path\", because {}", words[1], e); }
                }
            }
            // 列出现有文件
            "list" => if words.len() == 1 {
                println!("{}", manage::list_files(dir.as_str()));
            }
            // 启动跟踪
            "follow" => if 1 < words.len() && words.len() < 4 {
                let progress =
                    if words.len() == 3 {
                        match words[2].parse() {
                            Ok(p) => p,
                            Err(_) => continue,
                        }
                    } else {
                        Progress::Index(0)
                    };
                let path =
                    match follow::parse_path(dir.as_str(), words[1]) {
                        Ok(p) => p,
                        Err(e) => {
                            eprintln!("Failed to load \"{}.path\", because {}", words[1], e);
                            continue;
                        }
                    };
                follow_task = Some(FollowTask::new(path, progress));
                println!("follow \"{}.path\": {}", words[1], follow_task.as_ref().unwrap())
            }
            // 搜索当前路径
            "search" => if words.len() == 2 && follow_task.is_some() {
                match parse_isometry2(words[1]) {
                    Some(pose) => {
                        let local = follow_task
                            .as_mut()
                            .unwrap()
                            .search(pose, |delta| {
                                const LIMIT: f64 = std::f64::consts::FRAC_PI_3;
                                let len = delta.translation.vector.norm();
                                let v = delta.translation;
                                let angle1 = v.y.atan2(v.x);
                                let angle2 = delta.rotation.angle();
                                len < 0.5
                                    && -LIMIT < angle1 && angle1 < LIMIT
                                    && -LIMIT < angle2 && angle2 < LIMIT
                            });
                        print!("L ");
                        for p in local {
                            print!("{},{},{} ", p.translation.x, p.translation.y, p.rotation.angle());
                        }
                        println!();
                    }
                    None => {}
                }
            }
            _ => println!("{}", HELP)
        }
    }
}

/// 从参数解析路径文件路径
fn parse_dir_from_args() -> Result<String, &'static str> {
    match
    std::env::args()
        .skip(1)
        .next()
        .filter(|d| std::path::Path::new(&d).is_dir())
    {
        Some(s) => Ok(s),
        None => Err("Please pass the path directory by arg."),
    }
}
