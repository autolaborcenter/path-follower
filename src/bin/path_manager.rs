use internal::{manage, parse_isometry2};
use internal::manage::PathFile;

enum Progress {
    Index(u32),
    Percent(f64),
}

fn main() {
    let dir = match parse_dir_from_args() {
        Ok(s) => s,
        Err(e) => {
            eprintln!("{}", e);
            return;
        }
    };
    println!("path dir: {:?}", std::fs::canonicalize(&dir).unwrap());
    println!("commands: `<file> << <pose>`, `delete <file>`, `list`, `follow <file> [progress][%]`");

    let mut file_to_save: Option<PathFile> = None;
    loop {
        let mut line = String::new();
        if std::io::stdin().read_line(&mut line).is_err() { break; }
        let words: Vec<_> = line.split_whitespace().collect();

        if words.is_empty() { continue; }
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
            "delete" => if words.len() == 2 {
                match std::fs::remove_file(format!("{}/{}.path", dir, words[1])) {
                    Ok(_) => {
                        println!("\"{}.path\" is deleted.", words[1]);
                        if PathFile::name_equals(&file_to_save, words[1]) { file_to_save = None; }
                    }
                    Err(e) => { eprintln!("Failed to delete \"{}.path\", because {}", words[1], e); }
                }
            }
            "list" => if words.len() == 1 {
                println!("{}", manage::list_files(dir.as_str()));
            }
            "follow" => if 1 < words.len() && words.len() < 4 {
                let progress = if words.len() == 3 {
                    if words[2].ends_with('%') {
                        match words[2][0..words[2].len() - 1].parse::<f64>() {
                            Ok(f) => if f < 100.0 { Progress::Percent(f / 100.0) } else { continue; }
                            Err(_) => continue
                        }
                    } else {
                        match words[2].parse::<u32>() {
                            Ok(i) => Progress::Index(i),
                            Err(_) => match words[2].parse::<f64>() {
                                Ok(f) => if f < 1.0 { Progress::Percent(f) } else { continue; }
                                Err(_) => continue
                            }
                        }
                    }
                } else { Progress::Index(0) };
                let path =
                    match manage::parse_path(format!("{}/{}.path", dir, words[1]).as_str()) {
                        Ok(p) => p,
                        Err(e) => {
                            eprintln!("Failed to load \"{}.path\", because {}", words[1], e);
                            continue;
                        }
                    };
                let index = match progress {
                    Progress::Index(i) => std::cmp::min(i, (path.len() - 1) as u32),
                    Progress::Percent(f) => (path.len() as f64 * f) as u32
                };
                println!("follow \"{}.path\": {}/{}", words[1], index, path.len() - 1)
            }
            _ => println!("commands: `<file> << <pose>`, `delete <file>`, `list`, `follow <file> [progress][%]`")
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
