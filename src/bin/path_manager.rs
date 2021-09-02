use std::ffi::OsString;
use std::io::Write;

use nalgebra::{Isometry2, Vector2};

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

    let mut file_name = String::new();
    let mut file_to_save: Option<std::fs::File> = None;
    loop {
        let mut line = String::new();
        if std::io::stdin().read_line(&mut line).is_err() { break; }
        let words: Vec<_> = line.split_whitespace().collect();

        if words.len() == 3 && words[1] == "<<" {
            let pose = parse_isometry2(words[2]);
            if pose.is_some() {
                if file_name != words[0] {
                    file_name = words[0].to_string();
                    match std::fs::OpenOptions::new()
                        .append(true)
                        .create(true)
                        .open(format!("{}/{}.path", dir, words[0])) {
                        Ok(f) => file_to_save = Some(f),
                        Err(e) => {
                            eprintln!("Failed to open \"{}.path\", because {}", words[0], e);
                            continue;
                        }
                    }
                }
                let _ = write!(file_to_save.as_ref().unwrap(), "{}\n", words[2]);
                continue;
            }
        }
        match words[0] {
            "delete" => if words.len() == 2 {
                match std::fs::remove_file(format!("{}/{}.path", dir, words[1])) {
                    Ok(_) => { println!("\"{}.path\" is deleted.", words[1]); }
                    Err(e) => { eprintln!("Failed to delete \"{}.path\", because {}", words[1], e); }
                }
            }
            "list" => if words.len() == 1 {
                println!("{}", list_files(dir.as_str()));
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
                    match parse_path(format!("{}/{}.path", dir, words[1]).as_str()) {
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
            _ => {}
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

/// 列出文件路径下的路径文件
fn list_files(dir: &str) -> FileVec {
    FileVec(std::fs::read_dir(dir)
        .unwrap()
        .filter_map(|r| r.ok())
        .filter(|d| d.path().is_file())
        .filter(|d|
            match d.file_name().to_str() {
                Some(s) => s.ends_with(".path"),
                None => false,
            }
        )
        .collect())
}

/// 从文件解析路径
fn parse_path(path: &str) -> std::io::Result<Vec<Isometry2<f64>>> {
    Ok(std::fs::read_to_string(path)?
        .lines()
        .filter_map(|l| parse_isometry2(l))
        .collect())
}

/// 从字符串解析等距映射
fn parse_isometry2(s: &str) -> Option<Isometry2<f64>> {
    let mut i = 0;
    let mut numbers = [0.0; 3];
    for r in s.split(',').map(|s| s.trim().parse::<f64>()) {
        if i >= numbers.len() || r.is_err() { return None; }
        numbers[i] = r.unwrap();
        i += 1;
    }
    Some(Isometry2::new(Vector2::new(numbers[0], numbers[1]), numbers[2]))
}

#[test]
fn parse_test() {
    assert_eq!(parse_isometry2("-1,+2,-0"), Some(Isometry2::new(Vector2::new(-1.0, 2.0), 0.0)));
    assert_eq!(parse_isometry2("1,2,3,x"), None);
    assert_eq!(parse_isometry2(""), None);
}

struct FileVec(Vec<std::fs::DirEntry>);

impl std::fmt::Display for FileVec {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        fn cutoff(name: &OsString) -> &str { &name.to_str().unwrap()[0..name.len() - 5] }

        if self.0.is_empty() {
            write!(f, "[]")?;
        } else {
            let mut it = self.0.iter().map(|r| r.file_name());
            write!(f, "[{}", cutoff(&it.next().unwrap()))?;
            for e in it {
                write!(f, ", {}", cutoff(&e))?;
            }
            write!(f, "]")?;
        }
        Ok(())
    }
}
