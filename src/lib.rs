pub mod task;

use crate::task::{follow, record};
use na::Isometry2;
use nalgebra as na;
use std::{fs::File, io::BufReader, io::Read, path::PathBuf};
use task::Task;

/// 任务控制器
pub struct Controller {
    repo_path: PathBuf,
    task: Option<Task>,
}

impl Controller {
    /// 新建控制器
    /// 若仓库路径不存在，建立新仓库
    pub fn new(path: &str) -> std::io::Result<Self> {
        let repo_path = PathBuf::from(path);
        if !repo_path.is_dir() {
            std::fs::create_dir_all(path)?;
        }
        Ok(Self {
            repo_path,
            task: None,
        })
    }

    /// 列出仓库中的路径文件
    pub fn list(&self) -> Vec<String> {
        match std::fs::read_dir(&self.repo_path) {
            Ok(dir) => dir
                .filter_map(|r| r.ok())
                .filter_map(|d| {
                    let path = d.path();
                    if path.is_file() && path.ends_with(".path") {
                        Some(nameof(&path).to_string())
                    } else {
                        None
                    }
                })
                .collect(),
            Err(_) => Vec::new(),
        }
    }

    /// 从名字构造文件的绝对路径
    fn build_absolute(&self, name: &str) -> PathBuf {
        let mut path = self.repo_path.clone();
        path.push(format!("{}.path", name));
        path
    }

    /// 开始录制
    pub fn record_to(&mut self, name: &str) -> std::io::Result<()> {
        self.task = Pipe(name)
            .then(|it| self.build_absolute(it))
            .maybe(|it| record::Task::new(it))?
            .then(|it| Task::Record(it))
            .then(|it| Some(it))
            .finally();
        Ok(())
    }

    /// 开始循径
    pub fn follow(&mut self, name: &str) -> std::io::Result<()> {
        fn read_to_string(mut reader: BufReader<File>) -> std::io::Result<String> {
            let mut contents = String::new();
            reader.read_to_string(&mut contents)?;
            Ok(contents)
        }

        self.task = Pipe(name)
            .then(|it| self.build_absolute(it))
            .maybe(|it| File::open(it))?
            .then(|it| BufReader::new(it))
            .maybe(|it| read_to_string(it))?
            .then(|it| it.lines().filter_map(parse_isometry2).collect())
            .then(|it| follow::Task::new(it))
            .then(|it| Task::Follow(it))
            .then(|it| Some(it))
            .finally();
        Ok(())
    }

    /// 取消任务
    pub fn stop_task(&mut self) {
        self.task = None;
    }

    pub fn put_pose(&mut self, pose: &Isometry2<f32>) {
        if let Some(ref mut task) = self.task {
            match task {
                Task::Record(record) => record.append(pose),
                Task::Follow(follow) => match follow.search(pose) {
                    Some(seg) => {
                        println!("proportion = {}", seg.size_proportion());
                    }
                    None => {
                        eprintln!("The robot is too far away from any node of the path.")
                    }
                },
            }
        }
    }
}

/// 截取路径名
fn nameof<'a>(path: &'a PathBuf) -> &'a str {
    let name = path.file_name().unwrap();
    &name.to_str().unwrap()[..name.len() - 5]
}

/// 从字符串解析等距映射
fn parse_isometry2(s: &str) -> Option<na::Isometry2<f32>> {
    let mut i = 0;
    let mut numbers = [0.0; 3];
    for r in s.split(',').map(|s| s.trim().parse::<f32>()) {
        if i >= numbers.len() || r.is_err() {
            return None;
        }
        numbers[i] = r.unwrap();
        i += 1;
    }
    Some(na::Isometry2::new(
        na::Vector2::new(numbers[0], numbers[1]),
        numbers[2],
    ))
}

#[test]
fn test_parse() {
    assert_eq!(
        parse_isometry2("-1,+2,-0"),
        Some(na::Isometry2::new(na::Vector2::new(-1.0, 2.0), 0.0))
    );
    assert_eq!(parse_isometry2("1,2,3,x"), None);
    assert_eq!(parse_isometry2(""), None);
}

struct Pipe<T>(T);

impl<T> Pipe<T> {
    #[inline]
    fn then<F, U>(self, f: F) -> Pipe<U>
    where
        F: FnOnce(T) -> U,
    {
        Pipe(f(self.0))
    }

    #[inline]
    fn maybe<F, U>(self, f: F) -> std::io::Result<Pipe<U>>
    where
        F: FnOnce(T) -> std::io::Result<U>,
    {
        Ok(Pipe(f(self.0)?))
    }

    #[inline]
    fn finally(self) -> T {
        self.0
    }
}
