mod task;

use nalgebra::{Isometry2, Vector2};
use std::{
    fs::File,
    io::{BufReader, Read},
    path::{Path, PathBuf},
};
use task::{record, track, Task};

/// 任务控制器
pub struct Tracker {
    repo_path: PathBuf,
    task: Option<(String, Task)>,
}

impl Tracker {
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

    /// 获取仓库路径
    pub fn path<'a>(&'a self) -> &'a Path {
        self.repo_path.as_path()
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

    fn read_to_string(mut reader: BufReader<File>) -> std::io::Result<String> {
        let mut contents = String::new();
        reader.read_to_string(&mut contents)?;
        Ok(contents)
    }

    /// 读取指定路径
    pub fn read(&self, name: &str) -> std::io::Result<Vec<Isometry2<f32>>> {
        Pipe(name)
            .then(|it| self.build_absolute(it))
            .maybe(|it| File::open(it))?
            .then(|it| BufReader::new(it))
            .maybe(|it| Self::read_to_string(it))?
            .then(|it| Ok(it.lines().filter_map(parse_isometry2).collect()))
            .finally()
    }

    /// 开始录制
    pub fn record_to(&mut self, name: &str) -> std::io::Result<()> {
        if let Some((ref current, _)) = self.task {
            if name == current {
                return Ok(());
            }
        }
        self.task = Pipe(name)
            .then(|it| self.build_absolute(it))
            .maybe(|it| record::Task::new(it))?
            .then(|it| Task::Record(it))
            .then(|it| Some((name.into(), it)))
            .finally();
        Ok(())
    }

    /// 开始循径
    pub fn track(&mut self, name: &str) -> std::io::Result<()> {
        if let Some((ref current, _)) = self.task {
            if name == current {
                return Ok(());
            }
        }
        self.task = Pipe(name)
            .maybe(|it| self.read(it))?
            .then(|it| track::Task::new(it, track::Parameters::DEFAULT))
            .then(|it| Task::Follow(it))
            .then(|it| Some((name.into(), it)))
            .finally();
        Ok(())
    }

    /// 取消任务
    pub fn stop_task(&mut self) {
        self.task = None;
    }

    /// （向任务）传入位姿
    pub fn put_pose(&mut self, pose: &Isometry2<f32>) -> Option<(f32, f32)> {
        self.task.as_mut().and_then(|(_, ref mut task)| match task {
            Task::Record(record) => {
                record.append(pose);
                None
            }
            Task::Follow(follow) => match follow.track(pose) {
                Ok(value) => Some(value),
                Err(e) => {
                    eprintln!("{:?}", e);
                    None
                }
            },
        })
    }
}

/// 截取路径名
fn nameof<'a>(path: &'a PathBuf) -> &'a str {
    let name = path.file_name().unwrap();
    &name.to_str().unwrap()[..name.len() - 5]
}

/// 从字符串解析等距映射
fn parse_isometry2(s: &str) -> Option<Isometry2<f32>> {
    let mut i = 0;
    let mut numbers = [0.0; 3];
    for r in s.split(',').map(|s| s.trim().parse::<f32>()) {
        if i >= numbers.len() || r.is_err() {
            return None;
        }
        numbers[i] = r.unwrap();
        i += 1;
    }
    Some(Isometry2::new(
        Vector2::new(numbers[0], numbers[1]),
        numbers[2],
    ))
}

#[test]
fn test_parse() {
    assert_eq!(
        parse_isometry2("-1,+2,-0"),
        Some(Isometry2::new(Vector2::new(-1.0, 2.0), 0.0))
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
