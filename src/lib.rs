use nalgebra::{Isometry2, Point2, Vector2};
use std::path::PathBuf;

mod path;
mod task;

pub use path::*;
use task::{record, track, Task};

/// 扇形
#[derive(Clone, Copy)]
pub struct Sector {
    pub radius: f32,
    pub angle: f32,
}

/// 判断是否在扇形内
#[derive(Clone, Copy)]
pub struct InsideSectorChecker {
    squared: f32,
    half: f32,
}

impl Sector {
    #[inline]
    pub fn get_checker(&self) -> InsideSectorChecker {
        InsideSectorChecker {
            squared: self.radius.powi(2),
            half: self.angle * 0.5,
        }
    }
}

impl InsideSectorChecker {
    #[inline]
    pub fn contains(&self, p: Point2<f32>) -> bool {
        p.coords.norm_squared() < self.squared && p[1].atan2(p[0]).abs() < self.half
    }

    #[inline]
    pub fn contains_pose(&self, p: Isometry2<f32>) -> bool {
        self.contains(Point2 {
            coords: p.translation.vector,
        }) && p.rotation.angle().abs() < self.half
    }
}

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

    /// 列出仓库中的路径文件
    pub fn list(&self) -> Vec<String> {
        match std::fs::read_dir(&self.repo_path) {
            Ok(dir) => dir
                .filter_map(|r| r.ok())
                .filter_map(|d| {
                    let path = d.path();
                    if path.is_file() && path.extension().map_or(false, |s| s == "path") {
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
    fn build_absolute(&self, name: &str) -> async_std::path::PathBuf {
        let mut path = self.repo_path.clone();
        path.push(format!("{}.path", name));
        path.into()
    }

    fn read_stream(&self, name: &str) -> std::io::Result<track::Task> {
        Ok(self.build_absolute(name))
            .and_then(|it| async_std::task::block_on(PathFile::open(it.as_path())))
            .map(|it| track::Task::new(it, track::Parameters::DEFAULT))
    }

    /// 读取指定路径
    pub fn read(&self, name: &str) -> std::io::Result<Vec<Isometry2<f32>>> {
        self.read_stream(name)
            .map(|it| it.view().iter().flat_map(|v| v).map(|x| *x).collect())
    }

    /// 开始录制
    pub fn record_to(&mut self, name: &str) -> std::io::Result<()> {
        if let Some((ref current, _)) = self.task {
            if name == current {
                return Ok(());
            }
        }
        Ok(self.build_absolute(name))
            .and_then(|it| record::Task::new(it.as_path()))
            .map(|it| self.task = Some((name.into(), Task::Record(it))))
    }

    /// 开始循径
    pub fn track(&mut self, name: &str) -> std::io::Result<()> {
        if let Some((ref current, _)) = self.task {
            if name == current {
                return Ok(());
            }
        }
        self.read_stream(name)
            .map(|it| self.task = Some((name.into(), Task::Follow(it))))
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

#[inline]
const fn isometry(x: f32, y: f32, cos: f32, sin: f32) -> Isometry2<f32> {
    use nalgebra::{Complex, Translation, Unit};
    Isometry2 {
        translation: Translation {
            vector: vector(x, y),
        },
        rotation: Unit::new_unchecked(Complex { re: cos, im: sin }),
    }
}

#[inline]
const fn point(x: f32, y: f32) -> Point2<f32> {
    Point2 {
        coords: vector(x, y),
    }
}

#[inline]
const fn vector(x: f32, y: f32) -> Vector2<f32> {
    use nalgebra::{ArrayStorage, Vector};
    Vector::from_array_storage(ArrayStorage([[x, y]]))
}
