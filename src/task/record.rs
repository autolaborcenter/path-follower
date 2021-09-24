use crate::nameof;
use nalgebra as na;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

/// 路径文件
pub struct Task(File, PathBuf);

impl Task {
    pub fn new(path: PathBuf) -> std::io::Result<Self> {
        Ok(Self(File::open(&path)?, path))
    }

    /// 将一个点追加到路径文件
    pub fn append(&mut self, pose: &na::Isometry2<f64>) {
        let _ = write!(
            self.0,
            "{},{},{}\n",
            pose.translation.x,
            pose.translation.y,
            pose.rotation.angle()
        );
    }

    /// 打开的文件名
    pub fn name<'a>(&'a self) -> &'a str {
        nameof(&self.1)
    }
}
