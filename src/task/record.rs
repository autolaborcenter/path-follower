use super::normalize;
use crate::nameof;
use na::Isometry2;
use nalgebra as na;
use std::f32::consts::PI;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

/// 路径文件
pub struct Task {
    file: File,
    path: PathBuf,
    last: Option<Isometry2<f32>>,
}

impl Task {
    pub fn new(path: PathBuf) -> std::io::Result<Self> {
        Ok(Self {
            file: File::open(&path)?,
            path,
            last: None,
        })
    }

    /// 将一个点追加到路径文件
    pub fn append(&mut self, pose: &na::Isometry2<f32>) {
        if let Some(last) = self.last {
            let delta = last * pose;
            // 5cm ~ 20cm
            // 90° ~ 0°
            let rho = delta.translation.vector.norm();
            let theta = delta.rotation.angle();
            let value = rho + normalize(theta, -PI..PI).abs() / PI * 0.3;
            if rho < 0.05 || value < 0.2 {
                return;
            }
        }
        self.last = Some(pose.inverse());
        let _ = write!(
            self.file,
            "{},{},{}\n",
            pose.translation.x,
            pose.translation.y,
            pose.rotation.angle()
        );
    }

    /// 打开的文件名
    pub fn name<'a>(&'a self) -> &'a str {
        nameof(&self.path)
    }
}
