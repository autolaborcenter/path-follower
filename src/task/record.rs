use nalgebra::Isometry2;
use std::{f32::consts::PI, fs::File, io::Write, path::PathBuf};

/// 路径文件
pub struct Task {
    file: File,
    last: Option<Isometry2<f32>>,
}

impl Task {
    pub fn new(path: PathBuf) -> std::io::Result<Self> {
        Ok(Self {
            file: File::create(&path)?,
            last: None,
        })
    }

    /// 将一个点追加到路径文件
    pub fn append(&mut self, pose: &Isometry2<f32>) {
        if let Some(last) = self.last {
            let delta = last * pose;
            // 5cm ~ 20cm
            // 90° ~ 0°
            let rho = delta.translation.vector.norm();
            let theta = delta.rotation.angle();
            let value = rho + theta.abs() / PI * 0.3;
            if rho < 0.05 || value < 0.2 {
                return;
            }
        }
        self.last = Some(pose.inverse());
        let text = format!(
            "{},{},{}",
            pose.translation.x,
            pose.translation.y,
            pose.rotation.angle()
        );
        let _ = write!(self.file, "{}\n", text);
        println!("saved: {}", text);
    }
}
