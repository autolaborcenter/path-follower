use std::f32::consts::PI;

use async_std::{
    fs::{create_dir_all, File},
    io::WriteExt,
    path::Path,
};
use nalgebra::Isometry2;

pub struct RecordFile(File, Isometry2<f32>);

impl RecordFile {
    pub async fn new(path: &Path, pose: Isometry2<f32>) -> async_std::io::Result<Self> {
        if let Some(dir) = path.parent() {
            create_dir_all(dir).await?;
        }
        let mut file = File::create(path).await?;
        file.write_all(fmt(pose).as_bytes()).await?;
        Ok(Self(file, pose))
    }

    pub async fn record(&mut self, pose: Isometry2<f32>) -> async_std::io::Result<bool> {
        let delta = self.1 * pose;
        // 5cm ~ 20cm
        // 90° ~ 0°
        let rho = delta.translation.vector.norm();
        // 0.05 + (90°/180°)*0.3 == 0.2
        if rho < 0.05 || rho + delta.rotation.angle().abs() / PI * 0.3 < 0.2 {
            Ok(false)
        } else {
            self.0.write_all(fmt(pose).as_bytes()).await?;
            self.1 = pose.inverse();
            Ok(true)
        }
    }
}

#[inline]
fn fmt(pose: Isometry2<f32>) -> String {
    format!(
        "{},{},{}\n",
        pose.translation.x,
        pose.translation.y,
        pose.rotation.angle()
    )
}
