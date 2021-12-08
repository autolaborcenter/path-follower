use crate::{path, point, RelocateConfig, Sector};
use nalgebra::Isometry2;
use std::f32::consts::PI;

pub(super) struct Path {
    pub inner: path::Path,
    index: (usize, usize),
}

pub(super) enum TrackError {
    OutOfPath,
    Termination,
}

impl Path {
    pub fn new(path: path::Path) -> Self {
        Self {
            inner: path,
            index: (0, 0),
        }
    }

    /// 根据当前位姿重定位
    ///
    /// 将遍历整个路径，代价极大且计算密集
    pub fn relocate(
        &mut self,
        pose: &Isometry2<f32>,
        light_radius: f32,
        search_radius: f32,
        r#loop: bool,
    ) -> bool {
        if let Some(i) = self.inner.relocate(RelocateConfig {
            pose: *pose,
            index: self.index,
            light_radius,
            search_range: Sector {
                radius: search_radius,
                angle: PI,
            },
            r#loop,
        }) {
            self.index = i;
            true
        } else {
            false
        }
    }

    /// 在当前路段搜索并产生控制量
    pub fn track_within(
        &mut self,
        pose: &Isometry2<f32>,
        light_radius: f32,
    ) -> Result<f32, TrackError> {
        let c = (pose * point(light_radius, 0.0)).coords;
        let squared = light_radius.powi(2);

        // 遍历当前路段
        let first = self.inner.0[self.index.0]
            .iter()
            .enumerate()
            .skip(self.index.1)
            .find(|(_, p)| (c - p.translation.vector).norm_squared() < squared);

        // 生成控制量或异常
        if let Some((j, _)) = first {
            self.index.1 = j;
            Ok(
                crate::track::track(self.inner.slice(self.index).iter().copied(), light_radius)
                    .unwrap(),
            )
        } else if self.inner.slice(self.index).len() < 2 {
            Err(TrackError::Termination)
        } else {
            Err(TrackError::OutOfPath)
        }
    }

    /// 尝试加载下一个路段
    pub fn next_segment(&mut self, r#loop: bool) -> bool {
        if self.index.0 == self.inner.0.len() - 1 {
            if !r#loop {
                return false;
            }
            self.index.0 = 0;
        } else {
            self.index.0 += 1;
        }
        self.index.1 = 0;
        true
    }

    /// 生成初始化动作
    pub fn initialize(&self, pose: &Isometry2<f32>, light_radius: f32) -> Option<(f32, f32)> {
        crate::track::goto(pose.inv_mul(&self.inner.slice(self.index)[0]), light_radius)
    }
}
