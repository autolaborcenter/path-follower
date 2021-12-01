use crate::{isometry, vector};
use async_std::{
    fs::File,
    io::{prelude::BufReadExt, BufReader},
    task,
};
use nalgebra::{Isometry2, Vector2};

/// 打开的路径文件
pub struct PathFile(BufReader<File>);

/// 分段的路径
pub struct Path(pub Vec<Vec<Isometry2<f32>>>);

impl PathFile {
    #[inline]
    pub async fn open(path: &async_std::path::Path) -> std::io::Result<Self> {
        Ok(Self(BufReader::new(File::open(path).await?)))
    }
}

impl Iterator for PathFile {
    type Item = Isometry2<f32>;

    fn next(&mut self) -> Option<Self::Item> {
        let mut line = String::new();
        task::block_on(self.0.read_line(&mut line)).ok()?;
        let mut numbers = line.trim().split(',');
        let x = numbers.next()?.parse().ok()?;
        let y = numbers.next()?.parse().ok()?;
        let (sin, cos) = numbers.next()?.parse::<f32>().ok()?.sin_cos();
        Some(isometry(x, y, cos, sin))
    }
}

impl Path {
    /// 读取一条路径，检测其中的尖点
    pub fn new(
        path: impl IntoIterator<Item = Isometry2<f32>>,
        light_radius: f32,
        tip_ignore: usize,
    ) -> Self {
        // 光斑中心位置
        let c = vector(light_radius, 0.0);
        // 光斑范围
        let squared = light_radius.powi(2);

        let mut source = path.into_iter();
        let mut result = Self(vec![vec![]]);
        if let Some(mut reference) = source.next() {
            // reference 理解为上一个点的位姿，即处理这一个点时机器人预期的位姿
            result.0.last_mut().unwrap().push(reference);
            for p in source {
                // 使用 reference 逆变换，即将这一个点变换到理想的机器人坐标系
                let segment = result.0.last_mut().unwrap();
                // 如果这一个点在光斑内 且 与机器人差不多同向
                // 认为是一般的点
                if is_continious(&reference.inv_mul(&p), c, squared) {
                    segment.push(p);
                }
                // 判定为尖点
                else {
                    // 反向检查，看是否跳过一些点可以实现连续
                    let remain = segment
                        .iter()
                        .rev()
                        .take(tip_ignore + 1)
                        .skip_while(|r| !is_continious(&r.inv_mul(&p), c, squared))
                        .count();
                    if remain > 0 {
                        segment.truncate(segment.len() - (tip_ignore + 1) + remain);
                        segment.push(p);
                    } else {
                        result.0.push(vec![p]);
                    }
                }
                // 更新参考点
                reference = p;
            }
        }
        result
    }
}

#[inline]
fn is_continious(local: &Isometry2<f32>, c: Vector2<f32>, squared: f32) -> bool {
    use std::f32::consts::FRAC_PI_3;
    (local.translation.vector - c).norm_squared() < squared
        && local.rotation.angle().abs() < FRAC_PI_3
}
