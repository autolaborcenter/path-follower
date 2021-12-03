use std::f32::consts::PI;

use crate::vector;
use nalgebra::{Complex, Isometry2, Vector2};

/// 计算面积比并转化到 [-π/2, π/2]
pub fn track(part: &[Isometry2<f32>], light_radius: f32) -> Option<f32> {
    let squared = light_radius.powi(2);
    let delta = vector(light_radius, 0.0);

    // 查找路段起点、终点
    let mut begin = part[0];
    begin.translation.vector -= delta;
    if begin.translation.vector.norm_squared() > squared {
        return None;
    }
    let end = part
        .iter()
        .skip(1)
        .map(|p| {
            let mut p = *p;
            p.translation.vector -= delta;
            p
        })
        .take_while(|p| p.translation.vector.norm_squared() < squared)
        .last()
        .unwrap_or(part[0]);

    let begin = intersection(&begin, squared, -1.0);
    let end = intersection(&end, squared, 1.0);
    let diff = angle_of(end) - angle_of(begin); // [-2π, 2π]
    Some((diff.signum() * PI - diff) / 2.0) // [-π/2, π/2]
}

/// 求射线与圆交点
fn intersection(p: &Isometry2<f32>, r_squared: f32, signnum: f32) -> Vector2<f32> {
    let vp = p.translation.vector;
    let vd = dir_vector(p);

    // let a = 1.0;
    let b = 2.0 * vp.dot(&vd);
    let c = vp.norm_squared() - r_squared;

    #[allow(non_snake_case)]
    let Δ = b.powi(2) - 4.0 * c;
    let k = (-b + signnum * Δ.sqrt()) / 2.0;
    vp + vd * k
}

/// 求方向向量
#[inline]
fn dir_vector(p: &Isometry2<f32>) -> Vector2<f32> {
    let Complex { re, im } = *p.rotation.complex();
    vector(re, im)
}

/// 求方向角
#[inline]
fn angle_of(p: Vector2<f32>) -> f32 {
    p.y.atan2(p.x)
}
