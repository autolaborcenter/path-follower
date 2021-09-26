use super::normalize;
use na::{Isometry2, Translation2, Vector2};
use nalgebra as na;
use std::{
    f32::consts::{FRAC_PI_3, PI},
    fmt::{Display, Formatter},
};

/// 路径跟踪任务
pub struct Task {
    poses: Vec<na::Isometry2<f32>>,
    index: usize,
}

impl Task {
    ///
    pub fn new(poses: Vec<na::Isometry2<f32>>) -> Self {
        Self { poses, index: 0 }
    }

    /// 路径长度
    pub fn len(&self) -> usize {
        self.poses.len()
    }

    /// 跳过一些点
    pub fn jump(&mut self, len: usize) {
        self.index = (self.index + len) % self.poses.len();
    }

    /// 跳到一个点
    pub fn jump_to(&mut self, index: usize) -> Result<(), &str> {
        if index < self.poses.len() {
            self.index = index;
            Ok(())
        } else {
            Err("out of index")
        }
    }

    /// 从某点开始查找并返回路径的一个片段
    pub fn search<'a>(&'a mut self, pose: &na::Isometry2<f32>) -> Option<PathSegment<'a>> {
        let mut segment = PathSegment {
            to_robot: pose.inverse(),
            slice: self.poses.as_slice(),
            index: self.index,
            may_loop: false,
        };
        loop {
            // 查找局部起始点
            match segment.current() {
                Some(p) => {
                    let dir = p.rotation.angle();
                    let pos = p.translation.vector;
                    let pos_dir = pos.y.atan2(pos.x);
                    if dir.abs() < FRAC_PI_3 && pos_dir.abs() < FRAC_PI_3 && pos.norm() < 1.0 {
                        break;
                    } else {
                        segment.next();
                    }
                }
                None => return None,
            }
        }
        segment.index -= 1;
        self.index = segment.index;
        Some(segment)
    }
}

impl Display for Task {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}%({}/{})",
            (self.index * 100 / (self.poses.len() - 1)),
            self.index,
            self.poses.len() - 1
        )
    }
}

/// 路径片段
pub struct PathSegment<'a> {
    to_robot: Isometry2<f32>,
    slice: &'a [Isometry2<f32>],
    index: usize,
    may_loop: bool,
}

impl PathSegment<'_> {
    fn current(&self) -> Option<Isometry2<f32>> {
        self.slice
            .get(self.index)
            .and_then(|p| Some(self.to_robot * p))
    }

    pub fn size_proportion(self) -> f32 {
        const X: f32 = 1.0;
        const R: f32 = 1.0;
        const R_SQUARED: f32 = R * R;
        let o = Vector2::new(X, 0.0);

        let key_nodes: Vec<_> = self
            .map_while(|p| {
                let vector = p.translation.vector - o;
                if vector.norm_squared() < R_SQUARED {
                    Some(Isometry2 {
                        translation: Translation2 { vector },
                        ..p
                    })
                } else {
                    None
                }
            })
            .collect();
        let head = intersection(key_nodes.first().unwrap(), R_SQUARED, true);
        let tail = intersection(key_nodes.last().unwrap(), R_SQUARED, false);
        normalize(angle_of(tail) - angle_of(head), 0.0..2.0 * PI) / (2.0 * PI)
    }
}

/// 通过路径片段迭代局部路径
impl Iterator for PathSegment<'_> {
    type Item = Isometry2<f32>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index == self.slice.len() {
            if self.may_loop {
                self.index = 0;
            } else {
                return None;
            }
        } else {
            self.index += 1;
        }
        Some(self.to_robot * self.slice[self.index])
    }
}

fn dir_vector(p: &Isometry2<f32>) -> Vector2<f32> {
    let rad = p.rotation.complex();
    Vector2::new(rad.re, rad.im)
}

fn angle_of(p: Vector2<f32>) -> f32 {
    p.y.atan2(p.x)
}

fn intersection(p: &Isometry2<f32>, r_squared: f32, negtive: bool) -> Vector2<f32> {
    let vp = p.translation.vector;
    let vd = dir_vector(p);

    let a = vd.norm_squared();
    let b = vp.dot(&vd) * 2.0;
    let c = vp.norm_squared() - r_squared;

    let delta = (b * b - 4.0 * a * c).sqrt();
    if negtive {
        vp + vd * (-b - delta) / (2.0 * a)
    } else {
        vp + vd * (-b + delta) / (2.0 * a)
    }
}
