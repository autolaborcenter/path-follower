use super::normalize;
use nalgebra::{Isometry2, Translation2, Vector2};
use std::{
    f32::consts::{FRAC_PI_3, PI},
    fmt::{Display, Formatter},
};

/// 路径跟踪任务
pub struct Task {
    path: Vec<Isometry2<f32>>,
    index: usize,
    light_radius: f32,
}

impl Task {
    pub fn new(path: Vec<Isometry2<f32>>) -> Self {
        Self {
            path,
            index: 0,
            light_radius: 1.0,
        }
    }

    /// 路径长度
    pub fn len(&self) -> usize {
        self.path.len()
    }

    /// 跳过一些点
    pub fn jump(&mut self, len: usize) {
        self.index = (self.index + len) % self.path.len();
    }

    /// 跳到一个点
    pub fn jump_to(&mut self, index: usize) -> Result<(), &str> {
        if index < self.path.len() {
            self.index = index;
            Ok(())
        } else {
            Err("out of index")
        }
    }

    /// 从某点开始查找并返回路径的一个片段
    pub fn search<'a>(&'a mut self, pose: &Isometry2<f32>) -> Option<PathSegment<'a>> {
        let mut segment = PathSegment {
            to_robot: pose.inverse(),
            slice: self.path.as_slice(),
            index: self.index,
            may_loop: false,
            light_radius: self.light_radius,
        };
        loop {
            // 查找局部起始点
            match segment.current() {
                Some(p) => {
                    let dir = p.rotation.angle();
                    let pos = p.translation.vector;
                    let pos_dir = pos.y.atan2(pos.x);
                    if dir.abs() < FRAC_PI_3
                        && pos_dir.abs() < FRAC_PI_3
                        && pos.norm() < self.light_radius
                    {
                        break;
                    } else {
                        segment.next();
                    }
                }
                None => return None,
            }
        }
        self.index = segment.index;
        Some(segment)
    }
}

impl Display for Task {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}%({}/{})",
            (self.index * 100 / (self.path.len() - 1)),
            self.index,
            self.path.len() - 1
        )
    }
}

/// 路径片段
pub struct PathSegment<'a> {
    to_robot: Isometry2<f32>,
    slice: &'a [Isometry2<f32>],
    index: usize,
    may_loop: bool,
    light_radius: f32,
}

impl PathSegment<'_> {
    fn current(&self) -> Option<Isometry2<f32>> {
        self.slice
            .get(self.index)
            .and_then(|p| Some(self.to_robot * p))
    }

    pub fn size_proportion(self) -> f32 {
        const C: f32 = 1.0;
        let r_squared: f32 = self.light_radius * self.light_radius;
        let o = Vector2::new(C, 0.0);

        let key_nodes: Vec<_> = self
            // map_while (unstable)
            .map(|p| {
                let vector = p.translation.vector - o;
                if vector.norm_squared() < r_squared {
                    Some(Isometry2 {
                        translation: Translation2 { vector },
                        ..p
                    })
                } else {
                    None
                }
            })
            .take_while(|o| o.is_some())
            .map(|o| o.unwrap())
            .collect();
        let head = intersection(key_nodes.first().unwrap(), r_squared, true);
        let tail = intersection(key_nodes.last().unwrap(), r_squared, false);
        normalize(angle_of(tail) - angle_of(head), 0.0..2.0 * PI) / (2.0 * PI)
    }
}

/// 通过路径片段迭代局部路径
impl Iterator for PathSegment<'_> {
    type Item = Isometry2<f32>;

    fn next(&mut self) -> Option<Self::Item> {
        self.index += 1;
        if self.index == self.slice.len() {
            if self.may_loop {
                self.index = 0;
            } else {
                return None;
            }
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
