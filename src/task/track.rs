use super::normalize;
use nalgebra::{Isometry2, Translation2, Vector2};
use std::f32::consts::{FRAC_PI_3, PI};

/// 路径跟踪任务
pub struct Task {
    parameters: Parameters,
    path: Path,
    state: State,
}

enum State {
    Relocation,   // 重新搜索
    Rotation0,    // 面向目标
    GettingClose, // 接近目标
    Rotation1,    // 转向路线方向
    Tracking,     // 连续循线
}

struct Parameters {
    search_radius: f32,
    light_radius: f32,
    r#loop: bool,
}

struct Path {
    path: Vec<Vec<Isometry2<f32>>>,
    index: (usize, usize),
}

impl Task {
    pub fn new(path: Vec<Isometry2<f32>>) -> Self {
        Self {
            parameters: Parameters {
                search_radius: 5.0,
                light_radius: 1.0,
                r#loop: false,
            },
            path: Path {
                path: vec![path],
                index: (0, 0),
            },
            state: State::Tracking,
        }
    }

    /// 循线
    pub fn tack<'a>(&'a mut self, pose: &Isometry2<f32>) -> Option<f32> {
        match self.state {
            State::Tracking => {
                let mut segment = PathSegment {
                    to_robot: pose.inverse(),
                    slice: self.path.path[self.path.index.0].as_slice(),
                    index: self.path.index.1,
                    light_radius: self.parameters.light_radius,
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
                                && pos.norm() < self.parameters.light_radius
                            {
                                break;
                            } else {
                                segment.next();
                            }
                        }
                        None => return None,
                    }
                }
                self.path.index.1 = segment.index;
                Some(PI * (0.5 - segment.size_proportion()))
            }
            _ => todo!(),
        }
    }
}

/// 路径片段
#[derive(Clone)]
pub struct PathSegment<'a> {
    to_robot: Isometry2<f32>,
    slice: &'a [Isometry2<f32>],
    index: usize,
    light_radius: f32,
}

impl PathSegment<'_> {
    fn current(&self) -> Option<Isometry2<f32>> {
        self.slice
            .get(self.index)
            .and_then(|p| Some(self.to_robot * p))
    }

    pub fn size_proportion(self) -> f32 {
        let o = Vector2::new(self.light_radius, 0.0);
        let r_squared: f32 = self.light_radius * self.light_radius;

        let temp_path = self.clone();
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
        if key_nodes.is_empty() {
            println!(
                "KEY NODES IS EMPTY!!! WHY??? {:?}",
                &temp_path
                    .take(10)
                    .map(|i| format!(
                        "({} {} {})",
                        i.translation.vector[0],
                        i.translation.vector[1],
                        i.rotation.angle()
                    ))
                    .collect::<Vec<_>>()
            );
            0.5
        } else {
            let head = intersection(key_nodes.first().unwrap(), r_squared, true);
            let tail = intersection(key_nodes.last().unwrap(), r_squared, false);
            normalize(angle_of(tail) - angle_of(head), 0.0..2.0 * PI) / (2.0 * PI)
        }
    }
}

/// 通过路径片段迭代局部路径
impl Iterator for PathSegment<'_> {
    type Item = Isometry2<f32>;

    fn next(&mut self) -> Option<Self::Item> {
        self.index += 1;
        if self.index == self.slice.len() {
            return None;
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
