use nalgebra::{Isometry2, Point2, Vector2};

mod path;
mod record;
mod track;

pub use path::{Path, PathFile};
pub use record::RecordFile;

#[derive(Clone)]
pub struct Tracker<'a> {
    pub path: &'a Path,
    pub context: TrackContext,
}

#[derive(Clone)]
pub struct TrackContext {
    pub parameters: Parameters,
    pub index: (usize, usize),
    pub state: State,
}

#[derive(Clone, Copy)]
pub struct Parameters {
    pub search_range: Sector,    // 搜索范围
    pub light_radius: f32,       // 光斑半径
    pub r#loop: bool,            // 是否启动循环
    pub auto_reinitialize: bool, // 离开路径是否自动恢复
}

#[derive(Clone, Copy, Debug)]
pub enum State {
    Relocating,   // 重新搜索
    Initializing, // 初始化
    Tracking,     // 连续循线
}

/// 跟踪失败信息
#[derive(Debug)]
pub enum Error {
    RelocationFailed, // 搜索失败
    OutOfPath,        // 丢失路径
    Complete,         // 任务完成
}

impl TrackContext {
    #[inline]
    pub const fn new(parameters: Parameters) -> Self {
        Self {
            parameters,
            index: (0, 0),
            state: State::Relocating,
        }
    }
}

impl<'a> Tracker<'a> {
    /// 循线
    pub fn track(&mut self, pose: Isometry2<f32>) -> Result<(f32, f32), Error> {
        loop {
            match self.context.state {
                State::Relocating => {
                    if let Some(index) = self.path.relocate(path::RelocateConfig {
                        pose,
                        index: self.context.index,
                        light_radius: self.context.parameters.light_radius,
                        search_range: self.context.parameters.search_range,
                        r#loop: self.context.parameters.r#loop,
                    }) {
                        self.context.index = index;
                        self.context.state = State::Initializing;
                    } else {
                        return Err(Error::RelocationFailed);
                    }
                }
                State::Initializing => {
                    match track::goto(
                        pose.inv_mul(&self.path.slice(self.context.index)[0]),
                        self.context.parameters.light_radius,
                    ) {
                        Some(next) => return Ok(next),
                        None => self.context.state = State::Tracking,
                    }
                }
                State::Tracking => {
                    match track::track(
                        self.path.slice(self.context.index),
                        pose,
                        self.context.parameters.light_radius,
                    ) {
                        Some((i, rudder)) => {
                            self.context.index.1 += i;
                            return Ok((1.0, rudder));
                        }
                        None => {
                            if self.path.slice(self.context.index).len() < 2 {
                                self.context.state =
                                    if self.context.index.0 == self.path.0.len() - 1 {
                                        if !self.context.parameters.r#loop {
                                            return Err(Error::Complete);
                                        }
                                        self.context.index.0 = 0;
                                        self.context.index.1 = 0;
                                        State::Relocating
                                    } else {
                                        self.context.index.0 += 1;
                                        self.context.index.1 = 0;
                                        State::Initializing
                                    };
                            } else {
                                if self.context.parameters.auto_reinitialize {
                                    self.context.state = State::Initializing;
                                } else {
                                    return Err(Error::OutOfPath);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

/// 扇形
#[derive(Clone, Copy)]
pub struct Sector {
    pub radius: f32,
    pub angle: f32,
}

/// 判断是否在扇形内
#[derive(Clone, Copy)]
pub struct InsideSectorChecker {
    pub squared: f32,
    pub half: f32,
}

impl Sector {
    #[inline]
    pub fn get_checker(&self) -> InsideSectorChecker {
        InsideSectorChecker {
            squared: self.radius.powi(2),
            half: self.angle * 0.5,
        }
    }
}

impl InsideSectorChecker {
    #[inline]
    pub fn contains(&self, p: Vector2<f32>) -> bool {
        p.norm_squared() < self.squared && p[1].atan2(p[0]).abs() < self.half
    }
}

#[inline]
const fn isometry(x: f32, y: f32, cos: f32, sin: f32) -> Isometry2<f32> {
    use nalgebra::{Complex, Translation, Unit};
    Isometry2 {
        translation: Translation {
            vector: vector(x, y),
        },
        rotation: Unit::new_unchecked(Complex { re: cos, im: sin }),
    }
}

#[inline]
const fn point(x: f32, y: f32) -> Point2<f32> {
    Point2 {
        coords: vector(x, y),
    }
}

#[inline]
const fn vector(x: f32, y: f32) -> Vector2<f32> {
    use nalgebra::{ArrayStorage, Vector};
    Vector::from_array_storage(ArrayStorage([[x, y]]))
}
