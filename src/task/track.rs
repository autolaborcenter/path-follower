use nalgebra::Isometry2;

mod path;

use path::Path;

/// 路径跟踪任务
pub struct Task {
    parameters: Parameters,
    path: Path,
    state: State,
}

/// 跟踪失败信息
#[derive(Debug)]
pub enum Error {
    RelocationFailed, // 搜索失败
    OutOfPath,        // 丢失路径
    Complete,         // 任务完成
}

/// 循径参数
pub struct Parameters {
    search_radius: f32,
    light_radius: f32,
    r#loop: bool,
    auto_reinitialize: bool,
}

enum State {
    Relocating,   // 重新搜索
    Initializing, // 初始化
    Tracking,     // 连续循线
}

impl Parameters {
    pub const DEFAULT: Self = Self {
        search_radius: 5.0,
        light_radius: 1.0,
        r#loop: false,
        auto_reinitialize: true,
    };
}

impl Task {
    pub fn new(path: Vec<Isometry2<f32>>, parameters: Parameters) -> Self {
        Self {
            parameters,
            path: Path::new(vec![path]),
            state: State::Relocating,
        }
    }

    /// 下一次更新时进行重定位
    pub fn relocate(&mut self) {
        self.state = State::Relocating;
    }

    /// 循线
    pub fn track<'a>(&'a mut self, pose: &Isometry2<f32>) -> Result<(f32, f32), Error> {
        loop {
            match self.state {
                State::Relocating => {
                    if self.path.relocate(
                        pose,
                        self.parameters.light_radius,
                        self.parameters.search_radius,
                        self.parameters.r#loop,
                    ) {
                        self.state = State::Initializing;
                    } else {
                        return Err(Error::RelocationFailed);
                    }
                }
                State::Initializing => {
                    use path::InitializeResult::*;
                    match self.path.initialize(pose, self.parameters.light_radius) {
                        Complete => self.state = State::Tracking,
                        Drive(dir, value) => return Ok((dir, value)),
                    }
                }
                State::Tracking => {
                    use path::TrackError::*;
                    match self.path.track_within(pose, self.parameters.light_radius) {
                        Ok(value) => return Ok((1.0, value)),
                        Err(OutOfPath) => {
                            if self.parameters.auto_reinitialize {
                                self.state = State::Initializing;
                            } else {
                                return Err(Error::OutOfPath);
                            }
                        }
                        Err(Termination) => {
                            if self.path.next_segment(self.parameters.r#loop) {
                                self.state = State::Initializing;
                            } else {
                                return Err(Error::Complete);
                            }
                        }
                    }
                }
            }
        }
    }
}
