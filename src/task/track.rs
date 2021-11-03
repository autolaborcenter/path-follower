use nalgebra::Isometry2;

mod path;

use path::{LocalSearchError, Path};

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

impl Task {
    pub fn new(path: Vec<Isometry2<f32>>) -> Self {
        Self {
            parameters: Parameters {
                search_radius: 5.0,
                light_radius: 1.0,
                r#loop: false,
            },
            path: Path::new(vec![path]),
            state: State::Tracking,
        }
    }

    /// 循线
    pub fn track<'a>(&'a mut self, pose: &Isometry2<f32>) -> Result<f32, Error> {
        loop {
            match self.state {
                State::Relocation => {
                    match self.path.relocate(
                        pose,
                        self.parameters.search_radius,
                        self.parameters.r#loop,
                    ) {
                        Ok(()) => {
                            // TODO
                        }
                        Err(()) => return Err(Error::RelocationFailed),
                    }
                }
                State::Rotation0 => {
                    // TODO
                }
                State::GettingClose => {
                    // TODO
                }
                State::Rotation1 => {
                    // TODO
                }
                State::Tracking => {
                    match self.path.search_local(pose, self.parameters.light_radius) {
                        Ok(value) => return Ok(value),
                        Err(LocalSearchError::OutOfPath) => return Err(Error::OutOfPath),
                        Err(LocalSearchError::Termination) => {
                            if self.path.next_segment(self.parameters.r#loop) {
                                // TODO
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
