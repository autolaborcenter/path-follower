use nalgebra::{Isometry2, Vector2};

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
    search_radius: f32,      //
    light_radius: f32,       //
    r#loop: bool,            //
    auto_reinitialize: bool, //
    tip_ignore: usize,       // 是否严格循尖点
}

enum State {
    Relocating,   // 重新搜索
    Initializing, // 初始化
    Tracking,     // 连续循线
}

impl Parameters {
    pub const DEFAULT: Self = Self {
        search_radius: 5.0,
        light_radius: 0.4,
        r#loop: false,
        auto_reinitialize: true,
        tip_ignore: 10,
    };
}

impl Task {
    /// 读取一条路径，检测其中的尖点
    pub fn new<I>(path: I, parameters: Parameters) -> Self
    where
        I: IntoIterator<Item = Isometry2<f32>>,
    {
        // 光斑中心位置
        let c = Vector2::new(parameters.light_radius, 0.0);
        // 光斑范围
        let squared = parameters.light_radius.powi(2);

        let mut source = path.into_iter();
        let mut path = vec![vec![]];
        if let Some(mut reference) = source.next() {
            // reference 理解为上一个点的位姿，即处理这一个点时机器人预期的位姿
            path.last_mut().unwrap().push(reference);
            for p in source {
                // 使用 reference 逆变换，即将这一个点变换到理想的机器人坐标系
                let segment = path.last_mut().unwrap();
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
                        .take(parameters.tip_ignore + 1)
                        .skip_while(|r| !is_continious(&r.inv_mul(&p), c, squared))
                        .count();
                    if remain > 0 {
                        segment.truncate(segment.len() - (parameters.tip_ignore + 1) + remain);
                        segment.push(p);
                    } else {
                        path.push(vec![p]);
                    }
                }
                // 更新参考点
                reference = p;
            }
        }
        Self {
            parameters,
            path: Path::new(path),
            state: State::Relocating,
        }
    }

    /// 用于观察分段后的路径
    #[allow(dead_code)]
    #[inline]
    pub fn view<'a>(&'a self) -> &'a Vec<Vec<Isometry2<f32>>> {
        &self.path.inner
    }

    /// 下一次更新时进行重定位
    #[allow(dead_code)]
    #[inline]
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

#[inline]
fn is_continious(local: &Isometry2<f32>, c: Vector2<f32>, squared: f32) -> bool {
    use std::f32::consts::FRAC_PI_3;
    (local.translation.vector - c).norm_squared() < squared
        && local.rotation.angle().abs() < FRAC_PI_3
}
