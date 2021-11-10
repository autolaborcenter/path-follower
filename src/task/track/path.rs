use nalgebra::{Complex, Isometry2, Point2, Vector2};
use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI, SQRT_2};

pub(super) struct Path {
    path: Vec<Vec<Isometry2<f32>>>,
    index: (usize, usize),
}

pub(super) enum InitializeResult {
    Complete,
    Drive(f32, f32),
}

pub(super) enum TrackError {
    OutOfPath,
    Termination,
}

macro_rules! with_index {
    ($it:expr) => {
        $it.iter().enumerate()
    };
}

macro_rules! update {
    ($self:expr, $local:expr, $best:expr, $index:expr, $min:expr) => {
        let p = $local.translation.vector.norm_squared();
        let d = $local.rotation.angle().abs();

        if p < $best && d < FRAC_PI_2 {
            $self.index = $index;
            return true;
        } else if p < $min {
            $self.index = $index;
            $min = p;
        }
    };
}

impl Path {
    pub fn new(path: Vec<Vec<Isometry2<f32>>>) -> Self {
        Self {
            path,
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
        // 光斑中心
        let c = pose * Isometry2::new(Vector2::new(light_radius, 0.0), 0.0);
        // 到光斑坐标系的变换
        let to_local = c.inverse();

        let best = light_radius.powi(2);
        let available = search_radius.powi(2);
        let mut min = available;

        let ref path = self.path;
        let ref local = path[self.index.0];

        // 向后遍历
        {
            for (j, p) in with_index!(local).skip(self.index.1) {
                update!(self, to_local * p, best, (self.index.0, j), min);
            }
            for (i, segment) in with_index!(path).skip(self.index.0 + 1) {
                for (j, p) in with_index!(segment) {
                    update!(self, to_local * p, best, (i, j), min);
                }
            }
        }
        // 支持循环时从前遍历
        if r#loop {
            for (i, segment) in with_index!(path).take(self.index.0) {
                for (j, p) in with_index!(segment) {
                    update!(self, to_local * p, best, (i, j), min);
                }
            }
            for (j, p) in with_index!(local).take(self.index.1) {
                update!(self, to_local * p, best, (self.index.0, j), min);
            }
        }

        return min < available;
    }

    /// 在当前路段搜索并产生控制量
    pub fn track_within(
        &mut self,
        pose: &Isometry2<f32>,
        light_radius: f32,
    ) -> Result<f32, TrackError> {
        let c = (pose * Point2::from(Vector2::new(light_radius, 0.0))).coords;
        let squared = light_radius.powi(2);

        // 遍历当前路段
        let first = self.path[self.index.0]
            .iter()
            .enumerate()
            .skip(self.index.1)
            .find(|(_, p)| (c - p.translation.vector).norm_squared() < squared);

        // 生成控制量或异常
        if let Some((j, _)) = first {
            self.index.1 = j;
            Ok(self.size_proportion(Isometry2::new(c, pose.rotation.angle()), squared))
        } else if self.path[self.index.0].len() - self.index.1 < 2 {
            Err(TrackError::Termination)
        } else {
            Err(TrackError::OutOfPath)
        }
    }

    /// 尝试加载下一个路段
    pub fn next_segment(&mut self, r#loop: bool) -> bool {
        if self.index.0 == self.path.len() - 1 {
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
    pub fn initialize(&self, pose: &Isometry2<f32>, light_radius: f32) -> InitializeResult {
        const FRAC_PI_16: f32 = PI / 16.0;

        // 光斑中心相对机器人的位姿
        let c_light = Isometry2::new(Vector2::new(light_radius, 0.0), 0.0);
        // 机器人坐标系上机器人应该到达的目标位置
        let target = pose.inverse() * (self.path[self.index.0][self.index.1] * c_light.inverse());

        let p = target.translation.vector;
        let d = target.rotation.angle();

        // 退出临界角
        // 目标方向小于此角度时考虑退出
        let theta = FRAC_PI_16; // assert θ < π/4

        // 原地转安全半径
        // 目标距离小于此半径且目标方向小于临界角时可退出
        let squared = {
            let rho = SQRT_2 * light_radius;
            let theta = 3.0 * FRAC_PI_4 + theta; // 3π/4 + θ
            let (sin, cos) = theta.sin_cos();
            let vec = Vector2::new(light_radius + rho * cos, rho * sin);
            vec.norm_squared() * 0.95 // 略微收缩确保可靠性
        };

        return if p.norm_squared() < squared {
            if d.abs() < theta {
                // 位置方向条件都满足，退出
                InitializeResult::Complete
            } else {
                // 方向条件不满足，原地转
                InitializeResult::Drive(1.0, d.signum() * -FRAC_PI_2)
            }
        } else {
            let dir = -p[1].atan2(p[0].abs());
            // 位置条件不满足，逼近
            InitializeResult::Drive(
                f32::min(1.0, p.norm() * 0.2),
                dir.clamp(-FRAC_PI_2, FRAC_PI_2),
            )
        };
    }

    /// 计算面积比并转化到 [-π/2, π/2]
    fn size_proportion(&self, c: Isometry2<f32>, squared: f32) -> f32 {
        let segment = self.path[self.index.0].as_slice();
        let to_local = c.inverse();

        // 查找路段起点、终点
        let begin = to_local * segment[self.index.1];
        let end = segment[self.index.1 + 1..]
            .iter()
            .map(|p| {
                let local = to_local * p;
                if local.translation.vector.norm_squared() < squared {
                    Some(local)
                } else {
                    None
                }
            })
            .take_while(|p| p.is_some())
            .last()
            .flatten()
            .unwrap_or(begin);

        let begin = intersection(&begin, squared, -1.0);
        let end = intersection(&end, squared, 1.0);
        let diff = angle_of(end) - angle_of(begin); // [-2π, 2π]
        (diff.signum() * PI - diff) / 2.0 // [-π/2, π/2]
    }
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
    Vector2::new(re, im)
}

/// 求方向角
#[inline]
fn angle_of(p: Vector2<f32>) -> f32 {
    p.y.atan2(p.x)
}
