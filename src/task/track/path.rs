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
    ($segment:expr) => {
        $segment.iter().enumerate()
    };
}

macro_rules! find_in {
    ($iter:expr, $c:expr; $squared:expr) => {
        $iter.find(|(_, p)| ($c - p.translation.vector).norm_squared() < $squared)
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
        search_radius: f32,
        r#loop: bool,
    ) -> Result<(), ()> {
        let c = pose.translation.vector;
        let squared = search_radius.powi(2);

        // 顺序遍历
        let path = self.path.as_slice();
        {
            let segment = path[self.index.0].as_slice();
            if let Some((j, _)) = find_in!(with_index!(segment).skip(self.index.1), c; squared) {
                self.index.1 = j;
                return Ok(());
            }
        }
        for (i, segment) in with_index!(path).skip(self.index.0 + 1) {
            if let Some((j, _)) = find_in!(with_index!(segment.as_slice()), c; squared) {
                self.index.0 = i;
                self.index.1 = j;
                return Ok(());
            }
        }

        // 禁止循环时失败
        if !r#loop {
            return Err(());
        }

        for (i, segment) in with_index!(path).take(self.index.0) {
            if let Some((j, _)) = find_in!(with_index!(segment.as_slice()), c; squared) {
                self.index.0 = i;
                self.index.1 = j;
                return Ok(());
            }
        }
        {
            let segment = path[self.index.0].as_slice();
            if let Some((j, _)) = find_in!(with_index!(segment).take(self.index.1), c; squared) {
                self.index.1 = j;
                return Ok(());
            }
        }

        // 重定位失败
        Err(())
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
        let segment = self.path[self.index.0].as_slice();
        if let Some((j, _)) = find_in!(with_index!(segment).skip(self.index.1), c; squared) {
            self.index.1 = j;
            Ok(self.size_proportion(Isometry2::new(c, pose.rotation.angle()), squared))
        } else if segment.len() - self.index.1 < 2 {
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
            vec.norm_squared()
        };

        return if p.norm_squared() < squared {
            if d.abs() < theta {
                // 位置方向条件都满足，退出
                InitializeResult::Complete
            } else {
                // 方向条件不满足，原地转
                InitializeResult::Drive(
                    // 移动方向
                    p[0].signum(),
                    // 转向方向
                    -p[0].signum() * d.signum() * FRAC_PI_2,
                )
            }
        } else {
            // 位置条件不满足，逼近
            InitializeResult::Drive(
                // 速度
                f32::min(1.0, p.norm() / 2.0) * p[0].signum(),
                // 方向
                -p[1].atan2(p[0].abs()),
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
