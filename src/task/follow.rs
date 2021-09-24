use nalgebra as na;
use std::fmt::{Display, Formatter};
use std::str::FromStr;
use Progress::{Index, Percent};

/// 任务进度（可读表示）
pub enum Progress {
    Index(usize),
    Percent(f64),
}

impl FromStr for Progress {
    type Err = ();

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        if s.ends_with('%') {
            match s[0..s.len() - 1].parse::<f64>() {
                Ok(f) => {
                    if f < 100.0 {
                        Ok(Percent(f / 100.0))
                    } else {
                        Err(())
                    }
                }
                Err(_) => Err(()),
            }
        } else {
            match s.parse::<usize>() {
                Ok(u) => Ok(Index(u)),
                Err(_) => match s.parse::<f64>() {
                    Ok(f) => {
                        if f < 1.0 {
                            Ok(Percent(f))
                        } else {
                            Err(())
                        }
                    }
                    Err(_) => Err(()),
                },
            }
        }
    }
}

/// 路径跟踪任务
pub struct Task {
    poses: Vec<na::Isometry2<f32>>,
    index: usize,
}

impl Task {
    pub fn new(poses: Vec<na::Isometry2<f32>>) -> Self {
        Self { poses, index: 0 }
    }

    pub fn jump(&mut self, len: usize) {
        self.index = (self.index + len) % self.poses.len();
    }

    pub fn jump_to(&mut self, index: usize) -> Result<(), &str> {
        if index < self.poses.len() {
            self.index = index;
            Ok(())
        } else {
            Err("out of index")
        }
    }

    pub fn search(
        &mut self,
        pose: na::Isometry2<f32>,
        first: fn(&na::Isometry2<f32>) -> bool,
    ) -> Vec<na::Isometry2<f32>> {
        let to_robot = pose.inverse();
        let mut it = self.poses[self.index..self.poses.len()]
            .iter()
            .cloned()
            .map(|p| to_robot * p);
        let mut local: Vec<na::Isometry2<f32>> = vec![];
        loop {
            // 查找局部起始点
            match it.next() {
                Some(p) => {
                    if first(&p) {
                        local = vec![p];
                        break;
                    } else {
                        self.index += 1;
                    }
                }
                None => break,
            }
        }
        local.append(it.take(40).collect::<Vec<_>>().as_mut());
        return local;
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
