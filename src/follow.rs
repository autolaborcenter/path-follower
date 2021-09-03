use std::fmt::{Display, Formatter};
use std::str::FromStr;

use nalgebra as na;
use nalgebra::Isometry2;

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
                Ok(f) => if f < 100.0 { Ok(Percent(f / 100.0)) } else { Err(()) }
                Err(_) => Err(())
            }
        } else {
            match s.parse::<usize>() {
                Ok(u) => Ok(Index(u)),
                Err(_) => match s.parse::<f64>() {
                    Ok(f) => if f < 1.0 { Ok(Percent(f)) } else { Err(()) }
                    Err(_) => Err(())
                }
            }
        }
    }
}

/// 从文件解析路径
pub fn parse_path(dir: &str, name: &str) -> std::io::Result<Vec<na::Isometry2<f64>>> {
    Ok(std::fs::read_to_string(format!("{}/{}.path", dir, name).as_str())?
        .lines()
        .filter_map(|l| super::parse_isometry2(l))
        .collect())
}

/// 路径跟踪任务
pub struct FollowTask {
    pub poses: Vec<na::Isometry2<f64>>,
    pub index: usize,
}

impl FollowTask {
    pub fn new(poses: Vec<na::Isometry2<f64>>, progress: Progress) -> FollowTask {
        let index = match progress {
            Index(u) => std::cmp::min(u, (poses.len() - 1) as usize),
            Percent(f) => (poses.len() as f64 * f) as usize
        };
        FollowTask { poses, index }
    }

    pub fn search(
        &mut self,
        pose: Isometry2<f64>,
        first: fn(&Isometry2<f64>) -> bool,
    ) -> Vec<Isometry2<f64>> {
        let to_robot = pose.inverse();
        let mut it =
            self.poses[self.index..self.poses.len()]
                .iter()
                .cloned()
                .map(|p| to_robot * p);
        let mut local: Vec<Isometry2<f64>> = vec![];
        loop { // 查找局部起始点
            match it.next() {
                Some(p) =>
                    if first(&p) {
                        local = vec![p];
                        break;
                    } else {
                        self.index += 1;
                    }
                None => break,
            }
        }
        local.append(it.take(40).collect::<Vec<Isometry2<f64>>>().as_mut());
        return local;
    }
}

impl Display for FollowTask {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}%({}/{})", (self.index * 100 / (self.poses.len() - 1)), self.index, self.poses.len() - 1)
    }
}
