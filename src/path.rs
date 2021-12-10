use crate::{isometry, InsideSectorChecker, Isometry2, Sector};
use async_std::{
    fs::File,
    io::{prelude::BufReadExt, BufReader},
    task,
};

/// 打开的路径文件
pub struct PathFile(BufReader<File>);

/// 分段的路径
pub struct Path(pub Vec<Vec<Isometry2<f32>>>);

pub(crate) struct RelocateConfig {
    pub pose: Isometry2<f32>,
    pub index: (usize, usize),
    pub light_radius: f32,
    pub search_range: Sector,
    pub r#loop: bool,
}

impl PathFile {
    #[inline]
    pub async fn open(path: &async_std::path::Path) -> std::io::Result<Self> {
        Ok(Self(BufReader::new(File::open(path).await?)))
    }
}

impl Iterator for PathFile {
    type Item = Isometry2<f32>;

    fn next(&mut self) -> Option<Self::Item> {
        let mut line = String::new();
        task::block_on(self.0.read_line(&mut line)).ok()?;
        let mut numbers = line.trim().split(',');
        let x = numbers.next()?.parse().ok()?;
        let y = numbers.next()?.parse().ok()?;
        let (sin, cos) = numbers.next()?.parse::<f32>().ok()?.sin_cos();
        Some(isometry(x, y, cos, sin))
    }
}

macro_rules! with_index {
    ($it:expr) => {
        $it.iter().enumerate()
    };
}

macro_rules! update {
    ($local:expr, $spuared:expr, $index:expr, $result:expr) => {{
        let p = $local.translation.vector.norm_squared();
        let d = *$local.rotation.complex();
        if p < $spuared && d.re.is_sign_positive() {
            return Some($index);
        } else if $result.1.contains($local.translation.vector) {
            $result = (
                $index,
                InsideSectorChecker {
                    squared: p,
                    ..$result.1
                },
            );
        }
    }};
}

impl Path {
    /// 读取一条路径，检测其中的尖点
    pub fn new(
        path: impl IntoIterator<Item = Isometry2<f32>>,
        sector: Sector,
        tip_ignore: usize,
    ) -> Self {
        let mut source = path.into_iter();
        let mut result = vec![vec![]];
        if let Some(r) = source.next() {
            let checker = sector.get_checker();

            result.last_mut().unwrap().push(r);
            for p in source {
                // 使用上一个点逆变换，即将这一个点变换到理想的机器人坐标系
                let segment = result.last_mut().unwrap();
                if checker.contains(segment.last().unwrap().inv_mul(&p).translation.vector) {
                    segment.push(p);
                } else {
                    // 反向检查，看跳过一些点能否实现连续
                    let remain = if tip_ignore > 0 {
                        segment
                            .iter()
                            .rev()
                            .take(tip_ignore + 1)
                            .skip_while(|r| !checker.contains(r.inv_mul(&p).translation.vector))
                            .count()
                    } else {
                        0
                    };
                    if remain > 0 {
                        segment.truncate(segment.len() - (tip_ignore + 1) + remain);
                        segment.push(p);
                    } else {
                        result.push(vec![p]);
                    }
                }
            }
        }
        Self(result)
    }

    /// 根据序号切片路段
    #[inline]
    pub fn slice<'a>(&'a self, i: (usize, usize)) -> &'a [Isometry2<f32>] {
        &self.0[i.0][i.1..]
    }

    /// 根据当前位姿重定位
    ///
    /// 将遍历整个路径，代价极大且计算密集
    pub(crate) fn relocate(&self, config: RelocateConfig) -> Option<(usize, usize)> {
        // 光斑中心
        let c = config.pose * isometry(config.light_radius, 0.0, 1.0, 0.0);
        // 到光斑坐标系的变换
        let to_local = c.inverse();

        let best = config.light_radius.powi(2);
        let checker = config.search_range.get_checker();
        let mut context = (config.index, checker);

        let ref path = self.0;
        let ref local = path[0];

        // 向后遍历
        {
            for (j, p) in with_index!(local).skip(context.0 .1) {
                update!(to_local * p, best, (context.0 .0, j), context);
            }
            for (i, segment) in with_index!(path).skip(context.0 .0 + 1) {
                for (j, p) in with_index!(segment) {
                    update!(to_local * p, best, (i, j), context);
                }
            }
        }
        // 支持循环时从前遍历
        if config.r#loop {
            for (i, segment) in with_index!(path).take(context.0 .0) {
                for (j, p) in with_index!(segment) {
                    update!(to_local * p, best, (i, j), context);
                }
            }
            for (j, p) in with_index!(local).take(context.0 .1) {
                update!(to_local * p, best, (context.0 .0, j), context);
            }
        }

        return Some(context.0).filter(|_| context.1.squared < checker.squared);
    }
}
