use std::ffi::OsString;
use std::io::Write;

use nalgebra as na;

/// 列出文件路径下的路径文件
pub fn list_files(dir: &str) -> FileVec {
    FileVec(std::fs::read_dir(dir)
        .unwrap()
        .filter_map(|r| r.ok())
        .filter(|d| d.path().is_file())
        .filter(|d|
            match d.file_name().to_str() {
                Some(s) => s.ends_with(".path"),
                None => false,
            }
        )
        .collect())
}

/// 从文件解析路径
pub fn parse_path(path: &str) -> std::io::Result<Vec<na::Isometry2<f64>>> {
    Ok(std::fs::read_to_string(path)?
        .lines()
        .filter_map(|l| super::parse_isometry2(l))
        .collect())
}

/// 文件列表
pub struct FileVec(Vec<std::fs::DirEntry>);

impl std::fmt::Display for FileVec {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        fn cutoff(name: &OsString) -> &str { &name.to_str().unwrap()[0..name.len() - 5] }

        if self.0.is_empty() {
            write!(f, "[]")?;
        } else {
            let mut it = self.0.iter().map(|r| r.file_name());
            write!(f, "[{}", cutoff(&it.next().unwrap()))?;
            for e in it {
                write!(f, ", {}", cutoff(&e))?;
            }
            write!(f, "]")?;
        }
        Ok(())
    }
}

/// 路径文件
pub struct PathFile {
    name: String,
    file: std::fs::File,
}

impl PathFile {
    pub fn new(dir: &str, name: &str) -> std::io::Result<PathFile> {
        Ok(PathFile {
            name: name.to_string(),
            file: std::fs::OpenOptions::new()
                .append(true)
                .create(true)
                .open(format!("{}/{}.path", dir, name))?,
        })
    }

    pub fn name_equals(pf: &Option<PathFile>, new: &str) -> bool {
        pf.as_ref().map_or(false, |f| f.name == new)
    }

    pub fn append(&mut self, pose: &na::Isometry2<f64>) {
        let _ = write!(self.file, "{},{},{}\n", pose.translation.x, pose.translation.y, pose.rotation.angle());
    }
}
