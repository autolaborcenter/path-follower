// use nalgebra as na;
// use std::{
//     collections::HashMap,
//     fs::File,
//     io::{BufReader, Read, Seek, SeekFrom, Write},
//     path::PathBuf,
// };

// /// 存放路径文件的目录
// pub struct PathRepository(PathBuf);

// /// 一个未打开的路径文件
// pub struct PathFileToOpen(PathBuf);

// /// 路径文件
// pub struct PathFile(File, PathBuf);

// impl PathRepository {
//     /// 创建路径仓库对象
//     pub fn new(path: &str) -> Option<Self> {
//         let path = PathBuf::from(path);
//         if path.is_dir() {
//             Some(Self(path))
//         } else {
//             None
//         }
//     }

//     /// 列出文件路径下所有路径文件
//     pub fn list(&self) -> HashMap<String, PathFileToOpen> {
//         match std::fs::read_dir(&self.0) {
//             Ok(dir) => dir
//                 .filter_map(|r| r.ok())
//                 .filter_map(|d| {
//                     let path = d.path();
//                     if path.is_file() && path.ends_with(".path") {
//                         Some((nameof(&path).to_string(), PathFileToOpen(path)))
//                     } else {
//                         None
//                     }
//                 })
//                 .collect(),
//             Err(_) => HashMap::new(),
//         }
//     }

//     /// 获取绝对路径
//     pub fn absolute(&self) -> PathBuf {
//         std::fs::canonicalize(&self.0).unwrap()
//     }
// }

// impl PathFileToOpen {
//     /// 文件名
//     pub fn name<'a>(&'a self) -> &'a str {
//         nameof(&self.0)
//     }

//     /// 打开路径文件
//     pub fn open(self) -> std::io::Result<PathFile> {
//         Ok(PathFile(
//             std::fs::OpenOptions::new()
//                 .append(true)
//                 .create(true)
//                 .open(&self.0)?,
//             self.0,
//         ))
//     }

//     /// 删除路径文件
//     pub fn delete(self) -> std::io::Result<()> {
//         std::fs::remove_file(self.0)
//     }
// }

// impl PathFile {
//     /// 文件名
//     pub fn name<'a>(&'a self) -> &'a str {
//         nameof(&self.1)
//     }

//     /// 清除文件内容
//     pub fn clear(&mut self) {
//         let _ = self.0.set_len(0);
//         let _ = self.0.seek(SeekFrom::Start(0));
//     }

//     /// 将一个点追加到路径文件
//     pub fn append(&mut self, pose: &na::Isometry2<f64>) {
//         let _ = write!(
//             self.0,
//             "{},{},{}\n",
//             pose.translation.x,
//             pose.translation.y,
//             pose.rotation.angle()
//         );
//     }

//     /// 读取路径中所有点并启动一个循径任务
//     pub fn build_follow_task(&mut self) -> std::io::Result<FollowTask> {
//         let mut buf_reader = BufReader::new(&self.0);
//         let mut contents = String::new();
//         buf_reader.read_to_string(&mut contents)?;
//         Ok(FollowTask::new(
//             contents.lines().filter_map(parse_isometry2).collect(),
//             0,
//         ))
//     }

//     /// 关闭路径文件
//     pub fn close(self) -> PathFileToOpen {
//         PathFileToOpen(self.1)
//     }
// }

// /// 截取路径名
// fn nameof<'a>(path: &'a PathBuf) -> &'a str {
//     let name = path.file_name().unwrap();
//     &name.to_str().unwrap()[..name.len() - 5]
// }

// /// 从字符串解析等距映射
// fn parse_isometry2(s: &str) -> Option<na::Isometry2<f32>> {
//     let mut i = 0;
//     let mut numbers = [0.0; 3];
//     for r in s.split(',').map(|s| s.trim().parse::<f32>()) {
//         if i >= numbers.len() || r.is_err() {
//             return None;
//         }
//         numbers[i] = r.unwrap();
//         i += 1;
//     }
//     Some(na::Isometry2::new(
//         na::Vector2::new(numbers[0], numbers[1]),
//         numbers[2],
//     ))
// }

// #[test]
// fn test_parse() {
//     assert_eq!(
//         parse_isometry2("-1,+2,-0"),
//         Some(na::Isometry2::new(na::Vector2::new(-1.0, 2.0), 0.0))
//     );
//     assert_eq!(parse_isometry2("1,2,3,x"), None);
//     assert_eq!(parse_isometry2(""), None);
// }
