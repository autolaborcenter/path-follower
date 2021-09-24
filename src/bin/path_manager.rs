fn main() {}

// #![feature(map_try_insert)]

// use std::collections::HashMap;

// use path_follower::{
//     control::{PathFile, PathFileToOpen, PathRepository},
//     follow::FollowTask,
// };

// const HELP: &str = "commands: `<file> << <pose>`, `delete <file>`, `list`, `follow <file> [progress][%]`, `search <pose>`";

// fn main() {
//     // 打开一个路径仓库
//     let dir = match std::env::args().skip(1).next() {
//         Some(dir) => dir,
//         None => {
//             eprintln!("Please pass the path directory by arg.");
//             return;
//         }
//     };
//     let repo = match PathRepository::new(dir.as_str()) {
//         Some(r) => r,
//         None => {
//             eprintln!("{} is not a valid path.", dir);
//             return;
//         }
//     };
//     println!("repo dir: {:?}", repo.absolute());
//     println!("{}", HELP);

//     let mut list: HashMap<String, PathFileToOpen> = HashMap::new();
//     let mut file: Option<PathFile> = None;
//     let mut follow_task: Option<FollowTask> = None;
//     loop {
//         let mut line = String::new();
//         if std::io::stdin().read_line(&mut line).is_err() {
//             break;
//         }
//         let words: Vec<_> = line.split_whitespace().collect();
//         if words.is_empty() {
//             continue;
//         }
//         match words[0] {
//             // 列出现有文件
//             "list" => {
//                 if words.len() == 1 {
//                     list = repo.list();
//                     if let Some(ref file) = file {
//                         list.remove(&file.name().to_string());
//                     }
//                     println!(
//                         "current opened file: {:?}",
//                         file.as_ref().and_then(|f| Some(f.name().to_string()))
//                     );
//                     for name in list.keys() {
//                         println!("{}", name)
//                     }
//                 }
//             }
//             // 删除路径文件
//             "delete" => {
//                 if words.len() == 2 {
//                     let name = words[1];
//                     match list.remove(name.into()) {
//                         Some(file) => {
//                             let _ = file.delete();
//                             println!("Path file \"{}\" is deleted.", name);
//                         }
//                         None => eprintln!("Path file \"{}\" not exist.", name),
//                     }
//                 }
//             }
//             // 打开一个路径文件
//             "open" => {
//                 if words.len() == 2 {
//                     let name = words[1];
//                     // 判断是否重复打开当前文件
//                     if let Some(ref f) = file {
//                         if f.name() == name {
//                             println!("Path file \"{}\" is open.", name);
//                             continue;
//                         }
//                     }
//                     // 判断能否从列表中打开文件
//                     match list.remove(name.into()) {
//                         Some(to_open) => match to_open.open() {
//                             Ok(new) => {
//                                 match file {
//                                     Some(old) => {
//                                         let f = old.close();
//                                         let _ = list.try_insert(f.name().to_string(), f);
//                                     }
//                                     None => {}
//                                 }
//                                 file = Some(new);
//                                 println!("Path file \"{}\" is open.", name);
//                             }
//                             Err(e) => eprintln!("failed to open path file: {}", e),
//                         },
//                         None => eprintln!("Path file \"{}\" not exist.", name),
//                     }
//                 }
//             }
//             // 清空当前路径
//             "clear" => match file {
//                 Some(ref mut f) => f.clear(),
//                 None => {}
//             },
//             // 启动跟踪
//             // "follow" => {
//             //     if words.len() == 2 {
//             //         follow_task = Some(FollowTask::new(path, progress));
//             //         println!(
//             //             "follow \"{}.path\": {}",
//             //             words[1],
//             //             follow_task.as_ref().unwrap()
//             //         )
//             //     }
//             // }
//             // 搜索当前路径
//             // "search" => {
//             //     if words.len() == 2 && follow_task.is_some() {
//             //         match parse_isometry2(words[1]) {
//             //             Some(pose) => {
//             //                 let local = follow_task.as_mut().unwrap().search(pose, |delta| {
//             //                     const LIMIT: f64 = std::f64::consts::FRAC_PI_3;
//             //                     let len = delta.translation.vector.norm();
//             //                     let v = delta.translation;
//             //                     let angle1 = v.y.atan2(v.x);
//             //                     let angle2 = delta.rotation.angle();
//             //                     len < 0.5
//             //                         && -LIMIT < angle1
//             //                         && angle1 < LIMIT
//             //                         && -LIMIT < angle2
//             //                         && angle2 < LIMIT
//             //                 });
//             //                 print!("L ");
//             //                 for p in local {
//             //                     print!(
//             //                         "{},{},{} ",
//             //                         p.translation.x,
//             //                         p.translation.y,
//             //                         p.rotation.angle()
//             //                     );
//             //                 }
//             //                 println!();
//             //             }
//             //             None => {}
//             //         }
//             //     }
//             // }
//             _ => println!("{}", HELP),
//         }
//     }
// }
