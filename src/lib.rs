pub mod controller;

use nalgebra::{Isometry2, Vector2};
use rtk_ins570_rs::{ins570, rtk_threads, RTKThreads};
use std::{
    sync::mpsc::*,
    thread,
    time::{Duration, Instant},
};

pub fn launch_rtk() -> Receiver<(Instant, Isometry2<f32>)> {
    let (sender, receiver) = channel::<(Instant, Isometry2<f32>)>();

    thread::spawn(move || loop {
        let sender = sender.clone();
        rtk_threads!(move |_, rtk| {
            for s in rtk {
                let now = Instant::now();
                match s {
                    ins570::Solution::Uninitialized(_) => {}
                    ins570::Solution::Data(data) => {
                        let ins570::SolutionData { state, enu, dir } = data;
                        let ins570::SolutionState {
                            state_pos,
                            satellites: _,
                            state_dir,
                        } = state;
                        if state_pos >= 40 && state_dir >= 40 {
                            let _ = sender.send((
                                now,
                                Isometry2::new(
                                    Vector2::new(enu.e as f32, enu.n as f32),
                                    dir as f32,
                                ),
                            ));
                        }
                    }
                }
            }
        })
        .join();
        std::thread::sleep(Duration::from_secs(1));
    });

    receiver
}
