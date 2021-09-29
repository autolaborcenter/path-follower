use nalgebra::{Isometry2, Vector2};
use pose_filter::{InterpolationAndPredictionFilter, PoseFilter, PoseType};
use rtk_ins570_rs::{ins570, rtk_threads, RTKThreads};
use std::{
    sync::mpsc::*,
    thread,
    time::{Duration, Instant},
};

pub struct Locator {
    filter: InterpolationAndPredictionFilter,
    receiver: Receiver<(PoseType, Instant, Isometry2<f32>)>,
}

impl Locator {
    pub fn new() -> Self {
        let (sender, receiver) = channel::<(PoseType, Instant, Isometry2<f32>)>();
        launch_rtk(sender.clone());
        launch_odometry(sender);
        Self {
            filter: InterpolationAndPredictionFilter::new(),
            receiver,
        }
    }
}

impl Iterator for Locator {
    type Item = Isometry2<f32>;

    fn next(&mut self) -> Option<Self::Item> {
        self.receiver
            .recv()
            .ok()
            .and_then(|(t, time, pose)| Some(self.filter.update(t, time, pose)))
    }
}

fn launch_rtk(sender: Sender<(PoseType, Instant, Isometry2<f32>)>) {
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
                                PoseType::Absolute,
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
}

fn launch_odometry(sender: Sender<(PoseType, Instant, Isometry2<f32>)>) {}
