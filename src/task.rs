pub mod follow;
pub mod record;

use std::ops::Range;

pub enum Task {
    Record(record::Task),
    Follow(follow::Task),
}

fn normalize(mut angle: f32, range: Range<f32>) -> f32 {
    let unit = range.end - range.start;
    while angle < range.start {
        angle += unit;
    }
    while angle >= range.end {
        angle -= unit;
    }
    angle
}
