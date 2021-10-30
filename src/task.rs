pub mod record;
pub mod track;

use std::ops::Range;

pub enum Task {
    Record(record::Task),
    Follow(track::Task),
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
