pub mod record;
pub mod track;

use std::ops::Range;

pub enum Task {
    Record(record::Task),
    Follow(track::Task),
}
