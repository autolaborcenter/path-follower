pub mod record;
pub mod track;

pub enum Task {
    Record(record::Task),
    Follow(track::Task),
}
