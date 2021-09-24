pub mod follow;
pub mod record;

pub enum Task {
    Record(record::Task),
    Follow(follow::Task),
}
