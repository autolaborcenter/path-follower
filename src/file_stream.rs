use nalgebra::Isometry2;
use std::{
    fs::File,
    io::{BufRead, BufReader},
};

pub(super) struct FileStream(BufReader<File>);

pub(super) struct IntoFileStream(pub File);

impl IntoIterator for IntoFileStream {
    type Item = Isometry2<f32>;
    type IntoIter = FileStream;

    fn into_iter(self) -> Self::IntoIter {
        FileStream(BufReader::new(self.0))
    }
}

impl Iterator for FileStream {
    type Item = Isometry2<f32>;

    fn next(&mut self) -> Option<Self::Item> {
        let mut line = String::new();
        self.0
            .read_line(&mut line)
            .ok()
            .and_then(|_| super::parse_isometry2(line.as_str()))
    }
}
