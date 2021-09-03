pub mod manage;

use nalgebra as na;

/// 从字符串解析等距映射
pub fn parse_isometry2(s: &str) -> Option<na::Isometry2<f64>> {
    let mut i = 0;
    let mut numbers = [0.0; 3];
    for r in s.split(',').map(|s| s.trim().parse::<f64>()) {
        if i >= numbers.len() || r.is_err() { return None; }
        numbers[i] = r.unwrap();
        i += 1;
    }
    Some(na::Isometry2::new(na::Vector2::new(numbers[0], numbers[1]), numbers[2]))
}

#[test]
fn parse_test() {
    assert_eq!(parse_isometry2("-1,+2,-0"), Some(na::Isometry2::new(na::Vector2::new(-1.0, 2.0), 0.0)));
    assert_eq!(parse_isometry2("1,2,3,x"), None);
    assert_eq!(parse_isometry2(""), None);
}
