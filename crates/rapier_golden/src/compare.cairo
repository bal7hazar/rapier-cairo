//! Tolerance-based comparison of raw Q32.32 values.
//!
//! Golden vectors come from an `f64` engine: a fixed-point port is expected to land within a few
//! raw units (ulps) of them, never exactly on them. Recommended tolerances per vector family are
//! documented in `tools/golden/README.md`.

use crate::types::Vec2Raw;

/// Absolute difference `|a - b|` of two raw values, without overflow for any pair of `i64`.
pub fn abs_diff(a: i64, b: i64) -> u64 {
    let (hi, lo) = if a < b {
        (b, a)
    } else {
        (a, b)
    };
    // `hi - lo` lies in `[0, 2^64)`: both conversions below always succeed.
    let wide: i128 = hi.into() - lo.into();
    let wide: u128 = wide.try_into().unwrap();
    wide.try_into().unwrap()
}

/// `true` when `actual` is within `tolerance` raw units of `expected` (bounds included).
pub fn within(actual: i64, expected: i64, tolerance: u64) -> bool {
    abs_diff(actual, expected) <= tolerance
}

/// Component-wise [`within`] for vectors.
pub fn vec2_within(actual: Vec2Raw, expected: Vec2Raw, tolerance: u64) -> bool {
    within(actual.x, expected.x, tolerance) && within(actual.y, expected.y, tolerance)
}

#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use crate::types::Vec2Raw;
    use super::{abs_diff, vec2_within, within};

    const MAX: i64 = 0x7fffffffffffffff;
    const MIN: i64 = -0x8000000000000000;

    #[test]
    fn test_abs_diff_basic() {
        assert_eq!(abs_diff(5, 3), 2);
        assert_eq!(abs_diff(3, 5), 2);
        assert_eq!(abs_diff(-3, 5), 8);
        assert_eq!(abs_diff(5, -3), 8);
        assert_eq!(abs_diff(-7, -7), 0);
        assert_eq!(abs_diff(0, 0), 0);
    }

    #[test]
    fn test_abs_diff_extremes() {
        assert_eq!(abs_diff(MAX, MIN), 0xffffffffffffffff);
        assert_eq!(abs_diff(MIN, MAX), 0xffffffffffffffff);
        assert_eq!(abs_diff(MIN, 0), 0x8000000000000000);
        assert_eq!(abs_diff(MAX, MAX), 0);
    }

    #[test]
    fn test_within_bounds_are_inclusive() {
        assert!(within(10, 12, 2));
        assert!(within(12, 10, 2));
        assert!(!within(10, 13, 2));
        assert!(within(-1, 1, 2));
        assert!(within(4, 4, 0));
        assert!(!within(4, 5, 0));
    }

    #[test]
    fn test_vec2_within_checks_both_components() {
        let expected = Vec2Raw { x: 100, y: -100 };
        assert!(vec2_within(Vec2Raw { x: 101, y: -99 }, expected, 1));
        assert!(!vec2_within(Vec2Raw { x: 102, y: -100 }, expected, 1));
        assert!(!vec2_within(Vec2Raw { x: 100, y: -98 }, expected, 1));
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_abs_diff() {
        assert_eq!(abs_diff(opaque(-3), opaque(5)), 8);
    }

    #[test]
    fn gas_within() {
        assert!(within(opaque(10), opaque(12), opaque(2)));
    }

    #[test]
    fn gas_vec2_within() {
        let expected = Vec2Raw { x: 100, y: -100 };
        assert!(vec2_within(opaque(Vec2Raw { x: 101, y: -99 }), opaque(expected), opaque(1)));
    }
}
