//! Minimal private copies of the MVP shape structs, until work package GB lands
//! `rapier_geometry2d::shape`.
//!
//! Field names and semantics are those of the GB brief (and of Parry's
//! `shape::{Ball, Cuboid, Capsule, Segment, HalfSpace}`); when `shape` is merged this module is
//! deleted and `super` re-exports the real types instead. Only the two helpers the point queries
//! need are defined here (`segment_scaled_direction`, `segment_normal`); everything else about
//! the shapes belongs to GB.

use fixed::{Fixed, ZERO};
use glam::vec2::Vec2;
use rapier_math::consts::DEFAULT_EPSILON;
use rapier_math::math_ext::vec2::try_normalize2_eps;

/// A disk of radius `radius` centred on the local origin.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Ball {
    pub radius: Fixed,
}

/// A box centred on the local origin, `half_extents` along each axis.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Cuboid {
    pub half_extents: Vec2,
}

/// The segment `a`–`b` dilated by `radius`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Capsule {
    pub segment: Segment,
    pub radius: Fixed,
}

/// A segment from `a` to `b`; a zero-length segment is legal.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Segment {
    pub a: Vec2,
    pub b: Vec2,
}

/// The half-space `{ p : normal . p <= 0 }`; `normal` is the outward unit normal and the boundary
/// passes through the local origin.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct HalfSpace {
    pub normal: Vec2,
}

/// Returns `b - a`, the direction of the segment scaled by its length.
///
/// Mirrors `parry::shape::Segment::scaled_direction`.
/// #### Panics
/// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if a component of the difference leaves the
///   scalar range.
/// #### Deviations
/// * None.
#[inline(always)]
pub fn segment_scaled_direction(seg: Segment) -> Vec2 {
    Vec2 { x: seg.b.x - seg.a.x, y: seg.b.y - seg.a.y }
}

/// Returns the normalised counter-clockwise normal `(dir.y, -dir.x) / |dir|`, or `None` when the
/// segment is shorter than `rapier_math::consts::DEFAULT_EPSILON`.
///
/// Mirrors `parry::shape::Segment::normal` (2D), whose guard is `length > DEFAULT_EPSILON` on the
/// **scaled** normal, i.e. on the length of the segment itself. The comparison is the wide
/// squared one of [`try_normalize2_eps`]: the squared length is never rescaled.
/// #### Panics
/// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if `b - a` leaves the scalar range.
/// * `'i64_neg Underflow'` if `b.x - a.x` is `fixed::MIN`.
/// #### Deviations
/// * None.
#[inline(always)]
pub fn segment_normal(seg: Segment) -> Option<Vec2> {
    let dir = segment_scaled_direction(seg);
    match try_normalize2_eps(dir.y, ZERO - dir.x, DEFAULT_EPSILON) {
        Some((x, y)) => Some(Vec2 { x, y }),
        None => None,
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, ONE, ZERO};
    use glam::vec2::Vec2;
    use rapier_testing::opaque;
    use super::{Segment, segment_normal, segment_scaled_direction};

    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: Fixed { raw: x }, y: Fixed { raw: y } }
    }

    #[test]
    fn test_scaled_direction_and_normal() {
        let unit: i64 = 0x100000000;
        // (a, b, expected normal or None)
        let cases: Span<(Vec2, Vec2, Option<Vec2>)> = array![
            (v(0, 0), v(unit, 0), Some(v(0, -unit))), (v(0, 0), v(0, unit), Some(v(unit, 0))),
            (
                v(unit, unit), v(unit, unit), None,
            ), // 512 raw is exactly DEFAULT_EPSILON: the guard is strict, so it is still degenerate.
            (v(0, 0), v(512, 0), None), (v(0, 0), v(513, 0), Some(v(0, -unit))),
        ]
            .span();
        for (a, b, expected) in cases {
            assert_eq!(segment_normal(Segment { a: *a, b: *b }), *expected);
        }
        assert_eq!(
            segment_scaled_direction(Segment { a: v(unit, 0), b: v(0, unit) }), v(-unit, unit),
        );
    }

    #[test]
    fn test_normal_is_unit_for_an_oblique_segment() {
        let n = segment_normal(Segment { a: v(0, 0), b: v(3 * 0x100000000, 4 * 0x100000000) })
            .unwrap();
        // (dir.y, -dir.x) / 5 = (0.8, -0.6).
        assert!(n.x.abs_diff_eq(FixedTrait::from_ratio(4, 5), Fixed { raw: 2 }));
        assert!(n.y.abs_diff_eq(ZERO - FixedTrait::from_ratio(3, 5), Fixed { raw: 2 }));
        assert!(ONE.abs_diff_eq(n.x * n.x + n.y * n.y, Fixed { raw: 8 }));
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_segment_scaled_direction() {
        let _ = segment_scaled_direction(opaque(Segment { a: v(1, 2), b: v(0x100000000, 5) }));
    }

    #[test]
    fn gas_segment_normal() {
        let _ = segment_normal(opaque(Segment { a: v(0, 0), b: v(3 * 0x100000000, 0x100000000) }));
    }
}
