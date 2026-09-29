//! Splitting a shape by a plane (Parry `query/split/split.rs`, `split_aabb.rs`,
//! `split_segment.rs`; PX3).
//!
//! # Fixed point
//!
//! * The split of a segment reads its decisions off exact quantities: the denominator
//!   `axis . (b - a)` and the numerator `bias - axis . a` are wide (raw Q64.64) values, so
//!   `relative_eq!(b, 0.0)` is the exact test `axis . (b - a) == 0`, and `bcoord * |dir|` is
//!   compared with `epsilon` on the wide product of the two raws, without overflow.
//! * `bcoord` is one correctly rounded [`div_wide`]; a quotient outside the scalar range means
//!   the plane is far beyond either end, which upstream's tests classify the same way.
//! * `bias` is the value of `axis . p` on the plane, like upstream; it is lifted to the wide
//!   scale (`bias * 2^32`) to be subtracted from the wide dot product.

use fixed::Fixed;
use fixed::wide::norm2;
use glam_core::{Vec2, Vec2Trait};
use crate::point::dot_wide;
use crate::ray::quotient::div_wide;
use crate::shape::segment::{Segment, SegmentTrait};

/// `2^32`: lifts a `Fixed` raw to the raw Q64.64 scale of a wide dot product.
const SCALE: i128 = 0x1_0000_0000;

/// Errors of the split queries.
pub mod errors {
    /// A canonical axis other than `0` (x) or `1` (y).
    pub const AXIS: felt252 = 'Split: axis out of range';
}

/// The result of splitting a shape by a plane (Parry `SplitResult`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum SplitResult<T> {
    /// The shape is split in two: the part on the negative side first, then the positive one.
    Pair: (T, T),
    /// The whole shape lies on the negative side of the plane.
    Negative,
    /// The whole shape lies on the positive side of the plane.
    Positive,
}

/// The result of intersecting a shape with a plane (Parry `IntersectResult`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum IntersectResult<T> {
    /// The intersection of the shape with the plane.
    Intersect: T,
    /// The whole shape lies on the negative side of the plane.
    Negative,
    /// The whole shape lies on the positive side of the plane.
    Positive,
}

/// Split methods of [`Segment`] (Parry `query/split/split_segment.rs`).
#[generate_trait]
pub impl SegmentSplitImpl of SegmentSplitTrait {
    /// Splits the segment by the plane `p[axis] = bias` (upstream `canonical_split`), `axis`
    /// being `0` (x) or `1` (y).
    /// #### Panics
    /// * `'Split: axis out of range'` for an axis above `1`; see [`Self::local_split`].
    fn canonical_split(
        self: Segment, axis: u32, bias: Fixed, epsilon: Fixed,
    ) -> SplitResult<Segment> {
        assert(axis < 2, errors::AXIS);
        let local_axis = if axis == 0 {
            Vec2Trait::X
        } else {
            Vec2Trait::Y
        };
        self.local_split(local_axis, bias, epsilon)
    }

    /// Splits the segment by the plane `local_axis . p = bias` (upstream `local_split`).
    /// #### Panics
    /// * See [`Self::local_split_and_get_intersection`].
    fn local_split(
        self: Segment, local_axis: Vec2, bias: Fixed, epsilon: Fixed,
    ) -> SplitResult<Segment> {
        let (result, _) = self.local_split_and_get_intersection(local_axis, bias, epsilon);
        result
    }

    /// Splits the segment by the plane `local_axis . p = bias` and returns the intersection
    /// point with its parameter along the segment (upstream `local_split_and_get_intersection`).
    ///
    /// The segment is unchanged (`Negative` / `Positive`, no intersection) when it is parallel
    /// to the plane or when the intersection is within `epsilon` of an end point (in length);
    /// `Negative` when `bias - axis . a >= 0`. Otherwise the pair lists the negative side first.
    /// #### Panics
    /// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if `b - a` leaves the scalar range;
    ///   `'Fixed: overflow'` if the intersection point does.
    /// #### Deviations
    /// * The decisions are exact (see the module documentation).
    fn local_split_and_get_intersection(
        self: Segment, local_axis: Vec2, bias: Fixed, epsilon: Fixed,
    ) -> (SplitResult<Segment>, Option<(Vec2, Fixed)>) {
        let dir = self.b - self.a;
        let a: i128 = bias.raw.into() * SCALE
            - dot_wide(local_axis.x, local_axis.y, self.a.x, self.a.y);
        let b: i128 = dot_wide(local_axis.x, local_axis.y, dir.x, dir.y);
        let dir_norm = norm2(dir.x, dir.y);
        let eps_wide: i128 = epsilon.raw.into() * SCALE;
        let full_wide: i128 = (dir_norm.raw - epsilon.raw).into() * SCALE;
        let no_split = if b == 0 {
            true
        } else {
            match div_wide(a, b) {
                // Beyond the scalar range: the plane is far from the segment on either side.
                None => true,
                Some(bcoord) => {
                    let t: i128 = bcoord.raw.into() * dir_norm.raw.into();
                    t <= eps_wide || t >= full_wide
                },
            }
        };
        if no_split {
            return if a >= 0 {
                (SplitResult::Negative, None)
            } else {
                (SplitResult::Positive, None)
            };
        }
        let bcoord = div_wide(a, b).unwrap();
        let intersection = self.a + dir.mul_scalar(bcoord);
        let s1 = SegmentTrait::new(self.a, intersection);
        let s2 = SegmentTrait::new(intersection, self.b);
        if a >= 0 {
            (SplitResult::Pair((s1, s2)), Some((intersection, bcoord)))
        } else {
            (SplitResult::Pair((s2, s1)), Some((intersection, bcoord)))
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, ONE, ZERO};
    use glam_core::Vec2;
    use rapier_testing::opaque;
    use crate::shape::segment::{Segment, SegmentTrait};
    use super::{SegmentSplitTrait, SplitResult};

    fn f(n: i64) -> Fixed {
        Fixed { raw: n * 0x1_0000_0000 }
    }

    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: f(x), y: f(y) }
    }

    fn seg(ax: i64, ay: i64, bx: i64, by: i64) -> Segment {
        SegmentTrait::new(v(ax, ay), v(bx, by))
    }

    #[test]
    fn test_split_pair_and_intersection() {
        let s = seg(0, 0, 4, 2);
        let x = unit_x();
        let (result, hit) = s.local_split_and_get_intersection(x, ONE, ZERO);
        // Intersection at `x = 1`, a quarter of the way: the start side (negative) comes first.
        let p = Vec2 { x: ONE, y: Fixed { raw: 0x8000_0000 } };
        assert_eq!(
            result,
            SplitResult::Pair((SegmentTrait::new(v(0, 0), p), SegmentTrait::new(p, v(4, 2)))),
        );
        assert_eq!(hit, Some((p, Fixed { raw: 0x4000_0000 })));
        // The reversed segment lists the negative side (`x < 1`) first as well.
        let (result, _) = seg(4, 2, 0, 0).local_split_and_get_intersection(x, ONE, ZERO);
        assert_eq!(
            result,
            SplitResult::Pair((SegmentTrait::new(p, v(0, 0)), SegmentTrait::new(v(4, 2), p))),
        );
    }

    fn unit_x() -> Vec2 {
        v(1, 0)
    }

    #[test]
    fn test_split_without_intersection() {
        let s = seg(0, 0, 4, 2);
        let x = v(1, 0);
        let none: Option<(Vec2, Fixed)> = None;
        assert_eq!(
            s.local_split_and_get_intersection(x, f(5), ZERO), (SplitResult::Negative, none),
        );
        assert_eq!(
            s.local_split_and_get_intersection(x, f(-1), ZERO), (SplitResult::Positive, none),
        );
        // Parallel to the plane: classified by the side of its start.
        assert_eq!(seg(0, 1, 4, 1).local_split(v(0, 1), f(2), ZERO), SplitResult::Negative);
        assert_eq!(seg(0, 1, 4, 1).local_split(v(0, 1), ONE / f(2), ZERO), SplitResult::Positive);
        // An intersection within epsilon of an end point is no split: classified by the side of
        // the start (`bias - axis . a >= 0` is negative), near either end.
        let quarter = ONE / f(4);
        assert_eq!(seg(0, 0, 4, 0).local_split(x, quarter / f(2), quarter), SplitResult::Negative);
        assert_eq!(
            seg(0, 0, 4, 0).local_split(x, f(4) - quarter / f(2), quarter), SplitResult::Negative,
        );
        // A zero-length segment on the plane.
        assert_eq!(seg(1, 1, 1, 1).local_split(x, ONE, ZERO), SplitResult::Negative);
    }

    #[test]
    fn test_canonical_split_axes() {
        let s = seg(0, 0, 4, 4);
        assert_eq!(s.canonical_split(0, f(2), ZERO), s.local_split(v(1, 0), f(2), ZERO));
        assert_eq!(s.canonical_split(1, f(2), ZERO), s.local_split(v(0, 1), f(2), ZERO));
    }

    #[test]
    #[should_panic(expected: ('Split: axis out of range',))]
    fn test_canonical_split_axis_out_of_range() {
        let _ = seg(0, 0, 1, 1).canonical_split(2, ZERO, ZERO);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_canonical_split() {
        let _ = opaque(seg(0, 0, 4, 2)).canonical_split(opaque(0), opaque(ONE), opaque(ZERO));
    }

    #[test]
    fn gas_local_split() {
        let _ = opaque(seg(0, 0, 4, 2)).local_split(opaque(v(1, 0)), opaque(ONE), opaque(ZERO));
    }

    #[test]
    fn gas_local_split_and_get_intersection() {
        let _ = opaque(seg(0, 0, 4, 2))
            .local_split_and_get_intersection(opaque(v(1, 0)), opaque(ONE), opaque(ZERO));
    }

    #[test]
    fn gas_local_split_parallel() {
        let _ = opaque(seg(0, 1, 4, 1)).local_split(opaque(v(0, 1)), opaque(f(2)), opaque(ZERO));
    }
}
