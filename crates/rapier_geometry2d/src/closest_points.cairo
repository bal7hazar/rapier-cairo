//! Closest points between two segments (Parry
//! `query/closest_points/closest_points_segment_segment.rs`, Ericson's routine).
//!
//! Both segments are given in the **same frame**; upstream takes a `pos12` and transforms the
//! second one first, which the caller does here (`Pose2::transform_point` on `a` and `b`).
//!
//! # The algorithm, and what fixed point changes
//!
//! With `d1 = b1 - a1`, `d2 = b2 - a2`, `r = a1 - a2`, upstream solves for the parameters
//! `(s, t)` of the closest pair:
//!
//! ```text
//! a = |d1|^2   e = |d2|^2   b = d1 . d2   c = d1 . r   f = d2 . r
//! s = clamp((b f - c e) / (a e - b b))   t = (b s + f) / e   (then the two t corrections)
//! ```
//!
//! Three things do not survive a naive port:
//!
//! 1. **The degeneracy tests.** Upstream writes `a <= eps` with `eps = DEFAULT_EPSILON`, i.e. it
//!    compares a *squared* length with an *unsquared* tolerance. The squared length itself is the
//!    hazard: `|d1|^2` is exactly 0 as a `Fixed` for any segment shorter than `2^-16`. Both tests
//!    are therefore done on the raw Q64.64 sum of squares of
//!    `rapier_math::math_ext::norm2::norm2_sq_wide`, against `DEFAULT_EPSILON` lifted to the same
//!    scale ([`DEGENERATE_SQ_RAW`]) — exact over the whole scalar range.
//! 2. **The determinant.** `denom = a e - b b` is a fourth-degree quantity; rescaling it to
//!    `Fixed` would send everything below `2^-32` to 0 and call perfectly transverse segments
//!    collinear. It is kept as the exact `i128` difference of the two raw products. Its
//!    *threshold* cannot be ported at all: upstream compares a fourth-degree quantity with a
//!    first-degree tolerance (`denom > eps`), which only ever works because `f64::EPSILON` is
//!    `2.2e-16`. The same absolute test in Q32.32 would call two perpendicular segments of
//!    length `0.004` collinear. The port therefore uses the **relative** form the absolute one
//!    stands for, `denom / (a e) > 2^-23`, i.e. `sin^2(angle) > 2^-23`: two segments are
//!    collinear when the angle between them is below `2^-11.5` rad (0.02 deg). The
//!    `seg/nearly_parallel` fixture, whose slope is `2^-10`, is eight times above that and goes
//!    through the regular branch, as `tools/golden/README.md` requires. Upstream's second guard,
//!    `!ulps_eq!(ae, bb)`, exists only because `ae - bb` is catastrophic cancellation in
//!    floating point; the difference here is exact, so it has nothing left to catch and is
//!    dropped.
//! 3. **The divisions.** All four are clamped ratios of two `i128` quantities of the same scale,
//!    and go through `super::point::ratio::clamped_ratio`: neither operand is rescaled to make
//!    the division fit, and the last bit is rounded to nearest. `a`, `e`,
//!    `b`, `c` and `f` themselves are `Fixed`, exactly as upstream holds them, which is what
//!    makes [`DEGENERATE_SQ_RAW`] both upstream's threshold and the smallest safe one.
//!
//! Ties (parallel or collinear segments) are broken exactly as upstream does: `s = 0`, then `t`
//! is clamped to `[0, 1]`, then the two `t` corrections may move `s`. The answer is one member
//! of an infinite family; it is deterministic and it matches the `ambiguous` fixtures of
//! `rapier_golden::segment_segment` in distance, not necessarily in location.

use fixed::wide::{dot2, norm2_squared};
use fixed::{Fixed, ONE, ZERO};
use glam::vec2::Vec2;
use rapier_math::math_ext::norm2::norm2_sq_wide;
use crate::point::SegmentPointLocation;
use crate::point::ratio::clamped_ratio;
use crate::point::segment::segment_point_at;
use crate::shape::{Segment, SegmentTrait};

/// `rapier_math::consts::DEFAULT_EPSILON` (`2^-23`) lifted to the raw Q64.64 scale of
/// `norm2_sq_wide`: `512 * 2^32 = 2^41`.
///
/// A segment whose squared length is at or below this is a point — upstream's `a <= _eps`, with
/// the same decimal meaning. It is also the smallest threshold at which the narrowed `a = |d|^2`
/// still carries information: a segment right at the limit has `a = 512` raw, and one ten times
/// shorter would have `a = 0`.
///
/// The threshold is on `|d|^2`, so the shortest segment that is not a point is `2^-11.5`, about
/// `3.4e-4` units. The `f64` build draws that line at `1.5e-8` instead; nothing in Q32.32 can.
pub const DEGENERATE_SQ_RAW: i128 = 2199023255552;

/// `2^23`: `denom > (a e) / 2^23` is the collinearity test, i.e. `sin^2(angle) > 2^-23`.
const COLLINEAR_REL: i128 = 0x80_0000;
/// `2^32`, the factor that lifts a `Fixed` raw to the Q64.64 scale of the other operand.
const SCALE: i128 = 0x1_0000_0000;

/// Returns `(s, t)`, the parameters of the closest pair of points of `seg1` and `seg2`.
fn closest_parameters(seg1: Segment, seg2: Segment) -> (Fixed, Fixed) {
    let d1 = seg1.scaled_direction();
    let d2 = seg2.scaled_direction();
    let r = Vec2 { x: seg1.a.x - seg2.a.x, y: seg1.a.y - seg2.a.y };
    let sq1 = norm2_sq_wide(d1.x, d1.y);
    let sq2 = norm2_sq_wide(d2.x, d2.y);
    let degenerate1 = sq1 <= DEGENERATE_SQ_RAW;
    let degenerate2 = sq2 <= DEGENERATE_SQ_RAW;
    if degenerate1 && degenerate2 {
        // Two points: the only pair there is.
        return (ZERO, ZERO);
    }
    if degenerate1 {
        // `seg1` is a point: project it on `seg2`.
        let f = dot2(d2.x, r.x, d2.y, r.y);
        return (ZERO, clamped_ratio(f.raw.into(), norm2_squared(d2.x, d2.y).raw.into()));
    }
    let c = dot2(d1.x, r.x, d1.y, r.y);
    let a: i128 = norm2_squared(d1.x, d1.y).raw.into();
    if degenerate2 {
        // `seg2` is a point: project it on `seg1`.
        return (clamped_ratio(-c.raw.into(), a), ZERO);
    }
    let b = dot2(d1.x, d2.x, d1.y, d2.y);
    let e: i128 = norm2_squared(d2.x, d2.y).raw.into();
    let f = dot2(d2.x, r.x, d2.y, r.y);
    // `denom = a e - b b`, exact at the raw Q64.64 scale, against `(a e) * 2^-23`.
    let ae = a * e;
    let denom = ae - b.raw.into() * b.raw.into();
    let s = if denom > collinear_limit(ae) {
        clamped_ratio(b.raw.into() * f.raw.into() - c.raw.into() * e, denom)
    } else {
        ZERO
    };
    // `t = (b s + f) / e`, numerator and denominator both at the raw Q64.64 scale.
    let t_num = b.raw.into() * s.raw.into() + f.raw.into() * SCALE;
    let t_den = e * SCALE;
    if t_num < 0 {
        (clamped_ratio(-c.raw.into(), a), ZERO)
    } else if t_num > t_den {
        (clamped_ratio(b.raw.into() - c.raw.into(), a), ONE)
    } else {
        (s, clamped_ratio(t_num, t_den))
    }
}

/// Returns `(a e) / 2^23`, the largest determinant that still counts as collinear.
///
/// `a` and `e` are non-negative `Fixed` raws, so the product is non-negative and the division
/// truncates downwards.
#[inline(always)]
fn collinear_limit(ae: i128) -> i128 {
    ae / COLLINEAR_REL
}

/// Turns a clamped parameter into the location upstream reports.
#[inline(always)]
fn location_of(parameter: Fixed) -> SegmentPointLocation {
    if parameter == ZERO {
        SegmentPointLocation::OnVertex(0)
    } else if parameter == ONE {
        SegmentPointLocation::OnVertex(1)
    } else {
        SegmentPointLocation::OnEdge((ONE - parameter, parameter))
    }
}

/// Returns where the closest points of `seg1` and `seg2` lie on each segment.
///
/// Mirrors `closest_points_segment_segment_with_locations`, with both segments already in the
/// same frame.
/// #### Panics
/// * `'Fixed: overflow'` if a squared length (`|d| >= 2^15.5`, i.e. 46 341 units) or one of the
///   dot products `d1 . d2`, `d1 . r`, `d2 . r` does not fit the scalar range.
/// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if `b - a` or `a1 - a2` leaves the scalar
///   range.
/// #### Deviations
/// * The degeneracy and collinearity tests are wide (see the module documentation), and
///   upstream's `!ulps_eq!(ae, bb)` guard is dropped because the difference is computed exactly.
/// * Each parameter is the correctly rounded exact ratio, within half an ulp of upstream's.
/// * A parameter that rounds to exactly 0 or 1 is reported as `OnVertex`, as upstream does with
///   its own `s == 0.0` test; a pair within an ulp of an end point may therefore be reported as
///   `OnVertex` here and `OnEdge` upstream, or the other way round.
pub fn closest_points_segment_segment_with_locations(
    seg1: Segment, seg2: Segment,
) -> (SegmentPointLocation, SegmentPointLocation) {
    let (s, t) = closest_parameters(seg1, seg2);
    (location_of(s), location_of(t))
}

/// Returns the closest pair of points of `seg1` and `seg2`, the first on `seg1`.
///
/// Mirrors `closest_points_segment_segment` without its `margin` test: upstream returns
/// `ClosestPoints::Disjoint` when the pair is further apart than the margin, which every caller
/// of this package re-tests itself against its own prediction distance.
/// #### Panics
/// * See [`closest_points_segment_segment_with_locations`].
/// #### Deviations
/// * See [`closest_points_segment_segment_with_locations`]; the points themselves are the fused
///   `a * u + b * v` of `crate::point::segment_point_at`.
#[inline(always)]
pub fn closest_points_segment_segment(seg1: Segment, seg2: Segment) -> (Vec2, Vec2) {
    let (loc1, loc2) = closest_points_segment_segment_with_locations(seg1, seg2);
    (segment_point_at(seg1, loc1), segment_point_at(seg2, loc2))
}

/// Rejected candidates, kept for the `gas_*` ranking.
#[cfg(test)]
pub mod alternatives {
    use fixed::wide::{dot2, mul_add, mul_sub, norm2_squared};
    use fixed::{Fixed, FixedTrait, ONE, ZERO};
    use glam::vec2::Vec2;
    use rapier_math::consts::DEFAULT_EPSILON;
    use crate::shape::{Segment, SegmentTrait};

    /// The literal port: `a`, `e` and `denom` as `Fixed`, `Fixed / Fixed` and `Fixed::clamp`
    /// instead of the wide ratios. **Wrong** for the inputs this package must handle — `|d|^2`
    /// rescales to 0 below `2^-16` (so `denom` does too, and every transverse pair of short
    /// segments is declared collinear), low bits are lost before division, and `a e` overflows
    /// above `2^15.5` just as the shipped version does.
    pub fn closest_parameters_narrow(seg1: Segment, seg2: Segment) -> (Fixed, Fixed) {
        let d1 = seg1.scaled_direction();
        let d2 = seg2.scaled_direction();
        let r = Vec2 { x: seg1.a.x - seg2.a.x, y: seg1.a.y - seg2.a.y };
        let a = norm2_squared(d1.x, d1.y);
        let e = norm2_squared(d2.x, d2.y);
        let f = dot2(d2.x, r.x, d2.y, r.y);
        if a <= DEFAULT_EPSILON && e <= DEFAULT_EPSILON {
            return (ZERO, ZERO);
        }
        if a <= DEFAULT_EPSILON {
            return (ZERO, (f / e).clamp(ZERO, ONE));
        }
        let c = dot2(d1.x, r.x, d1.y, r.y);
        if e <= DEFAULT_EPSILON {
            return ((-c / a).clamp(ZERO, ONE), ZERO);
        }
        let b = dot2(d1.x, d2.x, d1.y, d2.y);
        let denom = mul_sub(a, e, b, b);
        let s = if denom > DEFAULT_EPSILON {
            (mul_sub(b, f, c, e) / denom).clamp(ZERO, ONE)
        } else {
            ZERO
        };
        let t = mul_add(b, s, f) / e;
        if t < ZERO {
            ((-c / a).clamp(ZERO, ONE), ZERO)
        } else if t > ONE {
            (((b - c) / a).clamp(ZERO, ONE), ONE)
        } else {
            (s, t)
        }
    }

    /// `|p1 - p2|^2` of a candidate pair, as an `i128` at the raw Q64.64 scale: the quantity the
    /// routine actually minimises, used to compare two tie members.
    pub fn pair_dist_sq(p1: Vec2, p2: Vec2) -> i128 {
        let dx: i128 = (p1.x - p2.x).raw.into();
        let dy: i128 = (p1.y - p2.y).raw.into();
        dx * dx + dy * dy
    }

    /// The point of `seg` at parameter `u`, without going through a `SegmentPointLocation`.
    pub fn point_at_parameter(seg: Segment, u: Fixed) -> Vec2 {
        let _: Fixed = FixedTrait::from_raw(0);
        Vec2 { x: dot2(seg.a.x, ONE - u, seg.b.x, u), y: dot2(seg.a.y, ONE - u, seg.b.y, u) }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, ONE, ZERO};
    use glam::vec2::Vec2;
    use rapier_testing::opaque;
    use crate::point::SegmentPointLocation;
    use crate::shape::Segment;
    use super::alternatives::{closest_parameters_narrow, pair_dist_sq, point_at_parameter};
    use super::{
        closest_parameters, closest_points_segment_segment,
        closest_points_segment_segment_with_locations,
    };

    const UNIT: i64 = 0x1_0000_0000;
    const HALF: i64 = 0x8000_0000;

    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: Fixed { raw: x }, y: Fixed { raw: y } }
    }

    fn seg(ax: i64, ay: i64, bx: i64, by: i64) -> Segment {
        Segment { a: v(ax, ay), b: v(bx, by) }
    }

    fn edge(u: i64) -> SegmentPointLocation {
        SegmentPointLocation::OnEdge((Fixed { raw: UNIT - u }, Fixed { raw: u }))
    }

    #[test]
    fn test_locations_table() {
        let horizontal = seg(-UNIT, 0, UNIT, 0);
        // (seg1, seg2, expected loc1, expected loc2)
        let cases: Span<(Segment, Segment, SegmentPointLocation, SegmentPointLocation)> = array![
            // Crossing at the middle of both.
            (horizontal, seg(0, -UNIT, 0, UNIT), edge(HALF), edge(HALF)),
            // Transverse, closest at an end point of the second.
            (horizontal, seg(0, UNIT, 0, 2 * UNIT), edge(HALF), SegmentPointLocation::OnVertex(0)),
            // End point against end point.
            (
                horizontal,
                seg(2 * UNIT, 0, 4 * UNIT, 0),
                SegmentPointLocation::OnVertex(1),
                SegmentPointLocation::OnVertex(0),
            ),
            // Collinear and overlapping: `s = 0`, then `t = -0.5 < 0` pulls `t` back to the
            // start of `seg2` and moves `s` to `-c / a = 0.5` — upstream's exact tie-break.
            (horizontal, seg(0, 0, 2 * UNIT, 0), edge(HALF), SegmentPointLocation::OnVertex(0)),
            // The first segment is a point.
            (seg(0, UNIT, 0, UNIT), horizontal, SegmentPointLocation::OnVertex(0), edge(HALF)),
            // The second segment is a point.
            (horizontal, seg(0, UNIT, 0, UNIT), edge(HALF), SegmentPointLocation::OnVertex(0)),
            // Both are points.
            (
                seg(0, 0, 0, 0),
                seg(UNIT, UNIT, UNIT, UNIT),
                SegmentPointLocation::OnVertex(0),
                SegmentPointLocation::OnVertex(0),
            ),
        ]
            .span();
        for (s1, s2, l1, l2) in cases {
            let (got1, got2) = closest_points_segment_segment_with_locations(*s1, *s2);
            assert_eq!(got1, *l1);
            assert_eq!(got2, *l2);
        }
    }

    #[test]
    fn test_points_of_a_crossing_pair() {
        let (p1, p2) = closest_points_segment_segment(
            seg(-UNIT, 0, UNIT, 0), seg(0, -UNIT, 0, UNIT),
        );
        assert_eq!(p1, v(0, 0));
        assert_eq!(p2, v(0, 0));
        // Oblique: s = 0.625, t = 0.5 (the `seg/crossing_oblique` fixture).
        let (p1, p2) = closest_points_segment_segment(
            seg(-UNIT, 0, UNIT, 0), seg(-HALF, -UNIT, UNIT, UNIT),
        );
        assert!(p1.x.abs_diff_eq(Fixed { raw: HALF / 2 }, Fixed { raw: 4 }));
        assert!(p1.y.abs_diff_eq(ZERO, Fixed { raw: 4 }));
        assert!(p2.x.abs_diff_eq(Fixed { raw: HALF / 2 }, Fixed { raw: 4 }));
        assert!(p2.y.abs_diff_eq(ZERO, Fixed { raw: 4 }));
    }

    /// Two transverse segments of `2^-8` units, whose `a e` is `2^-32` — the regime where
    /// upstream's absolute `denom > eps` would answer "collinear" and the relative test does
    /// not. The narrowing candidate falls into the collinear branch here.
    #[test]
    fn test_short_transverse_segments() {
        let short = 0x80_0000;
        let s1 = seg(-short, 0, short, 0);
        let s2 = seg(0, -short, 0, short);
        let (l1, l2) = closest_points_segment_segment_with_locations(s1, s2);
        assert_eq!(l1, edge(HALF));
        assert_eq!(l2, edge(HALF));
        let (p1, p2) = closest_points_segment_segment(s1, s2);
        assert_eq!(pair_dist_sq(p1, p2), 0);
        // The rejected candidate answers the collinear fallback instead.
        let (s, _) = closest_parameters_narrow(s1, s2);
        assert_eq!(s, ZERO);
    }

    /// The pair is symmetric: swapping the two segments swaps the two answers.
    #[test]
    fn test_symmetry() {
        let pairs: Span<(Segment, Segment)> = array![
            (seg(-UNIT, 0, UNIT, 0), seg(-HALF, -UNIT, UNIT, UNIT)),
            (seg(0, 0, UNIT, UNIT), seg(2 * UNIT, 0, 3 * UNIT, UNIT)),
            (seg(0, 0, 0, 0), seg(-UNIT, UNIT, UNIT, UNIT)),
        ]
            .span();
        for (s1, s2) in pairs {
            let (p1, p2) = closest_points_segment_segment(*s1, *s2);
            let (q2, q1) = closest_points_segment_segment(*s2, *s1);
            assert!(p1.x.abs_diff_eq(q1.x, Fixed { raw: 4 }));
            assert!(p1.y.abs_diff_eq(q1.y, Fixed { raw: 4 }));
            assert!(p2.x.abs_diff_eq(q2.x, Fixed { raw: 4 }));
            assert!(p2.y.abs_diff_eq(q2.y, Fixed { raw: 4 }));
        }
    }

    #[test]
    #[should_panic(expected: 'Fixed: overflow')]
    fn test_overlong_segment_panics() {
        let big = 0x4000_0000_0000_0000;
        let _ = closest_points_segment_segment_with_locations(
            Segment { a: v(0, 0), b: v(big, 0) }, seg(0, UNIT, UNIT, UNIT),
        );
    }

    /// The pair the routine returns is at least as close as either end-point pair: it really is a
    /// minimiser, not just a fixed point of the algebra.
    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_answer_is_not_worse_than_the_end_points(ax: i16, ay: i16, bx: i16, by: i16) {
        let s1 = seg(-UNIT, 0, UNIT, 0);
        let s2 = Segment {
            a: v(ax.into() * 65536, ay.into() * 65536), b: v(bx.into() * 65536, by.into() * 65536),
        };
        let (p1, p2) = closest_points_segment_segment(s1, s2);
        let best = pair_dist_sq(p1, p2);
        let slack: i128 = 0x1_0000_0000_0000;
        for (q1, q2) in array![(s1.a, s2.a), (s1.a, s2.b), (s1.b, s2.a), (s1.b, s2.b)].span() {
            assert!(best <= pair_dist_sq(*q1, *q2) + slack);
        }
    }

    /// Both formulations agree on the well-conditioned pairs the rejected one can handle.
    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_candidates_agree(ax: i8, ay: i8, bx: i8, by: i8) {
        let s1 = seg(-UNIT, 0, UNIT, 0);
        let s2 = Segment {
            a: v(ax.into() * UNIT, ay.into() * UNIT),
            b: v(bx.into() * UNIT + UNIT, by.into() * UNIT + UNIT),
        };
        let (s, t) = closest_parameters(s1, s2);
        let (sn, tn) = closest_parameters_narrow(s1, s2);
        // The wide ratios keep the full operands; the narrow ones pre-round them: at most two ulp
        // apart unless the determinant was small enough for the narrow one to fall back to `s = 0`.
        if sn != ZERO || s == ZERO {
            assert!(s.abs_diff_eq(sn, Fixed { raw: 2 }));
            assert!(t.abs_diff_eq(tn, Fixed { raw: 2 }));
        }
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_closest_points_crossing() {
        let _ = closest_points_segment_segment(
            opaque(seg(-UNIT, 0, UNIT, 0)), opaque(seg(-HALF, -UNIT, UNIT, UNIT)),
        );
    }

    #[test]
    fn gas_closest_points_with_locations_crossing() {
        let _ = closest_points_segment_segment_with_locations(
            opaque(seg(-UNIT, 0, UNIT, 0)), opaque(seg(-HALF, -UNIT, UNIT, UNIT)),
        );
    }

    #[test]
    fn gas_closest_points_parallel() {
        let _ = closest_points_segment_segment_with_locations(
            opaque(seg(-UNIT, 0, UNIT, 0)), opaque(seg(-UNIT, UNIT, UNIT, UNIT)),
        );
    }

    #[test]
    fn gas_closest_points_degenerate() {
        let _ = closest_points_segment_segment_with_locations(
            opaque(seg(0, 0, 0, 0)), opaque(seg(-UNIT, UNIT, UNIT, UNIT)),
        );
    }

    #[test]
    fn gas_closest_parameters_narrow_crossing() {
        let _ = closest_parameters_narrow(
            opaque(seg(-UNIT, 0, UNIT, 0)), opaque(seg(-HALF, -UNIT, UNIT, UNIT)),
        );
    }

    #[test]
    fn gas_point_at_parameter() {
        let _ = point_at_parameter(opaque(seg(-UNIT, 0, UNIT, 0)), opaque(Fixed { raw: HALF }));
    }

    #[test]
    fn gas_location_of_edge() {
        let _ = super::location_of(opaque(Fixed { raw: HALF }));
    }

    #[test]
    fn gas_one_minus() {
        let _ = ONE - opaque(Fixed { raw: HALF });
    }
}
