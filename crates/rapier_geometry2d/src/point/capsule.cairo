//! Point queries on a [`Capsule`] (Parry `query/point/point_capsule.rs`).
//!
//! A capsule is its core segment dilated by `radius`: project on the segment, then push the
//! projection out by `radius` along the direction of the query point. The interesting case is the
//! one where that direction does not exist — a point exactly *on* the core segment — which
//! upstream guards with `dist >= f64::EPSILON`. That threshold is `2.2e-16`, three orders of
//! magnitude below one Q32.32 ulp (`2.3e-10`), so in this port it reads exactly as
//! `dist != 0` and the fallback is the segment normal, or `+y` for a segment that has none.

use fixed::{Fixed, ONE, ZERO};
use glam::vec2::Vec2;
use rapier_math::math_ext::norm2::is_norm2_le;
use rapier_math::math_ext::vec2::try_normalize2;
use crate::feature_id::{FeatureId, FeatureIdTrait};
use crate::shape::{Capsule, SegmentTrait};
use super::PointProjection;
use super::segment::project_local_point_segment;

/// Projects `pt` on `capsule`.
///
/// Mirrors `PointQuery::project_local_point` for `Capsule`. A point at distance exactly `radius`
/// from the core segment counts as inside. With `solid = true` an inside point projects to
/// itself; with `solid = false` it is pushed to the surface.
/// #### Panics
/// * `'Fixed: overflow'` if a projected component leaves the scalar range.
/// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if a difference leaves the scalar range.
/// #### Deviations
/// * The inside test compares the squared distance wide, so it is exact for radii below `2^-16`.
/// * A point exactly on the core segment is pushed along the counter-clockwise segment normal
///   `(dir.y, -dir.x)`, or along `+y` when the segment is shorter than
///   `rapier_math::consts::DEFAULT_EPSILON` (upstream's own two fallbacks, with its `dist >= eps`
///   guard read as `dist != 0`).
pub fn project_local_point_capsule(capsule: Capsule, pt: Vec2, solid: bool) -> PointProjection {
    let proj = project_local_point_segment(capsule.segment, pt, solid);
    let dx = pt.x - proj.point.x;
    let dy = pt.y - proj.point.y;
    match try_normalize2(dx, dy) {
        Some((
            ux, uy,
        )) => {
            let is_inside = is_norm2_le(dx, dy, capsule.radius);
            if solid && is_inside {
                PointProjection { is_inside: true, point: pt }
            } else {
                PointProjection {
                    is_inside,
                    point: Vec2 {
                        x: proj.point.x + ux * capsule.radius,
                        y: proj.point.y + uy * capsule.radius,
                    },
                }
            }
        },
        None => {
            if solid {
                return PointProjection { is_inside: true, point: pt };
            }
            // `+y` when the segment degenerates to a point and has no normal.
            let dir = capsule.segment.normal().unwrap_or(Vec2 { x: ZERO, y: ONE });
            PointProjection {
                is_inside: true,
                point: Vec2 {
                    x: proj.point.x + dir.x * capsule.radius,
                    y: proj.point.y + dir.y * capsule.radius,
                },
            }
        },
    }
}

/// Projects `pt` on `capsule` and names the feature it landed on.
///
/// Mirrors `PointQuery::project_local_point_and_get_feature` for `Capsule`: the surface of a
/// capsule is one face, so the answer is always `Face(0)`.
/// #### Panics
/// * See [`project_local_point_capsule`].
/// #### Deviations
/// * None.
#[inline(always)]
pub fn project_local_point_and_get_feature_capsule(
    capsule: Capsule, pt: Vec2,
) -> (PointProjection, FeatureId) {
    (project_local_point_capsule(capsule, pt, false), FeatureIdTrait::face(0))
}

/// Returns the distance from `pt` to `capsule`, negative inside when `solid = false`.
///
/// Mirrors the default `PointQuery::distance_to_local_point` on top of the capsule projection.
/// #### Panics
/// * See [`project_local_point_capsule`].
/// #### Deviations
/// * The length is floored (`fixed::wide::distance2`), so the magnitude is at most 1 ulp below
///   the exact distance.
pub fn distance_to_local_point_capsule(capsule: Capsule, pt: Vec2, solid: bool) -> Fixed {
    let proj = project_local_point_capsule(capsule, pt, solid);
    let dist = fixed::wide::distance2(pt.x, pt.y, proj.point.x, proj.point.y);
    if solid || !proj.is_inside {
        dist
    } else {
        ZERO - dist
    }
}

/// Returns `true` when `pt` is inside `capsule`, boundary included.
///
/// Mirrors the default `PointQuery::contains_local_point`: the distance to the core segment is
/// at most `radius`. The comparison is the wide squared one, and it never needs the projection's
/// direction, so it is cheaper than the full projection the default method would run.
/// #### Panics
/// * See [`project_local_point_capsule`].
/// #### Deviations
/// * None.
pub fn contains_local_point_capsule(capsule: Capsule, pt: Vec2) -> bool {
    let proj = project_local_point_segment(capsule.segment, pt, true);
    is_norm2_le(pt.x - proj.point.x, pt.y - proj.point.y, capsule.radius)
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, ZERO};
    use glam::vec2::Vec2;
    use rapier_testing::opaque;
    use crate::feature_id::FeatureIdTrait;
    use crate::shape::{Capsule, Segment};
    use super::super::PointProjection;
    use super::super::segment::distance_to_local_point_segment;
    use super::{
        contains_local_point_capsule, distance_to_local_point_capsule,
        project_local_point_and_get_feature_capsule, project_local_point_capsule,
    };

    const UNIT: i64 = 0x1_0000_0000;
    const HALF: i64 = 0x8000_0000;
    const QUARTER: i64 = 0x4000_0000;

    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: Fixed { raw: x }, y: Fixed { raw: y } }
    }

    /// The vertical capsule of the golden fixtures: core segment `(0, -0.5)`–`(0, 0.5)`,
    /// radius `0.25`.
    fn vertical() -> Capsule {
        Capsule {
            segment: Segment { a: v(0, -HALF), b: v(0, HALF) }, radius: Fixed { raw: QUARTER },
        }
    }

    fn degenerate() -> Capsule {
        Capsule { segment: Segment { a: v(0, 0), b: v(0, 0) }, radius: Fixed { raw: QUARTER } }
    }

    #[test]
    fn test_projection_table() {
        let c = vertical();
        // (capsule, point, solid, expected point, expected is_inside)
        let cases: Span<(Capsule, Vec2, bool, Vec2, bool)> = array![
            // Beside the core segment.
            (c, v(UNIT, 0), false, v(QUARTER, 0), false),
            (c, v(UNIT, 0), true, v(QUARTER, 0), false),
            // Exactly on the surface: inside, untouched.
            (c, v(QUARTER, 0), false, v(QUARTER, 0), true),
            (c, v(QUARTER, 0), true, v(QUARTER, 0), true),
            // Inside, off the core segment.
            (c, v(QUARTER / 2, 0), true, v(QUARTER / 2, 0), true),
            (c, v(QUARTER / 2, 0), false, v(QUARTER, 0), true),
            // Exactly on the core segment: pushed along the segment normal `(dir.y, -dir.x)`.
            (c, v(0, 0), false, v(QUARTER, 0), true), (c, v(0, 0), true, v(0, 0), true),
            (c, v(0, -HALF), false, v(QUARTER, -HALF), true),
            // Beyond a cap, along the axis.
            (c, v(0, 2 * UNIT), false, v(0, HALF + QUARTER), false),
            // A capsule whose core segment is a point: the `+y` fallback.
            (degenerate(), v(0, 0), false, v(0, QUARTER), true),
            (degenerate(), v(0, 0), true, v(0, 0), true),
            (degenerate(), v(UNIT, 0), false, v(QUARTER, 0), false),
        ]
            .span();
        for (c, pt, solid, point, is_inside) in cases {
            let got = project_local_point_capsule(*c, *pt, *solid);
            assert_eq!(got.is_inside, *is_inside);
            assert!(got.point.x.abs_diff_eq(*point.x, Fixed { raw: 8 }));
            assert!(got.point.y.abs_diff_eq(*point.y, Fixed { raw: 8 }));
        }
    }

    #[test]
    fn test_distance_and_containment() {
        let c = vertical();
        // (point, expected distance with solid = false, contains)
        let cases: Span<(Vec2, i64, bool)> = array![
            (v(UNIT, 0), UNIT - QUARTER, false), (v(QUARTER, 0), 0, true),
            (v(0, 0), -QUARTER, true), (v(0, HALF), -QUARTER, true),
            (v(0, HALF + UNIT), UNIT - QUARTER, false),
        ]
            .span();
        for (pt, expected, contains) in cases {
            let d = distance_to_local_point_capsule(c, *pt, false);
            assert!(d.abs_diff_eq(Fixed { raw: *expected }, Fixed { raw: 8 }));
            assert_eq!(contains_local_point_capsule(c, *pt), *contains);
            assert!(distance_to_local_point_capsule(c, *pt, true) >= ZERO);
        }
    }

    #[test]
    fn test_feature_is_always_face_zero() {
        let (proj, feature) = project_local_point_and_get_feature_capsule(vertical(), v(UNIT, 0));
        assert_eq!(feature, FeatureIdTrait::face(0));
        assert_eq!(proj, PointProjection { is_inside: false, point: v(QUARTER, 0) });
    }

    /// An oblique capsule: the projection stays on the surface, i.e. at `radius` from the core
    /// segment. The direction it is pushed along carries `1 ulp / d` of relative error, `d`
    /// being the distance from the query point to the core segment, so the fuzz skips the
    /// points closer than `1/16` — there the direction is genuinely undetermined, and upstream
    /// has the same cliff.
    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_projection_lies_on_the_surface(x: i16, y: i16) {
        let c = Capsule {
            segment: Segment { a: v(-HALF, -QUARTER), b: v(UNIT, HALF) },
            radius: Fixed { raw: QUARTER },
        };
        let pt = v(x.into() * 65536, y.into() * 65536);
        let min_dist = Fixed { raw: UNIT / 16 };
        if distance_to_local_point_segment(c.segment, pt, false) < min_dist {
            return;
        }
        let proj = project_local_point_capsule(c, pt, false);
        let d = distance_to_local_point_capsule(c, proj.point, false);
        assert!(d.abs_diff_eq(ZERO, Fixed { raw: 16 }));
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_project_capsule_outside() {
        let _ = project_local_point_capsule(opaque(vertical()), opaque(v(UNIT, 0)), false);
    }

    #[test]
    fn gas_project_capsule_inside_solid() {
        let _ = project_local_point_capsule(opaque(vertical()), opaque(v(QUARTER / 2, 0)), true);
    }

    #[test]
    fn gas_project_capsule_on_segment() {
        let _ = project_local_point_capsule(opaque(vertical()), opaque(v(0, 0)), false);
    }

    #[test]
    fn gas_project_and_get_feature_capsule() {
        let _ = project_local_point_and_get_feature_capsule(opaque(vertical()), opaque(v(UNIT, 0)));
    }

    #[test]
    fn gas_distance_to_local_point_capsule() {
        let _ = distance_to_local_point_capsule(opaque(vertical()), opaque(v(UNIT, 0)), false);
    }

    #[test]
    fn gas_contains_local_point_capsule() {
        let _ = contains_local_point_capsule(opaque(vertical()), opaque(v(UNIT, 0)));
    }
}
