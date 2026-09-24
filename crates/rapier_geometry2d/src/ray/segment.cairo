//! Ray casts on a [`Segment`] (Parry `query/ray/ray_support_map.rs`, `impl RayCast for
//! Segment`, 2D branch).
//!
//! Upstream solves the two lines with `closest_points_line_line_parameters_eps` (fourth-degree
//! determinant `|d|² |e|² - (d · e)²`) and then tests `s ∈ [0, max]`, `t ∈ [0, 1]`. In 2D
//! the same parameters are two ratios of **second-degree** exact cross products,
//!
//! ```text
//! o + d s = a + e t   =>   s = cross(a - o, e) / cross(d, e),   t = cross(a - o, d) / cross(d, e)
//! ```
//!
//! so the port decides `s >= 0` and `0 <= t <= 1` exactly on the wide numerators and rounds `s`
//! once ([`div_wide`]).
//!
//! # Thresholds
//!
//! * **Parallel**: upstream calls the lines parallel when `ulps_eq!(a e, b²)` (4 ulp of `f64`,
//!   i.e. `sin² <= 2^-50`) or the determinant is below `f64::EPSILON`. Its relative form is kept:
//!   `|cross(d, e)| * 2^25 <= |d| |e|`, i.e. `sin <= 2^-25`. A ray within that angle of the
//!   segment is then collinear or a miss, as upstream.
//! * **Degenerate operands**: upstream's `|d|² <= eps` / `|e|² <= eps` branches both end in a
//!   miss (a zero normal, or `normal · dir == 0`), except for a spurious hit of a ray a few
//!   `1e-8` long; the port tests for exactly zero vectors instead and answers a miss.
//! * **Collinear**: `|(a - o) · n| < DEFAULT_EPSILON` with the unit normal of the segment, as
//!   upstream (with the engine's `DEFAULT_EPSILON = 2^-23`).

use core::num::traits::WideMul;
use fixed::wide::norm2;
use fixed::{Fixed, FixedTrait, ZERO};
use glam::vec2::{Vec2, Vec2Trait};
use rapier_math::DEFAULT_EPSILON;
use rapier_math::math_ext::norm2::{is_zero2, norm2_sq_wide};
use crate::feature_id::FeatureIdTrait;
use crate::point::wide2::{cross_wide, dot_wide};
use crate::shape::{Segment, SegmentTrait};
use super::quotient::{abs_u128, div_wide};
use super::{Ray, RayIntersection};

/// `2^25`: `|cross(d, e)| <= |d| |e| / 2^25` is upstream's `ulps_eq!(a e, b b)`.
const PARALLEL_REL: u128 = 0x200_0000;

/// Returns `true` when the ray and the segment are parallel in upstream's sense.
#[inline(always)]
fn is_parallel(den: i128, d: Vec2, e: Vec2) -> bool {
    let lengths: u128 = norm2(d.x, d.y).raw.wide_mul(norm2(e.x, e.y).raw).try_into().unwrap();
    abs_u128(den) <= lengths / PARALLEL_REL
}

/// The collinear branch: the ray runs along the line of the segment.
fn cast_collinear(
    ray: Ray, normal: Vec2, r: Vec2, e: Vec2, max_time_of_impact: Fixed,
) -> Option<RayIntersection> {
    let dist1 = dot_wide(r.x, r.y, ray.dir.x, ray.dir.y);
    let dist2 = dist1 + dot_wide(e.x, e.y, ray.dir.x, ray.dir.y);
    let ahead1 = dist1 >= 0;
    let ahead2 = dist2 >= 0;
    if ahead1 != ahead2 {
        // The origin lies on the segment.
        return Some(
            RayIntersection { time_of_impact: ZERO, normal, feature: FeatureIdTrait::face(0) },
        );
    }
    if !ahead1 {
        return None;
    }
    let dd = norm2_sq_wide(ray.dir.x, ray.dir.y);
    let (dist, vertex) = if dist1 <= dist2 {
        (dist1, 0)
    } else {
        (dist2, 1)
    };
    let toi = div_wide(dist, dd)?;
    if toi > max_time_of_impact {
        return None;
    }
    Some(RayIntersection { time_of_impact: toi, normal, feature: FeatureIdTrait::vertex(vertex) })
}

/// Time of impact, normal and feature of `ray` on `segment` (local frame).
///
/// Mirrors `RayCast::cast_local_ray_and_get_normal` for `Segment` (2D): `solid` is ignored (a
/// segment has no interior). A transverse hit reports `Face(0)` with the segment normal
/// `(d.y, -d.x) / |d|` when the ray comes from its side, `Face(1)` with the opposite normal
/// otherwise. A collinear ray reports the first end point it meets (`Vertex(0|1)`, with the
/// segment normal), or `t = 0` and `Face(0)` when it starts on the segment.
/// #### Panics
/// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if `b - a` or `a - origin` leaves the scalar
///   range.
/// #### Deviations
/// * Exact parameters (see the module documentation); `s` is correctly rounded.
/// * A zero `dir` or a zero-length segment is a miss (upstream: a miss too, except for rays
///   shorter than `1.5e-8`).
pub fn cast_local_ray_and_get_normal_segment(
    segment: Segment, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<RayIntersection> {
    let d = ray.dir;
    let e = segment.b - segment.a;
    if is_zero2(d.x, d.y) || is_zero2(e.x, e.y) {
        return None;
    }
    let normal = segment.normal().unwrap_or(Vec2 { x: ZERO, y: ZERO });
    let r = segment.a - ray.origin;
    let den = cross_wide(d.x, d.y, e.x, e.y);
    if is_parallel(den, d, e) {
        if r.dot(normal).abs() < DEFAULT_EPSILON {
            return cast_collinear(ray, normal, r, e, max_time_of_impact);
        }
        return None;
    }
    let s_num = cross_wide(r.x, r.y, e.x, e.y);
    let t_num = cross_wide(r.x, r.y, d.x, d.y);
    let (s_num, t_num, den) = if den < 0 {
        (-s_num, -t_num, -den)
    } else {
        (s_num, t_num, den)
    };
    if s_num < 0 || t_num < 0 || t_num > den {
        return None;
    }
    let s = div_wide(s_num, den)?;
    if s > max_time_of_impact {
        return None;
    }
    let dot = dot_wide(normal.x, normal.y, d.x, d.y);
    if dot > 0 {
        Some(
            RayIntersection {
                time_of_impact: s,
                normal: Vec2 { x: -normal.x, y: -normal.y },
                feature: FeatureIdTrait::face(1),
            },
        )
    } else if dot < 0 {
        Some(RayIntersection { time_of_impact: s, normal, feature: FeatureIdTrait::face(0) })
    } else {
        None
    }
}

/// Time of impact of `ray` on `segment` (local frame): the time of
/// [`cast_local_ray_and_get_normal_segment`], as upstream's default `cast_local_ray`.
/// #### Panics
/// * See [`cast_local_ray_and_get_normal_segment`].
/// #### Deviations
/// * See [`cast_local_ray_and_get_normal_segment`].
#[inline(always)]
pub fn cast_local_ray_segment(
    segment: Segment, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<Fixed> {
    let hit = cast_local_ray_and_get_normal_segment(segment, ray, max_time_of_impact, solid)?;
    Some(hit.time_of_impact)
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::vec2::Vec2;
    use rapier_testing::opaque;
    use crate::feature_id::{FeatureId, FeatureIdTrait};
    use crate::shape::{Segment, SegmentTrait};
    use super::super::Ray;
    use super::{cast_local_ray_and_get_normal_segment, cast_local_ray_segment};

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn int(x: i32) -> Fixed {
        FixedTrait::from_int(x)
    }

    fn ray(ox: Fixed, oy: Fixed, dx: Fixed, dy: Fixed) -> Ray {
        Ray { origin: v(ox, oy), dir: v(dx, dy) }
    }

    /// `(-1, 0)`-`(1, 0)`: normal `(0, -1)`.
    fn seg() -> Segment {
        SegmentTrait::new(v(-ONE, ZERO), v(ONE, ZERO))
    }

    /// `(ray, max, expected (toi, normal, feature))`.
    #[test]
    fn test_cast_table() {
        let max = int(100);
        let down = v(ZERO, -ONE);
        let up = v(ZERO, ONE);
        let cases: Span<(Ray, Fixed, Option<(Fixed, Vec2, FeatureId)>)> = array![
            (ray(HALF, TWO, ZERO, -ONE), max, Some((TWO, up, FeatureIdTrait::face(1)))),
            (ray(HALF, -TWO, ZERO, ONE), max, Some((TWO, down, FeatureIdTrait::face(0)))),
            (ray(ONE, TWO, ZERO, -ONE), max, Some((TWO, up, FeatureIdTrait::face(1)))),
            (ray(ONE + Fixed { raw: 1 }, TWO, ZERO, -ONE), max, None),
            (ray(HALF, TWO, ZERO, ONE), max, None), (ray(HALF, TWO, ZERO, -ONE), ONE, None),
            (ray(HALF, TWO, ZERO, -ONE), TWO, Some((TWO, up, FeatureIdTrait::face(1)))),
            // Collinear.
            (ray(int(-3), ZERO, ONE, ZERO), max, Some((TWO, down, FeatureIdTrait::vertex(0)))),
            (ray(int(3), ZERO, -TWO, ZERO), max, Some((ONE, down, FeatureIdTrait::vertex(1)))),
            (ray(ZERO, ZERO, ONE, ZERO), max, Some((ZERO, down, FeatureIdTrait::face(0)))),
            (ray(int(3), ZERO, ONE, ZERO), max, None), (ray(int(-3), ZERO, ONE, ZERO), ONE, None),
            // Parallel, off the line; zero direction.
            (ray(int(-3), ONE, ONE, ZERO), max, None), (ray(HALF, ZERO, ZERO, ZERO), max, None),
            // Nearly parallel but above upstream's threshold (slope 2^-20): a regular hit.
            (
                ray(int(-3), Fixed { raw: -0x3000 }, ONE, Fixed { raw: 0x1000 }),
                max,
                Some((int(3), down, FeatureIdTrait::face(0))),
            ),
        ]
            .span();
        for (r, m, expected) in cases {
            let hit = cast_local_ray_and_get_normal_segment(seg(), *r, *m, true);
            match *expected {
                Some((
                    t, n, f,
                )) => {
                    let hit = hit.unwrap();
                    assert_eq!((hit.time_of_impact, hit.normal, hit.feature), (t, n, f));
                    assert_eq!(cast_local_ray_segment(seg(), *r, *m, false), Some(t));
                },
                None => assert!(hit.is_none(), "expected a miss"),
            }
        }
    }

    #[test]
    fn test_zero_length_segment_misses() {
        let point = SegmentTrait::new(v(HALF, HALF), v(HALF, HALF));
        assert!(cast_local_ray_segment(point, ray(ZERO, ZERO, ONE, ONE), int(100), true).is_none());
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_cast_local_ray_and_get_normal_segment() {
        let _ = cast_local_ray_and_get_normal_segment(
            opaque(seg()),
            opaque(ray(HALF, TWO, Fixed { raw: 0x1000_0000 }, -ONE)),
            opaque(int(100)),
            true,
        );
    }
}
