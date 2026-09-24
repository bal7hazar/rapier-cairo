//! Ray casts on a [`HalfSpace`] (Parry `query/ray/ray_halfspace.rs`).
//!
//! Upstream divides `n · (-o)` by `n · d` in `f64` and lets IEEE arithmetic answer the
//! degenerate cases: a ray parallel to the plane gives `±inf` (rejected by `t <= max`) or `NaN`
//! on the plane (rejected by `t >= 0`). Here both dot products are exact wide values, a zero
//! denominator is a miss, and the sign test `t >= 0` is read off the two exact operands.

use fixed::{Fixed, ZERO};
use glam::vec2::Vec2;
use crate::feature_id::FeatureIdTrait;
use crate::point::wide2::dot_wide;
use crate::shape::HalfSpace;
use super::quotient::div_wide;
use super::{Ray, RayIntersection};

/// Time of impact, normal and feature of `ray` on `halfspace` (local frame).
///
/// Mirrors `RayCast::cast_local_ray_and_get_normal` for `HalfSpace`: a `solid` ray starting
/// strictly inside answers `t = 0` with a **zero** normal; otherwise the plane is hit at
/// `t = n · (-o) / n · d` when `0 <= t <= max`, with the normal `-n` from inside and `n` from
/// outside or on the plane. The feature is always `Face(0)`.
/// #### Panics
/// * `'i128_add Overflow'` only for coordinates at the very ends of the scalar range.
/// #### Deviations
/// * `n · d == 0` is a miss (upstream: `±inf` or `NaN`, rejected by the same comparisons).
/// * `t` is the correctly rounded quotient of the exact dot products; a quotient beyond the
///   scalar range is a miss (it exceeds any representable `max`).
pub fn cast_local_ray_and_get_normal_halfspace(
    halfspace: HalfSpace, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<RayIntersection> {
    let n = halfspace.normal;
    // `n . (-o)`, positive when the origin is strictly inside.
    let num = -dot_wide(n.x, n.y, ray.origin.x, ray.origin.y);
    if solid && num > 0 {
        return Some(
            RayIntersection {
                time_of_impact: ZERO,
                normal: Vec2 { x: ZERO, y: ZERO },
                feature: FeatureIdTrait::face(0),
            },
        );
    }
    let den = dot_wide(n.x, n.y, ray.dir.x, ray.dir.y);
    if den == 0 {
        return None;
    }
    // `t >= 0` on the exact operands (`0 / x` is a hit at 0).
    if num != 0 && (num > 0) != (den > 0) {
        return None;
    }
    let t = div_wide(num, den)?;
    if t > max_time_of_impact {
        return None;
    }
    let normal = if num > 0 {
        Vec2 { x: -n.x, y: -n.y }
    } else {
        n
    };
    Some(RayIntersection { time_of_impact: t, normal, feature: FeatureIdTrait::face(0) })
}

/// Time of impact of `ray` on `halfspace` (local frame).
///
/// Mirrors the default `RayCast::cast_local_ray` of `HalfSpace` (the time of
/// [`cast_local_ray_and_get_normal_halfspace`]).
/// #### Panics
/// * See [`cast_local_ray_and_get_normal_halfspace`].
/// #### Deviations
/// * See [`cast_local_ray_and_get_normal_halfspace`].
#[inline(always)]
pub fn cast_local_ray_halfspace(
    halfspace: HalfSpace, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<Fixed> {
    let hit = cast_local_ray_and_get_normal_halfspace(halfspace, ray, max_time_of_impact, solid)?;
    Some(hit.time_of_impact)
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::vec2::Vec2;
    use rapier_testing::opaque;
    use crate::shape::{HalfSpace, HalfSpaceTrait};
    use super::super::Ray;
    use super::{cast_local_ray_and_get_normal_halfspace, cast_local_ray_halfspace};

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn ray(ox: Fixed, oy: Fixed, dx: Fixed, dy: Fixed) -> Ray {
        Ray { origin: v(ox, oy), dir: v(dx, dy) }
    }

    fn up() -> HalfSpace {
        HalfSpaceTrait::new(v(ZERO, ONE))
    }

    /// `(ray, max, solid, expected (toi, normal))`.
    #[test]
    fn test_cast_table() {
        let max = FixedTrait::from_int(100);
        let zero = v(ZERO, ZERO);
        let n = v(ZERO, ONE);
        let cases: Span<(Ray, Fixed, bool, Option<(Fixed, Vec2)>)> = array![
            (ray(ZERO, TWO, HALF, -ONE), max, true, Some((TWO, n))),
            // Inside: solid 0 with no normal, hollow the plane with -n or a miss.
            (ray(ZERO, -ONE, ONE, ZERO), max, true, Some((ZERO, zero))),
            (ray(ZERO, -ONE, ONE, ZERO), max, false, None),
            (ray(ZERO, -ONE, ZERO, TWO), max, false, Some((HALF, v(ZERO, -ONE)))),
            // Parallel outside, pointing away, on the plane, on the plane and parallel.
            (ray(ZERO, ONE, ONE, ZERO), max, true, None),
            (ray(ZERO, ONE, ZERO, ONE), max, true, None),
            (ray(ZERO, ZERO, ONE, -ONE), max, true, Some((ZERO, n))),
            (ray(ZERO, ZERO, ONE, ZERO), max, true, None),
            // max inclusive.
            (ray(ZERO, TWO, ZERO, -ONE), TWO, true, Some((TWO, n))),
            (ray(ZERO, TWO, ZERO, -ONE), ONE, true, None),
            // Zero direction.
            (ray(ZERO, ONE, ZERO, ZERO), max, true, None),
            (ray(ZERO, -ONE, ZERO, ZERO), max, true, Some((ZERO, zero))),
            (ray(ZERO, -ONE, ZERO, ZERO), max, false, None),
        ]
            .span();
        for (r, m, solid, expected) in cases {
            let hit = cast_local_ray_and_get_normal_halfspace(up(), *r, *m, *solid);
            match *expected {
                Some((
                    t, normal,
                )) => {
                    let hit = hit.unwrap();
                    assert_eq!((hit.time_of_impact, hit.normal), (t, normal));
                    assert_eq!(cast_local_ray_halfspace(up(), *r, *m, *solid), Some(t));
                },
                None => assert!(hit.is_none(), "expected a miss"),
            }
        }
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_cast_local_ray_and_get_normal_halfspace() {
        let _ = cast_local_ray_and_get_normal_halfspace(
            opaque(up()),
            opaque(ray(HALF, TWO, HALF, -ONE)),
            opaque(FixedTrait::from_int(100)),
            true,
        );
    }
}
