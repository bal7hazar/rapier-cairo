//! Ray casts on a [`Ball`] (Parry `query/ray/ray_ball.rs`), and the circle kernel the capsule
//! reuses for its two caps.
//!
//! Upstream solves `|o + d t - c|² = r²`: with `a = |d|²`, `b = (o - c) · d`,
//! `k = |o - c|² - r²`, the entry is `t = (-b - sqrt(b² - a k)) / a`. Here `a`, `b` and `k` are
//! the exact raw Q64.64 sums of products, the discriminant is the exact 256-bit
//! `b² - a k`, its square root is taken once at that width and the quotient is one correctly
//! rounded [`div_wide`]. Every branch upstream takes on a rounded value (`k > 0`, `b > 0`,
//! `delta < 0`, `t <= 0`) is taken here on the exact numerator.
//!
//! # Candidates
//!
//! Ranked by the `gas_*` probes of this module; the rejected one lives in
//! `#[cfg(test)] mod alternatives`.
//!
//! 1. **wide** (this module): exact coefficients, 256-bit discriminant, one `isqrt` and one
//!    wide division. **Winner** on accuracy: tangent rays keep their exact `delta == 0`.
//! 2. `alternatives::toi_normalized`: normalise `dir` first so that `a = 1`, then solve in
//!    `Fixed` (`dot`, `sqrt`, one division by `|dir|`). Cheaper, but `delta` is rounded to Q32.32
//!    before its square root, so a near-tangent ray loses half of its bits (an error of `2^-16`
//!    on `t` for a rounding of `2^-32` on `delta`).

use core::num::traits::WideMul;
use fixed::{Fixed, ZERO};
use glam::vec2::Vec2;
use rapier_math::math_ext::norm2::{norm2_sq_wide, sq_wide};
use rapier_math::math_ext::vec2::try_normalize2;
use crate::feature_id::FeatureIdTrait;
use crate::point::wide2::dot_wide;
use crate::shape::Ball;
use super::quotient::{abs_u128, div_wide, isqrt_wide};
use super::{Ray, RayIntersection, RayTrait};

/// The exact coefficients of `a t² + 2 b t + k = 0`, raw Q64.64.
#[derive(Copy, Drop, Debug)]
pub struct CircleCoefficients {
    /// `|dir|²`.
    pub a: i128,
    /// `(origin - center) · dir`.
    pub b: i128,
    /// `|origin - center|² - radius²`: `<= 0` when the origin is inside (boundary included).
    pub k: i128,
}

/// The coefficients of the ray against the circle of `center` and `radius`.
/// #### Panics
/// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if `origin - center` leaves the scalar range.
#[inline(always)]
pub fn circle_coefficients(center: Vec2, radius: Fixed, ray: Ray) -> CircleCoefficients {
    let dc = ray.origin - center;
    CircleCoefficients {
        a: norm2_sq_wide(ray.dir.x, ray.dir.y),
        b: dot_wide(dc.x, dc.y, ray.dir.x, ray.dir.y),
        k: norm2_sq_wide(dc.x, dc.y) - sq_wide(radius),
    }
}

/// `sqrt(b² - a k)` rounded to nearest, at the raw Q64.64 scale of `b`; `None` when the
/// discriminant is negative (the line misses the circle).
/// #### Panics
/// * `'u256_add Overflow'` / `'Option::unwrap failed.'` only for coordinates near `2^31`.
pub fn sqrt_discriminant(q: CircleCoefficients) -> Option<i128> {
    let b = abs_u128(q.b);
    let bb: u256 = b.wide_mul(b);
    let ak: u256 = abs_u128(q.a).wide_mul(abs_u128(q.k));
    let delta = if q.k > 0 {
        if ak > bb {
            return None;
        }
        bb - ak
    } else {
        bb + ak
    };
    Some(isqrt_wide(delta).try_into().unwrap())
}

/// Port of upstream `ray_toi_with_ball`: `(inside, time_of_impact)`, where `inside` tells that
/// the answer is `0` (solid) or the exit (hollow) because the origin is inside or on the circle.
/// A zero `dir` answers `(true, Some(0))` inside, whatever `solid`, as upstream.
/// #### Panics
/// * See [`circle_coefficients`] and [`sqrt_discriminant`].
pub fn ray_toi_with_ball(
    center: Vec2, radius: Fixed, ray: Ray, solid: bool,
) -> (bool, Option<Fixed>) {
    let q = circle_coefficients(center, radius, ray);
    if q.a == 0 {
        return if q.k > 0 {
            (false, None)
        } else {
            (true, Some(ZERO))
        };
    }
    if q.k > 0 && q.b > 0 {
        return (false, None);
    }
    let Some(s) = sqrt_discriminant(q) else {
        return (false, None);
    };
    let near = -q.b - s;
    if near <= 0 {
        if solid {
            (true, Some(ZERO))
        } else {
            (true, div_wide(s - q.b, q.a))
        }
    } else {
        (false, div_wide(near, q.a))
    }
}

/// Port of upstream `ray_toi_and_normal_with_ball`: [`ray_toi_with_ball`] plus the normal at
/// the hit point (`Face(0)`), pointing into the circle when `inside`, as upstream.
/// #### Panics
/// * See [`ray_toi_with_ball`]; `'Fixed: overflow'` if the hit point leaves the scalar range.
/// #### Deviations
/// * A hit at the centre (zero-radius ball) answers a zero normal, upstream `NaN`.
pub fn ray_toi_and_normal_with_ball(
    center: Vec2, radius: Fixed, ray: Ray, solid: bool,
) -> (bool, Option<RayIntersection>) {
    let (inside, toi) = ray_toi_with_ball(center, radius, ray, solid);
    match toi {
        Some(t) => (
            inside,
            Some(
                RayIntersection {
                    time_of_impact: t,
                    normal: circle_normal(center, ray.point_at(t), inside),
                    feature: FeatureIdTrait::face(0),
                },
            ),
        ),
        None => (inside, None),
    }
}

/// `normalize(point - center)`, negated when `inward`; zero when the point is the centre.
#[inline(always)]
pub fn circle_normal(center: Vec2, point: Vec2, inward: bool) -> Vec2 {
    let p = point - center;
    match try_normalize2(p.x, p.y) {
        Some((x, y)) => if inward {
            Vec2 { x: -x, y: -y }
        } else {
            Vec2 { x, y }
        },
        None => Vec2 { x: ZERO, y: ZERO },
    }
}

/// Time of impact of `ray` on `ball` (local frame).
///
/// Mirrors `RayCast::cast_local_ray` for `Ball`: `solid` answers 0 from inside, hollow answers
/// the exit; `t <= max_time_of_impact` is kept. A zero `dir` answers 0 inside (both modes) and
/// misses outside.
/// #### Panics
/// * See [`ray_toi_with_ball`].
/// #### Deviations
/// * `t` is the correctly rounded exact root (one rounding instead of upstream's four).
pub fn cast_local_ray_ball(
    ball: Ball, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<Fixed> {
    let (_, toi) = ray_toi_with_ball(Vec2 { x: ZERO, y: ZERO }, ball.radius, ray, solid);
    let toi = toi?;
    if toi <= max_time_of_impact {
        Some(toi)
    } else {
        None
    }
}

/// Time of impact, normal and feature of `ray` on `ball` (local frame).
///
/// Mirrors `RayCast::cast_local_ray_and_get_normal` for `Ball`: the normal is the outward one
/// at the hit point, or the inward one when the ray started inside; the feature is `Face(0)`.
/// #### Panics
/// * See [`ray_toi_with_ball`].
/// #### Deviations
/// * A solid ray starting exactly at the centre gets a zero normal (upstream: `NaN`,
///   `normalize(0)`).
pub fn cast_local_ray_and_get_normal_ball(
    ball: Ball, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<RayIntersection> {
    let center = Vec2 { x: ZERO, y: ZERO };
    let (inside, toi) = ray_toi_with_ball(center, ball.radius, ray, solid);
    let toi = toi?;
    if toi > max_time_of_impact {
        return None;
    }
    Some(
        RayIntersection {
            time_of_impact: toi,
            normal: circle_normal(center, ray.point_at(toi), inside),
            feature: FeatureIdTrait::face(0),
        },
    )
}

/// Rejected candidates, kept for the `gas_*` ranking.
#[cfg(test)]
pub mod alternatives {
    use fixed::wide::norm2;
    use fixed::{Fixed, FixedTrait, ZERO};
    use glam::vec2::{Vec2, Vec2Trait};
    use crate::shape::Ball;
    use super::super::Ray;

    /// The entry time with `dir` normalised first: `t' = -b' - sqrt(b'² - k)` in `Fixed`, then
    /// `t = t' / |dir|`. Solid casts only.
    pub fn toi_normalized(ball: Ball, ray: Ray) -> Option<Fixed> {
        let len = norm2(ray.dir.x, ray.dir.y);
        if len == ZERO {
            return None;
        }
        let u = Vec2 { x: ray.dir.x / len, y: ray.dir.y / len };
        let b = ray.origin.dot(u);
        let k = ray.origin.dot(ray.origin) - ball.radius * ball.radius;
        if k > ZERO && b > ZERO {
            return None;
        }
        let delta = b * b - k;
        if delta < ZERO {
            return None;
        }
        let t = -b - delta.sqrt();
        if t <= ZERO {
            Some(ZERO)
        } else {
            Some(t / len)
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::vec2::Vec2;
    use rapier_testing::opaque;
    use crate::feature_id::FeatureIdTrait;
    use crate::shape::{Ball, BallTrait};
    use super::alternatives::toi_normalized;
    use super::super::{Ray, RayIntersection};
    use super::{cast_local_ray_and_get_normal_ball, cast_local_ray_ball};

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn int(x: i32) -> Fixed {
        FixedTrait::from_int(x)
    }

    fn ray(ox: Fixed, oy: Fixed, dx: Fixed, dy: Fixed) -> Ray {
        Ray { origin: v(ox, oy), dir: v(dx, dy) }
    }

    fn ball() -> Ball {
        BallTrait::new(HALF)
    }

    /// `(ray, max, solid, expected toi)`; exact cases only (the golden test covers the rest).
    #[test]
    fn test_cast_table() {
        let max = int(100);
        let quarter = Fixed { raw: 0x4000_0000 };
        let cases: Span<(Ray, Fixed, bool, Option<Fixed>)> = array![
            (ray(int(-2), ZERO, ONE, ZERO), max, true, Some(TWO - HALF)),
            (ray(int(-2), ZERO, int(4), ZERO), max, true, Some(Fixed { raw: 0x6000_0000 })),
            // Tangent: a single root.
            (ray(int(-2), HALF, ONE, ZERO), max, true, Some(TWO)),
            (ray(int(-2), HALF + Fixed { raw: 1 }, ONE, ZERO), max, true, None),
            // Inside: solid 0, hollow the exit.
            (ray(ZERO, ZERO, ONE, ZERO), max, true, Some(ZERO)),
            (ray(ZERO, ZERO, ONE, ZERO), max, false, Some(HALF)),
            (ray(quarter, ZERO, -ONE, ZERO), max, false, Some(HALF + quarter)),
            // On the circle, pointing out: inside, hollow exit 0.
            (ray(HALF, ZERO, ONE, ZERO), max, false, Some(ZERO)),
            (ray(int(2), ZERO, ONE, ZERO), max, true, None),
            // Behind, pointing at the ball but c > 0 and b > 0.
            (ray(int(-2), ZERO, -ONE, ZERO), max, true, None),
            // max inclusive.
            (ray(int(-2), ZERO, ONE, ZERO), TWO - HALF, true, Some(TWO - HALF)),
            (ray(int(-2), ZERO, ONE, ZERO), ONE, true, None),
            // Zero direction.
            (ray(quarter, ZERO, ZERO, ZERO), max, false, Some(ZERO)),
            (ray(int(2), ZERO, ZERO, ZERO), max, true, None),
            // A ray a few ulp long still hits (the coefficients are wide).
            (ray(int(-1), ZERO, Fixed { raw: 2 }, ZERO), int(1_000_000_000), true, None),
            (
                ray(-ONE, ZERO, Fixed { raw: 0x100 }, ZERO),
                int(100_000_000),
                true,
                Some(int(8_388_608)),
            ),
        ]
            .span();
        for (r, m, solid, expected) in cases {
            assert_eq!(cast_local_ray_ball(ball(), *r, *m, *solid), *expected);
        }
    }

    #[test]
    fn test_normals() {
        let max = int(100);
        let hit = cast_local_ray_and_get_normal_ball(
            ball(), ray(int(-2), ZERO, ONE, ZERO), max, true,
        );
        let face = FeatureIdTrait::face(0);
        assert_eq!(
            hit,
            Some(
                RayIntersection {
                    time_of_impact: TWO - HALF, normal: v(-ONE, ZERO), feature: face,
                },
            ),
        );
        // Hollow from inside: the exit, with the inward normal.
        let hit = cast_local_ray_and_get_normal_ball(
            ball(), ray(ZERO, ZERO, ZERO, ONE), max, false,
        );
        assert_eq!(
            hit,
            Some(RayIntersection { time_of_impact: HALF, normal: v(ZERO, -ONE), feature: face }),
        );
        // Solid from the exact centre: zero normal (upstream NaN).
        let hit = cast_local_ray_and_get_normal_ball(ball(), ray(ZERO, ZERO, ONE, ZERO), max, true);
        assert_eq!(
            hit,
            Some(RayIntersection { time_of_impact: ZERO, normal: v(ZERO, ZERO), feature: face }),
        );
    }

    /// Against the normalised-direction candidate on unit axis rays, where both are exact.
    #[test]
    #[fuzzer(runs: 64, seed: 11)]
    fn fuzz_normalized_candidate_on_axis_rays(ox: i16, oy: i16) {
        let r = ray(
            Fixed { raw: ox.into() * 0x10000 }, Fixed { raw: oy.into() * 0x1000 }, ONE, ZERO,
        );
        let exact = cast_local_ray_ball(ball(), r, int(1000), true);
        let other = toi_normalized(ball(), r);
        match (exact, other) {
            (Some(a), Some(b)) => assert!((a - b).abs() <= Fixed { raw: 0x10000 }, "toi"),
            (None, None) => {},
            // A tangent ray may flip on the rounded discriminant.
            _ => {},
        }
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn test_ray_toi_and_normal_with_ball() {
        let center = v(ONE, ZERO);
        let (inside, hit) = super::ray_toi_and_normal_with_ball(
            center, HALF, ray(-ONE, ZERO, ONE, ZERO), true,
        );
        let hit = hit.unwrap();
        assert!(!inside);
        assert_eq!((hit.time_of_impact, hit.normal), (ONE + HALF, v(-ONE, ZERO)));
        // Hollow from inside: the exit, the normal pointing inwards.
        let (inside, hit) = super::ray_toi_and_normal_with_ball(
            center, HALF, ray(ONE, ZERO, ONE, ZERO), false,
        );
        assert!(inside);
        assert_eq!(hit.unwrap().normal, v(-ONE, ZERO));
        let (_, miss) = super::ray_toi_and_normal_with_ball(
            center, HALF, ray(-ONE, TWO, ONE, ZERO), true,
        );
        assert!(miss.is_none());
    }

    #[test]
    fn gas_ray_toi_and_normal_with_ball() {
        let _ = super::ray_toi_and_normal_with_ball(
            opaque(v(ONE, ZERO)), opaque(HALF), opaque(ray(-ONE, ZERO, ONE, ZERO)), true,
        );
    }

    #[test]
    fn gas_cast_local_ray_ball() {
        let _ = cast_local_ray_ball(
            opaque(ball()),
            opaque(ray(int(-2), Fixed { raw: 0x1000_0000 }, ONE, ZERO)),
            opaque(int(100)),
            true,
        );
    }

    #[test]
    fn gas_cast_local_ray_and_get_normal_ball() {
        let _ = cast_local_ray_and_get_normal_ball(
            opaque(ball()),
            opaque(ray(int(-2), Fixed { raw: 0x1000_0000 }, ONE, ZERO)),
            opaque(int(100)),
            true,
        );
    }

    #[test]
    fn gas_toi_normalized() {
        let _ = toi_normalized(
            opaque(ball()), opaque(ray(int(-2), Fixed { raw: 0x1000_0000 }, ONE, ZERO)),
        );
    }
}
