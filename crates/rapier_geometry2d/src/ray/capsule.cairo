//! Ray casts on a [`Capsule`] (Parry `query/ray/ray_support_map.rs`, `impl RayCast for
//! Capsule`).
//!
//! Upstream casts rays on a capsule through the generic support-map path: GJK ray casting
//! (`gjk::cast_local_ray`, iterative, `eps_tol = 10 f64::EPSILON`) and, for a hollow ray from
//! inside, a second GJK cast backwards from a point shifted beyond the shape. Porting GJK for
//! one convex shape whose boundary is known in closed form would buy iterations and tolerances
//! for nothing, so this module solves the same problem **analytically**: a capsule is the union
//! of the two discs of its end points and of the rectangle swept by its core segment, a line
//! crosses a convex union along one interval, and that interval is the union of the three
//! component intervals. The entry is therefore the smallest component entry and the exit the
//! largest component exit, each from exact wide quantities (circle kernel of
//! [`super::ball`], slab tests in the frame of the segment without normalising it).
//!
//! What is kept from upstream is what GJK *answers*, not how it gets there:
//!
//! * a zero `dir` is a miss, even from inside (`ray_length == 0 → None`);
//! * a `solid` ray starting inside (boundary included) answers `t = 0` with the normal
//!   `-dir / |dir|` (GJK's initial search direction);
//! * a hollow ray starting inside answers the exit with the **inward** normal, if within `max`;
//! * the feature is always `Unknown`;
//! * `t <= max_time_of_impact` is kept.
//!
//! A GJK answer is within its tolerance of the exact one; the analytic one is within an ulp or
//! two, so the golden comparison measures upstream's convergence rather than this port.

use core::num::traits::WideMul;
use fixed::wide::norm2;
use fixed::{Fixed, ZERO};
use glam::vec2::Vec2;
use rapier_math::math_ext::norm2::{is_zero2, norm2_sq_wide};
use rapier_math::math_ext::vec2::try_normalize2;
use crate::feature_id::FEATURE_UNKNOWN;
use crate::point::wide2::{cross_wide, dot_wide};
use crate::shape::Capsule;
use super::ball::{circle_coefficients, circle_normal, sqrt_discriminant};
use super::quotient::div_wide;
use super::{Ray, RayIntersection, RayTrait};

/// Which boundary piece an interval end lies on.
pub(crate) const DISC_A: u8 = 0;
pub(crate) const DISC_B: u8 = 1;
/// The side `cross(e, p - a) = +r |e|` (left of `a → b`).
const SIDE_LEFT: u8 = 2;
/// The side `cross(e, p - a) = -r |e|` (right of `a → b`).
const SIDE_RIGHT: u8 = 3;
/// The end lines of the rectangle, inside the discs (only reached on ties).
const END_A: u8 = 4;
const END_B: u8 = 5;

const TIME_MIN: Fixed = Fixed { raw: -0x7fff_ffff_ffff_ffff };
const TIME_MAX: Fixed = Fixed { raw: 0x7fff_ffff_ffff_ffff };

/// The interval of the line `origin + dir * t` inside one component, with the pieces its ends
/// lie on.
#[derive(Copy, Drop, Debug)]
pub(crate) struct Interval {
    pub(crate) entry: Fixed,
    pub(crate) exit: Fixed,
    pub(crate) entry_piece: u8,
    pub(crate) exit_piece: u8,
}

/// `num / den` (`den != 0`), saturated to the time sentinels.
#[inline(always)]
fn div_saturated(num: i128, den: i128) -> Fixed {
    let saturated = if (num < 0) != (den < 0) {
        TIME_MIN
    } else {
        TIME_MAX
    };
    div_wide(num, den).unwrap_or(saturated)
}

/// The interval of the line inside the disc of `center` (`None` when it misses), and whether
/// the origin is inside that disc. `dir` is not zero.
pub(crate) fn disc_interval(
    center: Vec2, radius: Fixed, ray: Ray, piece: u8,
) -> (bool, Option<Interval>) {
    let q = circle_coefficients(center, radius, ray);
    let inside = q.k <= 0;
    match sqrt_discriminant(q) {
        Some(s) => (
            inside,
            Some(
                Interval {
                    entry: div_saturated(-q.b - s, q.a),
                    exit: div_saturated(s - q.b, q.a),
                    entry_piece: piece,
                    exit_piece: piece,
                },
            ),
        ),
        None => (inside, None),
    }
}

/// The interval of `p0 + p1 t ∈ [lo, hi]` (all at the same wide scale), `None` when empty.
#[inline(always)]
fn slab(p0: i128, p1: i128, lo: i128, hi: i128, lo_piece: u8, hi_piece: u8) -> Option<Interval> {
    if p1 == 0 {
        if p0 < lo || p0 > hi {
            return None;
        }
        return Some(
            Interval {
                entry: TIME_MIN, exit: TIME_MAX, entry_piece: lo_piece, exit_piece: hi_piece,
            },
        );
    }
    let t_lo = div_saturated(lo - p0, p1);
    let t_hi = div_saturated(hi - p0, p1);
    if t_lo <= t_hi {
        Some(Interval { entry: t_lo, exit: t_hi, entry_piece: lo_piece, exit_piece: hi_piece })
    } else {
        Some(Interval { entry: t_hi, exit: t_lo, entry_piece: hi_piece, exit_piece: lo_piece })
    }
}

/// The interval of the line inside the rectangle swept by the core segment (`None` when it
/// misses or the segment has zero length), and whether the origin is inside it.
///
/// In the frame of `e = b - a`, without normalising it: the axial coordinate
/// `(p - a) · e ∈ [0, |e|²]` and the lateral one `cross(e, p - a) ∈ [-r |e|, r |e|]`, both
/// raw Q64.64 and linear in `t`.
pub(crate) fn rect_interval(capsule: Capsule, ray: Ray) -> (bool, Option<Interval>) {
    let a = capsule.segment.a;
    let e = capsule.segment.b - a;
    if is_zero2(e.x, e.y) {
        return (false, None);
    }
    let oa = ray.origin - a;
    let d = ray.dir;
    let u0 = dot_wide(oa.x, oa.y, e.x, e.y);
    let u1 = dot_wide(d.x, d.y, e.x, e.y);
    let ee = norm2_sq_wide(e.x, e.y);
    let w0 = cross_wide(e.x, e.y, oa.x, oa.y);
    let w1 = cross_wide(e.x, e.y, d.x, d.y);
    let h = capsule.radius.raw.wide_mul(norm2(e.x, e.y).raw);
    let inside = u0 >= 0 && u0 <= ee && w0 >= -h && w0 <= h;
    let Some(axial) = slab(u0, u1, 0, ee, END_A, END_B) else {
        return (inside, None);
    };
    let Some(lateral) = slab(w0, w1, -h, h, SIDE_RIGHT, SIDE_LEFT) else {
        return (inside, None);
    };
    // The later entry and the earlier exit; a tie goes to the side (a genuine face).
    let (entry, entry_piece) = if axial.entry > lateral.entry {
        (axial.entry, axial.entry_piece)
    } else {
        (lateral.entry, lateral.entry_piece)
    };
    let (exit, exit_piece) = if axial.exit < lateral.exit {
        (axial.exit, axial.exit_piece)
    } else {
        (lateral.exit, lateral.exit_piece)
    };
    if entry > exit {
        return (inside, None);
    }
    (inside, Some(Interval { entry, exit, entry_piece, exit_piece }))
}

/// The outward normal of `piece` at `point` (negated when `inward`).
pub(crate) fn piece_normal(capsule: Capsule, piece: u8, point: Vec2, inward: bool) -> Vec2 {
    if piece == DISC_A {
        return circle_normal(capsule.segment.a, point, inward);
    }
    if piece == DISC_B {
        return circle_normal(capsule.segment.b, point, inward);
    }
    let e = capsule.segment.b - capsule.segment.a;
    let (ux, uy) = try_normalize2(e.x, e.y).unwrap_or((ZERO, ZERO));
    let (x, y) = if piece == SIDE_LEFT {
        (-uy, ux)
    } else if piece == SIDE_RIGHT {
        (uy, -ux)
    } else if piece == END_A {
        (-ux, -uy)
    } else {
        (ux, uy)
    };
    if inward {
        Vec2 { x: -x, y: -y }
    } else {
        Vec2 { x, y }
    }
}

/// Keeps the earlier entry (`first`) or the later exit, preferring the incumbent on ties.
#[inline(always)]
fn better(
    candidate: Option<Interval>, best: Option<(Fixed, u8)>, entry: bool,
) -> Option<(Fixed, u8)> {
    let Some(i) = candidate else {
        return best;
    };
    if i.exit < ZERO {
        // Entirely behind the origin.
        return best;
    }
    let (t, piece) = if entry {
        (i.entry, i.entry_piece)
    } else {
        (i.exit, i.exit_piece)
    };
    match best {
        Some((b, _)) => if (entry && t < b) || (!entry && t > b) {
            Some((t, piece))
        } else {
            best
        },
        None => Some((t, piece)),
    }
}

/// Time of impact, normal and feature of `ray` on `capsule` (local frame).
///
/// Mirrors `RayCast::cast_local_ray_and_get_normal` for `Capsule` (see the module documentation
/// for the answers kept from upstream's GJK).
/// #### Panics
/// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if a coordinate difference leaves the scalar
///   range; `'u256_add Overflow'` only for coordinates near `2^31`.
/// #### Deviations
/// * Closed-form instead of GJK: the time of impact is within a few ulp of the exact one
///   (upstream: within GJK's tolerance), and the normal is the exact outward normal at the hit
///   point (upstream: GJK's last search direction).
pub fn cast_local_ray_and_get_normal_capsule(
    capsule: Capsule, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<RayIntersection> {
    let d = ray.dir;
    if is_zero2(d.x, d.y) {
        return None;
    }
    let (in_a, disc_a) = disc_interval(capsule.segment.a, capsule.radius, ray, DISC_A);
    let (in_b, disc_b) = disc_interval(capsule.segment.b, capsule.radius, ray, DISC_B);
    let (in_rect, rect) = rect_interval(capsule, ray);
    let inside = in_a || in_b || in_rect;
    if inside && solid {
        let (x, y) = try_normalize2(d.x, d.y).unwrap();
        return Some(
            RayIntersection {
                time_of_impact: ZERO, normal: Vec2 { x: -x, y: -y }, feature: FEATURE_UNKNOWN,
            },
        );
    }
    // Entering from outside: the earliest entry; leaving from inside: the latest exit.
    let entry = !inside;
    let best = better(disc_a, None, entry);
    let best = better(disc_b, best, entry);
    let (t, piece) = better(rect, best, entry)?;
    let t = if t < ZERO {
        ZERO
    } else {
        t
    };
    if t > max_time_of_impact {
        return None;
    }
    Some(
        RayIntersection {
            time_of_impact: t,
            normal: piece_normal(capsule, piece, ray.point_at(t), inside),
            feature: FEATURE_UNKNOWN,
        },
    )
}

/// Time of impact of `ray` on `capsule` (local frame): the time of
/// [`cast_local_ray_and_get_normal_capsule`], as upstream's default `cast_local_ray`.
/// #### Panics
/// * See [`cast_local_ray_and_get_normal_capsule`].
/// #### Deviations
/// * See [`cast_local_ray_and_get_normal_capsule`].
#[inline(always)]
pub fn cast_local_ray_capsule(
    capsule: Capsule, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<Fixed> {
    let hit = cast_local_ray_and_get_normal_capsule(capsule, ray, max_time_of_impact, solid)?;
    Some(hit.time_of_impact)
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::vec2::Vec2;
    use rapier_testing::opaque;
    use crate::point::contains_local_point_capsule;
    use crate::shape::{Capsule, CapsuleTrait};
    use super::super::{Ray, RayTrait};
    use super::{cast_local_ray_and_get_normal_capsule, cast_local_ray_capsule};

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn int(x: i32) -> Fixed {
        FixedTrait::from_int(x)
    }

    fn ray(ox: Fixed, oy: Fixed, dx: Fixed, dy: Fixed) -> Ray {
        Ray { origin: v(ox, oy), dir: v(dx, dy) }
    }

    /// Core `(0, -0.5)`-`(0, 0.5)`, radius `0.5`.
    fn capsule() -> Capsule {
        CapsuleTrait::new_y(HALF, HALF)
    }

    /// `(ray, max, solid, expected (toi, normal))`; exact cases only.
    #[test]
    fn test_cast_table() {
        let max = int(100);
        let cases: Span<(Ray, Fixed, bool, Option<(Fixed, Vec2)>)> = array![
            // Sides, caps, and their tangent junction.
            (ray(int(-2), ZERO, ONE, ZERO), max, true, Some((TWO - HALF, v(-ONE, ZERO)))),
            (ray(int(2), HALF, -ONE, ZERO), max, true, Some((TWO - HALF, v(ONE, ZERO)))),
            (ray(ZERO, int(3), ZERO, -ONE), max, true, Some((TWO, v(ZERO, ONE)))),
            (ray(ZERO, int(-3), ZERO, TWO), max, true, Some((ONE, v(ZERO, -ONE)))),
            // Inside: solid 0 with -dir, hollow the exit with the inward normal.
            (ray(ZERO, ZERO, ONE, ZERO), max, true, Some((ZERO, v(-ONE, ZERO)))),
            (ray(ZERO, ZERO, ONE, ZERO), max, false, Some((HALF, v(-ONE, ZERO)))),
            (ray(ZERO, ZERO, ZERO, ONE), max, false, Some((ONE, v(ZERO, -ONE)))),
            (ray(ZERO, ZERO, ZERO, ONE), HALF, false, None),
            // Misses: parallel outside, above, behind, cut by max, zero direction.
            (ray(ONE, int(-3), ZERO, ONE), max, true, None),
            (ray(int(-2), TWO, ONE, ZERO), max, true, None),
            (ray(int(2), ZERO, ONE, ZERO), max, true, None),
            (ray(int(-2), ZERO, ONE, ZERO), ONE, true, None),
            (ray(ZERO, ZERO, ZERO, ZERO), max, true, None),
        ]
            .span();
        for (r, m, solid, expected) in cases {
            let hit = cast_local_ray_and_get_normal_capsule(capsule(), *r, *m, *solid);
            match *expected {
                Some((
                    t, n,
                )) => {
                    let hit = hit.unwrap();
                    assert_eq!((hit.time_of_impact, hit.normal), (t, n));
                    assert_eq!(cast_local_ray_capsule(capsule(), *r, *m, *solid), Some(t));
                },
                None => assert!(hit.is_none(), "expected a miss"),
            }
        }
    }

    /// Brute force: the hit point is on the boundary (within a few ulp of the radius from the
    /// core), and marching the ray from 0 by 1/16 steps meets no inside point before it.
    #[test]
    #[fuzzer(runs: 48, seed: 5)]
    fn fuzz_hit_against_sampling(ox: i16, oy: i16, dx: i8, dy: i8) {
        let c = CapsuleTrait::new(v(-HALF, Fixed { raw: -0x4000_0000 }), v(ONE, HALF), HALF);
        let r = ray(
            Fixed { raw: ox.into() * 0x40000 },
            Fixed { raw: oy.into() * 0x40000 },
            Fixed { raw: dx.into() * 0x100_0000 },
            Fixed { raw: dy.into() * 0x100_0000 },
        );
        if contains_local_point_capsule(c, r.origin) || (dx == 0 && dy == 0) {
            return;
        }
        let hit = cast_local_ray_capsule(c, r, int(1000), true);
        let step = Fixed { raw: 0x1000_0000 };
        let mut t = ZERO;
        let limit = match hit {
            Some(t) => t,
            None => int(16),
        } - Fixed { raw: 0x100 };
        while t < limit {
            assert!(!contains_local_point_capsule(c, r.point_at(t)), "inside before the hit");
            t = t + step;
        }
        if let Some(t) = hit {
            let back = t - Fixed { raw: 0x100 };
            let forward = t + Fixed { raw: 0x100 };
            assert!(!contains_local_point_capsule(c, r.point_at(back)), "entry too late");
            assert!(contains_local_point_capsule(c, r.point_at(forward)), "entry too early");
        }
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_cast_local_ray_and_get_normal_capsule() {
        let _ = cast_local_ray_and_get_normal_capsule(
            opaque(capsule()),
            opaque(ray(int(-2), Fixed { raw: 0x1000_0000 }, ONE, HALF)),
            opaque(int(100)),
            true,
        );
    }
}
