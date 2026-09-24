//! Ray casts on a [`Cuboid`] (Parry `query/ray/ray_cuboid.rs`, which forwards to
//! `ray_aabb.rs` and `query/clip/clip_aabb_line.rs` on `[-half_extents, half_extents]`).
//!
//! Upstream implements the two entry points with **two different algorithms**, and this port
//! keeps both, because they do not agree:
//!
//! * `cast_local_ray` is the slab loop with `tmin = 0`, `tmax = max`: a hollow ray whose entry
//!   is exactly 0 (origin inside, or on the boundary pointing in) answers `tmax`, i.e. the exit
//!   clipped to `max` — possibly `max` itself;
//! * `cast_local_ray_and_get_normal` goes through `clip_aabb_line` (unbounded slabs, sides and
//!   diagonal ties tracked): a hollow ray from inside answers the exit only when it is within
//!   `max`, and a ray starting on the boundary pointing in answers `0`.
//!
//! Every slab time `(m - o) / d` is one correctly rounded [`div_wide`] of two raws (upstream
//! multiplies by a rounded `1 / d`), saturated to the scalar range so that the comparisons of
//! the slab loop stay meaningful for rays nearly parallel to an axis.

use fixed::{Fixed, ZERO};
use glam::vec2::Vec2;
use rapier_math::math_ext::vec2::try_normalize2;
use crate::feature_id::{FEATURE_UNKNOWN, FeatureId, FeatureIdTrait};
use crate::shape::Cuboid;
use super::quotient::div_wide;
use super::{Ray, RayIntersection};

/// Saturation bound of a slab time, one below the `clip_aabb_line` sentinels.
const SLAB_MAX_RAW: i64 = 0x7fff_ffff_ffff_fffe;
/// `-f64::MAX` / `f64::MAX` of `clip_aabb_line`.
const SENTINEL_MIN: Fixed = Fixed { raw: -0x7fff_ffff_ffff_ffff };
const SENTINEL_MAX: Fixed = Fixed { raw: 0x7fff_ffff_ffff_ffff };

/// `num / den` for `den != 0`, saturated to `±SLAB_MAX_RAW`.
#[inline(always)]
fn slab_time(num: Fixed, den: Fixed) -> Fixed {
    let saturated = if (num.raw < 0) != (den.raw < 0) {
        Fixed { raw: -SLAB_MAX_RAW }
    } else {
        Fixed { raw: SLAB_MAX_RAW }
    };
    div_wide(num.raw.into(), den.raw.into()).unwrap_or(saturated)
}

/// The two slab times of one axis, `(near, far, flipped)` with `near <= far`.
#[inline(always)]
fn slab(o: Fixed, d: Fixed, half_extent: Fixed) -> (Fixed, Fixed, bool) {
    let to_min = slab_time(-half_extent - o, d);
    let to_max = slab_time(half_extent - o, d);
    if to_min > to_max {
        (to_max, to_min, true)
    } else {
        (to_min, to_max, false)
    }
}

/// One axis of upstream's `cast_local_ray` slab loop; `false` when the ray misses.
#[inline(always)]
fn slab_step(o: Fixed, d: Fixed, half_extent: Fixed, ref tmin: Fixed, ref tmax: Fixed) -> bool {
    if d == ZERO {
        return !(o < -half_extent || o > half_extent);
    }
    let (near, far, _) = slab(o, d, half_extent);
    if near > tmin {
        tmin = near;
    }
    if far < tmax {
        tmax = far;
    }
    tmin <= tmax
}

/// Time of impact of `ray` on `cuboid` (local frame).
///
/// Mirrors `RayCast::cast_local_ray` for `Cuboid` (the `Aabb` slab loop): the entry time
/// clamped below by 0; for a hollow ray whose entry is exactly 0, the exit time clamped above
/// by `max_time_of_impact` (so a hollow ray from inside whose exit is beyond `max` answers
/// `max`, and a zero `dir` inside answers `max`).
/// #### Panics
/// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if `±half_extents - origin` leaves the scalar
///   range.
/// #### Deviations
/// * Each slab time is correctly rounded (upstream: `(m - o) * (1 / d)`, two roundings).
pub fn cast_local_ray_cuboid(
    cuboid: Cuboid, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<Fixed> {
    let he = cuboid.half_extents;
    let mut tmin = ZERO;
    let mut tmax = max_time_of_impact;
    if !slab_step(ray.origin.x, ray.dir.x, he.x, ref tmin, ref tmax) {
        return None;
    }
    if !slab_step(ray.origin.y, ray.dir.y, he.y, ref tmin, ref tmax) {
        return None;
    }
    if tmin == ZERO && !solid {
        Some(tmax)
    } else {
        Some(tmin)
    }
}

/// The running state of `clip_aabb_line`.
#[derive(Copy, Drop)]
struct Clip {
    tmin: Fixed,
    tmax: Fixed,
    /// `±(axis + 1)`, `0` while no axis constrained the line.
    near_side: i8,
    far_side: i8,
    near_diag: bool,
    far_diag: bool,
}

/// One axis of `clip_aabb_line`; `false` when the line misses.
#[inline(always)]
fn clip_step(ref clip: Clip, o: Fixed, d: Fixed, half_extent: Fixed, side: i8) -> bool {
    if d == ZERO {
        return !(o < -half_extent || o > half_extent);
    }
    let (near, far, flipped) = slab(o, d, half_extent);
    if near > clip.tmin {
        clip.tmin = near;
        clip.near_side = if flipped {
            -side
        } else {
            side
        };
        clip.near_diag = false;
    } else if near == clip.tmin {
        clip.near_diag = true;
    }
    if far < clip.tmax {
        clip.tmax = far;
        clip.far_side = if flipped {
            side
        } else {
            -side
        };
        clip.far_diag = false;
    } else if far == clip.tmax {
        clip.far_diag = true;
    }
    !(clip.tmax < ZERO || clip.tmin > clip.tmax)
}

/// The unit axis vector `sign * e_{|side| - 1}`.
#[inline(always)]
fn axis(side: i8, sign: Fixed) -> Vec2 {
    if side == 1 || side == -1 {
        Vec2 { x: sign, y: ZERO }
    } else {
        Vec2 { x: ZERO, y: sign }
    }
}

/// Upstream's feature code of a clipped side: `Face(side - 1)` for a `mins` side,
/// `Face(-side - 1 + 3)` for a `maxs` side, `Unknown` for none.
#[inline(always)]
fn side_feature(side: i8) -> FeatureId {
    if side == 0 {
        FEATURE_UNKNOWN
    } else if side < 0 {
        let code: u32 = (2 - side).try_into().unwrap();
        FeatureIdTrait::face(code)
    } else {
        let code: u32 = (side - 1).try_into().unwrap();
        FeatureIdTrait::face(code)
    }
}

/// `-dir / |dir|`, the normal upstream reports at a diagonal (corner) tie.
#[inline(always)]
fn diagonal_normal(dir: Vec2) -> Vec2 {
    match try_normalize2(dir.x, dir.y) {
        Some((x, y)) => Vec2 { x: -x, y: -y },
        None => Vec2 { x: ZERO, y: ZERO },
    }
}

/// Port of `clip_aabb_line`: the `(time, normal, side)` of the entry and of the exit of the
/// line `origin + dir * t` (`t` unbounded), or `None` when it misses the box.
fn clip_aabb_line(cuboid: Cuboid, ray: Ray) -> Option<((Fixed, Vec2, i8), (Fixed, Vec2, i8))> {
    let he = cuboid.half_extents;
    let mut clip = Clip {
        tmin: SENTINEL_MIN,
        tmax: SENTINEL_MAX,
        near_side: 0,
        far_side: 0,
        near_diag: false,
        far_diag: false,
    };
    if !clip_step(ref clip, ray.origin.x, ray.dir.x, he.x, 1) {
        return None;
    }
    if !clip_step(ref clip, ray.origin.y, ray.dir.y, he.y, 2) {
        return None;
    }
    // No axis constrained the line: `dir` is zero and every coordinate is within the box.
    let zero = (ZERO, Vec2 { x: ZERO, y: ZERO }, 0_i8);
    if clip.near_side == 0 {
        return Some((zero, zero));
    }
    let near_normal = if clip.near_diag {
        diagonal_normal(ray.dir)
    } else if clip.near_side < 0 {
        axis(clip.near_side, fixed::ONE)
    } else {
        axis(clip.near_side, -fixed::ONE)
    };
    let far_normal = if clip.far_diag {
        diagonal_normal(ray.dir)
    } else if clip.far_side < 0 {
        axis(clip.far_side, -fixed::ONE)
    } else {
        axis(clip.far_side, fixed::ONE)
    };
    Some(((clip.tmin, near_normal, clip.near_side), (clip.tmax, far_normal, clip.far_side)))
}

/// Time of impact, normal and feature of `ray` on `cuboid` (local frame).
///
/// Mirrors `RayCast::cast_local_ray_and_get_normal` for `Cuboid` (`ray_aabb`): from outside
/// (or on the boundary), the entry with the outward normal of the face hit; from strictly
/// inside, `solid` answers `t = 0` with a zero normal and the feature of the exit face, hollow
/// answers the exit (with upstream's inward normal) when it is within `max_time_of_impact`. A
/// ray reaching two slabs at the same time (a corner) gets the normal `-dir / |dir|`. Features:
/// `Face(0)` / `Face(1)` for the `-x` / `-y` faces, `Face(3)` / `Face(4)` for `+x` / `+y`
/// (upstream's 3D-flavoured `+ 3`), `Unknown` for a zero `dir` inside the box.
/// #### Panics
/// * See [`cast_local_ray_cuboid`].
/// #### Deviations
/// * Each slab time is correctly rounded, so a corner tie is detected exactly where it is
///   exact; upstream's rounded reciprocal can miss a tie (or invent one) by an ulp.
pub fn cast_local_ray_and_get_normal_cuboid(
    cuboid: Cuboid, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<RayIntersection> {
    let ((near_t, near_n, near_side), (far_t, far_n, far_side)) = clip_aabb_line(cuboid, ray)?;
    let (t, normal, side) = if near_t < ZERO {
        if solid {
            (ZERO, Vec2 { x: ZERO, y: ZERO }, far_side)
        } else if far_t <= max_time_of_impact {
            (far_t, far_n, far_side)
        } else {
            return None;
        }
    } else if near_t <= max_time_of_impact {
        (near_t, near_n, near_side)
    } else {
        return None;
    };
    Some(RayIntersection { time_of_impact: t, normal, feature: side_feature(side) })
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::vec2::Vec2;
    use rapier_testing::opaque;
    use crate::feature_id::{FEATURE_UNKNOWN, FeatureIdTrait};
    use crate::shape::{Cuboid, CuboidTrait};
    use super::super::Ray;
    use super::{cast_local_ray_and_get_normal_cuboid, cast_local_ray_cuboid, side_feature};

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn int(x: i32) -> Fixed {
        FixedTrait::from_int(x)
    }

    fn ray(ox: Fixed, oy: Fixed, dx: Fixed, dy: Fixed) -> Ray {
        Ray { origin: v(ox, oy), dir: v(dx, dy) }
    }

    /// Half extents `(1, 0.5)`.
    fn cuboid() -> Cuboid {
        CuboidTrait::new(v(ONE, HALF))
    }

    /// `(ray, max, solid, cast_local_ray, and_get_normal's (toi, normal, feature code or 99 for
    /// unknown))`.
    #[test]
    fn test_cast_table() {
        let max = int(100);
        let z = v(ZERO, ZERO);
        let cases: Span<(Ray, Fixed, bool, Option<Fixed>, Option<(Fixed, Vec2, u32)>)> = array![
            (ray(int(-3), ZERO, ONE, ZERO), max, true, Some(TWO), Some((TWO, v(-ONE, ZERO), 0))),
            (
                ray(ZERO, TWO, ZERO, -ONE),
                max,
                true,
                Some(TWO - HALF),
                Some((TWO - HALF, v(ZERO, ONE), 4)),
            ),
            (
                ray(ZERO, -TWO, ZERO, ONE),
                max,
                true,
                Some(TWO - HALF),
                Some((TWO - HALF, v(ZERO, -ONE), 1)),
            ),
            (ray(int(3), ZERO, -ONE, ZERO), max, true, Some(TWO), Some((TWO, v(ONE, ZERO), 3))),
            // Inside.
            (ray(ZERO, ZERO, ONE, ZERO), max, true, Some(ZERO), Some((ZERO, z, 3))),
            (ray(ZERO, ZERO, ONE, ZERO), max, false, Some(ONE), Some((ONE, v(-ONE, ZERO), 3))),
            (ray(ZERO, ZERO, ONE, ZERO), HALF, false, Some(HALF), None),
            // On the -x face pointing in: the two entry points disagree.
            (ray(-ONE, ZERO, ONE, ZERO), max, false, Some(TWO), Some((ZERO, v(-ONE, ZERO), 0))),
            // Misses.
            (ray(int(-3), ONE, ONE, ZERO), max, true, None, None),
            (ray(int(3), ZERO, ONE, ZERO), max, true, None, None),
            (ray(int(-3), ZERO, ONE, ZERO), ONE, true, None, None),
            // Zero direction.
            (ray(ZERO, ZERO, ZERO, ZERO), max, true, Some(ZERO), Some((ZERO, z, 99))),
            (ray(ZERO, ZERO, ZERO, ZERO), max, false, Some(max), Some((ZERO, z, 99))),
            (ray(int(2), ZERO, ZERO, ZERO), max, true, None, None),
        ]
            .span();
        for (r, m, solid, toi, hit) in cases {
            assert_eq!(cast_local_ray_cuboid(cuboid(), *r, *m, *solid), *toi);
            let got = cast_local_ray_and_get_normal_cuboid(cuboid(), *r, *m, *solid);
            match *hit {
                Some((
                    t, n, code,
                )) => {
                    let got = got.unwrap();
                    assert_eq!((got.time_of_impact, got.normal), (t, n));
                    if code == 99 {
                        assert_eq!(got.feature, FEATURE_UNKNOWN);
                    } else {
                        assert_eq!(got.feature, FeatureIdTrait::face(code));
                    }
                },
                None => assert!(got.is_none(), "expected a miss"),
            }
        }
    }

    /// A ray reaching both slabs at once gets `-dir / |dir|`.
    #[test]
    fn test_corner_tie() {
        let got = cast_local_ray_and_get_normal_cuboid(
            cuboid(), ray(int(-2), -TWO + HALF, ONE, ONE), int(100), true,
        )
            .unwrap();
        assert_eq!(got.time_of_impact, ONE);
        let s = Fixed { raw: -3037000500 };
        assert_eq!(got.normal, v(s, s));
        assert_eq!(side_feature(2), FeatureIdTrait::face(1));
        assert_eq!(side_feature(-1), FeatureIdTrait::face(3));
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_cast_local_ray_cuboid() {
        let _ = cast_local_ray_cuboid(
            opaque(cuboid()),
            opaque(ray(int(-3), Fixed { raw: 0x1000_0000 }, ONE, HALF)),
            opaque(int(100)),
            true,
        );
    }

    #[test]
    fn gas_cast_local_ray_and_get_normal_cuboid() {
        let _ = cast_local_ray_and_get_normal_cuboid(
            opaque(cuboid()),
            opaque(ray(int(-3), Fixed { raw: 0x1000_0000 }, ONE, HALF)),
            opaque(int(100)),
            true,
        );
    }
}
