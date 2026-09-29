//! Clipping lines, rays, segments and polygons against an [`Aabb`] (Parry
//! `query/clip/clip_aabb_line.rs`, `clip_aabb_polygon.rs`, `clip_halfspace_polygon.rs`; PX3).
//!
//! # Fixed point
//!
//! * Every slab time `(m - o) / d` is one correctly rounded [`div_wide`] of two raws (upstream
//!   multiplies by a rounded `1 / d`), saturated one raw below the `clip_aabb_line` sentinels so
//!   that the tie tests (`==`) stay meaningful; a tie is detected exactly where it is exact.
//! * `clip_aabb_line` is the general-box form of the slab loop `ray::cuboid` keeps for centred
//!   boxes; the sides are `±(axis + 1)`, `+` for the `mins` slab, as upstream.
//! * Polygons are `Span<Vec2>` in, `Array<Vec2>` out: Cairo arrays cannot be cleared or
//!   reused, so upstream's `&mut Vec` in-place update and its `workspace` scratch buffer have no
//!   counterpart (see [`AabbClipTrait::clip_polygon_with_workspace`]).
//! * The "keep" test of a polygon vertex is the exact sign of a wide dot product.

use fixed::{Fixed, FixedTrait, ONE, ZERO};
use glam_core::{Vec2, Vec2Trait};
use rapier_math::math_ext::vec2::try_normalize2;
use crate::point::dot_wide;
use crate::ray::halfspace::ray_toi_with_halfspace;
use crate::ray::quotient::div_wide;
use crate::ray::{Ray, RayTrait};
use crate::shape::segment::{Segment, SegmentTrait};
use super::{Aabb, AabbTrait};

/// Saturation bound of a slab time, one below the `clip_aabb_line` sentinels.
const SLAB_MAX_RAW: i64 = 0x7fff_ffff_ffff_fffe;
/// `-f64::MAX` / `f64::MAX` of `clip_aabb_line`.
const SENTINEL_MIN: Fixed = Fixed { raw: -0x7fff_ffff_ffff_ffff };
const SENTINEL_MAX: Fixed = Fixed { raw: 0x7fff_ffff_ffff_ffff };

/// One end of a clipped line: `(time, normal, side)`, `side` being `±(axis + 1)` (`0` for none).
pub type AabbClipHit = (Fixed, Vec2, i32);

/// The running state of `clip_aabb_line`.
#[derive(Copy, Drop)]
struct Clip {
    tmin: Fixed,
    tmax: Fixed,
    near_side: i32,
    far_side: i32,
    near_diag: bool,
    far_diag: bool,
}

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

/// One axis of `clip_aabb_line`; `false` when the line misses.
#[inline(always)]
fn clip_step(ref clip: Clip, o: Fixed, d: Fixed, min: Fixed, max: Fixed, side: i32) -> bool {
    if d == ZERO {
        return !(o < min || o > max);
    }
    let to_min = slab_time(min - o, d);
    let to_max = slab_time(max - o, d);
    let flipped = to_min > to_max;
    let (near, far) = if flipped {
        (to_max, to_min)
    } else {
        (to_min, to_max)
    };
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
fn axis(side: i32, sign: Fixed) -> Vec2 {
    if side == 1 || side == -1 {
        Vec2 { x: sign, y: ZERO }
    } else {
        Vec2 { x: ZERO, y: sign }
    }
}

/// `-dir / |dir|`, the normal upstream reports at a diagonal (corner) tie; zero for a zero `dir`.
#[inline(always)]
fn diagonal_normal(dir: Vec2) -> Vec2 {
    match try_normalize2(dir.x, dir.y) {
        Some((x, y)) => Vec2 { x: -x, y: -y },
        None => Vec2 { x: ZERO, y: ZERO },
    }
}

/// Clips the line `origin + dir * t` (`t` unbounded) against `aabb` (upstream `clip_aabb_line`):
/// the `(time, normal, side)` of its entry and of its exit, or `None` when it misses the box.
///
/// A zero `dir` answers `Some` (twice `(0, 0, 0)`) only when `origin` is inside the box.
/// #### Panics
/// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if `mins - origin` or `maxs - origin` leaves
///   the scalar range.
/// #### Deviations
/// * See the module documentation: correctly rounded slab times, saturated to the scalar range.
pub fn clip_aabb_line(aabb: Aabb, origin: Vec2, dir: Vec2) -> Option<(AabbClipHit, AabbClipHit)> {
    let mut clip = Clip {
        tmin: SENTINEL_MIN,
        tmax: SENTINEL_MAX,
        near_side: 0,
        far_side: 0,
        near_diag: false,
        far_diag: false,
    };
    if !clip_step(ref clip, origin.x, dir.x, aabb.mins.x, aabb.maxs.x, 1) {
        return None;
    }
    if !clip_step(ref clip, origin.y, dir.y, aabb.mins.y, aabb.maxs.y, 2) {
        return None;
    }
    let zero = (ZERO, Vec2 { x: ZERO, y: ZERO }, 0_i32);
    let near_normal = if clip.near_diag {
        diagonal_normal(dir)
    } else if clip.near_side == 0 {
        // `dir` is zero: `Some` only if the line starts inside the box.
        return if aabb.contains_local_point(origin) {
            Some((zero, zero))
        } else {
            None
        };
    } else if clip.near_side < 0 {
        axis(clip.near_side, ONE)
    } else {
        axis(clip.near_side, -ONE)
    };
    let far_normal = if clip.far_diag {
        diagonal_normal(dir)
    } else if clip.far_side == 0 {
        return if aabb.contains_local_point(origin) {
            Some((zero, zero))
        } else {
            None
        };
    } else if clip.far_side < 0 {
        axis(clip.far_side, -ONE)
    } else {
        axis(clip.far_side, ONE)
    };
    Some(((clip.tmin, near_normal, clip.near_side), (clip.tmax, far_normal, clip.far_side)))
}

/// Keeps the part of `polygon` behind the plane through `center` of normal `normal` (upstream
/// `clip_halfspace_polygon`, Sutherland-Hodgman): a vertex is kept when `(pt - center) . normal
/// <= 0`, exactly. An empty polygon gives an empty result.
/// #### Panics
/// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if `pt - center` leaves the scalar range.
/// #### Deviations
/// * The result is a new `Array` (upstream fills a `&mut Vec`).
pub fn clip_halfspace_polygon(center: Vec2, normal: Vec2, polygon: Span<Vec2>) -> Array<Vec2> {
    let mut result = array![];
    let n = polygon.len();
    if n == 0 {
        return result;
    }
    let last_pt = *polygon.at(n - 1);
    let mut last_keep = keep_point(last_pt, center, normal);
    if last_keep {
        result.append(last_pt);
    }
    let mut i = 0;
    while i != n {
        let pt = *polygon.at(i);
        let keep = keep_point(pt, center, normal);
        if keep != last_keep {
            let prev_i = if i == 0 {
                n - 1
            } else {
                i - 1
            };
            let prev_pt = *polygon.at(prev_i);
            let ray = Ray { origin: prev_pt, dir: pt - prev_pt };
            if let Some(t) = ray_toi_with_halfspace(center, normal, ray) {
                if t > ZERO && t < ONE {
                    result.append(ray.point_at(t));
                }
            }
            last_keep = keep;
        }
        if keep && i != n - 1 {
            result.append(pt);
        }
        i += 1;
    }
    result
}

/// `(pt - center) . normal <= 0`, on the exact wide dot product.
#[inline(always)]
fn keep_point(pt: Vec2, center: Vec2, normal: Vec2) -> bool {
    let d = pt - center;
    dot_wide(d.x, d.y, normal.x, normal.y) <= 0
}

/// Clipping methods of [`Aabb`] (Parry `query/clip/clip_aabb_line.rs`, `clip_aabb_polygon.rs`).
#[generate_trait]
pub impl AabbClipImpl of AabbClipTrait {
    /// The part of the segment `pa`–`pb` inside the box (upstream `clip_segment`), `None` when
    /// it misses. The parameters are clamped to `[0, 1]`.
    /// #### Panics
    /// * See [`clip_aabb_line`]; `'Fixed: overflow'` if `pb - pa` or a clipped end point leaves
    ///   the scalar range.
    fn clip_segment(self: Aabb, pa: Vec2, pb: Vec2) -> Option<Segment> {
        let ab = pb - pa;
        let ((t0, _, _), (t1, _, _)) = clip_aabb_line(self, pa, ab)?;
        Some(SegmentTrait::new(pa + ab.mul_scalar(t0.max(ZERO)), pa + ab.mul_scalar(t1.min(ONE))))
    }

    /// The parameters `(t0, t1)` of the part of the line `orig + dir * t` inside the box
    /// (upstream `clip_line_parameters`), `None` when it misses.
    /// #### Panics
    /// * See [`clip_aabb_line`].
    fn clip_line_parameters(self: Aabb, orig: Vec2, dir: Vec2) -> Option<(Fixed, Fixed)> {
        let ((t0, _, _), (t1, _, _)) = clip_aabb_line(self, orig, dir)?;
        Some((t0, t1))
    }

    /// The part of the line `orig + dir * t` inside the box (upstream `clip_line`).
    /// #### Panics
    /// * See [`clip_aabb_line`]; `'Fixed: overflow'` if a clipped end point leaves the scalar
    ///   range.
    fn clip_line(self: Aabb, orig: Vec2, dir: Vec2) -> Option<Segment> {
        let ((t0, _, _), (t1, _, _)) = clip_aabb_line(self, orig, dir)?;
        Some(SegmentTrait::new(orig + dir.mul_scalar(t0), orig + dir.mul_scalar(t1)))
    }

    /// The parameters of the part of `ray` inside the box (upstream `clip_ray_parameters`):
    /// `None` when the box is behind the origin, the entry clamped to `0` otherwise.
    /// #### Panics
    /// * See [`clip_aabb_line`].
    fn clip_ray_parameters(self: Aabb, ray: Ray) -> Option<(Fixed, Fixed)> {
        let (t0, t1) = self.clip_line_parameters(ray.origin, ray.dir)?;
        if t1 < ZERO {
            None
        } else {
            Some((t0.max(ZERO), t1))
        }
    }

    /// The part of `ray` inside the box (upstream `clip_ray`).
    /// #### Panics
    /// * See [`AabbClipTrait::clip_ray_parameters`]; `'Fixed: overflow'` if a clipped end point
    ///   leaves the scalar range.
    fn clip_ray(self: Aabb, ray: Ray) -> Option<Segment> {
        let (t0, t1) = self.clip_ray_parameters(ray)?;
        Some(SegmentTrait::new(ray.point_at(t0), ray.point_at(t1)))
    }

    /// Clips the convex polygon `points` to the box (upstream `clip_polygon`): four
    /// [`clip_halfspace_polygon`] passes, `-x`, `+x`, `-y`, `+y`.
    /// #### Panics
    /// * See [`clip_halfspace_polygon`].
    /// #### Deviations
    /// * Returns the clipped polygon instead of updating a `&mut Vec`.
    fn clip_polygon(self: Aabb, points: Span<Vec2>) -> Array<Vec2> {
        let left = clip_halfspace_polygon(self.mins, Vec2 { x: -ONE, y: ZERO }, points);
        let right = clip_halfspace_polygon(self.maxs, Vec2 { x: ONE, y: ZERO }, left.span());
        let bottom = clip_halfspace_polygon(self.mins, Vec2 { x: ZERO, y: -ONE }, right.span());
        clip_halfspace_polygon(self.maxs, Vec2 { x: ZERO, y: ONE }, bottom.span())
    }

    /// [`AabbClipTrait::clip_polygon`] with upstream's scratch buffer.
    /// #### Deviations
    /// * A Cairo `Array` can be neither cleared nor reused: `workspace` is dropped unread, the
    ///   argument only keeps upstream's signature. The clipped polygon is returned.
    fn clip_polygon_with_workspace(
        self: Aabb, points: Span<Vec2>, workspace: Array<Vec2>,
    ) -> Array<Vec2> {
        let _ = workspace;
        self.clip_polygon(points)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, ONE, ZERO};
    use glam_core::Vec2;
    use rapier_testing::opaque;
    use crate::ray::Ray;
    use crate::shape::segment::SegmentTrait;
    use super::super::{Aabb, AabbTrait};
    use super::{AabbClipTrait, clip_aabb_line, clip_halfspace_polygon};

    /// A vector of quarter units.
    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: Fixed { raw: x * 0x4000_0000 }, y: Fixed { raw: y * 0x4000_0000 } }
    }

    /// The box `[-1, 1] x [-1, 1]`.
    fn unit_box() -> Aabb {
        AabbTrait::new(v(-4, -4), v(4, 4))
    }

    #[test]
    fn test_clip_aabb_line_sides_and_normals() {
        // Left to right along the middle: enters the `mins.x` slab (+1), exits `maxs.x` (-1).
        let (near, far) = clip_aabb_line(unit_box(), v(-8, 0), v(4, 0)).unwrap();
        let (t0, n0, s0) = near;
        let (t1, n1, s1) = far;
        assert_eq!((t0, s0, n0), (Fixed { raw: 0x1_0000_0000 }, 1, Vec2 { x: -ONE, y: ZERO }));
        // Upstream's normals point into the box: `-x` at the `mins.x` slab, `-x` at `maxs.x` too.
        assert_eq!((t1, s1, n1), (Fixed { raw: 0x3_0000_0000 }, -1, Vec2 { x: -ONE, y: ZERO }));
        // Reversed: the sides flip (`-1` then `+1`).
        let (near, far) = clip_aabb_line(unit_box(), v(8, 0), v(-4, 0)).unwrap();
        let ((_, _, s0), (_, _, s1)) = (near, far);
        assert_eq!((s0, s1), (-1, 1));
        // Vertical line: sides on axis 2.
        let (near, far) = clip_aabb_line(unit_box(), v(0, -8), v(0, 4)).unwrap();
        let ((_, _, s0), (_, _, s1)) = (near, far);
        assert_eq!((s0, s1), (2, -2));
    }

    #[test]
    fn test_clip_aabb_line_degenerate_directions() {
        let zero = Vec2 { x: ZERO, y: ZERO };
        assert!(clip_aabb_line(unit_box(), v(0, 0), zero).is_some());
        assert!(clip_aabb_line(unit_box(), v(4, 0), zero).is_some());
        assert!(clip_aabb_line(unit_box(), v(5, 0), zero).is_none());
        // A point box on a zero direction: hit only at the point.
        let point = AabbTrait::new(v(2, 2), v(2, 2));
        assert!(clip_aabb_line(point, v(2, 2), zero).is_some());
        assert!(clip_aabb_line(point, v(0, 0), zero).is_none());
        // A direction of one raw along x, outside the y slab: a miss, not a saturated hit.
        let tiny = Vec2 { x: Fixed { raw: 1 }, y: ZERO };
        assert!(clip_aabb_line(unit_box(), v(-8, 8), tiny).is_none());
        // Inside the y slab the slab times saturate to the same bound: a hit, not an overflow.
        let ((t0, _, _), (t1, _, _)) = clip_aabb_line(unit_box(), v(-8, 0), tiny).unwrap();
        assert!(t0 > ZERO && t1 >= t0);
    }

    #[test]
    fn test_clip_segment_line_ray() {
        let b = unit_box();
        assert_eq!(
            b.clip_segment(v(-8, 0), v(8, 0)).unwrap(), SegmentTrait::new(v(-4, 0), v(4, 0)),
        );
        assert!(b.clip_segment(v(-8, 8), v(8, 8)).is_none());
        // A segment ending inside keeps its own end point.
        assert_eq!(
            b.clip_segment(v(-8, 0), v(0, 0)).unwrap(), SegmentTrait::new(v(-4, 0), v(0, 0)),
        );
        assert_eq!(b.clip_line(v(0, 0), v(4, 4)).unwrap(), SegmentTrait::new(v(-4, -4), v(4, 4)));
        let ray = Ray { origin: v(0, 0), dir: v(4, 0) };
        assert_eq!(b.clip_ray_parameters(ray).unwrap(), (ZERO, ONE));
        assert!(b.clip_ray(Ray { origin: v(8, 0), dir: v(4, 0) }).is_none());
        // Upstream's line clip drops a box entirely behind the origin, like the ray.
        assert!(b.clip_line(v(8, 0), v(4, 0)).is_none());
        assert!(b.clip_line(v(-8, 0), v(4, 0)).is_some());
    }

    #[test]
    fn test_polygon_clips() {
        let square = array![v(0, 0), v(8, 0), v(8, 8), v(0, 8)];
        let half = clip_halfspace_polygon(v(4, 0), Vec2 { x: ONE, y: ZERO }, square.span());
        // The result starts with the last vertex, as upstream's loop does.
        assert_eq!(half.span(), array![v(0, 8), v(0, 0), v(4, 0), v(4, 8)].span());
        assert_eq!(
            clip_halfspace_polygon(v(0, 0), Vec2 { x: ONE, y: ZERO }, array![].span()).len(), 0,
        );
        // Fully inside: unchanged, fully outside: empty, around the box: the box.
        let inside = array![v(-1, -1), v(1, -1), v(0, 1)];
        // (four passes, each starting with the previous last vertex: rotated by four places).
        assert_eq!(
            unit_box().clip_polygon(inside.span()).span(),
            array![v(0, 1), v(-1, -1), v(1, -1)].span(),
        );
        assert_eq!(unit_box().clip_polygon(array![v(8, 8), v(12, 8), v(8, 12)].span()).len(), 0);
        let around = array![v(-8, -8), v(8, -8), v(8, 8), v(-8, 8)];
        assert_eq!(unit_box().clip_polygon(around.span()).len(), 4);
        // The workspace variant answers the same polygon.
        let a = unit_box().clip_polygon(around.span());
        let b = unit_box().clip_polygon_with_workspace(around.span(), array![v(0, 0)]);
        assert_eq!(a.span(), b.span());
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_clip_aabb_line() {
        let _ = clip_aabb_line(opaque(unit_box()), opaque(v(-8, 1)), opaque(v(4, 1)));
    }

    #[test]
    fn gas_clip_aabb_line_miss() {
        let _ = clip_aabb_line(opaque(unit_box()), opaque(v(-8, 8)), opaque(v(4, 0)));
    }

    #[test]
    fn gas_clip_segment() {
        let _ = opaque(unit_box()).clip_segment(opaque(v(-8, 1)), opaque(v(8, 0)));
    }

    #[test]
    fn gas_clip_line_parameters() {
        let _ = opaque(unit_box()).clip_line_parameters(opaque(v(-8, 1)), opaque(v(4, 1)));
    }

    #[test]
    fn gas_clip_line() {
        let _ = opaque(unit_box()).clip_line(opaque(v(-8, 1)), opaque(v(4, 1)));
    }

    #[test]
    fn gas_clip_ray_parameters() {
        let _ = opaque(unit_box())
            .clip_ray_parameters(opaque(Ray { origin: v(-8, 1), dir: v(4, 1) }));
    }

    #[test]
    fn gas_clip_ray() {
        let _ = opaque(unit_box()).clip_ray(opaque(Ray { origin: v(-8, 1), dir: v(4, 1) }));
    }

    #[test]
    fn gas_clip_halfspace_polygon() {
        let square = array![v(0, 0), v(8, 0), v(8, 8), v(0, 8)];
        let _ = clip_halfspace_polygon(
            opaque(v(4, 0)), opaque(Vec2 { x: ONE, y: ZERO }), square.span(),
        );
    }

    #[test]
    fn gas_clip_polygon() {
        let tri = array![v(0, 0), v(8, 0), v(0, 8)];
        let _ = opaque(unit_box()).clip_polygon(tri.span());
    }

    #[test]
    fn gas_clip_polygon_with_workspace() {
        let tri = array![v(0, 0), v(8, 0), v(0, 8)];
        let _ = opaque(unit_box()).clip_polygon_with_workspace(tri.span(), array![]);
        let _ = FixedTrait::from_raw(0);
    }
}
