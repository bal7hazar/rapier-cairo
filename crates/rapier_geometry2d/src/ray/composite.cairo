//! Ray casts on the composite shapes (work package SH2a): the [`Polyline`] (Parry
//! `query/ray/ray_composite_shape.rs`) and the 2D [`HeightField`] (`query/ray/ray_heightfield.rs`).
//!
//! * Polyline: the earliest hit among its segments (`crate::ray::segment`, whose feature and
//!   normal the answer keeps, as upstream's part answer), strictly below `max_time_of_impact`,
//!   ascending segment index on exact ties (upstream: BVH order). The `*_part` function also
//!   returns the segment's index (upstream `CompositeShapeRef::cast_local_ray_and_get_normal`, and
//!   `RayIntersection::subshape` in parry 0.31, a field the port keeps out of
//!   `RayIntersection`, see the module documentation of `crate::ray`).
//! * Heightfield: upstream's walk, ported as is: the ray is clipped to the local box, the cell
//!   under its entry point is tested (`s >= 0`), then the cells it crosses in `x` order until the
//!   ray's parameter at a cell boundary reaches the clipped exit; `solid` is ignored. The hit's
//!   normal is the cell's `normal()` whatever the side, and its feature is `Face(cell)` from above,
//!   `Face(cell + num_cells)` from below. A vertical ray only tests the first cell.
//!
//! The line parameters are the exact cross-product ratios of `crate::ray::segment` (upstream:
//! `closest_points_line_line_parameters`, which gives the same values for crossing lines; a
//! parallel ray gets upstream's `s = 0` and the projection of its origin as `t`).

use fixed::{Fixed, MAX, ZERO};
use glam::vec2::Vec2Trait;
use rapier_math::math_ext::norm2::norm2_sq_wide;
use crate::aabb::Aabb;
use crate::feature_id::FeatureIdTrait;
use crate::point::wide2::{cross_wide, dot_wide};
use crate::shape::{HeightField, HeightFieldTrait, Polyline, PolylineTrait, Segment, SegmentTrait};
use super::quotient::div_wide;
use super::segment::cast_local_ray_and_get_normal_segment;
use super::{Ray, RayIntersection};

/// The earliest hit of `ray` on the segments of `polyline`, strictly below
/// `max_time_of_impact`: `(segment, hit)`.
pub fn cast_local_ray_and_get_normal_polyline_part(
    polyline: @Polyline, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<(u32, RayIntersection)> {
    let mut best: Option<(u32, RayIntersection)> = None;
    let mut bound = max_time_of_impact;
    let mut i: u32 = 0;
    for seg in polyline.segments() {
        if let Some(hit) = cast_local_ray_and_get_normal_segment(seg, ray, bound, solid) {
            let better = match best {
                Some((_, b)) => hit.time_of_impact < b.time_of_impact,
                None => hit.time_of_impact < max_time_of_impact,
            };
            if better {
                best = Some((i, hit));
                bound = hit.time_of_impact;
            }
        }
        i += 1;
    }
    best
}

/// Upstream `RayCast::cast_local_ray_and_get_normal` for `Polyline`.
pub fn cast_local_ray_and_get_normal_polyline(
    polyline: @Polyline, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<RayIntersection> {
    let (_, hit) = cast_local_ray_and_get_normal_polyline_part(
        polyline, ray, max_time_of_impact, solid,
    )?;
    Some(hit)
}

/// Upstream `RayCast::cast_local_ray` for `Polyline`.
pub fn cast_local_ray_polyline(
    polyline: @Polyline, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<Fixed> {
    let (_, hit) = cast_local_ray_and_get_normal_polyline_part(
        polyline, ray, max_time_of_impact, solid,
    )?;
    Some(hit.time_of_impact)
}

/// `(max(t_enter, 0), t_exit)` of the ray's line through `aabb` (upstream
/// `Aabb::clip_ray_parameters`), `None` when the ray misses the box or leaves it before `t = 0`.
pub fn clip_ray_parameters(aabb: Aabb, ray: Ray) -> Option<(Fixed, Fixed)> {
    let mut tmin = -MAX;
    let mut tmax = MAX;
    let (o, d) = (ray.origin, ray.dir);
    // x slab.
    if d.x == ZERO {
        if o.x < aabb.mins.x || o.x > aabb.maxs.x {
            return None;
        }
    } else {
        let (near, far) = slab(aabb.mins.x, aabb.maxs.x, o.x, d.x);
        if near > tmin {
            tmin = near;
        }
        if far < tmax {
            tmax = far;
        }
        if tmax < ZERO || tmin > tmax {
            return None;
        }
    }
    // y slab.
    if d.y == ZERO {
        if o.y < aabb.mins.y || o.y > aabb.maxs.y {
            return None;
        }
    } else {
        let (near, far) = slab(aabb.mins.y, aabb.maxs.y, o.y, d.y);
        if near > tmin {
            tmin = near;
        }
        if far < tmax {
            tmax = far;
        }
        if tmax < ZERO || tmin > tmax {
            return None;
        }
    }
    let tmin = if tmin > ZERO {
        tmin
    } else {
        ZERO
    };
    Some((tmin, tmax))
}

/// The ray parameters `(near, far)` of the slab `[lo, hi]` along one axis (`d != 0`), each a
/// correctly rounded quotient (`±MAX` beyond the scalar range).
fn slab(lo: Fixed, hi: Fixed, o: Fixed, d: Fixed) -> (Fixed, Fixed) {
    let den: i128 = d.raw.into();
    let a = ratio((lo - o).raw.into(), den);
    let b = ratio((hi - o).raw.into(), den);
    if a > b {
        (b, a)
    } else {
        (a, b)
    }
}

/// `num / den` (`den != 0`), saturated to `±MAX`.
#[inline(always)]
fn ratio(num: i128, den: i128) -> Fixed {
    let saturated = if (num < 0) == (den < 0) {
        MAX
    } else {
        -MAX
    };
    div_wide(num, den).unwrap_or(saturated)
}

/// The parameters `(s, t)` of the ray's line and the cell's line where they meet (upstream
/// `closest_points_line_line_parameters`): exact numerators over a positive common denominator,
/// `(s_num, t_num, den)`; a parallel pair answers upstream's `s = 0` and the projection of the
/// origin on the cell's line (`den = |e|^2`).
fn line_params(ray: Ray, seg: Segment) -> (i128, i128, i128) {
    let d = ray.dir;
    let e = seg.b - seg.a;
    let r = seg.a - ray.origin;
    let den = cross_wide(d.x, d.y, e.x, e.y);
    if den == 0 {
        // `t = e . (o - a) / |e|^2`.
        return (0, -dot_wide(e.x, e.y, r.x, r.y), norm2_sq_wide(e.x, e.y));
    }
    let s_num = cross_wide(r.x, r.y, e.x, e.y);
    let t_num = cross_wide(r.x, r.y, d.x, d.y);
    if den < 0 {
        (-s_num, -t_num, -den)
    } else {
        (s_num, t_num, den)
    }
}

/// The hit of cell `cell` (segment `seg`) at `s = s_num / den`: the cell's normal, the feature
/// by side.
fn cell_hit(
    h: @HeightField, cell: u32, seg: Segment, ray: Ray, s_num: i128, den: i128,
) -> RayIntersection {
    let n = seg.normal().unwrap();
    let side = if n.dot(ray.dir) > ZERO {
        cell + h.num_cells()
    } else {
        cell
    };
    RayIntersection {
        time_of_impact: ratio(s_num, den), normal: n, feature: FeatureIdTrait::face(side),
    }
}

/// Upstream `RayCast::cast_local_ray_and_get_normal` for the 2D `HeightField` (see the module
/// documentation), with the hit cell.
pub fn cast_local_ray_and_get_normal_heightfield_part(
    h: @HeightField, ray: Ray, max_time_of_impact: Fixed,
) -> Option<(u32, RayIntersection)> {
    let (min_t, max_t) = clip_ray_parameters(h.local_aabb(), ray)?;
    if min_t > max_time_of_impact {
        return None;
    }
    let max_t = if max_t < max_time_of_impact {
        max_t
    } else {
        max_time_of_impact
    };
    let entry = ray.origin + ray.dir.mul_scalar(min_t);
    let cells = h.num_cells();
    let outside = if ray.origin.x > ZERO {
        cells - 1
    } else {
        0
    };
    let mut curr: u32 = h.cell_at_point(entry).unwrap_or(outside);
    // The cell under the ray.
    if let Some(seg) = h.segment_at(curr) {
        let (s_num, t_num, den) = line_params(ray, seg);
        if s_num >= 0 && t_num >= 0 && t_num <= den {
            return Some((curr, cell_hit(h, curr, seg, ray, s_num, den)));
        }
    }
    if ray.dir.x == ZERO {
        return None;
    }
    let right = ray.dir.x > ZERO;
    let den_x: i128 = ray.dir.x.raw.into();
    while (right && curr < cells) || (!right && curr > 0) {
        let boundary = if right {
            curr += 1;
            h.x_at(curr)
        } else {
            let b = h.x_at(curr);
            curr -= 1;
            b
        };
        let param = ratio((boundary - ray.origin.x).raw.into(), den_x);
        if param >= max_t {
            return None;
        }
        if let Some(seg) = h.segment_at(curr) {
            let (s_num, t_num, den) = line_params(ray, seg);
            if t_num >= 0 && t_num <= den {
                let s = ratio(s_num, den);
                if s <= max_time_of_impact {
                    return Some((curr, cell_hit(h, curr, seg, ray, s_num, den)));
                }
            }
        }
    }
    None
}

/// Upstream `RayCast::cast_local_ray_and_get_normal` for `HeightField` (`solid` is ignored).
pub fn cast_local_ray_and_get_normal_heightfield(
    h: @HeightField, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<RayIntersection> {
    let (_, hit) = cast_local_ray_and_get_normal_heightfield_part(h, ray, max_time_of_impact)?;
    Some(hit)
}

/// Upstream's default `RayCast::cast_local_ray` for `HeightField`: the time of the hit.
pub fn cast_local_ray_heightfield(
    h: @HeightField, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<Fixed> {
    let (_, hit) = cast_local_ray_and_get_normal_heightfield_part(h, ray, max_time_of_impact)?;
    Some(hit.time_of_impact)
}
