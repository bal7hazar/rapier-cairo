//! Ray casts on a round shape (Parry `query/ray/ray_round_shape.rs`).
//!
//! Upstream casts on the support map of the dilated shape with GJK (and a second, backward GJK
//! cast for a hollow ray from inside). Here the closed form of `super::capsule` is extended: the
//! boundary of a convex polygon dilated by `r` lies on the union of the capsules of its edges
//! (one disc per vertex, one rectangle per edge), the line crosses the convex dilated shape along
//! one interval, and that interval's ends are the earliest component entry and the latest
//! component exit. The origin is inside when it is inside a component or inside the polygon
//! itself (exact half-plane tests, skipped for a degenerate polygon).
//!
//! What is kept from upstream's GJK answers, as for the capsule: a zero `dir` misses; a solid ray
//! from inside answers `t = 0` with the normal `-dir / |dir|`; a hollow ray from inside answers
//! the exit with the inward normal; the feature is always `Unknown`.

use fixed::{Fixed, ZERO};
use glam::vec2::Vec2;
use rapier_math::math_ext::norm2::is_zero2;
use rapier_math::math_ext::vec2::try_normalize2;
use crate::feature_id::FEATURE_UNKNOWN;
use crate::point::wide2::cross_wide;
use crate::shape::triangle::triangle_core;
use crate::shape::{Capsule, ConvexPolygon, Cuboid, Segment, Triangle};
use super::capsule::{DISC_A, Interval, disc_interval, piece_normal, rect_interval};
use super::{Ray, RayIntersection, RayTrait};

/// Whether `o` is inside the counter-clockwise polygon `vertices` (boundary included); `false`
/// for a degenerate (zero-area) polygon, whose points the capsules cover anyway.
fn interior_contains(vertices: Span<Vec2>, o: Vec2) -> bool {
    let n = vertices.len();
    let mut area: i128 = 0;
    let mut inside = true;
    let mut k = 0;
    while k != n {
        let a = *vertices[k];
        let b = *vertices[if k + 1 == n {
            0
        } else {
            k + 1
        }];
        let e = b - a;
        let d = o - a;
        if cross_wide(e.x, e.y, d.x, d.y) < 0 {
            inside = false;
        }
        area += cross_wide(a.x, a.y, b.x, b.y);
        k += 1;
    }
    inside && area > 0
}

/// The earliest entry (`entry`) or latest exit of `candidate`, with its piece and capsule
/// index, keeping the incumbent on ties (as `super::capsule`).
#[inline(always)]
fn better(
    candidate: Option<Interval>, k: u32, best: Option<(Fixed, u8, u32)>, entry: bool,
) -> Option<(Fixed, u8, u32)> {
    let Some(i) = candidate else {
        return best;
    };
    if i.exit < ZERO {
        return best;
    }
    let (t, piece) = if entry {
        (i.entry, i.entry_piece)
    } else {
        (i.exit, i.exit_piece)
    };
    match best {
        Some((b, _, _)) => if (entry && t < b) || (!entry && t > b) {
            Some((t, piece, k))
        } else {
            best
        },
        None => Some((t, piece, k)),
    }
}

/// Time of impact, normal and `Unknown` feature of `ray` on the counter-clockwise convex polygon
/// `vertices` dilated by `radius` (local frame). See the module documentation.
/// #### Panics
/// * As `super::capsule::cast_local_ray_and_get_normal_capsule`.
pub fn cast_local_ray_and_get_normal_round(
    vertices: Span<Vec2>, radius: Fixed, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<RayIntersection> {
    let d = ray.dir;
    if is_zero2(d.x, d.y) {
        return None;
    }
    let n = vertices.len();
    let mut inside = interior_contains(vertices, ray.origin);
    let mut components: Array<(Option<Interval>, u32)> = array![];
    let mut k = 0;
    while k != n {
        let a = *vertices[k];
        let b = *vertices[if k + 1 == n {
            0
        } else {
            k + 1
        }];
        let (in_disc, disc) = disc_interval(a, radius, ray, DISC_A);
        let (in_rect, rect) = rect_interval(Capsule { segment: Segment { a, b }, radius }, ray);
        inside = inside || in_disc || in_rect;
        components.append((disc, k));
        components.append((rect, k));
        k += 1;
    }
    if inside && solid {
        let (x, y) = try_normalize2(d.x, d.y).unwrap();
        return Some(
            RayIntersection {
                time_of_impact: ZERO, normal: Vec2 { x: -x, y: -y }, feature: FEATURE_UNKNOWN,
            },
        );
    }
    let entry = !inside;
    let mut best = None;
    for (interval, k) in components {
        best = better(interval, k, best, entry);
    }
    let (t, piece, k) = best?;
    let t = if t < ZERO {
        ZERO
    } else {
        t
    };
    if t > max_time_of_impact {
        return None;
    }
    let a = *vertices[k];
    let b = *vertices[if k + 1 == n {
        0
    } else {
        k + 1
    }];
    let capsule = Capsule { segment: Segment { a, b }, radius };
    Some(
        RayIntersection {
            time_of_impact: t,
            normal: piece_normal(capsule, piece, ray.point_at(t), inside),
            feature: FEATURE_UNKNOWN,
        },
    )
}

/// The counter-clockwise corners of `cuboid`.
#[inline(always)]
pub(crate) fn cuboid_vertices(cuboid: Cuboid) -> Span<Vec2> {
    let h = cuboid.half_extents;
    array![Vec2 { x: -h.x, y: -h.y }, Vec2 { x: h.x, y: -h.y }, h, Vec2 { x: -h.x, y: h.y }].span()
}

/// The counter-clockwise vertices of `triangle` (its core).
#[inline(always)]
pub(crate) fn triangle_vertices(triangle: Triangle) -> Span<Vec2> {
    let (core, _) = triangle_core(triangle);
    let [p0, p1, p2, _, _, _, _, _] = core.vertices;
    array![p0, p1, p2].span()
}

/// The live vertices of `polygon`.
pub(crate) fn polygon_vertices(polygon: ConvexPolygon) -> Span<Vec2> {
    let mut out = array![];
    let [p0, p1, p2, p3, p4, p5, p6, p7] = polygon.vertices;
    for p in [p0, p1, p2, p3, p4, p5, p6, p7].span() {
        if out.len() == polygon.count.into() {
            break;
        }
        out.append(*p);
    }
    out.span()
}

/// [`cast_local_ray_and_get_normal_round`] on a round cuboid.
pub fn cast_local_ray_and_get_normal_round_cuboid(
    cuboid: Cuboid, radius: Fixed, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<RayIntersection> {
    cast_local_ray_and_get_normal_round(
        cuboid_vertices(cuboid), radius, ray, max_time_of_impact, solid,
    )
}

/// [`cast_local_ray_and_get_normal_round`] on a round triangle.
pub fn cast_local_ray_and_get_normal_round_triangle(
    triangle: Triangle, radius: Fixed, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<RayIntersection> {
    cast_local_ray_and_get_normal_round(
        triangle_vertices(triangle), radius, ray, max_time_of_impact, solid,
    )
}

/// [`cast_local_ray_and_get_normal_round`] on a round convex polygon.
pub fn cast_local_ray_and_get_normal_round_convex_polygon(
    polygon: ConvexPolygon, radius: Fixed, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<RayIntersection> {
    cast_local_ray_and_get_normal_round(
        polygon_vertices(polygon), radius, ray, max_time_of_impact, solid,
    )
}
