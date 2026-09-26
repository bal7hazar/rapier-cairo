//! Point projection on a [`Triangle`] (Parry `query/point/point_triangle.rs`, 2D branch).
//!
//! Upstream's Voronoi walk, every region test on exact wide dot and cross products: vertex
//! regions `a`, `b`, `c`, then the edge regions `ab`, `ac`, `bc` (the sign tests `n * cross(...)`
//! are sign products, exact), then the interior. Projections on an edge are the exact segment
//! projections of [`crate::point::segment`] (one correctly rounded ratio, one floor per
//! coordinate), relabelled with the triangle's location.
//!
//! * `is_inside` of an outside projection is `pt == proj`, as upstream.
//! * A degenerate triangle (`cross(ab, ac) == 0` exactly, where upstream tests its rounded
//!   `va + vb + vc == 0`) projects on its longest edge.
//! * A hollow projection of an interior point goes to the edge whose line is closest: the line
//!   distances `|cross(e, p - o)| / |e|` truncated to `Fixed` (upstream compares rounded squared
//!   distances), upstream's strict comparison order `ab`, `bc`, `ac`.
//! * Orientation-independent, as upstream.

use fixed::wide::{distance2, norm2};
use fixed::{Fixed, ZERO};
use glam::Vec2;
use rapier_math::math_ext::norm2::norm2_sq_wide;
use crate::feature_id::{FeatureId, FeatureIdTrait};
use crate::shape::{Segment, Triangle, TrianglePointLocation};
use super::{
    PointProjection, SegmentPointLocation, cross_wide, dot_wide,
    project_local_point_and_get_location_segment,
};

/// `-1`, `0`, `1`.
#[inline(always)]
fn sign(x: i128) -> i8 {
    if x > 0 {
        1
    } else if x < 0 {
        -1
    } else {
        0
    }
}

#[inline(always)]
fn result(pt: Vec2, proj: Vec2) -> PointProjection {
    PointProjection { is_inside: pt == proj, point: proj }
}

/// The projection of `pt` on the edge `p`–`q` (edge id `eid`), with upstream's location.
fn on_edge(p: Vec2, q: Vec2, eid: u32, pt: Vec2) -> (PointProjection, TrianglePointLocation) {
    let (proj, loc) = project_local_point_and_get_location_segment(
        Segment { a: p, b: q }, pt, false,
    );
    let uv = match loc {
        SegmentPointLocation::OnVertex(i) => if i == 0 {
            (fixed::ONE, ZERO)
        } else {
            (ZERO, fixed::ONE)
        },
        SegmentPointLocation::OnEdge(uv) => uv,
    };
    (result(pt, proj.point), TrianglePointLocation::OnEdge((eid, uv)))
}

/// The distance from `pt` to the line of `o`–`o + e`, truncated (`|e| > 0`).
#[inline(always)]
fn line_distance(o: Vec2, e: Vec2, pt: Vec2) -> Fixed {
    let d = pt - o;
    let c = cross_wide(e.x, e.y, d.x, d.y);
    let c = if c < 0 {
        -c
    } else {
        c
    };
    let len: i128 = norm2(e.x, e.y).raw.into();
    if len == 0 {
        return fixed::MAX;
    }
    Fixed { raw: (c / len).try_into().unwrap() }
}

/// Projects `pt` on `triangle` and locates the projection (upstream
/// `project_local_point_and_get_location`): on the filled triangle when `solid` (an interior
/// point answers itself with `OnSolid`), on its boundary otherwise.
/// #### Panics
/// * The overflow panics of the wide products (coordinates below `2^31`, differences
///   representable).
pub fn project_local_point_and_get_location_triangle(
    triangle: Triangle, pt: Vec2, solid: bool,
) -> (PointProjection, TrianglePointLocation) {
    let (a, b, c) = (triangle.a, triangle.b, triangle.c);
    let ab = b - a;
    let ac = c - a;
    let ap = pt - a;
    let ab_ap = dot_wide(ab.x, ab.y, ap.x, ap.y);
    let ac_ap = dot_wide(ac.x, ac.y, ap.x, ap.y);
    if ab_ap <= 0 && ac_ap <= 0 {
        return (result(pt, a), TrianglePointLocation::OnVertex(0));
    }
    let bp = pt - b;
    let ab_bp = dot_wide(ab.x, ab.y, bp.x, bp.y);
    let ac_bp = dot_wide(ac.x, ac.y, bp.x, bp.y);
    if ab_bp >= 0 && ac_bp <= ab_bp {
        return (result(pt, b), TrianglePointLocation::OnVertex(1));
    }
    let cp = pt - c;
    let ab_cp = dot_wide(ab.x, ab.y, cp.x, cp.y);
    let ac_cp = dot_wide(ac.x, ac.y, cp.x, cp.y);
    if ac_cp >= 0 && ab_cp <= ac_cp {
        return (result(pt, c), TrianglePointLocation::OnVertex(2));
    }
    let bc = c - b;
    let n = sign(cross_wide(ab.x, ab.y, ac.x, ac.y));
    // vc = n cross(ab, ap), vb = -n cross(ac, cp), va = n cross(bc, bp): signs only.
    let vc = n * sign(cross_wide(ab.x, ab.y, ap.x, ap.y));
    if vc < 0 && ab_ap >= 0 && ab_bp <= 0 {
        return on_edge(a, b, 0, pt);
    }
    let vb = -n * sign(cross_wide(ac.x, ac.y, cp.x, cp.y));
    if vb < 0 && ac_ap >= 0 && ac_cp <= 0 {
        return on_edge(a, c, 2, pt);
    }
    let va = n * sign(cross_wide(bc.x, bc.y, bp.x, bp.y));
    if va < 0 && ac_bp - ab_bp >= 0 && ab_cp - ac_cp >= 0 {
        return on_edge(b, c, 1, pt);
    }
    if n == 0 {
        // Degenerate: the longest edge (upstream's `>=` order: ab, ac, bc).
        let sq_ab = norm2_sq_wide(ab.x, ab.y);
        let sq_ac = norm2_sq_wide(ac.x, ac.y);
        let sq_bc = norm2_sq_wide(bc.x, bc.y);
        let (p, q, eid, v0, v1) = if sq_ab >= sq_ac && sq_ab >= sq_bc {
            (a, b, 0, 0, 1)
        } else if sq_ac >= sq_bc {
            (a, c, 2, 0, 2)
        } else {
            (b, c, 1, 1, 2)
        };
        let (proj, loc) = project_local_point_and_get_location_segment(
            Segment { a: p, b: q }, pt, solid,
        );
        let loc = match loc {
            SegmentPointLocation::OnVertex(i) => TrianglePointLocation::OnVertex(
                if i == 0 {
                    v0
                } else {
                    v1
                },
            ),
            SegmentPointLocation::OnEdge(uv) => TrianglePointLocation::OnEdge((eid, uv)),
        };
        return (proj, loc);
    }
    if solid {
        return (PointProjection { is_inside: true, point: pt }, TrianglePointLocation::OnSolid);
    }
    // Inside, hollow: the closest edge line.
    let d_ab = line_distance(a, ab, pt);
    let d_ac = line_distance(a, ac, pt);
    let d_bc = line_distance(b, bc, pt);
    let (p, q, eid) = if d_ab < d_ac {
        if d_ab < d_bc {
            (a, b, 0)
        } else {
            (b, c, 1)
        }
    } else if d_ac < d_bc {
        (a, c, 2)
    } else {
        (b, c, 1)
    };
    let (proj, loc) = on_edge(p, q, eid, pt);
    (PointProjection { is_inside: true, point: proj.point }, loc)
}

/// Projects `pt` on the filled triangle (`solid`) or its boundary (upstream
/// `project_local_point`). See [`project_local_point_and_get_location_triangle`].
pub fn project_local_point_triangle(triangle: Triangle, pt: Vec2, solid: bool) -> PointProjection {
    let (proj, _) = project_local_point_and_get_location_triangle(triangle, pt, solid);
    proj
}

/// Projects `pt` on the boundary and names the feature (upstream 2D
/// `project_local_point_and_get_feature`): `Vertex(i)` on a vertex, `Face(i)` on edge `i` of the
/// location's numbering (`0 = ab`, `1 = bc`, `2 = ac`).
pub fn project_local_point_and_get_feature_triangle(
    triangle: Triangle, pt: Vec2,
) -> (PointProjection, FeatureId) {
    let (proj, loc) = project_local_point_and_get_location_triangle(triangle, pt, false);
    let feature = match loc {
        TrianglePointLocation::OnVertex(i) => FeatureIdTrait::vertex(i),
        TrianglePointLocation::OnEdge((i, _)) => FeatureIdTrait::face(i),
        TrianglePointLocation::OnFace((i, _)) => FeatureIdTrait::face(i),
        TrianglePointLocation::OnSolid => FeatureIdTrait::face(0),
    };
    (proj, feature)
}

/// Signed distance from `pt` to the triangle (negative inside when not `solid`, zero inside a
/// solid one); the floored length of `pt - proj`.
pub fn distance_to_local_point_triangle(triangle: Triangle, pt: Vec2, solid: bool) -> Fixed {
    let proj = project_local_point_triangle(triangle, pt, solid);
    let dist = distance2(pt.x, pt.y, proj.point.x, proj.point.y);
    if proj.is_inside && !solid {
        -dist
    } else {
        dist
    }
}

/// Whether `pt` is in the filled triangle, boundary included (upstream default: the solid
/// projection's `is_inside`).
pub fn contains_local_point_triangle(triangle: Triangle, pt: Vec2) -> bool {
    project_local_point_triangle(triangle, pt, true).is_inside
}
