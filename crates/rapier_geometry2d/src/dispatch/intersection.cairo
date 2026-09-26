//! Boolean intersection tests (Parry `query::intersection_test` and
//! `DefaultQueryDispatcher::intersection_test`), the query behind sensor pairs.
//!
//! `pos12` is the pose of shape 2 in the frame of shape 1 (`pose1.inv_mul(pose2)`, unit
//! rotation). Touching counts as intersecting (upstream's `<=` everywhere). Every pair of the
//! closed [`Shape`] enum is supported except half-space–half-space (upstream: `Unsupported`,
//! half-spaces are not support maps). Each kernel is exact up to the rounding of the pose
//! transforms: the decisions compare raw Q64.64 products (`crate::point::{dot_wide,
//! cross_wide}`, `rapier_math::math_ext::norm2`), never a rescaled square.
//!
//! Kernels, by pair (upstream's method in parentheses when it differs):
//!
//! * ball–ball: `|t|² <= (r1 + r2)²`;
//! * ball–anything: the solid projection of the ball centre is inside or within the radius; the
//!   cuboid, segment, capsule and half-space projections are analytic, the polygon's only visits
//!   the edges facing the centre (upstream: `PointQuery`, the full projection);
//! * cuboid–cuboid: face-axis SAT with absolute rotation terms, no normalisation (upstream: the
//!   two one-way `cuboid_cuboid_find_local_separating_normal_oneway`, kept as
//!   `alternatives::cuboid_cuboid_upstream_sat`);
//! * half-space–convex: the deepest point along `-n` is on the inner side;
//! * cuboid–segment, cuboid–capsule: three-axis SAT of the segment core (exact overlap test),
//!   then for a capsule the distances endpoint–box and corner–segment (upstream: GJK);
//! * segment / capsule pairs: exact crossing test by orientation signs, then (radius > 0) the
//!   distance of the closest points (upstream: GJK);
//! * convex-polygon pairs: SAT on the edge normals of both sides (a segment core contributes its
//!   two normals), then for a capsule the vertex–segment and endpoint–polygon distances
//!   (upstream: GJK).
//!
//! Candidates (`alternatives`, measured by `tests::gas_*`): deriving the answer from the contact
//! generators (`intersection_test_from_contacts`: manifold at zero prediction, any point with
//! `dist <= 0`), upstream's cuboid SAT, the capsule pair by endpoint projections, and the
//! ball–polygon test through the full polygon projection.

use core::num::traits::WideMul;
use fixed::{Fixed, FixedTrait, ONE, ZERO};
use glam::Vec2;
use rapier_math::math_ext::norm2::is_norm2_le;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::closest_points::closest_points_segment_segment;
use crate::point::{cross_wide, dot_wide, project_local_point_segment};
use crate::shape::{
    Ball, Capsule, ConvexPolygon, ConvexPolygonTrait, Cuboid, HalfSpace, Segment, Shape,
};

#[cfg(test)]
pub mod alternatives;
#[cfg(test)]
mod tests;

/// `x` as a raw Q64.64, the scale of the wide products.
#[inline(always)]
fn widen(x: Fixed) -> i128 {
    x.raw.wide_mul(ONE.raw)
}

/// Whether two shapes intersect (touching included), Parry
/// `DefaultQueryDispatcher::intersection_test`.
///
/// `pos12` is the pose of `shape2` in the local frame of `shape1`. Returns `None` for an
/// unsupported pair (half-space–half-space; upstream `Err(Unsupported)`), `Some(intersecting)`
/// otherwise.
/// #### Panics
/// * `'Fixed: overflow'` / integer overflow when a transformed coordinate or a wide product
///   leaves its range (coordinates bounded by 2^20 are safe).
/// #### Deviations
/// * Analytic or SAT kernels instead of GJK for the pairs upstream sends to GJK; they answer
///   exactly where GJK answers within its tolerance (a touching pair is always intersecting here).
/// * `#[inline(always)]`: inlined into a loop body, each arm is charged only when taken.
#[inline(always)]
pub fn intersection_test(pos12: Pose2, shape1: Shape, shape2: Shape) -> Option<bool> {
    match (shape1, shape2) {
        (Shape::Ball(b1), Shape::Ball(b2)) => Some(ball_ball(pos12.translation, b1, b2)),
        (Shape::Cuboid(c1), Shape::Cuboid(c2)) => Some(cuboid_cuboid(pos12, c1, c2)),
        (
            Shape::Ball(b1), _,
        ) => {
            let center = pos12.inverse_transform_point(Default::default());
            point_query_ball(shape2, center, b1.radius)
        },
        (_, Shape::Ball(b2)) => point_query_ball(shape1, pos12.translation, b2.radius),
        (Shape::HalfSpace(h1), _) => halfspace_convex(pos12, h1, shape2),
        (_, Shape::HalfSpace(h2)) => halfspace_convex(pos12.inverse(), h2, shape1),
        (Shape::Cuboid(c1), Shape::Capsule(c2)) => Some(cuboid_capsule(pos12, c1, c2)),
        (Shape::Capsule(c1), Shape::Cuboid(c2)) => Some(cuboid_capsule(pos12.inverse(), c2, c1)),
        (
            Shape::Cuboid(c1), Shape::Segment(s2),
        ) => Some(cuboid_capsule(pos12, c1, Capsule { segment: s2, radius: ZERO })),
        (
            Shape::Segment(s1), Shape::Cuboid(c2),
        ) => Some(cuboid_capsule(pos12.inverse(), c2, Capsule { segment: s1, radius: ZERO })),
        (
            Shape::Capsule(c1), Shape::Capsule(c2),
        ) => Some(segment_segment(pos12, c1.segment, c2.segment, c1.radius + c2.radius)),
        (
            Shape::Capsule(c1), Shape::Segment(s2),
        ) => Some(segment_segment(pos12, c1.segment, s2, c1.radius)),
        (
            Shape::Segment(s1), Shape::Capsule(c2),
        ) => Some(segment_segment(pos12, s1, c2.segment, c2.radius)),
        (Shape::Segment(s1), Shape::Segment(s2)) => Some(segment_segment(pos12, s1, s2, ZERO)),
        (Shape::ConvexPolygon(p1), _) => Some(polygon_shape(pos12, p1.unbox(), shape2)),
        (_, Shape::ConvexPolygon(p2)) => Some(polygon_shape(pos12.inverse(), p2.unbox(), shape1)),
    }
}

/// Ball–ball: `|center12| <= r1 + r2`, squares compared wide (Parry
/// `intersection_test_ball_ball`). Exact.
pub fn ball_ball(center12: Vec2, ball1: Ball, ball2: Ball) -> bool {
    is_norm2_le(center12.x, center12.y, ball1.radius + ball2.radius)
}

/// Whether a ball of `radius` centred at `center` (in the frame of `shape`) touches `shape`
/// (Parry `intersection_test_point_query_ball`: solid projection inside or within the radius).
/// `None` never happens: every shape supports point queries; the `Option` mirrors the table.
#[inline(always)]
pub fn point_query_ball(shape: Shape, center: Vec2, radius: Fixed) -> Option<bool> {
    Some(
        match shape {
            Shape::Ball(b) => is_norm2_le(center.x, center.y, b.radius + radius),
            Shape::Cuboid(c) => point_cuboid(c, center, radius),
            Shape::Capsule(c) => point_segment(c.segment, center, c.radius + radius),
            Shape::Segment(s) => point_segment(s, center, radius),
            Shape::HalfSpace(h) => point_halfspace(h, center, radius),
            Shape::ConvexPolygon(p) => point_polygon(p.unbox(), center, radius),
        },
    )
}

/// `pt` within `radius` of the solid cuboid: distance to the clamped point, compared wide.
pub fn point_cuboid(cuboid: Cuboid, pt: Vec2, radius: Fixed) -> bool {
    let h = cuboid.half_extents;
    let qx = pt.x.clamp(-h.x, h.x);
    let qy = pt.y.clamp(-h.y, h.y);
    is_norm2_le(pt.x - qx, pt.y - qy, radius)
}

/// `pt` within `radius` of `segment` (projection of `crate::point::segment`, compared wide).
pub fn point_segment(segment: Segment, pt: Vec2, radius: Fixed) -> bool {
    let proj = project_local_point_segment(segment, pt, true);
    is_norm2_le(pt.x - proj.point.x, pt.y - proj.point.y, radius)
}

/// `pt` within `radius` of the half-space: `n . pt <= radius`, exact.
pub fn point_halfspace(halfspace: HalfSpace, pt: Vec2, radius: Fixed) -> bool {
    let n = halfspace.normal;
    dot_wide(n.x, n.y, pt.x, pt.y) <= widen(radius)
}

/// `pt` within `radius` of the solid polygon: inside when no edge has `pt` strictly on its outer
/// side (exact cross products); otherwise the closest boundary point lies on an edge that faces
/// `pt` (a vertex region is outside both adjacent edges), so only those edges are projected on,
/// stopping at the first within `radius`. Same answer as the full projection
/// (`alternatives::point_polygon_projection`).
pub fn point_polygon(polygon: ConvexPolygon, pt: Vec2, radius: Fixed) -> bool {
    let mut outside = false;
    let mut hit = false;
    let mut i = 0;
    while i != polygon.count {
        let a = polygon.vertex(i);
        let b = polygon.vertex(polygon.next(i));
        let e = b - a;
        if cross_wide(e.x, e.y, pt.x - a.x, pt.y - a.y) < 0 {
            outside = true;
            if point_segment(Segment { a, b }, pt, radius) {
                hit = true;
                break;
            }
        }
        i += 1;
    }
    hit || !outside
}

/// Cuboid–cuboid: SAT on the four face axes. Cuboid 2's extent along an axis of cuboid 1 is
/// `|re| hx + |im| hy` (and symmetrically), one floored `dot2` each; no normalisation.
pub fn cuboid_cuboid(pos12: Pose2, cuboid1: Cuboid, cuboid2: Cuboid) -> bool {
    let h1 = cuboid1.half_extents;
    let h2 = cuboid2.half_extents;
    let re = pos12.rotation.re;
    let im = pos12.rotation.im;
    let are = re.abs();
    let aim = im.abs();
    let t = pos12.translation;
    if t.x.abs() > h1.x + fixed::wide::dot2(are, h2.x, aim, h2.y) {
        return false;
    }
    if t.y.abs() > h1.y + fixed::wide::dot2(aim, h2.x, are, h2.y) {
        return false;
    }
    // `t` on cuboid 2's axes: the inverse rotation of `t`.
    let tu = fixed::wide::dot2(t.x, re, t.y, im);
    let tv = fixed::wide::dot2(t.y, re, -t.x, im);
    if tu.abs() > h2.x + fixed::wide::dot2(are, h1.x, aim, h1.y) {
        return false;
    }
    tv.abs() <= h2.y + fixed::wide::dot2(aim, h1.x, are, h1.y)
}

/// Whether the segment `a b` (cuboid frame) meets the solid cuboid: the two face axes, then the
/// segment normal `(-d.y, d.x)` with `|n . a| <= hx |n.x| + hy |n.y|`, wide. Exact.
pub fn segment_meets_cuboid(cuboid: Cuboid, a: Vec2, b: Vec2) -> bool {
    let h = cuboid.half_extents;
    if a.x.max(b.x) < -h.x || a.x.min(b.x) > h.x || a.y.max(b.y) < -h.y || a.y.min(b.y) > h.y {
        return false;
    }
    let d = b - a;
    let na = cross_wide(d.x, d.y, a.x, a.y);
    let extent = d.y.abs().raw.wide_mul(h.x.raw) + d.x.abs().raw.wide_mul(h.y.raw);
    na <= extent && -na <= extent
}

/// Cuboid–capsule (a segment is a capsule of radius zero): the segment core meets the cuboid,
/// or (radius > 0) an endpoint is within the radius of the box or a corner within the radius of
/// the segment (the distance of two disjoint convex polygons is reached at a vertex).
pub fn cuboid_capsule(pos12: Pose2, cuboid1: Cuboid, capsule2: Capsule) -> bool {
    let a = pos12.transform_point(capsule2.segment.a);
    let b = pos12.transform_point(capsule2.segment.b);
    if segment_meets_cuboid(cuboid1, a, b) {
        return true;
    }
    let r = capsule2.radius;
    if r.raw == 0 {
        return false;
    }
    if point_cuboid(cuboid1, a, r) || point_cuboid(cuboid1, b, r) {
        return true;
    }
    let h = cuboid1.half_extents;
    let segment = Segment { a, b };
    point_segment(segment, h, r)
        || point_segment(segment, Vec2 { x: -h.x, y: h.y }, r)
        || point_segment(segment, -h, r)
        || point_segment(segment, Vec2 { x: h.x, y: -h.y }, r)
}

/// Sign of a wide value: -1, 0 or 1.
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

/// Whether the segments `a1 b1` and `a2 b2` share a point: orientation signs of exact wide
/// cross products, and the interval overlap along the common line for collinear segments.
pub fn segments_cross(a1: Vec2, b1: Vec2, a2: Vec2, b2: Vec2) -> bool {
    let d1 = b1 - a1;
    let d2 = b2 - a2;
    let o1 = sign(cross_wide(d1.x, d1.y, a2.x - a1.x, a2.y - a1.y));
    let o2 = sign(cross_wide(d1.x, d1.y, b2.x - a1.x, b2.y - a1.y));
    if o1 * o2 > 0 {
        return false;
    }
    let o3 = sign(cross_wide(d2.x, d2.y, a1.x - a2.x, a1.y - a2.y));
    let o4 = sign(cross_wide(d2.x, d2.y, b1.x - a2.x, b1.y - a2.y));
    if o3 * o4 > 0 {
        return false;
    }
    if o1 != 0 || o2 != 0 || o3 != 0 || o4 != 0 {
        return true;
    }
    // Collinear (or degenerate): the bounding intervals overlap on both axes.
    a1.x.max(b1.x) >= a2.x.min(b2.x)
        && a2.x.max(b2.x) >= a1.x.min(b1.x)
        && a1.y.max(b1.y) >= a2.y.min(b2.y)
        && a2.y.max(b2.y) >= a1.y.min(b1.y)
}

/// Segment / capsule pairs: the cores cross (exact), or (radius > 0) their closest points are
/// within `radius` (`crate::closest_points`, compared wide).
pub fn segment_segment(pos12: Pose2, segment1: Segment, segment2: Segment, radius: Fixed) -> bool {
    let a2 = pos12.transform_point(segment2.a);
    let b2 = pos12.transform_point(segment2.b);
    if segments_cross(segment1.a, segment1.b, a2, b2) {
        return true;
    }
    if radius.raw == 0 {
        return false;
    }
    let (p1, p2) = closest_points_segment_segment(segment1, Segment { a: a2, b: b2 });
    is_norm2_le(p1.x - p2.x, p1.y - p2.y, radius)
}

/// Half-space–convex (shape in the half-space frame at `pos12`): the deepest point of the shape
/// along `-n` satisfies `n . p <= 0` (Parry `intersection_test_halfspace_support_map`). `None`
/// for a half-space.
pub fn halfspace_convex(pos12: Pose2, halfspace: HalfSpace, shape: Shape) -> Option<bool> {
    let n = halfspace.normal;
    match shape {
        Shape::Ball(b) => Some(point_halfspace(halfspace, pos12.translation, b.radius)),
        Shape::Cuboid(c) => Some(halfspace_cuboid(pos12, n, c)),
        Shape::Capsule(c) => Some(halfspace_segment(pos12, n, c.segment, c.radius)),
        Shape::Segment(s) => Some(halfspace_segment(pos12, n, s, ZERO)),
        Shape::HalfSpace(_) => None,
        Shape::ConvexPolygon(p) => Some(halfspace_polygon(pos12, n, p.unbox())),
    }
}

/// Half-space–cuboid: `n . t <= |m.x| hx + |m.y| hy` with `m = R^T n`, wide.
pub fn halfspace_cuboid(pos12: Pose2, n: Vec2, cuboid: Cuboid) -> bool {
    let m = pos12.rotation.inverse_rotate(n);
    let t = pos12.translation;
    let h = cuboid.half_extents;
    let extent = m.x.abs().raw.wide_mul(h.x.raw) + m.y.abs().raw.wide_mul(h.y.raw);
    dot_wide(n.x, n.y, t.x, t.y) <= extent
}

/// Half-space–segment core of radius `radius`: the lower endpoint is within the radius.
pub fn halfspace_segment(pos12: Pose2, n: Vec2, segment: Segment, radius: Fixed) -> bool {
    let a = pos12.transform_point(segment.a);
    let b = pos12.transform_point(segment.b);
    let r = widen(radius);
    dot_wide(n.x, n.y, a.x, a.y) <= r || dot_wide(n.x, n.y, b.x, b.y) <= r
}

/// Half-space–polygon: `min_j m . v_j + n . t <= 0` with `m = R^T n`.
pub fn halfspace_polygon(pos12: Pose2, n: Vec2, polygon: ConvexPolygon) -> bool {
    let m = pos12.rotation.inverse_rotate(n);
    let t = pos12.translation;
    let depth = -dot_wide(n.x, n.y, t.x, t.y);
    let mut i = 0;
    let mut hit = false;
    while i != polygon.count {
        let v = polygon.vertex(i);
        if dot_wide(m.x, m.y, v.x, v.y) <= depth {
            hit = true;
            break;
        }
        i += 1;
    }
    hit
}

/// A SAT axis in the frame of shape 1: a direction and the support value of its own shape along
/// it (raw Q64.64); the other shape is separated when all its points are strictly beyond it.
#[derive(Copy, Drop)]
struct Axis {
    n: Vec2,
    support: i128,
}

/// Whether one axis of `axes` separates `points` (every point strictly beyond its support).
fn separates(axes: Span<Axis>, points: Span<Vec2>) -> bool {
    let mut found = false;
    for axis in axes {
        let n = *axis.n;
        let mut beyond = true;
        for p in points {
            if dot_wide(n.x, n.y, *p.x, *p.y) <= *axis.support {
                beyond = false;
                break;
            }
        }
        if beyond {
            found = true;
            break;
        }
    }
    found
}

/// The polygon's vertices and edge-normal axes, placed at `pose` (in the frame of shape 1).
fn polygon_core(polygon: ConvexPolygon, pose: Pose2) -> (Array<Vec2>, Array<Axis>) {
    let mut points = array![];
    let mut axes = array![];
    let mut i = 0;
    while i != polygon.count {
        let v = pose.transform_point(polygon.vertex(i));
        let n = pose.rotation.rotate(polygon.normal(i));
        points.append(v);
        axes.append(Axis { n, support: dot_wide(n.x, n.y, v.x, v.y) });
        i += 1;
    }
    (points, axes)
}

/// The cuboid's corners and face axes, placed at `pose`.
fn cuboid_core(cuboid: Cuboid, pose: Pose2) -> (Array<Vec2>, Array<Axis>) {
    let h = cuboid.half_extents;
    let x = pose.rotation.rotate(Vec2 { x: ONE, y: ZERO });
    let y = pose.rotation.rotate(Vec2 { x: ZERO, y: ONE });
    let c0 = pose.transform_point(h);
    let c1 = pose.transform_point(Vec2 { x: -h.x, y: h.y });
    let c2 = pose.transform_point(-h);
    let corners = array![c0, c1, c2, pose.transform_point(Vec2 { x: h.x, y: -h.y })];
    let axes = array![
        Axis { n: x, support: dot_wide(x.x, x.y, c0.x, c0.y) },
        Axis { n: y, support: dot_wide(y.x, y.y, c0.x, c0.y) },
        Axis { n: -x, support: dot_wide(-x.x, -x.y, c1.x, c1.y) },
        Axis { n: -y, support: dot_wide(-y.x, -y.y, c2.x, c2.y) },
    ];
    (corners, axes)
}

/// The segment's endpoints and its two normal axes (unnormalised: only signs are compared).
fn segment_core(a: Vec2, b: Vec2) -> (Array<Vec2>, Array<Axis>) {
    let d = b - a;
    let n = Vec2 { x: -d.y, y: d.x };
    let support = dot_wide(n.x, n.y, a.x, a.y);
    (array![a, b], array![Axis { n, support }, Axis { n: -n, support: -support }])
}

/// Convex polygon (frame 1) against any non-ball, non-half-space shape at `pos12`: SAT on the
/// edge normals of both sides (exact overlap test for convex polygons; the Minkowski difference
/// of a polygon and a segment only has their edge directions), then for a capsule the distances
/// endpoint–polygon and vertex–segment.
fn polygon_shape(pos12: Pose2, polygon1: ConvexPolygon, shape2: Shape) -> bool {
    let (points1, axes1) = polygon_core(polygon1, Default::default());
    let (points2, axes2, capsule) = match shape2 {
        Shape::Cuboid(c) => {
            let (p, a) = cuboid_core(c, pos12);
            (p, a, None)
        },
        Shape::ConvexPolygon(p) => {
            let (p, a) = polygon_core(p.unbox(), pos12);
            (p, a, None)
        },
        Shape::Segment(s) => {
            let (p, a) = segment_core(pos12.transform_point(s.a), pos12.transform_point(s.b));
            (p, a, None)
        },
        Shape::Capsule(c) => {
            let segment = Segment {
                a: pos12.transform_point(c.segment.a), b: pos12.transform_point(c.segment.b),
            };
            let (p, a) = segment_core(segment.a, segment.b);
            (p, a, Some((segment, c.radius)))
        },
        // Handled by the table before this kernel.
        _ => (array![], array![], None),
    };
    if !separates(axes1.span(), points2.span()) && !separates(axes2.span(), points1.span()) {
        return true;
    }
    let Some((segment, r)) = capsule else {
        return false;
    };
    if point_polygon(polygon1, segment.a, r) || point_polygon(polygon1, segment.b, r) {
        return true;
    }
    let mut hit = false;
    for v in points1 {
        if point_segment(segment, v, r) {
            hit = true;
            break;
        }
    }
    hit
}
