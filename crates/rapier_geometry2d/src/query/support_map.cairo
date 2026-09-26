//! Support-map pairs of the shape-pair queries: every pair upstream sends to GJK (distance,
//! closest points) and GJK + EPA (penetrating contact), answered by one exact analytic kernel
//! (ADR 0001 entries 14, 17, 24: analytic kernels instead of GJK / EPA).
//!
//! # The kernel
//!
//! Every shape of the closed set but the half-space is a convex polygonal *core* dilated by a
//! radius: a ball is a point, a capsule a segment, a segment itself with radius 0, a cuboid or a
//! convex polygon its vertices. The distance between two dilated cores is the distance between
//! the cores minus the two radii, and the penetration depth of two overlapping cores grows by
//! the radii too (dilating a convex set moves every supporting line by the radius), so:
//!
//! 1. **SAT** over the unit edge normals of both cores (a segment core contributes its two
//!    normals and its two directions, so that collinear segments separate): the axis of largest
//!    separation `sep = min(u . core2) - max(u . core1)`.
//! 2. `sep > 0`: the cores are disjoint and their closest pair is a vertex of one against an edge
//!    of the other: every vertex is projected on every edge of the other core (exact segment
//!    projections, squared distances compared wide), the first minimum wins.
//! 3. `sep <= 0`: the cores overlap; `sep` is minus their penetration depth and the SAT axis the
//!    normal, the witness being the deepest vertex along it and its image on the opposite face.
//! 4. The radii: `dist = core distance - r1 - r2`, each point pushed along the normal by its
//!    radius.
//!
//! The answer is exact up to the rounding of the pose transform, of one square root and of the
//! unit axes, where GJK stops within its tolerance and EPA polygonises a rounded shape. Ties
//! (parallel faces, symmetric penetrations) pick the first candidate in the order above, where
//! GJK / EPA pick whatever their simplex converges to: the distance and the normal agree, the
//! witness points are then one member of the same family.

use fixed::wide::dot2;
use fixed::{Fixed, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::math_ext::norm2::{norm2_sq_wide, sq_wide};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::closest_points::closest_points_segment_segment;
use crate::point::{cross_wide, project_local_point_segment};
use crate::shape::{
    BallTrait, CapsuleTrait, ConvexPolygon, ConvexPolygonTrait, CuboidTrait, Segment, SegmentTrait,
    Shape,
};
use super::{ClosestPoints, Contact, ContactTrait, X, normalize_and_length};

/// The polygonal core of a shape and its radius, in one frame.
#[derive(Copy, Drop, Debug)]
pub struct Core {
    /// One vertex (a point), two (a segment) or the vertices of a convex polygon in order.
    pub vertices: Span<Vec2>,
    /// Unit SAT axes: the outward edge normals (a segment also gives its two directions).
    pub axes: Span<Vec2>,
    /// The face supporting each axis: its edge (the whole segment for a segment's normals, the
    /// end point for its directions).
    pub faces: Span<Segment>,
    /// The dilation radius.
    pub radius: Fixed,
}

/// The exact answer of a support-map pair, in the frame of shape 1.
#[derive(Copy, Drop, Debug, PartialEq)]
pub struct Witness {
    /// The point of shape 1's surface.
    pub point1: Vec2,
    /// The point of shape 2's surface, still in the frame of shape 1.
    pub point2: Vec2,
    /// Unit normal from shape 1 towards shape 2.
    pub normal1: Vec2,
    /// Signed distance, negative when penetrating: `point2 - point1 = dist * normal1`.
    pub dist: Fixed,
}

/// The support point of `shape` along the unit direction `dir`, in its local frame.
/// #### Panics
/// * `'Query: not a support map'` for a half-space.
pub fn local_support_point_toward(shape: Shape, dir: Vec2) -> Vec2 {
    match shape {
        Shape::Ball(s) => s.local_support_point_toward(dir),
        Shape::Cuboid(s) => s.local_support_point(dir),
        Shape::Capsule(s) => s.local_support_point_toward(dir),
        Shape::Segment(s) => s.local_support_point(dir),
        Shape::HalfSpace(_) => core::panic_with_felt252(super::errors::NOT_SUPPORT_MAP),
        Shape::ConvexPolygon(s) => s.unbox().local_support_point(dir),
    }
}

/// The core of a segment dilated by `radius`.
fn segment_core(segment: Segment, radius: Fixed) -> Core {
    let (a, b) = (segment.a, segment.b);
    let (axes, faces) = match segment.direction() {
        Some(d) => (
            array![Vec2 { x: d.y, y: -d.x }, Vec2 { x: -d.y, y: d.x }, d, -d].span(),
            array![segment, segment, Segment { a: b, b }, Segment { a, b: a }].span(),
        ),
        None => (array![].span(), array![].span()),
    };
    Core { vertices: array![a, b].span(), axes, faces, radius }
}

/// The core of a convex polygon.
fn polygon_core(polygon: ConvexPolygon) -> Core {
    let mut vertices = array![];
    let mut axes = array![];
    let mut faces = array![];
    let mut i = 0;
    while i != polygon.count {
        vertices.append(polygon.vertex(i));
        axes.append(polygon.normal(i));
        faces.append(Segment { a: polygon.vertex(i), b: polygon.vertex(polygon.next(i)) });
        i += 1;
    }
    Core { vertices: vertices.span(), axes: axes.span(), faces: faces.span(), radius: ZERO }
}

/// The core of `shape` in its local frame.
/// #### Panics
/// * `'Query: not a support map'` for a half-space.
pub fn local_core(shape: Shape) -> Core {
    match shape {
        Shape::Ball(s) => Core {
            vertices: array![Vec2 { x: ZERO, y: ZERO }].span(),
            axes: array![].span(),
            faces: array![].span(),
            radius: s.radius,
        },
        Shape::Cuboid(s) => {
            let h = s.half_extents;
            let one = fixed::ONE;
            let (v0, v1, v3) = (
                Vec2 { x: -h.x, y: -h.y }, Vec2 { x: h.x, y: -h.y }, Vec2 { x: -h.x, y: h.y },
            );
            Core {
                vertices: array![v0, v1, h, v3].span(),
                faces: array![
                    Segment { a: v0, b: v1 }, Segment { a: v1, b: h }, Segment { a: h, b: v3 },
                    Segment { a: v3, b: v0 },
                ]
                    .span(),
                axes: array![
                    Vec2 { x: ZERO, y: -one }, Vec2 { x: one, y: ZERO }, Vec2 { x: ZERO, y: one },
                    Vec2 { x: -one, y: ZERO },
                ]
                    .span(),
                radius: ZERO,
            }
        },
        Shape::Capsule(s) => segment_core(s.segment, s.radius),
        Shape::Segment(s) => segment_core(s, ZERO),
        Shape::HalfSpace(_) => core::panic_with_felt252(super::errors::NOT_SUPPORT_MAP),
        Shape::ConvexPolygon(s) => polygon_core(s.unbox()),
    }
}

/// `core` moved by `pose`.
pub fn transformed(core: Core, pose: Pose2) -> Core {
    let mut vertices = array![];
    for v in core.vertices {
        vertices.append(pose.transform_point(*v));
    }
    let mut axes = array![];
    for u in core.axes {
        axes.append(pose.rotation.rotate(*u));
    }
    let mut faces = array![];
    for f in core.faces {
        faces.append((*f).transformed(pose));
    }
    Core { vertices: vertices.span(), axes: axes.span(), faces: faces.span(), radius: core.radius }
}

#[inline(always)]
fn dot(a: Vec2, b: Vec2) -> Fixed {
    dot2(a.x, b.x, a.y, b.y)
}

/// `(index, value)` of the largest `u . v`, first wins ties.
fn max_dot(vertices: Span<Vec2>, u: Vec2) -> (u32, Fixed) {
    let mut best = dot(*vertices[0], u);
    let mut index = 0;
    let mut i = 1;
    let n = vertices.len();
    while i != n {
        let d = dot(*vertices[i], u);
        if d > best {
            best = d;
            index = i;
        }
        i += 1;
    }
    (index, best)
}

/// `(index, value)` of the smallest `u . v`, first wins ties.
fn min_dot(vertices: Span<Vec2>, u: Vec2) -> (u32, Fixed) {
    let (index, value) = max_dot(vertices, -u);
    (index, -value)
}

/// `min(u . core2) - max(u . core1)`.
#[inline(always)]
fn separation(core1: Core, core2: Core, u: Vec2) -> Fixed {
    let (_, max1) = max_dot(core1.vertices, u);
    let (_, min2) = min_dot(core2.vertices, u);
    min2 - max1
}

/// The axis of largest separation (core1's axes first, first wins ties), `None` when neither
/// core has an axis (two points).
#[derive(Copy, Drop, Debug, PartialEq)]
pub struct SatAxis {
    /// `min(u . core2) - max(u . core1)`.
    pub separation: Fixed,
    /// Unit axis, from core1 towards core2.
    pub axis: Vec2,
    /// The face supporting the axis, on core1 when `from1`, on core2 otherwise.
    pub face: Segment,
    pub from1: bool,
}

///
/// Stops at the first separating axis (`separation > 0`): the disjoint branch only needs one.
pub fn sat(core1: Core, core2: Core) -> Option<SatAxis> {
    let mut best: Option<SatAxis> = None;
    let mut faces = core1.faces;
    for u in core1.axes {
        let face = *faces.pop_front().unwrap();
        let s = separation(core1, core2, *u);
        best = better(best, SatAxis { separation: s, axis: *u, face, from1: true });
        if s > ZERO {
            return best;
        }
    }
    let mut faces = core2.faces;
    for m in core2.axes {
        let face = *faces.pop_front().unwrap();
        let u = -*m;
        let s = separation(core1, core2, u);
        best = better(best, SatAxis { separation: s, axis: u, face, from1: false });
        if s > ZERO {
            return best;
        }
    }
    best
}

#[inline(always)]
fn better(best: Option<SatAxis>, candidate: SatAxis) -> Option<SatAxis> {
    match best {
        Some(b) => if candidate.separation > b.separation {
            Some(candidate)
        } else {
            best
        },
        None => Some(candidate),
    }
}

/// The closest point of the boundary of the core `vertices` to `pt`, and its squared distance
/// (raw Q64.64). One vertex is a point, two a segment, more a closed counter-clockwise polygon,
/// of which only the edges `pt` sees from outside or on their line (exact orientation test) are
/// projected: the closest boundary point of an outside point is on one of them. A point that sees
/// no edge (inside, which the disjoint branch only meets through rounding) projects on them all.
pub fn closest_on(vertices: Span<Vec2>, pt: Vec2) -> (Vec2, i128) {
    let n = vertices.len();
    if n == 1 {
        let v = *vertices[0];
        let d = v - pt;
        return (v, norm2_sq_wide(d.x, d.y));
    }
    if let Some(answer) = closest_on_edges(vertices, pt, n != 2) {
        return answer;
    }
    closest_on_edges(vertices, pt, false).unwrap()
}

/// The closest projection of `pt` on the edges of `vertices` (one edge for two vertices), only
/// on the edges facing `pt` when `visible_only`; `None` when no edge qualifies.
fn closest_on_edges(vertices: Span<Vec2>, pt: Vec2, visible_only: bool) -> Option<(Vec2, i128)> {
    let n = vertices.len();
    let edges = if n == 2 {
        1
    } else {
        n
    };
    let mut answer: Option<(Vec2, i128)> = None;
    let mut i = 0;
    while i != edges {
        let next = if i + 1 == n {
            0
        } else {
            i + 1
        };
        let (a, b) = (*vertices[i], *vertices[next]);
        let (e, d) = (b - a, pt - a);
        if !visible_only || cross_wide(e.x, e.y, d.x, d.y) <= 0 {
            let q = project_local_point_segment(Segment { a, b }, pt, true).point;
            let dq = q - pt;
            let sq = norm2_sq_wide(dq.x, dq.y);
            answer = match answer {
                Some((_, best)) => if sq < best {
                    Some((q, sq))
                } else {
                    answer
                },
                None => Some((q, sq)),
            };
        }
        i += 1;
    }
    answer
}

/// Two ulps: the rounding slack of a separation read off two rounded dot products.
const SLACK: Fixed = Fixed { raw: 2 };

/// `true` when a vertex at separation `lower` from the other core (a lower bound of its
/// distance, less the rounding slack) cannot beat the squared distance `best`.
#[inline(always)]
fn pruned(lower: Fixed, best: i128) -> bool {
    let lower = lower - SLACK;
    lower > ZERO && sq_wide(lower) >= best
}

/// The closest pair of two disjoint cores `(on core1, on core2)`: every vertex of core2 against
/// core1's edges, then every vertex of core1 against core2's; the first minimum wins.
///
/// `u` is a separating axis (from core1 towards core2): a vertex whose separation along `u` is
/// already at least the best distance found cannot be closer, and is skipped without projecting
/// it (`alternatives::closest_pair_exhaustive` projects them all; same answer).
pub fn closest_pair(core1: Core, core2: Core, u: Vec2) -> (Vec2, Vec2) {
    let (_, max1) = max_dot(core1.vertices, u);
    let (_, min2) = min_dot(core2.vertices, u);
    let mut best: i128 = 0;
    let mut pair = (*core1.vertices[0], *core2.vertices[0]);
    let mut first = true;
    for v2 in core2.vertices {
        if first || !pruned(dot(*v2, u) - max1, best) {
            let (q, sq) = closest_on(core1.vertices, *v2);
            if first || sq < best {
                best = sq;
                pair = (q, *v2);
                first = false;
            }
        }
    }
    for v1 in core1.vertices {
        if !pruned(min2 - dot(*v1, u), best) {
            let (q, sq) = closest_on(core2.vertices, *v1);
            if sq < best {
                best = sq;
                pair = (*v1, q);
            }
        }
    }
    pair
}

/// The exact witness of two cores in the same frame (see the module documentation).
pub fn core_witness(core1: Core, core2: Core) -> Witness {
    let (p1, p2, normal, core_dist) = match sat(core1, core2) {
        Some(best) => if best.separation > ZERO {
            separated(core1, core2, best.axis)
        } else {
            penetrating(core1, core2, best)
        },
        None => separated(core1, core2, X),
    };
    Witness {
        point1: p1 + normal.mul_scalar(core1.radius),
        point2: p2 - normal.mul_scalar(core2.radius),
        normal1: normal,
        dist: core_dist - core1.radius - core2.radius,
    }
}

/// The overlapping branch: the deepest vertex of the other core along the axis, moved onto the
/// supporting face and clamped into it (so that a pair of parallel or collinear features answers
/// a point of their overlap), and its image on the other side.
fn penetrating(core1: Core, core2: Core, best: SatAxis) -> (Vec2, Vec2, Vec2, Fixed) {
    let (u, sep) = (best.axis, best.separation);
    let shift = u.mul_scalar(sep);
    if best.from1 {
        let (j, _) = min_dot(core2.vertices, u);
        let p1 = clamp(best.face, *core2.vertices[j] - shift);
        (p1, p1 + shift, u, sep)
    } else {
        let (i, _) = max_dot(core1.vertices, u);
        let p2 = clamp(best.face, *core1.vertices[i] + shift);
        (p2 - shift, p2, u, sep)
    }
}

#[inline(always)]
fn clamp(face: Segment, p: Vec2) -> Vec2 {
    project_local_point_segment(face, p, true).point
}

/// The disjoint (or coincident-point) branch: the closest pair, its direction (`fallback` when
/// the pair coincides) and its length.
///
/// Two segment cores take the exact segment–segment routine of [`crate::closest_points`]
/// (`alternatives::closest_pair_exhaustive` measures the generic search on them).
fn separated(core1: Core, core2: Core, fallback: Vec2) -> (Vec2, Vec2, Vec2, Fixed) {
    let (p1, p2) = if core1.vertices.len() == 2 && core2.vertices.len() == 2 {
        closest_points_segment_segment(
            Segment { a: *core1.vertices[0], b: *core1.vertices[1] },
            Segment { a: *core2.vertices[0], b: *core2.vertices[1] },
        )
    } else {
        closest_pair(core1, core2, fallback)
    };
    let (dir, len) = normalize_and_length(p2 - p1);
    (p1, p2, dir.unwrap_or(fallback), len)
}

/// The witness of two support-map shapes, `shape2` placed at `pos12` (frame of `shape1`).
/// #### Panics
/// * `'Query: not a support map'` when either shape is a half-space; the overflow panics of the
///   transforms and of the wide products (coordinates bounded by 2^20 are safe).
pub fn witness(pos12: Pose2, shape1: Shape, shape2: Shape) -> Witness {
    core_witness(local_core(shape1), transformed(local_core(shape2), pos12))
}

/// `(core segment, radius)` of a capsule or a segment.
#[inline(always)]
fn segment_core_of(shape: Shape) -> Option<(Segment, Fixed)> {
    match shape {
        Shape::Capsule(c) => Some((c.segment, c.radius)),
        Shape::Segment(s) => Some((s, ZERO)),
        _ => None,
    }
}

/// Two segment cores without the SAT: the closest points of the segments (exact routine of
/// [`crate::closest_points`], zero apart when they cross), pushed out by the radii. Enough for
/// `distance` and `closest_points`, which never need a penetration depth.
/// `(point1, point2 in the frame of shape 1, dist)`.
fn segments_witness(
    pos12: Pose2, seg1: Segment, r1: Fixed, seg2: Segment, r2: Fixed,
) -> (Vec2, Vec2, Fixed) {
    let (p1, p2) = closest_points_segment_segment(seg1, seg2.transformed(pos12));
    let (dir, len) = normalize_and_length(p2 - p1);
    let n = dir.unwrap_or(X);
    (p1 + n.mul_scalar(r1), p2 - n.mul_scalar(r2), len - r1 - r2)
}

/// Distance between two support-map shapes, zero when they touch or overlap (upstream
/// `distance_support_map_support_map`, GJK there). Two capsules / segments take the segment
/// routine (`alternatives::distance_witness` measures the full kernel on them), the other pairs
/// the full kernel ([`witness`]).
/// #### Panics
/// * See [`witness`].
pub fn distance_support_map_support_map(pos12: Pose2, shape1: Shape, shape2: Shape) -> Fixed {
    let dist = match (segment_core_of(shape1), segment_core_of(shape2)) {
        (
            Some((s1, r1)), Some((s2, r2)),
        ) => {
            let (_, _, dist) = segments_witness(pos12, s1, r1, s2, r2);
            dist
        },
        _ => witness(pos12, shape1, shape2).dist,
    };
    if dist > ZERO {
        dist
    } else {
        ZERO
    }
}

/// Closest points of two support-map shapes within `margin` (upstream
/// `closest_points_support_map_support_map`, GJK there): `Intersecting` when they touch or
/// overlap, `WithinMargin(point1, point2)` (each in its shape's frame) up to `margin`, `Disjoint`
/// beyond.
/// #### Panics
/// * See [`witness`].
/// #### Deviations
/// * Touching shapes answer `Intersecting`; GJK answers either, within its tolerance.
pub fn closest_points_support_map_support_map(
    pos12: Pose2, shape1: Shape, shape2: Shape, margin: Fixed,
) -> ClosestPoints {
    let (point1, point2, dist) = match (segment_core_of(shape1), segment_core_of(shape2)) {
        (Some((s1, r1)), Some((s2, r2))) => segments_witness(pos12, s1, r1, s2, r2),
        _ => {
            let w = witness(pos12, shape1, shape2);
            (w.point1, w.point2, w.dist)
        },
    };
    if dist <= ZERO {
        ClosestPoints::Intersecting
    } else if dist > margin {
        ClosestPoints::Disjoint
    } else {
        ClosestPoints::WithinMargin((point1, pos12.inverse_transform_point(point2)))
    }
}

/// Contact between two support-map shapes when their signed distance is at most `prediction`
/// (upstream `contact_support_map_support_map`, GJK + EPA there).
/// #### Panics
/// * See [`witness`].
/// #### Deviations
/// * The penetration depth is exact (SAT on the cores plus the radii) where EPA approximates a
///   rounded shape by a polygon; touching shapes always answer a contact at `dist = 0`.
pub fn contact_support_map_support_map(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed,
) -> Option<Contact> {
    let w = witness(pos12, shape1, shape2);
    if w.dist > prediction {
        return None;
    }
    Some(
        ContactTrait::new(
            w.point1,
            pos12.inverse_transform_point(w.point2),
            w.normal1,
            -pos12.rotation.inverse_rotate(w.normal1),
            w.dist,
        ),
    )
}

/// Rejected candidates, kept for the `gas_*` ranking.
#[cfg(test)]
pub mod alternatives;

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::query::ClosestPoints;
    use crate::shape::{
        Capsule, CapsuleTrait, ConvexPolygonTrait, CuboidTrait, HalfSpaceTrait, SegmentTrait, Shape,
    };
    use super::alternatives::{distance_exhaustive, distance_witness};
    use super::{
        closest_points_support_map_support_map, contact_support_map_support_map,
        distance_support_map_support_map, witness,
    };

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn int(x: i32) -> Fixed {
        FixedTrait::from_int(x)
    }

    fn at(x: Fixed, y: Fixed) -> Pose2 {
        Pose2Trait::new(v(x, y), Rot2 { re: ONE, im: ZERO })
    }

    fn quarter_at(x: Fixed, y: Fixed) -> Pose2 {
        Pose2Trait::new(v(x, y), Rot2 { re: ZERO, im: ONE })
    }

    fn cuboid() -> Shape {
        Shape::Cuboid(CuboidTrait::new(v(ONE, HALF)))
    }

    fn capsule() -> Capsule {
        CapsuleTrait::new_y(HALF, HALF)
    }

    fn segment() -> Shape {
        Shape::Segment(SegmentTrait::new(v(-ONE, ZERO), v(ONE, ZERO)))
    }

    fn triangle() -> Shape {
        Shape::ConvexPolygon(
            BoxTrait::new(
                ConvexPolygonTrait::from_convex_polyline(
                    array![v(-ONE, -ONE), v(ONE, -ONE), v(ZERO, ONE)].span(),
                )
                    .unwrap(),
            ),
        )
    }

    /// `(shape1, shape2, pos12, dist, normal1)`: exact answers of the analytic kernel.
    #[test]
    fn test_witness_table() {
        let cases: Span<(Shape, Shape, Pose2, Fixed, Vec2)> = array![
            // Separated faces: 3 - 1 - 1 = 1 along +x.
            (cuboid(), cuboid(), at(int(3), ZERO), ONE, v(ONE, ZERO)),
            // Overlapping faces: depth 0.25 along +y.
            (
                cuboid(),
                cuboid(),
                at(ZERO, HALF + HALF / FixedTrait::from_int(2)),
                -HALF / FixedTrait::from_int(2),
                v(ZERO, ONE),
            ),
            // Capsule above a segment: 2 - 0.5 - 0.5 = 1 along +y.
            (segment(), Shape::Capsule(capsule()), at(ZERO, TWO), ONE, v(ZERO, ONE)),
            // Collinear segments end to end, one unit apart: the direction axis separates them.
            (segment(), segment(), at(int(3), ZERO), ONE, v(ONE, ZERO)),
            // Crossing segments: their difference is a square, depth 1 (upstream's EPA agrees).
            (segment(), segment(), quarter_at(ZERO, ZERO), -ONE, v(ZERO, -ONE)),
            // Triangle apex under a cuboid: 2 - 1 - 0.5 = 0.5 along +y.
            (triangle(), cuboid(), at(ZERO, TWO), HALF, v(ZERO, ONE)),
        ]
            .span();
        for (s1, s2, pos12, dist, normal) in cases {
            let w = witness(*pos12, *s1, *s2);
            assert_eq!(w.dist, *dist);
            assert_eq!(w.normal1, *normal);
            // `point2 - point1 = dist * normal1`.
            assert_eq!(w.point2 - w.point1, v(*normal.x * *dist, *normal.y * *dist));
        }
    }

    #[test]
    fn test_queries_follow_the_witness() {
        let pos12 = at(ZERO, TWO);
        let (s1, s2) = (segment(), Shape::Capsule(capsule()));
        assert_eq!(distance_support_map_support_map(pos12, s1, s2), ONE);
        assert_eq!(
            closest_points_support_map_support_map(pos12, s1, s2, HALF), ClosestPoints::Disjoint,
        );
        assert_eq!(
            closest_points_support_map_support_map(pos12, s1, s2, ONE),
            ClosestPoints::WithinMargin((v(ZERO, ZERO), v(ZERO, -ONE))),
        );
        assert!(contact_support_map_support_map(pos12, s1, s2, HALF).is_none());
        let c = contact_support_map_support_map(pos12, s1, s2, ONE).unwrap();
        assert_eq!(
            (c.point1, c.point2, c.normal2, c.dist),
            (v(ZERO, ZERO), v(ZERO, -ONE), v(ZERO, -ONE), ONE),
        );
        // Overlapping: `Intersecting`, zero distance.
        let deep = at(ZERO, HALF);
        assert_eq!(distance_support_map_support_map(deep, s1, s2), ZERO);
        assert_eq!(
            closest_points_support_map_support_map(deep, s1, s2, ONE), ClosestPoints::Intersecting,
        );
    }

    #[test]
    #[should_panic(expected: 'Query: not a support map')]
    fn test_halfspace_is_not_a_support_map() {
        let _ = witness(
            at(ZERO, ZERO), cuboid(), Shape::HalfSpace(HalfSpaceTrait::new(v(ZERO, ONE))),
        );
    }

    /// The segment-pair candidate and the kernel agree on capsule distances.
    #[test]
    #[fuzzer(runs: 48, seed: 20260926)]
    fn fuzz_capsule_distance_candidates_agree(x: i8, y: i8, turn: bool) {
        let pos12 = if turn {
            quarter_at(
                FixedTrait::from_int(x.into()) / FixedTrait::from_int(4),
                FixedTrait::from_int(y.into()) / FixedTrait::from_int(4),
            )
        } else {
            at(
                FixedTrait::from_int(x.into()) / FixedTrait::from_int(4),
                FixedTrait::from_int(y.into()) / FixedTrait::from_int(4),
            )
        };
        let kernel = distance_support_map_support_map(
            pos12, Shape::Capsule(capsule()), Shape::Capsule(capsule()),
        );
        let candidate = distance_witness(
            pos12, Shape::Capsule(capsule()), Shape::Capsule(capsule()),
        );
        assert!(kernel.abs_diff_eq(candidate, Fixed { raw: 4 }));
    }

    /// The pruned vertex search answers what the exhaustive one does.
    #[test]
    #[fuzzer(runs: 48, seed: 20260926)]
    fn fuzz_pruning_is_exact(x: i8, y: i8, turn: bool) {
        let tx = FixedTrait::from_int(x.into()) / FixedTrait::from_int(8);
        let ty = FixedTrait::from_int(y.into()) / FixedTrait::from_int(8);
        let pos12 = if turn {
            Pose2Trait::new(v(tx, ty), Rot2 { re: Fixed { raw: 3719550787 }, im: HALF })
        } else {
            at(tx, ty)
        };
        assert_eq!(
            distance_support_map_support_map(pos12, triangle(), cuboid()),
            distance_exhaustive(pos12, triangle(), cuboid()),
        );
    }

    /// Every answer is a witness: the distance is never below the distance of the two points
    /// the kernel reports, and a separated pair is not penetrating.
    #[test]
    #[fuzzer(runs: 48, seed: 20260926)]
    fn fuzz_witness_is_consistent(x: i8, y: i8) {
        let pos12 = at(
            FixedTrait::from_int(x.into()) / FixedTrait::from_int(8),
            FixedTrait::from_int(y.into()) / FixedTrait::from_int(8),
        );
        let w = witness(pos12, triangle(), cuboid());
        let d = w.point2 - w.point1;
        assert!(d.x.abs_diff_eq(w.normal1.x * w.dist, Fixed { raw: 16 }));
        assert!(d.y.abs_diff_eq(w.normal1.y * w.dist, Fixed { raw: 16 }));
    }

    #[test]
    fn gas_baseline() {}

    // Separated pairs (vertex–edge search), then overlapping ones (SAT witness).
    #[test]
    fn gas_distance_support_map_cuboid_cuboid() {
        let _ = distance_support_map_support_map(
            opaque(at(int(3), HALF)), opaque(cuboid()), opaque(cuboid()),
        );
    }

    #[test]
    fn gas_distance_exhaustive_cuboid_cuboid() {
        let _ = distance_exhaustive(opaque(at(int(3), HALF)), opaque(cuboid()), opaque(cuboid()));
    }

    #[test]
    fn gas_distance_exhaustive_triangle_triangle() {
        let _ = distance_exhaustive(
            opaque(at(int(3), ZERO)), opaque(triangle()), opaque(triangle()),
        );
    }

    #[test]
    fn gas_distance_support_map_capsule_capsule() {
        let _ = distance_support_map_support_map(
            opaque(quarter_at(TWO, ONE)),
            opaque(Shape::Capsule(capsule())),
            opaque(Shape::Capsule(capsule())),
        );
    }

    #[test]
    fn gas_distance_witness_capsule_capsule() {
        let _ = distance_witness(
            opaque(quarter_at(TWO, ONE)),
            opaque(Shape::Capsule(capsule())),
            opaque(Shape::Capsule(capsule())),
        );
    }

    #[test]
    fn gas_distance_support_map_triangle_triangle() {
        let _ = distance_support_map_support_map(
            opaque(at(int(3), ZERO)), opaque(triangle()), opaque(triangle()),
        );
    }

    #[test]
    fn gas_contact_support_map_cuboid_cuboid_overlapping() {
        let _ = contact_support_map_support_map(
            opaque(at(ONE, HALF)), opaque(cuboid()), opaque(cuboid()), opaque(ZERO),
        );
    }

    #[test]
    fn gas_contact_support_map_segment_capsule_overlapping() {
        let _ = contact_support_map_support_map(
            opaque(at(HALF, ZERO)),
            opaque(segment()),
            opaque(Shape::Capsule(capsule())),
            opaque(ZERO),
        );
    }

    #[test]
    fn gas_contact_support_map_triangle_cuboid_overlapping() {
        let _ = contact_support_map_support_map(
            opaque(at(HALF, HALF)), opaque(triangle()), opaque(cuboid()), opaque(ZERO),
        );
    }

    #[test]
    fn gas_closest_points_support_map_cuboid_capsule() {
        let _ = closest_points_support_map_support_map(
            opaque(at(int(3), ZERO)),
            opaque(cuboid()),
            opaque(Shape::Capsule(capsule())),
            opaque(int(10)),
        );
    }
}
