//! Contact manifolds of the pairs involving a triangle or a round shape (work package SH1):
//! upstream's `contact_manifold_pfm_pfm` (and `contact_manifold_cuboid_triangle` for the
//! cuboid–triangle pairs), on the polygonal cores of the two shapes.
//!
//! Every shape of the pair is a polygonal core and a radius (`ShapeTrait::as_polygonal_feature_map`
//! upstream): a cuboid, a convex polygon or a triangle with radius 0, a segment with radius 0, a
//! capsule's segment with its radius, a round shape's inner shape with its border radius. As
//! upstream:
//!
//! 1. the persistence fast path (`try_update_contacts`) first;
//! 2. the separating axis of the two cores with the prediction grown by both radii, the
//!    polygonal features of both cores along it, clipped against each other;
//! 3. every contact pushed out by the radii along the normals (`local_p1 += n1 r1`,
//!    `local_p2 += n2 r2`, `dist -= r1 + r2`), and the warm-start data matched by feature ids.
//!
//! Step 2 is the bounded-polygon SAT of `polygon_polygon` (face axes, then the closest edge pair
//! of separated cores, whose direction is the Euclidean normal GJK would find) where upstream runs
//! GJK (ADR 0001 entry 17). The feature ids are the native ones of each core: cuboid ids for a
//! cuboid, `2 i` / `2 i + 1` for a polygon, the triangle's `Vertex(i)` / `Face(i)` (upstream's
//! `Triangle::support_face` ids, whatever the orientation) and the segment's `Vertex(0|2)` /
//! `Face(1)`. Segment–segment, segment–capsule and capsule–capsule cores are not routed here
//! (the dispatcher keeps its existing generators or reports them unsupported).

use fixed::{Fixed, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::contact::{ContactManifold, ContactManifoldTrait};
use crate::manifold::ManifoldTrait;
use crate::polygonal_feature::PolygonalFeature;
use crate::shape::triangle::{feature_to_triangle, triangle_core};
use crate::shape::{ConvexPolygon, ConvexPolygonTrait, Cuboid, CuboidTrait, Segment, Shape};
use super::polygon_polygon::{cuboid_core, finish, separating_axis};
use super::polygon_segment::core as segment_core;

/// The polygonal core of one side, with what its feature ids need.
#[derive(Copy, Drop)]
enum Inner {
    Cuboid: Cuboid,
    Polygon: ConvexPolygon,
    /// The counter-clockwise core and whether the triangle is clockwise.
    Triangle: (ConvexPolygon, bool),
    Segment: Segment,
}

/// `(core, radius)` of `shape`, `None` for a ball or a half-space.
fn decompose(shape: Shape) -> Option<(Inner, Fixed)> {
    match shape {
        Shape::Cuboid(c) => Some((Inner::Cuboid(c), ZERO)),
        Shape::ConvexPolygon(p) => Some((Inner::Polygon(p.unbox()), ZERO)),
        Shape::Triangle(t) => Some((Inner::Triangle(triangle_core(t.unbox())), ZERO)),
        Shape::Segment(s) => Some((Inner::Segment(s), ZERO)),
        Shape::Capsule(c) => Some((Inner::Segment(c.segment), c.radius)),
        Shape::RoundCuboid(s) => Some((Inner::Cuboid(s.inner_shape), s.border_radius)),
        Shape::RoundTriangle(s) => {
            let s = s.unbox();
            Some((Inner::Triangle(triangle_core(s.inner_shape)), s.border_radius))
        },
        Shape::RoundConvexPolygon(s) => {
            let s = s.unbox();
            Some((Inner::Polygon(s.inner_shape), s.border_radius))
        },
        _ => None,
    }
}

#[inline(always)]
fn core_of(inner: Inner) -> ConvexPolygon {
    match inner {
        Inner::Cuboid(c) => cuboid_core(c),
        Inner::Polygon(p) => p,
        Inner::Triangle((core, _)) => core,
        Inner::Segment(s) => segment_core(s),
    }
}

/// The support feature of `inner` along its local `dir`, with its native ids.
fn feature_of(inner: Inner, dir: Vec2) -> PolygonalFeature {
    match inner {
        Inner::Cuboid(c) => c.support_feature(dir),
        Inner::Polygon(p) => p.support_feature(dir),
        Inner::Triangle((
            core, reversed,
        )) => feature_to_triangle(core.support_feature(dir), reversed),
        Inner::Segment(s) => s.into(),
    }
}

/// `1 - 2^-24`: a closest-pair direction this close to a face normal is that face normal.
const SNAP_COS: Fixed = Fixed { raw: 0xffffff00 };

/// The face normal of either core (shape 2's moved into shape 1's frame) most aligned with the
/// unit `n`, when their cosine is at least [`SNAP_COS`]; `n` otherwise.
///
/// A closest-pair direction is the normalised difference of two witness points: when the cores
/// are close, its rounding error is `2^-32 / |q2 - q1|` (`~1e-8` at a gap of `0.02`), where
/// upstream's GJK lands on the exact face normal of a vertex–face pair. Snapping gives that
/// normal back; a genuine vertex–vertex direction within `3.5e-4` rad of a face normal moves by
/// at most that angle.
fn snap_normal(n: Vec2, core1: ConvexPolygon, core2: ConvexPolygon, pos12: Pose2) -> Vec2 {
    let mut best = n;
    let mut best_dot = SNAP_COS;
    let mut i = 0;
    while i != core1.count {
        let m = core1.normal(i);
        let d = m.dot(n);
        if d >= best_dot {
            best = m;
            best_dot = d;
        }
        i += 1;
    }
    let mut j = 0;
    while j != core2.count {
        let m = pos12.transform_vector(-core2.normal(j));
        let d = m.dot(n);
        if d >= best_dot && core2.normal(j) != Vec2Trait::ZERO {
            best = m;
            best_dot = d;
        }
        j += 1;
    }
    best
}

#[inline(always)]
fn is_segment(inner: Inner) -> bool {
    match inner {
        Inner::Segment(_) => true,
        _ => false,
    }
}

/// The contact manifold of `shape1` and `shape2` (`pos12` places shape 2 in shape 1's frame),
/// updated in place as `crate::dispatch::contact_manifold` does. Returns `false` (and clears the
/// manifold) when either shape is a ball or a half-space, or both cores are segments.
/// #### Panics
/// * The panics of `polygon_polygon` (overflow of the wide products; coordinates bounded by 8192
///   suffice for clipping) and `'Fixed: overflow'` when a pushed point leaves the scalar range.
/// #### Deviations
/// * SAT and closest edge pairs instead of GJK (see the module documentation): the distance and
///   the normal agree within rounding, the contact points are the clipped features in both.
/// * A clockwise triangle keeps its outward normals (its core is reoriented); upstream's
///   `support_face` then picks inward normals.
pub fn contact_manifold_pfm_pfm(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    let (Some((inner1, r1)), Some((inner2, r2))) = (decompose(shape1), decompose(shape2)) else {
        manifold.clear();
        return false;
    };
    if is_segment(inner1) && is_segment(inner2) {
        manifold.clear();
        return false;
    }
    if manifold.try_update_contacts(pos12) {
        return true;
    }
    let (core1, core2) = (core_of(inner1), core_of(inner2));
    match separating_axis(core1, core2, pos12, prediction + r1 + r2) {
        None => manifold.clear(),
        Some((
            n, witnesses,
        )) => {
            let n = if witnesses.is_some() {
                snap_normal(n, core1, core2, pos12)
            } else {
                n
            };
            let f1 = feature_of(inner1, n);
            let f2 = feature_of(inner2, pos12.inverse_transform_vector(-n));
            finish(pos12, n, f1, f2, witnesses, r2, false, ref manifold);
            if r1 != ZERO {
                let offset = manifold.local_n1.mul_scalar(r1);
                let [mut a, mut b] = manifold.points;
                a.local_p1 = a.local_p1 + offset;
                b.local_p1 = b.local_p1 + offset;
                a.dist = a.dist - r1;
                b.dist = b.dist - r1;
                manifold.points = [a, b];
            }
        },
    }
    true
}

/// The cuboid–triangle manifold (Parry `contact_manifold_cuboid_triangle`, with `pos12` placing
/// the triangle in the cuboid's frame): [`contact_manifold_pfm_pfm`] on the pair.
pub fn contact_manifold_cuboid_triangle(
    pos12: Pose2,
    cuboid1: Cuboid,
    triangle2: crate::shape::Triangle,
    prediction: Fixed,
    ref manifold: ContactManifold,
) {
    let _ = contact_manifold_pfm_pfm(
        pos12, Shape::Cuboid(cuboid1), triangle2.into(), prediction, ref manifold,
    );
}

/// Parry `contact_manifold_cuboid_triangle_shapes`: the cuboid–triangle pair in either order,
/// `false` (manifold untouched) for any other pair.
pub fn contact_manifold_cuboid_triangle_shapes(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    match (shape1, shape2) {
        (Shape::Cuboid(_), Shape::Triangle(_)) |
        (
            Shape::Triangle(_), Shape::Cuboid(_),
        ) => { contact_manifold_pfm_pfm(pos12, shape1, shape2, prediction, ref manifold) },
        _ => false,
    }
}

#[cfg(test)]
mod tests;
