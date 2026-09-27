//! SH2a contact manifolds of the polyline and the heightfield against the convex shapes (one per
//! part), `intersection_test` and `distance` on the same placements, and the AABBs, bounding
//! spheres and (zero) mass of the composite shapes, against Parry f64 0.30.2.
//!
//! Bands (raw Q32.32 units): `BAND` for normals, points and distances (the part generators are
//! the convex ones of the earlier families; upstream runs GJK on segment parts against rounded
//! shapes); `EXACT` for the boxes. Feature ids are not compared: a heightfield cell is a
//! zero-radius capsule upstream as here, but a polyline segment against a segment or capsule runs
//! upstream's PFM–PFM generator where the port runs the capsule–capsule one (see
//! `rapier_geometry2d::dispatch::composite`).
use fixed::Fixed;
use glam::Vec2;
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldTrait};
use rapier_geometry2d::dispatch::composite::contact_manifolds_composite;
use rapier_geometry2d::query::{distance, intersection_test};
use rapier_geometry2d::shape::{
    HeightFieldTrait, PolylineFlagsTrait, PolylineTrait, Shape, ShapeTrait,
};
use rapier_golden::compare::{abs_diff, vec2_within, within};
use rapier_golden::generated::{composite_aabbs, composite_contacts, composite_shapes};
use rapier_golden::types::{CompositeManifoldCase, CompositeRaw, PoseRaw, Vec2Raw};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use super::sh1_contacts_golden::shape;

const BAND: u64 = 64;
const EXACT: u64 = 4;

fn v(p: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: p.x }, y: Fixed { raw: p.y } }
}
fn raw(p: Vec2) -> Vec2Raw {
    Vec2Raw { x: p.x.raw, y: p.y.raw }
}
pub fn pose(p: PoseRaw) -> Pose2 {
    Pose2 {
        translation: v(p.translation),
        rotation: Rot2 { re: Fixed { raw: p.rotation.re }, im: Fixed { raw: p.rotation.im } },
    }
}

/// The composite shape of `c`: a polyline (chained when `indices` is empty) or a heightfield.
pub fn composite_shape(c: CompositeRaw) -> Shape {
    if c.is_heightfield {
        let mut heights = array![];
        for h in c.heights {
            heights.append(Fixed { raw: *h });
        }
        let mut hf = HeightFieldTrait::new(heights.span(), v(c.scale));
        for i in c.removed {
            hf.set_segment_removed(*i, true);
        }
        return hf.into();
    }
    let mut vertices = array![];
    for p in c.vertices {
        vertices.append(v(*p));
    }
    let indices = if c.indices.len() == 0 {
        None
    } else {
        let mut out = array![];
        for (a, b) in c.indices {
            out.append([*a, *b]);
        }
        Some(out.span())
    };
    let flags = if c.oriented {
        PolylineFlagsTrait::oriented()
    } else {
        PolylineFlagsTrait::empty()
    };
    PolylineTrait::with_flags(vertices.span(), indices, flags).into()
}

fn check_manifold(c: CompositeManifoldCase, index: u32, m: ContactManifold) {
    let e = *c.manifolds.span().at(index);
    let part = if c.composite_first {
        m.subshape1
    } else {
        m.subshape2
    };
    assert_eq!(part, e.part, "{} part {}", c.id, index);
    assert_eq!(m.num_points.into(), e.num_points, "{} count {}", c.id, index);
    assert!(vec2_within(raw(m.local_n1), e.local_n1, BAND), "{} n1 {}", c.id, index);
    assert!(vec2_within(raw(m.local_n2), e.local_n2, BAND), "{} n2 {}", c.id, index);
    let mut used = array![false, false];
    let mut i = 0;
    while i != m.num_points {
        let a = m.point(i);
        let mut found = false;
        let mut j = 0;
        let mut next = array![];
        while j != e.num_points {
            let b = *e.points.span().at(j);
            let free = !*used.at(j);
            if !found
                && free
                && within(a.dist.raw, b.dist, BAND)
                && vec2_within(raw(a.local_p1), b.local_p1, BAND)
                && vec2_within(raw(a.local_p2), b.local_p2, BAND) {
                found = true;
                next.append(true);
            } else {
                next.append(!free);
            }
            j += 1;
        }
        if e.num_points == 1 {
            next.append(false);
        }
        used = next;
        assert!(found, "{} part {} point {} {:?} expected {:?}", c.id, e.part, i, a, e.points);
        i += 1;
    }
}

#[test]
fn test_composite_manifolds_golden() {
    let prediction = Fixed { raw: composite_contacts::PREDICTION };
    let mut count = 0;
    for c in composite_contacts::cases() {
        let composite = composite_shape(composite_shapes::composite(*c.composite));
        let other = shape(*c.other);
        let (s1, s2) = if *c.composite_first {
            (composite, other)
        } else {
            (other, composite)
        };
        let p = pose(*c.pos12);
        let got = contact_manifolds_composite(p, s1, s2, prediction, array![].span()).unwrap();
        let mut touching = array![];
        for m in got.span() {
            if *m.num_points != 0 {
                touching.append(*m);
            }
        }
        assert_eq!(touching.len(), *c.num_manifolds, "{} manifolds", *c.id);
        for i in 0..touching.len() {
            check_manifold(*c, i, *touching.at(i));
        }
        // Warm start: the stored manifolds give the same answer.
        let again = contact_manifolds_composite(p, s1, s2, prediction, got.span()).unwrap();
        assert_eq!(again.len(), got.len(), "{} warm", *c.id);
        let id = rapier_math::pose2::IDENTITY;
        let intersects = intersection_test(id, s1, p, s2);
        assert_eq!(intersects.is_some(), *c.intersects_supported, "{} intersects?", *c.id);
        if let Some(b) = intersects {
            assert_eq!(b, *c.intersects, "{} intersects", *c.id);
        }
        let d = distance(id, s1, p, s2);
        assert_eq!(d.is_some(), *c.distance_supported, "{} distance?", *c.id);
        if let Some(d) = d {
            assert!(abs_diff(d.raw, *c.distance) <= BAND, "{} distance {:?}", *c.id, d);
        }
        count += 1;
    }
    assert_eq!(count, 115);
}

#[test]
fn test_composite_aabbs_golden() {
    for c in composite_aabbs::cases() {
        let s = composite_shape(composite_shapes::composite(*c.composite));
        let p = pose(*c.pose);
        let b = s.compute_aabb(p);
        assert!(vec2_within(raw(b.mins), *c.mins, EXACT), "{} mins {:?}", *c.id, b);
        assert!(vec2_within(raw(b.maxs), *c.maxs, EXACT), "{} maxs {:?}", *c.id, b);
        let sphere = s.compute_bounding_sphere(p);
        assert!(vec2_within(raw(sphere.center), *c.center, EXACT), "{} center", *c.id);
        assert!(abs_diff(sphere.radius.raw, *c.radius) <= EXACT, "{} radius", *c.id);
        assert_eq!(s.mass_properties(Fixed { raw: 1 }).inv_mass.raw, 0, "{} mass", *c.id);
        assert_eq!(*c.mass, 0);
    }
}
