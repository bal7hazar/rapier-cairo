//! Every capsule–capsule, cuboid–capsule and capsule–cuboid case of
//! `rapier_golden::contact_manifolds`.
//!
//! Upstream routes cuboid–capsule through its PFM–PFM (GJK) generator, this port through the
//! analytic generator upstream keeps disabled: the point *order* may differ, so those points are
//! matched by their `(fid1, fid2)` pair. Capsule–capsule is the same algorithm and is compared in
//! order. `ambiguous` cases compare only the point count and the multiset of distances.
use fixed::{Fixed, FixedTrait};
use glam::Vec2;
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldTrait, TrackedContact};
use rapier_geometry2d::contact_generators::capsule_capsule::contact_manifold_capsule_capsule_shapes;
use rapier_geometry2d::contact_generators::cuboid_capsule::contact_manifold_cuboid_capsule_shapes;
use rapier_geometry2d::shape::{CapsuleTrait, CuboidTrait, Shape};
use rapier_golden::compare::within;
use rapier_golden::contact_manifolds::{self, PREDICTION};
use rapier_golden::types::{ContactPointRaw, ManifoldCase, ShapeRaw, Vec2Raw};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;

/// Raw ulps allowed on points, normals and distances (measured maximum error: 3–4 ulps; 2 fails).
const TOL: u64 = 4;

fn vr(r: Vec2Raw) -> Vec2 {
    Vec2 { x: FixedTrait::from_raw(r.x), y: FixedTrait::from_raw(r.y) }
}
fn shape_of(s: ShapeRaw) -> Option<Shape> {
    match s {
        ShapeRaw::Capsule(c) => Some(
            Shape::Capsule(CapsuleTrait::new(vr(c.a), vr(c.b), FixedTrait::from_raw(c.radius))),
        ),
        ShapeRaw::Cuboid(h) => Some(Shape::Cuboid(CuboidTrait::new(vr(h)))),
        _ => None,
    }
}
fn pose_of(c: @ManifoldCase) -> Pose2 {
    let p = *c.pos12;
    Pose2 {
        translation: vr(p.translation),
        rotation: Rot2 {
            re: FixedTrait::from_raw(p.rotation.re), im: FixedTrait::from_raw(p.rotation.im),
        },
    }
}
/// Runs the generator of the pair; `None` when the case is not a capsule pair of this package.
fn generate(c: @ManifoldCase) -> Option<(ContactManifold, bool)> {
    let (s1, s2) = match (shape_of(*c.shape1), shape_of(*c.shape2)) {
        (Some(a), Some(b)) => (a, b),
        _ => { return None; },
    };
    let mut m: ContactManifold = Default::default();
    let prediction = FixedTrait::from_raw(PREDICTION);
    if contact_manifold_capsule_capsule_shapes(pose_of(c), s1, s2, prediction, ref m) {
        return Some((m, true));
    }
    let is_capsule = match s1 {
        Shape::Capsule(_) => true,
        _ => false,
        }
        || match s2 {
        Shape::Capsule(_) => true,
        _ => false,
    };
    if !is_capsule {
        return None;
    }
    assert!(contact_manifold_cuboid_capsule_shapes(pose_of(c), s1, s2, prediction, ref m));
    Some((m, false))
}
fn raw(v: Vec2) -> Vec2Raw {
    Vec2Raw { x: v.x.raw, y: v.y.raw }
}
fn vec_ok(a: Vec2, b: Vec2Raw) -> bool {
    within(a.x.raw, b.x, TOL) && within(a.y.raw, b.y, TOL)
}
fn point_ok(p: TrackedContact, q: ContactPointRaw) -> bool {
    p.fid1.packed == q.fid1
        && p.fid2.packed == q.fid2
        && vec_ok(p.local_p1, q.local_p1)
        && vec_ok(p.local_p2, q.local_p2)
        && within(p.dist.raw, q.dist, TOL)
}
fn dist_of(m: ContactManifold, i: u32) -> Fixed {
    m.point(i.try_into().unwrap()).dist
}

#[test]
fn test_all_capsule_manifold_vectors() {
    let mut count = 0_u32;
    for c in contact_manifolds::cases() {
        let (m, ordered) = match generate(c) {
            Some(r) => r,
            None => { continue; },
        };
        let id = *c.id;
        let n: u32 = m.num_points.into();
        assert_eq!(n, *c.num_points, "{}: num_points", id);
        let [e0, e1] = *c.points;
        let expected = array![e0, e1];
        if *c.ambiguous {
            // Distances as a multiset (two points: either order).
            if n == 1 {
                assert!(within(dist_of(m, 0).raw, e0.dist, TOL), "{}: dist", id);
            } else if n == 2 {
                let straight = within(dist_of(m, 0).raw, e0.dist, TOL)
                    && within(dist_of(m, 1).raw, e1.dist, TOL);
                let crossed = within(dist_of(m, 0).raw, e1.dist, TOL)
                    && within(dist_of(m, 1).raw, e0.dist, TOL);
                assert!(straight || crossed, "{}: dists", id);
            }
            count += 1;
            continue;
        }
        if n != 0 {
            assert!(vec_ok(m.local_n1, *c.local_n1), "{}: n1 {:?}", id, raw(m.local_n1));
            assert!(vec_ok(m.local_n2, *c.local_n2), "{}: n2 {:?}", id, raw(m.local_n2));
        }
        let mut i = 0_u32;
        while i != n {
            let p = m.point(i.try_into().unwrap());
            let ok = if ordered {
                point_ok(p, *expected.at(i))
            } else {
                let mut j = 0_u32;
                let mut found = false;
                while j != n {
                    found = found || point_ok(p, *expected.at(j));
                    j += 1;
                }
                found
            };
            assert!(ok, "{}: point {} {:?}", id, i, p);
            i += 1;
        }
        count += 1;
    }
    // 8 capsule–capsule + 6 cuboid–capsule + 1 capsule–cuboid.
    assert_eq!(count, 15);
}

#[test]
fn gas_baseline() {
    let _ = opaque(contact_manifolds::CAPSULE_CAPSULE_DEEP);
}
#[test]
fn gas_golden_capsule_capsule_deep() {
    let _ = generate(@opaque(contact_manifolds::CAPSULE_CAPSULE_DEEP));
}
#[test]
fn gas_golden_cuboid_capsule_shallow() {
    let _ = generate(@opaque(contact_manifolds::CUBOID_CAPSULE_SHALLOW));
}
