//! SH2b contact manifolds of the compounds against the convex shapes, the half-space, the SH2a
//! composites and another compound (one per part pair, both orders), `intersection_test` and
//! `distance` on the same placements, and the AABBs, bounding spheres and mass properties (sum of
//! the parts') of the compounds, against Parry f64 0.30.2.
//!
//! Bands (raw Q32.32 units): `BAND` for normals, points and distances (the part generators are
//! the convex ones of the earlier families, on part poses composed in fixed point); `EXACT` for
//! the boxes; `MASS_BAND` for the summed mass properties (one rounding per part and per sum).
//! A pair with a round cuboid (the `rcub` shape, a part of `mixed`) runs upstream's GJK
//! projection on the rounded corner where the port is exact: `ROUND_BAND` = `2^19` (the SH1
//! projection band; `trio_rcub/overlapping`, a ball part on the corner: 4.1e5 raw on the normal).
//! Manifolds are matched by their sub-shape ids (upstream's order is its BVH's); feature ids are
//! not compared (as SH2a: the part pairs run upstream's PFM–PFM generator on SAT here).
use fixed::Fixed;
use glam_core::Vec2;
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldTrait};
use rapier_geometry2d::dispatch::composite::contact_manifolds_composite;
use rapier_geometry2d::mass::MassPropertiesTrait;
use rapier_geometry2d::query::{distance, intersection_test};
use rapier_geometry2d::shape::{CompoundTrait, Shape, ShapeTrait};
use rapier_golden::compare::{abs_diff, vec2_within, within};
use rapier_golden::generated::{composite_shapes, compound_contacts, compound_mass, compound_shapes};
use rapier_golden::types::{CompoundManifoldCase, CompoundRaw, OtherRaw, Sh1ShapeRaw, Vec2Raw};
use super::composite_contacts_golden::{composite_shape, pose};
use super::sh1_contacts_golden::shape;

const BAND: u64 = 64;
const EXACT: u64 = 4;
const MASS_BAND: u64 = 64;
const ROUND_BAND: u64 = 0x80000;

fn raw(p: Vec2) -> Vec2Raw {
    Vec2Raw { x: p.x.raw, y: p.y.raw }
}

/// The compound of `c`.
pub fn compound_shape(c: CompoundRaw) -> Shape {
    let mut parts = array![];
    for (p, s) in c.parts {
        parts.append((pose(*p), shape(*s)));
    }
    CompoundTrait::new(parts.span()).into()
}

fn other_shape(o: OtherRaw) -> Shape {
    match o {
        OtherRaw::Convex(s) => shape(s),
        OtherRaw::Composite(i) => composite_shape(composite_shapes::composite(i)),
        OtherRaw::Compound(i) => compound_shape(compound_shapes::compound(i)),
    }
}

/// Whether the case pairs a round cuboid (see the module documentation).
fn round(c: CompoundManifoldCase) -> bool {
    match c.other {
        OtherRaw::Convex(Sh1ShapeRaw::RoundCuboid(_)) => true,
        OtherRaw::Compound(i) => i == 2,
        _ => c.compound == 2,
    }
}

fn check_manifold(c: CompoundManifoldCase, index: u32, m: ContactManifold) {
    let e = *c.manifolds.span().at(index);
    let band = if round(c) {
        ROUND_BAND
    } else {
        BAND
    };
    assert_eq!((m.subshape1, m.subshape2), (e.subshape1, e.subshape2), "{} ids {}", c.id, index);
    assert_eq!(m.num_points.into(), e.num_points, "{} count {}", c.id, index);
    assert!(vec2_within(raw(m.local_n1), e.local_n1, band), "{} n1 {} {:?}", c.id, index, m);
    assert!(vec2_within(raw(m.local_n2), e.local_n2, band), "{} n2 {}", c.id, index);
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
                && within(a.dist.raw, b.dist, band)
                && vec2_within(raw(a.local_p1), b.local_p1, band)
                && vec2_within(raw(a.local_p2), b.local_p2, band) {
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
        assert!(found, "{} pair {} point {} {:?} expected {:?}", c.id, index, i, a, e.points);
        i += 1;
    }
}

/// The manifolds of `got` with points, ascending `(subshape1, subshape2)`.
fn touching_sorted(got: Span<ContactManifold>) -> Array<ContactManifold> {
    let mut left: Array<ContactManifold> = array![];
    for m in got {
        if *m.num_points != 0 {
            left.append(*m);
        }
    }
    let mut out = array![];
    while !left.is_empty() {
        let s = left.span();
        let mut best: u32 = 0;
        let mut k: u32 = 1;
        while k != s.len() {
            let (a, b) = (s.at(k), s.at(best));
            if *a.subshape1 < *b.subshape1
                || (*a.subshape1 == *b.subshape1 && *a.subshape2 < *b.subshape2) {
                best = k;
            }
            k += 1;
        }
        out.append(*s.at(best));
        let mut rest = array![];
        let mut k: u32 = 0;
        while k != s.len() {
            if k != best {
                rest.append(*s.at(k));
            }
            k += 1;
        }
        left = rest;
    }
    out
}

#[test]
fn test_compound_manifolds_golden() {
    let prediction = Fixed { raw: compound_contacts::PREDICTION };
    let mut count = 0;
    for c in compound_contacts::cases() {
        let comp = compound_shape(compound_shapes::compound(*c.compound));
        let other = other_shape(*c.other);
        let (s1, s2) = if *c.compound_first {
            (comp, other)
        } else {
            (other, comp)
        };
        let p = pose(*c.pos12);
        let got = contact_manifolds_composite(p, s1, s2, prediction, array![].span()).unwrap();
        let touching = touching_sorted(got.span());
        assert_eq!(touching.len(), *c.num_manifolds, "{} manifolds {:?}", *c.id, got);
        for i in 0..touching.len() {
            check_manifold(*c, i, *touching.at(i));
        }
        // Warm start (the persistence path from the stored manifolds): the same manifolds.
        let again = touching_sorted(
            contact_manifolds_composite(p, s1, s2, prediction, got.span()).unwrap().span(),
        );
        assert_eq!(again.len(), *c.num_manifolds, "{} warm", *c.id);
        for i in 0..again.len() {
            check_manifold(*c, i, *again.at(i));
        }
        let id = rapier_math::pose2::IDENTITY;
        let intersects = intersection_test(id, s1, p, s2);
        assert_eq!(intersects.is_some(), *c.intersects_supported, "{} intersects?", *c.id);
        if let Some(b) = intersects {
            assert_eq!(b, *c.intersects, "{} intersects", *c.id);
        }
        let d = distance(id, s1, p, s2);
        assert_eq!(d.is_some(), *c.distance_supported, "{} distance?", *c.id);
        if let Some(d) = d {
            if *c.distance_infinite {
                assert_eq!(d, fixed::MAX, "{} distance max", *c.id);
            } else {
                assert!(abs_diff(d.raw, *c.distance) <= BAND, "{} distance {:?}", *c.id, d);
            }
        }
        count += 1;
    }
    assert_eq!(count, 93);
}

#[test]
fn test_compound_mass_golden() {
    let mut count = 0;
    let mut worst: u64 = 0;
    for c in compound_mass::cases() {
        let s = compound_shape(compound_shapes::compound(*c.compound));
        let p = pose(*c.pose);
        let b = s.compute_aabb(p);
        assert!(vec2_within(raw(b.mins), *c.mins, EXACT), "{} mins {:?}", *c.id, b);
        assert!(vec2_within(raw(b.maxs), *c.maxs, EXACT), "{} maxs {:?}", *c.id, b);
        let sphere = s.compute_bounding_sphere(p);
        assert!(vec2_within(raw(sphere.center), *c.center, EXACT), "{} center", *c.id);
        assert!(abs_diff(sphere.radius.raw, *c.radius) <= EXACT, "{} radius", *c.id);
        let props = s.mass_properties(Fixed { raw: *c.density });
        let devs = array![
            abs_diff(props.mass().raw, *c.mass), abs_diff(props.local_com.x.raw, *c.local_com.x),
            abs_diff(props.local_com.y.raw, *c.local_com.y),
            abs_diff(props.principal_inertia().raw, *c.inertia),
        ];
        for d in devs {
            assert!(d <= MASS_BAND, "{} mass properties {:?} off by {}", *c.id, props, d);
            if d > worst {
                worst = d;
            }
        }
        count += 1;
    }
    println!("compound mass properties: worst deviation {} ulp", worst);
    assert_eq!(count, 12);
}
