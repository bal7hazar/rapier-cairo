//! CE: contact manifolds of compounds with `FIX_INTERNAL_EDGES` (parry 0.31.1) against a ball, a
//! cuboid or a polygon sliding across the cut between two parts, through
//! `contact_manifolds_composite_constrained` (the constrained composite strategy), and the same
//! placements on the unflagged compounds through both the constrained and the default
//! `contact_manifolds_composite`, which must agree with each other and with upstream.
//!
//! Bands as `compound_contacts_golden` (raw Q32.32 units): `BAND` for normals, points and
//! distances; manifolds with points are matched by their sub-shape ids, feature ids are not
//! compared (the PFM–PFM part pairs run SAT here, GJK upstream).
use fixed::Fixed;
use glam_core::Vec2;
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldTrait};
use rapier_geometry2d::dispatch::composite::constrained::contact_manifolds_composite_constrained;
use rapier_geometry2d::dispatch::composite::contact_manifolds_composite;
use rapier_geometry2d::query::{distance, intersection_test};
use rapier_geometry2d::shape::{CompoundTrait, FIX_INTERNAL_EDGES, Shape};
use rapier_golden::compare::{abs_diff, vec2_within, within};
use rapier_golden::generated::{
    compound_internal_edges_flagged, compound_internal_edges_plain, compound_internal_edges_shapes,
};
use rapier_golden::types::{CompoundManifoldCase, CompoundRaw, OtherRaw, Vec2Raw};
use super::composite_contacts_golden::pose;
use super::sh1_contacts_golden::shape;

const BAND: u64 = 64;

fn raw(p: Vec2) -> Vec2Raw {
    Vec2Raw { x: p.x.raw, y: p.y.raw }
}

/// The compound of `c`, flagged or not.
fn compound(c: CompoundRaw, flagged: bool) -> Shape {
    let mut parts = array![];
    for (p, s) in c.parts {
        parts.append((pose(*p), shape(*s)));
    }
    if flagged {
        CompoundTrait::with_flags(parts.span(), FIX_INTERNAL_EDGES, None).into()
    } else {
        CompoundTrait::new(parts.span()).into()
    }
}

/// The manifolds of `got` with points, ascending `(subshape1, subshape2)` (at most a few).
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

/// Manifold `m` against the case's manifold `index`: ids, count and normals, and each point
/// matched to a distinct expected point.
fn check_manifold(c: @CompoundManifoldCase, index: u32, m: ContactManifold) {
    let e = *c.manifolds.span().at(index);
    assert_eq!((m.subshape1, m.subshape2), (e.subshape1, e.subshape2), "{} ids {}", *c.id, index);
    assert_eq!(m.num_points.into(), e.num_points, "{} count {}", *c.id, index);
    assert!(vec2_within(raw(m.local_n1), e.local_n1, BAND), "{} n1 {} {:?}", *c.id, index, m);
    assert!(vec2_within(raw(m.local_n2), e.local_n2, BAND), "{} n2 {}", *c.id, index);
    let expected = e.points.span();
    let mut taken: u32 = 2;
    let mut i = 0;
    while i != m.num_points {
        let a = m.point(i);
        let mut found = false;
        let mut j: u32 = 0;
        while j != e.num_points {
            let b = *expected.at(j);
            if !found
                && j != taken
                && within(a.dist.raw, b.dist, BAND)
                && vec2_within(raw(a.local_p1), b.local_p1, BAND)
                && vec2_within(raw(a.local_p2), b.local_p2, BAND) {
                found = true;
                taken = j;
            }
            j += 1;
        }
        assert!(found, "{} pair {} point {} {:?} expected {:?}", *c.id, index, i, a, expected);
        i += 1;
    }
}

fn check_all(c: @CompoundManifoldCase, got: Span<ContactManifold>) {
    let touching = touching_sorted(got);
    assert_eq!(touching.len(), *c.num_manifolds, "{} manifolds {:?}", *c.id, got);
    for i in 0..touching.len() {
        check_manifold(c, i, *touching.at(i));
    }
}

/// The shapes and `pos12` of case `c`.
fn pair(c: @CompoundManifoldCase, flagged: bool) -> (Shape, Shape, rapier_math::pose2::Pose2) {
    let comp = compound(compound_internal_edges_shapes::compound(*c.compound), flagged);
    let other = match *c.other {
        OtherRaw::Convex(s) => shape(s),
        _ => core::panic_with_felt252('CE: a convex other'),
    };
    let (s1, s2) = if *c.compound_first {
        (comp, other)
    } else {
        (other, comp)
    };
    (s1, s2, pose(*c.pos12))
}

/// `intersection_test` and `distance` do not read the cones: upstream's answers either way.
fn check_queries(c: @CompoundManifoldCase, s1: Shape, s2: Shape, p: rapier_math::pose2::Pose2) {
    let id = rapier_math::pose2::IDENTITY;
    let intersects = intersection_test(id, s1, p, s2);
    assert_eq!(intersects.is_some(), *c.intersects_supported, "{} intersects?", *c.id);
    if let Some(b) = intersects {
        assert_eq!(b, *c.intersects, "{} intersects", *c.id);
    }
    if let Some(d) = distance(id, s1, p, s2) {
        assert!(*c.distance_supported, "{} distance?", *c.id);
        assert!(abs_diff(d.raw, *c.distance) <= BAND, "{} distance {:?}", *c.id, d);
    }
}

#[test]
fn test_flagged_compounds_golden() {
    let prediction = Fixed { raw: compound_internal_edges_flagged::PREDICTION };
    let mut count = 0;
    for c in compound_internal_edges_flagged::cases() {
        let (s1, s2, p) = pair(c, true);
        let got = contact_manifolds_composite_constrained(p, s1, s2, prediction, array![].span())
            .unwrap();
        check_all(c, got.span());
        // Warm start: the persistence path from the stored manifolds gives the same manifolds.
        let again = contact_manifolds_composite_constrained(p, s1, s2, prediction, got.span())
            .unwrap();
        check_all(c, again.span());
        check_queries(c, s1, s2, p);
        count += 1;
    }
    assert_eq!(count, 21);
}

#[test]
fn test_unflagged_twins_golden() {
    let prediction = Fixed { raw: compound_internal_edges_plain::PREDICTION };
    let mut count = 0;
    for c in compound_internal_edges_plain::cases() {
        let (s1, s2, p) = pair(c, false);
        let got = contact_manifolds_composite_constrained(p, s1, s2, prediction, array![].span())
            .unwrap();
        let default = contact_manifolds_composite(p, s1, s2, prediction, array![].span()).unwrap();
        assert_eq!(got, default, "{} strategies", *c.id);
        check_all(c, got.span());
        check_queries(c, s1, s2, p);
        count += 1;
    }
    assert_eq!(count, 21);
}
