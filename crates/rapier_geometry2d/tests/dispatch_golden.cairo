//! Every case of `rapier_golden::contact_manifolds` (87) through `dispatch::contact_manifold`, in
//! the recorded order and with the two shapes swapped (`pos12.inverse()`), plus the unsupported
//! pairs.
//!
//! * Recorded order: point counts, normals and points within `TOL` raw units (64, the README's
//!   figure; 4 for the capsule generators, whose measured error is 3–4 ulps). Points are compared
//!   in order, except for the cuboid–capsule pairs, whose analytic generator may order them
//!   differently from upstream's GJK one and which are matched by `(fid1, fid2)`.
//! * Swapped: the manifold must be the mirror image of the recorded one (normals, points and
//!   feature ids exchanged between the two sides). Clipping follows the reversed tangent, so the
//!   points are matched as a set; the ids of a few tie cases are not compared, and four exact-tie
//!   cases only compare count and distances (see `tie_ids`, `tie_axes`).
//! * `ambiguous` cases: point count and the multiset of `dist` only.
use fixed::Fixed;
use glam::Vec2;
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldTrait, TrackedContact};
use rapier_geometry2d::dispatch::contact_manifold;
use rapier_geometry2d::shape::{Ball, Capsule, Cuboid, HalfSpace, Segment, Shape};
use rapier_golden::compare::within;
use rapier_golden::contact_manifolds::{self, PREDICTION};
use rapier_golden::types::{ContactPointRaw, ManifoldCase, PoseRaw, ShapeRaw, Vec2Raw};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;

const TOL: u64 = 64;
const TOL_CAPSULE: u64 = 4;

fn vector(v: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: v.x }, y: Fixed { raw: v.y } }
}

fn pose(p: PoseRaw) -> Pose2 {
    Pose2Trait::new(
        vector(p.translation),
        Rot2 { re: Fixed { raw: p.rotation.re }, im: Fixed { raw: p.rotation.im } },
    )
}

fn shape(s: ShapeRaw) -> Shape {
    match s {
        ShapeRaw::Ball(r) => Shape::Ball(Ball { radius: Fixed { raw: r } }),
        ShapeRaw::Cuboid(h) => Shape::Cuboid(Cuboid { half_extents: vector(h) }),
        ShapeRaw::Capsule(c) => Shape::Capsule(
            Capsule {
                segment: Segment { a: vector(c.a), b: vector(c.b) },
                radius: Fixed { raw: c.radius },
            },
        ),
        ShapeRaw::HalfSpace(n) => Shape::HalfSpace(HalfSpace { normal: vector(n) }),
        ShapeRaw::Segment(s) => Shape::Segment(Segment { a: vector(s.a), b: vector(s.b) }),
    }
}

fn prediction() -> Fixed {
    Fixed { raw: PREDICTION }
}

/// Dispatches a recorded case; `swapped` exchanges the two shapes and inverts `pos12`.
fn run(case: ManifoldCase, swapped: bool) -> (ContactManifold, bool) {
    let mut m: ContactManifold = Default::default();
    let ok = if swapped {
        contact_manifold(
            pose(case.pos12).inverse(), shape(case.shape2), shape(case.shape1), prediction(), ref m,
        )
    } else {
        contact_manifold(
            pose(case.pos12), shape(case.shape1), shape(case.shape2), prediction(), ref m,
        )
    };
    (m, ok)
}

fn is_capsule(s: ShapeRaw) -> bool {
    match s {
        ShapeRaw::Capsule(_) => true,
        _ => false,
    }
}

fn is_cuboid(s: ShapeRaw) -> bool {
    match s {
        ShapeRaw::Cuboid(_) => true,
        _ => false,
    }
}

/// The capsule–capsule and cuboid–capsule generators are the tight ones.
fn tolerance(case: ManifoldCase, swapped: bool) -> u64 {
    let capsule_pair = (is_capsule(case.shape1)
        && (is_capsule(case.shape2) || is_cuboid(case.shape2)))
        || (is_cuboid(case.shape1) && is_capsule(case.shape2));
    if capsule_pair && !swapped {
        TOL_CAPSULE
    } else {
        TOL
    }
}

/// Cuboid–capsule pairs (either order): points matched by feature ids, not by position in the
/// list.
fn unordered(case: ManifoldCase) -> bool {
    (is_capsule(case.shape1) && is_cuboid(case.shape2))
        || (is_cuboid(case.shape1) && is_capsule(case.shape2))
}

/// Segment coincident with a cuboid face: the SAT axes tie, the reference face is shape 1's, so the
/// swapped manifold is not the mirror image of the recorded one (points and ids). Only the count
/// and the distances are compared when swapped.
fn tie_axes(id: felt252) -> bool {
    id == 'cuboid_segment/within_pred'
        || id == 'cuboid_segment/touching'
        || id == 'cuboid_segment/deep'
        || id == 'segment_cuboid/shallow'
}

/// Cases whose feature ids depend on which shape supplies a tied vertex: not compared when swapped.
fn tie_ids(id: felt252) -> bool {
    id == 'cuboid_cuboid/touching' || id == 'cuboid_cuboid/degen_corner'
}

fn vec_ok(a: Vec2, b: Vec2Raw, tol: u64) -> bool {
    within(a.x.raw, b.x, tol) && within(a.y.raw, b.y, tol)
}

/// `actual` against the recorded point, mirrored when `swapped`.
fn point_ok(
    actual: TrackedContact, e: ContactPointRaw, swapped: bool, ids: bool, tol: u64,
) -> bool {
    let (e1, e2, f1, f2) = if swapped {
        (e.local_p2, e.local_p1, e.fid2, e.fid1)
    } else {
        (e.local_p1, e.local_p2, e.fid1, e.fid2)
    };
    vec_ok(actual.local_p1, e1, tol)
        && vec_ok(actual.local_p2, e2, tol)
        && within(actual.dist.raw, e.dist, tol)
        && (!ids || (actual.fid1.packed == f1 && actual.fid2.packed == f2))
}

fn dist_of(m: ContactManifold, i: u8) -> Fixed {
    m.point(i).dist
}

fn check_ambiguous(m: ContactManifold, case: ManifoldCase, tol: u64) {
    let [e0, e1] = case.points;
    if m.num_points == 1 {
        assert!(within(dist_of(m, 0).raw, e0.dist, tol), "{}: dist", case.id);
    } else if m.num_points == 2 {
        let straight = within(dist_of(m, 0).raw, e0.dist, tol)
            && within(dist_of(m, 1).raw, e1.dist, tol);
        let crossed = within(dist_of(m, 0).raw, e1.dist, tol)
            && within(dist_of(m, 1).raw, e0.dist, tol);
        assert!(straight || crossed, "{}: dists", case.id);
    }
}

fn check(case: ManifoldCase, m: ContactManifold, swapped: bool) {
    let id = case.id;
    let tol = tolerance(case, swapped);
    assert_eq!(m.num_points.into(), case.num_points, "{}: num_points", id);
    if m.num_points == 0 {
        return;
    }
    if case.ambiguous || (swapped && tie_axes(id)) {
        check_ambiguous(m, case, tol);
        return;
    }
    let (n1, n2) = if swapped {
        (case.local_n2, case.local_n1)
    } else {
        (case.local_n1, case.local_n2)
    };
    assert!(vec_ok(m.local_n1, n1, tol), "{}: n1", id);
    assert!(vec_ok(m.local_n2, n2, tol), "{}: n2", id);
    let ids = !(swapped && tie_ids(id));
    let expected = case.points;
    let mut i = 0_u8;
    while i != m.num_points {
        let p = m.point(i);
        let ok = if swapped || unordered(case) {
            let mut found = false;
            let mut j = 0_u32;
            while j != case.num_points {
                found = found || point_ok(p, *expected.span().at(j), swapped, ids, tol);
                j += 1;
            }
            found
        } else {
            point_ok(p, *expected.span().at(i.into()), false, ids, tol)
        };
        assert!(ok, "{}: point {}", id, i);
        i += 1;
    }
}

#[test]
fn test_every_golden_case_in_recorded_order() {
    let mut count = 0_u32;
    for case in contact_manifolds::cases() {
        let (m, ok) = run(*case, false);
        assert!(ok, "{}: supported", *case.id);
        check(*case, m, false);
        count += 1;
    }
    assert_eq!(count, 87);
}

#[test]
fn test_every_golden_case_with_the_shapes_swapped() {
    let mut count = 0_u32;
    for case in contact_manifolds::cases() {
        let (m, ok) = run(*case, true);
        assert!(ok, "{}: supported swapped", *case.id);
        check(*case, m, true);
        count += 1;
    }
    assert_eq!(count, 87);
}

/// Pairs without a generator: `false`, and a manifold that held points is cleared.
#[test]
fn test_unsupported_pairs_return_false_and_clear() {
    let seg = Segment {
        a: Vec2 { x: Fixed { raw: -1 }, y: Fixed { raw: 0 } },
        b: Vec2 { x: Fixed { raw: 1 }, y: Fixed { raw: 0 } },
    };
    let cap = Capsule { segment: seg, radius: Fixed { raw: 1 } };
    let hs = HalfSpace { normal: Vec2 { x: Fixed { raw: 0 }, y: Fixed { raw: 4294967296 } } };
    let live = run(contact_manifolds::BALL_BALL_SHALLOW, false);
    let (live, _) = live;
    assert_eq!(live.num_points, 1);
    let table = array![
        (Shape::Segment(seg), Shape::Segment(seg)), (Shape::Segment(seg), Shape::Capsule(cap)),
        (Shape::Capsule(cap), Shape::Segment(seg)), (Shape::HalfSpace(hs), Shape::HalfSpace(hs)),
    ];
    for (s1, s2) in table.span() {
        let mut m = live;
        assert!(
            !contact_manifold(
                pose(contact_manifolds::BALL_BALL_SHALLOW.pos12), *s1, *s2, prediction(), ref m,
            ),
        );
        assert_eq!(m.num_points, 0);
    }
}

/// Recorded order, inlined into the probe: the dispatcher is charged the arm it takes only when
/// its caller is not an outlined function like `run`.
#[inline(always)]
fn probe(case: ManifoldCase) -> ContactManifold {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold(
        pose(case.pos12), shape(case.shape1), shape(case.shape2), prediction(), ref m,
    );
    m
}

#[test]
fn gas_baseline() {
    let _ = opaque(contact_manifolds::BALL_BALL_SHALLOW);
}

/// Conversion of the raw case only, to subtract from the `gas_golden_*` figures.
#[test]
fn gas_golden_conversion() {
    let case = opaque(contact_manifolds::CUBOID_CUBOID_SHALLOW);
    let _ = (pose(case.pos12), shape(case.shape1), shape(case.shape2));
}

#[test]
fn gas_golden_ball_ball() {
    let _ = probe(opaque(contact_manifolds::BALL_BALL_SHALLOW));
}

#[test]
fn gas_golden_cuboid_cuboid() {
    let _ = probe(opaque(contact_manifolds::CUBOID_CUBOID_SHALLOW));
}

#[test]
fn gas_golden_capsule_capsule() {
    let _ = probe(opaque(contact_manifolds::CAPSULE_CAPSULE_SHALLOW));
}

#[test]
fn gas_golden_ball_cuboid() {
    let _ = probe(opaque(contact_manifolds::BALL_CUBOID_SHALLOW));
}

#[test]
fn gas_golden_cuboid_ball() {
    let _ = probe(opaque(contact_manifolds::CUBOID_BALL_SHALLOW));
}

#[test]
fn gas_golden_capsule_ball() {
    let _ = probe(opaque(contact_manifolds::CAPSULE_BALL_SHALLOW));
}

#[test]
fn gas_golden_halfspace_ball() {
    let _ = probe(opaque(contact_manifolds::HALFSPACE_BALL_SHALLOW));
}

#[test]
fn gas_golden_cuboid_capsule() {
    let _ = probe(opaque(contact_manifolds::CUBOID_CAPSULE_SHALLOW));
}

#[test]
fn gas_golden_capsule_cuboid() {
    let _ = probe(opaque(contact_manifolds::CAPSULE_CUBOID_SHALLOW));
}

#[test]
fn gas_golden_cuboid_segment() {
    let _ = probe(opaque(contact_manifolds::CUBOID_SEGMENT_SHALLOW));
}

#[test]
fn gas_golden_segment_cuboid() {
    let _ = probe(opaque(contact_manifolds::SEGMENT_CUBOID_SHALLOW));
}

#[test]
fn gas_golden_halfspace_cuboid() {
    let _ = probe(opaque(contact_manifolds::HALFSPACE_CUBOID_SHALLOW));
}

#[test]
fn gas_golden_halfspace_capsule() {
    let _ = probe(opaque(contact_manifolds::HALFSPACE_CAPSULE_SHALLOW));
}

#[test]
fn gas_golden_halfspace_segment() {
    let _ = probe(opaque(contact_manifolds::HALFSPACE_SEGMENT_SHALLOW));
}

#[test]
fn gas_golden_capsule_halfspace() {
    let _ = probe(opaque(contact_manifolds::CAPSULE_HALFSPACE_SHALLOW));
}

#[test]
fn gas_golden_segment_halfspace() {
    let _ = probe(opaque(contact_manifolds::SEGMENT_HALFSPACE_SHALLOW));
}
