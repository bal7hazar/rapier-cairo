//! Unit tests of the narrow-phase kernels and the carry-over candidates. Cross-module scenes
//! live in `tests/narrow_phase_scenes.cairo`.

use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::collider::events::{COLLISION_EVENTS, REMOVED, SENSOR};
use rapier_core::collider::{ActiveEventsTrait, CollisionEventFlagsTrait};
use rapier_core::interaction_groups::{
    ALL, GROUP_1, GROUP_2, InteractionGroupsTrait, InteractionTestMode,
};
use rapier_core::rigid_body::RigidBodyType;
use rapier_geometry2d::contact::{ContactData, NEW_CONTACT_BIT, TrackedContact};
use rapier_geometry2d::shape::{BallTrait, Shape};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::collider::ColliderBuilderTrait;
use crate::collider_set::{ColliderSet, ColliderSetTrait};
use crate::events::{
    CollisionEvent, CollisionEventTrait, PairEventStatus, PairEventStatusTrait, START_EVENT_EMITTED,
    started, stopped,
};
use crate::rigid_body_set::{RigidBodySet, RigidBodySetTrait, RigidBodyTrait};
use super::alternatives::{compute_contacts_dict, pair_key};
use super::mock::{MockDispatcher, OVERLAP, PREDICTION, RADIUS, at, ball, broad_phase, scene};
use super::{
    CarryOver, ContactPair, ContactPairTrait, NarrowPhase, NarrowPhaseTrait, PairCollider,
    SortedMerge, dropped_events, key_before, pair_collider, pair_filtered, process_pair,
    solver_contact, update_manifold,
};

fn h(index: u32) -> Handle {
    Handle { index, generation: 0 }
}

/// A solid collider on a body of type `body_type`, handle `h(index)`, parent `h(body)`.
fn side(index: u32, body: u32, body_type: RigidBodyType) -> PairCollider {
    let mut bodies = RigidBodySetTrait::new();
    let collider = ColliderBuilderTrait::ball(RADIUS).build();
    let mut co = pair_collider(h(index), collider, ref bodies);
    co.body = Some(h(body));
    co.body_type = body_type;
    co
}

#[test]
fn test_pair_filtered() {
    let dynamic = RigidBodyType::Dynamic;
    let fixed = RigidBodyType::Fixed;
    let kinematic = RigidBodyType::KinematicPositionBased;
    // Collider 1 in group 1 only, collider 2 in group 2 and only accepting group 2.
    let member_of_1 = InteractionGroupsTrait::new(GROUP_1, ALL, InteractionTestMode::And);
    let excluded = InteractionGroupsTrait::new(GROUP_2, GROUP_2, InteractionTestMode::And);
    // (type1, type2, same parent, groups of collider 2 exclude collider 1, filtered)
    let cases: Array<(RigidBodyType, RigidBodyType, bool, bool, bool)> = array![
        (dynamic, dynamic, false, false, false), (dynamic, fixed, false, false, false),
        (dynamic, kinematic, false, false, false), (kinematic, fixed, false, false, true),
        (fixed, kinematic, false, false, true), (kinematic, kinematic, false, false, true),
        (dynamic, dynamic, true, false, true), (dynamic, fixed, false, true, true),
    ];
    for (type1, type2, same_parent, exclude, expected) in cases {
        let mut co1 = side(1, 10, type1);
        co1.collision_groups = member_of_1;
        let mut co2 = side(2, if same_parent {
            10
        } else {
            11
        }, type2);
        if exclude {
            co2.collision_groups = excluded;
        }
        assert_eq!(pair_filtered(co1, co2), expected);
    }
    // Standalone colliders never share a parent.
    let mut co1 = side(1, 10, dynamic);
    let mut co2 = side(2, 10, dynamic);
    co1.body = None;
    co2.body = None;
    assert!(!pair_filtered(co1, co2));
}

#[test]
fn test_solver_contact_anchors_and_new_bit() {
    let mut co1 = side(1, 10, RigidBodyType::Dynamic);
    let mut co2 = side(2, 11, RigidBodyType::Dynamic);
    // Collider 1 turned a quarter and moved to (1, 0), centre of mass (1, 1); collider 2 at
    // (0, 2), standalone-like centre of mass at the origin.
    co1
        .pose =
            Pose2 { translation: Vec2 { x: ONE, y: ZERO }, rotation: Rot2 { re: ZERO, im: ONE } };
    co1.world_com = Vec2 { x: ONE, y: ONE };
    co2.pose = at(ZERO, TWO);
    co2.world_com = Vec2 { x: ZERO, y: ZERO };
    let point = TrackedContact {
        local_p1: Vec2 { x: ONE, y: ZERO },
        local_p2: Vec2 { x: ZERO, y: -ONE },
        dist: -HALF,
        ..Default::default(),
    };
    // (impulse, contact id)
    let cases: Array<(Fixed, u32)> = array![(ZERO, 1 + NEW_CONTACT_BIT), (HALF, 1)];
    for (impulse, id) in cases {
        let mut p = point;
        p.data = ContactData { impulse, ..Default::default() };
        let sc = solver_contact(p, 1, co1, co2);
        // World point 1: (1, 0) + rot90 (1, 0) = (1, 1); relative to (1, 1): zero.
        assert_eq!(sc.anchor1, Vec2 { x: ZERO, y: ZERO });
        assert_eq!(sc.anchor2, Vec2 { x: ZERO, y: ONE });
        assert_eq!(sc.dist, -HALF);
        assert_eq!(sc.contact_id, id);
    }
}

#[test]
fn test_update_manifold_prediction_boundary_and_data() {
    // Ball above a halfspace, gap `g`; the mock generates a point for g <= prediction, the
    // narrow phase keeps a solver contact for g < prediction only (upstream strictness).
    let mut bodies = RigidBodySetTrait::new();
    let mut colliders = ColliderSetTrait::new();
    let (_, _) = super::mock::ground(ref bodies, ref colliders);
    let ball_handle = ball(ref bodies, ref colliders, ZERO, ONE);
    let scratch = super::pair_colliders(ref bodies, ref colliders);
    let ground = *scratch.at(0);
    let mut co2 = *scratch.at(1);
    assert_eq!(co2.handle, ball_handle);
    // (gap, points, solver contacts)
    let cases: Array<(Fixed, u8, u8)> = array![
        (-OVERLAP, 1, 1), (ZERO, 1, 1), (PREDICTION, 1, 0), (PREDICTION + PREDICTION, 0, 0),
    ];
    for (gap, points, contacts) in cases {
        co2.pose = at(ZERO, RADIUS + gap);
        let m = update_manifold::<MockDispatcher>(PREDICTION, ground, co2, Default::default());
        assert_eq!(m.num_points, points);
        assert_eq!(m.data.num_solver_contacts, contacts);
        assert_eq!(m.data.normal, Vec2 { x: ZERO, y: ONE });
        assert_eq!(m.data.rigid_body2, co2.body);
        assert_eq!(m.data.solver_flags.bits, 1);
        // Fixed ground vs dynamic ball: effective groups 128 and 0.
        assert_eq!(m.data.relative_dominance, 128);
        // Frictions 0.5 and 0.5, `Average`.
        assert_eq!(m.data.friction, HALF);
    }
}

#[test]
fn test_unsupported_pair_and_solver_groups() {
    let mut co1 = side(1, 10, RigidBodyType::Dynamic);
    let mut co2 = side(2, 11, RigidBodyType::Dynamic);
    co2.shape = Shape::Ball(BallTrait::new(RADIUS));
    co1.shape = ColliderBuilderTrait::cuboid(HALF, HALF).build().shape;
    co1.pose = at(ZERO, ZERO);
    co2.pose = at(ZERO, HALF);
    let mut previous: ContactPair = ContactPairTrait::new(h(1), h(2));
    previous.manifold.num_points = 1;
    let m = update_manifold::<MockDispatcher>(PREDICTION, co1, co2, previous.manifold);
    assert_eq!(m.num_points, 0);
    assert_eq!(m.data.num_solver_contacts, 0);
    // Solver groups that do not match clear `COMPUTE_RIGID_IMPULSES` (contacts are kept).
    let mut co1 = side(1, 10, RigidBodyType::Dynamic);
    let mut co2 = side(2, 11, RigidBodyType::Dynamic);
    co1.pose = at(ZERO, ZERO);
    co2.pose = at(ZERO, ONE - OVERLAP);
    co1.solver_groups = InteractionGroupsTrait::new(GROUP_1, ALL, InteractionTestMode::And);
    co2.solver_groups = InteractionGroupsTrait::new(GROUP_2, GROUP_2, InteractionTestMode::And);
    let m = update_manifold::<MockDispatcher>(PREDICTION, co1, co2, Default::default());
    assert_eq!(m.data.solver_flags.bits, 0);
    assert_eq!(m.data.num_solver_contacts, 1);
}

#[test]
fn test_process_pair_transitions() {
    let mut co1 = side(1, 10, RigidBodyType::Dynamic);
    let mut co2 = side(2, 11, RigidBodyType::Dynamic);
    co1.pose = at(ZERO, ZERO);
    let touching = at(ZERO, ONE - OVERLAP);
    let apart = at(ZERO, TWO);
    // (events enabled, previous touching, now touching, expected event)
    let none: Option<CollisionEvent> = None;
    let cases: Array<(bool, bool, bool, Option<CollisionEvent>)> = array![
        (true, false, true, Some(started(h(1), h(2)))),
        (true, true, false, Some(stopped(h(1), h(2), CollisionEventFlagsTrait::empty()))),
        (true, true, true, none), (true, false, false, none), (false, false, true, none),
    ];
    for (events, was, is, expected) in cases {
        co2.active_events = if events {
            COLLISION_EVENTS
        } else {
            ActiveEventsTrait::empty()
        };
        co2.pose = if was {
            touching
        } else {
            apart
        };
        let (first, _) = process_pair::<MockDispatcher>(PREDICTION, co1, co2, None);
        co2.pose = if is {
            touching
        } else {
            apart
        };
        let (second, event) = process_pair::<MockDispatcher>(PREDICTION, co1, co2, Some(first));
        assert_eq!(event, expected);
        assert_eq!(second.has_any_active_contact(), is);
        assert_eq!(second.event_status.start_event_emitted(), events && is);
    }
}

#[test]
fn test_sorted_merge_take() {
    let pair = |a: Handle, b: Handle| -> ContactPair {
        ContactPairTrait::new(a, b)
    };
    let stale = Handle { index: 3, generation: 1 };
    let previous = array![
        pair(h(0), h(1)), pair(h(0), h(2)), pair(h(1), h(3)), pair(h(2), stale), pair(h(4), h(5)),
    ];
    let mut merge: SortedMerge = CarryOver::begin(previous.span());
    assert!(merge.take(h(0), h(2)).is_some());
    assert!(merge.take(h(1), h(2)).is_none());
    assert!(merge.take(h(1), h(3)).is_some());
    // Same slots as a previous pair, other generation: a new pair, the old one is dropped.
    assert!(merge.take(h(2), h(3)).is_none());
    let dropped = merge.finish();
    let expected = array![(h(0), h(1)), (h(2), stale), (h(4), h(5))];
    assert_eq!(dropped.len(), expected.len());
    let mut i = 0;
    for (a, b) in expected {
        assert_eq!(*dropped.at(i).collider1, a);
        assert_eq!(*dropped.at(i).collider2, b);
        i += 1;
    }
    assert!(key_before(h(0), h(9), h(1), h(0)));
    assert!(!key_before(h(1), h(2), h(1), h(2)));
    assert_ne!(pair_key(h(1), h(2)), pair_key(h(2), h(1)));
}

#[test]
fn test_dropped_events_flags() {
    let mut bodies = RigidBodySetTrait::new();
    let mut colliders = ColliderSetTrait::new();
    let a = ball(ref bodies, ref colliders, ZERO, ZERO);
    let b = ball(ref bodies, ref colliders, TWO, ZERO);
    let gone = Handle { index: 7, generation: 0 };
    let mut touching: ContactPair = ContactPairTrait::new(a, b);
    touching.event_status = START_EVENT_EMITTED;
    let mut removed = ContactPairTrait::new(a, gone);
    removed.event_status = START_EVENT_EMITTED;
    let quiet = ContactPairTrait::new(b, gone);
    let events = dropped_events(array![touching, removed, quiet], ref colliders);
    assert_eq!(
        events, array![stopped(a, b, CollisionEventFlagsTrait::empty()), stopped(a, gone, REMOVED)],
    );
    assert!(events.at(1).removed());
}

/// Moves ball `k` of a stack scene up by `lift` (and propagates to its collider).
fn lift_body(ref bodies: RigidBodySet, ref colliders: ColliderSet, k: u32, lift: Fixed) {
    let (handle, mut body) = *bodies.iter().at(k + 1);
    let mut pose = body.position();
    pose.translation.y = pose.translation.y + lift;
    body.set_position(pose);
    let _ = bodies.set(handle, body);
    bodies.propagate_modified_body_positions_to_colliders(ref colliders);
}

/// Runs the same step sequence with both carry-over candidates and compares pairs and events.
fn assert_candidates_agree(n: u32, lifts: Span<(u32, Fixed)>) {
    let (mut bodies_a, mut colliders_a) = scene(n, true);
    let (mut bodies_b, mut colliders_b) = scene(n, true);
    let mut merge: NarrowPhase = NarrowPhaseTrait::new();
    let mut dict: NarrowPhase = NarrowPhaseTrait::new();
    for (k, lift) in lifts {
        lift_body(ref bodies_a, ref colliders_a, *k, *lift);
        lift_body(ref bodies_b, ref colliders_b, *k, *lift);
        let pairs = broad_phase(ref bodies_a, ref colliders_a, PREDICTION);
        let events_a = merge
            .compute_contacts::<
                MockDispatcher,
            >(PREDICTION, ref bodies_a, ref colliders_a, pairs.span());
        let events_b = compute_contacts_dict::<
            MockDispatcher,
        >(ref dict, PREDICTION, ref bodies_b, ref colliders_b, pairs.span());
        assert_eq!(events_a, events_b);
        assert_eq!(merge.pairs, dict.pairs);
    }
}

#[test]
fn test_candidates_agree_on_a_sequence() {
    // Lift the top ball away (Stopped, pair dropped), put it back (Started), lift a middle one.
    let lifts = array![(2, ZERO), (2, TWO), (2, -TWO), (1, OVERLAP + OVERLAP), (0, ZERO)];
    assert_candidates_agree(3, lifts.span());
}

#[test]
#[fuzzer(runs: 12, seed: 7)]
fn fuzz_carry_over_candidates_agree(k: u8, lift: i16) {
    let k: u32 = (k % 4).into();
    let lift = FixedTrait::from_raw(lift.into() * 0x100000);
    let lifts = array![(k, ZERO), (k, lift), (k, -lift - lift)];
    assert_candidates_agree(4, lifts.span());
}

#[test]
fn test_warm_start_and_events_over_steps() {
    let (mut bodies, mut colliders) = scene(1, true);
    let pairs = broad_phase(ref bodies, ref colliders, PREDICTION);
    let mut narrow_phase: NarrowPhase = NarrowPhaseTrait::new();
    let events = narrow_phase
        .compute_contacts::<MockDispatcher>(PREDICTION, ref bodies, ref colliders, pairs.span());
    let (g, b) = (h(0), h(1));
    assert_eq!(events, array![started(g, b)]);
    let mut pair = *narrow_phase.pairs.at(0);
    assert_eq!(pair.manifold.data.solver_contacts.span().at(0).contact_id, @NEW_CONTACT_BIT);
    // The solver writes an impulse back into the stored point.
    let [mut p0, p1] = pair.manifold.points;
    p0.data.impulse = HALF;
    pair.manifold.points = [p0, p1];
    narrow_phase.pairs = array![pair];
    let events = narrow_phase
        .compute_contacts::<MockDispatcher>(PREDICTION, ref bodies, ref colliders, pairs.span());
    assert_eq!(events.len(), 0);
    let pair = *narrow_phase.pairs.at(0);
    assert_eq!(pair.manifold.data.solver_contacts.span().at(0).contact_id, @0);
    assert_eq!(pair.manifold.points.span().at(0).data.impulse, @HALF);
    // Removing the ball drops the pair with a `REMOVED` stop.
    let _ = colliders.remove(b, ref bodies);
    let pairs = broad_phase(ref bodies, ref colliders, PREDICTION);
    let events = narrow_phase
        .compute_contacts::<MockDispatcher>(PREDICTION, ref bodies, ref colliders, pairs.span());
    assert_eq!(events, array![stopped(g, b, REMOVED)]);
    assert_eq!(narrow_phase.len(), 0);
    assert!(narrow_phase.contact_pair(g, b).is_none());
}

/// `(collider1, collider2, status bits)`: contact (0, 1), sensor pairs (0, 2) intersecting and
/// (1, 2) not, with a start event on the first.
fn mixed_pairs() -> NarrowPhase {
    let mut pairs = array![];
    for (c1, c2, bits) in array![(0, 1, 1_u8), (0, 2, 13), (1, 2, 4)] {
        let mut pair = ContactPairTrait::new(h(c1), h(c2));
        pair.event_status = PairEventStatus { bits };
        pairs.append(pair);
    }
    NarrowPhase { pairs }
}

#[test]
fn test_intersection_queries() {
    let np = mixed_pairs();
    // (collider1, collider2, contact_pair found, intersection_pair)
    let cases = array![
        (0, 1, true, None), (1, 0, false, None), (0, 2, false, Some(true)),
        (2, 0, false, Some(true)), (2, 1, false, Some(false)), (1, 3, false, None),
    ];
    for (c1, c2, contact, intersection) in cases {
        assert_eq!(np.contact_pair(h(c1), h(c2)).is_some(), contact);
        assert_eq!(np.intersection_pair(h(c1), h(c2)), intersection);
    }
    assert_eq!(np.intersection_pairs(), array![(h(0), h(2), true), (h(1), h(2), false)]);
    assert_eq!(np.intersection_pairs_with(h(1)), array![(h(1), h(2), false)]);
    assert_eq!(np.intersection_pairs_with(h(3)), array![]);
    let [contact, sensor, idle] = [*np.pairs[0], *np.pairs[1], *np.pairs[2]];
    assert!(!contact.is_intersection_pair() && !contact.intersecting());
    assert!(sensor.is_intersection_pair() && sensor.intersecting());
    assert!(idle.is_intersection_pair() && !idle.intersecting());
    // A dropped sensor pair whose start was emitted: `SENSOR`, plus `REMOVED` without colliders.
    let mut colliders = ColliderSetTrait::new();
    assert_eq!(
        dropped_events(array![contact, sensor, idle], ref colliders),
        array![stopped(h(0), h(1), REMOVED), stopped(h(0), h(2), SENSOR | REMOVED)],
    );
}

fn h7(index: u32) -> Handle {
    Handle { index, generation: 7 }
}

#[test]
fn test_intersection_queries_unknown_gen() {
    // The same pairs as `mixed_pairs`, but every handle carries generation 7.
    let mut pairs = array![];
    for (c1, c2, bits) in array![(0, 1, 1_u8), (0, 2, 13), (1, 2, 4)] {
        let mut pair = ContactPairTrait::new(h7(c1), h7(c2));
        pair.event_status = PairEventStatus { bits };
        pairs.append(pair);
    }
    let np = NarrowPhase { pairs };
    // (collider1, collider2, expected): a contact pair and an absent pair answer `None`.
    let cases = array![
        (0, 1, None), (1, 0, None), (0, 2, Some(true)), (2, 0, Some(true)), (2, 1, Some(false)),
        (1, 3, None),
    ];
    for (c1, c2, expected) in cases {
        assert_eq!(np.intersection_pair_unknown_gen(c1, c2), expected);
        // The known-generation form does not find them under generation 0.
        assert_eq!(np.intersection_pair(h(c1), h(c2)), None);
    }
    assert_eq!(
        np.intersection_pairs_with_unknown_gen(2),
        array![(h7(0), h7(2), true), (h7(1), h7(2), false)],
    );
    assert_eq!(np.intersection_pairs_with_unknown_gen(1), array![(h7(1), h7(2), false)]);
    assert_eq!(np.intersection_pairs_with_unknown_gen(3), array![]);
}

#[test]
fn gas_intersection_pair_unknown_gen() {
    let np = mixed_pairs();
    let _ = opaque(np.intersection_pair_unknown_gen(opaque(2), 1));
}

#[test]
fn gas_intersection_pairs_with_unknown_gen() {
    let np = mixed_pairs();
    let _ = opaque(np.intersection_pairs_with_unknown_gen(opaque(2)));
}

#[test]
fn gas_intersection_pair() {
    let np = mixed_pairs();
    let _ = opaque(np.intersection_pair(opaque(h(2)), h(1)));
}

#[test]
fn gas_intersection_pairs_with() {
    let np = mixed_pairs();
    let _ = opaque(np.intersection_pairs_with(opaque(h(2))));
}

#[test]
fn gas_intersection_pairs() {
    let np = mixed_pairs();
    let _ = opaque(np.intersection_pairs());
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
fn gas_pair_filtered() {
    let co1 = opaque(side(1, 10, RigidBodyType::Dynamic));
    let co2 = opaque(side(2, 11, RigidBodyType::Fixed));
    assert!(!pair_filtered(co1, co2));
}

#[test]
fn gas_solver_contact() {
    let co1 = opaque(side(1, 10, RigidBodyType::Dynamic));
    let co2 = opaque(side(2, 11, RigidBodyType::Fixed));
    let _ = solver_contact(opaque(Default::default()), 0, co1, co2);
}

#[test]
fn gas_update_manifold_ball_ball() {
    let co1 = side(1, 10, RigidBodyType::Dynamic);
    let mut co2 = side(2, 11, RigidBodyType::Dynamic);
    co2.pose = at(ZERO, ONE - OVERLAP);
    let _ = update_manifold::<
        MockDispatcher,
    >(PREDICTION, opaque(co1), opaque(co2), Default::default());
}

#[test]
fn gas_process_pair_ball_ball() {
    let co1 = side(1, 10, RigidBodyType::Dynamic);
    let mut co2 = side(2, 11, RigidBodyType::Dynamic);
    co2.pose = at(ZERO, ONE - OVERLAP);
    let _ = process_pair::<MockDispatcher>(PREDICTION, opaque(co1), opaque(co2), None);
}

#[test]
fn gas_pair_collider() {
    let mut bodies = RigidBodySetTrait::new();
    let body = bodies.insert(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
    let mut collider = ColliderBuilderTrait::ball(RADIUS).build();
    collider
        .parent =
            Some(crate::collider::ColliderParent { handle: body, pos_wrt_parent: at(ZERO, ZERO) });
    let _ = pair_collider(h(0), opaque(collider), ref bodies);
}

#[test]
fn gas_dropped_events_2() {
    let mut colliders = ColliderSetTrait::new();
    let mut pair: ContactPair = ContactPairTrait::new(h(0), h(1));
    pair.event_status = START_EVENT_EMITTED;
    let _ = dropped_events(opaque(array![pair, pair]), ref colliders);
}
