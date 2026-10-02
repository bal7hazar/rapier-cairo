//! The packed wires' round trips and the gas probes of the previous-pair crossings (CX2).
use rapier2d::pipeline::stages::narrow::{ContactJob, ManifoldGeometry};
use rapier2d::prelude::{Fixed, Handle, Pose2, Rot2, Shape, Vec2};
use rapier_core::collider::{ActiveCollisionTypes, ActiveEvents, CoefficientCombineRule};
use rapier_core::interaction_groups::{Group, InteractionGroups, InteractionTestMode};
use rapier_core::rigid_body::RigidBodyType;
use rapier_dynamics2d::collider::components::OneWayPlatform;
use rapier_dynamics2d::events::PairEventStatus;
use rapier_dynamics2d::narrow_phase::{ContactPair, PairCollider};
use rapier_geometry2d::contact::{
    ContactData, ContactManifold, ContactManifoldData, SolverContact, SolverFlags, TrackedContact,
};
use rapier_geometry2d::feature_id::FeatureId;
use rapier_geometry2d::shape::{Ball, Cuboid, HalfSpace};
use rapier_testing::opaque;
use crate::narrow::{PreviousPair, pair_of, previous_of};
use super::{
    put_collider, put_current, put_job, put_previous, put_result, take_collider, take_current,
    take_job, take_previous, take_result,
};

const MIN: i64 = -0x8000000000000000;
const MAX: i64 = 0x7fffffffffffffff;

fn f(raw: i64) -> Fixed {
    Fixed { raw }
}

fn v(x: i64, y: i64) -> Vec2 {
    Vec2 { x: f(x), y: f(y) }
}

fn point(seed: i64) -> TrackedContact {
    TrackedContact {
        local_p1: v(seed, -seed),
        local_p2: v(MIN, MAX),
        dist: f(-3 * seed),
        fid1: FeatureId { packed: 0xffffffff },
        fid2: FeatureId { packed: 7 },
        data: ContactData {
            impulse: f(seed + 1),
            tangent_impulse: f(-seed - 2),
            warmstart_impulse: f(MAX - seed),
            warmstart_tangent_impulse: f(MIN + seed),
        },
    }
}

fn contact(seed: i64, id: u32) -> SolverContact {
    SolverContact {
        anchor1: v(seed, MIN),
        anchor2: v(-seed, MAX),
        dist: f(seed * 5),
        tangent_velocity: v(-7, seed),
        contact_id: id,
    }
}

fn pairs() -> Array<ContactPair> {
    let manifold = ContactManifold {
        points: [point(12345), point(987654321)],
        num_points: 2,
        local_n1: v(MAX, -1),
        local_n2: v(MIN, 1),
        subshape1: 0xffffffff,
        subshape2: 3,
        data: ContactManifoldData {
            rigid_body1: Some(Handle { index: 0xffffffff, generation: 0xffffffff }),
            rigid_body2: None,
            solver_flags: SolverFlags { bits: 0xffffffff },
            normal: v(-4, 4),
            solver_contacts: [contact(99, 0x80000001), contact(-98, 0xffffffff)],
            num_solver_contacts: 255,
            relative_dominance: -32768,
            user_data: 0xfffffffe,
            friction: f(MIN),
            restitution: f(MAX),
        },
    };
    let mut other = manifold;
    other.data.rigid_body1 = None;
    other.data.rigid_body2 = Some(Handle { index: 3, generation: 0 });
    other.data.relative_dominance = 32767;
    other.data.num_solver_contacts = 0;
    other.num_points = 0;
    array![
        ContactPair {
            collider1: Handle { index: 0, generation: 0xffffffff },
            collider2: Handle { index: 0xffffffff, generation: 1 },
            manifold,
            event_status: PairEventStatus { bits: 255 },
        },
        ContactPair {
            collider1: Handle { index: 5, generation: 6 },
            collider2: Handle { index: 7, generation: 8 },
            manifold: other,
            event_status: PairEventStatus { bits: 0 },
        },
        ContactPair {
            collider1: Default::default(),
            collider2: Default::default(),
            manifold: Default::default(),
            event_status: Default::default(),
        },
    ]
}

fn colliders() -> Array<PairCollider> {
    let groups = InteractionGroups {
        memberships: Group { bits: 0xffffffff },
        filter: Group { bits: 0x80000000 },
        test_mode: InteractionTestMode::Or,
    };
    let a = PairCollider {
        handle: Handle { index: 0xffffffff, generation: 0xffffffff },
        solid: true,
        sensor: false,
        shape: Shape::Cuboid(Cuboid { half_extents: v(MAX, 1) }),
        pose: Pose2 { translation: v(MIN, -5), rotation: Rot2 { re: f(MAX), im: f(-1) } },
        friction: f(-2),
        restitution: f(MAX),
        friction_combine_rule: CoefficientCombineRule::GeometricMean,
        restitution_combine_rule: CoefficientCombineRule::ClampedSum,
        active_collision_types: ActiveCollisionTypes { bits: 0xffff },
        collision_groups: groups,
        solver_groups: Default::default(),
        active_events: ActiveEvents { bits: 0xffffffff },
        one_way: BoxTrait::new(
            Some(OneWayPlatform { local_up: v(0, 1), cos_allowed_angle: f(-9) }),
        ),
        body: Some(Handle { index: 0xffffffff, generation: 0 }),
        body_type: RigidBodyType::KinematicVelocityBased,
        world_com: v(MIN, MAX),
        dominance: -32768,
    };
    let mut b = a;
    b.solid = false;
    b.sensor = true;
    b.shape = Shape::Ball(Ball { radius: f(3) });
    b.friction_combine_rule = CoefficientCombineRule::Average;
    b.restitution_combine_rule = CoefficientCombineRule::Max;
    b.collision_groups = Default::default();
    b.solver_groups = groups;
    b.one_way = BoxTrait::new(None);
    b.body = None;
    b.body_type = RigidBodyType::Dynamic;
    b.dominance = 32767;
    let mut c = b;
    c.shape = Shape::HalfSpace(HalfSpace { normal: v(0, -1) });
    c.body_type = RigidBodyType::Fixed;
    c.friction_combine_rule = CoefficientCombineRule::Min;
    c.restitution_combine_rule = CoefficientCombineRule::Multiply;
    c.dominance = 0;
    let mut d = c;
    d.body_type = RigidBodyType::KinematicPositionBased;
    array![a, b, c, d]
}

fn felts<T, +Serde<T>, +Drop<T>>(value: @T) -> Array<felt252> {
    let mut out = array![];
    value.serialize(ref out);
    out
}

#[test]
fn test_current_round_trip() {
    let mut wire = array![];
    for pair in pairs().span() {
        put_current(ref wire, pair);
    }
    assert_eq!(wire.len(), 3 * 17);
    let mut wire = wire.span();
    for pair in pairs() {
        assert_eq!(take_current(ref wire), pair);
    }
    assert!(wire.is_empty());
}

#[test]
fn test_previous_round_trip() {
    let mut wire = array![];
    for pair in pairs().span() {
        put_previous(ref wire, pair);
    }
    assert_eq!(wire.len(), 3 * 10);
    let mut wire = wire.span();
    for pair in pairs() {
        let mut expected = pair;
        let mut data: ContactManifoldData = Default::default();
        data.num_solver_contacts = pair.manifold.data.num_solver_contacts;
        data.user_data = pair.manifold.data.user_data;
        expected.manifold.data = data;
        assert_eq!(take_previous(ref wire), expected);
    }
    assert!(wire.is_empty());
}

#[test]
fn test_collider_round_trip() {
    let mut wire = array![];
    for collider in colliders().span() {
        put_collider(ref wire, collider);
    }
    let mut wire = wire.span();
    for collider in colliders() {
        assert_eq!(take_collider(ref wire), collider);
    }
    assert!(wire.is_empty());
}

#[test]
fn test_job_and_result_round_trip() {
    let shapes = array![
        Shape::Ball(Ball { radius: f(MAX) }), Shape::Cuboid(Cuboid { half_extents: v(1, 2) }),
    ];
    for pair in pairs() {
        let geometry: ManifoldGeometry = crate::contact::geometry(@pair.manifold);
        let job = ContactJob {
            pos12: Pose2 { translation: v(MIN, MAX), rotation: Rot2 { re: f(-1), im: f(MIN) } },
            shape1: *shapes[0],
            shape2: *shapes[1],
            geometry,
        };
        let mut wire = array![];
        put_job(ref wire, @job);
        put_result(ref wire, true, @geometry);
        put_result(ref wire, false, @geometry);
        let mut wire = wire.span();
        assert_eq!(felts(@take_job(ref wire)), felts(@job));
        let (supported, got) = take_result(ref wire);
        assert!(supported);
        assert_eq!(felts(@got), felts(@geometry));
        let (supported, got) = take_result(ref wire);
        assert!(!supported);
        assert_eq!(felts(@got), felts(@geometry));
        assert!(wire.is_empty());
    }
}

/// `pair_of(previous_of(pair))`: the pair with the default solver data but the contact count
/// and the user data.
#[test]
fn test_previous_pair_keeps_what_the_loop_reads() {
    for pair in pairs() {
        let mut expected = pair;
        let mut data: ContactManifoldData = Default::default();
        data.num_solver_contacts = pair.manifold.data.num_solver_contacts;
        data.user_data = pair.manifold.data.user_data;
        expected.manifold.data = data;
        let mut wire = array![];
        previous_of(@pair).serialize(ref wire);
        assert_eq!(wire.len(), 36);
        let mut wire = wire.span();
        let previous: PreviousPair = Serde::deserialize(ref wire).unwrap();
        assert_eq!(pair_of(previous), expected);
    }
}

// Gas probes: one encode and decode of the three fixture pairs, per candidate.

#[test]
fn gas_baseline() {
    opaque(opaque(pairs()).span());
}

/// CS6: the whole pairs by their `Serde`, 64 felts each.
#[test]
fn gas_previous_whole_serde() {
    let pairs = opaque(pairs());
    let mut wire = array![];
    pairs.serialize(ref wire);
    let mut wire = opaque(wire.span());
    let back: Array<ContactPair> = Serde::deserialize(ref wire).unwrap();
    opaque(back);
}

/// The winner: `PreviousPair` by its `Serde`, 36 felts each.
#[test]
fn gas_previous_trimmed_serde() {
    let pairs = opaque(pairs());
    let mut wire = array![];
    wire.append(pairs.len().into());
    for pair in pairs.span() {
        previous_of(pair).serialize(ref wire);
    }
    let mut wire = opaque(wire.span());
    let previous: Array<PreviousPair> = Serde::deserialize(ref wire).unwrap();
    let mut back = array![];
    for pair in previous {
        back.append(pair_of(pair));
    }
    opaque(back);
}

/// The same fields packed, 10 felts each.
#[test]
fn gas_previous_lanes() {
    let pairs = opaque(pairs());
    let mut wire = array![];
    for pair in pairs.span() {
        put_previous(ref wire, pair);
    }
    let mut wire = opaque(wire.span());
    let mut back = array![];
    while !wire.is_empty() {
        back.append(take_previous(ref wire));
    }
    opaque(back);
}

/// The whole pairs packed, 17 felts each (against `gas_previous_whole_serde`).
#[test]
fn gas_current_lanes() {
    let pairs = opaque(pairs());
    let mut wire = array![];
    for pair in pairs.span() {
        put_current(ref wire, pair);
    }
    let mut wire = opaque(wire.span());
    let mut back = array![];
    while !wire.is_empty() {
        back.append(take_current(ref wire));
    }
    opaque(back);
}

/// CX3's rejected new-pair record rebuilds the pair it was made of.
#[test]
fn test_new_pair_round_trip() {
    let colliders = colliders();
    let (co1, co2) = (colliders.at(0), colliders.at(1));
    for pair in pairs() {
        let mut pair = pair;
        pair.collider1 = *co1.handle;
        pair.collider2 = *co2.handle;
        pair.manifold.data.rigid_body1 = *co1.body;
        pair.manifold.data.rigid_body2 = *co2.body;
        pair.manifold.data.num_solver_contacts = 1;
        let mut felts = array![];
        super::new_contact(pair.manifold, pair.event_status).serialize(ref felts);
        let mut felts = felts.span();
        let record = Serde::deserialize(ref felts).unwrap();
        assert(super::contact_pair(record, co1, co2) == pair, 'new pair');
    }
}
