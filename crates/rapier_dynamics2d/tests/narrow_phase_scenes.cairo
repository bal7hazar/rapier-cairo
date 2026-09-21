//! Cross-module narrow-phase scenes: body and collider sets, broad phase, `compute_contacts`.
//! Boxes rest on a halfspace; the dispatcher is a mock with the exact axis-aligned
//! halfspace–cuboid manifold (work package GG replaces it in the pipeline).

use fixed::{Fixed, HALF, ONE, TWO, ZERO};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::collider::events::COLLISION_EVENTS;
use rapier_core::collider::{CoefficientCombineRule, CollisionEventFlagsTrait};
use rapier_core::interaction_groups::{
    ALL, GROUP_1, GROUP_2, InteractionGroupsTrait, InteractionTestMode,
};
use rapier_dynamics2d::collider::{ColliderBuilderTrait, ColliderTrait};
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::events::{CollisionEvent, started, stopped};
use rapier_dynamics2d::narrow_phase::{
    ContactDispatcher, ContactPair, ContactPairTrait, NarrowPhase, NarrowPhaseTrait,
};
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet, RigidBodySetTrait, RigidBodyTrait};
use rapier_geometry2d::broad_phase::find_pairs;
use rapier_geometry2d::contact::{ContactManifold, NEW_CONTACT_BIT, SolverContact, TrackedContact};
use rapier_geometry2d::feature_id::FeatureIdTrait;
use rapier_geometry2d::manifold::ManifoldTrait;
use rapier_geometry2d::shape::Shape;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::IDENTITY;
use rapier_testing::opaque;

/// Upstream's default prediction distance, `0.002`.
const PREDICTION: Fixed = Fixed { raw: 8589935 };
/// Penetration of the resting boxes, `1/256`.
const OVERLAP: Fixed = Fixed { raw: 0x1000000 };
/// Friction of box B, combined with `Max`.
const GRIPPY: Fixed = Fixed { raw: 0xCCCCCCCC };

/// Exact manifold of an upward halfspace (shape 1) and an axis-aligned cuboid (shape 2).
impl HalfspaceCuboid of ContactDispatcher {
    fn contact_manifold(
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) -> bool {
        let (Shape::HalfSpace(_), Shape::Cuboid(c)) = (shape1, shape2) else {
            return false;
        };
        let old = manifold;
        let mut new: ContactManifold = Default::default();
        new.data = old.data;
        new.local_n1 = Vec2 { x: ZERO, y: ONE };
        new.local_n2 = Vec2 { x: ZERO, y: -ONE };
        let h = c.half_extents;
        let mut points: Array<TrackedContact> = array![];
        for (x, id) in array![(-h.x, 0_u32), (h.x, 1_u32)] {
            let v = Vec2 { x, y: -h.y };
            let p = pos12.transform_point(v);
            if p.y <= prediction {
                points
                    .append(
                        TrackedContact {
                            local_p1: Vec2 { x: p.x, y: ZERO },
                            local_p2: v,
                            dist: p.y,
                            fid1: FeatureIdTrait::face(0),
                            fid2: FeatureIdTrait::vertex(id),
                            data: Default::default(),
                        },
                    );
            }
        }
        let n = points.len();
        if n == 2 {
            new.points = [*points.at(0), *points.at(1)];
        } else if n == 1 {
            new.points = [*points.at(0), Default::default()];
        }
        new.num_points = n.try_into().unwrap();
        new.match_contacts(@old);
        manifold = new;
        true
    }
}

fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2 { translation: Vec2 { x, y }, rotation: IDENTITY }
}

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

#[derive(Drop)]
struct Scene {
    ground_body: Handle,
    ground: Handle,
    box_a: Handle,
    box_b: Handle,
}

/// Ground (collider 0, events on), box A at x = -2 and box B at x = 2 (friction 0.8, `Max`),
/// both unit boxes resting `OVERLAP` deep.
fn two_boxes(ref bodies: RigidBodySet, ref colliders: ColliderSet) -> Scene {
    let ground_body = bodies.insert(RigidBodyTrait::fixed(at(ZERO, ZERO)));
    let ground = colliders
        .insert_with_parent(
            ColliderBuilderTrait::halfspace(v(ZERO, ONE)).active_events(COLLISION_EVENTS).build(),
            ground_body,
            ref bodies,
        );
    let box_a = add_box(
        ref bodies, ref colliders, RigidBodyTrait::dynamic(at(-TWO, HALF - OVERLAP)),
    );
    let body_b = bodies.insert(RigidBodyTrait::dynamic(at(TWO, HALF - OVERLAP)));
    let box_b = colliders
        .insert_with_parent(
            ColliderBuilderTrait::cuboid(HALF, HALF)
                .friction(GRIPPY)
                .friction_combine_rule(CoefficientCombineRule::Max)
                .build(),
            body_b,
            ref bodies,
        );
    Scene { ground_body, ground, box_a, box_b }
}

fn add_box(ref bodies: RigidBodySet, ref colliders: ColliderSet, body: RigidBody) -> Handle {
    let body = bodies.insert(body);
    colliders.insert_with_parent(ColliderBuilderTrait::cuboid(HALF, HALF).build(), body, ref bodies)
}

fn step(
    ref narrow_phase: NarrowPhase, ref bodies: RigidBodySet, ref colliders: ColliderSet,
) -> Array<CollisionEvent> {
    let proxies = colliders.broad_phase_proxies(ref bodies, PREDICTION);
    let pairs = find_pairs(proxies.span());
    narrow_phase
        .compute_contacts::<HalfspaceCuboid>(PREDICTION, ref bodies, ref colliders, pairs.span())
}

fn pair_of(narrow_phase: @NarrowPhase, c1: Handle, c2: Handle) -> ContactPair {
    narrow_phase.contact_pair(c1, c2).expect('missing pair')
}

fn contacts(pair: ContactPair) -> Array<SolverContact> {
    let [s0, s1] = pair.manifold.data.solver_contacts;
    let n = pair.manifold.data.num_solver_contacts;
    if n == 2 {
        array![s0, s1]
    } else if n == 1 {
        array![s0]
    } else {
        array![]
    }
}

#[test]
fn test_two_boxes_on_a_halfspace() {
    let mut bodies = RigidBodySetTrait::new();
    let mut colliders = ColliderSetTrait::new();
    let s = two_boxes(ref bodies, ref colliders);
    let mut narrow_phase: NarrowPhase = NarrowPhaseTrait::new();
    let events = step(ref narrow_phase, ref bodies, ref colliders);
    assert_eq!(events, array![started(s.ground, s.box_a), started(s.ground, s.box_b)]);
    assert_eq!(narrow_phase.len(), 2);
    // (box, x centre, combined friction)
    let cases = array![(s.box_a, -TWO, HALF), (s.box_b, TWO, GRIPPY)];
    for (collider, x, friction) in cases {
        let pair = pair_of(@narrow_phase, s.ground, collider);
        let data = pair.manifold.data;
        assert_eq!(pair.manifold.num_points, 2);
        assert_eq!(data.rigid_body1, Some(s.ground_body));
        assert_eq!(data.rigid_body2, colliders.get(collider).unwrap().parent.map(|p| p.handle));
        assert_eq!(data.normal, v(ZERO, ONE));
        assert_eq!(data.friction, friction);
        assert_eq!(data.relative_dominance, 128);
        assert_eq!(data.solver_flags.bits, 1);
        // Anchors: world points relative to each body's centre of mass (ground at the origin,
        // box at its centre), bottom-left point first.
        let expected = array![(-HALF, 0_u32), (HALF, 1_u32)];
        let mut i = 0;
        for (dx, id) in expected {
            let sc = *contacts(pair).at(i);
            assert_eq!(sc.anchor1, v(x + dx, ZERO));
            assert_eq!(sc.anchor2, v(dx, -HALF));
            assert_eq!(sc.dist, -OVERLAP);
            assert_eq!(sc.contact_id, id + NEW_CONTACT_BIT);
            i += 1;
        }
    }
    // Lift box B beyond the prediction distance (the halfspace AABB keeps the pair alive).
    let body_b = colliders.get(s.box_b).unwrap().parent.unwrap().handle;
    let mut body = bodies.get(body_b).unwrap();
    body.set_position(at(TWO, HALF + PREDICTION + PREDICTION));
    let _ = bodies.set(body_b, body);
    bodies.propagate_modified_body_positions_to_colliders(ref colliders);
    let events = step(ref narrow_phase, ref bodies, ref colliders);
    assert_eq!(events, array![stopped(s.ground, s.box_b, CollisionEventFlagsTrait::empty())]);
    let pair = pair_of(@narrow_phase, s.ground, s.box_b);
    assert!(!pair.has_any_active_contact());
    assert!(pair_of(@narrow_phase, s.ground, s.box_a).has_any_active_contact());
}

#[test]
fn test_filtered_pairs() {
    let mut bodies = RigidBodySetTrait::new();
    let mut colliders = ColliderSetTrait::new();
    let s = two_boxes(ref bodies, ref colliders);
    // A sensor box, a box whose groups exclude the ground, a kinematic box: all on the ground.
    let sensor_body = bodies.insert(RigidBodyTrait::dynamic(at(ZERO, HALF - OVERLAP)));
    let sensor = colliders
        .insert_with_parent(
            ColliderBuilderTrait::cuboid(HALF, HALF).sensor(true).build(), sensor_body, ref bodies,
        );
    let lonely_body = bodies.insert(RigidBodyTrait::dynamic(at(TWO + TWO, HALF - OVERLAP)));
    let groups = InteractionGroupsTrait::new(GROUP_2, GROUP_2, InteractionTestMode::And);
    let mut ground = colliders.get(s.ground).unwrap();
    ground
        .set_collision_groups(InteractionGroupsTrait::new(GROUP_1, ALL, InteractionTestMode::And));
    let _ = colliders.set(s.ground, ground);
    let lonely = colliders
        .insert_with_parent(
            ColliderBuilderTrait::cuboid(HALF, HALF).collision_groups(groups).build(),
            lonely_body,
            ref bodies,
        );
    let kinematic = add_box(
        ref bodies,
        ref colliders,
        RigidBodyTrait::kinematic_position_based(at(-TWO - TWO, HALF - OVERLAP)),
    );
    let mut narrow_phase: NarrowPhase = NarrowPhaseTrait::new();
    let events = step(ref narrow_phase, ref bodies, ref colliders);
    assert_eq!(events, array![started(s.ground, s.box_a), started(s.ground, s.box_b)]);
    // The sensor takes part in no contact pair; filtered pairs are kept, without contacts.
    assert!(narrow_phase.contact_pair(s.ground, sensor).is_none());
    let cases = array![lonely, kinematic];
    for collider in cases {
        let pair = pair_of(@narrow_phase, s.ground, collider);
        assert_eq!(pair.manifold.num_points, 0);
        assert!(!pair.has_any_active_contact());
    }
    assert_eq!(narrow_phase.len(), 4);
}

#[test]
fn test_warm_start_across_two_steps() {
    let mut bodies = RigidBodySetTrait::new();
    let mut colliders = ColliderSetTrait::new();
    let s = two_boxes(ref bodies, ref colliders);
    let mut narrow_phase: NarrowPhase = NarrowPhaseTrait::new();
    let _ = step(ref narrow_phase, ref bodies, ref colliders);
    // The solver of step 1 writes impulses back into box A's points (not box B's).
    let mut written = array![];
    for pair in narrow_phase.pairs.span() {
        let mut pair = *pair;
        if pair.collider2 == s.box_a {
            let [mut p0, mut p1] = pair.manifold.points;
            p0.data.impulse = HALF;
            p0.data.tangent_impulse = ONE;
            p1.data.impulse = ONE;
            pair.manifold.points = [p0, p1];
        }
        written.append(pair);
    }
    narrow_phase.pairs = written;
    let events = step(ref narrow_phase, ref bodies, ref colliders);
    assert_eq!(events.len(), 0);
    // (box, contact ids, impulses)
    let cases = array![
        (s.box_a, (0, 1), (HALF, ONE)),
        (s.box_b, (NEW_CONTACT_BIT, 1 + NEW_CONTACT_BIT), (ZERO, ZERO)),
    ];
    for (collider, ids, impulses) in cases {
        let pair = pair_of(@narrow_phase, s.ground, collider);
        let (id0, id1) = ids;
        let (i0, i1) = impulses;
        let sc = contacts(pair);
        assert_eq!(sc.at(0).contact_id, @id0);
        assert_eq!(sc.at(1).contact_id, @id1);
        let [p0, p1] = pair.manifold.points;
        assert_eq!(p0.data.impulse, i0);
        assert_eq!(p1.data.impulse, i1);
    }
}

#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}

/// Two boxes on the ground: sets, broad phase and two narrow-phase steps.
#[test]
fn gas_two_boxes_two_steps() {
    let mut bodies = RigidBodySetTrait::new();
    let mut colliders = ColliderSetTrait::new();
    let _ = two_boxes(ref bodies, ref colliders);
    let mut narrow_phase: NarrowPhase = opaque(NarrowPhaseTrait::new());
    let _ = step(ref narrow_phase, ref bodies, ref colliders);
    let _ = step(ref narrow_phase, ref bodies, ref colliders);
}
