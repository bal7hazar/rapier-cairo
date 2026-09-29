//! CP3: the contact-pair read API of `World` on stepped scenes (a resting ball, a box on a
//! polyline whose contact spans two parts, removed colliders). The step path is untouched.

use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam_core::Vec2;
use rapier2d::world::{World, WorldTrait};
use rapier_core::Handle;
use rapier_dynamics2d::collider::{ColliderBuilder, ColliderBuilderTrait};
use rapier_dynamics2d::narrow_phase::composite::contact_pair_manifolds;
use rapier_dynamics2d::narrow_phase::contact_pairs::{
    ContactPairViewTrait, NarrowPhaseContactPairsTrait,
};
use rapier_dynamics2d::rigid_body_set::RigidBodyTrait;
use rapier_geometry2d::contact::ContactManifoldExt;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;

const GRAVITY_Y: i64 = -42133629174;

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2 { translation: v(x, y), rotation: Rot2 { re: ONE, im: ZERO } }
}

fn steps(ref world: World, n: u32) {
    let mut i = 0;
    while i != n {
        let _ = world.step();
        i += 1;
    }
}

/// A half-space and a resting ball: (world, ground, ball collider).
fn ball_scene() -> (World, Handle, Handle) {
    let mut world = WorldTrait::new(v(ZERO, Fixed { raw: GRAVITY_Y }), Default::default());
    let ground = world.insert_collider(ColliderBuilderTrait::halfspace(v(ZERO, ONE)).build(), None);
    let (_, ball) = world
        .insert(RigidBodyTrait::dynamic(at(ZERO, HALF)), ColliderBuilderTrait::ball(HALF).build());
    (world, ground, ball)
}

/// A chained polyline of segments 2 wide over `[-4, 4]` and a box of half-extent 1/2 straddling
/// the vertex at `x = 0`: (world, ground, box collider).
fn polyline_scene() -> (World, Handle, Handle) {
    let mut world = WorldTrait::new(v(ZERO, Fixed { raw: GRAVITY_Y }), Default::default());
    let mut vertices = array![];
    let mut x: Fixed = FixedTrait::from_int(-4);
    while x != FixedTrait::from_int(6) {
        vertices.append(v(x, ZERO));
        x = x + FixedTrait::from_int(2);
    }
    let builder: ColliderBuilder = ColliderBuilderTrait::polyline(vertices.span(), None);
    let ground = world.insert_collider(builder.build(), None);
    let (_, c) = world
        .insert(
            RigidBodyTrait::dynamic(at(ZERO, HALF)),
            ColliderBuilderTrait::cuboid(HALF, HALF).build(),
        );
    (world, ground, c)
}

#[test]
fn test_world_contact_pairs_resting_ball() {
    let (mut world, ground, ball) = ball_scene();
    assert_eq!(world.contact_pairs().len(), 0);
    steps(ref world, 3);
    let pairs = world.contact_pairs();
    assert_eq!(pairs.len(), 1);
    let pair = pairs.at(0);
    assert_eq!((*pair.collider1, *pair.collider2), (ground, ball));
    assert!(pair.has_any_active_contact());
    // The accessors agree with the stored manifold and the frozen entry read.
    let entry = world.contact_pair(ground, ball).unwrap();
    assert_eq!(pair.manifolds().len(), 1);
    assert_eq!(*pair.manifolds().at(0), entry.manifold);
    let impulse = entry.manifold.total_impulse();
    assert!(impulse > ZERO, "a resting ball is pushed up");
    assert_eq!(pair.total_impulse_magnitude(), impulse);
    let normal = entry.manifold.data.normal;
    assert_eq!(pair.max_impulse(), (impulse, normal));
    assert_eq!(pair.total_impulse(), v(impulse * normal.x, impulse * normal.y));
    let (m, deepest) = pair.find_deepest_contact().unwrap();
    assert_eq!(m, entry.manifold);
    let [p0, _] = entry.manifold.points;
    assert_eq!(deepest, p0);
    // By slot, both orders, and through the narrow phase.
    assert_eq!(world.narrow_phase.contact_pair_unknown_gen(1, 0).unwrap().manifolds().len(), 1);
    assert_eq!(world.narrow_phase.contact_pair_at_index(0).unwrap().manifolds().len(), 1);
    assert_eq!(world.contact_pairs_with(ball).len(), 1);
    assert_eq!(world.narrow_phase.contact_pairs_with_unknown_gen(0).len(), 1);
    assert_eq!(world.contact_pairs_with(Handle { index: 7, generation: 0 }).len(), 0);
}

#[test]
fn test_world_contact_pairs_composite_run() {
    let (mut world, ground, c) = polyline_scene();
    steps(ref world, 3);
    let expected = contact_pair_manifolds(world.narrow_phase.pairs.span(), ground, c);
    assert!(expected.len() >= 2, "the box spans two parts");
    let pairs = world.contact_pairs();
    assert_eq!(pairs.len(), 1, "one pair per collider pair");
    let pair = pairs.at(0);
    assert_eq!(pair.manifolds(), expected.span());
    let mut sum: Fixed = ZERO;
    for m in expected.span() {
        sum = sum + m.total_impulse();
    }
    assert_eq!(pair.total_impulse_magnitude(), sum);
    assert!(pair.has_any_active_contact());
    // The gathered read carries the event status of the run's first entry.
    assert_eq!(*pair.event_status, *world.narrow_phase.pairs.at(0).event_status);
    // Only the run's first entry is an index that answers.
    let n = world.narrow_phase.pairs.len();
    assert!(n >= 2);
    assert!(world.narrow_phase.contact_pair_at_index(0).is_some());
    assert!(world.narrow_phase.contact_pair_at_index(1).is_none());
}

#[test]
fn test_world_contact_pairs_removed_collider() {
    let (mut world, ground, ball) = ball_scene();
    steps(ref world, 2);
    assert!(world.remove_collider(ball).is_some());
    // Until the next step the narrow phase still holds the pair; the world hides it.
    assert_eq!(world.narrow_phase.contact_pairs().len(), 1);
    assert_eq!(world.contact_pairs().len(), 0);
    assert_eq!(world.contact_pairs_with(ground).len(), 0);
    steps(ref world, 1);
    assert_eq!(world.narrow_phase.contact_pairs().len(), 0);
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
fn gas_setup_ball() {
    let (mut world, _, _) = ball_scene();
    steps(ref world, opaque(3));
}

#[test]
fn gas_world_contact_pairs() {
    let (mut world, _, _) = ball_scene();
    steps(ref world, 3);
    let _ = opaque(world.contact_pairs());
}

#[test]
fn gas_world_contact_pairs_with() {
    let (mut world, _, ball) = ball_scene();
    steps(ref world, 3);
    let _ = opaque(world.contact_pairs_with(opaque(ball)));
}
