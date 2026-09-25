//! Gas probes of sensor pairs (work package SE): one standalone sensor (collider 0, events on)
//! and one awake dynamic body overlapping it, no gravity, warmed up by one step so that the pair
//! is carried over. Kinds: `bb` ball sensor–ball, `cb` cuboid sensor–ball, `cc` cuboid
//! sensor–cuboid, `pc` triangle sensor–cuboid; `cs` is `cc` with a solid (contact) collider.
//!
//! `gas_se_setup_<kind>` builds the scene, runs the warm-up step and the fused stages of the
//! next step up to `find_pairs`; `gas_se_np_<kind>` adds the narrow phase: the difference is the
//! pair's narrow-phase cost per step. `gas_se_step_<kind>` runs a whole second step instead, to
//! compare with `gas_se_warm_<kind>` (warm-up only).

use fixed::{HALF, ONE, ZERO};
use glam::Vec2;
use rapier_core::collider::events::COLLISION_EVENTS;
use rapier_core::integration_parameters::IntegrationParametersTrait;
use rapier_core::rigid_body::RigidBodyActivationTrait;
use rapier_dynamics2d::collider::ColliderBuilderTrait;
use rapier_dynamics2d::narrow_phase::compute_contacts_from_scratch;
use rapier_dynamics2d::rigid_body_set::RigidBodyTrait;
use rapier_geometry2d::broad_phase::find_pairs;
use rapier_geometry2d::shape::{ConvexPolygonTrait, Shape};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::IDENTITY;
use rapier_testing::opaque;
use crate::dispatcher::DefaultDispatcher;
use crate::world::{World, WorldTrait};
use super::{collision_inputs, user_changes_bodies};

fn v(x: fixed::Fixed, y: fixed::Fixed) -> Vec2 {
    Vec2 { x, y }
}

/// The scene of `kind`, warmed up by one step.
#[inline(never)]
fn warm(kind: felt252) -> World {
    let mut world = WorldTrait::new(v(ZERO, ZERO), Default::default());
    let sensor_shape = if kind == 'bb' {
        Shape::Ball(rapier_geometry2d::shape::BallTrait::new(ONE))
    } else if kind == 'pc' {
        let triangle = ConvexPolygonTrait::from_convex_polyline(
            array![v(-ONE, -ONE), v(ONE, -ONE), v(ZERO, ONE)].span(),
        )
            .unwrap();
        Shape::ConvexPolygon(BoxTrait::new(triangle))
    } else {
        Shape::Cuboid(rapier_geometry2d::shape::CuboidTrait::new(v(ONE, ONE)))
    };
    let _ = world
        .insert_collider(
            ColliderBuilderTrait::new(sensor_shape)
                .sensor(kind != 'cs')
                .active_events(COLLISION_EVENTS)
                .build(),
            None,
        );
    let other = if kind == 'cc' || kind == 'pc' || kind == 'cs' {
        ColliderBuilderTrait::cuboid(HALF, HALF).build()
    } else {
        ColliderBuilderTrait::ball(HALF).build()
    };
    let mut body = RigidBodyTrait::dynamic(
        Pose2 { translation: v(HALF, ZERO), rotation: IDENTITY },
    );
    body.activation = RigidBodyActivationTrait::cannot_sleep();
    let _ = world.insert(body, other);
    let _ = world.step();
    world
}

/// The narrow-phase stage of the second step; nothing when `!narrow`.
#[inline(never)]
fn stage(kind: felt252, narrow: bool) {
    let mut world = warm(kind);
    let (snapshot, infos, _, _) = user_changes_bodies(
        ref world.bodies, ref world.colliders, world.narrow_phase.pairs.span(),
    );
    let prediction = world.integration_parameters.prediction_distance();
    let (proxies, scratch) = collision_inputs(snapshot, infos, ref world.bodies, prediction);
    let pairs = find_pairs(proxies.span());
    if !narrow {
        return;
    }
    let _ = compute_contacts_from_scratch::<
        DefaultDispatcher,
    >(ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders);
}

/// A whole second step; only the warm-up when `!step`.
#[inline(never)]
fn full(kind: felt252, step: bool) {
    let mut world = warm(kind);
    if step {
        let _ = world.step();
    }
}

#[test]
fn test_scenes_hold_one_intersecting_pair() {
    let first = rapier_core::Handle { index: 0, generation: 0 };
    let second = rapier_core::Handle { index: 1, generation: 0 };
    for kind in array!['bb', 'cb', 'cc', 'pc'] {
        let mut world = warm(kind);
        assert_eq!(world.intersection_pairs(), array![(first, second, true)]);
    }
    let mut world = warm('cs');
    assert_eq!(world.intersection_pairs(), array![]);
    assert!(world.contact_pair(first, second).is_some());
}

#[test]
fn gas_baseline() {
    let _ = opaque(0);
}

#[test]
fn gas_se_setup_bb() {
    stage(opaque('bb'), false);
}

#[test]
fn gas_se_np_bb() {
    stage(opaque('bb'), true);
}

#[test]
fn gas_se_setup_cb() {
    stage(opaque('cb'), false);
}

#[test]
fn gas_se_np_cb() {
    stage(opaque('cb'), true);
}

#[test]
fn gas_se_setup_cc() {
    stage(opaque('cc'), false);
}

#[test]
fn gas_se_np_cc() {
    stage(opaque('cc'), true);
}

#[test]
fn gas_se_setup_pc() {
    stage(opaque('pc'), false);
}

#[test]
fn gas_se_np_pc() {
    stage(opaque('pc'), true);
}

#[test]
fn gas_se_setup_cs() {
    stage(opaque('cs'), false);
}

#[test]
fn gas_se_np_cs() {
    stage(opaque('cs'), true);
}

#[test]
fn gas_se_warm_cc() {
    full(opaque('cc'), false);
}

#[test]
fn gas_se_step_cc() {
    full(opaque('cc'), true);
}

#[test]
fn gas_se_warm_cs() {
    full(opaque('cs'), false);
}

#[test]
fn gas_se_step_cs() {
    full(opaque('cs'), true);
}
