//! Tests and gas probes of the pipeline objects (`super`).

use fixed::{Fixed, HALF, ONE, ZERO};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::collider::events::{COLLISION_EVENTS, REMOVED, SENSOR};
use rapier_core::integration_parameters::IntegrationParametersTrait;
use rapier_core::rigid_body::RigidBodyChangesTrait;
use rapier_dynamics2d::collider::ColliderBuilderTrait;
use rapier_dynamics2d::events::CollisionEvent;
use rapier_dynamics2d::rigid_body_set::RigidBodyTrait;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::world::{World, WorldTrait};
use super::{CollisionPipeline, CollisionPipelineTrait, PhysicsPipeline, PhysicsPipelineTrait};

fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2 { translation: Vec2 { x, y }, rotation: Rot2 { re: ONE, im: ZERO } }
}

/// A ball sinking into a standalone ground (contact events on) and a standalone sensor around
/// it: `(world, ball body, ground, ball collider, sensor)`.
fn scene() -> (World, Handle, Handle, Handle, Handle) {
    let mut world = WorldTrait::new(Vec2 { x: ZERO, y: -ONE }, Default::default());
    let ground = world
        .insert_collider(
            ColliderBuilderTrait::halfspace(Vec2 { x: ZERO, y: ONE })
                .active_events(COLLISION_EVENTS)
                .build(),
            None,
        );
    let (ball, ball_co) = world
        .insert(
            RigidBodyTrait::dynamic(at(ZERO, HALF - Fixed { raw: 0x10000000 })),
            ColliderBuilderTrait::ball(HALF).build(),
        );
    let sensor = world
        .insert_collider(
            ColliderBuilderTrait::ball(ONE)
                .sensor(true)
                .active_events(COLLISION_EVENTS)
                .position(at(ZERO, HALF))
                .build(),
            None,
        );
    (world, ball, ground, ball_co, sensor)
}

/// Collision detection only: the events of `World::step`'s first step, the bodies do not move,
/// the pairs persist (no event at the next call), a removal ends its pairs with `REMOVED`.
#[test]
fn test_collision_pipeline_step_events() {
    let (mut reference, _, _, _, _) = scene();
    let expected = reference.step();
    let (mut world, ball, ground, ball_co, sensor) = scene();
    let prediction = world.integration_parameters.prediction_distance();
    let pipeline: CollisionPipeline = Default::default();
    assert_eq!(pipeline, CollisionPipelineTrait::new());
    let before = world.body(ball).unwrap().position();
    let events = pipeline
        .step(prediction, ref world.narrow_phase, ref world.bodies, ref world.colliders);
    assert_eq!(events, expected);
    assert_eq!(
        events,
        array![
            CollisionEvent::Started((ground, ball_co, Default::default())),
            CollisionEvent::Started((ball_co, sensor, SENSOR)),
        ],
    );
    assert_eq!(world.body(ball).unwrap().position(), before);
    assert!(world.body(ball).unwrap().changes.is_empty());
    assert!(world.contact_pair(ground, ball_co).is_some());
    // (step, expected events): unchanged state, then the sensor removed.
    let again = pipeline
        .step(prediction, ref world.narrow_phase, ref world.bodies, ref world.colliders);
    assert_eq!(again, array![]);
    let _ = world.remove_collider(sensor);
    let events = pipeline
        .step(prediction, ref world.narrow_phase, ref world.bodies, ref world.colliders);
    assert_eq!(events, array![CollisionEvent::Stopped((ball_co, sensor, SENSOR | REMOVED))]);
    assert_eq!(world.body(ball).unwrap().position(), before);
}

/// `PhysicsPipeline` is the world's step.
#[test]
fn test_physics_pipeline_step() {
    let (mut reference, _, _, _, _) = scene();
    let (mut world, ball, _, _, _) = scene();
    let pipeline: PhysicsPipeline = Default::default();
    assert_eq!(pipeline, PhysicsPipelineTrait::new());
    assert_eq!(pipeline.step(ref world), reference.step());
    assert_eq!(world.body(ball), reference.body(ball));
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
fn gas_scene_setup() {
    let _ = scene();
}

#[test]
fn gas_collision_pipeline_step() {
    let (mut world, _, _, _, _) = scene();
    let prediction = opaque(world.integration_parameters.prediction_distance());
    let _ = CollisionPipelineTrait::new()
        .step(prediction, ref world.narrow_phase, ref world.bodies, ref world.colliders);
}

#[test]
fn gas_world_step_same_scene() {
    let (mut world, _, _, _, _) = scene();
    world.gravity = opaque(world.gravity);
    let _ = world.step();
}
