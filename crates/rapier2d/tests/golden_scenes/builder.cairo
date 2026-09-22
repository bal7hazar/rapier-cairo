//! Scene builder shared by the golden replays: a `SceneCase` fixture turned into a `World`
//! through the public facade (`rapier2d::prelude`), plus the raw-to-fixed conversions.

use rapier2d::prelude::{
    ColliderBuilderTrait, Fixed, Handle, IntegrationParameters, RevoluteJointBuilderTrait,
    RigidBodyTrait, Vec2, World, WorldTrait,
};
use rapier_golden::types::{BodyKindRaw, PoseRaw, SceneCase, SceneSampleRaw, ShapeRaw, Vec2Raw};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;

/// Solver iterations (substeps) of every scene: the upstream default, as `tools/golden/README.md`
/// states for the traces.
pub const NUM_SOLVER_ITERATIONS: u32 = 4;

pub fn f(raw: i64) -> Fixed {
    Fixed { raw }
}

pub fn vr(raw: Vec2Raw) -> Vec2 {
    Vec2 { x: f(raw.x), y: f(raw.y) }
}

pub fn pose(raw: PoseRaw) -> Pose2 {
    Pose2 {
        translation: vr(raw.translation),
        rotation: Rot2 { re: f(raw.rotation.re), im: f(raw.rotation.im) },
    }
}

/// Handle of the `index`-th body of a scene: bodies go into a fresh arena in scene order, so
/// the handle is the index at generation 0 (`build_world` asserts it).
pub fn body_handle(index: u32) -> Handle {
    Handle { index, generation: 0 }
}

/// The world of `scene`: gravity and `dt` from the case, the other integration parameters at
/// their upstream defaults (`num_solver_iterations = 4`); bodies in insertion order with their
/// damping and gravity scale, each with its collider (shape, pose, density, friction,
/// restitution), then the revolute joints.
pub fn build_world(scene: SceneCase) -> World {
    let params = IntegrationParameters {
        dt: f(scene.dt), num_solver_iterations: NUM_SOLVER_ITERATIONS, ..Default::default(),
    };
    let mut world = WorldTrait::new(vr(scene.gravity), params);
    let mut i = 0;
    for desc in scene.bodies.span() {
        if i == scene.num_bodies {
            break;
        }
        let mut body = match desc.kind {
            BodyKindRaw::Fixed => RigidBodyTrait::fixed(pose(*desc.pose)),
            BodyKindRaw::Dynamic => RigidBodyTrait::dynamic(pose(*desc.pose)),
        };
        body.damping.linear_damping = f(*desc.linear_damping);
        body.damping.angular_damping = f(*desc.angular_damping);
        body.forces.gravity_scale = f(*desc.gravity_scale);
        let handle = world.insert_body(body);
        assert!(handle == body_handle(i), "scene body handles follow insertion order");
        if *desc.num_colliders != 0 {
            let co = *desc.colliders.span().at(0);
            let builder = match co.shape {
                ShapeRaw::Ball(radius) => ColliderBuilderTrait::ball(f(radius)),
                ShapeRaw::Cuboid(half) => ColliderBuilderTrait::cuboid(f(half.x), f(half.y)),
                _ => panic!("unexpected scene shape"),
            };
            let collider = builder
                .position(pose(co.pose_wrt_parent))
                .density(f(co.density))
                .friction(f(co.friction))
                .restitution(f(co.restitution))
                .build();
            let _ = world.insert_collider(collider, Some(handle));
        }
        i += 1;
    }
    let mut j = 0;
    for joint in scene.joints.span() {
        if j == scene.num_joints {
            break;
        }
        let data = RevoluteJointBuilderTrait::new()
            .local_anchor1(vr(*joint.local_anchor1))
            .local_anchor2(vr(*joint.local_anchor2))
            .build();
        let _ = world
            .insert_impulse_joint(body_handle(*joint.body1), body_handle(*joint.body2), data);
        j += 1;
    }
    world
}

/// Overwrites every dynamic body with its upstream state in `sample` (pose, linear and angular
/// velocity). The narrow-phase cache is left as is: warm-start impulses are the port's own.
pub fn reseed(ref world: World, scene: SceneCase, sample: SceneSampleRaw) {
    let mut k = 0;
    for state in sample.states.span() {
        if k == scene.num_dynamic {
            break;
        }
        let handle = body_handle(*state.body);
        let mut rb = world.body(handle).unwrap();
        rb
            .set_position(
                Pose2 {
                    translation: vr(*state.translation),
                    rotation: Rot2 { re: f(*state.rotation.re), im: f(*state.rotation.im) },
                },
            );
        rb.set_linvel(vr(*state.linvel));
        rb.set_angvel(f(*state.angvel));
        assert!(world.set_body(handle, rb));
        k += 1;
    }
}
