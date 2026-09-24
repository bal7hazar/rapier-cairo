//! Worlds used by the pipeline tests and gas probes: golden scenes (`rapier_golden::scenes`)
//! rebuilt through the public `World` API, and synthetic layouts.

use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_dynamics2d::collider::{Collider, ColliderBuilder, ColliderBuilderTrait};
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::joint::RevoluteJointBuilderTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBodySetTrait, RigidBodyTrait};
use rapier_geometry2d::shape::{
    BallTrait, CapsuleTrait, CuboidTrait, HalfSpaceTrait, SegmentTrait, Shape,
};
use rapier_golden::types::{BodyKindRaw, PoseRaw, SceneCase, ShapeRaw, Vec2Raw};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use crate::world::{World, WorldTrait};

pub fn f(raw: i64) -> Fixed {
    Fixed { raw }
}

pub fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

pub fn vr(raw: Vec2Raw) -> Vec2 {
    v(f(raw.x), f(raw.y))
}

pub fn pose(raw: PoseRaw) -> Pose2 {
    Pose2 {
        translation: vr(raw.translation),
        rotation: Rot2 { re: f(raw.rotation.re), im: f(raw.rotation.im) },
    }
}

pub fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2 { translation: v(x, y), rotation: Rot2 { re: ONE, im: ZERO } }
}

pub fn shape(raw: ShapeRaw) -> Shape {
    match raw {
        ShapeRaw::Ball(radius) => Shape::Ball(BallTrait::new(f(radius))),
        ShapeRaw::Cuboid(half) => Shape::Cuboid(CuboidTrait::new(vr(half))),
        ShapeRaw::Capsule(c) => Shape::Capsule(CapsuleTrait::new(vr(c.a), vr(c.b), f(c.radius))),
        ShapeRaw::HalfSpace(n) => Shape::HalfSpace(HalfSpaceTrait::new(vr(n))),
        ShapeRaw::Segment(s) => Shape::Segment(SegmentTrait::new(vr(s.a), vr(s.b))),
    }
}

/// The parameters of a golden scene: upstream defaults with the scene's `dt`.
pub fn scene_params(scene: SceneCase) -> IntegrationParameters {
    IntegrationParameters { dt: f(scene.dt), ..Default::default() }
}

/// `scene` rebuilt in a fresh world; the bodies' handles in scene order.
pub fn scene_world(scene: SceneCase) -> (World, Array<Handle>) {
    let mut world = WorldTrait::new(vr(scene.gravity), scene_params(scene));
    let mut handles = array![];
    for desc in scene.bodies.span() {
        if handles.len() == scene.num_bodies {
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
        if *desc.num_colliders != 0 {
            let co = *desc.colliders.span().at(0);
            let collider = ColliderBuilderTrait::new(shape(co.shape))
                .position(pose(co.pose_wrt_parent))
                .density(f(co.density))
                .friction(f(co.friction))
                .restitution(f(co.restitution))
                .build();
            let _ = world.insert_collider(collider, Some(handle));
        }
        handles.append(handle);
    }
    for joint in scene.joints.span() {
        if scene.num_joints == 0 {
            break;
        }
        let data = RevoluteJointBuilderTrait::new()
            .local_anchor1(vr(*joint.local_anchor1))
            .local_anchor2(vr(*joint.local_anchor2))
            .build();
        let _ = world
            .insert_impulse_joint(*handles.at(*joint.body1), *handles.at(*joint.body2), data);
    }
    (world, handles)
}

/// A ball of radius 1/2 above a half-space `y ≥ 0`, its centre at height `height`; returns
/// the world and the ball's body and collider handles. `events` enables collision events on the
/// ball. Default parameters, gravity `(0, -9.81)`.
pub fn ball_on_ground(height: Fixed, events: bool) -> (World, Handle, Handle) {
    let mut world = WorldTrait::new(v(ZERO, f(-42133629174)), Default::default());
    let _ = world.insert_collider(ColliderBuilderTrait::halfspace(v(ZERO, ONE)).build(), None);
    let mut builder = ColliderBuilderTrait::ball(HALF);
    if events {
        builder = builder.active_events(rapier_core::collider::events::COLLISION_EVENTS);
    }
    let (body, collider) = world.insert(RigidBodyTrait::dynamic(at(ZERO, height)), builder.build());
    (world, body, collider)
}

/// `n` dynamic bodies in a row along x, each touching the next (`n - 1` touching pairs, all of
/// the same kind), no gravity: balls of radius 1/2 when `balls`, unit cuboids otherwise.
pub fn row(n: u32, balls: bool) -> World {
    let mut world = WorldTrait::new(v(ZERO, ZERO), Default::default());
    let mut i: u32 = 0;
    while i != n {
        let builder: ColliderBuilder = if balls {
            ColliderBuilderTrait::ball(HALF)
        } else {
            ColliderBuilderTrait::cuboid(HALF, HALF)
        };
        let x = FixedTrait::from_int(i.try_into().unwrap());
        let _ = world.insert(RigidBodyTrait::dynamic(at(x, ZERO)), builder.build());
        i += 1;
    }
    world
}

/// No gravity; a ball touching a half-space `y ≥ 0` (one touching pair) and two balls far above,
/// diagonal neighbours whose AABBs overlap without contact (one non-touching pair).
pub fn mixed() -> World {
    let mut world = WorldTrait::new(v(ZERO, ZERO), Default::default());
    let _ = world.insert_collider(ColliderBuilderTrait::halfspace(v(ZERO, ONE)).build(), None);
    let five = FixedTrait::from_int(5);
    let near = five + FixedTrait::from_raw(3865470566); // 5.9
    for (x, y) in array![(ZERO, HALF), (five, five), (near, near)].span() {
        let _ = world
            .insert(RigidBodyTrait::dynamic(at(*x, *y)), ColliderBuilderTrait::ball(HALF).build());
    }
    world
}

/// `n` fixed unit platforms side by side at `y = -1` (static proxies) and one dynamic ball
/// falling from `y = 5` (gravity `(0, -9.81)`).
pub fn statics(n: u32) -> World {
    let mut world = WorldTrait::new(v(ZERO, f(-42133629174)), Default::default());
    let mut i: u32 = 0;
    while i != n {
        let x = FixedTrait::from_int((2 * i).try_into().unwrap());
        let _ = world
            .insert(
                RigidBodyTrait::fixed(at(x, -ONE)), ColliderBuilderTrait::cuboid(ONE, HALF).build(),
            );
        i += 1;
    }
    let _ = world
        .insert(
            RigidBodyTrait::dynamic(at(ZERO, FixedTrait::from_int(5))),
            ColliderBuilderTrait::ball(HALF).build(),
        );
    world
}

/// `n` dynamic balls of radius 1/2 falling from `y = 100`, 4 apart along x (no contact at all),
/// gravity `(0, -9.81)`: the `free_fall` scene of `tests/gas_scenes.cairo`.
pub fn free_fall(n: u32) -> World {
    let mut world = WorldTrait::new(v(ZERO, f(-42133629174)), Default::default());
    let mut i: u32 = 0;
    while i != n {
        let x = FixedTrait::from_int((4 * i).try_into().unwrap());
        let _ = world
            .insert(
                RigidBodyTrait::dynamic(at(x, FixedTrait::from_int(100))),
                ColliderBuilderTrait::ball(HALF).build(),
            );
        i += 1;
    }
    world
}

/// One draw of a 31-bit linear congruential generator (glibc constants).
pub fn draw(ref state: u64) -> u32 {
    state = (state * 1103515245 + 12345) % 0x80000000;
    (state / 0x10000).try_into().unwrap()
}

/// A random world for `seed`: a ground half-space, 3–7 bodies (dynamic, fixed or
/// position-based kinematic) 0.9 apart so that neighbours touch, each with one ball, cuboid or
/// capsule (some offset, sensor or disabled) and sometimes a second collider; then, depending on
/// the seed, a body and a collider removed and a body inserted into a freed slot, so that the
/// sets have free slots and bumped generations.
pub fn random_world(seed: u32) -> World {
    let mut state: u64 = seed.into();
    let mut world = WorldTrait::new(v(ZERO, f(-42133629174)), Default::default());
    let _ = world.insert_collider(ColliderBuilderTrait::halfspace(v(ZERO, ONE)).build(), None);
    let n = 3 + draw(ref state) % 5;
    let mut bodies = array![];
    let mut k: u32 = 0;
    while k != n {
        let pos = at(f(3865470566 * k.into()), HALF + f(1932735283 * (draw(ref state) % 3).into()));
        let kind = draw(ref state) % 6;
        let body = if kind == 4 {
            RigidBodyTrait::fixed(pos)
        } else if kind == 5 {
            RigidBodyTrait::kinematic_position_based(pos)
        } else {
            RigidBodyTrait::dynamic(pos)
        };
        let (handle, _) = world.insert(body, random_collider(ref state));
        if draw(ref state) % 4 == 0 {
            let _ = world.insert_collider(random_collider(ref state), Some(handle));
        }
        bodies.append(handle);
        k += 1;
    }
    let roll = draw(ref state);
    if roll % 2 == 0 {
        let _ = world.remove_body(*bodies.at(1));
    }
    if roll % 3 == 0 {
        let (first, _) = *world.colliders.iter().at(1);
        let _ = world.remove_collider(first);
        let _ = world
            .insert(
                RigidBodyTrait::dynamic(at(f(-3865470566), ONE)),
                ColliderBuilderTrait::ball(HALF).build(),
            );
    }
    world
}

fn random_collider(ref state: u64) -> Collider {
    let shape = draw(ref state) % 3;
    let mut builder = if shape == 0 {
        ColliderBuilderTrait::ball(HALF)
    } else if shape == 1 {
        ColliderBuilderTrait::cuboid(HALF, f(1932735283))
    } else {
        ColliderBuilderTrait::capsule_x(f(1073741824), f(1073741824))
    };
    let roll = draw(ref state);
    if roll % 5 == 0 {
        builder = builder.translation(v(f(858993459), ZERO));
    }
    if roll % 7 == 0 {
        builder = builder.sensor(true);
    }
    if roll % 11 == 0 {
        builder = builder.enabled(false);
    }
    builder.build()
}

/// [`random_world`] plus what the fused solve (work package OI) splits on: free bodies far above
/// the others (a spinning dynamic one with damping and a user force, a velocity-based kinematic
/// one), a disabled dynamic body, and a revolute joint between the first two bodies of the set
/// (enabled or not, from `seed`) so that constrained, free and fixed bodies all mix.
pub fn oi_world(seed: u32) -> World {
    let mut world = random_world(seed);
    let mut state: u64 = seed.into() + 7;
    let mut spinning = RigidBodyTrait::dynamic(at(f(-8589934592), f(214748364800)));
    spinning.vels.angvel = f(4294967296 * (draw(ref state) % 5).into());
    spinning.vels.linvel = v(f(2147483648), ZERO);
    spinning.damping.linear_damping = f(429496730);
    spinning.damping.angular_damping = f(858993459 * (draw(ref state) % 2).into());
    spinning.forces.user_force = v(f(4294967296), ZERO);
    let _ = world.insert(spinning, random_collider(ref state));
    let mut kinematic = RigidBodyTrait::new(
        rapier_core::rigid_body::RigidBodyType::KinematicVelocityBased,
        at(f(42949672960), f(214748364800)),
    );
    kinematic.vels.linvel = v(ZERO, f(-4294967296));
    let _ = world.insert(kinematic, ColliderBuilderTrait::ball(HALF).build());
    let mut disabled = RigidBodyTrait::dynamic(at(f(85899345920), f(214748364800)));
    disabled.enabled = false;
    let _ = world.insert(disabled, ColliderBuilderTrait::ball(HALF).build());
    let bodies = world.bodies.iter();
    let (first, _) = *bodies.at(0);
    let (second, _) = *bodies.at(1);
    let mut joint = RevoluteJointBuilderTrait::new()
        .local_anchor1(v(HALF, ZERO))
        .local_anchor2(v(-HALF, ZERO))
        .build();
    if draw(ref state) % 3 == 0 {
        joint.enabled = rapier_dynamics2d::joint::JointEnabled::Disabled;
    }
    let _ = world.insert_impulse_joint(first, second, joint);
    world
}
