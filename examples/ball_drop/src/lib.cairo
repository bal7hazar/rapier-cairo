//! `#[executable]` physics step for `scarb execute` / `scarb prove` (`docs/PLAN.md` P4).
//!
//! `main(scene, steps)` builds one of three scenes, runs `steps` calls of `World::step` and
//! returns the final state of every dynamic body as raw Q32.32 felts plus the number of collision
//! events emitted. Output layout, all values `felt252` (a negative `i64` is `P - |raw|`):
//!
//! ```text
//! [num_events, num_dynamic_bodies, then per dynamic body (scene order):
//!  translation.x, translation.y, rotation.re, rotation.im, linvel.x, linvel.y, angvel]
//! ```
//!
//! Scenes: `0` ball drop (ball of radius 1/2 from height 2 on a half-space), `1` box stack of 3
//! (`BOX_STACK3` of the golden scenes), `2` pendulum (`PENDULUM` of the golden scenes).

use rapier2d::prelude::*;
use rapier_core::collider::events::COLLISION_EVENTS;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;

pub const BALL_DROP: u8 = 0;
pub const BOX_STACK3: u8 = 1;
pub const PENDULUM: u8 = 2;

/// Felts per dynamic body in the output.
pub const FELTS_PER_BODY: u32 = 7;

pub mod errors {
    pub const UNKNOWN_SCENE: felt252 = 'ball_drop: unknown scene';
}

/// Earth gravity, `-9.81` in raw Q32.32.
const GRAVITY_Y: i64 = -42133629174;
/// Golden scenes' timestep, `1/60` in raw Q32.32.
const DT: i64 = 71582788;
const TEN: i64 = 42949672960;
const HALF_RAW: i64 = 2147483648;
const ONE_RAW: i64 = 4294967296;

fn f(raw: i64) -> Fixed {
    Fixed { raw }
}

fn at(x: i64, y: i64) -> Pose2 {
    Pose2 { translation: Vec2 { x: f(x), y: f(y) }, rotation: Rot2 { re: f(ONE_RAW), im: f(0) } }
}

fn empty_world() -> World {
    let params = IntegrationParameters { dt: f(DT), ..Default::default() };
    WorldTrait::new(Vec2 { x: f(0), y: f(GRAVITY_Y) }, params)
}

/// A ball of radius 1/2 dropped from height 2 on a half-space `y ≥ 0`.
fn ball_drop(ref world: World) -> Array<Handle> {
    let ground = ColliderBuilderTrait::halfspace(Vec2 { x: f(0), y: f(ONE_RAW) }).build();
    let _ = world.insert_collider(ground, None);
    let ball = ColliderBuilderTrait::ball(f(HALF_RAW)).active_events(COLLISION_EVENTS).build();
    let (body, _) = world.insert(RigidBodyTrait::dynamic(at(0, 2 * ONE_RAW)), ball);
    array![body]
}

/// A fixed ground box (half extents 10 × 1/2, top at `y = 0`) and three unit boxes stacked with
/// a small gap, exactly as the golden scene `BOX_STACK3`.
fn box_stack3(ref world: World) -> Array<Handle> {
    let ground = ColliderBuilderTrait::cuboid(f(TEN), f(HALF_RAW)).build();
    let _ = world.insert(RigidBodyTrait::fixed(at(0, -HALF_RAW)), ground);
    let mut handles = array![];
    for y in array![2190433321_i64, 6528350290, 10866267259].span() {
        let collider = ColliderBuilderTrait::cuboid(f(HALF_RAW), f(HALF_RAW))
            .active_events(COLLISION_EVENTS)
            .build();
        let (body, _) = world.insert(RigidBodyTrait::dynamic(at(0, *y)), collider);
        handles.append(body);
    }
    handles
}

/// A ball of radius 1/4 released at `(1, 0)`, hinged to a fixed pivot at the origin by a
/// revolute joint, exactly as the golden scene `PENDULUM`.
fn pendulum(ref world: World) -> Array<Handle> {
    let pivot = world.insert_body(RigidBodyTrait::fixed(at(0, 0)));
    let bob = ColliderBuilderTrait::ball(f(ONE_RAW / 4)).active_events(COLLISION_EVENTS).build();
    let (body, _) = world.insert(RigidBodyTrait::dynamic(at(ONE_RAW, 0)), bob);
    let joint = RevoluteJointBuilderTrait::new()
        .local_anchor1(Vec2 { x: f(0), y: f(0) })
        .local_anchor2(Vec2 { x: f(-ONE_RAW), y: f(0) })
        .build();
    let _ = world.insert_impulse_joint(pivot, body, joint);
    array![body]
}

/// Builds `scene`, runs `steps` steps and serialises the result (layout in the module docs).
///
/// # Panics
/// `ball_drop: unknown scene` when `scene` is not `0`, `1` or `2`.
pub fn simulate(scene: u8, steps: u32) -> Array<felt252> {
    assert(scene <= PENDULUM, errors::UNKNOWN_SCENE);
    let mut world = empty_world();
    let bodies = if scene == BALL_DROP {
        ball_drop(ref world)
    } else if scene == BOX_STACK3 {
        box_stack3(ref world)
    } else {
        pendulum(ref world)
    };
    let mut events: u32 = 0;
    let mut i = 0;
    while i != steps {
        events += world.step().len();
        i += 1;
    }
    let mut out = array![events.into(), bodies.len().into()];
    for handle in bodies.span() {
        let body = world.body(*handle).unwrap();
        let pose = body.position();
        let linvel = body.linvel();
        out.append(pose.translation.x.raw.into());
        out.append(pose.translation.y.raw.into());
        out.append(pose.rotation.re.raw.into());
        out.append(pose.rotation.im.raw.into());
        out.append(linvel.x.raw.into());
        out.append(linvel.y.raw.into());
        out.append(body.vels.angvel.raw.into());
    }
    out
}

/// Entry point of `scarb execute` / `scarb prove`: arguments `scene: u8, steps: u32`.
#[executable]
fn main(scene: u8, steps: u32) -> Array<felt252> {
    simulate(scene, steps)
}

#[cfg(test)]
mod tests {
    use super::{BALL_DROP, BOX_STACK3, FELTS_PER_BODY, PENDULUM, simulate};

    /// `(scene, dynamic bodies)`: one step returns `2 + 7 · bodies` felts.
    #[test]
    fn test_one_step_returns_the_expected_outputs() {
        for (scene, bodies) in array![(BALL_DROP, 1_u32), (BOX_STACK3, 3), (PENDULUM, 1)].span() {
            let out = simulate(*scene, 1);
            assert_eq!(out.len(), 2 + FELTS_PER_BODY * *bodies);
            assert_eq!(*out.at(1), (*bodies).into());
        }
    }

    #[test]
    #[should_panic(expected: 'ball_drop: unknown scene')]
    fn test_unknown_scene_panics() {
        let _ = simulate(3, 1);
    }
}
