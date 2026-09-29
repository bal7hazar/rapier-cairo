//! SH2b per-step cost of a compound of 2, 4 or 8 boxes (a plank of `n` boxes of half-extents
//! 0.25, side by side) resting on a half-space and on a flat 10-segment polyline:
//! `gas_setup_*` builds the scene and runs `WARMUP` steps, `gas_step_*` one more; the difference is
//! one warm-started `World::step` with `n` part manifolds (two points each). Compare with
//! `cuboid_stack1` (`gas_scenes.cairo`) and the SH2a `composite_budget` probes.

use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam_core::Vec2;
use rapier2d::world::{World, WorldTrait};
use rapier_dynamics2d::collider::{ColliderBuilder, ColliderBuilderTrait};
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::narrow_phase::composite::contact_pair_manifolds;
use rapier_dynamics2d::rigid_body_set::RigidBodyTrait;
use rapier_geometry2d::shape::{CuboidTrait, Shape};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;

const WARMUP: u32 = 3;
const GRAVITY_Y: i64 = -42133629174;
/// `0.25`.
const QUARTER: Fixed = Fixed { raw: 1073741824 };

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

/// The half-space `y <= 0` (`halfspace`) or a flat polyline of 10 segments over `[-10, 10]`.
fn ground(halfspace: bool) -> ColliderBuilder {
    if halfspace {
        return ColliderBuilderTrait::halfspace(v(ZERO, ONE));
    }
    let mut vertices = array![];
    let mut k: i32 = -10;
    while k != 12 {
        vertices.append(v(FixedTrait::from_int(k), ZERO));
        k += 2;
    }
    ColliderBuilderTrait::polyline(vertices.span(), None)
}

/// `n` boxes of half-extents 0.25 side by side, centred on the body.
fn plank(n: u32) -> ColliderBuilder {
    let mut parts = array![];
    let mut i: u32 = 0;
    let first = -(QUARTER * FixedTrait::from_int((n - 1).try_into().unwrap()));
    let mut x = first;
    while i != n {
        let pose = Pose2 { translation: v(x, ZERO), rotation: Rot2 { re: ONE, im: ZERO } };
        parts.append((pose, Shape::Cuboid(CuboidTrait::new(v(QUARTER, QUARTER)))));
        x = x + HALF;
        i += 1;
    }
    ColliderBuilderTrait::compound(parts.span())
}

fn scene(n: u32, halfspace: bool) -> World {
    let mut world = WorldTrait::new(v(ZERO, Fixed { raw: GRAVITY_Y }), Default::default());
    let g = world.insert_collider(ground(halfspace).build(), None);
    let body = RigidBodyTrait::dynamic(
        Pose2 {
            translation: v(Fixed { raw: 429496730 }, QUARTER), rotation: Rot2 { re: ONE, im: ZERO },
        },
    );
    let (_, c) = world.insert(body, plank(n).build());
    assert!(g.index == 0 && c.index == 1);
    world
}

#[inline(never)]
fn probe(n: u32, halfspace: bool, steps: u32) {
    let mut world = scene(n, halfspace);
    let mut k = 0;
    while k != steps {
        let _ = world.step();
        k += 1;
    }
    let g = rapier_core::Handle { index: 0, generation: 0 };
    let c = rapier_core::Handle { index: 1, generation: 0 };
    let manifolds = contact_pair_manifolds(world.narrow_phase.pairs.span(), g, c);
    let mut touching: u32 = 0;
    for m in manifolds {
        if m.data.num_solver_contacts == 2 {
            touching += 1;
        }
    }
    assert!(touching >= n, "every part rests with two contacts");
    let _ = opaque(world.colliders.len());
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
fn gas_setup_compound2_halfspace() {
    probe(opaque(2), opaque(true), WARMUP);
}

#[test]
fn gas_step_compound2_halfspace() {
    probe(opaque(2), opaque(true), WARMUP + 1);
}

#[test]
fn gas_setup_compound4_halfspace() {
    probe(opaque(4), opaque(true), WARMUP);
}

#[test]
fn gas_step_compound4_halfspace() {
    probe(opaque(4), opaque(true), WARMUP + 1);
}

#[test]
fn gas_setup_compound8_halfspace() {
    probe(opaque(8), opaque(true), WARMUP);
}

#[test]
fn gas_step_compound8_halfspace() {
    probe(opaque(8), opaque(true), WARMUP + 1);
}

#[test]
fn gas_setup_compound2_polyline() {
    probe(opaque(2), opaque(false), WARMUP);
}

#[test]
fn gas_step_compound2_polyline() {
    probe(opaque(2), opaque(false), WARMUP + 1);
}

#[test]
fn gas_setup_compound4_polyline() {
    probe(opaque(4), opaque(false), WARMUP);
}

#[test]
fn gas_step_compound4_polyline() {
    probe(opaque(4), opaque(false), WARMUP + 1);
}

#[test]
fn gas_setup_compound8_polyline() {
    probe(opaque(8), opaque(false), WARMUP);
}

#[test]
fn gas_step_compound8_polyline() {
    probe(opaque(8), opaque(false), WARMUP + 1);
}
