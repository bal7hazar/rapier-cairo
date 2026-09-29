//! SH2a per-step cost of a ball and a box resting on a composite ground of 10 or 50 segments
//! (polyline and heightfield): `gas_setup_*` builds the scene and runs `WARMUP` steps,
//! `gas_step_*` one more; the difference is one warm-started `World::step`. The ground spans
//! `[-10, 10]` with heights alternating `0` and `0.05`; the body rests near the middle, so the
//! prefilter keeps one or two parts whatever the segment count. Compare with the P3 half-space
//! scenes `balls_halfspace1` and `cuboid_stack1` (`gas_scenes.cairo`).

use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam_core::Vec2;
use rapier2d::world::{World, WorldTrait};
use rapier_dynamics2d::collider::{ColliderBuilder, ColliderBuilderTrait};
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::narrow_phase::composite::contact_pair_manifolds;
use rapier_dynamics2d::rigid_body_set::RigidBodyTrait;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;

const WARMUP: u32 = 3;
const GRAVITY_Y: i64 = -42133629174;
/// `0.05`.
const BUMP: Fixed = Fixed { raw: 214748365 };

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

/// `n` segments over `[-10, 10]`: a heightfield (`heightfield`) or a chained polyline.
fn ground(n: u32, heightfield: bool) -> ColliderBuilder {
    let mut heights = array![];
    let mut vertices = array![];
    let width: Fixed = FixedTrait::from_int(20);
    let step = width / FixedTrait::from_int(n.try_into().unwrap());
    let mut x: Fixed = FixedTrait::from_int(-10);
    let mut k: u32 = 0;
    while k != n + 1 {
        let h = if k % 2 == 0 {
            ZERO
        } else {
            BUMP
        };
        heights.append(h);
        vertices.append(v(x, h));
        x = x + step;
        k += 1;
    }
    if heightfield {
        ColliderBuilderTrait::heightfield(heights.span(), v(width, ONE))
    } else {
        ColliderBuilderTrait::polyline(vertices.span(), None)
    }
}

/// `kind`: 0 a ball of radius 0.5, 1 a box of half-extents 0.5.
fn scene(kind: u32, n: u32, heightfield: bool) -> World {
    let mut world = WorldTrait::new(v(ZERO, Fixed { raw: GRAVITY_Y }), Default::default());
    let g = world.insert_collider(ground(n, heightfield).build(), None);
    let body = RigidBodyTrait::dynamic(
        Pose2 {
            translation: v(Fixed { raw: 858993459 }, HALF + BUMP),
            rotation: Rot2 { re: ONE, im: ZERO },
        },
    );
    let shape = if kind == 0 {
        ColliderBuilderTrait::ball(HALF)
    } else {
        ColliderBuilderTrait::cuboid(HALF, HALF)
    };
    let (_, c) = world.insert(body, shape.build());
    assert!(g.index == 0 && c.index == 1);
    world
}

#[inline(never)]
fn probe(kind: u32, n: u32, heightfield: bool, steps: u32) {
    let mut world = scene(kind, n, heightfield);
    let mut k = 0;
    while k != steps {
        let _ = world.step();
        k += 1;
    }
    let g = rapier_core::Handle { index: 0, generation: 0 };
    let c = rapier_core::Handle { index: 1, generation: 0 };
    let manifolds = contact_pair_manifolds(world.narrow_phase.pairs.span(), g, c);
    assert!(manifolds.len() >= 1 && manifolds.len() <= 3, "parts kept by the prefilter");
    let _ = opaque(world.colliders.len());
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
fn gas_setup_ball_polyline10() {
    probe(opaque(0), opaque(10), opaque(false), WARMUP);
}

#[test]
fn gas_step_ball_polyline10() {
    probe(opaque(0), opaque(10), opaque(false), WARMUP + 1);
}

#[test]
fn gas_setup_ball_polyline50() {
    probe(opaque(0), opaque(50), opaque(false), WARMUP);
}

#[test]
fn gas_step_ball_polyline50() {
    probe(opaque(0), opaque(50), opaque(false), WARMUP + 1);
}

#[test]
fn gas_setup_box_polyline10() {
    probe(opaque(1), opaque(10), opaque(false), WARMUP);
}

#[test]
fn gas_step_box_polyline10() {
    probe(opaque(1), opaque(10), opaque(false), WARMUP + 1);
}

#[test]
fn gas_setup_box_polyline50() {
    probe(opaque(1), opaque(50), opaque(false), WARMUP);
}

#[test]
fn gas_step_box_polyline50() {
    probe(opaque(1), opaque(50), opaque(false), WARMUP + 1);
}

#[test]
fn gas_setup_ball_heightfield10() {
    probe(opaque(0), opaque(10), opaque(true), WARMUP);
}

#[test]
fn gas_step_ball_heightfield10() {
    probe(opaque(0), opaque(10), opaque(true), WARMUP + 1);
}

#[test]
fn gas_setup_ball_heightfield50() {
    probe(opaque(0), opaque(50), opaque(true), WARMUP);
}

#[test]
fn gas_step_ball_heightfield50() {
    probe(opaque(0), opaque(50), opaque(true), WARMUP + 1);
}

#[test]
fn gas_setup_box_heightfield10() {
    probe(opaque(1), opaque(10), opaque(true), WARMUP);
}

#[test]
fn gas_step_box_heightfield10() {
    probe(opaque(1), opaque(10), opaque(true), WARMUP + 1);
}

#[test]
fn gas_setup_box_heightfield50() {
    probe(opaque(1), opaque(50), opaque(true), WARMUP);
}

#[test]
fn gas_step_box_heightfield50() {
    probe(opaque(1), opaque(50), opaque(true), WARMUP + 1);
}
