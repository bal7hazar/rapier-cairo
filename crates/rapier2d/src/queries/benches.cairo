//! Gas probes of the scene queries: the brute-force winner and the broad-phase-assisted
//! candidates of `super::alternatives`, on rows of balls (16 per row, one unit apart) hit by a
//! ray along the first row. Subtract the matching `gas_setup_*` probe to get the query alone.

use fixed::{Fixed, FixedTrait, HALF};
use glam::vec2::Vec2;
use rapier_dynamics2d::collider::ColliderBuilderTrait;
use rapier_geometry2d::aabb::AabbTrait;
use rapier_geometry2d::ray::Ray;
use rapier_testing::opaque;
use crate::world::{World, WorldTrait};
use super::alternatives::{cast_ray_aabb_pretest, cast_ray_grid};
use super::{
    QueryFilterTrait, cast_ray, cast_ray_and_get_normal, intersect_aabb, intersect_point,
    intersect_ray, project_point,
};

fn int(x: i32) -> Fixed {
    FixedTrait::from_int(x)
}

/// `n` standalone balls of radius 0.4 at `(i % 16, i / 16)`.
fn scene(n: u32) -> World {
    let mut world = WorldTrait::new(Vec2 { x: int(0), y: int(-1) }, Default::default());
    let mut i: u32 = 0;
    while i != n {
        let x: i32 = (i % 16).try_into().unwrap();
        let y: i32 = (i / 16).try_into().unwrap();
        let collider = ColliderBuilderTrait::ball(Fixed { raw: 0x6666_6666 })
            .translation(Vec2 { x: int(x), y: int(y) })
            .build();
        let _ = world.insert_collider(collider, None);
        i += 1;
    }
    world
}

/// Along the first row from `x = -2`, `max_toi = 100`.
fn ray() -> Ray {
    opaque(
        Ray {
            origin: Vec2 { x: int(-2), y: Fixed { raw: 0x1000_0000 } },
            dir: Vec2 { x: int(1), y: int(0) },
        },
    )
}

#[test]
fn gas_baseline() {}

#[test]
fn gas_setup_8() {
    let _ = scene(opaque(8));
}

#[test]
fn gas_setup_32() {
    let _ = scene(opaque(32));
}

#[test]
fn gas_setup_128() {
    let _ = scene(opaque(128));
}

#[test]
fn gas_cast_ray_8() {
    let mut world = scene(opaque(8));
    let _ = cast_ray(ref world, ray(), int(100), true, QueryFilterTrait::new());
}

#[test]
fn gas_cast_ray_32() {
    let mut world = scene(opaque(32));
    let _ = cast_ray(ref world, ray(), int(100), true, QueryFilterTrait::new());
}

#[test]
fn gas_cast_ray_128() {
    let mut world = scene(opaque(128));
    let _ = cast_ray(ref world, ray(), int(100), true, QueryFilterTrait::new());
}

#[test]
fn gas_cast_ray_aabb_pretest_8() {
    let mut world = scene(opaque(8));
    let _ = cast_ray_aabb_pretest(ref world, ray(), int(100), true, QueryFilterTrait::new());
}

#[test]
fn gas_cast_ray_aabb_pretest_32() {
    let mut world = scene(opaque(32));
    let _ = cast_ray_aabb_pretest(ref world, ray(), int(100), true, QueryFilterTrait::new());
}

#[test]
fn gas_cast_ray_aabb_pretest_128() {
    let mut world = scene(opaque(128));
    let _ = cast_ray_aabb_pretest(ref world, ray(), int(100), true, QueryFilterTrait::new());
}

#[test]
fn gas_cast_ray_grid_128() {
    let mut world = scene(opaque(128));
    let _ = cast_ray_grid(ref world, ray(), int(100), true, QueryFilterTrait::new());
}

#[test]
fn gas_cast_ray_and_get_normal_32() {
    let mut world = scene(opaque(32));
    let _ = cast_ray_and_get_normal(ref world, ray(), int(100), true, QueryFilterTrait::new());
}

#[test]
fn gas_intersect_ray_32() {
    let mut world = scene(opaque(32));
    let _ = intersect_ray(ref world, ray(), int(100), true, QueryFilterTrait::new());
}

#[test]
fn gas_project_point_32() {
    let mut world = scene(opaque(32));
    let p = opaque(Vec2 { x: HALF, y: HALF });
    let _ = project_point(ref world, p, int(100), true, QueryFilterTrait::new());
}

#[test]
fn gas_intersect_point_32() {
    let mut world = scene(opaque(32));
    let _ = intersect_point(
        ref world, opaque(Vec2 { x: int(1), y: int(1) }), QueryFilterTrait::new(),
    );
}

#[test]
fn gas_intersect_aabb_32() {
    let mut world = scene(opaque(32));
    let aabb = opaque(AabbTrait::new(Vec2 { x: HALF, y: HALF }, Vec2 { x: int(3), y: int(1) }));
    let _ = intersect_aabb(ref world, aabb, QueryFilterTrait::new());
}
