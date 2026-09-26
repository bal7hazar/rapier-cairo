//! Gas probes of the QY2 queries on a 20-collider world (5 × 4 grid, one unit apart, four shape
//! kinds, every fourth a sensor). Subtract `gas_setup_20` to get the query alone; the older
//! queries are probed on the same scene (`*_20`) for comparison.

use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam::vec2::Vec2;
use rapier_dynamics2d::collider::ColliderBuilderTrait;
use rapier_geometry2d::aabb::AabbTrait;
use rapier_geometry2d::ray::Ray;
use rapier_geometry2d::shape::{Ball, Shape};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::world::{World, WorldTrait};
use super::alternatives::intersect_shape_direct;
use super::{
    QueryFilterTrait, QueryPipelineTrait, cast_ray, intersect_aabb, intersect_aabb_conservative,
    intersect_point, intersect_shape, project_point, project_point_and_get_feature,
};

fn int(x: i32) -> Fixed {
    FixedTrait::from_int(x)
}

/// 20 standalone colliders at `(i % 5, i / 5)`: balls, boxes, capsules, balls again (`i % 4`),
/// the ones with `i % 8 == 3` sensors.
fn scene() -> World {
    let mut world = WorldTrait::new(Vec2 { x: ZERO, y: -ONE }, Default::default());
    let mut i: u32 = 0;
    while i != opaque(20) {
        let x: i32 = (i % 5).try_into().unwrap();
        let y: i32 = (i / 5).try_into().unwrap();
        let builder = match i % 4 {
            0 => ColliderBuilderTrait::ball(HALF),
            1 => ColliderBuilderTrait::cuboid(HALF, Fixed { raw: 0x6666_6666 }),
            2 => ColliderBuilderTrait::capsule_y(
                Fixed { raw: 0x4CCC_CCCC }, Fixed { raw: 0x4CCC_CCCC },
            ),
            _ => ColliderBuilderTrait::ball(Fixed { raw: 0x6666_6666 }).sensor(i % 8 == 3),
        };
        let _ = world
            .insert_collider(builder.translation(Vec2 { x: int(x), y: int(y) }).build(), None);
        i += 1;
    }
    world
}

fn pose() -> Pose2 {
    opaque(Pose2Trait::new(Vec2 { x: ONE + HALF, y: ONE }, Rot2 { re: ONE, im: ZERO }))
}

/// A ball of radius 0.75: meets roughly six neighbours.
fn ball() -> Shape {
    opaque(Shape::Ball(Ball { radius: Fixed { raw: 0x3000_0000 } }))
}

#[test]
fn gas_baseline() {}

#[test]
fn gas_setup_20() {
    let _ = scene();
}

#[test]
fn gas_intersect_shape_20() {
    let mut world = scene();
    let _ = intersect_shape(ref world, pose(), ball(), QueryFilterTrait::new());
}

#[test]
fn gas_intersect_shape_direct_20() {
    let mut world = scene();
    let _ = intersect_shape_direct(ref world, pose(), ball(), QueryFilterTrait::new());
}

#[test]
fn gas_intersect_shape_filtered_20() {
    let mut world = scene();
    let _ = intersect_shape(
        ref world, pose(), ball(), QueryFilterTrait::only_fixed().exclude_sensors(),
    );
}

#[test]
fn gas_project_point_and_get_feature_20() {
    let mut world = scene();
    let p = opaque(Vec2 { x: HALF, y: HALF });
    let _ = project_point_and_get_feature(ref world, p, int(100), QueryFilterTrait::new());
}

#[test]
fn gas_project_point_20() {
    let mut world = scene();
    let p = opaque(Vec2 { x: HALF, y: HALF });
    let _ = project_point(ref world, p, int(100), false, QueryFilterTrait::new());
}

#[test]
fn gas_intersect_aabb_conservative_20() {
    let mut world = scene();
    let aabb = opaque(AabbTrait::new(Vec2 { x: HALF, y: HALF }, Vec2 { x: int(3), y: int(1) }));
    let _ = intersect_aabb_conservative(ref world, aabb, QueryFilterTrait::new());
}

#[test]
fn gas_intersect_aabb_20() {
    let mut world = scene();
    let aabb = opaque(AabbTrait::new(Vec2 { x: HALF, y: HALF }, Vec2 { x: int(3), y: int(1) }));
    let _ = intersect_aabb(ref world, aabb, QueryFilterTrait::new());
}

#[test]
fn gas_intersect_point_20() {
    let mut world = scene();
    let _ = intersect_point(ref world, opaque(Vec2 { x: ONE, y: ONE }), QueryFilterTrait::new());
}

#[test]
fn gas_cast_ray_20() {
    let mut world = scene();
    let ray = opaque(Ray { origin: Vec2 { x: int(-2), y: ZERO }, dir: Vec2 { x: ONE, y: ZERO } });
    let _ = cast_ray(ref world, ray, int(100), true, QueryFilterTrait::new());
}

#[test]
fn gas_cast_ray_flags_20() {
    let mut world = scene();
    let ray = opaque(Ray { origin: Vec2 { x: int(-2), y: ZERO }, dir: Vec2 { x: ONE, y: ZERO } });
    let filter = QueryFilterTrait::new().exclude_sensors().exclude_solids();
    let _ = cast_ray(ref world, ray, int(100), true, filter);
}

/// The view adds nothing over the free function: same cost as `gas_cast_ray_20`.
#[test]
fn gas_query_pipeline_cast_ray_20() {
    let mut world = scene();
    let ray = opaque(Ray { origin: Vec2 { x: int(-2), y: ZERO }, dir: Vec2 { x: ONE, y: ZERO } });
    let view = world.query_pipeline_with_filter(QueryFilterTrait::new());
    let _ = view.cast_ray(ref world, ray, int(100), true);
}
