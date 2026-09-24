//! Scene queries through `WorldTrait`: filters, ties, sensors, disabled colliders, ordering of
//! the array queries, and a fuzzed comparison of `cast_ray` with the per-collider casts.

use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
use glam::vec2::Vec2;
use rapier2d::queries::{QueryFilter, QueryFilterTrait};
use rapier2d::world::{World, WorldTrait};
use rapier_core::Handle;
use rapier_core::interaction_groups::{Group, InteractionGroupsTrait, InteractionTestMode};
use rapier_dynamics2d::collider::{ColliderBuilderTrait, ColliderTrait};
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::rigid_body_set::RigidBodyTrait;
use rapier_geometry2d::aabb::AabbTrait;
use rapier_geometry2d::feature_id::FeatureIdTrait;
use rapier_geometry2d::ray::{Ray, cast_ray};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;

fn int(x: i32) -> Fixed {
    FixedTrait::from_int(x)
}

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2Trait::new(v(x, y), Rot2 { re: ONE, im: ZERO })
}

fn along_x(y: Fixed) -> Ray {
    Ray { origin: v(int(-10), y), dir: v(ONE, ZERO) }
}

fn none() -> QueryFilter {
    QueryFilterTrait::new()
}

/// Handles, in insertion order:
/// 0. standalone ground cuboid (fixed) at x = 4,
/// 1. dynamic ball at x = 0,
/// 2. kinematic ball at x = 2,
/// 3. sensor ball (standalone) at x = -2,
/// 4. disabled ball at x = -4,
/// 5. dynamic ball at x = 0 again (same place as 1: a tie).
#[derive(Destruct)]
struct Scene {
    world: World,
    ground: Handle,
    dynamic: Handle,
    dynamic_body: Handle,
    kinematic: Handle,
    sensor: Handle,
    disabled: Handle,
    twin: Handle,
}

fn scene() -> Scene {
    let mut world = WorldTrait::new(v(ZERO, -ONE), Default::default());
    let ground = world
        .insert_collider(
            ColliderBuilderTrait::cuboid(HALF, HALF).translation(v(int(4), ZERO)).build(), None,
        );
    let (dynamic_body, dynamic) = world
        .insert(RigidBodyTrait::dynamic(at(ZERO, ZERO)), ColliderBuilderTrait::ball(HALF).build());
    let (_, kinematic) = world
        .insert(
            RigidBodyTrait::kinematic_position_based(at(TWO, ZERO)),
            ColliderBuilderTrait::ball(HALF).build(),
        );
    let sensor = world
        .insert_collider(
            ColliderBuilderTrait::ball(HALF).translation(v(int(-2), ZERO)).sensor(true).build(),
            None,
        );
    let disabled = world
        .insert_collider(
            ColliderBuilderTrait::ball(HALF).translation(v(int(-4), ZERO)).enabled(false).build(),
            None,
        );
    let (_, twin) = world
        .insert(RigidBodyTrait::dynamic(at(ZERO, ZERO)), ColliderBuilderTrait::ball(HALF).build());
    Scene { world, ground, dynamic, dynamic_body, kinematic, sensor, disabled, twin }
}

#[test]
fn test_cast_ray_filters() {
    let mut s = scene();
    let ray = along_x(ZERO);
    let max = int(100);
    // `(filter, expected collider, expected toi)`: the disabled ball at x = -4 never answers.
    let cases: Array<(QueryFilter, Handle, Fixed)> = array![
        (none(), s.sensor, int(7) + HALF), (none().exclude_sensors(), s.dynamic, int(9) + HALF),
        (QueryFilterTrait::exclude_fixed(), s.dynamic, int(9) + HALF),
        (QueryFilterTrait::only_kinematic(), s.kinematic, int(11) + HALF),
        (QueryFilterTrait::only_fixed(), s.sensor, int(7) + HALF),
        (QueryFilterTrait::only_fixed().exclude_sensors(), s.ground, int(13) + HALF),
        (QueryFilterTrait::exclude_dynamic().exclude_sensors(), s.kinematic, int(11) + HALF),
        (none().exclude_sensors().exclude_collider(s.dynamic), s.twin, int(9) + HALF),
        (none().exclude_sensors().exclude_rigid_body(s.dynamic_body), s.twin, int(9) + HALF),
    ];
    for (filter, handle, toi) in cases {
        assert_eq!(s.world.cast_ray(ray, max, true, filter), Some((handle, toi)));
        let (h, hit) = s.world.cast_ray_and_get_normal(ray, max, true, filter).unwrap();
        assert_eq!((h, hit.time_of_impact), (handle, toi));
    }
    assert!(s.disabled != s.sensor);
}

#[test]
fn test_ties_go_to_the_lowest_handle_and_bounds_are_strict() {
    let mut s = scene();
    let filter = QueryFilterTrait::exclude_fixed();
    let ray = along_x(ZERO);
    // `dynamic` and `twin` are hit at the same time: the lower handle wins.
    assert_eq!(s.world.cast_ray(ray, int(100), true, filter), Some((s.dynamic, int(9) + HALF)));
    // A hit exactly at `max_toi` is rejected, as upstream's `find_best`.
    assert_eq!(s.world.cast_ray(ray, int(9) + HALF, true, filter), None);
    assert!(s.world.cast_ray(ray, int(9) + HALF + Fixed { raw: 1 }, true, filter).is_some());
}

#[test]
fn test_collision_groups() {
    let mut world = WorldTrait::new(v(ZERO, -ONE), Default::default());
    let g1: Group = 1_u32.into();
    let g2: Group = 2_u32.into();
    let near = world
        .insert_collider(
            ColliderBuilderTrait::ball(HALF)
                .collision_groups(InteractionGroupsTrait::new(g1, g1, InteractionTestMode::And))
                .build(),
            None,
        );
    let far = world
        .insert_collider(
            ColliderBuilderTrait::ball(HALF)
                .translation(v(int(3), ZERO))
                .collision_groups(InteractionGroupsTrait::new(g2, g2, InteractionTestMode::And))
                .build(),
            None,
        );
    let ray = along_x(ZERO);
    let only2 = none().groups(InteractionGroupsTrait::new(g2, g2, InteractionTestMode::And));
    assert_eq!(world.cast_ray(ray, int(100), true, only2), Some((far, int(12) + HALF)));
    assert_eq!(world.cast_ray(ray, int(100), true, none()), Some((near, int(9) + HALF)));
}

#[test]
fn test_intersect_ray_lists_every_hit_in_handle_order() {
    let mut s = scene();
    let hits = s.world.intersect_ray(along_x(ZERO), int(100), true, none());
    let mut handles = array![];
    for (h, hit) in hits.span() {
        handles.append(*h);
        assert!(*hit.feature == FeatureIdTrait::face(0) || *h == s.ground, "ball face");
    }
    assert_eq!(handles, array![s.ground, s.dynamic, s.kinematic, s.sensor, s.twin]);
    // Inclusive per-shape bound: the sensor is hit at exactly 7.5.
    let hits = s.world.intersect_ray(along_x(ZERO), int(7) + HALF, true, none());
    assert_eq!(hits.len(), 1);
}

#[test]
fn test_hollow_cast_from_inside() {
    let mut s = scene();
    let ray = Ray { origin: v(ZERO, ZERO), dir: v(ONE, ZERO) };
    let filter = QueryFilterTrait::exclude_fixed();
    assert_eq!(s.world.cast_ray(ray, int(100), true, filter), Some((s.dynamic, ZERO)));
    let (h, hit) = s.world.cast_ray_and_get_normal(ray, int(100), false, filter).unwrap();
    assert_eq!((h, hit.time_of_impact, hit.normal), (s.dynamic, HALF, v(-ONE, ZERO)));
}

#[test]
fn test_point_and_aabb_queries() {
    let mut s = scene();
    // Inside both balls at the origin: solid distance 0, lowest handle.
    let (h, proj) = s.world.project_point(v(ZERO, ZERO), int(100), true, none()).unwrap();
    assert_eq!((h, proj.is_inside, proj.point), (s.dynamic, true, v(ZERO, ZERO)));
    // From above the kinematic ball.
    let (h, proj) = s.world.project_point(v(TWO, TWO), int(100), false, none()).unwrap();
    assert_eq!((h, proj.is_inside, proj.point), (s.kinematic, false, v(TWO, HALF)));
    // Strict bound: the kinematic ball is exactly 1.5 away.
    assert!(s.world.project_point(v(TWO, TWO), int(1) + HALF, false, none()).is_none());
    assert_eq!(s.world.intersect_point(v(ZERO, HALF), none()), array![s.dynamic, s.twin]);
    assert_eq!(s.world.intersect_point(v(int(-4), ZERO), none()), array![]);
    let aabb = AabbTrait::new(v(HALF, -ONE), v(int(4), ONE));
    assert_eq!(
        s.world.intersect_aabb(aabb, none()), array![s.ground, s.dynamic, s.kinematic, s.twin],
    );
    assert_eq!(
        s.world.intersect_aabb(aabb, QueryFilterTrait::only_dynamic()), array![s.dynamic, s.twin],
    );
}

/// The world answer is the minimum over the colliders of the per-shape casts, ties to the
/// lowest handle.
#[test]
#[fuzzer(runs: 32, seed: 3)]
fn fuzz_cast_ray_is_the_minimum(oy: i16, dx: i8, dy: i8) {
    let mut s = scene();
    let ray = Ray {
        origin: v(int(-10), Fixed { raw: oy.into() * 0x1_0000 }),
        dir: v(Fixed { raw: dx.into() * 0x100_0000 }, Fixed { raw: dy.into() * 0x10_0000 }),
    };
    let got = s.world.cast_ray(ray, int(1000), true, none());
    let mut best: Option<(Handle, Fixed)> = None;
    for (h, c) in s.world.colliders.iter() {
        if !c.is_enabled() {
            continue;
        }
        if let Some(t) = cast_ray(c.shape, c.position(), ray, int(1000), true) {
            let better = match best {
                Some((_, b)) => t < b,
                None => t < int(1000),
            };
            if better {
                best = Some((h, t));
            }
        }
    }
    assert_eq!(got, best);
}
