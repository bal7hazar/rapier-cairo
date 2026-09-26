//! QY2: `intersect_shape`, `project_point_and_get_feature`, `intersect_aabb_conservative`, the
//! `QueryPipeline` view and the `QueryFilterFlags` / `QueryFilter` completion, through the public
//! API. Every answer is compared with a brute-force pass over all the colliders.

use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
use glam::vec2::Vec2;
use rapier2d::queries::{
    EXCLUDE_DYNAMIC, EXCLUDE_FIXED, EXCLUDE_KINEMATIC, EXCLUDE_SENSORS, EXCLUDE_SOLIDS,
    ONLY_DYNAMIC, ONLY_FIXED, ONLY_KINEMATIC, QueryFilter, QueryFilterFlags, QueryFilterFlagsTrait,
    QueryFilterTrait, QueryPipelineTrait,
};
use rapier2d::world::{World, WorldTrait};
use rapier_core::Handle;
use rapier_core::interaction_groups::{
    Group, InteractionGroups, InteractionGroupsTrait, InteractionTestMode,
};
use rapier_dynamics2d::collider::{ColliderBuilderTrait, ColliderTrait};
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::rigid_body_set::RigidBodyTrait;
use rapier_geometry2d::aabb::AabbTrait;
use rapier_geometry2d::feature_id::FeatureIdTrait;
use rapier_geometry2d::point::PointQuery;
use rapier_geometry2d::query::intersection_test;
use rapier_geometry2d::shape::{Ball, Cuboid, Shape};
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

fn none() -> QueryFilter {
    QueryFilterTrait::new()
}

fn ball(r: Fixed) -> Shape {
    Shape::Ball(Ball { radius: r })
}

/// Handles, in insertion order:
/// 0. standalone ground cuboid (fixed) at (4, 0), half extents 0.5,
/// 1. dynamic ball (radius 0.5) at (0, 0),
/// 2. kinematic ball at (2, 0),
/// 3. standalone sensor ball at (-2, 0),
/// 4. disabled ball at (-4, 0),
/// 5. dynamic ball at (0, 0) again,
/// 6. standalone capsule (half height 0.5, radius 0.25) at (0, 3),
/// 7. a ball at (1, 1) that is inserted, then removed: it never answers.
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
    capsule: Handle,
    removed: Handle,
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
    let capsule = world
        .insert_collider(
            ColliderBuilderTrait::capsule_y(HALF, HALF / TWO).translation(v(ZERO, int(3))).build(),
            None,
        );
    let removed = world
        .insert_collider(ColliderBuilderTrait::ball(HALF).translation(v(ONE, ONE)).build(), None);
    assert!(world.remove_collider(removed).is_some());
    Scene {
        world, ground, dynamic, dynamic_body, kinematic, sensor, disabled, twin, capsule, removed,
    }
}

/// Every enabled collider of `world` that passes `filter` and whose shape meets `shape`.
fn brute_shape(ref world: World, pos: Pose2, shape: Shape, filter: QueryFilter) -> Array<Handle> {
    let mut out = array![];
    for (h, c) in world.colliders.iter() {
        if c.is_enabled() && filter.test(ref world.bodies, h, c) {
            if intersection_test(pos, shape, c.position(), c.shape) == Some(true) {
                out.append(h);
            }
        }
    }
    out
}

#[test]
fn test_intersect_shape_table() {
    let mut s = scene();
    let rot90 = Pose2Trait::new(v(int(3), ZERO), Rot2 { re: ZERO, im: ONE });
    let tall = Shape::Cuboid(Cuboid { half_extents: v(HALF, int(2)) });
    // `(pose, shape, filter, expected)`: the disabled ball at x = -4 and the removed one at (1, 1)
    // never answer.
    let cases: Array<(Pose2, Shape, QueryFilter, Array<Handle>)> = array![
        // A ball on the two coincident dynamic balls (touching the kinematic one at x = 1.5).
        (at(ZERO, ZERO), ball(HALF), none(), array![s.dynamic, s.twin]),
        (at(HALF, ZERO), ball(ONE), none(), array![s.dynamic, s.kinematic, s.twin]),
        // Touching counts: ball radius 0.5 at x = 1 touches the two dynamic balls and the
        // kinematic.
        (at(ONE, ZERO), ball(HALF), none(), array![s.dynamic, s.kinematic, s.twin]),
        (at(ONE, ZERO), ball(HALF), QueryFilterTrait::only_dynamic(), array![s.dynamic, s.twin]),
        (at(ONE, ZERO), ball(HALF), QueryFilterTrait::only_kinematic(), array![s.kinematic]),
        (
            at(ONE, ZERO),
            ball(HALF),
            none().exclude_collider(s.dynamic),
            array![s.kinematic, s.twin],
        ),
        (
            at(ONE, ZERO),
            ball(HALF),
            none().exclude_rigid_body(s.dynamic_body),
            array![s.kinematic, s.twin],
        ),
        // The sensor at x = -2, with and without the sensor filters.
        (at(int(-2), ZERO), ball(HALF), none(), array![s.sensor]),
        (at(int(-2), ZERO), ball(HALF), none().exclude_sensors(), array![]),
        (at(int(-2), ZERO), ball(HALF), QueryFilterTrait::new().exclude_solids(), array![s.sensor]),
        (at(ZERO, ZERO), ball(HALF), QueryFilterTrait::new().exclude_solids(), array![]),
        // A cuboid rotated by 90 degrees: 4 x 1 extents on x; hits the ground, the kinematic.
        (rot90, tall, none(), array![s.ground, s.kinematic]),
        (rot90, tall, QueryFilterTrait::only_fixed(), array![s.ground]),
        (rot90, tall, QueryFilterTrait::exclude_fixed(), array![s.kinematic]),
        // The capsule at (0, 3) and nothing else.
        (at(ZERO, int(4)), ball(HALF), none(), array![s.capsule]),
        // Nothing in reach, an empty answer.
        (at(int(20), int(20)), ball(ONE), none(), array![]),
        // The removed ball's place at (1, 1) only meets the balls it did not replace.
        (at(ONE, ONE), ball(Fixed { raw: 0x1000_0000 }), none(), array![]),
    ];
    for (pos, shape, filter, expected) in cases {
        let got = s.world.intersect_shape(pos, shape, filter);
        assert_eq!(got, expected);
        assert_eq!(got, brute_shape(ref s.world, pos, shape, filter));
    }
    assert!(s.disabled != s.sensor && s.removed != s.ground);
}

#[test]
fn test_intersect_shape_empty_world_and_half_space_pair() {
    let mut world = WorldTrait::new(v(ZERO, -ONE), Default::default());
    assert_eq!(world.intersect_shape(at(ZERO, ZERO), ball(ONE), none()), array![]);
    let floor = world.insert_collider(ColliderBuilderTrait::halfspace(v(ZERO, ONE)).build(), None);
    let ball_in = world
        .insert_collider(ColliderBuilderTrait::ball(HALF).translation(v(ZERO, -ONE)).build(), None);
    // A ball resting on the floor plane (y = 0) meets the half-space only; one at the origin
    // (touching the ball below) meets both.
    let above = Fixed { raw: 0x4000_0000 };
    assert_eq!(world.intersect_shape(at(ZERO, above), ball(above), none()), array![floor]);
    assert_eq!(world.intersect_shape(at(ZERO, ZERO), ball(HALF), none()), array![floor, ball_in]);
    // A half-space against a half-space has no kernel: reported as not intersecting.
    let plane = Shape::HalfSpace(rapier_geometry2d::shape::HalfSpace { normal: v(ZERO, ONE) });
    assert_eq!(world.intersect_shape(at(ZERO, ZERO), plane, none()), array![ball_in]);
}

#[test]
fn test_project_point_and_get_feature_table() {
    let mut s = scene();
    let face0 = FeatureIdTrait::face(0);
    // `(point, max_dist, filter, expected)`.
    let cases: Array<(Vec2, Fixed, QueryFilter, Option<(Handle, Vec2)>)> = array![
        // Above the kinematic ball: its top.
        (v(TWO, TWO), int(100), none(), Some((s.kinematic, v(TWO, HALF)))),
        // Strict bound: exactly 1.5 away is rejected, a hair more is accepted.
        (v(TWO, TWO), int(1) + HALF, none(), None),
        (v(TWO, TWO), int(1) + HALF + Fixed { raw: 1 }, none(), Some((s.kinematic, v(TWO, HALF)))),
        // A tie between the ground (its left face, x = 3.5) and the kinematic ball (x = 2.5)
        // from (3, 0): the lowest handle wins, and the other one once the ground is excluded.
        (v(int(3), ZERO), int(100), none(), Some((s.ground, v(int(3) + HALF, ZERO)))),
        (
            v(int(3), ZERO),
            int(100),
            QueryFilterTrait::exclude_fixed(),
            Some((s.kinematic, v(TWO + HALF, ZERO))),
        ),
        // The sensor is a normal target unless excluded.
        (v(int(-3), ZERO), int(100), none(), Some((s.sensor, v(int(-2) - HALF, ZERO)))),
        (
            v(int(-3), ZERO),
            int(100),
            QueryFilterTrait::new().exclude_solids(),
            Some((s.sensor, v(int(-2) - HALF, ZERO))),
        ),
        // Nothing but sensors is left with `exclude_solids` far from it, within a small bound.
        (v(int(4), TWO), int(1), QueryFilterTrait::new().exclude_solids(), None),
    ];
    for (point, max_dist, filter, expected) in cases {
        let got = s.world.project_point_and_get_feature(point, max_dist, filter);
        match expected {
            None => assert!(got.is_none(), "expected no answer"),
            Some((
                handle, on,
            )) => {
                let (h, proj, feature) = got.unwrap();
                assert_eq!(h, handle);
                assert_eq!(proj.point.y, on.y);
                assert_eq!(proj.point.x, on.x);
                if handle != s.ground {
                    assert_eq!(feature, face0);
                }
            },
        }
    }
}

/// Inside a collider the projection is on the boundary, `is_inside` is set, and ties go to the
/// lowest handle.
#[test]
fn test_project_point_and_get_feature_inside_and_ties() {
    let mut s = scene();
    let (h, proj, feature) = s
        .world
        .project_point_and_get_feature(v(ZERO, ZERO), int(100), QueryFilterTrait::only_dynamic())
        .unwrap();
    // Twin balls at the same place: the lowest handle wins; the ball's inside projection is
    // on its boundary along +x.
    assert_eq!((h, proj.is_inside, feature), (s.dynamic, true, FeatureIdTrait::face(0)));
    assert_eq!(proj.point.x * proj.point.x + proj.point.y * proj.point.y, HALF * HALF);
    // The ground cuboid's feature is the face hit from above: the same as the shape's own answer.
    let ground = s.world.collider(s.ground).unwrap();
    let (_, expected) = ground
        .shape
        .project_point_and_get_feature(ground.position(), v(int(4), TWO));
    let (h, _, feature) = s
        .world
        .project_point_and_get_feature(v(int(4), TWO), int(100), QueryFilterTrait::only_fixed())
        .unwrap();
    assert_eq!((h, feature), (s.ground, expected));
    assert!(feature.is_face() || feature.is_vertex());
}

#[test]
fn test_intersect_aabb_conservative_and_empty_world() {
    let mut s = scene();
    let aabb = AabbTrait::new(v(HALF, -ONE), v(int(4), ONE));
    let expected = array![s.ground, s.dynamic, s.kinematic, s.twin];
    assert_eq!(s.world.intersect_aabb_conservative(aabb, none()), expected);
    assert_eq!(s.world.intersect_aabb(aabb, none()), expected);
    assert_eq!(
        s.world.intersect_aabb_conservative(aabb, QueryFilterTrait::only_dynamic()),
        array![s.dynamic, s.twin],
    );
    // The removed ball's cell (1, 1) is empty, the disabled ball is never reported.
    let corner = AabbTrait::new(v(int(-5), int(-5)), v(int(-3), int(5)));
    assert_eq!(s.world.intersect_aabb_conservative(corner, none()), array![]);
    let mut empty = WorldTrait::new(v(ZERO, -ONE), Default::default());
    assert_eq!(empty.intersect_aabb_conservative(aabb, none()), array![]);
    assert!(empty.project_point_and_get_feature(v(ZERO, ZERO), int(100), none()).is_none());
}

#[test]
fn test_query_filter_flags() {
    let mut s = scene();
    assert_eq!(
        (EXCLUDE_FIXED.bits, EXCLUDE_KINEMATIC.bits, EXCLUDE_DYNAMIC.bits, EXCLUDE_SENSORS.bits),
        (1, 2, 4, 8),
    );
    assert_eq!(
        (EXCLUDE_SOLIDS.bits, ONLY_DYNAMIC.bits, ONLY_KINEMATIC.bits, ONLY_FIXED.bits),
        (16, 3, 5, 6),
    );
    let none_flags: QueryFilterFlags = Default::default();
    assert!(none_flags.is_empty() && !ONLY_FIXED.is_empty());
    assert!(ONLY_DYNAMIC.contains(EXCLUDE_FIXED) && !ONLY_DYNAMIC.contains(EXCLUDE_DYNAMIC));
    assert_eq!(EXCLUDE_FIXED.union(EXCLUDE_KINEMATIC), ONLY_DYNAMIC);
    assert_eq!(QueryFilterFlagsTrait::from_bits(6), ONLY_FIXED);
    // `test` against every kind of collider: (collider, [none, fixed, kinematic, dynamic,
    // sensors, solids, only_dynamic, only_fixed]).
    let flags = array![
        none_flags, EXCLUDE_FIXED, EXCLUDE_KINEMATIC, EXCLUDE_DYNAMIC, EXCLUDE_SENSORS,
        EXCLUDE_SOLIDS, ONLY_DYNAMIC, ONLY_FIXED,
    ];
    let rows: Array<(Handle, Array<bool>)> = array![
        // standalone fixed ground: only `exclude_fixed`, `only_dynamic` and `exclude_solids` reject
        // it.
        (s.ground, array![true, false, true, true, true, false, false, true]),
        (s.dynamic, array![true, true, true, false, true, false, true, false]),
        (s.kinematic, array![true, true, false, true, true, false, false, false]),
        // a standalone sensor: standalone counts as fixed.
        (s.sensor, array![true, false, true, true, false, true, false, true]),
    ];
    for (handle, expected) in rows {
        let collider = s.world.collider(handle).unwrap();
        let mut i = 0;
        for f in flags.span() {
            assert_eq!(f.test(ref s.world.bodies, collider), *expected.at(i), "flag row");
            i += 1;
        }
    }
}

#[test]
fn test_query_filter_conversions_and_exclude_solids() {
    let g1: Group = 1_u32.into();
    let groups: InteractionGroups = InteractionGroupsTrait::new(g1, g1, InteractionTestMode::And);
    let from_flags: QueryFilter = ONLY_FIXED.into();
    assert_eq!(from_flags, QueryFilterTrait::only_fixed());
    let from_groups: QueryFilter = groups.into();
    assert_eq!(from_groups, none().groups(groups));
    assert_eq!(from_groups.flags, Default::default());
    let f = none().exclude_sensors().exclude_solids().exclude_solids();
    assert_eq!(f.flags.bits, 24);
    assert_eq!(QueryFilterTrait::only_dynamic().exclude_solids().flags.bits, 19);
    // Sensors and solids both excluded: nothing passes.
    let mut s = scene();
    assert_eq!(s.world.intersect_point(v(int(-2), ZERO), f), array![]);
    assert_eq!(
        s.world.intersect_point(v(int(-2), ZERO), none().exclude_solids()), array![s.sensor],
    );
}

#[test]
fn test_query_pipeline_view_matches_the_world_queries() {
    let mut s = scene();
    let view = s.world.query_pipeline();
    assert_eq!(view.filter, none());
    let filtered = s.world.query_pipeline_with_filter(QueryFilterTrait::only_dynamic());
    assert_eq!(filtered, view.with_filter(QueryFilterTrait::only_dynamic()));
    // `with_filter` replaces the filter.
    assert_eq!(filtered.with_filter(none()), view);
    let ray = rapier_geometry2d::ray::Ray { origin: v(int(-10), ZERO), dir: v(ONE, ZERO) };
    let f = QueryFilterTrait::only_dynamic();
    assert_eq!(
        filtered.cast_ray(ref s.world, ray, int(100), true),
        s.world.cast_ray(ray, int(100), true, f),
    );
    assert_eq!(
        view.cast_ray(ref s.world, ray, int(100), true),
        s.world.cast_ray(ray, int(100), true, none()),
    );
    assert_eq!(
        filtered.cast_ray_and_get_normal(ref s.world, ray, int(100), true),
        s.world.cast_ray_and_get_normal(ray, int(100), true, f),
    );
    assert_eq!(
        filtered.intersect_ray(ref s.world, ray, int(100), true).len(),
        s.world.intersect_ray(ray, int(100), true, f).len(),
    );
    let p = v(TWO, TWO);
    assert_eq!(
        filtered.project_point(ref s.world, p, int(100), false),
        s.world.project_point(p, int(100), false, f),
    );
    assert_eq!(
        filtered.project_point_and_get_feature(ref s.world, p, int(100)),
        s.world.project_point_and_get_feature(p, int(100), f),
    );
    assert_eq!(filtered.intersect_point(ref s.world, v(ZERO, ZERO)), array![s.dynamic, s.twin]);
    let aabb = AabbTrait::new(v(HALF, -ONE), v(int(4), ONE));
    assert_eq!(filtered.intersect_aabb_conservative(ref s.world, aabb), array![s.dynamic, s.twin]);
    assert_eq!(
        filtered.intersect_shape(ref s.world, at(ONE, ZERO), ball(HALF)), array![s.dynamic, s.twin],
    );
}

/// `intersect_shape` is the exact test on every collider (no pruning may drop one) and
/// `project_point_and_get_feature` the minimum over the colliders, ties to the lowest handle.
#[test]
#[fuzzer(runs: 32, seed: 7)]
fn fuzz_shape_queries_match_brute_force(px: i16, py: i8, radius: u8) {
    let mut s = scene();
    let pos = at(Fixed { raw: px.into() * 0x4_0000 }, Fixed { raw: py.into() * 0x400_0000 });
    let shape = ball(Fixed { raw: radius.into() * 0x400_0000 });
    let filter = none();
    assert_eq!(
        s.world.intersect_shape(pos, shape, filter), brute_shape(ref s.world, pos, shape, filter),
    );
    let point = pos.translation;
    let got = s.world.project_point_and_get_feature(point, int(1000), filter);
    let mut best: Option<(Handle, i128)> = Option::None;
    for (h, c) in s.world.colliders.iter() {
        if !c.is_enabled() {
            continue;
        }
        let (proj, _) = c.shape.project_point_and_get_feature(c.position(), point);
        let d = rapier_math::math_ext::norm2::norm2_sq_wide(
            proj.point.x - point.x, proj.point.y - point.y,
        );
        let better = match best {
            Option::Some((_, b)) => d < b,
            Option::None => true,
        };
        if better {
            best = Option::Some((h, d));
        }
    }
    match (got, best) {
        (Option::Some((h, _, _)), Option::Some((bh, _))) => assert_eq!(h, bh),
        (Option::None, Option::None) => {},
        _ => panic!("brute force disagrees"),
    }
}
