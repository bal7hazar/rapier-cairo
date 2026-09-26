//! Tests and `gas_*` probes of the scene shape casts, on the 20-collider scene of
//! `super::super::benches_qy2` (5 × 4 grid one unit apart: balls, boxes, capsules, sensors).
//! Subtract `gas_setup_20` to get the query alone.

use fixed::{FRAC_PI_2, Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
use glam::vec2::Vec2;
use rapier_dynamics2d::collider::{ColliderBuilderTrait, ColliderTrait};
use rapier_geometry2d::query::{
    NonlinearRigidMotion, NonlinearRigidMotionTrait, ShapeCastHitTrait, ShapeCastOptions,
    ShapeCastOptionsTrait, ShapeCastStatus, cast_shapes,
};
use rapier_geometry2d::shape::{Ball, Cuboid, HalfSpace, Shape};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::world::{World, WorldTrait};
use super::alternatives::{cast_shape_direct, cast_shape_nonlinear_direct};
use super::super::{QueryFilterTrait, QueryPipelineTrait, candidates};
use super::{cast_shape, cast_shape_nonlinear};

fn int(x: i32) -> Fixed {
    FixedTrait::from_int(x)
}

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2Trait::new(v(x, y), Rot2 { re: ONE, im: ZERO })
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
            0 => ColliderBuilderTrait::ball(Fixed { raw: 0x3000_0000 }),
            1 => ColliderBuilderTrait::cuboid(
                Fixed { raw: 0x3000_0000 }, Fixed { raw: 0x2000_0000 },
            ),
            2 => ColliderBuilderTrait::capsule_y(
                Fixed { raw: 0x2000_0000 }, Fixed { raw: 0x2000_0000 },
            ),
            _ => ColliderBuilderTrait::ball(Fixed { raw: 0x2000_0000 }).sensor(i % 8 == 3),
        };
        let _ = world
            .insert_collider(builder.translation(Vec2 { x: int(x), y: int(y) }).build(), None);
        i += 1;
    }
    world
}

fn small_box() -> Shape {
    Shape::Cuboid(Cuboid { half_extents: v(ratio(1, 8), ratio(1, 8)) })
}

fn ratio(n: i64, d: i64) -> Fixed {
    FixedTrait::from_ratio(n, d)
}

/// `(start, velocity)` of the linear casts: along a row, between rows, diagonal, from inside a
/// collider, missing everything.
fn casts() -> Array<(Pose2, Vec2)> {
    array![
        (at(-TWO, ZERO), v(ONE, ZERO)), (at(-TWO, HALF), v(ONE, ZERO)),
        (at(-ONE, -ONE), v(ONE, ONE)), (at(int(2), int(1)), v(ZERO, ONE)),
        (at(-TWO, int(10)), v(ONE, ZERO)), (at(int(6), ratio(3, 2)), v(-ONE, ratio(1, 10))),
    ]
}

/// The pre-tested winner answers exactly what the exact cast on every candidate answers.
#[test]
fn test_cast_shape_matches_direct() {
    let mut world = scene();
    let filter = QueryFilterTrait::new();
    for (pos, vel) in casts() {
        for opts in array![
            Default::default(), ShapeCastOptionsTrait::with_max_time_of_impact(TWO),
        ] {
            let opts: ShapeCastOptions = opts;
            assert_eq!(
                cast_shape(ref world, pos, vel, small_box(), opts, filter),
                cast_shape_direct(ref world, pos, vel, small_box(), opts, filter),
            );
        }
    }
}

/// World frames: `witness1` / `normal1` on the collider in world space, `witness2` / `normal2` in
/// the cast shape's frame; the filter and the lowest-handle tie rule.
#[test]
fn test_cast_shape_frames_filter_and_ties() {
    let mut world = scene();
    let filter = QueryFilterTrait::new();
    // Along the first row from x = -2: the ball at the origin (handle 0, radius 0.1875) first.
    let (handle, hit) = cast_shape(
        ref world, at(-TWO, ZERO), v(ONE, ZERO), small_box(), Default::default(), filter,
    )
        .unwrap();
    let (h0, c0) = *candidates(ref world, filter).at(0);
    assert_eq!(handle, h0);
    assert_eq!(hit.time_of_impact, TWO - ratio(3, 16) - ratio(1, 8));
    assert_eq!((hit.normal1, hit.normal2), (v(-ONE, ZERO), v(ONE, ZERO)));
    assert_eq!((hit.witness1, hit.witness2), (v(-ratio(3, 16), ZERO), v(ratio(1, 8), ZERO)));
    assert_eq!(hit.status, ShapeCastStatus::Converged);
    // Excluding it, the box at (1, 0) is next.
    let (next, hit) = cast_shape(
        ref world,
        at(-TWO, ZERO),
        v(ONE, ZERO),
        small_box(),
        Default::default(),
        filter.exclude_collider(h0),
    )
        .unwrap();
    assert!(next != h0);
    assert_eq!(hit.time_of_impact, int(3) - ratio(3, 16) - ratio(1, 8));
    // Two identical balls hit at the same time: the lower handle wins.
    let mut w = WorldTrait::new(v(ZERO, -ONE), Default::default());
    let a = w
        .insert_collider(ColliderBuilderTrait::ball(HALF).translation(v(ZERO, ONE)).build(), None);
    let _ = w
        .insert_collider(ColliderBuilderTrait::ball(HALF).translation(v(ZERO, -ONE)).build(), None);
    let (tie, _) = cast_shape(
        ref w,
        at(-int(3), ZERO),
        v(ONE, ZERO),
        Shape::Ball(Ball { radius: HALF }),
        Default::default(),
        filter,
    )
        .unwrap();
    assert_eq!(tie, a);
    // A half-space cast against a half-space collider is skipped (no kernel), not a panic.
    let mut ground = WorldTrait::new(v(ZERO, -ONE), Default::default());
    let _ = ground.insert_collider(ColliderBuilderTrait::halfspace(v(ZERO, ONE)).build(), None);
    let hs = Shape::HalfSpace(HalfSpace { normal: v(ZERO, ONE) });
    assert!(
        cast_shape(ref ground, at(ZERO, ONE), v(ZERO, -ONE), hs, Default::default(), filter)
            .is_none(),
    );
    // A box falling on the ground half-space: no box pre-test on the half-space, a hit.
    let (_, hit) = cast_shape(
        ref ground, at(ZERO, TWO), v(ZERO, -ONE), small_box(), Default::default(), filter,
    )
        .unwrap();
    assert_eq!(hit.time_of_impact, TWO - ratio(1, 8));
    let still = NonlinearRigidMotionTrait::constant_position(at(ZERO, ONE));
    assert!(cast_shape_nonlinear(ref ground, still, hs, ZERO, ONE, true, filter).is_none());
    // The pipeline view and the geometry kernel agree.
    let pipeline = world.query_pipeline_with_filter(filter);
    assert_eq!(
        pipeline
            .cast_shape(ref world, at(-TWO, ZERO), v(ONE, ZERO), small_box(), Default::default()),
        Some(
            (
                h0,
                cast_shapes(
                    c0.position(),
                    v(ZERO, ZERO),
                    c0.shape,
                    at(-TWO, ZERO),
                    v(ONE, ZERO),
                    small_box(),
                    Default::default(),
                )
                    .unwrap()
                    .unwrap()
                    .transform1_by(c0.position()),
            ),
        ),
    );
}

/// A box turning a quarter turn while moving along the first row.
fn turning(x: Fixed, y: Fixed) -> NonlinearRigidMotion {
    NonlinearRigidMotionTrait::new(at(x, y), v(ZERO, ZERO), v(ONE, ZERO), FRAC_PI_2)
}

/// The swept-box winner answers what the exact cast on every candidate answers: a hit on the
/// first row (stopping at penetration or not) and a miss far above.
#[test]
fn test_cast_shape_nonlinear_matches_direct() {
    let mut world = scene();
    let filter = QueryFilterTrait::new();
    let bar = Shape::Cuboid(Cuboid { half_extents: v(ratio(1, 2), ratio(1, 16)) });
    let hit = cast_shape_nonlinear(ref world, turning(-TWO, ZERO), bar, ZERO, int(4), true, filter);
    assert!(hit.is_some());
    assert_eq!(
        hit,
        cast_shape_nonlinear_direct(
            ref world, turning(-TWO, ZERO), bar, ZERO, int(4), true, filter,
        ),
    );
    let far = turning(-TWO, int(10));
    assert!(cast_shape_nonlinear(ref world, far, bar, ZERO, int(4), false, filter).is_none());
    assert!(
        cast_shape_nonlinear_direct(ref world, far, bar, ZERO, int(4), false, filter).is_none(),
    );
    let view = QueryPipelineTrait::new();
    assert_eq!(
        view.cast_shape_nonlinear(ref world, turning(-TWO, ZERO), bar, ZERO, int(4), true), hit,
    );
}

#[test]
fn gas_baseline() {}

#[test]
fn gas_setup_20() {
    let _ = scene();
}

#[test]
fn gas_cast_shape_20() {
    let mut world = scene();
    let _ = cast_shape(
        ref world,
        opaque(at(-TWO, HALF)),
        opaque(v(ONE, ZERO)),
        opaque(small_box()),
        opaque(Default::default()),
        QueryFilterTrait::new(),
    );
}

#[test]
fn gas_cast_shape_direct_20() {
    let mut world = scene();
    let _ = cast_shape_direct(
        ref world,
        opaque(at(-TWO, HALF)),
        opaque(v(ONE, ZERO)),
        opaque(small_box()),
        opaque(Default::default()),
        QueryFilterTrait::new(),
    );
}

#[test]
fn gas_cast_shape_nonlinear_20() {
    let mut world = scene();
    let bar = Shape::Cuboid(Cuboid { half_extents: v(ratio(1, 2), ratio(1, 16)) });
    let _ = cast_shape_nonlinear(
        ref world,
        opaque(turning(-TWO, HALF)),
        opaque(bar),
        opaque(ZERO),
        opaque(int(4)),
        opaque(true),
        QueryFilterTrait::new(),
    );
}

#[test]
fn gas_cast_shape_nonlinear_direct_20() {
    let mut world = scene();
    let bar = Shape::Cuboid(Cuboid { half_extents: v(ratio(1, 2), ratio(1, 16)) });
    let _ = cast_shape_nonlinear_direct(
        ref world,
        opaque(turning(-TWO, HALF)),
        opaque(bar),
        opaque(ZERO),
        opaque(int(4)),
        opaque(true),
        QueryFilterTrait::new(),
    );
}
