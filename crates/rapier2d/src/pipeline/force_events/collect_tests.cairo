//! RG1: the shipped `collect` against SH2a's (`alternatives::collect_group_scan`, every entry
//! scanned for its group) and the gated scan (`alternatives::collect_gated_scan`), on a convex
//! scene and on a composite one (a box and a ball in a polyline V and on a heightfield, each
//! touching two parts: groups of two entries). Same events and same pair list, bit for bit;
//! `gas_collect_*` probes each candidate after the same `WARMUP` steps (`gas_setup_*`).

use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam::Vec2;
use rapier_core::collider::events::CONTACT_FORCE_EVENTS;
use rapier_dynamics2d::collider::{ColliderBuilder, ColliderBuilderTrait};
use rapier_dynamics2d::events::ContactForceEvent;
use rapier_dynamics2d::narrow_phase::ContactPair;
use rapier_dynamics2d::rigid_body_set::RigidBodyTrait;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::world::{World, WorldTrait};
use super::alternatives::{
    collect_abort, collect_alpha4, collect_alpha4_pop, collect_by_value, collect_gated_scan,
    collect_group_scan, collect_peek, collect_single_loop,
};
use super::collect;

const WARMUP: u32 = 8;
const GRAVITY_Y: i64 = -42133629174;
/// Just above rest: the box in the V (corners on the slopes at `y = 0.25`, centre `0.75`) and
/// the ball in the notch (centre `0.5·√2 ≈ 0.707`), `0.005` higher.
const BOX_Y: i64 = 3242697277;
const BALL_Y: i64 = 3058499764;

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2 { translation: v(x, y), rotation: Rot2 { re: ONE, im: ZERO } }
}

fn events_on(builder: ColliderBuilder) -> ColliderBuilder {
    builder.active_events(CONTACT_FORCE_EVENTS).contact_force_event_threshold(ZERO)
}

/// `composite`: a box in a polyline V (slopes `±1/2`) and a ball in a heightfield notch; else
/// six balls and a box on a cuboid ground. Every collider has force events at threshold 0.
fn scene(composite: bool) -> World {
    let mut world = WorldTrait::new(v(ZERO, Fixed { raw: GRAVITY_Y }), Default::default());
    let two: Fixed = FixedTrait::from_int(2);
    if composite {
        let vertices = array![v(-two - two, two), v(ZERO, ZERO), v(two + two, two)];
        world
            .insert_collider(
                events_on(ColliderBuilderTrait::polyline(vertices.span(), None)).build(), None,
            );
        let heights = array![ONE, ZERO, ONE];
        world
            .insert_collider(
                events_on(ColliderBuilderTrait::heightfield(heights.span(), v(two, ONE)))
                    .translation(v(FixedTrait::from_int(10), ZERO))
                    .build(),
                None,
            );
        let _ = world
            .insert(
                RigidBodyTrait::dynamic(at(ZERO, Fixed { raw: BOX_Y })),
                events_on(ColliderBuilderTrait::cuboid(HALF, HALF)).build(),
            );
        let _ = world
            .insert(
                RigidBodyTrait::dynamic(at(FixedTrait::from_int(10), Fixed { raw: BALL_Y })),
                events_on(ColliderBuilderTrait::ball(HALF)).build(),
            );
    } else {
        world
            .insert_collider(
                events_on(ColliderBuilderTrait::cuboid(FixedTrait::from_int(20), HALF))
                    .translation(v(ZERO, -HALF))
                    .build(),
                None,
            );
        let mut i: i32 = 0;
        while i != 6 {
            let x: Fixed = FixedTrait::from_int(i) * two;
            let _ = world
                .insert(
                    RigidBodyTrait::dynamic(at(x, HALF)),
                    events_on(ColliderBuilderTrait::ball(HALF)).build(),
                );
            i += 1;
        }
        let _ = world
            .insert(
                RigidBodyTrait::dynamic(at(-two - two, HALF)),
                events_on(ColliderBuilderTrait::cuboid(HALF, HALF)).build(),
            );
    }
    let mut k = 0;
    while k != WARMUP {
        let _ = world.step();
        k += 1;
    }
    world
}

/// Runs candidate `which` (0 shipped, 1 group scan, 2 gated scan, 3 peek, 4 by value, 5 the
/// alpha.4 floor and 6 its `pop_front` form, convex only; 7 abort, 8 single loop) on `world`'s
/// pairs.
fn run(ref world: World, which: u32) -> Array<ContactForceEvent> {
    let dt = world.integration_parameters.dt;
    if which == 0 {
        collect(dt, ref world.narrow_phase, ref world.colliders)
    } else if which == 1 {
        collect_group_scan(dt, ref world.narrow_phase, ref world.colliders)
    } else if which == 2 {
        collect_gated_scan(dt, ref world.narrow_phase, ref world.colliders)
    } else if which == 3 {
        collect_peek(dt, ref world.narrow_phase, ref world.colliders)
    } else if which == 4 {
        collect_by_value(dt, ref world.narrow_phase, ref world.colliders)
    } else if which == 5 {
        collect_alpha4(dt, ref world.narrow_phase, ref world.colliders)
    } else if which == 6 {
        collect_alpha4_pop(dt, ref world.narrow_phase, ref world.colliders)
    } else if which == 7 {
        collect_abort(dt, ref world.narrow_phase, ref world.colliders)
    } else {
        collect_single_loop(dt, ref world.narrow_phase, ref world.colliders)
    }
}

/// Entries of `pairs` whose next entry has the same collider pair (group members).
fn grouped(pairs: Span<ContactPair>) -> u32 {
    let mut n = 0;
    let mut i = 1;
    while i < pairs.len() {
        let a = pairs.at(i - 1);
        let b = pairs.at(i);
        if a.collider1 == b.collider1 && a.collider2 == b.collider2 {
            n += 1;
        }
        i += 1;
    }
    n
}

/// A copy of the world serialised in `image`.
fn restore(image: Span<felt252>) -> World {
    let mut image = image;
    WorldTrait::from_state(Serde::deserialize(ref image).unwrap())
}

#[test]
fn test_candidates_agree() {
    for composite in array![false, true] {
        let mut image = array![];
        scene(composite).into_state().serialize(ref image);
        let mut shipped = restore(image.span());
        let expected = run(ref shipped, 0);
        assert!(!expected.is_empty(), "events expected");
        let members = grouped(shipped.narrow_phase.pairs.span());
        assert!((members != 0) == composite, "groups only in the composite scene");
        // A second pass reads the statuses written by the first (started flags).
        let again = run(ref shipped, 0);
        for which in array![1_u32, 2, 3, 4, 7, 8] {
            let mut other = restore(image.span());
            assert!(run(ref other, which) == expected, "events of candidate {which}");
            assert!(run(ref other, which) == again, "second pass of candidate {which}");
            assert!(other.narrow_phase == shipped.narrow_phase, "pairs of candidate {which}");
        }
    }
}

fn probe(composite: bool, which: Option<u32>) {
    let mut world = scene(opaque(composite));
    if let Some(which) = which {
        let _ = run(ref world, opaque(which));
    }
    let _ = opaque(world.gravity);
}

#[test]
fn gas_setup_convex() {
    probe(false, None);
}
#[test]
fn gas_collect_convex_shipped() {
    probe(false, Some(0));
}
#[test]
fn gas_collect_convex_group_scan() {
    probe(false, Some(1));
}
#[test]
fn gas_collect_convex_gated_scan() {
    probe(false, Some(2));
}
#[test]
fn gas_setup_composite() {
    probe(true, None);
}
#[test]
fn gas_collect_composite_shipped() {
    probe(true, Some(0));
}
#[test]
fn gas_collect_composite_group_scan() {
    probe(true, Some(1));
}
#[test]
fn gas_collect_composite_gated_scan() {
    probe(true, Some(2));
}
#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}
#[test]
fn gas_collect_convex_peek() {
    probe(false, Some(3));
}
#[test]
fn gas_collect_convex_by_value() {
    probe(false, Some(4));
}
#[test]
fn gas_collect_convex_alpha4() {
    probe(false, Some(5));
}
#[test]
fn gas_collect_composite_peek() {
    probe(true, Some(3));
}
#[test]
fn gas_collect_composite_by_value() {
    probe(true, Some(4));
}
#[test]
fn gas_collect_convex_alpha4_pop() {
    probe(false, Some(6));
}
#[test]
fn gas_collect_convex_abort() {
    probe(false, Some(7));
}
#[test]
fn gas_collect_composite_abort() {
    probe(true, Some(7));
}
#[test]
fn gas_collect_convex_single_loop() {
    probe(false, Some(8));
}
#[test]
fn gas_collect_composite_single_loop() {
    probe(true, Some(8));
}
