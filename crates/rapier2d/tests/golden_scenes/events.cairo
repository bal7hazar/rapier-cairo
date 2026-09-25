//! EV: force thresholds, persistent one-way direction, and upstream trajectories.
use fixed::{Fixed, HALF, ONE, ZERO};
use rapier_core::collider::events::{COLLISION_EVENTS, CONTACT_FORCE_EVENTS};
use rapier_dynamics2d::collider::{ColliderBuilderTrait, ColliderTrait};
use rapier_dynamics2d::events::ContactForceEvent;
use rapier_dynamics2d::rigid_body_set::RigidBodyTrait;
use rapier_golden::types::ForceEventRaw;
use rapier_testing::opaque;
use super::*;

fn world(case: SceneCase) -> World {
    let mut w = build_world(case);
    let ground = rapier_core::Handle { index: 0, generation: 0 };
    let mut co = w.collider(ground).unwrap();
    if case.id == 'one_way_jump' {
        let config = ColliderBuilderTrait::ball(ONE)
            .one_way(Vec2 { x: ZERO, y: ONE }, f(429496730))
            .build();
        co.one_way = config.one_way;
        let h = body_handle(1);
        let mut body = w.body(h).unwrap();
        body.set_linvel(Vec2 { x: ZERO, y: f(34359738368) });
        w.set_body(h, body);
    } else {
        let h = body_handle(1);
        let mut body = w.body(h).unwrap();
        body.mprops.flags = rapier_dynamics2d::rigid_body::ROTATION_LOCKED;
        w.set_body(h, body);
        co.set_active_events(CONTACT_FORCE_EVENTS | COLLISION_EVENTS);
        co.set_contact_force_event_threshold(f(85899345920));
    }
    w.set_collider(ground, co);
    w
}

fn check_force(actual: ContactForceEvent, expected: ForceEventRaw) {
    assert_eq!(actual.collider1.index, expected.collider1);
    assert_eq!(actual.collider2.index, expected.collider2);
    assert_eq!(actual.started, expected.started);
    let tol = 8192 * expected.step.into();
    for (a, b) in array![
        (actual.total_force.x.raw, expected.total_force.x),
        (actual.total_force.y.raw, expected.total_force.y),
        (actual.total_force_magnitude.raw, expected.total_force_magnitude),
        (actual.max_force_direction.x.raw, expected.max_force_direction.x),
        (actual.max_force_direction.y.raw, expected.max_force_direction.y),
        (actual.max_force_magnitude.raw, expected.max_force_magnitude),
    ] {
        assert!(abs_diff(a, b) <= tol, "force mismatch: {a} {b}");
    }
}

fn replay(case: SceneCase, expected: Span<ForceEventRaw>) {
    let mut w = world(case);
    let mut expected = expected;
    let mut step = 0;
    let mut stats = Default::default();
    for sample in case.samples.span() {
        while step != *sample.step {
            step += 1;
            let (_, events) = w.step_with_force_events();
            for event in events {
                let next = *expected.pop_front().expect('unexpected force event');
                assert_eq!(step, next.step);
                check_force(event, next);
            }
            if let Some(next) = expected.get(0) {
                assert!(*next.unbox().step > step);
            }
        }
        stats = compare(ref w, case, *sample, stats, true);
    }
    assert!(expected.is_empty());
    assert_eq!(stats.violations, 0);
}

#[test]
fn test_one_way_jump() {
    replay(scenes::one_way_jump::ONE_WAY_JUMP, array![].span());
}
#[test]
fn test_force_event_drop() {
    replay(scenes::force_event_drop::FORCE_EVENT_DROP, scenes::force_event_drop::EVENTS.span());
}


fn resting(a: Fixed, b: Fixed, flags_a: bool, flags_b: bool) -> World {
    let mut w = WorldTrait::new(Vec2 { x: ZERO, y: -ONE }, Default::default());
    let mut ca = ColliderBuilderTrait::halfspace(Vec2 { x: ZERO, y: ONE })
        .contact_force_event_threshold(a);
    let mut cb = ColliderBuilderTrait::cuboid(HALF, HALF).contact_force_event_threshold(b);
    if flags_a {
        ca = ca.active_events(CONTACT_FORCE_EVENTS | COLLISION_EVENTS);
    }
    if flags_b {
        cb = cb.active_events(CONTACT_FORCE_EVENTS);
    }
    w.insert_collider(ca.build(), None);
    w
        .insert(
            RigidBodyTrait::dynamic(
                Pose2 { translation: Vec2 { x: ZERO, y: HALF }, ..Default::default() },
            ),
            cb.build(),
        );
    w
}

#[test]
fn test_thresholds_and_crossing_reset() {
    for (a, b, on_a, on_b, emitted) in array![
        (ZERO, ZERO, false, false, false), (ONE, ONE, false, false, false),
        (f(429496729600), ZERO, true, false, false), (f(429496729600), ZERO, true, true, true),
        (ZERO, f(429496729600), true, true, true), (ZERO, f(429496729600), false, true, false),
    ] {
        let mut w = resting(a, b, on_a, on_b);
        let (_, events) = w.step_with_force_events();
        assert_eq!(!events.is_empty(), emitted);
        if emitted {
            assert!(*events.at(0).started);
            let (_, second) = w.step_with_force_events();
            assert!(!*second.at(0).started);
            let mut co = w.collider(body_handle(0)).unwrap();
            co.set_contact_force_event_threshold(f(429496729600));
            w.set_collider(body_handle(0), co);
            let mut co = w.collider(body_handle(1)).unwrap();
            co.set_contact_force_event_threshold(f(429496729600));
            w.set_collider(body_handle(1), co);
            let (_, empty) = w.step_with_force_events();
            assert!(empty.is_empty());
            let mut co = w.collider(body_handle(0)).unwrap();
            co.set_contact_force_event_threshold(ZERO);
            w.set_collider(body_handle(0), co);
            let (_, crossed) = w.step_with_force_events();
            assert!(*crossed.at(0).started);
        }
    }
}

fn probe(case: SceneCase, advance: bool) {
    let mut w = world(opaque(case));
    // First contact step for force drop, passing contact for one-way.
    let n = if case.id == 'one_way_jump' {
        7
    } else {
        33
    };
    let mut i = 0;
    while i != n {
        w.step();
        i += 1;
    }
    if advance {
        let _ = w.step_with_force_events();
    }
}
#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}
#[test]
fn gas_setup_one_way() {
    probe(scenes::one_way_jump::ONE_WAY_JUMP, false);
}
#[test]
fn gas_step_one_way() {
    probe(scenes::one_way_jump::ONE_WAY_JUMP, true);
}
#[test]
fn gas_setup_force_event() {
    probe(scenes::force_event_drop::FORCE_EVENT_DROP, false);
}
#[test]
fn gas_step_force_event() {
    probe(scenes::force_event_drop::FORCE_EVENT_DROP, true);
}

#[test]
fn test_step_preserves_force_crossing_and_sleep_omits_stale_impulses() {
    let mut w = resting(ZERO, ZERO, true, false);
    let _ = w.step();
    let (_, events) = w.step_with_force_events();
    assert_eq!(events.len(), 1);
    assert!(!*events.at(0).started);
    let mut n = 0;
    while n != 180 {
        let _ = w.step();
        n += 1;
    }
    assert!(w.body(body_handle(0)).unwrap().is_sleeping());
    let (_, sleeping) = w.step_with_force_events();
    assert!(sleeping.is_empty());
}
