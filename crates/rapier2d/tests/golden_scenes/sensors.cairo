//! SE: the `sensor_trigger` scene (a ball falling through a standalone sensor slab) against
//! upstream's trace: the `Started` / `Stopped` events (flag `SENSOR`) at the exact steps, the
//! intersection pair's `intersecting` state after every step, and the ball's free fall within
//! the scene tolerances (the sensor applies no force). The pair's lifetime follows each broad
//! phase and may differ by a step at each end (D7).
use rapier_core::collider::events::COLLISION_EVENTS;
use rapier_dynamics2d::collider::ColliderBuilderTrait;
use rapier_dynamics2d::events::CollisionEventTrait;
use rapier_golden::generated::sensor_trigger;
use super::*;

#[test]
fn test_sensor_trigger_matches_upstream() {
    let params = rapier_core::integration_parameters::IntegrationParameters {
        dt: f(sensor_trigger::DT), ..Default::default(),
    };
    let mut world = WorldTrait::new(vr(sensor_trigger::GRAVITY), params);
    let half = vr(sensor_trigger::SLAB_HALF_EXTENTS);
    let slab = world
        .insert_collider(
            ColliderBuilderTrait::cuboid(half.x, half.y)
                .position(
                    Pose2 {
                        translation: vr(sensor_trigger::SLAB_CENTER),
                        rotation: Rot2 { re: f(4294967296), im: f(0) },
                    },
                )
                .sensor(true)
                .active_events(COLLISION_EVENTS)
                .build(),
            None,
        );
    let mut body = RigidBodyTrait::dynamic(
        Pose2 {
            translation: vr(sensor_trigger::BALL_START),
            rotation: Rot2 { re: f(4294967296), im: f(0) },
        },
    );
    body.activation = rapier_core::rigid_body::RigidBodyActivationTrait::cannot_sleep();
    let (ball_body, ball) = world
        .insert(body, ColliderBuilderTrait::ball(f(sensor_trigger::BALL_RADIUS)).build());
    assert_eq!((slab.index, ball.index), (0, 1));
    let mut expected = sensor_trigger::events();
    let mut max_y: u64 = 0;
    let mut max_vy: u64 = 0;
    let mut pair_lifetime: u32 = 0;
    for sample in sensor_trigger::samples() {
        let (step, y, vy, has_pair, intersecting) = *sample;
        for event in world.step() {
            let (at, started, c1, c2, sensor, removed) = *expected
                .pop_front()
                .expect('unexpected event');
            assert_eq!(at, step);
            assert_eq!(event.started(), started);
            assert_eq!(event.sensor(), sensor);
            assert_eq!(event.removed(), removed);
            let (i1, i2) = (event.collider1().index, event.collider2().index);
            assert!((i1, i2) == (c1, c2) || (i2, i1) == (c1, c2));
        }
        if let Some(next) = expected.get(0) {
            let (at, _, _, _, _, _) = *next.unbox();
            assert!(at > step, "missed event at step {}", step);
        }
        // `intersecting` exactly; the pair's existence follows each broad phase (D7: upstream's
        // BVH keeps a non-intersecting pair alive over a different margin), counted apart.
        let pair = world.intersection_pair(slab, ball);
        assert!((pair == Some(true)) == intersecting, "intersecting at step {}", step);
        if pair.is_some() != has_pair {
            pair_lifetime += 1;
        }
        let rb = world.body(ball_body).unwrap();
        let dy = abs_diff(rb.position().translation.y.raw, y);
        let dvy = abs_diff(rb.linvel().y.raw, vy);
        assert!(dy <= TOL_PER_STEP * step.into(), "y at step {}: {}", step, dy);
        assert!(dvy <= 2 * TOL_PER_STEP * step.into(), "vy at step {}: {}", step, dvy);
        if dy > max_y {
            max_y = dy;
        }
        if dvy > max_vy {
            max_vy = dvy;
        }
    }
    assert!(expected.is_empty());
    println!(
        "sensor_trigger: max |dy| {} ulp, max |dvy| {} ulp, {} steps with a pair on one side only",
        max_y,
        max_vy,
        pair_lifetime,
    );
    assert!(pair_lifetime <= 2);
}
