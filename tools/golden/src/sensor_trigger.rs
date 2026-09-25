//! Scene `sensor_trigger` (work package SE): a ball of radius 1/2 falls from `y = 4` through a
//! standalone sensor slab (half extents `(4, 1/4)` at `(0, 2)`, `COLLISION_EVENTS`), with the
//! comparability settings of the `scenes` family (D11). Recorded: every collision event with its
//! step, and the ball's height and vertical velocity after each step. Its own vector file, so that
//! `scenes.json` stays byte-identical.

use crate::q::{jf, jq, jqvec, QVec, Q};
use rapier2d_f64::prelude::*;
use serde_json::{json, Value};
use std::sync::Mutex;

const NUM_STEPS: usize = 60;

#[derive(Default)]
struct Collector(Mutex<Vec<CollisionEvent>>);

impl EventHandler for Collector {
    fn handle_collision_event(
        &self,
        _: &RigidBodySet,
        _: &ColliderSet,
        event: CollisionEvent,
        _: Option<&ContactPair>,
    ) {
        self.0.lock().unwrap().push(event);
    }
    fn handle_contact_force_event(
        &self,
        _: Real,
        _: &RigidBodySet,
        _: &ColliderSet,
        _: &ContactPair,
        _: Real,
    ) {
    }
}

pub fn generate() -> Value {
    let gravity = QVec::snap(0.0, -9.81);
    let dt = Q::snap(1.0 / 60.0);
    let slab_half = QVec::snap(4.0, 0.25);
    let slab_center = QVec::snap(0.0, 2.0);
    let radius = Q::snap(0.5);
    let start = QVec::snap(0.0, 4.0);
    let params = IntegrationParameters {
        dt: dt.f(),
        contact_recycling: false,
        contact_clustering: false,
        max_ccd_substeps: 0,
        ..Default::default()
    };
    let mut pipeline = PhysicsPipeline::new();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();

    let slab = colliders.insert(
        ColliderBuilder::cuboid(slab_half.x.f(), slab_half.y.f())
            .translation(slab_center.v())
            .sensor(true)
            .active_events(ActiveEvents::COLLISION_EVENTS),
    );
    let ball_body = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(start.v())
            .can_sleep(false)
            .ccd_enabled(false),
    );
    let ball = colliders.insert_with_parent(
        ColliderBuilder::ball(radius.f()).density(1.0),
        ball_body,
        &mut bodies,
    );
    assert_eq!((slab.into_raw_parts().0, ball.into_raw_parts().0), (0, 1));

    let collector = Collector::default();
    let mut events = Vec::new();
    let mut samples = Vec::new();
    for step in 1..=NUM_STEPS {
        pipeline.step(
            gravity.v(),
            &params,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd_solver,
            &(),
            &collector,
        );
        for event in collector.0.lock().unwrap().drain(..) {
            events.push(json!({
                "step": step,
                "started": event.started(),
                "collider1": event.collider1().into_raw_parts().0,
                "collider2": event.collider2().into_raw_parts().0,
                "sensor": event.sensor(),
                "removed": event.removed(),
            }));
        }
        let body = &bodies[ball_body];
        samples.push(json!({
            "step": step,
            "y": jf(body.translation().y),
            "vy": jf(body.linvel().y),
            "intersecting": narrow_phase.intersection_pair(slab, ball),
        }));
    }
    json!({
        "family": "sensor_trigger",
        "note": "ball (r = 1/2) falling from (0, 4) through a standalone sensor slab (half extents (4, 1/4)) at (0, 2) with COLLISION_EVENTS",
        "gravity": jqvec(gravity),
        "dt": jq(dt),
        "slab_half_extents": jqvec(slab_half),
        "slab_center": jqvec(slab_center),
        "ball_radius": jq(radius),
        "ball_start": jqvec(start),
        "num_steps": NUM_STEPS,
        "events": events,
        "samples": samples,
    })
}
