//! Work package SF1: the minimal scene of the tilted-landing divergence. `ell_topple` (SH2b's
//! L-shaped compound on a half-space) left the port's trajectory from step 7; the same body built
//! from two plain cuboid colliders, `ell_twin`, diverged the same way, so the family records that
//! twin: one dynamic body with two colliders (a tall bar and an arm, the centre of mass off the
//! bar's base), dropped flat on the upward half-space. It lands at step 6 on the bar's two
//! corners, one of which closes to a near-zero gap (the soft / rigid switch of the substep
//! solver), then pivots on the other corner. 90 steps: the arm hits the ground at step 94, where
//! upstream solves the new (ground, arm) pair before the (ground, bar) one and the port in pair
//! order (the constraint-order divergence of SO / DO, not this family's subject).
//!
//! rapier2d-f64, no CCD, no contact recycling or clustering, as the other scene families. The
//! ground is a standalone collider (index 0, `COLLISION_EVENTS`), the parts are colliders 1 and 2.

use crate::q::{jf, jq, jqpose, jqvec, QPose, QRot, QVec, Q};
use crate::sh1::Sh1Shape;
use crate::sh2a::{b, qv, Collector};
use crate::shapes::ShapeSpec;
use rapier2d_f64::prelude::*;
use serde_json::{json, Value};

/// `(id, parts, start, steps)`: the parts are `(pose in the body frame, shape)`.
fn scene_configs() -> Vec<(&'static str, Vec<(QPose, Sh1Shape)>, (f64, f64), usize)> {
    vec![(
        "ell_twin",
        vec![
            (QPose::new(qv(0.0, 0.0), QRot::IDENTITY), b(ShapeSpec::cuboid(0.15, 0.8))),
            (QPose::new(qv(0.55, 0.65), QRot::IDENTITY), b(ShapeSpec::cuboid(0.4, 0.15))),
        ],
        (0.0, 0.85),
        90,
    )]
}

fn scene(config: (&'static str, Vec<(QPose, Sh1Shape)>, (f64, f64), usize)) -> Value {
    let (id, parts, start, num_steps) = config;
    let dt = Q::snap(1.0 / 60.0);
    let gravity = QVec::snap(0.0, -9.81);
    let start = qv(start.0, start.1);
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
    let ground = b(ShapeSpec::halfspace_up());
    let g = colliders.insert(ColliderBuilder::new(ground.shared()).active_events(ActiveEvents::COLLISION_EVENTS));
    let body = bodies.insert(RigidBodyBuilder::dynamic().translation(start.v()));
    let handles: Vec<ColliderHandle> = parts
        .iter()
        .map(|(p, s)| colliders.insert_with_parent(ColliderBuilder::new(s.shared()).position(p.p()), body, &mut bodies))
        .collect();
    let collector = Collector::default();
    let mut samples = Vec::new();
    let mut events = Vec::new();
    for step in 1..=num_steps {
        pipeline.step(
            gravity.v(), &params, &mut islands, &mut broad_phase, &mut narrow_phase, &mut bodies,
            &mut colliders, &mut impulse_joints, &mut multibody_joints, &mut ccd_solver, &(), &collector,
        );
        let idx = |h: ColliderHandle| {
            if h == g {
                0
            } else {
                1 + handles.iter().position(|c| *c == h).unwrap()
            }
        };
        for e in collector.0.lock().unwrap().drain(..) {
            let (h1, h2, started) = match e {
                CollisionEvent::Started(a, b, _) => (a, b, true),
                CollisionEvent::Stopped(a, b, _) => (a, b, false),
            };
            events.push(json!({ "step": step, "started": started, "collider1": idx(h1), "collider2": idx(h2) }));
        }
        let rb = &bodies[body];
        let manifolds: usize = handles
            .iter()
            .map(|c| {
                narrow_phase
                    .contact_pair(g, *c)
                    .map(|p| p.manifolds.iter().filter(|m| !m.data.solver_contacts.is_empty()).count())
                    .unwrap_or(0)
            })
            .sum();
        samples.push(json!({
            "step": step,
            "x": jf(rb.translation().x), "y": jf(rb.translation().y),
            "re": jf(rb.rotation().re), "im": jf(rb.rotation().im),
            "vx": jf(rb.linvel().x), "vy": jf(rb.linvel().y), "w": jf(rb.angvel()),
            "manifolds": manifolds,
        }));
    }
    json!({
        "id": id,
        "ground": ground.json(),
        "parts": parts.iter().map(|(p, s)| json!({ "pose": jqpose(*p), "shape": s.json() })).collect::<Vec<_>>(),
        "start": jqvec(start),
        "samples": samples,
        "events": events,
    })
}

pub fn tilted_landing() -> Value {
    json!({
        "family": "tilted_landing",
        "note": "SF1 minimal scene: an L made of two plain cuboid colliders on one body, dropped flat on a half-space; it lands on the bar's corners (one at a near-zero gap) and pivots; see tools/golden/src/sf1.rs",
        "dt": jq(Q::snap(1.0 / 60.0)),
        "gravity": jqvec(QVec::snap(0.0, -9.81)),
        "scenes": scene_configs().into_iter().map(scene).collect::<Vec<_>>(),
    })
}
