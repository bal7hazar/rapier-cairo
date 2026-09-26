//! Family `ccd_scenes` (work package CC2): Rapier's CCD solver in the step. Unlike the other
//! scene families (D11: CCD disabled), each scene sets `max_ccd_substeps` and `ccd_enabled`
//! explicitly; contact recycling and clustering stay off, as in `scenes`.
//!
//! * `plank_*`: a ball of radius 1/10 at `(-13/10, 0)` thrown at 30 m/s along `+x` (gravity on)
//!   at a thin fixed plank (half extents `(1/20, 1)` at the origin). Half a metre per step: the
//!   ball skips the plank between two steps unless CCD stops it.
//!   * `plank_bullet_sub{1,4}`: `ccd_enabled`, `max_ccd_substeps = 1`, 1 or 4 solver iterations;
//!   * `plank_auto_sub{1,4}`: no `ccd_enabled`, `max_ccd_substeps = 1` (upstream's automatic
//!     CCD of fast dynamic bodies against fixed colliders);
//!   * `plank_off_sub{1,4}`: no `ccd_enabled`, `max_ccd_substeps = 0` (no CCD: tunnels);
//!   * `plank_bullet_split4`: `ccd_enabled`, `max_ccd_substeps = 4` (the substep splitter).
//! * `box_bullet`: the same ball (`ccd_enabled`, no gravity) at a dynamic box (half extents
//!   `(1/20, 1/2)`, density 1) at rest at the origin, 4 solver iterations.
//! * `slow_bullet`: a ball of radius 1/4 (`ccd_enabled`) dropped from `y = 1` on a fixed ground
//!   box (half extents `(2, 1/10)` at the origin): never fast, CCD changes nothing.
//!
//! Recorded after each step: the ball's position, velocity and `is_ccd_active`, and the second
//! body's position and angle (`box_bullet`).

use crate::q::{jf, jq, jqvec, QVec, Q};
use rapier2d_f64::prelude::*;
use serde_json::{json, Value};

const NUM_STEPS: usize = 30;

/// `(id, kind, iterations, max_ccd_substeps, ccd_enabled, gravity on)`.
type Config = (&'static str, &'static str, usize, usize, bool, bool);

const CONFIGS: [Config; 9] = [
    ("plank_bullet_sub1", "plank", 1, 1, true, true),
    ("plank_bullet_sub4", "plank", 4, 1, true, true),
    ("plank_auto_sub1", "plank", 1, 1, false, true),
    ("plank_auto_sub4", "plank", 4, 1, false, true),
    ("plank_off_sub1", "plank", 1, 0, false, true),
    ("plank_off_sub4", "plank", 4, 0, false, true),
    ("plank_bullet_split4", "plank", 4, 4, true, true),
    ("box_bullet", "box", 4, 1, true, false),
    ("slow_bullet", "slow", 4, 1, true, true),
];

/// The scene inputs, as Q32.32 values fed to both engines.
pub struct Inputs {
    pub dt: Q,
    pub gravity: QVec,
    pub ball_radius: Q,
    pub ball_start: QVec,
    pub ball_linvel: QVec,
    pub plank_half: QVec,
    pub box_half: QVec,
    pub slow_radius: Q,
    pub slow_start: QVec,
    pub ground_half: QVec,
}

pub fn inputs() -> Inputs {
    Inputs {
        dt: Q::snap(1.0 / 60.0),
        gravity: QVec::snap(0.0, -9.81),
        ball_radius: Q::snap(0.1),
        ball_start: QVec::snap(-1.3, 0.0),
        ball_linvel: QVec::snap(30.0, 0.0),
        plank_half: QVec::snap(0.05, 1.0),
        box_half: QVec::snap(0.05, 0.5),
        slow_radius: Q::snap(0.25),
        slow_start: QVec::snap(0.0, 1.0),
        ground_half: QVec::snap(2.0, 0.1),
    }
}

fn run(config: Config, inp: &Inputs) -> Value {
    let (id, kind, iterations, substeps, ccd, gravity_on) = config;
    let params = IntegrationParameters {
        dt: inp.dt.f(),
        num_solver_iterations: iterations,
        contact_recycling: false,
        contact_clustering: false,
        max_ccd_substeps: substeps,
        ..Default::default()
    };
    let gravity = if gravity_on { inp.gravity } else { QVec::ZERO };
    let mut pipeline = PhysicsPipeline::new();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();

    // Collider 0 is the target (standalone, or the box's), collider 1 the ball's.
    let other = match kind {
        "plank" => {
            colliders.insert(ColliderBuilder::cuboid(
                inp.plank_half.x.f(),
                inp.plank_half.y.f(),
            ));
            None
        }
        "box" => {
            let body = bodies.insert(RigidBodyBuilder::dynamic().ccd_enabled(false));
            colliders.insert_with_parent(
                ColliderBuilder::cuboid(inp.box_half.x.f(), inp.box_half.y.f()).density(1.0),
                body,
                &mut bodies,
            );
            Some(body)
        }
        _ => {
            colliders.insert(ColliderBuilder::cuboid(
                inp.ground_half.x.f(),
                inp.ground_half.y.f(),
            ));
            None
        }
    };
    let (radius, start, linvel) = if kind == "slow" {
        (inp.slow_radius, inp.slow_start, QVec::ZERO)
    } else {
        (inp.ball_radius, inp.ball_start, inp.ball_linvel)
    };
    let ball = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(start.v())
            .linvel(linvel.v())
            .ccd_enabled(ccd),
    );
    colliders.insert_with_parent(ColliderBuilder::ball(radius.f()), ball, &mut bodies);

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
            &(),
        );
        let b = &bodies[ball];
        let mut sample = json!({
            "step": step,
            "x": jf(b.translation().x),
            "y": jf(b.translation().y),
            "vx": jf(b.linvel().x),
            "vy": jf(b.linvel().y),
            "ccd_active": b.is_ccd_active(),
        });
        if let Some(h) = other {
            let o = &bodies[h];
            sample["other_x"] = jf(o.translation().x);
            sample["other_y"] = jf(o.translation().y);
            sample["other_angle"] = jf(o.rotation().angle());
        }
        samples.push(sample);
    }
    json!({
        "id": id,
        "kind": kind,
        "num_solver_iterations": iterations,
        "max_ccd_substeps": substeps,
        "ccd_enabled": ccd,
        "gravity": jqvec(gravity),
        "samples": samples,
    })
}

pub fn generate() -> Value {
    let inp = inputs();
    let scenes: Vec<Value> = CONFIGS.iter().map(|c| run(*c, &inp)).collect();
    json!({
        "family": "ccd_scenes",
        "note": "CCD in the step (CC2): fast ball vs thin fixed plank, bullet vs dynamic box, slow drop; see tools/golden/src/ccd_scenes.rs",
        "dt": jq(inp.dt),
        "ball_radius": jq(inp.ball_radius),
        "ball_start": jqvec(inp.ball_start),
        "ball_linvel": jqvec(inp.ball_linvel),
        "plank_half_extents": jqvec(inp.plank_half),
        "box_half_extents": jqvec(inp.box_half),
        "slow_radius": jq(inp.slow_radius),
        "slow_start": jqvec(inp.slow_start),
        "ground_half_extents": jqvec(inp.ground_half),
        "num_steps": NUM_STEPS,
        "scenes": scenes,
    })
}
