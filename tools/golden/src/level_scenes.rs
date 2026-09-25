//! Family `level_scenes` (work package G0): level-shaped scenes of the game programme
//! (`docs/PLAN.md`, "Programme target"). A half-space ground; a structure of 10 (or 20) cuboid and
//! convex-polygon blocks resting in exact contact and **asleep at t = 0**
//! (`RigidBodyBuilder::sleeping(true)`); three cores (two balls, one small cuboid) resting on
//! it, asleep too; a pebble (ball r = 1/4, density 4) launched with `linvel` from 7.25 m in
//! front of the structure at about 18.4 m/s. Sleeping on for every body, upstream thresholds.
//!
//! Each level runs under four settings: 60 Hz with 4, 2 and 1 solver iterations (substeps), and
//! 30 Hz with 4, for the same 5 simulated seconds (300 ticks at 60 Hz, 150 at 30 Hz). Recorded
//! per run: the number of awake dynamic bodies after every tick, the first tick with a pebble
//! contact, the first tick after which every dynamic body sleeps, the tick at which the
//! programme's calm rule would end the shot, and every body's state at sampled ticks (every
//! 10th tick of the reference run `hz60_sub4`, every 30th of the others, every 15th at 30 Hz;
//! plus every 5th before the impact and every tick from the one before the first pebble contact
//! to four after it). Its own vector file, so that `scenes.json` stays byte-identical.

use crate::q::{jf, jq, jqpose, jqvec, jrot, jvec, QPose, QVec, Q};
use crate::shapes::ShapeSpec;
use rapier2d_f64::prelude::*;
use serde_json::{json, Value};

/// Simulated time of one run, in 60 Hz ticks.
const TICKS_60HZ: usize = 300;
/// The programme's calm rule: every awake dynamic body below these speeds for `CALM_TICKS`
/// consecutive ticks (`pm/research/R2-game-design-and-client.md`, end of shot).
const CALM_LINEAR: f64 = 0.05;
const CALM_ANGULAR: f64 = 0.05;
const CALM_TICKS: usize = 20;

#[derive(Copy, Clone)]
pub struct Material {
    pub name: &'static str,
    pub density: f64,
    pub friction: f64,
    pub restitution: f64,
}

/// Programme materials (R2, `(est.)` values), the core and pebble materials, the ground.
const TIMBER: Material = Material {
    name: "timber",
    density: 1.0,
    friction: 0.6,
    restitution: 0.1,
};
const SLATE: Material = Material {
    name: "slate",
    density: 2.5,
    friction: 0.8,
    restitution: 0.05,
};
const FROST: Material = Material {
    name: "frost",
    density: 0.9,
    friction: 0.05,
    restitution: 0.2,
};
const CORE: Material = Material {
    name: "core",
    density: 1.0,
    friction: 0.5,
    restitution: 0.1,
};
const PEBBLE: Material = Material {
    name: "pebble",
    density: 4.0,
    friction: 0.5,
    restitution: 0.2,
};
const GROUND: Material = Material {
    name: "ground",
    density: 1.0,
    friction: 0.6,
    restitution: 0.0,
};

struct Part {
    name: String,
    role: &'static str,
    shape: ShapeSpec,
    pose: QPose,
    material: Material,
}

fn part(
    name: &str,
    role: &'static str,
    shape: ShapeSpec,
    x: f64,
    y: f64,
    material: Material,
) -> Part {
    Part {
        name: name.into(),
        role,
        shape,
        pose: QPose::translation(x, y),
        material,
    }
}

/// The 10 blocks of the base structure, shifted by `dx`: three slate pillars, two timber planks
/// on them, two timber pillars on the planks, a frost lintel, a timber roof triangle and a slate
/// trapezoid on the lintel. Every block rests in exact contact on the one below.
fn structure(dx: f64, suffix: &str) -> Vec<Part> {
    let name = |n: &str| format!("{n}{suffix}");
    let pillar = ShapeSpec::cuboid(0.2, 0.6);
    let plank = ShapeSpec::cuboid(0.9, 0.15);
    let post = ShapeSpec::cuboid(0.2, 0.5);
    let roof = ShapeSpec::polygon(&[(-0.6, -0.3), (0.6, -0.3), (0.0, 0.3)]);
    let trapezoid = ShapeSpec::polygon(&[(-0.4, -0.3), (0.4, -0.3), (0.25, 0.3), (-0.25, 0.3)]);
    vec![
        part(&name("pillar_l"), "block", pillar, dx + 6.4, 0.6, SLATE),
        part(&name("pillar_c"), "block", pillar, dx + 8.0, 0.6, SLATE),
        part(&name("pillar_r"), "block", pillar, dx + 9.6, 0.6, SLATE),
        part(&name("plank_l"), "block", plank, dx + 7.1, 1.35, TIMBER),
        part(&name("plank_r"), "block", plank, dx + 8.9, 1.35, TIMBER),
        part(&name("post_l"), "block", post, dx + 6.8, 2.0, TIMBER),
        part(&name("post_r"), "block", post, dx + 9.2, 2.0, TIMBER),
        part(
            &name("lintel"),
            "block",
            ShapeSpec::cuboid(1.6, 0.15),
            dx + 8.0,
            2.65,
            FROST,
        ),
        part(&name("roof"), "block", roof, dx + 8.0, 3.1, TIMBER),
        part(&name("trapezoid"), "block", trapezoid, dx + 6.9, 3.1, SLATE),
    ]
}

struct Level {
    id: &'static str,
    note: &'static str,
    parts: Vec<Part>,
    pebble_linvel: QVec,
}

fn level(id: &'static str, note: &'static str, copies: usize) -> Level {
    let mut parts = vec![part(
        "ground",
        "ground",
        ShapeSpec::halfspace_up(),
        0.0,
        0.0,
        GROUND,
    )];
    parts.extend(structure(0.0, ""));
    if copies == 2 {
        parts.extend(structure(4.0, "_2"));
    }
    parts.push(part(
        "core_ball",
        "core",
        ShapeSpec::ball(0.3),
        7.4,
        1.8,
        CORE,
    ));
    parts.push(part(
        "core_box",
        "core",
        ShapeSpec::cuboid(0.25, 0.25),
        8.6,
        1.75,
        CORE,
    ));
    parts.push(part(
        "core_top",
        "core",
        ShapeSpec::ball(0.25),
        9.1,
        3.05,
        CORE,
    ));
    parts.push(part(
        "pebble",
        "pebble",
        ShapeSpec::ball(0.25),
        -1.0,
        1.5,
        PEBBLE,
    ));
    Level {
        id,
        note,
        parts,
        pebble_linvel: QVec::snap(18.0, 4.0),
    }
}

fn levels() -> Vec<Level> {
    vec![
        level("level10", "10 blocks (5 slate, 4 timber, 1 frost; 2 convex polygons) + 3 cores asleep, pebble at 18.4 m/s", 1),
        level("level20", "level10 plus a second copy of its structure 4 m further (20 blocks), same cores and pebble", 2),
    ]
}

/// `(id, dt, solver iterations, ticks, sample period)` of the four settings.
fn configs() -> Vec<Config> {
    let dt60 = Q::snap(1.0 / 60.0);
    // 30 Hz: exactly twice the 60 Hz dt, so that its substep dt stays exact too.
    let dt30 = Q(2 * dt60.0);
    vec![
        ("hz60_sub4", dt60, 4, TICKS_60HZ, 10),
        ("hz60_sub2", dt60, 2, TICKS_60HZ, 30),
        ("hz60_sub1", dt60, 1, TICKS_60HZ, 30),
        ("hz30_sub4", dt30, 4, TICKS_60HZ / 2, 15),
    ]
}

fn state_json(body: &RigidBody) -> Value {
    json!({
        "translation": jvec(body.translation()),
        "rotation": jrot(*body.rotation()),
        "linvel": jvec(body.linvel()),
        "angvel": jf(body.angvel()),
        "sleeping": body.is_sleeping(),
    })
}

/// Game-style bounds (R2 "despawn"): a dynamic body whose centre leaves `x_min..=x_max` after a
/// tick is removed with its collider.
fn bounds(level: &Level) -> (Q, Q) {
    (
        Q::snap(-3.0),
        Q::snap(if level.id == "level10" { 14.0 } else { 18.0 }),
    )
}

type Config = (&'static str, Q, usize, usize, usize);

#[allow(clippy::too_many_arguments)]
fn step(
    pipeline: &mut PhysicsPipeline,
    gravity: QVec,
    params: &IntegrationParameters,
    islands: &mut IslandManager,
    broad_phase: &mut DefaultBroadPhase,
    narrow_phase: &mut NarrowPhase,
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    multibody_joints: &mut MultibodyJointSet,
    ccd_solver: &mut CCDSolver,
) {
    pipeline.step(
        gravity.v(),
        params,
        islands,
        broad_phase,
        narrow_phase,
        bodies,
        colliders,
        impulse_joints,
        multibody_joints,
        ccd_solver,
        &(),
        &(),
    );
}

fn run(level: &Level, config: Config, gravity: QVec) -> Value {
    run_with(level, config, gravity, None)
}

/// [`run`], with every dynamic body woken up (strongly) right before tick `prewake` when given
/// (diagnostic counterfactual, `level_checks`).
fn run_with(level: &Level, config: Config, gravity: QVec, prewake: Option<usize>) -> Value {
    let (id, dt, iterations, ticks, period) = config;
    let params = IntegrationParameters {
        dt: dt.f(),
        num_solver_iterations: iterations,
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

    let mut handles = Vec::new();
    let insert = |p: &Part, bodies: &mut RigidBodySet, colliders: &mut ColliderSet| {
        let builder = match p.role {
            "ground" => RigidBodyBuilder::fixed(),
            "pebble" => RigidBodyBuilder::dynamic().linvel(level.pebble_linvel.v()),
            _ => RigidBodyBuilder::dynamic(),
        };
        let handle = bodies.insert(builder.pose(p.pose.p()).ccd_enabled(false));
        let collider = colliders.insert_with_parent(
            ColliderBuilder::new(p.shape.shared())
                .density(Q::snap(p.material.density).f())
                .friction(Q::snap(p.material.friction).f())
                .restitution(Q::snap(p.material.restitution).f()),
            handle,
            bodies,
        );
        (handle, collider)
    };
    // Level load: the structure and the cores, one setup step **without gravity** (it creates
    // their contacts, whose start wakes every body; with no load and exact contact nothing moves,
    // in either engine), then `sleep()` on each of them; the pebble comes after, launched.
    let (pebble_part, structure) = level.parts.split_last().unwrap();
    for p in structure {
        handles.push(insert(p, &mut bodies, &mut colliders).0);
    }
    step(
        &mut pipeline,
        QVec::ZERO,
        &params,
        &mut islands,
        &mut broad_phase,
        &mut narrow_phase,
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        &mut ccd_solver,
    );
    for (h, p) in handles[1..].iter().zip(&structure[1..]) {
        assert_eq!(
            *bodies[*h].position(),
            p.pose.p(),
            "the setup step moves nothing"
        );
    }
    for h in &handles[1..] {
        bodies[*h].sleep();
    }
    let (pebble_body, pebble) = insert(pebble_part, &mut bodies, &mut colliders);
    handles.push(pebble_body);
    let dynamic: Vec<RigidBodyHandle> = handles[1..].to_vec();
    let (x_min, x_max) = bounds(level);

    let states = |bodies: &RigidBodySet| -> Vec<Value> {
        dynamic
            .iter()
            .map(|h| bodies.get(*h).map_or(Value::Null, state_json))
            .collect()
    };
    let mut awake = Vec::new();
    let mut recorded = vec![states(&bodies)];
    let mut removals = Vec::new();
    let mut first_contact = None;
    let mut all_asleep = None;
    let mut calm_run = 0;
    let mut calm_tick = None;
    for tick in 1..=ticks {
        if prewake == Some(tick) {
            for h in &dynamic {
                if let Some(b) = bodies.get_mut(*h) {
                    b.wake_up(true);
                }
            }
        }
        step(
            &mut pipeline,
            gravity,
            &params,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd_solver,
        );
        if first_contact.is_none()
            && narrow_phase
                .contact_pairs_with(pebble)
                .any(|pair| pair.has_any_active_contact())
        {
            first_contact = Some(tick);
        }
        for (i, h) in dynamic.iter().enumerate() {
            let out = bodies.get(*h).is_some_and(|b| {
                let x = b.translation().x;
                x < x_min.f() || x > x_max.f()
            });
            if out {
                bodies.remove(
                    *h,
                    &mut islands,
                    &mut colliders,
                    &mut impulse_joints,
                    &mut multibody_joints,
                    true,
                );
                removals.push(json!({ "tick": tick, "body": i + 1 }));
            }
        }
        let alive: Vec<&RigidBody> = dynamic.iter().filter_map(|h| bodies.get(*h)).collect();
        let n_awake = alive.iter().filter(|b| !b.is_sleeping()).count();
        awake.push(n_awake);
        if all_asleep.is_none() && n_awake == 0 {
            all_asleep = Some(tick);
        }
        let calm = alive.iter().all(|b| {
            b.is_sleeping()
                || (b.linvel().length_squared() < CALM_LINEAR * CALM_LINEAR
                    && b.angvel() * b.angvel() < CALM_ANGULAR * CALM_ANGULAR)
        });
        calm_run = if calm { calm_run + 1 } else { 0 };
        if calm_tick.is_none() && calm_run == CALM_TICKS {
            calm_tick = Some(tick);
        }
        recorded.push(states(&bodies));
    }
    // Sampled ticks: every `period`-th, plus every 5th before the impact and every tick from the
    // one before the first pebble contact to four ticks after it.
    let contact = first_contact.expect("the pebble hits the structure");
    let samples: Vec<Value> = recorded
        .into_iter()
        .enumerate()
        .filter(|(t, _)| {
            t.is_multiple_of(period)
                || (*t < contact && t.is_multiple_of(5))
                || (contact - 1..=contact + 4).contains(t)
        })
        .map(|(t, states)| json!({ "tick": t, "states": states }))
        .collect();
    json!({
        "level": level.id,
        "config": id,
        "dt": jq(dt),
        "num_solver_iterations": iterations,
        "num_ticks": ticks,
        "bounds_x": [jq(x_min), jq(x_max)],
        "first_pebble_contact": first_contact,
        "all_asleep": all_asleep,
        "calm_end": calm_tick,
        "removals": removals,
        "awake": awake,
        "samples": samples,
    })
}

pub fn generate() -> Value {
    let gravity = QVec::snap(0.0, -9.81);
    let levels = levels();
    let levels_json: Vec<Value> = levels
        .iter()
        .map(|l| {
            let parts: Vec<Value> = l
                .parts
                .iter()
                .map(|p| {
                    json!({
                        "name": p.name, "role": p.role, "shape": p.shape.json(),
                        "pose": jqpose(p.pose), "material": p.material.name,
                        "density": jq(Q::snap(p.material.density)),
                        "friction": jq(Q::snap(p.material.friction)),
                        "restitution": jq(Q::snap(p.material.restitution)),
                                            })
                })
                .collect();
            json!({ "id": l.id, "note": l.note, "pebble_linvel": jqvec(l.pebble_linvel), "bodies": parts })
        })
        .collect();
    let mut runs = Vec::new();
    for l in &levels {
        for c in configs() {
            runs.push(run(l, c, gravity));
        }
    }
    json!({
        "family": "level_scenes",
        "note": "G0 level-shaped scenes: sleeping structure, cores, pebble; four step settings, 5 simulated seconds each",
        "gravity": jqvec(gravity),
        "calm_rule": { "linear": CALM_LINEAR, "angular": CALM_ANGULAR, "ticks": CALM_TICKS },
        "levels": levels_json,
        "runs": runs,
    })
}

/// Diagnostic of the wake tick (G0): upstream's trace at the first impact tick, with and
/// without the whole structure woken up before it (`cargo test --release level_checks -- --nocapture`).
#[cfg(test)]
mod level_checks {
    use super::*;

    #[test]
    fn prewake_before_impact() {
        let gravity = QVec::snap(0.0, -9.81);
        let level = &levels()[0];
        let config = configs()[0];
        for prewake in [None, Some(26)] {
            let run = run_with(level, config, gravity, prewake);
            let sample = run["samples"]
                .as_array()
                .unwrap()
                .iter()
                .find(|s| s["tick"] == 26)
                .unwrap();
            let vx: Vec<i64> = sample["states"]
                .as_array()
                .unwrap()
                .iter()
                .map(|s| s["linvel"]["raw"][0].as_i64().unwrap())
                .collect();
            println!("prewake {prewake:?}: vx at tick 26 {vx:?}");
        }
    }
}
