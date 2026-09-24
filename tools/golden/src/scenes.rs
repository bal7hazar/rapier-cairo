//! Family `scenes`: full-engine traces with the comparability settings of decision D11.

use crate::q::{jf, jq, jqpose, jqvec, jrot, jvec, QPose, QRot, QVec, Q};
use crate::shapes::ShapeSpec;
use rapier2d_f64::prelude::*;
use serde_json::{json, Value};

const NUM_STEPS: usize = 120;
/// Steps of `box_stack3` recorded with per-pair contact diagnostics (SO).
const STACK_DIAGNOSTIC_STEPS: usize = 8;
/// Re-seed step of the second `box_stack3` window: its contact impulses are recorded too (SO).
const STACK_RESEED_STEP: usize = 60;

#[cfg(test)]
mod kd_checks;

struct ColliderSpec {
    shape: ShapeSpec,
    pose_wrt_parent: QPose,
    density: Q,
    friction: Q,
    restitution: Q,
}

struct BodySpec {
    name: &'static str,
    dynamic: bool,
    pose: QPose,
    colliders: Vec<ColliderSpec>,
}

struct MotorSpec {
    target_pos: Q,
    target_vel: Q,
    stiffness: Q,
    damping: Q,
    max_force: Q,
    force_based: bool,
}

struct JointSpec {
    axis: Option<QVec>,
    limits: Option<[Q; 2]>,
    motor: Option<MotorSpec>,
    body1: usize,
    body2: usize,
    local_anchor1: QVec,
    local_anchor2: QVec,
}

struct SceneSpec {
    id: &'static str,
    note: &'static str,
    bodies: Vec<BodySpec>,
    joints: Vec<JointSpec>,
    /// Sleeping on (`RigidBodyBuilder::can_sleep(true)`, upstream's default thresholds): the
    /// samples then carry `sleeping` per body and the scene its `sleep_transitions` (SL).
    can_sleep: bool,
}

fn collider(shape: ShapeSpec, friction: f64, restitution: f64) -> ColliderSpec {
    ColliderSpec {
        shape,
        pose_wrt_parent: QPose::new(QVec::ZERO, QRot::IDENTITY),
        density: Q::ONE,
        friction: Q::snap(friction),
        restitution: Q::snap(restitution),
    }
}

/// Fixed 20 x 1 slab whose top face is the line y = 0.
fn ground(friction: f64, restitution: f64) -> BodySpec {
    BodySpec {
        name: "ground",
        dynamic: false,
        pose: QPose::translation(0.0, -0.5),
        colliders: vec![collider(
            ShapeSpec::cuboid(10.0, 0.5),
            friction,
            restitution,
        )],
    }
}

fn dynamic(name: &'static str, pose: QPose, collider: ColliderSpec) -> BodySpec {
    BodySpec {
        name,
        dynamic: true,
        pose,
        colliders: vec![collider],
    }
}

fn slope_scene(id: &'static str, note: &'static str, friction: f64) -> SceneSpec {
    // 30° slope: im = sin 30° = 0.5 is exact, re = cos 30° is snapped to Q32.32.
    let rot = QRot::from_degrees(30.0);
    assert_eq!(rot.im, Q::snap(0.5));
    // Slope normal n = (-sin, cos). The slab (half height 0.5) is centred at the origin and the
    // unit box sits 0.01 above its surface: centre = n * (0.5 + 0.5 + 0.01).
    let (nx, ny) = (-rot.im.f(), rot.re.f());
    let box_pose = QPose::new(QVec::snap(nx * 1.01, ny * 1.01), rot);
    SceneSpec {
        id,
        note,
        bodies: vec![
            BodySpec {
                name: "slope",
                dynamic: false,
                pose: QPose::new(QVec::ZERO, rot),
                colliders: vec![collider(ShapeSpec::cuboid(10.0, 0.5), friction, 0.0)],
            },
            dynamic(
                "box",
                box_pose,
                collider(ShapeSpec::cuboid(0.5, 0.5), friction, 0.0),
            ),
        ],
        joints: vec![],
        can_sleep: false,
    }
}

fn scenes() -> Vec<SceneSpec> {
    let mut scenes = vec![
        SceneSpec {
            id: "ball_drop",
            note: "ball (r = 0.5) released at y = 2 above a fixed slab, no restitution: free fall, speculative contact, rest",
            bodies: vec![
                ground(0.5, 0.0),
                dynamic("ball", QPose::translation(0.0, 2.0), collider(ShapeSpec::ball(0.5), 0.5, 0.0)),
            ],
            joints: vec![],
            can_sleep: false,
        },
        SceneSpec {
            id: "ball_bounce",
            note: "same as ball_drop with restitution 0.7 on both colliders (combine rule: average)",
            bodies: vec![
                ground(0.5, 0.7),
                dynamic("ball", QPose::translation(0.0, 2.0), collider(ShapeSpec::ball(0.5), 0.5, 0.7)),
            ],
            joints: vec![],
            can_sleep: false,
        },
        slope_scene(
            "box_slope_stick",
            "unit box on a 30 degree slope, friction 0.7 > tan(30 deg): the box must stay put",
            0.7,
        ),
        slope_scene(
            "box_slope_slide",
            "unit box on a 30 degree slope, friction 0.25 < tan(30 deg): the box slides",
            0.25,
        ),
        SceneSpec {
            id: "box_stack3",
            note: "three unit boxes stacked on a fixed slab with 0.01 gaps",
            bodies: vec![
                ground(0.5, 0.0),
                dynamic("box0", QPose::translation(0.0, 0.51), collider(ShapeSpec::cuboid(0.5, 0.5), 0.5, 0.0)),
                dynamic("box1", QPose::translation(0.0, 1.52), collider(ShapeSpec::cuboid(0.5, 0.5), 0.5, 0.0)),
                dynamic("box2", QPose::translation(0.0, 2.53), collider(ShapeSpec::cuboid(0.5, 0.5), 0.5, 0.0)),
            ],
            joints: vec![],
            can_sleep: false,
        },
        SceneSpec {
            id: "pendulum",
            note: "ball (r = 0.25) hanging from a fixed pivot by a revolute joint of length 1, released horizontally",
            bodies: vec![
                BodySpec {
                    name: "pivot",
                    dynamic: false,
                    pose: QPose::translation(0.0, 0.0),
                    colliders: vec![],
                },
                dynamic("bob", QPose::translation(1.0, 0.0), collider(ShapeSpec::ball(0.25), 0.5, 0.0)),
            ],
            joints: vec![JointSpec {
                axis: None, limits: None, motor: None,
                body1: 0,
                body2: 1,
                local_anchor1: QVec::ZERO,
                local_anchor2: QVec::snap(-1.0, 0.0),
            }],
            can_sleep: false,
        },
        // Sleeping on (SL): the stack settles and falls asleep as one island; the sleeping ball
        // is hit by a second one released high enough to land after it fell asleep, which wakes
        // it up. The first six scenes keep `can_sleep(false)`.
        SceneSpec {
            id: "box_stack3_sleep",
            note: "box_stack3 with sleeping on: the three boxes settle, then the whole island falls asleep",
            bodies: vec![
                ground(0.5, 0.0),
                dynamic("box0", QPose::translation(0.0, 0.51), collider(ShapeSpec::cuboid(0.5, 0.5), 0.5, 0.0)),
                dynamic("box1", QPose::translation(0.0, 1.52), collider(ShapeSpec::cuboid(0.5, 0.5), 0.5, 0.0)),
                dynamic("box2", QPose::translation(0.0, 2.53), collider(ShapeSpec::cuboid(0.5, 0.5), 0.5, 0.0)),
            ],
            joints: vec![],
            can_sleep: true,
        },
        SceneSpec {
            id: "ball_drop_sleep",
            note: "ball_drop with sleeping on, plus a second ball (r = 0.5) released at y = 11.5 that lands on the sleeping first ball and wakes it up",
            bodies: vec![
                ground(0.5, 0.0),
                dynamic("ball", QPose::translation(0.0, 2.0), collider(ShapeSpec::ball(0.5), 0.5, 0.0)),
                dynamic("ball2", QPose::translation(0.0, 11.5), collider(ShapeSpec::ball(0.5), 0.5, 0.0)),
            ],
            joints: vec![],
            can_sleep: true,
        },
    ];
    for id in ["pendulum_limited", "wheel_motor", "slider_limited", "servo"] {
        let pendulum = id == "pendulum_limited";
        let slider = id == "slider_limited";
        let motor = match id {
            "wheel_motor" => Some(MotorSpec {
                target_pos: Q::snap(0.0),
                target_vel: Q::snap(4.0),
                stiffness: Q::snap(0.0),
                damping: Q::snap(10.0),
                max_force: Q::snap(2.0),
                force_based: false,
            }),
            "servo" => Some(MotorSpec {
                target_pos: Q::snap(0.75),
                target_vel: Q::snap(0.0),
                stiffness: Q::snap(40.0),
                damping: Q::snap(8.0),
                max_force: Q::snap(3.0),
                force_based: true,
            }),
            _ => None,
        };
        scenes.push(SceneSpec {
            can_sleep: false,
            id,
            note: match id {
                "pendulum_limited" => "pendulum falling onto its -0.5 rad angular stop and held by gravity",
                "wheel_motor" => "free wheel pinned at its center, velocity motor target 4 rad/s, force cap 2",
                "slider_limited" => "vertical prismatic slider, gravity drives its free axis onto the 0.5 upper stop",
                _ => "pinned wheel with force-based angular position servo targeting 0.75 rad",
            },
            bodies: vec![
                BodySpec { name: "pivot", dynamic: false, pose: QPose::translation(0.0, 2.0), colliders: vec![] },
                dynamic("body", QPose::translation(if pendulum { 1.0 } else { 0.0 }, 2.0), collider(ShapeSpec::ball(0.25), 0.5, 0.0)),
            ],
            joints: vec![JointSpec {
                body1: 0, body2: 1,
                local_anchor1: QVec::ZERO,
                local_anchor2: if pendulum { QVec::snap(-1.0, 0.0) } else { QVec::ZERO },
                axis: if slider { Some(QVec::snap(0.0, -1.0)) } else { None },
                limits: if pendulum { Some([Q::snap(-0.5), Q::snap(0.5)]) } else if slider { Some([Q::snap(0.0), Q::snap(0.5)]) } else { None },
                motor,
            }],
        });
    }
    scenes
}

/// Steps recorded: the initial state, every step up to 10, then every 10th step.
fn is_sampled(step: usize) -> bool {
    step <= 10 || step.is_multiple_of(10)
}

/// One manifold as recorded by the contact diagnostics (SD, SO).
fn manifold_json(m: &ContactManifold) -> Value {
    let points: Vec<Value> = m
        .points
        .iter()
        .map(|p| {
            json!({
                "local_p1": jvec(p.local_p1), "local_p2": jvec(p.local_p2),
                "dist": jf(p.dist), "fid1": p.fid1.0, "fid2": p.fid2.0,
                "impulse": jf(p.data.impulse),
                "tangent_impulse": jf(p.data.tangent_impulse.x),
                "warmstart_impulse": jf(p.data.warmstart_impulse),
                "warmstart_tangent_impulse": jf(p.data.warmstart_tangent_impulse.x),
                "solver_dp1": jvec(p.data.solver_dp1),
                "solver_dp2": jvec(p.data.solver_dp2),
            })
        })
        .collect();
    let solver_contacts: Vec<Value> = m
        .data
        .solver_contacts
        .iter()
        .map(|c| {
            json!({
                "contact_id": c.contact_id[0], "anchor1": jvec(c.anchor1),
                "anchor2": jvec(c.anchor2), "dist": jf(c.dist),
                "tangent_velocity": jvec(c.tangent_velocity),
            })
        })
        .collect();
    json!({
        "local_n1": jvec(m.local_n1), "local_n2": jvec(m.local_n2),
        "normal": jvec(m.data.normal), "friction": jf(m.data.friction),
        "restitution": jf(m.data.restitution), "points": points,
        "solver_contacts": solver_contacts,
    })
}

fn run(scene: &SceneSpec, gravity: QVec, dt: Q, prewake: bool) -> Value {
    // Comparability settings (decision D11). The block solver is removed at compile time by
    // building rapier without its `block-solver` default feature.
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

    let mut handles = Vec::new();
    let mut bodies_json = Vec::new();
    for spec in &scene.bodies {
        let builder = if spec.dynamic {
            RigidBodyBuilder::dynamic()
        } else {
            RigidBodyBuilder::fixed()
        };
        let handle = bodies.insert(
            builder
                .pose(spec.pose.p())
                .can_sleep(scene.can_sleep)
                .ccd_enabled(false),
        );
        let mut colliders_json = Vec::new();
        for c in &spec.colliders {
            let builder = ColliderBuilder::new(c.shape.shared())
                .position(c.pose_wrt_parent.p())
                .density(c.density.f())
                .friction(c.friction.f())
                .restitution(c.restitution.f());
            colliders.insert_with_parent(builder, handle, &mut bodies);
            colliders_json.push(json!({
                "shape": c.shape.json(),
                "pose_wrt_parent": jqpose(c.pose_wrt_parent),
                "density": jq(c.density),
                "friction": jq(c.friction),
                "restitution": jq(c.restitution),
            }));
        }
        let body = &bodies[handle];
        bodies_json.push(json!({
            "name": spec.name,
            "type": if spec.dynamic { "dynamic" } else { "fixed" },
            "pose": jqpose(spec.pose),
            "linear_damping": jf(body.linear_damping()),
            "angular_damping": jf(body.angular_damping()),
            "gravity_scale": jf(body.gravity_scale()),
            "colliders": colliders_json,
        }));
        handles.push(handle);
    }

    let mut joints_json = Vec::new();
    for j in &scene.joints {
        let mut joint = if let Some(axis) = j.axis {
            PrismaticJointBuilder::new(axis.v())
                .local_anchor1(j.local_anchor1.v())
                .local_anchor2(j.local_anchor2.v())
                .build()
                .data
        } else {
            RevoluteJointBuilder::new()
                .local_anchor1(j.local_anchor1.v())
                .local_anchor2(j.local_anchor2.v())
                .build()
                .data
        };
        let axis = if j.axis.is_some() {
            JointAxis::LinX
        } else {
            JointAxis::AngX
        };
        if let Some(l) = j.limits {
            joint.set_limits(axis, [l[0].f(), l[1].f()]);
        }
        if let Some(m) = &j.motor {
            joint.set_motor(
                axis,
                m.target_pos.f(),
                m.target_vel.f(),
                m.stiffness.f(),
                m.damping.f(),
            );
            joint.set_motor_max_force(axis, m.max_force.f());
            joint.set_motor_model(
                axis,
                if m.force_based {
                    MotorModel::ForceBased
                } else {
                    MotorModel::AccelerationBased
                },
            );
        }
        impulse_joints.insert(handles[j.body1], handles[j.body2], joint, true);
        let mut desc = json!({
            "type": if j.axis.is_some() { "prismatic" } else { "revolute" },
            "body1": scene.bodies[j.body1].name,
            "body2": scene.bodies[j.body2].name,
            "local_anchor1": jqvec(j.local_anchor1),
            "local_anchor2": jqvec(j.local_anchor2),
        });
        if let Some(axis) = j.axis {
            desc["axis"] = jqvec(axis);
        }
        if let Some(l) = j.limits {
            desc["limits"] = json!([jq(l[0]), jq(l[1])]);
        }
        if let Some(m) = &j.motor {
            desc["motor"] = json!({ "target_pos": jq(m.target_pos), "target_vel": jq(m.target_vel), "stiffness": jq(m.stiffness), "damping": jq(m.damping), "max_force": jq(m.max_force), "force_based": m.force_based });
        }
        joints_json.push(desc);
    }

    let sample = |step: usize, bodies: &RigidBodySet| -> Value {
        let states: Vec<Value> = scene
            .bodies
            .iter()
            .zip(&handles)
            .filter(|(spec, _)| spec.dynamic)
            .map(|(spec, handle)| {
                let body = &bodies[*handle];
                let mut state = json!({
                    "body": spec.name,
                    "translation": jvec(body.translation()),
                    "rotation": jrot(*body.rotation()),
                    "linvel": jvec(body.linvel()),
                    "angvel": jf(body.angvel()),
                });
                if scene.can_sleep {
                    state["sleeping"] = json!(body.is_sleeping());
                }
                state
            })
            .collect();
        json!({ "step": step, "bodies": states })
    };

    let mut diagnostics = Vec::new();
    let mut samples = vec![sample(0, &bodies)];
    // Sleeping (SL): every flip of `is_sleeping` of a dynamic body, with the step after which
    // it is observed (the sleep decision and the wake-ups happen inside that step).
    let mut sleep_transitions = Vec::new();
    let mut was_sleeping: Vec<bool> = handles.iter().map(|h| bodies[*h].is_sleeping()).collect();
    for step in 1..=NUM_STEPS {
        // SI control: wake before candidate collection, so the ground contact participates
        // immediately instead of retaining its dormant solver hint through the impact.
        if prewake && step == 87 {
            bodies[handles[1]].wake_up(true);
        }
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
        if scene.id.starts_with("box_slope_") && step <= 10 {
            let c1 = bodies[handles[0]].colliders()[0];
            let c2 = bodies[handles[1]].colliders()[0];
            let manifolds: Vec<Value> = narrow_phase
                .contact_pair(c1, c2)
                .map(|pair| pair.manifolds.iter().map(manifold_json).collect())
                .unwrap_or_default();
            diagnostics.push(json!({ "step": step, "manifolds": manifolds,
                "body": sample(step, &bodies)["bodies"][0] }));
        }
        if scene.id == "box_stack3" && (step <= STACK_DIAGNOSTIC_STEPS || step == STACK_RESEED_STEP)
        {
            // Every contact pair in upstream's contact-graph edge order.
            let pairs: Vec<Value> = narrow_phase
                .contact_pairs()
                .map(|pair| {
                    json!({
                        "collider1": pair.collider1.into_raw_parts().0,
                        "collider2": pair.collider2.into_raw_parts().0,
                        "active": pair.has_any_active_contact(),
                        "manifolds": pair.manifolds.iter().map(manifold_json).collect::<Vec<_>>(),
                    })
                })
                .collect();
            diagnostics.push(json!({ "step": step, "pairs": pairs,
                "bodies": sample(step, &bodies)["bodies"] }));
        }
        if scene.id == "ball_drop_sleep" && (step == 65 || step == 66 || step >= 80) {
            let pairs: Vec<Value> = narrow_phase.contact_pairs().map(|pair| json!({
                "collider1": pair.collider1.into_raw_parts().0,
                "collider2": pair.collider2.into_raw_parts().0,
                "active": pair.has_any_active_contact(),
                "manifolds": pair.manifolds.iter().map(|m| {
                    let mut v = manifold_json(m);
                    v["solver_contact_ids"] = json!(m.data.solver_contacts.iter().map(|c| c.contact_id[0]).collect::<Vec<_>>());
                    v
                }).collect::<Vec<_>>(),
            })).collect();
            let activation: Vec<Value> = handles.iter().skip(1).map(|h| {
                let a = bodies[*h].activation();
                json!({"sleeping": a.sleeping, "timer": jf(a.time_since_can_sleep)})
            }).collect();
            diagnostics.push(json!({"step": step, "bodies": sample(step, &bodies)["bodies"],
                "activation": activation, "pairs": pairs}));
        }
        if scene.can_sleep {
            for (k, handle) in handles.iter().enumerate() {
                let sleeping = bodies[*handle].is_sleeping();
                if scene.bodies[k].dynamic && sleeping != was_sleeping[k] {
                    sleep_transitions.push(json!({
                        "step": step, "body": scene.bodies[k].name, "sleeping": sleeping,
                    }));
                }
                was_sleeping[k] = sleeping;
            }
        }
        if is_sampled(step) {
            samples.push(sample(step, &bodies));
        }
    }

    let mut result = json!({
        "id": scene.id,
        "note": scene.note,
        "can_sleep": scene.can_sleep,
        "bodies": bodies_json,
        "joints": joints_json,
        "samples": samples,
    });
    if scene.can_sleep {
        result["sleep_transitions"] = json!(sleep_transitions);
    }
    if scene.id.starts_with("box_slope_") {
        result["contact_diagnostics"] = json!({
            "timing": "after step; geometry and solver arms from pre-solve collision detection; impulses and body state after solve",
            "anchor_frames": "solver_contacts anchors are CoM-local (world for fixed side); solver_dp1/2 are frozen world lever arms at the common midpoint",
            "substeps": "body velocities inside a step are not exposed by the public API",
            "steps": diagnostics,
        });
    } else if scene.id == "ball_drop_sleep" {
        result["impact_diagnostics"] = json!({
            "timing": "after step; manifold geometry is pre-solve, impulses post-solve; timers are upstream start-of-step updates; internal solver substeps are not exposed by the public API",
            "steps": diagnostics,
        });
    } else if scene.id == "box_stack3" {
        result["contact_diagnostics"] = json!({
            "timing": "after step; geometry and solver arms from pre-solve collision detection; impulses and body state after solve",
            "pair_order": "narrow_phase.contact_pairs(): contact-graph edge order; colliders by arena index (collider i belongs to body i)",
            "solve_order": "not observable through the public API: the solver walks the persistent colour buckets (ContactPair::solver_color, pub(crate)); see README, SO",
            "anchor_frames": "solver_contacts anchors are CoM-local (world for fixed side); solver_dp1/2 are frozen world lever arms at the common midpoint",
            "steps": diagnostics,
        });
    }
    result
}

pub fn generate() -> Value {
    let gravity = QVec::snap(0.0, -9.81);
    let dt = Q::snap(1.0 / 60.0);
    let cases: Vec<Value> = scenes().iter().map(|s| {
        let mut trace = run(s, gravity, dt, false);
        if s.id == "ball_drop_sleep" {
            let control = run(s, gravity, dt, true);
            trace["impact_diagnostics"]["prewake_steps"] =
                control["impact_diagnostics"]["steps"].clone();
            trace["impact_diagnostics"]["prewake_note"] = json!(
                "counterfactual: RigidBody::wake_up(true) on lower ball immediately before step 87; all other inputs unchanged");
        }
        trace
    }).collect();
    let defaults = IntegrationParameters::default();

    json!({
        "family": "scenes",
        "gravity": jqvec(gravity),
        "dt": jq(dt),
        "num_steps": NUM_STEPS,
        "sampling": "step 0 (initial state), steps 1..=10, then every 10th step; state read after PhysicsPipeline::step",
        "body_order": "bodies are inserted in the listed order; colliders and joints likewise",
        "settings": {
            "contact_recycling": false,
            "contact_clustering": false,
            "max_ccd_substeps": 0,
            "block_solver": false,
            "can_sleep": false,
            "can_sleep_note": "per scene: `can_sleep` is true for box_stack3_sleep and ball_drop_sleep only (SL), whose samples carry `sleeping` and whose `sleep_transitions` list every flip",
            "ccd_enabled": false,
            "num_solver_iterations": defaults.num_solver_iterations,
            "num_internal_pgs_iterations": defaults.num_internal_pgs_iterations,
            "num_internal_stabilization_iterations": defaults.num_internal_stabilization_iterations,
            "warmstart_coefficient": jf(defaults.warmstart_coefficient),
            "friction_combine_rule": "average (default)",
            "restitution_combine_rule": "average (default)",
        },
        "scenes": cases,
    })
}
