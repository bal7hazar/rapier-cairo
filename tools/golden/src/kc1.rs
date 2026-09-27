//! Work package KC1: the controllers of `rapier::control`. Two families:
//!
//! * `pid_corrections`: `PdController` / `PidController` on a dynamic body whose centre of mass
//!   is off its origin (one cuboid collider offset in the body frame), towards a target pose and
//!   velocity: the PD's `rigid_body_correction`, `linear_rigid_body_correction` and
//!   `angular_rigid_body_correction`; three successive `PidController::rigid_body_correction`
//!   calls (the last output and the integrals), and a fresh PID's linear and angular corrections.
//! * `character_moves`: `KinematicCharacterController::move_shape` in small scenes (flat ground,
//!   a wall, 30 and 60 degree ramps, a step, a ledge, a kinematic platform, a dynamic box): the
//!   effective translation, the grounded / sliding flags, every collision event (collider index,
//!   time of impact, world normal and witness on the collider, translations applied and
//!   remaining); for `push`, `solve_character_collision_impulses` on the box afterwards (its
//!   velocities).
//!
//! Every input is Q32.32 (`q.rs`): the controller's options too (upstream's `0.01`, `0.2`, `1e-4`
//! and `pi / 4` defaults are snapped and handed over explicitly). The scenes are built without a
//! step: bodies and colliders inserted (upstream updates the mass properties at attachment, as the
//! port), each collider's AABB given to the broad phase, then `as_query_pipeline`.

use crate::q::{jf, jq, jqpose, jqvec, jvec, QPose, QRot, QVec, Q};
use crate::shapes::ShapeSpec;
use rapier2d_f64::control::{
    CharacterAutostep, CharacterLength, KinematicCharacterController, PdController, PidController,
};
use rapier2d_f64::parry::query::DefaultQueryDispatcher;
use rapier2d_f64::prelude::*;
use serde_json::{json, Value};

fn qv(x: f64, y: f64) -> QVec {
    QVec::snap(x, y)
}

/// The `AxesMask` of `bits` (`LIN_X` 1, `LIN_Y` 2, `ANG_Z` 32).
fn axes(bits: u8) -> AxesMask {
    AxesMask::from_bits(bits).unwrap()
}

// ---------------------------------------------------------------------------------------------
// pid_corrections
// ---------------------------------------------------------------------------------------------

/// `(id, body pose, linvel, angvel, collider offset, kp, ki, kd, axes, dt, target pose, target
/// linvel, target angvel)`.
type PidConfig = (&'static str, QPose, QVec, Q, QVec, Q, Q, Q, u8, Q, QPose, QVec, Q);

fn pid_configs() -> Vec<PidConfig> {
    let dt = Q::snap(1.0 / 60.0);
    vec![
        (
            "turned_all_axes",
            QPose::new(qv(1.0, 2.0), QRot::from_degrees(30.0)),
            qv(0.5, -0.25),
            Q::snap(0.75),
            qv(0.3, -0.2),
            Q::snap(60.0),
            Q::snap(1.0),
            Q::snap(0.8),
            35,
            dt,
            QPose::new(qv(1.5, 1.75), QRot::from_degrees(75.0)),
            qv(0.25, 0.5),
            Q::snap(-0.5),
        ),
        (
            "linear_only",
            QPose::new(qv(-3.0, 0.5), QRot::from_degrees(-40.0)),
            qv(-1.0, 2.0),
            Q::snap(-1.5),
            qv(-0.1, 0.4),
            Q::snap(20.0),
            Q::snap(0.5),
            Q::snap(0.3),
            3,
            dt,
            QPose::new(qv(-2.0, 0.0), QRot::IDENTITY),
            qv(0.0, 0.0),
            Q::ZERO,
        ),
        (
            "angular_only_half_turn",
            QPose::new(qv(0.0, 0.0), QRot::from_degrees(170.0)),
            qv(0.0, 0.0),
            Q::snap(0.25),
            qv(0.2, 0.2),
            Q::snap(10.0),
            Q::snap(2.0),
            Q::snap(1.0),
            32,
            Q::snap(1.0 / 30.0),
            QPose::new(qv(0.1, -0.1), QRot::from_degrees(-170.0)),
            qv(0.0, 0.0),
            Q::snap(1.0),
        ),
        (
            "y_axis_at_target",
            QPose::new(qv(4.0, -1.0), QRot::from_degrees(10.0)),
            qv(0.2, 0.1),
            Q::snap(0.1),
            qv(0.0, 0.0),
            Q::snap(60.0),
            Q::snap(1.0),
            Q::snap(0.8),
            2,
            dt,
            QPose::new(qv(4.0, -1.0), QRot::from_degrees(10.0)),
            qv(0.2, 0.1),
            Q::snap(0.1),
        ),
    ]
}

fn vels(v: RigidBodyVelocity<Real>) -> Value {
    json!({ "linvel": jvec(v.linvel), "angvel": jf(v.angvel) })
}

fn pid_case(c: PidConfig) -> Value {
    let (id, pose, linvel, angvel, offset, kp, ki, kd, bits, dt, target, tlin, tang) = c;
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let h = bodies.insert(
        RigidBodyBuilder::dynamic().pose(pose.p()).linvel(linvel.v()).angvel(angvel.f()),
    );
    colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.4, 0.3).translation(offset.v()),
        h,
        &mut bodies,
    );
    let rb = &bodies[h];
    let target_vels = RigidBodyVelocity { linvel: tlin.v(), angvel: tang.f() };
    let pd = PdController::new(kp.f(), kd.f(), axes(bits));
    let pd_full = pd.rigid_body_correction(rb, target.p(), target_vels);
    let pd_linear = pd.linear_rigid_body_correction(rb, target.translation.v(), tlin.v());
    let pd_angular = pd.angular_rigid_body_correction(rb, target.rotation.r(), tang.f());
    let mut pid = PidController::new(kp.f(), ki.f(), kd.f(), axes(bits));
    let mut last = pid.rigid_body_correction(dt.f(), rb, target.p(), target_vels);
    for _ in 0..2 {
        last = pid.rigid_body_correction(dt.f(), rb, target.p(), target_vels);
    }
    let mut fresh = PidController::new(kp.f(), ki.f(), kd.f(), axes(bits));
    let pid_linear = fresh.linear_rigid_body_correction(dt.f(), rb, target.translation.v(), tlin.v());
    let mut fresh = PidController::new(kp.f(), ki.f(), kd.f(), axes(bits));
    let pid_angular = fresh.angular_rigid_body_correction(dt.f(), rb, target.rotation.r(), tang.f());
    json!({
        "id": id,
        "pose": jqpose(pose),
        "linvel": jqvec(linvel),
        "angvel": jq(angvel),
        "offset": jqvec(offset),
        "local_com": jvec(rb.local_center_of_mass()),
        "kp": jq(kp), "ki": jq(ki), "kd": jq(kd),
        "axes": bits,
        "dt": jq(dt),
        "target": jqpose(target),
        "target_linvel": jqvec(tlin),
        "target_angvel": jq(tang),
        "pd": vels(pd_full),
        "pd_linear": jvec(pd_linear),
        "pd_angular": jf(pd_angular),
        "pid_third": vels(last),
        "pid_lin_integral": jvec(pid.lin_integral),
        "pid_ang_integral": jf(pid.ang_integral),
        "pid_linear": jvec(pid_linear),
        "pid_angular": jf(pid_angular),
    })
}

pub fn pid_corrections() -> Value {
    json!({
        "description": "PdController / PidController corrections of a dynamic body (COM off its origin: one cuboid 0.4 x 0.3 collider at `offset`) towards a target; `pid_third` is the third of three successive PID calls, then the integrals; `pid_linear` / `pid_angular` are single calls on fresh PIDs.",
        "cases": pid_configs().into_iter().map(pid_case).collect::<Vec<_>>(),
    })
}

// ---------------------------------------------------------------------------------------------
// character_moves
// ---------------------------------------------------------------------------------------------

#[derive(Copy, Clone)]
enum Kind {
    /// A standalone collider (no parent).
    Fixed,
    /// A velocity-based kinematic body moving at the given linear velocity.
    Kinematic(QVec),
    /// A dynamic body.
    Dynamic,
}

/// `(kind, pose, shape)` of every collider of a scene, in handle order.
type Scene = (&'static str, Vec<(Kind, QPose, ShapeSpec)>);

fn ground() -> (Kind, QPose, ShapeSpec) {
    (Kind::Fixed, QPose::translation(0.0, -0.5), ShapeSpec::cuboid(10.0, 0.5))
}

fn scenes() -> Vec<Scene> {
    vec![
        ("flat", vec![ground()]),
        ("wall", vec![ground(), (Kind::Fixed, QPose::translation(2.25, 2.0), ShapeSpec::cuboid(0.25, 2.0))]),
        (
            "ramp30",
            vec![ground(), (Kind::Fixed, QPose::new(qv(2.5, 0.6), QRot::from_degrees(30.0)), ShapeSpec::cuboid(2.0, 0.1))],
        ),
        (
            "ramps60",
            vec![
                ground(),
                (Kind::Fixed, QPose::new(qv(2.0, 1.2), QRot::from_degrees(60.0)), ShapeSpec::cuboid(2.0, 0.1)),
                (Kind::Fixed, QPose::new(qv(-2.0, 1.2), QRot::from_degrees(-60.0)), ShapeSpec::cuboid(2.0, 0.1)),
            ],
        ),
        ("step", vec![ground(), (Kind::Fixed, QPose::translation(2.0, 0.1), ShapeSpec::cuboid(1.0, 0.1))]),
        ("ledge", vec![ground(), (Kind::Fixed, QPose::translation(-1.0, 0.075), ShapeSpec::cuboid(1.0, 0.075))]),
        (
            "platform",
            vec![(Kind::Kinematic(qv(1.0, 0.0)), QPose::translation(0.0, -0.25), ShapeSpec::cuboid(2.0, 0.25))],
        ),
        ("push", vec![ground(), (Kind::Dynamic, QPose::translation(1.2, 0.3), ShapeSpec::cuboid(0.3, 0.3))]),
    ]
}

/// Upstream's defaults, snapped: offset `Relative(0.01)`, climb / slide `pi / 4`, snap
/// `Relative(0.2)`, nudge `1e-4`.
#[derive(Copy, Clone)]
struct Options {
    slide: bool,
    /// `(max_height, min_width, include_dynamic_bodies)`, absolute lengths.
    autostep: Option<(Q, Q, bool)>,
    max_climb: Q,
    min_slide: Q,
    /// Relative snap distance.
    snap: Option<Q>,
}

fn defaults() -> Options {
    Options {
        slide: true,
        autostep: None,
        max_climb: Q::snap(std::f64::consts::FRAC_PI_4),
        min_slide: Q::snap(std::f64::consts::FRAC_PI_4),
        snap: Some(Q::snap(0.2)),
    }
}

fn offset() -> Q {
    Q::snap(0.01)
}

fn nudge() -> Q {
    Q::snap(1.0e-4)
}

fn controller(o: Options) -> KinematicCharacterController {
    KinematicCharacterController {
        up: Vector::Y,
        offset: CharacterLength::Relative(offset().f()),
        slide: o.slide,
        autostep: o.autostep.map(|(h, w, d)| CharacterAutostep {
            max_height: CharacterLength::Absolute(h.f()),
            min_width: CharacterLength::Absolute(w.f()),
            include_dynamic_bodies: d,
        }),
        max_slope_climb_angle: o.max_climb.f(),
        min_slope_slide_angle: o.min_slide.f(),
        snap_to_ground: o.snap.map(|s| CharacterLength::Relative(s.f())),
        normal_nudge_factor: nudge().f(),
    }
}

fn options_json(o: Options) -> Value {
    json!({
        "slide": o.slide,
        "autostep": o.autostep.map(|(h, w, d)| json!({ "max_height": jq(h), "min_width": jq(w), "include_dynamic_bodies": d })),
        "max_slope_climb_angle": jq(o.max_climb),
        "min_slope_slide_angle": jq(o.min_slide),
        "snap_to_ground": o.snap.map(jq),
        "offset": jq(offset()),
        "normal_nudge_factor": jq(nudge()),
    })
}

/// `(id, scene index, character shape, pose, desired translation, options)`.
type MoveConfig = (&'static str, usize, ShapeSpec, QPose, QVec, Options);

fn move_configs() -> Vec<MoveConfig> {
    let ball = ShapeSpec::ball(0.5);
    let cub = ShapeSpec::cuboid(0.3, 0.5);
    let cap = ShapeSpec::capsule_y(0.3, 0.3);
    let d = defaults();
    let at = |x: f64, y: f64| QPose::translation(x, y);
    let step = Options { autostep: Some((Q::snap(0.3), Q::snap(0.2), true)), ..d };
    vec![
        ("flat_walk_ball", 0, ball, at(0.0, 0.505), qv(0.3, -0.1), d),
        ("flat_walk_cuboid", 0, cub, at(0.0, 0.51), qv(0.25, -0.05), d),
        ("flat_walk_capsule", 0, cap, at(0.0, 0.61), qv(0.2, -0.2), d),
        ("airborne", 0, ball, at(0.0, 3.0), qv(0.1, -0.3), d),
        ("wall_slide", 1, ball, at(1.3, 0.505), qv(0.6, -0.05), d),
        ("wall_no_slide", 1, ball, at(1.3, 0.505), qv(0.6, -0.05), Options { slide: false, ..d }),
        ("wall_climb_intent", 1, cub, at(1.5, 0.51), qv(0.4, 0.3), d),
        ("slope_climb", 2, ball, at(0.0, 0.505), qv(1.5, -0.05), d),
        ("slope_rest", 2, ball, at(2.2, 1.124), qv(0.0, -0.3), d),
        ("steep_blocked", 3, ball, at(0.0, 0.505), qv(1.2, -0.05), d),
        ("steep_left_signed_angle", 3, ball, at(0.0, 0.505), qv(-1.2, -0.05), d),
        ("steep_slide_down", 3, ball, at(1.476, 1.5025), qv(0.0, -0.3), d),
        ("step_autostep", 4, cub, at(0.4, 0.51), qv(0.6, -0.02), step),
        ("step_blocked", 4, cub, at(0.4, 0.51), qv(0.6, -0.02), d),
        ("step_too_high", 4, cub, at(0.4, 0.51), qv(0.6, -0.02), Options { autostep: Some((Q::snap(0.05), Q::snap(0.2), true)), ..d }),
        ("snap_ledge", 5, ball, at(-0.2, 0.68), qv(1.2, 0.0), d),
        ("no_snap_ledge", 5, ball, at(-0.2, 0.68), qv(1.2, 0.0), Options { snap: None, ..d }),
        ("depenetrate", 0, ball, at(0.0, 0.4), qv(0.0, 0.0), d),
        ("platform", 6, ball, at(0.0, 0.505), qv(0.1, -0.01), d),
        ("push", 7, cub, at(0.3, 0.51), qv(0.5, -0.01), d),
    ]
}

fn scene_json(s: &Scene) -> Value {
    json!({
        "id": s.0,
        "colliders": s.1.iter().map(|(k, p, sh)| {
            let (kind, linvel) = match k {
                Kind::Fixed => ("fixed", QVec::ZERO),
                Kind::Kinematic(v) => ("kinematic", *v),
                Kind::Dynamic => ("dynamic", QVec::ZERO),
            };
            json!({ "kind": kind, "pose": jqpose(*p), "linvel": jqvec(linvel), "shape": sh.json() })
        }).collect::<Vec<_>>(),
    })
}

fn move_case(c: MoveConfig, scenes: &[Scene]) -> Value {
    let (id, scene, shape, pos, desired, options) = c;
    let dt = Q::snap(1.0 / 60.0);
    let params = IntegrationParameters { dt: dt.f(), ..Default::default() };
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut body_of = vec![];
    for (kind, p, sh) in &scenes[scene].1 {
        let builder = ColliderBuilder::new(sh.shared());
        match kind {
            Kind::Fixed => {
                colliders.insert(builder.position(p.p()));
                body_of.push(None);
            }
            Kind::Kinematic(v) => {
                let b = bodies.insert(RigidBodyBuilder::kinematic_velocity_based().pose(p.p()).linvel(v.v()));
                colliders.insert_with_parent(builder, b, &mut bodies);
                body_of.push(Some(b));
            }
            Kind::Dynamic => {
                let b = bodies.insert(RigidBodyBuilder::dynamic().pose(p.p()));
                colliders.insert_with_parent(builder, b, &mut bodies);
                body_of.push(Some(b));
            }
        }
    }
    let mut broad_phase = BroadPhaseBvh::new();
    for (handle, co) in colliders.iter() {
        broad_phase.set_aabb(&params, handle, co.compute_aabb());
    }
    let ctrl = controller(options);
    let shared = shape.shared();
    let mut collisions = vec![];
    let movement = {
        let queries = broad_phase.as_query_pipeline(&DefaultQueryDispatcher, &bodies, &colliders, QueryFilter::default());
        ctrl.move_shape(dt.f(), &queries, &*shared, &pos.p(), desired.v(), |c| collisions.push(c))
    };
    let mut pushed = Value::Null;
    if id == "push" {
        let mass = Q::snap(2.0);
        let mut queries = broad_phase.as_query_pipeline_mut(&DefaultQueryDispatcher, &mut bodies, &mut colliders, QueryFilter::default());
        ctrl.solve_character_collision_impulses(dt.f(), &mut queries, &*shared, mass.f(), collisions.iter());
        let b = body_of[1].unwrap();
        pushed = json!({ "mass": jq(mass), "linvel": jvec(bodies[b].linvel()), "angvel": jf(bodies[b].angvel()) });
    }
    json!({
        "id": id,
        "scene": scene,
        "shape": shape.json(),
        "pos": jqpose(pos),
        "desired": jqvec(desired),
        "dt": jq(dt),
        "options": options_json(options),
        "translation": jvec(movement.translation),
        "grounded": movement.grounded,
        "is_sliding_down_slope": movement.is_sliding_down_slope,
        "collisions": collisions.iter().map(|c| json!({
            "collider": c.handle.into_raw_parts().0,
            "toi": jf(c.hit.time_of_impact),
            "normal1": jvec(c.hit.normal1),
            "witness1": jvec(c.hit.witness1),
            "translation_applied": jvec(c.translation_applied),
            "translation_remaining": jvec(c.translation_remaining),
        })).collect::<Vec<_>>(),
        "pushed": pushed,
    })
}

pub fn character_moves() -> Value {
    let scenes = scenes();
    json!({
        "description": "KinematicCharacterController::move_shape (up = +Y) in small scenes; collisions in callback order, `collider` = collider index (scene order); `pushed` = the dynamic box after solve_character_collision_impulses.",
        "scenes": scenes.iter().map(scene_json).collect::<Vec<_>>(),
        "cases": move_configs().into_iter().map(|c| move_case(c, &scenes)).collect::<Vec<_>>(),
    })
}
