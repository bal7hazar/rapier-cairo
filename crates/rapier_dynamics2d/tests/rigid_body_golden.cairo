//! Golden replay of the rigid-body components against the `rapier2d-f64` traces.
//!
//! Two comparisons, both from `rapier_golden`:
//!
//! * the free-fall phase of the `ball_drop` scene, replayed step by step with the four substeps
//!   upstream uses (`num_solver_iterations = 4`), stopping before the ball touches the ground —
//!   there is no contact solver yet (package DB) and no shape query (GA/GB);
//! * the world-space mass properties of the two-collider body of `mass_properties.json`.
//!
//! Tolerances follow `tools/golden/README.md`: `2^12 · step` ulp on the positions of a
//! single-contact scene and twice that on its velocities, 16 ulp on the mass properties (the
//! world centre of mass is one pose product away from an exact input).

use fixed::{Fixed, ONE, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_core::rigid_body::{RigidBodyDamping, RigidBodyType};
use rapier_dynamics2d::rigid_body::forces::{RigidBodyForces, RigidBodyForcesTrait};
use rapier_dynamics2d::rigid_body::locked_axes::LockedAxesTrait;
use rapier_dynamics2d::rigid_body::mass_props::{RigidBodyMassProps, RigidBodyMassPropsTrait};
use rapier_dynamics2d::rigid_body::position::{RigidBodyPosition, RigidBodyPositionTrait};
use rapier_dynamics2d::rigid_body::velocity::{RigidBodyVelocity, RigidBodyVelocityTrait};
use rapier_geometry2d::mass::MassProperties;
use rapier_golden::compare::{vec2_within, within};
use rapier_golden::types::{BodyStateRaw, MassPropertiesRaw, PoseRaw, SceneSampleRaw, Vec2Raw};
use rapier_golden::{integration_parameters, mass_properties, scenes};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;

/// Number of substeps per step, upstream's `num_solver_iterations`.
const SUBSTEPS: u32 = 4;
/// Last sample of `ball_drop` taken before the ball reaches the ground (`y = 0.5` at step ~33).
const LAST_FREE_FALL_STEP: u32 = 30;

fn vector(v: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: v.x }, y: Fixed { raw: v.y } }
}
fn pose(p: PoseRaw) -> Pose2 {
    Pose2Trait::new(
        vector(p.translation),
        Rot2 { re: Fixed { raw: p.rotation.re }, im: Fixed { raw: p.rotation.im } },
    )
}
fn raw(v: Vec2) -> Vec2Raw {
    Vec2Raw { x: v.x.raw, y: v.y.raw }
}
fn local_mprops(m: MassPropertiesRaw) -> MassProperties {
    MassProperties {
        local_com: vector(m.local_com),
        inv_mass: Fixed { raw: m.inv_mass },
        inv_principal_inertia: Fixed { raw: m.inv_principal_inertia },
    }
}

/// One step of the engine on a body in free flight: gravity becomes a force once, then four
/// substeps of `forces.integrate` followed by `vels.integrate`, exactly as the substep solver
/// does between two constraint passes.
fn step(
    dt: Fixed,
    gravity: Vec2,
    mprops: RigidBodyMassProps,
    forces: RigidBodyForces,
    vels: RigidBodyVelocity,
    pos: RigidBodyPosition,
) -> (RigidBodyForces, RigidBodyVelocity, RigidBodyPosition) {
    let forces = forces.compute_effective_force_and_torque(gravity, mprops.effective_mass());
    let substep_dt = Fixed { raw: dt.raw / SUBSTEPS.into() };
    let mut vels = vels;
    let mut pos = pos;
    let mut i: u32 = 0;
    while i != SUBSTEPS {
        // Upstream's substep: add the velocity increment of the external forces, solve the
        // constraints (none here), then integrate the pose with the updated velocities.
        vels = forces.integrate(substep_dt, vels, mprops);
        pos =
            RigidBodyPositionTrait::from_position(
                pos.predict_position_using_velocity(substep_dt, vels, mprops),
            );
        i += 1;
    }
    (forces, vels, pos)
}

/// The free-fall phase of `ball_drop`: the ball starts at `y = 2`, falls under `g = -9.81` and
/// is sampled at steps 1..10, 20 and 30, all before it reaches the ground at `y = 0.5`.
#[test]
fn test_ball_drop_free_fall_replay() {
    let scene = scenes::BALL_DROP;
    let dt = Fixed { raw: scene.dt };
    // The scene runs at the default parameters: four substeps of `dt / 4`, exactly.
    assert_eq!(integration_parameters::DT_Q32.dt, scene.dt);
    assert_eq!(integration_parameters::DT_Q32.num_solver_iterations, SUBSTEPS);
    assert_eq!(integration_parameters::DT_Q32.substep_dt * 4, scene.dt);
    let gravity = vector(scene.gravity);
    // Body 1 is the dynamic ball: one collider of radius 0.5 at unit density, at its origin.
    let bodies = scene.bodies.span();
    let ball = *bodies[1];
    let mut mprops = RigidBodyMassPropsTrait::from_local(
        local_mprops(mass_properties::BALL_R0_5_D1.expected), LockedAxesTrait::empty(),
    );
    let mut pos = RigidBodyPositionTrait::from_position(pose(ball.pose));
    mprops = mprops.update_world_mass_properties(RigidBodyType::Dynamic, pos.position);
    let mut vels = RigidBodyVelocityTrait::zero();
    let mut forces: RigidBodyForces = Default::default();
    assert_eq!(forces.gravity_scale, Fixed { raw: ball.gravity_scale });

    let mut step_index: u32 = 0;
    for sample in scene.samples.span() {
        let sample: SceneSampleRaw = *sample;
        if sample.step > LAST_FREE_FALL_STEP {
            break;
        }
        while step_index != sample.step {
            let (f, v, p) = step(dt, gravity, mprops, forces, vels, pos);
            forces = f;
            vels = v;
            pos = p;
            step_index += 1;
        }
        let states = sample.states.span();
        let expected: BodyStateRaw = *states[0];
        // `2^12 · step` ulp on the position, twice that on the velocity (golden README).
        let tolerance: u64 = 4096 * (sample.step.into() + 1);
        assert!(
            vec2_within(raw(pos.position.translation), expected.translation, tolerance),
            "translation at step {}",
            sample.step,
        );
        assert!(
            vec2_within(raw(vels.linvel), expected.linvel, tolerance * 2),
            "linvel at step {}",
            sample.step,
        );
        // The ball never spins and its rotation stays the exact identity.
        assert_eq!(vels.angvel, ZERO);
        assert_eq!(pos.position.rotation, Rot2 { re: ONE, im: ZERO });
        assert!(within(pos.position.rotation.re.raw, expected.rotation.re, 0));
        // Still above the ground: the replay has no contact solver.
        assert!(pos.position.translation.y > Fixed { raw: 2147483648 });
    }
    assert_eq!(step_index, LAST_FREE_FALL_STEP);
}

/// Upstream's own world-space mass properties for a body carrying two colliders.
#[test]
fn test_compound_body_world_mass_properties() {
    for case in mass_properties::body_cases() {
        let case = *case;
        let position = pose(case.body_pose);
        let props = RigidBodyMassPropsTrait::from_local(
            local_mprops(case.expected), LockedAxesTrait::empty(),
        )
            .update_world_mass_properties(RigidBodyType::Dynamic, position);
        assert!(vec2_within(raw(props.world_com), case.world_com, 16), "world com {}", case.id);
        assert!(
            vec2_within(raw(props.effective_inv_mass), case.effective_inv_mass, 0),
            "effective inverse mass {}",
            case.id,
        );
        assert!(
            within(props.effective_world_inv_inertia.raw, case.effective_world_inv_inertia, 0),
            "effective inverse inertia {}",
            case.id,
        );
        // The same body, fixed: every effective inverse vanishes, the centre of mass does not.
        let fixed = RigidBodyMassPropsTrait::from_local(
            local_mprops(case.expected), LockedAxesTrait::empty(),
        )
            .update_world_mass_properties(RigidBodyType::Fixed, position);
        assert_eq!(fixed.effective_inv_mass, Vec2Trait::ZERO);
        assert_eq!(fixed.effective_world_inv_inertia, ZERO);
        assert_eq!(fixed.world_com, props.world_com);
    }
}

/// The damping multipliers of `rapier_core` applied to a velocity, at the default step length.
#[test]
fn test_damping_matches_core_factors() {
    let dt = Fixed { raw: integration_parameters::DT_Q32.dt };
    let v = RigidBodyVelocityTrait::new(Vec2 { x: ONE, y: -ONE }, ONE);
    // `1 / (1 + dt * d)` for `d = 0`, `0.5` and `1`, rounded as `rapier_core` documents.
    for (coefficient, factor) in array![
        (0_i64, 4294967296_i64), (2147483648_i64, 4259471699_i64), (4294967296_i64, 4224557996_i64),
    ]
        .span() {
        let damping = RigidBodyDamping {
            linear_damping: Fixed { raw: *coefficient },
            angular_damping: Fixed { raw: *coefficient },
        };
        let damped = v.apply_damping(dt, damping);
        let expected = Fixed { raw: *factor };
        assert_eq!(damped.angvel, expected);
        assert_eq!(damped.linvel.x, expected);
        assert_eq!(damped.linvel.y, -expected);
    }
}

#[test]
fn gas_baseline() {}

/// One full step of a free body: the cost the pipeline pays per body and per step.
#[test]
fn gas_free_fall_step() {
    let scene = scenes::BALL_DROP;
    let mprops = opaque(
        RigidBodyMassPropsTrait::from_local(
            local_mprops(mass_properties::BALL_R0_5_D1.expected), LockedAxesTrait::empty(),
        )
            .update_world_mass_properties(RigidBodyType::Dynamic, Pose2Trait::IDENTITY),
    );
    let (_, vels, _) = step(
        opaque(Fixed { raw: scene.dt }),
        opaque(vector(scene.gravity)),
        mprops,
        opaque(Default::<RigidBodyForces>::default()),
        opaque(RigidBodyVelocityTrait::zero()),
        opaque(RigidBodyPositionTrait::from_position(Pose2Trait::IDENTITY)),
    );
    assert!(vels.linvel.y < ZERO);
}
