//! JL: full 120-step controlled-joint replays, and paired setup/one-step gas probes.
use fixed::ZERO;
use rapier_dynamics2d::joint::{
    GenericJointTrait, MotorModel, PrismaticJointBuilderTrait, RevoluteJointBuilderTrait,
};
use rapier_golden::types::JointSceneCase;
use rapier_testing::opaque;
use super::*;

fn world(case: JointSceneCase, controls: bool) -> World {
    let mut w = build_world(case.scene);
    let j = case.joint;
    let mut data = if j.prismatic {
        PrismaticJointBuilderTrait::new(vr(j.axis))
            .local_anchor1(vr(j.local_anchor1))
            .local_anchor2(vr(j.local_anchor2))
            .build()
    } else {
        RevoluteJointBuilderTrait::new()
            .local_anchor1(vr(j.local_anchor1))
            .local_anchor2(vr(j.local_anchor2))
            .build()
    };
    let axis = if j.prismatic {
        0
    } else {
        2
    };
    if controls && j.has_limits {
        data.set_limits(axis, [f(j.min), f(j.max)]);
    }
    if controls && j.has_motor {
        data.set_motor(axis, f(j.target_pos), f(j.target_vel), f(j.stiffness), f(j.damping));
        data.set_motor_max_force(axis, f(j.max_force));
        data
            .set_motor_model(
                axis,
                if j.force_based {
                    MotorModel::ForceBased
                } else {
                    MotorModel::AccelerationBased
                },
            );
    }
    let _ = w.insert_impulse_joint(body_handle(j.body1), body_handle(j.body2), data);
    w
}
fn replay_joint(case: JointSceneCase) {
    let mut w = world(case, true);
    let mut step = 0;
    let mut stats = Default::default();
    for sample in case.scene.samples.span() {
        while step != *sample.step {
            w.step();
            step += 1;
            assert_fixed_bodies_still(ref w, case.scene, step);
        }
        stats = compare(ref w, case.scene, *sample, stats, true);
    }
    println!(
        "{}: max t {},{} r {},{} v {},{} w {}; violations {}",
        case.scene.id,
        stats.tx.ulps,
        stats.ty.ulps,
        stats.re.ulps,
        stats.im.ulps,
        stats.vx.ulps,
        stats.vy.ulps,
        stats.w.ulps,
        stats.violations,
    );
    assert_eq!(stats.violations, 0);
}
#[test]
fn test_pendulum_limited() {
    replay_joint(scenes::PENDULUM_LIMITED);
}
#[test]
fn test_wheel_motor() {
    replay_joint(scenes::WHEEL_MOTOR);
}
#[test]
fn test_slider_limited() {
    replay_joint(scenes::SLIDER_LIMITED);
}
#[test]
fn test_servo() {
    replay_joint(scenes::SERVO);
}

fn probe(case: JointSceneCase, controls: bool, active: bool, step: bool) {
    let case = opaque(case);
    let mut w = world(case, opaque(controls));
    if active {
        // Measured at a realistic stopped pose after 30 upstream steps, with cold impulses.
        reseed(ref w, case.scene, *case.scene.samples.span().at(12));
    }
    // Charge the step only when executed, including its outlined static Sierra cost.
    let mut pending = step;
    while pending {
        w.step();
        pending = false;
    }
    let b = w.body(body_handle(case.joint.body2)).unwrap();
    let _ = opaque((b.position(), b.vels));
}
#[test]
fn gas_baseline() {
    let _ = opaque(ZERO);
}
#[test]
fn gas_setup_plain() {
    probe(scenes::PENDULUM_LIMITED, false, false, false);
}
#[test]
fn gas_step_plain() {
    probe(scenes::PENDULUM_LIMITED, false, false, true);
}
#[test]
fn gas_setup_inactive_limit() {
    probe(scenes::PENDULUM_LIMITED, true, false, false);
}
#[test]
fn gas_step_inactive_limit() {
    probe(scenes::PENDULUM_LIMITED, true, false, true);
}
#[test]
fn gas_setup_active_limit() {
    probe(scenes::PENDULUM_LIMITED, true, true, false);
}
#[test]
fn gas_step_active_limit() {
    probe(scenes::PENDULUM_LIMITED, true, true, true);
}
#[test]
fn gas_setup_velocity_motor() {
    probe(scenes::WHEEL_MOTOR, true, false, false);
}
#[test]
fn gas_step_velocity_motor() {
    probe(scenes::WHEEL_MOTOR, true, false, true);
}
#[test]
fn gas_setup_position_motor() {
    probe(scenes::SERVO, true, false, false);
}
#[test]
fn gas_step_position_motor() {
    probe(scenes::SERVO, true, false, true);
}
#[test]
fn gas_setup_slider_limit() {
    probe(scenes::SLIDER_LIMITED, true, true, false);
}
#[test]
fn gas_step_slider_limit() {
    probe(scenes::SLIDER_LIMITED, true, true, true);
}
