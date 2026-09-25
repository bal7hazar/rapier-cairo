//! RJ: full 120-step rope and spring replays, and paired setup/one-step gas probes.
use fixed::ZERO;
use rapier_dynamics2d::joint::{MotorModel, RopeJointBuilderTrait, SpringJointBuilderTrait};
use rapier_golden::types::CoupledJointSceneCase;
use rapier_testing::opaque;
use super::*;

fn world(case: CoupledJointSceneCase) -> World {
    let mut w = build_world(case.scene);
    let j = case.joint;
    let data = if j.rope {
        RopeJointBuilderTrait::new(f(j.max_dist))
            .local_anchor1(vr(j.local_anchor1))
            .local_anchor2(vr(j.local_anchor2))
            .build()
    } else {
        SpringJointBuilderTrait::new(f(j.rest_length), f(j.stiffness), f(j.damping))
            .local_anchor1(vr(j.local_anchor1))
            .local_anchor2(vr(j.local_anchor2))
            .spring_model(
                if j.force_based {
                    MotorModel::ForceBased
                } else {
                    MotorModel::AccelerationBased
                },
            )
            .build()
    };
    let _ = w.insert_impulse_joint(body_handle(j.body1), body_handle(j.body2), data);
    w
}
fn replay_coupled(case: CoupledJointSceneCase) {
    let mut w = world(case);
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
fn test_rope_pendulum() {
    replay_coupled(scenes::rope_pendulum::ROPE_PENDULUM);
}
#[test]
fn test_spring_mass() {
    replay_coupled(scenes::spring_mass::SPRING_MASS);
}
#[test]
fn test_spring_mass_accel() {
    replay_coupled(scenes::spring_mass_accel::SPRING_MASS_ACCEL);
}

/// `sample`: reseed from that sample (cold impulses) before the measured step, or not at all.
fn probe(case: CoupledJointSceneCase, sample: Option<u32>, step: bool) {
    let case = opaque(case);
    let mut w = world(case);
    if let Some(i) = sample {
        reseed(ref w, case.scene, *case.scene.samples.span().at(i));
    }
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
fn gas_setup_rope_slack() {
    probe(scenes::rope_pendulum::ROPE_PENDULUM, None, false);
}
#[test]
fn gas_step_rope_slack() {
    probe(scenes::rope_pendulum::ROPE_PENDULUM, None, true);
}
/// Sample 16 is step 60: the rope is taut and swinging.
#[test]
fn gas_setup_rope_taut() {
    probe(scenes::rope_pendulum::ROPE_PENDULUM, Some(16), false);
}
#[test]
fn gas_step_rope_taut() {
    probe(scenes::rope_pendulum::ROPE_PENDULUM, Some(16), true);
}
#[test]
fn gas_setup_spring() {
    probe(scenes::spring_mass::SPRING_MASS, None, false);
}
#[test]
fn gas_step_spring() {
    probe(scenes::spring_mass::SPRING_MASS, None, true);
}
#[test]
fn gas_setup_spring_accel() {
    probe(scenes::spring_mass_accel::SPRING_MASS_ACCEL, None, false);
}
#[test]
fn gas_step_spring_accel() {
    probe(scenes::spring_mass_accel::SPRING_MASS_ACCEL, None, true);
}
