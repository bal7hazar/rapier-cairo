//! Mock substep driver: DA pose integration and DE joint rebuild/solve against golden scenes.
use fixed::{Fixed, FixedTrait, ONE, ZERO};
use glam::Vec2;
use rapier_core::data::handle::Handle;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_dynamics2d::joint::{
    FixedJointBuilderTrait, GenericJoint, ImpulseJoint, PrismaticJointBuilderTrait,
    RevoluteJointBuilderTrait,
};
use rapier_dynamics2d::rigid_body::{RigidBodyVelocity, RigidBodyVelocityTrait};
use rapier_dynamics2d::solver::body::SolverBody;
use rapier_dynamics2d::solver::joint::JointConstraintTrait;
use rapier_geometry2d::mass::MassPropertiesTrait;
use rapier_golden::compare::within;
use rapier_golden::scenes;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_testing::opaque;

const GRAVITY: Fixed = Fixed { raw: -42133629174 };
const TOL: Fixed = Fixed { raw: 4294967 };
fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}
fn h(index: u32) -> Handle {
    Handle { index, generation: 0 }
}
fn joint(data: GenericJoint) -> ImpulseJoint {
    ImpulseJoint { body1: h(0), body2: h(1), data, impulses: [ZERO, ZERO, ZERO] }
}
fn body(index: u32, position: Vec2) -> SolverBody {
    SolverBody {
        handle: h(index),
        position: Pose2 { translation: position, ..Default::default() },
        im: v(ONE, ONE),
        ii: ONE,
        ..Default::default(),
    }
}
fn step(ref bs: Array<SolverBody>, ref j: ImpulseJoint, p: IntegrationParameters, gravity: Fixed) {
    let dt = p.substep_dt();
    let mut sub = 0;
    while sub != p.num_solver_iterations {
        let mut out = array![];
        while let Some(mut b) = bs.pop_front() {
            if b.im.y != ZERO {
                b.linvel.y += gravity * dt;
            }
            out.append(b);
        }
        bs = out;
        let mut c = JointConstraintTrait::generate(j, bs.span(), p);
        c.warmstart(ref bs);
        let mut i = 0;
        while i != p.num_internal_pgs_iterations {
            c.solve(ref bs, true);
            i += 1;
        }
        let mut out = array![];
        while let Some(mut b) = bs.pop_front() {
            let vel = RigidBodyVelocity { linvel: b.linvel, angvel: b.angvel };
            b.position = vel.integrate(dt, b.position, Default::default());
            out.append(b);
        }
        bs = out;
        c.remove_bias();
        let mut i = 0;
        while i != p.num_internal_stabilization_iterations {
            c.solve(ref bs, false);
            i += 1;
        }
        c.writeback_impulses(ref j);
        sub += 1;
    }
}
fn pendulum() -> (Array<SolverBody>, ImpulseJoint, IntegrationParameters) {
    let m = MassPropertiesTrait::from_ball(ONE, Fixed { raw: 1073741824 });
    let pivot = SolverBody { handle: h(0), ..Default::default() };
    let bob = SolverBody {
        im: v(m.inv_mass, m.inv_mass), ii: m.inv_principal_inertia, ..body(1, v(ONE, ZERO)),
    };
    let data = RevoluteJointBuilderTrait::new().local_anchor2(v(-ONE, ZERO)).build();
    (
        array![pivot, bob],
        joint(data),
        IntegrationParameters { dt: Fixed { raw: scenes::PENDULUM.dt }, ..Default::default() },
    )
}
#[test]
fn test_pendulum_all_twenty_two_golden_samples() {
    let scene = scenes::PENDULUM;
    let (mut bs, mut j, p) = pendulum();
    let mut frame = 0;
    let mut samples = scene.samples.span();
    let mut checked = 0;
    while let Some(sample) = samples.pop_front() {
        while frame != *sample.step {
            step(ref bs, ref j, p, GRAVITY);
            frame += 1;
        }
        let [expected, _, _] = *sample.states;
        let b = *bs.at(1);
        let ptol: u64 = 4096 * frame.into();
        let vtol = ptol * 2;
        assert!(
            within(b.position.translation.x.raw, expected.translation.x, ptol),
            "position x step {}",
            frame,
        );
        assert!(
            within(b.position.translation.y.raw, expected.translation.y, ptol),
            "position y step {}",
            frame,
        );
        assert!(
            within(b.position.rotation.re.raw, expected.rotation.re, ptol),
            "rotation re step {}",
            frame,
        );
        assert!(
            within(b.position.rotation.im.raw, expected.rotation.im, ptol),
            "rotation im step {}",
            frame,
        );
        assert!(within(b.linvel.x.raw, expected.linvel.x, vtol), "velocity x step {}", frame);
        assert!(within(b.linvel.y.raw, expected.linvel.y, vtol), "velocity y step {}", frame);
        assert!(within(b.angvel.raw, expected.angvel, vtol), "angular velocity step {}", frame);
        checked += 1;
    }
    assert_eq!(checked, 22);
    assert_eq!(frame, 120);
}
#[test]
fn test_fixed_and_prismatic_under_gravity() {
    for fixed in [true, false].span() {
        let a = SolverBody { handle: h(0), ..Default::default() };
        let b = body(1, v(ONE, ZERO));
        // Vertical prismatic axis leaves gravity free while locking its perpendicular and angle.
        let data = if *fixed {
            FixedJointBuilderTrait::new().local_anchor1(v(ONE, ZERO)).build()
        } else {
            PrismaticJointBuilderTrait::new(v(ZERO, ONE)).local_anchor1(v(ONE, ZERO)).build()
        };
        let mut j = joint(data);
        let mut bs = array![a, b];
        let p = Default::default();
        let mut i = 0;
        while i != 60 {
            step(ref bs, ref j, p, GRAVITY);
            i += 1;
        }
        let b = *bs.at(1);
        assert!((b.position.translation.x - ONE).abs() < TOL);
        assert!(b.angvel.abs() < TOL);
        assert!(b.position.rotation.im.abs() < TOL);
        if *fixed {
            assert!(b.position.translation.y.abs() < TOL);
            assert!(b.linvel.y.abs() < TOL);
        } else {
            assert!(b.position.translation.y < -ONE);
            assert!(b.linvel.y < -ONE);
        }
    }
}
#[test]
fn test_revolute_anchor_and_dynamic_fixed_pair() {
    let (mut bs, mut j, p) = pendulum();
    let mut i = 0;
    while i != 120 {
        step(ref bs, ref j, p, GRAVITY);
        let anchor = bs.at(1).position.transform_point(v(-ONE, ZERO));
        assert!(anchor.x.abs() < TOL && anchor.y.abs() < TOL);
        i += 1;
    }
    let data = FixedJointBuilderTrait::new().local_anchor1(v(ONE, ZERO)).build();
    let mut j = joint(data);
    let mut bs = array![body(0, v(ZERO, ZERO)), body(1, v(ONE, ZERO))];
    let mut i = 0;
    while i != 60 {
        step(ref bs, ref j, p, GRAVITY);
        i += 1;
    }
    let a = *bs.at(0);
    let b = *bs.at(1);
    assert!((b.position.translation.x - a.position.translation.x - ONE).abs() < TOL);
    assert!((b.position.translation.y - a.position.translation.y).abs() < TOL);
    assert!(a.position.translation.y < -ONE);
}
#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}
#[test]
fn gas_pendulum_step() {
    let (mut bs, mut j, p) = pendulum();
    j = opaque(j);
    step(ref bs, ref j, opaque(p), opaque(GRAVITY));
    let _ = opaque(*bs.at(1));
}
