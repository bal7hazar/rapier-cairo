//! Full step candidates and separate four-substep stage probes (setup probes permit subtraction).
use fixed::ONE;
use rapier_testing::opaque;
use crate::joint::RevoluteJointBuilderTrait;
use super::*;
use super::fixtures::stack;
use super::super::body_store::DenseBodies;
use super::super::body_store::alternatives::ArrayBodies;

fn step_probe<B, +DenseBodiesTrait<B>, +Destruct<B>>(n: u32) {
    let (bs, steps, mut ms) = stack(opaque(n));
    let mut bodies: B = DenseBodiesTrait::new(bs.span());
    let mut js = array![];
    run(opaque(Default::default()), ref bodies, steps.span(), ref ms, ref js);
    let _ = opaque(bodies.get(0));
}
#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}
#[test]
fn gas_dict_stack1_step() {
    step_probe::<DenseBodies>(1);
}
#[test]
fn gas_array_stack1_step() {
    step_probe::<ArrayBodies>(1);
}
#[test]
fn gas_dict_stack2_step() {
    step_probe::<DenseBodies>(2);
}
#[test]
fn gas_array_stack2_step() {
    step_probe::<ArrayBodies>(2);
}
#[test]
fn gas_dict_stack3_step() {
    step_probe::<DenseBodies>(3);
}
#[test]
fn gas_array_stack3_step() {
    step_probe::<ArrayBodies>(3);
}
#[test]
fn gas_dict_stack5_step() {
    step_probe::<DenseBodies>(5);
}
#[test]
fn gas_array_stack5_step() {
    step_probe::<ArrayBodies>(5);
}
#[test]
fn gas_dict_stack8_step() {
    step_probe::<DenseBodies>(8);
}
#[test]
fn gas_array_stack8_step() {
    step_probe::<ArrayBodies>(8);
}
#[test]
fn gas_dict_stack16_step() {
    step_probe::<DenseBodies>(16);
}
#[test]
fn gas_array_stack16_step() {
    step_probe::<ArrayBodies>(16);
}

fn stage_probe(n: u32, stage: u8) {
    let (bs, steps, ms) = stack(opaque(n));
    let mut bodies: DenseBodies = DenseBodiesTrait::new(bs.span());
    let p: IntegrationParameters = opaque(Default::default());
    let mut cs = ContactConstraintsSetTrait::generate(ms.span(), bs.span(), p, p.substep_dt());
    let directions = super::super::contact::cached::prepare(cs.constraints.span());
    let mut sub = 0;
    while sub != p.num_solver_iterations {
        if stage == 1 {
            contacts(ref cs, ref bodies, ms.span(), p, 0, directions.span());
            contacts(ref cs, ref bodies, ms.span(), p, 1, directions.span());
            contacts(ref cs, ref bodies, ms.span(), p, 2, directions.span());
        }
        if stage == 2 {
            add_forces(ref bodies, steps.span());
            integrate(
                ref bodies,
                steps.span(),
                p.substep_dt(),
                p.max_linear_velocity(),
                Fixed { raw: 3373259426 } * p.inv_dt(),
            );
        }
        sub += 1;
    }
    let _ = opaque(bodies.get(0));
}
#[test]
fn gas_stage_setup3() {
    stage_probe(3, 0);
}
#[test]
fn gas_contact_sweeps3() {
    stage_probe(3, 1);
}
#[test]
fn gas_integration3() {
    stage_probe(3, 2);
}
#[test]
fn gas_stage_setup5() {
    stage_probe(5, 0);
}
#[test]
fn gas_contact_sweeps5() {
    stage_probe(5, 1);
}
#[test]
fn gas_integration5() {
    stage_probe(5, 2);
}
#[test]
fn gas_stage_setup1() {
    stage_probe(1, 0);
}
#[test]
fn gas_contact_sweeps1() {
    stage_probe(1, 1);
}
#[test]
fn gas_integration1() {
    stage_probe(1, 2);
}

fn joint_stage(solve: bool) {
    let (bs, steps, _) = stack(2);
    let pivot = SolverBody {
        im: Default::default(), ii: ZERO, position: Default::default(), ..*bs.at(0),
    };
    let bob = SolverBody {
        position: rapier_math::pose2::Pose2 {
            translation: super::fixtures::v(ONE, ZERO), ..Default::default(),
        },
        ..*bs.at(1),
    };
    let mut bodies: DenseBodies = DenseBodiesTrait::new([opaque(pivot), opaque(bob)].span());
    let joint = ImpulseJoint {
        body1: pivot.handle,
        body2: bob.handle,
        impulses: [ZERO, ZERO, ZERO],
        data: crate::joint::RevoluteJointBuilderTrait::new()
            .local_anchor2(super::fixtures::v(-ONE, ZERO))
            .build(),
    };
    let p: IntegrationParameters = opaque(Default::default());
    let builders = prepare_joints([opaque(joint)].span(), [pivot, bob].span(), steps.span());
    let mut rows = array![];
    let mut i = 0;
    while i != p.num_solver_iterations {
        if solve {
            rows = rebuild_joints(ref bodies, builders.span(), rows.span(), p, i != 0);
            joints(ref rows, ref bodies, true, false);
            joints(ref rows, ref bodies, false, false);
        }
        i += 1;
    }
    let _ = opaque(bodies.get(1));
}
#[test]
fn gas_joint_setup() {
    joint_stage(false);
}
#[test]
fn gas_joint_rebuild_sweeps() {
    joint_stage(true);
}
