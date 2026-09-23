//! One joint with one bilateral row, warmstart/biased then relaxed sweep.
use fixed::{HALF, ONE, ZERO};
use rapier_testing::opaque;
use crate::joint::{GenericJoint, JointAxesMask};
use super::*;
use super::super::super::body_store::DenseBodies;

trait Sweep {
    fn run(ref rows: Array<JointConstraint>, ref bodies: DenseBodies, biased: bool, warm: bool);
}
impl Original of Sweep {
    #[inline(always)]
    fn run(ref rows: Array<JointConstraint>, ref bodies: DenseBodies, biased: bool, warm: bool) {
        alternatives::joints_original(ref rows, ref bodies, biased, warm);
    }
}
impl Separate of Sweep {
    #[inline(always)]
    fn run(ref rows: Array<JointConstraint>, ref bodies: DenseBodies, biased: bool, warm: bool) {
        array_joint::joints(ref rows, ref bodies, biased, warm);
    }
}
impl SkipStatic of Sweep {
    #[inline(always)]
    fn run(ref rows: Array<JointConstraint>, ref bodies: DenseBodies, biased: bool, warm: bool) {
        alternatives::joints_skip_static(ref rows, ref bodies, biased, warm);
    }
}
impl Direct of Sweep {
    #[inline(always)]
    fn run(ref rows: Array<JointConstraint>, ref bodies: DenseBodies, biased: bool, warm: bool) {
        direct::joints(ref rows, ref bodies, biased, warm);
    }
}
impl Specialized of Sweep {
    #[inline(always)]
    fn run(ref rows: Array<JointConstraint>, ref bodies: DenseBodies, biased: bool, warm: bool) {
        specialized::joints(ref rows, ref bodies, biased, warm);
    }
}
impl MeteredPair of Sweep {
    #[inline(always)]
    fn run(ref rows: Array<JointConstraint>, ref bodies: DenseBodies, biased: bool, warm: bool) {
        metered_pair::joints(ref rows, ref bodies, biased, warm);
    }
}
fn input(fixed: bool) -> (Array<JointConstraint>, DenseBodies) {
    let (bs, _, _) = super::super::fixtures::stack(2);
    let first = if fixed {
        SolverBody { im: Default::default(), ii: ZERO, ..*bs.at(0) }
    } else {
        *bs.at(0)
    };
    let bs = array![first, *bs.at(1)];
    let joint = ImpulseJoint {
        body1: *bs.at(0).handle,
        body2: *bs.at(1).handle,
        data: GenericJoint { locked_axes: JointAxesMask { bits: 1 }, ..Default::default() },
        impulses: [HALF, ZERO, ZERO],
    };
    let p = IntegrationParameters { warmstart_joints: true, ..Default::default() };
    let c = JointConstraintTrait::generate(opaque(joint), bs.span(), opaque(p));
    let bodies: DenseBodies = DenseBodiesTrait::new(opaque(bs.span()));
    (array![opaque(c)], bodies)
}
fn consume(rows: Array<JointConstraint>, ref bodies: DenseBodies) {
    let _ = opaque((*rows.at(0), bodies.get(0), bodies.get(1), ONE));
}
fn probe<impl S: Sweep>(solve: bool) {
    let (mut rows, mut bodies) = input(false);
    if solve {
        S::run(ref rows, ref bodies, true, true);
        S::run(ref rows, ref bodies, false, false);
    }
    consume(rows, ref bodies);
}
#[test]
fn gas_baseline() {
    probe::<Original>(false);
}
#[test]
fn gas_original_row() {
    probe::<Original>(true);
}
#[test]
fn gas_direct_row() {
    probe::<Direct>(true);
}
#[test]
fn gas_dict_row() {
    let (rows, mut bodies) = input(false);
    let mut rows = direct::alternatives::new_joints(rows.span());
    direct::alternatives::joints(ref rows, ref bodies, true, true);
    direct::alternatives::joints(ref rows, ref bodies, false, false);
    let rows = direct::alternatives::finish_joints(ref rows);
    consume(rows, ref bodies);
}

#[test]
fn gas_metered_pair_row() {
    probe::<MeteredPair>(true);
}

#[test]
fn gas_specialized_row() {
    probe::<Specialized>(true);
}

fn static_probe<impl S: Sweep>(solve: bool) {
    let (mut rows, mut bodies) = input(true);
    if solve {
        S::run(ref rows, ref bodies, true, true);
        S::run(ref rows, ref bodies, false, false);
    }
    consume(rows, ref bodies);
}
#[test]
fn gas_static_setup() {
    static_probe::<Original>(false);
}
#[test]
fn gas_static_original_row() {
    static_probe::<Original>(true);
}
#[test]
fn gas_static_skip_row() {
    static_probe::<SkipStatic>(true);
}
#[test]
fn gas_dynamic_skip_row() {
    probe::<SkipStatic>(true);
}

#[test]
fn gas_separate_row() {
    probe::<Separate>(true);
}
