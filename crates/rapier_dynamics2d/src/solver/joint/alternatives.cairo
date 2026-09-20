//! Upstream generic three-axis scan and loop-based modified Gram–Schmidt candidate.
use fixed::Fixed;
use rapier_core::integration_parameters::IntegrationParameters;
use crate::joint::{ImpulseJoint, JointAxesMaskTrait, JointEnabled};
use super::row::{finish, project};
use super::super::body::SolverBody;
use super::{
    JointConstraint, JointConstraintHelperTrait, JointGenericConstraint, prepare, push, seed,
};

pub fn generate(
    joint: ImpulseJoint, bodies: Span<SolverBody>, params: IntegrationParameters,
) -> JointConstraint {
    let (mut c, h, b1, b2, erp, cfm) = prepare(joint, bodies, params);
    if joint.data.enabled != JointEnabled::Enabled {
        return c;
    }
    // Same angular-first order as upstream, followed by ascending linear axes.
    if joint.data.locked_axes.contains_axis(2) {
        push(ref c, h.lock_angular(b1, b2, erp, cfm));
    }
    let mut axis = 0;
    while axis != 2 {
        if joint.data.locked_axes.contains_axis(axis) {
            push(ref c, h.lock_linear(axis, b1, b2, erp, cfm));
        }
        axis += 1;
    }
    finalize(ref c);
    seed(ref c, joint, params);
    c
}
fn get(c: JointConstraint, index: u8) -> JointGenericConstraint {
    let [a, b, d] = c.rows;
    match index {
        0 => a,
        1 => b,
        _ => d,
    }
}
fn set(ref c: JointConstraint, index: u8, row: JointGenericConstraint) {
    let [mut a, mut b, mut d] = c.rows;
    match index {
        0 => a = row,
        1 => b = row,
        _ => d = row,
    }
    c.rows = [a, b, d];
}
fn finalize(ref c: JointConstraint) {
    let imsum = c.im1 + c.im2;
    let mut j = 0;
    while j != c.num_rows {
        let mut a = get(c, j);
        let inv_mass: Fixed = finish(ref a, imsum);
        set(ref c, j, a);
        let mut i = j + 1;
        while i != c.num_rows {
            let mut b = get(c, i);
            project(ref b, a, imsum, inv_mass);
            set(ref c, i, b);
            i += 1;
        }
        j += 1;
    }
}
