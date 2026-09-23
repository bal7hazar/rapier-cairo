//! Ordered joint preparation/writeback and contact/joint sweep modules.
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_geometry2d::contact::ContactManifold;
use crate::joint::{ImpulseJoint, JointEnabled};
use super::super::body::{SolverBody, WORLD};
use super::super::body_store::{BodyStep, DenseBodiesTrait};
#[cfg(test)]
use super::super::contact::ContactConstraintTrait;
use super::super::contact::{ContactConstraint, ContactConstraintsSet};
use super::super::joint::{JointConstraint, JointConstraintTrait};

#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub(crate) struct JointBuilder {
    joint: ImpulseJoint,
    i: u32,
    j: u32,
}
fn resolve(mut bs: Span<SolverBody>, h: rapier_core::Handle) -> u32 {
    let mut i = 0;
    while let Some(b) = bs.pop_front() {
        if *b.handle == h {
            return i;
        }
        i += 1;
    }
    core::panic_with_felt252(super::super::joint::errors::BODY)
}
pub(crate) fn prepare_joints(
    mut js: Span<ImpulseJoint>, bs: Span<SolverBody>, steps: Span<BodyStep>,
) -> Array<JointBuilder> {
    let mut out = array![];
    while let Some(joint) = js.pop_front() {
        let mut joint = *joint;
        if joint.data.enabled == JointEnabled::Enabled {
            let i = resolve(bs, joint.body1);
            let j = resolve(bs, joint.body2);
            // DE frames are CoM-local, whereas persistent joint frames are body-local.
            joint.data.local_frame1.translation = joint.data.local_frame1.translation
                - *steps.at(i).local_com;
            joint.data.local_frame2.translation = joint.data.local_frame2.translation
                - *steps.at(j).local_com;
            out.append(JointBuilder { joint, i, j });
        } else {
            out.append(JointBuilder { joint, i: WORLD, j: WORLD });
        }
    }
    out
}
pub(crate) fn rebuild_joints<B, +DenseBodiesTrait<B>, +Destruct<B>>(
    ref bodies: B,
    mut builders: Span<JointBuilder>,
    old: Span<JointConstraint>,
    p: IntegrationParameters,
    reuse: bool,
) -> Array<JointConstraint> {
    let mut out = array![];
    while let Some(builder) = builders.pop_front() {
        let JointBuilder { mut joint, i, j } = *builder;
        if reuse {
            (*old.at(out.len())).writeback_impulses(ref joint);
        }
        let pair = [bodies.get(i), bodies.get(j)];
        let mut row = JointConstraintTrait::generate(joint, pair.span(), p);
        row.solver_vel1 = i;
        row.solver_vel2 = j;
        out.append(row);
    }
    out
}
pub(crate) fn write_joints(mut rows: Span<JointConstraint>, ref js: Array<ImpulseJoint>) {
    let mut out = array![];
    while let Some(mut j) = js.pop_front() {
        (*rows.pop_front().unwrap()).writeback_impulses(ref j);
        out.append(j);
    }
    js = out;
}

#[cfg(test)]
mod alternatives;
#[cfg(test)]
mod benches;


#[cfg(test)]
mod checks;


pub(crate) mod contact;
#[cfg(test)]
mod joint_benches;
#[cfg(test)]
use contact as zero;
pub(crate) mod array_joint;

#[cfg(test)]
mod joint_checks;

#[cfg(test)]
mod zero_checks;
#[cfg(test)]
use alternatives::stages::contacts;
#[cfg(test)]
use alternatives::{cached, direct, metered_pair, sparse, specialized};

#[cfg(test)]
mod variants;
