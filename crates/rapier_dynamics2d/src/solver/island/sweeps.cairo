//! Two-body adapters around the unchanged DC/DE APIs; indices are restored after every call.
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_geometry2d::contact::ContactManifold;
use crate::joint::{ImpulseJoint, JointEnabled};
use super::super::body::{SolverBody, WORLD};
use super::super::body_store::{BodyStep, DenseBodiesTrait};
use super::super::contact::{ContactConstraintTrait, ContactConstraintsSet};
use super::super::joint::{JointConstraint, JointConstraintTrait};

pub(crate) fn contacts<B, +DenseBodiesTrait<B>, +Destruct<B>>(
    ref cs: ContactConstraintsSet,
    ref bodies: B,
    ms: Span<ContactManifold>,
    p: IntegrationParameters,
    stage: u8,
) {
    let mut out = array![];
    while let Some(mut c) = cs.constraints.pop_front() {
        if c.num_elements != 0 {
            let i = c.solver_vel1;
            let j = c.solver_vel2;
            let mut pair = array![bodies.get(i), bodies.get(j)];
            c.solver_vel1 = if i == WORLD {
                WORLD
            } else {
                0
            };
            c.solver_vel2 = if j == WORLD {
                WORLD
            } else {
                1
            };
            match stage {
                0 => {
                    c.update(p, pair.span(), *ms.at(c.manifold_id));
                    c.warmstart(ref pair);
                },
                1 => c.solve(ref pair, true, p.friction_in_bias_pass),
                2 => {
                    c.update_rhs_wo_bias(pair.span());
                    c.solve(ref pair, true, true);
                },
                3 => c.solve(ref pair, true, true),
                _ => c.apply_restitution(ref pair),
            }
            bodies.set_pair(i, *pair.at(0), j, *pair.at(1));
            c.solver_vel1 = i;
            c.solver_vel2 = j;
        }
        out.append(c);
    }
    cs.constraints = out;
}

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
pub(crate) fn joints<B, +DenseBodiesTrait<B>, +Destruct<B>>(
    ref rows: Array<JointConstraint>, ref bodies: B, biased: bool, warmstart: bool,
) {
    let mut out = array![];
    while let Some(mut c) = rows.pop_front() {
        if c.num_rows != 0 {
            let i = c.solver_vel1;
            let j = c.solver_vel2;
            let mut pair = array![bodies.get(i), bodies.get(j)];
            c.solver_vel1 = 0;
            c.solver_vel2 = 1;
            if warmstart {
                c.warmstart(ref pair);
            }
            c.solve(ref pair, biased);
            bodies.set_pair(i, *pair.at(0), j, *pair.at(1));
            c.solver_vel1 = i;
            c.solver_vel2 = j;
        }
        out.append(c);
    }
    rows = out;
}
pub(crate) fn write_joints(mut rows: Span<JointConstraint>, ref js: Array<ImpulseJoint>) {
    let mut out = array![];
    while let Some(mut j) = js.pop_front() {
        (*rows.pop_front().unwrap()).writeback_impulses(ref j);
        out.append(j);
    }
    js = out;
}
