//! Ordered joint preparation/writeback and contact/joint sweep modules. Joint kinds and step
//! constants are specialised once per step (JM, `joint::step`).
use fixed::Fixed;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_geometry2d::contact::ContactManifold;
use crate::joint::{ImpulseJoint, JointEnabled};
use super::super::body::{SolverBody, WORLD};
use super::super::body_store::{BodyStep, DenseBodiesTrait};
#[cfg(test)]
use super::super::contact::ContactConstraintTrait;
use super::super::contact::{ContactConstraint, ContactConstraintsSet};
use super::super::joint::step::{StepJoint, StepKind, specialise};
use super::super::joint::{JointConstraint, JointConstraintTrait, step, write_rows};

/// One joint of the step: kind and constants specialised once (JM), current lock impulses at
/// the start of the step, dense body indices (`WORLD` for a disabled joint).
#[derive(Copy, Drop, Debug)]
pub(crate) struct JointBuilder {
    joint: StepJoint,
    kind: StepKind,
    impulses: [Fixed; 3],
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
    if js.is_empty() {
        return out;
    }
    while let Some(joint) = js.pop_front() {
        let mut joint = *joint;
        let (i, j) = if joint.data.enabled == JointEnabled::Enabled {
            let i = resolve(bs, joint.body1);
            let j = resolve(bs, joint.body2);
            // DE frames are CoM-local, whereas persistent joint frames are body-local.
            joint.data.local_frame1.translation = joint.data.local_frame1.translation
                - *steps.at(i).local_com;
            joint.data.local_frame2.translation = joint.data.local_frame2.translation
                - *steps.at(j).local_com;
            (i, j)
        } else {
            (WORLD, WORLD)
        };
        let kind = if i == WORLD
            || (joint.data.limit_axes.bits == 0 && joint.data.motor_axes.bits == 0) {
            StepKind::Plain
        } else {
            specialise(joint)
        };
        let step = StepJoint {
            frame1: joint.data.local_frame1,
            frame2: joint.data.local_frame2,
            locks: joint.data.locked_axes,
            softness: joint.data.softness,
        };
        out.append(JointBuilder { joint: step, kind, impulses: joint.impulses, i, j });
    }
    out
}
/// Regenerate every joint for a substep in caller order. With `reuse`, warmstart seeds are the
/// impulses `old` would write back (JL wrote them into a joint copy, then read them again).
pub(crate) fn rebuild_joints<B, +DenseBodiesTrait<B>, +Destruct<B>>(
    ref bodies: B,
    mut builders: Span<JointBuilder>,
    old: Span<JointConstraint>,
    p: IntegrationParameters,
    reuse: bool,
) -> Array<JointConstraint> {
    let mut out = array![];
    // Joint-free islands skip the loop entry, whose Sierra charge follows the body's size.
    if builders.is_empty() {
        return out;
    }
    while let Some(builder) = builders.pop_front() {
        let JointBuilder { joint, kind, impulses, i, j } = *builder;
        let mut row = if i == WORLD {
            Default::default()
        } else {
            match kind {
                StepKind::Plain => {
                    let mut seeds = impulses;
                    if reuse {
                        let c = *old.at(out.len());
                        write_rows(c.rows, c.num_rows, ref seeds);
                    }
                    assert(i != j, super::super::joint::errors::SAME_BODY);
                    step::plain(joint, seeds, bodies.get(i), bodies.get(j), p)
                },
                StepKind::Controlled(controls) => {
                    let controls = controls.unbox();
                    let (seeds, motors, limits) = if reuse {
                        step::carried(*old.at(out.len()), impulses)
                    } else {
                        (impulses, controls.motors, controls.limits)
                    };
                    assert(i != j, super::super::joint::errors::SAME_BODY);
                    step::generate(
                        joint,
                        controls,
                        seeds,
                        motors,
                        limits,
                        bodies.get(i),
                        bodies.get(j),
                        p,
                        true,
                    )
                },
                StepKind::Legacy(legacy) => {
                    let previous = if reuse {
                        Some(*old.at(out.len()))
                    } else {
                        None
                    };
                    step::legacy(joint, legacy, impulses, previous, bodies.get(i), bodies.get(j), p)
                },
            }
        };
        row.solver_vel1 = i;
        row.solver_vel2 = j;
        out.append(row);
    }
    out
}
pub(crate) fn write_joints(mut rows: Span<JointConstraint>, ref js: Array<ImpulseJoint>) {
    if js.is_empty() {
        return;
    }
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
mod joint_rows;

#[cfg(test)]
mod zero_checks;
#[cfg(test)]
use alternatives::stages::contacts;
#[cfg(test)]
use alternatives::{cached, direct, metered_pair, sparse, specialized};

#[cfg(test)]
mod variants;

#[cfg(test)]
mod joint_profile {
    use rapier_testing::opaque;
    use super::*;
    use super::joint_rows::{JlBuilder, prepare_jl};
    use super::super::super::body_store::DenseBodies;
    use super::super::super::joint::alternatives::original;
    use super::super::super::joint::probes::chain;

    // A matched setup for each stage; differences include its loop and dictionary access.
    #[inline(always)]
    fn probe(kind: u8, n: u32, stage: u8, run: bool, old: bool) {
        let (bs, mut js, steps) = chain(opaque(n), opaque(kind));
        let p: IntegrationParameters = opaque(Default::default());
        let mut bodies: DenseBodies = DenseBodiesTrait::new(bs.span());
        if stage == 0 {
            if run {
                let _ = opaque(prepare_joints(js.span(), bs.span(), steps.span()).span());
            }
        } else {
            let builders = prepare_joints(js.span(), bs.span(), steps.span());
            let jl = prepare_jl(js.span(), bs.span(), steps.span());
            let builders = (builders.span(), jl.span());
            if stage == 1 {
                if run {
                    let mut rows = array![];
                    let mut i = 0;
                    while i != 4 {
                        rows = rebuild(ref bodies, builders, rows.span(), p, i != 0, old);
                        i += 1;
                    }
                    let _ = opaque(rows.span());
                }
            } else {
                let mut rows = rebuild(ref bodies, builders, [].span(), p, false, old);
                if run {
                    if stage == 2 {
                        for c in rows.span() {
                            let i = *c.solver_vel1;
                            let j = *c.solver_vel2;
                            let mut pair = array![bodies.get(i), bodies.get(j)];
                            let c = JointConstraint { solver_vel1: 0, solver_vel2: 1, ..*c };
                            if old {
                                original::warmstart(c, ref pair);
                            } else {
                                c.warmstart(ref pair);
                            }
                            bodies.set_pair(i, *pair.at(0), j, *pair.at(1));
                        }
                    } else if stage == 3 || stage == 4 {
                        if old {
                            sweep_old(ref rows, ref bodies, stage == 3);
                        } else {
                            array_joint::joints(ref rows, ref bodies, stage == 3, false);
                        }
                    } else {
                        if old {
                            let mut out = array![];
                            let mut cs = rows.span();
                            while let Some(mut j) = js.pop_front() {
                                original::writeback_impulses(*cs.pop_front().unwrap(), ref j);
                                out.append(j);
                            }
                            js = out;
                        } else {
                            write_joints(rows.span(), ref js);
                        }
                    }
                }
                let _ = opaque(rows.span());
            }
        }
        let _ = opaque((js.span(), bodies.get(n)));
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque(1_u32);
    }
    #[test]
    fn gas_revolute_prepare_setup() {
        probe(0, 3, 0, false, true);
    }
    #[test]
    fn gas_revolute_prepare() {
        probe(0, 3, 0, true, true);
    }
    #[test]
    fn gas_revolute_rebuild4_setup() {
        probe(0, 3, 1, false, true);
    }
    #[test]
    fn gas_revolute_rebuild4() {
        probe(0, 3, 1, true, true);
    }
    #[test]
    fn gas_revolute_warm_setup() {
        probe(0, 3, 2, false, true);
    }
    #[test]
    fn gas_revolute_warm() {
        probe(0, 3, 2, true, true);
    }
    #[test]
    fn gas_revolute_biased_setup() {
        probe(0, 3, 3, false, true);
    }
    #[test]
    fn gas_revolute_biased() {
        probe(0, 3, 3, true, true);
    }
    #[test]
    fn gas_revolute_relaxed_setup() {
        probe(0, 3, 4, false, true);
    }
    #[test]
    fn gas_revolute_relaxed() {
        probe(0, 3, 4, true, true);
    }
    #[test]
    fn gas_revolute_write_setup() {
        probe(0, 3, 5, false, true);
    }
    #[test]
    fn gas_revolute_write() {
        probe(0, 3, 5, true, true);
    }
    #[test]
    fn gas_prismatic_prepare_setup() {
        probe(1, 3, 0, false, true);
    }
    #[test]
    fn gas_prismatic_prepare() {
        probe(1, 3, 0, true, true);
    }
    #[test]
    fn gas_prismatic_rebuild4_setup() {
        probe(1, 3, 1, false, true);
    }
    #[test]
    fn gas_prismatic_rebuild4() {
        probe(1, 3, 1, true, true);
    }
    #[test]
    fn gas_prismatic_warm_setup() {
        probe(1, 3, 2, false, true);
    }
    #[test]
    fn gas_prismatic_warm() {
        probe(1, 3, 2, true, true);
    }
    #[test]
    fn gas_prismatic_biased_setup() {
        probe(1, 3, 3, false, true);
    }
    #[test]
    fn gas_prismatic_biased() {
        probe(1, 3, 3, true, true);
    }
    #[test]
    fn gas_prismatic_relaxed_setup() {
        probe(1, 3, 4, false, true);
    }
    #[test]
    fn gas_prismatic_relaxed() {
        probe(1, 3, 4, true, true);
    }
    #[test]
    fn gas_prismatic_write_setup() {
        probe(1, 3, 5, false, true);
    }
    #[test]
    fn gas_prismatic_write() {
        probe(1, 3, 5, true, true);
    }
    #[test]
    fn gas_fixed_prepare_setup() {
        probe(2, 3, 0, false, true);
    }
    #[test]
    fn gas_fixed_prepare() {
        probe(2, 3, 0, true, true);
    }
    #[test]
    fn gas_fixed_rebuild4_setup() {
        probe(2, 3, 1, false, true);
    }
    #[test]
    fn gas_fixed_rebuild4() {
        probe(2, 3, 1, true, true);
    }
    #[test]
    fn gas_fixed_warm_setup() {
        probe(2, 3, 2, false, true);
    }
    #[test]
    fn gas_fixed_warm() {
        probe(2, 3, 2, true, true);
    }
    #[test]
    fn gas_fixed_biased_setup() {
        probe(2, 3, 3, false, true);
    }
    #[test]
    fn gas_fixed_biased() {
        probe(2, 3, 3, true, true);
    }
    #[test]
    fn gas_fixed_relaxed_setup() {
        probe(2, 3, 4, false, true);
    }
    #[test]
    fn gas_fixed_relaxed() {
        probe(2, 3, 4, true, true);
    }
    #[test]
    fn gas_fixed_write_setup() {
        probe(2, 3, 5, false, true);
    }
    #[test]
    fn gas_fixed_write() {
        probe(2, 3, 5, true, true);
    }

    #[inline(always)]
    fn rebuild(
        ref bodies: DenseBodies,
        builders: (Span<JointBuilder>, Span<JlBuilder>),
        rows: Span<JointConstraint>,
        p: IntegrationParameters,
        reuse: bool,
        old: bool,
    ) -> Array<JointConstraint> {
        let (builders, mut jl) = builders;
        if !old {
            return rebuild_joints(ref bodies, builders, rows, p, reuse);
        }
        let mut out = array![];
        while let Some(builder) = jl.pop_front() {
            let JlBuilder { mut joint, i, j } = *builder;
            if reuse {
                original::writeback_impulses(*rows.at(out.len()), ref joint);
            }
            let pair = [bodies.get(i), bodies.get(j)];
            let mut c = original::generate(joint, pair.span(), p);
            c.solver_vel1 = i;
            c.solver_vel2 = j;
            out.append(c);
        }
        out
    }
    fn sweep_old(ref rows: Array<JointConstraint>, ref bodies: DenseBodies, biased: bool) {
        let mut out = array![];
        while let Some(mut c) = rows.pop_front() {
            if c.num_rows != 0 {
                let i = c.solver_vel1;
                let j = c.solver_vel2;
                let mut pair = array![bodies.get(i), bodies.get(j)];
                c.solver_vel1 = 0;
                c.solver_vel2 = 1;
                original::solve(ref c, ref pair, biased);
                bodies.set_pair(i, *pair.at(0), j, *pair.at(1));
                c.solver_vel1 = i;
                c.solver_vel2 = j;
            }
            out.append(c);
        }
        rows = out;
    }
    #[test]
    fn gas_revolute_prepare_selected_setup() {
        probe(0, 3, 0, false, false);
    }
    #[test]
    fn gas_revolute_prepare_selected() {
        probe(0, 3, 0, true, false);
    }
    #[test]
    fn gas_revolute_rebuild4_selected_setup() {
        probe(0, 3, 1, false, false);
    }
    #[test]
    fn gas_revolute_rebuild4_selected() {
        probe(0, 3, 1, true, false);
    }
    #[test]
    fn gas_revolute_warm_selected_setup() {
        probe(0, 3, 2, false, false);
    }
    #[test]
    fn gas_revolute_warm_selected() {
        probe(0, 3, 2, true, false);
    }
    #[test]
    fn gas_revolute_biased_selected_setup() {
        probe(0, 3, 3, false, false);
    }
    #[test]
    fn gas_revolute_biased_selected() {
        probe(0, 3, 3, true, false);
    }
    #[test]
    fn gas_revolute_relaxed_selected_setup() {
        probe(0, 3, 4, false, false);
    }
    #[test]
    fn gas_revolute_relaxed_selected() {
        probe(0, 3, 4, true, false);
    }
    #[test]
    fn gas_revolute_write_selected_setup() {
        probe(0, 3, 5, false, false);
    }
    #[test]
    fn gas_revolute_write_selected() {
        probe(0, 3, 5, true, false);
    }
    #[test]
    fn gas_prismatic_prepare_selected_setup() {
        probe(1, 3, 0, false, false);
    }
    #[test]
    fn gas_prismatic_prepare_selected() {
        probe(1, 3, 0, true, false);
    }
    #[test]
    fn gas_prismatic_rebuild4_selected_setup() {
        probe(1, 3, 1, false, false);
    }
    #[test]
    fn gas_prismatic_rebuild4_selected() {
        probe(1, 3, 1, true, false);
    }
    #[test]
    fn gas_prismatic_warm_selected_setup() {
        probe(1, 3, 2, false, false);
    }
    #[test]
    fn gas_prismatic_warm_selected() {
        probe(1, 3, 2, true, false);
    }
    #[test]
    fn gas_prismatic_biased_selected_setup() {
        probe(1, 3, 3, false, false);
    }
    #[test]
    fn gas_prismatic_biased_selected() {
        probe(1, 3, 3, true, false);
    }
    #[test]
    fn gas_prismatic_relaxed_selected_setup() {
        probe(1, 3, 4, false, false);
    }
    #[test]
    fn gas_prismatic_relaxed_selected() {
        probe(1, 3, 4, true, false);
    }
    #[test]
    fn gas_prismatic_write_selected_setup() {
        probe(1, 3, 5, false, false);
    }
    #[test]
    fn gas_prismatic_write_selected() {
        probe(1, 3, 5, true, false);
    }
    #[test]
    fn gas_fixed_prepare_selected_setup() {
        probe(2, 3, 0, false, false);
    }
    #[test]
    fn gas_fixed_prepare_selected() {
        probe(2, 3, 0, true, false);
    }
    #[test]
    fn gas_fixed_rebuild4_selected_setup() {
        probe(2, 3, 1, false, false);
    }
    #[test]
    fn gas_fixed_rebuild4_selected() {
        probe(2, 3, 1, true, false);
    }
    #[test]
    fn gas_fixed_warm_selected_setup() {
        probe(2, 3, 2, false, false);
    }
    #[test]
    fn gas_fixed_warm_selected() {
        probe(2, 3, 2, true, false);
    }
    #[test]
    fn gas_fixed_biased_selected_setup() {
        probe(2, 3, 3, false, false);
    }
    #[test]
    fn gas_fixed_biased_selected() {
        probe(2, 3, 3, true, false);
    }
    #[test]
    fn gas_fixed_relaxed_selected_setup() {
        probe(2, 3, 4, false, false);
    }
    #[test]
    fn gas_fixed_relaxed_selected() {
        probe(2, 3, 4, true, false);
    }
    #[test]
    fn gas_fixed_write_selected_setup() {
        probe(2, 3, 5, false, false);
    }
    #[test]
    fn gas_fixed_write_selected() {
        probe(2, 3, 5, true, false);
    }
}
