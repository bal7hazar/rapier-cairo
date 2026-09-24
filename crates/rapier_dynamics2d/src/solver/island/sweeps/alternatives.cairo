//! Measured sweep alternatives. The original array adapter is the equivalence oracle.
use super::*;

pub(crate) fn contacts_original<B, +DenseBodiesTrait<B>, +Destruct<B>>(
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

pub(crate) fn contacts_metered<B, +DenseBodiesTrait<B>, +Destruct<B>>(
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
                    let mut pending = true;
                    while pending {
                        c.update(p, pair.span(), *ms.at(c.manifold_id));
                        c.warmstart(ref pair);
                        pending = false;
                    }
                },
                1 => {
                    let mut pending = true;
                    while pending {
                        c.solve(ref pair, true, p.friction_in_bias_pass);
                        pending = false;
                    }
                },
                2 => {
                    let mut pending = true;
                    while pending {
                        c.update_rhs_wo_bias(pair.span());
                        c.solve(ref pair, true, true);
                        pending = false;
                    }
                },
                3 => {
                    let mut pending = true;
                    while pending {
                        c.solve(ref pair, true, true);
                        pending = false;
                    }
                },
                _ => {
                    let mut pending = true;
                    while pending {
                        c.apply_restitution(ref pair);
                        pending = false;
                    }
                },
            }
            bodies.set_pair(i, *pair.at(0), j, *pair.at(1));
            c.solver_vel1 = i;
            c.solver_vel2 = j;
        }
        out.append(c);
    }
    cs.constraints = out;
}

pub(crate) fn joints_original<B, +DenseBodiesTrait<B>, +Destruct<B>>(
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

pub(crate) fn joints_skip_static<B, +DenseBodiesTrait<B>, +Destruct<B>>(
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
            let first = *pair.at(0);
            let second = *pair.at(1);
            let write_i = if first.im == Default::default() && first.ii == fixed::ZERO {
                WORLD
            } else {
                i
            };
            let write_j = if second.im == Default::default() && second.ii == fixed::ZERO {
                WORLD
            } else {
                j
            };
            bodies.set_pair(write_i, first, write_j, second);
            c.solver_vel1 = i;
            c.solver_vel2 = j;
        }
        out.append(c);
    }
    rows = out;
}

pub(crate) mod cached;

pub(crate) mod direct;

pub(crate) mod metered_pair;

pub(crate) mod sparse;

pub(crate) mod specialized;

pub(crate) mod stages;
