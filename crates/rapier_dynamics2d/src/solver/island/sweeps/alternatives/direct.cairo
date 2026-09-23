//! Stage-specialised direct two-body sweeps; preserve pair and row order.
use crate::solver::body::BodyPair;
use crate::solver::contact::pair::alternatives::raw as pair_contact;
use crate::solver::joint::alternatives::pair as pair_joint;
use super::*;

// Dispatch once, outside the manifold loop. Each monomorphised loop has one stage's cost.
pub(crate) fn contacts<B, +DenseBodiesTrait<B>, +Destruct<B>>(
    ref cs: ContactConstraintsSet,
    ref bodies: B,
    ms: Span<ContactManifold>,
    p: IntegrationParameters,
    stage: u8,
) {
    match stage {
        0 => contacts_stage::<Update, B>(ref cs, ref bodies, ms, p),
        1 => contacts_stage::<Biased, B>(ref cs, ref bodies, ms, p),
        2 => contacts_stage::<Refresh, B>(ref cs, ref bodies, ms, p),
        3 => contacts_stage::<Relax, B>(ref cs, ref bodies, ms, p),
        _ => contacts_stage::<Restitution, B>(ref cs, ref bodies, ms, p),
    }
}

trait PairStage {
    fn apply(
        ref c: ContactConstraint,
        ref pair: BodyPair,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
    );
}
impl Update of PairStage {
    #[inline(always)]
    fn apply(
        ref c: ContactConstraint,
        ref pair: BodyPair,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
    ) {
        let _ = ms.at(c.manifold_id);
        pair_contact::update(ref c, p, pair);
        pair_contact::warmstart(c, ref pair);
    }
}
impl Biased of PairStage {
    #[inline(always)]
    fn apply(
        ref c: ContactConstraint,
        ref pair: BodyPair,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
    ) {
        pair_contact::solve(ref c, ref pair, true, p.friction_in_bias_pass);
    }
}
impl Refresh of PairStage {
    #[inline(always)]
    fn apply(
        ref c: ContactConstraint,
        ref pair: BodyPair,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
    ) {
        pair_contact::update_rhs_wo_bias(ref c, pair);
        pair_contact::solve(ref c, ref pair, true, true);
    }
}
impl Relax of PairStage {
    #[inline(always)]
    fn apply(
        ref c: ContactConstraint,
        ref pair: BodyPair,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
    ) {
        pair_contact::solve(ref c, ref pair, true, true);
    }
}
impl Restitution of PairStage {
    #[inline(always)]
    fn apply(
        ref c: ContactConstraint,
        ref pair: BodyPair,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
    ) {
        pair_contact::apply_restitution(ref c, ref pair);
    }
}

fn contacts_stage<impl S: PairStage, B, +DenseBodiesTrait<B>, +Destruct<B>>(
    ref cs: ContactConstraintsSet,
    ref bodies: B,
    ms: Span<ContactManifold>,
    p: IntegrationParameters,
) {
    let mut out = array![];
    while let Some(mut c) = cs.constraints.pop_front() {
        if c.num_elements != 0 {
            let i = c.solver_vel1;
            let j = c.solver_vel2;
            let mut pair = BodyPair { first: bodies.get(i), second: bodies.get(j) };
            S::apply(ref c, ref pair, ms, p);
            bodies.set_pair(i, pair.first, j, pair.second);
        }
        out.append(c);
    }
    cs.constraints = out;
}

pub(crate) fn joints<B, +DenseBodiesTrait<B>, +Destruct<B>>(
    ref rows: Array<JointConstraint>, ref bodies: B, biased: bool, warmstart: bool,
) {
    let mut out = array![];
    while let Some(mut c) = rows.pop_front() {
        if c.num_rows != 0 {
            let i = c.solver_vel1;
            let j = c.solver_vel2;
            let mut pair = BodyPair { first: bodies.get(i), second: bodies.get(j) };
            if warmstart {
                pair_joint::warmstart(c, ref pair);
            }
            pair_joint::solve(ref c, ref pair, biased);
            bodies.set_pair(i, pair.first, j, pair.second);
        }
        out.append(c);
    }
    rows = out;
}

#[cfg(test)]
pub(crate) mod alternatives;
