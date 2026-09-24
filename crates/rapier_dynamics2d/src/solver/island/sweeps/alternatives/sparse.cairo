//! Stage-specialised direct two-body sweeps; preserve pair and row order.
use crate::solver::body::BodyPair;
use crate::solver::contact::cached::{self, Directions};
use crate::solver::contact::pair as pair_contact;
use super::*;

// Dispatch once, outside the manifold loop. Each monomorphised loop has one stage's cost.
pub(crate) fn contacts<B, +DenseBodiesTrait<B>, +Destruct<B>>(
    ref cs: ContactConstraintsSet,
    ref bodies: B,
    ms: Span<ContactManifold>,
    p: IntegrationParameters,
    stage: u8,
    directions: Span<Directions>,
) {
    match stage {
        0 => contacts_stage::<Update, B>(ref cs, ref bodies, ms, p, directions),
        1 => {
            if p.friction_in_bias_pass {
                contacts_stage::<Relax, B>(ref cs, ref bodies, ms, p, directions);
            } else {
                contacts_stage::<Biased, B>(ref cs, ref bodies, ms, p, directions);
            }
        },
        2 => contacts_stage::<Refresh, B>(ref cs, ref bodies, ms, p, directions),
        3 => contacts_stage::<Relax, B>(ref cs, ref bodies, ms, p, directions),
        _ => contacts_stage::<Restitution, B>(ref cs, ref bodies, ms, p, directions),
    }
}

trait PairStage {
    fn apply(
        ref c: ContactConstraint,
        ref pair: BodyPair,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
        directions: Directions,
    );
}
impl Update of PairStage {
    #[inline(always)]
    fn apply(
        ref c: ContactConstraint,
        ref pair: BodyPair,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
        directions: Directions,
    ) {
        let _ = ms.at(c.manifold_id);
        pair_contact::update(ref c, p, pair);
        cached::warmstart_sparse(c, directions, ref pair);
    }
}
impl Biased of PairStage {
    #[inline(always)]
    fn apply(
        ref c: ContactConstraint,
        ref pair: BodyPair,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
        directions: Directions,
    ) {
        cached::alternatives::solve_normal_only(ref c, directions, ref pair);
    }
}
impl Refresh of PairStage {
    #[inline(always)]
    fn apply(
        ref c: ContactConstraint,
        ref pair: BodyPair,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
        directions: Directions,
    ) {
        pair_contact::update_rhs_wo_bias(ref c, pair);
        cached::alternatives::solve_both(ref c, directions, ref pair);
    }
}
impl Relax of PairStage {
    #[inline(always)]
    fn apply(
        ref c: ContactConstraint,
        ref pair: BodyPair,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
        directions: Directions,
    ) {
        cached::alternatives::solve_both(ref c, directions, ref pair);
    }
}
impl Restitution of PairStage {
    #[inline(always)]
    fn apply(
        ref c: ContactConstraint,
        ref pair: BodyPair,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
        directions: Directions,
    ) {
        pair_contact::apply_restitution_metered(ref c, ref pair);
    }
}

fn contacts_stage<impl S: PairStage, B, +DenseBodiesTrait<B>, +Destruct<B>>(
    ref cs: ContactConstraintsSet,
    ref bodies: B,
    ms: Span<ContactManifold>,
    p: IntegrationParameters,
    directions: Span<Directions>,
) {
    let mut out = array![];
    while let Some(mut c) = cs.constraints.pop_front() {
        if c.num_elements != 0 {
            let i = c.solver_vel1;
            let j = c.solver_vel2;
            let mut pair = BodyPair { first: bodies.get(i), second: bodies.get(j) };
            S::apply(ref c, ref pair, ms, p, *directions.at(c.manifold_id));
            bodies.set_pair(i, pair.first, j, pair.second);
        }
        out.append(c);
    }
    cs.constraints = out;
}
