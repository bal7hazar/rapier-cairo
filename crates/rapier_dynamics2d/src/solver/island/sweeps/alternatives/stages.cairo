//! Stage-only array candidate.
use super::super::*;

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

trait ContactStage {
    fn apply(
        ref c: ContactConstraint,
        ref pair: Array<SolverBody>,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
    );
}
impl Update of ContactStage {
    #[inline(always)]
    fn apply(
        ref c: ContactConstraint,
        ref pair: Array<SolverBody>,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
    ) {
        c.update(p, pair.span(), *ms.at(c.manifold_id));
        c.warmstart(ref pair);
    }
}
impl Biased of ContactStage {
    #[inline(always)]
    fn apply(
        ref c: ContactConstraint,
        ref pair: Array<SolverBody>,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
    ) {
        c.solve(ref pair, true, p.friction_in_bias_pass);
    }
}
impl Refresh of ContactStage {
    #[inline(always)]
    fn apply(
        ref c: ContactConstraint,
        ref pair: Array<SolverBody>,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
    ) {
        c.update_rhs_wo_bias(pair.span());
        c.solve(ref pair, true, true);
    }
}
impl Relax of ContactStage {
    #[inline(always)]
    fn apply(
        ref c: ContactConstraint,
        ref pair: Array<SolverBody>,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
    ) {
        c.solve(ref pair, true, true);
    }
}
impl Restitution of ContactStage {
    #[inline(always)]
    fn apply(
        ref c: ContactConstraint,
        ref pair: Array<SolverBody>,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
    ) {
        c.apply_restitution(ref pair);
    }
}

fn contacts_stage<impl S: ContactStage, B, +DenseBodiesTrait<B>, +Destruct<B>>(
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
            S::apply(ref c, ref pair, ms, p);
            bodies.set_pair(i, *pair.at(0), j, *pair.at(1));
            c.solver_vel1 = i;
            c.solver_vel2 = j;
        }
        out.append(c);
    }
    cs.constraints = out;
}
