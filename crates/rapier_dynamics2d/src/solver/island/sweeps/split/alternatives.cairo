//! Rejected BT1 candidates, kept for re-ranking (measured on the level-10 impact tick 28 solver
//! input, `rapier2d` `level_budget::solver10_*`, exact Cairo steps of `solve_island`):
//!
//! * every kernel's computing branch behind a one-iteration `while pending` loop (the metered
//!   form of `cached::zero`, `MeteredBiased` below): 363,781 steps against 321,135 without the
//!   loops (the loop is a call carrying the hot values and both velocities); the P3 contact
//!   scenes' Sierra gas also falls without it (−27 % to −42 % gross, `gas_scenes`);
//! * a two-slot cache of the last constraint's velocities in `SweepBodies`, flushed after each
//!   sweep (fewer dictionary accesses when consecutive constraints share a body): 324,819 steps
//!   against 315,043 (the larger loop state and the comparisons cost more than the saved
//!   accesses);
//! * the kernels gathering and scattering their two bodies unconditionally in the sweep loop:
//!   370,598 → 363,781 once the scatter moved after the metered loop (both metered).
use super::super::super::super::body::SolverBody;
use super::super::super::super::contact::element::tangent;
use super::super::super::super::contact::{
    ContactConstraint, ContactConstraintElement, SoftCacheTrait, generate_cached,
};
use super::*;

#[derive(Copy, Drop)]
pub(crate) struct MeteredBiased {}

impl MeteredBiasedKernel of Kernel<MeteredBiased> {
    #[inline(always)]
    fn apply(
        self: MeteredBiased, ref h: Hot, f: @Frozen, ref bodies: SweepBodies, poses: Span<Pose2>,
    ) {
        let mut v1 = bodies.vel(*f.i);
        let mut v2 = bodies.vel(*f.j);
        if idle(h, *f.count, v1, v2) {
            return;
        }
        let mut pending = true;
        while pending {
            let dir = *f.dir;
            solve_normal(ref h.a, dir, f.a.n, f.wn, ref v1, ref v2);
            if *f.count == 2 {
                solve_normal(ref h.b, dir, f.b.n, f.wn, ref v1, ref v2);
            }
            pending = false;
        }
        bodies.set_vels(*f.i, v1, *f.j, v2);
    }
}

/// The biased sweep with the metered kernel; same results as `contacts(.., 1)`.
pub(crate) fn biased_metered(ref hot: Array<Hot>, frozen: Span<Frozen>, ref bodies: SweepBodies) {
    sweep(ref hot, frozen, ref bodies, MeteredBiased {});
}

#[inline(always)]
fn frozen_point(e: ContactConstraintElement) -> FrozenPoint {
    let n = e.normal_part;
    let t = e.tangent_part;
    FrozenPoint {
        n: Row { g1: n.gcross1, g2: n.gcross2, ig1: n.ii_gcross1, ig2: n.ii_gcross2, r: n.r },
        t: Row { g1: t.gcross1, g2: t.gcross2, ig1: t.ii_gcross1, ig2: t.ii_gcross2, r: t.r },
        local_p1: e.local_p1,
        local_p2: e.local_p2,
        dist: e.dist,
        t_rhs_wo_bias: t.rhs_wo_bias,
        seed: e.restitution_seed,
        contact_id: e.contact_id,
    }
}
#[inline(always)]
fn hot_point(e: ContactConstraintElement) -> HotPoint {
    HotPoint {
        impulse: e.normal_part.impulse,
        rhs: e.normal_part.rhs,
        cfm: e.normal_part.cfm_factor,
        t_impulse: e.tangent_part.impulse,
        t_rhs: e.tangent_part.rhs,
        acc: e.normal_part.impulse_accumulator,
        t_acc: e.tangent_part.impulse_accumulator,
    }
}

/// Append the split of `c` when it is active; caches `dir * im` for both rows, as
/// `cached::prepare` does (products floor).
#[inline(always)]
fn push(ref frozen: Array<Frozen>, ref hot: Array<Hot>, c: ContactConstraint) {
    if c.num_elements != 0 {
        let t = tangent(c.dir1);
        let [a, b] = c.elements;
        frozen
            .append(
                Frozen {
                    i: c.solver_vel1,
                    j: c.solver_vel2,
                    dir: c.dir1,
                    t,
                    wn: Weights { first: c.dir1 * c.im1, neg_second: -(c.dir1 * c.im2) },
                    wt: Weights { first: t * c.im1, neg_second: -(t * c.im2) },
                    limit: c.limit,
                    count: c.num_elements,
                    manifold_id: c.manifold_id,
                    inv_dt: c.inv_dt,
                    erp_inv_dt: c.erp_inv_dt,
                    soft_cfm: c.soft_cfm_factor,
                    a: frozen_point(a),
                    b: frozen_point(b),
                },
            );
        hot.append(Hot { a: hot_point(a), b: hot_point(b) });
    }
}

/// BT1's generation: `contact::generate_cached` per manifold, then the split of the constraint
/// (`push`). Same results as `generation::generate`.
pub(crate) fn generate_via_constraints(
    mut manifolds: Span<ContactManifold>,
    bodies: Span<SolverBody>,
    params: IntegrationParameters,
    dt: Fixed,
) -> (Array<Frozen>, Array<Hot>) {
    let mut cache = SoftCacheTrait::new(params, dt);
    let mut frozen = array![];
    let mut hot = array![];
    let mut id = 0;
    while let Some(m) = manifolds.pop_front() {
        let mut c = generate_cached(*m, bodies, dt, ref cache);
        c.manifold_id = id;
        push(ref frozen, ref hot, c);
        id += 1;
    }
    (frozen, hot)
}

