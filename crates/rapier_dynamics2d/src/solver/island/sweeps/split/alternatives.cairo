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
