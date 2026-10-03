//! Frozen/hot contact sweeps (BT1). Generation's constraints are split once per step into a
//! frame-constant span (`Frozen`: endpoints, weighted directions, row coefficients, anchors) and
//! small arrays of the values the sweeps change (`Hot`: impulses, rhs, cfm; `Bank`: accumulators
//! and cached separations), so a sweep rebuilds 10 felts per manifold instead of the whole
//! constraint. Inert constraints are dropped. Bodies are a `SweepBodies` (velocities in the
//! dictionary, poses in an array rebuilt once per substep). Same values, rounding and panics as
//! `contact`'s sweeps.
//!
//! BT3 (exact Cairo steps of `solve_island` on the level-10 impact tick 28, 290 613 → 194 418;
//! every result bit-identical, the impact digests pinned): generation straight into
//! `Frozen` / `Hot` / `Bank` (`generation`, −8.5k, then its lean state −7.2k, the slot index
//! −4.4k); fused exact wide sums (`jv_add`, `separation`, inlined `transform`), pre-negated
//! second weights, unit warm start (−17.0k); skipped exact zeros: a zero impulse delta changes
//! no velocity (−25.1k), a unit `cfm` needs no product (−2.0k), a zero friction limit makes the
//! tangent row zero (−7.5k), zero rhs terms (−4.7k), zero warm-start impulses one by one
//! (−2.0k), unchanged velocities are not written back (−2.0k); the next substep's update reuses
//! the refresh's separations (−3.1k, and −1.6k once they moved to `Bank`); the writeback loop
//! carries an id, not a constraint (−3.5k); the reusing update reads no pose, the restitution
//! sweep is skipped without a negative seed, unclamped velocities are not rewritten (−3.4k);
//! `bodies`: zero-com translations and zero damping (−4.4k). Rejected ones: `alternatives`.
use fixed::wide::mul_add;
use fixed::{Fixed, ONE, ZERO};
use glam_core::Vec2;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_geometry2d::contact::ContactManifold;
use rapier_math::pose2::Pose2;
use super::super::super::body::{SolverVel, WORLD};
use super::super::super::contact::element::{max, min, row_impulse, separation};
use super::super::super::contact::errors;

/// Frame-constant coefficients of one row (normal or tangent).
#[derive(Copy, Drop, Debug, PartialEq, Default)]
pub(crate) struct Row {
    pub g1: Fixed,
    pub g2: Fixed,
    pub ig1: Fixed,
    pub ig2: Fixed,
    pub r: Fixed,
}

/// Frame-constant part of one contact point.
#[derive(Copy, Drop, Debug, PartialEq, Default)]
pub(crate) struct FrozenPoint {
    pub n: Row,
    pub t: Row,
    pub local_p1: Vec2,
    pub local_p2: Vec2,
    pub dist: Fixed,
    pub t_rhs_wo_bias: Fixed,
    pub seed: Fixed,
    pub contact_id: u8,
}

/// Frame-constant `direction * inverse_mass` of both endpoints (floored once per component),
/// the second one negated so that applying an impulse needs no negation (`floor(-w * i)` is
/// `floor(w * -i)`: the exact product is the same).
#[derive(Copy, Drop, Debug, PartialEq)]
pub(crate) struct Weights {
    pub first: Vec2,
    pub neg_second: Vec2,
}

/// Frame-constant part of one active constraint; `count` is 1 or 2; `t` is `tangent(dir)`.
#[derive(Copy, Drop, Debug, PartialEq)]
pub(crate) struct Frozen {
    pub i: u32,
    pub j: u32,
    pub dir: Vec2,
    pub t: Vec2,
    pub wn: Weights,
    pub wt: Weights,
    pub limit: Fixed,
    pub count: u8,
    pub manifold_id: u32,
    pub inv_dt: Fixed,
    pub erp_inv_dt: Fixed,
    pub soft_cfm: Fixed,
    pub a: FrozenPoint,
    pub b: FrozenPoint,
}

/// What every sweep reads or changes for one contact point (the rows).
#[derive(Copy, Drop, Debug, PartialEq, Default)]
pub(crate) struct HotPoint {
    pub impulse: Fixed,
    pub rhs: Fixed,
    pub cfm: Fixed,
    pub t_impulse: Fixed,
    pub t_rhs: Fixed,
}

/// The rows of one active constraint.
#[derive(Copy, Drop, Debug, PartialEq)]
pub(crate) struct Hot {
    pub a: HotPoint,
    pub b: HotPoint,
}

/// What only the update, refresh, restitution and writeback stages read or change for one
/// contact point (BT3: kept out of `Hot`, which the biased and relaxation sweeps copy three
/// times per constraint): the banked impulses of the completed substeps and the separations
/// the last refresh computed from the current poses, which the next substep's update reuses
/// instead of recomputing them from the same poses.
#[derive(Copy, Drop, Debug, PartialEq, Default)]
pub(crate) struct BankPoint {
    pub acc: Fixed,
    pub t_acc: Fixed,
    pub dist: Fixed,
    pub t_dist: Fixed,
}

/// The banked values of one active constraint.
#[derive(Copy, Drop, Debug, PartialEq)]
pub(crate) struct Bank {
    pub a: BankPoint,
    pub b: BankPoint,
}

/// The sweeps' state, one `Hot` and one `Bank` per active constraint, in constraint order;
/// `bounce` is `false` when no point has a negative restitution seed (no bounce is possible).
#[derive(Drop)]
pub(crate) struct State {
    pub hot: Array<Hot>,
    pub bank: Array<Bank>,
    pub bounce: bool,
}

/// Visit the active constraints in order: 0 update/warmstart, 1 bias, 2 rhs/relax, 3 relax,
/// 5 update/warmstart reusing the separations of the last stage 2 (valid when no pose changed
/// since), else bounce (the stages of `contact::contacts`).
pub(crate) fn contacts(
    ref state: State,
    frozen: Span<Frozen>,
    ref bodies: SweepBodies,
    p: IntegrationParameters,
    stage: u8,
) {
    match stage {
        0 |
        5 => {
            assert(p.warmstart_coefficient >= ZERO, errors::NEGATIVE);
            let warm = p.warmstart_coefficient;
            let k = Update {
                warm, unit: warm == ONE, neg_cap: -p.max_corrective_velocity(), reuse: stage == 5,
            };
            banked(ref state, frozen, ref bodies, k)
        },
        1 => {
            if p.friction_in_bias_pass {
                sweep(ref state.hot, frozen, ref bodies, Relax {});
            } else {
                sweep(ref state.hot, frozen, ref bodies, Biased {});
            }
        },
        2 => banked(ref state, frozen, ref bodies, Refresh {}),
        3 => sweep(ref state.hot, frozen, ref bodies, Relax {}),
        _ => {
            // BT3: without a negative seed every constraint would leave the stage unchanged.
            if state.bounce {
                banked(ref state, frozen, ref bodies, Restitution {});
            }
        },
    }
}

/// One stage on one constraint; gathers and scatters its two bodies only when it changes them.
trait Kernel<K> {
    fn apply(self: K, ref h: Hot, f: @Frozen, ref bodies: SweepBodies, poses: Span<Pose2>);
}
/// A stage that also reads or changes the constraint's `Bank`.
trait BankKernel<K> {
    fn apply(
        self: K, ref h: Hot, ref b: Bank, f: @Frozen, ref bodies: SweepBodies, poses: Span<Pose2>,
    );
}

#[derive(Copy, Drop)]
struct Update {
    warm: Fixed,
    unit: bool,
    neg_cap: Fixed,
    /// Stage 5: the separations cached by the last refresh are those of the current poses.
    reuse: bool,
}
#[derive(Copy, Drop)]
struct Biased {}
#[derive(Copy, Drop)]
struct Refresh {}
#[derive(Copy, Drop)]
struct Relax {}
#[derive(Copy, Drop)]
struct Restitution {}

fn sweep<K, +Kernel<K>, +Copy<K>, +Drop<K>>(
    ref hot: Array<Hot>, mut frozen: Span<Frozen>, ref bodies: SweepBodies, k: K,
) {
    let poses = bodies.poses.span();
    let mut out = array![];
    while let Some(f) = frozen.pop_front() {
        let mut h = hot.pop_front().unwrap();
        k.apply(ref h, f, ref bodies, poses);
        out.append(h);
    }
    hot = out;
}

fn banked<K, +BankKernel<K>, +Copy<K>, +Drop<K>>(
    ref state: State, mut frozen: Span<Frozen>, ref bodies: SweepBodies, k: K,
) {
    let poses = bodies.poses.span();
    let mut hot = array![];
    let mut bank = array![];
    while let Some(f) = frozen.pop_front() {
        let mut h = state.hot.pop_front().unwrap();
        let mut b = state.bank.pop_front().unwrap();
        k.apply(ref h, ref b, f, ref bodies, poses);
        hot.append(h);
        bank.append(b);
    }
    state = State { hot, bank, bounce: state.bounce };
}

/// `apply` of `contact::cached`: weighted linear parts, inertia-weighted angular parts. Each
/// `v + floor(w * impulse)` is one `mul_add` (`floor(v + p) = v + floor(p)` for an integer `v`);
/// the second body's weights are stored negated (BT3), so the impulse is not.
#[inline(always)]
fn apply(
    w: @Weights, ig1: Fixed, ig2: Fixed, impulse: Fixed, ref v1: SolverVel, ref v2: SolverVel,
) {
    let (w1, w2) = (*w.first, *w.neg_second);
    v1
        .linear =
            Vec2 { x: mul_add(w1.x, impulse, v1.linear.x), y: mul_add(w1.y, impulse, v1.linear.y) };
    v2
        .linear =
            Vec2 { x: mul_add(w2.x, impulse, v2.linear.x), y: mul_add(w2.y, impulse, v2.linear.y) };
    v1.angular = mul_add(ig1, impulse, v1.angular);
    v2.angular = mul_add(ig2, impulse, v2.angular);
}
/// FU1 P1: the new impulse `impulse - r * (jv + rhs)` is `element::row_impulse`, one exact wide
/// sum floored once (BT3's `jv_add`, which floored `jv + rhs` before the product, is
/// `element::alternatives::row_impulse_unfused`).
#[inline(always)]
fn solve_normal(
    ref h: HotPoint, dir: Vec2, row: @Row, w: @Weights, ref v1: SolverVel, ref v2: SolverVel,
) -> bool {
    let clamped = max(ZERO, row_impulse(dir, *row.g1, *row.g2, v1, v2, h.rhs, h.impulse, *row.r));
    // BT3: a rigid row (`cfm == ONE`: every relaxation row, speculative biased rows) skips the
    // product, exact since `x * ONE == x`.
    let new_impulse = if h.cfm == ONE {
        clamped
    } else {
        h.cfm * clamped
    };
    let delta = new_impulse - h.impulse;
    h.impulse = new_impulse;
    // BT3: a zero delta changes no velocity (`floor(w * 0) == 0`); reports whether it did.
    if delta == ZERO {
        return false;
    }
    apply(w, *row.ig1, *row.ig2, delta, ref v1, ref v2);
    true
}
#[inline(always)]
fn solve_tangent(
    ref h: HotPoint,
    dir: Vec2,
    row: @Row,
    w: @Weights,
    limit: Fixed,
    ref v1: SolverVel,
    ref v2: SolverVel,
) -> bool {
    // BT3: a zero limit (zero normal impulse) clamps to zero whatever the velocity is.
    let new_impulse = if limit == ZERO {
        ZERO
    } else {
        min(
            limit,
            max(-limit, row_impulse(dir, *row.g1, *row.g2, v1, v2, h.t_rhs, h.t_impulse, *row.r)),
        )
    };
    let delta = new_impulse - h.t_impulse;
    h.t_impulse = new_impulse;
    if delta == ZERO {
        return false;
    }
    apply(w, *row.ig1, *row.ig2, delta, ref v1, ref v2);
    true
}
/// `limit * impulse`, zero without the product for a zero impulse (exact: `floor(l * 0) == 0`).
#[inline(always)]
fn friction_limit(limit: Fixed, impulse: Fixed) -> Fixed {
    if impulse == ZERO {
        ZERO
    } else {
        limit * impulse
    }
}
#[inline(always)]
fn row_zero(h: HotPoint) -> bool {
    h.rhs == ZERO && h.impulse == ZERO && h.t_rhs == ZERO && h.t_impulse == ZERO
}
/// `cached::zero::idle`: the all-zero state, where every delta is exactly zero.
#[inline(always)]
fn idle(h: Hot, count: u8, v1: SolverVel, v2: SolverVel) -> bool {
    v1.linear == Default::default()
        && v2.linear == Default::default()
        && v1.angular == ZERO
        && v2.angular == ZERO
        && row_zero(h.a)
        && (count == 1 || row_zero(h.b))
}
/// `update_element` on the hot values (the unbiased rhs is transient: refresh recomputes it).
/// A warm-start coefficient of exactly one (the default) skips its two products (`x * ONE ==
/// x` exactly, BT3).
#[inline(always)]
fn update_point(
    ref h: HotPoint, ref b: BankPoint, f: @FrozenPoint, c: @Frozen, p1: Pose2, p2: Pose2, k: Update,
) {
    let (dist, t_dist) = if k.reuse {
        (b.dist, b.t_dist)
    } else {
        separation(p1, *f.local_p1, p2, *f.local_p2, *c.dir, *c.t, *f.dist)
    };
    // `max(0, dist) * inv_dt + min(0, max(-cap, dist * erp_inv_dt))` without its exact zeros
    // (BT3): the first term is zero for `dist <= 0`; the second for `dist > 0` when
    // `erp_inv_dt >= 0` (a floored non-negative product). After a refresh (`reuse`), `h.rhs`
    // already is the first term of the same `dist`.
    let erp_inv_dt = *c.erp_inv_dt;
    h
        .rhs =
            if dist > ZERO && erp_inv_dt >= ZERO {
                if k.reuse {
                    h.rhs
                } else {
                    dist * *c.inv_dt
                }
            } else if dist > ZERO {
                dist * *c.inv_dt + min(ZERO, max(k.neg_cap, dist * erp_inv_dt))
            } else {
                min(ZERO, max(k.neg_cap, dist * erp_inv_dt))
            };
    h.cfm = if dist > ZERO {
        ONE
    } else {
        *c.soft_cfm
    };
    b.acc += h.impulse;
    b.t_acc += h.t_impulse;
    if !k.unit {
        h.impulse = h.impulse * k.warm;
        h.t_impulse = h.t_impulse * k.warm;
    }
    h.t_rhs = mul_add(t_dist, *c.inv_dt, *f.t_rhs_wo_bias);
}
/// `refresh_unbiased` then `strip`. FU1 P2: both separations are `element::separation`, one
/// exact wide sum per component of the anchors' world difference and one floor per separation
/// (BT3's two inlined transforms then `separation` are
/// `element::alternatives::separation_of_transforms`).
#[inline(always)]
fn refresh_point(
    ref h: HotPoint, ref b: BankPoint, f: @FrozenPoint, c: @Frozen, p1: Pose2, p2: Pose2,
) {
    let (dist, t_dist) = separation(p1, *f.local_p1, p2, *f.local_p2, *c.dir, *c.t, *f.dist);
    b.dist = dist;
    b.t_dist = t_dist;
    // `max(0, dist) * inv_dt`, zero without the product for `dist <= 0` (BT3).
    h.rhs = if dist > ZERO {
        dist * *c.inv_dt
    } else {
        ZERO
    };
    h.cfm = ONE;
    h.t_rhs = *f.t_rhs_wo_bias;
}

impl UpdateKernel of BankKernel<Update> {
    #[inline(always)]
    fn apply(
        self: Update,
        ref h: Hot,
        ref b: Bank,
        f: @Frozen,
        ref bodies: SweepBodies,
        poses: Span<Pose2>,
    ) {
        // BT3: a reusing update reads no pose.
        let (p1, p2) = if self.reuse {
            (Default::default(), Default::default())
        } else {
            (pose(poses, *f.i), pose(poses, *f.j))
        };
        let two = *f.count == 2;
        update_point(ref h.a, ref b.a, f.a, f, p1, p2, self);
        if two {
            update_point(ref h.b, ref b.b, f.b, f, p1, p2, self);
        }
        // `cached::warmstart_sparse`: zero impulses are an exact no-op.
        if h.a.impulse == ZERO
            && h.a.t_impulse == ZERO
            && (!two || (h.b.impulse == ZERO && h.b.t_impulse == ZERO)) {
            return;
        }
        let mut v1 = bodies.vel(*f.i);
        let mut v2 = bodies.vel(*f.j);
        // BT3: each zero impulse is skipped on its own (for a one-point constraint the second
        // point's impulses are zero).
        if h.a.impulse != ZERO {
            apply(f.wn, *f.a.n.ig1, *f.a.n.ig2, h.a.impulse, ref v1, ref v2);
        }
        if h.b.impulse != ZERO {
            apply(f.wn, *f.b.n.ig1, *f.b.n.ig2, h.b.impulse, ref v1, ref v2);
        }
        if h.a.t_impulse != ZERO {
            apply(f.wt, *f.a.t.ig1, *f.a.t.ig2, h.a.t_impulse, ref v1, ref v2);
        }
        if h.b.t_impulse != ZERO {
            apply(f.wt, *f.b.t.ig1, *f.b.t.ig2, h.b.t_impulse, ref v1, ref v2);
        }
        bodies.set_vels(*f.i, v1, *f.j, v2);
    }
}
impl BiasedKernel of Kernel<Biased> {
    #[inline(always)]
    fn apply(self: Biased, ref h: Hot, f: @Frozen, ref bodies: SweepBodies, poses: Span<Pose2>) {
        let mut v1 = bodies.vel(*f.i);
        let mut v2 = bodies.vel(*f.j);
        if idle(h, *f.count, v1, v2) {
            return;
        }
        let dir = *f.dir;
        let mut changed = solve_normal(ref h.a, dir, f.a.n, f.wn, ref v1, ref v2);
        if *f.count == 2 {
            changed = solve_normal(ref h.b, dir, f.b.n, f.wn, ref v1, ref v2) || changed;
        }
        // BT3: unchanged velocities are not written back.
        if changed {
            bodies.set_vels(*f.i, v1, *f.j, v2);
        }
    }
}
/// Normal then tangent rows (`cached::zero::solve_both`); the all-zero state is left untouched.
#[inline(always)]
fn solve_both(ref h: Hot, f: @Frozen, ref bodies: SweepBodies) {
    let mut v1 = bodies.vel(*f.i);
    let mut v2 = bodies.vel(*f.j);
    if idle(h, *f.count, v1, v2) {
        return;
    }
    let dir = *f.dir;
    let two = *f.count == 2;
    let mut changed = solve_normal(ref h.a, dir, f.a.n, f.wn, ref v1, ref v2);
    if two {
        changed = solve_normal(ref h.b, dir, f.b.n, f.wn, ref v1, ref v2) || changed;
    }
    let t = *f.t;
    let limit = *f.limit;
    changed =
        solve_tangent(ref h.a, t, f.a.t, f.wt, friction_limit(limit, h.a.impulse), ref v1, ref v2)
        || changed;
    if two {
        changed =
            solve_tangent(
                ref h.b, t, f.b.t, f.wt, friction_limit(limit, h.b.impulse), ref v1, ref v2,
            )
            || changed;
    }
    // BT3: unchanged velocities are not written back.
    if changed {
        bodies.set_vels(*f.i, v1, *f.j, v2);
    }
}
impl RefreshKernel of BankKernel<Refresh> {
    #[inline(always)]
    fn apply(
        self: Refresh,
        ref h: Hot,
        ref b: Bank,
        f: @Frozen,
        ref bodies: SweepBodies,
        poses: Span<Pose2>,
    ) {
        let p1 = pose(poses, *f.i);
        let p2 = pose(poses, *f.j);
        refresh_point(ref h.a, ref b.a, f.a, f, p1, p2);
        if *f.count == 2 {
            refresh_point(ref h.b, ref b.b, f.b, f, p1, p2);
        }
        solve_both(ref h, f, ref bodies);
    }
}
impl RelaxKernel of Kernel<Relax> {
    #[inline(always)]
    fn apply(self: Relax, ref h: Hot, f: @Frozen, ref bodies: SweepBodies, poses: Span<Pose2>) {
        solve_both(ref h, f, ref bodies);
    }
}
/// `bounce`: rhs and cfm are not restored, nothing reads them after the final sweep.
#[inline(always)]
fn bounce(
    ref h: HotPoint,
    b: BankPoint,
    f: @FrozenPoint,
    dir: Vec2,
    w: @Weights,
    ref v1: SolverVel,
    ref v2: SolverVel,
) {
    if *f.seed < ZERO && b.acc + h.impulse > ZERO {
        h.rhs = *f.seed;
        h.cfm = ONE;
        let _ = solve_normal(ref h, dir, f.n, w, ref v1, ref v2);
    }
}
impl RestitutionKernel of BankKernel<Restitution> {
    #[inline(always)]
    fn apply(
        self: Restitution,
        ref h: Hot,
        ref b: Bank,
        f: @Frozen,
        ref bodies: SweepBodies,
        poses: Span<Pose2>,
    ) {
        let two = *f.count == 2;
        if *f.a.seed >= ZERO && (!two || *f.b.seed >= ZERO) {
            return;
        }
        let mut v1 = bodies.vel(*f.i);
        let mut v2 = bodies.vel(*f.j);
        let dir = *f.dir;
        bounce(ref h.a, b.a, f.a, dir, f.wn, ref v1, ref v2);
        if two {
            bounce(ref h.b, b.b, f.b, dir, f.wn, ref v1, ref v2);
        }
        bodies.set_vels(*f.i, v1, *f.j, v2);
    }
}

#[inline(always)]
fn write_point(h: HotPoint, b: BankPoint, f: @FrozenPoint, ref m: ContactManifold) {
    let [mut p0, mut p1] = m.points;
    let first = *f.contact_id == 0;
    let mut p = if first {
        p0
    } else {
        p1
    };
    p.data.impulse = b.acc + h.impulse;
    p.data.tangent_impulse = b.t_acc + h.t_impulse;
    p.data.warmstart_impulse = h.impulse;
    p.data.warmstart_tangent_impulse = h.t_impulse;
    if first {
        p0 = p;
    } else {
        p1 = p;
    }
    m.points = [p0, p1];
}

/// `ContactConstraintsSetTrait::writeback_impulses` from the split state: one ordered rebuild
/// of `manifolds`, the active constraints in ascending manifold id.
pub(crate) fn writeback(
    mut frozen: Span<Frozen>, state: @State, ref manifolds: Array<ContactManifold>,
) {
    let mut hot = state.hot.span();
    let mut bank = state.bank.span();
    let mut out = array![];
    let mut id = 0;
    // BT3: the loop carries the next constraint's manifold id, not the constraint (an
    // `Option<@Frozen>` in the loop state was copied whole on every iteration).
    let mut next = next_id(frozen);
    while let Some(mut m) = manifolds.pop_front() {
        if id == next {
            let f = frozen.pop_front().unwrap();
            let h = *hot.pop_front().unwrap();
            let b = *bank.pop_front().unwrap();
            write_point(h.a, b.a, f.a, ref m);
            if *f.count == 2 {
                write_point(h.b, b.b, f.b, ref m);
            }
            next = next_id(frozen);
        }
        out.append(m);
        id += 1;
    }
    manifolds = out;
}

/// The manifold id of the first constraint of `frozen`; `WORLD` (no manifold has it) when empty.
#[inline(always)]
fn next_id(frozen: Span<Frozen>) -> u32 {
    match frozen.get(0) {
        Some(f) => *f.unbox().manifold_id,
        None => WORLD,
    }
}

#[inline(always)]
fn pose(poses: Span<Pose2>, i: u32) -> Pose2 {
    if i == WORLD {
        Default::default()
    } else {
        *poses.at(i)
    }
}
#[cfg(test)]
mod alternatives;

mod bodies;
pub(crate) mod generation;
#[cfg(test)]
mod tests;
pub(crate) use bodies::{SweepBodies, SweepBodiesTrait};
