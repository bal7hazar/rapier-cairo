//! Frozen/hot contact sweeps (BT1). Generation's constraints are split once per step into a
//! frame-constant span (`Frozen`: endpoints, weighted directions, row coefficients, anchors) and
//! a small array of the values the sweeps change (`Hot`: impulses, rhs, cfm, accumulators), so a
//! sweep rebuilds 14 felts per manifold instead of the whole constraint. Inert constraints are
//! dropped. Bodies are a `SweepBodies` (velocities in the dictionary, poses in an array
//! rebuilt once per substep). Same expressions, operand order, rounding and panics as `contact`'s
//! sweeps.
use fixed::wide::{WideAdd, WideNarrow, WideSub, dot2_add, mul_add, wide_from, wide_mul};
use fixed::{Fixed, ONE, ZERO};
use glam::Vec2;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_geometry2d::contact::ContactManifold;
use rapier_math::pose2::Pose2;
use super::super::super::body::{SolverVel, WORLD};
use super::super::super::contact::element::{max, min};
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

/// What the sweeps change for one contact point. `dist` / `t_dist` cache the separations the
/// last refresh computed from the current poses (BT3): the next substep's update reads them
/// instead of recomputing them from the same poses.
#[derive(Copy, Drop, Debug, PartialEq, Default)]
pub(crate) struct HotPoint {
    pub impulse: Fixed,
    pub rhs: Fixed,
    pub cfm: Fixed,
    pub t_impulse: Fixed,
    pub t_rhs: Fixed,
    pub acc: Fixed,
    pub t_acc: Fixed,
    pub dist: Fixed,
    pub t_dist: Fixed,
}

/// What the sweeps change for one active constraint.
#[derive(Copy, Drop, Debug, PartialEq)]
pub(crate) struct Hot {
    pub a: HotPoint,
    pub b: HotPoint,
}

/// Visit the active constraints in order: 0 update/warmstart, 1 bias, 2 rhs/relax, 3 relax,
/// 5 update/warmstart reusing the separations of the last stage 2 (valid when no pose changed
/// since), else bounce (the stages of `contact::contacts`).
pub(crate) fn contacts(
    ref hot: Array<Hot>,
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
            sweep(ref hot, frozen, ref bodies, k)
        },
        1 => {
            if p.friction_in_bias_pass {
                sweep(ref hot, frozen, ref bodies, Relax {});
            } else {
                sweep(ref hot, frozen, ref bodies, Biased {});
            }
        },
        2 => sweep(ref hot, frozen, ref bodies, Refresh {}),
        3 => sweep(ref hot, frozen, ref bodies, Relax {}),
        _ => sweep(ref hot, frozen, ref bodies, Restitution {}),
    }
}

/// One stage on one constraint; gathers and scatters its two bodies only when it changes them.
trait Kernel<K> {
    fn apply(self: K, ref h: Hot, f: @Frozen, ref bodies: SweepBodies, poses: Span<Pose2>);
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
/// `jv(..) + rhs` with one rescale: `element::jv`'s `dot4` plus `rhs`. The velocity difference
/// is distributed over the exact wide sum (BT3: `d * (a - b) = d * a - d * b` exactly), which
/// saves the two checked subtractions; the floored result is the same.
#[inline(always)]
fn jv_add(dir: Vec2, g1: Fixed, g2: Fixed, v1: SolverVel, v2: SolverVel, rhs: Fixed) -> Fixed {
    wide_mul(dir.x, v1.linear.x)
        .sub(wide_mul(dir.x, v2.linear.x))
        .add(wide_mul(dir.y, v1.linear.y))
        .sub(wide_mul(dir.y, v2.linear.y))
        .add(wide_mul(g1, v1.angular))
        .add(wide_mul(g2, v2.angular))
        .add(wide_from(rhs))
        .narrow()
}
#[inline(always)]
fn solve_normal(
    ref h: HotPoint, dir: Vec2, row: @Row, w: @Weights, ref v1: SolverVel, ref v2: SolverVel,
) -> bool {
    let dv = jv_add(dir, *row.g1, *row.g2, v1, v2, h.rhs);
    let clamped = max(ZERO, h.impulse - *row.r * dv);
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
        let dv = jv_add(dir, *row.g1, *row.g2, v1, v2, h.t_rhs);
        min(limit, max(-limit, h.t_impulse - *row.r * dv))
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
/// `Pose2::transform_point` inlined (BT3: the call was 785 outlined calls per impact tick):
/// the same wide sums, `re * x - im * y + t` instead of `re * x + (-im) * y + t`, one floor per
/// component.
#[inline(always)]
fn transform(p: Pose2, l: Vec2) -> Vec2 {
    let r = p.rotation;
    Vec2 {
        x: wide_mul(r.re, l.x).sub(wide_mul(r.im, l.y)).add(wide_from(p.translation.x)).narrow(),
        y: dot2_add(r.im, l.x, r.re, l.y, p.translation.y),
    }
}
/// `(dist + dot(a - b, dir), dot(a - b, t))` with the difference distributed over the exact
/// wide sums, as in `jv_add`.
#[inline(always)]
fn separation(a: Vec2, b: Vec2, dir: Vec2, t: Vec2, dist: Fixed) -> (Fixed, Fixed) {
    (
        wide_mul(a.x, dir.x)
            .sub(wide_mul(b.x, dir.x))
            .add(wide_mul(a.y, dir.y))
            .sub(wide_mul(b.y, dir.y))
            .add(wide_from(dist))
            .narrow(),
        wide_mul(a.x, t.x)
            .sub(wide_mul(b.x, t.x))
            .add(wide_mul(a.y, t.y))
            .sub(wide_mul(b.y, t.y))
            .narrow(),
    )
}
/// `update_element` on the hot values (the unbiased rhs is transient: refresh recomputes it).
/// A warm-start coefficient of exactly one (the default) skips its two products (`x * ONE ==
/// x` exactly, BT3).
#[inline(always)]
fn update_point(ref h: HotPoint, f: @FrozenPoint, c: @Frozen, p1: Pose2, p2: Pose2, k: Update) {
    let (dist, t_dist) = if k.reuse {
        (h.dist, h.t_dist)
    } else {
        separation(transform(p1, *f.local_p1), transform(p2, *f.local_p2), *c.dir, *c.t, *f.dist)
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
    h.acc += h.impulse;
    h.t_acc += h.t_impulse;
    if !k.unit {
        h.impulse = h.impulse * k.warm;
        h.t_impulse = h.t_impulse * k.warm;
    }
    h.t_rhs = mul_add(t_dist, *c.inv_dt, *f.t_rhs_wo_bias);
}
/// `refresh_unbiased` then `strip`.
#[inline(always)]
fn refresh_point(ref h: HotPoint, f: @FrozenPoint, c: @Frozen, p1: Pose2, p2: Pose2) {
    let (dist, t_dist) = separation(
        transform(p1, *f.local_p1), transform(p2, *f.local_p2), *c.dir, *c.t, *f.dist,
    );
    h.dist = dist;
    h.t_dist = t_dist;
    // `max(0, dist) * inv_dt`, zero without the product for `dist <= 0` (BT3).
    h.rhs = if dist > ZERO {
        dist * *c.inv_dt
    } else {
        ZERO
    };
    h.cfm = ONE;
    h.t_rhs = *f.t_rhs_wo_bias;
}

impl UpdateKernel of Kernel<Update> {
    #[inline(always)]
    fn apply(self: Update, ref h: Hot, f: @Frozen, ref bodies: SweepBodies, poses: Span<Pose2>) {
        let p1 = pose(poses, *f.i);
        let p2 = pose(poses, *f.j);
        let two = *f.count == 2;
        update_point(ref h.a, f.a, f, p1, p2, self);
        if two {
            update_point(ref h.b, f.b, f, p1, p2, self);
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
impl RefreshKernel of Kernel<Refresh> {
    #[inline(always)]
    fn apply(self: Refresh, ref h: Hot, f: @Frozen, ref bodies: SweepBodies, poses: Span<Pose2>) {
        let p1 = pose(poses, *f.i);
        let p2 = pose(poses, *f.j);
        refresh_point(ref h.a, f.a, f, p1, p2);
        if *f.count == 2 {
            refresh_point(ref h.b, f.b, f, p1, p2);
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
    ref h: HotPoint, f: @FrozenPoint, dir: Vec2, w: @Weights, ref v1: SolverVel, ref v2: SolverVel,
) {
    if *f.seed < ZERO && h.acc + h.impulse > ZERO {
        h.rhs = *f.seed;
        h.cfm = ONE;
        let _ = solve_normal(ref h, dir, f.n, w, ref v1, ref v2);
    }
}
impl RestitutionKernel of Kernel<Restitution> {
    #[inline(always)]
    fn apply(
        self: Restitution, ref h: Hot, f: @Frozen, ref bodies: SweepBodies, poses: Span<Pose2>,
    ) {
        let two = *f.count == 2;
        if *f.a.seed >= ZERO && (!two || *f.b.seed >= ZERO) {
            return;
        }
        let mut v1 = bodies.vel(*f.i);
        let mut v2 = bodies.vel(*f.j);
        let dir = *f.dir;
        bounce(ref h.a, f.a, dir, f.wn, ref v1, ref v2);
        if two {
            bounce(ref h.b, f.b, dir, f.wn, ref v1, ref v2);
        }
        bodies.set_vels(*f.i, v1, *f.j, v2);
    }
}

#[inline(always)]
fn write_point(h: HotPoint, f: @FrozenPoint, ref m: ContactManifold) {
    let [mut p0, mut p1] = m.points;
    let first = *f.contact_id == 0;
    let mut p = if first {
        p0
    } else {
        p1
    };
    p.data.impulse = h.acc + h.impulse;
    p.data.tangent_impulse = h.t_acc + h.t_impulse;
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
    mut frozen: Span<Frozen>, mut hot: Span<Hot>, ref manifolds: Array<ContactManifold>,
) {
    let mut out = array![];
    let mut id = 0;
    // BT3: the loop carries the next constraint's manifold id, not the constraint (an
    // `Option<@Frozen>` in the loop state was copied whole on every iteration).
    let mut next = next_id(frozen);
    while let Some(mut m) = manifolds.pop_front() {
        if id == next {
            let f = frozen.pop_front().unwrap();
            let h = *hot.pop_front().unwrap();
            write_point(h.a, f.a, ref m);
            if *f.count == 2 {
                write_point(h.b, f.b, ref m);
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
