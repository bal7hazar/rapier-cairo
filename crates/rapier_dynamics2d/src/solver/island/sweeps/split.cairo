//! Frozen/hot contact sweeps (BT1). Generation's constraints are split once per step into a
//! frame-constant span (`Frozen`: endpoints, weighted directions, row coefficients, anchors) and
//! a small array of the values the sweeps change (`Hot`: impulses, rhs, cfm, accumulators), so a
//! sweep rebuilds 14 felts per manifold instead of the whole constraint. Inert constraints are
//! dropped. Bodies are a `SweepBodies` (velocities in the dictionary, poses in an array
//! rebuilt once per substep). Same expressions, operand order, rounding and panics as `contact`'s
//! sweeps.
use fixed::{Fixed, ONE, ZERO};
use glam::Vec2;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_geometry2d::contact::ContactManifold;
use rapier_math::pose2::{Pose2, Pose2Trait};
use super::super::super::body::{SolverVel, WORLD};
use super::super::super::contact::cached::WeightedPair;
use super::super::super::contact::element::{dot, jv, max, min, tangent};
use super::super::super::contact::{ContactConstraint, ContactConstraintElement, errors};

/// Frame-constant coefficients of one row (normal or tangent).
#[derive(Copy, Drop, Debug, PartialEq)]
pub(crate) struct Row {
    pub g1: Fixed,
    pub g2: Fixed,
    pub ig1: Fixed,
    pub ig2: Fixed,
    pub r: Fixed,
}

/// Frame-constant part of one contact point.
#[derive(Copy, Drop, Debug, PartialEq)]
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

/// Frame-constant part of one active constraint; `count` is 1 or 2.
#[derive(Copy, Drop, Debug, PartialEq)]
pub(crate) struct Frozen {
    pub i: u32,
    pub j: u32,
    pub dir: Vec2,
    pub wn: WeightedPair,
    pub wt: WeightedPair,
    pub limit: Fixed,
    pub count: u8,
    pub manifold_id: u32,
    pub inv_dt: Fixed,
    pub erp_inv_dt: Fixed,
    pub soft_cfm: Fixed,
    pub a: FrozenPoint,
    pub b: FrozenPoint,
}

/// What the sweeps change for one contact point.
#[derive(Copy, Drop, Debug, PartialEq)]
pub(crate) struct HotPoint {
    pub impulse: Fixed,
    pub rhs: Fixed,
    pub cfm: Fixed,
    pub t_impulse: Fixed,
    pub t_rhs: Fixed,
    pub acc: Fixed,
    pub t_acc: Fixed,
}

/// What the sweeps change for one active constraint.
#[derive(Copy, Drop, Debug, PartialEq)]
pub(crate) struct Hot {
    pub a: HotPoint,
    pub b: HotPoint,
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

/// Split the active constraints (in order) and cache `dir * im` for both rows, as
/// `cached::prepare` does (products floor).
pub(crate) fn prepare(mut cs: Span<ContactConstraint>) -> (Array<Frozen>, Array<Hot>) {
    let mut frozen = array![];
    let mut hot = array![];
    while let Some(c) = cs.pop_front() {
        let c = *c;
        if c.num_elements != 0 {
            let t = tangent(c.dir1);
            let [a, b] = c.elements;
            frozen
                .append(
                    Frozen {
                        i: c.solver_vel1,
                        j: c.solver_vel2,
                        dir: c.dir1,
                        wn: WeightedPair { first: c.dir1 * c.im1, second: c.dir1 * c.im2 },
                        wt: WeightedPair { first: t * c.im1, second: t * c.im2 },
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
    (frozen, hot)
}

/// Visit the active constraints in order: 0 update/warmstart, 1 bias, 2 rhs/relax, 3 relax,
/// else bounce (the stages of `contact::contacts`).
pub(crate) fn contacts(
    ref hot: Array<Hot>,
    frozen: Span<Frozen>,
    ref bodies: SweepBodies,
    p: IntegrationParameters,
    stage: u8,
) {
    match stage {
        0 => {
            assert(p.warmstart_coefficient >= ZERO, errors::NEGATIVE);
            let k = Update { warm: p.warmstart_coefficient, cap: p.max_corrective_velocity() };
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
    cap: Fixed,
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

/// `apply` of `contact::cached`: weighted linear parts, inertia-weighted angular parts.
#[inline(always)]
fn apply(
    w: @WeightedPair, ig1: Fixed, ig2: Fixed, impulse: Fixed, ref v1: SolverVel, ref v2: SolverVel,
) {
    v1.linear = v1.linear + *w.first * Vec2 { x: impulse, y: impulse };
    v2.linear = v2.linear + *w.second * Vec2 { x: -impulse, y: -impulse };
    v1.angular += ig1 * impulse;
    v2.angular += ig2 * impulse;
}
#[inline(always)]
fn solve_normal(
    ref h: HotPoint, dir: Vec2, row: @Row, w: @WeightedPair, ref v1: SolverVel, ref v2: SolverVel,
) {
    let dv = jv(dir, *row.g1, *row.g2, v1, v2) + h.rhs;
    let new_impulse = h.cfm * max(ZERO, h.impulse - *row.r * dv);
    let delta = new_impulse - h.impulse;
    h.impulse = new_impulse;
    apply(w, *row.ig1, *row.ig2, delta, ref v1, ref v2);
}
#[inline(always)]
fn solve_tangent(
    ref h: HotPoint,
    dir: Vec2,
    row: @Row,
    w: @WeightedPair,
    limit: Fixed,
    ref v1: SolverVel,
    ref v2: SolverVel,
) {
    let dv = jv(dir, *row.g1, *row.g2, v1, v2) + h.t_rhs;
    let new_impulse = min(limit, max(-limit, h.t_impulse - *row.r * dv));
    let delta = new_impulse - h.t_impulse;
    h.t_impulse = new_impulse;
    apply(w, *row.ig1, *row.ig2, delta, ref v1, ref v2);
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
#[inline(always)]
fn update_point(
    ref h: HotPoint, f: @FrozenPoint, c: @Frozen, p1: Pose2, p2: Pose2, warm: Fixed, cap: Fixed,
) {
    let dp = p1.transform_point(*f.local_p1) - p2.transform_point(*f.local_p2);
    let dist = *f.dist + dot(dp, *c.dir);
    let rhs_wo_bias = max(ZERO, dist) * *c.inv_dt;
    h.rhs = rhs_wo_bias + min(ZERO, max(-cap, dist * *c.erp_inv_dt));
    h.cfm = if dist > ZERO {
        ONE
    } else {
        *c.soft_cfm
    };
    h.acc += h.impulse;
    h.impulse = h.impulse * warm;
    h.t_acc += h.t_impulse;
    h.t_impulse = h.t_impulse * warm;
    h.t_rhs = *f.t_rhs_wo_bias + dot(dp, tangent(*c.dir)) * *c.inv_dt;
}
/// `refresh_unbiased` then `strip`.
#[inline(always)]
fn refresh_point(ref h: HotPoint, f: @FrozenPoint, c: @Frozen, p1: Pose2, p2: Pose2) {
    let dp = p1.transform_point(*f.local_p1) - p2.transform_point(*f.local_p2);
    h.rhs = max(ZERO, *f.dist + dot(dp, *c.dir)) * *c.inv_dt;
    h.cfm = ONE;
    h.t_rhs = *f.t_rhs_wo_bias;
}

impl UpdateKernel of Kernel<Update> {
    #[inline(always)]
    fn apply(self: Update, ref h: Hot, f: @Frozen, ref bodies: SweepBodies, poses: Span<Pose2>) {
        let p1 = pose(poses, *f.i);
        let p2 = pose(poses, *f.j);
        let two = *f.count == 2;
        update_point(ref h.a, f.a, f, p1, p2, self.warm, self.cap);
        if two {
            update_point(ref h.b, f.b, f, p1, p2, self.warm, self.cap);
        }
        // `cached::warmstart_sparse`: zero impulses are an exact no-op.
        if h.a.impulse == ZERO
            && h.a.t_impulse == ZERO
            && (!two || (h.b.impulse == ZERO && h.b.t_impulse == ZERO)) {
            return;
        }
        let mut v1 = bodies.vel(*f.i);
        let mut v2 = bodies.vel(*f.j);
        let mut pending = true;
        while pending {
            apply(f.wn, *f.a.n.ig1, *f.a.n.ig2, h.a.impulse, ref v1, ref v2);
            if two {
                apply(f.wn, *f.b.n.ig1, *f.b.n.ig2, h.b.impulse, ref v1, ref v2);
            }
            apply(f.wt, *f.a.t.ig1, *f.a.t.ig2, h.a.t_impulse, ref v1, ref v2);
            if two {
                apply(f.wt, *f.b.t.ig1, *f.b.t.ig2, h.b.t_impulse, ref v1, ref v2);
            }
            pending = false;
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
/// Normal then tangent rows (`cached::zero::solve_both`); the all-zero state is left untouched.
#[inline(always)]
fn solve_both(ref h: Hot, f: @Frozen, ref bodies: SweepBodies) {
    let mut v1 = bodies.vel(*f.i);
    let mut v2 = bodies.vel(*f.j);
    if idle(h, *f.count, v1, v2) {
        return;
    }
    let mut pending = true;
    while pending {
        let dir = *f.dir;
        let two = *f.count == 2;
        solve_normal(ref h.a, dir, f.a.n, f.wn, ref v1, ref v2);
        if two {
            solve_normal(ref h.b, dir, f.b.n, f.wn, ref v1, ref v2);
        }
        let t = tangent(dir);
        let limit = *f.limit;
        solve_tangent(ref h.a, t, f.a.t, f.wt, limit * h.a.impulse, ref v1, ref v2);
        if two {
            solve_tangent(ref h.b, t, f.b.t, f.wt, limit * h.b.impulse, ref v1, ref v2);
        }
        pending = false;
    }
    bodies.set_vels(*f.i, v1, *f.j, v2);
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
    ref h: HotPoint,
    f: @FrozenPoint,
    dir: Vec2,
    w: @WeightedPair,
    ref v1: SolverVel,
    ref v2: SolverVel,
) {
    if *f.seed < ZERO && h.acc + h.impulse > ZERO {
        h.rhs = *f.seed;
        h.cfm = ONE;
        solve_normal(ref h, dir, f.n, w, ref v1, ref v2);
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
        let mut pending = true;
        while pending {
            let dir = *f.dir;
            bounce(ref h.a, f.a, dir, f.wn, ref v1, ref v2);
            if two {
                bounce(ref h.b, f.b, dir, f.wn, ref v1, ref v2);
            }
            pending = false;
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
    let mut next = frozen.pop_front();
    while let Some(mut m) = manifolds.pop_front() {
        if let Some(f) = next {
            if *f.manifold_id == id {
                let h = *hot.pop_front().unwrap();
                write_point(h.a, f.a, ref m);
                if *f.count == 2 {
                    write_point(h.b, f.b, ref m);
                }
                next = frozen.pop_front();
            }
        }
        out.append(m);
        id += 1;
    }
    manifolds = out;
}

#[inline(always)]
fn pose(poses: Span<Pose2>, i: u32) -> Pose2 {
    if i == WORLD {
        Default::default()
    } else {
        *poses.at(i)
    }
}

mod bodies;
pub(crate) use bodies::{SweepBodies, SweepBodiesTrait};
