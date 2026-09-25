//! Optional bounded rows. Motors form an independent group before locks/limits, as upstream.
//! Only bilateral locks project onto limits. Products floor; reciprocals round to nearest.
#[cfg(test)]
use fixed::PI;
use fixed::trig::TrigTrait;
use fixed::wide::{dot2, dot4};
use fixed::{Fixed, FixedTrait, MAX, MIN, ONE, TAU, ZERO};
use rapier_math::math_ext::inv;
use rapier_math::rot2::Rot2;
use crate::joint::{GenericJoint, JointLimits, JointMotor, MotorModel};
use super::*;
use super::super::body::SolverVel;

/// One unilateral limit or force-bounded motor, in the bilateral row sign convention.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct BoundedRow {
    pub row: JointGenericConstraint,
    pub min: Fixed,
    pub max: Fixed,
}
/// Optional rows, allocated only for enabled free, uncoupled limit/motor axes.
/// Order is angular, X, Y within each group; motors precede locks, which precede limits.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct BoundedRows {
    pub motors: Span<BoundedRow>,
    pub limits: Span<BoundedRow>,
    pub locks: u8,
}

/// Boxed optional row state; manual serialization/equality delegate to the value payload.
#[derive(Copy, Drop, Debug)]
pub struct BoundedState {
    pub data: Box<BoundedRows>,
}
pub impl BoundedStateSerde of Serde<BoundedState> {
    fn serialize(self: @BoundedState, ref output: Array<felt252>) {
        let rows = (*self.data).unbox();
        rows.serialize(ref output);
    }
    fn deserialize(ref serialized: Span<felt252>) -> Option<BoundedState> {
        Some(
            BoundedState {
                data: BoxTrait::new(Serde::<BoundedRows>::deserialize(ref serialized)?),
            },
        )
    }
}
pub impl BoundedStatePartialEq of PartialEq<BoundedState> {
    fn eq(lhs: @BoundedState, rhs: @BoundedState) -> bool {
        (*lhs.data).unbox() == (*rhs.data).unbox()
    }
    fn ne(lhs: @BoundedState, rhs: @BoundedState) -> bool {
        !Self::eq(lhs, rhs)
    }
}
pub(crate) fn boxed(rows: BoundedRows) -> Option<BoundedState> {
    Some(BoundedState { data: BoxTrait::new(rows) })
}
#[inline(always)]
pub(crate) fn base(
    h: JointConstraintHelper, axis: u8, b1: SolverBody, b2: SolverBody,
) -> JointGenericConstraint {
    if axis == 2 {
        h.lock_angular(b1, b2, ZERO, ZERO)
    } else {
        h.lock_linear(axis, b1, b2, ZERO, ZERO)
    }
}
pub(crate) fn get_limit(j: GenericJoint, axis: u8) -> JointLimits {
    let [x, y, w] = j.limits;
    match axis {
        0 => x,
        1 => y,
        _ => w,
    }
}
pub(crate) fn get_motor(j: GenericJoint, axis: u8) -> JointMotor {
    let [x, y, w] = j.motors;
    match axis {
        0 => x,
        1 => y,
        _ => w,
    }
}
fn clamp(x: Fixed, min: Fixed, max: Fixed) -> Fixed {
    x.max(min).min(max)
}
/// Limit row from its signed distance and range: bias only outside the range, impulse bounds
/// opened on the violated side (`dist <= lo` / `dist >= hi`), as upstream.
#[inline(always)]
pub(crate) fn limit_row(
    mut row: JointGenericConstraint,
    dist: Fixed,
    lo: Fixed,
    hi: Fixed,
    p: IntegrationParameters,
    erp: Fixed,
    cfm: Fixed,
) -> BoundedRow {
    // Branching avoids overflowing dist-MIN / MAX-dist on inactive finite sentinel bounds.
    let high_error = if dist > hi {
        dist - hi
    } else {
        ZERO
    };
    let low_error = if dist < lo {
        lo - dist
    } else {
        ZERO
    };
    let max_corrective = p.max_corrective_velocity();
    row.rhs = clamp((high_error - low_error) * erp, -max_corrective, max_corrective);
    row.cfm_coeff = cfm;
    row.erp_inv_dt = erp;
    BoundedRow {
        row, min: if dist <= lo {
            MIN
        } else {
            ZERO
        }, max: if dist >= hi {
            MAX
        } else {
            ZERO
        },
    }
}
/// Motor row. `limit` is the same free linear axis' range (clamps the target velocity);
/// `r1`/`r2` are the world frame rotations (angular position error). Nonnegative inputs.
pub(crate) fn motor_row(
    h: JointConstraintHelper,
    m: JointMotor,
    limit: Option<(Fixed, Fixed)>,
    axis: u8,
    b1: SolverBody,
    b2: SolverBody,
    p: IntegrationParameters,
    r1: Rot2,
    r2: Rot2,
) -> BoundedRow {
    let dt = p.substep_dt();
    assert(m.stiffness >= ZERO && m.damping >= ZERO && m.max_force >= ZERO, errors::NEGATIVE);
    let mut row = base(h, axis, b1, b2);
    let erp = m.stiffness * inv(dt * m.stiffness + m.damping);
    // Preserve upstream operation order (not dt * (dt * stiffness + damping)).
    let cfm = inv(dt * dt * m.stiffness + dt * m.damping);
    match m.model {
        MotorModel::AccelerationBased => row.cfm_coeff = cfm,
        MotorModel::ForceBased => row.cfm_gain = cfm,
    }
    let mut rhs = ZERO;
    // Metered: velocity motors (erp = 0) do not pay the position error (angular: atan2).
    let mut pending = erp != ZERO;
    while pending {
        let error = if axis == 2 {
            let r = r1.inverse() * r2;
            let error = r.im.atan2(r.re) - m.target_pos;
            let complement = error - if error >= ZERO {
                TAU
            } else {
                -TAU
            };
            if error.abs() < complement.abs() {
                error
            } else {
                complement
            }
        } else {
            dot2(row.lin_jac.x, h.lin_err.x, row.lin_jac.y, h.lin_err.y) - m.target_pos
        };
        rhs = error * erp;
        pending = false;
    }
    let mut target = m.target_vel;
    if let Some((min, max)) = limit {
        let dist = dot2(row.lin_jac.x, h.lin_err.x, row.lin_jac.y, h.lin_err.y);
        target =
            clamp(
                target,
                if min == MIN {
                    MIN
                } else {
                    (min - dist) * p.substep_inv_dt()
                },
                if max == MAX {
                    MAX
                } else {
                    (max - dist) * p.substep_inv_dt()
                },
            );
    }
    row.rhs = rhs - target;
    row.rhs_wo_bias = row.rhs;
    row.erp_inv_dt = erp;
    // Finite MAX represents upstream's unbounded default; avoid MAX * dt overflow for dt>1.
    let max = if m.max_force == MAX && dt > ONE {
        MAX
    } else {
        m.max_force * dt
    };
    BoundedRow { row, min: -max, max }
}

#[inline(always)]
fn solve_bounded(
    ref r: BoundedRow, im1: Vec2, im2: Vec2, ref v1: SolverVel, ref v2: SolverVel, biased: bool,
) {
    let mut a = r.row;
    if !biased {
        a.rhs = a.rhs_wo_bias;
    }
    let dv = v2.linear - v1.linear;
    let rhs = dot4(
        a.lin_jac.x, dv.x, a.lin_jac.y, dv.y, a.ang_jac2, v2.angular, -a.ang_jac1, v1.angular,
    )
        + a.rhs;
    let total = clamp(a.impulse + a.inv_lhs * (rhs - a.cfm_gain * a.impulse), r.min, r.max);
    let delta = total - a.impulse;
    a.impulse = total;
    apply(a, delta, im1, im2, ref v1, ref v2);
    r.row = a;
}
/// Solve bounded rows in order, returning the updated rows (inverse masses only are passed).
fn sweep(
    mut rows: Span<BoundedRow>,
    im1: Vec2,
    im2: Vec2,
    ref v1: SolverVel,
    ref v2: SolverVel,
    biased: bool,
) -> Span<BoundedRow> {
    let mut out = array![];
    while let Some(r) = rows.pop_front() {
        let mut r = *r;
        solve_bounded(ref r, im1, im2, ref v1, ref v2, biased);
        out.append(r);
    }
    out.span()
}
/// Apply seeded bounded impulses in order; rows are unchanged.
fn apply_rows(
    mut rows: Span<BoundedRow>, im1: Vec2, im2: Vec2, ref v1: SolverVel, ref v2: SolverVel,
) {
    while let Some(r) = rows.pop_front() {
        apply(*r.row, *r.row.impulse, im1, im2, ref v1, ref v2);
    }
}
/// Motors, then locks, then limits, as upstream. `warm` applies the seeded impulses instead
/// (see `warmstart`, which also skips re-boxing the unchanged rows).
pub(crate) fn solve(
    ref c: JointConstraint, ref bodies: Array<SolverBody>, biased: bool, warm: bool,
) {
    if warm {
        warmstart(c, ref bodies);
        return;
    }
    let mut extra = c.bounded.unwrap().data.unbox();
    let mut v1 = velocity(read(bodies.span(), c.solver_vel1));
    let mut v2 = velocity(read(bodies.span(), c.solver_vel2));
    if extra.motors.len() != 0 {
        extra.motors = sweep(extra.motors, c.im1, c.im2, ref v1, ref v2, biased);
    }
    let [mut a, mut b, mut d] = c.rows;
    if !biased {
        a.rhs = a.rhs_wo_bias;
        b.rhs = b.rhs_wo_bias;
        d.rhs = d.rhs_wo_bias;
    }
    if extra.locks != 0 {
        solve_row(ref a, c.im1, c.im2, ref v1, ref v2);
    }
    if extra.locks >= 2 {
        solve_row(ref b, c.im1, c.im2, ref v1, ref v2);
    }
    if extra.locks == 3 {
        solve_row(ref d, c.im1, c.im2, ref v1, ref v2);
    }
    c.rows = [a, b, d];
    if extra.limits.len() != 0 {
        extra.limits = sweep(extra.limits, c.im1, c.im2, ref v1, ref v2, biased);
    }
    c.bounded = boxed(extra);
    scatter(ref bodies, c.solver_vel1, v1, c.solver_vel2, v2);
}
/// Apply every seeded impulse once (motors, locks, limits); no division, products floor.
pub(crate) fn warmstart(c: JointConstraint, ref bodies: Array<SolverBody>) {
    let extra = c.bounded.unwrap().data.unbox();
    let mut v1 = velocity(read(bodies.span(), c.solver_vel1));
    let mut v2 = velocity(read(bodies.span(), c.solver_vel2));
    apply_rows(extra.motors, c.im1, c.im2, ref v1, ref v2);
    let [a, b, d] = c.rows;
    if extra.locks != 0 {
        apply(a, a.impulse, c.im1, c.im2, ref v1, ref v2);
    }
    if extra.locks >= 2 {
        apply(b, b.impulse, c.im1, c.im2, ref v1, ref v2);
    }
    if extra.locks == 3 {
        apply(d, d.impulse, c.im1, c.im2, ref v1, ref v2);
    }
    apply_rows(extra.limits, c.im1, c.im2, ref v1, ref v2);
    scatter(ref bodies, c.solver_vel1, v1, c.solver_vel2, v2);
}
pub(crate) fn remove_bias(ref c: JointConstraint) {
    let mut extra = c.bounded.unwrap().data.unbox();
    let mut rows = array![];
    for r in extra.limits {
        let mut r = *r;
        r.row.rhs = r.row.rhs_wo_bias;
        rows.append(r);
    }
    extra.limits = rows.span();
    c.bounded = boxed(extra);
}
pub(crate) fn writeback(c: JointConstraint, ref j: ImpulseJoint) {
    let extra = c.bounded.unwrap().data.unbox();
    for r in extra.motors {
        let [mut x, mut y, mut w] = j.data.motors;
        match *r.row.axis {
            0 => x.impulse = *r.row.impulse,
            1 => y.impulse = *r.row.impulse,
            _ => w.impulse = *r.row.impulse,
        }
        j.data.motors = [x, y, w];
    }
    for r in extra.limits {
        let [mut x, mut y, mut w] = j.data.limits;
        match *r.row.axis {
            0 => x.impulse = *r.row.impulse,
            1 => y.impulse = *r.row.impulse,
            _ => w.impulse = *r.row.impulse,
        }
        j.data.limits = [x, y, w];
    }
    let [a, b, d] = c.rows;
    let [mut x, mut y, mut w] = j.impulses;
    if extra.locks != 0 {
        super::write(a.axis, a.impulse, ref x, ref y, ref w);
    }
    if extra.locks >= 2 {
        super::write(b.axis, b.impulse, ref x, ref y, ref w);
    }
    if extra.locks == 3 {
        super::write(d.axis, d.impulse, ref x, ref y, ref w);
    }
    j.impulses = [x, y, w];
}

#[cfg(test)]
pub(crate) mod alternatives;

#[cfg(test)]
mod tests;
