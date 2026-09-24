//! Optional bounded rows. Motors form an independent group before locks/limits, as upstream.
//! Only bilateral locks project onto limits. Products floor; reciprocals round to nearest.
use fixed::trig::TrigTrait;
use fixed::wide::{dot2, dot4};
use fixed::{Fixed, FixedTrait, MAX, MIN, ONE, PI, TAU, ZERO};
use rapier_math::math_ext::inv;
use crate::joint::{GenericJoint, JointLimits, JointMotor, MotorModel};
use super::*;
use super::row::{finish, metric, project};
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
fn boxed(rows: BoundedRows) -> Option<BoundedState> {
    Some(BoundedState { data: BoxTrait::new(rows) })
}

pub(crate) fn generate(
    ref c: JointConstraint,
    j: ImpulseJoint,
    h: JointConstraintHelper,
    b1: SolverBody,
    b2: SolverBody,
    p: IntegrationParameters,
    erp: Fixed,
    cfm: Fixed,
) {
    let mut motors = array![];
    let mut limits = array![];
    for axis in [2_u8, 0, 1].span() {
        let axis = *axis;
        if !j.data.locked_axes.contains_axis(axis) && !j.data.coupled_axes.contains_axis(axis) {
            if j.data.motor_axes.contains_axis(axis) {
                let mut r = motor(h, j.data, axis, b1, b2, p);
                // Only a truly unbounded prior motor can be projected out (upstream).
                for prior in motors.span() {
                    let prior: BoundedRow = *prior;
                    if prior.min == -MAX && prior.max == MAX {
                        project(
                            ref r.row,
                            prior.row,
                            c.im1 + c.im2,
                            inv(metric(prior.row, prior.row, c.im1 + c.im2)),
                        );
                    }
                }
                let _ = finish(ref r.row, c.im1 + c.im2);
                if p.warmstart_joints {
                    r.row.impulse = get_motor(j.data, axis).impulse * p.warmstart_coefficient;
                }
                motors.append(r);
            }
            if j.data.limit_axes.contains_axis(axis) {
                let mut r = limit(h, j.data, axis, b1, b2, p, erp, cfm);
                let [a, b, d] = c.rows;
                if c.num_rows != 0 {
                    project(ref r.row, a, c.im1 + c.im2, inv(metric(a, a, c.im1 + c.im2)));
                }
                if c.num_rows >= 2 {
                    project(ref r.row, b, c.im1 + c.im2, inv(metric(b, b, c.im1 + c.im2)));
                }
                if c.num_rows == 3 {
                    project(ref r.row, d, c.im1 + c.im2, inv(metric(d, d, c.im1 + c.im2)));
                }
                let _ = finish(ref r.row, c.im1 + c.im2);
                if p.warmstart_joints {
                    r.row.impulse = get_limit(j.data, axis).impulse * p.warmstart_coefficient;
                }
                limits.append(r);
            }
        }
    }
    if motors.len() != 0 || limits.len() != 0 {
        c
            .bounded =
                boxed(
                    BoundedRows { motors: motors.span(), limits: limits.span(), locks: c.num_rows },
                );
        // The island adapter skips zero-row constraints. Four denotes this extended path.
        c.num_rows = 4;
    }
}
fn base(
    h: JointConstraintHelper, axis: u8, b1: SolverBody, b2: SolverBody,
) -> JointGenericConstraint {
    if axis == 2 {
        h.lock_angular(b1, b2, ZERO, ZERO)
    } else {
        h.lock_linear(axis, b1, b2, ZERO, ZERO)
    }
}
fn rotation(j: GenericJoint, b1: SolverBody, b2: SolverBody) -> rapier_math::rot2::Rot2 {
    (b1.position.rotation * j.local_frame1.rotation).inverse()
        * (b2.position.rotation * j.local_frame2.rotation)
}
fn get_limit(j: GenericJoint, axis: u8) -> JointLimits {
    let [x, y, w] = j.limits;
    match axis {
        0 => x,
        1 => y,
        _ => w,
    }
}
fn get_motor(j: GenericJoint, axis: u8) -> JointMotor {
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

fn limit(
    h: JointConstraintHelper,
    j: GenericJoint,
    axis: u8,
    b1: SolverBody,
    b2: SolverBody,
    p: IntegrationParameters,
    erp: Fixed,
    cfm: Fixed,
) -> BoundedRow {
    let l = get_limit(j, axis);
    let mut row = base(h, axis, b1, b2);
    let (dist, lo, hi) = if axis == 2 {
        // Widen before subtracting: the default [MIN, MAX] must disable, not overflow.
        let range: i128 = l.max.raw.into() - l.min.raw.into();
        if range >= 2 * PI.raw.into() {
            (ZERO, -ONE, ONE)
        } else {
            let center: i128 = (l.min.raw.into() + l.max.raw.into()) / 2;
            let half: i128 = range / 2;
            let center = Fixed { raw: center.try_into().expect('Joint: angle range') };
            let half = Fixed { raw: half.try_into().expect('Joint: angle range') };
            let (s, co) = center.sin_cos();
            let r = rotation(j, b1, b2);
            ((co * r.im - s * r.re).atan2(co * r.re + s * r.im), -half, half)
        }
    } else {
        (dot2(row.lin_jac.x, h.lin_err.x, row.lin_jac.y, h.lin_err.y), l.min, l.max)
    };
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
    row
        .rhs =
            clamp(
                (high_error - low_error) * erp,
                -p.max_corrective_velocity(),
                p.max_corrective_velocity(),
            );
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
fn motor(
    h: JointConstraintHelper,
    j: GenericJoint,
    axis: u8,
    b1: SolverBody,
    b2: SolverBody,
    p: IntegrationParameters,
) -> BoundedRow {
    let m = get_motor(j, axis);
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
    if erp != ZERO {
        let error = if axis == 2 {
            let r = rotation(j, b1, b2);
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
    }
    let mut target = m.target_vel;
    if axis != 2 && j.limit_axes.contains_axis(axis) {
        let l = get_limit(j, axis);
        let dist = dot2(row.lin_jac.x, h.lin_err.x, row.lin_jac.y, h.lin_err.y);
        target =
            clamp(
                target,
                if l.min == MIN {
                    MIN
                } else {
                    (l.min - dist) * p.substep_inv_dt()
                },
                if l.max == MAX {
                    MAX
                } else {
                    (l.max - dist) * p.substep_inv_dt()
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
fn sweep(
    mut rows: Span<BoundedRow>,
    c: JointConstraint,
    ref v1: SolverVel,
    ref v2: SolverVel,
    biased: bool,
    warm: bool,
) -> Span<BoundedRow> {
    let mut out = array![];
    while let Some(r) = rows.pop_front() {
        let mut r = *r;
        if warm {
            apply(r.row, r.row.impulse, c.im1, c.im2, ref v1, ref v2);
        } else {
            solve_bounded(ref r, c.im1, c.im2, ref v1, ref v2, biased);
        }
        out.append(r);
    }
    out.span()
}
pub(crate) fn solve(
    ref c: JointConstraint, ref bodies: Array<SolverBody>, biased: bool, warm: bool,
) {
    let mut extra = c.bounded.unwrap().data.unbox();
    let mut v1 = velocity(read(bodies.span(), c.solver_vel1));
    let mut v2 = velocity(read(bodies.span(), c.solver_vel2));
    extra.motors = sweep(extra.motors, c, ref v1, ref v2, biased, warm);
    let [mut a, mut b, mut d] = c.rows;
    if !biased && !warm {
        a.rhs = a.rhs_wo_bias;
        b.rhs = b.rhs_wo_bias;
        d.rhs = d.rhs_wo_bias;
    }
    if warm {
        if extra.locks != 0 {
            apply(a, a.impulse, c.im1, c.im2, ref v1, ref v2);
        }
        if extra.locks >= 2 {
            apply(b, b.impulse, c.im1, c.im2, ref v1, ref v2);
        }
        if extra.locks == 3 {
            apply(d, d.impulse, c.im1, c.im2, ref v1, ref v2);
        }
    } else {
        if extra.locks != 0 {
            solve_row(ref a, c.im1, c.im2, ref v1, ref v2);
        }
        if extra.locks >= 2 {
            solve_row(ref b, c.im1, c.im2, ref v1, ref v2);
        }
        if extra.locks == 3 {
            solve_row(ref d, c.im1, c.im2, ref v1, ref v2);
        }
    }
    c.rows = [a, b, d];
    extra.limits = sweep(extra.limits, c, ref v1, ref v2, biased, warm);
    c.bounded = boxed(extra);
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
mod alternatives;

#[cfg(test)]
mod tests;
