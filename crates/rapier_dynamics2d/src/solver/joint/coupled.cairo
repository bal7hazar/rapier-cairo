//! RJ: coupled linear rows along the anchor separation, as upstream's `limit_linear_coupled` /
//! `motor_linear_coupled` (rope and spring joints). Upstream emits no coupled angular row in 2D
//! (`limit_angular_coupled` is 3D only, the coupled angular motor is a TODO), so coupled angular
//! axes get no row at all, as upstream.
//! The direction is the separation divided by its length: one wide square root and one wide
//! reciprocal (products round to nearest, the length floors); a zero separation gives a zero
//! Jacobian, as upstream's `simd_inv(0) = 0`. Other products floor, as JL.
use core::num::traits::DivRem;
use fixed::wide::{NormTrait, RecipTrait, dot2, norm2_wide};
use fixed::{Fixed, FixedTrait, MAX, MIN, ONE, ZERO};
use rapier_math::math_ext::{gcross_vv, inv};
use crate::joint::{GenericJoint, JointMotor, MotorModel};
use super::*;
use super::bounded::BoundedRow;
use super::row::{finish, metric, project, scale};

/// Coupled linear rows of a joint, constant during a step. `axis` is the first coupled linear
/// axis (upstream's `first_coupled_lin_axis_id`: its motor/limit data and impulse slots are used),
/// `bits` the coupled linear mask (1 = X, 2 = Y, 3 = both). `limit` is that axis' `[min, max]`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub(crate) struct CoupledControl {
    pub axis: u8,
    pub bits: u8,
    pub motor: Option<JointMotor>,
    pub limit: Option<(Fixed, Fixed)>,
}

/// Coupled rows of a joint with valid masks, upstream's selection: a motor when any coupled,
/// unlocked linear axis has its motor enabled (with the first coupled axis' data), a limit when
/// the first coupled axis is unlocked and limited. `None` when neither exists.
pub(crate) fn coupled(data: GenericJoint) -> Option<CoupledControl> {
    let (axis, motor, limit) = flags(
        data.coupled_axes.bits, data.locked_axes.bits, data.motor_axes.bits, data.limit_axes.bits,
    );
    if !motor && !limit {
        return None;
    }
    let [mx, my, _] = data.motors;
    let [lx, ly, _] = data.limits;
    let (m, l) = if axis == 0 {
        (mx, lx)
    } else {
        (my, ly)
    };
    Some(
        CoupledControl {
            axis,
            bits: if axis == 0 && !has_y(data.coupled_axes.bits) {
                1
            } else if axis == 0 {
                3
            } else {
                2
            },
            motor: if motor {
                Some(m)
            } else {
                None
            },
            limit: if limit {
                Some((l.min, l.max))
            } else {
                None
            },
        },
    )
}
#[inline(always)]
fn has_y(mask: u8) -> bool {
    let (q, _) = DivRem::div_rem(mask, 2);
    let (_, y) = DivRem::div_rem(q, 2);
    y != 0
}
/// Linear X/Y bits of a mask (0..7), without bitwise builtins.
#[inline(always)]
fn xy(mask: u8) -> (bool, bool) {
    let (q, x) = DivRem::div_rem(mask, 2);
    let (_, y) = DivRem::div_rem(q, 2);
    (x != 0, y != 0)
}
/// Whether a free (neither locked nor coupled) axis has a limit or motor (valid masks), without
/// bitwise builtins (which every caller up to the pipeline would then have to thread).
pub(crate) fn free_controls(data: GenericJoint) -> bool {
    let (cx, cy) = xy(data.coupled_axes.bits);
    let (lx, ly) = xy(data.locked_axes.bits);
    let (mx, my) = xy(data.motor_axes.bits);
    let (bx, by) = xy(data.limit_axes.bits);
    let angular = (data.motor_axes.bits >= 4 || data.limit_axes.bits >= 4)
        && data.locked_axes.bits < 4
        && data.coupled_axes.bits < 4;
    angular || ((mx || bx) && !lx && !cx) || ((my || by) && !ly && !cy)
}
/// First coupled linear axis and whether the coupled motor / limit exist (small arguments: the
/// selection is pre-paid in Sierra gas by every joint `specialise` handles).
fn flags(coupled: u8, locked: u8, motors: u8, limits: u8) -> (u8, bool, bool) {
    let (cx, cy) = xy(coupled);
    let (lx, ly) = xy(locked);
    let (mx, my) = xy(motors);
    let (bx, by) = xy(limits);
    let motor = (cx && mx && !lx) || (cy && my && !ly);
    if cx {
        (0, motor, bx && !lx)
    } else {
        (1, motor, cy && by && !ly)
    }
}

/// Coupled separation: the projection of the frame error on the coupled axes (the error itself
/// when both are coupled, exactly as upstream's sum up to rounding).
#[inline(always)]
fn separation(h: JointConstraintHelper, bits: u8) -> Vec2 {
    match bits {
        1 => scale(h.x, dot2(h.x.x, h.lin_err.x, h.x.y, h.lin_err.y)),
        2 => scale(h.y, dot2(h.y.x, h.lin_err.x, h.y.y, h.lin_err.y)),
        _ => h.lin_err,
    }
}

/// Unit-direction row (no rhs, softness or bounds yet) and the separation length.
#[inline(always)]
pub(crate) fn base(
    h: JointConstraintHelper, bits: u8, axis: u8, b1: SolverBody, b2: SolverBody,
) -> (JointGenericConstraint, Fixed) {
    let v = separation(h, bits);
    let n = norm2_wide(v.x, v.y);
    let dir = match n.try_recip() {
        Some(r) => Vec2 { x: r.mul(v.x), y: r.mul(v.y) },
        None => Default::default(),
    };
    let a1 = gcross_vv(h.r1.x, h.r1.y, dir.x, dir.y);
    let a2 = gcross_vv(h.r2.x, h.r2.y, dir.x, dir.y);
    (
        JointGenericConstraint {
            lin_jac: dir,
            ang_jac1: a1,
            ang_jac2: a2,
            ii_ang_jac1: b1.ii * a1,
            ii_ang_jac2: b2.ii * a2,
            axis,
            ..Default::default(),
        },
        n.to_fixed(),
    )
}

/// Coupled limit on `[.., max]` (upstream ignores `min`, FIXME upstream): impulse in `[0, MAX]`,
/// speculative `min(dist - max, 0) / dt` kept by `remove_bias`, plus the clamped ERP bias past
/// `max`. Always emitted (a slack row only acts when the separation velocity would overshoot
/// `max` within the substep). `(dist - max) / dt` must fit Fixed.
#[inline(always)]
pub(crate) fn limit_row(
    mut row: JointGenericConstraint,
    dist: Fixed,
    max: Fixed,
    p: IntegrationParameters,
    erp: Fixed,
    cfm: Fixed,
) -> BoundedRow {
    let gap = dist - max;
    let (speculative, error) = if gap < ZERO {
        (gap * p.substep_inv_dt(), ZERO)
    } else {
        (ZERO, gap)
    };
    let max_corrective = p.max_corrective_velocity();
    let bias = (error * erp).min(max_corrective);
    row.rhs_wo_bias = speculative;
    row.rhs = speculative + bias;
    row.cfm_coeff = cfm;
    row.erp_inv_dt = erp;
    BoundedRow { row, min: ZERO, max: MAX }
}

/// Coupled motor toward `target_pos` along the separation (spring), both motor models, target
/// velocity clamped by the coupled limit as upstream. Nonnegative stiffness/damping/force.
#[inline(always)]
pub(crate) fn motor_row(
    mut row: JointGenericConstraint,
    dist: Fixed,
    m: JointMotor,
    limit: Option<(Fixed, Fixed)>,
    p: IntegrationParameters,
) -> BoundedRow {
    // JL's `bounded::motor_row` coefficients, same operation order.
    let dt = p.substep_dt();
    assert(m.stiffness >= ZERO && m.damping >= ZERO && m.max_force >= ZERO, errors::NEGATIVE);
    let erp = m.stiffness * inv(dt * m.stiffness + m.damping);
    let cfm = inv(dt * dt * m.stiffness + dt * m.damping);
    match m.model {
        MotorModel::AccelerationBased => row.cfm_coeff = cfm,
        MotorModel::ForceBased => row.cfm_gain = cfm,
    }
    let mut rhs = ZERO;
    if erp != ZERO {
        rhs = (dist - m.target_pos) * erp;
    }
    let mut target = m.target_vel;
    if let Some((min, max)) = limit {
        // Sentinel bounds stay unbounded (MIN/MAX would overflow once divided by dt).
        let lo = if min == MIN {
            MIN
        } else {
            (min - dist) * p.substep_inv_dt()
        };
        let hi = if max == MAX {
            MAX
        } else {
            (max - dist) * p.substep_inv_dt()
        };
        target = target.max(lo).min(hi);
    }
    row.rhs = rhs - target;
    row.rhs_wo_bias = row.rhs;
    row.erp_inv_dt = erp;
    let max = if m.max_force == MAX && dt > ONE {
        MAX
    } else {
        m.max_force * dt
    };
    BoundedRow { row, min: -max, max }
}

/// Coupled motor (after the free-axis motors) and limit (after the free-axis limits), in
/// upstream order, finalised as JL: the motor is projected out of prior unbounded motors, the
/// limit out of the locks; each row is seeded from its first-coupled-axis impulse slot.
#[inline(always)]
pub(crate) fn rows(
    control: CoupledControl,
    ref motors: Array<BoundedRow>,
    ref limits: Array<BoundedRow>,
    c: JointConstraint,
    inverses: (Fixed, Fixed, Fixed),
    motor_seeds: [Fixed; 3],
    limit_seeds: [Fixed; 3],
    h: JointConstraintHelper,
    b1: SolverBody,
    b2: SolverBody,
    p: IntegrationParameters,
    erp: Fixed,
    cfm: Fixed,
) {
    let CoupledControl { axis, bits, motor, limit } = control;
    let imsum = c.im1 + c.im2;
    let (row, dist) = base(h, bits, axis, b1, b2);
    if let Some(m) = motor {
        let mut r = motor_row(row, dist, m, limit, p);
        if motors.len() != 0 {
            for prior in motors.span() {
                let prior: BoundedRow = *prior;
                if prior.min == -MAX && prior.max == MAX {
                    project(ref r.row, prior.row, imsum, inv(metric(prior.row, prior.row, imsum)));
                }
            }
        }
        let _ = finish(ref r.row, imsum);
        if p.warmstart_joints {
            r.row.impulse = seed_row(axis, motor_seeds) * p.warmstart_coefficient;
        }
        motors.append(r);
    }
    if let Some((_, max)) = limit {
        // An unset upper bound (the MAX sentinel of upstream's infinity) can never act.
        if max != MAX {
            let mut r = limit_row(row, dist, max, p, erp, cfm);
            super::step::project_locks(ref r.row, c, inverses, imsum);
            let _ = finish(ref r.row, imsum);
            if p.warmstart_joints {
                r.row.impulse = seed_row(axis, limit_seeds) * p.warmstart_coefficient;
            }
            limits.append(r);
        }
    }
}

#[cfg(test)]
pub(crate) mod alternatives;
#[cfg(test)]
mod tests;
