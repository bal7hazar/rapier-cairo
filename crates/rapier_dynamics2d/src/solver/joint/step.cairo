//! JM: the joint kind and its step constants are specialised once per step (island driver), and
//! limit/motor rows are built from them after a single frame construction (also the public path).
//! The island path additionally omits an interior limit whose current and step-initial impulses
//! are both zero: that row would clamp to a zero impulse (no velocity change, zero written back),
//! so every velocity and persisted impulse stays bit-identical. Products floor; reciprocals round
//! to nearest; the angular range centre/half are halved toward zero, as JL.
use fixed::trig::TrigTrait;
use fixed::wide::{dot2, mul_sub};
use fixed::{Fixed, FixedTrait, MAX, ONE, PI, ZERO};
use rapier_core::integration_parameters::spring::SpringCoefficients;
use rapier_math::math_ext::inv;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use crate::joint::{GenericJoint, JointAxesMask, JointLimits, JointMotor};
use super::*;
use super::bounded::{BoundedRow, BoundedRows, base, boxed, limit_row, motor_row};
use super::helper::{finalize2_inverses, finalize3_inverses};
use super::row::{finish, gas_wallet, metric, project};

/// Angular margin (2^-16 rad) of the conservative interior test, far above trig/rounding error.
const INSIDE_MARGIN: Fixed = Fixed { raw: 65536 };

/// Step-constant part of an enabled joint: CoM-local frames, lock mask and softness.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub(crate) struct StepJoint {
    pub frame1: Pose2,
    pub frame2: Pose2,
    pub locks: JointAxesMask,
    pub softness: SpringCoefficients,
}
/// Row kind of a joint, constant during a step.
#[derive(Copy, Drop, Debug)]
pub(crate) enum StepKind {
    /// Bilateral locks only (no limit and no motor axis).
    Plain,
    /// Limits/motors on free, uncoupled axes; every mask valid.
    Controlled: Box<StepControls>,
    /// Limits/motors with an out-of-range mask: JL's per-substep path, same panics.
    Legacy: Box<ImpulseJoint>,
}
/// Limit range of one axis. `mode`: 0 linear, 1 angular, 2 angular spanning a full turn
/// (never active). Angular: sine/cosine of the range centre, half-range, and the half-range
/// less a margin for the interior test (zero disables it).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub(crate) struct StepLimit {
    pub min: Fixed,
    pub max: Fixed,
    pub mode: u8,
    pub sin: Fixed,
    pub cos: Fixed,
    pub half: Fixed,
    pub inside: Fixed,
}
/// Motor and/or limit of one free, uncoupled axis.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub(crate) struct AxisControl {
    pub axis: u8,
    pub motor: Option<JointMotor>,
    pub limit: Option<StepLimit>,
}
/// Controlled axes in angular, X, Y order, and the step-initial motor/limit impulses per DOF.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub(crate) struct StepControls {
    pub axes: Span<AxisControl>,
    pub motors: [Fixed; 3],
    pub limits: [Fixed; 3],
}

/// Specialise a joint whose frames are already CoM-local.
pub(crate) fn specialise(joint: ImpulseJoint) -> StepKind {
    let data = joint.data;
    if data.limit_axes.bits == 0 && data.motor_axes.bits == 0 {
        return StepKind::Plain;
    }
    if data.locked_axes.bits > 7
        || data.coupled_axes.bits > 7
        || data.limit_axes.bits > 7
        || data.motor_axes.bits > 7 {
        return StepKind::Legacy(BoxTrait::new(joint));
    }
    let controls = controls(data, true);
    if controls.axes.is_empty() {
        StepKind::Plain
    } else {
        StepKind::Controlled(BoxTrait::new(controls))
    }
}
/// Controlled axes, with JL's mask checks in JL's order. `inside` enables the interior test.
/// Unrolled over the three axes: a loop would pass the whole joint per iteration.
pub(crate) fn controls(data: GenericJoint, inside: bool) -> StepControls {
    gas_wallet();
    let mut axes = array![];
    let [lx, ly, lw] = data.limits;
    let [mx, my, mw] = data.motors;
    let masks = (data.locked_axes, data.coupled_axes, data.motor_axes, data.limit_axes);
    control(ref axes, masks, 2, mw, lw, inside);
    control(ref axes, masks, 0, mx, lx, inside);
    control(ref axes, masks, 1, my, ly, inside);
    StepControls {
        axes: axes.span(),
        motors: [mx.impulse, my.impulse, mw.impulse],
        limits: [lx.impulse, ly.impulse, lw.impulse],
    }
}
#[inline(always)]
fn control(
    ref axes: Array<AxisControl>,
    masks: (JointAxesMask, JointAxesMask, JointAxesMask, JointAxesMask),
    axis: u8,
    m: JointMotor,
    l: JointLimits,
    inside: bool,
) {
    let (locked, coupled, motors, limits) = masks;
    if !locked.contains_axis(axis) && !coupled.contains_axis(axis) {
        let motor = if motors.contains_axis(axis) {
            Some(m)
        } else {
            None
        };
        let limit = if limits.contains_axis(axis) {
            if axis == 2 {
                Some(angular(l, inside))
            } else {
                Some(
                    StepLimit {
                        min: l.min,
                        max: l.max,
                        mode: 0,
                        sin: ZERO,
                        cos: ZERO,
                        half: ZERO,
                        inside: ZERO,
                    },
                )
            }
        } else {
            None
        };
        if motor.is_some() || limit.is_some() {
            axes.append(AxisControl { axis, motor, limit });
        }
    }
}
#[inline(never)]
fn angular(l: JointLimits, inside: bool) -> StepLimit {
    // Widen before subtracting: the default [MIN, MAX] must disable, not overflow.
    let range: i128 = l.max.raw.into() - l.min.raw.into();
    if range >= 2 * PI.raw.into() {
        return StepLimit {
            min: l.min, max: l.max, mode: 2, sin: ZERO, cos: ZERO, half: ONE, inside: ZERO,
        };
    }
    let center: i128 = (l.min.raw.into() + l.max.raw.into()) / 2;
    let half: i128 = range / 2;
    let center = Fixed { raw: center.try_into().expect('Joint: angle range') };
    let half = Fixed { raw: half.try_into().expect('Joint: angle range') };
    let (sin, cos) = center.sin_cos();
    let threshold = if inside && half > INSIDE_MARGIN {
        half - INSIDE_MARGIN
    } else {
        ZERO
    };
    StepLimit { min: l.min, max: l.max, mode: 1, sin, cos, half, inside: threshold }
}

/// Island bilateral generation: public `generate_plain` arithmetic without body resolution.
pub(crate) fn plain(
    step: StepJoint, seeds: [Fixed; 3], b1: SolverBody, b2: SolverBody, p: IntegrationParameters,
) -> JointConstraint {
    gas_wallet();
    let (h, erp, cfm, _, _) = frame(b1, b2, step.frame1, step.frame2, step.locks, step.softness, p);
    let mut c: JointConstraint = Default::default();
    c.im1 = b1.im;
    c.im2 = b2.im;
    lock_rows(ref c, h, b1, b2, erp, cfm, step.locks);
    JointConstraintHelperTrait::finalize(ref c);
    seed(ref c, seeds, p);
    c
}

/// Locks, then motors and limits (JL order and arithmetic) from one frame construction.
/// `seeds`/`motor_seeds`/`limit_seeds` are the current per-DOF impulses; `skip` omits interior
/// limits whose current and step-initial (`controls.limits`) impulses are zero.
pub(crate) fn generate(
    step: StepJoint,
    controls: StepControls,
    seeds: [Fixed; 3],
    motor_seeds: [Fixed; 3],
    limit_seeds: [Fixed; 3],
    b1: SolverBody,
    b2: SolverBody,
    p: IntegrationParameters,
    skip: bool,
) -> JointConstraint {
    gas_wallet();
    let (h, erp, cfm, r1, r2) = frame(
        b1, b2, step.frame1, step.frame2, step.locks, step.softness, p,
    );
    let mut c: JointConstraint = Default::default();
    c.im1 = b1.im;
    c.im2 = b2.im;
    lock_rows(ref c, h, b1, b2, erp, cfm, step.locks);
    let inverses = finalize_inverses(ref c);
    seed(ref c, seeds, p);
    let mut motors: Array<BoundedRow> = array![];
    let mut limits: Array<BoundedRow> = array![];
    let mut axes = controls.axes;
    // Revolute and prismatic joints have one free axis: no loop (a loop passes every live
    // value per iteration). Free joints loop over the remaining axes.
    if let Some(control) = axes.pop_front() {
        axis_rows(
            *control,
            ref motors,
            ref limits,
            c,
            inverses,
            controls.limits,
            motor_seeds,
            limit_seeds,
            h,
            b1,
            b2,
            p,
            erp,
            cfm,
            r1,
            r2,
            skip,
        );
    }
    if axes.len() != 0 {
        while let Some(control) = axes.pop_front() {
            axis_rows(
                *control,
                ref motors,
                ref limits,
                c,
                inverses,
                controls.limits,
                motor_seeds,
                limit_seeds,
                h,
                b1,
                b2,
                p,
                erp,
                cfm,
                r1,
                r2,
                skip,
            );
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
    c
}
/// Motor then limit rows of one controlled axis (JL order and arithmetic).
#[inline(always)]
fn axis_rows(
    control: AxisControl,
    ref motors: Array<BoundedRow>,
    ref limits: Array<BoundedRow>,
    c: JointConstraint,
    inverses: (Fixed, Fixed, Fixed),
    initial_limits: [Fixed; 3],
    motor_seeds: [Fixed; 3],
    limit_seeds: [Fixed; 3],
    h: JointConstraintHelper,
    b1: SolverBody,
    b2: SolverBody,
    p: IntegrationParameters,
    erp: Fixed,
    cfm: Fixed,
    r1: Rot2,
    r2: Rot2,
    skip: bool,
) {
    let AxisControl { axis, motor, limit } = control;
    let imsum = c.im1 + c.im2;
    if let Some(m) = motor {
        let range = match limit {
            Some(l) => if axis != 2 {
                Some((l.min, l.max))
            } else {
                None
            },
            None => None,
        };
        let mut r = motor_row(h, m, range, axis, b1, b2, p, r1, r2);
        if motors.len() != 0 {
            // Only a truly unbounded prior motor can be projected out (upstream).
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
    if let Some(l) = limit {
        let stored = seed_row(axis, limit_seeds);
        let may_skip = skip && stored == ZERO && seed_row(axis, initial_limits) == ZERO;
        let (dist, lo, hi, inside) = distance(l, axis, h, r1, r2, may_skip);
        if !(may_skip && (inside || (dist > lo && dist < hi))) {
            let mut r = limit_row(base(h, axis, b1, b2), dist, lo, hi, p, erp, cfm);
            project_locks(ref r.row, c, inverses, imsum);
            let _ = finish(ref r.row, imsum);
            if p.warmstart_joints {
                r.row.impulse = stored * p.warmstart_coefficient;
            }
            limits.append(r);
        }
    }
}
/// Signed limit distance and range; `inside` when the angular shortcut proves an interior
/// angle (then no atan2 is evaluated and the distance is unused).
#[inline(always)]
fn distance(
    l: StepLimit, axis: u8, h: JointConstraintHelper, r1: Rot2, r2: Rot2, may_skip: bool,
) -> (Fixed, Fixed, Fixed, bool) {
    match l.mode {
        0 => {
            let jac = if axis == 0 {
                h.x
            } else {
                h.y
            };
            (dot2(jac.x, h.lin_err.x, jac.y, h.lin_err.y), l.min, l.max, false)
        },
        1 => {
            let r = r1.inverse() * r2;
            if may_skip && interior(l, r) {
                (ZERO, -l.half, l.half, true)
            } else {
                (atan2_centered(l, r), -l.half, l.half, false)
            }
        },
        _ => (ZERO, -ONE, ONE, false),
    }
}
/// Conservative interior test without trigonometry: with `x`, `y` the cosine and sine of the
/// angle `phi` from the range centre, `x > 0` and `|y| < (half - margin) x` give
/// `|phi| <= |tan phi| < half - margin` (`atan t <= t`); the margin (2^-16 rad) dominates every
/// floor and trig error, so JL's `atan2` distance is then strictly inside the range too.
#[inline(always)]
fn interior(l: StepLimit, r: Rot2) -> bool {
    let x = dot2(l.cos, r.re, l.sin, r.im);
    x > ZERO && mul_sub(l.cos, r.im, l.sin, r.re).abs() < l.inside * x
}
#[inline(never)]
fn atan2_centered(l: StepLimit, r: Rot2) -> Fixed {
    (l.cos * r.im - l.sin * r.re).atan2(l.cos * r.re + l.sin * r.im)
}
/// Project a limit row out of the finalized locks, reusing their cached inverse masses.
#[inline(always)]
fn project_locks(
    ref row: JointGenericConstraint,
    c: JointConstraint,
    inverses: (Fixed, Fixed, Fixed),
    imsum: Vec2,
) {
    let [a, b, d] = c.rows;
    let (ia, ib, id) = inverses;
    if c.num_rows != 0 {
        project(ref row, a, imsum, ia);
    }
    if c.num_rows >= 2 {
        project(ref row, b, imsum, ib);
    }
    if c.num_rows == 3 {
        project(ref row, d, imsum, id);
    }
}
/// `finalize`, also returning each lock's `inv(metric(row, row))` (what `finish` returns).
#[inline(always)]
fn finalize_inverses(ref c: JointConstraint) -> (Fixed, Fixed, Fixed) {
    let imsum = c.im1 + c.im2;
    let [mut a, mut b, mut d] = c.rows;
    let inverses = match c.num_rows {
        0 => (ZERO, ZERO, ZERO),
        1 => (finish(ref a, imsum), ZERO, ZERO),
        2 => {
            let (ia, ib) = finalize2_inverses(ref a, ref b, imsum);
            (ia, ib, ZERO)
        },
        _ => finalize3_inverses(ref a, ref b, ref d, imsum),
    };
    c.rows = [a, b, d];
    inverses
}

/// Per-DOF lock, motor and limit impulses left by `old` over the lock impulses `locks`, as
/// `writeback_impulses` would persist them; an omitted limit reads zero.
#[inline(always)]
pub(crate) fn carried(
    old: JointConstraint, mut locks: [Fixed; 3],
) -> ([Fixed; 3], [Fixed; 3], [Fixed; 3]) {
    let mut motors = [ZERO, ZERO, ZERO];
    let mut limits = [ZERO, ZERO, ZERO];
    if old.num_rows == 4 {
        let extra = old.bounded.unwrap().data.unbox();
        write_rows(old.rows, extra.locks, ref locks);
        write_bounded(extra.motors, ref motors);
        write_bounded(extra.limits, ref limits);
    } else {
        write_rows(old.rows, old.num_rows, ref locks);
    }
    (locks, motors, limits)
}
fn write_bounded(mut rows: Span<BoundedRow>, ref impulses: [Fixed; 3]) {
    let [mut x, mut y, mut w] = impulses;
    while let Some(r) = rows.pop_front() {
        write(*r.row.axis, *r.row.impulse, ref x, ref y, ref w);
    }
    impulses = [x, y, w];
}

#[cfg(test)]
pub(crate) mod alternatives;
#[cfg(test)]
mod tests;
