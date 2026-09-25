//! JM candidates kept for measurement: plain generation without the gas wallet (static Sierra
//! charge of the three-row arms), metered in a one-iteration loop, or inlined in a loop body;
//! controlled generation and specialisation looping over axes; the cosine interior test.
use fixed::{Fixed, MAX, ZERO};
use super::{*, StepJoint};

pub(crate) fn plain_static(
    step: StepJoint, seeds: [Fixed; 3], b1: SolverBody, b2: SolverBody, p: IntegrationParameters,
) -> JointConstraint {
    plain_inline(step, seeds, b1, b2, p)
}
pub(crate) fn plain_metered(
    step: StepJoint, seeds: [Fixed; 3], b1: SolverBody, b2: SolverBody, p: IntegrationParameters,
) -> JointConstraint {
    let mut c: JointConstraint = Default::default();
    let mut pending = true;
    while pending {
        c = plain_inline(step, seeds, b1, b2, p);
        pending = false;
    }
    c
}
#[inline(always)]
pub(crate) fn plain_inline(
    step: StepJoint, seeds: [Fixed; 3], b1: SolverBody, b2: SolverBody, p: IntegrationParameters,
) -> JointConstraint {
    let (h, erp, cfm, _, _) = frame(b1, b2, step.frame1, step.frame2, step.locks, step.softness, p);
    let mut c: JointConstraint = Default::default();
    c.im1 = b1.im;
    c.im2 = b2.im;
    lock_rows(ref c, h, b1, b2, erp, cfm, step.locks);
    JointConstraintHelperTrait::finalize(ref c);
    seed(ref c, seeds, p);
    c
}
/// `generate` with every controlled axis in the loop (all live values pass per iteration).
pub(crate) fn generate_looped(
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
    if motors.len() != 0 || limits.len() != 0 {
        c
            .bounded =
                boxed(
                    BoundedRows { motors: motors.span(), limits: limits.span(), locks: c.num_rows },
                );
        c.num_rows = 4;
    }
    c
}
/// `controls` looping over the three axes (the whole joint passes per iteration).
pub(crate) fn controls_looped(data: GenericJoint, inside: bool) -> StepControls {
    let mut axes = array![];
    for axis in [2_u8, 0, 1].span() {
        let axis = *axis;
        let [lx, ly, lw] = data.limits;
        let [mx, my, mw] = data.motors;
        let (m, l) = match axis {
            0 => (mx, lx),
            1 => (my, ly),
            _ => (mw, lw),
        };
        control(
            ref axes,
            (data.locked_axes, data.coupled_axes, data.motor_axes, data.limit_axes),
            axis,
            m,
            l,
            inside,
        );
    }
    let [lx, ly, lw] = data.limits;
    let [mx, my, mw] = data.motors;
    StepControls {
        axes: axes.span(),
        motors: [mx.impulse, my.impulse, mw.impulse],
        limits: [lx.impulse, ly.impulse, lw.impulse],
    }
}
/// First interior test: `cos(phi) > cos(half) + margin`; pays `cos(half)` at specialisation.
pub(crate) fn angular_cos(l: JointLimits) -> StepLimit {
    let mut r = angular(l, false);
    if r.mode == 1 && r.half > ZERO {
        r.inside = r.half.cos() + INSIDE_MARGIN;
    } else {
        r.inside = MAX;
    }
    r
}
pub(crate) fn interior_cos(l: StepLimit, r: Rot2) -> bool {
    dot2(l.cos, r.re, l.sin, r.im) > l.inside
}
