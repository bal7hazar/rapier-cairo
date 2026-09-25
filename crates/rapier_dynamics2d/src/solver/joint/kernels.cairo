//! Outlined bilateral kernels behind the inlined row-kind dispatch.
use rapier_core::integration_parameters::spring::SpringCoefficients;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use crate::joint::JointAxesMask;
use super::*;

pub(crate) fn generate_plain(
    joint: ImpulseJoint, bodies: Span<SolverBody>, params: IntegrationParameters,
) -> JointConstraint {
    let (mut constraint, h, b1, b2, erp, cfm) = prepare(joint, bodies, params);
    if joint.data.enabled != JointEnabled::Enabled {
        return constraint;
    }
    lock_rows(ref constraint, h, b1, b2, erp, cfm, joint.data.locked_axes);
    JointConstraintHelperTrait::finalize(ref constraint);
    seed(ref constraint, joint.impulses, params);

    constraint
}
/// Validated world frames, helper, softness and both world frame rotations (for angles).
/// Same checks, order and arithmetic as `prepare` after body resolution.
#[inline(always)]
pub(crate) fn frame(
    b1: SolverBody,
    b2: SolverBody,
    frame1: Pose2,
    frame2: Pose2,
    locks: JointAxesMask,
    softness: SpringCoefficients,
    params: IntegrationParameters,
) -> (JointConstraintHelper, Fixed, Fixed, Rot2, Rot2) {
    assert(
        b1.im.x >= ZERO
            && b1.im.y >= ZERO
            && b1.ii >= ZERO
            && b2.im.x >= ZERO
            && b2.im.y >= ZERO
            && b2.ii >= ZERO
            && params.warmstart_coefficient >= ZERO,
        errors::NEGATIVE,
    );
    assert(
        b1.position.rotation.is_unit()
            && b2.position.rotation.is_unit()
            && frame1.rotation.is_unit()
            && frame2.rotation.is_unit(),
        errors::ROTATION,
    );
    let f1 = b1.position.mul(frame1);
    let f2 = b2.position.mul(frame2);
    let h = JointConstraintHelperTrait::new(
        f1, f2, b1.position.translation, b2.position.translation, locks,
    );
    let soft = params.joint_softness_coefficients(softness);
    (h, soft.erp_inv_dt, rigid_cfm(soft.cfm_coeff), f1.rotation, f2.rotation)
}
/// Locked rows in upstream order (angular, X, Y); specialised masks keep the same arithmetic.
#[inline(always)]
pub(crate) fn lock_rows(
    ref constraint: JointConstraint,
    h: JointConstraintHelper,
    b1: SolverBody,
    b2: SolverBody,
    erp: Fixed,
    cfm: Fixed,
    locks: JointAxesMask,
) {
    match locks.bits {
        3 => {
            constraint
                .rows =
                    [
                        h.lock_linear(0, b1, b2, erp, cfm), h.lock_linear(1, b1, b2, erp, cfm),
                        Default::default(),
                    ];
            constraint.num_rows = 2;
        },
        6 => {
            constraint
                .rows =
                    [
                        h.lock_angular(b1, b2, erp, cfm), h.lock_linear(1, b1, b2, erp, cfm),
                        Default::default(),
                    ];
            constraint.num_rows = 2;
        },
        7 => {
            constraint
                .rows =
                    [
                        h.lock_angular(b1, b2, erp, cfm), h.lock_linear(0, b1, b2, erp, cfm),
                        h.lock_linear(1, b1, b2, erp, cfm),
                    ];
            constraint.num_rows = 3;
        },
        _ => {
            if locks.contains_axis(2) {
                push(ref constraint, h.lock_angular(b1, b2, erp, cfm));
            }
            if locks.contains_axis(0) {
                push(ref constraint, h.lock_linear(0, b1, b2, erp, cfm));
            }
            if locks.contains_axis(1) {
                push(ref constraint, h.lock_linear(1, b1, b2, erp, cfm));
            }
        },
    }
}
pub(crate) fn warmstart_plain(constraint: JointConstraint, ref bodies: Array<SolverBody>) {
    if constraint.num_rows == 0 {
        return;
    }
    let mut v1 = velocity(read(bodies.span(), constraint.solver_vel1));
    let mut v2 = velocity(read(bodies.span(), constraint.solver_vel2));
    let [a, b, d] = constraint.rows;
    apply(a, a.impulse, constraint.im1, constraint.im2, ref v1, ref v2);
    if constraint.num_rows >= 2 {
        apply(b, b.impulse, constraint.im1, constraint.im2, ref v1, ref v2);
    }
    if constraint.num_rows == 3 {
        apply(d, d.impulse, constraint.im1, constraint.im2, ref v1, ref v2);
    }
    scatter(ref bodies, constraint.solver_vel1, v1, constraint.solver_vel2, v2);
}
pub(crate) fn solve_plain(
    ref constraint: JointConstraint, ref bodies: Array<SolverBody>, biased: bool,
) {
    if constraint.num_rows == 0 {
        return;
    }
    if !biased {
        remove_bias_plain(ref constraint);
    }
    let mut v1 = velocity(read(bodies.span(), constraint.solver_vel1));
    let mut v2 = velocity(read(bodies.span(), constraint.solver_vel2));
    let [mut a, mut b, mut d] = constraint.rows;
    solve_row(ref a, constraint.im1, constraint.im2, ref v1, ref v2);
    if constraint.num_rows >= 2 {
        solve_row(ref b, constraint.im1, constraint.im2, ref v1, ref v2);
    }
    if constraint.num_rows == 3 {
        solve_row(ref d, constraint.im1, constraint.im2, ref v1, ref v2);
    }
    constraint.rows = [a, b, d];
    scatter(ref bodies, constraint.solver_vel1, v1, constraint.solver_vel2, v2);
}
#[inline(always)]
pub(crate) fn remove_bias_plain(ref constraint: JointConstraint) {
    let [mut a, mut b, mut d] = constraint.rows;
    a.rhs = a.rhs_wo_bias;
    b.rhs = b.rhs_wo_bias;
    d.rhs = d.rhs_wo_bias;
    constraint.rows = [a, b, d];
}
#[inline(always)]
pub(crate) fn writeback_impulses_plain(constraint: JointConstraint, ref joint: ImpulseJoint) {
    let mut impulses = joint.impulses;
    write_rows(constraint.rows, constraint.num_rows, ref impulses);
    joint.impulses = impulses;
}
/// Write the first `count` lock rows' impulses at their DOF indices (no arithmetic).
#[inline(always)]
pub(crate) fn write_rows(rows: [JointGenericConstraint; 3], count: u8, ref impulses: [Fixed; 3]) {
    let [a, b, d] = rows;
    let [mut x, mut y, mut w] = impulses;
    if count != 0 {
        write(a.axis, a.impulse, ref x, ref y, ref w);
    }
    if count >= 2 {
        write(b.axis, b.impulse, ref x, ref y, ref w);
    }
    if count == 3 {
        write(d.axis, d.impulse, ref x, ref y, ref w);
    }
    impulses = [x, y, w];
}
/// Limits and motors, with one body resolution and one frame construction.
pub(crate) fn generate_extended(
    j: ImpulseJoint, bodies: Span<SolverBody>, p: IntegrationParameters,
) -> JointConstraint {
    if j.data.enabled != JointEnabled::Enabled {
        return Default::default();
    }
    let solver_vel1 = resolve(bodies, j.body1);
    let solver_vel2 = resolve(bodies, j.body2);
    assert(solver_vel1 != solver_vel2, errors::SAME_BODY);
    let b1 = read(bodies, solver_vel1);
    let b2 = read(bodies, solver_vel2);
    let step = StepJoint {
        frame1: j.data.local_frame1,
        frame2: j.data.local_frame2,
        locks: j.data.locked_axes,
        softness: j.data.softness,
    };
    let controls = step::controls(j.data, false);
    let mut c = step::generate(
        step, controls, j.impulses, controls.motors, controls.limits, b1, b2, p, false,
    );
    c.solver_vel1 = solver_vel1;
    c.solver_vel2 = solver_vel2;
    c
}
