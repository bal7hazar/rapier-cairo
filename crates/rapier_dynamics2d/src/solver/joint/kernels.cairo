//! Outlined bilateral kernels behind the inlined row-kind dispatch.
use super::*;

pub(crate) fn generate_plain(
    joint: ImpulseJoint, bodies: Span<SolverBody>, params: IntegrationParameters,
) -> JointConstraint {
    let (mut constraint, h, b1, b2, erp, cfm) = prepare(joint, bodies, params);
    if joint.data.enabled != JointEnabled::Enabled {
        return constraint;
    }
    let locks = joint.data.locked_axes;
    // Specialised rows keep the same upstream order and arithmetic as generic assembly.
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
    JointConstraintHelperTrait::finalize(ref constraint);
    seed(ref constraint, joint, params);

    constraint
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
    let [a, b, d] = constraint.rows;
    let [mut x, mut y, mut w] = joint.impulses;
    if constraint.num_rows != 0 {
        write(a.axis, a.impulse, ref x, ref y, ref w);
    }
    if constraint.num_rows >= 2 {
        write(b.axis, b.impulse, ref x, ref y, ref w);
    }
    if constraint.num_rows == 3 {
        write(d.axis, d.impulse, ref x, ref y, ref w);
    }
    joint.impulses = [x, y, w];
}
pub(crate) fn generate_extended(
    j: ImpulseJoint, bodies: Span<SolverBody>, p: IntegrationParameters,
) -> JointConstraint {
    let mut constraint = generate_plain(j, bodies, p);
    if j.data.enabled != JointEnabled::Enabled {
        return constraint;
    }
    let (_, h, b1, b2, erp, cfm) = prepare(j, bodies, p);
    bounded::generate(ref constraint, j, h, b1, b2, p, erp, cfm);
    constraint
}
