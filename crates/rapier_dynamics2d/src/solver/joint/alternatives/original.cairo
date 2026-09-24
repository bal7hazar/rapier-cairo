//! Pre-OJ implementation retained for raw equivalence and cost comparisons.
use super::super::*;
mod row;
use row::{apply, finish, project, solve_row};
pub(crate) fn generate(
    joint: ImpulseJoint, bodies: Span<SolverBody>, params: IntegrationParameters,
) -> JointConstraint {
    let (mut c, h, b1, b2, erp, cfm) = prepare(joint, bodies, params);
    if joint.data.enabled != JointEnabled::Enabled {
        return c;
    }
    let locks = joint.data.locked_axes;
    // Specialised rows keep the same upstream order and arithmetic as generic assembly.
    match locks.bits {
        3 => {
            c
                .rows =
                    [
                        h.lock_linear(0, b1, b2, erp, cfm), h.lock_linear(1, b1, b2, erp, cfm),
                        Default::default(),
                    ];
            c.num_rows = 2;
        },
        6 => {
            c
                .rows =
                    [
                        h.lock_angular(b1, b2, erp, cfm), h.lock_linear(1, b1, b2, erp, cfm),
                        Default::default(),
                    ];
            c.num_rows = 2;
        },
        7 => {
            c
                .rows =
                    [
                        h.lock_angular(b1, b2, erp, cfm), h.lock_linear(0, b1, b2, erp, cfm),
                        h.lock_linear(1, b1, b2, erp, cfm),
                    ];
            c.num_rows = 3;
        },
        _ => {
            if locks.contains_axis(2) {
                push(ref c, h.lock_angular(b1, b2, erp, cfm));
            }
            if locks.contains_axis(0) {
                push(ref c, h.lock_linear(0, b1, b2, erp, cfm));
            }
            if locks.contains_axis(1) {
                push(ref c, h.lock_linear(1, b1, b2, erp, cfm));
            }
        },
    }
    finalize(ref c);
    seed(ref c, joint, params);
    c
}
/// Apply seeded impulses once before solving; no division, products floor, overflow panics.
pub(crate) fn warmstart(constraint: JointConstraint, ref bodies: Array<SolverBody>) {
    if constraint.num_rows == 0 {
        return;
    }
    let mut v1 = velocity(read(bodies.span(), constraint.solver_vel1));
    let mut v2 = velocity(read(bodies.span(), constraint.solver_vel2));
    let [a, b, c] = constraint.rows;
    apply(a, a.impulse, constraint.im1, constraint.im2, ref v1, ref v2);
    if constraint.num_rows >= 2 {
        apply(b, b.impulse, constraint.im1, constraint.im2, ref v1, ref v2);
    }
    if constraint.num_rows == 3 {
        apply(c, c.impulse, constraint.im1, constraint.im2, ref v1, ref v2);
    }
    scatter(ref bodies, constraint.solver_vel1, v1, constraint.solver_vel2, v2);
}
/// Division-free ordered Gauss–Seidel sweep. `biased=false` permanently removes rhs bias
/// until regeneration; upstream retains softness during relaxation. Fixed overflow panics.
pub(crate) fn solve(ref constraint: JointConstraint, ref bodies: Array<SolverBody>, biased: bool) {
    if constraint.num_rows == 0 {
        return;
    }
    if !biased {
        remove_bias(ref constraint);
    }
    let mut v1 = velocity(read(bodies.span(), constraint.solver_vel1));
    let mut v2 = velocity(read(bodies.span(), constraint.solver_vel2));
    let [mut a, mut b, mut c] = constraint.rows;
    solve_row(ref a, constraint.im1, constraint.im2, ref v1, ref v2);
    if constraint.num_rows >= 2 {
        solve_row(ref b, constraint.im1, constraint.im2, ref v1, ref v2);
    }
    if constraint.num_rows == 3 {
        solve_row(ref c, constraint.im1, constraint.im2, ref v1, ref v2);
    }
    constraint.rows = [a, b, c];
    scatter(ref bodies, constraint.solver_vel1, v1, constraint.solver_vel2, v2);
}
/// Exact rhs copies only; masses, impulses and CFM are preserved, as upstream.
pub(crate) fn remove_bias(ref constraint: JointConstraint) {
    let [mut a, mut b, mut c] = constraint.rows;
    a.rhs = a.rhs_wo_bias;
    b.rhs = b.rhs_wo_bias;
    c.rhs = c.rhs_wo_bias;
    constraint.rows = [a, b, c];
}
/// Persist active row impulses at their original DOF indices; no arithmetic or rounding.
/// Disabled/no-row joints preserve previous impulses; free axes are untouched.
pub(crate) fn writeback_impulses(constraint: JointConstraint, ref joint: ImpulseJoint) {
    let [a, b, c] = constraint.rows;
    if constraint.num_rows != 0 {
        write(a, ref joint);
    }
    if constraint.num_rows >= 2 {
        write(b, ref joint);
    }
    if constraint.num_rows == 3 {
        write(c, ref joint);
    }
}
fn write(a: JointGenericConstraint, ref joint: ImpulseJoint) {
    let [mut x, mut y, mut w] = joint.impulses;
    match a.axis {
        0 => x = a.impulse,
        1 => y = a.impulse,
        _ => w = a.impulse,
    }
    joint.impulses = [x, y, w];
}
fn seed(ref c: JointConstraint, joint: ImpulseJoint, params: IntegrationParameters) {
    if params.warmstart_joints {
        let [mut a, mut b, mut d] = c.rows;
        a.impulse = seed_row(a, joint) * params.warmstart_coefficient;
        if c.num_rows >= 2 {
            b.impulse = seed_row(b, joint) * params.warmstart_coefficient;
        }
        if c.num_rows == 3 {
            d.impulse = seed_row(d, joint) * params.warmstart_coefficient;
        }
        c.rows = [a, b, d];
    }
}
fn seed_row(a: JointGenericConstraint, joint: ImpulseJoint) -> Fixed {
    let [x, y, w] = joint.impulses;
    match a.axis {
        0 => x,
        1 => y,
        _ => w,
    }
}
fn push(ref c: JointConstraint, a: JointGenericConstraint) {
    let [mut x, mut y, mut z] = c.rows;
    match c.num_rows {
        0 => x = a,
        1 => y = a,
        _ => z = a,
    }
    c.num_rows += 1;
    c.rows = [x, y, z];
}
fn resolve(mut bodies: Span<SolverBody>, handle: Handle) -> u32 {
    let mut i = 0;
    while let Some(b) = bodies.pop_front() {
        if *b.handle == handle {
            return i;
        }
        i += 1;
    }
    core::panic_with_felt252(errors::BODY)
}
fn prepare(
    joint: ImpulseJoint, bodies: Span<SolverBody>, params: IntegrationParameters,
) -> (JointConstraint, JointConstraintHelper, SolverBody, SolverBody, Fixed, Fixed) {
    let mut c: JointConstraint = Default::default();
    let dummy = JointConstraintHelper {
        x: Default::default(),
        y: Default::default(),
        r1: Default::default(),
        r2: Default::default(),
        lin_err: Default::default(),
        ang_err: ZERO,
    };
    if joint.data.enabled != JointEnabled::Enabled {
        return (c, dummy, Default::default(), Default::default(), ZERO, ZERO);
    }
    c.solver_vel1 = resolve(bodies, joint.body1);
    c.solver_vel2 = resolve(bodies, joint.body2);
    assert(c.solver_vel1 != c.solver_vel2, errors::SAME_BODY);
    let b1 = read(bodies, c.solver_vel1);
    let b2 = read(bodies, c.solver_vel2);
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
            && joint.data.local_frame1.rotation.is_unit()
            && joint.data.local_frame2.rotation.is_unit(),
        errors::ROTATION,
    );
    c.im1 = b1.im;
    c.im2 = b2.im;
    let f1 = b1.position.mul(joint.data.local_frame1);
    let f2 = b2.position.mul(joint.data.local_frame2);
    let h = JointConstraintHelperTrait::new(
        f1, f2, b1.position.translation, b2.position.translation, joint.data.locked_axes,
    );
    let soft = params.joint_softness_coefficients(joint.data.softness);
    let cfm = rigid_cfm(soft.cfm_coeff);
    (c, h, b1, b2, soft.erp_inv_dt, cfm)
}
fn rigid_cfm(cfm: Fixed) -> Fixed {
    if cfm < RIGID_CFM_THRESHOLD {
        ZERO
    } else {
        cfm
    }
}

pub(crate) fn finalize(ref constraint: JointConstraint) {
    let imsum = constraint.im1 + constraint.im2;
    let [mut a, mut b, mut c] = constraint.rows;
    if constraint.num_rows != 0 {
        let ia = finish(ref a, imsum);
        if constraint.num_rows >= 2 {
            project(ref b, a, imsum, ia);
        }
        if constraint.num_rows == 3 {
            project(ref c, a, imsum, ia);
        }
        if constraint.num_rows >= 2 {
            let ib = finish(ref b, imsum);
            if constraint.num_rows == 3 {
                project(ref c, b, imsum, ib);
            }
        }
        if constraint.num_rows == 3 {
            let _ = finish(ref c, imsum);
        }
    }
    constraint.rows = [a, b, c];
}
