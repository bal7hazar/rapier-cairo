//! Scalar row maths; fused dots floor once, multiplication floors, reciprocals round to nearest.
use fixed::Fixed;
use fixed::wide::dot4;
use glam::Vec2;
use rapier_math::math_ext::inv;
use crate::solver::body::SolverVel;
use crate::solver::joint::JointGenericConstraint;

pub(crate) fn scale(v: Vec2, s: Fixed) -> Vec2 {
    v * Vec2 { x: s, y: s }
}
pub(crate) fn metric(a: JointGenericConstraint, b: JointGenericConstraint, imsum: Vec2) -> Fixed {
    let lin = imsum * b.lin_jac;
    dot4(
        a.lin_jac.x,
        lin.x,
        a.lin_jac.y,
        lin.y,
        a.ii_ang_jac1,
        b.ang_jac1,
        a.ii_ang_jac2,
        b.ang_jac2,
    )
}
pub(crate) fn finish(ref a: JointGenericConstraint, imsum: Vec2) -> Fixed {
    let mass = metric(a, a, imsum);
    a.cfm_gain = mass * a.cfm_coeff + a.cfm_gain;
    a.inv_lhs = inv(mass + a.cfm_gain);
    inv(mass)
}
pub(crate) fn project(
    ref a: JointGenericConstraint, b: JointGenericConstraint, imsum: Vec2, inverse: Fixed,
) {
    let coeff = metric(a, b, imsum) * inverse;
    a.lin_jac = a.lin_jac - scale(b.lin_jac, coeff);
    a.ang_jac1 -= b.ang_jac1 * coeff;
    a.ang_jac2 -= b.ang_jac2 * coeff;
    a.ii_ang_jac1 -= b.ii_ang_jac1 * coeff;
    a.ii_ang_jac2 -= b.ii_ang_jac2 * coeff;
    a.rhs -= b.rhs * coeff;
    a.rhs_wo_bias -= b.rhs_wo_bias * coeff;
}
pub(crate) fn apply(
    a: JointGenericConstraint,
    impulse: Fixed,
    im1: Vec2,
    im2: Vec2,
    ref v1: SolverVel,
    ref v2: SolverVel,
) {
    let lin = scale(a.lin_jac, impulse);
    v1.linear = v1.linear + lin * im1;
    v2.linear = v2.linear - lin * im2;
    v1.angular += a.ii_ang_jac1 * impulse;
    v2.angular -= a.ii_ang_jac2 * impulse;
}
pub(crate) fn solve_row(
    ref a: JointGenericConstraint, im1: Vec2, im2: Vec2, ref v1: SolverVel, ref v2: SolverVel,
) {
    let dv = v2.linear - v1.linear;
    let rhs = dot4(
        a.lin_jac.x, dv.x, a.lin_jac.y, dv.y, a.ang_jac2, v2.angular, -a.ang_jac1, v1.angular,
    )
        + a.rhs;
    let delta = a.inv_lhs * (rhs - a.cfm_gain * a.impulse);
    a.impulse += delta;
    apply(a, delta, im1, im2, ref v1, ref v2);
}
