//! Rejected coupled-row bases (RJ). `literal`: upstream's operation order — sum of per-axis
//! projections, Fixed length, `inv`, then every Jacobian scaled (overflows for a separation below
//! 2^-31). `fixed_inv`: the winner's cross products on a direction scaled by a Fixed `inv`.
use fixed::wide::norm2;
use fixed::{Fixed, ZERO};
use super::*;

pub fn literal(
    h: JointConstraintHelper, bits: u8, axis: u8, b1: SolverBody, b2: SolverBody,
) -> (JointGenericConstraint, Fixed) {
    let mut lin = Vec2 { x: ZERO, y: ZERO };
    let mut a1 = ZERO;
    let mut a2 = ZERO;
    if bits != 2 {
        let coeff = dot2(h.x.x, h.lin_err.x, h.x.y, h.lin_err.y);
        lin = lin + scale(h.x, coeff);
        a1 += gcross_vv(h.r1.x, h.r1.y, h.x.x, h.x.y) * coeff;
        a2 += gcross_vv(h.r2.x, h.r2.y, h.x.x, h.x.y) * coeff;
    }
    if bits != 1 {
        let coeff = dot2(h.y.x, h.lin_err.x, h.y.y, h.lin_err.y);
        lin = lin + scale(h.y, coeff);
        a1 += gcross_vv(h.r1.x, h.r1.y, h.y.x, h.y.y) * coeff;
        a2 += gcross_vv(h.r2.x, h.r2.y, h.y.x, h.y.y) * coeff;
    }
    let dist = norm2(lin.x, lin.y);
    let inv_dist = inv(dist);
    let a1 = a1 * inv_dist;
    let a2 = a2 * inv_dist;
    (
        JointGenericConstraint {
            lin_jac: scale(lin, inv_dist),
            ang_jac1: a1,
            ang_jac2: a2,
            ii_ang_jac1: b1.ii * a1,
            ii_ang_jac2: b2.ii * a2,
            axis,
            ..Default::default(),
        },
        dist,
    )
}

pub fn fixed_inv(
    h: JointConstraintHelper, bits: u8, axis: u8, b1: SolverBody, b2: SolverBody,
) -> (JointGenericConstraint, Fixed) {
    let v = separation(h, bits);
    let dist = norm2(v.x, v.y);
    let dir = scale(v, inv(dist));
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
        dist,
    )
}
