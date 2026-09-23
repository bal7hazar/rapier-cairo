//! Upstream frame construction and modified Gram–Schmidt, angular before X before Y.
use fixed::wide::dot2;
use fixed::{Fixed, ONE};
use glam::Vec2;
use rapier_math::math_ext::gcross_vv;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2Trait;
use crate::joint::{JointAxesMask, JointAxesMaskTrait};
use super::row::{finish, project, scale};
use super::super::body::SolverBody;
use super::{JointConstraint, JointGenericConstraint, errors};

/// World-space basis and errors; lever arms account for free linear motion of frame 2.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct JointConstraintHelper {
    pub x: Vec2,
    pub y: Vec2,
    pub r1: Vec2,
    pub r2: Vec2,
    pub lin_err: Vec2,
    pub ang_err: Fixed,
}
#[generate_trait]
pub impl JointConstraintHelperImpl of JointConstraintHelperTrait {
    /// Build from world frames and centres of mass; unit frame rotations required.
    /// Fixed products floor, overflow panics; mask must be 0..7.
    fn new(
        frame1: Pose2, frame2: Pose2, com1: Vec2, com2: Vec2, locks: JointAxesMask,
    ) -> JointConstraintHelper {
        let x = Vec2 { x: frame1.rotation.re, y: frame1.rotation.im };
        let y = Vec2 { x: -x.y, y: x.x };
        let lin_err = frame2.translation - frame1.translation;
        let mut center = frame2.translation;
        if locks.contains_axis(0) {
            center = center - scale(x, dot(x, lin_err));
        }
        if locks.contains_axis(1) {
            center = center - scale(y, dot(y, lin_err));
        }
        JointConstraintHelper {
            x,
            y,
            r1: center - com1,
            r2: frame2.translation - com2,
            lin_err,
            ang_err: (frame1.rotation.inverse() * frame2.rotation).im,
        }
    }
    /// Linear lock row for axis 0 or 1. Softness is already thresholded; same numeric policy.
    /// Other axes panic with Joint: invalid axis. No division.
    fn lock_linear(
        self: JointConstraintHelper,
        axis: u8,
        b1: SolverBody,
        b2: SolverBody,
        erp_inv_dt: Fixed,
        cfm_coeff: Fixed,
    ) -> JointGenericConstraint {
        let lin_jac = match axis {
            0 => self.x,
            1 => self.y,
            _ => core::panic_with_felt252(errors::AXIS),
        };
        let a1 = gcross_vv(self.r1.x, self.r1.y, lin_jac.x, lin_jac.y);
        let a2 = gcross_vv(self.r2.x, self.r2.y, lin_jac.x, lin_jac.y);
        JointGenericConstraint {
            lin_jac,
            ang_jac1: a1,
            ang_jac2: a2,
            ii_ang_jac1: b1.ii * a1,
            ii_ang_jac2: b2.ii * a2,
            rhs: dot(lin_jac, self.lin_err) * erp_inv_dt,
            cfm_coeff,
            erp_inv_dt,
            axis,
            ..Default::default(),
        }
    }
    /// Angular lock uses sine of relative rotation, exactly as upstream (no atan2).
    /// No division; fixed multiplication floors and overflow panics.
    fn lock_angular(
        self: JointConstraintHelper,
        b1: SolverBody,
        b2: SolverBody,
        erp_inv_dt: Fixed,
        cfm_coeff: Fixed,
    ) -> JointGenericConstraint {
        JointGenericConstraint {
            ang_jac1: ONE,
            ang_jac2: ONE,
            ii_ang_jac1: b1.ii,
            ii_ang_jac2: b2.ii,
            rhs: self.ang_err * erp_inv_dt,
            cfm_coeff,
            erp_inv_dt,
            axis: 2,
            ..Default::default(),
        }
    }
    /// Orthogonalize in row order (modified Gram–Schmidt), then cache inverse lhs.
    /// At most three rows. Zero mass has zero inverse, including dependent rows.
    /// Division rounds to nearest; products/dots floor. Overflow/too-small inverse panics in Fixed.
    fn finalize(ref constraint: JointConstraint) {
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
}
fn dot(a: Vec2, b: Vec2) -> Fixed {
    dot2(a.x, b.x, a.y, b.y)
}
