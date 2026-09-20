//! Rebuild each substep after forces: generate, optional warmstart, biased solve, integrate,
//! remove_bias, relaxed solve, writeback_impulses. Array order must remain fixed until discard.
//! Body poses are at CoM: callers must shift body-local joint translations by local_com first.
//! D4: CFM below 8 Q32.32 ulp becomes zero, preserving computed ERP. Products/dots floor;
//! reciprocals truncate. All intermediates must fit Fixed; nonnegative masses required.
mod helper;
mod row;
use fixed::{Fixed, ZERO};
use glam::Vec2;
pub use helper::{JointConstraintHelper, JointConstraintHelperTrait};
use rapier_core::data::handle::Handle;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_math::pose2::Pose2Trait;
use rapier_math::rot2::Rot2Trait;
pub use row::JointGenericConstraint;
use row::{apply, solve_row};
use crate::joint::{ImpulseJoint, JointAxesMaskTrait, JointEnabled};
use super::body::{SolverBody, read, scatter, velocity};

/// Validation failures outside the solve hot path.
pub mod errors {
    pub const AXIS: felt252 = 'Joint: invalid axis';
    pub const BODY: felt252 = 'Joint: missing body';
    pub const SAME_BODY: felt252 = 'Joint: same body';
    pub const NEGATIVE: felt252 = 'Joint: negative input';
    pub const ROTATION: felt252 = 'Joint: nonunit rotation';
}
/// Default joint CFM near 240 Hz is only six ulp. Values strictly below this become rigid.
pub const RIGID_CFM_THRESHOLD: Fixed = Fixed { raw: 8 };
/// Up to three ordered bilateral rows with resolved dense-body indices and inverse masses.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct JointConstraint {
    pub solver_vel1: u32,
    pub solver_vel2: u32,
    pub im1: Vec2,
    pub im2: Vec2,
    pub rows: [JointGenericConstraint; 3],
    pub num_rows: u8,
}
#[generate_trait]
pub impl JointConstraintImpl of JointConstraintTrait {
    /// Rebuild locks from current CoM poses, resolving complete generational body handles.
    /// Disabled joints return zero rows. Missing/same bodies, negative mass/warmstart, nonunit
    /// frames panic with errors constants; parameter/Fixed panics propagate. Reserved masks
    /// ignored.
    fn generate(
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
        JointConstraintHelperTrait::finalize(ref c);
        seed(ref c, joint, params);
        c
    }
    /// Apply seeded impulses once before solving; no division, products floor, overflow panics.
    fn warmstart(self: JointConstraint, ref bodies: Array<SolverBody>) {
        if self.num_rows == 0 {
            return;
        }
        let mut v1 = velocity(read(bodies.span(), self.solver_vel1));
        let mut v2 = velocity(read(bodies.span(), self.solver_vel2));
        let [a, b, c] = self.rows;
        apply(a, a.impulse, self.im1, self.im2, ref v1, ref v2);
        if self.num_rows >= 2 {
            apply(b, b.impulse, self.im1, self.im2, ref v1, ref v2);
        }
        if self.num_rows == 3 {
            apply(c, c.impulse, self.im1, self.im2, ref v1, ref v2);
        }
        scatter(ref bodies, self.solver_vel1, v1, self.solver_vel2, v2);
    }
    /// Division-free ordered Gauss–Seidel sweep. `biased=false` permanently removes rhs bias
    /// until regeneration; upstream retains softness during relaxation. Fixed overflow panics.
    fn solve(ref self: JointConstraint, ref bodies: Array<SolverBody>, biased: bool) {
        if self.num_rows == 0 {
            return;
        }
        if !biased {
            self.remove_bias();
        }
        let mut v1 = velocity(read(bodies.span(), self.solver_vel1));
        let mut v2 = velocity(read(bodies.span(), self.solver_vel2));
        let [mut a, mut b, mut c] = self.rows;
        solve_row(ref a, self.im1, self.im2, ref v1, ref v2);
        if self.num_rows >= 2 {
            solve_row(ref b, self.im1, self.im2, ref v1, ref v2);
        }
        if self.num_rows == 3 {
            solve_row(ref c, self.im1, self.im2, ref v1, ref v2);
        }
        self.rows = [a, b, c];
        scatter(ref bodies, self.solver_vel1, v1, self.solver_vel2, v2);
    }
    /// Exact rhs copies only; masses, impulses and CFM are preserved, as upstream.
    fn remove_bias(ref self: JointConstraint) {
        let [mut a, mut b, mut c] = self.rows;
        a.rhs = a.rhs_wo_bias;
        b.rhs = b.rhs_wo_bias;
        c.rhs = c.rhs_wo_bias;
        self.rows = [a, b, c];
    }
    /// Persist active row impulses at their original DOF indices; no arithmetic or rounding.
    /// Disabled/no-row joints preserve previous impulses; free axes are untouched.
    fn writeback_impulses(self: JointConstraint, ref joint: ImpulseJoint) {
        let [a, b, c] = self.rows;
        if self.num_rows != 0 {
            write(a, ref joint);
        }
        if self.num_rows >= 2 {
            write(b, ref joint);
        }
        if self.num_rows == 3 {
            write(c, ref joint);
        }
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
#[cfg(test)]
mod alternatives;
#[cfg(test)]
mod tests {
    use fixed::{FixedTrait, HALF, ONE};
    use rapier_math::pose2::Pose2;
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::joint::{GenericJoint, JointAxesMask};
    use super::*;
    use super::row::metric;

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }
    fn h(index: u32) -> Handle {
        Handle { index, generation: 1 }
    }
    fn fixture(bits: u8) -> (ImpulseJoint, Array<SolverBody>, IntegrationParameters) {
        let b1 = SolverBody {
            handle: h(0),
            im: v(ONE, HALF),
            ii: HALF,
            linvel: v(HALF, -ONE),
            angvel: HALF,
            ..Default::default(),
        };
        let b2 = SolverBody {
            handle: h(1),
            position: Pose2 { translation: v(HALF, -HALF), ..Default::default() },
            im: v(HALF, ONE),
            ii: ONE,
            linvel: v(-ONE, HALF),
            angvel: -ONE,
        };
        let data = GenericJoint {
            locked_axes: JointAxesMask { bits },
            local_frame1: Pose2 { translation: v(ONE, HALF), ..Default::default() },
            local_frame2: Pose2 { translation: v(-HALF, ONE), ..Default::default() },
            ..Default::default(),
        };
        (
            ImpulseJoint { body1: h(0), body2: h(1), data, impulses: [HALF, -ONE, ONE] },
            array![b1, b2],
            Default::default(),
        )
    }
    #[test]
    fn test_generic_specialised_all_masks_and_orthogonality() {
        let mut bits = 0;
        while bits != 8 {
            let (j, bs, p) = fixture(bits);
            let c = JointConstraintTrait::generate(j, bs.span(), p);
            let generic = alternatives::generate(j, bs.span(), p);
            assert_eq!(c, generic);
            let [a, b, d] = c.rows;
            if c.num_rows >= 2 {
                assert!(metric(a, b, c.im1 + c.im2).abs().raw <= 16);
            }
            if c.num_rows == 3 {
                assert!(metric(a, d, c.im1 + c.im2).abs().raw <= 16);
                assert!(metric(b, d, c.im1 + c.im2).abs().raw <= 16);
            }
            if bits == 7 {
                assert_eq!(a.axis, 2);
                assert_eq!(b.axis, 0);
                assert_eq!(d.axis, 1);
            }
            bits += 1;
        }
    }
    #[test]
    fn test_softness_threshold_and_relaxation() {
        let (mut j, bs, mut p) = fixture(3);
        let mut c = JointConstraintTrait::generate(j, bs.span(), p);
        let [a, _, _] = c.rows;
        assert_eq!(a.cfm_coeff, ZERO);
        assert_eq!(a.erp_inv_dt, p.joint_softness_coefficients(j.data.softness).erp_inv_dt);
        // A shorter step puts default softness above the rigid threshold.
        p.dt = Fixed { raw: 35791394 };
        c = JointConstraintTrait::generate(j, bs.span(), p);
        let [a, _, _] = c.rows;
        assert!(a.cfm_coeff >= RIGID_CFM_THRESHOLD);
        c.remove_bias();
        let [b, _, _] = c.rows;
        assert_eq!(b.rhs, ZERO);
        assert_eq!(b.cfm_gain, a.cfm_gain);
        // Fully static endpoints have exactly zero inverse mass, with no divide by zero.
        let mut b1 = *bs.at(0);
        let mut b2 = *bs.at(1);
        b1.im = Default::default();
        b1.ii = ZERO;
        b2.im = Default::default();
        b2.ii = ZERO;
        let mut bs = array![b1, b2];
        j.data.locked_axes.bits = 7;
        let mut c = JointConstraintTrait::generate(j, bs.span(), p);
        c.solve(ref bs, true);
        assert_eq!(*bs.at(0), b1);
        assert_eq!(*bs.at(1), b2);
    }
    #[test]
    fn test_warmstart_writeback_disabled_and_rebuild() {
        let (mut j, mut bs, mut p) = fixture(6);
        let cold = JointConstraintTrait::generate(j, bs.span(), p);
        let [a, b, _] = cold.rows;
        assert_eq!(a.impulse, ZERO);
        assert_eq!(b.impulse, ZERO);
        p.warmstart_joints = true;
        p.warmstart_coefficient = HALF;
        let mut c = JointConstraintTrait::generate(j, bs.span(), p);
        let [a, b, _] = c.rows;
        assert_eq!(a.impulse, HALF);
        assert_eq!(b.impulse, -HALF);
        let old = *bs.at(0);
        c.warmstart(ref bs);
        assert!(*bs.at(0) != old);
        c.solve(ref bs, true);
        c.solve(ref bs, false);
        c.writeback_impulses(ref j);
        let [a, b, _] = c.rows;
        let [x, y, w] = j.impulses;
        assert_eq!(x, HALF);
        assert_eq!(y, b.impulse);
        assert_eq!(w, a.impulse);
        let moved = SolverBody {
            position: Pose2 { translation: v(ONE, ONE), ..Default::default() }, ..*bs.at(1),
        };
        let bs = array![*bs.at(0), moved];
        let next = JointConstraintTrait::generate(j, bs.span(), p);
        assert!(next.rows != c.rows);
        for enabled in [JointEnabled::Disabled, JointEnabled::DisabledByAttachedBody].span() {
            j.data.enabled = *enabled;
            let mut c = JointConstraintTrait::generate(j, [].span(), p);
            let mut empty = array![];
            c.warmstart(ref empty);
            c.solve(ref empty, true);
            c.writeback_impulses(ref j);
            assert_eq!(c.num_rows, 0);
            assert_eq!(j.impulses, [x, y, w]);
        }
    }
    #[test]
    fn test_prismatic_free_axis_force_point() {
        let f1: Pose2 = Default::default();
        let f2 = Pose2 { translation: v(ONE, HALF), ..f1 };
        let h = JointConstraintHelperTrait::new(
            f1, f2, Default::default(), f2.translation, JointAxesMask { bits: 6 },
        );
        assert_eq!(h.r1, v(ONE, ZERO));
        assert_eq!(h.lin_err, v(ONE, HALF));
    }
    #[test]
    #[should_panic(expected: 'Joint: missing body')]
    fn test_stale_handle() {
        let (mut j, bs, p) = fixture(3);
        j.body1.generation += 1;
        let _ = JointConstraintTrait::generate(j, bs.span(), p);
    }
    #[test]
    #[should_panic(expected: 'Joint: same body')]
    fn test_same_body() {
        let (mut j, bs, p) = fixture(3);
        j.body2 = j.body1;
        let _ = JointConstraintTrait::generate(j, bs.span(), p);
    }
    #[test]
    #[should_panic(expected: 'Joint: negative input')]
    fn test_negative_mass() {
        let (j, bs, p) = fixture(3);
        let a = SolverBody { ii: -ONE, ..*bs.at(0) };
        let _ = JointConstraintTrait::generate(j, [a, *bs.at(1)].span(), p);
    }
    #[test]
    #[should_panic(expected: 'Joint: nonunit rotation')]
    fn test_bad_rotation() {
        let (mut j, bs, p) = fixture(3);
        j.data.local_frame1.rotation = Rot2 { re: ZERO, im: ZERO };
        let _ = JointConstraintTrait::generate(j, bs.span(), p);
    }
    #[test]
    #[fuzzer(runs: 32, seed: 7)]
    fn fuzz_candidates(x: i16, y: i16) {
        for bits in [3_u8, 6, 7].span() {
            let (mut j, bs, p) = fixture(*bits);
            j
                .data
                .local_frame2
                .translation = v(Fixed { raw: x.into() * 65536 }, Fixed { raw: y.into() * 65536 });
            let mut a = JointConstraintTrait::generate(j, bs.span(), p);
            let mut b = alternatives::generate(j, bs.span(), p);
            assert_eq!(a, b);
            let mut bs1 = array![*bs.at(0), *bs.at(1)];
            let mut bs2 = array![*bs.at(0), *bs.at(1)];
            a.solve(ref bs1, true);
            b.solve(ref bs2, true);
            assert_eq!(a, b);
            assert_eq!(*bs1.at(0), *bs2.at(0));
            assert_eq!(*bs1.at(1), *bs2.at(1));
        }
    }
    #[test]
    fn test_threshold_boundary_zero_dt_and_dependent_rows() {
        for raw in [0_i64, 1, 6, 7, 8, 9].span() {
            assert_eq!(rigid_cfm(Fixed { raw: *raw }).raw, if *raw < 8 {
                0
            } else {
                *raw
            });
        }
        let (j, bs, mut p) = fixture(7);
        p.dt = ZERO;
        let mut c = JointConstraintTrait::generate(j, bs.span(), p);
        let [a, b, d] = c.rows;
        assert_eq!(a.cfm_coeff, ZERO);
        assert_eq!(b.cfm_coeff, ZERO);
        assert_eq!(d.cfm_coeff, ZERO);
        // A body with translation locked has linearly dependent angular/linear rows.
        let b1 = SolverBody { im: Default::default(), ii: ONE, ..*bs.at(0) };
        let b2 = SolverBody { im: Default::default(), ii: ZERO, ..*bs.at(1) };
        c = JointConstraintTrait::generate(j, [b1, b2].span(), Default::default());
        let [a, b, d] = c.rows;
        assert_eq!(a.inv_lhs, ONE);
        assert_eq!(b.inv_lhs, ZERO);
        assert_eq!(d.inv_lhs, ZERO);
    }
    #[test]
    fn test_reserved_state_and_extreme_impulse_copy() {
        let (mut j, bs, p) = fixture(6);
        j.data.limit_axes.bits = 7;
        j.data.motor_axes.bits = 7;
        j.data.coupled_axes.bits = 7;
        let mut c = JointConstraintTrait::generate(j, bs.span(), p);
        let [mut a, mut b, d] = c.rows;
        a.impulse = fixed::MAX;
        b.impulse = fixed::MIN;
        c.rows = [a, b, d];
        let before = j.data;
        c.writeback_impulses(ref j);
        assert_eq!(j.data, before);
        assert_eq!(j.impulses, [HALF, fixed::MIN, fixed::MAX]);
    }
    #[test]
    #[should_panic(expected: 'Fixed: overflow')]
    fn test_unrepresentable_inverse_mass_panics() {
        let (j, bs, p) = fixture(3);
        let b1 = SolverBody { im: v(Fixed { raw: 1 }, ZERO), ii: ZERO, ..*bs.at(0) };
        let b2 = SolverBody { im: Default::default(), ii: ZERO, ..*bs.at(1) };
        let _ = JointConstraintTrait::generate(j, [b1, b2].span(), p);
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }
    fn bench(bits: u8, generic: bool) {
        let (j, mut bs, p) = fixture(opaque(bits));
        let mut c = if generic {
            alternatives::generate(opaque(j), bs.span(), opaque(p))
        } else {
            JointConstraintTrait::generate(opaque(j), bs.span(), opaque(p))
        };
        c.solve(ref bs, true);
        let _ = opaque((c, *bs.at(0), *bs.at(1)));
    }
    #[test]
    fn gas_fixed_specialised_generate_solve() {
        bench(7, false);
    }
    #[test]
    fn gas_fixed_generic_generate_solve() {
        bench(7, true);
    }
    #[test]
    fn gas_revolute_specialised_generate_solve() {
        bench(3, false);
    }
    #[test]
    fn gas_revolute_generic_generate_solve() {
        bench(3, true);
    }
    #[test]
    fn gas_prismatic_specialised_generate_solve() {
        bench(6, false);
    }
    #[test]
    fn gas_prismatic_generic_generate_solve() {
        bench(6, true);
    }
    fn probe(op: u8) {
        let (mut j, mut bs, mut p) = fixture(7);
        p.warmstart_joints = true;
        let mut c = opaque(JointConstraintTrait::generate(opaque(j), bs.span(), opaque(p)));
        match op {
            0 => c.warmstart(ref bs),
            1 => c.solve(ref bs, true),
            2 => c.remove_bias(),
            3 => c.writeback_impulses(ref j),
            _ => {},
        }
        let _ = opaque((c, j, *bs.at(0), *bs.at(1)));
    }
    #[test]
    fn gas_generate() {
        probe(4);
    }
    #[test]
    fn gas_warmstart() {
        probe(0);
    }
    #[test]
    fn gas_solve() {
        probe(1);
    }
    #[test]
    fn gas_remove_bias() {
        probe(2);
    }
    #[test]
    fn gas_writeback_impulses() {
        probe(3);
    }
    #[test]
    fn gas_helper_new() {
        let (j, bs, _) = fixture(7);
        let _ = opaque(
            JointConstraintHelperTrait::new(
                opaque(j.data.local_frame1),
                opaque(j.data.local_frame2),
                opaque(*bs.at(0).position.translation),
                opaque(*bs.at(1).position.translation),
                opaque(j.data.locked_axes),
            ),
        );
    }
    fn helper_probe(op: u8) {
        let (j, bs, p) = fixture(7);
        let (mut c, h, b1, b2, erp, cfm) = prepare(opaque(j), bs.span(), opaque(p));
        let a = if op == 0 {
            h.lock_linear(opaque(1), b1, b2, erp, cfm)
        } else {
            h.lock_angular(b1, b2, erp, cfm)
        };
        if op == 2 {
            c.rows = [a, h.lock_linear(0, b1, b2, erp, cfm), h.lock_linear(1, b1, b2, erp, cfm)];
            c.num_rows = 3;
            JointConstraintHelperTrait::finalize(ref c);
        }
        let _ = opaque((a, c));
    }
    #[test]
    fn gas_lock_linear() {
        helper_probe(0);
    }
    #[test]
    fn gas_lock_angular() {
        helper_probe(1);
    }
    #[test]
    fn gas_finalize() {
        helper_probe(2);
    }
}
