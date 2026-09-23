//! Simulation parameters (upstream `rapier::dynamics::IntegrationParameters`).
//!
//! Every upstream field that survives the plan's cuts is kept, with upstream's defaults. The CCD
//! (`min_ccd_dt`, `max_ccd_substeps`), contact clustering / recycling and `warmstart_joints`
//! fields are plain data for now: nothing in this port reads them yet, they exist so that the
//! defaults match the golden fixtures field by field. The soft-body settings and the 3D-only
//! `friction_model` are cut.
//!
//! The quantities the solver derives from the parameters are methods mirroring upstream names.
//! The spring maths lives in [`spring`];
//! [`IntegrationParametersTrait::contact_softness_coefficients`]
//! and friends evaluate it at the **substep** length, once per step, so that the substep loop only
//! reads precomputed [`SoftnessCoefficients`].
//!
//! Rounding: `*` floors and `/` rounds to nearest, ties to even like every `fixed::Fixed`
//! operator (see the item docs); raw substep splitting truncates by construction.

pub mod spring;
use fixed::{Fixed, MAX, ONE, ZERO};
pub use spring::{
    SoftnessCoefficients, SpringCoefficients, SpringCoefficientsImpl, SpringCoefficientsTrait,
};

/// Failure modes of [`IntegrationParametersTrait`].
pub mod errors {
    /// `num_solver_iterations` is zero, so the substep length is undefined (upstream: `inf`).
    pub const ZERO_ITERATIONS: felt252 = 'IntegrationParams: zero iters';
}

/// Configuration parameters that control the physics simulation quality and behaviour.
///
/// Reals are Q32.32 [`Fixed`] values; counts are `u32` (upstream `usize`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct IntegrationParameters {
    /// Timestep length in seconds (default `1 / 60`, i.e. raw `71582788`, rounded to nearest).
    pub dt: Fixed,
    /// Minimum timestep when CCD subdivides a step (default `1 / 60 / 100`, raw `715828`).
    /// Unused for now (CCD is out of scope).
    pub min_ccd_dt: Fixed,
    /// Softness of contacts between two dynamic bodies (default 30 Hz, ζ = 10).
    pub contact_softness: SpringCoefficients,
    /// Softness of contacts touching a fixed body (default 60 Hz, ζ = 10).
    pub static_contact_softness: SpringCoefficients,
    /// Coefficient in `[0, 1]` applied to warmstart impulses (default `1`).
    pub warmstart_coefficient: Fixed,
    /// Scale factor of the world when it is not in meters (default `1`); scales every
    /// `normalized_*` length below.
    pub length_unit: Fixed,
    /// Geometric slop distance, before `length_unit` (default `0.005`, raw `21474836`).
    pub normalized_allowed_linear_error: Fixed,
    /// Maximum speed at which contact penetration is pushed out, before `length_unit`
    /// (default `3`). [`MAX`] disables the cap.
    pub normalized_max_corrective_velocity: Fixed,
    /// Maximal distance separating two objects that generate predictive contacts, before
    /// `length_unit` (default `0.02`, raw `85899346`).
    pub normalized_prediction_distance: Fixed,
    /// Maximum linear velocity of a body after each substep, before `length_unit` (default `400`).
    /// [`MAX`] disables the cap.
    pub normalized_max_linear_velocity: Fixed,
    /// Number of solver iterations, i.e. substeps per step (default `4`).
    pub num_solver_iterations: u32,
    /// Internal PGS iterations per solver iteration (default `1`).
    pub num_internal_pgs_iterations: u32,
    /// Stabilization iterations per solver iteration (default `1`).
    pub num_internal_stabilization_iterations: u32,
    /// Maximum number of CCD substeps; `0` disables all CCD (default `1`). Unused for now.
    pub max_ccd_substeps: u32,
    /// Merge manifolds sharing a normal into one cluster (default `true`, 3D only). Unused.
    pub contact_clustering: bool,
    /// Skip narrow-phase updates of pairs that barely moved (default `true`). Unused for now.
    pub contact_recycling: bool,
    /// Maximum relative-pose drift below which a pair is recycled, before `length_unit`
    /// (default `0.05`, raw `214748365`). Unused for now.
    pub normalized_contact_recycle_distance: Fixed,
    /// Solve friction in the biased pass too (default `false`).
    pub friction_in_bias_pass: bool,
    /// Warmstart impulse joints like contacts (default `false`). Unused for now.
    pub warmstart_joints: bool,
}

/// The parameters of `IntegrationParameters::default()` upstream, as raw Q32.32 where rounded to
/// nearest.
const DEFAULT_PARAMETERS: IntegrationParameters = IntegrationParameters {
    dt: Fixed { raw: 71582788 },
    min_ccd_dt: Fixed { raw: 715828 },
    contact_softness: spring::CONTACT_DEFAULTS,
    static_contact_softness: spring::CONTACT_STATIC_DEFAULTS,
    warmstart_coefficient: ONE,
    length_unit: ONE,
    normalized_allowed_linear_error: Fixed { raw: 21474836 },
    normalized_max_corrective_velocity: Fixed { raw: 12884901888 },
    normalized_prediction_distance: Fixed { raw: 85899346 },
    normalized_max_linear_velocity: Fixed { raw: 1717986918400 },
    num_solver_iterations: 4,
    num_internal_pgs_iterations: 1,
    num_internal_stabilization_iterations: 1,
    max_ccd_substeps: 1,
    contact_clustering: true,
    contact_recycling: true,
    normalized_contact_recycle_distance: Fixed { raw: 214748365 },
    friction_in_bias_pass: false,
    warmstart_joints: false,
};

/// Upstream defaults: 60 Hz, 4 solver iterations, meters, 30 Hz / ζ = 10 contacts.
pub impl IntegrationParametersDefault of Default<IntegrationParameters> {
    #[inline(always)]
    fn default() -> IntegrationParameters {
        DEFAULT_PARAMETERS
    }
}

/// Derived quantities of [`IntegrationParameters`].
#[generate_trait]
pub impl IntegrationParametersImpl of IntegrationParametersTrait {
    /// Inverse of the step length (steps per second); zero when `dt` is zero. Rounded to nearest
    /// (one `Fixed` division).
    ///
    /// # Panics
    /// * `Fixed: overflow` if `1 / dt` exceeds the Q32.32 range (`dt < 2^-31`).
    fn inv_dt(self: IntegrationParameters) -> Fixed {
        inv_or_zero(self.dt)
    }

    /// Sets the step length from a frequency: `dt = 1 / inv_dt`, or zero when `inv_dt` is zero.
    /// Rounded to nearest.
    ///
    /// # Panics
    /// * `Fixed: overflow` if `1 / inv_dt` exceeds the Q32.32 range.
    fn set_inv_dt(ref self: IntegrationParameters, inv_dt: Fixed) {
        self.dt = inv_or_zero(inv_dt);
    }

    /// Length of one solver substep, `dt / num_solver_iterations` (upstream divides `dt` in place
    /// in the island solver). Truncated toward zero: `raw / n`.
    ///
    /// # Panics
    /// * `IntegrationParams: zero iters` if `num_solver_iterations` is zero.
    fn substep_dt(self: IntegrationParameters) -> Fixed {
        assert(self.num_solver_iterations != 0, errors::ZERO_ITERATIONS);
        Fixed { raw: self.dt.raw / self.num_solver_iterations.into() }
    }

    /// Inverse of [`Self::substep_dt`]; zero when it is zero. Rounded to nearest.
    ///
    /// # Panics
    /// * `IntegrationParams: zero iters` if `num_solver_iterations` is zero.
    /// * `Fixed: overflow` if the inverse exceeds the Q32.32 range.
    fn substep_inv_dt(self: IntegrationParameters) -> Fixed {
        inv_or_zero(self.substep_dt())
    }

    /// Amount of penetration the engine will not correct:
    /// `normalized_allowed_linear_error * length_unit`, floored.
    ///
    /// # Panics
    /// * `Fixed: overflow` if the product exceeds the Q32.32 range.
    fn allowed_linear_error(self: IntegrationParameters) -> Fixed {
        self.normalized_allowed_linear_error * self.length_unit
    }

    /// Maximum speed at which penetration is corrected:
    /// `normalized_max_corrective_velocity * length_unit` (floored), or [`MAX`] when the
    /// normalized value is [`MAX`] (cap disabled).
    ///
    /// # Panics
    /// * `Fixed: overflow` if the product exceeds the Q32.32 range.
    fn max_corrective_velocity(self: IntegrationParameters) -> Fixed {
        scale_or_max(self.normalized_max_corrective_velocity, self.length_unit)
    }

    /// Maximal distance separating two objects that generate predictive contacts:
    /// `normalized_prediction_distance * length_unit`, floored.
    ///
    /// # Panics
    /// * `Fixed: overflow` if the product exceeds the Q32.32 range.
    fn prediction_distance(self: IntegrationParameters) -> Fixed {
        self.normalized_prediction_distance * self.length_unit
    }

    /// Maximum linear velocity after each substep:
    /// `normalized_max_linear_velocity * length_unit` (floored), or [`MAX`] when the normalized
    /// value is [`MAX`] (cap disabled).
    ///
    /// # Panics
    /// * `Fixed: overflow` if the product exceeds the Q32.32 range.
    fn max_linear_velocity(self: IntegrationParameters) -> Fixed {
        scale_or_max(self.normalized_max_linear_velocity, self.length_unit)
    }

    /// Relative-pose drift below which a contact pair can be recycled:
    /// `normalized_contact_recycle_distance * length_unit`, floored.
    ///
    /// # Panics
    /// * `Fixed: overflow` if the product exceeds the Q32.32 range.
    fn contact_recycle_distance(self: IntegrationParameters) -> Fixed {
        self.normalized_contact_recycle_distance * self.length_unit
    }

    /// Softness coefficients of dynamic–dynamic contacts at the substep length. Compute once per
    /// step (four `u128` divisions), not per substep.
    ///
    /// # Panics
    /// * `IntegrationParams: zero iters` if `num_solver_iterations` is zero.
    /// * The panics of [`SpringCoefficientsTrait::coefficients`].
    fn contact_softness_coefficients(self: IntegrationParameters) -> SoftnessCoefficients {
        self.contact_softness.coefficients(self.substep_dt())
    }

    /// Softness coefficients of contacts touching a fixed body at the substep length (upstream
    /// `static_contact_softness`). Compute once per step.
    ///
    /// # Panics
    /// * `IntegrationParams: zero iters` if `num_solver_iterations` is zero.
    /// * The panics of [`SpringCoefficientsTrait::coefficients`].
    fn static_contact_softness_coefficients(self: IntegrationParameters) -> SoftnessCoefficients {
        self.static_contact_softness.coefficients(self.substep_dt())
    }

    /// Softness coefficients of a joint with the given softness (upstream keeps it on the joint:
    /// `GenericJoint::softness`, default [`SpringCoefficientsTrait::joint_defaults`]) at the
    /// substep length. Compute once per step and joint softness.
    ///
    /// # Panics
    /// * `IntegrationParams: zero iters` if `num_solver_iterations` is zero.
    /// * The panics of [`SpringCoefficientsTrait::coefficients`].
    fn joint_softness_coefficients(
        self: IntegrationParameters, softness: SpringCoefficients,
    ) -> SoftnessCoefficients {
        softness.coefficients(self.substep_dt())
    }
}

/// `1 / x`, or zero for `x = 0` (upstream `inv_dt`).
#[inline(always)]
fn inv_or_zero(x: Fixed) -> Fixed {
    if x.raw == 0 {
        ZERO
    } else {
        ONE / x
    }
}

/// `normalized * length_unit`, keeping the `MAX` "disabled" sentinel untouched (upstream
/// `Real::MAX`).
#[inline(always)]
fn scale_or_max(normalized: Fixed, length_unit: Fixed) -> Fixed {
    if normalized == MAX {
        MAX
    } else {
        normalized * length_unit
    }
}

#[cfg(test)]
mod alternatives {
    //! Rejected candidates for the per-step derived quantities, kept for re-ranking.
    use fixed::wide::RecipTrait;
    use fixed::{Fixed, FixedTrait, ONE, ZERO};
    use super::IntegrationParameters;

    /// `dt / Fixed::from_int(n)`: the literal port of upstream's `dt /= n as Real`; two shifts and
    /// a division of wide operands instead of one 64-bit division.
    pub fn substep_dt_fixed_div(p: IntegrationParameters) -> Fixed {
        p.dt / FixedTrait::from_int(p.num_solver_iterations.try_into().unwrap())
    }

    /// `1 / x` through [`RecipTrait`]: a wide reciprocal multiply, one more step.
    pub fn inv_recip(x: Fixed) -> Fixed {
        if x.raw == 0 {
            ZERO
        } else {
            RecipTrait::new(x).mul(ONE)
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, MAX, ONE, ZERO};
    use rapier_testing::opaque;
    use super::alternatives::{inv_recip, substep_dt_fixed_div};
    use super::{
        IntegrationParameters, IntegrationParametersTrait, SoftnessCoefficients, SpringCoefficients,
        SpringCoefficientsTrait, inv_or_zero,
    };

    fn defaults() -> IntegrationParameters {
        Default::default()
    }

    #[test]
    fn test_default_scalars() {
        let p = defaults();
        assert_eq!(p.dt.raw, 71582788);
        assert_eq!(p.min_ccd_dt.raw, 715828);
        assert_eq!(p.warmstart_coefficient, ONE);
        assert_eq!(p.length_unit, ONE);
        assert_eq!(p.num_solver_iterations, 4);
        assert_eq!(p.num_internal_pgs_iterations, 1);
        assert_eq!(p.num_internal_stabilization_iterations, 1);
        assert_eq!(p.max_ccd_substeps, 1);
        assert!(p.contact_clustering && p.contact_recycling);
        assert!(!p.friction_in_bias_pass && !p.warmstart_joints);
        assert_eq!(p.contact_softness, SpringCoefficientsTrait::contact_defaults());
        assert_eq!(p.static_contact_softness, SpringCoefficientsTrait::contact_static_defaults());
    }

    #[test]
    fn test_substep_and_inverses_at_defaults() {
        let p = defaults();
        assert_eq!(p.inv_dt().raw, 257698038720);
        assert_eq!(p.substep_dt().raw, 17895697);
        assert_eq!(p.substep_inv_dt().raw, 1030792154880);
    }

    #[test]
    fn test_substep_dt_truncates_toward_zero() {
        let mut p = defaults();
        p.dt = Fixed { raw: 10 };
        p.num_solver_iterations = 3;
        assert_eq!(p.substep_dt().raw, 3);
        p.num_solver_iterations = 1;
        assert_eq!(p.substep_dt(), p.dt);
        // More iterations than raw units: the substep vanishes and so does its inverse.
        p.num_solver_iterations = 11;
        assert_eq!(p.substep_dt(), ZERO);
        assert_eq!(p.substep_inv_dt(), ZERO);
    }

    #[test]
    fn test_zero_dt_mirrors_upstream() {
        // inv_dt() is zero when dt is zero; the spring coefficients then follow the `erp = 0` rule.
        let mut p = defaults();
        p.dt = ZERO;
        assert_eq!(p.inv_dt(), ZERO);
        assert_eq!(p.substep_dt(), ZERO);
        assert_eq!(p.substep_inv_dt(), ZERO);
        let c = p.contact_softness_coefficients();
        assert_eq!(c.erp, ZERO);
        assert_eq!(c.cfm_coeff, ZERO);
        assert_eq!(c.cfm_factor, ONE);
    }

    #[test]
    fn test_set_inv_dt() {
        let mut p = defaults();
        p.set_inv_dt(Fixed { raw: 60 * 0x100000000 });
        // 1 / 60 rounded to nearest.
        assert_eq!(p.dt.raw, 71582788);
        p.set_inv_dt(ZERO);
        assert_eq!(p.dt, ZERO);
    }

    #[test]
    #[should_panic(expected: ('IntegrationParams: zero iters',))]
    fn test_zero_iterations_panics_in_substep_dt() {
        let mut p = defaults();
        p.num_solver_iterations = 0;
        p.substep_dt();
    }

    #[test]
    #[should_panic(expected: ('IntegrationParams: zero iters',))]
    fn test_zero_iterations_panics_in_softness() {
        let mut p = defaults();
        p.num_solver_iterations = 0;
        p.contact_softness_coefficients();
    }

    #[test]
    fn test_length_unit_scales_normalized_lengths() {
        let mut p = defaults();
        p.length_unit = Fixed { raw: 100 * 0x100000000 };
        assert_eq!(p.allowed_linear_error().raw, 21474836 * 100);
        assert_eq!(p.prediction_distance().raw, 85899346 * 100);
        assert_eq!(p.contact_recycle_distance().raw, 214748365 * 100);
        assert_eq!(p.max_corrective_velocity().raw, 3 * 100 * 0x100000000);
        assert_eq!(p.max_linear_velocity().raw, 400 * 100 * 0x100000000);
    }

    #[test]
    fn test_disabled_caps_keep_the_max_sentinel() {
        let mut p = defaults();
        p.normalized_max_corrective_velocity = MAX;
        p.normalized_max_linear_velocity = MAX;
        // Scaling MAX would overflow: upstream returns Real::MAX untouched.
        p.length_unit = Fixed { raw: 3 * 0x100000000 };
        assert_eq!(p.max_corrective_velocity(), MAX);
        assert_eq!(p.max_linear_velocity(), MAX);
    }

    #[test]
    #[should_panic(expected: ('Fixed: overflow',))]
    fn test_scaled_velocity_overflow_panics() {
        let mut p = defaults();
        // 400 · (2^31 − 1) does not fit Q32.32.
        p.length_unit = Fixed { raw: 0x7fffffff00000000 };
        p.max_linear_velocity();
    }

    #[test]
    fn test_coefficients_match_the_spring_functions() {
        let p = defaults();
        let sub = p.substep_dt();
        assert_eq!(p.contact_softness_coefficients(), p.contact_softness.coefficients(sub));
        assert_eq!(
            p.static_contact_softness_coefficients(), p.static_contact_softness.coefficients(sub),
        );
        let joint = SpringCoefficientsTrait::joint_defaults();
        assert_eq!(p.joint_softness_coefficients(joint), joint.coefficients(sub));
    }

    #[test]
    fn test_static_contacts_are_stiffer() {
        let p = defaults();
        let dynamic = p.contact_softness_coefficients();
        let fixed_body = p.static_contact_softness_coefficients();
        assert!(fixed_body.erp > dynamic.erp);
        assert!(fixed_body.cfm_coeff < dynamic.cfm_coeff);
    }

    #[test]
    fn test_alternatives_agree_with_shipped() {
        let p = defaults();
        assert_eq!(substep_dt_fixed_div(p), p.substep_dt());
        // The wide reciprocal multiply differs from `/` by at most one ulp.
        let d = inv_recip(p.dt).raw - p.inv_dt().raw;
        assert!(d >= 0 && d <= 1);
        assert_eq!(inv_recip(ZERO), ZERO);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_default() {
        let p: IntegrationParameters = opaque(Default::default());
        assert_eq!(p.num_solver_iterations, 4);
    }

    #[test]
    fn gas_inv_dt() {
        assert_eq!(opaque(defaults()).inv_dt().raw, 257698038720);
    }

    #[test]
    fn gas_substep_dt() {
        assert_eq!(opaque(defaults()).substep_dt().raw, 17895697);
    }

    #[test]
    fn gas_substep_dt_fixed_div() {
        assert_eq!(substep_dt_fixed_div(opaque(defaults())).raw, 17895697);
    }

    #[test]
    fn gas_substep_inv_dt() {
        assert_eq!(opaque(defaults()).substep_inv_dt().raw, 1030792154880);
    }

    #[test]
    fn gas_inv_recip() {
        assert_eq!(inv_recip(opaque(defaults().dt)).raw, 257698038720);
    }

    #[test]
    fn gas_inv_or_zero_div() {
        // Same input shape as `gas_inv_recip`: the `1 / x` kernel of `inv_dt`.
        assert_eq!(inv_or_zero(opaque(defaults().dt)).raw, 257698038720);
    }

    #[test]
    fn gas_max_linear_velocity() {
        assert_eq!(opaque(defaults()).max_linear_velocity().raw, 1717986918400);
    }

    #[test]
    fn gas_contact_softness_coefficients() {
        let c: SoftnessCoefficients = opaque(defaults()).contact_softness_coefficients();
        assert_eq!(c.cfm_factor.raw, 4047058867);
    }

    #[test]
    fn gas_static_contact_softness_coefficients() {
        let c: SoftnessCoefficients = opaque(defaults()).static_contact_softness_coefficients();
        assert_eq!(c.cfm_factor.raw, 4171843511);
    }

    #[test]
    fn gas_joint_softness_coefficients() {
        let joint: SpringCoefficients = opaque(SpringCoefficientsTrait::joint_defaults());
        let c = opaque(defaults()).joint_softness_coefficients(joint);
        assert_eq!(c.cfm_coeff.raw, 6);
    }
}
