//! Golden-vector comparison of `IntegrationParameters` and the spring maths against the `f64`
//! upstream (`rapier2d-f64 0.35.3`), recorded in `rapier_golden::integration_parameters`.
//!
//! Tolerances (raw Q32.32 units, "ulp"):
//! * `dt_q32` is upstream evaluated at a `dt` that is exactly representable, so the port sees the
//!   same inputs and is expected to agree to rounding only. The port floors / truncates, the
//!   `f64` reference is rounded to nearest: 1 ulp. The angular frequency is compared with a
//!   relative tolerance (3.6e-12) because the Q32.32 `TAU` has a relative error of 1.1e-12.
//! * `dt_f64` feeds upstream `1/60` unquantised, so the fixture differs from the port's inputs by
//!   up to 0.5 ulp of `dt`, a relative perturbation of `0.5 / 71582788 = 7e-9` (`2.8e-8` at the
//!   substep, `0.5 / 17895697`). `erp_inv_dt`, `erp` and `cfm_factor` are (at most) linear in that
//!   perturbation and `cfm_coeff` quadratic, hence the relative tolerance `2^-25` (3e-8) `+ 8`.

use fixed::{Fixed, MAX, ONE};
use rapier_core::integration_parameters::{
    IntegrationParameters, IntegrationParametersTrait, SoftnessCoefficients, SpringCoefficients,
    SpringCoefficientsTrait,
};
use rapier_golden::compare::{abs_diff, within};
use rapier_golden::integration_parameters::{DEFAULTS, DT_F64, DT_Q32};
use rapier_golden::types::{IntegrationDerivedRaw, SpringDefaultsRaw, SpringDerivedRaw};

fn params_with_dt(dt: i64) -> IntegrationParameters {
    let mut p: IntegrationParameters = Default::default();
    p.dt = Fixed { raw: dt };
    p
}

fn assert_spring_defaults(actual: SpringCoefficients, expected: SpringDefaultsRaw) {
    assert_eq!(actual.natural_frequency.raw, expected.natural_frequency);
    assert_eq!(actual.damping_ratio.raw, expected.damping_ratio);
    // `ω = natural_frequency * TAU` with the Q32.32 `TAU` (relative error 1.1e-12): 1 ulp for the
    // contact springs, 44038 ulp for the joint spring. Tolerance `expected / 2^38 + 4` (3.6e-12).
    let tolerance: u64 = (expected.angular_frequency / 0x4000000000).try_into().unwrap() + 4;
    assert!(within(actual.angular_frequency().raw, expected.angular_frequency, tolerance));
}

/// `|actual - expected|`, in raw units.
fn ulps(actual: Fixed, expected: i64) -> u64 {
    abs_diff(actual.raw, expected)
}

/// Relative tolerance `expected / 2^25 + 8` for the unquantised `dt_f64` case.
fn relative_tolerance(expected: i64) -> u64 {
    let magnitude: u64 = if expected < 0 {
        (-expected).try_into().unwrap()
    } else {
        expected.try_into().unwrap()
    };
    magnitude / 0x2000000 + 8
}

fn assert_spring_within(
    actual: SoftnessCoefficients, expected: SpringDerivedRaw, absolute: bool, tolerance: u64,
) {
    let (a, e, c, f) = if absolute {
        (tolerance, tolerance, tolerance, tolerance)
    } else {
        (
            relative_tolerance(expected.erp_inv_dt),
            relative_tolerance(expected.erp),
            relative_tolerance(expected.cfm_coeff),
            relative_tolerance(expected.cfm_factor),
        )
    };
    assert!(ulps(actual.erp_inv_dt, expected.erp_inv_dt) <= a, "erp_inv_dt");
    assert!(ulps(actual.erp, expected.erp) <= e, "erp");
    assert!(ulps(actual.cfm_coeff, expected.cfm_coeff) <= c, "cfm_coeff");
    assert!(ulps(actual.cfm_factor, expected.cfm_factor) <= f, "cfm_factor");
}

fn scalar_within(absolute: bool, tolerance: u64, actual: Fixed, expected: i64) -> bool {
    if absolute {
        ulps(actual, expected) <= tolerance
    } else {
        ulps(actual, expected) <= relative_tolerance(expected)
    }
}

fn assert_derived_within(
    p: IntegrationParameters, expected: IntegrationDerivedRaw, absolute: bool, tolerance: u64,
) {
    assert_eq!(p.dt.raw, expected.dt);
    assert_eq!(p.num_solver_iterations, expected.num_solver_iterations);
    // `inv_dt` of the unquantised case is 1/60 exactly, not 1 / quantised dt: relative tolerance.
    assert!(scalar_within(absolute, tolerance, p.inv_dt(), expected.inv_dt), "inv_dt");
    assert!(scalar_within(absolute, tolerance, p.substep_dt(), expected.substep_dt), "substep_dt");
    assert!(
        scalar_within(absolute, tolerance, p.substep_inv_dt(), expected.substep_inv_dt),
        "substep_inv_dt",
    );
    assert!(
        scalar_within(absolute, tolerance, p.allowed_linear_error(), expected.allowed_linear_error),
        "allowed_linear_error",
    );
    assert!(
        scalar_within(
            absolute, tolerance, p.max_corrective_velocity(), expected.max_corrective_velocity,
        ),
        "max_corrective_velocity",
    );
    assert!(
        scalar_within(absolute, tolerance, p.prediction_distance(), expected.prediction_distance),
        "prediction_distance",
    );
    assert!(
        scalar_within(absolute, tolerance, p.max_linear_velocity(), expected.max_linear_velocity),
        "max_linear_velocity",
    );
    assert!(
        scalar_within(
            absolute, tolerance, p.contact_recycle_distance(), expected.contact_recycle_distance,
        ),
        "contact_recycle_distance",
    );
    assert_spring_within(p.contact_softness_coefficients(), expected.contact, absolute, tolerance);
    assert_spring_within(
        p.static_contact_softness_coefficients(), expected.static_contact, absolute, tolerance,
    );
    assert_spring_within(
        p.joint_softness_coefficients(SpringCoefficientsTrait::joint_defaults()),
        expected.joint,
        absolute,
        tolerance,
    );
}

#[test]
fn test_defaults_match_golden() {
    let p: IntegrationParameters = Default::default();
    assert_eq!(p.dt.raw, DEFAULTS.dt);
    assert_eq!(p.min_ccd_dt.raw, DEFAULTS.min_ccd_dt);
    assert_spring_defaults(p.contact_softness, DEFAULTS.contact_softness);
    assert_spring_defaults(p.static_contact_softness, DEFAULTS.static_contact_softness);
    // The joint softness lives on the joint upstream (`GenericJoint::default().softness`).
    assert_spring_defaults(SpringCoefficientsTrait::joint_defaults(), DEFAULTS.joint_softness);
    assert_eq!(p.warmstart_coefficient.raw, DEFAULTS.warmstart_coefficient);
    assert_eq!(p.length_unit.raw, DEFAULTS.length_unit);
    assert_eq!(p.normalized_allowed_linear_error.raw, DEFAULTS.normalized_allowed_linear_error);
    assert_eq!(
        p.normalized_max_corrective_velocity.raw, DEFAULTS.normalized_max_corrective_velocity,
    );
    assert_eq!(p.normalized_prediction_distance.raw, DEFAULTS.normalized_prediction_distance);
    assert_eq!(p.normalized_max_linear_velocity.raw, DEFAULTS.normalized_max_linear_velocity);
    assert_eq!(
        p.normalized_contact_recycle_distance.raw, DEFAULTS.normalized_contact_recycle_distance,
    );
    assert_eq!(p.num_solver_iterations, DEFAULTS.num_solver_iterations);
    assert_eq!(p.num_internal_pgs_iterations, DEFAULTS.num_internal_pgs_iterations);
    assert_eq!(
        p.num_internal_stabilization_iterations, DEFAULTS.num_internal_stabilization_iterations,
    );
    assert_eq!(p.max_ccd_substeps, DEFAULTS.max_ccd_substeps);
    assert_eq!(p.contact_clustering, DEFAULTS.contact_clustering);
    assert_eq!(p.contact_recycling, DEFAULTS.contact_recycling);
    assert_eq!(p.friction_in_bias_pass, DEFAULTS.friction_in_bias_pass);
    assert_eq!(p.warmstart_joints, DEFAULTS.warmstart_joints);
}

#[test]
fn test_derived_match_golden_dt_q32() {
    // Same inputs as upstream. Every derived field is a floor / truncation of the exact value
    // while the `f64` reference is rounded to nearest, so the port lands 0 or 1 ulp below it:
    // tolerance 1 ulp. (The scalars are exact products or truncated quotients of representable
    // values.)
    let p = params_with_dt(DT_Q32.dt);
    assert_derived_within(p, DT_Q32, true, 1);
}

fn assert_below_by_at_most_one(actual: Fixed, expected: i64) {
    let below = expected - actual.raw;
    assert!(below >= 0 && below <= 1, "spring coefficient not 0 or 1 ulp below the reference");
}

#[test]
fn test_derived_dt_q32_springs_are_truncations_of_the_reference() {
    let p = params_with_dt(DT_Q32.dt);
    let contact = p.contact_softness_coefficients();
    let static_contact = p.static_contact_softness_coefficients();
    let joint = p.joint_softness_coefficients(SpringCoefficientsTrait::joint_defaults());
    assert_below_by_at_most_one(contact.erp_inv_dt, DT_Q32.contact.erp_inv_dt);
    assert_below_by_at_most_one(contact.erp, DT_Q32.contact.erp);
    assert_below_by_at_most_one(contact.cfm_coeff, DT_Q32.contact.cfm_coeff);
    assert_below_by_at_most_one(contact.cfm_factor, DT_Q32.contact.cfm_factor);
    assert_below_by_at_most_one(static_contact.erp_inv_dt, DT_Q32.static_contact.erp_inv_dt);
    assert_below_by_at_most_one(static_contact.erp, DT_Q32.static_contact.erp);
    assert_below_by_at_most_one(static_contact.cfm_coeff, DT_Q32.static_contact.cfm_coeff);
    assert_below_by_at_most_one(static_contact.cfm_factor, DT_Q32.static_contact.cfm_factor);
    assert_below_by_at_most_one(joint.erp_inv_dt, DT_Q32.joint.erp_inv_dt);
    assert_below_by_at_most_one(joint.erp, DT_Q32.joint.erp);
    assert_below_by_at_most_one(joint.cfm_coeff, DT_Q32.joint.cfm_coeff);
    assert_below_by_at_most_one(joint.cfm_factor, DT_Q32.joint.cfm_factor);
}

#[test]
fn test_derived_dt_f64_within_tolerance() {
    // The fixture's `dt` is `1/60` as an `f64`, snapped to Q32.32 for the `dt` field only.
    let p = params_with_dt(DT_F64.dt);
    assert_derived_within(p, DT_F64, false, 0);
}

#[test]
fn test_max_sentinel_is_not_scaled() {
    let mut p: IntegrationParameters = Default::default();
    p.normalized_max_linear_velocity = MAX;
    p.length_unit = ONE + ONE;
    assert_eq!(p.max_linear_velocity(), MAX);
}
