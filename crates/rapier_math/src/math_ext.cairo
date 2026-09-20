//! The scalar and 2D helpers Rapier and Parry rely on, on top of the shared `fixed` scalar.
//!
//! Upstream keeps them in `rapier::utils` and `parry::utils`; they are split here by the kind of
//! hazard they answer:
//!
//! * [`scalar`] — `inv(0) = 0`, `copy_sign_to`, magnitude clamps, component selection;
//! * [`norm2`] — comparisons of squared lengths kept wide (the fixed-point hazard of the
//!   package: a rescaled square loses half of its bits);
//! * [`vec2`] — `gdot`, `gcross`, `perp`, `orthonormal_vector`, guarded normalisations, on
//!   component tuples (the `Vec2`, `Rot2` and `Pose2` types belong to package M2).
//!
//! The tolerances every one of them is compared against live in `super::consts`.

pub mod norm2;
pub mod scalar;
pub mod vec2;

pub use norm2::{
    Cmp, is_norm2_between, is_norm2_ge, is_norm2_ge_raw, is_norm2_gt, is_norm2_le, is_norm2_lt,
    is_norm2_lt_raw, is_unit2, is_unit2_raw, is_zero2, norm2_cmp, norm2_sq_wide, sq_wide,
};
pub use scalar::{
    cap_magnitude, copy_sign_to, inv, smallest_abs_component, smallest_abs_component_index,
};
pub use vec2::{
    cap_magnitude2, gcross_sv, gcross_vs, gcross_vv, gdot, orthonormal_vector, perp, try_normalize2,
    try_normalize2_and_length, try_normalize2_eps,
};
