//! Physics-side maths for rapier.cairo, on top of the shared `fixed` scalar from glam.cairo.
//!
//! This crate plays the role of upstream's `glamx`: `Rot2`, `Pose2`, the fused kernels the engine
//! needs and the scalar helpers Rapier/Parry rely on (`inv(0) = 0`, wide comparisons of squared
//! quantities, angular constants). See `docs/PLAN.md` (D12) for the split with glam.cairo.

pub mod consts;
pub mod math_ext;

pub use consts::{
    COS_10_DEGREES, COS_1_DEGREES, COS_45_DEGREES, COS_5_DEGREES, COS_FRAC_PI_8, DEFAULT_EPSILON,
    DEFAULT_EPSILON_SQ_RAW, DIST_SQ_THRESHOLD_RAW, EPA_EPS_TOL, GJK_EPS_REL, GJK_EPS_TOL,
    GJK_EPS_TOL_SQ_RAW, ONE_SQ_RAW, SIN_10_DEGREES, SIN_45_DEGREES, SIN_FRAC_PI_8, UNIT_TOL_SQ_RAW,
    UNIT_TOL_ULPS,
};
pub use math_ext::norm2::{
    Cmp, is_norm2_between, is_norm2_ge, is_norm2_ge_raw, is_norm2_gt, is_norm2_le, is_norm2_lt,
    is_norm2_lt_raw, is_unit2, is_unit2_raw, is_zero2, norm2_cmp, norm2_sq_wide, sq_wide,
};
pub use math_ext::scalar::{
    cap_magnitude, copy_sign_to, inv, smallest_abs_component, smallest_abs_component_index,
};
pub use math_ext::vec2::{
    cap_magnitude2, gcross_sv, gcross_vs, gcross_vv, gdot, orthonormal_vector, perp, try_normalize2,
    try_normalize2_and_length, try_normalize2_eps,
};

#[cfg(test)]
mod tests {
    use fixed::wide::dot2;
    use fixed::{Fixed, FixedTrait, ONE};
    use rapier_testing::opaque;

    /// Empty probe: harness overhead to subtract from other entries of `.gas-snapshot`.
    #[test]
    fn gas_baseline() {}

    /// Confirms the shared scalar links and behaves as documented: `dot2` rescales once and
    /// `sqrt` is exact on a perfect square.
    #[test]
    fn gas_fixed_dot2_sqrt() {
        let a: Fixed = FixedTrait::from_int(opaque(3));
        let b: Fixed = FixedTrait::from_int(opaque(4));
        assert_eq!(dot2(a, a, b, b).sqrt(), FixedTrait::from_int(5));
        assert_eq!(ONE * ONE, ONE);
    }
}
