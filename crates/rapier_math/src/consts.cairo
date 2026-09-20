//! Tolerances and angular constants of Rapier/Parry, expressed in Q32.32.
//!
//! Every value below is given as a raw `Fixed` (`value = raw / 2^32`) together with the exact
//! decimal it approximates, and every *squared* threshold is given a second time as a pre-scaled
//! raw Q64.64 `i128` (`_SQ_RAW`), because that is the only form in which a squared quantity can be
//! compared without losing half of its bits (see [`super::math_ext::norm2`]).
//!
//! # Why `DEFAULT_EPSILON = 2^-23` (512 ulp)
//!
//! Upstream defines `DEFAULT_EPSILON = Real::EPSILON`, the *relative* spacing of the float format
//! at 1.0 (`2^-23 ≈ 1.19e-7` in f32, `2^-52` in f64), and then uses it as an *absolute* geometric
//! tolerance: `len >= DEFAULT_EPSILON`, `length_squared() > eps * eps`, `denom > eps`. Q32.32 has
//! no relative spacing to borrow: its resolution is uniform and absolute, 1 ulp `= 2^-32 ≈
//! 2.33e-10`, so the tolerance has to be argued in ulps rather than inherited.
//!
//! The argument is the accuracy of a *direction*, since that is what all these guards protect.
//! `fixed::wide::normalize2` divides by a length floored to 1 ulp, so the normalised components
//! carry a relative error of about `1 ulp / L` (plus half an ulp of rounding). Requiring a
//! direction to be meaningful to better than ~0.2 % — 0.11°, an order of magnitude below the 1°
//! that upstream itself treats as "the same normal" ([`COS_1_DEGREES`]) — gives
//! `L >= 2^-32 / 2^-9 = 2^-23`. Below that length a normalised vector is mostly noise: at
//! `L = 1 ulp`, `normalize2` returns a vector that carries no direction information at all.
//!
//! `2^-23` is also exactly upstream's f32 epsilon, so every threshold derived from it
//! (`gjk::eps_tol() = 10 eps`, EPA's `100 eps`, `eps_rel = sqrt(eps_tol)`) keeps its upstream
//! decimal meaning, and all of them stay comfortably representable (`docs/research/
//! 02-parry-analysis.md` §5). It is exactly representable: `2^-23 = 512 ulp`.
//!
//! What it must **not** be used for: `eps * eps = 2^-46` is **zero** in Q32.32 (it is 2^14 times
//! smaller than 1 ulp). A port of `length_squared() > eps * eps` that rescales its product
//! therefore degenerates into `0 > 0` and silently moves the threshold to `2^-16`, a factor 128.
//! Compare with [`DEFAULT_EPSILON_SQ_RAW`] through the helpers of
//! [`super::math_ext::norm2`] instead.
//!
//! # Zero and unit tolerances
//!
//! * **zero**: none. `is_zero2` compares the raw Q64.64 sum of squares against 0, which is exact
//!   for every input (`x^2 + y^2 = 0` iff both raws are 0) and never underflows, so no tolerance
//!   has to be invented. Use `is_norm2_lt(x, y, DEFAULT_EPSILON)` when what is wanted is a
//!   *degeneracy* test rather than an exact zero test.
//! * **unit**: [`UNIT_TOL_ULPS`] `= 8` ulp on the squared norm. `normalize2` of an input of
//!   length `L >= 1` returns a vector whose length differs from 1 by at most
//!   `1 ulp / L + sqrt(2)/2 ulp <= 1.71 ulp`, hence `|L'^2 - 1| <= 2 * 1.71 + O(ulp^2) ≈ 3.5
//!   ulp`;
//!   8 ulp leaves a factor-2 margin. Inputs shorter than 1 lose accuracy linearly in `L`
//!   (`1 ulp / L`), so a vector normalised from a length of `2^-23` needs ~2^9 ulp instead: pass
//!   the tolerance explicitly there.

use fixed::Fixed;

/// `cos(1°) ≈ 0.99984769515` (upstream `parry::utils::COS_1_DEGREES`), the dot-product threshold
/// under which two normals are considered the same (manifold fast path).
///
/// Raw 4294313152; `1 - cos(1°)` is 654144 ulp, so the constant is far from saturating at 1.
pub const COS_1_DEGREES: Fixed = Fixed { raw: 4294313152 };

/// `cos(5°) ≈ 0.99619469809` (upstream `parry::utils::COS_5_DEGREES`). Raw 4278623649.
pub const COS_5_DEGREES: Fixed = Fixed { raw: 4278623649 };

/// `cos(10°) ≈ 0.98480775301` (upstream `parry::utils::COS_10_DEGREES`). Raw 4229717092.
pub const COS_10_DEGREES: Fixed = Fixed { raw: 4229717092 };

/// `cos(45°) ≈ 0.70710678118` (upstream `parry::utils::COS_45_DEGREES`). Raw 3037000500, which
/// is exactly `fixed::FRAC_1_SQRT_2`.
pub const COS_45_DEGREES: Fixed = Fixed { raw: 3037000500 };

/// `cos(pi/8) ≈ 0.92387953251` (upstream `parry::utils::COS_FRAC_PI_8`), the "edges are parallel"
/// threshold of capsule-capsule and of 3D polygonal features. Raw 3968032378.
pub const COS_FRAC_PI_8: Fixed = Fixed { raw: 3968032378 };

/// The geometric tolerance of the engine, `2^-23 = 1.1920928955078125e-7` exactly (512 ulp).
///
/// Mirrors `parry::math::DEFAULT_EPSILON` (`f32::EPSILON`); see the module documentation for why
/// the decimal value is kept and why it must never be squared in Q32.32.
pub const DEFAULT_EPSILON: Fixed = Fixed { raw: 512 };

/// [`DEFAULT_EPSILON`] squared, pre-scaled to raw Q64.64: `512 * 512 = 262144`.
///
/// This is the constant to compare a raw sum of squares against (`is_norm2_lt_raw`). The same
/// quantity as a `Fixed` would be 0.
pub const DEFAULT_EPSILON_SQ_RAW: i128 = 262144;

/// The squared-distance threshold of the manifold fast path, `1e-6`, pre-scaled to raw Q64.64
/// (`round(1e-6 * 2^64) = 18446744073710`).
///
/// Mirrors the `dist_sq_threshold = 1.0e-6` argument of upstream's
/// `ContactManifold::try_update_contacts_eps`: points that moved by less than `1e-3` keep their
/// contact. As a `Fixed` this value would be 0 (it is 2^14 times smaller than 1 ulp).
pub const DIST_SQ_THRESHOLD_RAW: i128 = 18446744073710;

/// `100 * DEFAULT_EPSILON = 1.1920928955078125e-5` (51200 ulp), the EPA tolerance
/// (`epa*.rs: DEFAULT_EPSILON * 100.0`).
pub const EPA_EPS_TOL: Fixed = Fixed { raw: 51200 };

/// `sqrt(GJK_EPS_TOL) ≈ 1.0918300e-3` (raw 4689374), upstream's `eps_rel` in `gjk.rs`.
pub const GJK_EPS_REL: Fixed = Fixed { raw: 4689374 };

/// `10 * DEFAULT_EPSILON = 1.1920928955078125e-6` (5120 ulp), upstream's `gjk::eps_tol()`.
pub const GJK_EPS_TOL: Fixed = Fixed { raw: 5120 };

/// [`GJK_EPS_TOL`] squared, pre-scaled to raw Q64.64: `5120 * 5120 = 26214400`.
///
/// GJK compares `|v - p|^2` against `eps_tol` (an *unsquared* tolerance used on a squared
/// quantity, upstream); use this constant when porting a guard that squares it explicitly.
pub const GJK_EPS_TOL_SQ_RAW: i128 = 26214400;

/// `1.0^2` as a raw Q64.64 value (`2^64`), the reference of `is_unit2`.
pub const ONE_SQ_RAW: i128 = 18446744073709551616;

/// `sin(10°) ≈ 0.17364817766` (upstream `parry::utils::SIN_10_DEGREES`). Raw 745813244.
pub const SIN_10_DEGREES: Fixed = Fixed { raw: 745813244 };

/// `sin(45°) ≈ 0.70710678118` (upstream `parry::utils::SIN_45_DEGREES`), equal to
/// [`COS_45_DEGREES`]. Raw 3037000500.
pub const SIN_45_DEGREES: Fixed = Fixed { raw: 3037000500 };

/// `sin(pi/8) ≈ 0.38268343236` (upstream `parry::utils::SIN_FRAC_PI_8`). Raw 1643612827.
pub const SIN_FRAC_PI_8: Fixed = Fixed { raw: 1643612827 };

/// [`UNIT_TOL_ULPS`] pre-scaled to raw Q64.64 (`8 * 2^32 = 34359738368`), for `is_unit2_raw`.
pub const UNIT_TOL_SQ_RAW: i128 = 34359738368;

/// Default tolerance of `is_unit2`, in ulp of the **squared** norm (8 ulp of Q32.32, i.e.
/// `8 * 2^32` raw Q64.64 units). See the module documentation for the derivation.
pub const UNIT_TOL_ULPS: u32 = 8;

#[cfg(test)]
mod tests {
    use fixed::wide::{WideAdd, WideNarrow, wide_mul};
    use fixed::{FRAC_1_SQRT_2, Fixed, FixedTrait, ONE};
    use super::{
        COS_10_DEGREES, COS_1_DEGREES, COS_45_DEGREES, COS_5_DEGREES, COS_FRAC_PI_8,
        DEFAULT_EPSILON, DEFAULT_EPSILON_SQ_RAW, DIST_SQ_THRESHOLD_RAW, EPA_EPS_TOL, GJK_EPS_REL,
        GJK_EPS_TOL, GJK_EPS_TOL_SQ_RAW, ONE_SQ_RAW, SIN_10_DEGREES, SIN_45_DEGREES, SIN_FRAC_PI_8,
        UNIT_TOL_SQ_RAW, UNIT_TOL_ULPS,
    };

    /// Every angular constant lies in `(0, 1)` and the cosines are ordered by angle.
    #[test]
    fn test_angular_constants_ordered() {
        assert!(COS_1_DEGREES < ONE);
        assert!(COS_5_DEGREES < COS_1_DEGREES);
        assert!(COS_10_DEGREES < COS_5_DEGREES);
        assert!(COS_FRAC_PI_8 < COS_10_DEGREES);
        assert!(COS_45_DEGREES < COS_FRAC_PI_8);
        assert!(SIN_FRAC_PI_8 < COS_45_DEGREES);
        assert!(SIN_10_DEGREES < SIN_FRAC_PI_8);
        assert!(SIN_10_DEGREES > FixedTrait::from_raw(0));
    }

    /// `cos(1°)` must stay distinguishable from 1: 654 144 ulp of headroom.
    #[test]
    fn test_cos_1_degree_resolution() {
        assert_eq!(ONE.raw - COS_1_DEGREES.raw, 654144);
    }

    /// `cos(45°)` is `sin(45°)` and `fixed`'s own `1/sqrt(2)`.
    #[test]
    fn test_cos_45_matches_fixed() {
        assert_eq!(COS_45_DEGREES, SIN_45_DEGREES);
        assert_eq!(COS_45_DEGREES, FRAC_1_SQRT_2);
    }

    /// `cos^2 + sin^2 = 1` within 1 ulp of the squared norm, for the two pairs we store.
    #[test]
    fn test_pythagorean_pairs() {
        let pi8 = wide_mul(COS_FRAC_PI_8, COS_FRAC_PI_8)
            .add(wide_mul(SIN_FRAC_PI_8, SIN_FRAC_PI_8))
            .narrow();
        assert!(pi8.abs_diff_eq(ONE, FixedTrait::from_raw(1)));
        let d45 = wide_mul(COS_45_DEGREES, COS_45_DEGREES)
            .add(wide_mul(SIN_45_DEGREES, SIN_45_DEGREES))
            .narrow();
        assert!(d45.abs_diff_eq(ONE, FixedTrait::from_raw(1)));
    }

    /// The epsilon family is the upstream one: 1, 10 and 100 times `2^-23`, and `eps_rel` is the
    /// square root of `10 eps`.
    #[test]
    fn test_epsilon_family() {
        assert_eq!(DEFAULT_EPSILON.raw, 512);
        assert_eq!(GJK_EPS_TOL.raw, 10 * DEFAULT_EPSILON.raw);
        assert_eq!(EPA_EPS_TOL.raw, 100 * DEFAULT_EPSILON.raw);
        assert!(GJK_EPS_REL.abs_diff_eq(GJK_EPS_TOL.sqrt(), FixedTrait::from_raw(2)));
    }

    /// The pre-scaled squared thresholds are the exact raw products, and each of them is 0 once
    /// rescaled to Q32.32 — the hazard this module exists for.
    #[test]
    fn test_squared_thresholds_are_prescaled() {
        let eps: i128 = DEFAULT_EPSILON.raw.into();
        assert_eq!(DEFAULT_EPSILON_SQ_RAW, eps * eps);
        let tol: i128 = GJK_EPS_TOL.raw.into();
        assert_eq!(GJK_EPS_TOL_SQ_RAW, tol * tol);
        assert_eq!(ONE_SQ_RAW, 18446744073709551616);
        // Rescaling any of them to Q32.32 gives zero.
        assert_eq!(DEFAULT_EPSILON * DEFAULT_EPSILON, Fixed { raw: 0 });
        assert_eq!(GJK_EPS_TOL * GJK_EPS_TOL, Fixed { raw: 0 });
        assert!(DIST_SQ_THRESHOLD_RAW > 0);
        assert!(DIST_SQ_THRESHOLD_RAW < ONE_SQ_RAW);
    }

    /// The unit tolerance is the one the module documentation derives.
    #[test]
    fn test_unit_tolerance() {
        assert_eq!(UNIT_TOL_ULPS, 8);
        let ulps: i128 = UNIT_TOL_ULPS.into();
        assert_eq!(UNIT_TOL_SQ_RAW, ulps * 4294967296);
    }
}
