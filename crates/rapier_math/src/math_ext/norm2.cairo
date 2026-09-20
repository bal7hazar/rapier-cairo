//! Comparisons of squared 2D lengths, kept wide.
//!
//! # The hazard
//!
//! Rapier and Parry compare squared quantities all the time: `length_squared() > eps * eps`,
//! `dist_sq < prediction * prediction`, `|length_squared() - 1| < tol` for normalisation checks.
//! In a float that is free. In Q32.32 it is not: a product of two scalars is exact at Q64.64 and
//! **loses its lower 32 bits when it is rescaled** to Q32.32. Anything shorter than `2^-16`
//! therefore has a squared length of exactly 0, and so does every threshold below `2^-16` —
//! `DEFAULT_EPSILON^2 = 2^-46`, the manifold's `1e-6`, `prediction^2` for a millimetre
//! prediction. A faithful port of `length_squared() > eps * eps` becomes `0 > 0` and answers
//! "degenerate" for every vector shorter than `2^-16`, 128 times upstream's threshold.
//! At the other end, `norm2_squared` of a vector longer than `2^15.5` overflows the scalar range
//! and panics, although the comparison it feeds is perfectly well defined.
//!
//! # The remedy
//!
//! Every helper here keeps the raw product wide — `i64 * i64 -> i128` through
//! `core::num::traits::WideMul`, no shift, no range check — and compares it against a threshold
//! pre-scaled the same way (`super::super::consts::DEFAULT_EPSILON_SQ_RAW`, `ONE_SQ_RAW`, ...). The
//! result is exact over the whole scalar range and cheaper than the rescale it replaces, which is
//! what `docs/research/02-parry-analysis.md` §5 prescribes.
//!
//! The raw value manipulated by this module is the Q64.64 integer `x_raw^2 + y_raw^2`, i.e.
//! `(x^2 + y^2) * 2^64`; [`ONE_SQ_RAW`](super::super::consts::ONE_SQ_RAW) is its value for a unit
//! vector.
//!
//! # Candidates
//!
//! Ranked by the `gas_*` probes of this module; the rejected ones live in
//! `#[cfg(test)] mod alternatives`.
//!
//! 1. **wide** (this module): one `i64_wide_mul` libfunc per square, then one `i128` comparison.
//!    Exact everywhere, never rescales. **Winner** (net gas, against the `gas_baseline` of this
//!    module: 4 160 for `is_norm2_lt` against 5 510 for the square root and 8 210 for the
//!    rescale; 4 600 for `is_unit2_raw` against 8 280 for the rescale).
//! 2. `alternatives::*_sqrt`: `fixed::wide::norm2` then compare the lengths. Also exact — the
//!    floored length satisfies `floor(L) < t` iff `L < t` for a representable `t` — but it pays
//!    an integer square root, and it panics for lengths above the scalar range.
//! 3. `alternatives::*_rescaled`: `fixed::wide::norm2_squared` then compare. **Wrong**: it
//!    underflows for short vectors and panics for long ones (`test_rescaled_*` below).
//!
//! # Tolerances
//!
//! See `super::super::consts` for the derivation of `DEFAULT_EPSILON` (512 ulp), of the unit
//! tolerance ([`UNIT_TOL_ULPS`](super::super::consts::UNIT_TOL_ULPS) `= 8` ulp of the squared
//! norm) and of why the zero test needs no tolerance at all.

use core::num::traits::WideMul;
use fixed::Fixed;
use crate::consts::ONE_SQ_RAW;

/// The result of comparing two quantities.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum Cmp {
    /// The left-hand side is the smaller one.
    Less,
    /// Both sides are equal.
    Equal,
    /// The left-hand side is the larger one.
    Greater,
}

/// Returns the exact raw Q64.64 square of `x` (`x_raw^2`, one step, no range check).
///
/// This is how a `Fixed` threshold is brought to the scale of [`norm2_sq_wide`]; a constant
/// threshold should be pre-scaled once in `super::super::consts` instead.
/// #### Panics
/// * Never.
/// #### Deviations
/// * None (`Fixed * Fixed` would floor the result to Q32.32).
#[inline(always)]
pub fn sq_wide(x: Fixed) -> i128 {
    x.raw.wide_mul(x.raw)
}

/// Returns the exact raw Q64.64 squared length of `(x, y)` (`x_raw^2 + y_raw^2`).
///
/// Mirrors `glam::Vec2::length_squared`, kept wide: no rescale, no underflow, and defined over
/// the whole scalar range.
/// #### Panics
/// * `'i128_add Overflow'` for the single input `x = y = fixed::MIN`, whose raw sum of squares is
///   `2^127`, one above `i128::MAX`.
/// #### Deviations
/// * Not a `Fixed`: consume it with the comparisons below, or with `fixed::wide::norm2_squared`
///   when the value itself (and not a comparison) is wanted.
#[inline(always)]
pub fn norm2_sq_wide(x: Fixed, y: Fixed) -> i128 {
    x.raw.wide_mul(x.raw) + y.raw.wide_mul(y.raw)
}

/// Returns `true` when `(x, y)` is exactly the zero vector.
///
/// Equivalent to `norm2_sq_wide(x, y) == 0` (a sum of squares vanishes only when both components
/// do) and to the zero test of `fixed::wide::Norm::is_zero`, hence to the `None` case of
/// [`super::vec2::try_normalize2`]. It needs no tolerance: the wide test never underflows. For a
/// *degeneracy* test use `is_norm2_lt(x, y, DEFAULT_EPSILON)`.
/// #### Panics
/// * Never.
/// #### Deviations
/// * None.
#[inline(always)]
pub fn is_zero2(x: Fixed, y: Fixed) -> bool {
    x.raw == 0 && y.raw == 0
}

/// Returns `true` when `|(x, y)| < t`, comparing the squares wide.
///
/// Mirrors `v.length_squared() < t * t`.
/// #### Panics
/// * `'i128_add Overflow'` for `x = y = fixed::MIN` (see [`norm2_sq_wide`]).
/// #### Deviations
/// * The threshold is used squared, so a negative `t` behaves like `|t|`; upstream thresholds are
///   never negative.
#[inline(always)]
pub fn is_norm2_lt(x: Fixed, y: Fixed, t: Fixed) -> bool {
    norm2_sq_wide(x, y) < sq_wide(t)
}

/// Returns `true` when `|(x, y)| <= t`, comparing the squares wide (see [`is_norm2_lt`]).
/// #### Panics
/// * `'i128_add Overflow'` for `x = y = fixed::MIN`.
/// #### Deviations
/// * A negative `t` behaves like `|t|`.
#[inline(always)]
pub fn is_norm2_le(x: Fixed, y: Fixed, t: Fixed) -> bool {
    norm2_sq_wide(x, y) <= sq_wide(t)
}

/// Returns `true` when `|(x, y)| > t`, comparing the squares wide (see [`is_norm2_lt`]).
/// #### Panics
/// * `'i128_add Overflow'` for `x = y = fixed::MIN`.
/// #### Deviations
/// * A negative `t` behaves like `|t|`.
#[inline(always)]
pub fn is_norm2_gt(x: Fixed, y: Fixed, t: Fixed) -> bool {
    norm2_sq_wide(x, y) > sq_wide(t)
}

/// Returns `true` when `|(x, y)| >= t`, comparing the squares wide (see [`is_norm2_lt`]).
/// #### Panics
/// * `'i128_add Overflow'` for `x = y = fixed::MIN`.
/// #### Deviations
/// * A negative `t` behaves like `|t|`.
#[inline(always)]
pub fn is_norm2_ge(x: Fixed, y: Fixed, t: Fixed) -> bool {
    norm2_sq_wide(x, y) >= sq_wide(t)
}

/// Returns `true` when the squared length of `(x, y)` is below the pre-scaled raw Q64.64
/// threshold `t_sq_raw`.
///
/// The form to use with the `_SQ_RAW` constants of `super::super::consts`, whose `Fixed` square
/// would be 0: `is_norm2_lt_raw(x, y, DEFAULT_EPSILON_SQ_RAW)`.
/// #### Panics
/// * `'i128_add Overflow'` for `x = y = fixed::MIN`.
/// #### Deviations
/// * None.
#[inline(always)]
pub fn is_norm2_lt_raw(x: Fixed, y: Fixed, t_sq_raw: i128) -> bool {
    norm2_sq_wide(x, y) < t_sq_raw
}

/// Returns `true` when the squared length of `(x, y)` is at or above the pre-scaled raw Q64.64
/// threshold `t_sq_raw` (see [`is_norm2_lt_raw`]).
/// #### Panics
/// * `'i128_add Overflow'` for `x = y = fixed::MIN`.
/// #### Deviations
/// * None.
#[inline(always)]
pub fn is_norm2_ge_raw(x: Fixed, y: Fixed, t_sq_raw: i128) -> bool {
    norm2_sq_wide(x, y) >= t_sq_raw
}

/// Returns `true` when `lo <= |(x, y)| <= hi`, with a single wide squared length.
///
/// Mirrors the `(prediction, 0)` style interval tests of the narrow phase.
/// #### Panics
/// * `'i128_add Overflow'` for `x = y = fixed::MIN`.
/// #### Deviations
/// * Both bounds are used squared, so their signs are ignored. An empty interval (`hi < lo`)
///   returns `false`, as a direct comparison would.
#[inline(always)]
pub fn is_norm2_between(x: Fixed, y: Fixed, lo: Fixed, hi: Fixed) -> bool {
    let s = norm2_sq_wide(x, y);
    sq_wide(lo) <= s && s <= sq_wide(hi)
}

/// Returns `true` when `(x, y)` has unit length to within `tol_ulps` ulp of the **squared** norm,
/// i.e. `|x^2 + y^2 - 1| <= tol_ulps * 2^-32`.
///
/// Mirrors `glam::Vec2::is_normalized` (which tests `|length_squared() - 1| <= 2e-4`, a tolerance
/// tied to f32's 24-bit mantissa); `super::super::consts::UNIT_TOL_ULPS` is the Q32.32
/// equivalent, derived in that module. A direct port would compare `norm2_squared` against
/// `1 +- tol`, which is off by up to 1 ulp because the rescale floors, and panics for long
/// vectors.
/// #### Panics
/// * `'i128_add Overflow'` for `x = y = fixed::MIN`.
/// #### Deviations
/// * The tolerance is expressed in ulp instead of as a decimal.
/// * Scaling the tolerance costs more than the test itself (7 450 gas against 4 600, measured):
///   on a hot path call [`is_unit2_raw`] with a pre-scaled constant such as
///   `super::super::consts::UNIT_TOL_SQ_RAW`.
#[inline(always)]
pub fn is_unit2(x: Fixed, y: Fixed, tol_ulps: u32) -> bool {
    let ulps: i128 = tol_ulps.into();
    is_unit2_raw(x, y, ulps * 4294967296)
}

/// Returns `true` when `(x, y)` has unit length to within the pre-scaled raw Q64.64 tolerance
/// `tol_sq_raw`, i.e. `|x_raw^2 + y_raw^2 - 2^64| <= tol_sq_raw`.
///
/// The form to use with `super::super::consts::UNIT_TOL_SQ_RAW` and with any tolerance known at
/// compile time: it saves the scaling multiplication of [`is_unit2`].
/// #### Panics
/// * `'i128_add Overflow'` for `x = y = fixed::MIN`.
/// #### Deviations
/// * A negative tolerance accepts nothing.
#[inline(always)]
pub fn is_unit2_raw(x: Fixed, y: Fixed, tol_sq_raw: i128) -> bool {
    let d = norm2_sq_wide(x, y) - ONE_SQ_RAW;
    -tol_sq_raw <= d && d <= tol_sq_raw
}

/// Compares the lengths of `(ax, ay)` and `(bx, by)` without a square root, wide.
///
/// Mirrors `a.length_squared().partial_cmp(&b.length_squared())`. The comparison of the squares
/// is the comparison of the lengths: both are non-negative.
/// #### Panics
/// * `'i128_add Overflow'` if either vector is `(fixed::MIN, fixed::MIN)`.
/// #### Deviations
/// * None.
#[inline(always)]
pub fn norm2_cmp(ax: Fixed, ay: Fixed, bx: Fixed, by: Fixed) -> Cmp {
    let a = norm2_sq_wide(ax, ay);
    let b = norm2_sq_wide(bx, by);
    if a < b {
        Cmp::Less
    } else if b < a {
        Cmp::Greater
    } else {
        Cmp::Equal
    }
}

/// Rejected candidates, kept for the `gas_*` ranking and for the tests that show why they lose.
#[cfg(test)]
mod alternatives {
    use fixed::wide::{norm2, norm2_squared};
    use fixed::{Fixed, ONE};
    use super::Cmp;

    /// `is_norm2_lt` through `fixed::wide::norm2_squared`: one rescale, then one comparison.
    ///
    /// Underflows to `0 < 0` whenever both sides are shorter than `2^-16`, and panics
    /// (`'Fixed: overflow'`) as soon as a squared length leaves the scalar range.
    pub fn is_norm2_lt_rescaled(x: Fixed, y: Fixed, t: Fixed) -> bool {
        norm2_squared(x, y) < t * t
    }

    /// `is_norm2_lt` through `fixed::wide::norm2`: one integer square root, then one comparison.
    ///
    /// Exact (the floored length satisfies `floor(L) < t` iff `L < t`), but it pays the square
    /// root and panics when the length itself leaves the scalar range.
    pub fn is_norm2_lt_sqrt(x: Fixed, y: Fixed, t: Fixed) -> bool {
        norm2(x, y) < t
    }

    /// `is_unit2` through `fixed::wide::norm2_squared`, with the tolerance rebuilt as a `Fixed`.
    pub fn is_unit2_rescaled(x: Fixed, y: Fixed, tol_ulps: u32) -> bool {
        let tol = Fixed { raw: tol_ulps.into() };
        let s = norm2_squared(x, y);
        ONE - tol <= s && s <= ONE + tol
    }

    /// `is_zero2` through the wide sum of squares instead of two raw comparisons.
    pub fn is_zero2_wide(x: Fixed, y: Fixed) -> bool {
        super::norm2_sq_wide(x, y) == 0
    }

    /// `norm2_cmp` through two `fixed::wide::norm2` square roots.
    ///
    /// Not equivalent: two different lengths that floor to the same ulp compare `Equal`.
    pub fn norm2_cmp_sqrt(ax: Fixed, ay: Fixed, bx: Fixed, by: Fixed) -> Cmp {
        let a = norm2(ax, ay);
        let b = norm2(bx, by);
        if a < b {
            Cmp::Less
        } else if b < a {
            Cmp::Greater
        } else {
            Cmp::Equal
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::wide::normalize2;
    use fixed::{Fixed, FixedTrait, MAX, MIN, ONE, ZERO};
    use rapier_testing::opaque;
    use crate::consts::{
        DEFAULT_EPSILON, DEFAULT_EPSILON_SQ_RAW, ONE_SQ_RAW, UNIT_TOL_SQ_RAW, UNIT_TOL_ULPS,
    };
    use super::alternatives::{
        is_norm2_lt_rescaled, is_norm2_lt_sqrt, is_unit2_rescaled, is_zero2_wide, norm2_cmp_sqrt,
    };
    use super::{
        Cmp, is_norm2_between, is_norm2_ge, is_norm2_ge_raw, is_norm2_gt, is_norm2_le, is_norm2_lt,
        is_norm2_lt_raw, is_unit2, is_unit2_raw, is_zero2, norm2_cmp, norm2_sq_wide, sq_wide,
    };

    /// `2^-30` (4 ulp): below `DEFAULT_EPSILON`, and far below the `2^-16` resolution of a
    /// rescaled squared length.
    const TINY: Fixed = Fixed { raw: 4 };
    /// `2^-18`: above `DEFAULT_EPSILON`, still below that `2^-16` resolution.
    const SMALL: Fixed = Fixed { raw: 16384 };
    /// `65536.0`, whose squared length leaves the scalar range.
    const HUGE: Fixed = Fixed { raw: 281474976710656 };

    // ---------------------------------------------------------------- wide value

    #[test]
    fn test_sq_wide_is_exact() {
        assert_eq!(sq_wide(ONE), ONE_SQ_RAW);
        assert_eq!(sq_wide(DEFAULT_EPSILON), DEFAULT_EPSILON_SQ_RAW);
        assert_eq!(sq_wide(-DEFAULT_EPSILON), DEFAULT_EPSILON_SQ_RAW);
        assert_eq!(sq_wide(FixedTrait::from_raw(1)), 1);
        assert_eq!(sq_wide(ZERO), 0);
    }

    #[test]
    fn test_norm2_sq_wide_is_exact() {
        assert_eq!(norm2_sq_wide(ONE, ZERO), ONE_SQ_RAW);
        assert_eq!(norm2_sq_wide(ONE, ONE), 2 * ONE_SQ_RAW);
        // (3, 4) has length 5 exactly.
        let three: Fixed = FixedTrait::from_int(3);
        let four: Fixed = FixedTrait::from_int(4);
        assert_eq!(norm2_sq_wide(three, four), 25 * ONE_SQ_RAW);
        // Signs cancel, and the smallest non-zero vector is representable.
        assert_eq!(norm2_sq_wide(-three, -four), 25 * ONE_SQ_RAW);
        assert_eq!(norm2_sq_wide(FixedTrait::from_raw(1), ZERO), 1);
    }

    /// The extremes of the scalar range: `MAX` and `MIN` alone are fine, `MIN` twice is the one
    /// documented panic.
    #[test]
    fn test_norm2_sq_wide_extremes() {
        let max_sq: i128 = 85070591730234615847396907784232501249; // (2^63 - 1)^2
        assert_eq!(norm2_sq_wide(MAX, ZERO), max_sq);
        assert_eq!(norm2_sq_wide(MIN, ZERO), 85070591730234615865843651857942052864); // 2^126
        assert_eq!(norm2_sq_wide(MAX, MAX), 2 * max_sq);
        assert_eq!(norm2_sq_wide(MIN, MAX), max_sq + 85070591730234615865843651857942052864);
    }

    #[test]
    #[should_panic(expected: 'i128_add Overflow')]
    fn test_norm2_sq_wide_min_min_panics() {
        norm2_sq_wide(opaque(MIN), opaque(MIN));
    }

    // ---------------------------------------------------------------- comparisons

    #[test]
    fn test_comparisons_on_a_known_length() {
        let three: Fixed = FixedTrait::from_int(3);
        let four: Fixed = FixedTrait::from_int(4);
        let five: Fixed = FixedTrait::from_int(5);
        assert!(is_norm2_le(three, four, five));
        assert!(is_norm2_ge(three, four, five));
        assert!(!is_norm2_lt(three, four, five));
        assert!(!is_norm2_gt(three, four, five));
        assert!(is_norm2_lt(three, four, five + FixedTrait::from_raw(1)));
        assert!(is_norm2_gt(three, four, five - FixedTrait::from_raw(1)));
        assert!(is_norm2_between(three, four, five, five));
        assert!(!is_norm2_between(three, four, five + FixedTrait::from_raw(1), five * five));
        // An empty interval is empty.
        assert!(!is_norm2_between(three, four, five, three));
    }

    /// One ulp on either side of the threshold is resolved exactly, at every scale.
    #[test]
    fn test_comparisons_are_ulp_exact() {
        let one_ulp: Fixed = FixedTrait::from_raw(1);
        assert!(is_norm2_lt(one_ulp, ZERO, FixedTrait::from_raw(2)));
        assert!(!is_norm2_lt(one_ulp, ZERO, one_ulp));
        assert!(is_norm2_gt(FixedTrait::from_raw(2), ZERO, one_ulp));
        // Against the engine tolerance, 1 ulp below and 1 ulp above.
        assert!(is_norm2_lt(FixedTrait::from_raw(511), ZERO, DEFAULT_EPSILON));
        assert!(!is_norm2_lt(FixedTrait::from_raw(512), ZERO, DEFAULT_EPSILON));
        assert!(is_norm2_gt(FixedTrait::from_raw(513), ZERO, DEFAULT_EPSILON));
    }

    /// The pre-scaled form and the `Fixed` form agree wherever both are usable.
    #[test]
    fn test_raw_thresholds_match_fixed_thresholds() {
        assert_eq!(
            is_norm2_lt_raw(SMALL, ZERO, DEFAULT_EPSILON_SQ_RAW),
            is_norm2_lt(SMALL, ZERO, DEFAULT_EPSILON),
        );
        assert_eq!(
            is_norm2_ge_raw(TINY, ZERO, DEFAULT_EPSILON_SQ_RAW),
            is_norm2_ge(TINY, ZERO, DEFAULT_EPSILON),
        );
        assert!(is_norm2_ge_raw(SMALL, ZERO, DEFAULT_EPSILON_SQ_RAW));
        assert!(is_norm2_lt_raw(TINY, ZERO, DEFAULT_EPSILON_SQ_RAW));
    }

    /// A negative threshold is used squared, as documented.
    #[test]
    fn test_negative_threshold_is_used_squared() {
        let three: Fixed = FixedTrait::from_int(3);
        let four: Fixed = FixedTrait::from_int(4);
        assert!(is_norm2_lt(three, four, FixedTrait::from_int(-6)));
        assert!(!is_norm2_lt(three, four, FixedTrait::from_int(-4)));
    }

    // ------------------------------------------------- why the rescaled candidate loses

    /// Short vectors: the rescaled candidate answers the opposite of the truth, because both
    /// sides of its comparison underflow to 0.
    #[test]
    fn test_rescaled_underflows_for_short_vectors() {
        // |(2^-30, 0)| = 2^-30 is genuinely below the threshold 2^-18.
        assert!(is_norm2_lt(TINY, ZERO, SMALL));
        assert!(!is_norm2_lt_rescaled(TINY, ZERO, SMALL));
        // The sqrt candidate gets it right, like the wide one.
        assert!(is_norm2_lt_sqrt(TINY, ZERO, SMALL));
        // Worse: against DEFAULT_EPSILON the rescaled test declares every vector shorter than
        // 2^-16 non-degenerate, 128 times upstream's threshold.
        assert!(is_norm2_lt(TINY, ZERO, DEFAULT_EPSILON));
        assert!(!is_norm2_lt_rescaled(TINY, ZERO, DEFAULT_EPSILON));
    }

    /// Long vectors: the rescaled candidate panics where the wide one answers.
    #[test]
    fn test_wide_survives_long_vectors() {
        assert!(is_norm2_gt(HUGE, HUGE, FixedTrait::from_int(1000)));
        assert!(!is_norm2_lt(HUGE, HUGE, HUGE));
        // Even at the very edge of the scalar range.
        assert!(is_norm2_gt(MAX, MAX, HUGE));
    }

    #[test]
    #[should_panic(expected: 'Fixed: overflow')]
    fn test_rescaled_panics_on_long_vectors() {
        is_norm2_lt_rescaled(opaque(HUGE), opaque(HUGE), opaque(HUGE));
    }

    #[test]
    #[should_panic(expected: 'Fixed: overflow')]
    fn test_sqrt_panics_on_the_longest_vectors() {
        // |(MAX, MAX)| = 2^31.5 does not fit the scalar range; the wide comparison above does.
        is_norm2_lt_sqrt(opaque(MAX), opaque(MAX), opaque(HUGE));
    }

    // ---------------------------------------------------------------- zero and unit

    #[test]
    fn test_is_zero2() {
        assert!(is_zero2(ZERO, ZERO));
        assert!(!is_zero2(FixedTrait::from_raw(1), ZERO));
        assert!(!is_zero2(ZERO, FixedTrait::from_raw(-1)));
        assert!(!is_zero2(MIN, ZERO));
        // Same answer as the wide candidate, everywhere.
        assert_eq!(is_zero2(ZERO, ZERO), is_zero2_wide(ZERO, ZERO));
        assert_eq!(is_zero2(MIN, ZERO), is_zero2_wide(MIN, ZERO));
        assert_eq!(
            is_zero2(FixedTrait::from_raw(1), FixedTrait::from_raw(-1)),
            is_zero2_wide(FixedTrait::from_raw(1), FixedTrait::from_raw(-1)),
        );
    }

    #[test]
    fn test_is_unit2_exact_and_tolerant() {
        assert!(is_unit2(ONE, ZERO, UNIT_TOL_ULPS));
        assert!(is_unit2(ZERO, -ONE, UNIT_TOL_ULPS));
        assert!(is_unit2(ONE, ZERO, 0));
        // 3 ulp on the component moves the squared norm by 6 ulp: inside the 8 ulp band.
        let three_ulp_off = Fixed { raw: ONE.raw + 3 };
        assert!(is_unit2(three_ulp_off, ZERO, UNIT_TOL_ULPS));
        // 5 ulp on the component moves it by 10 ulp: outside.
        let five_ulp_off = Fixed { raw: ONE.raw + 5 };
        assert!(!is_unit2(five_ulp_off, ZERO, UNIT_TOL_ULPS));
        assert!(is_unit2(five_ulp_off, ZERO, 16));
        assert!(!is_unit2(ZERO, ZERO, UNIT_TOL_ULPS));
        // The pre-scaled form is the same test.
        assert_eq!(is_unit2(ONE, ZERO, UNIT_TOL_ULPS), is_unit2_raw(ONE, ZERO, UNIT_TOL_SQ_RAW));
        assert_eq!(
            is_unit2(five_ulp_off, ZERO, UNIT_TOL_ULPS),
            is_unit2_raw(five_ulp_off, ZERO, UNIT_TOL_SQ_RAW),
        );
        assert_eq!(is_unit2(ZERO, ZERO, UNIT_TOL_ULPS), is_unit2_raw(ZERO, ZERO, UNIT_TOL_SQ_RAW));
    }

    /// The tolerance of `consts` covers what `normalize2` actually produces for inputs of length
    /// at least 1.
    #[test]
    fn test_normalize2_is_unit_within_the_documented_tolerance() {
        let (nx, ny) = normalize2(FixedTrait::from_int(3), FixedTrait::from_int(4));
        assert!(is_unit2(nx, ny, UNIT_TOL_ULPS));
        let (nx, ny) = normalize2(FixedTrait::from_int(-7), FixedTrait::from_int(11));
        assert!(is_unit2(nx, ny, UNIT_TOL_ULPS));
        let (nx, ny) = normalize2(FixedTrait::from_int(1), FixedTrait::from_int(1));
        assert!(is_unit2(nx, ny, UNIT_TOL_ULPS));
        let (nx, ny) = normalize2(HUGE, FixedTrait::from_int(1));
        assert!(is_unit2(nx, ny, UNIT_TOL_ULPS));
    }

    /// ... and stops covering it exactly where `consts` says it does: normalising a vector of a
    /// few ulp divides by a length that has been floored to almost nothing.
    #[test]
    fn test_normalize2_of_a_few_ulps_is_not_unit() {
        let one_ulp: Fixed = FixedTrait::from_raw(1);
        let (nx, ny) = normalize2(one_ulp, one_ulp);
        assert_eq!((nx, ny), (ONE, ONE)); // length sqrt(2), not 1
        assert!(!is_unit2(nx, ny, UNIT_TOL_ULPS));
    }

    /// The rescaled unit test is off by up to 1 ulp of the squared norm, because its rescale
    /// floors the fractional part away: `(1 + 5 ulp)^2 = 1 + 10 ulp + 25 ulp^2` is outside a
    /// 10 ulp band, but the rescale drops the `25 ulp^2` and accepts it.
    #[test]
    fn test_rescaled_unit_test_is_off_by_the_floor() {
        let off = Fixed { raw: ONE.raw + 5 };
        assert!(!is_unit2(off, ZERO, 10));
        assert!(is_unit2_rescaled(off, ZERO, 10));
    }

    // ---------------------------------------------------------------- ordering

    #[test]
    fn test_norm2_cmp() {
        let three: Fixed = FixedTrait::from_int(3);
        let four: Fixed = FixedTrait::from_int(4);
        assert_eq!(norm2_cmp(three, four, four, three), Cmp::Equal);
        assert_eq!(norm2_cmp(three, ZERO, four, ZERO), Cmp::Less);
        assert_eq!(norm2_cmp(four, ZERO, three, ZERO), Cmp::Greater);
        assert_eq!(norm2_cmp(ZERO, ZERO, ZERO, ZERO), Cmp::Equal);
        // Lengths a single ulp apart are still ordered, at any scale.
        assert_eq!(norm2_cmp(FixedTrait::from_raw(1), ZERO, ZERO, ZERO), Cmp::Greater);
        assert_eq!(norm2_cmp(MAX, ZERO, MAX, FixedTrait::from_raw(1)), Cmp::Less);
    }

    /// The square-root candidate cannot separate lengths that floor to the same ulp.
    #[test]
    fn test_cmp_sqrt_collapses_neighbouring_lengths() {
        let one_ulp: Fixed = FixedTrait::from_raw(1);
        assert_eq!(norm2_cmp(MAX, ZERO, MAX, one_ulp), Cmp::Less);
        assert_eq!(norm2_cmp_sqrt(MAX, ZERO, MAX, one_ulp), Cmp::Equal);
    }

    // ---------------------------------------------------------------- fuzz

    const LCG_MUL: u128 = 6364136223846793005;
    const LCG_INC: u128 = 1442695040888963407;
    const NZ_TWO_POW_64: NonZero<u128> = 0x10000000000000000;
    const NZ_TWO_POW_32: NonZero<u128> = 0x100000000;
    const NZ_512: NonZero<u64> = 512;

    /// The LCG of `rapier_core`: returns the high 32 bits of the next state.
    fn next(ref state: u64) -> u32 {
        let (_, low) = DivRem::div_rem(state.into() * LCG_MUL + LCG_INC, NZ_TWO_POW_64);
        state = low.try_into().unwrap();
        let (high, _) = DivRem::div_rem(low, NZ_TWO_POW_32);
        high.try_into().unwrap()
    }

    /// Draws a raw value in `[-2^40, 2^40)`, small enough that every candidate stays inside its
    /// domain (no squared length above the scalar range, no length above it either).
    fn draw(ref state: u64) -> Fixed {
        let high: u64 = next(ref state).into();
        let low: u64 = next(ref state).into();
        let (_, low) = DivRem::div_rem(low, NZ_512);
        let raw: i64 = (high * 512 + low).try_into().unwrap();
        Fixed { raw: raw - 1099511627776 } // 2^40
    }

    /// On the domain where both are defined, the wide comparison and the square-root comparison
    /// agree exactly — both are exact, so this is equivalence, not tolerance.
    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_wide_matches_sqrt(seed: u64) {
        let mut state = seed | 1;
        let mut i: u32 = 0;
        while i != 16 {
            let x = draw(ref state);
            let y = draw(ref state);
            let t = draw(ref state).abs();
            assert_eq!(is_norm2_lt(x, y, t), is_norm2_lt_sqrt(x, y, t));
            assert_eq!(is_norm2_ge(x, y, t), !is_norm2_lt_sqrt(x, y, t));
            i += 1;
        }
    }

    /// The four comparison helpers are mutually consistent and match a direct comparison of the
    /// squared values.
    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_comparisons_are_consistent(seed: u64) {
        let mut state = seed | 1;
        let mut i: u32 = 0;
        while i != 16 {
            let x = draw(ref state);
            let y = draw(ref state);
            let t = draw(ref state).abs();
            let s = norm2_sq_wide(x, y);
            let ts = sq_wide(t);
            assert_eq!(is_norm2_lt(x, y, t), s < ts);
            assert_eq!(is_norm2_le(x, y, t), s <= ts);
            assert_eq!(is_norm2_gt(x, y, t), s > ts);
            assert_eq!(is_norm2_ge(x, y, t), s >= ts);
            assert_eq!(is_norm2_lt(x, y, t), !is_norm2_ge(x, y, t));
            assert_eq!(is_norm2_le(x, y, t), !is_norm2_gt(x, y, t));
            assert_eq!(is_norm2_between(x, y, ZERO, t), is_norm2_le(x, y, t));
            assert_eq!(is_norm2_lt_raw(x, y, ts), is_norm2_lt(x, y, t));
            assert_eq!(is_norm2_ge_raw(x, y, ts), is_norm2_ge(x, y, t));
            i += 1;
        }
    }

    /// `norm2_cmp` orders the same way the comparison helpers do.
    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_cmp_matches_comparisons(seed: u64) {
        let mut state = seed | 1;
        let mut i: u32 = 0;
        while i != 16 {
            let ax = draw(ref state);
            let ay = draw(ref state);
            let bx = draw(ref state);
            let by = draw(ref state);
            let expected = if norm2_sq_wide(ax, ay) < norm2_sq_wide(bx, by) {
                Cmp::Less
            } else if norm2_sq_wide(bx, by) < norm2_sq_wide(ax, ay) {
                Cmp::Greater
            } else {
                Cmp::Equal
            };
            assert_eq!(norm2_cmp(ax, ay, bx, by), expected);
            i += 1;
        }
    }

    // ---------------------------------------------------------------- gas

    /// Empty probe: the harness overhead to subtract from the entries below.
    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_sq_wide() {
        assert!(sq_wide(opaque(ONE)) > 0);
    }

    #[test]
    fn gas_norm2_sq_wide() {
        assert!(norm2_sq_wide(opaque(ONE), opaque(ONE)) > 0);
    }

    #[test]
    fn gas_is_zero2() {
        assert!(!is_zero2(opaque(ONE), opaque(ONE)));
    }

    #[test]
    fn gas_is_zero2_wide() {
        assert!(!is_zero2_wide(opaque(ONE), opaque(ONE)));
    }

    #[test]
    fn gas_is_norm2_lt() {
        assert!(!is_norm2_lt(opaque(ONE), opaque(ONE), opaque(ONE)));
    }

    #[test]
    fn gas_is_norm2_lt_rescaled() {
        assert!(!is_norm2_lt_rescaled(opaque(ONE), opaque(ONE), opaque(ONE)));
    }

    #[test]
    fn gas_is_norm2_lt_sqrt() {
        assert!(!is_norm2_lt_sqrt(opaque(ONE), opaque(ONE), opaque(ONE)));
    }

    #[test]
    fn gas_is_norm2_le() {
        assert!(!is_norm2_le(opaque(ONE), opaque(ONE), opaque(ONE)));
    }

    #[test]
    fn gas_is_norm2_gt() {
        assert!(is_norm2_gt(opaque(ONE), opaque(ONE), opaque(ONE)));
    }

    #[test]
    fn gas_is_norm2_ge() {
        assert!(is_norm2_ge(opaque(ONE), opaque(ONE), opaque(ONE)));
    }

    #[test]
    fn gas_is_norm2_lt_raw() {
        assert!(!is_norm2_lt_raw(opaque(ONE), opaque(ONE), opaque(DEFAULT_EPSILON_SQ_RAW)));
    }

    #[test]
    fn gas_is_norm2_ge_raw() {
        assert!(is_norm2_ge_raw(opaque(ONE), opaque(ONE), opaque(DEFAULT_EPSILON_SQ_RAW)));
    }

    #[test]
    fn gas_is_norm2_between() {
        assert!(is_norm2_between(opaque(ONE), opaque(ZERO), opaque(ZERO), opaque(ONE)));
    }

    #[test]
    fn gas_is_unit2() {
        assert!(is_unit2(opaque(ONE), opaque(ZERO), opaque(UNIT_TOL_ULPS)));
    }

    #[test]
    fn gas_is_unit2_raw() {
        assert!(is_unit2_raw(opaque(ONE), opaque(ZERO), opaque(UNIT_TOL_SQ_RAW)));
    }

    #[test]
    fn gas_is_unit2_rescaled() {
        assert!(is_unit2_rescaled(opaque(ONE), opaque(ZERO), opaque(UNIT_TOL_ULPS)));
    }

    #[test]
    fn gas_norm2_cmp() {
        assert_eq!(norm2_cmp(opaque(ONE), opaque(ZERO), opaque(ZERO), opaque(ONE)), Cmp::Equal);
    }

    #[test]
    fn gas_norm2_cmp_sqrt() {
        assert_eq!(
            norm2_cmp_sqrt(opaque(ONE), opaque(ZERO), opaque(ZERO), opaque(ONE)), Cmp::Equal,
        );
    }
}
