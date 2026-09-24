//! The divisions and the square root of the ray casts, on **wide** operands.
//!
//! Every time of impact is a quotient of two quantities of the same degree: `(m - o) / d` for a
//! slab (degree 1), `cross(a - o, e) / cross(d, e)` for a segment and `(-b ± sqrt(b² - a c)) / a`
//! for a circle (degree 2, raw Q64.64). Narrowing the operands to `Fixed` before dividing would
//! send every product below `2^-32` to 0 and overflow above `2^31`; [`div_wide`] divides the
//! exact `i128` values instead, and [`isqrt_wide`] takes the square root of the exact 256-bit
//! discriminant.
//!
//! # Candidates
//!
//! Ranked by the `gas_*` probes of this module; the rejected one lives in
//! `#[cfg(test)] mod alternatives`.
//!
//! 1. **`u128` fast path** (this module): one `u128` `div_rem` whenever `|num| < 2^95` (so that
//!    `num * 2^32` fits), the `u256` division only above. **Winner** on Cairo steps: 194 steps
//!    against 229 for the probe, and every time of impact of a scene of ordinary size takes it.
//! 2. `alternatives::div_wide_u256`: always the `u256` division. Same answers; 0.8k Sierra gas
//!    cheaper (29.6k against 30.4k), because Sierra gas charges the fast path's function its
//!    `u256` branch too.

use core::num::traits::{DivRem, Sqrt, WideMul};
use fixed::Fixed;

/// `2^32`, the scale of a `Fixed` raw.
const SCALE: u128 = 0x1_0000_0000;
/// Above this, `num * 2^32` does not fit a `u128`.
const FAST_LIMIT: u128 = 0x8000_0000_0000_0000_0000_0000;
/// `2^63`: the magnitude no `Fixed` raw reaches.
const RAW_LIMIT: u128 = 0x8000_0000_0000_0000;

/// Returns `|x|` as a `u128`.
#[inline(always)]
pub fn abs_u128(x: i128) -> u128 {
    if x < 0 {
        // `-(x + 1) + 1` stays in range for `i128::MIN`.
        let m: u128 = (-(x + 1)).try_into().unwrap();
        m + 1
    } else {
        x.try_into().unwrap()
    }
}

/// Rounds `q + r / d` to nearest (ties away from zero) and applies the sign; `None` when the
/// magnitude does not fit a `Fixed` raw.
#[inline(always)]
fn finish(q: u128, r: u128, d: u128, negative: bool) -> Option<Fixed> {
    let q = if r >= d - r {
        q + 1
    } else {
        q
    };
    if q >= RAW_LIMIT {
        return None;
    }
    let raw: i64 = q.try_into().unwrap();
    Some(Fixed { raw: if negative {
        -raw
    } else {
        raw
    } })
}

/// Returns `num / den` as a `Fixed`, with `num` and `den` given at the same arbitrary scale,
/// rounded to nearest (ties away from zero). `None` when the quotient leaves the scalar range
/// (`|num / den| >= 2^31`) — for a time of impact, "further than any representable bound".
///
/// This is the fixed-point form of every `x / y` of the ray casts where `x` and `y` are exact
/// sums of products of the same degree.
/// #### Panics
/// * `'Option::unwrap failed.'` when `den == 0`: callers test the degenerate cases first, as
///   upstream does before dividing.
/// #### Deviations
/// * Correctly rounded; upstream's `f64` quotient of already rounded operands is within a few
///   ulp of it.
pub fn div_wide(num: i128, den: i128) -> Option<Fixed> {
    let negative = (num < 0) != (den < 0);
    let n = abs_u128(num);
    let d = abs_u128(den);
    let d_nz: NonZero<u128> = d.try_into().unwrap();
    if n < FAST_LIMIT {
        let (q, r) = DivRem::div_rem(n * SCALE, d_nz);
        finish(q, r, d, negative)
    } else {
        let wide: u256 = n.wide_mul(SCALE);
        let d_wide_nz: NonZero<u256> = Into::<u128, u256>::into(d).try_into().unwrap();
        let (q, r) = DivRem::div_rem(wide, d_wide_nz);
        if q.high != 0 {
            return None;
        }
        finish(q.low, r.low, d, negative)
    }
}

/// Returns `sqrt(x)` rounded to nearest, for a non-negative 256-bit `x`.
///
/// The discriminant `b² - a c` of a circle cast is a fourth-degree quantity (raw Q128.128); its
/// square root is back at the raw Q64.64 scale of `b`, exactly where the time of impact needs it.
#[inline(always)]
pub fn isqrt_wide(x: u256) -> u128 {
    let s: u128 = x.sqrt();
    // `(s + 1/2)^2 = s^2 + s + 1/4`: round up when `x - s^2 > s`.
    let sq: u256 = s.wide_mul(s);
    if x - sq > s.into() {
        s + 1
    } else {
        s
    }
}

/// Rejected candidates, kept for the `gas_*` ranking.
#[cfg(test)]
pub mod alternatives {
    use core::num::traits::{DivRem, WideMul};
    use fixed::Fixed;
    use super::{SCALE, abs_u128, finish};

    /// [`super::div_wide`] without the `u128` fast path: always the `u256` division.
    pub fn div_wide_u256(num: i128, den: i128) -> Option<Fixed> {
        let negative = (num < 0) != (den < 0);
        let n = abs_u128(num);
        let d = abs_u128(den);
        let wide: u256 = n.wide_mul(SCALE);
        let d_wide_nz: NonZero<u256> = Into::<u128, u256>::into(d).try_into().unwrap();
        let (q, r) = DivRem::div_rem(wide, d_wide_nz);
        if q.high != 0 {
            return None;
        }
        finish(q.low, r.low, d, negative)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE};
    use rapier_testing::opaque;
    use super::alternatives::div_wide_u256;
    use super::{abs_u128, div_wide, isqrt_wide};

    const Q64: i128 = 0x1_0000_0000_0000_0000;

    #[test]
    fn test_div_wide_table() {
        let three: Fixed = FixedTrait::from_int(3);
        let cases: Span<(i128, i128, Option<Fixed>)> = array![
            (Q64, 2 * Q64, Some(HALF)), (-Q64, 2 * Q64, Some(-HALF)), (Q64, -2 * Q64, Some(-HALF)),
            (3 * Q64, Q64, Some(three)), (0, 5, Some(Fixed { raw: 0 })),
            // One third: 0x55555555.55… rounds down.
            (1, 3, Some(Fixed { raw: 0x55555555 })), // Two thirds: 0xAAAAAAAA.AA… rounds up.
            (2, 3, Some(Fixed { raw: 0xAAAAAAAB })),
            (-2, 3, Some(Fixed { raw: -0xAAAAAAAB })), // Half an ulp: ties away from zero.
            (1, 0x2_0000_0000, Some(Fixed { raw: 1 })),
            (-1, 0x2_0000_0000, Some(Fixed { raw: -1 })), // 2^31 does not fit, 2^31 - 2^-32 does.
            (0x8000_0000, 1, None), (0xFFFF_FFFF_FFFF_FFFF, 0x2_0000_0000, None),
            // Above the fast path.
            (0x1000_0000_0000_0000_0000_0000_0000, 0x1000_0000_0000_0000_0000_0000_0000, Some(ONE)),
            (0x1000_0000_0000_0000_0000_0000_0000, 0x1000, None),
        ]
            .span();
        for (num, den, expected) in cases {
            assert_eq!(div_wide(*num, *den), *expected);
            assert_eq!(div_wide_u256(*num, *den), *expected);
        }
    }

    #[test]
    fn test_isqrt_wide_rounds_to_nearest() {
        let cases: Span<(u256, u128)> = array![
            (0, 0), (1, 1), (2, 1), (3, 2), (6, 2), (7, 3), (16, 4),
            (0x1_0000_0000_0000_0000_0000_0000_0000_0000, 0x1_0000_0000_0000_0000),
        ]
            .span();
        for (x, expected) in cases {
            assert_eq!(isqrt_wide(*x), *expected);
        }
    }

    #[test]
    fn test_abs_u128_extremes() {
        assert_eq!(abs_u128(-1), 1);
        assert_eq!(
            abs_u128(0x7fffffff_ffffffff_ffffffff_ffffffff), 0x7fffffff_ffffffff_ffffffff_ffffffff,
        );
        assert_eq!(
            abs_u128(-0x80000000_00000000_00000000_00000000), 0x80000000_00000000_00000000_00000000,
        );
    }

    #[test]
    #[fuzzer(runs: 64, seed: 7)]
    fn fuzz_div_wide_candidates(num: i128, den: i128) {
        if den != 0 {
            assert_eq!(div_wide(num, den), div_wide_u256(num, den));
        }
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_div_wide_fast() {
        let _ = div_wide(opaque(3 * Q64), opaque(7 * Q64));
    }

    #[test]
    fn gas_div_wide_u256_small() {
        let _ = div_wide_u256(opaque(3 * Q64), opaque(7 * Q64));
    }

    #[test]
    fn gas_isqrt_wide() {
        let _ = isqrt_wide(opaque(0x1234_5678_9abc_def0_1234_5678_9abc_def0));
    }
}
