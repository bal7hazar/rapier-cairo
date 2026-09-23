//! The one division of the point and closest-point queries: an exact ratio of two **wide**
//! quantities, clamped to `[0, 1]`.
//!
//! Every parameter Parry solves for in this package is a clamped ratio of two quantities of the
//! same degree — `u = (ab . ap) / |ab|^2` for a segment, `s = (b f - c e) / (a e - b b)` and
//! `t = (b s + f) / e` for two segments. Both operands are exact `i128` values at the same scale
//! (raw Q64.64 sums of products, or raw Q32.32 scalars), and the scale cancels in the quotient.
//!
//! Narrowing them to `Fixed` first and dividing would be wrong twice over: a squared length below
//! `2^-32` rescales to 0 (division by zero where upstream has a perfectly well-defined ratio),
//! and a squared length above `2^31` overflows. [`clamped_ratio`] therefore divides the `i128`
//! values directly. The quotient is wanted with 32 fractional bits and the operands are at most
//! 127 bits, so `num * 2^32` would not fit an `i128` — but it fits a `u128`, which is what the
//! division actually runs on once the clamp has settled the sign. One `div_rem` is therefore
//! enough, plus one comparison to round the last bit to nearest (upstream's `f64` division and
//! `Fixed / Fixed` round to nearest too).
//!
//! # Candidates
//!
//! Ranked by the `gas_*` probes of this module; the rejected ones live in
//! `#[cfg(test)] mod alternatives`.
//!
//! 1. **wide** (this module): one `u128` `div_rem`, one rounding comparison. Exact for every
//!    operand pair. **Winner**.
//! 2. `alternatives::clamped_ratio_two_step`: the same value in two 16-bit steps, the shape a
//!    signed 127-bit intermediate would force. Same answers, one `div_rem` more.
//! 3. `alternatives::clamped_ratio_narrow`: narrow both operands to `Fixed`, then `Fixed / Fixed`
//!    and `clamp`. Cheaper than either, but it panics (`'Fixed: division by zero'`) for the short
//!    segments upstream handles, loses low bits before the division and overflows for long ones.

use core::num::traits::DivRem;
use fixed::{Fixed, ONE, ZERO};

/// `2^32`, the scale of a `Fixed` raw.
const SCALE: u128 = 0x1_0000_0000;
const SCALE_NZ: NonZero<u128> = 0x1_0000_0000;
/// `2^96`: above this, `num * 2^32` would not fit a `u128`.
const WIDE_LIMIT: u128 = 0x1_0000_0000_0000_0000_0000_0000;

pub mod errors {
    pub const NON_POSITIVE_DEN: felt252 = 'Ratio: denominator <= 0';
}

/// Computes `clamp(num / den, 0, 1)` as a `Fixed`, with `num` and `den` given at the same
/// arbitrary scale.
///
/// This is the fixed-point form of upstream's `(x / y).clamp(0.0, 1.0)` wherever `x` and `y` are
/// dot products or squared lengths of the same pair of vectors.
/// #### Panics
/// * `'Ratio: denominator <= 0'` if `den` is not strictly positive.
/// #### Deviations
/// * The last bit is rounded to nearest (ties away from zero, i.e. up: the result is
///   non-negative). This can differ from `Fixed / Fixed` only on exact half-ULP ties, where the
///   scalar rounds to even.
/// * Operands above `2^96` are shifted down by 32 bits before the division; the quotient keeps
///   more than 64 exact bits, far beyond the 32 it is reported with.
pub fn clamped_ratio(num: i128, den: i128) -> Fixed {
    assert(den > 0, errors::NON_POSITIVE_DEN);
    if num <= 0 {
        ZERO
    } else if num >= den {
        ONE
    } else {
        // `0 < num < den`: both conversions succeed and the quotient lies in `(0, 1)`.
        unit_ratio(
            num.try_into().expect(errors::NON_POSITIVE_DEN),
            den.try_into().expect(errors::NON_POSITIVE_DEN),
        )
    }
}

/// `round(num * 2^32 / den)` for `0 < num < den`, as a `Fixed` raw in `[0, 2^32]`.
fn unit_ratio(num: u128, den: u128) -> Fixed {
    // Bring the pair under 2^96 so that `num * 2^32` fits a `u128`.
    let (num, den) = if den >= WIDE_LIMIT {
        let (n, _) = DivRem::div_rem(num, SCALE_NZ);
        let (d, _) = DivRem::div_rem(den, SCALE_NZ);
        (n, d)
    } else {
        (num, den)
    };
    if num >= den {
        // Only reachable when the truncation above made the two equal.
        return ONE;
    }
    let den_nz: NonZero<u128> = den.try_into().expect(errors::NON_POSITIVE_DEN);
    let (raw, rem) = DivRem::div_rem(num * SCALE, den_nz);
    let raw = if rem * 2 >= den {
        raw + 1
    } else {
        raw
    };
    // `raw <= 2^32` by construction, so the conversion never fails.
    Fixed { raw: raw.try_into().expect(errors::NON_POSITIVE_DEN) }
}

/// Rejected candidates, kept for the `gas_*` ranking.
#[cfg(test)]
pub mod alternatives {
    use core::num::traits::DivRem;
    use fixed::{Fixed, FixedTrait, ONE, ZERO};

    const STEP: u128 = 0x10000;

    /// The same exact ratio computed in two 16-bit steps, which is the shape a signed 127-bit
    /// intermediate would force. Identical answers, one `div_rem` more.
    pub fn clamped_ratio_two_step(num: i128, den: i128) -> Fixed {
        assert(den > 0, super::errors::NON_POSITIVE_DEN);
        if num <= 0 {
            return ZERO;
        }
        if num >= den {
            return ONE;
        }
        let num: u128 = num.try_into().unwrap();
        let den: u128 = den.try_into().unwrap();
        let (num, den) = if den >= super::WIDE_LIMIT {
            (num / 0x1_0000_0000, den / 0x1_0000_0000)
        } else {
            (num, den)
        };
        if num >= den {
            return ONE;
        }
        let den_nz: NonZero<u128> = den.try_into().unwrap();
        let (hi, rem) = DivRem::div_rem(num * STEP, den_nz);
        let (lo, rem) = DivRem::div_rem(rem * STEP, den_nz);
        let raw = hi * STEP + lo;
        let raw = if rem * 2 >= den {
            raw + 1
        } else {
            raw
        };
        Fixed { raw: raw.try_into().unwrap() }
    }

    /// `clamped_ratio` written the way upstream reads: narrow both operands to `Fixed`, divide,
    /// clamp. **Wrong** for the inputs this package must handle — it panics with `'Fixed:
    /// division by zero'` as soon as the denominator is a squared length below `2^-32`, panics
    /// with `'Fixed: overflow'` above `2^31`, and loses low bits before dividing.
    pub fn clamped_ratio_narrow(num: i128, den: i128) -> Fixed {
        let n = Fixed { raw: (num / 0x1_0000_0000).try_into().unwrap() };
        let d = Fixed { raw: (den / 0x1_0000_0000).try_into().unwrap() };
        (n / d).clamp(ZERO, ONE)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, HALF, ONE, ZERO};
    use rapier_testing::opaque;
    use super::alternatives::{clamped_ratio_narrow, clamped_ratio_two_step};
    use super::clamped_ratio;

    const Q: i128 = 0x1_0000_0000;

    #[test]
    fn test_exact_and_clamped_values() {
        // (num, den, expected raw)
        let cases: Span<(i128, i128, i64)> = array![
            (0, 5, 0), // exactly 0
            (5, 5, 0x1_0000_0000), // exactly 1
            (7, 5, 0x1_0000_0000), // clamped above
            (-3, 5, 0), // clamped below
            (1, 2, 0x8000_0000), // 0.5, exact
            (1, 4, 0x4000_0000), // 0.25, exact
            (2, 5, 1717986918), // 0.4, rounded to nearest (0.4 * 2^32 = ...918.4)
            (1, 3, 1431655765), // 1/3, rounded to nearest (...765.33)
            (2, 3, 2863311531), // 2/3, rounded up (...530.67)
            (1, 0x1_0000_0000, 1), // one ulp
            (1, 0x2_0000_0000, 1), // half an ulp, rounded up
            (1, 0x4_0000_0000, 0) // a quarter of an ulp, rounded down
        ]
            .span();
        for (num, den, expected) in cases {
            assert_eq!(clamped_ratio(*num, *den), Fixed { raw: *expected }, "{} / {}", *num, *den);
        }
    }

    /// The same ratio at four very different scales gives the same answer: the wide division
    /// neither underflows (squared lengths below one raw unit) nor overflows (above `2^31`).
    #[test]
    fn test_scale_invariance_over_the_whole_range() {
        let expected = Fixed { raw: 0x4000_0000 };
        let scales: Span<i128> = array![
            1, 1024, Q, Q * Q / 1024, Q * Q, 0x1000_0000_0000_0000_0000_0000_0000,
        ]
            .span();
        for s in scales {
            assert_eq!(clamped_ratio(*s, *s * 4), expected, "scale {}", *s);
        }
    }

    /// Rounding is to nearest and never leaves `[0, 1]`: `|2 q d - 2 n 2^32| <= d`.
    #[test]
    fn test_rounding_is_to_nearest_and_bounded() {
        let den = 1000_i128;
        for num in array![0_i128, 1, 137, 274, 411, 500, 685, 822, 959, 999, 1000].span() {
            let got: i128 = clamped_ratio(*num, den).raw.into();
            assert!(got >= 0 && got <= Q, "range {}", *num);
            let err = 2 * got * den - 2 * *num * Q;
            assert!(err >= -den && err <= den, "num {}", *num);
        }
    }

    #[test]
    #[should_panic(expected: 'Ratio: denominator <= 0')]
    fn test_zero_denominator_panics() {
        clamped_ratio(1, 0);
    }

    #[test]
    #[should_panic(expected: 'Ratio: denominator <= 0')]
    fn test_negative_denominator_panics() {
        clamped_ratio(1, -4);
    }

    /// The narrowing candidate agrees with the winner when its operands are representable and no
    /// low bits are lost before the division.
    #[test]
    fn test_alternative_agrees_only_on_representable_operands() {
        assert_eq!(clamped_ratio_narrow(Q * Q, Q * Q * 2), HALF);
        assert_eq!(clamped_ratio_narrow(-Q * Q, Q * Q), ZERO);
        assert_eq!(clamped_ratio_narrow(Q * Q * 3, Q * Q), ONE);
        // The scalar division now rounds 2/3 to nearest, matching the wide quotient here.
        assert_eq!(clamped_ratio(2 * Q * Q, 3 * Q * Q), Fixed { raw: 2863311531 });
        assert_eq!(clamped_ratio_narrow(2 * Q * Q, 3 * Q * Q), Fixed { raw: 2863311531 });
    }

    /// The wide division is the correctly rounded exact quotient for every pair it is given.
    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_matches_the_exact_quotient(num: u32, den: u32) {
        if den == 0 {
            return;
        }
        let n: i128 = num.into();
        let d: i128 = den.into();
        let got: i128 = clamped_ratio(n, d).raw.into();
        if n >= d {
            assert_eq!(got, Q);
        } else {
            let err = 2 * got * d - 2 * n * Q;
            assert!(err >= -d && err <= d);
        }
    }

    /// Both formulations agree to 1 ulp wherever the narrowing one is defined at all.
    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_candidates_agree(num: u16, den: u16) {
        if den == 0 {
            return;
        }
        let n: i128 = num.into();
        let d: i128 = den.into();
        let wide = clamped_ratio(n * Q * Q, d * Q * Q);
        let narrow = clamped_ratio_narrow(n * Q * Q, d * Q * Q);
        let delta = wide.raw - narrow.raw;
        assert!(delta >= 0 && delta <= 1);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_clamped_ratio_interior() {
        let _ = clamped_ratio(opaque(2 * Q * Q), opaque(5 * Q * Q));
    }

    #[test]
    fn gas_clamped_ratio_wide_operands() {
        let _ = clamped_ratio(opaque(2 * Q * Q * Q), opaque(5 * Q * Q * Q));
    }

    #[test]
    fn gas_clamped_ratio_clamped_low() {
        let _ = clamped_ratio(opaque(-2 * Q * Q), opaque(5 * Q * Q));
    }

    /// The two-step candidate is the same function, one `div_rem` more.
    #[test]
    fn test_two_step_candidate_agrees_everywhere() {
        let cases: Span<(i128, i128)> = array![
            (2, 5), (1, 3), (2, 3), (0, 7), (9, 7), (-4, 7), (1, Q), (1, Q * Q),
            (3 * Q * Q, 7 * Q * Q), (Q * Q * Q, 3 * Q * Q * Q),
        ]
            .span();
        for (num, den) in cases {
            assert_eq!(clamped_ratio(*num, *den), clamped_ratio_two_step(*num, *den));
        }
    }

    #[test]
    fn gas_clamped_ratio_two_step() {
        let _ = clamped_ratio_two_step(opaque(2 * Q * Q), opaque(5 * Q * Q));
    }

    #[test]
    fn gas_clamped_ratio_narrow() {
        let _ = clamped_ratio_narrow(opaque(2 * Q * Q), opaque(5 * Q * Q));
    }
}
