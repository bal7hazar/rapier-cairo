//! How the friction / restitution of two touching colliders are combined (upstream
//! `dynamics::CoefficientCombineRule`).
//!
//! Each collider carries a rule; when the two rules differ, the one with the higher priority
//! wins: `GeometricMean > ClampedSum > Max > Multiply > Min > Average`.
//!
//! | Rule | Result | `Fixed` rounding |
//! |---|---|---|
//! | `Average` | `(a + b) / 2` | truncated toward zero (odd raw sum: the half-ULP is dropped) |
//! | `Min` | `abs(min(a, b))` (negative values tolerated, as upstream) | exact |
//! | `Multiply` | `a * b` | floored |
//! | `Max` | `max(a, b)` | exact |
//! | `ClampedSum` | `clamp(a + b, 0, 1)` | exact |
//! | `GeometricMean` | `sqrt(max(a, 0) * max(b, 0))` | product floored, root truncated |
//!
//! Upstream's `combine` takes `(coeff1, coeff2, rule1, rule2)`; so does
//! [`CoefficientCombineRuleTrait::combine`].
//!
//! Candidates (ranked by the `gas_*` probes):
//!
//! * `Average`: a raw `i64` `DivRem` of the sum by 2 **(winner)** ties with
//!   `alternatives::average_mul_half` (`* HALF`), which floors instead of truncating and so
//!   differs on negative odd sums, hence is not preferred; `alternatives::average_div` (`Fixed`
//!   division by `TWO`, a wide division) costs about 2.5k more;
//! * rule resolution: comparing the priorities **(winner)** against
//!   `alternatives::effective_eq_first` (early exit on equal rules: 400 gas cheaper for equal
//!   rules, 1000 dearer for different ones);
//! * dispatch: `alternatives::apply_early` (a `return` per arm, no join) costs exactly as much as
//!   the joined `match`: for a rule unknown at compile time every call pays for the dearest arm
//!   (`GeometricMean`, about 5.7k above `Multiply`), whichever rule it takes. A rule known at
//!   compile time is folded away and pays for its own arm only (`Max` is the cheapest).

use fixed::{Fixed, FixedTrait, ONE, ZERO};

const NZ_TWO_I64: NonZero<i64> = 2;

/// `(a + b) / 2`: a `DivRem` of the raw sum by the constant two, truncated toward zero.
#[inline(always)]
fn average(coeff1: Fixed, coeff2: Fixed) -> Fixed {
    let (half_sum, _) = DivRem::div_rem(coeff1.raw + coeff2.raw, NZ_TWO_I64);
    Fixed { raw: half_sum }
}

/// How to combine friction / restitution values when two colliders touch. Default `Average`.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub enum CoefficientCombineRule {
    /// Average of the two values (default, most common).
    #[default]
    Average,
    /// The smaller value, made positive ("slippery / soft wins").
    Min,
    /// The product of the two values (both must be high).
    Multiply,
    /// The larger value ("sticky / bouncy wins").
    Max,
    /// The sum of the two values, clamped to `[0, 1]`.
    ClampedSum,
    /// The square root of the product of the two values (zero if either is `<= 0`).
    GeometricMean,
}

/// Combination of two coefficients.
#[generate_trait]
pub impl CoefficientCombineRuleImpl of CoefficientCombineRuleTrait {
    /// Upstream's discriminant, the priority of the rule: `Average` 0, `Min` 1, `Multiply` 2,
    /// `Max` 3, `ClampedSum` 4, `GeometricMean` 5.
    #[inline(always)]
    fn priority(self: CoefficientCombineRule) -> u8 {
        match self {
            CoefficientCombineRule::Average => 0,
            CoefficientCombineRule::Min => 1,
            CoefficientCombineRule::Multiply => 2,
            CoefficientCombineRule::Max => 3,
            CoefficientCombineRule::ClampedSum => 4,
            CoefficientCombineRule::GeometricMean => 5,
        }
    }

    /// The rule that applies to a pair of colliders: the one of higher priority (either one
    /// when equal).
    #[inline(always)]
    fn effective(
        rule1: CoefficientCombineRule, rule2: CoefficientCombineRule,
    ) -> CoefficientCombineRule {
        if rule1.priority() >= rule2.priority() {
            rule1
        } else {
            rule2
        }
    }

    /// Combines two coefficients with one rule, see the module table.
    ///
    /// # Panics
    /// * `'i64_add Overflow'` / `'i64_add Underflow'` for `Average` and `ClampedSum` if the sum
    ///   leaves the Q32.32 range.
    /// * `'Fixed: overflow'` for `Multiply` if the product leaves the range.
    /// * `'Fixed: overflow'` for `Min` of `MIN` (`abs` of the most negative value).
    fn apply(self: CoefficientCombineRule, coeff1: Fixed, coeff2: Fixed) -> Fixed {
        match self {
            CoefficientCombineRule::Average => average(coeff1, coeff2),
            CoefficientCombineRule::Min => coeff1.min(coeff2).abs(),
            CoefficientCombineRule::Multiply => coeff1 * coeff2,
            CoefficientCombineRule::Max => coeff1.max(coeff2),
            CoefficientCombineRule::ClampedSum => (coeff1 + coeff2).clamp(ZERO, ONE),
            CoefficientCombineRule::GeometricMean => (coeff1.max(ZERO) * coeff2.max(ZERO)).sqrt(),
        }
    }

    /// Combines the coefficients of two colliders: the rule of higher priority is applied.
    /// Upstream's `CoefficientCombineRule::combine(coeff1, coeff2, rule1, rule2)`.
    ///
    /// # Panics
    /// As [`apply`](CoefficientCombineRuleTrait::apply).
    #[inline(always)]
    fn combine(
        coeff1: Fixed, coeff2: Fixed, rule1: CoefficientCombineRule, rule2: CoefficientCombineRule,
    ) -> Fixed {
        Self::effective(rule1, rule2).apply(coeff1, coeff2)
    }
}

#[cfg(test)]
mod alternatives {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use super::{CoefficientCombineRule, CoefficientCombineRuleTrait, average};

    /// `Fixed` division by two: a wide `2^64`-scaled division for what is a raw halving.
    pub fn average_div(coeff1: Fixed, coeff2: Fixed) -> Fixed {
        (coeff1 + coeff2) / TWO
    }

    /// Multiplication by one half: floors instead of truncating.
    pub fn average_mul_half(coeff1: Fixed, coeff2: Fixed) -> Fixed {
        (coeff1 + coeff2) * HALF
    }

    /// `apply` with a `return` in every arm: no join after the `match`, so each path pays only
    /// its own arm instead of the most expensive one.
    pub fn apply_early(rule: CoefficientCombineRule, coeff1: Fixed, coeff2: Fixed) -> Fixed {
        match rule {
            CoefficientCombineRule::Average => { return average(coeff1, coeff2); },
            CoefficientCombineRule::Min => { return coeff1.min(coeff2).abs(); },
            CoefficientCombineRule::Multiply => { return coeff1 * coeff2; },
            CoefficientCombineRule::Max => { return coeff1.max(coeff2); },
            CoefficientCombineRule::ClampedSum => { return (coeff1 + coeff2).clamp(ZERO, ONE); },
            CoefficientCombineRule::GeometricMean => {
                return (coeff1.max(ZERO) * coeff2.max(ZERO)).sqrt();
            },
        }
    }

    /// Rule resolution with an early return when the rules are equal (the common case).
    pub fn effective_eq_first(
        rule1: CoefficientCombineRule, rule2: CoefficientCombineRule,
    ) -> CoefficientCombineRule {
        if rule1 == rule2 {
            rule1
        } else if rule1.priority() > rule2.priority() {
            rule1
        } else {
            rule2
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, NEG_ONE, ONE, TWO, ZERO};
    use rapier_testing::opaque;
    use super::alternatives::{apply_early, average_div, average_mul_half, effective_eq_first};
    use super::{CoefficientCombineRule, CoefficientCombineRuleTrait, average};

    const AVERAGE: CoefficientCombineRule = CoefficientCombineRule::Average;
    const MIN: CoefficientCombineRule = CoefficientCombineRule::Min;
    const MULTIPLY: CoefficientCombineRule = CoefficientCombineRule::Multiply;
    const MAX: CoefficientCombineRule = CoefficientCombineRule::Max;
    const CLAMPED_SUM: CoefficientCombineRule = CoefficientCombineRule::ClampedSum;
    const GEOMETRIC_MEAN: CoefficientCombineRule = CoefficientCombineRule::GeometricMean;

    /// `0.3` and `0.6`, rounded to nearest.
    const A: Fixed = Fixed { raw: 1288490189 };
    const B: Fixed = Fixed { raw: 2576980378 };
    /// `0.7` and `0.4`, rounded to nearest.
    const C: Fixed = Fixed { raw: 3006477107 };
    const D: Fixed = Fixed { raw: 1717986918 };

    fn rules() -> Array<CoefficientCombineRule> {
        array![AVERAGE, MIN, MULTIPLY, MAX, CLAMPED_SUM, GEOMETRIC_MEAN]
    }

    fn same(c1: Fixed, c2: Fixed, rule: CoefficientCombineRule) -> Fixed {
        CoefficientCombineRuleTrait::combine(c1, c2, rule, rule)
    }

    #[test]
    fn test_default_is_average() {
        let default: CoefficientCombineRule = Default::default();
        assert_eq!(default, AVERAGE);
    }

    #[test]
    fn test_priorities() {
        assert_eq!(AVERAGE.priority(), 0);
        assert_eq!(MIN.priority(), 1);
        assert_eq!(MULTIPLY.priority(), 2);
        assert_eq!(MAX.priority(), 3);
        assert_eq!(CLAMPED_SUM.priority(), 4);
        assert_eq!(GEOMETRIC_MEAN.priority(), 5);
    }

    /// Expected raw values computed with exact integers: `avg = (a + b) / 2`, `min`, `max`,
    /// `mul = floor(a * b / 2^32)`, `clamped = min(a + b, 2^32)`, `geo = isqrt(mul * 2^32)`.
    #[test]
    fn test_average() {
        assert_eq!(same(A, B, AVERAGE).raw, 1932735283);
        assert_eq!(same(B, A, AVERAGE).raw, 1932735283);
        assert_eq!(same(C, D, AVERAGE).raw, 2362232012);
        assert_eq!(same(ONE, ONE, AVERAGE), ONE);
        assert_eq!(same(ZERO, ONE, AVERAGE), HALF);
        assert_eq!(same(NEG_ONE, HALF, AVERAGE).raw, -1073741824);
    }

    /// The halving truncates toward zero, on both signs.
    #[test]
    fn test_average_truncates_toward_zero() {
        assert_eq!(same(Fixed { raw: 1 }, Fixed { raw: 0 }, AVERAGE).raw, 0);
        assert_eq!(same(Fixed { raw: 3 }, Fixed { raw: 0 }, AVERAGE).raw, 1);
        assert_eq!(same(Fixed { raw: -1 }, Fixed { raw: 0 }, AVERAGE).raw, 0);
        assert_eq!(same(Fixed { raw: -3 }, Fixed { raw: 0 }, AVERAGE).raw, -1);
    }

    #[test]
    fn test_min() {
        assert_eq!(same(A, B, MIN), A);
        assert_eq!(same(B, A, MIN), A);
        assert_eq!(same(ZERO, ONE, MIN), ZERO);
        // Negative values are tolerated: the smaller one is made positive.
        assert_eq!(same(NEG_ONE, HALF, MIN), ONE);
        assert_eq!(same(Fixed { raw: -5 }, Fixed { raw: -3 }, MIN).raw, 5);
    }

    #[test]
    fn test_multiply() {
        assert_eq!(same(A, B, MULTIPLY).raw, 773094113);
        assert_eq!(same(C, D, MULTIPLY).raw, 1202590842);
        assert_eq!(same(ONE, HALF, MULTIPLY), HALF);
        assert_eq!(same(ZERO, TWO, MULTIPLY), ZERO);
        // The product floors: raw -1 * raw 1 = -2^-64 -> -1 ULP.
        assert_eq!(same(Fixed { raw: -1 }, Fixed { raw: 1 }, MULTIPLY).raw, -1);
    }

    #[test]
    fn test_max() {
        assert_eq!(same(A, B, MAX), B);
        assert_eq!(same(B, A, MAX), B);
        assert_eq!(same(NEG_ONE, HALF, MAX), HALF);
        assert_eq!(same(ZERO, ZERO, MAX), ZERO);
    }

    #[test]
    fn test_clamped_sum() {
        assert_eq!(same(A, B, CLAMPED_SUM).raw, 3865470567);
        assert_eq!(same(C, D, CLAMPED_SUM), ONE);
        assert_eq!(same(ONE, ONE, CLAMPED_SUM), ONE);
        assert_eq!(same(NEG_ONE, HALF, CLAMPED_SUM), ZERO);
        assert_eq!(same(ZERO, ZERO, CLAMPED_SUM), ZERO);
    }

    #[test]
    fn test_geometric_mean() {
        assert_eq!(same(A, B, GEOMETRIC_MEAN).raw, 1822200299);
        assert_eq!(same(C, D, GEOMETRIC_MEAN).raw, 2272683070);
        // sqrt(0.25 * 1) = 0.5 exactly.
        assert_eq!(same(Fixed { raw: 1073741824 }, ONE, GEOMETRIC_MEAN), HALF);
        // Zero if either is zero.
        assert_eq!(same(Fixed { raw: 3006477107 }, ZERO, GEOMETRIC_MEAN), ZERO);
        // Negative coefficients are clamped to zero first (no negative square root).
        assert_eq!(same(NEG_ONE, HALF, GEOMETRIC_MEAN), ZERO);
        assert_eq!(same(NEG_ONE, NEG_ONE, GEOMETRIC_MEAN), ZERO);
    }

    /// The rule of higher priority wins, in every order, for the 36 ordered pairs.
    #[test]
    fn test_mixed_rules_priority_wins() {
        for r1 in rules() {
            for r2 in rules() {
                let winner = if r1.priority() >= r2.priority() {
                    r1
                } else {
                    r2
                };
                assert_eq!(CoefficientCombineRuleTrait::effective(r1, r2), winner);
                assert_eq!(effective_eq_first(r1, r2), winner);
                assert_eq!(CoefficientCombineRuleTrait::combine(A, B, r1, r2), winner.apply(A, B));
                assert_eq!(CoefficientCombineRuleTrait::combine(A, B, r2, r1), winner.apply(A, B));
            }
        }
    }

    /// The early-return dispatch agrees with `apply` on every rule and several inputs.
    #[test]
    fn test_apply_early_agrees() {
        let values = array![ZERO, ONE, NEG_ONE, A, B, C, D, HALF];
        for rule in rules() {
            for a in values.span() {
                for b in values.span() {
                    assert_eq!(apply_early(rule, *a, *b), rule.apply(*a, *b));
                }
            }
        }
    }

    #[test]
    fn test_mixed_rules_examples() {
        // Average vs Min: Min wins.
        assert_eq!(CoefficientCombineRuleTrait::combine(A, B, AVERAGE, MIN), A);
        // Min vs Multiply: Multiply wins.
        assert_eq!(CoefficientCombineRuleTrait::combine(A, B, MIN, MULTIPLY).raw, 773094113);
        // Multiply vs Max: Max wins.
        assert_eq!(CoefficientCombineRuleTrait::combine(A, B, MAX, MULTIPLY), B);
        // Max vs ClampedSum: ClampedSum wins.
        assert_eq!(CoefficientCombineRuleTrait::combine(C, D, MAX, CLAMPED_SUM), ONE);
        // ClampedSum vs GeometricMean: GeometricMean wins.
        assert_eq!(
            CoefficientCombineRuleTrait::combine(A, B, GEOMETRIC_MEAN, CLAMPED_SUM).raw, 1822200299,
        );
    }

    /// `Fixed` division by two agrees with the winner on every sign; `* HALF` floors, so it
    /// differs from the truncation on negative odd sums only.
    #[test]
    fn test_average_candidates() {
        let values = array![
            ZERO, ONE, NEG_ONE, A, B, C, D, Fixed { raw: 1 }, Fixed { raw: 3 }, Fixed { raw: -3 },
            Fixed { raw: -1 },
        ];
        for a in values.span() {
            for b in values.span() {
                let (a, b) = (*a, *b);
                let winner = same(a, b, AVERAGE);
                assert_eq!(average_div(a, b), winner);
                let odd = (a.raw + b.raw) % 2 != 0;
                if a.raw + b.raw >= 0 || !odd {
                    assert_eq!(average_mul_half(a, b), winner);
                } else {
                    assert_eq!(average_mul_half(a, b).raw, winner.raw - 1);
                }
            }
        }
    }

    #[test]
    #[should_panic(expected: 'Fixed: overflow')]
    fn test_multiply_overflow_panics() {
        same(FixedTrait::from_int(100000), FixedTrait::from_int(100000), MULTIPLY);
    }

    #[test]
    fn gas_baseline() {}

    /// Builds the probe inputs alone: subtract it from the `combine` probes.
    #[test]
    fn gas_inputs() {
        assert!(opaque(A) != opaque(B) && opaque(AVERAGE) != opaque(MIN));
    }

    #[test]
    fn gas_combine_average() {
        let rule = opaque(AVERAGE);
        assert!(
            CoefficientCombineRuleTrait::combine(opaque(A), opaque(B), rule, rule)
                .raw == 1932735283,
        );
    }

    #[test]
    fn gas_combine_min() {
        let rule = opaque(MIN);
        assert!(CoefficientCombineRuleTrait::combine(opaque(A), opaque(B), rule, rule) == A);
    }

    #[test]
    fn gas_combine_multiply() {
        let rule = opaque(MULTIPLY);
        assert!(
            CoefficientCombineRuleTrait::combine(opaque(A), opaque(B), rule, rule).raw == 773094113,
        );
    }

    #[test]
    fn gas_combine_max() {
        let rule = opaque(MAX);
        assert!(CoefficientCombineRuleTrait::combine(opaque(A), opaque(B), rule, rule) == B);
    }

    #[test]
    fn gas_combine_clamped_sum() {
        let rule = opaque(CLAMPED_SUM);
        assert!(
            CoefficientCombineRuleTrait::combine(opaque(A), opaque(B), rule, rule)
                .raw == 3865470567,
        );
    }

    #[test]
    fn gas_combine_geometric_mean() {
        let rule = opaque(GEOMETRIC_MEAN);
        assert!(
            CoefficientCombineRuleTrait::combine(opaque(A), opaque(B), rule, rule)
                .raw == 1822200299,
        );
    }

    /// Mixed rules: resolution plus dispatch (`Average` vs `Max`).
    #[test]
    fn gas_combine_mixed() {
        assert!(
            CoefficientCombineRuleTrait::combine(
                opaque(A), opaque(B), opaque(AVERAGE), opaque(MAX),
            ) == B,
        );
    }

    #[test]
    fn gas_effective() {
        assert!(CoefficientCombineRuleTrait::effective(opaque(AVERAGE), opaque(MAX)) == MAX);
    }

    #[test]
    fn gas_effective_eq_first() {
        assert!(effective_eq_first(opaque(AVERAGE), opaque(MAX)) == MAX);
    }

    #[test]
    fn gas_effective_equal_rules() {
        assert!(CoefficientCombineRuleTrait::effective(opaque(MAX), opaque(MAX)) == MAX);
    }

    #[test]
    fn gas_effective_eq_first_equal_rules() {
        assert!(effective_eq_first(opaque(MAX), opaque(MAX)) == MAX);
    }

    #[test]
    fn gas_apply_early_average() {
        assert!(apply_early(opaque(AVERAGE), opaque(A), opaque(B)).raw == 1932735283);
    }

    #[test]
    fn gas_apply_early_min() {
        assert!(apply_early(opaque(MIN), opaque(A), opaque(B)) == A);
    }

    #[test]
    fn gas_apply_early_max() {
        assert!(apply_early(opaque(MAX), opaque(A), opaque(B)) == B);
    }

    #[test]
    fn gas_apply_early_geometric_mean() {
        assert!(apply_early(opaque(GEOMETRIC_MEAN), opaque(A), opaque(B)).raw == 1822200299);
    }

    #[test]
    fn gas_apply_dispatch_average() {
        assert!(opaque(AVERAGE).apply(opaque(A), opaque(B)).raw == 1932735283);
    }

    #[test]
    fn gas_apply_dispatch_geometric_mean() {
        assert!(opaque(GEOMETRIC_MEAN).apply(opaque(A), opaque(B)).raw == 1822200299);
    }

    #[test]
    fn gas_average_divrem() {
        assert!(average(opaque(A), opaque(B)).raw == 1932735283);
    }

    /// The full dispatch on a rule known at compile time (the `match` may be folded away).
    #[test]
    fn gas_apply_const_average() {
        assert!(AVERAGE.apply(opaque(A), opaque(B)).raw == 1932735283);
    }

    #[test]
    fn gas_apply_const_min() {
        assert!(MIN.apply(opaque(A), opaque(B)) == A);
    }

    #[test]
    fn gas_apply_const_multiply() {
        assert!(MULTIPLY.apply(opaque(A), opaque(B)).raw == 773094113);
    }

    #[test]
    fn gas_apply_const_max() {
        assert!(MAX.apply(opaque(A), opaque(B)) == B);
    }

    #[test]
    fn gas_apply_const_clamped_sum() {
        assert!(CLAMPED_SUM.apply(opaque(A), opaque(B)).raw == 3865470567);
    }

    #[test]
    fn gas_apply_const_geometric_mean() {
        assert!(GEOMETRIC_MEAN.apply(opaque(A), opaque(B)).raw == 1822200299);
    }

    #[test]
    fn gas_average_div() {
        assert!(average_div(opaque(A), opaque(B)).raw == 1932735283);
    }

    #[test]
    fn gas_average_mul_half() {
        assert!(average_mul_half(opaque(A), opaque(B)).raw == 1932735283);
    }
}
