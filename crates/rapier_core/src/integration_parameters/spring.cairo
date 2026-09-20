//! Spring-like constraint softness (upstream `SpringCoefficients`).
//!
//! A [`SpringCoefficients`] is the *user-facing* description of a soft constraint (natural
//! frequency in Hz and damping ratio). The solver never uses it directly: once per step it turns it
//! into the four numbers of a [`SoftnessCoefficients`] evaluated at the substep length, and reads
//! only those inside the substep loop (no division, no square root there).
//!
//! With `ω = 2π · natural_frequency`, `ζ = damping_ratio` and `x = dt · ω`, upstream computes
//!
//! ```text
//! erp_inv_dt = ω / (x + 2ζ)          erp        = dt · erp_inv_dt = x / (x + 2ζ)
//! cfm_coeff  = 1 / (x · (x + 2ζ))    cfm_factor = 1 / (1 + cfm_coeff)
//! ```
//!
//! (upstream's "damped" `cfm_coeff` branch `(1/erp - 1)² / ((1/erp) · 4ζ²)` simplifies to the
//! same closed form as its "undamped" branch, so a single expression covers both), and forces
//! `erp = cfm_coeff = 0`, `cfm_factor = 1` when `erp` is zero.
//!
//! Numerics (decision D3, measured by `tests/integration_parameters.cairo`): the shipped kernel
//! is the plain `Fixed` operator chain above, in that order, except that `cfm_coeff` is evaluated
//! as `(1 / x) / (x + 2ζ)` instead of `1 / (x · (x + 2ζ))`: the product reaches `1e10` for the
//! default joint softness at 60 Hz with one substep, beyond the Q32.32 range, while the quotient
//! form never overflows for a valid result. Every `*` floors and every `/` truncates. With `dt`
//! already a Q32.32 value the outputs of the three default springs are within 1 ulp of the `f64`
//! upstream, including the joint softness whose `cfm_coeff` is only ~6 ulp: Q32.32 reproduces
//! the coefficients, the (unavoidable) coarseness is the *resolution* of `cfm_coeff` itself.
//!
//! An exact `u128` kernel (0 ulp, round to nearest, wider range) costs 2.4× the gas and is kept
//! in `alternatives` together with a `Recip` variant and upstream's literal expression.

use fixed::{Fixed, ONE, TAU, ZERO};

/// Failure modes of the spring maths.
pub mod errors {
    /// A frequency, damping ratio or step length was negative.
    pub const NEGATIVE_INPUT: felt252 = 'Spring: negative input';
    /// `dt · ω + 2ζ` is zero (upstream divides by zero: `inf` / `NaN`).
    pub const ZERO_DENOMINATOR: felt252 = 'Spring: zero denominator';
    /// `1 / (dt · ω)` does not fit the Q32.32 range (`dt · ω` is zero or below `2^-31`).
    pub const UNDERFLOW: felt252 = 'Spring: underflow';
}

/// Softness description of a constraint (upstream `SpringCoefficients<Real>`).
///
/// Both fields must be non-negative; constructors of [`SpringCoefficientsTrait`] check it, the
/// evaluation functions check it again (the fields are public).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SpringCoefficients {
    /// Natural frequency (Hz) of the spring-like constraint. Higher values make the constraint
    /// stiffer. Range `[0, 2^31 / 2π)`, i.e. below ~3.4e8 Hz.
    pub natural_frequency: Fixed,
    /// Damping ratio. Larger values make the constraint more compliant. Range `[0, 2^31)`.
    pub damping_ratio: Fixed,
}

/// The four quantities the solver reads per constraint, evaluated at one step length `dt`
/// (normally the substep length). Every field is floored or truncated, see the field docs.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SoftnessCoefficients {
    /// `ω / (dt · ω + 2ζ)`: [`Self::erp`] divided by the step length. Truncated.
    pub erp_inv_dt: Fixed,
    /// Error reduction parameter `dt · erp_inv_dt = dt · ω / (dt · ω + 2ζ)`, in `[0, 1]`.
    /// Floored.
    pub erp: Fixed,
    /// `1 / (dt · ω · (dt · ω + 2ζ))`: the softening added to the inverse projected mass.
    /// Zero when `erp` is zero. Truncated. Its resolution is one Q32.32 ulp (`2^-32`): the default
    /// joint softness at 240 Hz yields `6`, i.e. only ~1 significant digit; prefer
    /// [`Self::cfm_factor`] for anything that scales with impulses.
    pub cfm_coeff: Fixed,
    /// `1 / (1 + cfm_coeff)`, in `[0, 1]`: the factor the impulse update multiplies by, computed
    /// from the truncated `cfm_coeff`. Truncated.
    pub cfm_factor: Fixed,
}

/// Constructors, defaults and evaluation of [`SpringCoefficients`].
#[generate_trait]
pub impl SpringCoefficientsImpl of SpringCoefficientsTrait {
    /// Builds a spring from its natural frequency (Hz) and damping ratio.
    ///
    /// # Panics
    /// * `Spring: negative input` if either argument is negative.
    fn new(natural_frequency: Fixed, damping_ratio: Fixed) -> SpringCoefficients {
        assert(natural_frequency.raw >= 0 && damping_ratio.raw >= 0, errors::NEGATIVE_INPUT);
        SpringCoefficients { natural_frequency, damping_ratio }
    }

    /// Default softness of contacts between two dynamic bodies: 30 Hz, ζ = 10
    /// (upstream `contact_defaults`). Exact in Q32.32.
    fn contact_defaults() -> SpringCoefficients {
        CONTACT_DEFAULTS
    }

    /// Default softness of contacts touching a fixed body: 60 Hz, ζ = 10
    /// (upstream `contact_static_defaults`). Exact in Q32.32.
    fn contact_static_defaults() -> SpringCoefficients {
        CONTACT_STATIC_DEFAULTS
    }

    /// Default softness of joints: 1e6 Hz, ζ = 1 (upstream `joint_defaults`). Exact in Q32.32.
    fn joint_defaults() -> SpringCoefficients {
        JOINT_DEFAULTS
    }

    /// Angular frequency `ω = 2π · natural_frequency` (rad/s): `natural_frequency * TAU` with
    /// the Q32.32 `TAU`, floored. `TAU` carries a relative error of 1.1e-12, i.e. the result is up
    /// to 1 ulp off for 30 Hz and 44038 ulp (1.6e-12 relative) off for 1e6 Hz; measured to be
    /// immaterial for the derived coefficients (`erp_inv_dt` is insensitive to `ω` when `dt · ω`
    /// dominates `2ζ`, as it does exactly when `ω` is that large).
    ///
    /// # Panics
    /// * `Spring: negative input` if the frequency is negative.
    /// * `Fixed: overflow` if the result exceeds the Q32.32 range.
    fn angular_frequency(self: SpringCoefficients) -> Fixed {
        assert(self.natural_frequency.raw >= 0, errors::NEGATIVE_INPUT);
        self.natural_frequency * TAU
    }

    /// `erp / dt`, i.e. `ω / (dt · ω + 2ζ)`, truncated. At `dt = 0` this is `ω / 2ζ`.
    ///
    /// # Panics
    /// * `Spring: negative input` if an input is negative.
    /// * `Spring: zero denominator` if `dt · ω + 2ζ` is zero (upstream: `NaN`).
    /// * `Fixed: overflow` if `ω`, `dt · ω` or the result exceeds the Q32.32 range.
    fn erp_inv_dt(self: SpringCoefficients, dt: Fixed) -> Fixed {
        let (omega, _, denom) = terms(self, dt);
        quotient_erp_inv_dt(omega, denom)
    }

    /// Error reduction parameter `dt * erp_inv_dt` in `[0, 1]` (upstream's expression), floored;
    /// zero at `dt = 0`.
    ///
    /// # Panics
    /// * The panics of [`Self::erp_inv_dt`].
    fn erp(self: SpringCoefficients, dt: Fixed) -> Fixed {
        let (omega, _, denom) = terms(self, dt);
        dt * quotient_erp_inv_dt(omega, denom)
    }

    /// `1 / (dt · ω · (dt · ω + 2ζ))`, truncated; zero when `erp` is zero.
    ///
    /// # Panics
    /// * The panics of [`Self::erp_inv_dt`].
    /// * `Spring: underflow` if `erp` is not zero but `dt · ω` is below `2^-31` (the result
    ///   exceeds the Q32.32 range).
    /// * `Fixed: overflow` if the result exceeds the Q32.32 range.
    fn cfm_coeff(self: SpringCoefficients, dt: Fixed) -> Fixed {
        let (omega, x, denom) = terms(self, dt);
        let erp = dt * quotient_erp_inv_dt(omega, denom);
        cfm_of(erp, x, denom)
    }

    /// `1 / (1 + cfm_coeff)` in `[0, 1]`, truncated; one when `erp` is zero.
    ///
    /// # Panics
    /// * The panics of [`Self::cfm_coeff`].
    fn cfm_factor(self: SpringCoefficients, dt: Fixed) -> Fixed {
        let (omega, x, denom) = terms(self, dt);
        let erp = dt * quotient_erp_inv_dt(omega, denom);
        ONE / (ONE + cfm_of(erp, x, denom))
    }

    /// All four [`SoftnessCoefficients`] at step length `dt`, sharing one evaluation of `ω`,
    /// `dt · ω` and `dt · ω + 2ζ` (four `Fixed` divisions in total). Call it once per step.
    ///
    /// # Panics
    /// * `Spring: negative input` if an input is negative.
    /// * `Spring: zero denominator` if `dt · ω + 2ζ` is zero (upstream: `NaN`).
    /// * `Spring: underflow` if `erp` is not zero but `dt · ω` is below `2^-31`.
    /// * `Fixed: overflow` if an intermediate or a result exceeds the Q32.32 range.
    fn coefficients(self: SpringCoefficients, dt: Fixed) -> SoftnessCoefficients {
        let (omega, x, denom) = terms(self, dt);
        let erp_inv_dt = quotient_erp_inv_dt(omega, denom);
        let erp = dt * erp_inv_dt;
        let cfm_coeff = cfm_of(erp, x, denom);
        SoftnessCoefficients { erp_inv_dt, erp, cfm_coeff, cfm_factor: ONE / (ONE + cfm_coeff) }
    }
}

pub const CONTACT_DEFAULTS: SpringCoefficients = SpringCoefficients {
    natural_frequency: Fixed { raw: 128849018880 }, damping_ratio: Fixed { raw: 42949672960 },
};
pub const CONTACT_STATIC_DEFAULTS: SpringCoefficients = SpringCoefficients {
    natural_frequency: Fixed { raw: 257698037760 }, damping_ratio: Fixed { raw: 42949672960 },
};
pub const JOINT_DEFAULTS: SpringCoefficients = SpringCoefficients {
    natural_frequency: Fixed { raw: 4294967296000000 }, damping_ratio: ONE,
};

/// `(ω, x = dt · ω, x + 2ζ)` after checking the inputs are non-negative.
#[inline(always)]
fn terms(spring: SpringCoefficients, dt: Fixed) -> (Fixed, Fixed, Fixed) {
    assert(
        spring.natural_frequency.raw >= 0 && spring.damping_ratio.raw >= 0 && dt.raw >= 0,
        errors::NEGATIVE_INPUT,
    );
    let omega = spring.natural_frequency * TAU;
    let x = dt * omega;
    (omega, x, x + spring.damping_ratio + spring.damping_ratio)
}

#[inline(always)]
fn quotient_erp_inv_dt(omega: Fixed, denom: Fixed) -> Fixed {
    assert(denom.raw != 0, errors::ZERO_DENOMINATOR);
    omega / denom
}

/// `cfm_coeff` for a given `erp`: zero when `erp` is zero (upstream), else `(1 / x) / denom`.
#[inline(always)]
fn cfm_of(erp: Fixed, x: Fixed, denom: Fixed) -> Fixed {
    if erp.raw == 0 {
        ZERO
    } else {
        assert(x.raw != 0, errors::UNDERFLOW);
        ONE / x / denom
    }
}

#[cfg(test)]
mod alternatives {
    //! Rejected candidates for [`SpringCoefficientsTrait::coefficients`], kept for re-ranking.
    use fixed::wide::{RecipTrait, WideNarrow, wide_mul};
    use fixed::{Fixed, ONE, TAU, ZERO};
    use super::{SoftnessCoefficients, SpringCoefficients};

    const FOUR: Fixed = Fixed { raw: 0x400000000 };

    /// The chain with `cfm_coeff = 1 / (x · (x + 2ζ))` as written in the module doc: same cost,
    /// but the product overflows Q32.32 (`Fixed: overflow`) once `x · (x + 2ζ) >= 2^31`, e.g. the
    /// default joint softness at `dt = 1/60`.
    pub fn coefficients_chain_product(s: SpringCoefficients, dt: Fixed) -> SoftnessCoefficients {
        let omega = s.natural_frequency * TAU;
        let x = dt * omega;
        let denom = x + s.damping_ratio + s.damping_ratio;
        let erp_inv_dt = omega / denom;
        let erp = dt * erp_inv_dt;
        let cfm_coeff = if erp.raw == 0 {
            ZERO
        } else {
            ONE / (x * denom)
        };
        SoftnessCoefficients { erp_inv_dt, erp, cfm_coeff, cfm_factor: ONE / (ONE + cfm_coeff) }
    }

    /// `fixed::wide` variant: one shared [`RecipTrait`] reciprocal of the denominator (quotients
    /// rounded to nearest) plus one reciprocal for `cfm_coeff` and one for `cfm_factor`. Rounds
    /// `x · denom` to Q32.32, so it overflows like [`coefficients_chain_product`].
    pub fn coefficients_recip(s: SpringCoefficients, dt: Fixed) -> SoftnessCoefficients {
        let omega = s.natural_frequency * TAU;
        let x = wide_mul(dt, omega).narrow();
        let denom = x + s.damping_ratio + s.damping_ratio;
        let r = RecipTrait::new(denom);
        let erp_inv_dt = r.mul(omega);
        let erp = r.mul(x);
        let cfm_coeff = if erp.raw == 0 {
            ZERO
        } else {
            RecipTrait::new(wide_mul(x, denom).narrow()).mul(ONE)
        };
        SoftnessCoefficients {
            erp_inv_dt, erp, cfm_coeff, cfm_factor: RecipTrait::new(ONE + cfm_coeff).mul(ONE),
        }
    }

    /// Upstream's literal "damped" `cfm_coeff` branch `(1/erp - 1)² / ((1/erp) · 4ζ²)` with
    /// `Fixed` operators. Measured: within 1 ulp of the closed form on the default springs at the
    /// substep length, at three times the gas of the closed form.
    pub fn cfm_coeff_upstream_literal(s: SpringCoefficients, dt: Fixed) -> Fixed {
        let omega = s.natural_frequency * TAU;
        let denom = dt * omega + s.damping_ratio + s.damping_ratio;
        let erp = dt * (omega / denom);
        if erp.raw == 0 {
            return ZERO;
        }
        let inv_erp_minus_one = ONE / erp - ONE;
        inv_erp_minus_one
            * inv_erp_minus_one
            / ((ONE + inv_erp_minus_one) * FOUR * s.damping_ratio * s.damping_ratio)
    }

    /// Exact kernel: every quotient is a `u128` division of an exact numerator, rounded to nearest
    /// (0 ulp against the `f64` upstream on the default springs; `ω` from `2π` with 60 fractional
    /// bits). Costs 2.4× the chain and adds nothing measurable on the physical range.
    pub mod wide {
        use fixed::{Fixed, ONE, ZERO};
        use super::super::{SoftnessCoefficients, SpringCoefficients};

        /// `round(2π · 2^60)`.
        const TAU_Q60: u128 = 7244019458077122842;
        const NZ_TWO_POW_60: NonZero<u128> = 0x1000000000000000;
        const NZ_TWO_POW_32: NonZero<u128> = 0x100000000;
        const TWO_POW_32: u128 = 0x100000000;
        const TWO_POW_64: u128 = 0x10000000000000000;
        const TWO_POW_96: u128 = 0x1000000000000000000000000;
        const I64_MAX: u128 = 0x7fffffffffffffff;

        pub mod errors {
            pub const NEGATIVE_INPUT: felt252 = 'Spring: negative input';
            pub const ZERO_DENOMINATOR: felt252 = 'Spring: zero denominator';
            pub const OVERFLOW: felt252 = 'Spring: overflow';
        }

        #[derive(Copy, Drop)]
        struct Terms {
            omega: u128,
            x: u128,
            denom: u128,
        }

        fn unsigned(v: Fixed) -> u128 {
            assert(v.raw >= 0, errors::NEGATIVE_INPUT);
            v.raw.try_into().expect(errors::NEGATIVE_INPUT)
        }

        fn to_fixed(v: u128) -> Fixed {
            Fixed { raw: v.try_into().expect(errors::OVERFLOW) }
        }

        /// `n / d` rounded to nearest (ties up). `d` must be non-zero.
        fn div_round(n: u128, d: u128) -> u128 {
            let (q, r) = DivRem::div_rem(n, d.try_into().expect(errors::ZERO_DENOMINATOR));
            if r >= d - r {
                q + 1
            } else {
                q
            }
        }

        /// `ω = natural_frequency · 2π`, rounded to nearest.
        pub fn angular_frequency(s: SpringCoefficients) -> Fixed {
            to_fixed(omega_of(unsigned(s.natural_frequency)))
        }

        fn omega_of(natural_frequency: u128) -> u128 {
            let (q, r) = DivRem::div_rem(natural_frequency * TAU_Q60, NZ_TWO_POW_60);
            let omega = if r >= 0x800000000000000 {
                q + 1
            } else {
                q
            };
            assert(omega <= I64_MAX, errors::OVERFLOW);
            omega
        }

        fn terms(spring: SpringCoefficients, dt: Fixed) -> Terms {
            let omega = omega_of(unsigned(spring.natural_frequency));
            let damping = unsigned(spring.damping_ratio);
            let (q, r) = DivRem::div_rem(unsigned(dt) * omega, NZ_TWO_POW_32);
            let x = if r >= 0x80000000 {
                q + 1
            } else {
                q
            };
            assert(x <= I64_MAX, errors::OVERFLOW);
            Terms { omega, x, denom: x + 2 * damping }
        }

        /// The four coefficients, each with a single rounding to nearest.
        pub fn coefficients(s: SpringCoefficients, dt: Fixed) -> SoftnessCoefficients {
            let t = terms(s, dt);
            assert(t.denom != 0, errors::ZERO_DENOMINATOR);
            let erp_inv_dt = to_fixed(div_round(t.omega * TWO_POW_32, t.denom));
            if t.x == 0 {
                return SoftnessCoefficients {
                    erp_inv_dt, erp: ZERO, cfm_coeff: ZERO, cfm_factor: ONE,
                };
            }
            let xd = t.x * t.denom;
            SoftnessCoefficients {
                erp_inv_dt,
                erp: to_fixed(div_round(t.x * TWO_POW_32, t.denom)),
                cfm_coeff: to_fixed(div_round(TWO_POW_96, xd)),
                cfm_factor: to_fixed(TWO_POW_32 - div_round(TWO_POW_96, xd + TWO_POW_64)),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, ONE, ZERO};
    use rapier_testing::opaque;
    use super::alternatives::{
        cfm_coeff_upstream_literal, coefficients_chain_product, coefficients_recip, wide,
    };
    use super::{SoftnessCoefficients, SpringCoefficients, SpringCoefficientsTrait};

    /// dt of the default 60 Hz step divided by the 4 default solver iterations.
    const SUBSTEP: Fixed = Fixed { raw: 17895697 };

    fn near(a: Fixed, b: Fixed, tolerance: i64) -> bool {
        let d = a.raw - b.raw;
        d <= tolerance && -d <= tolerance
    }

    #[test]
    fn test_default_springs() {
        let c = SpringCoefficientsTrait::contact_defaults();
        assert_eq!(c.natural_frequency.raw, 30 * 0x100000000);
        assert_eq!(c.damping_ratio.raw, 10 * 0x100000000);
        let s = SpringCoefficientsTrait::contact_static_defaults();
        assert_eq!(s.natural_frequency.raw, 60 * 0x100000000);
        assert_eq!(s.damping_ratio, c.damping_ratio);
        let j = SpringCoefficientsTrait::joint_defaults();
        assert_eq!(j.natural_frequency.raw, 1000000 * 0x100000000);
        assert_eq!(j.damping_ratio, ONE);
    }

    #[test]
    fn test_angular_frequency_is_two_pi_times_frequency() {
        // Floored product with the Q32.32 TAU; the exact values are 809582262271, 1619164524543
        // and 26986075409044038 (the `wide` kernel reproduces them).
        let c = SpringCoefficientsTrait::contact_defaults();
        let s = SpringCoefficientsTrait::contact_static_defaults();
        let j = SpringCoefficientsTrait::joint_defaults();
        assert_eq!(c.angular_frequency().raw, 809582262270);
        assert_eq!(s.angular_frequency().raw, 1619164524540);
        assert_eq!(j.angular_frequency().raw, 26986075409000000);
        assert_eq!(wide::angular_frequency(c).raw, 809582262271);
        assert_eq!(wide::angular_frequency(s).raw, 1619164524543);
        assert_eq!(wide::angular_frequency(j).raw, 26986075409044038);
        assert_eq!(SpringCoefficientsTrait::new(ZERO, ONE).angular_frequency(), ZERO);
    }

    #[test]
    fn test_individual_functions_agree_with_coefficients() {
        let springs = array![
            SpringCoefficientsTrait::contact_defaults(),
            SpringCoefficientsTrait::contact_static_defaults(),
            SpringCoefficientsTrait::joint_defaults(),
        ];
        for s in springs.span() {
            let all = (*s).coefficients(SUBSTEP);
            assert_eq!((*s).erp_inv_dt(SUBSTEP), all.erp_inv_dt);
            assert_eq!((*s).erp(SUBSTEP), all.erp);
            assert_eq!((*s).cfm_coeff(SUBSTEP), all.cfm_coeff);
            assert_eq!((*s).cfm_factor(SUBSTEP), all.cfm_factor);
        }
    }

    #[test]
    fn test_contact_at_substep() {
        // Within 1 ulp of the f64 upstream fed with the same Q32.32 `dt` (fixture `dt_q32`:
        // 38949567192, 162289863, 263094418, 4047058867).
        let got = SpringCoefficientsTrait::contact_defaults().coefficients(SUBSTEP);
        assert_eq!(
            got,
            SoftnessCoefficients {
                erp_inv_dt: Fixed { raw: 38949567192 },
                erp: Fixed { raw: 162289862 },
                cfm_coeff: Fixed { raw: 263094417 },
                cfm_factor: Fixed { raw: 4047058867 },
            },
        );
    }

    #[test]
    fn test_undamped_spring_has_closed_form() {
        // ζ = 0: erp = 1, cfm_coeff = 1 / x², cfm_factor = x² / (1 + x²) with x = dt · ω.
        // f = 1 / 2π Hz and dt = 1 give ω = 1 up to the rounding of the frequency, hence x ≈ 1.
        let s = SpringCoefficientsTrait::new(Fixed { raw: 683565276 }, ZERO);
        let got = s.coefficients(ONE);
        assert!(near(got.erp, ONE, 8));
        assert!(near(got.cfm_coeff, ONE, 16));
        assert!(near(got.cfm_factor, Fixed { raw: 0x80000000 }, 16));
        // erp_inv_dt = ω / x = 1 / dt.
        assert!(near(got.erp_inv_dt, ONE, 8));
    }

    #[test]
    fn test_zero_dt_mirrors_upstream() {
        // erp = 0 forces cfm_coeff = 0 and cfm_factor = 1; erp_inv_dt = ω / 2ζ stays finite.
        let s = SpringCoefficientsTrait::contact_defaults();
        let got = s.coefficients(ZERO);
        assert_eq!(got.erp, ZERO);
        assert_eq!(got.cfm_coeff, ZERO);
        assert_eq!(got.cfm_factor, ONE);
        assert_eq!(got.erp_inv_dt.raw, 40479113113); // ω / 20, truncated
        assert_eq!(s.erp(ZERO), ZERO);
        assert_eq!(s.cfm_coeff(ZERO), ZERO);
        assert_eq!(s.cfm_factor(ZERO), ONE);
    }

    #[test]
    fn test_zero_frequency_has_no_stiffness() {
        let s = SpringCoefficientsTrait::new(ZERO, ONE);
        let got = s.coefficients(SUBSTEP);
        assert_eq!(got.erp_inv_dt, ZERO);
        assert_eq!(got.erp, ZERO);
        assert_eq!(got.cfm_coeff, ZERO);
        assert_eq!(got.cfm_factor, ONE);
    }

    #[test]
    fn test_coefficients_are_monotone_in_dt() {
        // A longer step gives a larger erp and a smaller cfm_coeff.
        let s = SpringCoefficientsTrait::contact_defaults();
        let short = s.coefficients(SUBSTEP);
        let long = s.coefficients(Fixed { raw: 2 * SUBSTEP.raw });
        assert!(long.erp > short.erp);
        assert!(long.cfm_coeff < short.cfm_coeff);
        assert!(long.cfm_factor > short.cfm_factor);
        assert!(long.erp_inv_dt < short.erp_inv_dt);
    }

    #[test]
    fn test_large_x_underflows_to_unit_factor() {
        // dt = 1/60 on the joint spring (one substep): x ≈ 1e5, cfm_coeff ≈ 1e-10 truncates to
        // 0 and the factor to exactly 1. The product form `1 / (x · denom)` overflows on this
        // input.
        let dt = Fixed { raw: 71582788 };
        let got = SpringCoefficientsTrait::joint_defaults().coefficients(dt);
        assert_eq!(got.cfm_coeff, ZERO);
        assert_eq!(got.cfm_factor, ONE);
        // erp = 1 − 2 / (x + 2) with x = 104719.
        assert!(near(got.erp, Fixed { raw: 0x100000000 - 82000 }, 1000));
        assert_eq!(
            wide::coefficients(SpringCoefficientsTrait::joint_defaults(), dt).cfm_coeff, ZERO,
        );
    }

    #[test]
    #[should_panic(expected: ('Fixed: overflow',))]
    fn test_product_form_overflows_where_the_shipped_form_does_not() {
        coefficients_chain_product(
            SpringCoefficientsTrait::joint_defaults(), Fixed { raw: 71582788 },
        );
    }

    #[test]
    #[should_panic(expected: ('Spring: negative input',))]
    fn test_negative_dt_panics() {
        SpringCoefficientsTrait::contact_defaults().coefficients(Fixed { raw: -1 });
    }

    #[test]
    #[should_panic(expected: ('Spring: negative input',))]
    fn test_negative_frequency_panics() {
        SpringCoefficients { natural_frequency: Fixed { raw: -1 }, damping_ratio: ONE }
            .coefficients(SUBSTEP);
    }

    #[test]
    #[should_panic(expected: ('Spring: negative input',))]
    fn test_new_rejects_negative_frequency() {
        SpringCoefficientsTrait::new(Fixed { raw: -1 }, ONE);
    }

    #[test]
    #[should_panic(expected: ('Spring: negative input',))]
    fn test_new_rejects_negative_damping() {
        SpringCoefficientsTrait::new(ONE, Fixed { raw: -1 });
    }

    #[test]
    #[should_panic(expected: ('Spring: zero denominator',))]
    fn test_zero_dt_and_zero_damping_panics() {
        // Upstream evaluates ω / 0.
        SpringCoefficientsTrait::new(ONE, ZERO).coefficients(ZERO);
    }

    #[test]
    #[should_panic(expected: ('Fixed: overflow',))]
    fn test_huge_dt_overflows() {
        // x = dt · ω does not fit Q32.32.
        SpringCoefficientsTrait::joint_defaults().coefficients(Fixed { raw: 0x7fffffffffffffff });
    }

    #[test]
    #[should_panic(expected: ('Fixed: overflow',))]
    fn test_tiny_undamped_step_overflows_cfm() {
        // cfm_coeff = 1 / x² is far beyond the Q32.32 range for x = 2^-16.
        SpringCoefficientsTrait::new(ONE, ZERO).coefficients(Fixed { raw: 10 });
    }

    #[test]
    #[should_panic(expected: ('Spring: underflow',))]
    fn test_vanishing_x_with_nonzero_erp_underflows() {
        // ω ≈ 2^20 raw and dt = 2^11 raw give dt · ω = 0.5 ulp, which floors to zero, while
        // erp = dt · (ω / 2ζ) is 4 ulp for ζ = 1/16: cfm_coeff is out of range.
        let s = SpringCoefficientsTrait::new(Fixed { raw: 166886 }, Fixed { raw: 0x10000000 });
        s.coefficients(Fixed { raw: 2048 });
    }

    #[test]
    fn test_alternatives_agree_on_defaults() {
        let springs = array![
            SpringCoefficientsTrait::contact_defaults(),
            SpringCoefficientsTrait::contact_static_defaults(),
            SpringCoefficientsTrait::joint_defaults(),
        ];
        for s in springs.span() {
            let shipped = (*s).coefficients(SUBSTEP);
            let exact = wide::coefficients(*s, SUBSTEP);
            let product = coefficients_chain_product(*s, SUBSTEP);
            let recip = coefficients_recip(*s, SUBSTEP);
            assert_eq!(product, shipped);
            for other in array![exact, recip].span() {
                assert!(near(*other.erp_inv_dt, shipped.erp_inv_dt, 2));
                assert!(near(*other.erp, shipped.erp, 2));
                assert!(near(*other.cfm_coeff, shipped.cfm_coeff, 2));
                assert!(near(*other.cfm_factor, shipped.cfm_factor, 2));
            }
            assert!(near(cfm_coeff_upstream_literal(*s, SUBSTEP), shipped.cfm_coeff, 1));
        }
    }

    /// The shipped chain agrees with the exact kernel to a relative 2^-22 (2.4e-7) plus 4 ulp over
    /// 1..100 Hz, ζ in 1/16..5 and dt in 1/1024..1/64 s (so `x = dt · ω >= 6e-3`). The dominant
    /// error is the 1 ulp floor of `x`, i.e. a relative `2^-32 / x <= 3.8e-8` that `erp_inv_dt`,
    /// `erp` and `cfm_coeff` inherit.
    #[test]
    #[fuzzer(runs: 256, seed: 1)]
    fn fuzz_chain_vs_exact(frequency: u32, damping: u32, step: u32) {
        let s = SpringCoefficientsTrait::new(
            Fixed { raw: 0x100000000 + frequency.into() * 23 },
            Fixed { raw: 0x10000000 + damping.into() * 5 },
        );
        let dt = Fixed { raw: 0x400000 + (step % 0x3c00000).into() };
        let shipped = s.coefficients(dt);
        let exact = wide::coefficients(s, dt);
        assert!(near(shipped.erp_inv_dt, exact.erp_inv_dt, 4 + exact.erp_inv_dt.raw / 0x400000));
        assert!(near(shipped.erp, exact.erp, 4 + exact.erp.raw / 0x400000));
        assert!(near(shipped.cfm_coeff, exact.cfm_coeff, 4 + exact.cfm_coeff.raw / 0x400000));
        assert!(near(shipped.cfm_factor, exact.cfm_factor, 4 + exact.cfm_factor.raw / 0x1000000));
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_coefficients_chain() {
        let s = opaque(SpringCoefficientsTrait::contact_defaults());
        let c = s.coefficients(opaque(SUBSTEP));
        assert_eq!(c.cfm_factor.raw, 4047058867);
    }

    #[test]
    fn gas_coefficients_chain_product() {
        let s = opaque(SpringCoefficientsTrait::contact_defaults());
        let c = coefficients_chain_product(s, opaque(SUBSTEP));
        assert_eq!(c.cfm_factor.raw, 4047058867);
    }

    #[test]
    fn gas_coefficients_recip() {
        let s = opaque(SpringCoefficientsTrait::contact_defaults());
        let c = coefficients_recip(s, opaque(SUBSTEP));
        assert!(c.cfm_factor.raw > 0);
    }

    #[test]
    fn gas_coefficients_wide() {
        let s = opaque(SpringCoefficientsTrait::contact_defaults());
        let c = wide::coefficients(s, opaque(SUBSTEP));
        assert_eq!(c.cfm_factor.raw, 4047058867);
    }

    #[test]
    fn gas_cfm_coeff_upstream_literal() {
        let s = opaque(SpringCoefficientsTrait::contact_defaults());
        assert!(cfm_coeff_upstream_literal(s, opaque(SUBSTEP)).raw > 0);
    }

    #[test]
    fn gas_angular_frequency() {
        let s = opaque(SpringCoefficientsTrait::contact_defaults());
        assert_eq!(s.angular_frequency().raw, 809582262270);
    }

    #[test]
    fn gas_erp_inv_dt() {
        let s = opaque(SpringCoefficientsTrait::contact_defaults());
        assert_eq!(s.erp_inv_dt(opaque(SUBSTEP)).raw, 38949567192);
    }

    #[test]
    fn gas_cfm_coeff() {
        let s = opaque(SpringCoefficientsTrait::contact_defaults());
        assert_eq!(s.cfm_coeff(opaque(SUBSTEP)).raw, 263094417);
    }

    #[test]
    fn gas_joint_coefficients() {
        let s = opaque(SpringCoefficientsTrait::joint_defaults());
        assert_eq!(s.coefficients(opaque(SUBSTEP)).cfm_coeff.raw, 6);
    }
}
