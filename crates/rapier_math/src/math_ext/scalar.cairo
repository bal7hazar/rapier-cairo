//! Scalar helpers of Rapier and Parry that do not belong to the shared `fixed` scalar.
//!
//! These are the `utils::` free functions of upstream: the safe reciprocal every inverse mass
//! goes through, the sign-copy used to pick support points, the magnitude clamp of the contact
//! solver and the component selection of the support functions.

use fixed::{Fixed, FixedTrait, ZERO};

/// Returns `1 / x`, or `0` when `x` is zero.
///
/// Mirrors `parry::utils::inv` and `rapier::utils::inv`. Rapier's version returns 0 over the
/// whole interval `[-1e-20, 1e-20]`; in Q32.32 that interval contains no representable value but
/// zero (1 ulp is `2.3e-10`, twelve orders of magnitude above `1e-20`), so the exact zero test is
/// a faithful port of both. This is what gives an infinite-mass body an inverse mass of 0.
/// #### Panics
/// * `'Fixed: overflow'` if `|x| < 2^-31`, where the reciprocal leaves the scalar range.
/// #### Deviations
/// * Upstream returns an infinity instead of panicking for an `x` that small.
#[inline(always)]
pub fn inv(x: Fixed) -> Fixed {
    if x.raw == 0 {
        ZERO
    } else {
        x.recip()
    }
}

/// Returns `to` with the sign of `sign`.
///
/// Mirrors `rapier::utils::CopySign::copy_sign_to` (`sign.copy_sign_to(to)`), i.e. `f32::copysign`
/// with the arguments in upstream's order. There is no negative zero in Q32.32, so a zero `sign`
/// counts as positive — upstream's float version would propagate `-0.0` here, which it only ever
/// does for `copy_sign_to(1.0)` on an exact `-0.0`.
/// #### Panics
/// * `'Fixed: overflow'` if `to` is `fixed::MIN` and `sign` is not negative.
/// #### Deviations
/// * Signed zero: see above.
#[inline(always)]
pub fn copy_sign_to(sign: Fixed, to: Fixed) -> Fixed {
    to.copysign(sign)
}

/// Clamps `x` to `[-max, max]`, keeping its sign.
///
/// The scalar case of `nalgebra`'s `cap_magnitude`, which the contact solver applies to the
/// tangent impulse (`new_impulse.cap_magnitude(limit)`).
/// #### Panics
/// * `'i64_neg Overflow'` if `max` is `fixed::MIN`.
/// #### Deviations
/// * A negative `max` gives an empty interval; upstream assumes `max >= 0` as well.
#[inline(always)]
pub fn cap_magnitude(x: Fixed, max: Fixed) -> Fixed {
    x.clamp(-max, max)
}

/// Returns the component of `(x, y)` with the smallest absolute value, `x` on ties.
///
/// The 2D case of the component selection used by the cuboid support function
/// (`local_dir.abs().min_position()`) and by upstream's 3D orthonormal bases. Works over the
/// whole scalar range: the magnitudes are compared as `i128`, so `fixed::MIN` is handled like
/// any other value (`Fixed::abs` would panic on it).
/// #### Panics
/// * Never.
/// #### Deviations
/// * None.
#[inline(always)]
pub fn smallest_abs_component(x: Fixed, y: Fixed) -> Fixed {
    if abs_raw(x) <= abs_raw(y) {
        x
    } else {
        y
    }
}

/// Returns the index (`0` or `1`) of the component of `(x, y)` with the smallest absolute value,
/// `0` on ties.
///
/// Mirrors `glam::Vec2::min_position` applied to `v.abs()` (see [`smallest_abs_component`]).
/// #### Panics
/// * Never.
/// #### Deviations
/// * None.
#[inline(always)]
pub fn smallest_abs_component_index(x: Fixed, y: Fixed) -> u8 {
    if abs_raw(x) <= abs_raw(y) {
        0
    } else {
        1
    }
}

/// `|x.raw|` as an `i128`: defined for `fixed::MIN`, unlike `Fixed::abs`.
#[inline(always)]
fn abs_raw(x: Fixed) -> i128 {
    let raw: i128 = x.raw.into();
    if raw < 0 {
        -raw
    } else {
        raw
    }
}

/// Rejected candidates, kept for the `gas_*` ranking.
#[cfg(test)]
mod alternatives {
    use fixed::{Fixed, FixedTrait, ZERO};

    /// `cap_magnitude` written the way upstream's float version reads: compare the magnitude,
    /// then copy the sign back. Panics on `fixed::MIN` (`abs` overflows).
    pub fn cap_magnitude_copysign(x: Fixed, max: Fixed) -> Fixed {
        if x.abs() > max {
            max.copysign(x)
        } else {
            x
        }
    }

    /// `smallest_abs_component` through `Fixed::abs`: one comparison less, but it panics for
    /// `fixed::MIN`.
    pub fn smallest_abs_component_abs(x: Fixed, y: Fixed) -> Fixed {
        if x.abs() <= y.abs() {
            x
        } else {
            y
        }
    }

    /// `inv` through a `Fixed` comparison instead of a raw one.
    pub fn inv_is_zero(x: Fixed) -> Fixed {
        if x == ZERO {
            ZERO
        } else {
            x.recip()
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, MAX, MIN, NEG_ONE, ONE, TWO, ZERO};
    use rapier_testing::opaque;
    use super::alternatives::{cap_magnitude_copysign, inv_is_zero, smallest_abs_component_abs};
    use super::{
        cap_magnitude, copy_sign_to, inv, smallest_abs_component, smallest_abs_component_index,
    };

    // ---------------------------------------------------------------- inv

    #[test]
    fn test_inv_of_zero_is_zero() {
        assert_eq!(inv(ZERO), ZERO);
        assert_eq!(inv(ZERO), inv_is_zero(ZERO));
    }

    #[test]
    fn test_inv_round_trip() {
        assert_eq!(inv(ONE), ONE);
        assert_eq!(inv(TWO), HALF);
        assert_eq!(inv(HALF), TWO);
        assert_eq!(inv(NEG_ONE), NEG_ONE);
        assert_eq!(inv(FixedTrait::from_int(4)), FixedTrait::from_ratio(1, 4));
        // Reciprocal division rounds to nearest, ties to even.
        assert_eq!(inv(FixedTrait::from_int(3)), FixedTrait::from_ratio(1, 3));
        assert_eq!(inv(FixedTrait::from_int(-3)), FixedTrait::from_ratio(-1, 3));
    }

    /// The reciprocal of a value below `2^-31` leaves the scalar range.
    #[test]
    #[should_panic(expected: 'Fixed: overflow')]
    fn test_inv_of_one_ulp_panics() {
        inv(opaque(FixedTrait::from_raw(1)));
    }

    // ---------------------------------------------------------------- copy_sign_to

    #[test]
    fn test_copy_sign_to() {
        assert_eq!(copy_sign_to(NEG_ONE, ONE), NEG_ONE);
        assert_eq!(copy_sign_to(ONE, NEG_ONE), ONE);
        assert_eq!(copy_sign_to(ZERO, NEG_ONE), ONE);
        assert_eq!(copy_sign_to(NEG_ONE, ZERO), ZERO);
        assert_eq!(
            copy_sign_to(FixedTrait::from_int(-5), FixedTrait::from_int(3)),
            FixedTrait::from_int(-3),
        );
        // The upstream idiom: turn a value into the sign it carries.
        assert_eq!(copy_sign_to(FixedTrait::from_int(-7), ONE), NEG_ONE);
        assert_eq!(copy_sign_to(FixedTrait::from_int(7), ONE), ONE);
    }

    // ---------------------------------------------------------------- cap_magnitude

    #[test]
    fn test_cap_magnitude() {
        let three: Fixed = FixedTrait::from_int(3);
        assert_eq!(cap_magnitude(three, TWO), TWO);
        assert_eq!(cap_magnitude(-three, TWO), -TWO);
        assert_eq!(cap_magnitude(ONE, TWO), ONE);
        assert_eq!(cap_magnitude(-ONE, TWO), -ONE);
        assert_eq!(cap_magnitude(ZERO, TWO), ZERO);
        assert_eq!(cap_magnitude(three, ZERO), ZERO);
        // Exactly at the cap.
        assert_eq!(cap_magnitude(TWO, TWO), TWO);
        assert_eq!(cap_magnitude(-TWO, TWO), -TWO);
        // Both candidates agree away from `MIN`.
        assert_eq!(cap_magnitude(three, TWO), cap_magnitude_copysign(three, TWO));
        assert_eq!(cap_magnitude(-three, TWO), cap_magnitude_copysign(-three, TWO));
        assert_eq!(cap_magnitude(MAX, TWO), cap_magnitude_copysign(MAX, TWO));
    }

    /// The upstream-shaped candidate cannot cap the smallest value of the range.
    #[test]
    #[should_panic(expected: 'Fixed: overflow')]
    fn test_cap_magnitude_copysign_panics_on_min() {
        cap_magnitude_copysign(opaque(MIN), opaque(TWO));
    }

    #[test]
    fn test_cap_magnitude_of_min() {
        assert_eq!(cap_magnitude(MIN, TWO), -TWO);
    }

    // ---------------------------------------------------------------- smallest_abs_component

    #[test]
    fn test_smallest_abs_component() {
        let three: Fixed = FixedTrait::from_int(3);
        let four: Fixed = FixedTrait::from_int(4);
        assert_eq!(smallest_abs_component(three, four), three);
        assert_eq!(smallest_abs_component(four, three), three);
        assert_eq!(smallest_abs_component(-three, four), -three);
        assert_eq!(smallest_abs_component(four, -three), -three);
        // Ties keep the first component.
        assert_eq!(smallest_abs_component(three, -three), three);
        assert_eq!(smallest_abs_component(ZERO, ZERO), ZERO);
        // Indices follow.
        assert_eq!(smallest_abs_component_index(three, four), 0);
        assert_eq!(smallest_abs_component_index(four, three), 1);
        assert_eq!(smallest_abs_component_index(three, -three), 0);
    }

    /// The extremes of the range: `MIN` has the largest magnitude of all, and the `abs`
    /// candidate cannot even look at it.
    #[test]
    fn test_smallest_abs_component_at_min() {
        assert_eq!(smallest_abs_component(MIN, ONE), ONE);
        assert_eq!(smallest_abs_component(ONE, MIN), ONE);
        assert_eq!(smallest_abs_component(MIN, MAX), MAX);
        assert_eq!(smallest_abs_component_index(MIN, ONE), 1);
    }

    #[test]
    #[should_panic(expected: 'Fixed: overflow')]
    fn test_smallest_abs_component_abs_panics_on_min() {
        smallest_abs_component_abs(opaque(MIN), opaque(ONE));
    }

    // ---------------------------------------------------------------- fuzz

    const LCG_MUL: u128 = 6364136223846793005;
    const LCG_INC: u128 = 1442695040888963407;
    const NZ_TWO_POW_64: NonZero<u128> = 0x10000000000000000;
    const NZ_TWO_POW_32: NonZero<u128> = 0x100000000;
    const NZ_TWO_POW_16: NonZero<u64> = 0x10000;

    fn next(ref state: u64) -> u32 {
        let (_, low) = DivRem::div_rem(state.into() * LCG_MUL + LCG_INC, NZ_TWO_POW_64);
        state = low.try_into().unwrap();
        let (high, _) = DivRem::div_rem(low, NZ_TWO_POW_32);
        high.try_into().unwrap()
    }

    /// Draws a raw value in `[-2^47, 2^47)` (values in `[-32768, 32768)`), away from the two
    /// extremes of the range where the candidates stop agreeing.
    fn draw(ref state: u64) -> Fixed {
        let high: u64 = next(ref state).into();
        let low: u64 = next(ref state).into();
        let (_, low) = DivRem::div_rem(low, NZ_TWO_POW_16);
        let raw: i64 = (high * 65536 + low).try_into().unwrap();
        Fixed { raw: raw - 140737488355328 } // 2^47
    }

    /// Away from `fixed::MIN`, both `cap_magnitude` and both `smallest_abs_component`
    /// candidates are the same function.
    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_candidates_equivalent(seed: u64) {
        let mut state = seed | 1;
        let mut i: u32 = 0;
        while i != 16 {
            let x = draw(ref state);
            let y = draw(ref state);
            let max = draw(ref state).abs();
            assert_eq!(cap_magnitude(x, max), cap_magnitude_copysign(x, max));
            assert_eq!(smallest_abs_component(x, y), smallest_abs_component_abs(x, y));
            // `recip` is only defined above 2^-31; both `inv` candidates agree where it is.
            if x.abs() >= ONE {
                assert_eq!(inv(x), inv_is_zero(x));
            }
            // `cap_magnitude` never grows a value and never flips its sign.
            let capped = cap_magnitude(x, max);
            assert!(capped.abs() <= x.abs());
            assert!(capped.abs() <= max);
            assert_eq!(capped.is_negative(), x.is_negative() && capped != ZERO);
            i += 1;
        }
    }

    // ---------------------------------------------------------------- gas

    /// Empty probe: the harness overhead to subtract from the entries below.
    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_inv() {
        assert_eq!(inv(opaque(TWO)), HALF);
    }

    #[test]
    fn gas_inv_is_zero() {
        assert_eq!(inv_is_zero(opaque(TWO)), HALF);
    }

    #[test]
    fn gas_copy_sign_to() {
        assert_eq!(copy_sign_to(opaque(NEG_ONE), opaque(TWO)), -TWO);
    }

    #[test]
    fn gas_cap_magnitude() {
        assert_eq!(cap_magnitude(opaque(TWO), opaque(ONE)), ONE);
    }

    #[test]
    fn gas_cap_magnitude_copysign() {
        assert_eq!(cap_magnitude_copysign(opaque(TWO), opaque(ONE)), ONE);
    }

    #[test]
    fn gas_smallest_abs_component() {
        assert_eq!(smallest_abs_component(opaque(TWO), opaque(ONE)), ONE);
    }

    #[test]
    fn gas_smallest_abs_component_abs() {
        assert_eq!(smallest_abs_component_abs(opaque(TWO), opaque(ONE)), ONE);
    }

    #[test]
    fn gas_smallest_abs_component_index() {
        assert_eq!(smallest_abs_component_index(opaque(TWO), opaque(ONE)), 1);
    }
}
