//! Damping factors of a rigid-body (upstream `RigidBodyDamping`) and the velocity multiplier
//! they induce.
//!
//! Upstream's `RigidBodyVelocity::apply_damping(dt, damping)` scales a velocity by
//! `1 / (1 + dt * damping)` (implicit Euler decay), separately for the linear and angular parts.
//! The vector multiplication belongs to the crates owning `Vec2`; this module provides the
//! factor.
//!
//! Rounding: `dt * damping` floors (`Fixed` product) and the reciprocal truncates toward zero
//! (`Fixed` quotient), so the factor is never above the exact value, by at most 1 ULP (`2^-32`).
//! `damping == 0` gives exactly `1`.
//!
//! Candidates (ranked by the `gas_*` probes):
//!
//! 1. `ONE / (ONE + dt * d)` — the upstream shape, one `Fixed` division;
//! 2. `alternatives::factor_recip` — `(ONE + dt * d).recip()`, the sign-split-free reciprocal;
//! 3. `alternatives::factor_skip_zero` — winner plus an early return of `ONE` when `d == 0`.

use fixed::{Fixed, ONE, ZERO};

/// The velocity multiplier `1 / (1 + dt * damping)`.
///
/// # Arguments
/// * `dt` — step length in seconds, `>= 0`.
/// * `damping` — damping coefficient in `1/s`, `>= 0` (upstream default `0`).
///
/// # Returns
/// A factor in `(0, 1]`; e.g. `dt = 1/60`, `damping = 0.5` gives `0.99174…`.
///
/// # Panics
/// * `'Fixed: overflow'` / `'i64_add Overflow'` if `dt * damping` (or `1 + dt * damping`)
///   exceeds the Q32.32 range, i.e. `dt * damping >= 2^31 - 1`.
/// * `'Fixed: division by zero'` if `1 + dt * damping == 0` (only for a negative `damping`).
#[inline(always)]
pub fn damping_factor(dt: Fixed, damping: Fixed) -> Fixed {
    ONE / (ONE + dt * damping)
}

/// Damping factors to progressively slow down a rigid-body. Default: no damping.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RigidBodyDamping {
    /// Damping of the translational motion, in `1/s`, `>= 0`.
    pub linear_damping: Fixed,
    /// Damping of the angular motion, in `1/s`, `>= 0`.
    pub angular_damping: Fixed,
}

/// Upstream default: both dampings are zero.
pub impl RigidBodyDampingDefault of Default<RigidBodyDamping> {
    #[inline(always)]
    fn default() -> RigidBodyDamping {
        RigidBodyDamping { linear_damping: ZERO, angular_damping: ZERO }
    }
}

/// Damping multipliers of [`RigidBodyDamping`].
#[generate_trait]
pub impl RigidBodyDampingImpl of RigidBodyDampingTrait {
    /// Multiplier of the linear velocity over `dt`, see [`damping_factor`].
    #[inline(always)]
    fn linear_factor(self: RigidBodyDamping, dt: Fixed) -> Fixed {
        damping_factor(dt, self.linear_damping)
    }

    /// Multiplier of the angular velocity over `dt`, see [`damping_factor`].
    #[inline(always)]
    fn angular_factor(self: RigidBodyDamping, dt: Fixed) -> Fixed {
        damping_factor(dt, self.angular_damping)
    }
}

#[cfg(test)]
mod alternatives {
    use fixed::{Fixed, FixedTrait, ONE, ZERO};

    pub fn factor_recip(dt: Fixed, damping: Fixed) -> Fixed {
        (ONE + dt * damping).recip()
    }

    pub fn factor_skip_zero(dt: Fixed, damping: Fixed) -> Fixed {
        if damping == ZERO {
            ONE
        } else {
            ONE / (ONE + dt * damping)
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
    use rapier_testing::opaque;
    use super::alternatives::{factor_recip, factor_skip_zero};
    use super::{RigidBodyDamping, RigidBodyDampingTrait, damping_factor};

    /// `1 / 60` rounded to nearest, as `IntegrationParameters::default().dt`.
    const DT: Fixed = Fixed { raw: 71582788 };

    /// Expected raw factors, `floor(2^64 / (2^32 + floor(DT.raw * d.raw / 2^32)))`, computed
    /// with exact integers.
    fn assert_factor(damping: Fixed, expected_raw: i64) {
        let expected = Fixed { raw: expected_raw };
        assert_eq!(damping_factor(DT, damping), expected);
        assert_eq!(factor_recip(DT, damping), expected);
        assert_eq!(factor_skip_zero(DT, damping), expected);
    }

    #[test]
    fn test_default_is_zero() {
        let default: RigidBodyDamping = Default::default();
        assert_eq!(default, RigidBodyDamping { linear_damping: ZERO, angular_damping: ZERO });
        assert_eq!(default.linear_factor(DT), ONE);
        assert_eq!(default.angular_factor(DT), ONE);
    }

    #[test]
    fn test_factor_zero_damping() {
        // d = 0: 1 / 1 = exactly 2^32.
        assert_factor(ZERO, 4294967296);
    }

    #[test]
    fn test_factor_half() {
        // d = 0.5: dt * d = 71582788 * 2^31 / 2^32 = 35791394; 1 + it = 4330758690;
        // 2^64 / 4330758690 = 4259471698.4 -> 4259471698 (0.99174...).
        assert_factor(HALF, 4259471698);
    }

    #[test]
    fn test_factor_one() {
        // d = 1: dt * d = 71582788; 1 + it = 4366550084; 2^64 / 4366550084 = 4224557996.8
        // -> 4224557996 (0.98360...).
        assert_factor(ONE, 4224557996);
    }

    #[test]
    fn test_factor_hundred() {
        // d = 100: dt * d = 7158278800; 1 + it = 11453246096; 2^64 / 11453246096 = 1610612739.9
        // -> 1610612739 (0.37500...).
        assert_factor(FixedTrait::from_int(100), 1610612739);
    }

    #[test]
    fn test_factor_is_monotonic_and_bounded() {
        let mut previous = damping_factor(DT, ZERO);
        let mut d = 1;
        while d != 50 {
            let factor = damping_factor(DT, FixedTrait::from_int(d));
            assert!(factor < previous);
            assert!(factor > ZERO);
            previous = factor;
            d += 1;
        }
    }

    #[test]
    fn test_linear_and_angular_are_independent() {
        let damping = RigidBodyDamping { linear_damping: ONE, angular_damping: HALF };
        assert_eq!(damping.linear_factor(DT), Fixed { raw: 4224557996 });
        assert_eq!(damping.angular_factor(DT), Fixed { raw: 4259471698 });
    }

    #[test]
    fn test_zero_dt_does_not_damp() {
        assert_eq!(damping_factor(ZERO, FixedTrait::from_int(100)), ONE);
    }

    #[test]
    #[should_panic(expected: 'Fixed: overflow')]
    fn test_overflow_panics() {
        // 100000 * 100000 = 1e10 exceeds 2^31.
        damping_factor(FixedTrait::from_int(100000), FixedTrait::from_int(100000));
    }

    #[test]
    fn gas_baseline() {}

    /// Cost of building the probe inputs alone.
    #[test]
    fn gas_inputs() {
        assert!(opaque(DT) != opaque(ZERO));
    }

    #[test]
    fn gas_damping_factor_zero() {
        assert!(damping_factor(opaque(DT), opaque(ZERO)) == ONE);
    }

    #[test]
    fn gas_damping_factor_one() {
        assert!(damping_factor(opaque(DT), opaque(ONE)) < ONE);
    }

    #[test]
    fn gas_factor_recip_zero() {
        assert!(factor_recip(opaque(DT), opaque(ZERO)) == ONE);
    }

    #[test]
    fn gas_factor_recip_one() {
        assert!(factor_recip(opaque(DT), opaque(ONE)) < ONE);
    }

    #[test]
    fn gas_factor_skip_zero_zero() {
        assert!(factor_skip_zero(opaque(DT), opaque(ZERO)) == ONE);
    }

    #[test]
    fn gas_factor_skip_zero_one() {
        assert!(factor_skip_zero(opaque(DT), opaque(ONE)) < ONE);
    }

    #[test]
    fn gas_linear_and_angular_factor() {
        let damping = opaque(RigidBodyDamping { linear_damping: ONE, angular_damping: HALF });
        assert!(damping.linear_factor(opaque(DT)) < damping.angular_factor(opaque(DT)));
    }
}
