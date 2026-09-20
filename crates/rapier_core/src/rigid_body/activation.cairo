//! Sleeping state of a rigid-body (upstream `RigidBodyActivation`).
//!
//! A body falls asleep once it stayed below the motion thresholds for `time_until_sleep`
//! seconds. [`RigidBodyActivationTrait::update_energy`] is upstream's `update_energy` split at the
//! vector boundary: upstream reads the body's pose to measure the drift since the previous
//! step (`relative_pose_drift`) and remembers it (`sleep_prev_pose`); both stay with the caller,
//! which passes the resulting scalar `drift` in. Every comparison is upstream's, in the same
//! order, on `Fixed` values.
//!
//! * A negative threshold means "never sleeps": `cannot_sleep()` sets both thresholds to `-1`.
//!   For a dynamic body the linear gate becomes `drift / 2 < negative`, which never holds; the
//!   angular gate becomes `sq_angvel < threshold * |threshold|` (a negative number, never
//!   holds) or `threshold >= 0` (false).
//! * Rounding: `*` floors (`Fixed` product), so thresholds squared and `normalized threshold *
//!   length_unit * dt` are rounded down by at most 1 ULP (`2^-32`); halving is exact for the
//!   non-negative drift.
//!
//! Cut: upstream's `SoftFrame` branch (soft bodies).
//!
//! Candidates (ranked by the `gas_*` probes):
//!
//! * `drift / 2`: `drift * HALF` (upstream's `drift * 0.5`) **(winner)** against a raw `DivRem` by
//!   2 (`alternatives::dynamic_gate_half_divrem`, about 3.9k dearer: the `Fixed` product is a
//!   single wide multiplication, the `DivRem` of an `i64` costs more);
//! * dispatch on the body type: a `return` per arm (`alternatives::can_sleep_early`) costs exactly
//!   as much as the joined `match`; a non-inlined call pays for its dearest arm (the dynamic
//!   one), whatever the body type.

use fixed::{Fixed, FixedTrait, HALF, NEG_ONE, ZERO};
use super::body_type::RigidBodyType;

/// Default `normalized_linear_threshold`: `0.05` length units per second, rounded to nearest.
pub const DEFAULT_NORMALIZED_LINEAR_THRESHOLD: Fixed = Fixed { raw: 214748365 };
/// Default `angular_threshold`: `0.5` rad/s.
pub const DEFAULT_ANGULAR_THRESHOLD: Fixed = HALF;
/// Default `time_until_sleep`: half a second.
pub const DEFAULT_TIME_UNTIL_SLEEP: Fixed = HALF;

/// `(pi / 2)^2` = `floor(FRAC_PI_2 * FRAC_PI_2)`: the squared angular speed above which a
/// dynamic body with colliders is considered moving.
const SQ_FRAC_PI_2: Fixed = Fixed { raw: 10597407030 };

/// When a body goes to sleep, and whether it currently sleeps.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RigidBodyActivation {
    /// Speed threshold for sleeping, before `length_unit` scaling, in length units per second.
    /// Compared against the farthest-point displacement rate of the body. Negative: never
    /// sleeps. Default `0.05`.
    pub normalized_linear_threshold: Fixed,
    /// Angular speed threshold for sleeping, in rad/s. Only used for collider-less bodies
    /// (`max_extent == 0`); with colliders, a fixed `pi / 2` rad/s is used. Negative: never
    /// sleeps. Default `0.5`.
    pub angular_threshold: Fixed,
    /// Seconds the body must stay below the thresholds before sleeping. Default `0.5`.
    pub time_until_sleep: Fixed,
    /// Seconds the body has been below the thresholds so far.
    pub time_since_can_sleep: Fixed,
    /// Is the body currently asleep?
    pub sleeping: bool,
}

/// Upstream default: an active body with the default thresholds.
pub impl RigidBodyActivationDefault of Default<RigidBodyActivation> {
    #[inline(always)]
    fn default() -> RigidBodyActivation {
        RigidBodyActivationImpl::active()
    }
}

/// Constructors, state transitions and the per-step timer of [`RigidBodyActivation`].
#[generate_trait]
pub impl RigidBodyActivationImpl of RigidBodyActivationTrait {
    /// An active body with the default thresholds and a reset timer.
    #[inline(always)]
    fn active() -> RigidBodyActivation {
        RigidBodyActivation {
            normalized_linear_threshold: DEFAULT_NORMALIZED_LINEAR_THRESHOLD,
            angular_threshold: DEFAULT_ANGULAR_THRESHOLD,
            time_until_sleep: DEFAULT_TIME_UNTIL_SLEEP,
            time_since_can_sleep: ZERO,
            sleeping: false,
        }
    }

    /// A sleeping body with the default thresholds and a full timer.
    #[inline(always)]
    fn inactive() -> RigidBodyActivation {
        RigidBodyActivation {
            normalized_linear_threshold: DEFAULT_NORMALIZED_LINEAR_THRESHOLD,
            angular_threshold: DEFAULT_ANGULAR_THRESHOLD,
            time_until_sleep: DEFAULT_TIME_UNTIL_SLEEP,
            time_since_can_sleep: DEFAULT_TIME_UNTIL_SLEEP,
            sleeping: true,
        }
    }

    /// An active body that can never sleep (both thresholds are `-1`).
    #[inline(always)]
    fn cannot_sleep() -> RigidBodyActivation {
        RigidBodyActivation {
            normalized_linear_threshold: NEG_ONE, angular_threshold: NEG_ONE, ..Self::active(),
        }
    }

    /// Returns `true` if the body is not asleep.
    #[inline(always)]
    fn is_active(self: RigidBodyActivation) -> bool {
        !self.sleeping
    }

    /// Wakes the body up; a `strong` wake-up also resets the still-time counter.
    #[inline(always)]
    fn wake_up(ref self: RigidBodyActivation, strong: bool) {
        self.sleeping = false;
        if strong {
            self.time_since_can_sleep = ZERO;
        }
    }

    /// Puts the body to sleep and fills the still-time counter.
    #[inline(always)]
    fn sleep(ref self: RigidBodyActivation) {
        self.sleeping = true;
        self.time_since_can_sleep = self.time_until_sleep;
    }

    /// Has the body been still long enough (`time_since_can_sleep >= time_until_sleep`)?
    #[inline(always)]
    fn is_eligible_for_sleep(self: RigidBodyActivation) -> bool {
        self.time_since_can_sleep >= self.time_until_sleep
    }

    /// The still-time counter step: adds `dt` when `can_sleep`, resets it otherwise.
    ///
    /// # Panics
    /// * `'i64_add Overflow'` if the counter would exceed the Q32.32 range (about 68 years).
    #[inline(always)]
    fn update_timer(ref self: RigidBodyActivation, can_sleep: bool, dt: Fixed) {
        if can_sleep {
            self.time_since_can_sleep += dt;
        } else {
            self.time_since_can_sleep = ZERO;
        }
    }

    /// The gate of a dynamic body: the angular gate and `drift / 2 < normalized_linear_threshold *
    /// length_unit * dt`. See [`can_sleep`](RigidBodyActivationTrait::can_sleep).
    #[inline(always)]
    fn dynamic_gate(
        self: RigidBodyActivation,
        length_unit: Fixed,
        sq_angvel: Fixed,
        max_extent: Fixed,
        drift: Fixed,
        dt: Fixed,
    ) -> bool {
        let linear_threshold = self.normalized_linear_threshold * length_unit;
        let angular_ok = if max_extent > ZERO {
            self.angular_threshold >= ZERO && sq_angvel < SQ_FRAC_PI_2
        } else {
            sq_angvel < self.angular_threshold * self.angular_threshold.abs()
        };
        angular_ok && drift * HALF < linear_threshold * dt
    }

    /// Does the body pass the motion gates of its type for this step?
    ///
    /// * `Dynamic`: the angular gate (`sq_angvel < (pi / 2)^2` with colliders, `sq_angvel <
    ///   angular_threshold * |angular_threshold|` without) and `drift / 2 <
    ///   normalized_linear_threshold * length_unit * dt`;
    /// * kinematic: both squared velocities exactly zero;
    /// * `Fixed`: always.
    ///
    /// # Arguments
    /// * `body_type` — the body type.
    /// * `length_unit` — world scale, see `IntegrationParameters::length_unit`.
    /// * `sq_linvel`, `sq_angvel` — squared linear / angular speed of the body.
    /// * `max_extent` — distance from the center of mass to the farthest collider point, `0`
    ///   for a body without colliders.
    /// * `drift` — the farthest-point displacement since the previous step
    ///   (`relative_pose_drift` upstream).
    /// * `dt` — step length.
    ///
    /// # Panics
    /// * `'Fixed: overflow'` if a product leaves the Q32.32 range.
    fn can_sleep(
        self: RigidBodyActivation,
        body_type: RigidBodyType,
        length_unit: Fixed,
        sq_linvel: Fixed,
        sq_angvel: Fixed,
        max_extent: Fixed,
        drift: Fixed,
        dt: Fixed,
    ) -> bool {
        match body_type {
            RigidBodyType::Dynamic => self
                .dynamic_gate(length_unit, sq_angvel, max_extent, drift, dt),
            RigidBodyType::KinematicPositionBased |
            RigidBodyType::KinematicVelocityBased => sq_linvel == ZERO && sq_angvel == ZERO,
            RigidBodyType::Fixed => true,
        }
    }

    /// Upstream's `update_energy`: advances the still-time counter of an awake body.
    ///
    /// A sleeping body is pinned eligible (`time_since_can_sleep = time_until_sleep`) and the
    /// gates are not evaluated. Otherwise the counter grows by `dt` when
    /// [`can_sleep`](RigidBodyActivationTrait::can_sleep) holds and resets to zero when it does
    /// not. Arguments as for `can_sleep`.
    fn update_energy(
        ref self: RigidBodyActivation,
        body_type: RigidBodyType,
        length_unit: Fixed,
        sq_linvel: Fixed,
        sq_angvel: Fixed,
        max_extent: Fixed,
        drift: Fixed,
        dt: Fixed,
    ) {
        if self.sleeping {
            self.time_since_can_sleep = self.time_until_sleep;
            return;
        }
        let can_sleep = self
            .can_sleep(body_type, length_unit, sq_linvel, sq_angvel, max_extent, drift, dt);
        self.update_timer(can_sleep, dt);
    }
}

#[cfg(test)]
mod alternatives {
    use fixed::{Fixed, FixedTrait, ZERO};
    use crate::rigid_body::body_type::RigidBodyType;
    use super::{RigidBodyActivation, RigidBodyActivationTrait, SQ_FRAC_PI_2};

    const NZ_TWO: NonZero<i64> = 2;

    /// `can_sleep` with a `return` in every arm: no join after the `match`, so each body type
    /// pays only its own arm instead of the most expensive one.
    pub fn can_sleep_early(
        self: RigidBodyActivation,
        body_type: RigidBodyType,
        length_unit: Fixed,
        sq_linvel: Fixed,
        sq_angvel: Fixed,
        max_extent: Fixed,
        drift: Fixed,
        dt: Fixed,
    ) -> bool {
        match body_type {
            RigidBodyType::Dynamic => {
                return self.dynamic_gate(length_unit, sq_angvel, max_extent, drift, dt);
            },
            RigidBodyType::KinematicPositionBased |
            RigidBodyType::KinematicVelocityBased => {
                return sq_linvel == ZERO && sq_angvel == ZERO;
            },
            RigidBodyType::Fixed => { return true; },
        }
    }

    /// `dynamic_gate` with `drift / 2` computed as a raw `DivRem` by two
    /// instead of `drift * HALF`. Exact for a non-negative drift, like the winner.
    pub fn dynamic_gate_half_divrem(
        self: RigidBodyActivation,
        length_unit: Fixed,
        sq_angvel: Fixed,
        max_extent: Fixed,
        drift: Fixed,
        dt: Fixed,
    ) -> bool {
        let linear_threshold = self.normalized_linear_threshold * length_unit;
        let angular_ok = if max_extent > ZERO {
            self.angular_threshold >= ZERO && sq_angvel < SQ_FRAC_PI_2
        } else {
            sq_angvel < self.angular_threshold * self.angular_threshold.abs()
        };
        let (half_drift, _) = DivRem::div_rem(drift.raw, NZ_TWO);
        angular_ok && Fixed { raw: half_drift } < linear_threshold * dt
    }
}

#[cfg(test)]
mod tests {
    use fixed::{FRAC_PI_2, Fixed, FixedTrait, HALF, NEG_ONE, ONE, ZERO};
    use rapier_testing::opaque;
    use super::alternatives::{can_sleep_early, dynamic_gate_half_divrem};
    use super::super::body_type::RigidBodyType;
    use super::{
        DEFAULT_ANGULAR_THRESHOLD, DEFAULT_NORMALIZED_LINEAR_THRESHOLD, DEFAULT_TIME_UNTIL_SLEEP,
        RigidBodyActivation, RigidBodyActivationTrait, SQ_FRAC_PI_2,
    };

    /// `1 / 60` rounded to nearest, as `IntegrationParameters::default().dt`.
    const DT: Fixed = Fixed { raw: 71582788 };
    const D: RigidBodyType = RigidBodyType::Dynamic;
    const F: RigidBodyType = RigidBodyType::Fixed;
    const KP: RigidBodyType = RigidBodyType::KinematicPositionBased;
    const KV: RigidBodyType = RigidBodyType::KinematicVelocityBased;

    /// `can_sleep` of a default body with zero velocities and no collider.
    fn still(body_type: RigidBodyType, drift: Fixed) -> bool {
        RigidBodyActivationTrait::active().can_sleep(body_type, ONE, ZERO, ZERO, ZERO, drift, DT)
    }

    #[test]
    fn test_default_constants() {
        // 0.05 rounded to nearest: 0.05 * 2^32 = 214748364.8.
        assert_eq!(DEFAULT_NORMALIZED_LINEAR_THRESHOLD.raw, 214748365);
        assert_eq!(DEFAULT_ANGULAR_THRESHOLD, HALF);
        assert_eq!(DEFAULT_TIME_UNTIL_SLEEP, HALF);
        // (pi / 2)^2 is the floored Fixed product.
        assert_eq!(SQ_FRAC_PI_2, FRAC_PI_2 * FRAC_PI_2);
    }

    #[test]
    fn test_constructors() {
        let active = RigidBodyActivationTrait::active();
        assert_eq!(
            active,
            RigidBodyActivation {
                normalized_linear_threshold: DEFAULT_NORMALIZED_LINEAR_THRESHOLD,
                angular_threshold: HALF,
                time_until_sleep: HALF,
                time_since_can_sleep: ZERO,
                sleeping: false,
            },
        );
        assert!(active.is_active());
        assert!(!active.is_eligible_for_sleep());
        let default: RigidBodyActivation = Default::default();
        assert_eq!(default, active);

        let inactive = RigidBodyActivationTrait::inactive();
        assert!(inactive.sleeping);
        assert!(!inactive.is_active());
        assert_eq!(inactive.time_since_can_sleep, HALF);
        assert!(inactive.is_eligible_for_sleep());
        assert_eq!(inactive.normalized_linear_threshold, DEFAULT_NORMALIZED_LINEAR_THRESHOLD);

        let cannot = RigidBodyActivationTrait::cannot_sleep();
        assert_eq!(cannot.normalized_linear_threshold, NEG_ONE);
        assert_eq!(cannot.angular_threshold, NEG_ONE);
        assert_eq!(cannot.time_until_sleep, HALF);
        assert_eq!(cannot.time_since_can_sleep, ZERO);
        assert!(!cannot.sleeping);
    }

    #[test]
    fn test_wake_up_and_sleep() {
        let mut activation = RigidBodyActivationTrait::active();
        activation.time_since_can_sleep = Fixed { raw: 1000 };

        activation.sleep();
        assert!(activation.sleeping);
        assert_eq!(activation.time_since_can_sleep, HALF);

        // A weak wake-up keeps the timer: the body is eligible again on the next check.
        activation.wake_up(false);
        assert!(!activation.sleeping);
        assert_eq!(activation.time_since_can_sleep, HALF);
        assert!(activation.is_eligible_for_sleep());

        // A strong wake-up resets it.
        activation.sleep();
        activation.wake_up(true);
        assert!(!activation.sleeping);
        assert_eq!(activation.time_since_can_sleep, ZERO);
    }

    #[test]
    fn test_update_timer() {
        let mut activation = RigidBodyActivationTrait::active();
        activation.update_timer(true, DT);
        activation.update_timer(true, DT);
        assert_eq!(activation.time_since_can_sleep.raw, 2 * 71582788);
        activation.update_timer(false, DT);
        assert_eq!(activation.time_since_can_sleep, ZERO);
    }

    #[test]
    fn test_eligible_at_exactly_the_delay() {
        let mut activation = RigidBodyActivationTrait::active();
        activation.time_since_can_sleep = Fixed { raw: HALF.raw - 1 };
        assert!(!activation.is_eligible_for_sleep());
        activation.time_since_can_sleep = HALF;
        assert!(activation.is_eligible_for_sleep());
    }

    /// A still dynamic body accumulates `dt` every step and becomes eligible after 31 steps
    /// (30 * 71582788 = 2147483640 < 2^31 <= 31 * 71582788).
    #[test]
    fn test_timer_accumulates_across_steps() {
        let mut activation = RigidBodyActivationTrait::active();
        let mut steps = 0;
        while !activation.is_eligible_for_sleep() {
            activation.update_energy(D, ONE, ZERO, ZERO, ONE, ZERO, DT);
            steps += 1;
            assert_eq!(activation.time_since_can_sleep.raw, steps * 71582788);
            assert!(steps <= 31);
        }
        assert_eq!(steps, 31);
        assert!(!activation.sleeping);
    }

    /// Any step in which the body moves resets the timer.
    #[test]
    fn test_moving_step_resets_timer() {
        let mut activation = RigidBodyActivationTrait::active();
        activation.update_energy(D, ONE, ZERO, ZERO, ONE, ZERO, DT);
        activation.update_energy(D, ONE, ZERO, ZERO, ONE, ZERO, DT);
        assert_eq!(activation.time_since_can_sleep.raw, 2 * 71582788);
        // Drift past the threshold.
        activation.update_energy(D, ONE, ZERO, ZERO, ONE, ONE, DT);
        assert_eq!(activation.time_since_can_sleep, ZERO);
        activation.update_energy(D, ONE, ZERO, ZERO, ONE, ZERO, DT);
        assert_eq!(activation.time_since_can_sleep, DT);
        // Angular speed past pi / 2.
        activation.update_energy(D, ONE, ZERO, FixedTrait::from_int(3), ONE, ZERO, DT);
        assert_eq!(activation.time_since_can_sleep, ZERO);
    }

    /// A sleeping body is pinned eligible whatever the gates say.
    #[test]
    fn test_sleeping_body_is_pinned() {
        let mut activation = RigidBodyActivationTrait::active();
        activation.sleep();
        activation.time_since_can_sleep = ZERO;
        activation.update_energy(D, ONE, ONE, ONE, ONE, ONE, DT);
        assert_eq!(activation.time_since_can_sleep, HALF);
        assert!(activation.sleeping);
    }

    /// Linear gate: `drift * 0.5 < 0.05 * length_unit * dt`, i.e. `3579139` raw
    /// (`floor(214748365 * 71582788 / 2^32)`): a raw drift of 7158277 passes (half = 3579138),
    /// 7158278 does not (half = 3579139).
    #[test]
    fn test_linear_gate_boundary() {
        assert!(still(D, Fixed { raw: 7158277 }));
        assert!(!still(D, Fixed { raw: 7158278 }));
        assert!(still(D, ZERO));
    }

    #[test]
    fn test_linear_gate_scales_with_length_unit() {
        let activation = RigidBodyActivationTrait::active();
        let drift = Fixed { raw: 10 * 7158277 };
        assert!(!activation.can_sleep(D, ONE, ZERO, ZERO, ZERO, drift, DT));
        // length_unit = 10 scales the threshold: 10 * 214748365 * dt.
        let ten = FixedTrait::from_int(10);
        assert!(activation.can_sleep(D, ten, ZERO, ZERO, ZERO, drift, DT));
    }

    /// With colliders (`max_extent > 0`) the angular gate is `sq_angvel < (pi / 2)^2`, whatever
    /// the (non-negative) `angular_threshold`.
    #[test]
    fn test_angular_gate_with_colliders() {
        let activation = RigidBodyActivationTrait::active();
        let below = Fixed { raw: SQ_FRAC_PI_2.raw - 1 };
        assert!(activation.can_sleep(D, ONE, ZERO, below, ONE, ZERO, DT));
        assert!(!activation.can_sleep(D, ONE, ZERO, SQ_FRAC_PI_2, ONE, ZERO, DT));
        let lax = RigidBodyActivation { angular_threshold: ZERO, ..activation };
        assert!(lax.can_sleep(D, ONE, ZERO, below, ONE, ZERO, DT));
    }

    /// Without colliders (`max_extent == 0`) the angular gate is
    /// `sq_angvel < angular_threshold * |angular_threshold|`, `0.25` by default.
    #[test]
    fn test_angular_gate_without_colliders() {
        let activation = RigidBodyActivationTrait::active();
        let quarter = Fixed { raw: 1073741824 };
        let below = Fixed { raw: quarter.raw - 1 };
        assert!(activation.can_sleep(D, ONE, ZERO, below, ZERO, ZERO, DT));
        assert!(!activation.can_sleep(D, ONE, ZERO, quarter, ZERO, ZERO, DT));
    }

    /// Negative thresholds never sleep, for either angular gate and for the linear gate, even
    /// when the body is perfectly still.
    #[test]
    fn test_negative_threshold_never_sleeps() {
        let cannot = RigidBodyActivationTrait::cannot_sleep();
        // Still, with colliders.
        assert!(!cannot.can_sleep(D, ONE, ZERO, ZERO, ONE, ZERO, DT));
        // Still, without colliders.
        assert!(!cannot.can_sleep(D, ONE, ZERO, ZERO, ZERO, ZERO, DT));
        // Only the linear threshold negative.
        let linear = RigidBodyActivation { angular_threshold: HALF, ..cannot };
        assert!(!linear.can_sleep(D, ONE, ZERO, ZERO, ONE, ZERO, DT));
        assert!(!linear.can_sleep(D, ONE, ZERO, ZERO, ZERO, ZERO, DT));
        // Only the angular threshold negative.
        let angular = RigidBodyActivation {
            normalized_linear_threshold: DEFAULT_NORMALIZED_LINEAR_THRESHOLD, ..cannot,
        };
        assert!(!angular.can_sleep(D, ONE, ZERO, ZERO, ONE, ZERO, DT));
        assert!(!angular.can_sleep(D, ONE, ZERO, ZERO, ZERO, ZERO, DT));
        // The timer never grows.
        let mut activation = cannot;
        activation.update_energy(D, ONE, ZERO, ZERO, ONE, ZERO, DT);
        activation.update_energy(D, ONE, ZERO, ZERO, ONE, ZERO, DT);
        assert_eq!(activation.time_since_can_sleep, ZERO);
    }

    /// Kinematic bodies sleep only with exactly zero velocities, whatever the thresholds.
    #[test]
    fn test_kinematic_gate() {
        let activation = RigidBodyActivationTrait::active();
        let eps = Fixed { raw: 1 };
        assert!(activation.can_sleep(KP, ONE, ZERO, ZERO, ONE, ONE, DT));
        assert!(activation.can_sleep(KV, ONE, ZERO, ZERO, ONE, ONE, DT));
        assert!(!activation.can_sleep(KP, ONE, eps, ZERO, ONE, ZERO, DT));
        assert!(!activation.can_sleep(KV, ONE, ZERO, eps, ONE, ZERO, DT));
        // As upstream, the thresholds are not consulted.
        let cannot = RigidBodyActivationTrait::cannot_sleep();
        assert!(cannot.can_sleep(KP, ONE, ZERO, ZERO, ZERO, ZERO, DT));
    }

    #[test]
    fn test_fixed_body_always_sleeps() {
        assert!(still(F, ZERO));
        assert!(still(F, ONE));
        let activation = RigidBodyActivationTrait::cannot_sleep();
        assert!(activation.can_sleep(F, ONE, ONE, ONE, ONE, ONE, DT));
    }

    #[test]
    fn test_can_sleep_early_agrees() {
        let activation = RigidBodyActivationTrait::active();
        let types = array![D, F, KP, KV];
        for t in types {
            let drifts = array![ZERO, Fixed { raw: 7158277 }, Fixed { raw: 7158278 }, ONE];
            for drift in drifts {
                let angvels = array![ZERO, Fixed { raw: 1073741824 }, SQ_FRAC_PI_2];
                for w in angvels {
                    for extent in array![ZERO, ONE] {
                        let a = activation.can_sleep(t, ONE, ZERO, w, extent, drift, DT);
                        assert_eq!(
                            can_sleep_early(activation, t, ONE, ZERO, w, extent, drift, DT), a,
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn test_half_divrem_candidate_agrees() {
        let activation = RigidBodyActivationTrait::active();
        let mut i = 0;
        while i != 10 {
            let drift = Fixed { raw: 7158270 + i };
            assert_eq!(
                dynamic_gate_half_divrem(activation, ONE, ZERO, ONE, drift, DT),
                activation.can_sleep(D, ONE, ZERO, ZERO, ONE, drift, DT),
            );
            i += 1;
        }
    }

    #[test]
    fn gas_baseline() {}

    /// Builds the probe inputs alone: subtract it from the `can_sleep` / `update_energy` probes.
    #[test]
    fn gas_inputs() {
        let activation = opaque(RigidBodyActivationTrait::active());
        let dt = opaque(DT);
        assert!(opaque(D) == D && opaque(ONE) != opaque(ZERO) && !activation.sleeping);
        assert!(dt != opaque(ZERO));
    }

    #[test]
    fn gas_constructors() {
        assert!(!opaque(RigidBodyActivationTrait::active()).sleeping);
        assert!(opaque(RigidBodyActivationTrait::inactive()).sleeping);
        assert!(!opaque(RigidBodyActivationTrait::cannot_sleep()).sleeping);
    }

    #[test]
    fn gas_wake_up_and_sleep() {
        let mut activation = opaque(RigidBodyActivationTrait::active());
        activation.sleep();
        activation.wake_up(opaque(true));
        assert!(!activation.sleeping);
    }

    #[test]
    fn gas_update_timer() {
        let mut activation = opaque(RigidBodyActivationTrait::active());
        activation.update_timer(opaque(true), opaque(DT));
        assert!(activation.time_since_can_sleep == DT);
    }

    #[test]
    fn gas_can_sleep_dynamic() {
        let activation = opaque(RigidBodyActivationTrait::active());
        let (t, l, o, e, dr, dt) = (
            opaque(D), opaque(ONE), opaque(ZERO), opaque(ONE), opaque(ZERO), opaque(DT),
        );
        assert!(activation.can_sleep(t, l, o, o, e, dr, dt));
    }

    #[test]
    fn gas_dynamic_gate() {
        let activation = opaque(RigidBodyActivationTrait::active());
        let (l, o, e, dt) = (opaque(ONE), opaque(ZERO), opaque(ONE), opaque(DT));
        assert!(activation.dynamic_gate(l, o, e, o, dt));
    }

    #[test]
    fn gas_dynamic_gate_half_divrem() {
        let activation = opaque(RigidBodyActivationTrait::active());
        let (l, o, e, dt) = (opaque(ONE), opaque(ZERO), opaque(ONE), opaque(DT));
        assert!(dynamic_gate_half_divrem(activation, l, o, e, o, dt));
    }

    #[test]
    fn gas_can_sleep_early_dynamic() {
        let activation = opaque(RigidBodyActivationTrait::active());
        let (t, l, o, e, dr, dt) = (
            opaque(D), opaque(ONE), opaque(ZERO), opaque(ONE), opaque(ZERO), opaque(DT),
        );
        assert!(can_sleep_early(activation, t, l, o, o, e, dr, dt));
    }

    #[test]
    fn gas_can_sleep_early_kinematic() {
        let activation = opaque(RigidBodyActivationTrait::active());
        let (t, l, o, e, dr, dt) = (
            opaque(KV), opaque(ONE), opaque(ZERO), opaque(ONE), opaque(ZERO), opaque(DT),
        );
        assert!(can_sleep_early(activation, t, l, o, o, e, dr, dt));
    }

    #[test]
    fn gas_can_sleep_early_fixed() {
        let activation = opaque(RigidBodyActivationTrait::active());
        let (t, l, o, e, dr, dt) = (
            opaque(F), opaque(ONE), opaque(ZERO), opaque(ONE), opaque(ZERO), opaque(DT),
        );
        assert!(can_sleep_early(activation, t, l, o, o, e, dr, dt));
    }

    #[test]
    fn gas_can_sleep_kinematic() {
        let activation = opaque(RigidBodyActivationTrait::active());
        let (t, l, o, e, dr, dt) = (
            opaque(KV), opaque(ONE), opaque(ZERO), opaque(ONE), opaque(ZERO), opaque(DT),
        );
        assert!(activation.can_sleep(t, l, o, o, e, dr, dt));
    }

    #[test]
    fn gas_can_sleep_fixed() {
        let activation = opaque(RigidBodyActivationTrait::active());
        let (t, l, o, e, dr, dt) = (
            opaque(F), opaque(ONE), opaque(ZERO), opaque(ONE), opaque(ZERO), opaque(DT),
        );
        assert!(activation.can_sleep(t, l, o, o, e, dr, dt));
    }

    #[test]
    fn gas_update_energy_awake() {
        let mut activation = opaque(RigidBodyActivationTrait::active());
        let (t, l, o, e, dr, dt) = (
            opaque(D), opaque(ONE), opaque(ZERO), opaque(ONE), opaque(ZERO), opaque(DT),
        );
        activation.update_energy(t, l, o, o, e, dr, dt);
        assert!(activation.time_since_can_sleep == DT);
    }

    #[test]
    fn gas_update_energy_sleeping() {
        let mut activation = opaque(RigidBodyActivationTrait::inactive());
        let (t, l, o, e, dr, dt) = (
            opaque(D), opaque(ONE), opaque(ZERO), opaque(ONE), opaque(ZERO), opaque(DT),
        );
        activation.update_energy(t, l, o, o, e, dr, dt);
        assert!(activation.sleeping);
    }
}
