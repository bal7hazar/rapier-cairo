//! The CCD members of [`RigidBody`] (upstream `dynamics/rigid_body.rs`: `enable_ccd`,
//! `is_ccd_enabled`, `is_ccd_active`, `set_soft_ccd_prediction`, `soft_ccd_prediction`), work
//! package CC2. The state is the body's cold [`RigidBodyCcd`]: a body that never enables CCD keeps
//! no cold data. As upstream, none of these setters raises a change flag or wakes the body.

use fixed::{Fixed, ZERO};
use crate::rigid_body::ccd::RigidBodyCcd;
use super::{ColdExtraSlotTrait, RigidBody, cold_or_default};

/// Panic messages of the CCD members.
pub mod errors {
    /// `set_soft_ccd_prediction` received a negative distance.
    pub const NEGATIVE_PREDICTION: felt252 = 'RigidBody: negative prediction';
}

/// The CCD component of `body` (upstream `RigidBody::ccd`, crate-private there): the default one
/// for a body without cold data.
#[inline(always)]
pub fn body_ccd(body: @RigidBody) -> RigidBodyCcd {
    match (*body.cold).unbox() {
        Some(cold) => cold.extra.value().ccd,
        None => Default::default(),
    }
}

/// Whether `body` asks for full CCD, without building a default component (the CCD pass's
/// discovery walk).
#[inline(always)]
pub fn wants_ccd(body: @RigidBody) -> bool {
    match (*body.cold).unbox() {
        Some(cold) => match cold.extra.get() {
            Some(extra) => extra.ccd.ccd_enabled,
            None => false,
        },
        None => false,
    }
}

#[generate_trait]
pub impl RigidBodyCcdApiImpl of RigidBodyCcdApiTrait {
    /// Enables or disables full ("bullet") CCD (upstream `enable_ccd`): a fast `ccd_enabled` body
    /// sweeps fixed, kinematic and dynamic bodies (never other bullets) in
    /// `rapier2d::pipeline::step_with_ccd`. No change flag, no wake-up (as upstream).
    fn enable_ccd(ref self: RigidBody, enabled: bool) {
        let mut ccd = body_ccd(@self);
        ccd.ccd_enabled = enabled;
        self.set_ccd(ccd);
    }

    /// Whether full CCD is enabled (upstream `is_ccd_enabled`).
    #[inline(always)]
    fn is_ccd_enabled(self: @RigidBody) -> bool {
        wants_ccd(self)
    }

    /// Whether the last CCD pass found this body fast enough to sweep it (upstream
    /// `is_ccd_active`); `false` for a body the pass never examined.
    #[inline(always)]
    fn is_ccd_active(self: @RigidBody) -> bool {
        body_ccd(self).ccd_active
    }

    /// Sets the soft-CCD prediction distance (upstream `set_soft_ccd_prediction`; `0` disables).
    /// Stored and returned; its pipeline effect (enlarged broad-phase box and per-pair prediction
    /// distance) is deferred (see `rapier2d::pipeline::ccd`).
    /// #### Panics
    /// * `'RigidBody: negative prediction'` for a negative distance.
    fn set_soft_ccd_prediction(ref self: RigidBody, prediction_distance: Fixed) {
        assert(prediction_distance >= ZERO, errors::NEGATIVE_PREDICTION);
        let mut ccd = body_ccd(@self);
        ccd.soft_ccd_prediction = prediction_distance;
        self.set_ccd(ccd);
    }

    /// The soft-CCD prediction distance (upstream `soft_ccd_prediction`), `0` by default.
    #[inline(always)]
    fn soft_ccd_prediction(self: @RigidBody) -> Fixed {
        body_ccd(self).soft_ccd_prediction
    }

    /// The whole CCD component (upstream reads `rb.ccd` inside the crate).
    #[inline(always)]
    fn ccd(self: @RigidBody) -> RigidBodyCcd {
        body_ccd(self)
    }

    /// Overwrites the CCD component (the CCD pass's write-back of `ccd_active` and
    /// `ccd_thickness`).
    fn set_ccd(ref self: RigidBody, ccd: RigidBodyCcd) {
        let mut cold = cold_or_default(self.cold);
        let mut extra = cold.extra.value();
        extra.ccd = ccd;
        cold.extra = ColdExtraSlotTrait::new(extra);
        self.cold = BoxTrait::new(Some(cold));
    }
}

#[cfg(test)]
mod tests;
