//! The continuous-collision component of a rigid body (upstream `RigidBodyCcd` in
//! `dynamics/rigid_body_components.rs`), work package CC2.
//!
//! A body is *fast* when the farthest point of its colliders can move more than
//! [`FAST_BODY_SAFETY_FACTOR`] times its thinnest extent (`ccd_thickness`) within one timestep;
//! `rapier2d::pipeline::ccd` sweeps the fast bodies and clamps their motion to the first impact.
//!
//! Storage: the component lives in the body's cold data (`RigidBodyCold::ccd`, boxed), so a body
//! that never touches CCD carries none of it. `ccd_thickness` is derived by the CCD pass from the
//! attached colliders (upstream caches it when a collider is attached); the stored value is the
//! last one the pass computed.

use fixed::{Fixed, FixedTrait, HALF, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::Pose2Trait;
use rapier_math::rot2::Rot2Trait;
use super::forces::RigidBodyForces;
use super::position::RigidBodyPosition;
use super::velocity::RigidBodyVelocity;

/// The fast-body safety factor (upstream `RigidBodyCcd::FAST_BODY_SAFETY_FACTOR`): a body is fast
/// when it can move more than half its thinnest extent in one step.
pub const FAST_BODY_SAFETY_FACTOR: Fixed = HALF;

/// Upstream `Real::MAX` for the thickness of a body without swept collider (`fixed::MAX`).
pub const NO_THICKNESS: Fixed = fixed::MAX;

/// Continuous-collision state of a rigid body (upstream `RigidBodyCcd`, minus the
/// `allow_fast_rotation` switch, which the port keeps in the body's solver flags).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RigidBodyCcd {
    /// The distance under which a motion may tunnel: the smallest `ccd_thickness` of the
    /// attached swept colliders ([`NO_THICKNESS`] without any).
    pub ccd_thickness: Fixed,
    /// The CCD pass swept this body during the last step (set by the pass for every body it
    /// examined: `ccd_enabled` bodies, and every dynamic body in the automatic mode).
    pub ccd_active: bool,
    /// Full ("bullet") CCD: the body sweeps fixed, kinematic and dynamic bodies (never other
    /// bullets).
    pub ccd_enabled: bool,
    /// Soft-CCD prediction distance (upstream `soft_ccd_prediction`, `0` disables it).
    pub soft_ccd_prediction: Fixed,
}

/// Upstream defaults: no thickness bound (`Real::MAX`), inactive, disabled, no soft CCD.
pub impl RigidBodyCcdDefault of Default<RigidBodyCcd> {
    #[inline(always)]
    fn default() -> RigidBodyCcd {
        RigidBodyCcd {
            ccd_thickness: NO_THICKNESS,
            ccd_active: false,
            ccd_enabled: false,
            soft_ccd_prediction: ZERO,
        }
    }
}

/// Upstream `RigidBodyCcd` methods.
pub trait RigidBodyCcdTrait {
    /// Upstream `FAST_BODY_SAFETY_FACTOR` (`1/2`).
    const FAST_BODY_SAFETY_FACTOR: Fixed;
    /// The largest speed of a point of the body's colliders moving with `vels` (upstream
    /// `max_point_velocity`): `|linvel| + |angvel| · max_extent`, `max_extent` the farthest
    /// collider point from the centre of mass. Rounded as the `Fixed` operations (toward zero).
    fn max_point_velocity(self: @RigidBodyCcd, vels: RigidBodyVelocity, max_extent: Fixed) -> Fixed;
    /// Whether the body may tunnel within `dt` (upstream `is_moving_fast`): `max_point_velocity ·
    /// dt > FAST_BODY_SAFETY_FACTOR · ccd_thickness`. With `forces`, the velocities are first
    /// advanced by `force · dt` and `torque · dt` (upstream's pre-solve estimate: the force, not
    /// the acceleration).
    /// #### Panics
    /// * `'Fixed: overflow'` for products leaving the scalar range.
    fn is_moving_fast(
        self: @RigidBodyCcd,
        dt: Fixed,
        vels: RigidBodyVelocity,
        forces: Option<RigidBodyForces>,
        max_extent: Fixed,
    ) -> bool;
    /// The fast-body test on the solved motion (upstream `is_moving_fast_with_next_position`):
    /// the larger of the centre-of-mass displacement from `pos.position` to `pos.next_position`
    /// plus `|sin Δθ| · max_extent`, and `max_point_velocity · dt`, against
    /// `FAST_BODY_SAFETY_FACTOR · ccd_thickness`.
    /// #### Panics
    /// * `'Fixed: overflow'` for products leaving the scalar range.
    fn is_moving_fast_with_next_position(
        self: @RigidBodyCcd,
        dt: Fixed,
        vels: RigidBodyVelocity,
        pos: RigidBodyPosition,
        local_com: Vec2,
        max_extent: Fixed,
    ) -> bool;
}

pub impl RigidBodyCcdImpl of RigidBodyCcdTrait {
    const FAST_BODY_SAFETY_FACTOR: Fixed = FAST_BODY_SAFETY_FACTOR;

    fn max_point_velocity(
        self: @RigidBodyCcd, vels: RigidBodyVelocity, max_extent: Fixed,
    ) -> Fixed {
        vels.linvel.length() + vels.angvel.abs() * max_extent
    }

    fn is_moving_fast(
        self: @RigidBodyCcd,
        dt: Fixed,
        vels: RigidBodyVelocity,
        forces: Option<RigidBodyForces>,
        max_extent: Fixed,
    ) -> bool {
        let max_point_velocity = match forces {
            Some(forces) => {
                let linear = (vels.linvel + forces.force.mul_scalar(dt)).length();
                let angular = (vels.angvel + forces.torque * dt).abs() * max_extent;
                linear + angular
            },
            None => self.max_point_velocity(vels, max_extent),
        };
        max_point_velocity * dt > FAST_BODY_SAFETY_FACTOR * *self.ccd_thickness
    }

    fn is_moving_fast_with_next_position(
        self: @RigidBodyCcd,
        dt: Fixed,
        vels: RigidBodyVelocity,
        pos: RigidBodyPosition,
        local_com: Vec2,
        max_extent: Fixed,
    ) -> bool {
        let com1 = pos.position.transform_point(local_com);
        let com2 = pos.next_position.transform_point(local_com);
        let delta_rot = pos.next_position.rotation * pos.position.rotation.inverse();
        let angular_delta = delta_rot.im.abs() * max_extent;
        let max_delta_position = (com2 - com1).length() + angular_delta;
        let by_velocity = self.max_point_velocity(vels, max_extent) * dt;
        let max_motion = if max_delta_position > by_velocity {
            max_delta_position
        } else {
            by_velocity
        };
        max_motion > FAST_BODY_SAFETY_FACTOR * *self.ccd_thickness
    }
}

#[cfg(test)]
mod tests;
