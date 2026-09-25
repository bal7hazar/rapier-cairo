//! World pose of a rigid-body (upstream `RigidBodyPosition`) and the one-substep prediction
//! built on the other components.
//!
//! `position` is the pose the rest of the engine sees; `next_position` is where the body is
//! heading. They are equal outside a step, except for position-based kinematic bodies whose
//! `next_position` is set by the user. The substep loop writes `next_position`, and the step
//! validates it at the end (`position := next_position`).
//!
//! Deferred: the CCD clamping of `predict_position_using_velocity_and_forces_with_max_dist`.

use fixed::Fixed;
use fixed::trig::TrigTrait;
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::{IDENTITY, Pose2, Pose2Trait};
use rapier_math::rot2::{Rot2, Rot2Trait};
use super::forces::{RigidBodyForces, RigidBodyForcesTrait};
use super::mass_props::RigidBodyMassProps;
use super::velocity::{RigidBodyVelocity, RigidBodyVelocityTrait};

/// The current and next world poses of a rigid-body. Default: both the identity.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RigidBodyPosition {
    /// World pose of the body.
    pub position: Pose2,
    /// Pose the body is heading to; written by the solver, validated at the end of the step.
    pub next_position: Pose2,
}

/// Upstream default: both poses are the identity.
pub impl RigidBodyPositionDefault of Default<RigidBodyPosition> {
    #[inline(always)]
    fn default() -> RigidBodyPosition {
        RigidBodyPosition { position: IDENTITY, next_position: IDENTITY }
    }
}

/// Pose updates of [`RigidBodyPosition`].
#[generate_trait]
pub impl RigidBodyPositionImpl of RigidBodyPositionTrait {
    /// Both poses set to `position` (upstream's `From<Pose>`), i.e. a body at rest there.
    #[inline(always)]
    fn from_position(position: Pose2) -> RigidBodyPosition {
        RigidBodyPosition { position, next_position: position }
    }

    /// Teleports the body: both poses become `position` (upstream `RigidBody::set_position`).
    ///
    /// The caller must follow with
    /// [`update_world_mass_properties`](super::mass_props::RigidBodyMassPropsTrait::update_world_mass_properties),
    /// as upstream does, so that the world centre of mass stays consistent.
    #[inline(always)]
    fn set_position(self: RigidBodyPosition, position: Pose2) -> RigidBodyPosition {
        RigidBodyPosition { position, next_position: position }
    }

    /// Teleports the body to `translation`, keeping its orientation (upstream
    /// `RigidBody::set_translation`). Same follow-up as [`Self::set_position`].
    #[inline(always)]
    fn set_translation(self: RigidBodyPosition, translation: Vec2) -> RigidBodyPosition {
        self.set_position(Pose2 { translation, rotation: self.position.rotation })
    }

    /// Reorients the body, keeping its translation (upstream `RigidBody::set_rotation`).
    /// Same follow-up as [`Self::set_position`]; `rotation` must be unit.
    #[inline(always)]
    fn set_rotation(self: RigidBodyPosition, rotation: Rot2) -> RigidBodyPosition {
        self.set_position(Pose2 { translation: self.position.translation, rotation })
    }

    /// Upstream `interpolate_velocity`: COM displacement and shortest relative angle times
    /// `inv_dt`. Algebraically simplifies upstream's COM-shift conjugation. Products floor;
    /// angle uses fixed atan2. Unit rotations required; overflow propagates. Zero inv_dt
    /// yields zero velocity for representable errors. No pose is changed.
    fn interpolate_velocity(
        self: RigidBodyPosition, inv_dt: Fixed, local_com: Vec2,
    ) -> RigidBodyVelocity {
        let errors = self.pose_errors(local_com);
        RigidBodyVelocity {
            linvel: errors.linvel.mul_scalar(inv_dt), angvel: errors.angvel * inv_dt,
        }
    }

    /// Difference between `next_position` and `position`, in upstream's 2D PD-error layout.
    fn pose_errors(self: RigidBodyPosition, local_com: Vec2) -> RigidBodyVelocity {
        let linear = self.next_position.transform_point(local_com)
            - self.position.transform_point(local_com);
        let rotation = self.next_position.rotation * self.position.rotation.inverse();
        RigidBodyVelocity { linvel: linear, angvel: rotation.im.atan2(rotation.re) }
    }

    /// Pose of the body after `dt`, integrating the forces first and the resulting velocities
    /// second (symplectic Euler), from `self.position`.
    ///
    /// Mirrors `RigidBody::predict_position_using_velocity_and_forces`, i.e. upstream's
    /// `RigidBodyPosition::integrate_forces_and_velocities`. `dt` is the **substep** length in
    /// the solver loop.
    ///
    /// # Panics
    /// * `'Fixed: overflow'` if a velocity or a pose component leaves the scalar range.
    #[inline(always)]
    fn predict_position_using_velocity_and_forces(
        self: RigidBodyPosition,
        dt: Fixed,
        forces: RigidBodyForces,
        vels: RigidBodyVelocity,
        mprops: RigidBodyMassProps,
    ) -> Pose2 {
        forces
            .integrate(dt, vels, mprops)
            .integrate(dt, self.position, mprops.local_mprops.local_com)
    }

    /// Advances `next_position` by one substep of `dt`, leaving `position` untouched.
    ///
    /// # Deviations
    /// * Upstream returns the `Pose` alone and lets the caller assign it to `next_position`;
    ///   the assignment is done here (see [`Self::predict_position_using_velocity_and_forces`]
    ///   for the bare pose). The two are the same computation.
    /// # Panics
    /// * As [`Self::predict_position_using_velocity_and_forces`].
    #[inline(always)]
    fn integrate_forces_and_velocities(
        self: RigidBodyPosition,
        dt: Fixed,
        forces: RigidBodyForces,
        vels: RigidBodyVelocity,
        mprops: RigidBodyMassProps,
    ) -> RigidBodyPosition {
        RigidBodyPosition {
            position: self.position,
            next_position: self
                .predict_position_using_velocity_and_forces(dt, forces, vels, mprops),
        }
    }

    /// Pose of the body after `dt` of its current velocities only, forces ignored (upstream
    /// `RigidBody::predict_position_using_velocity`).
    ///
    /// # Panics
    /// * `'Fixed: overflow'` if a pose component leaves the scalar range.
    #[inline(always)]
    fn predict_position_using_velocity(
        self: RigidBodyPosition, dt: Fixed, vels: RigidBodyVelocity, mprops: RigidBodyMassProps,
    ) -> Pose2 {
        vels.integrate(dt, self.position, mprops.local_mprops.local_com)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::{Vec2, Vec2Trait};
    use rapier_core::rigid_body::RigidBodyType;
    use rapier_geometry2d::mass::MassProperties;
    use rapier_math::pose2::{IDENTITY, Pose2};
    use rapier_math::rot2::{Rot2, Rot2Trait};
    use rapier_testing::opaque;
    use super::super::forces::{RigidBodyForces, RigidBodyForcesTrait};
    use super::super::locked_axes::{LockedAxes, LockedAxesTrait};
    use super::super::mass_props::{RigidBodyMassProps, RigidBodyMassPropsTrait};
    use super::super::velocity::RigidBodyVelocityTrait;
    use super::{RigidBodyPosition, RigidBodyPositionTrait};

    /// `1 / 60` rounded to nearest, as `IntegrationParameters::default().dt`.
    const DT: Fixed = Fixed { raw: 71582788 };
    const LOCAL: MassProperties = MassProperties {
        local_com: Vec2 { x: ZERO, y: ZERO }, inv_mass: TWO, inv_principal_inertia: HALF,
    };
    const POSE: Pose2 = Pose2 {
        translation: Vec2 { x: ONE, y: TWO }, rotation: Rot2 { re: ONE, im: ZERO },
    };
    /// [`LOCAL`] updated for [`POSE`] as a free dynamic body; `mprops()` rebuilds it and
    /// `test_prediction_is_forces_then_velocities` checks that the two agree.
    const MPROPS: RigidBodyMassProps = RigidBodyMassProps {
        flags: LockedAxes { bits: 0 },
        local_mprops: LOCAL,
        world_com: Vec2 { x: ONE, y: TWO },
        effective_inv_mass: Vec2 { x: TWO, y: TWO },
        effective_world_inv_inertia: HALF,
        max_extent: ZERO,
    };

    fn mprops() -> RigidBodyMassProps {
        RigidBodyMassPropsTrait::from_local(LOCAL, LockedAxesTrait::empty())
            .update_world_mass_properties(RigidBodyType::Dynamic, POSE)
    }

    fn forces_with(force: Vec2, torque: Fixed) -> RigidBodyForces {
        RigidBodyForces {
            force, torque, gravity_scale: ONE, user_force: Vec2Trait::ZERO, user_torque: ZERO,
        }
    }

    fn close(a: Vec2, b: Vec2, ulps: i64) {
        assert!(a.x.abs_diff_eq(b.x, Fixed { raw: ulps }), "x {:?} vs {:?}", a.x, b.x);
        assert!(a.y.abs_diff_eq(b.y, Fixed { raw: ulps }), "y {:?} vs {:?}", a.y, b.y);
    }

    #[test]
    fn test_default_and_setters_keep_both_poses_equal() {
        let default: RigidBodyPosition = Default::default();
        assert_eq!(default.position, IDENTITY);
        assert_eq!(default.next_position, IDENTITY);
        let placed = RigidBodyPositionTrait::from_position(POSE);
        assert_eq!(placed.position, POSE);
        assert_eq!(placed.next_position, POSE);
        let moved = placed.set_translation(Vec2 { x: -ONE, y: HALF });
        assert_eq!(moved.position.translation, Vec2 { x: -ONE, y: HALF });
        assert_eq!(moved.position.rotation, POSE.rotation);
        assert_eq!(moved.position, moved.next_position);
        let quarter = Rot2 { re: ZERO, im: ONE };
        let turned = placed.set_rotation(quarter);
        assert_eq!(turned.position.rotation, quarter);
        assert_eq!(turned.position.translation, POSE.translation);
        assert_eq!(turned.position, turned.next_position);
        let teleported = turned.set_position(IDENTITY);
        assert_eq!(teleported.position, IDENTITY);
        assert_eq!(teleported.next_position, IDENTITY);
    }

    #[test]
    fn test_prediction_is_forces_then_velocities() {
        let mprops = mprops();
        assert_eq!(mprops, MPROPS);
        let pos = RigidBodyPositionTrait::from_position(POSE);
        let vels = RigidBodyVelocityTrait::new(Vec2 { x: HALF, y: -ONE }, HALF);
        let forces = forces_with(Vec2 { x: ONE, y: -TWO }, TWO);
        let predicted = pos.predict_position_using_velocity_and_forces(DT, forces, vels, mprops);
        // Symplectic Euler: the velocities used for the pose are the integrated ones.
        let integrated = forces.integrate(DT, vels, mprops);
        assert_eq!(predicted, integrated.integrate(DT, POSE, LOCAL.local_com));
        assert!(predicted.rotation.is_unit());
        // Ignoring the forces uses the initial velocities instead, hence a shorter step.
        let inertial = pos.predict_position_using_velocity(DT, vels, mprops);
        assert_eq!(inertial, vels.integrate(DT, POSE, LOCAL.local_com));
        assert!(predicted.translation.y < inertial.translation.y);
        // `integrate_forces_and_velocities` writes the same pose into `next_position` only.
        let stepped = pos.integrate_forces_and_velocities(DT, forces, vels, mprops);
        assert_eq!(stepped.position, POSE);
        assert_eq!(stepped.next_position, predicted);
    }

    #[test]
    fn test_body_at_rest_without_forces_does_not_move() {
        let mprops = mprops();
        let pos = RigidBodyPositionTrait::from_position(POSE);
        let idle = forces_with(Vec2Trait::ZERO, ZERO);
        let still = RigidBodyVelocityTrait::zero();
        let stepped = pos.integrate_forces_and_velocities(DT, idle, still, mprops);
        assert_eq!(stepped, pos);
        // A zero substep never moves anything either.
        let vels = RigidBodyVelocityTrait::new(Vec2 { x: HALF, y: -ONE }, HALF);
        let forces = forces_with(Vec2 { x: ONE, y: -TWO }, TWO);
        assert_eq!(pos.integrate_forces_and_velocities(ZERO, forces, vels, mprops), pos);
    }

    /// Four substeps of `dt / 4` are the pose a solver iteration reaches; the total displacement
    /// stays the constant-acceleration one within the substep rounding.
    #[test]
    fn test_substep_chain_accumulates_the_displacement() {
        let mprops = mprops();
        let substep = Fixed { raw: DT.raw / 4 };
        let forces = forces_with(Vec2 { x: ZERO, y: -TWO }, ZERO);
        let mut pos = RigidBodyPositionTrait::from_position(POSE);
        let mut vels = RigidBodyVelocityTrait::zero();
        let mut n: u8 = 0;
        while n != 4 {
            vels = forces.integrate(substep, vels, mprops);
            pos =
                RigidBodyPositionTrait::from_position(
                    vels.integrate(substep, pos.position, LOCAL.local_com),
                );
            n += 1;
        }
        // Acceleration is `force * inv_mass = -4`; after 4 substeps of h: v = -16h (exact, `16`
        // and `4` being powers of two) and y = y0 - 4h²(1 + 2 + 3 + 4) = y0 - 40h², the latter
        // up to the four substep floors.
        let h = substep;
        assert_eq!(vels.linvel, Vec2 { x: ZERO, y: Fixed { raw: -16 * h.raw } });
        let drop = Fixed { raw: 40 * h.raw * h.raw / 4294967296 };
        close(pos.position.translation, Vec2 { x: ONE, y: TWO - drop }, 4);
        assert_eq!(pos.position.rotation, POSE.rotation);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_default() {
        assert_eq!(Default::<RigidBodyPosition>::default().position, IDENTITY);
    }

    #[test]
    fn gas_from_position() {
        assert_eq!(RigidBodyPositionTrait::from_position(opaque(POSE)).next_position, POSE);
    }

    #[test]
    fn gas_set_translation() {
        assert_eq!(
            opaque(RigidBodyPositionTrait::from_position(POSE))
                .set_translation(opaque(Vec2 { x: -ONE, y: HALF }))
                .position
                .translation,
            Vec2 { x: -ONE, y: HALF },
        );
    }

    #[test]
    fn gas_set_rotation() {
        assert_eq!(
            opaque(RigidBodyPositionTrait::from_position(POSE))
                .set_rotation(opaque(Rot2 { re: ZERO, im: ONE }))
                .position
                .rotation,
            Rot2 { re: ZERO, im: ONE },
        );
    }

    #[test]
    fn gas_set_position() {
        assert_eq!(
            opaque(RigidBodyPositionTrait::from_position(POSE))
                .set_position(opaque(IDENTITY))
                .next_position,
            IDENTITY,
        );
    }

    #[test]
    fn gas_predict_position_using_velocity_and_forces() {
        assert!(
            opaque(RigidBodyPositionTrait::from_position(POSE))
                .predict_position_using_velocity_and_forces(
                    opaque(DT),
                    opaque(forces_with(Vec2 { x: ONE, y: -TWO }, TWO)),
                    opaque(RigidBodyVelocityTrait::new(Vec2 { x: HALF, y: -ONE }, HALF)),
                    opaque(MPROPS),
                )
                .rotation
                .is_unit(),
        );
    }

    #[test]
    fn gas_predict_position_using_velocity() {
        assert!(
            opaque(RigidBodyPositionTrait::from_position(POSE))
                .predict_position_using_velocity(
                    opaque(DT),
                    opaque(RigidBodyVelocityTrait::new(Vec2 { x: HALF, y: -ONE }, HALF)),
                    opaque(MPROPS),
                )
                .rotation
                .is_unit(),
        );
    }

    #[test]
    fn gas_integrate_forces_and_velocities() {
        assert!(
            opaque(RigidBodyPositionTrait::from_position(POSE))
                .integrate_forces_and_velocities(
                    opaque(DT),
                    opaque(forces_with(Vec2 { x: ONE, y: -TWO }, TWO)),
                    opaque(RigidBodyVelocityTrait::new(Vec2 { x: HALF, y: -ONE }, HALF)),
                    opaque(MPROPS),
                )
                .next_position
                .rotation
                .is_unit(),
        );
    }
}

#[cfg(test)]
mod body_tests;
#[cfg(test)]
mod kinematic_tests;
