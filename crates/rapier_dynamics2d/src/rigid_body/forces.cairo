//! External forces applied to a rigid-body (upstream `RigidBodyForces`) and their integration
//! into velocities.
//!
//! Upstream splits the accumulation in two: `user_force` / `user_torque` are what the user adds
//! and survive a step, while `force` / `torque` are the effective values the solver reads,
//! rebuilt once per step by `compute_effective_force_and_torque` (user force plus scaled
//! gravity). [`RigidBodyForcesTrait::integrate`] then turns them into a velocity increment, once
//! per body per substep.
//!
//! Gravity is applied as a **force**, `gravity * mass * gravity_scale`, and divided back by the
//! mass in `integrate`: in Q32.32 the two products do not cancel exactly (the acceleration of the
//! `ball_drop` scene lands 3 ulp below `-9.81`), but the order is upstream's and the golden
//! replay depends on it.
//!
//! Deferred: the 3D gyroscopic switch and the CCD fields.

use fixed::wide::mul_add;
use fixed::{Fixed, ONE, ZERO};
use glam::{Vec2, Vec2Trait};
use super::mass_props::RigidBodyMassProps;
use super::velocity::RigidBodyVelocity;

/// The external forces applied to a rigid-body. Default: no force, no torque, gravity scale `1`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RigidBodyForces {
    /// Effective force of the step: user force plus scaled gravity.
    pub force: Vec2,
    /// Effective torque of the step, counter-clockwise.
    pub torque: Fixed,
    /// Gravity is multiplied by this factor before being applied to this body.
    pub gravity_scale: Fixed,
    /// Force accumulated by the user; not cleared by a step.
    pub user_force: Vec2,
    /// Torque accumulated by the user; not cleared by a step.
    pub user_torque: Fixed,
}

/// Upstream default: everything zero but a gravity scale of `1`.
pub impl RigidBodyForcesDefault of Default<RigidBodyForces> {
    #[inline(always)]
    fn default() -> RigidBodyForces {
        RigidBodyForces {
            force: Vec2Trait::ZERO,
            torque: ZERO,
            gravity_scale: ONE,
            user_force: Vec2Trait::ZERO,
            user_torque: ZERO,
        }
    }
}

/// Accumulation and integration of [`RigidBodyForces`].
#[generate_trait]
pub impl RigidBodyForcesImpl of RigidBodyForcesTrait {
    /// Integrates the effective force and torque over `dt`, returning the new velocities.
    ///
    /// Symplectic Euler: `linvel + force * inv_mass * dt`, `angvel + inv_inertia * torque * dt`.
    /// The inverse mass and inertia are the **effective** ones, so a locked axis or a
    /// non-dynamic body simply does not accelerate. Each component costs two floors — the
    /// acceleration, then the fused `mul_add` of the increment and the previous velocity — which
    /// is the rounding the `ball_drop` golden replay reproduces.
    ///
    /// # Arguments
    /// * `dt` — substep length in seconds.
    /// * `init_vels` — velocities at the beginning of the substep.
    /// * `mprops` — mass properties, already updated for the current pose.
    ///
    /// # Panics
    /// * `'Fixed: overflow'` if an acceleration or a velocity leaves the scalar range.
    fn integrate(
        self: RigidBodyForces, dt: Fixed, init_vels: RigidBodyVelocity, mprops: RigidBodyMassProps,
    ) -> RigidBodyVelocity {
        RigidBodyVelocity {
            linvel: Vec2 {
                x: mul_add(self.force.x * mprops.effective_inv_mass.x, dt, init_vels.linvel.x),
                y: mul_add(self.force.y * mprops.effective_inv_mass.y, dt, init_vels.linvel.y),
            },
            angvel: mul_add(mprops.effective_world_inv_inertia * self.torque, dt, init_vels.angvel),
        }
    }

    /// The gravitational force on a body of effective `mass`: `gravity * mass * gravity_scale`.
    ///
    /// The term upstream adds to the user force in `compute_effective_force_and_torque`. A
    /// locked axis (or a non-dynamic body) has a zero effective mass and therefore no gravity.
    /// # Panics
    /// * `'Fixed: overflow'` if a component leaves the scalar range.
    #[inline(always)]
    fn gravity_force(self: RigidBodyForces, gravity: Vec2, mass: Vec2) -> Vec2 {
        (gravity * mass).mul_scalar(self.gravity_scale)
    }

    /// Rebuilds the effective force and torque of the step from the user ones and gravity.
    ///
    /// Mirrors upstream's `compute_effective_force_and_torque`, called once per step before the
    /// substep loop. `mass` is [`RigidBodyMassPropsTrait::effective_mass`](
    /// super::mass_props::RigidBodyMassPropsTrait::effective_mass).
    /// # Panics
    /// * `'Fixed: overflow'` if the force leaves the scalar range.
    #[inline(always)]
    fn compute_effective_force_and_torque(
        self: RigidBodyForces, gravity: Vec2, mass: Vec2,
    ) -> RigidBodyForces {
        RigidBodyForces {
            force: self.user_force + self.gravity_force(gravity, mass),
            torque: self.user_torque,
            gravity_scale: self.gravity_scale,
            user_force: self.user_force,
            user_torque: self.user_torque,
        }
    }

    /// Accumulates a user force expressed as an **acceleration**: `user_force += acc * mass`.
    ///
    /// # Deviations
    /// * rapier.cairo addition. Upstream only has the force-valued `add_force`, and applies an
    ///   acceleration through `gravity_scale`; a mass-independent thruster otherwise has to
    ///   multiply by the effective mass at every call site.
    /// # Panics
    /// * `'Fixed: overflow'` if the accumulated force leaves the scalar range.
    #[inline(always)]
    fn add_linear_acceleration(self: RigidBodyForces, acc: Vec2, mass: Vec2) -> RigidBodyForces {
        RigidBodyForces {
            force: self.force,
            torque: self.torque,
            gravity_scale: self.gravity_scale,
            user_force: self.user_force + acc * mass,
            user_torque: self.user_torque,
        }
    }

    /// Accumulates a user force (upstream `RigidBody::add_force`).
    ///
    /// The body-type guard of upstream lives in the body set (package DD): a non-dynamic body
    /// never accelerates anyway, its effective inverse mass being zero.
    /// # Panics
    /// * `'Fixed: overflow'` if the accumulated force leaves the scalar range.
    #[inline(always)]
    fn add_force(self: RigidBodyForces, force: Vec2) -> RigidBodyForces {
        RigidBodyForces {
            force: self.force,
            torque: self.torque,
            gravity_scale: self.gravity_scale,
            user_force: self.user_force + force,
            user_torque: self.user_torque,
        }
    }

    /// Accumulates a user torque (upstream `RigidBody::add_torque`), counter-clockwise.
    ///
    /// # Panics
    /// * `'Fixed: overflow'` if the accumulated torque leaves the scalar range.
    #[inline(always)]
    fn add_torque(self: RigidBodyForces, torque: Fixed) -> RigidBodyForces {
        RigidBodyForces {
            force: self.force,
            torque: self.torque,
            gravity_scale: self.gravity_scale,
            user_force: self.user_force,
            user_torque: self.user_torque + torque,
        }
    }

    /// Accumulates a user force applied at the world-space `point` (upstream
    /// `RigidBody::add_force_at_point`): the force itself plus the torque of its lever arm.
    ///
    /// # Panics
    /// * `'i64_sub Overflow'` if the lever arm leaves the scalar range, `'Fixed: overflow'` if
    ///   the accumulated force or torque does.
    fn add_force_at_point(
        self: RigidBodyForces, mprops: RigidBodyMassProps, force: Vec2, point: Vec2,
    ) -> RigidBodyForces {
        let dpt = point - mprops.world_com;
        RigidBodyForces {
            force: self.force,
            torque: self.torque,
            gravity_scale: self.gravity_scale,
            user_force: self.user_force + force,
            user_torque: self.user_torque + dpt.perp_dot(force),
        }
    }

    /// Clears the user force (upstream `RigidBody::reset_forces`); the effective `force` keeps
    /// its value until the next `compute_effective_force_and_torque`.
    #[inline(always)]
    fn reset_forces(self: RigidBodyForces) -> RigidBodyForces {
        RigidBodyForces {
            force: self.force,
            torque: self.torque,
            gravity_scale: self.gravity_scale,
            user_force: Vec2Trait::ZERO,
            user_torque: self.user_torque,
        }
    }

    /// Clears the user torque (upstream `RigidBody::reset_torques`).
    #[inline(always)]
    fn reset_torques(self: RigidBodyForces) -> RigidBodyForces {
        RigidBodyForces {
            force: self.force,
            torque: self.torque,
            gravity_scale: self.gravity_scale,
            user_force: self.user_force,
            user_torque: ZERO,
        }
    }
}

#[cfg(test)]
mod alternatives {
    use fixed::Fixed;
    use glam::{Vec2, Vec2Trait};
    use super::RigidBodyForces;
    use super::super::mass_props::RigidBodyMassProps;
    use super::super::velocity::RigidBodyVelocity;

    /// The composed port: one rounded acceleration vector, one rounded increment, one sum.
    pub fn integrate(
        forces: RigidBodyForces,
        dt: Fixed,
        init_vels: RigidBodyVelocity,
        mprops: RigidBodyMassProps,
    ) -> RigidBodyVelocity {
        let linear_acc = forces.force * mprops.effective_inv_mass;
        let angular_acc = mprops.effective_world_inv_inertia * forces.torque;
        RigidBodyVelocity {
            linvel: init_vels.linvel + linear_acc.mul_scalar(dt),
            angvel: init_vels.angvel + angular_acc * dt,
        }
    }

    /// `gravity * gravity_scale` first, then the mass: the same three products in the other
    /// order, which rounds the scaled gravity instead of the force.
    pub fn gravity_force(forces: RigidBodyForces, gravity: Vec2, mass: Vec2) -> Vec2 {
        gravity.mul_scalar(forces.gravity_scale) * mass
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::{Vec2, Vec2Trait};
    use rapier_core::rigid_body::RigidBodyType;
    use rapier_geometry2d::mass::MassProperties;
    use rapier_math::pose2::Pose2;
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use super::super::locked_axes::{
        LockedAxes, LockedAxesTrait, ROTATION_LOCKED, TRANSLATION_LOCKED_X,
    };
    use super::super::mass_props::{RigidBodyMassProps, RigidBodyMassPropsTrait};
    use super::super::velocity::RigidBodyVelocityTrait;
    use super::{RigidBodyForces, RigidBodyForcesTrait, alternatives};

    /// `1 / 60` rounded to nearest, as `IntegrationParameters::default().dt`.
    const DT: Fixed = Fixed { raw: 71582788 };
    /// Mass `0.5`, angular inertia `2`, centre of mass at the origin.
    const LOCAL: MassProperties = MassProperties {
        local_com: Vec2 { x: ZERO, y: ZERO }, inv_mass: TWO, inv_principal_inertia: HALF,
    };
    const POSE: Pose2 = Pose2 {
        translation: Vec2 { x: ONE, y: TWO }, rotation: Rot2 { re: ONE, im: ZERO },
    };
    /// [`LOCAL`] updated for [`POSE`] as a free dynamic body, checked in
    /// `test_gravity_force_and_effective_force` and used as-is by the probes.
    const FREE: RigidBodyMassProps = RigidBodyMassProps {
        flags: LockedAxes { bits: 0 },
        local_mprops: LOCAL,
        world_com: Vec2 { x: ONE, y: TWO },
        effective_inv_mass: Vec2 { x: TWO, y: TWO },
        effective_world_inv_inertia: HALF,
    };
    /// `-9.81` on the Y axis, the gravity of every golden scene.
    const GRAVITY: Vec2 = Vec2 { x: ZERO, y: Fixed { raw: -42133629174 } };

    fn mprops(flags: LockedAxes) -> RigidBodyMassProps {
        RigidBodyMassPropsTrait::from_local(LOCAL, flags)
            .update_world_mass_properties(RigidBodyType::Dynamic, POSE)
    }

    fn close(a: Vec2, b: Vec2, ulps: i64) {
        assert!(a.x.abs_diff_eq(b.x, Fixed { raw: ulps }), "x {:?} vs {:?}", a.x, b.x);
        assert!(a.y.abs_diff_eq(b.y, Fixed { raw: ulps }), "y {:?} vs {:?}", a.y, b.y);
    }

    #[test]
    fn test_default_and_accumulators() {
        let forces: RigidBodyForces = Default::default();
        assert_eq!(forces.force, Vec2Trait::ZERO);
        assert_eq!(forces.torque, ZERO);
        assert_eq!(forces.gravity_scale, ONE);
        assert_eq!(forces.user_force, Vec2Trait::ZERO);
        assert_eq!(forces.user_torque, ZERO);
        // The user accumulators add up and survive until they are reset.
        let pushed = forces.add_force(Vec2 { x: ONE, y: ZERO }).add_force(Vec2 { x: ONE, y: TWO });
        assert_eq!(pushed.user_force, Vec2 { x: TWO, y: TWO });
        assert_eq!(pushed.force, Vec2Trait::ZERO);
        let spun = pushed.add_torque(HALF).add_torque(HALF);
        assert_eq!(spun.user_torque, ONE);
        assert_eq!(spun.reset_forces().user_force, Vec2Trait::ZERO);
        assert_eq!(spun.reset_forces().user_torque, ONE);
        assert_eq!(spun.reset_torques().user_torque, ZERO);
        assert_eq!(spun.reset_torques().user_force, Vec2 { x: TWO, y: TWO });
        // An acceleration is stored as the force it takes on this body (mass 0.5).
        let free = mprops(LockedAxesTrait::empty());
        let accelerated = forces.add_linear_acceleration(GRAVITY, free.effective_mass());
        close(accelerated.user_force, GRAVITY.mul_scalar(HALF), 2);
    }

    #[test]
    fn test_force_at_point_builds_a_torque() {
        let free = mprops(LockedAxesTrait::empty());
        let forces: RigidBodyForces = Default::default();
        let force = Vec2 { x: ONE, y: -HALF };
        let point = free.world_com + Vec2 { x: ONE, y: TWO };
        let applied = forces.add_force_at_point(free, force, point);
        assert_eq!(applied.user_force, force);
        // gcross((1, 2), (1, -0.5)) = 1 * -0.5 - 2 * 1 = -2.5.
        assert_eq!(applied.user_torque, Fixed { raw: -10737418240 });
        // A force through the centre of mass is pure translation.
        assert_eq!(forces.add_force_at_point(free, force, free.world_com).user_torque, ZERO);
    }

    #[test]
    fn test_gravity_force_and_effective_force() {
        let free = mprops(LockedAxesTrait::empty());
        assert_eq!(free, FREE);
        let forces: RigidBodyForces = Default::default();
        let mass = free.effective_mass();
        assert_eq!(mass, Vec2 { x: HALF, y: HALF });
        let weight = forces.gravity_force(GRAVITY, mass);
        close(weight, GRAVITY.mul_scalar(HALF), 2);
        close(weight, alternatives::gravity_force(forces, GRAVITY, mass), 2);
        // The effective force is the user force plus the weight; the torque is the user torque.
        let effective = forces
            .add_force(Vec2 { x: ONE, y: ZERO })
            .add_torque(TWO)
            .compute_effective_force_and_torque(GRAVITY, mass);
        assert_eq!(effective.force, Vec2 { x: ONE, y: weight.y });
        assert_eq!(effective.torque, TWO);
        // A doubled gravity scale doubles the weight; a zero one removes it.
        let heavy = forces_with(Vec2Trait::ZERO, ZERO, TWO);
        close(heavy.gravity_force(GRAVITY, mass), GRAVITY, 4);
        let floating = forces_with(Vec2Trait::ZERO, ZERO, ZERO);
        assert_eq!(floating.gravity_force(GRAVITY, mass), Vec2Trait::ZERO);
        // A locked axis has an infinite effective mass, reported as zero: no weight at all.
        let locked = mprops(TRANSLATION_LOCKED_X | ROTATION_LOCKED);
        assert_eq!(locked.effective_mass(), Vec2 { x: ZERO, y: HALF });
        assert_eq!(forces.gravity_force(GRAVITY, locked.effective_mass()).x, ZERO);
    }

    #[test]
    fn test_integrate_against_composed_port() {
        let free = mprops(LockedAxesTrait::empty());
        let init = RigidBodyVelocityTrait::new(Vec2 { x: HALF, y: -ONE }, HALF);
        for (force, torque) in array![
            (Vec2Trait::ZERO, ZERO), (Vec2 { x: ONE, y: -TWO }, ZERO), (Vec2Trait::ZERO, TWO),
            (Vec2 { x: -HALF, y: TWO }, -ONE),
        ]
            .span() {
            let forces = forces_with(*force, *torque, ONE);
            let actual = forces.integrate(DT, init, free);
            let candidate = alternatives::integrate(forces, DT, init, free);
            close(actual.linvel, candidate.linvel, 2);
            assert!(actual.angvel.abs_diff_eq(candidate.angvel, Fixed { raw: 2 }));
            // Acceleration is `force * inv_mass`, i.e. twice the force for this body.
            close(actual.linvel - init.linvel, (*force).mul_scalar(TWO).mul_scalar(DT), 2);
        }
        // No force, no change; zero `dt`, no change either.
        let idle: RigidBodyForces = Default::default();
        assert_eq!(idle.integrate(DT, init, free), init);
        let pushing = forces_with(Vec2 { x: ONE, y: -TWO }, TWO, ONE);
        assert_eq!(pushing.integrate(ZERO, init, free), init);
    }

    #[test]
    fn test_locked_and_fixed_bodies_do_not_accelerate() {
        let init = RigidBodyVelocityTrait::new(Vec2 { x: HALF, y: -ONE }, HALF);
        let forces = forces_with(Vec2 { x: ONE, y: -TWO }, TWO, ONE);
        let locked = mprops(TRANSLATION_LOCKED_X | ROTATION_LOCKED);
        let after = forces.integrate(DT, init, locked);
        assert_eq!(after.linvel.x, init.linvel.x);
        assert!(after.linvel.y < init.linvel.y);
        assert_eq!(after.angvel, init.angvel);
        let fixed = RigidBodyMassPropsTrait::from_local(LOCAL, LockedAxesTrait::empty())
            .update_world_mass_properties(RigidBodyType::Fixed, POSE);
        assert_eq!(forces.integrate(DT, init, fixed), init);
    }

    /// The two orders of the three gravity products agree within their rounding, and the two
    /// integration kernels within theirs.
    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_candidates(fx: i32, fy: i32, torque: i32, scale: u32) {
        let free = mprops(LockedAxesTrait::empty());
        let forces = forces_with(
            Vec2 { x: Fixed { raw: fx.into() }, y: Fixed { raw: fy.into() } },
            Fixed { raw: torque.into() },
            Fixed { raw: scale.into() },
        );
        let init = RigidBodyVelocityTrait::zero();
        let actual = forces.integrate(DT, init, free);
        let candidate = alternatives::integrate(forces, DT, init, free);
        close(actual.linvel, candidate.linvel, 2);
        assert!(actual.angvel.abs_diff_eq(candidate.angvel, Fixed { raw: 2 }));
        let mass = free.effective_mass();
        close(
            forces.gravity_force(GRAVITY, mass),
            alternatives::gravity_force(forces, GRAVITY, mass),
            2,
        );
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_default() {
        assert_eq!(Default::<RigidBodyForces>::default().gravity_scale, ONE);
    }

    #[test]
    fn gas_integrate_fused() {
        assert!(
            opaque(pushing_forces())
                .integrate(opaque(DT), opaque(RigidBodyVelocityTrait::zero()), opaque(FREE))
                .angvel != ZERO,
        );
    }

    #[test]
    fn gas_integrate_composed() {
        assert!(
            alternatives::integrate(
                opaque(pushing_forces()),
                opaque(DT),
                opaque(RigidBodyVelocityTrait::zero()),
                opaque(FREE),
            )
                .angvel != ZERO,
        );
    }

    #[test]
    fn gas_gravity_force_mass_first() {
        assert!(
            opaque(pushing_forces())
                .gravity_force(
                    opaque(GRAVITY), opaque(Vec2 { x: HALF, y: HALF }),
                ) != Vec2Trait::ZERO,
        );
    }

    #[test]
    fn gas_gravity_force_scale_first() {
        assert!(
            alternatives::gravity_force(
                opaque(pushing_forces()), opaque(GRAVITY), opaque(Vec2 { x: HALF, y: HALF }),
            ) != Vec2Trait::ZERO,
        );
    }

    #[test]
    fn gas_compute_effective_force_and_torque() {
        assert!(
            opaque(pushing_forces())
                .compute_effective_force_and_torque(
                    opaque(GRAVITY), opaque(Vec2 { x: HALF, y: HALF }),
                )
                .force != Vec2Trait::ZERO,
        );
    }

    #[test]
    fn gas_add_force() {
        assert_eq!(
            opaque(pushing_forces()).add_force(opaque(Vec2 { x: ONE, y: ZERO })).user_force.x, ONE,
        );
    }

    #[test]
    fn gas_add_torque() {
        assert_eq!(opaque(pushing_forces()).add_torque(opaque(ONE)).user_torque, ONE);
    }

    #[test]
    fn gas_add_linear_acceleration() {
        assert!(
            opaque(pushing_forces())
                .add_linear_acceleration(opaque(GRAVITY), opaque(Vec2 { x: HALF, y: HALF }))
                .user_force != Vec2Trait::ZERO,
        );
    }

    #[test]
    fn gas_add_force_at_point() {
        assert!(
            opaque(pushing_forces())
                .add_force_at_point(
                    opaque(FREE),
                    opaque(Vec2 { x: ONE, y: -HALF }),
                    opaque(Vec2 { x: TWO, y: ONE }),
                )
                .user_torque != ZERO,
        );
    }

    #[test]
    fn gas_reset_forces() {
        assert_eq!(opaque(pushing_forces()).reset_forces().user_force, Vec2Trait::ZERO);
    }

    #[test]
    fn gas_reset_torques() {
        assert_eq!(opaque(pushing_forces()).reset_torques().user_torque, ZERO);
    }

    /// Forces carrying `force` and `torque`, with no user accumulation yet.
    fn forces_with(force: Vec2, torque: Fixed, gravity_scale: Fixed) -> RigidBodyForces {
        RigidBodyForces {
            force, torque, gravity_scale, user_force: Vec2Trait::ZERO, user_torque: ZERO,
        }
    }

    /// A body with a force and a torque, shared by the probes.
    fn pushing_forces() -> RigidBodyForces {
        forces_with(Vec2 { x: ONE, y: -TWO }, TWO, ONE)
    }
}
