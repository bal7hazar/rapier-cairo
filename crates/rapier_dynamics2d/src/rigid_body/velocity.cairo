//! Linear and angular velocity of a rigid-body (upstream `RigidBodyVelocity`) and the two hot
//! kernels built on it: the symplectic-Euler position update and the impulse application.
//!
//! [`RigidBodyVelocityTrait::integrate`] runs once per body per substep and
//! [`RigidBodyVelocityTrait::apply_impulse_at_point`] once per contact point per iteration, so
//! both are written as fused `fixed::wide` kernels (one rescale per output component) and both
//! keep the composed formulation under `mod alternatives` with a `gas_*` probe.
//!
//! Rotations are advanced with `Rot2::integrate`, i.e. upstream's `integrate_linearized`
//! (`(re - dθ·im, im + dθ·re)` renormalised), not with `sin`/`cos`: the exact update needs
//! trig, which `rapier_math` (M2) does not provide. The renormalisation is M2's and is not optional
//! —
//! the linearisation lengthens the rotation by `dθ²/2` per substep.
//!
//! Deferred: `is_finite` / `from_slice` / `as_vector` (no NaN, no slice view in Cairo), the
//! gyroscopic 3D terms.

use fixed::wide::{dot2, dot3, dot3_add, mul_add, mul_sub};
use fixed::{Fixed, HALF, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_core::rigid_body::{RigidBodyDamping, damping_factor};
use rapier_math::math_ext::scalar::inv;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::{Rot2, Rot2Trait};
use super::mass_props::{RigidBodyMassProps, RigidBodyMassPropsTrait};

/// The velocities of a rigid-body: linear velocity of its centre of mass and angular velocity
/// (counter-clockwise, radians per second). Default: both zero.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct RigidBodyVelocity {
    /// Linear velocity of the centre of mass, in world space.
    pub linvel: Vec2,
    /// Angular velocity, radians per second, positive counter-clockwise.
    pub angvel: Fixed,
}

/// Component-wise sum, as upstream's `Add`.
pub impl RigidBodyVelocityAdd of Add<RigidBodyVelocity> {
    #[inline(always)]
    fn add(lhs: RigidBodyVelocity, rhs: RigidBodyVelocity) -> RigidBodyVelocity {
        RigidBodyVelocity { linvel: lhs.linvel + rhs.linvel, angvel: lhs.angvel + rhs.angvel }
    }
}

/// Component-wise difference, as upstream's `Sub`.
pub impl RigidBodyVelocitySub of Sub<RigidBodyVelocity> {
    #[inline(always)]
    fn sub(lhs: RigidBodyVelocity, rhs: RigidBodyVelocity) -> RigidBodyVelocity {
        RigidBodyVelocity { linvel: lhs.linvel - rhs.linvel, angvel: lhs.angvel - rhs.angvel }
    }
}

/// Velocity kernels of [`RigidBodyVelocity`].
#[generate_trait]
pub impl RigidBodyVelocityImpl of RigidBodyVelocityTrait {
    /// Both velocities exactly zero (upstream `RigidBodyVelocity::zero`).
    #[inline(always)]
    fn zero() -> RigidBodyVelocity {
        RigidBodyVelocity { linvel: Vec2Trait::ZERO, angvel: ZERO }
    }

    /// Stores the velocities as-is; never panics (upstream `RigidBodyVelocity::new`).
    #[inline(always)]
    fn new(linvel: Vec2, angvel: Fixed) -> RigidBodyVelocity {
        RigidBodyVelocity { linvel, angvel }
    }

    /// Returns `true` when both velocities are exactly zero (upstream `is_zero`).
    #[inline(always)]
    fn is_zero(self: RigidBodyVelocity) -> bool {
        self.linvel == Vec2Trait::ZERO && self.angvel == ZERO
    }

    /// Scales both velocities by `rhs`, flooring each component once.
    ///
    /// Mirrors upstream's `Mul<Real> for RigidBodyVelocity`; Cairo's `Mul` needs both operands
    /// to have the same type, hence the `glam`-style `mul_scalar` name.
    /// # Panics
    /// * `'Fixed: overflow'` if a component leaves the scalar range.
    #[inline(always)]
    fn mul_scalar(self: RigidBodyVelocity, rhs: Fixed) -> RigidBodyVelocity {
        RigidBodyVelocity { linvel: self.linvel.mul_scalar(rhs), angvel: self.angvel * rhs }
    }

    /// Returns `self` with its linear part rotated by `rotation` (upstream `transformed`).
    ///
    /// The angular velocity is a scalar in 2D and is left untouched.
    /// # Panics
    /// * `'Fixed: overflow'` if a rotated component leaves the scalar range.
    #[inline(always)]
    fn transformed(self: RigidBodyVelocity, rotation: Rot2) -> RigidBodyVelocity {
        RigidBodyVelocity { linvel: rotation.rotate(self.linvel), angvel: self.angvel }
    }

    /// Advances `position` by `self` over `dt`, rotating about the world centre of mass.
    ///
    /// Symplectic Euler, as upstream: the centre of mass translates by `linvel * dt` while the
    /// pose rotates about it. The rotation goes through `Rot2::integrate` (linearised and
    /// renormalised); the translation is one fused `dot3_add` per component, which keeps the
    /// centre-of-mass correction, the displacement and the previous translation in the same wide
    /// accumulator.
    ///
    /// # Arguments
    /// * `dt` — substep length in seconds.
    /// * `position` — pose at the beginning of the substep, with a unit rotation.
    /// * `local_com` — centre of mass in the local frame; `Vec2::ZERO` for a solver pose, which
    ///   reduces the kernel to `translation + linvel * dt`.
    ///
    /// # Returns
    /// The pose at the end of the substep, with a renormalised rotation.
    ///
    /// # Panics
    /// * `'Fixed: overflow'` if a translation component or the rotation update leaves Q32.32.
    /// * `'Rot2: zero'` if the rotation update cancels exactly (impossible for a unit input).
    ///
    /// # Deviations
    /// * Upstream rotates by the exact angle `angvel * dt`; the linearised update is upstream's
    ///   own `integrate_linearized`, which the substep solver uses for every body.
    fn integrate(self: RigidBodyVelocity, dt: Fixed, position: Pose2, local_com: Vec2) -> Pose2 {
        let rotation = position.rotation.integrate(self.angvel, dt);
        // `(R - R') * local_com` moves the body's origin so that the centre of mass, and not the
        // origin, is the point that only translates by `linvel * dt`.
        let dre = position.rotation.re - rotation.re;
        let dim = position.rotation.im - rotation.im;
        Pose2 {
            translation: Vec2 {
                x: dot3_add(
                    dre, local_com.x, -dim, local_com.y, self.linvel.x, dt, position.translation.x,
                ),
                y: dot3_add(
                    dim, local_com.x, dre, local_com.y, self.linvel.y, dt, position.translation.y,
                ),
            },
            rotation,
        }
    }

    /// Returns the velocities damped over `dt`, i.e. scaled by `1 / (1 + dt * damping)`.
    ///
    /// The factor is `rapier_core`'s [`damping_factor`], one division per part; it is in `(0, 1]`
    /// and never above the exact value (see that module for the rounding).
    /// # Panics
    /// * `'Fixed: overflow'` if `dt * damping` leaves the scalar range.
    #[inline(always)]
    fn apply_damping(
        self: RigidBodyVelocity, dt: Fixed, damping: RigidBodyDamping,
    ) -> RigidBodyVelocity {
        RigidBodyVelocity {
            linvel: self.linvel.mul_scalar(damping_factor(dt, damping.linear_damping)),
            angvel: self.angvel * damping_factor(dt, damping.angular_damping),
        }
    }

    /// Velocity of the world-space `point` of this body: `linvel + angvel x (point - world_com)`.
    ///
    /// One fused `mul_add` per component (the lever arm is exact, the product floors once).
    /// # Panics
    /// * `'i64_sub Overflow'` if the lever arm leaves the scalar range, `'Fixed: overflow'` if
    ///   the velocity does.
    #[inline(always)]
    fn velocity_at_point(self: RigidBodyVelocity, point: Vec2, world_com: Vec2) -> Vec2 {
        let dpt = point - world_com;
        Vec2 {
            x: mul_add(-dpt.y, self.angvel, self.linvel.x),
            y: mul_add(dpt.x, self.angvel, self.linvel.y),
        }
    }

    /// Kinetic energy `(m |v|² + I ω²) / 2`, with the **effective** angular inertia.
    ///
    /// Mirrors upstream: the mass is the body's own (`local_mprops.inv_mass` inverted), the
    /// inertia the effective one, so a rotation-locked body contributes no angular energy
    /// (`inv(0) = 0` replaces upstream's explicit test). The halving floors, which for a
    /// non-negative energy is the truncation upstream performs.
    /// # Panics
    /// * `'Fixed: overflow'` if a squared velocity or the energy leaves the scalar range.
    fn kinetic_energy(self: RigidBodyVelocity, mprops: RigidBodyMassProps) -> Fixed {
        let inertia = inv(mprops.effective_world_inv_inertia);
        dot2(mprops.mass(), self.linvel.length_squared(), inertia * self.angvel, self.angvel) * HALF
    }

    /// Mass-normalised kinetic energy `(|v|² + ω²) / 2` (upstream `pseudo_kinetic_energy`), the
    /// quantity the sleeping heuristic compares against a threshold.
    ///
    /// The three squares accumulate wide and floor once.
    /// # Panics
    /// * `'Fixed: overflow'` if the sum of squares leaves the scalar range.
    #[inline(always)]
    fn pseudo_kinetic_energy(self: RigidBodyVelocity) -> Fixed {
        dot3(self.linvel.x, self.linvel.x, self.linvel.y, self.linvel.y, self.angvel, self.angvel)
            * HALF
    }

    /// Applies an impulse at the centre of mass: `linvel += impulse * effective_inv_mass`.
    ///
    /// A locked axis or a non-dynamic body has a zero inverse mass and is therefore left alone,
    /// which is upstream's "does nothing on non-dynamic bodies".
    /// # Panics
    /// * `'Fixed: overflow'` if the new velocity leaves the scalar range.
    #[inline(always)]
    fn apply_impulse(
        self: RigidBodyVelocity, mprops: RigidBodyMassProps, impulse: Vec2,
    ) -> RigidBodyVelocity {
        RigidBodyVelocity {
            linvel: Vec2 {
                x: mul_add(impulse.x, mprops.effective_inv_mass.x, self.linvel.x),
                y: mul_add(impulse.y, mprops.effective_inv_mass.y, self.linvel.y),
            },
            angvel: self.angvel,
        }
    }

    /// Applies an angular impulse: `angvel += effective_world_inv_inertia * torque_impulse`.
    ///
    /// # Panics
    /// * `'Fixed: overflow'` if the new angular velocity leaves the scalar range.
    #[inline(always)]
    fn apply_torque_impulse(
        self: RigidBodyVelocity, mprops: RigidBodyMassProps, torque_impulse: Fixed,
    ) -> RigidBodyVelocity {
        RigidBodyVelocity {
            linvel: self.linvel,
            angvel: mul_add(mprops.effective_world_inv_inertia, torque_impulse, self.angvel),
        }
    }

    /// Applies an impulse at the world-space `point`, changing both velocities.
    ///
    /// The torque impulse is the lever arm crossed with the impulse,
    /// `gcross(point - world_com, impulse)`, kept wide until the angular update floors it.
    /// # Panics
    /// * `'i64_sub Overflow'` if the lever arm leaves the scalar range, `'Fixed: overflow'` if a
    ///   velocity does.
    fn apply_impulse_at_point(
        self: RigidBodyVelocity, mprops: RigidBodyMassProps, impulse: Vec2, point: Vec2,
    ) -> RigidBodyVelocity {
        let dpt = point - mprops.world_com;
        RigidBodyVelocity {
            linvel: Vec2 {
                x: mul_add(impulse.x, mprops.effective_inv_mass.x, self.linvel.x),
                y: mul_add(impulse.y, mprops.effective_inv_mass.y, self.linvel.y),
            },
            angvel: mul_add(
                mul_sub(dpt.x, impulse.y, dpt.y, impulse.x),
                mprops.effective_world_inv_inertia,
                self.angvel,
            ),
        }
    }
}

#[cfg(test)]
mod alternatives {
    use fixed::{Fixed, HALF};
    use glam::{Vec2, Vec2Trait};
    use rapier_math::math_ext::scalar::inv;
    use rapier_math::math_ext::vec2::{gcross_sv, gcross_vv};
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2Trait;
    use super::RigidBodyVelocity;
    use super::super::mass_props::{RigidBodyMassProps, RigidBodyMassPropsTrait};

    /// Upstream's composition, kernel by kernel: move the origin to the centre of mass, rotate,
    /// move back and translate. Three rounded intermediate vectors instead of one wide sum.
    pub fn integrate(v: RigidBodyVelocity, dt: Fixed, position: Pose2, local_com: Vec2) -> Pose2 {
        let com = position.transform_point(local_com);
        let rotation = position.rotation.integrate(v.angvel, dt);
        let delta = Rot2Trait::mul(rotation, position.rotation.inverse());
        Pose2 {
            translation: delta.rotate(position.translation - com) + com + v.linvel.mul_scalar(dt),
            rotation,
        }
    }

    /// `linvel + gcross(angvel, point - world_com)` through the `rapier_math` helpers: each
    /// product floors before the vector sum.
    pub fn velocity_at_point(v: RigidBodyVelocity, point: Vec2, world_com: Vec2) -> Vec2 {
        let dpt = point - world_com;
        let (x, y) = gcross_sv(v.angvel, dpt.x, dpt.y);
        v.linvel + Vec2 { x, y }
    }

    /// `apply_impulse` then `apply_torque_impulse`, with a rounded torque impulse in between.
    pub fn apply_impulse_at_point(
        v: RigidBodyVelocity, mprops: RigidBodyMassProps, impulse: Vec2, point: Vec2,
    ) -> RigidBodyVelocity {
        let dpt = point - mprops.world_com;
        let torque_impulse = gcross_vv(dpt.x, dpt.y, impulse.x, impulse.y);
        RigidBodyVelocity {
            linvel: v.linvel + impulse * mprops.effective_inv_mass,
            angvel: v.angvel + mprops.effective_world_inv_inertia * torque_impulse,
        }
    }

    /// Two halved terms instead of one halved wide sum.
    pub fn kinetic_energy(v: RigidBodyVelocity, mprops: RigidBodyMassProps) -> Fixed {
        let linear = mprops.mass() * v.linvel.length_squared() * HALF;
        linear + inv(mprops.effective_world_inv_inertia) * v.angvel * v.angvel * HALF
    }

    /// `|v|²` through `glam` then the angular square, each floored before the sum.
    pub fn pseudo_kinetic_energy(v: RigidBodyVelocity) -> Fixed {
        (v.linvel.length_squared() + v.angvel * v.angvel) * HALF
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::{Vec2, Vec2Trait};
    use rapier_core::rigid_body::{RigidBodyDamping, RigidBodyType, damping_factor};
    use rapier_geometry2d::mass::MassProperties;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::{Rot2, Rot2Trait};
    use rapier_testing::opaque;
    use super::super::locked_axes::{
        LockedAxes, LockedAxesTrait, ROTATION_LOCKED, TRANSLATION_LOCKED_X,
    };
    use super::super::mass_props::{RigidBodyMassProps, RigidBodyMassPropsTrait};
    use super::{RigidBodyVelocity, RigidBodyVelocityTrait, alternatives};

    /// `1 / 60` rounded to nearest, as `IntegrationParameters::default().dt`.
    const DT: Fixed = Fixed { raw: 71582788 };
    /// Mass `0.5`, angular inertia `2`, centre of mass at `(1, 0.5)` in the local frame.
    const LOCAL: MassProperties = MassProperties {
        local_com: Vec2 { x: ONE, y: HALF }, inv_mass: TWO, inv_principal_inertia: HALF,
    };
    /// Pose rotated by a quarter turn and translated by `(1, 2)`.
    const POSE: Pose2 = Pose2 {
        translation: Vec2 { x: ONE, y: TWO }, rotation: Rot2 { re: ZERO, im: ONE },
    };
    const V: RigidBodyVelocity = RigidBodyVelocity {
        linvel: Vec2 { x: HALF, y: Fixed { raw: -8589934592 } }, angvel: ONE,
    };
    /// [`LOCAL`] updated for [`POSE`] as a free dynamic body; `mprops()` rebuilds it and
    /// `test_apply_impulse_and_torque_impulse` checks that the two agree.
    const MPROPS: RigidBodyMassProps = RigidBodyMassProps {
        flags: LockedAxes { bits: 0 },
        local_mprops: LOCAL,
        world_com: Vec2 { x: HALF, y: Fixed { raw: 12884901888 } },
        effective_inv_mass: Vec2 { x: TWO, y: TWO },
        effective_world_inv_inertia: HALF,
    };

    fn mprops() -> RigidBodyMassProps {
        RigidBodyMassPropsTrait::from_local(LOCAL, LockedAxesTrait::empty())
            .update_world_mass_properties(RigidBodyType::Dynamic, POSE)
    }

    fn close(a: Vec2, b: Vec2, ulps: i64) {
        assert!(a.x.abs_diff_eq(b.x, Fixed { raw: ulps }), "x {:?} vs {:?}", a.x, b.x);
        assert!(a.y.abs_diff_eq(b.y, Fixed { raw: ulps }), "y {:?} vs {:?}", a.y, b.y);
    }
    fn close_scalar(a: Fixed, b: Fixed, ulps: i64) {
        assert!(a.abs_diff_eq(b, Fixed { raw: ulps }), "{:?} vs {:?}", a, b);
    }
    fn close_pose(a: Pose2, b: Pose2, ulps: i64) {
        close(a.translation, b.translation, ulps);
        close(
            Vec2 { x: a.rotation.re, y: a.rotation.im },
            Vec2 { x: b.rotation.re, y: b.rotation.im },
            ulps,
        );
    }

    #[test]
    fn test_zero_new_and_algebra() {
        let zero = RigidBodyVelocityTrait::zero();
        assert_eq!(zero, Default::<RigidBodyVelocity>::default());
        assert!(zero.is_zero());
        assert!(!V.is_zero());
        assert!(!RigidBodyVelocityTrait::new(Vec2Trait::ZERO, ONE).is_zero());
        assert!(!RigidBodyVelocityTrait::new(Vec2 { x: ONE, y: ZERO }, ZERO).is_zero());
        assert_eq!(RigidBodyVelocityTrait::new(V.linvel, V.angvel), V);
        assert_eq!(V + zero, V);
        assert_eq!(V - V, zero);
        assert_eq!((V + V).angvel, TWO);
        assert_eq!(V.mul_scalar(TWO), V + V);
        assert_eq!(V.mul_scalar(ZERO), zero);
        // The angular velocity is a scalar: only the linear part rotates.
        let turned = V.transformed(Rot2 { re: ZERO, im: ONE });
        assert_eq!(turned.angvel, V.angvel);
        assert_eq!(turned.linvel, Vec2 { x: TWO, y: HALF });
    }

    /// Free translation, free rotation and the combination, against the composed formulation.
    #[test]
    fn test_integrate_against_composed_port() {
        for (linvel, angvel, local_com) in array![
            (Vec2Trait::ZERO, ZERO, Vec2Trait::ZERO),
            (Vec2 { x: HALF, y: -TWO }, ZERO, Vec2Trait::ZERO),
            (Vec2Trait::ZERO, ONE, Vec2Trait::ZERO),
            (Vec2Trait::ZERO, ONE, Vec2 { x: ONE, y: HALF }),
            (Vec2 { x: HALF, y: -TWO }, -TWO, Vec2 { x: -ONE, y: HALF }),
        ]
            .span() {
            let v = RigidBodyVelocityTrait::new(*linvel, *angvel);
            let actual = v.integrate(DT, POSE, *local_com);
            close_pose(actual, alternatives::integrate(v, DT, POSE, *local_com), 4);
            assert!(actual.rotation.is_unit());
            // Whatever the local centre of mass, the world one only translates by `linvel * dt`.
            let com = POSE.transform_point(*local_com);
            let moved = actual.transform_point(*local_com);
            close(moved, com + (*linvel).mul_scalar(DT), 4);
        }
        // A body at rest keeps its pose exactly.
        assert_eq!(
            RigidBodyVelocityTrait::zero().integrate(DT, POSE, Vec2 { x: ONE, y: HALF }), POSE,
        );
        // With a zero local centre of mass the kernel is the plain displacement.
        let straight = RigidBodyVelocityTrait::new(V.linvel, ZERO)
            .integrate(DT, POSE, Vec2Trait::ZERO);
        assert_eq!(straight.translation, POSE.translation + V.linvel.mul_scalar(DT));
        assert_eq!(straight.rotation, POSE.rotation);
    }

    #[test]
    fn test_apply_impulse_and_torque_impulse() {
        let mprops = mprops();
        assert_eq!(mprops, MPROPS);
        assert_eq!(mprops.effective_inv_mass, Vec2 { x: TWO, y: TWO });
        assert_eq!(mprops.effective_world_inv_inertia, HALF);
        let impulse = Vec2 { x: ONE, y: -HALF };
        // Linear: `linvel + impulse * inv_mass`, angular untouched.
        let pushed = V.apply_impulse(mprops, impulse);
        assert_eq!(pushed.linvel, Vec2 { x: V.linvel.x + TWO, y: V.linvel.y - ONE });
        assert_eq!(pushed.angvel, V.angvel);
        // Angular: `angvel + inv_inertia * torque`, linear untouched.
        let spun = V.apply_torque_impulse(mprops, TWO);
        assert_eq!(spun.angvel, V.angvel + ONE);
        assert_eq!(spun.linvel, V.linvel);
        // At a point: both, with the torque impulse `gcross(point - world_com, impulse)`.
        let point = mprops.world_com + Vec2 { x: ONE, y: TWO };
        let at_point = V.apply_impulse_at_point(mprops, impulse, point);
        assert_eq!(at_point.linvel, pushed.linvel);
        let torque = ONE * -HALF - TWO * ONE;
        close_scalar(at_point.angvel, V.angvel + mprops.effective_world_inv_inertia * torque, 2);
        close_scalar(
            at_point.angvel,
            alternatives::apply_impulse_at_point(V, mprops, impulse, point).angvel,
            2,
        );
        // An impulse through the centre of mass generates no torque.
        assert_eq!(V.apply_impulse_at_point(mprops, impulse, mprops.world_com), pushed);
    }

    /// A locked axis has a zero inverse, so the corresponding impulse response vanishes.
    #[test]
    fn test_locked_axes_absorb_impulses() {
        let locked = RigidBodyMassPropsTrait::from_local(
            LOCAL, TRANSLATION_LOCKED_X | ROTATION_LOCKED,
        )
            .update_world_mass_properties(RigidBodyType::Dynamic, POSE);
        let impulse = Vec2 { x: ONE, y: -HALF };
        let point = locked.world_com + Vec2 { x: ONE, y: TWO };
        let after = V.apply_impulse_at_point(locked, impulse, point);
        assert_eq!(after.linvel.x, V.linvel.x);
        assert_eq!(after.linvel.y, V.linvel.y - ONE);
        assert_eq!(after.angvel, V.angvel);
        // A fixed body absorbs everything.
        let fixed = RigidBodyMassPropsTrait::from_local(LOCAL, LockedAxesTrait::empty())
            .update_world_mass_properties(RigidBodyType::Fixed, POSE);
        assert_eq!(V.apply_impulse_at_point(fixed, impulse, point), V);
        assert_eq!(V.apply_impulse(fixed, impulse), V);
        assert_eq!(V.apply_torque_impulse(fixed, TWO), V);
    }

    #[test]
    fn test_velocity_at_point_and_damping() {
        let mprops = mprops();
        // At the centre of mass the point velocity is the linear velocity.
        assert_eq!(V.velocity_at_point(mprops.world_com, mprops.world_com), V.linvel);
        for arm in array![
            Vec2 { x: ONE, y: ZERO }, Vec2 { x: ZERO, y: -TWO }, Vec2 { x: -HALF, y: HALF },
        ]
            .span() {
            let arm = *arm;
            let point = mprops.world_com + arm;
            let actual = V.velocity_at_point(point, mprops.world_com);
            close(actual, alternatives::velocity_at_point(V, point, mprops.world_com), 2);
            // `omega x r` is perpendicular to the lever arm and scales with `angvel`.
            let perp = Vec2 { x: -arm.y, y: arm.x };
            close(actual - V.linvel, perp.mul_scalar(V.angvel), 2);
        }
        // Damping scales each part by `1 / (1 + dt * damping)`.
        let damping = RigidBodyDamping { linear_damping: ONE, angular_damping: TWO };
        let damped = V.apply_damping(DT, damping);
        assert_eq!(damped.linvel, V.linvel.mul_scalar(damping_factor(DT, ONE)));
        assert_eq!(damped.angvel, V.angvel * damping_factor(DT, TWO));
        assert!(damped.angvel < V.angvel);
        let none: RigidBodyDamping = Default::default();
        assert_eq!(V.apply_damping(DT, none), V);
        assert!(RigidBodyVelocityTrait::zero().apply_damping(DT, damping).is_zero());
    }

    #[test]
    fn test_kinetic_energies() {
        let mprops = mprops();
        // mass 0.5, |v|² = 0.25 + 4 = 4.25, inertia 2, ω² = 1: (0.5 * 4.25 + 2) / 2 = 2.0625.
        let expected = Fixed { raw: 8858370048 };
        close_scalar(V.kinetic_energy(mprops), expected, 2);
        close_scalar(V.kinetic_energy(mprops), alternatives::kinetic_energy(V, mprops), 2);
        // (4.25 + 1) / 2 = 2.625.
        close_scalar(V.pseudo_kinetic_energy(), Fixed { raw: 11274289152 }, 2);
        close_scalar(V.pseudo_kinetic_energy(), alternatives::pseudo_kinetic_energy(V), 2);
        // A body at rest has no energy of either kind.
        let zero = RigidBodyVelocityTrait::zero();
        assert_eq!(zero.kinetic_energy(mprops), ZERO);
        assert_eq!(zero.pseudo_kinetic_energy(), ZERO);
        // A rotation-locked body has no angular energy, but the pseudo-energy ignores the lock.
        let locked = RigidBodyMassPropsTrait::from_local(LOCAL, ROTATION_LOCKED)
            .update_world_mass_properties(RigidBodyType::Dynamic, POSE);
        close_scalar(V.kinetic_energy(locked), Fixed { raw: 4563402752 }, 2);
        // An infinite-mass body stores no linear energy either.
        let massless: RigidBodyMassProps = Default::default();
        assert_eq!(V.kinetic_energy(massless), ZERO);
    }

    /// Products of these raws stay in Q32.32; both kernels answer within their rounding.
    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_integrate_candidates(vx: i32, vy: i32, w: i32, cx: i32) {
        let v = RigidBodyVelocityTrait::new(
            Vec2 { x: Fixed { raw: vx.into() }, y: Fixed { raw: vy.into() } },
            Fixed { raw: w.into() },
        );
        let local_com = Vec2 { x: Fixed { raw: cx.into() }, y: HALF };
        let actual = v.integrate(DT, POSE, local_com);
        close_pose(actual, alternatives::integrate(v, DT, POSE, local_com), 4);
        assert!(actual.rotation.is_unit());
    }

    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_impulse_candidates(ix: i32, iy: i32, px: i32, py: i32) {
        let mprops = mprops();
        let impulse = Vec2 { x: Fixed { raw: ix.into() }, y: Fixed { raw: iy.into() } };
        let point = Vec2 { x: Fixed { raw: px.into() }, y: Fixed { raw: py.into() } };
        let actual = V.apply_impulse_at_point(mprops, impulse, point);
        let candidate = alternatives::apply_impulse_at_point(V, mprops, impulse, point);
        close(actual.linvel, candidate.linvel, 1);
        close_scalar(actual.angvel, candidate.angvel, 2);
        close(actual.linvel, V.apply_impulse(mprops, impulse).linvel, 0);
        close(
            V.velocity_at_point(point, mprops.world_com),
            alternatives::velocity_at_point(V, point, mprops.world_com),
            2,
        );
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_zero() {
        assert!(RigidBodyVelocityTrait::zero().is_zero());
    }

    #[test]
    fn gas_is_zero() {
        assert!(!opaque(V).is_zero());
    }

    #[test]
    fn gas_mul_scalar() {
        assert_eq!(opaque(V).mul_scalar(opaque(TWO)).angvel, TWO);
    }

    #[test]
    fn gas_add() {
        assert_eq!((opaque(V) + opaque(V)).angvel, TWO);
    }

    #[test]
    fn gas_transformed() {
        assert_eq!(opaque(V).transformed(opaque(Rot2 { re: ZERO, im: ONE })).angvel, ONE);
    }

    #[test]
    fn gas_integrate_fused() {
        assert!(
            opaque(V)
                .integrate(opaque(DT), opaque(POSE), opaque(LOCAL.local_com))
                .rotation
                .is_unit(),
        );
    }

    #[test]
    fn gas_integrate_composed() {
        assert!(
            alternatives::integrate(opaque(V), opaque(DT), opaque(POSE), opaque(LOCAL.local_com))
                .rotation
                .is_unit(),
        );
    }

    #[test]
    fn gas_integrate_fused_zero_com() {
        assert!(
            opaque(V)
                .integrate(opaque(DT), opaque(POSE), opaque(Vec2Trait::ZERO))
                .rotation
                .is_unit(),
        );
    }

    #[test]
    fn gas_integrate_composed_zero_com() {
        assert!(
            alternatives::integrate(opaque(V), opaque(DT), opaque(POSE), opaque(Vec2Trait::ZERO))
                .rotation
                .is_unit(),
        );
    }

    #[test]
    fn gas_apply_damping() {
        assert!(
            opaque(V)
                .apply_damping(
                    opaque(DT),
                    opaque(RigidBodyDamping { linear_damping: ONE, angular_damping: TWO }),
                )
                .angvel < ONE,
        );
    }

    #[test]
    fn gas_velocity_at_point_fused() {
        assert!(
            opaque(V)
                .velocity_at_point(
                    opaque(Vec2 { x: TWO, y: ONE }), opaque(POSE.translation),
                ) != Vec2Trait::ZERO,
        );
    }

    #[test]
    fn gas_velocity_at_point_composed() {
        assert!(
            alternatives::velocity_at_point(
                opaque(V), opaque(Vec2 { x: TWO, y: ONE }), opaque(POSE.translation),
            ) != Vec2Trait::ZERO,
        );
    }

    #[test]
    fn gas_kinetic_energy_fused() {
        assert!(opaque(V).kinetic_energy(opaque(MPROPS)) > ZERO);
    }

    #[test]
    fn gas_kinetic_energy_composed() {
        assert!(alternatives::kinetic_energy(opaque(V), opaque(MPROPS)) > ZERO);
    }

    #[test]
    fn gas_pseudo_kinetic_energy_fused() {
        assert!(opaque(V).pseudo_kinetic_energy() > ZERO);
    }

    #[test]
    fn gas_pseudo_kinetic_energy_composed() {
        assert!(alternatives::pseudo_kinetic_energy(opaque(V)) > ZERO);
    }

    #[test]
    fn gas_apply_impulse() {
        assert!(
            opaque(V)
                .apply_impulse(opaque(MPROPS), opaque(Vec2 { x: ONE, y: -HALF }))
                .angvel == ONE,
        );
    }

    #[test]
    fn gas_apply_torque_impulse() {
        assert_eq!(opaque(V).apply_torque_impulse(opaque(MPROPS), opaque(TWO)).angvel, TWO);
    }

    #[test]
    fn gas_apply_impulse_at_point_fused() {
        assert!(
            opaque(V)
                .apply_impulse_at_point(
                    opaque(MPROPS),
                    opaque(Vec2 { x: ONE, y: -HALF }),
                    opaque(Vec2 { x: TWO, y: ONE }),
                )
                .angvel != ZERO,
        );
    }

    #[test]
    fn gas_apply_impulse_at_point_composed() {
        assert!(
            alternatives::apply_impulse_at_point(
                opaque(V),
                opaque(MPROPS),
                opaque(Vec2 { x: ONE, y: -HALF }),
                opaque(Vec2 { x: TWO, y: ONE }),
            )
                .angvel != ZERO,
        );
    }
}
