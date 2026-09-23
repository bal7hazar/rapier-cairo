//! World-space mass properties of a rigid-body (upstream `RigidBodyMassProps`).
//!
//! The local properties come from the colliders (package GB, `rapier_geometry2d::mass`); this
//! module only derives their world-space counterpart for the current pose and cancels the
//! components the body type or [`LockedAxes`] forbid. Every inverse goes through
//! `rapier_math::inv` (`inv(0) = 0`), which is what makes an infinite-mass body work without a
//! special case: a fixed body simply has zero inverse mass and zero inverse inertia.
//!
//! In 2D the world inverse angular inertia is the local one (`MassProperties::world_inv_inertia`
//! ignores the rotation), so `update_world_mass_properties` performs **no division at all**: the
//! inverses are stored, never recomputed, and the solver multiplies by them.
//!
//! Deferred: `additional_local_mprops` and `recompute_mass_properties_from_colliders` (they need
//! the collider set, package DD), `max_extent` (CCD and sleeping, out of the MVP).

use fixed::{Fixed, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_core::rigid_body::{RigidBodyType, RigidBodyTypeTrait};
use rapier_geometry2d::mass::MassProperties;
use rapier_math::math_ext::scalar::inv;
use rapier_math::pose2::{Pose2, Pose2Trait};
use super::locked_axes::{
    LockedAxes, LockedAxesTrait, ROTATION_LOCKED, TRANSLATION_LOCKED_X, TRANSLATION_LOCKED_Y,
};

/// Mass properties of a rigid-body: the local ones, their world-space projection and the locked
/// axes. The `effective_*` fields are **inverses**, zero when the corresponding axis is locked,
/// and are only meaningful after [`RigidBodyMassPropsTrait::update_world_mass_properties`].
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct RigidBodyMassProps {
    /// Translation and rotation axes the body may not move along.
    pub flags: LockedAxes,
    /// Mass properties in the local frame of the body (centre of mass, inverse mass, inverse
    /// principal angular inertia).
    pub local_mprops: MassProperties,
    /// World-space centre of mass, `position * local_mprops.local_com`.
    pub world_com: Vec2,
    /// Inverse mass per world axis, zeroed on a locked axis or a non-dynamic body.
    pub effective_inv_mass: Vec2,
    /// World-space inverse angular inertia, zeroed when the rotation is locked or the body is
    /// not dynamic. Equal to `local_mprops.inv_principal_inertia` otherwise (2D).
    pub effective_world_inv_inertia: Fixed,
}

/// Accessors and world-space update of [`RigidBodyMassProps`].
#[generate_trait]
pub impl RigidBodyMassPropsImpl of RigidBodyMassPropsTrait {
    /// Wraps local mass properties and a locked-axes mask; the world-space fields stay zero
    /// until [`Self::update_world_mass_properties`] runs.
    ///
    /// Mirrors upstream's `From<MassProperties>` and `From<LockedAxes>` constructors, merged.
    /// # Panics
    /// * Never.
    #[inline(always)]
    fn from_local(local_mprops: MassProperties, flags: LockedAxes) -> RigidBodyMassProps {
        RigidBodyMassProps {
            flags,
            local_mprops,
            world_com: Vec2Trait::ZERO,
            effective_inv_mass: Vec2Trait::ZERO,
            effective_world_inv_inertia: ZERO,
        }
    }

    /// The mass of the body, `inv(local_mprops.inv_mass)`; `0` for an infinite mass.
    ///
    /// # Returns
    /// A non-negative mass, rounded to nearest (`Fixed::recip`).
    /// # Panics
    /// * `'Fixed: overflow'` if the inverse mass is below `2^-31`, i.e. the mass above `2^31`.
    #[inline(always)]
    fn mass(self: RigidBodyMassProps) -> Fixed {
        inv(self.local_mprops.inv_mass)
    }

    /// The effective mass per world axis, `inv` of [`RigidBodyMassProps::effective_inv_mass`];
    /// a locked axis (or a non-dynamic body) reports `0`, i.e. an infinite mass.
    ///
    /// Upstream feeds this to `compute_effective_force_and_torque`, so that gravity produces the
    /// same acceleration whatever the mass, and none at all on a locked axis.
    /// # Panics
    /// * `'Fixed: overflow'` if a component is below `2^-31`.
    #[inline(always)]
    fn effective_mass(self: RigidBodyMassProps) -> Vec2 {
        Vec2 { x: inv(self.effective_inv_mass.x), y: inv(self.effective_inv_mass.y) }
    }

    /// The effective angular inertia, `inv(effective_world_inv_inertia)`; `0` when the rotation
    /// is locked or the body is not dynamic.
    ///
    /// # Panics
    /// * `'Fixed: overflow'` if the inverse inertia is below `2^-31`.
    #[inline(always)]
    fn effective_angular_inertia(self: RigidBodyMassProps) -> Fixed {
        inv(self.effective_world_inv_inertia)
    }

    /// Returns `self` with `world_com`, `effective_inv_mass` and `effective_world_inv_inertia`
    /// recomputed for `position` and `body_type`.
    ///
    /// Call it after every change of pose, of local mass properties, of body type or of locked
    /// axes: the solver and `apply_torque_impulse` read the world-space fields.
    ///
    /// # Arguments
    /// * `body_type` — a non-dynamic body gets zero inverse mass and inertia on every axis.
    /// * `position` — the world pose of the body.
    ///
    /// # Returns
    /// The updated properties; the local ones and the flags are untouched.
    ///
    /// # Panics
    /// * `'Fixed: overflow'` if the world centre of mass leaves the scalar range.
    ///
    /// # Deviations
    /// * The brief spells this `update_world_mass_properties(position)`; upstream also takes the
    ///   body type, without which a fixed body would keep a non-zero inverse mass. Upstream's
    ///   argument order is kept.
    /// * Upstream mutates in place; here the updated value is returned (no `ref` in the
    ///   component API, see `docs/PLAN.md` D9).
    fn update_world_mass_properties(
        self: RigidBodyMassProps, body_type: RigidBodyType, position: Pose2,
    ) -> RigidBodyMassProps {
        // A non-dynamic body locks every axis at once: folding the body type into the mask costs
        // one `insert` and saves two of the three `is_dynamic` tests of the direct port
        // (`alternatives::update_world_mass_properties`, 1 030 gas dearer).
        let mut locked = self.flags;
        if !body_type.is_dynamic() {
            locked.insert(LockedAxesTrait::all());
        }
        let inv_mass = self.local_mprops.inv_mass;
        RigidBodyMassProps {
            flags: self.flags,
            local_mprops: self.local_mprops,
            world_com: position.transform_point(self.local_mprops.local_com),
            effective_inv_mass: Vec2 {
                x: if locked.contains(TRANSLATION_LOCKED_X) {
                    ZERO
                } else {
                    inv_mass
                },
                y: if locked.contains(TRANSLATION_LOCKED_Y) {
                    ZERO
                } else {
                    inv_mass
                },
            },
            effective_world_inv_inertia: if locked.contains(ROTATION_LOCKED) {
                ZERO
            } else {
                self.local_mprops.inv_principal_inertia
            },
        }
    }
}

#[cfg(test)]
mod alternatives {
    use fixed::ZERO;
    use glam::Vec2;
    use rapier_core::rigid_body::{RigidBodyType, RigidBodyTypeTrait};
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use super::RigidBodyMassProps;
    use super::super::locked_axes::{
        LockedAxesTrait, ROTATION_LOCKED, TRANSLATION_LOCKED_X, TRANSLATION_LOCKED_Y,
    };


    /// The direct port: upstream's `!body_type.is_dynamic() || flags.contains(..)` per component.
    pub fn update_world_mass_properties(
        props: RigidBodyMassProps, body_type: RigidBodyType, position: Pose2,
    ) -> RigidBodyMassProps {
        let movable = body_type.is_dynamic();
        let inv_mass = props.local_mprops.inv_mass;
        RigidBodyMassProps {
            flags: props.flags,
            local_mprops: props.local_mprops,
            world_com: position.transform_point(props.local_mprops.local_com),
            effective_inv_mass: Vec2 {
                x: if movable && !props.flags.contains(TRANSLATION_LOCKED_X) {
                    inv_mass
                } else {
                    ZERO
                },
                y: if movable && !props.flags.contains(TRANSLATION_LOCKED_Y) {
                    inv_mass
                } else {
                    ZERO
                },
            },
            effective_world_inv_inertia: if movable && !props.flags.contains(ROTATION_LOCKED) {
                props.local_mprops.inv_principal_inertia
            } else {
                ZERO
            },
        }
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
        LockedAxes, LockedAxesTrait, ROTATION_LOCKED, TRANSLATION_LOCKED, TRANSLATION_LOCKED_X,
        TRANSLATION_LOCKED_Y,
    };
    use super::{RigidBodyMassProps, RigidBodyMassPropsTrait, alternatives};

    /// A unit-mass, unit-inertia body whose centre of mass is off the origin.
    const LOCAL: MassProperties = MassProperties {
        local_com: Vec2 { x: ONE, y: HALF }, inv_mass: TWO, inv_principal_inertia: HALF,
    };
    /// Quarter turn about the origin, translated by (1, 2).
    const POSE: Pose2 = Pose2 {
        translation: Vec2 { x: ONE, y: TWO }, rotation: Rot2 { re: ZERO, im: ONE },
    };
    /// [`LOCAL`] updated for [`POSE`] as a free dynamic body; checked in
    /// `test_world_com_and_free_body` and used as-is by the accessor probes.
    const UPDATED: RigidBodyMassProps = RigidBodyMassProps {
        flags: LockedAxes { bits: 0 },
        local_mprops: LOCAL,
        world_com: Vec2 { x: HALF, y: Fixed { raw: 12884901888 } },
        effective_inv_mass: Vec2 { x: TWO, y: TWO },
        effective_world_inv_inertia: HALF,
    };

    fn updated(flags: LockedAxes, body_type: RigidBodyType) -> RigidBodyMassProps {
        let props = RigidBodyMassPropsTrait::from_local(LOCAL, flags);
        let actual = props.update_world_mass_properties(body_type, POSE);
        assert_eq!(actual, alternatives::update_world_mass_properties(props, body_type, POSE));
        actual
    }

    #[test]
    fn test_from_local_leaves_world_fields_zero() {
        let props = RigidBodyMassPropsTrait::from_local(LOCAL, ROTATION_LOCKED);
        assert_eq!(props.local_mprops, LOCAL);
        assert_eq!(props.flags, ROTATION_LOCKED);
        assert_eq!(props.world_com, Vec2Trait::ZERO);
        assert_eq!(props.effective_inv_mass, Vec2Trait::ZERO);
        assert_eq!(props.effective_world_inv_inertia, ZERO);
        let default: RigidBodyMassProps = Default::default();
        assert_eq!(default.flags, LockedAxesTrait::empty());
        assert_eq!(default.mass(), ZERO);
        assert_eq!(default.effective_mass(), Vec2Trait::ZERO);
        assert_eq!(default.effective_angular_inertia(), ZERO);
    }

    /// The world centre of mass is the pose applied to the local one: the quarter turn sends
    /// `(1, 0.5)` to `(-0.5, 1)`, then the translation `(1, 2)` gives `(0.5, 3)`.
    #[test]
    fn test_world_com_and_free_body() {
        let props = updated(LockedAxesTrait::empty(), RigidBodyType::Dynamic);
        assert_eq!(props, UPDATED);
        assert_eq!(props.world_com, Vec2 { x: HALF, y: FixedTrait::from_int(3) });
        assert_eq!(props.effective_inv_mass, Vec2 { x: TWO, y: TWO });
        assert_eq!(props.effective_world_inv_inertia, HALF);
        assert_eq!(props.mass(), HALF);
        assert_eq!(props.effective_mass(), Vec2 { x: HALF, y: HALF });
        assert_eq!(props.effective_angular_inertia(), TWO);
    }

    #[test]
    fn test_locked_axes_zero_the_matching_components() {
        for (flags, x, y, inertia) in array![
            (TRANSLATION_LOCKED_X, ZERO, TWO, HALF), (TRANSLATION_LOCKED_Y, TWO, ZERO, HALF),
            (TRANSLATION_LOCKED, ZERO, ZERO, HALF), (ROTATION_LOCKED, TWO, TWO, ZERO),
            (LockedAxesTrait::all(), ZERO, ZERO, ZERO),
        ]
            .span() {
            let props = updated(*flags, RigidBodyType::Dynamic);
            assert_eq!(props.effective_inv_mass, Vec2 { x: *x, y: *y });
            assert_eq!(props.effective_world_inv_inertia, *inertia);
            // A zero inverse is an infinite mass / inertia, reported as zero by `inv`.
            assert_eq!(props.effective_mass().x, if *x == ZERO {
                ZERO
            } else {
                HALF
            });
            // The local mass is never affected by the locks.
            assert_eq!(props.mass(), HALF);
        }
    }

    #[test]
    fn test_non_dynamic_bodies_have_no_effective_inverse() {
        for body_type in array![
            RigidBodyType::Fixed, RigidBodyType::KinematicPositionBased,
            RigidBodyType::KinematicVelocityBased,
        ]
            .span() {
            let props = updated(LockedAxesTrait::empty(), *body_type);
            assert_eq!(props.effective_inv_mass, Vec2Trait::ZERO);
            assert_eq!(props.effective_world_inv_inertia, ZERO);
            // The world centre of mass is still tracked: contacts need it.
            assert_eq!(props.world_com, Vec2 { x: HALF, y: FixedTrait::from_int(3) });
        }
    }

    /// `inv(0) = 0` both ways: an infinite-mass dynamic body keeps zero inverses.
    #[test]
    fn test_infinite_mass_body() {
        let zeroed = MassProperties {
            local_com: Vec2Trait::ZERO, inv_mass: ZERO, inv_principal_inertia: ZERO,
        };
        let props = RigidBodyMassPropsTrait::from_local(zeroed, LockedAxesTrait::empty())
            .update_world_mass_properties(RigidBodyType::Dynamic, POSE);
        assert_eq!(props.effective_inv_mass, Vec2Trait::ZERO);
        assert_eq!(props.mass(), ZERO);
        assert_eq!(props.effective_mass(), Vec2Trait::ZERO);
        assert_eq!(props.effective_angular_inertia(), ZERO);
        assert_eq!(props.world_com, POSE.translation);
    }

    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_update_matches_the_direct_port(mass: u32, inertia: u32, bits: u8) {
        let local = MassProperties {
            local_com: Vec2 { x: HALF, y: -ONE },
            inv_mass: Fixed { raw: mass.into() },
            inv_principal_inertia: Fixed { raw: inertia.into() },
        };
        let flags: LockedAxes = bits.into();
        let props = RigidBodyMassPropsTrait::from_local(local, flags);
        for body_type in array![RigidBodyType::Dynamic, RigidBodyType::Fixed].span() {
            let actual = props.update_world_mass_properties(*body_type, POSE);
            assert_eq!(actual, alternatives::update_world_mass_properties(props, *body_type, POSE));
            assert_eq!(actual.local_mprops, local);
            assert_eq!(actual.flags, flags);
        }
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_from_local() {
        assert_eq!(
            RigidBodyMassPropsTrait::from_local(opaque(LOCAL), opaque(ROTATION_LOCKED))
                .local_mprops,
            LOCAL,
        );
    }

    #[test]
    fn gas_update_world_mass_properties_masked() {
        assert_eq!(
            opaque(RigidBodyMassPropsTrait::from_local(LOCAL, LockedAxesTrait::empty()))
                .update_world_mass_properties(opaque(RigidBodyType::Dynamic), opaque(POSE))
                .effective_world_inv_inertia,
            HALF,
        );
    }

    #[test]
    fn gas_update_world_mass_properties_direct() {
        assert_eq!(
            alternatives::update_world_mass_properties(
                opaque(RigidBodyMassPropsTrait::from_local(LOCAL, LockedAxesTrait::empty())),
                opaque(RigidBodyType::Dynamic),
                opaque(POSE),
            )
                .effective_world_inv_inertia,
            HALF,
        );
    }

    #[test]
    fn gas_mass() {
        assert_eq!(opaque(UPDATED).mass(), HALF);
    }

    #[test]
    fn gas_effective_mass() {
        assert_eq!(opaque(UPDATED).effective_mass(), Vec2 { x: HALF, y: HALF });
    }

    #[test]
    fn gas_effective_angular_inertia() {
        assert_eq!(opaque(UPDATED).effective_angular_inertia(), TWO);
    }
}
