//! The type of a rigid-body (upstream `RigidBodyType`).
//!
//! The predicates are `match`es on the enum: the derived `PartialEq` of an enum matches both
//! operands, which the `alternatives::*_eq` forms (upstream's `==` chains) pay for.
//!
//! Cut: upstream's `SoftFrame` (proxy body of a soft-body cluster), soft bodies being out of
//! scope. Every predicate below is therefore upstream's with `SoftFrame` removed.

/// How a rigid-body responds to forces and movement.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum RigidBodyType {
    /// Fully simulated: responds to forces, gravity and collisions.
    Dynamic,
    /// Never moves: infinite mass, unaffected by anything.
    Fixed,
    /// Driven by setting its next position; pushes but is not pushed.
    KinematicPositionBased,
    /// Driven by setting its velocity; pushes but is not pushed.
    KinematicVelocityBased,
}

/// Predicates and indices of [`RigidBodyType`].
#[generate_trait]
pub impl RigidBodyTypeImpl of RigidBodyTypeTrait {
    /// Is this rigid-body fixed (i.e. cannot move)?
    #[inline(always)]
    fn is_fixed(self: RigidBodyType) -> bool {
        match self {
            RigidBodyType::Fixed => true,
            _ => false,
        }
    }

    /// Is this rigid-body dynamic (i.e. can move and be affected by forces)?
    #[inline(always)]
    fn is_dynamic(self: RigidBodyType) -> bool {
        match self {
            RigidBodyType::Dynamic => true,
            _ => false,
        }
    }

    /// Is this rigid-body kinematic (i.e. can move but is unaffected by forces)?
    #[inline(always)]
    fn is_kinematic(self: RigidBodyType) -> bool {
        match self {
            RigidBodyType::KinematicPositionBased | RigidBodyType::KinematicVelocityBased => true,
            _ => false,
        }
    }

    /// Is this rigid-body dynamic or kinematic (i.e. anything but fixed)?
    #[inline(always)]
    fn is_dynamic_or_kinematic(self: RigidBodyType) -> bool {
        match self {
            RigidBodyType::Fixed => false,
            _ => true,
        }
    }

    /// Upstream's discriminant (`rb_type as u32`): `Dynamic` 0, `Fixed` 1,
    /// `KinematicPositionBased` 2, `KinematicVelocityBased` 3.
    #[inline(always)]
    fn index(self: RigidBodyType) -> u32 {
        match self {
            RigidBodyType::Dynamic => 0,
            RigidBodyType::Fixed => 1,
            RigidBodyType::KinematicPositionBased => 2,
            RigidBodyType::KinematicVelocityBased => 3,
        }
    }
}

#[cfg(test)]
mod alternatives {
    use super::RigidBodyType;

    pub fn is_fixed_eq(self: RigidBodyType) -> bool {
        self == RigidBodyType::Fixed
    }

    pub fn is_dynamic_eq(self: RigidBodyType) -> bool {
        self == RigidBodyType::Dynamic
    }

    pub fn is_kinematic_eq(self: RigidBodyType) -> bool {
        self == RigidBodyType::KinematicPositionBased
            || self == RigidBodyType::KinematicVelocityBased
    }

    pub fn is_dynamic_or_kinematic_eq(self: RigidBodyType) -> bool {
        self != RigidBodyType::Fixed
    }
}

#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use super::alternatives::{
        is_dynamic_eq, is_dynamic_or_kinematic_eq, is_fixed_eq, is_kinematic_eq,
    };
    use super::{RigidBodyType, RigidBodyTypeTrait};

    const D: RigidBodyType = RigidBodyType::Dynamic;
    const F: RigidBodyType = RigidBodyType::Fixed;
    const KP: RigidBodyType = RigidBodyType::KinematicPositionBased;
    const KV: RigidBodyType = RigidBodyType::KinematicVelocityBased;

    /// Asserts the four predicates on the shipped implementation and on the `==` candidates.
    fn assert_predicates(t: RigidBodyType, fixed: bool, dynamic: bool, kinematic: bool) {
        assert_eq!(t.is_fixed(), fixed);
        assert_eq!(t.is_dynamic(), dynamic);
        assert_eq!(t.is_kinematic(), kinematic);
        assert_eq!(t.is_dynamic_or_kinematic(), !fixed);
        assert_eq!(is_fixed_eq(t), fixed);
        assert_eq!(is_dynamic_eq(t), dynamic);
        assert_eq!(is_kinematic_eq(t), kinematic);
        assert_eq!(is_dynamic_or_kinematic_eq(t), !fixed);
    }

    #[test]
    fn test_predicates() {
        assert_predicates(D, false, true, false);
        assert_predicates(F, true, false, false);
        assert_predicates(KP, false, false, true);
        assert_predicates(KV, false, false, true);
    }

    #[test]
    fn test_index_is_upstream_discriminant() {
        assert_eq!(D.index(), 0);
        assert_eq!(F.index(), 1);
        assert_eq!(KP.index(), 2);
        assert_eq!(KV.index(), 3);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_is_fixed() {
        assert!(!opaque(D).is_fixed());
    }

    #[test]
    fn gas_is_fixed_eq() {
        assert!(!is_fixed_eq(opaque(D)));
    }

    #[test]
    fn gas_is_dynamic() {
        assert!(opaque(D).is_dynamic());
    }

    #[test]
    fn gas_is_dynamic_eq() {
        assert!(is_dynamic_eq(opaque(D)));
    }

    #[test]
    fn gas_is_kinematic() {
        assert!(opaque(KV).is_kinematic());
    }

    #[test]
    fn gas_is_kinematic_eq() {
        assert!(is_kinematic_eq(opaque(KV)));
    }

    #[test]
    fn gas_is_dynamic_or_kinematic() {
        assert!(opaque(KV).is_dynamic_or_kinematic());
    }

    #[test]
    fn gas_is_dynamic_or_kinematic_eq() {
        assert!(is_dynamic_or_kinematic_eq(opaque(KV)));
    }

    #[test]
    fn gas_index() {
        assert!(opaque(KV).index() == 3);
    }
}
