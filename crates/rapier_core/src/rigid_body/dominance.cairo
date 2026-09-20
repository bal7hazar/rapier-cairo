//! Dominance groups of a rigid-body (upstream `RigidBodyDominance`).

use super::body_type::{RigidBodyType, RigidBodyTypeTrait};

/// Effective dominance of every body that is not dynamic or kinematic: one above `i8::MAX`.
pub const FIXED_EFFECTIVE_GROUP: i16 = 128;

/// The dominance group of a rigid-body: in a contact, the body of the higher group is treated as
/// having infinite mass. Default `0`.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct RigidBodyDominance {
    /// Signed group, `-127..=127` in practice (`i8`).
    pub group: i8,
}

/// Dominance resolution.
#[generate_trait]
pub impl RigidBodyDominanceImpl of RigidBodyDominanceTrait {
    /// The actual dominance group after taking the body type into account: `group` for dynamic
    /// and kinematic bodies, [`FIXED_EFFECTIVE_GROUP`] (`128`) for fixed ones, so that a fixed
    /// body always dominates.
    #[inline(always)]
    fn effective_group(self: RigidBodyDominance, status: RigidBodyType) -> i16 {
        if status.is_dynamic_or_kinematic() {
            self.group.into()
        } else {
            FIXED_EFFECTIVE_GROUP
        }
    }
}

#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use super::super::body_type::RigidBodyType;
    use super::{FIXED_EFFECTIVE_GROUP, RigidBodyDominance, RigidBodyDominanceTrait};

    #[test]
    fn test_default_is_zero() {
        let default: RigidBodyDominance = Default::default();
        assert_eq!(default, RigidBodyDominance { group: 0 });
    }

    #[test]
    fn test_effective_group() {
        let d = RigidBodyDominance { group: -3 };
        assert_eq!(d.effective_group(RigidBodyType::Dynamic), -3);
        assert_eq!(d.effective_group(RigidBodyType::KinematicPositionBased), -3);
        assert_eq!(d.effective_group(RigidBodyType::KinematicVelocityBased), -3);
        assert_eq!(d.effective_group(RigidBodyType::Fixed), 128);
        assert_eq!(FIXED_EFFECTIVE_GROUP, 128);
    }

    #[test]
    fn test_effective_group_extremes() {
        let max = RigidBodyDominance { group: 127 };
        let min = RigidBodyDominance { group: -128 };
        // A fixed body dominates even the highest dynamic group.
        assert!(
            max.effective_group(RigidBodyType::Fixed) > max.effective_group(RigidBodyType::Dynamic),
        );
        assert_eq!(min.effective_group(RigidBodyType::Dynamic), -128);
        assert_eq!(min.effective_group(RigidBodyType::Fixed), 128);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_effective_group_dynamic() {
        let d = opaque(RigidBodyDominance { group: -3 });
        assert!(d.effective_group(opaque(RigidBodyType::Dynamic)) == -3);
    }

    #[test]
    fn gas_effective_group_fixed() {
        let d = opaque(RigidBodyDominance { group: -3 });
        assert!(d.effective_group(opaque(RigidBodyType::Fixed)) == 128);
    }
}
