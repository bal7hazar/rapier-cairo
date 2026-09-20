//! Solver-related material properties of a collider (upstream `ColliderMaterial`).

use fixed::{Fixed, ONE, ZERO};
use super::combine_rule::CoefficientCombineRule;

/// The friction and restitution of a collider, and how they combine with the other collider's.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ColliderMaterial {
    /// Friction coefficient; the greater, the stronger the friction forces. Should be `>= 0`.
    /// Default `1`.
    pub friction: Fixed,
    /// Restitution coefficient; increase to make contacts bouncier. Should be `>= 0` and
    /// generally not above `1` (perfectly elastic). Default `0`.
    pub restitution: Fixed,
    /// Rule combining the friction of two colliders in contact. Default `Average`.
    pub friction_combine_rule: CoefficientCombineRule,
    /// Rule combining the restitution of two colliders in contact. Default `Average`.
    pub restitution_combine_rule: CoefficientCombineRule,
}

/// Upstream default: friction `1`, restitution `0`, both rules `Average`.
pub impl ColliderMaterialDefault of Default<ColliderMaterial> {
    #[inline(always)]
    fn default() -> ColliderMaterial {
        ColliderMaterial {
            friction: ONE,
            restitution: ZERO,
            friction_combine_rule: CoefficientCombineRule::Average,
            restitution_combine_rule: CoefficientCombineRule::Average,
        }
    }
}

/// Constructors of [`ColliderMaterial`].
#[generate_trait]
pub impl ColliderMaterialImpl of ColliderMaterialTrait {
    /// A material with the given friction and restitution and the default (`Average`) rules.
    #[inline(always)]
    fn new(friction: Fixed, restitution: Fixed) -> ColliderMaterial {
        ColliderMaterial {
            friction,
            restitution,
            friction_combine_rule: CoefficientCombineRule::Average,
            restitution_combine_rule: CoefficientCombineRule::Average,
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{FixedTrait, HALF, ONE, ZERO};
    use rapier_testing::opaque;
    use super::super::combine_rule::CoefficientCombineRule;
    use super::{ColliderMaterial, ColliderMaterialTrait};

    #[test]
    fn test_default() {
        let material: ColliderMaterial = Default::default();
        assert_eq!(material.friction, ONE);
        assert_eq!(material.restitution, ZERO);
        assert_eq!(material.friction_combine_rule, CoefficientCombineRule::Average);
        assert_eq!(material.restitution_combine_rule, CoefficientCombineRule::Average);
    }

    #[test]
    fn test_new_keeps_default_rules() {
        let material = ColliderMaterialTrait::new(HALF, ONE);
        assert_eq!(material.friction, HALF);
        assert_eq!(material.restitution, ONE);
        let default: ColliderMaterial = Default::default();
        assert_eq!(material.friction_combine_rule, default.friction_combine_rule);
        assert_eq!(material.restitution_combine_rule, default.restitution_combine_rule);
        assert_eq!(ColliderMaterialTrait::new(ONE, ZERO), default);
    }

    #[test]
    fn test_rules_are_independent() {
        let material = ColliderMaterial {
            friction_combine_rule: CoefficientCombineRule::Min,
            restitution_combine_rule: CoefficientCombineRule::Max,
            ..ColliderMaterialTrait::new(FixedTrait::from_int(2), HALF),
        };
        let mixed = CoefficientCombineRule::Min;
        assert_eq!(material.friction_combine_rule, mixed);
        assert_eq!(material.restitution_combine_rule, CoefficientCombineRule::Max);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_default() {
        let material: ColliderMaterial = Default::default();
        assert!(opaque(material).friction == ONE);
    }

    #[test]
    fn gas_new() {
        let material = ColliderMaterialTrait::new(opaque(HALF), opaque(ONE));
        assert!(opaque(material).restitution == ONE);
    }
}
