//! `ShapeDistance` (Parry `query/distance/distance.rs`; PX6): a distance between two shapes with
//! the sub-shapes it was measured between.
//!
//! The port's [`crate::query::distance`] answers a bare `Fixed` (the closed shape set has no
//! composite sub-shape to name); `ShapeDistance` is the upstream value type for a caller that
//! carries sub-shape ids next to the distance, with its three constructors and the `From<Real>`
//! conversion. It is not on the step path.
//!
//! # Deviations
//!
//! * Upstream's `BvhLeafCost` impl (the BVH ordering key) is not ported: the port has no BVH
//!   (SH2b).

use fixed::Fixed;
use crate::feature_id::SubShapeId;

/// The distance between two shapes, and the sub-shapes it was measured between (upstream
/// `ShapeDistance`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ShapeDistance {
    /// The separation between the two shapes; zero when they touch or overlap.
    pub distance: Fixed,
    /// The sub-shape of the first shape the distance was measured to; `0` for a shape with no
    /// sub-shapes.
    pub subshape1: SubShapeId,
    /// The sub-shape of the second shape the distance was measured to; `0` for a shape with no
    /// sub-shapes.
    pub subshape2: SubShapeId,
}

#[generate_trait]
pub impl ShapeDistanceImpl of ShapeDistanceTrait {
    /// A distance measured between shapes with no sub-shapes to distinguish (upstream `new`).
    #[inline(always)]
    fn new(distance: Fixed) -> ShapeDistance {
        ShapeDistance { distance, subshape1: 0, subshape2: 0 }
    }

    /// Sets the sub-shapes this distance was measured between (upstream `with_subshapes`).
    #[inline(always)]
    fn with_subshapes(
        self: ShapeDistance, subshape1: SubShapeId, subshape2: SubShapeId,
    ) -> ShapeDistance {
        ShapeDistance { subshape1, subshape2, ..self }
    }

    /// Swaps the roles of the two shapes: the sub-shapes trade places (upstream `swapped`).
    #[inline(always)]
    fn swapped(self: ShapeDistance) -> ShapeDistance {
        ShapeDistance { subshape1: self.subshape2, subshape2: self.subshape1, ..self }
    }
}

/// Upstream `impl From<Real> for ShapeDistance`: `ShapeDistance::new`.
pub impl FixedIntoShapeDistance of Into<Fixed, ShapeDistance> {
    #[inline(always)]
    fn into(self: Fixed) -> ShapeDistance {
        ShapeDistanceTrait::new(self)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ZERO};
    use rapier_testing::opaque;
    use super::{ShapeDistance, ShapeDistanceTrait};

    #[test]
    fn test_new_has_no_subshapes() {
        let d = ShapeDistanceTrait::new(HALF);
        assert_eq!(d, ShapeDistance { distance: HALF, subshape1: 0, subshape2: 0 });
        let zero = ShapeDistanceTrait::new(ZERO);
        assert_eq!(zero.distance, ZERO);
    }

    #[test]
    fn test_with_subshapes_and_swapped() {
        let two: Fixed = FixedTrait::from_int(2);
        let d = ShapeDistanceTrait::new(two).with_subshapes(3, 7);
        assert_eq!(d, ShapeDistance { distance: two, subshape1: 3, subshape2: 7 });
        let s = d.swapped();
        assert_eq!(s, ShapeDistance { distance: two, subshape1: 7, subshape2: 3 });
        // Swapping twice is the identity; the distance is never touched.
        assert_eq!(s.swapped(), d);
        // `with_subshapes` replaces both ids.
        assert_eq!(d.with_subshapes(0, 1).subshape2, 1);
    }

    #[test]
    fn test_from_fixed_is_new() {
        let two: Fixed = FixedTrait::from_int(2);
        let d: ShapeDistance = two.into();
        assert_eq!(d, ShapeDistanceTrait::new(two));
    }

    #[test]
    fn gas_baseline() {}
    #[test]
    fn gas_new() {
        let _ = ShapeDistanceTrait::new(opaque(HALF));
    }
    #[test]
    fn gas_with_subshapes() {
        let _ = opaque(ShapeDistanceTrait::new(HALF)).with_subshapes(opaque(3), opaque(7));
    }
    #[test]
    fn gas_swapped() {
        let _ = opaque(ShapeDistanceTrait::new(HALF).with_subshapes(3, 7)).swapped();
    }
    #[test]
    fn gas_from_fixed() {
        let _: ShapeDistance = opaque(HALF).into();
    }
}
