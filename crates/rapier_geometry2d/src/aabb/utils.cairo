//! Small `Aabb` queries and transforms of Parry `bounding_volume/aabb.rs` and
//! `query/split/split_aabb.rs` (PX3): distance to the origin, projection on an axis, scaling about
//! the centre and the canonical split.

use fixed::wide::{dot2, norm2};
use fixed::{Fixed, FixedTrait};
use glam_core::{Vec2, Vec2Trait};
use crate::query::split::SplitResult;
use super::{Aabb, AabbTrait};

/// Errors of the `Aabb` utilities.
pub mod errors {
    /// A canonical axis other than `0` (x) or `1` (y).
    pub const AXIS: felt252 = 'Aabb: axis out of range';
}

/// Utility methods of [`Aabb`].
#[generate_trait]
pub impl AabbUtilsImpl of AabbUtilsTrait {
    /// The distance from the origin to the box (upstream `distance_to_origin`):
    /// `|max(mins, -maxs, 0)|`, zero when the box contains the origin. The length is one wide
    /// square root, rounded to nearest.
    /// #### Panics
    /// * `'i64_neg Underflow'` if `maxs` has a component equal to `fixed::MIN`.
    fn distance_to_origin(self: Aabb) -> Fixed {
        let v = self.mins.max(-self.maxs).max(Vec2Trait::ZERO);
        norm2(v.x, v.y)
    }

    /// The interval `(min, max)` the box covers along `axis` (upstream `project_on_axis`):
    /// `center . axis ± |support(axis) . axis|`, the support point being the corner of the
    /// half-extents box toward `axis`. `axis` need not be unit.
    /// #### Panics
    /// * `'Fixed: overflow'` if a product or a bound leaves the scalar range.
    /// #### Deviations
    /// * Each dot product is accumulated wide and floored once, and the support-point sign
    ///   choice is folded into `|axis|` (the two are equal for non-negative half extents).
    fn project_on_axis(self: Aabb, axis: Vec2) -> (Fixed, Fixed) {
        let h = self.half_extents();
        let shift = dot2(h.x, axis.x.abs(), h.y, axis.y.abs()).abs();
        let c = self.center();
        let center = dot2(c.x, axis.x, c.y, axis.y);
        (center - shift, center + shift)
    }

    /// The box scaled about its centre (upstream `scaled_wrt_center`): the half extents are
    /// multiplied by `|scale|` component-wise, the centre stays.
    /// #### Panics
    /// * `'Fixed: overflow'` if a corner leaves the scalar range.
    fn scaled_wrt_center(self: Aabb, scale: Vec2) -> Aabb {
        let center = self.center();
        let half_extents = self.half_extents() * scale.abs();
        AabbTrait::from_half_extents(center, half_extents)
    }

    /// Splits the box by the plane `p[axis] = bias` (upstream `canonical_split`), `axis` being
    /// `0` (x) or `1` (y): `Positive` when the box lies at or above `bias - epsilon`, `Negative`
    /// when it lies at or below `bias + epsilon`, otherwise the two halves (below first).
    /// #### Panics
    /// * `'Aabb: axis out of range'` for an axis above `1`; `'i64_sub Overflow'` /
    ///   `'i64_add Overflow'` if `bias ± epsilon` leaves the scalar range.
    fn canonical_split(self: Aabb, axis: u32, bias: Fixed, epsilon: Fixed) -> SplitResult<Aabb> {
        assert(axis < 2, errors::AXIS);
        let (min, max) = if axis == 0 {
            (self.mins.x, self.maxs.x)
        } else {
            (self.mins.y, self.maxs.y)
        };
        if min >= bias - epsilon {
            SplitResult::Positive
        } else if max <= bias + epsilon {
            SplitResult::Negative
        } else {
            let mut left = self;
            let mut right = self;
            if axis == 0 {
                left.maxs.x = bias;
                right.mins.x = bias;
            } else {
                left.maxs.y = bias;
                right.mins.y = bias;
            }
            SplitResult::Pair((left, right))
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, ONE, ZERO};
    use glam_core::Vec2;
    use rapier_testing::opaque;
    use crate::query::split::SplitResult;
    use super::AabbUtilsTrait;
    use super::super::{Aabb, AabbTrait};

    fn f(n: i64) -> Fixed {
        Fixed { raw: n * 0x1_0000_0000 }
    }

    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: f(x), y: f(y) }
    }

    #[test]
    fn test_distance_to_origin() {
        let cases: Span<(Aabb, Fixed)> = array![
            (AabbTrait::new(v(-1, -1), v(1, 1)), ZERO), (AabbTrait::new(v(3, 4), v(6, 8)), f(5)),
            (AabbTrait::new(v(-6, -8), v(-3, -4)), f(5)), (AabbTrait::new(v(2, -1), v(5, 1)), f(2)),
            (AabbTrait::new(v(0, 0), v(1, 1)), ZERO),
        ]
            .span();
        for case in cases {
            let (aabb, expected) = *case;
            assert_eq!(aabb.distance_to_origin(), expected);
        }
    }

    #[test]
    fn test_project_on_axis() {
        let aabb = AabbTrait::new(v(0, 1), v(2, 5));
        assert_eq!(aabb.project_on_axis(v(1, 0)), (ZERO, f(2)));
        assert_eq!(aabb.project_on_axis(v(0, 1)), (f(1), f(5)));
        // The axis need not be unit: `(1, 1)` sees `centre 4 ± |(1, 2) . (1, 1)| = 3`.
        assert_eq!(aabb.project_on_axis(v(1, 1)), (f(1), f(7)));
        assert_eq!(aabb.project_on_axis(v(-1, -1)), (f(-7), f(-1)));
        assert_eq!(aabb.project_on_axis(v(0, 0)), (ZERO, ZERO));
    }

    #[test]
    fn test_scaled_wrt_center() {
        let aabb = AabbTrait::new(v(0, 0), v(2, 4));
        assert_eq!(aabb.scaled_wrt_center(v(2, 1)), AabbTrait::new(v(-1, 0), v(3, 4)));
        // A negative component acts by its absolute value.
        assert_eq!(aabb.scaled_wrt_center(v(-1, 1)), aabb);
        assert_eq!(aabb.scaled_wrt_center(v(0, 0)), AabbTrait::new(v(1, 2), v(1, 2)));
    }

    #[test]
    fn test_canonical_split() {
        let aabb = AabbTrait::new(v(-1, -1), v(1, 1));
        let half = f(1) / f(2);
        match aabb.canonical_split(0, half, ZERO) {
            SplitResult::Pair((
                l, r,
            )) => {
                assert_eq!(l, AabbTrait::new(v(-1, -1), Vec2 { x: half, y: ONE }));
                assert_eq!(r, AabbTrait::new(Vec2 { x: half, y: -ONE }, v(1, 1)));
            },
            _ => panic!("expected a pair"),
        }
        assert_eq!(aabb.canonical_split(1, f(2), ZERO), SplitResult::Negative);
        assert_eq!(aabb.canonical_split(1, f(-2), ZERO), SplitResult::Positive);
        // Touching planes, and epsilon absorbing the box.
        assert_eq!(aabb.canonical_split(0, ONE, ZERO), SplitResult::Negative);
        assert_eq!(aabb.canonical_split(0, -ONE, ZERO), SplitResult::Positive);
        // Epsilon absorbing the whole box: the positive test comes first, as upstream.
        assert_eq!(aabb.canonical_split(0, half, f(2)), SplitResult::Positive);
    }

    #[test]
    #[should_panic(expected: ('Aabb: axis out of range',))]
    fn test_canonical_split_axis_out_of_range() {
        let _ = AabbTrait::new(v(0, 0), v(1, 1)).canonical_split(2, ZERO, ZERO);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_distance_to_origin() {
        let _ = opaque(AabbTrait::new(v(-6, -8), v(-3, -4))).distance_to_origin();
    }

    #[test]
    fn gas_project_on_axis() {
        let _ = opaque(AabbTrait::new(v(0, 1), v(2, 5))).project_on_axis(opaque(v(1, 1)));
    }

    #[test]
    fn gas_scaled_wrt_center() {
        let _ = opaque(AabbTrait::new(v(0, 0), v(2, 4))).scaled_wrt_center(opaque(v(2, -1)));
    }

    #[test]
    fn gas_canonical_split_pair() {
        let _ = opaque(AabbTrait::new(v(-1, -1), v(1, 1)))
            .canonical_split(opaque(0), opaque(ZERO), opaque(ZERO));
    }

    #[test]
    fn gas_canonical_split_positive() {
        let _ = opaque(AabbTrait::new(v(-1, -1), v(1, 1)))
            .canonical_split(opaque(0), opaque(f(-2)), opaque(ZERO));
    }
}
