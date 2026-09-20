//! The two exact 2D products the point and closest-point queries branch on.
//!
//! `rapier_math::math_ext::norm2` keeps *squared lengths* wide; the same argument applies to the
//! dot and perp-dot products these queries compare against them: `ab . ap <= 0` and
//! `ab . ap >= |ab|^2` decide a Voronoi region, and `ab x ap == 0` decides collinearity. A
//! rescaled `Fixed` product would answer "zero" for everything below `2^-32` and panic above
//! `2^31`, so both sides stay at the raw Q64.64 scale of
//! [`norm2_sq_wide`](rapier_math::math_ext::norm2::norm2_sq_wide), where the comparison is exact
//! over the whole scalar range.

use core::num::traits::WideMul;
use fixed::Fixed;

/// Returns the exact raw Q64.64 dot product `a . b` (`ax bx + ay by`), without rescale.
///
/// Mirrors `glam::Vec2::dot`, kept wide; the `Fixed` form is `rapier_math::math_ext::vec2::gdot`.
/// #### Panics
/// * `'i128_add Overflow'` only for raws at the very ends of the range (the sum needs 128 bits).
/// #### Deviations
/// * Not a `Fixed`: compare it with another wide quantity, or narrow it once at the end.
#[inline(always)]
pub fn dot_wide(ax: Fixed, ay: Fixed, bx: Fixed, by: Fixed) -> i128 {
    ax.raw.wide_mul(bx.raw) + ay.raw.wide_mul(by.raw)
}

/// Returns the exact raw Q64.64 perp-dot product `a x b` (`ax by - ay bx`), without rescale.
///
/// Mirrors `glam::Vec2::perp_dot`, kept wide; the `Fixed` form is
/// `rapier_math::math_ext::vec2::gcross_vv`. Its **sign** is the side of the line `a` that `b`
/// lies on and its vanishing is exact collinearity — neither survives a rescale.
/// #### Panics
/// * `'i128_sub Overflow'` only for raws at the very ends of the range.
/// #### Deviations
/// * Not a `Fixed`: see [`dot_wide`].
#[inline(always)]
pub fn cross_wide(ax: Fixed, ay: Fixed, bx: Fixed, by: Fixed) -> i128 {
    ax.raw.wide_mul(by.raw) - ay.raw.wide_mul(bx.raw)
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, ONE, ZERO};
    use rapier_math::math_ext::norm2::norm2_sq_wide;
    use rapier_testing::opaque;
    use super::{cross_wide, dot_wide};

    const Q64: i128 = 0x1_0000_0000_0000_0000;

    #[test]
    fn test_products_are_exact() {
        let three: Fixed = FixedTrait::from_int(3);
        let four: Fixed = FixedTrait::from_int(4);
        assert_eq!(dot_wide(three, four, three, four), 25 * Q64);
        assert_eq!(dot_wide(ONE, ZERO, ZERO, ONE), 0);
        assert_eq!(cross_wide(three, four, -four, three), 25 * Q64);
        assert_eq!(cross_wide(three, four, three, four), 0);
        assert_eq!(dot_wide(three, four, three, four), norm2_sq_wide(three, four));
    }

    /// A vector of a few raw units: every `Fixed` product of it is 0, the wide ones are not.
    #[test]
    fn test_tiny_operands_do_not_underflow() {
        let a = Fixed { raw: 3 };
        let b = Fixed { raw: 5 };
        assert_eq!(a * b, ZERO);
        assert_eq!(dot_wide(a, b, b, a), 30);
        assert_eq!(cross_wide(a, b, b, a), 9 - 25);
        // Collinearity stays exact at that scale.
        assert_eq!(cross_wide(a, b, Fixed { raw: 6 }, Fixed { raw: 10 }), 0);
    }

    /// Operands a `Fixed` product would overflow.
    #[test]
    fn test_huge_operands_do_not_overflow() {
        let big = Fixed { raw: 0x4000_0000_0000_0000 };
        assert_eq!(dot_wide(big, ZERO, big, ZERO), 0x1000_0000_0000_0000_0000_0000_0000_0000);
        assert_eq!(cross_wide(big, ZERO, ZERO, big), 0x1000_0000_0000_0000_0000_0000_0000_0000);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_dot_wide() {
        let _ = dot_wide(
            opaque(Fixed { raw: 3 }),
            opaque(Fixed { raw: 4 }),
            opaque(Fixed { raw: 5 }),
            opaque(Fixed { raw: 6 }),
        );
    }

    #[test]
    fn gas_cross_wide() {
        let _ = cross_wide(
            opaque(Fixed { raw: 3 }),
            opaque(Fixed { raw: 4 }),
            opaque(Fixed { raw: 5 }),
            opaque(Fixed { raw: 6 }),
        );
    }
}
