//! Rigid 2D transforms (the `glamx::Pose2` layer), with fused Q32.32 kernels.
use fixed::ZERO;
use fixed::wide::{dot2, dot2_add, mul_sub};
use glam::Vec2;
use crate::rot2::{Rot2, Rot2Trait};

/// Translation and unit-complex rotation; applies rotation first, then translation.
/// `rotation` is unchecked and must stay unit for inverse operations (see `Rot2`). Products
/// never renormalize. Each output component floors once; coordinates must fit Q32.32.
/// `lerp_slerp` is deferred until trig is available.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Pose2 {
    /// Displacement in the parent frame.
    pub translation: Vec2,
    /// Orientation relative to the parent frame.
    pub rotation: Rot2,
}

/// Exact identity pose: zero displacement and identity rotation.
pub const IDENTITY: Pose2 = Pose2 {
    translation: Vec2 { x: ZERO, y: ZERO }, rotation: crate::rot2::IDENTITY,
};

#[generate_trait]
pub impl Pose2Impl of Pose2Trait {
    /// Exact identity pose.
    const IDENTITY: Pose2 = super::pose2::IDENTITY;

    /// Stores translation and rotation as-is, without normalization or rounding; never panics.
    fn new(translation: Vec2, rotation: Rot2) -> Pose2 {
        Pose2 { translation, rotation }
    }

    /// Composition `self * other`, applying `other` first. Floors once per output component.
    /// Panics as `transform_point` or `Rot2::mul` when an output leaves Q32.32.
    fn mul(self: Pose2, other: Pose2) -> Pose2 {
        Pose2 {
            translation: self.transform_point(other.translation),
            rotation: Rot2Trait::mul(self.rotation, other.rotation),
        }
    }

    /// Conjugates the rotation and rotates the negative translation, flooring once per output.
    /// Requires a unit rotation. Panics as `inverse_transform_point` / `Rot2::inverse`.
    fn inverse(self: Pose2) -> Pose2 {
        Pose2 {
            translation: self.inverse_transform_point(Vec2 { x: ZERO, y: ZERO }),
            rotation: self.rotation.inverse(),
        }
    }

    /// Pose of `other` in this frame (`pos12`): conjugate rotation and rotated displacement.
    /// Direct fused kernels avoid constructing and rounding an intermediate inverse pose.
    /// Floors once per output. Panics as `inverse_transform_point` or 'Fixed: overflow'
    /// on rotation overflow; requires a unit rotation and representable translation difference.
    fn inv_mul(self: Pose2, other: Pose2) -> Pose2 {
        let a = self.rotation;
        let b = other.rotation;
        Pose2 {
            translation: self.inverse_transform_point(other.translation),
            rotation: Rot2 {
                re: dot2(a.re, b.re, a.im, b.im), im: mul_sub(a.re, b.im, a.im, b.re),
            },
        }
    }

    /// `R * point + t`, accumulated wide with one floor per output.
    /// Panics with 'Fixed: overflow' if an output leaves Q32.32, or 'i64_neg Underflow' if
    /// an unchecked rotation has imaginary component `fixed::MIN`.
    fn transform_point(self: Pose2, point: Vec2) -> Vec2 {
        let r = self.rotation;
        Vec2 {
            x: dot2_add(r.re, point.x, -r.im, point.y, self.translation.x),
            y: dot2_add(r.im, point.x, r.re, point.y, self.translation.y),
        }
    }

    /// Rotates a direction (no translation), flooring once per component.
    /// Panics with 'Fixed: overflow' if an output leaves Q32.32.
    fn transform_vector(self: Pose2, vector: Vec2) -> Vec2 {
        self.rotation.rotate(vector)
    }

    /// `conj(R) * (point - t)`, flooring once per component; requires a unit rotation.
    /// Panics with 'i64_sub Overflow' if the displacement leaves Q32.32, or 'Fixed: overflow'
    /// if a rotated component does. The subtraction is exact before rotation.
    fn inverse_transform_point(self: Pose2, point: Vec2) -> Vec2 {
        self.rotation.inverse_rotate(point - self.translation)
    }

    /// Rotates a direction by the conjugate (no translation), flooring once per component.
    /// Panics with 'Fixed: overflow' if an output leaves Q32.32; requires a unit rotation.
    fn inverse_transform_vector(self: Pose2, vector: Vec2) -> Vec2 {
        self.rotation.inverse_rotate(vector)
    }
}

/// Default is the exact identity; cannot panic.
pub impl Pose2Default of Default<Pose2> {
    fn default() -> Pose2 {
        IDENTITY
    }
}

/// Composition with the same flooring and overflow policy as `Pose2Trait::mul`.
pub impl Pose2Mul of Mul<Pose2> {
    fn mul(lhs: Pose2, rhs: Pose2) -> Pose2 {
        Pose2Trait::mul(lhs, rhs)
    }
}

#[cfg(test)]
mod alternatives {
    use glam::Vec2;
    use crate::rot2::Rot2;
    use super::Pose2;

    fn rotate(r: Rot2, v: Vec2) -> Vec2 {
        Vec2 { x: r.re, y: r.im } * Vec2 { x: v.x, y: v.x }
            + Vec2 { x: -r.im, y: r.re } * Vec2 { x: v.y, y: v.y }
    }
    pub fn transform_point(p: Pose2, v: Vec2) -> Vec2 {
        rotate(p.rotation, v) + p.translation
    }
    pub fn inverse_transform_point(p: Pose2, v: Vec2) -> Vec2 {
        rotate(Rot2 { re: p.rotation.re, im: -p.rotation.im }, v - p.translation)
    }
    pub fn mul(a: Pose2, b: Pose2) -> Pose2 {
        let r = rotate(a.rotation, Vec2 { x: b.rotation.re, y: b.rotation.im });
        Pose2 {
            translation: transform_point(a, b.translation), rotation: Rot2 { re: r.x, im: r.y },
        }
    }
    pub fn inv_mul(a: Pose2, b: Pose2) -> Pose2 {
        let r = rotate(
            Rot2 { re: a.rotation.re, im: -a.rotation.im },
            Vec2 { x: b.rotation.re, y: b.rotation.im },
        );
        Pose2 {
            translation: inverse_transform_point(a, b.translation),
            rotation: Rot2 { re: r.x, im: r.y },
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, MAX, MIN, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_testing::opaque;
    use crate::rot2::{Rot2, Rot2Trait};
    use super::{IDENTITY, Pose2, Pose2Trait, alternatives};

    const A: Pose2 = Pose2 {
        translation: Vec2 { x: HALF, y: Fixed { raw: -8589934592 } },
        rotation: Rot2 { re: Fixed { raw: 2576980378 }, im: Fixed { raw: 3435973837 } },
    };
    const B: Pose2 = Pose2 {
        translation: Vec2 { x: Fixed { raw: -4294967296 }, y: HALF },
        rotation: Rot2 { re: Fixed { raw: 3037000500 }, im: Fixed { raw: -3037000500 } },
    };
    const V: Vec2 = Vec2 { x: HALF, y: Fixed { raw: -8589934592 } };

    fn close(a: Vec2, b: Vec2, ulps: i64) {
        assert!(a.x.abs_diff_eq(b.x, Fixed { raw: ulps }));
        assert!(a.y.abs_diff_eq(b.y, Fixed { raw: ulps }));
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
    fn test_identity_inverse_relative_pose() {
        assert_eq!(Pose2Trait::IDENTITY, IDENTITY);
        assert_eq!(Default::<Pose2>::default(), IDENTITY);
        for a in array![IDENTITY, A, B].span() {
            let a = *a;
            assert_eq!(Pose2Trait::new(a.translation, a.rotation), a);
            assert_eq!(a * IDENTITY, a);
            assert_eq!(IDENTITY * a, a);
            close_pose(a * a.inverse(), IDENTITY, 4);
            close_pose(a.inverse() * a, IDENTITY, 4);
            close_pose(a.inverse().inverse(), a, 4);
            let same = a.inv_mul(a);
            close(same.translation, IDENTITY.translation, 2);
            assert!(same.rotation.is_unit());
            close_pose(same, IDENTITY, 2);
            for b in array![IDENTITY, A, B].span() {
                close_pose(a.inv_mul(*b), a.inverse() * *b, 2);
                assert_eq!(Pose2Trait::mul(a, *b), a * *b);
            }
        }
    }

    #[test]
    fn test_point_vector_round_trips_and_order() {
        for a in array![IDENTITY, A, B].span() {
            let a = *a;
            for p in array![V, Vec2 { x: ZERO, y: ZERO }, Vec2 { x: -ONE, y: HALF }].span() {
                let p = *p;
                close(a.inverse_transform_point(a.transform_point(p)), p, 4);
                close(a.transform_point(a.inverse_transform_point(p)), p, 4);
                close(a.inverse_transform_vector(a.transform_vector(p)), p, 4);
                close(a.transform_vector(a.inverse_transform_vector(p)), p, 4);
                assert_eq!(a.transform_vector(p), a.rotation.rotate(p));
                assert_eq!(a.inverse_transform_vector(p), a.rotation.inverse_rotate(p));
                close((a * B).transform_point(p), a.transform_point(B.transform_point(p)), 4);
            }
        }
        // Noncommuting rotation/translation catches reversed composition order.
        assert_ne!(A * B, B * A);
    }

    #[test]
    fn test_extreme_identity_and_wide_translation_cancellation() {
        let v = Vec2 { x: MIN, y: MAX };
        assert_eq!(IDENTITY.transform_point(v), v);
        assert_eq!(IDENTITY.inverse_transform_point(v), v);
        // R*v alone overflows, but adding t in the wide accumulator brings it back in range.
        let p = Pose2 { translation: Vec2 { x: ZERO, y: -MAX }, rotation: A.rotation };
        let out = p.transform_point(Vec2 { x: MAX, y: MAX });
        assert!(out.x < ZERO && out.y > ZERO);
        let far = Pose2 { translation: v, rotation: A.rotation };
        assert_eq!(far.inv_mul(far).translation, IDENTITY.translation);
        assert_eq!(far.inverse_transform_point(v), IDENTITY.translation);
        // Constructor preserves the unchecked rotation: no implicit normalization.
        let raw = Rot2 { re: TWO, im: ZERO };
        assert_eq!(Pose2Trait::new(V, raw).rotation, raw);
    }

    #[test]
    #[should_panic(expected: ('Fixed: overflow',))]
    fn test_transform_overflow() {
        Pose2 { translation: Vec2 { x: MAX, y: ZERO }, rotation: crate::rot2::IDENTITY }
            .transform_point(Vec2 { x: ONE, y: ZERO });
    }

    #[test]
    #[should_panic(expected: ('i64_sub Overflow',))]
    fn test_displacement_overflow() {
        Pose2 { translation: Vec2 { x: MIN, y: ZERO }, rotation: crate::rot2::IDENTITY }
            .inverse_transform_point(Vec2 { x: ONE, y: ZERO });
    }

    #[test]
    #[fuzzer(runs: 128, seed: 20260920)]
    fn fuzz_candidates(x: i32, y: i32, re: i32, im: i32) {
        let v = Vec2 { x: Fixed { raw: x.into() }, y: Fixed { raw: y.into() } };
        let a = Pose2 {
            translation: v,
            rotation: Rot2 { re: Fixed { raw: re.into() }, im: Fixed { raw: im.into() } },
        };
        close(a.transform_point(V), alternatives::transform_point(a, V), 1);
        close(a.inverse_transform_point(V), alternatives::inverse_transform_point(a, V), 1);
        close_pose(a * B, alternatives::mul(a, B), 1);
        close_pose(a.inv_mul(B), alternatives::inv_mul(a, B), 1);
        assert_eq!(a.inv_mul(a).translation, IDENTITY.translation);
    }

    #[test]
    fn gas_baseline() {}
    #[test]
    fn gas_default() {
        assert_eq!(Default::<Pose2>::default(), IDENTITY);
    }
    #[test]
    fn gas_new() {
        assert_eq!(Pose2Trait::new(opaque(A.translation), opaque(A.rotation)), A);
    }
    #[test]
    fn gas_transform_point_fused() {
        assert_eq!(
            Pose2Trait::transform_point(opaque(A), opaque(V)),
            Vec2 { x: Fixed { raw: 10307921511 }, y: Fixed { raw: -12025908430 } },
        );
    }
    #[test]
    fn gas_transform_point_composed() {
        assert_eq!(
            alternatives::transform_point(opaque(A), opaque(V)),
            Vec2 { x: Fixed { raw: 10307921511 }, y: Fixed { raw: -12025908430 } },
        );
    }
    #[test]
    fn gas_inverse_transform_point_fused() {
        assert_eq!(
            Pose2Trait::inverse_transform_point(opaque(A), opaque(V)),
            Vec2 { x: Fixed { raw: 0 }, y: Fixed { raw: 0 } },
        );
    }
    #[test]
    fn gas_inverse_transform_point_composed() {
        assert_eq!(
            alternatives::inverse_transform_point(opaque(A), opaque(V)),
            Vec2 { x: Fixed { raw: 0 }, y: Fixed { raw: 0 } },
        );
    }
    #[test]
    fn gas_mul_fused() {
        assert_eq!(
            Pose2Trait::mul(opaque(A), opaque(B)),
            Pose2 {
                translation: Vec2 { x: Fixed { raw: -2147483649 }, y: Fixed { raw: -10737418240 } },
                rotation: Rot2 { re: Fixed { raw: 4251800700 }, im: Fixed { raw: 607400099 } },
            },
        );
    }
    #[test]
    fn gas_mul_composed() {
        assert_eq!(
            alternatives::mul(opaque(A), opaque(B)),
            Pose2 {
                translation: Vec2 { x: Fixed { raw: -2147483649 }, y: Fixed { raw: -10737418240 } },
                rotation: Rot2 { re: Fixed { raw: 4251800700 }, im: Fixed { raw: 607400099 } },
            },
        );
    }
    #[test]
    fn gas_inv_mul_fused() {
        assert_eq!(
            Pose2Trait::inv_mul(opaque(A), opaque(B)),
            Pose2 {
                translation: Vec2 { x: Fixed { raw: 4724464025 }, y: Fixed { raw: 11596411700 } },
                rotation: Rot2 { re: Fixed { raw: -607400100 }, im: Fixed { raw: -4251800701 } },
            },
        );
    }
    #[test]
    fn gas_inv_mul_composed() {
        assert_eq!(
            alternatives::inv_mul(opaque(A), opaque(B)),
            Pose2 {
                translation: Vec2 { x: Fixed { raw: 4724464025 }, y: Fixed { raw: 11596411700 } },
                rotation: Rot2 { re: Fixed { raw: -607400101 }, im: Fixed { raw: -4251800702 } },
            },
        );
    }
    #[test]
    fn gas_inverse() {
        assert_eq!(
            opaque(A).inverse(),
            Pose2 {
                translation: Vec2 { x: Fixed { raw: 5583457485 }, y: Fixed { raw: 6871947674 } },
                rotation: Rot2 { re: Fixed { raw: 2576980378 }, im: Fixed { raw: -3435973837 } },
            },
        );
    }
    #[test]
    fn gas_transform_vector() {
        assert_eq!(
            opaque(A).transform_vector(opaque(V)),
            Vec2 { x: Fixed { raw: 8160437863 }, y: Fixed { raw: -3435973838 } },
        );
    }
    #[test]
    fn gas_inverse_transform_vector() {
        assert_eq!(
            opaque(A).inverse_transform_vector(opaque(V)),
            Vec2 { x: Fixed { raw: -5583457485 }, y: Fixed { raw: -6871947675 } },
        );
    }
    #[test]
    fn gas_mul_operator() {
        assert_eq!(opaque(A) * opaque(IDENTITY), A);
    }
}
