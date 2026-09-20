//! Q32.32 unit-complex rotations, the 2D `glamx` layer.
use fixed::wide::{dot2, mul_add, mul_sub, normalize2};
use fixed::{Fixed, ONE, ZERO};
use glam::Vec2;
use crate::consts::UNIT_TOL_SQ_RAW;
use crate::math_ext::is_unit2_raw;

/// Rotation represented by cosine and sine. Fields are unchecked; normally each is in [-1, 1].
/// Multiplication floors each output once and does not renormalize (as upstream).
/// Renormalize after each solver substep; `integrate` already does so. Conjugation is an
/// inverse only for a unit input. `angle`, `from_angle` and `lerp_slerp` await trig support.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Rot2 {
    /// Cosine, the real component.
    pub re: Fixed,
    /// Sine, the imaginary component.
    pub im: Fixed,
}

/// Identity rotation, exactly (1, 0).
pub const IDENTITY: Rot2 = Rot2 { re: ONE, im: ZERO };

/// Rotation-specific failures.
pub mod errors {
    /// A zero complex number cannot specify a rotation.
    pub const ZERO: felt252 = 'Rot2: zero';
}

#[generate_trait]
pub impl Rot2Impl of Rot2Trait {
    /// Identity rotation, exactly (1, 0).
    const IDENTITY: Rot2 = super::rot2::IDENTITY;

    /// Normalizes `(re, im)` with the wide sum of squares; see `renormalize` for precision.
    /// Panics with 'Rot2: zero' for (0, 0); see `renormalize` for other range restrictions.
    fn from_cos_sin(re: Fixed, im: Fixed) -> Rot2 {
        Rot2 { re, im }.renormalize()
    }

    /// Complex product, applying `other` first. Floors once per output, without normalization.
    /// Panics with 'Fixed: overflow' if an output leaves Q32.32; unit inputs cannot overflow.
    fn mul(self: Rot2, other: Rot2) -> Rot2 {
        Rot2 {
            re: mul_sub(self.re, other.re, self.im, other.im),
            im: dot2(self.re, other.im, self.im, other.re),
        }
    }

    /// Exact conjugate `(re, -im)`, the inverse for unit inputs.
    /// Panics with 'i64_neg Underflow' for an unchecked imaginary component of `fixed::MIN`.
    fn inverse(self: Rot2) -> Rot2 {
        Rot2 { re: self.re, im: -self.im }
    }

    /// Rotates `v` counter-clockwise, flooring each component once.
    /// Panics with 'Fixed: overflow' if the rotated vector leaves Q32.32.
    fn rotate(self: Rot2, v: Vec2) -> Vec2 {
        Vec2 { x: mul_sub(self.re, v.x, self.im, v.y), y: dot2(self.im, v.x, self.re, v.y) }
    }

    /// Rotates `v` by the conjugate, flooring each component once.
    /// Panics with 'Fixed: overflow' if the rotated vector leaves Q32.32.
    fn inverse_rotate(self: Rot2, v: Vec2) -> Vec2 {
        Vec2 { x: dot2(self.re, v.x, self.im, v.y), y: mul_sub(self.re, v.y, self.im, v.x) }
    }

    /// Tests the raw squared norm against `UNIT_TOL_SQ_RAW` (8 Q32.32 ulps).
    /// Unchecked `re = im = fixed::MIN` panics with 'i128_add Overflow'.
    fn is_unit(self: Rot2) -> bool {
        is_unit2_raw(self.re, self.im, UNIT_TOL_SQ_RAW)
    }

    /// Returns the normalized rotation: wide squared norm, floored length, nearest components.
    /// The unit tolerance is guaranteed for input length >= 1, not for tiny inputs: (1 ulp,
    /// 1 ulp) becomes (1, 1). Intended for cos/sin pairs and near-unit accumulated rotations.
    /// Panics with 'Rot2: zero' for zero; 'Fixed: overflow' for an unrepresentable result.
    fn renormalize(self: Rot2) -> Rot2 {
        assert(self.re != ZERO || self.im != ZERO, errors::ZERO);
        let (re, im) = normalize2(self.re, self.im);
        Rot2 { re, im }
    }

    /// Rapier's linearized angular update: `(re - d*im, im + d*re)`, then normalize,
    /// where `d = angvel * dt` floors. Products accumulate wide and floor once per component.
    /// `angvel` is radians/second and `dt` seconds; use small `|d|` for angular accuracy.
    /// Panics with 'Fixed: overflow' on out-of-range intermediates or 'Rot2: zero' on zero.
    fn integrate(self: Rot2, angvel: Fixed, dt: Fixed) -> Rot2 {
        let d = angvel * dt;
        Rot2 { re: mul_sub(self.re, ONE, d, self.im), im: mul_add(d, self.re, self.im) }
            .renormalize()
    }
}

/// Default is the exact identity; cannot panic.
pub impl Rot2Default of Default<Rot2> {
    fn default() -> Rot2 {
        IDENTITY
    }
}

/// Complex multiplication with the same flooring and overflow policy as `Rot2Trait::mul`.
pub impl Rot2Mul of Mul<Rot2> {
    fn mul(lhs: Rot2, rhs: Rot2) -> Rot2 {
        Rot2Trait::mul(lhs, rhs)
    }
}

#[cfg(test)]
mod alternatives {
    use fixed::Fixed;
    use glam::Vec2;
    use super::{Rot2, Rot2Trait};

    // Composed Vec2 arithmetic rounds each product before the vector addition (<= 1 ulp
    // difference from the fused result). Restricted to representable intermediate products.
    pub fn rotate(r: Rot2, v: Vec2) -> Vec2 {
        Vec2 { x: r.re, y: r.im } * Vec2 { x: v.x, y: v.x }
            + Vec2 { x: -r.im, y: r.re } * Vec2 { x: v.y, y: v.y }
    }
    pub fn inverse_rotate(r: Rot2, v: Vec2) -> Vec2 {
        Vec2 { x: r.re, y: -r.im } * Vec2 { x: v.x, y: v.x }
            + Vec2 { x: r.im, y: r.re } * Vec2 { x: v.y, y: v.y }
    }
    pub fn mul(a: Rot2, b: Rot2) -> Rot2 {
        let v = rotate(a, Vec2 { x: b.re, y: b.im });
        Rot2 { re: v.x, im: v.y }
    }
    pub fn integrate(r: Rot2, angvel: Fixed, dt: Fixed) -> Rot2 {
        let d = angvel * dt;
        Rot2 { re: r.re - d * r.im, im: r.im + d * r.re }.renormalize()
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, MAX, MIN, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_testing::opaque;
    use super::{IDENTITY, Rot2, Rot2Trait, alternatives};

    const R: Rot2 = Rot2 { re: Fixed { raw: 2576980378 }, im: Fixed { raw: 3435973837 } };
    const V: Vec2 = Vec2 { x: HALF, y: Fixed { raw: -8589934592 } };

    fn close(a: Vec2, b: Vec2, ulps: i64) {
        assert!(a.x.abs_diff_eq(b.x, Fixed { raw: ulps }));
        assert!(a.y.abs_diff_eq(b.y, Fixed { raw: ulps }));
    }

    #[test]
    fn test_identity_inverse_and_round_trips() {
        assert_eq!(Rot2Trait::IDENTITY, IDENTITY);
        assert_eq!(Default::<Rot2>::default(), IDENTITY);
        for r in array![
            IDENTITY, Rot2 { re: ZERO, im: ONE }, Rot2 { re: -ONE, im: ZERO }, R, R.inverse(),
        ]
            .span() {
            assert!((*r).is_unit());
            assert_eq!(*r * IDENTITY, *r);
            assert_eq!(IDENTITY * *r, *r);
            assert_eq!((*r).inverse().inverse(), *r);
            assert!((*r * (*r).inverse()).is_unit());
            for v in array![V, Vec2 { x: ZERO, y: ZERO }, Vec2 { x: -ONE, y: HALF }].span() {
                close((*r).inverse_rotate((*r).rotate(*v)), *v, 4);
                assert_eq!((*r).inverse_rotate(*v), (*r).inverse().rotate(*v));
            }
        }
    }

    #[test]
    fn test_normalization_and_linearized_integration() {
        for (re, im) in array![(TWO, ZERO), (ZERO, -TWO), (ONE, ONE), (MAX, MIN)].span() {
            assert!(Rot2Trait::from_cos_sin(*re, *im).is_unit());
        }
        // The frozen normalizer floors the length: tiny inputs do not promise a unit result.
        let tiny = Rot2Trait::from_cos_sin(Fixed { raw: 1 }, Fixed { raw: 1 });
        assert_eq!(tiny, Rot2 { re: ONE, im: ONE });
        assert!(!tiny.is_unit());
        for (w, dt) in array![(ZERO, ONE), (ONE, ZERO), (ONE, HALF), (-ONE, HALF), (TWO, -HALF)]
            .span() {
            let actual = IDENTITY.integrate(*w, *dt);
            let expected = Rot2Trait::from_cos_sin(ONE, *w * *dt);
            assert_eq!(actual, expected);
            assert!(actual.is_unit());
            let got = R.integrate(*w, *dt);
            let expected = alternatives::integrate(R, *w, *dt);
            close(Vec2 { x: got.re, y: got.im }, Vec2 { x: expected.re, y: expected.im }, 2);
        }
    }

    #[test]
    fn test_extremes_and_single_rescale() {
        let v = Vec2 { x: MIN, y: MAX };
        assert_eq!(IDENTITY.rotate(v), v);
        assert_eq!(IDENTITY.inverse_rotate(v), v);
        // Separate product floors lose one raw unit; the fused real part cancels exactly.
        let e = Fixed { raw: 1 };
        let r = Rot2 { re: e, im: e };
        let v = Vec2 { x: e, y: e };
        assert_eq!(r.rotate(v).x, ZERO);
        assert_eq!(alternatives::rotate(r, v).x.raw, -1);
        // No implicit normalization: preserve the norm error in unchecked inputs.
        let nonunit = Rot2 { re: TWO, im: ZERO };
        assert_eq!((nonunit * IDENTITY).re, TWO);
        assert!(!(nonunit * IDENTITY).is_unit());
    }

    #[test]
    #[should_panic(expected: ('Rot2: zero',))]
    fn test_from_zero_panics() {
        Rot2Trait::from_cos_sin(ZERO, ZERO);
    }

    #[test]
    #[should_panic(expected: ('Rot2: zero',))]
    fn test_renormalize_zero_panics() {
        Rot2 { re: ZERO, im: ZERO }.renormalize();
    }

    #[test]
    #[should_panic(expected: ('Fixed: overflow',))]
    fn test_rotate_overflow() {
        R.rotate(Vec2 { x: MAX, y: MAX });
    }

    #[test]
    #[should_panic(expected: ('Fixed: overflow',))]
    fn test_mul_overflow() {
        Rot2 { re: MAX, im: ZERO } * Rot2 { re: TWO, im: ZERO };
    }

    #[test]
    #[should_panic(expected: ('Fixed: overflow',))]
    fn test_integrate_overflow() {
        IDENTITY.integrate(MAX, TWO);
    }

    #[test]
    #[should_panic(expected: ('i64_neg Underflow',))]
    fn test_conjugate_min_panics() {
        Rot2 { re: ZERO, im: MIN }.inverse();
    }

    /// Products of these raws fit Q32.32; compare within the one-ulp difference between
    /// flooring a sum and adding separately floored products, for both signs.
    #[test]
    #[fuzzer(runs: 128, seed: 20260920)]
    fn fuzz_candidates(a: i32, b: i32, x: i32, y: i32) {
        let r = Rot2 { re: Fixed { raw: a.into() }, im: Fixed { raw: b.into() } };
        let v = Vec2 { x: Fixed { raw: x.into() }, y: Fixed { raw: y.into() } };
        close(r.rotate(v), alternatives::rotate(r, v), 1);
        close(r.inverse_rotate(v), alternatives::inverse_rotate(r, v), 1);
        let s = Rot2 { re: v.x, im: v.y };
        let product = r * s;
        let candidate = alternatives::mul(r, s);
        close(Vec2 { x: product.re, y: product.im }, Vec2 { x: candidate.re, y: candidate.im }, 1);
        assert_eq!(Rot2Trait::mul(r, s), product);
    }

    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_integrate(w: i32, dt: i32) {
        let w = Fixed { raw: w.into() };
        let dt = Fixed { raw: dt.into() };
        let a = R.integrate(w, dt);
        let b = alternatives::integrate(R, w, dt);
        close(Vec2 { x: a.re, y: a.im }, Vec2 { x: b.re, y: b.im }, 3);
        assert!(a.is_unit());
    }

    #[test]
    fn gas_baseline() {}
    #[test]
    fn gas_default() {
        assert_eq!(Default::<Rot2>::default(), IDENTITY);
    }
    #[test]
    fn gas_from_cos_sin() {
        assert!(Rot2Trait::from_cos_sin(opaque(R.re), opaque(R.im)).is_unit());
    }
    #[test]
    fn gas_renormalize() {
        assert!(opaque(R).renormalize().is_unit());
    }
    #[test]
    fn gas_is_unit() {
        assert!(opaque(R).is_unit());
    }
    #[test]
    fn gas_inverse() {
        assert_eq!(opaque(R).inverse(), Rot2 { re: R.re, im: -R.im });
    }
    #[test]
    fn gas_integrate_fused() {
        assert!(opaque(R).integrate(opaque(ONE), opaque(HALF)).is_unit());
    }
    #[test]
    fn gas_integrate_composed() {
        assert!(alternatives::integrate(opaque(R), opaque(ONE), opaque(HALF)).is_unit());
    }
    #[test]
    fn gas_rotate_fused() {
        assert_eq!(
            Rot2Trait::rotate(opaque(R), opaque(V)),
            Vec2 { x: Fixed { raw: 8160437863 }, y: Fixed { raw: -3435973838 } },
        );
    }
    #[test]
    fn gas_rotate_composed() {
        assert_eq!(
            alternatives::rotate(opaque(R), opaque(V)),
            Vec2 { x: Fixed { raw: 8160437863 }, y: Fixed { raw: -3435973838 } },
        );
    }
    #[test]
    fn gas_inverse_rotate_fused() {
        assert_eq!(
            Rot2Trait::inverse_rotate(opaque(R), opaque(V)),
            Vec2 { x: Fixed { raw: -5583457485 }, y: Fixed { raw: -6871947675 } },
        );
    }
    #[test]
    fn gas_inverse_rotate_composed() {
        assert_eq!(
            alternatives::inverse_rotate(opaque(R), opaque(V)),
            Vec2 { x: Fixed { raw: -5583457485 }, y: Fixed { raw: -6871947675 } },
        );
    }
    #[test]
    fn gas_mul_fused() {
        assert_eq!(
            Rot2Trait::mul(opaque(R), opaque(R)),
            Rot2 { re: Fixed { raw: -1202590843 }, im: Fixed { raw: 4123168605 } },
        );
    }
    #[test]
    fn gas_mul_composed() {
        assert_eq!(
            alternatives::mul(opaque(R), opaque(R)),
            Rot2 { re: Fixed { raw: -1202590843 }, im: Fixed { raw: 4123168604 } },
        );
    }
    #[test]
    fn gas_mul_operator() {
        assert_eq!(opaque(R) * opaque(IDENTITY), R);
    }
}
