//! The 2D vector operations Rapier needs beyond glam's `Vec2` API.
//!
//! Vectors are plain component tuples here: the `Vec2`, `Rot2` and `Pose2` types belong to
//! package M2 and will be built on these functions. Every product goes through a fused kernel of
//! `fixed::wide` (one rescale per output scalar) and every length test goes through
//! [`super::norm2`] (no rescale at all).
//!
//! Upstream reference: `rapier::utils::{CrossProduct, DotProduct, OrthonormalBasis}` and
//! `rapier::utils::try_normalize_and_get_length`.

use fixed::Fixed;
use fixed::wide::{NormTrait, RecipTrait, dot2, mul_sub, norm2_wide};
use super::norm2::{is_norm2_gt, is_norm2_le};

/// Computes the dot product `a . b` with a single rescale.
///
/// Mirrors `rapier::utils::DotProduct::gdot` for `Vector2` (`glam::Vec2::dot`).
/// #### Panics
/// * `'Fixed: overflow'` if the result does not fit the scalar range.
/// #### Deviations
/// * None.
#[inline(always)]
pub fn gdot(ax: Fixed, ay: Fixed, bx: Fixed, by: Fixed) -> Fixed {
    dot2(ax, bx, ay, by)
}

/// Computes the generalised cross product of two 2D vectors, `ax * by - ay * bx` (the perp-dot
/// product, i.e. the `z` component of the 3D cross product), with a single rescale.
///
/// Mirrors `rapier::utils::CrossProduct<Vector> for Vector` in 2D (`glam::Vec2::perp_dot`).
/// #### Panics
/// * `'Fixed: overflow'` if the result does not fit the scalar range.
/// #### Deviations
/// * The two products are not rounded before being subtracted: the result is the floor of the
///   exact difference, which `fixed::wide::mul_sub` documents. Antisymmetry therefore holds only
///   up to 1 ulp (`gcross_vv(a, b)` and `-gcross_vv(b, a)` straddle the exact value when it is
///   not a multiple of 1 ulp); upstream, rounding to nearest, has the same kind of asymmetry.
#[inline(always)]
pub fn gcross_vv(ax: Fixed, ay: Fixed, bx: Fixed, by: Fixed) -> Fixed {
    mul_sub(ax, by, ay, bx)
}

/// Computes the generalised cross product of the out-of-plane scalar `s` with the vector
/// `(x, y)`: `(-y * s, x * s)`.
///
/// Mirrors `rapier::utils::CrossProduct<Vector> for Real` in 2D — the `omega x r` of the solver,
/// which turns an angular velocity into the linear velocity of a point.
/// #### Panics
/// * `'Fixed: overflow'` if either component does not fit the scalar range.
/// #### Deviations
/// * `Fixed * Fixed` floors, so this is not exactly `-gcross_vs(x, y, s)`: the two straddle the
///   exact value and can differ by 1 ulp per component.
#[inline(always)]
pub fn gcross_sv(s: Fixed, x: Fixed, y: Fixed) -> (Fixed, Fixed) {
    (-y * s, x * s)
}

/// Computes the generalised cross product of the vector `(x, y)` with the out-of-plane scalar
/// `s`: `(y * s, -x * s)`, the negation of [`gcross_sv`].
///
/// Mirrors `WCross<Real> for Vector2` of rapier <= 0.34 (`r x omega`); the referenced upstream
/// revision only keeps the scalar-first form, but the constraint builders use both orders.
/// #### Panics
/// * `'Fixed: overflow'` if either component does not fit the scalar range.
/// #### Deviations
/// * See [`gcross_sv`]: the two orders are opposite up to the 1 ulp of their floor.
#[inline(always)]
pub fn gcross_vs(x: Fixed, y: Fixed, s: Fixed) -> (Fixed, Fixed) {
    (y * s, -x * s)
}

/// Returns `(x, y)` rotated by a quarter turn counter-clockwise: `(-y, x)`.
///
/// Mirrors `glam::Vec2::perp`.
/// #### Panics
/// * `'i64_neg Overflow'` if `y` is `fixed::MIN`.
/// #### Deviations
/// * None.
#[inline(always)]
pub fn perp(x: Fixed, y: Fixed) -> (Fixed, Fixed) {
    (-y, x)
}

/// Returns a vector orthogonal to `(x, y)` of the same length: `(-y, x)`.
///
/// Mirrors `rapier::utils::OrthonormalBasis::orthonormal_vector` for `Vector2`, whose 2D
/// implementation is exactly `perp` (it is unit-length when the input is).
/// #### Panics
/// * `'i64_neg Overflow'` if `y` is `fixed::MIN`.
/// #### Deviations
/// * None.
#[inline(always)]
pub fn orthonormal_vector(x: Fixed, y: Fixed) -> (Fixed, Fixed) {
    (-y, x)
}

/// Normalises `(x, y)`, or returns `None` when it is exactly the zero vector.
///
/// Mirrors `glam::Vec2::try_normalize` (whose threshold is the zero test of the length).
/// The zero test is the wide one — `x^2 + y^2 = 0` iff both components are 0 — so it never
/// underflows.
///
/// **Precision cliff**: the length is floored to 1 ulp before the division, so a vector of a few
/// ulp is normalised from almost no information at all (`(1 ulp, 1 ulp)` returns `(1, 1)`, of
/// length `sqrt(2)`). Use [`try_normalize2_eps`] with `consts::DEFAULT_EPSILON` wherever
/// upstream guards a normalisation with a tolerance, which is nearly everywhere.
/// #### Panics
/// * `'Fixed: overflow'` if a component of the result leaves the scalar range, which can only
///   happen for the inputs described above.
/// #### Deviations
/// * Each component is rounded to nearest (`fixed::wide::Recip`), so the result is exact for
///   axis-aligned vectors.
#[inline(always)]
pub fn try_normalize2(x: Fixed, y: Fixed) -> Option<(Fixed, Fixed)> {
    match norm2_wide(x, y).try_recip() {
        Some(r) => Some((r.mul(x), r.mul(y))),
        None => None,
    }
}

/// Normalises `(x, y)`, or returns `None` when its length is not strictly above `min_len`.
///
/// Mirrors `rapier::utils::try_normalize_and_get_length(v, threshold)` and the
/// `len >= DEFAULT_EPSILON` guards of parry, without their square root: the threshold test is the
/// wide squared comparison of [`is_norm2_gt`].
/// #### Panics
/// * `'i128_add Overflow'` for `x = y = fixed::MIN`.
/// #### Deviations
/// * None (with `min_len = 0` this is [`try_normalize2`]).
#[inline(always)]
pub fn try_normalize2_eps(x: Fixed, y: Fixed, min_len: Fixed) -> Option<(Fixed, Fixed)> {
    if is_norm2_gt(x, y, min_len) {
        let r = norm2_wide(x, y).recip();
        Some((r.mul(x), r.mul(y)))
    } else {
        None
    }
}

/// Normalises `(x, y)` and returns its length as well, or `None` when that length is not
/// strictly above `min_len`.
///
/// Mirrors `rapier::utils::try_normalize_and_get_length`, which the solver uses to normalise a
/// direction and keep its magnitude in one go.
/// #### Panics
/// * `'i128_add Overflow'` for `x = y = fixed::MIN`.
/// * `'Fixed: overflow'` if the length itself does not fit the scalar range (`|v| >= 2^31`),
///   although the normalisation would.
/// #### Deviations
/// * The returned length is floored to 1 ulp (`fixed::wide::norm2`).
#[inline(always)]
pub fn try_normalize2_and_length(
    x: Fixed, y: Fixed, min_len: Fixed,
) -> Option<(Fixed, Fixed, Fixed)> {
    if is_norm2_gt(x, y, min_len) {
        let n = norm2_wide(x, y);
        let r = n.recip();
        Some((r.mul(x), r.mul(y), n.to_fixed()))
    } else {
        None
    }
}

/// Scales `(x, y)` down so that its length is at most `max`, leaving it untouched otherwise.
///
/// Mirrors `nalgebra`'s `cap_magnitude`, applied by the contact solver to tangent impulses. The
/// "is it already short enough" test is the wide squared comparison, so it neither underflows
/// nor overflows.
/// #### Panics
/// * `'i128_add Overflow'` for `x = y = fixed::MIN`.
/// * `'Fixed: overflow'` if a scaled component leaves the scalar range.
/// #### Deviations
/// * `max` is assumed non-negative, as upstream does; a negative `max` flips the vector.
#[inline(always)]
pub fn cap_magnitude2(x: Fixed, y: Fixed, max: Fixed) -> (Fixed, Fixed) {
    if is_norm2_le(x, y, max) {
        (x, y)
    } else {
        let r = norm2_wide(x, y).recip();
        (r.mul(x) * max, r.mul(y) * max)
    }
}

#[cfg(test)]
mod tests {
    use fixed::wide::normalize2;
    use fixed::{Fixed, FixedTrait, HALF, MAX, ONE, TWO, ZERO};
    use rapier_testing::opaque;
    use crate::consts::{DEFAULT_EPSILON, UNIT_TOL_ULPS};
    use super::super::norm2::{is_norm2_gt, is_unit2, norm2_sq_wide};
    use super::{
        cap_magnitude2, gcross_sv, gcross_vs, gcross_vv, gdot, orthonormal_vector, perp,
        try_normalize2, try_normalize2_and_length, try_normalize2_eps,
    };

    // ---------------------------------------------------------------- products

    #[test]
    fn test_gdot() {
        let three: Fixed = FixedTrait::from_int(3);
        let four: Fixed = FixedTrait::from_int(4);
        assert_eq!(gdot(three, four, three, four), FixedTrait::from_int(25));
        assert_eq!(gdot(ONE, ZERO, ZERO, ONE), ZERO);
        assert_eq!(gdot(ONE, ZERO, -ONE, ZERO), -ONE);
        assert_eq!(gdot(ZERO, ZERO, MAX, MAX), ZERO);
        // The fused kernel keeps the cross terms exact: 0.5 * 0.5 + 0.5 * 0.5 = 0.5.
        assert_eq!(gdot(HALF, HALF, HALF, HALF), HALF);
    }

    #[test]
    fn test_gcross_vv_is_the_perp_dot() {
        let three: Fixed = FixedTrait::from_int(3);
        let four: Fixed = FixedTrait::from_int(4);
        assert_eq!(gcross_vv(ONE, ZERO, ZERO, ONE), ONE);
        assert_eq!(gcross_vv(ZERO, ONE, ONE, ZERO), -ONE);
        assert_eq!(gcross_vv(three, four, three, four), ZERO);
        assert_eq!(gcross_vv(three, four, -four, three), FixedTrait::from_int(25));
        // Antisymmetry.
        assert_eq!(gcross_vv(three, four, ONE, TWO), -gcross_vv(ONE, TWO, three, four));
    }

    /// The single rescale of `gcross_vv` floors, so swapping its arguments can move the result
    /// by 1 ulp. Documented on the function; pinned here.
    #[test]
    fn test_gcross_vv_antisymmetry_is_only_ulp_exact() {
        let e: Fixed = FixedTrait::from_raw(1);
        // e * e = 2^-64, a positive value below 1 ulp: it floors to 0 one way, to -1 ulp the
        // other way round.
        assert_eq!(gcross_vv(e, ZERO, ZERO, e), ZERO);
        assert_eq!(gcross_vv(ZERO, e, e, ZERO), -e);
        assert!(gcross_vv(e, ZERO, ZERO, e).abs_diff_eq(-gcross_vv(ZERO, e, e, ZERO), e));
    }

    #[test]
    fn test_gcross_with_a_scalar() {
        let two_x: (Fixed, Fixed) = (TWO, ZERO);
        let (x, y) = two_x;
        assert_eq!(gcross_sv(ONE, x, y), (ZERO, TWO));
        assert_eq!(gcross_vs(x, y, ONE), (ZERO, -TWO));
        // The two orders are opposite, as in 3D.
        let (ax, ay) = gcross_sv(HALF, ONE, TWO);
        let (bx, by) = gcross_vs(ONE, TWO, HALF);
        assert_eq!((ax, ay), (-bx, -by));
        // A zero angular velocity moves nothing.
        assert_eq!(gcross_sv(ZERO, MAX, MAX), (ZERO, ZERO));
    }

    #[test]
    fn test_perp_and_orthonormal_vector() {
        assert_eq!(perp(ONE, ZERO), (ZERO, ONE));
        assert_eq!(perp(ZERO, ONE), (-ONE, ZERO));
        assert_eq!(orthonormal_vector(ONE, TWO), perp(ONE, TWO));
        // Orthogonal, same length, and unit when the input is.
        let (px, py) = perp(HALF, -TWO);
        assert_eq!(gdot(HALF, -TWO, px, py), ZERO);
        assert_eq!(norm2_sq_wide(px, py), norm2_sq_wide(HALF, -TWO));
        let (ux, uy) = normalize2(FixedTrait::from_int(3), FixedTrait::from_int(4));
        let (ox, oy) = orthonormal_vector(ux, uy);
        assert!(is_unit2(ox, oy, UNIT_TOL_ULPS));
    }

    // ---------------------------------------------------------------- normalisation

    #[test]
    fn test_try_normalize2() {
        assert_eq!(try_normalize2(ZERO, ZERO), None);
        assert_eq!(try_normalize2(TWO, ZERO), Some((ONE, ZERO)));
        assert_eq!(try_normalize2(ZERO, -TWO), Some((ZERO, -ONE)));
        let (nx, ny) = try_normalize2(FixedTrait::from_int(3), FixedTrait::from_int(4)).unwrap();
        assert!(is_unit2(nx, ny, UNIT_TOL_ULPS));
        assert_eq!((nx, ny), normalize2(FixedTrait::from_int(3), FixedTrait::from_int(4)));
    }

    /// The documented cliff: a vector of a few ulp normalises to nonsense, which is why the
    /// guarded form exists.
    #[test]
    fn test_try_normalize2_cliff_and_the_guard() {
        let one_ulp: Fixed = FixedTrait::from_raw(1);
        let (nx, ny) = try_normalize2(one_ulp, one_ulp).unwrap();
        assert!(!is_unit2(nx, ny, UNIT_TOL_ULPS));
        assert_eq!(try_normalize2_eps(one_ulp, one_ulp, DEFAULT_EPSILON), None);
    }

    #[test]
    fn test_try_normalize2_eps() {
        // Just above and just below the tolerance, resolved to the ulp.
        let below: Fixed = FixedTrait::from_raw(512);
        let above: Fixed = FixedTrait::from_raw(513);
        assert_eq!(try_normalize2_eps(below, ZERO, DEFAULT_EPSILON), None);
        assert!(try_normalize2_eps(above, ZERO, DEFAULT_EPSILON).is_some());
        assert_eq!(try_normalize2_eps(ZERO, ZERO, DEFAULT_EPSILON), None);
        assert_eq!(try_normalize2_eps(TWO, ZERO, DEFAULT_EPSILON), Some((ONE, ZERO)));
        // With a zero threshold it is `try_normalize2`.
        assert_eq!(try_normalize2_eps(TWO, ZERO, ZERO), try_normalize2(TWO, ZERO));
        assert_eq!(try_normalize2_eps(ZERO, ZERO, ZERO), try_normalize2(ZERO, ZERO));
    }

    #[test]
    fn test_try_normalize2_and_length() {
        let (nx, ny, len) = try_normalize2_and_length(
            FixedTrait::from_int(3), FixedTrait::from_int(4), DEFAULT_EPSILON,
        )
            .unwrap();
        assert_eq!(len, FixedTrait::from_int(5));
        assert!(is_unit2(nx, ny, UNIT_TOL_ULPS));
        assert_eq!(try_normalize2_and_length(ZERO, ZERO, DEFAULT_EPSILON), None);
        // The length is exact when it is representable.
        let (_, _, len) = try_normalize2_and_length(ZERO, -TWO, ZERO).unwrap();
        assert_eq!(len, TWO);
    }

    // ---------------------------------------------------------------- cap_magnitude2

    #[test]
    fn test_cap_magnitude2() {
        let three: Fixed = FixedTrait::from_int(3);
        let four: Fixed = FixedTrait::from_int(4);
        let five: Fixed = FixedTrait::from_int(5);
        // Shorter than the cap: untouched, to the bit.
        assert_eq!(cap_magnitude2(three, four, FixedTrait::from_int(6)), (three, four));
        // Exactly at the cap: untouched.
        assert_eq!(cap_magnitude2(three, four, five), (three, four));
        // Longer: scaled down to the cap, keeping the direction.
        let (cx, cy) = cap_magnitude2(FixedTrait::from_int(6), FixedTrait::from_int(8), five);
        assert!(cx.abs_diff_eq(three, FixedTrait::from_raw(4)));
        assert!(cy.abs_diff_eq(four, FixedTrait::from_raw(4)));
        // A zero cap zeroes the vector, and the zero vector survives any cap.
        assert_eq!(cap_magnitude2(three, four, ZERO), (ZERO, ZERO));
        assert_eq!(cap_magnitude2(ZERO, ZERO, five), (ZERO, ZERO));
    }

    // ---------------------------------------------------------------- fuzz

    const LCG_MUL: u128 = 6364136223846793005;
    const LCG_INC: u128 = 1442695040888963407;
    const NZ_TWO_POW_64: NonZero<u128> = 0x10000000000000000;
    const NZ_TWO_POW_32: NonZero<u128> = 0x100000000;
    const NZ_TWO_POW_16: NonZero<u64> = 0x10000;

    fn next(ref state: u64) -> u32 {
        let (_, low) = DivRem::div_rem(state.into() * LCG_MUL + LCG_INC, NZ_TWO_POW_64);
        state = low.try_into().unwrap();
        let (high, _) = DivRem::div_rem(low, NZ_TWO_POW_32);
        high.try_into().unwrap()
    }

    /// Draws a raw value in `[-2^47, 2^47)` (values in `[-32768, 32768)`): products of two such
    /// values stay inside the scalar range.
    fn draw(ref state: u64) -> Fixed {
        let high: u64 = next(ref state).into();
        let low: u64 = next(ref state).into();
        let (_, low) = DivRem::div_rem(low, NZ_TWO_POW_16);
        let raw: i64 = (high * 65536 + low).try_into().unwrap();
        Fixed { raw: raw - 140737488355328 } // 2^47
    }

    /// Draws a raw value in `[-2^31, 2^31)` (values in `[-0.5, 0.5)`): the product of two such
    /// values always fits the scalar range.
    fn draw_small(ref state: u64) -> Fixed {
        let v: u64 = next(ref state).into();
        let raw: i64 = v.try_into().unwrap();
        Fixed { raw: raw - 2147483648 }
    }

    /// Identities that hold for every vector: `perp` is orthogonal and length-preserving,
    /// `gcross_vv` is antisymmetric, and the two scalar cross products are opposite.
    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_identities(seed: u64) {
        let mut state = seed | 1;
        let mut i: u32 = 0;
        while i != 8 {
            let ax = draw_small(ref state);
            let ay = draw_small(ref state);
            let bx = draw_small(ref state);
            let by = draw_small(ref state);
            let (px, py) = perp(ax, ay);
            assert_eq!(gdot(ax, ay, px, py), ZERO);
            assert_eq!(norm2_sq_wide(px, py), norm2_sq_wide(ax, ay));
            assert_eq!(orthonormal_vector(ax, ay), (px, py));
            // gdot is symmetric exactly (the same wide sum is narrowed on both sides), while
            // gcross is antisymmetric only up to the floor of its single rescale.
            assert!(
                gcross_vv(ax, ay, bx, by)
                    .abs_diff_eq(-gcross_vv(bx, by, ax, ay), FixedTrait::from_raw(1)),
            );
            assert_eq!(gdot(ax, ay, bx, by), gdot(bx, by, ax, ay));
            // The perp-dot of a vector with its own perp is its squared length.
            assert_eq!(gcross_vv(ax, ay, px, py), gdot(ax, ay, ax, ay));
            // The two cross-product orders differ by a sign, up to the floor of their rescale.
            let (sx, sy) = gcross_sv(bx, ax, ay);
            let (vx, vy) = gcross_vs(ax, ay, bx);
            let one_ulp: Fixed = FixedTrait::from_raw(1);
            assert!(sx.abs_diff_eq(-vx, one_ulp));
            assert!(sy.abs_diff_eq(-vy, one_ulp));
            i += 1;
        }
    }

    /// `try_normalize2_eps` agrees with `try_normalize2` above the threshold and rejects below
    /// it, and what it returns is a unit vector whenever the input is long enough to define one.
    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_try_normalize2(seed: u64) {
        let mut state = seed | 1;
        let mut i: u32 = 0;
        while i != 8 {
            let x = draw(ref state);
            let y = draw(ref state);
            match try_normalize2_eps(x, y, DEFAULT_EPSILON) {
                Some((
                    nx, ny,
                )) => {
                    assert_eq!(Some((nx, ny)), try_normalize2(x, y));
                    // `normalize2` has a relative error of `1 ulp / L`, so at the threshold
                    // `L = 2^-23` the squared norm may be off by `2 * 2^-9 = 2^24` ulp; 2^25
                    // leaves the usual factor-2 margin (`consts`).
                    assert!(is_unit2(nx, ny, 33554432));
                },
                None => { assert!(!is_norm2_gt(x, y, DEFAULT_EPSILON)); },
            }
            i += 1;
        }
    }

    // ---------------------------------------------------------------- gas

    /// Empty probe: the harness overhead to subtract from the entries below.
    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_gdot() {
        assert_eq!(gdot(opaque(ONE), opaque(ZERO), opaque(ONE), opaque(ZERO)), ONE);
    }

    #[test]
    fn gas_gcross_vv() {
        assert_eq!(gcross_vv(opaque(ONE), opaque(ZERO), opaque(ZERO), opaque(ONE)), ONE);
    }

    #[test]
    fn gas_gcross_sv() {
        assert_eq!(gcross_sv(opaque(ONE), opaque(ONE), opaque(ZERO)), (ZERO, ONE));
    }

    #[test]
    fn gas_gcross_vs() {
        assert_eq!(gcross_vs(opaque(ONE), opaque(ZERO), opaque(ONE)), (ZERO, -ONE));
    }

    #[test]
    fn gas_perp() {
        assert_eq!(perp(opaque(ONE), opaque(ZERO)), (ZERO, ONE));
    }

    #[test]
    fn gas_orthonormal_vector() {
        assert_eq!(orthonormal_vector(opaque(ONE), opaque(ZERO)), (ZERO, ONE));
    }

    #[test]
    fn gas_try_normalize2() {
        assert_eq!(try_normalize2(opaque(TWO), opaque(ZERO)), Some((ONE, ZERO)));
    }

    #[test]
    fn gas_try_normalize2_eps() {
        assert_eq!(
            try_normalize2_eps(opaque(TWO), opaque(ZERO), opaque(DEFAULT_EPSILON)),
            Some((ONE, ZERO)),
        );
    }

    #[test]
    fn gas_try_normalize2_and_length() {
        assert_eq!(
            try_normalize2_and_length(opaque(TWO), opaque(ZERO), opaque(DEFAULT_EPSILON)),
            Some((ONE, ZERO, TWO)),
        );
    }

    #[test]
    fn gas_cap_magnitude2() {
        assert_eq!(cap_magnitude2(opaque(TWO), opaque(ZERO), opaque(ONE)), (ONE, ZERO));
    }

    /// The whole point of the package, end to end: a guarded normalisation of a vector far below
    /// the resolution of a rescaled squared length, which costs the same as any other.
    #[test]
    fn gas_try_normalize2_eps_rejects_tiny() {
        assert_eq!(
            try_normalize2_eps(
                opaque(FixedTrait::from_raw(4)), opaque(ZERO), opaque(DEFAULT_EPSILON),
            ),
            None,
        );
    }
}
