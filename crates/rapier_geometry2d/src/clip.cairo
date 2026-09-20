//! Segment clipping with exact wide projection comparisons and upstream point order.
//! Inputs and displacements must fit Q32.32. Projection spans must be below 2^95 raw
//! Q64.64 units (coordinates bounded by 8192 are sufficient). Ratios and products floor.
use core::num::traits::WideMul;
use fixed::{Fixed, ZERO};
use glam::Vec2;

/// A pair of points and their features: 0 first vertex, 1 interior, 2 second vertex.
pub type ClippingPoints = (Vec2, Vec2, u32, u32);

pub mod errors {
    pub const PROJECTION_RANGE: felt252 = 'Clip: projection range';
}

fn dot(a: Vec2, b: Vec2) -> i128 {
    a.x.raw.wide_mul(b.x.raw) + a.y.raw.wide_mul(b.y.raw)
}

fn ratio(n: i128, d: i128) -> Fixed {
    if d == 0 {
        return ZERO;
    }
    // Intersected ranges guarantee 0 <= n <= d. Keep the division wide: even a
    // one-ulp segment has a nonzero squared projection.
    assert(d < 39614081257132168796771975168, errors::PROJECTION_RANGE);
    let n: u128 = n.try_into().unwrap();
    let d: u128 = d.try_into().unwrap();
    Fixed { raw: (n * 4294967296 / d).try_into().unwrap() }
}

fn interpolate(a: Vec2, b: Vec2, n: i128, d: i128) -> Vec2 {
    let t = ratio(n, d);
    let delta = b - a;
    a + Vec2 { x: delta.x * t, y: delta.y * t }
}

fn clip_ranges(
    a: Vec2, b: Vec2, c: Vec2, d: Vec2, ra: i128, rb: i128, rc: i128, rd: i128, guarded: bool,
) -> Option<(ClippingPoints, ClippingPoints)> {
    let (a, b, ra, rb, fa, fb) = if rb < ra {
        (b, a, rb, ra, 2, 0)
    } else {
        (a, b, ra, rb, 0, 2)
    };
    let (c, d, rc, rd, fc, fd) = if rd < rc {
        (d, c, rd, rc, 2, 0)
    } else {
        (c, d, rc, rd, 0, 2)
    };
    if rc > rb || ra > rd {
        return None;
    }
    // The plain upstream routine divides by zero only if the chosen branch uses
    // the degenerate range. Preserve finite point-in-segment results.
    if !guarded
        && ((rc > ra && rb == ra)
            || (rc <= ra && rd == rc)
            || (rd < rb && rb == ra)
            || (rd >= rb && rd == rc)) {
        return None;
    }
    let ca = if rc > ra {
        (interpolate(a, b, rc - ra, rb - ra), c, 1, fc)
    } else {
        (a, interpolate(c, d, ra - rc, rd - rc), fa, 1)
    };
    let cb = if rd < rb {
        (interpolate(a, b, rd - ra, rb - ra), d, 1, fd)
    } else {
        (b, interpolate(c, d, rb - rc, rd - rc), fb, 1)
    };
    Some((ca, cb))
}

/// Clips on segment 1's tangent, returning points in that direction's order.
/// Returns `None` for disjoint projections or a selected zero divisor (upstream NaN).
/// Floors interpolation; panics on coordinate overflow or `Clip: projection range`.
pub fn clip_segment_segment(
    a1: Vec2, b1: Vec2, a2: Vec2, b2: Vec2,
) -> Option<((Vec2, Vec2), (Vec2, Vec2))> {
    match clip_segment_segment_with_features((a1, b1), (a2, b2)) {
        Some(((a, b, _, _), (c, d, _, _))) => Some(((a, b), (c, d))),
        None => None,
    }
}

/// Upstream's feature-carrying plain clipping result, including strict endpoint ties.
/// Same bounds, floor rounding and panic/degeneracy policy as `clip_segment_segment`.
pub fn clip_segment_segment_with_features(
    seg1: (Vec2, Vec2), seg2: (Vec2, Vec2),
) -> Option<(ClippingPoints, ClippingPoints)> {
    let (a, b) = seg1;
    let (c, d) = seg2;
    let t = b - a;
    if t.x == ZERO && t.y == ZERO {
        return None;
    }
    clip_ranges(a, b, c, d, 0, dot(t, t), dot(c - a, t), dot(d - a, t), false)
}

/// Clips along `normal`, ordered on `(-normal.y, normal.x)` (not segment 1).
/// Zero projection spans use a zero ratio, like upstream `inv(0)`. Normal need not
/// be unit. Floors interpolation; same coordinate/projection bounds and panics as above.
pub fn clip_segment_segment_with_normal(
    seg1: (Vec2, Vec2), seg2: (Vec2, Vec2), normal: Vec2,
) -> Option<(ClippingPoints, ClippingPoints)> {
    let t = Vec2 { x: -normal.y, y: normal.x };
    let (a, b) = seg1;
    let (c, d) = seg2;
    clip_ranges(a, b, c, d, dot(a, t), dot(b, t), dot(c, t), dot(d, t), true)
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_testing::opaque;
    use super::{
        clip_segment_segment, clip_segment_segment_with_features, clip_segment_segment_with_normal,
    };
    const O: Vec2 = Vec2 { x: ZERO, y: ZERO };
    const X: Vec2 = Vec2 { x: ONE, y: ZERO };
    const Y: Vec2 = Vec2 { x: ZERO, y: ONE };
    const B: Vec2 = Vec2 { x: TWO, y: ZERO };

    #[test]
    fn test_degenerate_and_tiny() {
        for p in array![O, X, Y].span() {
            assert!(clip_segment_segment(*p, *p, O, B).is_none());
            assert!(clip_segment_segment(O, B, *p, *p).is_none() || *p == X);
            assert!(clip_segment_segment_with_normal((*p, *p), (*p, *p), Y).is_some());
        }
        let tiny = Vec2 { x: Fixed { raw: 1 }, y: ZERO };
        assert_eq!(clip_segment_segment(O, tiny, O, tiny), Some(((O, O), (tiny, tiny))));
        assert_eq!(clip_segment_segment(O, B, X, X), Some(((X, X), (X, X))));
    }
    #[test]
    fn test_order_and_ties() {
        assert_eq!(
            clip_segment_segment_with_features((O, B), (O, B)), Some(((O, O, 0, 1), (B, B, 2, 1))),
        );
        assert_eq!(
            clip_segment_segment_with_normal((O, B), (O, B), Y), Some(((B, B, 2, 1), (O, O, 0, 1))),
        );
        assert_eq!(clip_segment_segment(O, X, X, B), Some(((X, X), (X, X))));
    }
    #[test]
    #[should_panic(expected: 'Clip: projection range')]
    fn test_projection_overflow_policy() {
        let far = Vec2 { x: Fixed { raw: 281474976710656 }, y: ZERO };
        let _ = clip_segment_segment(O, far, O, far);
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque((O, B));
    }
    #[test]
    fn gas_clip_segment_segment() {
        let (a, b) = opaque((O, B));
        let _ = clip_segment_segment(a, b, X, B);
    }
    #[test]
    fn gas_clip_segment_segment_with_features() {
        let _ = clip_segment_segment_with_features(opaque((O, B)), (X, B));
    }
    #[test]
    fn gas_clip_segment_segment_with_normal() {
        let _ = clip_segment_segment_with_normal(opaque((O, B)), (X, B), Y);
    }
}
