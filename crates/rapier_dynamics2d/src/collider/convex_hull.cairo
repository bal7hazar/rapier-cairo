//! The 2D convex hull of a point set, for `ColliderBuilderTrait::convex_hull` (upstream parry
//! `transformation::convex_hull2` followed by `ConvexPolygon::from_convex_polyline`).
//!
//! Exact: every orientation test is the `i128` cross product of raw coordinate differences
//! (`rapier_geometry2d::point::cross_wide`), with no tolerance. Collinear points on an edge and
//! duplicates are dropped, so the hull is strictly convex. The vertices come out counter-clockwise
//! from the lexicographically smallest point (smallest `x`, then smallest `y`), whatever the
//! input order: the result depends on the point set only.
//!
//! Upstream's QuickHull starts the cycle at a vertex that depends on its facet history and
//! removes *nearly* collinear vertices (`sqrt(f64::EPSILON)` on the normals); the port starts at
//! the lexicographic minimum and removes exactly collinear ones. The polygon is the same up to
//! that start vertex and those near-collinear vertices.
//!
//! Candidates (`alternatives`, measured in `tests`): gift wrapping (shipped: no sort, no stack,
//! at most 9 passes over the points since a `ConvexPolygon` has at most 8 vertices) and Andrew's
//! monotone chain (insertion sort and a stack, both in dicts).

use glam::Vec2;
use rapier_geometry2d::point::cross_wide;
use rapier_math::math_ext::norm2::norm2_sq_wide;

/// The most vertices a `ConvexPolygon` holds; a larger hull is rejected.
pub const MAX_HULL_VERTICES: u32 = 8;

/// `true` when `a` precedes `b` in the lexicographic order (`x`, then `y`).
#[inline(always)]
pub fn lex_less(a: Vec2, b: Vec2) -> bool {
    a.x < b.x || (a.x == b.x && a.y < b.y)
}

/// Twice the signed area of `(a, b, c)`: positive for a counter-clockwise turn, exact.
/// #### Panics
/// * `'Fixed: overflow'` when a coordinate difference leaves the Q32.32 range.
#[inline(always)]
pub fn turn(a: Vec2, b: Vec2, c: Vec2) -> i128 {
    let ab = b - a;
    let ac = c - a;
    cross_wide(ab.x, ab.y, ac.x, ac.y)
}

/// The convex hull of `points`, counter-clockwise from the lexicographically smallest point, with
/// no duplicate and no collinear vertex; `None` when it has fewer than 3 vertices (fewer than 3
/// points, all points equal or collinear: upstream returns `None` or panics) or more than
/// [`MAX_HULL_VERTICES`] (no `ConvexPolygon` holds it).
///
/// Gift wrapping: from the current vertex, the next one is the point that leaves no other point
/// strictly to its right, the farthest one among collinear candidates. Cost: `O(n)` per vertex.
/// #### Panics
/// * `'Fixed: overflow'` when a coordinate difference leaves the Q32.32 range; the squared
///   distances and cross products are exact in `i128` for differences below `2^62` raw.
pub fn convex_hull(points: Span<Vec2>) -> Option<Array<Vec2>> {
    if points.len() < 3 {
        return None;
    }
    let mut start = *points.at(0);
    for p in points {
        if lex_less(*p, start) {
            start = *p;
        }
    }
    let mut hull = array![];
    let mut current = start;
    let mut closed = false;
    while hull.len() != MAX_HULL_VERTICES + 1 {
        hull.append(current);
        let mut next = current;
        let mut next_dist: i128 = 0;
        for r in points {
            let r = *r;
            if r != current {
                let d = r - current;
                let dist = norm2_sq_wide(d.x, d.y);
                if next == current {
                    next = r;
                    next_dist = dist;
                } else {
                    let t = turn(current, next, r);
                    if t < 0 || (t == 0 && dist > next_dist) {
                        next = r;
                        next_dist = dist;
                    }
                }
            }
        }
        if next == start || next == current {
            closed = true;
            break;
        }
        current = next;
    }
    if !closed || hull.len() < 3 || hull.len() > MAX_HULL_VERTICES {
        return None;
    }
    Some(hull)
}

#[cfg(test)]
pub mod alternatives {
    use core::dict::{Felt252Dict, Felt252DictTrait};
    use glam::Vec2;
    use super::{MAX_HULL_VERTICES, lex_less, turn};

    /// Andrew's monotone chain: insertion sort of the indices, then the lower and upper chains
    /// on a stack (both kept in dicts: a Cairo array has no `pop_back`). Same output as
    /// [`super::convex_hull`] (collinear and duplicate points popped by `turn <= 0`).
    pub fn convex_hull_monotone(points: Span<Vec2>) -> Option<Array<Vec2>> {
        let n = points.len();
        if n < 3 {
            return None;
        }
        let mut order: Felt252Dict<u32> = Default::default();
        let mut i: u32 = 0;
        while i != n {
            let p = *points.at(i);
            let mut j = i;
            while j != 0 && lex_less(p, *points.at(order.get((j - 1).into()))) {
                order.insert(j.into(), order.get((j - 1).into()));
                j -= 1;
            }
            order.insert(j.into(), i);
            i += 1;
        }
        let mut stack: Felt252Dict<u32> = Default::default();
        let mut len: u32 = 0;
        let mut k: u32 = 0;
        while k != n {
            let idx = order.get(k.into());
            push(ref stack, ref len, points, idx, 2);
            k += 1;
        }
        let floor = len + 1;
        k = n - 1;
        while k != 0 {
            k -= 1;
            let idx = order.get(k.into());
            push(ref stack, ref len, points, idx, floor);
        }
        // The last point pushed is the first one again.
        len -= 1;
        if len < 3 || len > MAX_HULL_VERTICES {
            return None;
        }
        let mut hull = array![];
        let mut m: u32 = 0;
        while m != len {
            hull.append(*points.at(stack.get(m.into())));
            m += 1;
        }
        Some(hull)
    }

    /// Pops while the top two and `points[idx]` do not turn counter-clockwise (keeping at least
    /// `floor - 1` entries), then pushes `idx`.
    fn push(ref stack: Felt252Dict<u32>, ref len: u32, points: Span<Vec2>, idx: u32, floor: u32) {
        let p = *points.at(idx);
        while len >= floor
            && turn(
                *points.at(stack.get((len - 2).into())), *points.at(stack.get((len - 1).into())), p,
            ) <= 0 {
            len -= 1;
        }
        stack.insert(len.into(), idx);
        len += 1;
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_testing::opaque;
    use super::alternatives::convex_hull_monotone;
    use super::convex_hull;

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn i(x: i32, y: i32) -> Vec2 {
        Vec2 { x: FixedTrait::from_int(x), y: FixedTrait::from_int(y) }
    }

    /// Both candidates give `expected` (`None` for a rejected set).
    fn check(points: Span<Vec2>, expected: Option<Span<Vec2>>) {
        let wrap = convex_hull(points);
        let chain = convex_hull_monotone(points);
        match expected {
            Some(hull) => {
                assert_eq!(wrap.unwrap().span(), hull);
                assert_eq!(chain.unwrap().span(), hull);
            },
            None => {
                assert!(wrap.is_none());
                assert!(chain.is_none());
            },
        }
    }

    /// (points, expected hull): shuffled squares, interior, duplicate and collinear points,
    /// degenerate sets, too many vertices.
    #[test]
    fn test_hull_table() {
        let square = array![i(-1, -1), i(1, -1), i(1, 1), i(-1, 1)].span();
        check(array![i(1, 1), i(-1, 1), i(1, -1), i(-1, -1)].span(), Some(square));
        // Interior points, duplicates and points in the middle of the edges disappear.
        check(
            array![
                i(0, 0), i(1, 1), i(-1, -1), i(0, -1), i(1, -1), i(1, 0), i(-1, 1), i(1, 1),
                i(0, 1), v(HALF, HALF), i(-1, 0), i(-1, -1),
            ]
                .span(),
            Some(square),
        );
        // A triangle, from any order.
        let triangle = array![i(0, 0), i(2, 0), i(0, 2)].span();
        check(array![i(0, 2), i(2, 0), i(0, 0)].span(), Some(triangle));
        check(array![i(0, 2), i(1, 1), i(2, 0), i(0, 0), i(0, 1)].span(), Some(triangle));
        // The lexicographic minimum starts the cycle: smallest x, then smallest y.
        check(
            array![i(0, 3), i(0, -3), i(2, 0), i(-2, 0), i(-2, 1)].span(),
            Some(array![i(-2, 0), i(0, -3), i(2, 0), i(0, 3), i(-2, 1)].span()),
        );
        // Degenerate: too few points, all equal, collinear.
        check(array![].span(), None);
        check(array![i(0, 0), i(1, 1)].span(), None);
        check(array![i(1, 1), i(1, 1), i(1, 1)].span(), None);
        check(array![i(0, 0), i(1, 1), i(2, 2), i(-3, -3)].span(), None);
        check(array![i(0, 5), i(0, 1), i(0, -2)].span(), None);
        // Nine hull vertices: more than a `ConvexPolygon` holds.
        check(
            array![
                i(4, 0), i(3, 3), i(0, 4), i(-3, 3), i(-4, 0), i(-3, -3), i(0, -4), i(3, -3),
                i(4, 1),
            ]
                .span(),
            None,
        );
        // Eight is fine, and fractional coordinates are exact.
        let octagon = array![
            i(-4, 0), i(-3, -3), i(0, -4), i(3, -3), i(4, 0), i(3, 3), i(0, 4), i(-3, 3),
        ]
            .span();
        check(
            array![
                i(3, 3), i(0, -4), i(-4, 0), i(0, 0), i(4, 0), i(-3, 3), i(3, -3), i(0, 4),
                i(-3, -3),
            ]
                .span(),
            Some(octagon),
        );
        let quarter = FixedTrait::from_raw(HALF.raw / 2);
        check(
            array![v(ZERO, ZERO), v(quarter, ZERO), v(ONE, ZERO), v(ZERO, TWO)].span(),
            Some(array![v(ZERO, ZERO), v(ONE, ZERO), v(ZERO, TWO)].span()),
        );
    }

    /// A coordinate in `-32..32` from the low bits of `bits`.
    fn coord(bits: u128) -> i32 {
        let low: u8 = (bits % 64).try_into().unwrap();
        let low: i32 = low.into();
        low - 32
    }

    /// Pseudo-random point clouds (LCG on the fuzzer seed): both candidates agree, every hull
    /// vertex turns strictly counter-clockwise and every point is inside or on the hull.
    #[test]
    #[fuzzer(runs: 64, seed: 20260925)]
    fn fuzz_candidates_agree(seed: u32, count: u8) {
        let n: u32 = 3 + (count % 14).into();
        let mut state: u128 = seed.into();
        let mut points = array![];
        let mut k = 0;
        while k != n {
            state = (state * 6364136223846793005 + 1442695040888963407) % 0x10000000000000000;
            points.append(i(coord(state / 0x100000000), coord(state / 0x1000000000000)));
            k += 1;
        }
        let wrap = convex_hull(points.span());
        let chain = convex_hull_monotone(points.span());
        assert_eq!(wrap, chain);
        if let Some(hull) = wrap {
            let h = hull.span();
            let m = h.len();
            let mut a = 0;
            while a != m {
                let b = (a + 1) % m;
                assert!(super::turn(*h.at(a), *h.at(b), *h.at((a + 2) % m)) > 0);
                for p in points.span() {
                    assert!(super::turn(*h.at(a), *h.at(b), *p) >= 0);
                }
                a += 1;
            }
        }
    }

    /// Shuffled clouds of 4, 8 and 16 points (4, 8 and 8 hull vertices).
    fn cloud(n: u32) -> Span<Vec2> {
        let all = array![
            i(3, 3), i(0, -4), i(-4, 0), i(1, 1), i(4, 0), i(-3, 3), i(3, -3), i(0, 4), i(-3, -3),
            i(-1, 2), i(2, -1), i(0, 0), i(-2, -2), i(2, 2), i(1, -2), i(-1, 1),
        ];
        if n == 4 {
            array![i(1, 1), i(-1, 1), i(1, -1), i(-1, -1)].span()
        } else if n == 8 {
            array![i(3, 3), i(0, -4), i(-4, 0), i(4, 0), i(-3, 3), i(3, -3), i(0, 4), i(-3, -3)]
                .span()
        } else {
            all.span()
        }
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(cloud(16));
    }

    #[test]
    fn gas_convex_hull_4() {
        let _ = convex_hull(opaque(cloud(4)));
    }

    #[test]
    fn gas_convex_hull_8() {
        let _ = convex_hull(opaque(cloud(8)));
    }

    #[test]
    fn gas_convex_hull_16() {
        let _ = convex_hull(opaque(cloud(16)));
    }

    #[test]
    fn gas_convex_hull_monotone_4() {
        let _ = convex_hull_monotone(opaque(cloud(4)));
    }

    #[test]
    fn gas_convex_hull_monotone_8() {
        let _ = convex_hull_monotone(opaque(cloud(8)));
    }

    #[test]
    fn gas_convex_hull_monotone_16() {
        let _ = convex_hull_monotone(opaque(cloud(16)));
    }
}
