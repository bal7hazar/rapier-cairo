//! Distance between two TOI proxies (Parry `sweep_toi/proxy_distance.rs`), answered by an exact
//! closest-feature kernel instead of upstream's warm-started GJK (ADR 0001 entries 14, 17, 24).
//!
//! A proxy is a convex point cloud (one point, a segment, or a convex polygon in order) dilated
//! by a radius. Two disjoint cores are closest at a vertex of one against an edge of the other:
//! every vertex is projected on every edge of the other proxy (exact wide squared distances, the
//! first minimum wins), and the pair reports its features as upstream's GJK simplex would: one
//! vertex on each side (`count = 1`) or an edge of one side against a vertex of the other
//! (`count = 2`, the vertex index repeated). Overlapping cores (an edge crossing, or a vertex of
//! one inside the other) answer a zero distance, as upstream's full simplex does.
//!
//! # Candidates
//!
//! `super::tests::gas_proxy_distance_*` (box against a turned box): **visibility pruning** (a
//! vertex is only projected on the edges it sees from outside): 1.77M gas / 12,626 steps; the
//! exhaustive search ([`proxy_distance_exhaustive`], same answers, `tests::test_pruning_agrees`):
//! 2.41M / 14,157.
//!
//! [`SimplexCache`] is kept for upstream's signature and for the separation functions of
//! `super::separation`: the kernel is exact, so the cache is written but never read.

use fixed::wide::{NormTrait, norm2_wide};
use fixed::{Fixed, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::math_ext::norm2::norm2_sq_wide;
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::point::cross_wide;
use crate::point::wide2::dot_wide;
use super::ToiProxy;
use super::super::normalize_and_length;

/// The features of the closest pair (Parry `SimplexCache`, 2D): `count` vertex pairs, their
/// indices in each proxy. `count = 2` with `index_a[0] == index_a[1]` is a vertex of A against
/// the edge `index_b` of B, and conversely.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct SimplexCache {
    pub count: u8,
    pub index_a: [u32; 2],
    pub index_b: [u32; 2],
}

/// The closest points of two proxies (Parry `ProxyDistanceOutput`), in the frame of proxy A.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ProxyDistanceOutput {
    pub point_a: Vec2,
    pub point_b: Vec2,
    /// Unit direction from `point_a` to `point_b`; zero when the cores overlap.
    pub normal: Vec2,
    /// Distance of the cores (less both radii, clamped at 0, with `use_radii`); zero when they
    /// overlap.
    pub distance: Fixed,
    /// Always 0: the kernel does not iterate (upstream: GJK's support calls).
    pub iterations: u32,
}

/// The index after `i` on a closed polygon of `n` points.
#[inline(always)]
fn next(i: u32, n: u32) -> u32 {
    if i + 1 == n {
        0
    } else {
        i + 1
    }
}

/// The number of edges of a proxy of `n` points: none for a point, one for a segment.
#[inline(always)]
fn edge_count(n: u32) -> u32 {
    if n < 3 {
        n - 1
    } else {
        n
    }
}

/// The closest point of the segment `[a, b]` to `p`, and whether it is inside the segment (not
/// an end point).
fn project_on_edge(a: Vec2, b: Vec2, p: Vec2) -> (Vec2, bool) {
    let e = b - a;
    let num = dot_wide(p.x - a.x, p.y - a.y, e.x, e.y);
    let den = norm2_sq_wide(e.x, e.y);
    if num <= 0 || den == 0 {
        return (a, false);
    }
    if num >= den {
        return (b, false);
    }
    let t = crate::ray::quotient::div_wide(num, den).unwrap();
    (a + e.mul_scalar(t), true)
}

/// Whether `p` is inside or on the convex polygon `points` (`n >= 3`, either orientation).
fn inside(points: Span<Vec2>, p: Vec2) -> bool {
    let n = points.len();
    let mut sign: i128 = 0;
    let mut i = 0;
    while i != n {
        let (a, b) = (*points[i], *points[next(i, n)]);
        let c = cross_wide(b.x - a.x, b.y - a.y, p.x - a.x, p.y - a.y);
        if c != 0 {
            if sign == 0 {
                sign = c;
            } else if (sign > 0) != (c > 0) {
                return false;
            }
        }
        i += 1;
    }
    true
}

/// Whether the segments `[a, b]` and `[c, d]` cross or touch (exact orientation tests).
fn segments_meet(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool {
    let o = |p: Vec2, q: Vec2, r: Vec2| -> i128 {
        cross_wide(q.x - p.x, q.y - p.y, r.x - p.x, r.y - p.y)
    };
    let (d1, d2) = (o(a, b, c), o(a, b, d));
    let (d3, d4) = (o(c, d, a), o(c, d, b));
    if ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)) {
        return true;
    }
    false
}

/// A candidate pair: `(squared distance, point on A, point on B, cache)`.
#[derive(Copy, Drop, Debug)]
pub struct Pair {
    pub sq: i128,
    pub point_a: Vec2,
    pub point_b: Vec2,
    pub cache: SimplexCache,
}

#[inline(always)]
fn better(best: Option<Pair>, candidate: Pair) -> Option<Pair> {
    match best {
        Some(b) => if candidate.sq < b.sq {
            Some(candidate)
        } else {
            best
        },
        None => Some(candidate),
    }
}

/// The orientation of a polygon's vertices: `1` counter-clockwise, `-1` clockwise, `0` for a
/// point, a segment or a degenerate polygon (every edge then counts as visible).
fn orientation(points: Span<Vec2>) -> i8 {
    if points.len() < 3 {
        return 0;
    }
    let (a, b, c) = (*points[0], *points[1], *points[2]);
    let s = cross_wide(b.x - a.x, b.y - a.y, c.x - a.x, c.y - a.y);
    if s > 0 {
        1
    } else if s < 0 {
        -1
    } else {
        0
    }
}

/// Whether `q` sees the edge `[a, b]` of a polygon of orientation `o` from outside or on its
/// line (exact); always for `o = 0` or when not pruning.
#[inline(always)]
fn visible(a: Vec2, b: Vec2, q: Vec2, o: i8, prune: bool) -> bool {
    if !prune || o == 0 {
        return true;
    }
    let c = cross_wide(b.x - a.x, b.y - a.y, q.x - a.x, q.y - a.y);
    if o > 0 {
        c <= 0
    } else {
        c >= 0
    }
}

/// The closest pair of the core point clouds `pa` and `pb` (same frame): every vertex of B
/// against the edges of A, then every vertex of A against the edges of B, then the vertex pairs
/// of two single points; the first minimum wins. With `prune`, a vertex is only projected on the
/// edges it sees from outside (the closest boundary point of an outside point is on one of them;
/// [`proxy_distance_exhaustive`] measures the full search). `None` when no
/// vertex sees any edge (the cores overlap).
pub fn closest_pair(pa: Span<Vec2>, pb: Span<Vec2>, prune: bool) -> Option<Pair> {
    let (na, nb) = (pa.len(), pb.len());
    let (oa, ob) = (orientation(pa), orientation(pb));
    let mut best: Option<Pair> = None;
    let mut j = 0;
    while j != nb {
        let q = *pb[j];
        let mut i = 0;
        while i != edge_count(na) {
            let i2 = next(i, na);
            if visible(*pa[i], *pa[i2], q, oa, prune) {
                let (p, interior) = project_on_edge(*pa[i], *pa[i2], q);
                let d = q - p;
                let cache = if interior {
                    SimplexCache { count: 2, index_a: [i, i2], index_b: [j, j] }
                } else {
                    let k = if p == *pa[i] {
                        i
                    } else {
                        i2
                    };
                    SimplexCache { count: 1, index_a: [k, 0], index_b: [j, 0] }
                };
                best =
                    better(
                        best, Pair { sq: norm2_sq_wide(d.x, d.y), point_a: p, point_b: q, cache },
                    );
            }
            i += 1;
        }
        j += 1;
    }
    let mut i = 0;
    while i != na {
        let p = *pa[i];
        let mut j = 0;
        while j != edge_count(nb) {
            let j2 = next(j, nb);
            if visible(*pb[j], *pb[j2], p, ob, prune) {
                let (q, interior) = project_on_edge(*pb[j], *pb[j2], p);
                let d = q - p;
                let cache = if interior {
                    SimplexCache { count: 2, index_a: [i, i], index_b: [j, j2] }
                } else {
                    let k = if q == *pb[j] {
                        j
                    } else {
                        j2
                    };
                    SimplexCache { count: 1, index_a: [i, 0], index_b: [k, 0] }
                };
                best =
                    better(
                        best, Pair { sq: norm2_sq_wide(d.x, d.y), point_a: p, point_b: q, cache },
                    );
            }
            j += 1;
        }
        i += 1;
    }
    if na == 1 && nb == 1 {
        let (p, q) = (*pa[0], *pb[0]);
        let d = q - p;
        return Some(
            Pair {
                sq: norm2_sq_wide(d.x, d.y),
                point_a: p,
                point_b: q,
                cache: SimplexCache { count: 1, index_a: [0, 0], index_b: [0, 0] },
            },
        );
    }
    best
}

/// Whether the cores overlap: an edge of one crossing an edge of the other, or the first vertex
/// of one inside the other (only a polygon can contain; a zero distance is caught by the caller).
fn cores_overlap(pa: Span<Vec2>, pb: Span<Vec2>) -> bool {
    let (na, nb) = (pa.len(), pb.len());
    if nb >= 3 && inside(pb, *pa[0]) {
        return true;
    }
    if na >= 3 && inside(pa, *pb[0]) {
        return true;
    }
    let mut i = 0;
    while i != edge_count(na) {
        let (a, b) = (*pa[i], *pa[next(i, na)]);
        let mut j = 0;
        while j != edge_count(nb) {
            if segments_meet(a, b, *pb[j], *pb[next(j, nb)]) {
                return true;
            }
            j += 1;
        }
        i += 1;
    }
    false
}

/// The distance and closest points of `proxy_a` and `proxy_b` placed at `pos12` (frame of A;
/// upstream `proxy_distance`). `cache` receives the features of the closest pair (upstream's
/// warm start; not read). With `use_radii`, the distance loses both radii (clamped at 0) and
/// the points move onto the dilated surfaces along the normal.
/// #### Panics
/// * The overflow panics of the transform and of the wide products.
pub fn proxy_distance(
    pos12: Pose2, proxy_a: ToiProxy, proxy_b: ToiProxy, use_radii: bool, ref cache: SimplexCache,
) -> ProxyDistanceOutput {
    distance_with(pos12, proxy_a, proxy_b, use_radii, ref cache, true)
}

/// [`proxy_distance`] projecting every vertex on every edge (no visibility pruning), for the
/// `gas_*` ranking and the agreement tests.
pub fn proxy_distance_exhaustive(
    pos12: Pose2, proxy_a: ToiProxy, proxy_b: ToiProxy, use_radii: bool, ref cache: SimplexCache,
) -> ProxyDistanceOutput {
    distance_with(pos12, proxy_a, proxy_b, use_radii, ref cache, false)
}

fn distance_with(
    pos12: Pose2,
    proxy_a: ToiProxy,
    proxy_b: ToiProxy,
    use_radii: bool,
    ref cache: SimplexCache,
    prune: bool,
) -> ProxyDistanceOutput {
    let pa = proxy_a.points;
    let mut moved = array![];
    for p in proxy_b.points {
        moved.append(pos12.transform_point(*p));
    }
    let pb = moved.span();
    let overlap = |
        pair: Pair,
    | ProxyDistanceOutput {
        point_a: pair.point_a,
        point_b: pair.point_b,
        normal: Vec2 { x: ZERO, y: ZERO },
        distance: ZERO,
        iterations: 0,
    };
    let Some(pair) = closest_pair(pa, pb, prune) else {
        cache = SimplexCache { count: 1, index_a: [0, 0], index_b: [0, 0] };
        return overlap(Pair { sq: 0, point_a: *pa[0], point_b: *pb[0], cache });
    };
    cache = pair.cache;
    if pair.sq == 0 || cores_overlap(pa, pb) {
        return overlap(pair);
    }
    let (normal, _) = normalize_and_length(pair.point_b - pair.point_a);
    let normal = normal.unwrap();
    let d = pair.point_b - pair.point_a;
    let mut output = ProxyDistanceOutput {
        point_a: pair.point_a,
        point_b: pair.point_b,
        normal,
        distance: norm2_wide(d.x, d.y).to_fixed(),
        iterations: 0,
    };
    if use_radii {
        let dist = output.distance - proxy_a.radius - proxy_b.radius;
        output.distance = if dist > ZERO {
            dist
        } else {
            ZERO
        };
        output.point_a = output.point_a + normal.mul_scalar(proxy_a.radius);
        output.point_b = output.point_b - normal.mul_scalar(proxy_b.radius);
    }
    output
}
