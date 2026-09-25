//! Bounded polygon SAT and PFM clipping, replacing upstream's GJK/EPA PFM query.
//! Face axes choose the first maximum (shape 1 wins cross-shape ties). For separated cores,
//! closest edge pairs supply the Euclidean normal, including rounded/speculative corners.
use fixed::wide::dot2;
use fixed::{Fixed, MAX, ONE, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::math_ext::norm2::norm2_sq_wide;
use rapier_math::math_ext::vec2::try_normalize2_and_length;
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::closest_points::closest_points_segment_segment;
use crate::contact::{ContactManifold, ContactManifoldTrait, TrackedContact};
use crate::manifold::ManifoldTrait;
use crate::polygonal_feature::{PolygonalFeature, PolygonalFeatureTrait};
use crate::shape::{ConvexPolygon, ConvexPolygonTrait, Cuboid, CuboidTrait, Segment};

const O: Vec2 = Vec2 { x: ZERO, y: ZERO };
const X: Vec2 = Vec2 { x: ONE, y: ZERO };
const Y: Vec2 = Vec2 { x: ZERO, y: ONE };
fn dot(a: Vec2, b: Vec2) -> Fixed {
    dot2(a.x, b.x, a.y, b.y)
}

// Internal polygonal cores may have two vertices, or repeated vertices for degenerate cuboids.
// They never escape through the public ConvexPolygon constructor/API.
/// Internal four-vertex cuboid core; native cuboid ids are supplied separately to clipping.
pub(crate) fn cuboid_core(c: Cuboid) -> ConvexPolygon {
    let h = c.half_extents;
    ConvexPolygon {
        vertices: [
            Vec2 { x: -h.x, y: -h.y }, Vec2 { x: h.x, y: -h.y }, h, Vec2 { x: -h.x, y: h.y }, O, O,
            O, O,
        ],
        normals: [-Y, X, Y, -X, O, O, O, O],
        count: 4,
    }
}

// Support-direction search: measured against transforming every vertex in alternatives.
fn sat_support(a: ConvexPolygon, b: ConvexPolygon, p: Pose2) -> (Fixed, Vec2) {
    let mut best = -MAX;
    let mut axis = O;
    let mut i = 0;
    while i != a.count {
        let n = a.normal(i);
        if n == O {
            i += 1;
            continue;
        }
        let q = p.transform_point(b.local_support_point(p.inverse_transform_vector(-n)));
        let s = dot(q - a.vertex(i), n);
        if s > best {
            best = s;
            axis = n;
        }
        i += 1;
    }
    (best, axis)
}

/// Transforms padded core vertices; normals remain local and are not queried on this value.
pub(crate) fn transformed(b: ConvexPolygon, p: Pose2) -> ConvexPolygon {
    let [a, c, d, e, f, g, h, i] = b.vertices;
    ConvexPolygon {
        vertices: [
            p.transform_point(a), p.transform_point(c), p.transform_point(d), p.transform_point(e),
            p.transform_point(f), p.transform_point(g), p.transform_point(h), p.transform_point(i),
        ],
        ..b,
    }
}


fn closest(a: ConvexPolygon, b: ConvexPolygon) -> (Vec2, Vec2) {
    let mut p1 = a.vertex(0);
    let mut p2 = b.vertex(0);
    let d = p2 - p1;
    let mut best = norm2_sq_wide(d.x, d.y);
    let mut i = 0;
    while i != a.count {
        let edge1 = Segment { a: a.vertex(i), b: a.vertex(a.next(i)) };
        let mut j = 0;
        while j != b.count {
            let (q1, q2) = closest_points_segment_segment(
                edge1, Segment { a: b.vertex(j), b: b.vertex(b.next(j)) },
            );
            let d = q2 - q1;
            let score = norm2_sq_wide(d.x, d.y);
            if score < best {
                best = score;
                p1 = q1;
                p2 = q2;
            }
            j += 1;
        }
        i += 1;
    }
    (p1, p2)
}

// A positive SAT gap needs a Euclidean distance check: face-axis prediction alone admits
// false positives at corners. Penetrating cores only need the two face-axis searches.
/// SAT axis plus optional separated-core witnesses. Returns None beyond prediction.
pub(crate) fn separating_axis(
    a: ConvexPolygon, b: ConvexPolygon, p: Pose2, prediction: Fixed,
) -> Option<(Vec2, Option<(Vec2, Vec2)>)> {
    let (s1, n1) = sat_support(a, b, p);
    if s1 > prediction {
        return None;
    }
    let (s2, n2) = sat_support(b, a, p.inverse());
    if s2 > prediction {
        return None;
    }
    let n = if s2 > s1 {
        p.transform_vector(-n2)
    } else {
        n1
    };
    if s1 > ZERO || s2 > ZERO {
        let (q1, q2) = closest(a, transformed(b, p));
        let d = q2 - q1;
        if let Some((x, y, len)) = try_normalize2_and_length(d.x, d.y, ZERO) {
            if len > prediction {
                return None;
            }
            return Some((Vec2 { x, y }, Some((q1, p.inverse_transform_point(q2)))));
        }
    }
    Some((n, None))
}

/// Clips features and inflates shape 2. Only actual closest-point witnesses permit a fallback.
pub(crate) fn finish(
    p: Pose2,
    n1: Vec2,
    f1: PolygonalFeature,
    f2: PolygonalFeature,
    witnesses: Option<(Vec2, Vec2)>,
    radius: Fixed,
    flipped: bool,
    ref m: ContactManifold,
) {
    let n2 = p.inverse_transform_vector(-n1);
    let old = m;
    m.clear();
    PolygonalFeatureTrait::contacts(p, p.inverse(), n1, n2, f1, f2, ref m, flipped);
    if m.num_points == 0 {
        if let Some((q1, q2)) = witnesses {
            let c = TrackedContact {
                local_p1: q1,
                local_p2: q2,
                dist: dot(p.transform_point(q2) - q1, n1),
                ..Default::default(),
            };
            m
                .points =
                    [
                        if flipped {
                            TrackedContact { local_p1: q2, local_p2: q1, ..c }
                        } else {
                            c
                        },
                        Default::default(),
                    ];
            m.num_points = 1;
        }
    }
    let [mut a, mut b] = m.points;
    let offset = n2.mul_scalar(radius);
    if flipped {
        a.local_p1 = a.local_p1 + offset;
        b.local_p1 = b.local_p1 + offset;
        m.local_n1 = n2;
        m.local_n2 = n1;
    } else {
        a.local_p2 = a.local_p2 + offset;
        b.local_p2 = b.local_p2 + offset;
        m.local_n1 = n1;
        m.local_n2 = n2;
    }
    a.dist = a.dist - radius;
    b.dist = b.dist - radius;
    m.points = [a, b];
    m.match_contacts(@old);
}

/// Polygon–polygon SAT and clipping. `pos12` places polygon 2 in polygon 1's frame.
/// Requires constructed polygons, unit rotation and representable Q32.32 intermediates
/// (coordinates bounded by 8192 suffice for clipping). Products floor, normalization rounds
/// nearest. Overflow and clipping-range violations panic. Keeps 0..2 points, including clipped
/// points beyond prediction; exact SAT ties choose the first face of the first polygon.
#[inline(never)]
pub fn contact_manifold_polygon_polygon(
    pos12: Pose2,
    polygon1: ConvexPolygon,
    polygon2: ConvexPolygon,
    prediction: Fixed,
    ref manifold: ContactManifold,
) {
    if manifold.try_update_contacts(pos12) {
        return;
    }
    polygon_polygon_fresh(pos12, polygon1, polygon2, prediction, ref manifold);
}

/// [`contact_manifold_polygon_polygon`] without its persistence check (see
/// `cuboid_cuboid::cuboid_cuboid_fresh`).
#[inline(never)]
pub(crate) fn polygon_polygon_fresh(
    pos12: Pose2,
    polygon1: ConvexPolygon,
    polygon2: ConvexPolygon,
    prediction: Fixed,
    ref manifold: ContactManifold,
) {
    match separating_axis(polygon1, polygon2, pos12, prediction) {
        Some((
            n, witnesses,
        )) => finish(
            pos12,
            n,
            polygon1.support_feature(n),
            polygon2.support_feature(pos12.inverse_transform_vector(-n)),
            witnesses,
            ZERO,
            false,
            ref manifold,
        ),
        None => manifold.clear(),
    }
}

/// Polygon–cuboid SAT and clipping, preserving native cuboid feature ids. `flipped` exchanges
/// stored sides; `pos12` always places the cuboid in the polygon frame. Bounds, rounding,
/// panics and retention are those of `contact_manifold_polygon_polygon`.
#[inline(never)]
pub fn contact_manifold_polygon_cuboid(
    pos12: Pose2,
    polygon1: ConvexPolygon,
    cuboid2: Cuboid,
    prediction: Fixed,
    flipped: bool,
    ref manifold: ContactManifold,
) {
    if manifold.try_update_contacts(if flipped {
        pos12.inverse()
    } else {
        pos12
    }) {
        return;
    }
    polygon_cuboid_fresh(pos12, polygon1, cuboid2, prediction, flipped, ref manifold);
}

/// [`contact_manifold_polygon_cuboid`] without its persistence check (see
/// `cuboid_cuboid::cuboid_cuboid_fresh`).
#[inline(never)]
pub(crate) fn polygon_cuboid_fresh(
    pos12: Pose2,
    polygon1: ConvexPolygon,
    cuboid2: Cuboid,
    prediction: Fixed,
    flipped: bool,
    ref manifold: ContactManifold,
) {
    match separating_axis(polygon1, cuboid_core(cuboid2), pos12, prediction) {
        Some((
            n, witnesses,
        )) => finish(
            pos12,
            n,
            polygon1.support_feature(n),
            cuboid2.support_feature(pos12.inverse_transform_vector(-n)),
            witnesses,
            ZERO,
            flipped,
            ref manifold,
        ),
        None => manifold.clear(),
    }
}

#[cfg(test)]
mod alternatives {
    use fixed::FixedTrait;
    use super::{ConvexPolygon, ConvexPolygonTrait, Fixed, MAX, O, Pose2, Vec2, dot, sat_support};
    pub fn sat_transformed(a: ConvexPolygon, b: ConvexPolygon) -> (Fixed, Vec2) {
        let mut best = -MAX;
        let mut axis = O;
        let mut i = 0;
        while i != a.count {
            let n = a.normal(i);
            if n != O {
                let origin = a.vertex(i);
                let mut sep = MAX;
                let mut j = 0;
                while j != b.count {
                    sep = sep.min(dot(b.vertex(j) - origin, n));
                    j += 1;
                }
                if sep > best {
                    best = sep;
                    axis = n;
                }
            }
            i += 1;
        }
        (best, axis)
    }

    pub fn support_sat(a: ConvexPolygon, b: ConvexPolygon, p: Pose2) -> (Fixed, Vec2) {
        sat_support(a, b, p)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, HALF, ONE, ZERO};
    use glam::Vec2;
    use rapier_math::consts::UNIT_TOL_SQ_RAW;
    use rapier_math::math_ext::norm2::is_unit2_raw;
    use rapier_math::pose2::{IDENTITY, Pose2};
    use rapier_testing::opaque;
    use crate::contact::{ContactManifold, ContactManifoldTrait};
    use crate::feature_id::FeatureIdTrait;
    use crate::shape::{ConvexPolygonTrait, Cuboid};
    use super::alternatives::sat_transformed;
    use super::{
        alternatives, contact_manifold_polygon_cuboid, contact_manifold_polygon_polygon,
        cuboid_core, transformed,
    };

    fn cube() -> Cuboid {
        Cuboid { half_extents: Vec2 { x: ONE, y: HALF } }
    }
    fn pose(x: Fixed) -> Pose2 {
        Pose2 { translation: Vec2 { x, y: ZERO }, ..IDENTITY }
    }
    #[test]
    fn test_regimes_and_persistence() {
        let a = cuboid_core(cube());
        for (x, count, dist) in array![
            (Fixed { raw: 3 * 4294967296 }, 0_u8, ZERO),
            (Fixed { raw: 2 * 4294967296 + 100 }, 2, Fixed { raw: 100 }),
            (Fixed { raw: 2 * 4294967296 }, 2, ZERO),
            (Fixed { raw: 2 * 4294967296 - 100 }, 2, Fixed { raw: -100 }), (ZERO, 2, -ONE),
        ]
            .span() {
            for is_cuboid in array![false, true].span() {
                let mut m: ContactManifold = Default::default();
                if *is_cuboid {
                    contact_manifold_polygon_cuboid(pose(*x), a, cube(), HALF, false, ref m);
                } else {
                    contact_manifold_polygon_polygon(pose(*x), a, a, HALF, ref m);
                }
                assert_eq!(m.num_points, *count);
                if *count != 0 {
                    assert_eq!(m.point(0).dist, *dist);
                    assert!(m.point(0).fid1.is_face() || m.point(0).fid1.is_vertex());
                    let [mut p, q] = m.points;
                    p.data.impulse = ONE;
                    m.points = [p, q];
                    let old = m;
                    if *is_cuboid {
                        contact_manifold_polygon_cuboid(pose(*x), a, cube(), HALF, false, ref m);
                    } else {
                        contact_manifold_polygon_polygon(pose(*x), a, a, HALF, ref m);
                    }
                    assert_eq!(m, old);
                }
            }
        }
    }
    #[test]
    fn test_corner_prediction_is_euclidean() {
        let a = cuboid_core(cube());
        let p = Pose2 {
            translation: Vec2 {
                x: Fixed { raw: 2 * 4294967296 + 42949673 },
                y: Fixed { raw: 4294967296 + 42949673 },
            },
            ..IDENTITY,
        };
        let mut m: ContactManifold = Default::default();
        contact_manifold_polygon_polygon(p, a, a, Fixed { raw: 42949673 }, ref m);
        assert_eq!(m.num_points, 0);
        contact_manifold_polygon_polygon(p, a, a, Fixed { raw: 85899346 }, ref m);
        assert!(m.num_points != 0);
        assert!(m.local_n1.x > HALF && m.local_n1.y > HALF);
    }
    #[test]
    #[fuzzer(runs: 32, seed: 20260924)]
    fn fuzz_sat_candidates(x: i16, y: i16) {
        let a = if x < 0 {
            octagon()
        } else {
            cuboid_core(cube())
        };
        let p = Pose2 {
            translation: Vec2 {
                x: Fixed { raw: x.into() * 65536 }, y: Fixed { raw: y.into() * 65536 },
            },
            ..IDENTITY,
        };
        assert_eq!(sat_transformed(a, transformed(a, p)), alternatives::support_sat(a, a, p));
    }
    fn octagon() -> super::ConvexPolygon {
        ConvexPolygonTrait::from_convex_polyline(
            array![
                Vec2 { x: -HALF, y: -ONE }, Vec2 { x: HALF, y: -ONE }, Vec2 { x: ONE, y: -HALF },
                Vec2 { x: ONE, y: HALF }, Vec2 { x: HALF, y: ONE }, Vec2 { x: -HALF, y: ONE },
                Vec2 { x: -ONE, y: HALF }, Vec2 { x: -ONE, y: -HALF },
            ]
                .span(),
        )
            .unwrap()
    }
    #[test]
    fn test_nonrectangular_and_thin_polygons() {
        let o = Vec2 { x: ZERO, y: ZERO };
        let triangle = ConvexPolygonTrait::from_convex_polyline(
            array![o, Vec2 { x: ONE, y: ZERO }, Vec2 { x: ZERO, y: ONE }].span(),
        )
            .unwrap();
        let tiny = Fixed { raw: 10 };
        let thin = cuboid_core(Cuboid { half_extents: Vec2 { x: ONE, y: tiny } });
        for a in array![triangle, octagon(), thin].span() {
            let mut m: ContactManifold = Default::default();
            contact_manifold_polygon_polygon(IDENTITY, *a, *a, ZERO, ref m);
            assert!(m.num_points > 0 && m.num_points <= 2);
            assert!(is_unit2_raw(m.local_n1.x, m.local_n1.y, UNIT_TOL_SQ_RAW));
            assert!(m.point(0).dist < ZERO || (m.num_points == 2 && m.point(1).dist < ZERO));
            for i in array![0_u8, 1].span() {
                if *i < m.num_points {
                    let point = m.point(*i);
                    let count: u32 = (*a.count).into();
                    assert!(point.fid1.code() < count * 2);
                    assert!(point.fid2.code() < count * 2);
                }
            }
        }
    }
    #[test]
    fn gas_sat_transformed_octagon() {
        let a = octagon();
        let _ = sat_transformed(a, transformed(a, opaque(pose(ONE))));
    }
    #[test]
    fn gas_sat_support_octagon() {
        let a = octagon();
        let _ = alternatives::support_sat(a, a, opaque(pose(ONE)));
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque(IDENTITY);
    }
    #[test]
    fn gas_polygon_polygon() {
        let mut m = Default::default();
        let a = cuboid_core(cube());
        contact_manifold_polygon_polygon(opaque(pose(ONE)), a, a, ZERO, ref m);
    }
    #[test]
    fn gas_polygon_cuboid() {
        let mut m = Default::default();
        contact_manifold_polygon_cuboid(
            opaque(pose(ONE)), cuboid_core(cube()), cube(), ZERO, false, ref m,
        );
    }
    #[test]
    fn gas_polygon_polygon_separated() {
        let mut m = Default::default();
        let a = cuboid_core(cube());
        contact_manifold_polygon_polygon(
            opaque(pose(Fixed { raw: 3 * 4294967296 })), a, a, ZERO, ref m,
        );
    }
    #[test]
    fn gas_polygon_polygon_prediction() {
        let mut m = Default::default();
        let a = cuboid_core(cube());
        contact_manifold_polygon_polygon(
            opaque(pose(Fixed { raw: 2 * 4294967296 + 100 })), a, a, HALF, ref m,
        );
    }
    #[test]
    fn gas_sat_transformed() {
        let a = cuboid_core(cube());
        let _ = sat_transformed(a, transformed(a, opaque(pose(ONE))));
    }
    #[test]
    fn gas_sat_support() {
        let a = cuboid_core(cube());
        let _ = alternatives::support_sat(a, a, opaque(pose(ONE)));
    }
}
