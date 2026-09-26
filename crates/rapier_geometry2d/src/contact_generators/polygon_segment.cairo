//! Polygon–segment and polygon–capsule SAT, followed by polygonal-feature clipping.
//! Rounded corners use the closest core edge pair; radius is applied after clipping.
use fixed::{Fixed, ZERO};
use glam::Vec2;
use rapier_math::math_ext::vec2::try_normalize2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::contact::{ContactManifold, ContactManifoldTrait};
use crate::feature_id::FeatureIdTrait;
use crate::manifold::ManifoldTrait;
use crate::polygonal_feature::PolygonalFeature;
use crate::shape::{Capsule, ConvexPolygon, ConvexPolygonTrait, Segment};
use super::polygon_polygon::{finish, separating_axis};

fn core(s: Segment) -> ConvexPolygon {
    let d = s.b - s.a;
    let o = Vec2 { x: ZERO, y: ZERO };
    let n = match try_normalize2(d.y, -d.x) {
        Some((x, y)) => Vec2 { x, y },
        None => o,
    };
    ConvexPolygon {
        vertices: [s.a, s.b, o, o, o, o, o, o], normals: [n, -n, o, o, o, o, o, o], count: 2,
    }
}
fn generate(
    p: Pose2,
    polygon: ConvexPolygon,
    s: Segment,
    radius: Fixed,
    prediction: Fixed,
    flipped: bool,
    ref m: ContactManifold,
) {
    if m.try_update_contacts(if flipped {
        p.inverse()
    } else {
        p
    }) {
        return;
    }
    generate_body(p, polygon, s, radius, prediction, flipped, ref m);
}

/// `generate` without its persistence check (see `cuboid_cuboid::cuboid_cuboid_fresh`).
pub(crate) fn generate_fresh(
    p: Pose2,
    polygon: ConvexPolygon,
    s: Segment,
    radius: Fixed,
    prediction: Fixed,
    flipped: bool,
    ref m: ContactManifold,
) {
    generate_body(p, polygon, s, radius, prediction, flipped, ref m);
}

/// SAT and clipping: the body of both entries (inlined in each).
#[inline(always)]
fn generate_body(
    p: Pose2,
    polygon: ConvexPolygon,
    s: Segment,
    radius: Fixed,
    prediction: Fixed,
    flipped: bool,
    ref m: ContactManifold,
) {
    match separating_axis(polygon, core(s), p, prediction + radius) {
        None => m.clear(),
        Some((
            n, witnesses,
        )) => {
            let feature = PolygonalFeature {
                vertices: [s.a, s.b],
                vids: [FeatureIdTrait::vertex(0), FeatureIdTrait::vertex(2)],
                fid: FeatureIdTrait::face(1),
                num_vertices: 2,
            };
            finish(p, n, polygon.support_feature(n), feature, witnesses, radius, flipped, ref m);
        },
    }
}

/// Polygon–segment contact manifold. `pos12` places the segment in the polygon frame;
/// `flipped` exchanges stored sides. Zero-length segments are valid. Uses persistence then
/// SAT and clipping, keeping 0..2 points (possibly beyond prediction). Unit poses and constructed
/// polygons required. Q32.32 products floor, normalization rounds nearest. Panics on overflow
/// or clipping projection-range violation; coordinates bounded by 8192 suffice for clipping.
#[inline(never)]
pub fn contact_manifold_polygon_segment(
    pos12: Pose2,
    polygon1: ConvexPolygon,
    segment2: Segment,
    prediction: Fixed,
    flipped: bool,
    ref manifold: ContactManifold,
) {
    generate(pos12, polygon1, segment2, ZERO, prediction, flipped, ref manifold);
}

/// Polygon–capsule contact manifold. Nonnegative capsule radius required. Arguments, rounding,
/// bounds and panics as `contact_manifold_polygon_segment`; radius offsets the capsule anchors
/// toward the polygon and is subtracted from the clipped core distances.
#[inline(never)]
pub fn contact_manifold_polygon_capsule(
    pos12: Pose2,
    polygon1: ConvexPolygon,
    capsule2: Capsule,
    prediction: Fixed,
    flipped: bool,
    ref manifold: ContactManifold,
) {
    generate(pos12, polygon1, capsule2.segment, capsule2.radius, prediction, flipped, ref manifold);
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, HALF, ONE, ZERO};
    use glam::Vec2;
    use rapier_math::consts::UNIT_TOL_SQ_RAW;
    use rapier_math::math_ext::norm2::is_unit2_raw;
    use rapier_math::pose2::{IDENTITY, Pose2, Pose2Trait};
    use rapier_testing::opaque;
    use crate::contact::{ContactManifold, ContactManifoldTrait};
    use crate::shape::{Capsule, Cuboid, Segment};
    use super::super::polygon_polygon::cuboid_core;
    use super::{contact_manifold_polygon_capsule, contact_manifold_polygon_segment};
    fn polygon() -> crate::shape::ConvexPolygon {
        cuboid_core(Cuboid { half_extents: Vec2 { x: ONE, y: ONE } })
    }
    fn segment() -> Segment {
        Segment { a: Vec2 { x: ZERO, y: -HALF }, b: Vec2 { x: ZERO, y: HALF } }
    }
    fn pose(x: Fixed) -> Pose2 {
        Pose2 { translation: Vec2 { x, y: ZERO }, ..IDENTITY }
    }
    #[test]
    fn test_regimes_flips_and_degenerate() {
        for s in array![
            segment(), Segment { a: Vec2 { x: ZERO, y: ZERO }, b: Vec2 { x: ZERO, y: ZERO } },
        ]
            .span() {
            for radius in array![ZERO, HALF].span() {
                for (gap, count) in array![
                    (ONE, 0_u8), (Fixed { raw: 100 }, 2), (ZERO, 2), (-HALF, 2), (-ONE, 2),
                ]
                    .span() {
                    let p = pose(ONE + *radius + *gap);
                    let mut m: ContactManifold = Default::default();
                    let mut rev: ContactManifold = Default::default();
                    if *radius == ZERO {
                        contact_manifold_polygon_segment(p, polygon(), *s, HALF, false, ref m);
                        contact_manifold_polygon_segment(p, polygon(), *s, HALF, true, ref rev);
                    } else {
                        let cap = Capsule { segment: *s, radius: *radius };
                        contact_manifold_polygon_capsule(p, polygon(), cap, HALF, false, ref m);
                        contact_manifold_polygon_capsule(p, polygon(), cap, HALF, true, ref rev);
                    }
                    assert_eq!(m.num_points, *count);
                    assert_eq!(rev.num_points, *count);
                    if *count != 0 {
                        assert_eq!(m.point(0).dist, *gap);
                        assert_eq!(rev.point(0).local_p1, m.point(0).local_p2);
                        assert_eq!(rev.point(0).fid1, m.point(0).fid2);
                        assert_eq!(rev.local_n1, m.local_n2);
                        assert!(is_unit2_raw(m.local_n1.x, m.local_n1.y, UNIT_TOL_SQ_RAW));
                        assert_eq!(m.local_n2, p.inverse_transform_vector(-m.local_n1));
                    }
                }
            }
        }
    }
    #[test]
    fn test_round_corner_and_warmstart_matching() {
        let p = Pose2 {
            translation: Vec2 {
                x: Fixed { raw: 5 * 4294967296 / 4 }, y: Fixed { raw: 7 * 4294967296 / 4 },
            },
            ..IDENTITY,
        };
        let mut m: ContactManifold = Default::default();
        let cap = Capsule { segment: segment(), radius: HALF };
        contact_manifold_polygon_capsule(p, polygon(), cap, ZERO, false, ref m);
        assert!(m.num_points != 0);
        assert!(m.local_n1.x > HALF && m.local_n1.y > HALF);
        let [mut a, mut b] = m.points;
        a.data.impulse = ONE;
        b.data.impulse = ONE;
        m.points = [a, b];
        contact_manifold_polygon_capsule(p, polygon(), cap, ZERO, false, ref m);
        assert_eq!(m.point(0).data.impulse, ONE);
        // Force regeneration by changing the cached normal, while keeping feature pairs.
        m.local_n2 = -m.local_n2;
        contact_manifold_polygon_capsule(p, polygon(), cap, ZERO, false, ref m);
        assert_eq!(m.point(0).data.impulse, ONE);
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque(IDENTITY);
    }
    #[test]
    fn gas_polygon_segment() {
        let mut m = Default::default();
        contact_manifold_polygon_segment(
            opaque(pose(HALF)), polygon(), segment(), ZERO, false, ref m,
        );
    }
    #[test]
    fn gas_polygon_capsule() {
        let mut m = Default::default();
        contact_manifold_polygon_capsule(
            opaque(pose(HALF)),
            polygon(),
            Capsule { segment: segment(), radius: HALF },
            ZERO,
            false,
            ref m,
        );
    }
    #[test]
    fn gas_polygon_capsule_corner() {
        let mut m = Default::default();
        let p = Pose2 {
            translation: Vec2 {
                x: Fixed { raw: 5 * 4294967296 / 4 }, y: Fixed { raw: 7 * 4294967296 / 4 },
            },
            ..IDENTITY,
        };
        contact_manifold_polygon_capsule(
            opaque(p), polygon(), Capsule { segment: segment(), radius: HALF }, ZERO, false, ref m,
        );
    }
}
