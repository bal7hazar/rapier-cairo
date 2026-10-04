//! SH2a point projections, ray casts and shape-pair queries (`intersection_test`, `distance`,
//! `contact`, `closest_points`, `cast_shapes`) of the polyline and the heightfield against Parry
//! f64 0.31.1, with upstream's support matrix (`None` where upstream answers `Unsupported`).
//!
//! Features: a point projection and a ray hit report the hit segment's own feature (`Face(0 / 1)`,
//! `Vertex(0 / 1)`, the segment in `subshape`), as parry 0.31.1 (CE; ADR 0001 entry 48 closed).
//!
//! Bands (raw Q32.32 units): `EXACT` for the point and ray queries (segment kernels on exact
//! decisions); `BAND` for the pair queries (the convex part kernels against upstream's GJK / EPA
//! on the segment parts). A contact or closest-point pair must match upstream's points, or be a
//! witness pair at upstream's distance (two parts at the same distance). A cast's witnesses and
//! normal are within `CAST_BAND` = 4096 (`~1e-6`, the GJK band of the SH1 queries: upstream's GJK
//! stops within its tolerance, largest gaps 3798 raw on a witness, `flat_ball/hover`, and 281 raw
//! on the normal of a ball hitting a vertex, `hills_ball/hover`). Two parallel faces touch along a
//! segment: there the witnesses and contact points may differ along the faces (`tangential`,
//! `flat_cuboid/hover`), and overlapping collinear segments answer any common point as their
//! closest points (`bumps_segment/deep`).
//!
//! Ties: a point projecting on a vertex shared by two segments reports the segment upstream's BVH
//! visits first, the port the lowest index (`TIE`): the same point, as the end vertex of segment 0
//! here (`Vertex(1)`) and the start vertex of segment 1 upstream (`Vertex(0)`), so the feature must
//! be a vertex.
use fixed::Fixed;
use glam_core::Vec2;
use rapier_geometry2d::feature_id::FeatureIdTrait;
use rapier_geometry2d::point::PointQuery;
use rapier_geometry2d::query::{
    ClosestPoints, ShapeCastOptionsTrait, cast_shapes, closest_points, contact, distance,
    intersection_test,
};
use rapier_geometry2d::ray::{Ray, cast_ray, cast_ray_and_get_normal};
use rapier_golden::compare::{abs_diff, vec2_within};
use rapier_golden::generated::{composite_pairs, composite_points, composite_rays, composite_shapes};
use rapier_golden::types::{PointFeatureRaw, RayAnswerRaw, Vec2Raw};
use super::composite_contacts_golden::{composite_shape, pose};
use super::sh1_contacts_golden::shape;

const EXACT: u64 = 4;
const BAND: u64 = 64;
const CAST_BAND: u64 = 4096;
const TIE: felt252 = 'vee/p5';

fn v(p: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: p.x }, y: Fixed { raw: p.y } }
}
fn raw(p: Vec2) -> Vec2Raw {
    Vec2Raw { x: p.x.raw, y: p.y.raw }
}
fn feature_eq(f: rapier_geometry2d::feature_id::FeatureId, e: PointFeatureRaw) -> bool {
    match e {
        PointFeatureRaw::Face(i) => f.is_face() && f.code() == i,
        PointFeatureRaw::Vertex(i) => f.is_vertex() && f.code() == i,
        PointFeatureRaw::Unknown => f.is_unknown(),
    }
}

#[test]
fn test_composite_point_queries_golden() {
    let mut count = 0;
    for c in composite_points::cases() {
        let s = composite_shape(composite_shapes::composite(*c.composite));
        let pt = v(*c.point);
        let p = s.project_local_point(pt, false);
        assert_eq!(p.is_inside, *c.projection.is_inside, "{} inside", *c.id);
        assert!(vec2_within(raw(p.point), *c.projection.point, EXACT), "{} point {:?}", *c.id, p);
        let p = s.project_local_point(pt, true);
        assert_eq!(p.is_inside, *c.projection_solid.is_inside, "{} solid inside", *c.id);
        assert!(vec2_within(raw(p.point), *c.projection_solid.point, EXACT), "{} solid", *c.id);
        let (p, f) = s.project_local_point_and_get_feature(pt);
        assert_eq!(p.is_inside, *c.feature_projection.is_inside, "{} f inside", *c.id);
        assert!(vec2_within(raw(p.point), *c.feature_projection.point, EXACT), "{} f", *c.id);
        if *c.id == TIE {
            assert!(f.is_vertex(), "{} tie", *c.id);
        } else {
            assert!(feature_eq(f, *c.feature), "{} feature {:?}", *c.id, f);
        }
        let d = s.distance_to_local_point(pt, false);
        assert!(abs_diff(d.raw, *c.distance) <= EXACT, "{} distance {:?}", *c.id, d);
        assert_eq!(s.contains_local_point(pt), *c.contains, "{} contains", *c.id);
        count += 1;
    }
    assert_eq!(count, 42);
}

fn check_ray(
    id: felt252,
    s: rapier_geometry2d::shape::Shape,
    p: rapier_math::pose2::Pose2,
    ray: Ray,
    max: Fixed,
    solid: bool,
    e: RayAnswerRaw,
) {
    let toi = cast_ray(s, p, ray, max, solid);
    assert_eq!(toi.is_some(), e.has_toi, "{} has_toi {}", id, solid);
    if let Some(t) = toi {
        assert!(abs_diff(t.raw, e.toi) <= EXACT, "{} toi {:?} {}", id, t, e.toi);
    }
    let hit = cast_ray_and_get_normal(s, p, ray, max, solid);
    assert_eq!(hit.is_some(), e.hit.hit, "{} hit {}", id, solid);
    if let Some(h) = hit {
        assert!(abs_diff(h.time_of_impact.raw, e.hit.time_of_impact) <= EXACT, "{} hit toi", id);
        assert!(vec2_within(raw(h.normal), e.hit.normal, EXACT), "{} normal {:?}", id, h);
        assert!(feature_eq(h.feature, e.hit.feature), "{} feature {:?}", id, h);
    }
}

#[test]
fn test_composite_ray_casts_golden() {
    let mut count = 0;
    for c in composite_rays::cases() {
        let s = composite_shape(composite_shapes::composite(*c.composite));
        let ray = Ray { origin: v(*c.origin), dir: v(*c.dir) };
        let max = Fixed { raw: *c.max_toi };
        check_ray(*c.id, s, pose(*c.pose), ray, max, true, *c.solid);
        check_ray(*c.id, s, pose(*c.pose), ray, max, false, *c.hollow);
        count += 1;
    }
    assert_eq!(count, 35);
}

/// `a` and `e` differ by a vector orthogonal to the unit `n` (within `CAST_BAND`).
fn tangential(a: Vec2, e: Vec2Raw, n: Vec2) -> bool {
    let d = a - v(e);
    abs_diff((d.x * n.x + d.y * n.y).raw, 0) <= CAST_BAND
}

/// `(p1, p2)` matches `(e1, e2)`, or is a witness pair `dist` apart (another part at the same
/// distance).
fn points_match(p1: Vec2, p2: Vec2, e1: Vec2Raw, e2: Vec2Raw) -> bool {
    vec2_within(raw(p1), e1, BAND) && vec2_within(raw(p2), e2, BAND)
}

#[test]
fn test_composite_pair_queries_golden() {
    let mut count = 0;
    for c in composite_pairs::cases() {
        let id = *c.id;
        let composite = composite_shape(composite_shapes::composite(*c.composite));
        let other = shape(*c.other);
        let (s1, s2) = if *c.composite_first {
            (composite, other)
        } else {
            (other, composite)
        };
        let (p1, p2) = (pose(*c.pos1), pose(*c.pos2));
        let got = intersection_test(p1, s1, p2, s2);
        assert_eq!(got.is_some(), *c.intersects_supported, "{} intersects?", id);
        if let Some(b) = got {
            assert_eq!(b, *c.intersects, "{} intersects", id);
        }
        let got = distance(p1, s1, p2, s2);
        assert_eq!(got.is_some(), *c.distance_supported, "{} distance?", id);
        if let Some(d) = got {
            assert!(abs_diff(d.raw, *c.distance) <= BAND, "{} distance {:?}", id, d);
        }
        let got = contact(p1, s1, p2, s2, Fixed { raw: *c.prediction });
        assert_eq!(got.is_some(), *c.contact_supported, "{} contact?", id);
        if let Some(got) = got {
            let e = *c.contact;
            assert_eq!(got.is_some(), e.some, "{} contact some", id);
            if let Some(k) = got {
                assert!(abs_diff(k.dist.raw, e.dist) <= BAND, "{} dist {:?}", id, k);
                assert!(
                    points_match(k.point1, k.point2, e.point1, e.point2)
                        || (tangential(k.point1, e.point1, k.normal1)
                            && tangential(k.point2, e.point2, k.normal1)),
                    "{} points {:?}",
                    id,
                    k,
                );
                assert!(vec2_within(raw(k.normal1), e.normal1, BAND), "{} normal1", id);
                assert!(vec2_within(raw(k.normal2), e.normal2, BAND), "{} normal2", id);
            }
        }
        let got = closest_points(p1, s1, p2, s2, Fixed { raw: *c.margin });
        assert_eq!(got.is_some(), *c.closest_supported, "{} closest?", id);
        if let Some(got) = got {
            let e = *c.closest;
            match got {
                ClosestPoints::Intersecting => assert_eq!(e.kind, 0, "{} kind", id),
                ClosestPoints::WithinMargin((
                    a, b,
                )) => {
                    assert_eq!(e.kind, 1, "{} kind", id);
                    // Collinear overlapping segments: any common point.
                    let common = vec2_within(raw(a), raw(b), BAND) && vec2_within(e.p1, e.p2, BAND);
                    assert!(
                        points_match(a, b, e.p1, e.p2) || common, "{} closest {:?} {:?}", id, a, b,
                    );
                },
                ClosestPoints::Disjoint => assert_eq!(e.kind, 2, "{} kind", id),
            }
        }
        let vel = v(*c.vel);
        let zero = Vec2 { x: Fixed { raw: 0 }, y: Fixed { raw: 0 } };
        let (v1, v2) = if *c.composite_first {
            (zero, vel)
        } else {
            (vel, zero)
        };
        let options = ShapeCastOptionsTrait::with_max_time_of_impact(
            Fixed { raw: 10 * 0x100000000 },
        );
        let got = cast_shapes(p1, v1, s1, p2, v2, s2, options);
        assert_eq!(got.is_some(), *c.cast_supported, "{} cast?", id);
        if let Some(got) = got {
            let e = *c.cast;
            assert_eq!(got.is_some(), e.some, "{} cast some", id);
            if let Some(h) = got {
                assert!(abs_diff(h.time_of_impact.raw, e.toi) <= BAND, "{} toi {:?}", id, h);
                assert!(
                    (vec2_within(raw(h.witness1), e.witness1, CAST_BAND)
                        && vec2_within(raw(h.witness2), e.witness2, CAST_BAND))
                        || (tangential(h.witness1, e.witness1, h.normal1)
                            && tangential(h.witness2, e.witness2, h.normal2)),
                    "{} witnesses {:?}",
                    id,
                    h,
                );
                assert!(
                    vec2_within(raw(h.normal1), e.normal1, CAST_BAND), "{} cast n1 {:?}", id, h,
                );
            }
        }
        count += 1;
    }
    assert_eq!(count, 48);
}
