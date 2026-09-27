//! SH2b point projections, ray casts and shape-pair queries (`intersection_test`, `distance`,
//! `contact`, `closest_points`, `cast_shapes`) of the compounds against Parry f64 0.30.2.
//!
//! Bands (raw Q32.32 units), as the SH1 and SH2a queries: the point and ray answers within `GJK`
//! = 4096 (`~1e-6`; the parts include a round cuboid, which upstream projects and casts on with
//! GJK), projected points within `GJK_POINT` = `2^19` (`mixed/p3`: 2.5e-4 on a rounded corner, the
//! SH1 band), `BAND` for the pair queries, `CAST_BAND` for the cast witnesses and normals.
//!
//! Exceptions: `mixed/inside` casts from inside the round cuboid: the hollow ray's exit is the
//! exact `t = 0.97859` here (the flat side at `x = 0.5` in the part's frame), upstream's backward
//! GJK stops at `0.99833`, 0.02 past the boundary; its normal and hit agree. Closest points of
//! parallel faces (`ell_cuboid/hover`) may slide along the faces (`parallel`).
use fixed::Fixed;
use glam::{Vec2, Vec2Trait};
use rapier_geometry2d::feature_id::FeatureIdTrait;
use rapier_geometry2d::point::PointQuery;
use rapier_geometry2d::query::{
    ClosestPoints, ShapeCastOptionsTrait, cast_shapes, closest_points, contact, distance,
    intersection_test,
};
use rapier_geometry2d::ray::{Ray, cast_ray, cast_ray_and_get_normal};
use rapier_golden::compare::{abs_diff, vec2_within};
use rapier_golden::generated::{compound_pairs, compound_points, compound_rays, compound_shapes};
use rapier_golden::types::{PointFeatureRaw, RayAnswerRaw, Vec2Raw};
use super::composite_contacts_golden::pose;
use super::compound_contacts_golden::compound_shape;
use super::sh1_contacts_golden::shape;

const GJK: u64 = 4096;
const GJK_POINT: u64 = 0x80000;
const BAND: u64 = 64;
const EXIT: felt252 = 'mixed/inside';
const CAST_BAND: u64 = 4096;

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
fn test_compound_point_queries_golden() {
    let mut count = 0;
    for c in compound_points::cases() {
        let s = compound_shape(compound_shapes::compound(*c.composite));
        let pt = v(*c.point);
        let p = s.project_local_point(pt, false);
        assert_eq!(p.is_inside, *c.projection.is_inside, "{} inside", *c.id);
        assert!(
            vec2_within(raw(p.point), *c.projection.point, GJK_POINT), "{} point {:?}", *c.id, p,
        );
        let p = s.project_local_point(pt, true);
        assert_eq!(p.is_inside, *c.projection_solid.is_inside, "{} solid inside", *c.id);
        assert!(vec2_within(raw(p.point), *c.projection_solid.point, GJK_POINT), "{} solid", *c.id);
        let (p, f) = s.project_local_point_and_get_feature(pt);
        assert_eq!(p.is_inside, *c.feature_projection.is_inside, "{} f inside", *c.id);
        assert!(vec2_within(raw(p.point), *c.feature_projection.point, GJK_POINT), "{} f", *c.id);
        assert!(feature_eq(f, *c.feature), "{} feature {:?}", *c.id, f);
        let d = s.distance_to_local_point(pt, false);
        assert!(abs_diff(d.raw, *c.distance) <= GJK, "{} distance {:?}", *c.id, d);
        assert_eq!(s.contains_local_point(pt), *c.contains, "{} contains", *c.id);
        count += 1;
    }
    assert_eq!(count, 18);
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
    let exit = id == EXIT && !solid;
    let toi = cast_ray(s, p, ray, max, solid);
    assert_eq!(toi.is_some(), e.has_toi, "{} has_toi {}", id, solid);
    if let Some(t) = toi {
        assert!(exit || abs_diff(t.raw, e.toi) <= GJK, "{} toi {:?} {}", id, t, e.toi);
    }
    let hit = cast_ray_and_get_normal(s, p, ray, max, solid);
    assert_eq!(hit.is_some(), e.hit.hit, "{} hit {}", id, solid);
    if let Some(h) = hit {
        assert!(
            exit || abs_diff(h.time_of_impact.raw, e.hit.time_of_impact) <= GJK, "{} hit toi", id,
        );
        assert!(vec2_within(raw(h.normal), e.hit.normal, GJK), "{} normal {:?}", id, h);
        assert!(feature_eq(h.feature, e.hit.feature), "{} feature {:?}", id, h);
    }
}

#[test]
fn test_compound_ray_casts_golden() {
    let mut count = 0;
    for c in compound_rays::cases() {
        let s = compound_shape(compound_shapes::compound(*c.composite));
        let ray = Ray { origin: v(*c.origin), dir: v(*c.dir) };
        let max = Fixed { raw: *c.max_toi };
        check_ray(*c.id, s, pose(*c.pose), ray, max, true, *c.solid);
        check_ray(*c.id, s, pose(*c.pose), ray, max, false, *c.hollow);
        count += 1;
    }
    assert_eq!(count, 15);
}

/// `a` and `e` differ by a vector orthogonal to the unit `n` (within `CAST_BAND`).
fn tangential(a: Vec2, e: Vec2Raw, n: Vec2) -> bool {
    let d = a - v(e);
    abs_diff((d.x * n.x + d.y * n.y).raw, 0) <= CAST_BAND
}

fn points_match(p1: Vec2, p2: Vec2, e1: Vec2Raw, e2: Vec2Raw) -> bool {
    vec2_within(raw(p1), e1, BAND) && vec2_within(raw(p2), e2, BAND)
}

/// `(p1, p2)` and `(e1, e2)` are two witness pairs of the same parallel faces: the same gap, both
/// points shifted along the faces (orthogonally to the gap).
fn parallel(p1: Vec2, p2: Vec2, e1: Vec2Raw, e2: Vec2Raw) -> bool {
    let gap = p2 - p1;
    let (n, len) = (gap.normalize(), gap.length());
    let e_len = (v(e2) - v(e1)).length();
    abs_diff(len.raw, e_len.raw) <= BAND && tangential(p1, e1, n) && tangential(p2, e2, n)
}

#[test]
fn test_compound_pair_queries_golden() {
    let mut count = 0;
    for c in compound_pairs::cases() {
        let id = *c.id;
        let comp = compound_shape(compound_shapes::compound(*c.composite));
        let other = shape(*c.other);
        let (s1, s2) = if *c.composite_first {
            (comp, other)
        } else {
            (other, comp)
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
                    let common = vec2_within(raw(a), raw(b), BAND) && vec2_within(e.p1, e.p2, BAND);
                    assert!(
                        points_match(a, b, e.p1, e.p2) || common || parallel(a, b, e.p1, e.p2),
                        "{} closest {:?} {:?}",
                        id,
                        a,
                        b,
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
    assert_eq!(count, 36);
}
