//! Every `rapier_golden::ray_casts` case against `rapier_geometry2d::ray`.
//!
//! Each case casts a world-space ray on one shape placed at a pose, `solid` and hollow, through
//! both entry points (`cast_ray`, `cast_ray_and_get_normal`). Tolerances are the ones
//! `tools/golden/README.md` documents for this family: hit / miss and features exact; time of
//! impact and normal within [`TOI_TOLERANCE`] / [`NORMAL_TOLERANCE`] ulp.
//!
//! Parry 0.31.1 casts on a capsule analytically, as this port does (times of impact equal, normals
//! within 3 raw: ADR 0001 entries 4 and 5 closed). Three of its capsule answers moved away from the
//! port, which keeps parry 0.30.2's: the feature (`Face(0)`, the port `Unknown`), the normal of a
//! solid cast from inside (zero, the port `-dir / |dir|`) and a zero `dir` from inside (a hit at
//! `t = 0`, the port a miss). Those fields are compared with the frozen 0.30.2 copy
//! (`frozen_parry030::ray_cast_answer`, ADR 0001 entry 47).
//!
//! Parry 0.30.2 answered one case wrongly, checked against the frozen copy: a hollow capsule cast
//! from inside with a non-unit `dir` (`capsule/inside`) mixed unit-length and `dir` units in its
//! shift trick; the test verifies that 0.30.2's value is exactly that mix applied to this port's
//! exit time.

use fixed::wide::norm2;
use fixed::{Fixed, FixedTrait};
use glam_core::vec2::{Vec2, Vec2Trait};
use rapier_geometry2d::feature_id::{FEATURE_UNKNOWN, FeatureId, FeatureIdTrait};
use rapier_geometry2d::ray::{Ray, RayIntersection, cast_ray, cast_ray_and_get_normal};
use rapier_geometry2d::shape::{
    BallTrait, CapsuleTrait, CuboidTrait, HalfSpaceTrait, SegmentTrait, Shape,
};
use rapier_golden::compare::{vec2_within, within};
use rapier_golden::generated::{frozen_parry030, ray_casts};
use rapier_golden::types::{PointFeatureRaw, PoseRaw, RayAnswerRaw, RayCase, ShapeRaw, Vec2Raw};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;

const TOI_TOLERANCE: u64 = 4;
const NORMAL_TOLERANCE: u64 = 8;
/// `0.001`, the `eps` of upstream's hollow support-map cast.
const SHIFT_EPS: Fixed = Fixed { raw: 4294967 };

fn fx(raw: i64) -> Fixed {
    Fixed { raw }
}

fn vector(v: Vec2Raw) -> Vec2 {
    Vec2 { x: fx(v.x), y: fx(v.y) }
}

fn raw(v: Vec2) -> Vec2Raw {
    Vec2Raw { x: v.x.raw, y: v.y.raw }
}

fn pose(p: PoseRaw) -> Pose2 {
    Pose2Trait::new(vector(p.translation), Rot2 { re: fx(p.rotation.re), im: fx(p.rotation.im) })
}

fn shape(s: ShapeRaw) -> Shape {
    match s {
        ShapeRaw::Ball(r) => Shape::Ball(BallTrait::new(fx(r))),
        ShapeRaw::Cuboid(he) => Shape::Cuboid(CuboidTrait::new(vector(he))),
        ShapeRaw::Capsule(c) => Shape::Capsule(
            CapsuleTrait::new(vector(c.a), vector(c.b), fx(c.radius)),
        ),
        ShapeRaw::HalfSpace(n) => Shape::HalfSpace(HalfSpaceTrait::new(vector(n))),
        ShapeRaw::Segment(s) => Shape::Segment(SegmentTrait::new(vector(s.a), vector(s.b))),
    }
}

fn feature(f: PointFeatureRaw) -> FeatureId {
    match f {
        PointFeatureRaw::Unknown => FEATURE_UNKNOWN,
        PointFeatureRaw::Vertex(i) => FeatureIdTrait::vertex(i),
        PointFeatureRaw::Face(i) => FeatureIdTrait::face(i),
    }
}

/// Checks one `solid` value of `case`.
fn check(case: @RayCase, solid: bool, expected: RayAnswerRaw) {
    let s = shape(*case.shape);
    let p = pose(*case.pose);
    let ray = Ray { origin: vector(*case.origin), dir: vector(*case.dir) };
    let max = fx(*case.max_toi);
    match cast_ray(s, p, ray, max, solid) {
        Some(t) => {
            assert!(expected.has_toi, "{}: unexpected toi (solid {})", *case.id, solid);
            assert!(
                within(t.raw, expected.toi, TOI_TOLERANCE),
                "{}: toi {} vs {}",
                *case.id,
                t.raw,
                expected.toi,
            );
        },
        None => assert!(!expected.has_toi, "{}: missing toi (solid {})", *case.id, solid),
    }
    let hit: Option<RayIntersection> = cast_ray_and_get_normal(s, p, ray, max, solid);
    let e = expected.hit;
    match hit {
        Some(h) => {
            assert!(e.hit, "{}: unexpected hit (solid {})", *case.id, solid);
            assert!(
                within(h.time_of_impact.raw, e.time_of_impact, TOI_TOLERANCE),
                "{}: hit toi {} vs {}",
                *case.id,
                h.time_of_impact.raw,
                e.time_of_impact,
            );
            assert!(
                vec2_within(raw(h.normal), e.normal, NORMAL_TOLERANCE),
                "{}: normal {:?} vs {:?}",
                *case.id,
                raw(h.normal),
                e.normal,
            );
            assert!(h.feature == feature(e.feature), "{}: feature", *case.id);
        },
        None => assert!(!e.hit, "{}: missing hit (solid {})", *case.id, solid),
    }
}

#[test]
fn test_ray_casts_golden() {
    for case in ray_casts::cases() {
        check(case, true, frozen_parry030::ray_cast_answer(*case.id, true, *case.solid));
        check(case, false, frozen_parry030::ray_cast_answer(*case.id, false, *case.hollow));
    }
}

/// `capsule/inside`, hollow, in parry 0.30.2 (the frozen copy; 0.31.1 answers the true exit, as
/// the port): upstream shifted the origin by `shift` along `dir / |dir|` (a length), cast back
/// along `-dir` (whose times are in units of `|dir|`) and returned `shift - toi_back`, which is
/// the exit time only for a unit `dir`. With `L = t |dir|` the exit distance, 0.30.2 answered
/// `shift - (shift - L) / |dir|`.
#[test]
fn test_capsule_hollow_upstream_units_bug() {
    let case = ray_casts::CAPSULE_INSIDE;
    let capsule = match shape(case.shape) {
        Shape::Capsule(c) => c,
        _ => panic!("capsule/inside is a capsule"),
    };
    let ray = Ray { origin: vector(case.origin), dir: vector(case.dir) };
    let t = cast_ray(Shape::Capsule(capsule), pose(case.pose), ray, fx(case.max_toi), false)
        .unwrap();
    let len = norm2(ray.dir.x, ray.dir.y);
    let ndir = Vec2 { x: ray.dir.x / len, y: ray.dir.y / len };
    let supp = capsule.local_support_point_toward(ndir);
    let shift = (supp - ray.origin).dot(ndir) + SHIFT_EPS;
    let upstream = shift - (shift - t * len) / len;
    let frozen = frozen_parry030::ray_cast_toi(case.id, false).unwrap();
    assert!(within(upstream.raw, frozen, 64), "{} vs {}", upstream.raw, frozen);
    assert!(!within(t.raw, frozen, 1_000_000), "the port answers the true exit");
    // The rest of the answer (hit, normal, feature) agrees.
    let hit = cast_ray_and_get_normal(
        Shape::Capsule(capsule), pose(case.pose), ray, fx(case.max_toi), false,
    )
        .unwrap();
    assert!(vec2_within(raw(hit.normal), case.hollow.hit.normal, NORMAL_TOLERANCE));
}

#[test]
fn gas_baseline() {}

/// One posed cast per shape family, the `hit` case of each group.
#[test]
fn gas_cast_ray_and_get_normal_ball_golden() {
    let case = opaque(ray_casts::BALL_HIT);
    let _ = cast_ray_and_get_normal(
        shape(case.shape),
        pose(case.pose),
        Ray { origin: vector(case.origin), dir: vector(case.dir) },
        fx(case.max_toi),
        true,
    );
}
