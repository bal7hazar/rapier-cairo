//! SH1 contact manifolds (triangles, round shapes) against Parry f64 0.30.2, plus upstream's
//! `intersection_test` and `distance` on the same placements.
//!
//! Bands (raw Q32.32 units): normals and points 64 for the triangle family (SAT against
//! upstream's SAT or GJK), `ROUND_BAND` for the round shapes (upstream's GJK stops within its
//! tolerance, `~1e-9`); distances as the points. Ambiguous cases (exact touching) only compare
//! the point count and distances.
use fixed::Fixed;
use glam::Vec2;
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldTrait};
use rapier_geometry2d::dispatch::{contact_manifold, contact_manifold_step, intersection_test};
use rapier_geometry2d::query::dispatcher::distance;
use rapier_geometry2d::shape::{
    Ball, Capsule, ConvexPolygonTrait, Cuboid, HalfSpace, RoundShape, Segment, Shape, Triangle,
};
use rapier_golden::compare::{abs_diff, vec2_within, within};
use rapier_golden::generated::{round_shape_contacts, triangle_contacts};
use rapier_golden::types::{
    ConvexPolygonRaw, PoseRaw, Sh1ManifoldCase, Sh1ShapeRaw, ShapeRaw, TriangleRaw, Vec2Raw,
};
use rapier_math::consts::UNIT_TOL_SQ_RAW;
use rapier_math::math_ext::norm2::is_unit2_raw;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;

const TRIANGLE_BAND: u64 = 64;
const ROUND_BAND: u64 = 64;

fn v(p: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: p.x }, y: Fixed { raw: p.y } }
}
fn raw(p: Vec2) -> Vec2Raw {
    Vec2Raw { x: p.x.raw, y: p.y.raw }
}
fn pose(p: PoseRaw) -> Pose2 {
    Pose2 {
        translation: v(p.translation),
        rotation: Rot2 { re: Fixed { raw: p.rotation.re }, im: Fixed { raw: p.rotation.im } },
    }
}
fn triangle(t: TriangleRaw) -> Triangle {
    Triangle { a: v(t.a), b: v(t.b), c: v(t.c) }
}
fn polygon(p: ConvexPolygonRaw) -> rapier_geometry2d::shape::ConvexPolygon {
    let mut vertices = array![];
    let mut i = 0;
    while i != p.count {
        vertices.append(v(*p.vertices.span().at(i.into())));
        i += 1;
    }
    ConvexPolygonTrait::from_convex_polyline(vertices.span()).unwrap()
}
pub fn shape(s: Sh1ShapeRaw) -> Shape {
    match s {
        Sh1ShapeRaw::Other(s) => match s {
            ShapeRaw::Ball(r) => Shape::Ball(Ball { radius: Fixed { raw: r } }),
            ShapeRaw::Cuboid(h) => Shape::Cuboid(Cuboid { half_extents: v(h) }),
            ShapeRaw::Segment(s) => Shape::Segment(Segment { a: v(s.a), b: v(s.b) }),
            ShapeRaw::Capsule(c) => Shape::Capsule(
                Capsule {
                    segment: Segment { a: v(c.a), b: v(c.b) }, radius: Fixed { raw: c.radius },
                },
            ),
            ShapeRaw::HalfSpace(n) => Shape::HalfSpace(HalfSpace { normal: v(n) }),
        },
        Sh1ShapeRaw::Polygon(p) => Shape::ConvexPolygon(BoxTrait::new(polygon(p))),
        Sh1ShapeRaw::Triangle(t) => Shape::Triangle(BoxTrait::new(triangle(t))),
        Sh1ShapeRaw::RoundCuboid(r) => Shape::RoundCuboid(
            RoundShape {
                inner_shape: Cuboid { half_extents: v(r.half_extents) },
                border_radius: Fixed { raw: r.border_radius },
            },
        ),
        Sh1ShapeRaw::RoundTriangle(r) => Shape::RoundTriangle(
            BoxTrait::new(
                RoundShape {
                    inner_shape: triangle(r.triangle),
                    border_radius: Fixed { raw: r.border_radius },
                },
            ),
        ),
        Sh1ShapeRaw::RoundPolygon(r) => RoundShape {
            inner_shape: polygon(r.polygon), border_radius: Fixed { raw: r.border_radius },
        }
            .into(),
    }
}

fn check(c: Sh1ManifoldCase, m: ContactManifold, band: u64) {
    assert_eq!(m.num_points.into(), c.num_points, "{} count", c.id);
    if m.num_points == 0 {
        return;
    }
    assert!(is_unit2_raw(m.local_n1.x, m.local_n1.y, UNIT_TOL_SQ_RAW), "{} unit", c.id);
    assert!(
        vec2_within(raw(m.local_n2), raw(pose(c.pos12).inverse_transform_vector(-m.local_n1)), 4),
    );
    if !c.ambiguous {
        assert!(
            vec2_within(raw(m.local_n1), c.local_n1, band),
            "{} n1 {:?} {:?}",
            c.id,
            raw(m.local_n1),
            c.local_n1,
        );
        assert!(vec2_within(raw(m.local_n2), c.local_n2, band), "{} n2", c.id);
    }
    let mut used0 = false;
    let mut used1 = false;
    let mut i = 0;
    while i != m.num_points {
        let a = m.point(i);
        let mut found = false;
        let mut j = 0;
        while j != c.num_points {
            let b = *c.points.span().at(j);
            let unused = if j == 0 {
                !used0
            } else {
                !used1
            };
            if !found
                && unused
                && within(a.dist.raw, b.dist, band)
                && (c.ambiguous
                    || (vec2_within(raw(a.local_p1), b.local_p1, band)
                        && vec2_within(raw(a.local_p2), b.local_p2, band)
                        && a.fid1.packed == b.fid1
                        && a.fid2.packed == b.fid2)) {
                found = true;
                if j == 0 {
                    used0 = true;
                } else {
                    used1 = true;
                }
            }
            j += 1;
        }
        assert!(found, "{} point {} {:?} expected {:?}", c.id, i, a, c.points);
        i += 1;
    }
}

fn run(cases: Span<Sh1ManifoldCase>, prediction: i64, band: u64) -> u32 {
    let mut count = 0;
    for c in cases {
        let (s1, s2, p) = (shape(*c.shape1), shape(*c.shape2), pose(*c.pos12));
        let mut m: ContactManifold = Default::default();
        assert!(contact_manifold(p, s1, s2, Fixed { raw: prediction }, ref m), "{}", *c.id);
        check(*c, m, band);
        // The step's table answers the same.
        let mut step: ContactManifold = Default::default();
        assert!(contact_manifold_step(p, s1, s2, Fixed { raw: prediction }, ref step));
        assert_eq!(step, m);
        // Warm start: a second call on the stored manifold keeps the answer.
        let mut again = m;
        assert!(contact_manifold(p, s1, s2, Fixed { raw: prediction }, ref again));
        check(*c, again, band);
        if !*c.ambiguous {
            assert_eq!(intersection_test(p, s1, s2), Some(*c.intersects), "{} intersects", *c.id);
            let d = distance(p, s1, s2).unwrap();
            assert!(abs_diff(d.raw, *c.distance) <= band, "{} distance {:?}", *c.id, d);
        }
        count += 1;
    }
    count
}

#[test]
fn test_triangle_manifolds_golden() {
    assert_eq!(run(triangle_contacts::cases(), triangle_contacts::PREDICTION, TRIANGLE_BAND), 40);
}

#[test]
fn test_round_shape_manifolds_golden() {
    assert_eq!(
        run(round_shape_contacts::cases(), round_shape_contacts::PREDICTION, ROUND_BAND), 56,
    );
}
