//! SAT/PFM vs upstream GJK/EPA: 48 cases over every pair/order/regime.
use fixed::Fixed;
use glam::Vec2;
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldTrait};
use rapier_geometry2d::dispatch::contact_manifold;
use rapier_geometry2d::shape::{
    Ball, Capsule, ConvexPolygonTrait, Cuboid, HalfSpace, Segment, Shape,
};
use rapier_golden::compare::{vec2_within, within};
use rapier_golden::generated::polygon_contacts;
use rapier_golden::types::{PolygonContactShapeRaw, PolygonManifoldCase, PoseRaw, ShapeRaw, Vec2Raw};
use rapier_math::consts::UNIT_TOL_SQ_RAW;
use rapier_math::math_ext::norm2::is_unit2_raw;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;

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
fn shape(s: PolygonContactShapeRaw) -> Shape {
    match s {
        PolygonContactShapeRaw::Polygon(p) => {
            let mut vertices = array![];
            let mut i = 0;
            while i != p.count {
                vertices.append(v(*p.vertices.span().at(i.into())));
                i += 1;
            }
            Shape::ConvexPolygon(
                BoxTrait::new(ConvexPolygonTrait::from_convex_polyline(vertices.span()).unwrap()),
            )
        },
        PolygonContactShapeRaw::Other(s) => match s {
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
    }
}
fn check(c: PolygonManifoldCase, m: ContactManifold) {
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
            vec2_within(raw(m.local_n1), c.local_n1, 64),
            "{} n1 {:?} {:?}",
            c.id,
            raw(m.local_n1),
            c.local_n1,
        );
        assert!(vec2_within(raw(m.local_n2), c.local_n2, 64), "{} n2", c.id);
    }
    let mut i = 0;
    let mut used0 = false;
    let mut used1 = false;
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
                && (within(a.dist.raw, b.dist, 64)
                    && (c.ambiguous
                        || (vec2_within(raw(a.local_p1), b.local_p1, 64)
                            && vec2_within(raw(a.local_p2), b.local_p2, 64)
                            && a.fid1.packed == b.fid1
                            && a.fid2.packed == b.fid2))) {
                found = true;
                if j == 0 {
                    used0 = true;
                } else {
                    used1 = true;
                }
            }
            j += 1;
        }
        assert!(found, "{} point {} {:?}", c.id, i, a);
        i += 1;
    }
}
#[test]
fn test_polygon_manifolds_golden() {
    let mut count = 0;
    for c in polygon_contacts::cases() {
        let mut m: ContactManifold = Default::default();
        assert!(
            contact_manifold(
                pose(*c.pos12),
                shape(*c.shape1),
                shape(*c.shape2),
                Fixed { raw: polygon_contacts::PREDICTION },
                ref m,
            ),
        );
        check(*c, m);
        count += 1;
    }
    assert_eq!(count, 48);
}
