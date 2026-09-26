//! Work package SH1: triangles and round shapes. Three families, in files of their own so that
//! no existing table grows:
//!
//! * `triangle_contacts`: `DefaultQueryDispatcher::contact_manifolds` of a triangle against every
//!   shape of the closed set (and a few reversed orders), four regimes each;
//! * `round_shape_contacts`: the same for the round cuboid, triangle and convex polygon;
//! * `sh1_queries`: point projections (with the triangle's `TrianglePointLocation`), world-space
//!   ray casts, mass properties and AABBs of the four new shapes.
//!
//! Every manifold case also records `query::intersection_test` and `query::distance` for the
//! same placement (`pos1` = identity, `pos2` = `pos12`).
//!
//! Placement: shape 2 sits below shape 1 (`pos12 = (dx, -(bottom1 + top2 + gap))`, `bottom` and
//! `top` the local extents along `y`), except for a half-space first, where shape 2 sits above
//! the plane (`pos12 = (dx, bottom2 + gap)`). Regimes: `separated` (gap 0.5), `within_pred`
//! (gap 0.01 < prediction), `touching` (gap 0, tagged ambiguous: exact ties) and `overlapping`
//! (gap -0.05, shape 2 turned by 10 degrees).

use crate::leaf::qv;
use crate::manifolds::fid_json;
use crate::mass::mprops_json;
use crate::point_projection::{feature_json, proj_json};
use crate::q::{jf, jq, jqpose, jqvec, jvec, QPose, QRot, QVec, Q};
use crate::ray_casts::hit_json;
use crate::shapes::ShapeSpec;
use rapier2d_f64::dynamics::IntegrationParameters;
use rapier2d_f64::math::{Pose, Vector};
use rapier2d_f64::parry::query::{
    self, ContactManifold, DefaultQueryDispatcher, PersistentQueryDispatcher,
    PointQueryWithLocation, Ray,
};
use rapier2d_f64::parry::shape::{
    ConvexPolygon, Cuboid, RoundShape, Shape, SharedShape, Triangle, TrianglePointLocation,
};
use serde_json::{json, Value};

/// A shape of the SH1 families: one of the closed MVP set, or a new one.
#[derive(Copy, Clone, Debug)]
pub enum Sh1Shape {
    Base(ShapeSpec),
    Triangle { a: QVec, b: QVec, c: QVec },
    RoundCuboid { half_extents: QVec, border_radius: Q },
    RoundTriangle { a: QVec, b: QVec, c: QVec, border_radius: Q },
    RoundPolygon { vertices: [QVec; 8], count: usize, border_radius: Q },
}

fn polygon_f64(vertices: &[QVec; 8], count: usize) -> ConvexPolygon {
    ConvexPolygon::from_convex_polyline(vertices[..count].iter().map(|p| p.v()).collect()).unwrap()
}

impl Sh1Shape {
    fn triangle(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> Self {
        Sh1Shape::Triangle { a: qv(a.0, a.1), b: qv(b.0, b.1), c: qv(c.0, c.1) }
    }

    fn rounded(self, r: f64) -> Self {
        let border_radius = Q::snap(r);
        match self {
            Sh1Shape::Base(ShapeSpec::Cuboid { half_extents }) => {
                Sh1Shape::RoundCuboid { half_extents, border_radius }
            }
            Sh1Shape::Base(ShapeSpec::ConvexPolygon { vertices, count }) => {
                Sh1Shape::RoundPolygon { vertices, count, border_radius }
            }
            Sh1Shape::Triangle { a, b, c } => Sh1Shape::RoundTriangle { a, b, c, border_radius },
            other => panic!("cannot round {other:?}"),
        }
    }

    pub fn shared(&self) -> SharedShape {
        match *self {
            Sh1Shape::Base(s) => s.shared(),
            Sh1Shape::Triangle { a, b, c } => SharedShape::new(Triangle::new(a.v(), b.v(), c.v())),
            Sh1Shape::RoundCuboid { half_extents, border_radius } => SharedShape::new(RoundShape {
                inner_shape: Cuboid::new(half_extents.v()),
                border_radius: border_radius.f(),
            }),
            Sh1Shape::RoundTriangle { a, b, c, border_radius } => SharedShape::new(RoundShape {
                inner_shape: Triangle::new(a.v(), b.v(), c.v()),
                border_radius: border_radius.f(),
            }),
            Sh1Shape::RoundPolygon { vertices, count, border_radius } => {
                SharedShape::new(RoundShape {
                    inner_shape: polygon_f64(&vertices, count),
                    border_radius: border_radius.f(),
                })
            }
        }
    }

    fn shared_f32(&self) -> parry2d::shape::SharedShape {
        use parry2d::math::Vector as V;
        use parry2d::shape as sh;
        let v = |q: QVec| V::new(q.x.f() as f32, q.y.f() as f32);
        match *self {
            Sh1Shape::Base(s) => s.shared_f32(),
            Sh1Shape::Triangle { a, b, c } => {
                sh::SharedShape::new(sh::Triangle::new(v(a), v(b), v(c)))
            }
            Sh1Shape::RoundCuboid { half_extents, border_radius } => {
                sh::SharedShape::new(sh::RoundShape {
                    inner_shape: sh::Cuboid::new(v(half_extents)),
                    border_radius: border_radius.f() as f32,
                })
            }
            Sh1Shape::RoundTriangle { a, b, c, border_radius } => {
                sh::SharedShape::new(sh::RoundShape {
                    inner_shape: sh::Triangle::new(v(a), v(b), v(c)),
                    border_radius: border_radius.f() as f32,
                })
            }
            Sh1Shape::RoundPolygon { vertices, count, border_radius } => {
                sh::SharedShape::new(sh::RoundShape {
                    inner_shape: sh::ConvexPolygon::from_convex_polyline(
                        vertices[..count].iter().map(|p| v(*p)).collect(),
                    )
                    .unwrap(),
                    border_radius: border_radius.f() as f32,
                })
            }
        }
    }

    pub fn json(&self) -> Value {
        match *self {
            Sh1Shape::Base(s) => s.json(),
            Sh1Shape::Triangle { a, b, c } => {
                json!({ "type": "triangle", "a": jqvec(a), "b": jqvec(b), "c": jqvec(c) })
            }
            Sh1Shape::RoundCuboid { half_extents, border_radius } => json!({
                "type": "round_cuboid", "half_extents": jqvec(half_extents),
                "border_radius": jq(border_radius) }),
            Sh1Shape::RoundTriangle { a, b, c, border_radius } => json!({
                "type": "round_triangle", "a": jqvec(a), "b": jqvec(b), "c": jqvec(c),
                "border_radius": jq(border_radius) }),
            Sh1Shape::RoundPolygon { vertices, count, border_radius } => json!({
                "type": "round_convex_polygon",
                "vertices": vertices[..count].iter().map(|p| jqvec(*p)).collect::<Vec<_>>(),
                "border_radius": jq(border_radius) }),
        }
    }

    /// `(min y, max y)` of the unrotated shape; `(0, 0)` for the half-space.
    fn y_range(&self) -> (f64, f64) {
        let pts = |ps: &[QVec]| {
            let ys: Vec<f64> = ps.iter().map(|p| p.y.f()).collect();
            (ys.iter().cloned().fold(f64::MAX, f64::min), ys.iter().cloned().fold(f64::MIN, f64::max))
        };
        match *self {
            Sh1Shape::Base(ShapeSpec::Ball { radius }) => (-radius.f(), radius.f()),
            Sh1Shape::Base(ShapeSpec::Cuboid { half_extents }) => {
                (-half_extents.y.f(), half_extents.y.f())
            }
            Sh1Shape::Base(ShapeSpec::Capsule { a, b, radius }) => {
                let (lo, hi) = pts(&[a, b]);
                (lo - radius.f(), hi + radius.f())
            }
            Sh1Shape::Base(ShapeSpec::Segment { a, b }) => pts(&[a, b]),
            Sh1Shape::Base(ShapeSpec::HalfSpace { .. }) => (0.0, 0.0),
            Sh1Shape::Base(ShapeSpec::ConvexPolygon { vertices, count }) => pts(&vertices[..count]),
            Sh1Shape::Triangle { a, b, c } => pts(&[a, b, c]),
            Sh1Shape::RoundCuboid { half_extents, border_radius } => {
                let r = half_extents.y.f() + border_radius.f();
                (-r, r)
            }
            Sh1Shape::RoundTriangle { a, b, c, border_radius } => {
                let (lo, hi) = pts(&[a, b, c]);
                (lo - border_radius.f(), hi + border_radius.f())
            }
            Sh1Shape::RoundPolygon { vertices, count, border_radius } => {
                let (lo, hi) = pts(&vertices[..count]);
                (lo - border_radius.f(), hi + border_radius.f())
            }
        }
    }

    fn is_halfspace(&self) -> bool {
        matches!(self, Sh1Shape::Base(ShapeSpec::HalfSpace { .. }))
    }
}

// ---------------------------------------------------------------------------------------------
// The shapes
// ---------------------------------------------------------------------------------------------

fn tri() -> Sh1Shape {
    Sh1Shape::triangle((-1.0, -0.5), (1.0, -0.5), (0.25, 0.75))
}

fn tri_cw() -> Sh1Shape {
    Sh1Shape::triangle((-1.0, -0.5), (0.25, 0.75), (1.0, -0.5))
}

fn small_tri() -> Sh1Shape {
    Sh1Shape::triangle((-0.4, -0.3), (0.4, -0.3), (0.0, 0.4))
}

fn pentagon() -> ShapeSpec {
    ShapeSpec::polygon(&[(-0.5, -0.4), (0.5, -0.4), (0.6, 0.1), (0.0, 0.5), (-0.6, 0.1)])
}

fn b(s: ShapeSpec) -> Sh1Shape {
    Sh1Shape::Base(s)
}

fn rcub() -> Sh1Shape {
    b(ShapeSpec::cuboid(0.4, 0.25)).rounded(0.1)
}

fn rtri() -> Sh1Shape {
    tri().rounded(0.1)
}

fn rpoly() -> Sh1Shape {
    b(pentagon()).rounded(0.1)
}

// ---------------------------------------------------------------------------------------------
// Manifolds
// ---------------------------------------------------------------------------------------------

struct Case {
    pair: &'static str,
    regime: &'static str,
    shape1: Sh1Shape,
    shape2: Sh1Shape,
    pos12: QPose,
    ambiguous: bool,
}

const REGIMES: [(&str, f64, f64, f64, bool); 4] = [
    ("separated", 0.5, 0.1, 0.0, false),
    ("within_pred", 0.01, 0.1, 0.0, false),
    ("touching", 0.0, 0.1, 0.0, true),
    ("overlapping", -0.05, 0.15, 10.0, false),
];

/// The `x` of the topmost point of `shape` when it is the triangle's apex (so that shape 2's
/// apex comes under shape 1's centre), 0 otherwise.
fn apex_x(shape: &Sh1Shape) -> f64 {
    match *shape {
        Sh1Shape::Triangle { c, .. } | Sh1Shape::RoundTriangle { c, .. } => c.x.f(),
        _ => 0.0,
    }
}

fn pair_cases(pairs: &[(&'static str, Sh1Shape, Sh1Shape)], ambiguous: &[&str]) -> Vec<Case> {
    let mut out = Vec::new();
    for &(pair, shape1, shape2) in pairs {
        let dx0 = if shape1.is_halfspace() { 0.0 } else { -apex_x(&shape2) };
        for (regime, gap, dx, deg, tie) in REGIMES {
            let y = if shape1.is_halfspace() {
                -shape2.y_range().0 + gap
            } else {
                -(-shape1.y_range().0 + shape2.y_range().1 + gap)
            };
            let rot = if deg == 0.0 { QRot::IDENTITY } else { QRot::from_degrees(deg) };
            let id = format!("{pair}/{regime}");
            out.push(Case {
                pair,
                regime,
                shape1,
                shape2,
                pos12: QPose::new(QVec::snap(dx0 + dx, y), rot),
                ambiguous: tie || ambiguous.contains(&id.as_str()),
            });
        }
    }
    out
}

fn triangle_cases() -> Vec<Case> {
    let halfspace = b(ShapeSpec::halfspace_up());
    pair_cases(
        &[
            ("tri_ball", tri(), b(ShapeSpec::ball(0.4))),
            ("ball_tri", b(ShapeSpec::ball(0.4)), tri()),
            ("tri_cuboid", tri(), b(ShapeSpec::cuboid(0.5, 0.3))),
            ("cuboid_tri", b(ShapeSpec::cuboid(0.5, 0.3)), tri()),
            ("tri_capsule", tri(), b(ShapeSpec::capsule_y(0.3, 0.2))),
            ("tri_segment", tri(), b(ShapeSpec::segment((-0.4, 0.0), (0.4, 0.0)))),
            ("halfspace_tri", halfspace, tri()),
            ("tri_halfspace", tri(), halfspace),
            ("tri_poly", tri(), b(pentagon())),
            ("tri_tri", tri(), small_tri()),
        ],
        &[],
    )
}

fn round_cases() -> Vec<Case> {
    let halfspace = b(ShapeSpec::halfspace_up());
    pair_cases(
        &[
            ("rcub_ball", rcub(), b(ShapeSpec::ball(0.4))),
            ("ball_rcub", b(ShapeSpec::ball(0.4)), rcub()),
            ("rcub_cuboid", rcub(), b(ShapeSpec::cuboid(0.5, 0.3))),
            ("cuboid_rcub", b(ShapeSpec::cuboid(0.5, 0.3)), rcub()),
            ("rcub_capsule", rcub(), b(ShapeSpec::capsule_y(0.3, 0.2))),
            ("rcub_rcub", rcub(), rcub()),
            ("halfspace_rcub", halfspace, rcub()),
            ("rcub_halfspace", rcub(), halfspace),
            ("rtri_ball", rtri(), b(ShapeSpec::ball(0.4))),
            ("rtri_cuboid", rtri(), b(ShapeSpec::cuboid(0.5, 0.3))),
            ("rtri_tri", rtri(), small_tri()),
            ("rpoly_poly", rpoly(), b(pentagon())),
            ("rpoly_rcub", rpoly(), rcub()),
            ("halfspace_rpoly", halfspace, rpoly()),
        ],
        &[],
    )
}

type PointF32 = ([f32; 2], u32, u32);

fn run_f32(case: &Case, prediction: Q) -> Vec<Vec<PointF32>> {
    use parry2d::math::{Pose, Rotation, Vector};
    use parry2d::query::{ContactManifold, DefaultQueryDispatcher, PersistentQueryDispatcher};
    let t = case.pos12.translation;
    let r = case.pos12.rotation;
    let pos12 = Pose::from_parts(
        Vector::new(t.x.f() as f32, t.y.f() as f32),
        Rotation { re: r.re.f() as f32, im: r.im.f() as f32 },
    );
    let mut manifolds: Vec<ContactManifold<(), ()>> = Vec::new();
    let mut workspace = None;
    let (shape1, shape2) = (case.shape1.shared_f32(), case.shape2.shared_f32());
    DefaultQueryDispatcher
        .contact_manifolds(&pos12, &*shape1.0, &*shape2.0, prediction.f() as f32, &mut manifolds, &mut workspace)
        .expect("unsupported shape pair");
    manifolds
        .iter()
        .map(|m| m.points.iter().map(|p| ([p.local_p1.x, p.local_p1.y], p.fid1.0, p.fid2.0)).collect())
        .collect()
}

fn run(case: &Case, prediction: Q) -> Value {
    let mut manifolds: Vec<ContactManifold<(), ()>> = Vec::new();
    let mut workspace = None;
    let (shape1, shape2) = (case.shape1.shared(), case.shape2.shared());
    let pos12 = case.pos12.p();
    DefaultQueryDispatcher
        .contact_manifolds(&pos12, &*shape1.0, &*shape2.0, prediction.f(), &mut manifolds, &mut workspace)
        .expect("unsupported shape pair");
    let manifolds_f32 = run_f32(case, prediction);
    let id = format!("{}/{}", case.pair, case.regime);
    let manifolds_json: Vec<Value> = manifolds
        .iter()
        .enumerate()
        .map(|(i, m)| {
            let points_f32 = manifolds_f32.get(i).filter(|pts| {
                pts.len() == m.points.len()
                    && pts.iter().zip(&m.points).all(|(a, b)| {
                        (a.0[0] as f64 - b.local_p1.x).abs() < 1e-4
                            && (a.0[1] as f64 - b.local_p1.y).abs() < 1e-4
                    })
            });
            assert!(
                points_f32.is_some() || case.ambiguous,
                "{id}: the f32 and f64 builds disagree on a case not tagged ambiguous"
            );
            let points: Vec<Value> = m
                .points
                .iter()
                .enumerate()
                .map(|(k, p)| {
                    let (fid1, fid2) = match points_f32 {
                        Some(pts) => (fid_json(pts[k].1), fid_json(pts[k].2)),
                        None => (Value::Null, Value::Null),
                    };
                    json!({
                        "local_p1": jvec(p.local_p1),
                        "local_p2": jvec(p.local_p2),
                        "dist": jf(p.dist),
                        "fid1": fid1,
                        "fid2": fid2,
                        "fid1_f64_build": fid_json(p.fid1.0),
                        "fid2_f64_build": fid_json(p.fid2.0),
                    })
                })
                .collect();
            json!({
                "local_n1": jvec(m.local_n1),
                "local_n2": jvec(m.local_n2),
                "num_points": points.len(),
                "points": points,
            })
        })
        .collect();
    let identity = Pose::IDENTITY;
    let intersects = query::intersection_test(&identity, &*shape1.0, &pos12, &*shape2.0);
    let distance = query::distance(&identity, &*shape1.0, &pos12, &*shape2.0);
    json!({
        "id": id,
        "pair": case.pair,
        "regime": case.regime,
        "ambiguous": case.ambiguous,
        "shape1": case.shape1.json(),
        "shape2": case.shape2.json(),
        "pos12": jqpose(case.pos12),
        "expected": {
            "num_manifolds": manifolds_json.len(),
            "manifolds": manifolds_json,
            "intersects": intersects.ok(),
            "distance": distance.ok().map(jf),
        },
    })
}

fn contacts(family: &str, cases: Vec<Case>) -> Value {
    let default_prediction = IntegrationParameters::default().prediction_distance();
    let prediction = Q::snap(default_prediction);
    json!({
        "family": family,
        "prediction_upstream_default": jf(default_prediction),
        "prediction": jq(prediction),
        "cases": cases.iter().map(|c| run(c, prediction)).collect::<Vec<_>>(),
    })
}

pub fn triangle_contacts() -> Value {
    contacts("triangle_contacts", triangle_cases())
}

pub fn round_shape_contacts() -> Value {
    contacts("round_shape_contacts", round_cases())
}

// ---------------------------------------------------------------------------------------------
// Point, ray, mass and AABB queries
// ---------------------------------------------------------------------------------------------

fn query_shapes() -> Vec<(&'static str, Sh1Shape)> {
    vec![("tri", tri()), ("tri_cw", tri_cw()), ("rcub", rcub()), ("rtri", rtri()), ("rpoly", rpoly())]
}

fn location_json(l: &TrianglePointLocation) -> Value {
    match l {
        TrianglePointLocation::OnVertex(i) => json!({ "kind": "vertex", "vertex": i }),
        TrianglePointLocation::OnEdge(i, [u, v]) => {
            json!({ "kind": "edge", "edge": i, "u": jf(*u), "v": jf(*v) })
        }
        TrianglePointLocation::OnFace(..) => panic!("2D triangles never answer OnFace"),
        TrianglePointLocation::OnSolid => json!({ "kind": "solid" }),
    }
}

fn point_case(name: &str, shape: Sh1Shape, point: QVec) -> Value {
    let p: Vector = point.v();
    let s = shape.shared();
    let (from_feature, feature) = s.project_local_point_and_get_feature(p);
    let projection = s.project_local_point(p, false);
    assert_eq!(projection.point, from_feature.point);
    let location = match shape {
        Sh1Shape::Triangle { a, b, c } => {
            let t = Triangle::new(a.v(), b.v(), c.v());
            let (with_loc, loc) = t.project_local_point_and_get_location(p, false);
            assert_eq!(with_loc.point, projection.point);
            location_json(&loc)
        }
        _ => json!({ "kind": "none" }),
    };
    json!({
        "id": name,
        "shape": shape.json(),
        "point": jqvec(point),
        "expected": {
            "projection": proj_json(&projection),
            "projection_solid": proj_json(&s.project_local_point(p, true)),
            "distance": jf(s.distance_to_local_point(p, false)),
            "feature": feature_json(feature),
            "location": location,
        },
    })
}

fn points() -> Vec<Value> {
    let pts = [
        ("out_vertex", 1.5, -1.0),
        ("out_edge", 0.1, -1.2),
        ("out_slant", 1.2, 0.6),
        ("inside", 0.1, -0.2),
        ("near_edge", 0.0, -0.45),
        ("shell", 0.0, -0.55),
    ];
    let mut out = Vec::new();
    for (shape_name, shape) in query_shapes() {
        for (name, x, y) in pts {
            out.push(point_case(&format!("{shape_name}/{name}"), shape, qv(x, y)));
        }
    }
    out
}

fn ray_answer(s: &dyn Shape, pose: &Pose, ray: &Ray, max: f64, solid: bool) -> Value {
    json!({
        "toi": s.cast_ray(pose, ray, max, solid).map(jf).unwrap_or(Value::Null),
        "hit": hit_json(s.cast_ray_and_get_normal(pose, ray, max, solid)),
    })
}

fn rays() -> Vec<Value> {
    let posed = QPose::new(qv(0.5, -0.25), QRot::from_degrees(30.0));
    let id = QPose::new(QVec::ZERO, QRot::IDENTITY);
    let rs = [
        ("hit", id, (-3.0, 0.1), (1.0, 0.0)),
        ("oblique", posed, (2.0, 2.0), (-0.6, -0.8)),
        // Unit `dir`: upstream's hollow support-map cast from inside mixes units otherwise
        // (the documented `capsule/inside` bug of the `ray_casts` family).
        ("inside", id, (0.1, -0.2), (0.8, 0.6)),
        ("miss", id, (-3.0, 2.0), (1.0, 0.0)),
    ];
    let max = Q::snap(100.0);
    let mut out = Vec::new();
    for (shape_name, shape) in query_shapes() {
        for (name, pose, origin, dir) in rs {
            let (o, d) = (qv(origin.0, origin.1), qv(dir.0, dir.1));
            let s = shape.shared();
            let ray = Ray::new(o.v(), d.v());
            let p = pose.p();
            out.push(json!({
                "id": format!("{shape_name}/{name}"),
                "shape": shape.json(),
                "pose": jqpose(pose),
                "origin": jqvec(o),
                "dir": jqvec(d),
                "max_toi": jq(max),
                "expected": {
                    "solid": ray_answer(&*s.0, &p, &ray, max.f(), true),
                    "hollow": ray_answer(&*s.0, &p, &ray, max.f(), false),
                },
            }));
        }
    }
    out
}

fn masses() -> Vec<Value> {
    let density = Q::snap(2.0);
    query_shapes()
        .into_iter()
        .map(|(name, shape)| {
            json!({
                "id": format!("{name}/d2"),
                "shape": shape.json(),
                "density": jq(density),
                "expected": mprops_json(&shape.shared().mass_properties(density.f())),
            })
        })
        .collect()
}

fn aabbs() -> Vec<Value> {
    let poses = [
        ("identity", QPose::new(QVec::ZERO, QRot::IDENTITY)),
        ("rot30", QPose::new(qv(1.0, 2.0), QRot::from_degrees(30.0))),
    ];
    let mut out = Vec::new();
    for (shape_name, shape) in query_shapes() {
        for (pose_name, pose) in poses {
            let aabb = shape.shared().compute_aabb(&pose.p());
            out.push(json!({
                "id": format!("{shape_name}/{pose_name}"),
                "shape": shape.json(),
                "pose": jqpose(pose),
                "expected": { "mins": jvec(aabb.mins), "maxs": jvec(aabb.maxs) },
            }));
        }
    }
    out
}

pub fn sh1_queries() -> Value {
    json!({
        "family": "sh1_queries",
        "points": points(),
        "rays": rays(),
        "masses": masses(),
        "aabbs": aabbs(),
    })
}
