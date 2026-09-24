//! Shape descriptions with Q32.32 parameters.

use crate::q::{jq, jqvec, QVec, Q};
use rapier2d_f64::parry::shape::{Ball, Capsule, Cuboid, HalfSpace, Segment, SharedShape};
use serde_json::{json, Value};

#[derive(Copy, Clone, Debug)]
pub enum ShapeSpec {
    ConvexPolygon {
        vertices: [QVec; 8],
        count: usize,
    },
    Ball {
        radius: Q,
    },
    Cuboid {
        half_extents: QVec,
    },
    Capsule {
        a: QVec,
        b: QVec,
        radius: Q,
    },
    /// `normal` must be one of the four axis directions so that it is an exact unit vector.
    HalfSpace {
        normal: QVec,
    },
    Segment {
        a: QVec,
        b: QVec,
    },
}

impl ShapeSpec {
    pub fn polygon(points: &[(f64, f64)]) -> Self {
        assert!((3..=8).contains(&points.len()));
        let mut vertices = [QVec::ZERO; 8];
        for (i, &(x, y)) in points.iter().enumerate() {
            vertices[i] = QVec::snap(x, y);
        }
        Self::ConvexPolygon {
            vertices,
            count: points.len(),
        }
    }

    pub fn polygons() -> Vec<(&'static str, Self)> {
        vec![
            (
                "poly_tri",
                Self::polygon(&[(-1.0, -1.0), (1.0, -1.0), (0.0, 1.0)]),
            ),
            (
                "poly_quad",
                Self::polygon(&[(-1.0, -0.5), (1.0, -0.5), (1.0, 0.5), (-1.0, 0.5)]),
            ),
            (
                "poly_pent",
                Self::polygon(&[
                    (-1.0, -1.0),
                    (1.0, -1.0),
                    (1.5, 0.0),
                    (0.0, 1.5),
                    (-1.5, 0.0),
                ]),
            ),
            (
                "poly_oct",
                Self::polygon(&[
                    (-1.0, -2.0),
                    (1.0, -2.0),
                    (2.0, -1.0),
                    (2.0, 1.0),
                    (1.0, 2.0),
                    (-1.0, 2.0),
                    (-2.0, 1.0),
                    (-2.0, -1.0),
                ]),
            ),
            (
                "poly_rot",
                Self::polygon(&[(-0.2, -1.1), (1.0, 0.5), (0.2, 1.1), (-1.0, -0.5)]),
            ),
            (
                "poly_thin",
                Self::polygon(&[(-2.0, -0.01), (2.0, -0.01), (2.0, 0.01), (-2.0, 0.01)]),
            ),
        ]
    }

    pub fn ball(radius: f64) -> Self {
        ShapeSpec::Ball {
            radius: Q::snap(radius),
        }
    }

    pub fn cuboid(hx: f64, hy: f64) -> Self {
        ShapeSpec::Cuboid {
            half_extents: QVec::snap(hx, hy),
        }
    }

    /// Capsule aligned with the local X axis.
    pub fn capsule_x(half_height: f64, radius: f64) -> Self {
        ShapeSpec::Capsule {
            a: QVec::snap(-half_height, 0.0),
            b: QVec::snap(half_height, 0.0),
            radius: Q::snap(radius),
        }
    }

    /// Capsule aligned with the local Y axis.
    pub fn capsule_y(half_height: f64, radius: f64) -> Self {
        ShapeSpec::Capsule {
            a: QVec::snap(0.0, -half_height),
            b: QVec::snap(0.0, half_height),
            radius: Q::snap(radius),
        }
    }

    pub fn capsule(a: (f64, f64), b: (f64, f64), radius: f64) -> Self {
        ShapeSpec::Capsule {
            a: QVec::snap(a.0, a.1),
            b: QVec::snap(b.0, b.1),
            radius: Q::snap(radius),
        }
    }

    /// Half-space whose outward normal is +Y.
    pub fn halfspace_up() -> Self {
        ShapeSpec::HalfSpace {
            normal: QVec::snap(0.0, 1.0),
        }
    }

    pub fn segment(a: (f64, f64), b: (f64, f64)) -> Self {
        ShapeSpec::Segment {
            a: QVec::snap(a.0, a.1),
            b: QVec::snap(b.0, b.1),
        }
    }

    pub fn shared(&self) -> SharedShape {
        match *self {
            ShapeSpec::ConvexPolygon { vertices, count } => SharedShape::new(
                rapier2d_f64::parry::shape::ConvexPolygon::from_convex_polyline(
                    vertices[..count].iter().map(|p| p.v()).collect(),
                )
                .unwrap(),
            ),
            ShapeSpec::Ball { radius } => SharedShape::new(Ball::new(radius.f())),
            ShapeSpec::Cuboid { half_extents } => SharedShape::new(Cuboid::new(half_extents.v())),
            ShapeSpec::Capsule { a, b, radius } => {
                SharedShape::new(Capsule::new(a.v(), b.v(), radius.f()))
            }
            ShapeSpec::HalfSpace { normal } => SharedShape::new(HalfSpace::new(normal.v())),
            ShapeSpec::Segment { a, b } => SharedShape::new(Segment::new(a.v(), b.v())),
        }
    }

    /// The same shape for the f32 build of parry (feature-id cross-check only).
    pub fn shared_f32(&self) -> parry2d::shape::SharedShape {
        use parry2d::math::Vector as V;
        use parry2d::shape as sh;
        let v = |q: QVec| V::new(q.x.f() as f32, q.y.f() as f32);
        match *self {
            ShapeSpec::ConvexPolygon { vertices, count } => sh::SharedShape::new(
                sh::ConvexPolygon::from_convex_polyline(
                    vertices[..count].iter().map(|p| v(*p)).collect(),
                )
                .unwrap(),
            ),
            ShapeSpec::Ball { radius } => sh::SharedShape::new(sh::Ball::new(radius.f() as f32)),
            ShapeSpec::Cuboid { half_extents } => {
                sh::SharedShape::new(sh::Cuboid::new(v(half_extents)))
            }
            ShapeSpec::Capsule { a, b, radius } => {
                sh::SharedShape::new(sh::Capsule::new(v(a), v(b), radius.f() as f32))
            }
            ShapeSpec::HalfSpace { normal } => sh::SharedShape::new(sh::HalfSpace::new(v(normal))),
            ShapeSpec::Segment { a, b } => sh::SharedShape::new(sh::Segment::new(v(a), v(b))),
        }
    }

    pub fn json(&self) -> Value {
        match *self {
            ShapeSpec::ConvexPolygon { vertices, count } => json!({ "type": "convex_polygon",
                "vertices": vertices[..count].iter().map(|p| jqvec(*p)).collect::<Vec<_>>() }),
            ShapeSpec::Ball { radius } => json!({ "type": "ball", "radius": jq(radius) }),
            ShapeSpec::Cuboid { half_extents } => {
                json!({ "type": "cuboid", "half_extents": jqvec(half_extents) })
            }
            ShapeSpec::Capsule { a, b, radius } => {
                json!({ "type": "capsule", "a": jqvec(a), "b": jqvec(b), "radius": jq(radius) })
            }
            ShapeSpec::HalfSpace { normal } => {
                json!({ "type": "halfspace", "normal": jqvec(normal) })
            }
            ShapeSpec::Segment { a, b } => {
                json!({ "type": "segment", "a": jqvec(a), "b": jqvec(b) })
            }
        }
    }
}
