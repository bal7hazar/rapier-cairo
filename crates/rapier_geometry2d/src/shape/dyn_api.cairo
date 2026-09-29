//! The value meanings of upstream's `dyn Shape` API (work package PX4, off the step path): what
//! `Box<dyn Shape>` / `SharedShape` methods do when a shape is a value of the closed [`Shape`]
//! enum (ADR 0001 entries 35 and 37).
//!
//! * `SharedShape::new`, `Shape::as_shape`: the identity. `Shape::clone_box` / `clone_dyn`: a
//!   copy (a `Shape` is `Copy`).
//! * `Shape::scale_dyn`: the shape scaled by `scale`, dispatching to the per-shape `scaled` of
//!   PX3 (`None` in the cases where those return `None`); a compound scales its parts and their
//!   translations. Upstream's `num_subdivisions` is the outline resolution of a scaled ball or
//!   capsule.
//! * `Shape::ccd_thickness` / `ccd_angular_thickness`: the free functions of
//!   `query::nonlinear_shape_cast`, as methods.
//! * `SharedShape::convex_polyline_unmodified`:
//! [`ConvexPolygonTrait::from_convex_polyline_unmodified`]
//!   as a shape.
//!
//! `SharedShape::make_mut`, `Shape::as_shape_mut` and the `Shape` trait object itself are closed
//! (`scripts/api_parity.py`, "closed value enum replaces Arc / dyn shapes (SH2a)").

use fixed::Fixed;
use glam_core::Vec2;
use rapier_math::pose2::Pose2;
use crate::query::nonlinear_shape_cast::{ccd_angular_thickness, ccd_thickness};
use super::scaled::Either;
use super::{
    BallIntoShape, BallTrait, CapsuleIntoShape, CapsuleTrait, CompoundIntoShape, CompoundTrait,
    ConvexPolygon, ConvexPolygonIntoShape, ConvexPolygonTrait, CuboidIntoShape, CuboidTrait,
    HalfSpaceIntoShape, HalfSpaceTrait, HeightFieldIntoShape, HeightFieldTrait, PolylineIntoShape,
    PolylineTrait, RoundConvexPolygonIntoShape, RoundCuboidIntoShape, RoundShape,
    RoundTriangleIntoShape, SegmentIntoShape, SegmentTrait, Shape, TriangleIntoShape, TriangleTrait,
};

/// A shape out of upstream's `either::Either` of a `scaled` answer.
fn either_shape<L, R, +Into<L, Shape>, +Into<R, Shape>>(scaled: Either<L, R>) -> Shape {
    match scaled {
        Either::Left(shape) => shape.into(),
        Either::Right(shape) => shape.into(),
    }
}

/// The `dyn Shape` / `SharedShape` methods of [`Shape`] values (see the module documentation).
#[generate_trait]
pub impl ShapeDynImpl of ShapeDynTrait {
    /// A shared shape holding `shape` (upstream `SharedShape::new`): `shape`.
    #[inline(always)]
    fn new(shape: Shape) -> Shape {
        shape
    }

    /// The polygon of `points` with every vertex kept, as a shape (upstream
    /// `SharedShape::convex_polyline_unmodified`); `None` as
    /// [`ConvexPolygonTrait::from_convex_polyline_unmodified`].
    fn convex_polyline_unmodified(points: Span<Vec2>) -> Option<Shape> {
        let polygon = ConvexPolygonTrait::from_convex_polyline_unmodified(points)?;
        Some(polygon.into())
    }

    /// The shape itself (upstream `as_shape`, a downcast to the concrete type).
    #[inline(always)]
    fn as_shape(self: Shape) -> Shape {
        self
    }

    /// A copy of the shape (upstream `clone_box`).
    #[inline(always)]
    fn clone_box(self: Shape) -> Shape {
        self
    }

    /// A copy of the shape (upstream `clone_dyn`).
    #[inline(always)]
    fn clone_dyn(self: Shape) -> Shape {
        self
    }

    /// The shape scaled component-wise by `scale` (upstream `scale_dyn`): see the module
    /// documentation. `None` when the per-shape `scaled` answers `None` (a zero normal of a
    /// polygon or half-space, a degenerate outline) or when a part of a compound does.
    /// #### Panics
    /// * As the per-shape `scaled` functions (overflow of a coordinate).
    fn scale_dyn(self: Shape, scale: Vec2, num_subdivisions: u32) -> Option<Shape> {
        match self {
            Shape::Ball(s) => Some(either_shape(s.scaled(scale, num_subdivisions)?)),
            Shape::Cuboid(s) => Some(s.scaled(scale).into()),
            Shape::Capsule(s) => Some(either_shape(s.scaled(scale, num_subdivisions)?)),
            Shape::Segment(s) => Some(s.scaled(scale).into()),
            Shape::HalfSpace(s) => Some(s.scaled(scale)?.into()),
            Shape::ConvexPolygon(s) => Some(s.unbox().scaled(scale)?.into()),
            Shape::Triangle(s) => Some(s.unbox().scaled(scale).into()),
            Shape::RoundCuboid(s) => Some(
                RoundShape {
                    inner_shape: s.inner_shape.scaled(scale), border_radius: s.border_radius,
                }
                    .into(),
            ),
            Shape::RoundTriangle(s) => {
                let s = s.unbox();
                Some(
                    RoundShape {
                        inner_shape: s.inner_shape.scaled(scale), border_radius: s.border_radius,
                    }
                        .into(),
                )
            },
            Shape::RoundConvexPolygon(s) => {
                let s = s.unbox();
                let round: ConvexPolygon = s.inner_shape.scaled(scale)?;
                Some(RoundShape { inner_shape: round, border_radius: s.border_radius }.into())
            },
            Shape::Polyline(s) => Some(s.unbox().scaled(scale).into()),
            Shape::HeightField(s) => Some(s.unbox().scaled(scale).into()),
            Shape::Compound(s) => {
                let mut parts: Array<(Pose2, Shape)> = array![];
                for part in s.unbox().shapes() {
                    let (pose, shape) = *part;
                    let shape = shape.scale_dyn(scale, num_subdivisions)?;
                    parts
                        .append(
                            (
                                Pose2 {
                                    translation: pose.translation * scale, rotation: pose.rotation,
                                },
                                shape,
                            ),
                        );
                }
                Some(CompoundTrait::new(parts.span()).into())
            },
        }
    }

    /// The thickness under which the shape may tunnel (upstream `ccd_thickness`):
    /// `query::nonlinear_shape_cast::ccd_thickness`.
    #[inline(always)]
    fn ccd_thickness(self: Shape) -> Fixed {
        ccd_thickness(self)
    }

    /// The smallest rotation after which the shape may touch with a new contact (upstream
    /// `ccd_angular_thickness`): `query::nonlinear_shape_cast::ccd_angular_thickness`.
    #[inline(always)]
    fn ccd_angular_thickness(self: Shape) -> Fixed {
        ccd_angular_thickness(self)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE};
    use glam_core::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::IDENTITY;
    use rapier_testing::opaque;
    use crate::query::nonlinear_shape_cast::{ccd_angular_thickness, ccd_thickness};
    use super::ShapeDynTrait;
    use super::super::scaled::Either;
    use super::super::{
        Ball, BallTrait, Compound, CompoundTrait, ConvexPolygonTrait, Cuboid, CuboidTrait,
        HalfSpaceTrait, PolylineTrait, RoundShape, SegmentTrait, Shape, ShapeTrait, ShapeType,
        TriangleTrait,
    };

    fn int(x: i32) -> Fixed {
        FixedTrait::from_int(x)
    }

    fn v(x: i32, y: i32) -> Vec2 {
        Vec2 { x: int(x), y: int(y) }
    }

    fn square() -> Shape {
        ShapeDynTrait::convex_polyline_unmodified(array![v(0, 0), v(1, 0), v(1, 1), v(0, 1)].span())
            .unwrap()
    }

    #[test]
    fn test_identity_and_copies() {
        let shapes = array![
            Shape::Ball(BallTrait::new(ONE)), Shape::Cuboid(CuboidTrait::new(v(1, 2))), square(),
        ];
        for shape in shapes.span() {
            let shape = *shape;
            assert_eq!(ShapeDynTrait::new(shape), shape);
            assert_eq!(shape.as_shape(), shape);
            assert_eq!(shape.clone_box(), shape);
            assert_eq!(shape.clone_dyn(), shape);
            assert_eq!(shape.ccd_thickness(), ccd_thickness(shape));
            assert_eq!(shape.ccd_angular_thickness(), ccd_angular_thickness(shape));
        }
    }

    /// `scale_dyn` is the per-shape `scaled` of every kind (uniform and non-uniform scales).
    #[test]
    fn test_scale_dyn_matches_the_scaled_of_each_kind() {
        let s = v(2, 3);
        let u = v(2, 2);
        let ball = BallTrait::new(ONE);
        assert_eq!(Shape::Ball(ball).scale_dyn(u, 8), Some(Shape::Ball(Ball { radius: int(2) })));
        match ball.scaled(s, 8).unwrap() {
            Either::Right(polygon) => assert_eq!(
                Shape::Ball(ball).scale_dyn(s, 8),
                Some(Shape::ConvexPolygon(BoxTrait::new(polygon))),
            ),
            Either::Left(_) => panic!("a non-uniform ball scales to a polygon"),
        }
        let cuboid = CuboidTrait::new(v(1, 2));
        let scaled_cuboid: Cuboid = cuboid.scaled(s);
        assert_eq!(Shape::Cuboid(cuboid).scale_dyn(s, 8), Some(Shape::Cuboid(scaled_cuboid)));
        let segment = SegmentTrait::new(v(0, 0), v(1, 1));
        assert_eq!(
            Shape::Segment(segment).scale_dyn(s, 8), Some(Shape::Segment(segment.scaled(s))),
        );
        let triangle = TriangleTrait::new(v(0, 0), v(1, 0), v(0, 1));
        assert_eq!(
            Shape::Triangle(BoxTrait::new(triangle)).scale_dyn(s, 8),
            Some(Shape::Triangle(BoxTrait::new(triangle.scaled(s)))),
        );
        let round = RoundShape { inner_shape: cuboid, border_radius: HALF };
        let scaled_round = round.inner_shape.scaled(s);
        match Shape::RoundCuboid(round).scale_dyn(s, 8).unwrap() {
            Shape::RoundCuboid(r) => {
                assert_eq!(r.inner_shape, scaled_round);
                assert_eq!(r.border_radius, HALF);
            },
            _ => panic!("a round cuboid scales to a round cuboid"),
        }
        let polyline = PolylineTrait::new(array![v(0, 0), v(1, 0), v(1, 1)].span(), None);
        assert_eq!(
            Shape::Polyline(BoxTrait::new(polyline)).scale_dyn(s, 8),
            Some(Shape::Polyline(BoxTrait::new(polyline.scaled(s)))),
        );
        // A zero scale flattens a half-space normal to nothing.
        let half = HalfSpaceTrait::new(v(0, 1));
        assert_eq!(Shape::HalfSpace(half).scale_dyn(v(1, 0), 8), None);
    }

    #[test]
    fn test_compound_scales_its_parts_and_translations() {
        let part = Shape::Cuboid(CuboidTrait::new(v(1, 1)));
        let pose: Pose2 = Pose2Trait::new(v(1, 2), IDENTITY);
        let compound: Compound = CompoundTrait::new(array![(pose, part)].span());
        let scaled = Shape::Compound(BoxTrait::new(compound)).scale_dyn(v(2, 3), 8).unwrap();
        let Shape::Compound(scaled) = scaled else {
            panic!("a compound scales to a compound");
        };
        let (moved, moved_part) = scaled.unbox().part(0);
        assert_eq!(moved.translation, v(2, 6));
        assert_eq!(moved_part, part.scale_dyn(v(2, 3), 8).unwrap());
    }

    /// Every vertex is kept, collinear ones too (the validating constructor rejects them); the
    /// same polygon as `from_convex_polyline` when the input is strictly convex.
    #[test]
    fn test_convex_polyline_unmodified_keeps_every_vertex() {
        let strict = array![v(0, 0), v(1, 0), v(1, 1), v(0, 1)].span();
        assert_eq!(
            ConvexPolygonTrait::from_convex_polyline_unmodified(strict),
            ConvexPolygonTrait::from_convex_polyline(strict),
        );
        assert!(square().shape_type() == ShapeType::ConvexPolygon);
        let collinear = array![v(0, 0), v(1, 0), v(2, 0), v(2, 2), v(0, 2)].span();
        assert!(ConvexPolygonTrait::from_convex_polyline(collinear).is_none());
        let polygon = ConvexPolygonTrait::from_convex_polyline_unmodified(collinear).unwrap();
        assert_eq!(polygon.count, 5);
        // Fewer than three points, a duplicate, a clockwise outline, too many points.
        assert!(
            ShapeDynTrait::convex_polyline_unmodified(array![v(0, 0), v(1, 0)].span()).is_none(),
        );
        let duplicate = array![v(0, 0), v(0, 0), v(1, 0), v(0, 1)].span();
        assert!(ConvexPolygonTrait::from_convex_polyline_unmodified(duplicate).is_none());
        let clockwise = array![v(0, 0), v(0, 1), v(1, 1), v(1, 0)].span();
        assert!(ConvexPolygonTrait::from_convex_polyline_unmodified(clockwise).is_none());
        let nine = array![
            v(0, 0), v(1, 0), v(2, 0), v(3, 0), v(4, 0), v(5, 0), v(6, 0), v(7, 0), v(7, 1),
        ];
        assert!(ConvexPolygonTrait::from_convex_polyline_unmodified(nine.span()).is_none());
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }

    #[test]
    fn gas_identity_methods() {
        let shape = opaque(Shape::Ball(BallTrait::new(ONE)));
        let _ = shape.as_shape().clone_box().clone_dyn();
    }

    #[test]
    fn gas_ccd_methods() {
        let shape = opaque(Shape::Cuboid(CuboidTrait::new(v(1, 2))));
        let _ = shape.ccd_thickness();
        let _ = shape.ccd_angular_thickness();
    }

    #[test]
    fn gas_scale_dyn_cuboid() {
        let shape = opaque(Shape::Cuboid(CuboidTrait::new(v(1, 2))));
        let _ = shape.scale_dyn(opaque(v(2, 3)), 8);
    }

    #[test]
    fn gas_scale_dyn_ball_polygon() {
        let shape = opaque(Shape::Ball(BallTrait::new(ONE)));
        let _ = shape.scale_dyn(opaque(v(2, 3)), 8);
    }

    #[test]
    fn gas_convex_polyline_unmodified() {
        let _ = ShapeDynTrait::convex_polyline_unmodified(
            opaque(array![v(0, 0), v(1, 0), v(1, 1), v(0, 1)].span()),
        );
    }
}
