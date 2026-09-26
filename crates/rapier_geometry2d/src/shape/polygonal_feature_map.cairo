//! The polygonal-feature-map trait (Parry `shape/polygonal_feature_map.rs`): the face most
//! aligned with a direction, for the shapes whose features are polygonal.
//!
//! Upstream fills an `&mut PolygonalFeature`; here the feature is returned. Implemented for
//! `Segment`, `Cuboid` and `ConvexPolygon` (as upstream), and for the [`Shape`] view returned by
//! `ShapeTrait::as_polygonal_feature_map`.

use glam::Vec2;
use crate::polygonal_feature::PolygonalFeature;
use crate::shape::{ConvexPolygon, ConvexPolygonTrait, Cuboid, CuboidTrait, Segment, Shape};

pub mod errors {
    /// Ball, capsule and half-space have no polygonal feature map (upstream: not implemented).
    pub const NOT_FEATURE_MAP: felt252 = 'Shape: not a feature map';
}

/// A shape that can return its support face (upstream `PolygonalFeatureMap`; the 3D-only
/// `is_convex_polyhedron` is omitted).
pub trait PolygonalFeatureMap<T> {
    /// The feature of the shape most aligned with the local direction `dir`.
    fn local_support_feature(self: T, dir: Vec2) -> PolygonalFeature;
    /// Same, with a `hint` direction the 2D shapes ignore (upstream default).
    fn local_support_feature_toward(
        self: T, dir: Vec2, hint: Vec2,
    ) -> PolygonalFeature {
        Self::local_support_feature(self, dir)
    }
}

/// The segment itself, whatever `dir` (upstream `PolygonalFeature::from(segment)`).
pub impl SegmentPolygonalFeatureMap of PolygonalFeatureMap<Segment> {
    #[inline(always)]
    fn local_support_feature(self: Segment, dir: Vec2) -> PolygonalFeature {
        self.into()
    }
}

/// `CuboidTrait::support_face`.
pub impl CuboidPolygonalFeatureMap of PolygonalFeatureMap<Cuboid> {
    #[inline(always)]
    fn local_support_feature(self: Cuboid, dir: Vec2) -> PolygonalFeature {
        CuboidTrait::support_face(self, dir)
    }
}

/// `ConvexPolygonTrait::support_feature` (vertex ids `2 i`, face ids `2 i + 1`).
pub impl ConvexPolygonPolygonalFeatureMap of PolygonalFeatureMap<ConvexPolygon> {
    #[inline(always)]
    fn local_support_feature(self: ConvexPolygon, dir: Vec2) -> PolygonalFeature {
        ConvexPolygonTrait::support_feature(self, dir)
    }
}

/// Dispatch over the closed set.
/// #### Panics
/// * `'Shape: not a feature map'` for a ball, a capsule (use the segment returned by
///   `as_polygonal_feature_map`) or a half-space.
pub impl ShapePolygonalFeatureMap of PolygonalFeatureMap<Shape> {
    fn local_support_feature(self: Shape, dir: Vec2) -> PolygonalFeature {
        match self {
            Shape::Cuboid(s) => CuboidTrait::support_face(s, dir),
            Shape::Segment(s) => s.into(),
            Shape::ConvexPolygon(s) => ConvexPolygonTrait::support_feature(s.unbox(), dir),
            _ => core::panic_with_felt252(errors::NOT_FEATURE_MAP),
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_testing::opaque;
    use crate::polygonal_feature::PolygonalFeature;
    use crate::shape::{BallTrait, ConvexPolygonTrait, CuboidTrait, SegmentTrait, Shape, ShapeTrait};
    use super::PolygonalFeatureMap;

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    #[test]
    fn test_features_match_the_inherent_methods() {
        let dirs = array![v(ONE, ZERO), v(ZERO, -ONE), v(-ONE, ONE)];
        let cuboid = CuboidTrait::new(v(TWO, ONE));
        let segment = SegmentTrait::new(v(-ONE, ZERO), v(ONE, ONE));
        let square = ConvexPolygonTrait::from_convex_polyline(
            array![v(-ONE, -ONE), v(ONE, -ONE), v(ONE, ONE), v(-ONE, ONE)].span(),
        )
            .unwrap();
        let seg_feature: PolygonalFeature = segment.into();
        for dir in dirs.span() {
            let d = *dir;
            assert_eq!(
                PolygonalFeatureMap::local_support_feature(cuboid, d), cuboid.support_face(d),
            );
            assert_eq!(
                PolygonalFeatureMap::local_support_feature_toward(cuboid, d, -d),
                cuboid.support_feature(d),
            );
            assert_eq!(PolygonalFeatureMap::local_support_feature(segment, d), seg_feature);
            assert_eq!(
                PolygonalFeatureMap::local_support_feature(square, d), square.support_feature(d),
            );
            assert_eq!(
                PolygonalFeatureMap::local_support_feature(Shape::Cuboid(cuboid), d),
                cuboid.support_face(d),
            );
            assert_eq!(
                PolygonalFeatureMap::local_support_feature(
                    Shape::ConvexPolygon(BoxTrait::new(square)), d,
                ),
                square.support_feature(d),
            );
        }
        // The capsule view is its core segment.
        let capsule = Shape::Capsule(
            crate::shape::CapsuleTrait::new(v(-ONE, ZERO), v(ONE, ONE), ONE),
        );
        let (core, radius) = capsule.as_polygonal_feature_map().unwrap();
        assert_eq!(radius, ONE);
        assert_eq!(PolygonalFeatureMap::local_support_feature(core, v(ONE, ZERO)), seg_feature);
    }

    #[test]
    #[should_panic(expected: 'Shape: not a feature map')]
    fn test_ball_is_not_a_feature_map() {
        let _ = PolygonalFeatureMap::local_support_feature(
            Shape::Ball(BallTrait::new(ONE)), v(ONE, ZERO),
        );
    }

    #[test]
    fn gas_baseline() {}
    #[test]
    fn gas_local_support_feature_cuboid() {
        let _ = PolygonalFeatureMap::local_support_feature(
            opaque(CuboidTrait::new(v(TWO, ONE))), opaque(v(ONE, ZERO)),
        );
    }
    #[test]
    fn gas_local_support_feature_segment() {
        let _ = PolygonalFeatureMap::local_support_feature(
            opaque(SegmentTrait::new(v(-ONE, ZERO), v(ONE, ONE))), opaque(v(ONE, ZERO)),
        );
    }
}
