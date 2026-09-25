//! 2D collision geometry for rapier-cairo: the subset of Parry that Rapier's step consumes.
//!
//! The types shared with `rapier_dynamics2d` are frozen in `docs/interfaces/geometry-dynamics.md`
//! and live in [`contact`], [`feature_id`] and [`mass`]; the algorithms (shapes, AABB, SAT,
//! clipping, manifold generators, broad phase) are wave-3 work packages.

pub mod aabb;
pub mod broad_phase;
pub mod clip;
pub mod closest_points;
pub mod contact;
pub mod contact_generators;
pub mod dispatch;
pub mod feature_id;
pub mod manifold;
pub mod mass;
pub mod point;
pub mod polygonal_feature;
pub mod ray;
pub mod sat;
pub mod shape;
pub use closest_points::{
    closest_points_segment_segment, closest_points_segment_segment_with_locations,
};

pub use dispatch::{contact_manifold, intersection_test};
pub use mass::MassPropertiesTrait;
pub use point::{PointProjection, SegmentPointLocation};
pub use shape::convex_polygon::{ConvexPolygon, ConvexPolygonTrait};
pub use shape::{Shape, ShapeTrait, ShapeType};
