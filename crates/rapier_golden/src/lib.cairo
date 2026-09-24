//! Golden vectors recorded from upstream Rapier / Parry (f64, 2D) by `tools/golden`.
//!
//! Everything is a raw Q32.32 `i64` (`value = raw / 2^32`) so that the fixtures do not depend on
//! any scalar type: consumers wrap the raws into their own fixed-point type.

pub mod compare;
pub mod generated;
pub mod types;

pub use generated::{
    aabb, aabb_overlap, clip2d, contact_manifolds, integration_parameters, mass_properties,
    point_projection, pose2, ray_casts, sat2d, scenes, segment_segment,
};
