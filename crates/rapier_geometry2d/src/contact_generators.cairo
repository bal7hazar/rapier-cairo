//! Per-pair contact manifold generators (Parry's `contact_manifolds_*`), one module per pair.

pub mod ball_ball;
pub mod capsule_capsule;
pub mod convex_ball;
pub mod cuboid_capsule;
pub mod cuboid_cuboid;
pub mod cuboid_segment;
pub mod halfspace_pfm;
pub mod pfm_pfm;
pub mod polygon_polygon;
pub mod polygon_segment;
