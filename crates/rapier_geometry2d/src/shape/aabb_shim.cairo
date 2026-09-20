//! Deprecated path kept for `rapier_dynamics2d::collider::object` (`use
//! rapier_geometry2d::shape::aabb_shim::Aabb`, landed on `main` after work package GH started).
//! Switch that import to `rapier_geometry2d::aabb::Aabb` and delete this file.

pub use crate::aabb::{Aabb, AabbTrait};
