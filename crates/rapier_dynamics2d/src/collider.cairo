//! Vector-valued components of a collider and the [`ColliderBuilder`]: the port of the part of
//! upstream's `geometry/collider_components.rs` and `geometry/collider.rs` that holds a shape, a
//! pose or a mass property.
//!
//! The scalar components (type, material, filtering flags, change flags) are dimension-agnostic
//! and live in [`rapier_core::collider`]; the closed shape enum and the mass properties come from
//! `rapier_geometry2d`. This module adds:
//!
//! * [`components`]: `ColliderParent`, `ColliderPosition` and `ColliderMassProps` (which of
//!   density, mass or explicit mass properties defines the collider's mass);
//! * [`object`]: the `Collider` itself and its getters and setters, which raise the same
//!   `ColliderChanges` bits as upstream;
//! * [`builder`]: `ColliderBuilder` with upstream's defaults (density 1, friction 0.5,
//!   restitution 0, solid, enabled).
//!
//! A collider is built with no parent: attaching it to a body is the job of the collider set
//! (DD), which owns `ColliderParent` and the world pose `ColliderPosition` from then on.
//!
//! Deferred: `ColliderSet` (DD), the CCD entry points (`compute_swept_aabb`,
//! `compute_broad_phase_aabb`), the contact skin and `compute_collision_aabb`, `shape_mut` and
//! `copy_from`, and the deformable-mesh reference (soft bodies are out of scope).

pub mod builder;
pub mod components;
pub mod object;

pub use builder::{
    ColliderBuilder, ColliderBuilderDefault, ColliderBuilderImpl, ColliderBuilderTrait,
};
pub use components::{
    ColliderMassProps, ColliderMassPropsDefault, ColliderMassPropsImpl, ColliderMassPropsTrait,
    ColliderParent, ColliderPosition, ColliderPositionDefault,
};
pub use object::{Collider, ColliderImpl, ColliderTrait};
