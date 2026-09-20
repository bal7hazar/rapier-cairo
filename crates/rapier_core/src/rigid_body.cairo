//! Scalar-only components of a rigid-body (upstream `dynamics/rigid_body_components.rs`): the
//! body type, the change flags, damping, dominance and sleeping state.
//!
//! Everything holding a vector or a pose (position, velocities, forces, mass properties) is
//! left to the crates that own `Vec2` / `Pose`. Cut with the soft bodies: the `SoftFrame` body
//! type. `RigidBodyAdditionalMassProps` wraps `MassProperties` and is not ported here.

pub mod activation;
pub mod body_type;
pub mod changes;
pub mod damping;
pub mod dominance;

pub use activation::{RigidBodyActivation, RigidBodyActivationImpl, RigidBodyActivationTrait};
pub use body_type::{RigidBodyType, RigidBodyTypeImpl, RigidBodyTypeTrait};
pub use changes::{RigidBodyChanges, RigidBodyChangesImpl, RigidBodyChangesTrait};
pub use damping::{RigidBodyDamping, RigidBodyDampingImpl, RigidBodyDampingTrait, damping_factor};
pub use dominance::{RigidBodyDominance, RigidBodyDominanceImpl, RigidBodyDominanceTrait};
