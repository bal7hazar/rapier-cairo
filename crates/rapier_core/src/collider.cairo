//! Scalar-only components of a collider (upstream `geometry/collider_components.rs`, plus the
//! event / hook flags of `pipeline/`): type, material, filtering flags and change flags.
//!
//! Everything holding a shape, a pose or a mass property is left to the crates that own them.
//! Cut with the soft bodies: `ColliderChanges::DEFORMED`. The physics-hook flags are plain data,
//! `dyn PhysicsHooks` being cut.

pub mod active_collision_types;
pub mod changes;
pub mod combine_rule;
pub mod events;
pub mod flags;
pub mod hooks;
pub mod material;

pub use active_collision_types::{
    ActiveCollisionTypes, ActiveCollisionTypesDefault, ActiveCollisionTypesImpl,
    ActiveCollisionTypesTrait,
};
pub use changes::{ColliderChanges, ColliderChangesImpl, ColliderChangesTrait};
pub use combine_rule::{
    CoefficientCombineRule, CoefficientCombineRuleImpl, CoefficientCombineRuleTrait,
};
pub use events::{
    ActiveEvents, ActiveEventsImpl, ActiveEventsTrait, CollisionEventFlags, CollisionEventFlagsImpl,
    CollisionEventFlagsTrait,
};
pub use flags::{
    ColliderEnabled, ColliderFlags, ColliderFlagsDefault, ColliderType, ColliderTypeImpl,
    ColliderTypeTrait,
};
pub use hooks::{ActiveHooks, ActiveHooksImpl, ActiveHooksTrait};
pub use material::{
    ColliderMaterial, ColliderMaterialDefault, ColliderMaterialImpl, ColliderMaterialTrait,
};
