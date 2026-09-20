//! Dimension-agnostic data structures shared by every rapier.cairo crate.

pub mod collider;
pub mod data;
pub mod integration_parameters;
pub mod interaction_groups;
pub mod rigid_body;

pub use collider::{
    ActiveCollisionTypes, ActiveCollisionTypesDefault, ActiveCollisionTypesImpl,
    ActiveCollisionTypesTrait, ActiveEvents, ActiveEventsImpl, ActiveEventsTrait, ActiveHooks,
    ActiveHooksImpl, ActiveHooksTrait, CoefficientCombineRule, CoefficientCombineRuleImpl,
    CoefficientCombineRuleTrait, ColliderChanges, ColliderChangesImpl, ColliderChangesTrait,
    ColliderEnabled, ColliderFlags, ColliderFlagsDefault, ColliderMaterial, ColliderMaterialDefault,
    ColliderMaterialImpl, ColliderMaterialTrait, ColliderType, ColliderTypeImpl, ColliderTypeTrait,
    CollisionEventFlags, CollisionEventFlagsImpl, CollisionEventFlagsTrait,
};
pub use data::arena::{Arena, ArenaState, ArenaStateTrait, ArenaTrait};
pub use data::handle::{Handle, HandleTrait, INVALID_HANDLE};
pub use data::union_find::{UnionFind, UnionFindTrait};
pub use integration_parameters::{
    IntegrationParameters, IntegrationParametersTrait, SoftnessCoefficients, SpringCoefficients,
    SpringCoefficientsTrait,
};
pub use interaction_groups::{
    Group, GroupTrait, InteractionGroups, InteractionGroupsTrait, InteractionTestMode,
};
pub use rigid_body::{
    RigidBodyActivation, RigidBodyActivationImpl, RigidBodyActivationTrait, RigidBodyChanges,
    RigidBodyChangesImpl, RigidBodyChangesTrait, RigidBodyDamping, RigidBodyDampingImpl,
    RigidBodyDampingTrait, RigidBodyDominance, RigidBodyDominanceImpl, RigidBodyDominanceTrait,
    RigidBodyType, RigidBodyTypeImpl, RigidBodyTypeTrait, damping_factor,
};
