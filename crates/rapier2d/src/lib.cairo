//! Public 2D facade of rapier-cairo (wave 5, `docs/PLAN.md`): the `World` bundle, the step
//! pipeline and the dispatcher glue between `rapier_geometry2d` and `rapier_dynamics2d`.
//! Modules are pre-declared per work package (see `docs/briefs/`).

pub mod control;
pub mod dispatcher;
pub mod pipeline;
pub mod queries;
pub mod world;

/// Everything a game needs to build and step a world (requested by P1).
pub mod prelude {
    pub use fixed::Fixed;
    pub use glam_core::Vec2;
    pub use rapier_core::collider::events::{
        ActiveEvents, COLLISION_EVENTS, CONTACT_FORCE_EVENTS, CollisionEventFlags, REMOVED, SENSOR,
    };
    pub use rapier_core::data::handle::Handle;
    pub use rapier_core::integration_parameters::IntegrationParameters;
    pub use rapier_core::rigid_body::{RigidBodyType, RigidBodyTypeTrait};
    pub use rapier_dynamics2d::collider::builder::{ColliderBuilder, ColliderBuilderTrait};
    pub use rapier_dynamics2d::collider::{Collider, ColliderTrait};
    pub use rapier_dynamics2d::collider_set::access::{
        ColliderSetChangesImpl, ColliderSetChangesTrait, ColliderSetIndexImpl, ModifiedColliders,
    };
    pub use rapier_dynamics2d::joint::{
        FixedJoint, FixedJointBuilder, FixedJointBuilderIntoGeneric, FixedJointBuilderTrait,
        FixedJointIntoGeneric, FixedJointTrait, GenericJoint, GenericJointBuilder,
        GenericJointBuilderIntoGeneric, GenericJointBuilderTrait, GenericJointTrait, ImpulseJoint,
        ImpulseJointSet, ImpulseJointSetTrait, ImpulseJointTrait, JointAxesMask, JointAxis,
        JointAxisTrait, JointLimits, JointMotor, JointUserData, JointUserDataImpl,
        JointUserDataTrait, LIN_AXES, MotorModel, MotorModelTrait, PinSlotJoint,
        PinSlotJointBuilder, PinSlotJointBuilderIntoGeneric, PinSlotJointBuilderTrait,
        PinSlotJointIntoGeneric, PinSlotJointTrait, PrismaticJoint, PrismaticJointBuilder,
        PrismaticJointBuilderIntoGeneric, PrismaticJointBuilderTrait, PrismaticJointIntoGeneric,
        PrismaticJointTrait, RevoluteJoint, RevoluteJointBuilder, RevoluteJointBuilderIntoGeneric,
        RevoluteJointBuilderTrait, RevoluteJointIntoGeneric, RevoluteJointTrait, RopeJoint,
        RopeJointBuilder, RopeJointBuilderIntoGeneric, RopeJointBuilderTrait, RopeJointIntoGeneric,
        RopeJointTrait, SpringJoint, SpringJointBuilder, SpringJointBuilderIntoGeneric,
        SpringJointBuilderTrait, SpringJointIntoGeneric, SpringJointTrait,
    };
    pub use rapier_dynamics2d::narrow_phase::contact_pairs::{
        ContactId, ContactPairView, ContactPairViewTrait, NarrowPhaseContactPairsTrait,
        PairContacts, RigidPairContacts,
    };
    pub use rapier_dynamics2d::narrow_phase::interaction_graph::{
        ColliderGraphIndex, InteractionEdge, InteractionGraph, InteractionGraphTrait,
        IntersectionPair, NarrowPhaseInteractionGraphTrait, TemporaryInteractionIndex,
    };
    pub use rapier_dynamics2d::rigid_body::ccd::{RigidBodyCcd, RigidBodyCcdTrait};
    pub use rapier_dynamics2d::rigid_body_set::index::{RigidBodyIds, RigidBodySetIndexImpl};
    pub use rapier_dynamics2d::rigid_body_set::{
        BodyAngvel, BodyLinvel, BodyPose, BodySleeping, RigidBodyCcdApiTrait,
    };
    pub use rapier_dynamics2d::{
        ColliderSet, ColliderSetTrait, CollisionEvent, CollisionEventTrait, ContactForceEvent,
        ContactForceEventTrait, ContactPair, ContactPairTrait, OneWayPlatform, RigidBody,
        RigidBodyBuilder, RigidBodyBuilderTrait, RigidBodySet, RigidBodySetTrait, RigidBodyTrait,
    };
    pub use rapier_geometry2d::query::normal_constraints::{
        LocalNormalProjector, NormalConstraints, NormalConstraintsPair,
    };
    pub use rapier_geometry2d::query::shape_distance::{ShapeDistance, ShapeDistanceTrait};
    pub use rapier_geometry2d::query::subshape::{
        SubshapeContact, SubshapePointProjection, SubshapeRayIntersection,
    };
    pub use rapier_geometry2d::query::{
        NonlinearRigidMotion, NonlinearRigidMotionTrait, ShapeCastHit, ShapeCastHitTrait,
        ShapeCastOptions, ShapeCastOptionsTrait, ShapeCastStatus,
    };
    pub use rapier_geometry2d::ray::{Ray, RayIntersection, RayTrait};
    pub use rapier_geometry2d::shape::compound::{
        Compound, CompoundEdgeCone, CompoundFlags, CompoundFlagsTrait, CompoundPseudoNormals,
        CompoundTrait, FIX_INTERNAL_EDGES,
    };
    pub use rapier_geometry2d::shape::dyn_api::{ShapeDynImpl, ShapeDynTrait};
    pub use rapier_geometry2d::shape::heightfield::{
        HeightField, HeightFieldCellStatus, HeightFieldTrait,
    };
    pub use rapier_geometry2d::shape::polyline::{
        Polyline, PolylineFlags, PolylineFlagsTrait, PolylineTrait,
    };
    pub use rapier_geometry2d::shape::round_shape::{
        RoundConvexPolygon, RoundCuboid, RoundShape, RoundShapeTrait, RoundTriangle,
    };
    pub use rapier_geometry2d::shape::segment::SegmentPseudoNormals;
    pub use rapier_geometry2d::shape::triangle::{Triangle, TriangleTrait};
    pub use rapier_geometry2d::shape::triangle_pseudo_normals::TrianglePseudoNormals;
    pub use rapier_geometry2d::shape::{
        Ball, BallTrait, Capsule, CapsuleTrait, ConvexPolygon, ConvexPolygonTrait, Cuboid,
        CuboidTrait, HalfSpace, HalfSpaceTrait, Segment, SegmentTrait, Shape, ShapeTrait,
    };
    pub use rapier_math::pose2::{Pose2, Pose2Trait};
    pub use rapier_math::rot2::{Rot2, Rot2Trait};
    pub use crate::control::{
        CharacterAutostep, CharacterCollision, CharacterLength, CharacterLengthTrait,
        EffectiveCharacterMovement, KinematicCharacterController, KinematicCharacterControllerTrait,
        PdController, PdControllerTrait, PdErrors, PidController, PidControllerTrait,
    };
    pub use crate::dispatcher::DefaultDispatcher;
    pub use crate::pipeline::ccd::{CCDSolver, CCDSolverTrait};
    pub use crate::pipeline::config::{
        BasicStepConfig, ConstrainedCompositeManifolds, DefaultStepConfig, StepConfig,
    };
    pub use crate::pipeline::facade::{
        CollisionPipeline, CollisionPipelineTrait, PhysicsPipeline, PhysicsPipelineTrait,
    };
    pub use crate::pipeline::stages::{InProcessStages, StageConfig};
    pub use crate::queries::pipeline::{QueryPipeline, QueryPipelineTrait};
    pub use crate::queries::{
        EXCLUDE_DYNAMIC, EXCLUDE_FIXED, EXCLUDE_KINEMATIC, EXCLUDE_SENSORS, EXCLUDE_SOLIDS,
        ONLY_DYNAMIC, ONLY_FIXED, ONLY_KINEMATIC, QueryFilter, QueryFilterFlags,
        QueryFilterFlagsTrait, QueryFilterTrait, QueryPipelineMut, QueryPipelineMutTrait,
    };
    pub use crate::world::basic_state::{BasicWorldState, from_basic_state, into_basic_state};
    pub use crate::world::state::{WORLD_STATE_VERSION, WorldState};
    pub use crate::world::{PhysicsWorld, World, WorldTrait};
}
