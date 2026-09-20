//! Plain data carriers of the golden fixtures.
//!
//! Every `i64` is a raw Q32.32 number (`value = raw / 2^32`). Inputs are exact; expected values
//! are the upstream `f64` rounded to the nearest raw (ties away from zero).

/// A 2D vector.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Vec2Raw {
    pub x: i64,
    pub y: i64,
}

/// A rotation as the complex number `re + i·im`.
///
/// Inputs are handed to upstream as is: apart from multiples of 90° their norm is `1 ± 2^-32`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RotRaw {
    pub re: i64,
    pub im: i64,
}

/// A rigid transformation: rotation first, then translation.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct PoseRaw {
    pub translation: Vec2Raw,
    pub rotation: RotRaw,
}

/// A capsule: the segment `a`–`b` dilated by `radius`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct CapsuleRaw {
    pub a: Vec2Raw,
    pub b: Vec2Raw,
    pub radius: i64,
}

/// A segment from `a` to `b`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SegmentRaw {
    pub a: Vec2Raw,
    pub b: Vec2Raw,
}

/// The shapes of the MVP matrix, in their local frame.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum ShapeRaw {
    /// Radius.
    Ball: i64,
    /// Half extents.
    Cuboid: Vec2Raw,
    Capsule: CapsuleRaw,
    /// Outward unit normal; the boundary passes through the local origin.
    HalfSpace: Vec2Raw,
    Segment: SegmentRaw,
}

/// `SpringCoefficients` and its step-independent angular frequency (`2π · natural_frequency`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SpringDefaultsRaw {
    pub natural_frequency: i64,
    pub damping_ratio: i64,
    pub angular_frequency: i64,
}

/// Spring quantities evaluated by upstream at the **substep** length.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SpringDerivedRaw {
    pub erp_inv_dt: i64,
    pub erp: i64,
    pub cfm_coeff: i64,
    pub cfm_factor: i64,
}

/// `IntegrationParameters::default()` (plus the default joint softness).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct IntegrationDefaultsRaw {
    pub dt: i64,
    pub min_ccd_dt: i64,
    pub contact_softness: SpringDefaultsRaw,
    pub static_contact_softness: SpringDefaultsRaw,
    pub joint_softness: SpringDefaultsRaw,
    pub warmstart_coefficient: i64,
    pub length_unit: i64,
    pub normalized_allowed_linear_error: i64,
    pub normalized_max_corrective_velocity: i64,
    pub normalized_prediction_distance: i64,
    pub normalized_max_linear_velocity: i64,
    pub normalized_contact_recycle_distance: i64,
    pub num_solver_iterations: u32,
    pub num_internal_pgs_iterations: u32,
    pub num_internal_stabilization_iterations: u32,
    pub max_ccd_substeps: u32,
    pub contact_clustering: bool,
    pub contact_recycling: bool,
    pub friction_in_bias_pass: bool,
    pub warmstart_joints: bool,
}

/// Quantities the solver derives from the defaults for one step length `dt`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct IntegrationDerivedRaw {
    pub id: felt252,
    pub dt: i64,
    pub num_solver_iterations: u32,
    pub inv_dt: i64,
    /// `dt / num_solver_iterations`.
    pub substep_dt: i64,
    pub substep_inv_dt: i64,
    pub allowed_linear_error: i64,
    pub max_corrective_velocity: i64,
    pub prediction_distance: i64,
    pub max_linear_velocity: i64,
    pub contact_recycle_distance: i64,
    /// Contacts between two dynamic bodies.
    pub contact: SpringDerivedRaw,
    /// Contacts involving a fixed body.
    pub static_contact: SpringDerivedRaw,
    pub joint: SpringDerivedRaw,
}

/// Parry's `MassProperties` in 2D (the angular inertia is a scalar).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct MassPropertiesRaw {
    pub mass: i64,
    pub inv_mass: i64,
    pub local_com: Vec2Raw,
    pub principal_inertia: i64,
    pub inv_principal_inertia: i64,
}

/// `shape.mass_properties(density)`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ShapeMassCase {
    pub id: felt252,
    pub shape: ShapeRaw,
    pub density: i64,
    pub expected: MassPropertiesRaw,
}

/// A collider attached to a body of a [`CompoundMassCase`].
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct WeightedColliderRaw {
    pub shape: ShapeRaw,
    pub pose_wrt_parent: PoseRaw,
    pub density: i64,
}

/// Mass properties of a dynamic body carrying two colliders.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct CompoundMassCase {
    pub id: felt252,
    pub body_pose: PoseRaw,
    pub colliders: [WeightedColliderRaw; 2],
    /// Local mass properties of the body.
    pub expected: MassPropertiesRaw,
    pub world_com: Vec2Raw,
    pub effective_inv_mass: Vec2Raw,
    pub effective_world_inv_inertia: i64,
}

/// `shape.compute_aabb(pose)`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct AabbCase {
    pub id: felt252,
    pub shape: ShapeRaw,
    pub pose: PoseRaw,
    pub mins: Vec2Raw,
    pub maxs: Vec2Raw,
}

/// Parry's `TrackedContact`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ContactPointRaw {
    /// Contact point on shape 1, in the local frame of shape 1.
    pub local_p1: Vec2Raw,
    /// Contact point on shape 2, in the local frame of shape 2.
    pub local_p2: Vec2Raw,
    /// Signed distance along the normal; negative when penetrating.
    pub dist: i64,
    /// `PackedFeatureId` bits: `0b01 << 30 | code` vertex, `0b11 << 30 | code` face, `0` unknown.
    pub fid1: u32,
    pub fid2: u32,
}

/// `DefaultQueryDispatcher::contact_manifolds(pos12, shape1, shape2, PREDICTION, ..)`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ManifoldCase {
    pub id: felt252,
    pub shape1: ShapeRaw,
    pub shape2: ShapeRaw,
    /// Pose of shape 2 in the local frame of shape 1.
    pub pos12: PoseRaw,
    /// Discrete outputs hinge on an exact tie or a fallback branch upstream; compare loosely.
    pub ambiguous: bool,
    /// Number of meaningful entries of `points` (0, 1 or 2). Unused entries are zeroed.
    pub num_points: u32,
    /// Contact normal in the local frame of shape 1, pointing from 1 towards 2. Meaningless
    /// when `num_points == 0`.
    pub local_n1: Vec2Raw,
    /// Contact normal in the local frame of shape 2, pointing from 2 towards 1.
    pub local_n2: Vec2Raw,
    pub points: [ContactPointRaw; 2],
}

/// Whether a scene body is simulated.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum BodyKindRaw {
    Fixed,
    Dynamic,
}

/// A collider of a [`SceneBodyRaw`].
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SceneColliderRaw {
    pub shape: ShapeRaw,
    pub pose_wrt_parent: PoseRaw,
    pub density: i64,
    pub friction: i64,
    pub restitution: i64,
}

/// A rigid body of a scene, as built upstream (sleeping and CCD are off for every body).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SceneBodyRaw {
    /// Name used by upstream; sampled states refer to a body by its index instead.
    pub name: felt252,
    pub kind: BodyKindRaw,
    /// Initial pose.
    pub pose: PoseRaw,
    pub linear_damping: i64,
    pub angular_damping: i64,
    pub gravity_scale: i64,
    /// Number of meaningful entries of `colliders` (0 or 1). Unused entries are zeroed.
    pub num_colliders: u32,
    pub colliders: [SceneColliderRaw; 1],
}

/// A revolute impulse joint between two bodies of a scene.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RevoluteJointRaw {
    /// Index into `SceneCase::bodies`.
    pub body1: u32,
    /// Index into `SceneCase::bodies`.
    pub body2: u32,
    /// Anchor in the local frame of body 1.
    pub local_anchor1: Vec2Raw,
    /// Anchor in the local frame of body 2.
    pub local_anchor2: Vec2Raw,
}

/// State of one dynamic body after a step, read through the upstream accessors.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct BodyStateRaw {
    /// Index into `SceneCase::bodies`.
    pub body: u32,
    pub translation: Vec2Raw,
    pub rotation: RotRaw,
    pub linvel: Vec2Raw,
    pub angvel: i64,
}

/// The dynamic bodies of a scene after `step` calls of `PhysicsPipeline::step`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SceneSampleRaw {
    /// Number of steps taken so far; 0 is the initial state.
    pub step: u32,
    /// One entry per dynamic body, in body order; only the first `SceneCase::num_dynamic` count.
    /// Unused entries are zeroed. Fixed bodies never move and are not sampled.
    pub states: [BodyStateRaw; 3],
}

/// A full-engine trace: the description of a scene and the sampled states of its dynamic bodies.
///
/// Every scene uses the same gravity, step length and sampling schedule (step 0, steps 1–10, then
/// every 10th step up to `num_steps`).
///
/// Only `Copy` and `Drop` are derived: Cairo 2.19.4 has no `Serde`, `PartialEq` or `Debug`
/// implementation for a fixed-size array as long as `samples` (22 entries).
#[derive(Copy, Drop)]
pub struct SceneCase {
    pub id: felt252,
    pub gravity: Vec2Raw,
    /// Step length of every `PhysicsPipeline::step`.
    pub dt: i64,
    /// Steps simulated; the last sample is taken after this many.
    pub num_steps: u32,
    /// Number of meaningful entries of `bodies` (at most 4), in insertion order. Unused entries
    /// are zeroed.
    pub num_bodies: u32,
    pub bodies: [SceneBodyRaw; 4],
    /// Number of dynamic bodies, i.e. of meaningful entries of every sample.
    pub num_dynamic: u32,
    /// Number of meaningful entries of `joints` (0 or 1). Unused entries are zeroed.
    pub num_joints: u32,
    pub joints: [RevoluteJointRaw; 1],
    pub samples: [SceneSampleRaw; 22],
}
