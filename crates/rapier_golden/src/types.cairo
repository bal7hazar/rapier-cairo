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

// ---------------------------------------------------------------------------------------------
// Leaf-level families (G2): pose algebra and the narrow-phase building blocks.
// ---------------------------------------------------------------------------------------------

/// A triangle given by its three vertices, counter-clockwise.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct TriangleRaw {
    pub a: Vec2Raw,
    pub b: Vec2Raw,
    pub c: Vec2Raw,
}

/// Pose algebra of upstream's `Pose` / `Rotation` for the pose `a` (and `b`).
///
/// `rot_*` are the rotation-only operations on the rotations of `a` and `b`. Upstream multiplies
/// rotations as plain complex numbers: it never renormalises, so inputs whose norm is `1 ± 2^-32`
/// yield products whose norm error keeps accumulating.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Pose2Case {
    pub id: felt252,
    pub a: PoseRaw,
    pub b: PoseRaw,
    /// Points (and vectors) the poses are applied to.
    pub points: [Vec2Raw; 3],
    /// `a * b`: `b` first, then `a`.
    pub mul: PoseRaw,
    /// `a.inverse()`.
    pub inverse: PoseRaw,
    /// `a.inv_mul(b)`, i.e. `a⁻¹ * b`: the pose of `b` in the frame of `a` (Parry's `pos12`).
    pub inv_mul: PoseRaw,
    /// `a.rotation * b.rotation`.
    pub rot_mul: RotRaw,
    /// `a.rotation.inverse()`: the conjugate, exact in Q32.32.
    pub rot_inverse: RotRaw,
    /// `a.transform_point(p)` for each entry of `points`.
    pub transform_point: [Vec2Raw; 3],
    /// `a.inverse_transform_point(p)`.
    pub inverse_transform_point: [Vec2Raw; 3],
    /// `a.transform_vector(v)`: rotation only.
    pub transform_vector: [Vec2Raw; 3],
    /// `a.inverse_transform_vector(v)`.
    pub inverse_transform_vector: [Vec2Raw; 3],
}

/// A checkpoint of a [`RotChainCase`].
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RotChainSampleRaw {
    /// Number of multiplications performed so far.
    pub steps: u32,
    pub rotation: RotRaw,
    /// `re² + im²` computed in `f64` from the upstream rotation.
    pub norm_squared: i64,
    /// `norm_squared - 1`.
    pub drift: i64,
}

/// `acc = acc * step`, repeated 1 000 times from the identity, in `f64`.
///
/// The drift is dominated by the norm error of `step` itself (`1 ± 2^-32`, amplified linearly),
/// not by `f64` rounding: it is the value a Q32.32 chain has to be compared with.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RotChainCase {
    pub id: felt252,
    pub step: RotRaw,
    /// Checkpoints after 1, 10, 100 and 1 000 multiplications.
    pub samples: [RotChainSampleRaw; 4],
}

/// One AABB of an [`AabbOverlapCase`].
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct OverlapBoxRaw {
    pub mins: Vec2Raw,
    pub maxs: Vec2Raw,
    /// Belongs to a fixed body (upstream's `intersects` ignores this; the broad phase does not
    /// pair two fixed colliders).
    pub is_static: bool,
}

/// An overlapping pair of AABB indices, `i < j`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct OverlapPairRaw {
    pub i: u32,
    pub j: u32,
    /// Both AABBs are static: the broad phase would drop the pair.
    pub both_static: bool,
}

/// The overlapping pairs of a list of AABBs under `BoundingVolume::intersects`.
///
/// The convention is closed: AABBs that merely touch overlap. Coordinates are exact Q32.32, so a
/// port must reproduce `pairs` exactly.
#[derive(Copy, Drop)]
pub struct AabbOverlapCase {
    pub id: felt252,
    /// Number of meaningful entries of `aabbs`. Unused entries are zeroed.
    pub num_aabbs: u32,
    pub aabbs: [OverlapBoxRaw; 32],
    /// Number of meaningful entries of `pairs`. Unused entries are zeroed.
    pub num_pairs: u32,
    /// Every overlapping pair `i < j`, sorted by `i` then `j` (all pairs, static-static included).
    pub pairs: [OverlapPairRaw; 32],
    /// `merged` of every AABB of the list.
    pub merged_mins: Vec2Raw,
    pub merged_maxs: Vec2Raw,
}

/// The shapes SAT helpers are tested on, in their local frame.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum SatOperandRaw {
    /// Half extents.
    Cuboid: Vec2Raw,
    Segment: SegmentRaw,
    Triangle: TriangleRaw,
}

/// `(separation, axis)` as returned by a `*_find_local_separating_normal_oneway` function.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SatAxisRaw {
    /// Positive when the shapes are apart along `axis`, negative when they overlap.
    pub separation: i64,
    /// Unit axis oriented from the tested shape towards the other one, in the local frame of the
    /// tested shape.
    pub axis: Vec2Raw,
}

/// The separating-axis helpers of Parry, in both directions.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SatCase {
    pub id: felt252,
    pub shape1: SatOperandRaw,
    pub shape2: SatOperandRaw,
    /// Pose of shape 2 in the frame of shape 1.
    pub pos12: PoseRaw,
    /// Pose of shape 1 in the frame of shape 2: the inverse of `pos12`, snapped to Q32.32 (the
    /// rotation is exact, the translation is rounded), i.e. the exact input of the second call.
    pub pos21: PoseRaw,
    /// Two axes tie exactly, or the answer hinges on the sign of an exact zero: the discrete
    /// output (axis) may legitimately differ; `separation` is still comparable.
    pub ambiguous: bool,
    /// The normals of shape 1 tested against shape 2, with `pos12`.
    pub sep1: SatAxisRaw,
    /// The normals of shape 2 tested against shape 1, with `pos21`.
    pub sep2: SatAxisRaw,
}

/// A clipping point of `clip_segment_segment*`: `p1` lies on segment 1, `p2` on segment 2.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ClipPointRaw {
    pub p1: Vec2Raw,
    pub p2: Vec2Raw,
    /// Feature of segment 1: 0 = first vertex as passed, 1 = interior, 2 = second vertex.
    pub f1: u32,
    /// Feature of segment 2, same convention.
    pub f2: u32,
}

/// Result of a clipping function: `None` upstream maps to `clipped == false` and zeroed points.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ClipResultRaw {
    pub clipped: bool,
    pub points: [ClipPointRaw; 2],
}

/// `clip_segment_segment(seg1, seg2)` and `clip_segment_segment_with_normal(seg1, seg2, normal)`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ClipCase {
    pub id: felt252,
    pub seg1: SegmentRaw,
    pub seg2: SegmentRaw,
    /// Projection direction of the `with_normal` variant; its tangent is `(-normal.y, normal.x)`.
    pub normal: Vec2Raw,
    /// Projection on the direction of segment 1; points ordered along it.
    pub plain: ClipResultRaw,
    /// Projection on the tangent of `normal`; points ordered along the tangent.
    pub with_normal: ClipResultRaw,
}

/// `PointProjection`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ProjectionRaw {
    pub point: Vec2Raw,
    pub is_inside: bool,
}

/// `FeatureId` as returned by `project_local_point_and_get_feature` (2D: no edges).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum PointFeatureRaw {
    Unknown,
    Vertex: u32,
    Face: u32,
}

/// `SegmentPointLocation`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum SegmentLocationRaw {
    /// The query has no location (only segments report one).
    NoLocation,
    /// End point 0 (`a`) or 1 (`b`).
    OnVertex: u32,
    /// Interior point `a + u (b - a)`; the payload is `u` (upstream stores `[1 - u, u]`).
    OnEdge: i64,
}

/// Local-frame point queries on one shape.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ProjectionCase {
    pub id: felt252,
    pub shape: ShapeRaw,
    pub point: Vec2Raw,
    /// `project_local_point(point, false)`: an inside point is pushed to the boundary.
    pub projection: ProjectionRaw,
    /// `project_local_point(point, true)`: an inside point projects to itself.
    pub projection_solid: ProjectionRaw,
    /// `distance_to_local_point(point, false)`: negative inside.
    pub distance: i64,
    pub feature: PointFeatureRaw,
    /// `Segment::project_local_point_and_get_location`; `NoLocation` for the other shapes.
    pub location: SegmentLocationRaw,
}

/// Closest points between two segments (`pos12` places segment 2 in the frame of segment 1).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SegmentPairCase {
    pub id: felt252,
    pub seg1: SegmentRaw,
    pub seg2: SegmentRaw,
    pub pos12: PoseRaw,
    /// The closest pair is not unique (parallel or collinear overlap): only `dist_sq` is
    /// comparable, the points and locations are the member of the tie upstream happens to pick.
    pub ambiguous: bool,
    pub loc1: SegmentLocationRaw,
    pub loc2: SegmentLocationRaw,
    /// Closest point on segment 1, in the frame of segment 1.
    pub p1: Vec2Raw,
    /// Closest point on segment 2, in the frame of segment 2.
    pub p2: Vec2Raw,
    /// `|p1 - pos12 * p2|²`.
    pub dist_sq: i64,
}

/// `RayIntersection`; `hit = false` stands for upstream's `None` (the other fields are zeroed).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RayHitRaw {
    pub hit: bool,
    pub time_of_impact: i64,
    pub normal: Vec2Raw,
    pub feature: PointFeatureRaw,
}

/// Upstream's answers for one value of `solid`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RayAnswerRaw {
    /// `cast_ray` returned `Some(toi)`.
    pub has_toi: bool,
    pub toi: i64,
    /// `cast_ray_and_get_normal`.
    pub hit: RayHitRaw,
}

/// A world-space ray cast against one shape placed at `pose`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RayCase {
    pub id: felt252,
    pub shape: ShapeRaw,
    pub pose: PoseRaw,
    pub origin: Vec2Raw,
    /// Not normalised: the time of impact is in units of `dir`.
    pub dir: Vec2Raw,
    pub max_toi: i64,
    pub solid: RayAnswerRaw,
    pub hollow: RayAnswerRaw,
}

/// Settings of one free-axis joint, in Q32.32 raws; booleans distinguish absent controls.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SceneJointControlRaw {
    pub body1: u32,
    pub body2: u32,
    pub local_anchor1: Vec2Raw,
    pub local_anchor2: Vec2Raw,
    pub prismatic: bool,
    pub axis: Vec2Raw,
    pub has_limits: bool,
    pub min: i64,
    pub max: i64,
    pub has_motor: bool,
    pub target_pos: i64,
    pub target_vel: i64,
    pub stiffness: i64,
    pub damping: i64,
    pub max_force: i64,
    pub force_based: bool,
}
/// Controlled-joint scene. Base bodies/samples retain the original SceneCase layout;
/// the single joint is described separately so existing golden fixtures remain byte-identical.
#[derive(Copy, Drop)]
pub struct JointSceneCase {
    pub scene: SceneCase,
    pub joint: SceneJointControlRaw,
}

/// RJ: a rope (`max_dist`) or spring (`rest_length`, `stiffness`, `damping`, model) joint, in
/// Q32.32 raws; the fields of the other kind are zero.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SceneCoupledJointRaw {
    pub body1: u32,
    pub body2: u32,
    pub local_anchor1: Vec2Raw,
    pub local_anchor2: Vec2Raw,
    pub rope: bool,
    pub max_dist: i64,
    pub rest_length: i64,
    pub stiffness: i64,
    pub damping: i64,
    pub force_based: bool,
}
/// RJ: coupled-joint scene, laid out as `JointSceneCase`.
#[derive(Copy, Drop)]
pub struct CoupledJointSceneCase {
    pub scene: SceneCase,
    pub joint: SceneCoupledJointRaw,
}

/// SI: vertical state after a `ball_drop_sleep` step, Q32.32 raw. Upstream updates the
/// timer before solving; Cairo updates it after motion. Wake/sleep resets also affect it.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SleepImpactBodyRaw {
    pub y: i64,
    pub vy: i64,
    pub timer: i64,
    pub sleeping: bool,
}

/// SI: first solver contact of a pair; absent pairs have zero fields and `present = false`.
/// Geometry is pre-solve, impulses post-solve; contact_id includes upstream's NEW bit.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SleepImpactPairRaw {
    pub present: bool,
    pub dist: i64,
    pub contact_id: u32,
    pub impulse: i64,
    pub warmstart: i64,
}

/// SI: lower/upper ball states and ground/ball-ball contacts in canonical order.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SleepImpactRaw {
    pub step: u32,
    pub bodies: [SleepImpactBodyRaw; 2],
    pub pairs: [SleepImpactPairRaw; 2],
}

/// Fixed-capacity polygon fixture; only the first count vertices are live.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ConvexPolygonRaw {
    pub vertices: [Vec2Raw; 8],
    pub count: u8,
}

/// Polygon-specific fixture preserving the closed MVP fixture enum.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct PolygonProjectionCase {
    pub id: felt252,
    pub ambiguous: bool,
    /// Upstream EPA returns a non-nearest boundary point on the pentagon center.
    pub gjk_degenerate: bool,
    pub shape: ConvexPolygonRaw,
    pub point: Vec2Raw,
    /// `project_local_point(point, false)`: an inside point is pushed to the boundary.
    pub projection: ProjectionRaw,
    /// `project_local_point(point, true)`: an inside point projects to itself.
    pub projection_solid: ProjectionRaw,
    /// `distance_to_local_point(point, false)`: negative inside.
    pub distance: i64,
    pub feature: PointFeatureRaw,
    /// `Segment::project_local_point_and_get_location`; `NoLocation` for the other shapes.
    pub location: SegmentLocationRaw,
}

/// Polygon-specific fixture preserving the closed MVP fixture enum.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct PolygonRayCase {
    pub id: felt252,
    pub shape: ConvexPolygonRaw,
    pub pose: PoseRaw,
    pub origin: Vec2Raw,
    /// Not normalised: the time of impact is in units of `dir`.
    pub dir: Vec2Raw,
    pub max_toi: i64,
    pub solid: RayAnswerRaw,
    pub hollow: RayAnswerRaw,
}

/// Polygon-specific fixture preserving the closed MVP fixture enum.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct PolygonAabbCase {
    pub id: felt252,
    pub shape: ConvexPolygonRaw,
    pub pose: PoseRaw,
    pub mins: Vec2Raw,
    pub maxs: Vec2Raw,
}

/// Polygon-specific fixture preserving the closed MVP fixture enum.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct PolygonShapeMassCase {
    pub id: felt252,
    pub shape: ConvexPolygonRaw,
    pub density: i64,
    pub expected: MassPropertiesRaw,
}

/// Polygon contacts extend the golden shape set without changing the frozen MVP fixtures.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum PolygonContactShapeRaw {
    Polygon: ConvexPolygonRaw,
    Other: ShapeRaw,
}

/// PFM manifold fixture involving at least one polygon; semantics as `ManifoldCase`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct PolygonManifoldCase {
    pub id: felt252,
    pub shape1: PolygonContactShapeRaw,
    pub shape2: PolygonContactShapeRaw,
    pub pos12: PoseRaw,
    pub ambiguous: bool,
    pub num_points: u32,
    pub local_n1: Vec2Raw,
    pub local_n2: Vec2Raw,
    pub points: [ContactPointRaw; 2],
}

/// KD wrapper keeps existing SceneCase fixtures unchanged. In `scene`, Dynamic marks
/// every sampled body, including the controlled kinematic body. Body index 4 means absent.
#[derive(Copy, Drop)]
pub struct KinematicSceneCase {
    pub scene: SceneCase,
    pub kinematic_body: u32,
    pub position_based: bool,
    pub velocity: Vec2Raw,
    /// Integer Q32.32 translation increment per frame, starting at the initial pose.
    pub target_delta: Vec2Raw,
    pub dominance_body: u32,
    pub dominance_group: i8,
}

/// One upstream contact point at a replay-window boundary, matched by colliders/features.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SceneWarmstartRaw {
    pub collider1: u32,
    pub collider2: u32,
    pub fid1: u32,
    pub fid2: u32,
    pub local_p1: Vec2Raw,
    pub local_p2: Vec2Raw,
    pub local_n1: Vec2Raw,
    pub local_n2: Vec2Raw,
    pub dist: i64,
    pub impulse: i64,
    pub tangent_impulse: i64,
    pub warmstart_impulse: i64,
    pub warmstart_tangent_impulse: i64,
}

/// Post-solver normal-force event from upstream; Q32.32 raw fields.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ForceEventRaw {
    pub step: u32,
    pub collider1: u32,
    pub collider2: u32,
    pub total_force: Vec2Raw,
    pub total_force_magnitude: i64,
    pub max_force_direction: Vec2Raw,
    pub max_force_magnitude: i64,
    pub started: bool,
}

/// SE: `DefaultQueryDispatcher::intersection_test`; `supported == false` is `Err(Unsupported)`,
/// `gjk_touching` an exact contact that upstream answers with GJK.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct IntersectionCase {
    pub id: felt252,
    pub shape1: PolygonContactShapeRaw,
    pub shape2: PolygonContactShapeRaw,
    pub pos12: PoseRaw,
    pub supported: bool,
    pub intersecting: bool,
    pub gjk_touching: bool,
}

/// One body of a G0 level scene, in insertion order (ground, blocks, cores, pebble).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct LevelBodyRaw {
    /// `'ground'`, `'block'`, `'core'` or `'pebble'`.
    pub role: felt252,
    pub shape: PolygonContactShapeRaw,
    pub pose: PoseRaw,
    pub density: i64,
    pub friction: i64,
    pub restitution: i64,
}

/// One `closest_points` answer of the QY1 family: `kind` is 0 `Intersecting`, 1
/// `WithinMargin(p1, p2)` (world space), 2 `Disjoint`; the points are zero unless `kind == 1`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ClosestPointsRaw {
    pub margin: i64,
    pub kind: u8,
    pub p1: Vec2Raw,
    pub p2: Vec2Raw,
}

/// One `contact` answer of the QY1 family (world space); every field is zero when `some` is
/// false.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ContactAnswerRaw {
    pub prediction: i64,
    pub some: bool,
    pub point1: Vec2Raw,
    pub point2: Vec2Raw,
    pub normal1: Vec2Raw,
    pub normal2: Vec2Raw,
    pub dist: i64,
}

/// Parry's top-level shape-pair queries on one case (QY1): shape 1 at `pos1`, shape 2 at `pos2`.
/// `iterative_*` tell that upstream answers that query with GJK (EPA for a penetrating contact);
/// `contact_swapped` that the contacts are upstream's answer for the swapped pair, flipped back
/// (its half-space-second contact kernel does not invert `pos12`). Every answer is zero when
/// `supported` is false.
#[derive(Copy, Drop)]
pub struct ShapeQueryCase {
    pub id: felt252,
    pub shape1: PolygonContactShapeRaw,
    pub shape2: PolygonContactShapeRaw,
    pub pos1: PoseRaw,
    pub pos2: PoseRaw,
    pub supported: bool,
    pub iterative_distance: bool,
    pub iterative_closest_points: bool,
    pub iterative_contact: bool,
    pub contact_swapped: bool,
    pub distance: i64,
    pub closest_points: [ClosestPointsRaw; 3],
    pub contacts: [ContactAnswerRaw; 2],
}

/// SH1: a cuboid of `half_extents` with rounded corners of `border_radius`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RoundCuboidRaw {
    pub half_extents: Vec2Raw,
    pub border_radius: i64,
}

/// SH1: a triangle with rounded corners of `border_radius`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RoundTriangleRaw {
    pub triangle: TriangleRaw,
    pub border_radius: i64,
}

/// SH1: a convex polygon with rounded corners of `border_radius`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RoundPolygonRaw {
    pub polygon: ConvexPolygonRaw,
    pub border_radius: i64,
}

/// SH1 fixture shapes: the closed MVP set, polygons, triangles and round shapes (keeps the frozen
/// `ShapeRaw` unchanged).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum Sh1ShapeRaw {
    Other: ShapeRaw,
    Polygon: ConvexPolygonRaw,
    Triangle: TriangleRaw,
    RoundCuboid: RoundCuboidRaw,
    RoundTriangle: RoundTriangleRaw,
    RoundPolygon: RoundPolygonRaw,
}

/// SH1 manifold fixture; semantics as `ManifoldCase`, plus upstream's `query::intersection_test`
/// and `query::distance` for the same placement.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Sh1ManifoldCase {
    pub id: felt252,
    pub shape1: Sh1ShapeRaw,
    pub shape2: Sh1ShapeRaw,
    pub pos12: PoseRaw,
    pub ambiguous: bool,
    pub num_points: u32,
    pub local_n1: Vec2Raw,
    pub local_n2: Vec2Raw,
    pub points: [ContactPointRaw; 2],
    pub intersects: bool,
    pub distance: i64,
}

/// `TrianglePointLocation::OnEdge(edge, [u, v])`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct TriangleEdgeRaw {
    pub edge: u32,
    pub u: i64,
    pub v: i64,
}

/// `TrianglePointLocation` of a hollow projection; `NoLocation` for the other shapes.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum TriangleLocationRaw {
    NoLocation,
    OnVertex: u32,
    OnEdge: TriangleEdgeRaw,
    OnSolid,
}

/// SH1 point projection fixture (local frame); semantics as `ProjectionCase`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Sh1ProjectionCase {
    pub id: felt252,
    pub shape: Sh1ShapeRaw,
    pub point: Vec2Raw,
    pub projection: ProjectionRaw,
    pub projection_solid: ProjectionRaw,
    pub distance: i64,
    pub feature: PointFeatureRaw,
    pub location: TriangleLocationRaw,
}

/// SH1 world-space ray cast fixture; semantics as `RayCase`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Sh1RayCase {
    pub id: felt252,
    pub shape: Sh1ShapeRaw,
    pub pose: PoseRaw,
    pub origin: Vec2Raw,
    pub dir: Vec2Raw,
    pub max_toi: i64,
    pub solid: RayAnswerRaw,
    pub hollow: RayAnswerRaw,
}

/// SH1 `shape.mass_properties(density)`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Sh1MassCase {
    pub id: felt252,
    pub shape: Sh1ShapeRaw,
    pub density: i64,
    pub expected: MassPropertiesRaw,
}

/// SH1 `shape.compute_aabb(pose)`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Sh1AabbCase {
    pub id: felt252,
    pub shape: Sh1ShapeRaw,
    pub pose: PoseRaw,
    pub mins: Vec2Raw,
    pub maxs: Vec2Raw,
}
