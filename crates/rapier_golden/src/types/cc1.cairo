//! CC1 fixture types (shape casts and sweeps), declared apart from `super` (compile budget).

use super::{PoseRaw, Sh1ShapeRaw, Vec2Raw};

/// CC1: one shape-cast answer (world poses in, witnesses and normals in the local frame of their
/// shape). `status` is the declaration index of `ShapeCastStatus` (0 `OutOfIterations`,
/// 1 `Converged`, 2 `Failed`, 3 `PenetratingOrWithinTargetDist`); every field is zero when `some`
/// is false.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ShapeCastHitRaw {
    pub some: bool,
    pub toi: i64,
    pub witness1: Vec2Raw,
    pub witness2: Vec2Raw,
    pub normal1: Vec2Raw,
    pub normal2: Vec2Raw,
    pub status: u8,
}

/// CC1: one `ShapeCastOptions`; `max_time_of_impact = i64::MAX` stands for upstream's `Real::MAX`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ShapeCastOptionsRaw {
    pub max_time_of_impact: i64,
    pub target_distance: i64,
    pub stop_at_penetration: bool,
    pub compute_impact_geometry_on_penetration: bool,
}

/// CC1: `query::cast_shapes(pos1, vel1, shape1, pos2, vel2, shape2, options)` under each option
/// set of the family's `OPTIONS`, in order. `iterative`: upstream answers with GJK. Every answer
/// is zero when `supported` is false.
#[derive(Copy, Drop)]
pub struct ShapeCastCase {
    pub id: felt252,
    pub shape1: Sh1ShapeRaw,
    pub shape2: Sh1ShapeRaw,
    pub pos1: PoseRaw,
    pub vel1: Vec2Raw,
    pub pos2: PoseRaw,
    pub vel2: Vec2Raw,
    pub supported: bool,
    pub iterative: bool,
    pub answers: [ShapeCastHitRaw; 5],
}

/// CC1: a `NonlinearRigidMotion`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct NonlinearMotionRaw {
    pub start: PoseRaw,
    pub local_center: Vec2Raw,
    pub linvel: Vec2Raw,
    pub angvel: i64,
}

/// CC1: `query::cast_shapes_nonlinear(motion1, shape1, motion2, shape2, START_TIME, END_TIME,
/// stop)` with `stop` true then false. Every answer is zero when `supported` is false.
#[derive(Copy, Drop)]
pub struct NonlinearShapeCastCase {
    pub id: felt252,
    pub shape1: Sh1ShapeRaw,
    pub shape2: Sh1ShapeRaw,
    pub motion1: NonlinearMotionRaw,
    pub motion2: NonlinearMotionRaw,
    pub supported: bool,
    pub answers: [ShapeCastHitRaw; 2],
}

/// CC1: `query::sweep_time_of_impact(proxy(shape1), Sweep::constant(pose1, 0), proxy(shape2),
/// Sweep::from_poses(start2, end2, local_center2), MAX_FRACTION, LINEAR_SLOP)`. `status` is the
/// declaration index of `SweepToiStatus` (0 `Overlapped`, 1 `Hit`, 2 `Separated`, 3 `Failed`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SweepToiCase {
    pub id: felt252,
    pub shape1: Sh1ShapeRaw,
    pub shape2: Sh1ShapeRaw,
    pub pose1: PoseRaw,
    pub start2: PoseRaw,
    pub end2: PoseRaw,
    pub local_center2: Vec2Raw,
    pub status: u8,
    pub fraction: i64,
    pub point: Vec2Raw,
    pub normal: Vec2Raw,
}
