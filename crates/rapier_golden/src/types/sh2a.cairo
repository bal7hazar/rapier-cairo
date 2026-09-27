//! SH2a fixture types (polylines and 2D heightfields), declared apart from `super` (compile
//! budget).

use super::{
    ClosestPointsRaw, ContactAnswerRaw, ContactPointRaw, PointFeatureRaw, PoseRaw, ProjectionRaw,
    RayAnswerRaw, Sh1ShapeRaw, ShapeCastHitRaw, Vec2Raw,
};

/// SH2a: a composite shape of the families, built by the tests from its raws: a polyline
/// (`vertices`, `indices` or chained when empty, `oriented`) or a heightfield (`heights` times
/// `scale`, cells `removed`).
#[derive(Copy, Drop)]
pub struct CompositeRaw {
    pub is_heightfield: bool,
    pub vertices: Span<Vec2Raw>,
    pub indices: Span<(u32, u32)>,
    pub oriented: bool,
    pub heights: Span<i64>,
    pub scale: Vec2Raw,
    pub removed: Span<u32>,
}

/// SH2a: the manifold of one part (a polyline segment, a heightfield cell) with its points.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct PartManifoldRaw {
    pub part: u32,
    pub num_points: u32,
    pub local_n1: Vec2Raw,
    pub local_n2: Vec2Raw,
    pub points: [ContactPointRaw; 2],
}

/// SH2a: `contact_manifolds` of composite `composite` (see `composite_shapes`) and `other`
/// (composite first when `composite_first`), the manifolds with points in ascending part, plus
/// `intersection_test` and `distance` (each `*_supported` false for upstream's `Unsupported`).
#[derive(Copy, Drop)]
pub struct CompositeManifoldCase {
    pub id: felt252,
    pub composite: u32,
    pub composite_first: bool,
    pub other: Sh1ShapeRaw,
    pub pos12: PoseRaw,
    pub num_manifolds: u32,
    pub manifolds: [PartManifoldRaw; 4],
    pub intersects_supported: bool,
    pub intersects: bool,
    pub distance_supported: bool,
    pub distance: i64,
}

/// SH2a: local point queries on a composite shape.
#[derive(Copy, Drop)]
pub struct CompositePointCase {
    pub id: felt252,
    pub composite: u32,
    pub point: Vec2Raw,
    pub projection: ProjectionRaw,
    pub projection_solid: ProjectionRaw,
    pub feature_projection: ProjectionRaw,
    pub feature: PointFeatureRaw,
    pub distance: i64,
    pub contains: bool,
}

/// SH2a: world-space ray casts on a composite shape at `pose`.
#[derive(Copy, Drop)]
pub struct CompositeRayCase {
    pub id: felt252,
    pub composite: u32,
    pub pose: PoseRaw,
    pub origin: Vec2Raw,
    pub dir: Vec2Raw,
    pub max_toi: i64,
    pub solid: RayAnswerRaw,
    pub hollow: RayAnswerRaw,
}

/// SH2a: the world-space shape-pair queries of a composite and a convex shape (composite first
/// when `composite_first`): `intersection_test`, `distance`, `contact` at `prediction`,
/// `closest_points` at `margin` and `cast_shapes` of the convex shape moving at `vel` (maximum
/// time 10). Every answer is zero when its `*_supported` is false.
#[derive(Copy, Drop)]
pub struct CompositePairCase {
    pub id: felt252,
    pub composite: u32,
    pub composite_first: bool,
    pub other: Sh1ShapeRaw,
    pub pos1: PoseRaw,
    pub pos2: PoseRaw,
    pub vel: Vec2Raw,
    pub prediction: i64,
    pub margin: i64,
    pub intersects_supported: bool,
    pub intersects: bool,
    pub distance_supported: bool,
    pub distance: i64,
    pub contact_supported: bool,
    pub contact: ContactAnswerRaw,
    pub closest_supported: bool,
    pub closest: ClosestPointsRaw,
    pub cast_supported: bool,
    pub cast: ShapeCastHitRaw,
}

/// SH2a: world AABB, bounding sphere and mass of a composite shape.
#[derive(Copy, Drop)]
pub struct CompositeAabbCase {
    pub id: felt252,
    pub composite: u32,
    pub pose: PoseRaw,
    pub mins: Vec2Raw,
    pub maxs: Vec2Raw,
    pub center: Vec2Raw,
    pub radius: i64,
    pub mass: i64,
}
