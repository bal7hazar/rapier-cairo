//! SH2b fixture types (compound shapes), declared apart from `super` (compile budget).

use super::{ContactPointRaw, PoseRaw, Sh1ShapeRaw, Vec2Raw};

/// SH2b: a compound of the families, built by the tests from its raws: parts placed by their
/// poses in the compound's frame.
#[derive(Copy, Drop)]
pub struct CompoundRaw {
    pub parts: Span<(PoseRaw, Sh1ShapeRaw)>,
}

/// SH2b: the other shape of a compound case: a convex shape, an SH2a composite
/// (`composite_shapes::composite(index)`) or a compound (`compound_shapes::compound(index)`).
#[derive(Copy, Drop)]
pub enum OtherRaw {
    Convex: Sh1ShapeRaw,
    Composite: u32,
    Compound: u32,
}

/// SH2b: the manifold of one part pair with its points and both sub-shape ids.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct PairManifoldRaw {
    pub subshape1: u32,
    pub subshape2: u32,
    pub num_points: u32,
    pub local_n1: Vec2Raw,
    pub local_n2: Vec2Raw,
    pub points: [ContactPointRaw; 2],
}

/// SH2b: `contact_manifolds` of compound `compound` (see `compound_shapes`) and `other`
/// (compound first when `compound_first`), the manifolds with points in ascending
/// `(subshape1, subshape2)`, plus `intersection_test` and `distance` (each `*_supported` false for
/// upstream's `Unsupported`; `distance_infinite` for upstream's `Real::MAX`, no supported part).
#[derive(Copy, Drop)]
pub struct CompoundManifoldCase {
    pub id: felt252,
    pub compound: u32,
    pub compound_first: bool,
    pub other: OtherRaw,
    pub pos12: PoseRaw,
    pub num_manifolds: u32,
    pub manifolds: [PairManifoldRaw; 4],
    pub intersects_supported: bool,
    pub intersects: bool,
    pub distance_supported: bool,
    pub distance_infinite: bool,
    pub distance: i64,
}

/// SH2b: world AABB, bounding sphere and mass properties of a compound at `pose` for `density`.
#[derive(Copy, Drop)]
pub struct CompoundShapeMassCase {
    pub id: felt252,
    pub compound: u32,
    pub pose: PoseRaw,
    pub density: i64,
    pub mins: Vec2Raw,
    pub maxs: Vec2Raw,
    pub center: Vec2Raw,
    pub radius: i64,
    pub mass: i64,
    pub local_com: Vec2Raw,
    pub inertia: i64,
}
