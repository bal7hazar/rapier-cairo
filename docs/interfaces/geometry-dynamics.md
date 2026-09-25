# F3 — frozen interface between `rapier_geometry2d` and `rapier_dynamics2d`

Status: **draft v1, 2026-09-20**, to be turned into code in `rapier_geometry2d::contact` and
`rapier_geometry2d::mass` as the first wave-3 commit (orchestrator), once glam-cairo's `Vec2`
is available. Executors of wave 3 code against these definitions; any change goes through the
orchestrator (AGENTS.md §4).

Sources: parry `query/contact_manifolds/contact_manifold.rs` (`TrackedContact`,
`ContactManifold`), `shape/feature_id.rs` (`FeatureId`, `PackedFeatureId`),
`mass_properties/mass_properties.rs`; rapier `geometry/contact_pair.rs` (`ContactData`,
`ContactManifoldData`, `SolverContactGeneric`, `ContactId`, `SolverFlags`, `ContactPair`).
`Fixed` is glam-cairo's Q32.32 scalar, `Vec2` is glam-cairo's, `Rot2`/`Pose2` come from M2.

## 1. Feature ids

```cairo
/// Parry's `PackedFeatureId`: 2-bit header (`01` vertex, `11` face, `00` unknown; `10` edge is
/// 3D-only) in the top bits of a `u32`, 30-bit code below. Kept as one `u32` so that comparing two
/// ids is one felt comparison, which `match_contacts` does for every point every step.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct FeatureId { pub packed: u32 }

pub const FEATURE_UNKNOWN: FeatureId = FeatureId { packed: 0 };
pub trait FeatureIdTrait {
    fn vertex(code: u32) -> FeatureId;   // 0x4000_0000 + code   (code < 2^30)
    fn face(code: u32) -> FeatureId;     // 0xC000_0000 + code
    fn is_vertex(self: FeatureId) -> bool;
    fn is_face(self: FeatureId) -> bool;
    fn is_unknown(self: FeatureId) -> bool;
    fn code(self: FeatureId) -> u32;
}
```

Semantics to port from the **f32** build (see `docs/PLAN.md`, wave 1 outcomes): the f64 build's
cuboid ids are broken upstream; fixtures carry f32 ids. Header/code split is done with
`DivRem` by `0x4000_0000`, never with shifts.

## 2. Contact points and manifolds (geometry side)

```cairo
/// One tracked contact point, in the local frames of the two shapes (parry `TrackedContact`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct TrackedContact {
    pub local_p1: Vec2,
    pub local_p2: Vec2,
    /// Signed distance along the manifold normal: < 0 penetration, > 0 separation (speculative).
    pub dist: Fixed,
    pub fid1: FeatureId,
    pub fid2: FeatureId,
    /// Rapier's `ContactData`, carried on the point so warm-start impulses follow the feature ids.
    pub data: ContactData,
}

/// Solver state persisted on a contact point between steps (rapier `ContactData`).
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct ContactData {
    pub impulse: Fixed,
    pub tangent_impulse: Fixed,            // one tangent in 2D
    pub warmstart_impulse: Fixed,
    pub warmstart_tangent_impulse: Fixed,
}

/// A contact manifold between two (sub)shapes (parry `ContactManifold`, 2D: at most 2 points).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ContactManifold {
    /// Points, in generation order; `num_points ∈ {0, 1, 2}`. Fixed-size, count-tagged storage
    /// is the pattern already used by `rapier_golden` and costs no allocation.
    pub points: [TrackedContact; 2],
    pub num_points: u8,
    /// Manifold normal in the local frame of shape 1 / shape 2 (unit; pointing from 1 to 2).
    pub local_n1: Vec2,
    pub local_n2: Vec2,
    /// Subshape indices (compound shapes; 0 for simple shapes).
    pub subshape1: u32,
    pub subshape2: u32,
    /// Rapier's per-manifold solver data (`ContactManifoldData`), see §3.
    pub data: ContactManifoldData,
}
```

Invariants every generator must respect (they are what `try_update_contacts` and the solver
assume): `local_n1` and `local_n2` are unit within `UNIT_TOL_SQ_RAW` (`rapier_math::consts`);
`local_n2 == -pos12.rotation.inverse_rotate(local_n1)`; points beyond the prediction distance may
be present (upstream keeps them, report 02); a manifold with `num_points == 0` is legal and must
be ignored by the solver, not treated as an error.

Generator entry point (dispatch, package GG):

```cairo
/// Parry's `contact_manifolds` for one convex pair. `pos12` is the pose of shape 2 in the local
/// frame of shape 1 (`pose1.inv_mul(pose2)`). `manifold` is the previous step's manifold for
/// warm-start matching, or a default one. Returns `true` when a manifold generator exists for the
/// pair (upstream: `Ok`), `false` when the pair is unsupported (upstream: `Err(Unsupported)`).
fn contact_manifold(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool;
```

Persistence helpers (package GE): `try_update_contacts(ref manifold, pos12) -> bool` (fast path
when the normal moved less than 1° — `COS_1_DEGREES` compared on the wide dot — and points less
than `1e-3` scaled), and `match_contacts(ref new, old)` transferring `ContactData` where
`(fid1, fid2)` match (the `NEW_CONTACT_BIT` of rapier's `ContactId` is how the solver learns a
point is new; see §3).

## 3. Per-manifold solver data and solver contacts (dynamics side)

```cairo
/// rapier `SolverFlags`: bit 0 `COMPUTE_RIGID_IMPULSES`. Kept as a `u32` newtype.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct SolverFlags { pub bits: u32 }

/// rapier `ContactManifoldData` minus the parallel-solver fields (colour, graph position,
/// solver body ids), which the sequential solver does not need.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ContactManifoldData {
    pub rigid_body1: Option<Handle>,       // `rapier_core::data::Handle`
    pub rigid_body2: Option<Handle>,
    pub solver_flags: SolverFlags,
    /// World-space normal (from body 1 to body 2), refreshed by the narrow phase each step.
    pub normal: Vec2,
    pub solver_contacts: [SolverContact; 2],
    pub num_solver_contacts: u8,
    pub relative_dominance: i16,
    pub user_data: u32,
    pub friction: Fixed,
    pub restitution: Fixed,
}

/// rapier `SolverContact` (2D lanes = 1): what the constraint builder reads. `anchor*` are the
/// contact points relative to each body's centre of mass, in world space.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SolverContact {
    pub anchor1: Vec2,
    pub anchor2: Vec2,
    pub dist: Fixed,
    pub tangent_velocity: Vec2,
    /// Index of the `TrackedContact` in the manifold plus `NEW_CONTACT_BIT` (`2^31`) when the
    /// point had no match in the previous manifold (no warm start, restitution allowed).
    pub contact_id: u32,
}
pub const NEW_CONTACT_BIT: u32 = 0x8000_0000;
```

`num_solver_contacts ≤ num_points`; the narrow phase builds `solver_contacts` from
`points` by transforming `local_p1`/`local_p2` to world space, skipping points whose `dist`
exceeds the prediction distance (report 01 §2.3), and computing `dist`, anchors and
`tangent_velocity` (kinematic/conveyor surfaces; zero for the MVP).

## 4. Mass properties

```cairo
/// parry `MassProperties` (2D): centre of mass in the shape's local frame, inverse mass, inverse
/// principal angular inertia (a scalar in 2D). `inv_mass == 0` means infinite mass; `inv(0) = 0`
/// (`rapier_math::math_ext::inv`) is what makes fixed bodies work without special cases.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct MassProperties {
    pub local_com: Vec2,
    pub inv_mass: Fixed,
    pub inv_principal_inertia: Fixed,
}
```

Operations frozen with it: `from_ball(density, radius)`, `from_cuboid(density, half_extents)`,
`from_capsule(density, a, b, radius)`, `mass()`, `principal_inertia()` (via `inv`),
`transform_by(pose)`, `Add` (combine two properties; upstream's parallel-axis formula),
`with_inertia_scaled`. Golden vectors: `rapier_golden::mass_properties` (12 cases).

## 5. Ordering and determinism rules that cross the boundary

- Manifold points keep generation order; the solver iterates `0..num_points`.
- Pairs are processed in ascending `(collider1, collider2)` handle order (decision D8); the
  broad phase returns pairs already sorted that way.
- No dictionary iteration order is observable anywhere on this boundary.
- Rounding: all point/normal transforms go through `Pose2` fused kernels (floor); `dist` is
  computed as a wide dot product then rescaled once.
