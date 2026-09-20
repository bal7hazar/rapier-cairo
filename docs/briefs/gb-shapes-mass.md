# GB — `rapier_geometry2d::shape` and mass properties

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D6 closed shape enum, wave-1 outcomes); `docs/interfaces/geometry-dynamics.md`
§4; style precedents on `main`: `crates/rapier_core/src/integration_parameters.cairo` (golden
comparisons, ulp tables), `crates/rapier_math/src/math_ext/vec2.cairo` (fused kernels).
Upstream (`UP=/private/tmp/claude-501/-Users-bal7hazar-git-rapier-cairo--claude-worktrees-rapier-physics-cairo-benchmark-2d3317/239b7c62-97d7-47ea-9d79-74f81363555f/scratchpad/refs`):
`$UP/parry/src/shape/{ball,cuboid,capsule,segment,half_space,support_map}.rs`,
`$UP/parry/src/bounding_volume/{aabb_ball,aabb_cuboid,aabb_capsule,aabb_halfspace,aabb_support_map}.rs`,
`$UP/parry/src/mass_properties/{mass_properties,mass_properties_ball,mass_properties_cuboid,mass_properties_capsule}.rs`.
Golden vectors: `rapier_golden::mass_properties` (12 cases incl. a two-collider body),
`rapier_golden::aabb` (4 shapes × 8 poses).

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/shape.cairo` + `src/shape/{ball,cuboid,capsule,segment,halfspace}.cairo`,
`crates/rapier_geometry2d/src/mass.cairo` (add impls below the frozen struct; **do not change the
struct**), `crates/rapier_geometry2d/tests/{mass_golden,aabb_golden}.cairo`,
`gas/rapier_geometry2d/{shape,mass}.snap`, `gas/rapier_geometry2d_integrationtest/{mass_golden,aabb_golden}.snap`.
`lib.cairo` already declares `pub mod shape;`. Everything else is forbidden.

## 3. Expected API and semantics
Frozen inputs: `fixed`, `glam::vec2::Vec2`, `rapier_math::{consts, math_ext}`,
`rapier_math::pose2::Pose2` / `rot2::Rot2` (M2), `rapier_geometry2d::aabb::Aabb` (GA — if GA is not
merged when you start, define the four `Aabb` helpers you need privately in `shape/aabb_shim.cairo`
and list the swap under "Escalations"), `rapier_geometry2d::mass::MassProperties`,
`rapier_geometry2d::feature_id::FeatureId`.
```cairo
#[derive(Copy, Drop, Serde, PartialEq, Debug)] pub struct Ball { pub radius: Fixed }
#[derive(Copy, Drop, Serde, PartialEq, Debug)] pub struct Cuboid { pub half_extents: Vec2 }
#[derive(Copy, Drop, Serde, PartialEq, Debug)] pub struct Capsule { pub segment: Segment, pub radius: Fixed }
#[derive(Copy, Drop, Serde, PartialEq, Debug)] pub struct Segment { pub a: Vec2, pub b: Vec2 }
#[derive(Copy, Drop, Serde, PartialEq, Debug)] pub struct HalfSpace { pub normal: Vec2 }   // unit
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum Shape { Ball: Ball, Cuboid: Cuboid, Capsule: Capsule, Segment: Segment, HalfSpace: HalfSpace }
```
`ShapeTrait` (dispatch by `match`): `compute_local_aabb`, `compute_aabb(pose)`,
`mass_properties(density)`, `shape_type() -> ShapeType`, `as_ball()/as_cuboid()/…` returning
`Option`. Per shape, upstream names: `Ball::new`, `Cuboid::new`, `Cuboid::vertex_feature_id`
(**f32 semantics**: `FeatureIdTrait::vertex` of the 2-bit quadrant code as upstream's f32 build
computes it), `Cuboid::feature_normal`, `Cuboid::support_feature` (polygonal feature for GD),
`Cuboid::local_support_point`/`local_support_point_toward`, `Capsule::new`, `Capsule::segment`,
`Capsule::center`, `Capsule::height`, `Capsule::local_support_point`, `Segment::direction`,
`Segment::scaled_direction`, `Segment::length`, `Segment::normal`, `HalfSpace::new`,
`support_map` helpers (`local_support_point` for ball/cuboid/capsule/segment).
`MassProperties`: `from_ball(density, radius)`, `from_cuboid(density, half_extents)`,
`from_capsule(density, a, b, radius)`, `from_segment` (zero), `new(local_com, mass, principal_inertia)`,
`mass()`, `principal_inertia()`, `transform_by(pose)` (parallel-axis shift), `Add` (combine),
`Sub`, `set_mass`, `with_inertia` — with `inv(0) = 0` semantics. DEFER: convex polygon, round
shapes, compound, `scaled`, `cast_local_ray`.

## 4. Efficiency and variants
Fused kernels for inertia formulas and the AABB of a rotated cuboid (`|R|·h`); measure
`compute_aabb` per shape and `mass_properties` per shape against a composed-ops candidate for the
cuboid AABB and the capsule inertia. Keep divisions out of `compute_aabb`.

## 5. Tests
Golden: every `rapier_golden::mass_properties` case within the README tolerance (report a per-field
ulp table for mass, inv_mass, com, inv inertia); every `rapier_golden::aabb` case (pose rebuilt
from `PoseRaw` through `Rot2::from_cos_sin`) within tolerance. Table-driven `test_*` per method,
degenerate shapes (zero radius, zero-length segment), feature-id encoding matches the f32 ids in
`rapier_golden::contact_manifolds` for cuboid vertices. `fuzz_*` ≤ 4 per module; `gas_*` for
every public function and candidate. ≤ 800 lines per file.

## 6. Definition of done
Foreground gate (`scarb fmt --workspace && scarb lint --workspace --deny-warnings && scarb build
--workspace && snforge test --workspace`); `python3 scripts/gas.py snapshot --filter
rapier_geometry2d::shape`, `… ::mass`, `… rapier_geometry2d_integrationtest::mass_golden`,
`… ::aabb_golden`; conventional commits + trailer; push; `gh pr create` per template;
`gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · API · Gas table ·
Deviations · Deferred · Requested re-exports · Escalations · PR URL) with the ulp tables.

## 7. Work autonomously, do not ask questions, do not widen the scope.
