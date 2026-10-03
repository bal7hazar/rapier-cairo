# SC2 — the parked API families: what opt-in API they allow, and at what cost

Lot SC2 (research, `docs/briefs/sc2-parked-families.md`). Base: `main` at `7c4af6c` (after PX7). Toolchain: Scarb
2.20.1 / snforge 0.64.0 (`.tool-versions`), on the VPS through the shared build-lock shims, `RAYON_NUM_THREADS=1`.
Upstream read: rapier `0.35.3` (the crates.io copy, `~/.cargo/registry/src/index.crates.io-*/rapier2d-f64-0.35.3`) and
the reference clone `0.35.3+4` (`~/git/refs/rapier` at `28d0ba92`, the parity script's default), parry `0.31.1`
(`~/git/refs/parry` at tag `v0.31.1`, `3383f51`) and `0.30.2` (the crates.io copy and `tools/golden/vendor`). The
parry 0.31.1 registry copy the brief mentions is on the Mac only; the git tag is the same release.

**Nothing in this lot changes the library.** Every prototype stayed uncommitted in the worktree; its figures come with
the command that produced them. "Estimate" marks every figure that was not measured.

## Summary

| family | items | recommendation | cost |
|---|---:|---|---|
| 1. solver scalar API (A) | 26 | **exclude now**: upstream's `dynamics::solver` is `pub(crate)`, so no rapier user can call these; add them to "solver / island internals not exposed" | 0 library lines, 0 steps (a parity-script edit). The opt-in alternative (1B) measured 160 lines, 0 step differences over 1,033 tests |
| 2. sub-shape result widening (E) | 7 | **port now** as twin values beside the old ones (`with_subshape(s)` → `Subshape*` types) and computed `subshape_pos1 / 2` | 100 library lines (measured; +41.6 % → +41.4 % line margin), 0 step differences over 2,660 existing tests; 50 steps per `with_subshapes` |
| 3. `contact_skin` (F) | 3 | **exclude** with a closed reason and a round-shape recipe; a `StepConfig` strategy is the route if a game ever needs it | 0 now; the strategy is an estimated ≈ 200 lines on the step path, its zero-cost claim unproven |
| 4. compound internal edges (B) | 8 (+1, see §0) | **port later**, after a golden-oracle bump, as an opt-in `CompositeStrategy` that `World::step` does not compile | estimate ≈ 720 library lines, 0 steps for existing worlds except the compound codec. The bump to parry 0.31.1 was **run**: 30 of 33 vector files byte-identical (every scene and contact family); it moves feature ids in `composite_queries` / `compound_queries` and the 9 capsule ray casts (now analytic, toward the port) |

Open questions for the project manager:

* (1) closing PO1's "open gap" decision with the reachability proof;
* (3) the new closed reason for `contact_skin`;
* (4) the feature-id policy of a bump: follow 0.31 (moves the port's `FeatureId` results) or keep them as a divergence;
* (2) whether `set_subshape_pos1 / 2` go on `SubshapePoses` or close with a reason.

## 0. The families, counted

The four families are **44 of the 46 missing items** of `docs/API_PARITY.md` (the other two, `ColliderSet::take_removed`
and `GenericJointBuilder::user_data`, are missing by decision). The brief says 39; its own per-family counts add up to
44: solver 26, sub-shape widening 7, `contact_skin` 3, compound internal edges 8.

One parity-script finding on the way: `CompoundEdgeCone` is excluded as `dim3-only`, but in parry 0.31.1 it is the
**2D** type (`compound_pseudo_normals.rs`, `#[cfg(all(feature = "alloc", feature = "dim2"))]`, the element of
`CompoundPseudoNormals::boundary_edges`). Family 4 has 9 items, not 8; the exclusion is wrong and belongs to the next
parity lot.

## 1. The solver's scalar API (26 items, parked item (A))

### 1.1 What upstream exposes

Nothing. In rapier 0.35.3 (published) and in the reference clone, `dynamics::solver` is `pub(crate) mod solver`
(`src/dynamics/mod.rs`), `solver_body` is a private `mod solver_body` inside it, and `VelocitySolver` is
`pub(crate) struct VelocitySolver`. `SolverPoseRepr` and `SolverVelRepr` are private `struct`s (`#[repr(C)]` SIMD
blocks). **No user of rapier can name any of the 26 items**: the parity script lists them because it reads `pub` on the
item, not the reachability of its module. PO1 kept them open as "open gaps, not closed internals"
(`scripts/api_parity.py`, comment above `SOLVER_ISLAND_INTERNALS`); the reachability argument was not on record then.

The 26 items: `SolverBodies` (type, `clear`, `copy_from`, `get_pose`, `get_vel`, `len`, `resize`, `set_vel`),
`SolverVel` (`AddAssign`, `Sub`, `SubAssign`, `as_slice`, `as_mut_slice`, `as_vector_slice`, `as_vector_slice_mut`,
`zero`), `SolverPose` (type, `Default`, `pose`, `transform_point`, `inverse_transform_point`), `SolverTransform` (type,
`transform_point`), `SolverPoseRepr::identity`, `SolverVelRepr::zero`, `VelocitySolver::new`.

The port's own solver already has a value `SolverVel` (`solver/body.cairo`, `pub`, ported by name) and a `SolverBody`
that holds upstream's `SolverVel` and `SolverPose` in one value; `SolverBodyStore` / `DenseBodies` play
`SolverBodies`' role. EL1 (`origin/hp/slingfall-rapier/t-0035-el1-engine-levers`, head `1937924`) changes nothing
under `crates/rapier_dynamics2d/src/solver/**` (its only `rapier_dynamics2d` file is `collider_set.cairo`), so the
designs below do not conflict with it.

### 1.2 Options

* **1A — exclude by the existing closed reason (recommended).** Add the 26 items to `SOLVER_ISLAND_INTERNALS`
  ("solver / island internals not exposed"), with the reachability proof above as the comment. Cost: one parity-script
  change, 0 library lines, 0 steps. It closes the family and does not touch (c), the solver-graph order, which stays
  parked on its own.
* **1B — an opt-in value API, `solver::scalar`.** A new module that nothing on the step calls. Prototype
  (`crates/rapier_dynamics2d/src/solver/scalar.cairo`, uncommitted):

  ```cairo
  pub struct SolverPose { pub rotation: Rot2, pub translation: Vec2, pub ii: Fixed, pub im: Vec2 }
  pub struct SolverTransform { pub rotation: Rot2, pub translation: Vec2 }
  impl Default<SolverPose>;                                    // identity, zero inverse masses
  trait SolverPoseTrait { fn pose(self: @SolverPose) -> Pose2;
      fn transform_point(self: @SolverPose, pt: Vec2) -> Vec2;
      fn inverse_transform_point(self: @SolverPose, pt: Vec2) -> Vec2; }
  trait SolverTransformTrait { fn transform_point(self: @SolverTransform, pt: Vec2) -> Vec2; }
  trait SolverVelTrait { fn zero() -> SolverVel; fn as_slice(self: @SolverVel) -> [Fixed; 3];
      fn as_vector_slice(self: @SolverVel) -> Span<Fixed>; }
  impl Sub<SolverVel>; impl AddAssign<SolverVel, SolverVel>; impl SubAssign<SolverVel, SolverVel>;
  #[derive(Destruct, Default)] pub struct SolverBodies { vels: Felt252Dict<Nullable<SolverVel>>,
      poses: Felt252Dict<Nullable<SolverPose>>, len: u32 }
  trait SolverBodiesTrait { fn clear(ref self); fn resize(ref self, sz: u32); fn len(self: @SolverBodies) -> u32;
      fn copy_from(ref self, i: u32, rb: @RigidBody); fn get_vel(ref self, i: u32) -> SolverVel;
      fn set_vel(ref self, i: u32, vel: SolverVel); fn get_pose(ref self, i: u32) -> SolverPose; }
  ```

  `copy_from` follows upstream: pose at the centre of mass, effective inverse masses only for an awake dynamic or
  kinematic body. `as_mut_slice` / `as_vector_slice_mut` hand out `&mut` views and have no Cairo form (a
  `NO_INDEX_MUT`-like closed reason); `SolverPoseRepr::identity` / `SolverVelRepr::zero` are private SIMD block types
  (`SIMD/parallel`); `VelocitySolver::new` builds a holder of reusable buffers the port does not have. 1B therefore
  ports **21** items and closes 5. It gives a user nothing the port's own `SolverBody` / `SolverBodyStore` do not:
  upstream users cannot call these functions either.

### 1.3 Cost of 1B

Measured on the prototype, steps net of the module's `gas_baseline` (63 steps):

| quantity | value | how |
|---|---:|---|
| library lines (`wc -l`, inline tests excluded) | **160** (159 in `scalar.cairo` + the `pub mod scalar;` line) | `rapier_dynamics2d` 15,087 → 15,247 of 40,000: margin +62.3 % → **+61.9 %** |
| steps of the 1,033 existing `rapier_dynamics2d` tests | **0 differences** (steps and builtins, every test) | before / after `--detailed-resources` runs below, compared test by test |
| `SolverBodies`: `resize(4)`, `set_vel`, `get_vel`, `get_pose` | **313 steps** (376 − 63), 17 `range_check` | `gas_solver_bodies_set_get` |
| `SolverVel`: `+=`, `-=`, `-` | **146 steps** (209 − 63), 21 `range_check` | `gas_solver_vel_sub_add` |
| class sizes | 0 | no declared class reaches the module (a class compiles only what its entry points reach) |

Parity effect, measured by regenerating `docs/API_PARITY.md` with the prototype files present (then restored):
**18 of the 26 items match** (missing 46 → 28 overall, in-scope 97.1 % → 98.2 % with family 2's prototype present,
which matches none, see 2.3). The 8 left are the 5 to close by reason and the 3 operator impls: the matcher did not
take `pub impl SolverVelSub of Sub<SolverVel>` declared outside `SolverVel`'s module.

A lot would declare the `Sub` / `AddAssign` / `SubAssign` impls in `solver/body.cairo`, beside `SolverVel`. Declared in
another module, they must be imported at every use (the prototype's own test needed the import).

### 1.4 Results

1B adds a module and new impls on `SolverVel`. No existing function changes, so no existing result can move. All
1,033 existing tests of the crate pass with the same steps and builtins. Of the prototype's own 4 tests, 3 passed; the
fourth failed on a wrong expected value in the test itself (corrected afterwards, not re-run). 1A changes no Cairo.

### 1.5 Recommendation

**Exclude now (1A)**, in the next parity-script lot (PX8-like, Sonnet, documents + script, ≈ 30 lines): the items are
crate-private upstream, so they are not API. If the programme wants the names anyway, 1B is a Sonnet lot of ≈ 160
library lines (measured) plus tests, off the step, with the 5 items above closed by reason.

## 2. Sub-shape result widening (7 items, parked item (E))

### 2.1 Upstream

Parry 0.31 adds a `subshape: SubShapeId` field to `PointProjection` and `RayIntersection`, and `subshape1` /
`subshape2` to `Contact`, with the builders `with_subshape(s)` (`d604b86`, `e4340dc`). `ContactManifold::subshape_pos1 /
2` and `set_subshape_pos1 / 2` read and write a boxed `Option<Box<SubshapePoses>>` field of the manifold (already in
0.30.2). Rapier reads the poses when it builds solver contacts (`narrow_phase/pair_update.rs`, `prepend_to(co.pos)`).

The port keeps every result at its width (ADR 35) and returns the part index out of band (`*_part` functions);
`ShapeIntersection` and `ShapeDistance` already carry sub-shape ids as separate value types (LO1, PX6); the manifold
does not hold the poses (ADR 36: "the part pose read back from the compound"); `SubshapePoses` exists as plain data
(PX6, ADR 45).

### 2.2 Options

* **2A — widened twin values (recommended).** New types beside the old ones, built by the upstream method names:

  ```cairo
  pub struct SubshapeContact { pub contact: Contact, pub subshape1: SubShapeId, pub subshape2: SubShapeId }
  pub struct SubshapePointProjection { pub projection: PointProjection, pub subshape: SubShapeId }
  pub struct SubshapeRayIntersection { pub intersection: RayIntersection, pub subshape: SubShapeId }
  fn with_subshapes(self: Contact, subshape1: SubShapeId, subshape2: SubShapeId) -> SubshapeContact
  fn with_subshape(self: PointProjection, subshape: SubShapeId) -> SubshapePointProjection
  fn with_subshape(self: RayIntersection, subshape: SubShapeId) -> SubshapeRayIntersection
  ```

  The `*_part` functions that already return `(SubShapeId, T)` gain no twin: a user calls `with_subshape` on their
  answer. A lot would put the three methods in the existing traits (`ContactTrait`, `PointProjectionTrait`,
  `RayIntersectionTrait`) so that the parity matcher finds them by name. The prototype put them in separate traits
  so that no existing `#[generate_trait]` impl changed during the measurement; section 2.3 says what moving them costs.
* **2B — computed sub-shape poses.** `subshape_pos1(self: @ContactManifold, shape1: @Shape) -> Option<Pose2>` (and
  `2`) read the part pose back from the shape, as the narrow phase already does (ADR 36): the compound's part pose;
  `None` for a polyline (upstream's `Polyline::map_part_at` passes no pose), a 2D heightfield (its manifold generator
  sets none) and a simple shape.
  `set_subshape_pos1 / 2` cannot have a faithful form without a manifold field: they go on `SubshapePoses` (plain data)
  or are closed with a reason ("would widen stepped structs", PLAN.md's existing wording for `IndexMut` and
  `user_data`). The signature takes one more argument than upstream: a recorded deviation.
* **2C — the upstream layout (rejected).** A `subshape` field on `PointProjection` / `RayIntersection` / `Contact` and a
  boxed poses field on `ContactManifold`. Every query result and every stored manifold widens: the projection result is
  built in every point query, the manifold is copied and serialised on every step and in `WorldState`. This is the
  cost ADR 35 avoided; it also changes `WorldState`'s felts, so results stay but the state format moves. Not measured:
  it fails the rule by construction.

### 2.3 Cost

Measured on the prototype (`crates/rapier_geometry2d/src/query/subshape.cairo`, uncommitted), steps net of the
module's `gas_baseline` (63 steps):

| quantity | value | how |
|---|---:|---|
| library lines (`wc -l`, inline tests excluded) | **100** (99 + the `pub mod subshape;` line) | `rapier_geometry2d` 23,353 → 23,453 of 40,000: margin +41.6 % → **+41.4 %** |
| steps of the 1,627 existing `rapier_geometry2d` tests | **0 differences** (steps and builtins, every test) | before / after `--detailed-resources` runs, compared test by test |
| steps of the 1,033 existing `rapier_dynamics2d` tests (they run the geometry crate's queries) | **0 differences** | the family 1 runs above had both prototypes declared |
| `Contact::with_subshapes` | **50 steps** (113 − 63) | `gas_contact_with_subshapes` |
| `subshape_pos1` on a 2-part compound | **553 steps** (616 − 63), *including* building the compound (`CompoundTrait::new` computes the part boxes); the read alone is a `match` and one `part_pose` (estimate < 60 steps) | `gas_manifold_subshape_pos1_compound` |
| class sizes | 0 | no declared class calls the new functions |

Parity effect, measured the same way: **0 of the 7 items match** as prototyped, because the matcher looks for
the methods on `Contact` / `ContactTrait`, `PointProjection` / `PointProjectionTrait`, `RayIntersection` /
`RayIntersectionTrait` and `ContactManifold` / `ContactManifoldTrait`, not on the prototype's separate traits. A lot
puts them in those traits, or adds owner aliases to the script.

Moving the three builders into the existing traits (`ContactTrait`, …) adds functions to impls that the step uses. The
measurement above does not cover that layout: a lot must repeat the before / after comparison and add the step-path
probes of `rapier2d` (`gas_scenes`, `level_budget`, `game_path`), as AGENTS.md §7 asks.

### 2.4 Results

2A and 2B add types and functions; no existing type changes width and no existing function changes, so no existing
result can move. All 1,627 existing `rapier_geometry2d` tests and all 1,033 `rapier_dynamics2d` tests pass with the
same steps and builtins; the prototype's 4 tests pass. 2C would not move results either, but it moves the step.

### 2.5 Recommendation

**Port now as one Sonnet lot (SW1, ≈ 100 library lines + ≈ 80 test lines):** 2A in the existing traits, 2B's
getters, `set_subshape_pos1 / 2` on `SubshapePoses` (or closed by reason, the programme's choice). It closes the 7
items. The lot proves "steps unchanged" on the step-path probes before / after, as AGENTS.md §7 asks of any change
that touches a `#[generate_trait]` impl used on the step.

## 3. `contact_skin` (3 items, parked item (F))

### 3.1 Upstream

`Collider::contact_skin` / `set_contact_skin` / `ColliderBuilder::contact_skin(skin)`: a `contact_skin: Real` field
of the collider (default 0). It enters the step in four places (rapier 0.35.3):

1. the broad phase loosens each collider's box by `contact_skin + prediction / 2` (`collider.rs`
   `compute_collision_aabb`, `compute_broad_phase_aabb`, `pipeline/physics_pipeline/substep.rs`);
2. the narrow phase generates the pair's manifolds with `prediction + skin1 + skin2`
   (`narrow_phase/pair_update.rs` ≈ l. 325–354);
3. a manifold point becomes a solver contact when `dist − skin1 − skin2 < prediction` (l. 581);
4. the solver contact's `dist` is that effective distance, so bodies rest `skin1 + skin2` apart.

### 3.2 Where it would enter the port

The same four places are spread over three crates and the stage classes: the box margin in
`rapier_dynamics2d::collider_set` (l. 275), `rapier2d::pipeline` (l. 602), `pipeline/active_set.cairo` (l. 247, 591),
`pipeline/fused_alternatives.cairo`; the pair prediction and the solver-contact selection in the narrow-phase pair
loop. The collider crosses classes packed (ADR 42, 43): a skin would cross too.

### 3.3 Options

* **3A — a boxed collider field (the `one_way` precedent).** `Collider.one_way: Box<Option<OneWayPlatform>>` became a
  box of a wider cold value (`ColliderCold { one_way: Option<OneWayPlatform>, contact_skin: Fixed }`), so `Collider`
  keeps its width. Every step then reads the box of every collider in the four places above. Even when it is `None`,
  that read is new work on every existing world: steps move for everyone (estimate: +10 to +20 Cairo steps per
  collider per step for the box reads and the zero adds, and +2 to +4 per pair). `WorldState` keeps its felts for
  `None` only if the cold value's `Serde` writes the same tag; the one-way codec changes. **Fails the rule.**
* **3B — a `StepConfig` strategy.** A new associated impl `Skin: ContactSkinStrategy` with `NoSkin` (identity hooks,
  `#[inline(always)]`, the default and basic configs) and `ColliderSkin` (reads 3A's cold box). Existing worlds keep
  their code only if the identity hooks compile to nothing in all four places. That holds for an inlined identity on
  the box margin. It is not guaranteed where the hook sits inside a larger expression (AGENTS.md §7, SH1: shifted
  inlining thresholds moved steps). Adding an associated impl to `StepConfig` is also a breaking change for every user
  config (and for the class configs of `rapier2d_classes` and `rapier_sink`). Cost (estimate): ≈ 150–250 library
  lines across `rapier_dynamics2d`, `rapier2d` and the classes; a step-path proof over P3, levels and game-path probes;
  class sizes grow only for a config that selects `ColliderSkin` (`SlimEditStep`, the slim caller, is 68,850 CASM
  felts in `gas/bytecode.size`: 4,878 under 73,728).
* **3C — no port; a closed reason with a recipe.** In 2D the skin is equivalent to a border radius for the gap and the
  prediction: a round shape of border `s` (or a ball of radius `r + s`) gives contacts at `dist − s`, a box loosened by
  `s`, and prediction `prediction + s`. Two differences: the contact points lie on the dilated surface, and the mass
  properties grow (a user sets them explicitly with `ColliderBuilder::mass_properties`). Zero cost; a closed reason
  such as "contact skin: the collider layout and the step stay; a round shape gives the gap" needs a programme
  decision.

### 3.4 Results

3A and 3B do not move results of worlds that never set a skin: the skin is zero and every add is `+ 0`. 3A moves their
steps. 3B moves them only if the identity hooks do not compile away, which must be measured. 3C changes nothing.

### 3.5 Recommendation

**Exclude (3C)** with a closed reason and the round-shape recipe in the collider docs. If a game need appears, 3B
is the route (an `impl-opus` lot, step path, ≈ 200 lines, with the before / after steps proof). Its first measurement
would be whether `NoSkin` really costs 0 steps at the four sites. Not now: there is no consumer, and it is the only
family whose opt-in form still threatens existing steps.

## 4. Compound internal edges (8 + 1 items, parked item (B))

### 4.1 Upstream

Parry 0.31 (`3609fcc`) adds `CompoundFlags` (`FIX_INTERNAL_EDGES`), `Compound::{with_flags, set_flags, flags,
DEFAULT_WELD_TOLERANCE, part_normal_constraints}` and `CompoundPseudoNormals` / `CompoundEdgeCone` (2D: one cone per
boundary edge of each polygonal part, cut edges dropped, cones opened towards the next edge of the union's outline).
`Compound::new` sets **no** flag, so a compound behaves as in 0.30.2 unless the user opts in. The part's constraints
reach the convex–convex manifold generators, which project the contact normal into the cone and drop contacts that
leave it (`contact_manifolds_pfm_pfm.rs` l. 86, l. 129, `contact_manifolds_convex_ball.rs` l. 85). `bvh` is the
ninth `Compound` item missing; it is not part of the family (no BVH, ADR 36), and stays missing or closes with
`CompositeShape::bvh`'s reason.

### 4.2 Options

* **4A — opt-in flags, a constrained composite strategy (recommended when a need appears).**
  * `Compound` gains the flags and `Option<Span<Option<CompoundPseudoNormals>>>`. Its custom `Serde` packs the flags
    into the high bits of an existing felt (the part count), so a compound without flags keeps its `WorldState` felts.
  * `with_flags(shapes, flags, weld_tolerance: Option<Fixed>)` / `set_flags` compute the cones once (off the step).
  * A new `CompositeStrategy` impl (`ConstrainedCompositeManifolds`) passes `part_normal_constraints(i)` to
    constrained copies of `contact_manifold_convex_ball` and `contact_manifold_pfm_pfm`, used through
    `World::step_with::<C>` with a config `C` that selects it.
  * `DefaultStepConfig` keeps `CompositeManifolds`, so `World::step` does not compile the new path.
* **4B — flags in `DefaultStepConfig`'s path.** The existing composite strategy checks `flags` per compound pair.
  Simpler for users (`World::step` honours the flag), but every compound pair pays the check (estimate: +2 to +5
  steps per compound pair per step, the order of ADR 35's +2) and the compound code layout changes. **Fails the rule**
  for existing compound worlds.

### 4.3 Cost of 4A (estimate)

| part | upstream size | port estimate |
|---|---|---|
| outlines, welding, cones (`compound.rs` l. 260–630) | 280 non-blank, non-comment lines | ≈ 300 lines |
| `CompoundPseudoNormals` / `CompoundEdgeCone` + `NormalConstraints` impl (2D part of `compound_pseudo_normals.rs`) | ≈ 100 lines | ≈ 120 lines (on PX6's `LocalNormalProjector`) |
| constrained convex–ball and PFM–PFM manifolds | ≈ 40 lines of upstream branches | ≈ 150 lines (copies, the port's SAT path) |
| flags, `Serde` packing, `ConstrainedCompositeManifolds`, step config | — | ≈ 100 lines |
| tests and golden comparisons | — | ≈ 600 lines (≤ 800 per file) |

About 670 library lines in `rapier_geometry2d` (+42 % line margin: 23,353 → ≈ 24,020, margin ≈ +40 %) and ≈ 50 in
`rapier_dynamics2d` / `rapier2d`. Steps of existing probes: unchanged by construction except for the compound `Serde`
(the flag unpacking: estimate +5 to +10 steps per compound collider in a `WorldState` round trip, measurable and
removable by a flag-free fast path). Opt-in path: the cone projection per contact (≈ 2 cone tests per boundary edge
of the part, estimate 300–800 steps per constrained part manifold). Class sizes: none of today's declared classes
selects the new strategy, so none grows.

Two numeric points need ADR entries. (1) `DEFAULT_WELD_TOLERANCE` is **4 ULPs relative to each corner's magnitude** in
f64; Q32.32 has an absolute resolution, so the port needs a raw-unit reading (e.g. 4 raw, or `4 · 2^-52 · |corner|`
floored). (2) Upstream applies the cone inside its GJK path. The port's polygon contacts are SAT (ADR 17), so the
"normal changed, drop the GJK point" branch (l. 115) has no counterpart and the retain rule (l. 129) needs the SAT
penetration as `dist`.

### 4.4 Does it need the golden oracle bumped to parry 0.31?

The **existing** goldens do not need it: with no flags, 0.31 builds the same compound manifolds. **New** goldens for
flagged compounds need parry 0.31 in the oracle. What that bump moves was measured, not estimated. Two scratch
copies of `tools/golden` were made in the session scratchpad (outside the repository):

* `g030`, today's pins. It reproduces all 33 committed `vectors/*.json` byte for byte (`diff -rq` empty): the
  oracle is deterministic on this host.
* `g031`, the bump:
  * published `rapier2d-f64 0.35.3`, vendored with its parry requirement raised to `0.31.1`;
  * parry `0.31.1` from the tag, with the same one-line f64 cuboid feature-id patch as `tools/golden/vendor`, which
    0.31.1 still needs (`cuboid.rs` l. 168 still shifts by 31 / 30).

The reference clone (`28d0ba92`) is not usable as is: its soft-body API changes `PhysicsPipeline::step` (13 arguments),
`EventHandler`, `ContactPair::manifolds` and `RigidBodySet::remove`, so moving the oracle to it is a larger, separate
change.

Adapting rapier 0.35.3 to parry 0.31.1 took 9 mechanical edits, none numeric. `intersection_test` now returns
`ShapeIntersection` (read `.intersecting`), and `CompositeShapeRef::project_local_point_and_get_feature` returns a flat
3-tuple. The oracle needed 13 call sites adapted the same way (`.intersecting`, `ShapeDistance::distance`).

| vector file (33 in all) | changed values | what moves |
|---|---:|---|
| 30 files: every scene (`scenes`, `level_scenes`, `ccd_scenes`, `composite_scenes`, `compound_scenes`, `tilted_landing`, `sensor_trigger`), every contact family (`contact_manifolds`, `composite_contacts`, `compound_contacts`, `round_shape_contacts`, `triangle_contacts`), every other query family | **0** (byte-identical) | nothing |
| `composite_queries.json` | 39 | polyline / heightfield **feature ids**: point projections and ray hits now report the hit segment's own feature (`Face(0 / 1)`, `Vertex(0 / 1)`) with the segment in `subshape`, instead of a polyline-wide id (`segment_feature_to_polyline_feature`, `curr + num_cells`) |
| `compound_queries.json` | 24 | compound point-projection **feature ids**: the part's own feature instead of `Unknown` |
| `ray_casts.json` | 64, in 9 capsule cases | parry 0.31's analytic capsule ray cast (`3dbc3d0`) replaces GJK: feature `Unknown` → `Face(0)`; normals move by up to 283 raw (`capsule/posed`; 248 on `capsule/hit_cap`); `capsule/inside` hollow time of impact 778,334,551 → 644,245,094 raw (upstream's defect of ADR 0001 entry 4 is fixed); the solid inside casts report a zero normal; `capsule/zero_dir_inside` now hits at `t = 0` |

Times of impact and points do not move anywhere else, and no contact or scene value moves. The bump still moves
existing goldens in three families. The Cairo tests compare features exactly (`composite_queries_golden.cairo`,
`compound_queries_golden.cairo`, `ray_golden.cairo`). The port therefore has two choices:

* follow 0.31's feature ids, which **changes results the port returns today** (the `FeatureId` of polyline,
  heightfield and compound projections and ray hits);
* keep its ids and record them as a divergence of the bumped goldens.

The capsule changes go the port's way (ADR 0001 entries 4 and 5 shrink or close). A bump lot must choose the feature
policy explicitly; the conservative one, which keeps existing results, is to keep the port's ids and compare the
features against a frozen 0.30.2 copy.

A third route avoids moving any existing golden: a **second oracle binary** pinned to parry 0.31.1 for the new
`compound_internal_edges` family only, beside today's oracle. It costs a second dependency set in `tools/golden` (two
parry copies, which the README avoids on purpose) and leaves the API-parity target (0.31.1) and the golden pin
(0.30.2) split for longer.

**Done in OB (2026-10-03).** The oracle runs on parry 0.31.1 (published crate, the cuboid patch kept;
`rapier2d-f64 0.35.3` vendored with its requirement raised and 6 call sites adapted; 13 oracle call
sites). The move reproduces SC2's measurement exactly: 30 files change only their `parry` header line;
`composite_queries.json` 39 values, `compound_queries.json` 24, `ray_casts.json` 64 in 9 capsule cases.
No port result changed. Policy (a) for the three families: each moved field the port does not follow
is compared with a frozen 0.30.2 copy (`tools/golden/vectors/frozen/parry_0_30_2.json`, emitted as
`rapier_golden::generated::frozen_parry030`); every other field is compared with 0.31.1. The
divergences are recorded as ADR 0001 entries 47 (capsule) and 48 (feature ids).

The capsule cases, port against 0.31.1 (port values measured by running the port's cast):

| case | time of impact | normal | feature | other | ADR 4 / 5 |
|---|---|---|---|---|---|
| `hit_side` | equal (only 0.30.2's `f64` digits moved) | equal | port `Unknown`, 0.31.1 `Face(0)` | — | no old divergence |
| `hit_cap` | equal | equal (0.30.2: 248 raw off) | as above | — | 5 closes |
| `oblique` | equal | equal | as above | — | no old divergence |
| `inside` | hollow equal, 644,245,094 (0.30.2: 778,334,551) | hollow equal; solid: port `-dir / \|dir\|`, 0.31.1 zero | as above | — | 4 closes; smaller divergence (solid normal) |
| `inside_cap_exit` | equal | hollow equal; solid: port `(0, -1)`, 0.31.1 zero | as above | — | smaller divergence (solid normal) |
| `unnormalized_dir` | equal | equal | as above | — | no old divergence |
| `zero_dir_inside` | — | — | — | solid: port a miss, 0.31.1 a hit at `t = 0` with a zero normal | new divergence |
| `oblique_shape` | equal (`f64` digits only) | equal (`f64` digits only) | as above | — | no old divergence (raw) |
| `posed` | equal | port 3 / 2 raw from 0.31.1 (0.30.2: 283 raw off) | as above | — | 5 closes |

The capsule ray tolerances drop to the standard 4 / 8 ulp.

**Recommendation on the feature ids (a programme decision, not taken by OB):** follow 0.31.1. Its ids
are the API-parity target; per-segment / per-part ids with the part in `subshape` are what SW1's
`SubshapePointProjection` / `SubshapeRayIntersection` already carry; no contact or scene value moved
with the bump, so the step does not read these ids and following them changes only query results
(`FeatureId` of polyline, heightfield and compound projections and ray hits). The capsule answers of
entry 47 (constant `Face(0)`, zero solid-inside normal, zero-`dir` hit at `t = 0`) are cheap to follow in
the same lot. Each followed family deletes its frozen entries. Keeping the port's ids instead costs
nothing now: the frozen copy stays and entries 47 / 48 stay open.

<details><summary>Every moved value (127 rows)</summary>

| file / table | case | field | 0.30.2 | 0.31.1 | why | compared with |
|---|---|---|---|---|---|---|
| composite_queries/points | `vee/p2` | `feature.kind` | `"face"` | `"vertex"` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `vee/p2` | `feature.code` | `0` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `vee/p3` | `feature.kind` | `"face"` | `"vertex"` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `vee/p4` | `feature.code` | `1` | `0` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `vee/p5` | `feature.kind` | `"face"` | `"vertex"` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `vee/p5` | `feature.code` | `1` | `0` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `bumps/p0` | `feature.code` | `5` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `bumps/p1` | `feature.code` | `3` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `bumps/p2` | `feature.kind` | `"face"` | `"vertex"` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `bumps/p2` | `feature.code` | `4` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `bumps/p3` | `feature.code` | `7` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `bumps/p4` | `feature.code` | `5` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `bumps/p5` | `feature.code` | `5` | `0` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `square/p0` | `feature.code` | `2` | `0` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `square/p1` | `feature.code` | `3` | `0` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `square/p2` | `feature.code` | `0` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `square/p3` | `feature.code` | `1` | `0` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `square/p5` | `feature.code` | `0` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `plain_square/p0` | `feature.code` | `2` | `0` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `plain_square/p1` | `feature.code` | `3` | `0` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `plain_square/p2` | `feature.code` | `0` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `plain_square/p3` | `feature.code` | `1` | `0` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/points | `plain_square/p5` | `feature.code` | `0` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/rays | `flat/down` | `solid.hit.feature.code` | `6` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/rays | `flat/down` | `hollow.hit.feature.code` | `6` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/rays | `flat/slant` | `solid.hit.feature.code` | `6` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/rays | `flat/slant` | `hollow.hit.feature.code` | `6` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/rays | `flat/up` | `solid.hit.feature.code` | `2` | `0` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/rays | `flat/up` | `hollow.hit.feature.code` | `2` | `0` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/rays | `flat/posed` | `solid.hit.feature.code` | `5` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/rays | `flat/posed` | `hollow.hit.feature.code` | `5` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/rays | `hills/down` | `solid.hit.feature.code` | `7` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/rays | `hills/down` | `hollow.hit.feature.code` | `7` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/rays | `hills/slant` | `solid.hit.feature.code` | `7` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/rays | `hills/slant` | `hollow.hit.feature.code` | `7` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/rays | `hills/up` | `solid.hit.feature.code` | `2` | `0` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/rays | `hills/up` | `hollow.hit.feature.code` | `2` | `0` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/rays | `hills/posed` | `solid.hit.feature.code` | `7` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| composite_queries/rays | `hills/posed` | `hollow.hit.feature.code` | `7` | `1` | hit segment's own feature (0.31) vs shape-wide id (0.30.2) | frozen |
| compound_queries/points | `ell/p0` | `feature.kind` | `"unknown"` | `"vertex"` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `ell/p1` | `feature.kind` | `"unknown"` | `"face"` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `ell/p2` | `feature.kind` | `"unknown"` | `"face"` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `ell/p2` | `feature.code` | `0` | `1` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `ell/p3` | `feature.kind` | `"unknown"` | `"face"` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `ell/p4` | `feature.kind` | `"unknown"` | `"face"` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `ell/p4` | `feature.code` | `0` | `1` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `ell/p5` | `feature.kind` | `"unknown"` | `"face"` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `ell/p5` | `feature.code` | `0` | `3` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `trio/p0` | `feature.kind` | `"unknown"` | `"face"` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `trio/p1` | `feature.kind` | `"unknown"` | `"face"` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `trio/p2` | `feature.kind` | `"unknown"` | `"face"` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `trio/p2` | `feature.code` | `0` | `1` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `trio/p3` | `feature.kind` | `"unknown"` | `"face"` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `trio/p4` | `feature.kind` | `"unknown"` | `"face"` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `trio/p4` | `feature.code` | `0` | `1` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `trio/p5` | `feature.kind` | `"unknown"` | `"face"` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `trio/p5` | `feature.code` | `0` | `3` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `mixed/p0` | `feature.kind` | `"unknown"` | `"vertex"` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `mixed/p0` | `feature.code` | `0` | `1` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `mixed/p1` | `feature.kind` | `"unknown"` | `"vertex"` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `mixed/p4` | `feature.kind` | `"unknown"` | `"face"` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `mixed/p4` | `feature.code` | `0` | `1` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| compound_queries/points | `mixed/p5` | `feature.kind` | `"unknown"` | `"face"` | part's own feature (0.31) vs `Unknown` (0.30.2) | frozen |
| ray_casts | `capsule/hit_side` | `solid.toi.f64` | `1.7500000000009786` | `1.75` | analytic vs GJK time, f64 digits only (raw unchanged) | 0.31 (port agrees) |
| ray_casts | `capsule/hit_side` | `solid.hit.time_of_impact.f64` | `1.7500000000009786` | `1.75` | analytic vs GJK time, f64 digits only (raw unchanged) | 0.31 (port agrees) |
| ray_casts | `capsule/hit_side` | `solid.hit.feature.kind` | `"unknown"` | `"face"` | analytic capsule cast reports `Face(0)`; GJK `Unknown` | frozen |
| ray_casts | `capsule/hit_side` | `hollow.toi.f64` | `1.7500000000009786` | `1.75` | analytic vs GJK time, f64 digits only (raw unchanged) | 0.31 (port agrees) |
| ray_casts | `capsule/hit_side` | `hollow.hit.time_of_impact.f64` | `1.7500000000009786` | `1.75` | analytic vs GJK time, f64 digits only (raw unchanged) | 0.31 (port agrees) |
| ray_casts | `capsule/hit_side` | `hollow.hit.feature.kind` | `"unknown"` | `"face"` | analytic capsule cast reports `Face(0)`; GJK `Unknown` | frozen |
| ray_casts | `capsule/hit_cap` | `solid.toi.f64` | `2.2708712152928565` | `2.2708712152928543` | analytic vs GJK time, f64 digits only (raw unchanged) | 0.31 (port agrees) |
| ray_casts | `capsule/hit_cap` | `solid.hit.time_of_impact.f64` | `2.2708712152928565` | `2.2708712152928543` | analytic vs GJK time, f64 digits only (raw unchanged) | 0.31 (port agrees) |
| ray_casts | `capsule/hit_cap` | `solid.hit.normal.f64.0` | `0.4000000580622208` | `0.40000000037252914` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/hit_cap` | `solid.hit.normal.f64.1` | `0.9165151136507351` | `0.916515138828583` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/hit_cap` | `solid.hit.normal.raw.0` | `1717987168` | `1717986920` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/hit_cap` | `solid.hit.normal.raw.1` | `3936402439` | `3936402548` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/hit_cap` | `solid.hit.feature.kind` | `"unknown"` | `"face"` | analytic capsule cast reports `Face(0)`; GJK `Unknown` | frozen |
| ray_casts | `capsule/hit_cap` | `hollow.toi.f64` | `2.2708712152928565` | `2.2708712152928543` | analytic vs GJK time, f64 digits only (raw unchanged) | 0.31 (port agrees) |
| ray_casts | `capsule/hit_cap` | `hollow.hit.time_of_impact.f64` | `2.2708712152928565` | `2.2708712152928543` | analytic vs GJK time, f64 digits only (raw unchanged) | 0.31 (port agrees) |
| ray_casts | `capsule/hit_cap` | `hollow.hit.normal.f64.0` | `0.4000000580622208` | `0.40000000037252914` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/hit_cap` | `hollow.hit.normal.f64.1` | `0.9165151136507351` | `0.916515138828583` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/hit_cap` | `hollow.hit.normal.raw.0` | `1717987168` | `1717986920` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/hit_cap` | `hollow.hit.normal.raw.1` | `3936402439` | `3936402548` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/hit_cap` | `hollow.hit.feature.kind` | `"unknown"` | `"face"` | analytic capsule cast reports `Face(0)`; GJK `Unknown` | frozen |
| ray_casts | `capsule/oblique` | `solid.hit.feature.kind` | `"unknown"` | `"face"` | analytic capsule cast reports `Face(0)`; GJK `Unknown` | frozen |
| ray_casts | `capsule/oblique` | `hollow.hit.feature.kind` | `"unknown"` | `"face"` | analytic capsule cast reports `Face(0)`; GJK `Unknown` | frozen |
| ray_casts | `capsule/inside` | `solid.hit.normal.f64.0` | `-0.8944271909999159` | `0.0` | solid from inside: analytic zero normal; GJK `-dir/\|dir\|` | frozen |
| ray_casts | `capsule/inside` | `solid.hit.normal.f64.1` | `-0.4472135954999579` | `0.0` | solid from inside: analytic zero normal; GJK `-dir/\|dir\|` | frozen |
| ray_casts | `capsule/inside` | `solid.hit.normal.raw.0` | `-3841535534` | `0` | solid from inside: analytic zero normal; GJK `-dir/\|dir\|` | frozen |
| ray_casts | `capsule/inside` | `solid.hit.normal.raw.1` | `-1920767767` | `0` | solid from inside: analytic zero normal; GJK `-dir/\|dir\|` | frozen |
| ray_casts | `capsule/inside` | `solid.hit.feature.kind` | `"unknown"` | `"face"` | analytic capsule cast reports `Face(0)`; GJK `Unknown` | frozen |
| ray_casts | `capsule/inside` | `hollow.toi.f64` | `0.18122013450928892` | `0.14999999990686774` | 0.30.2 hollow unit-mixing defect fixed (ADR 4) | 0.31 (port agrees; 0.30.2 frozen for the defect test) |
| ray_casts | `capsule/inside` | `hollow.toi.raw` | `778334551` | `644245094` | 0.30.2 hollow unit-mixing defect fixed (ADR 4) | 0.31 (port agrees; 0.30.2 frozen for the defect test) |
| ray_casts | `capsule/inside` | `hollow.hit.time_of_impact.f64` | `0.18122013450928892` | `0.14999999990686774` | 0.30.2 hollow unit-mixing defect fixed (ADR 4) | 0.31 (port agrees; 0.30.2 frozen for the defect test) |
| ray_casts | `capsule/inside` | `hollow.hit.time_of_impact.raw` | `778334551` | `644245094` | 0.30.2 hollow unit-mixing defect fixed (ADR 4) | 0.31 (port agrees; 0.30.2 frozen for the defect test) |
| ray_casts | `capsule/inside` | `hollow.hit.feature.kind` | `"unknown"` | `"face"` | analytic capsule cast reports `Face(0)`; GJK `Unknown` | frozen |
| ray_casts | `capsule/inside_cap_exit` | `solid.hit.normal.f64.1` | `-1.0` | `0.0` | solid from inside: analytic zero normal; GJK `-dir/\|dir\|` | frozen |
| ray_casts | `capsule/inside_cap_exit` | `solid.hit.normal.raw.1` | `-4294967296` | `0` | solid from inside: analytic zero normal; GJK `-dir/\|dir\|` | frozen |
| ray_casts | `capsule/inside_cap_exit` | `solid.hit.feature.kind` | `"unknown"` | `"face"` | analytic capsule cast reports `Face(0)`; GJK `Unknown` | frozen |
| ray_casts | `capsule/inside_cap_exit` | `hollow.hit.feature.kind` | `"unknown"` | `"face"` | analytic capsule cast reports `Face(0)`; GJK `Unknown` | frozen |
| ray_casts | `capsule/unnormalized_dir` | `solid.hit.feature.kind` | `"unknown"` | `"face"` | analytic capsule cast reports `Face(0)`; GJK `Unknown` | frozen |
| ray_casts | `capsule/unnormalized_dir` | `hollow.hit.feature.kind` | `"unknown"` | `"face"` | analytic capsule cast reports `Face(0)`; GJK `Unknown` | frozen |
| ray_casts | `capsule/zero_dir_inside` | `solid.toi` | `null` | `{"f64":0.0,"raw":0}` | zero `dir` from inside: analytic hit at t = 0, zero normal; GJK miss | frozen |
| ray_casts | `capsule/zero_dir_inside` | `solid.hit` | `null` | `{"time_of_impact":{"f64":0.0,"raw":0},"normal":{"f64":[0.0,0.0],"raw":[0,0]},"feature":{"kind":"face","code":0}}` | zero `dir` from inside: analytic hit at t = 0, zero normal; GJK miss | frozen |
| ray_casts | `capsule/oblique_shape` | `solid.toi.f64` | `1.0336669234376497` | `1.0336669234385814` | analytic vs GJK time, f64 digits only (raw unchanged) | 0.31 (port agrees) |
| ray_casts | `capsule/oblique_shape` | `solid.hit.time_of_impact.f64` | `1.0336669234376497` | `1.0336669234385814` | analytic vs GJK time, f64 digits only (raw unchanged) | 0.31 (port agrees) |
| ray_casts | `capsule/oblique_shape` | `solid.hit.normal.f64.0` | `0.5547001962628922` | `0.5547001962252288` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/oblique_shape` | `solid.hit.normal.f64.1` | `-0.832050294312735` | `-0.8320502943378437` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/oblique_shape` | `solid.hit.feature.kind` | `"unknown"` | `"face"` | analytic capsule cast reports `Face(0)`; GJK `Unknown` | frozen |
| ray_casts | `capsule/oblique_shape` | `hollow.toi.f64` | `1.0336669234376497` | `1.0336669234385814` | analytic vs GJK time, f64 digits only (raw unchanged) | 0.31 (port agrees) |
| ray_casts | `capsule/oblique_shape` | `hollow.hit.time_of_impact.f64` | `1.0336669234376497` | `1.0336669234385814` | analytic vs GJK time, f64 digits only (raw unchanged) | 0.31 (port agrees) |
| ray_casts | `capsule/oblique_shape` | `hollow.hit.normal.f64.0` | `0.5547001962628922` | `0.5547001962252288` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/oblique_shape` | `hollow.hit.normal.f64.1` | `-0.832050294312735` | `-0.8320502943378437` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/oblique_shape` | `hollow.hit.feature.kind` | `"unknown"` | `"face"` | analytic capsule cast reports `Face(0)`; GJK `Unknown` | frozen |
| ray_casts | `capsule/posed` | `solid.toi.f64` | `0.9189016791293` | `0.9189016791292981` | analytic vs GJK time, f64 digits only (raw unchanged) | 0.31 (port agrees) |
| ray_casts | `capsule/posed` | `solid.hit.time_of_impact.f64` | `0.9189016791293` | `0.9189016791292981` | analytic vs GJK time, f64 digits only (raw unchanged) | 0.31 (port agrees) |
| ray_casts | `capsule/posed` | `solid.hit.normal.f64.0` | `-0.9101796911053848` | `-0.9101797211352216` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/posed` | `solid.hit.normal.f64.1` | `-0.41421362835507836` | `-0.4142135623684758` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/posed` | `solid.hit.normal.raw.0` | `-3909192007` | `-3909192136` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/posed` | `solid.hit.normal.raw.1` | `-1779033987` | `-1779033704` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/posed` | `solid.hit.feature.kind` | `"unknown"` | `"face"` | analytic capsule cast reports `Face(0)`; GJK `Unknown` | frozen |
| ray_casts | `capsule/posed` | `hollow.toi.f64` | `0.9189016791293` | `0.9189016791292981` | analytic vs GJK time, f64 digits only (raw unchanged) | 0.31 (port agrees) |
| ray_casts | `capsule/posed` | `hollow.hit.time_of_impact.f64` | `0.9189016791293` | `0.9189016791292981` | analytic vs GJK time, f64 digits only (raw unchanged) | 0.31 (port agrees) |
| ray_casts | `capsule/posed` | `hollow.hit.normal.f64.0` | `-0.9101796911053848` | `-0.9101797211352216` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/posed` | `hollow.hit.normal.f64.1` | `-0.41421362835507836` | `-0.4142135623684758` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/posed` | `hollow.hit.normal.raw.0` | `-3909192007` | `-3909192136` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/posed` | `hollow.hit.normal.raw.1` | `-1779033987` | `-1779033704` | analytic normal vs GJK search direction (ADR 5) | 0.31 (port agrees) |
| ray_casts | `capsule/posed` | `hollow.hit.feature.kind` | `"unknown"` | `"face"` | analytic capsule cast reports `Face(0)`; GJK `Unknown` | frozen |

</details>

### 4.5 Recommendation

**Port later, as two lots, when a game or the programme needs seamless compound floors:**

1. GB1, the oracle bump (an `impl-opus` lot, tools + goldens, no library change except feature-id expectations; see
   4.4 for what it moves);
2. CE1, 4A (`impl-opus`, numerics on the contact path, ≈ 720 library lines, golden family `compound_internal_edges`).

Until then the 9 items stay missing with today's reason ("parry 0.31 `FIX_INTERNAL_EDGES`, after the golden pin").
`CompoundEdgeCone`'s `dim3-only` exclusion is wrong and should be corrected in the next parity lot.

## 5. Commands

```sh
# upstream visibility (family 1)
grep -n 'mod solver' ~/.cargo/registry/src/index.crates.io-*/rapier2d-f64-0.35.3/src/dynamics/mod.rs   # 44: pub(crate) mod solver;
grep -n 'struct VelocitySolver' ~/git/refs/rapier/src/dynamics/solver/velocity_solver.rs               # 40: pub(crate) struct
# parry 0.30.2 → 0.31.1
git -C ~/git/refs/parry log --oneline v0.30.2..v0.31.1
git -C ~/git/refs/parry diff --stat v0.30.2 v0.31.1 -- src
# golden oracle, today's pins and the bump (scratch copies of tools/golden, outside the repository)
cargo build --release -j 3 && ./target/release/golden vectors      # in g030, then in g031
diff -rq g030/vectors tools/golden/vectors                         # empty
# per-file structural JSON diff g030 → g031 (python, session scratchpad): 3 files differ, listed in §4.4
# steps (prototype modules declared vs not; RAYON_NUM_THREADS=1, through the shared lock shim)
snforge test -p rapier_dynamics2d --tracked-resource cairo-steps --detailed-resources   # before: 1,033 passed
snforge test -p rapier_dynamics2d --tracked-resource cairo-steps --detailed-resources   # after (both modules): 1,036 passed, 1 failed (a prototype test's own expected value)
snforge test -p rapier_geometry2d --tracked-resource cairo-steps --detailed-resources   # after: 1,631 passed; before: 1,627 passed
# test-by-test comparison of `steps:` and `builtins:` (python, session scratchpad): 0 differences in both crates
```
