# Changelog

All crates of the workspace share one version. Alphas carry no API or numeric stability guarantee; every entry says
whether simulation results changed.

## 0.1.0-alpha.7 — 2026-09-28

**Results:** step results unchanged since `0.1.0-alpha.6` (every step probe and `program.*` identical); shape-cast
results changed for touching starts (CN1, below).

### Notes
- The slim caller path (`rapier2d_classes`, `SlimSplitStages`) steps the worlds of `BasicStepConfig` only: balls,
  cuboids, convex polygons and half-spaces, no sensor, composite or impulse joint, no position-based kinematic body;
  its caller class is 73,083 CASM felts (645 under the 73,728 limit), the pile10 reference shot +47.0 % Cairo steps
  over the in-process step in 4 transactions of ≤ 10M. The in-process step (`World::step*`) is unchanged.
- Package size (owner's rule, 2026-09-28; `docs/research/package-cost.md`): every published crate ≤ 40,000
  library lines (largest `rapier_geometry2d`, 22,013), marginal cost of each crate ≤ 2.3 s / 0.47 GB, closures
  `rapier2d` 8.9 s / 1.85 GB and `rapier2d_classes` 9.0 s / 1.95 GB (cold, local). `rapier2d_classes` is published
  from this release on.
- Correction to `0.1.0-alpha.6`'s notes: the "−1,612 Cairo steps per step" of `BasicStepConfig` was measured on the P3
  contact probes (`balls8`, `stack5`, `stack10`); the saving scales with the contact pairs — the game measures −551 on
  a flight step (11,918 → 11,367) and −6,853 on a pile10 impact step (398,167 → 391,314).

### Changed
- **Query results (numeric change; the step and CCD are unchanged):** a linear shape cast between support-map shapes
  that starts touching a face, or within `target_distance` of it (every answer with `t < 1e-4` that reports the
  contact geometry, on both `stop_at_penetration` paths), now reports the face's exact normal as upstream, where the
  start witness normal was tilted by the rounding of the closest pair (10 to 20 raw for a ball on a floor, up to 900
  for two boxes 0.01 apart, 327,680 for two boxes 2^-16 apart). Its witnesses move along the new normal by up to the
  same amount times the radii; times of impact and statuses are unchanged. `KinematicCharacterController::move_shape`
  follows: a move exactly parallel to the ground no longer counts as a ground hit (`character_moves/wall_slide` now
  matches rapier-rs: one collision fewer, `1e-4` lower), and its golden's worst error drops from 900 to 3 raw. A corner
  just past the end of a face keeps its corner normal (the face snap forgives an excess of at most `height / 2^30`).
  Touching starts cost +670 Cairo steps per cast; other casts are unchanged (CN1).
- Not breaking: `StepConfig` keeps its `0.1.0-alpha.6` shape; the stage slots CS5 added to it (#213) live in a separate
  `StageConfig` since CS6 (#215), taken by `World::step_with_stages` / `step_with_force_events_with_stages`
  (`InProcessStages<C>` is the in-process default).

### Added
- Compact crossings (CX1, #217): `SolveAdvanceClass` 58,547 and `IslandsClass` 19,083 CASM felts; per call 37,960 →
  ≈ 14,256 and 13,741 → 2,615 Cairo steps; the slim layout's pile10 shot 37.22M → 32.97M steps (+47.0 % over in
  process, 4 transactions of ≤ 10M), its caller 73,083 CASM felts; bit-identical, incl. force events every step and
  basic-codec save / restore mid-collapse.
- The slim caller (CS6, #215): `BasicWorldState` / `from_basic_state` / `into_basic_state` (the basic `WorldState`
  codec: the same felts as `WorldState` v3, other shapes and used joint arenas rejected), `NarrowPhaseClass`,
  `ActiveSetClass`, `ForceEventsClass`; a caller class stepping the game's worlds is 73,181 CASM felts (≤ 73,728), the
  pile10 shot +65.9 % Cairo steps in 5 transactions of ≤ 10M; in-process steps and every `program.*` unchanged.
- Stage classes in `rapier2d_classes` (`SolveAdvanceClass`, `IslandsClass`, `BroadPhaseClass`, `MassClass`, a batched
  contact entry point; each ≤ 73,728 felts) and configurations that library-call every stage; bit-identical to
  `BasicStepConfig` over the pile10 shot (CS5, #213).
- `rapier2d_classes` (new crate): the step's contact generation and island solve as declared Starknet classes
  (`ContactBallClass` 39,561, `ContactPolygonClass` 54,634, `SolverClass` 43,726 CASM felts, each ≤ 73,728),
  library-called at the constant hashes of a `ClassHashes` impl through `SplitStepConfig<H>`; bit-identical to
  `BasicStepConfig` over a 151-tick pile10 shot, +22.8 % Cairo steps there (CS4, #211).
- The island solver's input and output derive `Serde` and are `pub` (a Cairo-only extension for the declared classes;
  no logic change, in-process steps and `program.basic` unchanged; ADR 0001 entry 39).
- Controllers (`rapier2d::control`, in the prelude): `PdController`, `PidController`, `PdErrors` and the kinematic
  character controller (`KinematicCharacterController::move_shape` → `EffectiveCharacterMovement` + `CharacterCollision`s,
  `solve_character_collision_impulses`, `CharacterLength`, `CharacterAutostep`) on the shape casts; nothing of it
  enters a `BasicStepConfig` program or moves a step (#202).
- `ShapeTrait` constructors, as upstream's `SharedShape::*`: `ball`, `cuboid`, `capsule`, `capsule_x`, `capsule_y`,
  `segment`, `halfspace`, `triangle`, `round_cuboid`, `round_triangle`, `convex_hull`, `round_convex_hull`,
  `convex_polyline`, `round_convex_polyline`, `polyline`, `heightfield`, `compound` (#204).
- `rapier2d::prelude` exports the body field selectors `BodyPose`, `BodySleeping`, `BodyLinvel`, `BodyAngvel` (for
  `RigidBodySetTrait::get_field`).

## 0.1.0-alpha.6 — 2026-09-27

**Results: numeric change** (a MINOR bump by the versioning policy once out of alpha): contacts whose gap closes exactly
now solve rigidly as upstream (SF1, below), so worlds with resting or landing contacts give different — closer to
rapier-rs — trajectories than `0.1.0-alpha.5`; regenerate goldens. Cairo steps −0.20 % to +0.07 % from it; composite
pairs pay +170 to +235 steps per step (≈ 0.15 %).

### Measured
- `WorldState` round trips of the old shapes cost ≈ +15 Cairo steps per shape more than at `0.1.0-alpha.4`: the
  price of the 13-variant `Shape` enum, not of the dispatch shape (four candidates tie, the variant order does not
  matter); serialized bytes unchanged (#195).

### Changed (numeric)
- Contact generation rebases each frozen contact separation on the floored round trip of its anchors, as upstream
  rebases its separation exactly: an exactly closed gap no longer comes out ≈ 1 raw negative and takes the soft side
  of the `dist <= 0` switch. Fixes the drift on tilted landings and slides (`box_slope_slide` step 4: 26 184 → 5 ulps;
  `ell_topple`, `tilted_landing/ell_twin` within bands); force events unchanged on the game's reference shot (#191).

### Performance (Cairo steps)
- Force-event worlds step below `0.1.0-alpha.4` again: `0.1.0-alpha.5`'s force-event collect paid a composite-group
  check on every pair (+1.5 % on the game's cases, not announced in alpha.5's notes); the collect now takes the
  alpha.4 loop and hands off to the group-aware version at the first composite pair — game-shaped probe −0.11 % vs
  alpha.4, results bit-identical (#189).

### Added
- Configurable step: `WorldTrait::{step_with, step_with_force_events_with, step_with_ccd_with,
  step_with_ccd_and_force_events_with}::<C: StepConfig>`; `DefaultStepConfig` (what `World::step` & co. use) and
  `BasicStepConfig` (ball / cuboid / convex polygon / half-space, no sensors / composites / joints: a game program
  −59.1 % smaller, bit-identical results, −1,612 Cairo steps per step); a world using a disabled feature panics at the
  step that meets it (ADR 0001 entry 37) (#193).
- `Compound` shapes (posed convex parts) with every query, per-part contact manifolds against every supported shape,
  mass properties (`MassPropertiesTrait::from_compound`), CCD, `ColliderBuilder::compound` (ADR 0001 entry 36) (#187).

## 0.1.0-alpha.5 — 2026-09-27

**Results:** `World::step` is unchanged since `0.1.0-alpha.4` (same results, same Cairo steps).

### Breaking
- `WorldState` is **version 3** (the bodies' cold data gained a CCD slot; `user_data` moved into it): version-2 states
  panic with `'world state: version'` (#182).

### Added
- `AabbTrait::{aligned_intersections, intersects_moving_aabb}`, `MotorModelTrait::combine_coefficients`,
  `NarrowPhaseTrait::{intersection_pair_unknown_gen, intersection_pairs_with_unknown_gen}` (#178); `MotorModelTrait` in
  the prelude.
- Shape casts: `cast_shapes` (`ShapeCastOptions` / `ShapeCastHit` / `ShapeCastStatus`), `cast_shapes_nonlinear`
  (`NonlinearRigidMotion`), the swept TOI (`Sweep`, `ToiProxy`, `sweep_time_of_impact`) for every pair of the closed
  set, `World` / `QueryPipeline::cast_shape(_nonlinear)` (ADR 0001 entries 30–32) (#180).
- Continuous collision detection: `World::step_with_ccd(_and_force_events)(ref CCDSolver)`, `RigidBodyCcd`,
  `RigidBodyBuilder::{ccd_enabled, soft_ccd_prediction}`, `enable_ccd` / `is_ccd_enabled` / `is_ccd_active`; upstream's
  automatic CCD for fast bodies is off by default (`CCDSolverTrait::set_automatic`, ADR 0001 entries 33–34). A
  `CCDSolver` serializes only its switch: recreate it with the same setting after `from_state` (#182).
- Composite shapes: `Polyline` (flags, oriented polylines, pseudo-normals) and the 2D `HeightField` as new `Shape`
  variants with every query (per-part sub-shape ids through `*_part` functions), contact manifolds per part against
  every convex shape (events and force events per collider pair), shape casts and CCD against them,
  `ColliderBuilder::{polyline, polyline_with_flags, oriented_polyline, heightfield}` (ADR 0001 entry 35). Existing
  pairs pay +2 Cairo steps per pair per step for the composite group check (#184).

## 0.1.0-alpha.4 — 2026-09-26

**Results:** the step is unchanged since `0.1.0-alpha.3` for the existing shapes: every golden vector and scene test
gives the same results in the same Cairo steps (Sierra gas of existing probes ≤ +0.3 %, from the new `Shape` arms).
`WorldState` unchanged (version 2; states holding only the old shapes serialize as before). Contract class sizes grew
with the new shapes (the class-size path is closed for the MVP, `docs/research/class-size.md`).

### Breaking
- `QueryFilter.flags` is a `QueryFilterFlags` (was a `u32`); the `EXCLUDE_*` / `ONLY_*` constants are typed
  `QueryFilterFlags`, and `QueryFilterTrait::from_flags` takes one (#163).

### Added
- Parry's shape-pair queries for the closed shape set: `rapier_geometry2d::query::{distance, closest_points, contact,
  intersection_test}`, `PointQuery` / `RayCast` / `PointQueryWithLocation` impls, `Aabb` point and ray queries,
  manifold utilities; a parry defect fixed in `contact_support_map_halfspace` (ADR 0001 entry 26) (#152, #155).
- Joint API: typed joints (`FixedJoint`, `RevoluteJoint`, `PrismaticJoint`, `PinSlotJoint`, `RopeJoint`,
  `SpringJoint`) as views over `GenericJoint`, accessors / setters / builders, `ImpulseJointSet` queries,
  `WorldTrait::{impulse_joints, impulse_joints_with, set_impulse_joint, set_impulse_joint_bodies}` (#157, #158).
- Shape helpers: `Aabb` members, `BoundingSphere` / `BoundingVolume`, `SupportMap`, `PolygonalFeatureMap`, feature-id
  helpers, per-shape bounding volumes, `MassProperties` members (#160, #161).
- World scene queries: `intersect_shape`, `project_point_and_get_feature`, `intersect_aabb_conservative`,
  `QueryPipeline` / `with_filter`, `QueryFilterFlags`, `QueryFilter::exclude_solids` (#163).
- API polish: `AxesMask`, `BodyStatus`, `IntegrationParametersTrait::set_dt`, `ShapeTrait::is_convex`, `Into<Shape>`
  for every shape, `CapsuleTrait::{rotation_wrt_y, transform_wrt_y, canonical_transform}`, `SegmentTrait::point_at`,
  `SegmentPseudoNormals`, `ConvexPolygonTrait::{from_convex_hull, offsetted}` (#167).
- Parity leftovers: `RigidBodyActivationTrait::default_*` thresholds, `Default` for `RigidBodyChanges` / `ActiveEvents`,
  `Into<RigidBodyPosition>` from `Pose2`, `RigidBodyColliders`, `ShapeIntersection`, parry's `intersection_test_*`
  entry points (#170).
- Shapes: `Triangle` and round shapes (`RoundShape` over a cuboid, a triangle or a convex polygon) as new `Shape`
  variants with every query and contact manifolds against every shape, `ColliderBuilder::{triangle, round_triangle,
  round_cuboid, round_convex_hull, round_convex_polyline, convex_polyline}`, golden families from parry2d-f64; the
  existing shapes' Cairo steps are unchanged (ADR 0001 entries 27–29) (#174).

## 0.1.0-alpha.3 — 2026-09-26

**Results:** bit-identical to `0.1.0-alpha.2` on every golden vector and scene test (the level impact windows are
pinned by state digests). **`WorldState` is unchanged (version 2)**: states saved with alpha.2 load as they are.

### Performance (Cairo steps)
- Narrow phase, constraint generation and per-substep solve: −18.8 % / −16.3 % on the level-10 / level-20 impact
  windows, −12 % to −13 % on the load windows, −5.5 % to −15.1 % on the contact benchmark scenes (#146).
- Pipeline glue, mixed ticks and arena reads: −2.8 % / −9.0 % more on the level-10 / level-20 impact windows, −1.2 % to
  −3.8 % on every benchmark scene (recovering alpha.2's +1–3 %); `WorldTrait::{is_sleeping, linvel, angvel}` 146 → 88
  steps per read (#151). The level-10 impact tick is ≈ 360k steps (812k before alpha.2's BT1).

### Added
- `rapier_core`: `ArenaField`, `ArenaFieldTrait::get_field` (read one component of an entry without copying it),
  `ArenaStateTrait::{is_modified, clear_modified, mark_modified, set_untracked}` (#151, #153).
- `rapier_dynamics2d`: `RigidBodySetTrait::{get_field, set_internal, mark_modified}` with the field selectors
  `BodySleeping`, `BodyLinvel`, `BodyAngvel`, `BodyPose`; `ColliderSetTrait::{set_internal, mark_modified}` (#151).
- `rapier_dynamics2d::solver::island::solve_island_input` with `SolverInput`, `SolvedIsland`, `ManifoldImpulses`,
  `PointImpulses`: the entry point the pipeline now uses (no `DenseBodies` round trip); `solve_island` is unchanged
  (#151).

## 0.1.0-alpha.2 — 2026-09-26

**Results:** bit-identical to `0.1.0-alpha.1` on every golden vector and scene test (the level impact windows are
pinned by state digests), except for the two upstream-alignment changes listed under *Changed*.

### Breaking
- `WorldState` is **version 2**: it now carries the persistent active set. `WorldTrait::from_state` panics with
  `'world state: version'` on a version-1 state; save states again with this version (#143).
- `rapier2d::pipeline::sleeping::wake_removed_partners` is removed (#135).

### Performance (Cairo steps)
- Awake contact tick: split contact sweeps, −41.0 % / −37.5 % on the level-10 / level-20 impact windows, −32 % to
  −46 % on the contact benchmark scenes (#139).
- Sleeping bodies cost nothing per tick: a persistent active set walks only the awake bodies; the all-asleep engine step
  of a 10-block level goes from 36,141 to 1,000 steps, its flight tick from 60,230 to 16,489 (#143). World loading
  +4–5 %, the small benchmark scenes +1–3 %.

### Added
- `WorldTrait::{is_sleeping, linvel, angvel}`: activation and velocity reads without copying the body (#143).
- Collider API and world facade (#135): `ColliderTrait::{compute_collision_aabb, compute_broad_phase_aabb,
  compute_swept_aabb, copy_from}`, `ColliderBuilderTrait::{convex_hull, position_wrt_parent, delta, default_density,
  default_friction}`, `ColliderSetTrait::{set_parent, iter_enabled, with_capacity, invalid_handle, get_pair_mut}`,
  `WorldTrait::{active_bodies, num_active_bodies, wake_up_all, rigid_bodies, all_colliders}`, `CollisionPipeline`
  (broad and narrow phase and events without the solver), `PhysicsWorld` alias; prelude re-exports of `PhysicsWorld`,
  `CollisionPipeline(Trait)`, `PhysicsPipeline(Trait)`, and `ColliderPair(Trait)` in `rapier_geometry2d` (#137).

### Changed (upstream alignment)
- A body built `.sleeping(true)` stays asleep when it is inserted: a collider inserted since the last step wakes
  nobody, as in rapier-rs (a pile whose bodies touch still wakes at its first step through contact start, as upstream)
  (#143).
- Removing a collider ends its pairs at the next step with `Stopped | REMOVED` (`| SENSOR`) without waking its sensor
  partners, as upstream (#135).

## 0.1.0-alpha.1 — 2026-09-25

First publication on scarbs.xyz: `rapier_math`, `rapier_core`, `rapier_geometry2d`, `rapier_dynamics2d`, `rapier2d`
(#134). 2D rigid bodies (dynamic, fixed, kinematic), colliders (ball, cuboid, capsule, segment, half-space, convex
polygon), sensors and collision / contact-force events, one-way platforms, fixed / revolute / prismatic / rope /
spring joints with limits and motors, the soft-contact substep solver, sleeping, ray / point / AABB queries, and a
versioned `WorldState` save / restore. Validated against golden vectors from rapier2d-f64 0.35.3 / parry2d-f64 0.30.2;
deliberate divergences in `docs/adr/0001-upstream-divergences.md`.
