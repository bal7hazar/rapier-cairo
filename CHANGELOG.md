# Changelog

All crates of the workspace share one version. Alphas carry no API or numeric stability guarantee; every entry says
whether simulation results changed.

## Unreleased

## 0.1.0-alpha.10 — 2026-10-04

Two MINOR result changes ship together so a consumer re-pins once: the fused fixed-point rescales on the step path (FU1)
and the parry 0.31.1 query answers with opt-in compound internal edges (CE).

**Results:** a MINOR result change (FU1, CE). A consumer re-pins once: class hashes, `WorldState` and replay digests,
and the cost sheet. FU1 changes the simulation results in their last bits (golden scenes stay within their bands; no
API, signature or `WorldState` format change); FU1 and CE together: a game re-pins `ActiveSetClass`,
`ContactBallClass`, `ContactPolygonClass`, `ForceEventsClass`, `MassClass`, `NarrowPhaseClass`, `SolverClass` and
`SolveAdvanceClass`. FU1 (#270) moved `ContactPolygonClass`, `SolverClass`, `SolveAdvanceClass` and `NarrowPhaseClass`;
CE (#271) moved `ContactBallClass`, `ContactPolygonClass`, `MassClass`, `NarrowPhaseClass`, `ActiveSetClass` and
`ForceEventsClass`; `IslandsClass` and `BroadPhaseClass` are unchanged. CE moves class hashes but no contact, scene or
`WorldState` value; it moves query answers (polyline, heightfield, compound and capsule), and its compound strategy is
opt-in.

### Added
- Compound internal edges, opt-in (CE, parry 0.31.1's `CompoundFlags::FIX_INTERNAL_EDGES`): `CompoundFlags`, `FIX_INTERNAL_EDGES`, `Compound::{with_flags, set_flags, flags, DEFAULT_WELD_TOLERANCE, part_normal_constraints}`, `CompoundPseudoNormals` / `CompoundEdgeCone` (a `LocalNormalProjector`), and the `ConstrainedCompositeManifolds` composite strategy, which a step selects with its own `StepConfig` through `World::step_with::<C>`: a flagged compound clamps its parts' contact normals to the union's outline, so a body sliding across the cut between two parts no longer catches on it. `DefaultStepConfig` keeps `CompositeManifolds`, so `World::step` does not compile the new path and ignores the flag. A flag-free compound keeps its `WorldState` felts; a flagged one adds its flag and cones (readable by this version on). `DEFAULT_WELD_TOLERANCE` is 4 raw Q32.32 units, absolute (ADR 0001 entries 50 to 52). New golden family `compound_internal_edges` (oracle on parry 0.31.1). No existing result changes.
- Sub-shape result widening (SW1): `SubshapeContact`, `SubshapePointProjection` and `SubshapeRayIntersection` (twin values, in the prelude) built by `ContactTrait::with_subshapes` and `PointProjectionTrait` / `RayIntersectionTrait::with_subshape`; `ContactManifoldTrait::subshape_pos1 / 2` (read the part pose back from the shape) and `set_subshape_pos1 / 2` (on a `SubshapePoses`, ADR 46). Opt-in: no existing type, signature or result changes; the 7 parity items are `ported`.

### Changed
- **Query answers follow parry 0.31.1 (CE), a MINOR result change:** query results move, while contacts, scenes, digests and `WorldState` do not (every contact and scene golden and digest unchanged); the class hashes of six classes move (the compound arm of `Shape`'s Serde). One line per family:
  - polyline point projections (`project_local_point_and_get_feature`): the closest segment's own feature (`Vertex(0 / 1)`, `Face(0 / 1)`; the segment is the `subshape` of `project_local_point_and_get_location_polyline_part` / `SubshapePointProjection`) instead of `Face(segment)`;
  - heightfield ray hits: the cell segment's side, `Face(0)` from above and `Face(1)` from below (the cell is the hit's `subshape`), instead of `Face(cell)` / `Face(cell + num_cells)`;
  - compound point projections: the part's own feature instead of `Unknown`;
  - capsule ray casts: feature `Face(0)` instead of `Unknown`; a solid cast from inside answers a zero normal instead of `-dir / |dir|`; a zero `dir` from inside a solid capsule hits at `t = 0` instead of missing.
  The golden query tests compare these with 0.31.1 and the frozen parry 0.30.2 copy (`tools/golden/vectors/frozen/`, `rapier_golden::generated::frozen_parry030`) is removed; ADR 0001 entries 47 and 48 are closed.
- **Results change (FU1, the fused rescales), a MINOR result change:** simulation results differ in their last bits while no API, signature or `WorldState` format changes (from `0.1.0` on, a change of this kind is a MINOR bump; it ships in `0.1.0-alpha.10`). Four formulas of the step path keep their chains of products in one exact wide sum floored once. Each output is within one ulp of the exact value of its formula (the floored chains were up to 2.5 ulp off). Every golden scene stays within its band, and a game re-pins its digests and class hashes once. The figures are exact Cairo steps per call (scratch probe, net of baseline) and the largest error over 50,000 simulated cases:
  - P1, the contact row solve `impulse − r·(jv + rhs)` (`contact::element::row_impulse`, used by the split sweeps): 2 rescales → 1, ≤ 1 ulp (was 2.46), 55 → 38 steps per row.
  - P2, the contact separation refresh: both anchor transforms and both separations, together with generation's round trip, so a refresh at the build-time poses still gives back the solver contact's distance exactly (SF1). 6 rescales → 2, ≤ 1 ulp (was 2.26), 131 → 72 steps per refresh.
  - P3, the projected mass `dir·(im_sum ∘ dir) + g1·ig1 + g2·ig2` in contact generation: 3 rescales → 1, ≤ 1 ulp (was 2.32), 57 → 32 steps.
  - C5, the manifold update (`rapier_geometry2d` `try_update_contacts`): the stored separation is ≤ 1 ulp (the point's outputs were up to 2.32) and the normal test ≤ 1 ulp (was 2.34). The reprojected offset only feeds the 1e-3 threshold test: ≤ 1 ulp for the stored separation, 1.97 against the unrounded one. 144 → 84 steps per point, 57 → 30 per normal test.
- Narrow phase (FU1 B1): each solver-contact anchor folds the body's world centre of mass into the wide sum of its point transform (`floor(x) - c = floor(x - c)` for an exact `c`), which drops one checked `Vec2` subtraction: −8 Cairo steps per anchor (scratch probe), two anchors per solver contact and tick. Results bit-identical.
- Golden oracle (OB): `tools/golden` runs on parry 0.31.1 (vendored, with `rapier2d-f64 0.35.3` raised to it) instead of 0.30.2. The vectors moved in three files (127 values: polyline, heightfield and compound feature ids, and 9 capsule ray casts); the fields the port does not follow are compared with a frozen 0.30.2 copy (`rapier_golden::generated::frozen_parry030`, ADR 0001 entries 47 and 48), the capsule times of impact and normals with 0.31.1 (ADR 0001 entries 4 and 5 closed). No library code, results unchanged.
- API parity (PX8): the project manager's SC2 decisions of 2026-10-03 as table rules: 26 scalar items of the solver (`dynamics::solver` is `pub(crate)` upstream) and the 3 `contact_skin` items closed with two new reasons, `CompoundEdgeCone` (parry 0.31's 2D type) back to `missing` for lot CE; 46 → 18 `missing`. Table and script only, no library code, results unchanged.

## 0.1.0-alpha.9 — 2026-10-03

The toolchain moves to Scarb 2.20.1 / snforge 0.64.0, the engine's step gets cheaper (EL1, CX3), and the API parity
surface grows (PX3 to PX7, CP3, IG1) off the step path.

**Results:** bit-identical since `0.1.0-alpha.8` (the toolchain bump, the slim crossings and the step levers change no
result), and Cairo steps lower with EL1 and CX3, partly offset by the toolchain bump (see Changed).

### Added
- `ColliderSetTrait::get_field` (EL1, #259): reads one field of a collider by handle without copying the collider.
- API parity (PX7): the project manager's decisions of 2026-10-03 as table rules: three matcher gaps now `ported` (`is_bouncy`, `NEW_CONTACT_BIT`, `ShapeDistance::from`), 25 items closed with four new reasons (no `IndexMut`, no faithful `Default` for the contact-graph order, no faithful solver-contact form, V-HACD / voxelisation) or the existing closed-enum one; 74 → 46 `missing`. Table and script only, no library code.
- API parity (PX6): `NormalConstraints` / `NormalConstraintsPair` (value style, `LocalNormalProjector` is the required
  method) with `SegmentPseudoNormals` and the new `TrianglePseudoNormals` as projectors (`project_into_cone` in Q32.32),
  `ShapeDistance`, `SubshapePoses` (plain data), `is_bouncy`, `SolverContactGeneric` / `contact_indices` (one-lane alias);
  the main ones in the prelude. No result changed; the 74 items still `missing` are parked families (sub-shape widening,
  `contact_skin`, composites, the solver's scalar API, V-HACD) or need a table rule (see the PX6 report).
- `ImpulseJointSet::map_attached_joints_mut` (PX5, #242): the user's closure is called on a copy of each joint attached
  to a body and returns the edited joint, written back (Cairo closures have no `&mut`).
- Cheap API items off the step path (PX4, #240): `Index` on `ColliderSet` / `RigidBodySet` (`set[handle]`),
  `ColliderSet::take_modified` / `ModifiedColliders` (a read, never a drain), `RigidBodyIds`, `RigidPairContacts` /
  `PairContacts` / `ContactId` over `ContactPairView`, `ImpulseJointSet::joint_graph` (an `InteractionGraph` of the
  joints), `RayCast` for `Polyline` / `HeightField` and `PointQueryWithLocation` for `HeightField`,
  `QueryPipelineMut::as_ref`, the value meanings of the dyn-shape API (`ShapeDynTrait::{new, as_shape, clone_box,
  clone_dyn, scale_dyn, ccd_thickness, ccd_angular_thickness, convex_polyline_unmodified}`),
  `ConvexPolygonTrait::from_convex_polyline_unmodified`, `contact_manifold_pfm_pfm_shapes`, `Unsupported`,
  `DefaultBroadPhase`, `RigidBodyGraphIndex`; the main ones in the prelude.
- A read-only `InteractionGraph` view over the step's pair list (IG1, #238; off the step path): `Default`, `new`,
  `interactions`, `interactions_with_endpoints`, `interaction_pair`, `interactions_between`, `interactions_with`,
  `index_interaction`; `NarrowPhase::{contact_graph, intersection_graph}`; `ColliderGraphIndex` /
  `TemporaryInteractionIndex` are indices into the step's pair list, valid until the next step; in the prelude.
- The contact-pair read API (CP3, #235; off the step path): `ContactPairView` (one per collider pair, the manifolds
  of a composite run gathered) with `manifolds`, `solver_manifolds`, `rigid`, `total_impulse`,
  `total_impulse_magnitude`, `max_impulse`, `find_deepest_contact`, `clear`; `NarrowPhase::{contact_pairs,
  contact_pairs_with, contact_pairs_with_unknown_gen, contact_pair_unknown_gen, contact_pair_at_index,
  contact_pair_view}`; `World::{contact_pairs, contact_pairs_with}`; in the prelude.
- `crates/rapier2d/tests/finite_state.cairo`: Q32.32 state cannot become NaN or infinite (a tiny mass and a huge
  impulse panic with `'Fixed: overflow'`, a small mass stays finite) — the reason upstream's `Quarantine` is excluded.
- Parry geometry utilities in `rapier_geometry2d` (PX3, #231; off the step path): `Aabb::{distance_to_origin,
  project_on_axis, scaled_wrt_center, canonical_split, clip_line, clip_line_parameters, clip_ray, clip_ray_parameters,
  clip_segment, clip_polygon, clip_polygon_with_workspace}`, `clip_aabb_line`, `clip_halfspace_polygon`,
  `closest_points_line_line{,_parameters,_parameters_eps}`, `local_point_projection_on_support_map`,
  `convex_polygon_area_and_center_of_mass`, `Segment::{canonical_split, local_split, local_split_and_get_intersection,
  from_array}`, `SplitResult`, `IntersectResult`, `Ball` / `Capsule` / `ConvexPolygon::scaled` (a polygon of at most 8
  vertices, `None` beyond), the `BoundingSphere` point and ray queries, `PolygonalFeature::{face_face_contacts,
  face_vertex_contacts}`; golden family `geometry_utils` (254 cases). Parity raw 80.8 %, in scope 88.1 %.
- `rapier2d::prelude` exports `RigidBodyType` / `RigidBodyTypeTrait`, `ShapeTrait` (the `SharedShape`-style
  constructors) and the basic shapes with their traits (`Ball`, `Cuboid`, `ConvexPolygon`, `HalfSpace`, `Capsule`,
  `Segment`): a game builds a `rapier2d_classes::BodyInsert` without depending on `rapier_core` or
  `rapier_geometry2d` (programme request after slingfall's alpha.8 bump).

### Changed
- The engine's step levers (EL1, #259): R1 (`remove_body` wakes and releases a collider's pairs in one walk), F1 (the
  force-event pass keeps the pair list when no status bit changes and reads three collider fields instead of whole
  colliders) and W1 (`body_status` reads the slot's entry before walking), all bit-identical. The pile10 whole shot is
  −1.58 % Cairo steps for the owner's shot and −1.91 % for the reference shot in process, −1.17 % and −1.32 % in the
  slim layout; `steps_game_step` 2,758,024 → 2,726,098 (`docs/BUDGETS.md`). A game re-pins `IslandsClass` and
  `ForceEventsClass` (their class hashes change); the other classes are unchanged.
- Toolchain Scarb 2.20.1 (Cairo 2.20.0) / starknet-foundry 0.64.0 (TC1, #250; was Scarb 2.19.4 / snforge 0.61.0):
  results bit-identical, Cairo steps game path +1.25 to +1.38 %, `rapier2d` probes up to +1.84 % (median +1.33 %), `rapier2d_classes` up to +3.78 % (median +1.61 %) (`docs/BUDGETS.md`), class hashes re-pinned from
  CI's artefact; a consumer on another toolchain re-declares.
- `NarrowPhaseClass` runs its own pair loop on the previous pairs as they cross (CX3, #253): the slim layout's whole
  shot is −3.53 % Cairo steps for the owner's shot and −3.85 % for the reference shot, bit-identical; a game re-pins
  `NarrowPhaseClass` only (the caller and every other class are unchanged).
- CI: the scarb / snforge downloads are retried (#252) and a new push cancels the superseded run of a pull request
  only, never a run on `main`; `scripts/prepush.sh` and its `.githooks/pre-push` hook run what CI would reject before a
  push (PP1, #251).

## 0.1.0-alpha.8 — 2026-09-29

**Results:** step results unchanged since `0.1.0-alpha.7`: every `steps_*` probe (the game-shaped path included),
every golden and scene test, every `program.*` and the full `WorldState` codec (serialized felts and Cairo steps of its
round trips) identical. The basic `WorldState` codec writes and reads the same felts; its round trip costs +5,065 Cairo
steps (outlined readers, CS7).

### Notes
- The slim layout (`SlimSplitStages`): caller 67,076 CASM felts (6,652 under 73,728), pile10 reference shot +36.9 %
  Cairo steps over the in-process step, 4 transactions of ≤ 10M. The classes to declare and their sizes are in
  `rapier2d_classes`' README; `scripts/bytecode_size.py check` keeps each of them at least 1,000 felts under 73,728.
- Class hashes change with this release (type paths of `glam_core`, CX2, CS7): games re-declare every class.

### Changed
- Depends on `fixed` 0.4.0 and `glam_core` 0.4.1 (was `fixed` 0.3.0 and `glam` 0.3.0); step results, gas and felt
  counts unchanged; class hashes change (type paths), consumers re-declare; a consumer must itself be on `fixed` 0.4
  and `glam` ≥ 0.4.1 (or `glam_core`), otherwise it holds two generations of the same types. `rapier2d::prelude`
  still re-exports `Vec2` and `Fixed` (DU1, #225).
- `rapier2d_classes` (CS7, #226): the slim caller 73,204 → 67,076 CASM felts (outlined basic-codec readers, the mass
  crossing written by the basic collider writer), the slim layout's pile10 shot 30.77M → 30.71M Cairo steps (+36.9 %
  over in process, 4 transactions of ≤ 10M); `ContactPolygonClass` is no longer called by the slim layout (CX2 moved
  the polygon family into `NarrowPhaseClass`). Bit-identical; the basic codec writes and reads the same felts.
- `rapier2d_classes` (CX2, #222): `NarrowPhaseClass` computes the polygon-family contacts itself (68,372 CASM felts)
  and previous pairs cross trimmed (`PreviousPair`); the slim layout's pile10 shot 32.97M → 30.77M Cairo steps (+37.2 %
  over in process, 4 transactions of ≤ 10M), its caller 73,204 CASM felts. `NarrowPhaseClass::compute_contacts` has new
  arguments: games re-declare the class. Bit-identical.

### Added
- `WorldEditClass` (CS7, #226): the World edits between steps (insert a body with its collider and velocities, remove
  bodies, put bodies to sleep) as a declared class, 51,865 CASM felts; `edit_world` takes its class hash as a parameter
  (`ClassHashes` is unchanged). A caller that forwards the edits is 68,818 CASM felts (the same edits compiled in the
  caller: 96,657).
- Outlined readers of the basic `WorldState` codec (`world::basic_state::decode`; ADR 0001 entry 44).

### Removed
- From `rapier2d_classes`: `OrchestratorClass` (CS6's rejected route (b)) and the other measured-only classes, now
  unpublished fixtures of `rapier_sink` (CS7, #226). A game declares only the classes the crate's README lists.

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
