# 01 — Architectural analysis of `rapier` (input for `rapier.cairo`)

Source analysed: `dimforge/rapier` @ `28d0ba9` ("feat: add support for soft-bodies (#1010)"),
workspace version **0.35.3**, against `parry` **0.31.1**. All paths below are relative to the
rapier repo root unless prefixed with `parry/`. Line counts come from `wc -l` on that commit.

> Heads-up for readers who know older Rapier (<= 0.2x): this version is materially different.
> Math is **glam-based** (`glamx`), the broad phase is a **BVH** (SAP is gone), the solver is a
> **single "staged" substepping soft-constraint solver** (the old `IslandSolver`/`VelocitySolver`
> split, the separate "parallel" solver, `inv_principal_inertia_sqrt`, exponential damping, etc.
> are gone), islands are **persistent** (incremental union / deferred split), and a very large
> **soft-body** subsystem (~26k lines) was just merged. A Cairo port should treat this file as
> the reference, not blog posts or older docs.

---

## 1. Workspace layout, feature flags, public API

### 1.1 One source tree, four crates

```
Cargo.toml                 workspace (members: crates/rapier{2d,3d}{,-f64}, testbeds, examples, loaders, python)
src/                       THE engine source (79,876 lines of Rust)
crates/rapier2d/Cargo.toml [lib] path = "../../src/lib.rs", required-features = ["dim2","f32"]
crates/rapier2d-f64/...    same src, features dim2+f64
crates/rapier3d/...        same src, features dim3+f32
crates/rapier3d-f64/...    same src, features dim3+f64
crates/rapier2d/tests/     integration/regression tests (issue_XXX_*.rs, ccd_*.rs)
src_testbed/               kiss3d/egui testbed shared the same way
examples2d/ examples3d/    scenes (also used as benchmarks: b2d_*, b3d_*, s2d_* = Box2D/Solver2D ports)
```

(`ARCHITECTURE.md` still says `build/`; the directory is now `crates/`.)

Dimension/precision selection is purely `#[cfg(feature = ...)]`:

- `src/lib.rs` picks the parry flavour: `pub extern crate parry2d as parry` (`dim2`+`f32`), `parry2d_f64`,
  `parry3d`, `parry3d_f64`. Everything else imports `crate::math::*` which is `pub use parry::math::*`.
- 851 `cfg(feature = "dim2"|"dim3")` sites in `src/`. They gate: type aliases, 1-vs-3 angular DOF code,
  tangent count (`DIM - 1`), `MAX_MANIFOLD_POINTS` (2 vs 4), `SPATIAL_DIM` (3 vs 6), `ANG_DIM` (1 vs 3),
  joint axis masks, gyroscopic term, twist friction (3D only), contact clustering (3D only), `block-solver`.
- Dimension-generic math is expressed with small traits in `src/utils/` (`ScalarType`, `DotProduct::gdot`,
  `CrossProduct::gcross`, `CrossProductMatrix::gcross_matrix`, `AngularInertiaOps`, `OrthonormalBasis`,
  `RotationOps`, `PoseOps`, `ComponentMul`) so that `AngVector` can be `Real` in 2D and `Vec3` in 3D.

Feature flags (from `crates/rapier2d/Cargo.toml`):

| Feature | Effect | Relevance to Cairo |
|---|---|---|
| `dim2` / `dim3` | dimension | port as two packages or one package with a `dim` module split; **start with dim2** |
| `f32` / `f64` | `Real` alias | replaced by a fixed-point `Real` |
| `block-solver` (default **on**) | 2x2 coupled normal solve for pairs of manifold points (`solve_pair` / `solve_mlcp_two_constraints`) | optional; nice for stacks, costs 4-5 divisions per pair per iteration |
| `enhanced-determinism` | `simba/libm_force`, `parry/enhanced-determinism` (IndexMap, glamx `libm` + `scalar-math`) | N/A (see §6) |
| `parallel` | rayon; staged workers, parallel narrow phase, parallel BVH refit | cut |
| `simd8` | 8-lane solver SIMD | cut. NOTE: there is **no scalar build any more** — `SIMD_WIDTH` is 4 by default (`parry/src/lib.rs:107`), contacts are always packed 4 manifolds per constraint |
| `fem` | alternative implicit FEM soft-body solver | cut |
| `serde-serialize`, `bytemuck`, `debug-render`, `profiler` | tooling | cut |
| `unsync-callbacks`, `solver-bounds-checks`, `dev-*`, `debug-*` | tooling | cut |
| `alloc`/`std` | `#![no_std]` support; `target_arch = "spirv"` drops nalgebra | N/A |

### 1.2 Public API surface

`PhysicsPipeline::step` (`src/pipeline/physics_pipeline/mod.rs:193`):

```rust
pub fn step(
    &mut self,
    gravity: Vector,
    integration_parameters: &IntegrationParameters,
    islands: &mut IslandManager,
    broad_phase: &mut BroadPhaseBvh,
    narrow_phase: &mut NarrowPhase,
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    multibody_joints: &mut MultibodyJointSet,
    soft_bodies: &mut SoftBodySet,          // new
    ccd_solver: &mut CCDSolver,
    hooks: &dyn PhysicsHooks,
    events: &dyn EventHandler,
)
```

Note: no `QueryPipeline` argument any more — `QueryPipeline<'a>` is now a **borrowed view** created on
demand from the broad-phase BVH (`BroadPhaseBvh::as_query_pipeline(dispatcher, bodies, colliders, filter)`,
`src/pipeline/query_pipeline.rs`, 935 lines): `cast_ray`, `cast_ray_and_get_normal`, `intersect_ray`,
`project_point`, `intersect_point`, `intersect_aabb_conservative`, `cast_shape`, `cast_shape_nonlinear`,
`intersect_shape`, plus `QueryFilter` (flags, groups, exclude collider/body, predicate).

`PhysicsWorld` (`src/pipeline/physics_world.rs`, 974 lines) is a convenience bundle of all of the above
(`gravity`, `integration_parameters`, `physics_pipeline`, `islands`, `broad_phase`, `narrow_phase`, `bodies`,
`colliders`, `impulse_joints`, `multibody_joints`, `soft_bodies`, `ccd_solver`). **This is the natural shape
of the Cairo `World` struct.** `CollisionPipeline` (324 lines) is the collision-only variant.

The sets:

| Set | File | Storage |
|---|---|---|
| `RigidBodySet` | `src/dynamics/rigid_body_set.rs` (464) | `Arena<RigidBody>` + `ModifiedRigidBodies(Vec<Handle>)` + a `default_fixed` body |
| `ColliderSet` | `src/geometry/collider_set.rs` (610) | `Arena<Collider>` + `modified_colliders` + `removed_colliders` |
| `ImpulseJointSet` | `src/dynamics/joint/impulse_joint/impulse_joint_set.rs` (675) | `InteractionGraph<RigidBodyHandle, ImpulseJoint>` + `Arena<edge id>` + `Coarena<graph node id>` + `to_wake_up`/`to_join` hash sets + island events |
| `MultibodyJointSet` | `src/dynamics/joint/multibody_joint/multibody_joint_set.rs` (479) | `Arena<Multibody>` + link lookup |
| `SoftBodySet` | `src/dynamics/soft_body/soft_body_set/*` | new |
| `IslandManager` | `src/dynamics/island_manager/*` (2,686) | `VecMap<Island>` (active-set chunks), `awake_island: Option<usize>`, `PersistentIslands`, substep `solve_groups` |
| `BroadPhaseBvh` | `src/geometry/broad_phase_bvh/*` (939) + `parry::partitioning::Bvh` | BVH + `HashMap<(CH,CH),u32>` pairs + per-collider adjacency |
| `NarrowPhase` | `src/geometry/narrow_phase/*` (7,429 incl. 3,422 soft) | contact graph + intersection graph + persistent coloured solver-contact graph |
| `CCDSolver` | `src/dynamics/ccd/*` (1,073) | stateless except a fixed-targets cache |

`IntegrationParameters` (`src/dynamics/integration_parameters.rs`, 432 lines), all fields + defaults:

| Field | Default | Meaning |
|---|---|---|
| `dt` | `1/60` | step length |
| `min_ccd_dt` | `1/60/100` | smallest CCD substep |
| `contact_softness: SpringCoefficients` | `natural_frequency = 30.0`, `damping_ratio = 10.0` | soft-contact spring (dynamic vs dynamic) |
| `static_contact_softness` | `60.0`, `10.0` | stiffer spring when one side is world-attached |
| `warmstart_coefficient` | `1.0` | contact warm-start scaling |
| `length_unit` | `1.0` | scales all `normalized_*` values (pixels-per-metre support) |
| `soft_bodies: SoftBodiesSettings` | default | soft body tuning |
| `normalized_allowed_linear_error` | `0.005` | geometric slop (no longer a solver dead-zone; used by CCD) |
| `normalized_max_corrective_velocity` | `3.0` | clamp on penetration-recovery bias |
| `normalized_prediction_distance` | `0.02` | speculative contact margin |
| `normalized_max_linear_velocity` | `400.0` | per-substep linear speed cap |
| `num_solver_iterations` | `4` | **number of substeps** |
| `num_internal_pgs_iterations` | `1` | biased PGS sweeps per substep |
| `num_internal_stabilization_iterations` | `1` | unbiased ("relax") sweeps per substep |
| `max_ccd_substeps` | `1` | 0 disables CCD |
| `contact_clustering` | `true` | merge near-parallel manifolds (3D only) |
| `contact_recycling` | `true` | skip narrow-phase for pairs whose relative pose barely moved |
| `normalized_contact_recycle_distance` | `0.05` | drift threshold for recycling |
| `friction_in_bias_pass` | `false` | friction only solved in the relax pass |
| `warmstart_joints` | `false` | joints restart from zero impulse by default |
| `friction_model` (3D) | `Simplified` (twist model) vs `Coulomb` | 3D only |

Joint softness is **per joint** (`GenericJoint::softness`, default `SpringCoefficients::joint_defaults()` =
`natural_frequency 1.0e6`, `damping_ratio 1.0`), not in `IntegrationParameters`.

`SpringCoefficients` formulas (the core of the "soft step" solver), with `w = 2*pi*f`, `z = damping_ratio`:

```
erp_inv_dt(dt) = w / (dt*w + 2*z)
erp(dt)        = dt * erp_inv_dt
cfm_coeff(dt)  = (1/erp - 1)^2 / ((1 + (1/erp - 1)) * 4 * z^2)      (z != 0, erp != 0)
cfm_factor(dt) = 1 / (1 + cfm_coeff)
```

Only `+ - * /` — no transcendental. With defaults and `dt_sub = 1/240`: contact `erp_inv_dt ~ 9.07`,
`cfm_factor ~ 0.942`. **Fixed-point hazard**: the joint default (`f = 1e6`) yields `cfm_coeff ~ 1.5e-9`
(about 6 ulp in Q32.32). The Cairo port should special-case "rigid" joints (`erp_inv_dt = ~inv_dt`,
`cfm_coeff = 0`) instead of evaluating the formula with 1e6.

---

## 2. Order of operations in `PhysicsPipeline::step`

Entry: `step` -> `step_inner` (`src/pipeline/physics_pipeline/substep.rs:264`). Stage timers are in
`self.counters` (`src/counters/*`, 556 lines — cut in Cairo, replace with gas measurements).

### 2.1 Pre-step: user changes (`substep.rs:280-380`)

1. `counters.reset()`, `quarantine.clear()` (`quarantine.rs`: non-finite state containment — NaN/inf
   cannot exist in fixed point; replace with overflow policy).
2. Drain delayed wake-ups: `impulse_joints.to_wake_up` + `multibody_joints.to_wake_up` -> `islands.wake_up(bodies, h, true)`.
3. `soft_bodies.apply_user_changes(...)`.
4. `colliders.take_modified()`, `colliders.take_removed()`;
   `user_changes::handle_user_changes_to_colliders` (`src/pipeline/user_changes.rs`): recompute collider
   world pose from parent when `PARENT` changed, flag parent body `LOCAL_MASS_PROPERTIES` on shape/density/enable change.
5. `bodies.take_modified()`; `handle_user_changes_to_rigid_bodies`: island insertion/removal on type or
   enable change, propagate `POSITION` to attached colliders, recompute mass properties from colliders
   (`RigidBodyMassProps::recompute_mass_properties_from_colliders`), `update_world_mass_properties`, wake on change.
6. Disabled colliders are appended to `removed_colliders`. Any change invalidates the CCD fixed-target cache.
7. Drain `to_join` -> `islands.interaction_changed(...)`; apply `impulse_joints.island_events`
   (Link/Unlink) to persistent islands; update multibody chains and soft-body attachments.
8. Multibody `forward_kinematics` + `update_rigid_bodies_internal` for every multibody.

Change tracking is a **dirty-list + bitflags** design: every `get_mut` on a set pushes the handle in the
modified list (once, guarded by `IN_MODIFIED_SET`) and setters OR a bit into `RigidBodyChanges` /
`ColliderChanges`. Internal motion does **not** go through this path (moved colliders are harvested by
`advance_to_final_positions`).

### 2.2 Collision detection: `detect_collisions` (`solve.rs:47`)

1. `broad_phase.update(params, colliders, bodies, modified, removed, &mut events)` (`broad_phase_bvh/update.rs:35`):
   - leaf AABB = `collider.compute_broad_phase_aabb` = shape AABB loosened by `prediction/2 + soft margins`
     (+ soft-CCD swept AABB), then fattened by a change-detection skin (`CHANGE_DETECTION_FACTOR = 0.04 * length_unit`);
     a leaf that stays inside its fat AABB is not touched;
   - `tree.insert_or_update_partially`, remove leaves, incremental optimise (`optimize_incremental`, may be deferred to
     another thread), `refit` / `refit_partial`;
   - pair finding: `tree.traverse_bvtt_single_tree::<CHANGE_DETECTION>` (self-collision traversal of the BVH
     restricted to changed subtrees) -> candidates filtered by `pairs` hash map -> `BroadPhasePairEvent::AddPair`;
   - stale-pair removal: for updated colliders walk `pair_adjacency` and test AABB overlap -> `DeletePair`.
   - Pair pre-filter in the broad phase (`update.rs:345-395`): same-parent pairs, pairs rejected by `ActiveCollisionTypes`
     (default = `DYNAMIC_DYNAMIC | DYNAMIC_KINEMATIC | DYNAMIC_FIXED`, i.e. no fixed-fixed, kinematic-fixed or kinematic-kinematic) and pairs rejected by `collision_groups` are never created
     (`solver_groups` is deliberately *not* tested here: solver-filtered pairs still emit events).
2. `narrow_phase.handle_user_changes` (`pair_management.rs`): remove graph nodes of removed colliders,
   drop pairs whose collider type/parent changed.
3. `narrow_phase.register_pairs`: add/remove `ContactPair` or `IntersectionPair` edges in the two
   `InteractionGraph`s (sensor => intersection graph). Emits `Stopped` events for removed pairs.
4. `narrow_phase.compute_contacts` (`contacts.rs` -> `pair_update.rs::process_pair`, 804 lines), per candidate edge:
   - skip if both bodies sleeping / no dynamic participant; **contact recycling**: if relative pose drift
     (`relative_pose_drift`) since last full update < `contact_recycle_distance`, keep the manifolds untouched;
   - filters: `ActiveCollisionTypes`, `InteractionGroups` (collision groups), joints with `contacts_enabled = false`,
     `PhysicsHooks::filter_contact_pair` -> `SolverFlags`;
   - `CoefficientCombineRule::combine` for friction/restitution (`Average, Min, Multiply, Max, ClampedSum, GeometricMean`);
   - effective prediction distance = `max(prediction, dt * |linvel1 - linvel2|)` (+ soft-CCD);
   - **`query_dispatcher.contact_manifolds(&pos12, shape1, shape2, prediction, &mut manifolds, &mut workspace)`** — parry;
   - 3D: `contact_clustering::cluster_manifolds_for_solver` (merge manifolds with normals within ~5.1 deg),
     `manifold_reduction::reduce_manifold_naive` (keep <= 4 points: deepest, farthest, 2 extremal along tangent);
   - build `SolverContact { contact_id, anchor1, anchor2, dist, tangent_velocity }` for points with
     `dist - skins < prediction` (or predicted to be within one step);
   - `PhysicsHooks::modify_solver_contacts`;
   - "localize": bake skin into anchors, express anchors in each body's **CoM frame**, freeze world-space lever arms
     `ContactData::solver_dp1/dp2` (anchor freezing);
   - start/stop events (`CollisionEvent::Started/Stopped`), wake-ups via `islands`, touching-link bookkeeping for persistent islands.
5. `narrow_phase.compute_intersections` (`intersections.rs`): boolean `intersection_test` per sensor pair + events.

### 2.3 CCD substep loop (`substep.rs:405-560`)

`remaining_substeps = max_ccd_substeps` (default 1 => a single pass, motion clamping only). With more than
one substep, `ccd_solver.find_first_impact` computes the earliest TOI and the step is split at it.

Per (CCD) substep:

1. `interpolate_kinematic_velocities`: position-based kinematic bodies get `vels = pose_errors * inv_dt`.
2. **`build_islands_and_solve_velocity_constraints`** (`solve.rs:161`) — §2.4.
3. If CCD enabled and any body flagged `ccd_active`: `ccd_solver.solve_continuous` (**motion clamping**):
   sweep each fast body (`sweep_fast_body`, `src/dynamics/ccd/sweeps.rs`: per-pair proxy sweeps from `parry::query::sweep_toi`,
   with a `NonlinearRigidMotion` shape-cast fallback for cylinders/cones/voxels/custom shapes)
   against fixed targets (non-bullets) or the whole BVH (bullets, `ccd_enabled = true`), then
   `apply_clamps`: `next_position = Sweep::transform_at(fraction)`. Velocities are **not** modified.
   A body is "fast" if `max_point_velocity * dt > 0.5 * ccd_thickness` (`RigidBodyCcd::is_moving_fast`).
4. `advance_to_final_positions`: `pos.position = pos.next_position`, update collider world poses,
   compute fresh broad-phase AABBs, `update_world_mass_properties`.
5. `quarantine.apply_end_step`.
6. `update_moved_collider_aabbs` -> `broad_phase.set_aabb` (and a full `detect_collisions` again if another CCD substep follows).

Post-loop: soft-body tears and sync, `counters.step_completed()`.

### 2.4 Islands + solver (`solve.rs:161`)

1. `islands.persistent.resolve_removals` (local bounded bidirectional search proving connectivity after a
   contact/joint removal, `local_split.rs`) and `run_pending_split` (global union-find split, at most one
   island per step, `global_split.rs` + `data/union_find.rs`).
2. **Fused traversal of active bodies**: `IslandManager::update_body_energy` (sleep timer, §4.7),
   `forces.compute_effective_force_and_torque(gravity, effective_mass)`
   (`force = user_force + gravity * mass * gravity_scale`), collect sleep observations + split bids.
3. `islands.update_islands`: whole-island sleep decision (an island sleeps when **every** body has
   `time_since_can_sleep >= time_until_sleep` (0.5 s)); `commit_sleeping_chunks`.
   There is exactly **one awake island** (`awake_island`), containing all awake bodies; persistent islands
   only drive sleeping/waking. `update_substep_groups` partitions bodies by `additional_solver_iterations`.
4. `narrow_phase.maintain_solver_contact_graph` (`narrow_phase/solver_graph.rs`, 843 lines): incremental
   maintenance of a persistent **graph colouring** (up to 128 colours + overflow, `u128` per-body colour masks)
   of solver manifolds; buckets per colour. `impulse_joints.select_active_interactions`.
5. `staged_solver.init_and_solve(...)` (`src/dynamics/solver/staged_island_solver/*`, 3,488 lines) — §2.5.
6. `narrow_phase.emit_contact_force_events` (sum of impulses * inv_dt vs `contact_force_event_threshold`).

### 2.5 The velocity solver (current algorithm)

**Algorithm: substepped soft-constraint PGS ("TGS-soft" / Box2D-v3 "soft step")**, implemented in
`staged_island_solver/worker.rs::run_worker` + `solve.rs::solve_pass`. With one worker it runs inline.
Sequence for one island:

```
init:
  for each body i: solver_bodies.copy_from(i, rb)          // pose at CoM, vels, im (Vector), ii (AngularInertia)
                   incr[i] = (force * inv_mass * dt_sub, inv_inertia_world * torque * dt_sub)
  for each manifold chunk: Contact*Builder::generate        // effective masses, lever arms, warm-start impulses
repeat num_solver_iterations (=4) times, dt_sub = dt / 4:
  1. vels += incr ; (3D, opt-in) gyroscopic_corrected_angvel
  2. joint builders .update()  (rebuild joint rows from current poses; Gram-Schmidt)
  3. per contact: builder.update() (recompute dist from current poses -> rhs, cfm) ; warmstart()
  4. repeat num_internal_pgs_iterations (=1): solve_pass(with bias)
        joints (coloured, then overflow, then multibody generic) -> soft-body rows -> contacts (colour by colour)
        friction skipped in this pass unless friction_in_bias_pass
  5. clamp |linvel| <= max_linear_velocity, |angvel| <= (pi/4)/dt ; integrate_linearized(dt_sub) on solver poses
  6. repeat num_internal_stabilization_iterations (=1): solve_pass(without bias)
        joints: rhs = rhs_wo_bias ; contacts: update_rhs_wo_bias from *current* poses, cfm_factor = 1, friction solved here
after all substeps:
  7. restitution pass (only if any contact has a restitution seed): solve_restitution per point
  8. writeback impulses to manifolds (warm-start data) and joints
  9. writeback bodies: vels = apply_damping(dt, solver vels); next_position = solver pose - local_com;
     compute ccd_vels / ccd_active
```

Details that matter for a faithful port:

- **Normal row** (`contact_constraint_element.rs::ContactConstraintNormalPart::solve`):
  `dvel = n.(v1 + w1 x r1) - n.(v2 + w2 x r2) + rhs`;
  `new_impulse = cfm_factor * max(0, impulse - r * dvel)`; apply `dlambda`.
  `r = 1 / (n.(im1+im2)*n + (r1 x n).II1.(r1 x n) + (r2 x n).II2.(r2 x n))`.
- **rhs** (`ContactWithCoulombFrictionBuilder::update`): `dist` re-derived every substep from the solver poses and
  frozen local anchors; `rhs_wo_bias = max(dist, 0) * inv_dt` (speculative), `rhs_bias = clamp(dist * erp_inv_dt, -max_corrective_velocity, 0)`;
  `rhs = rhs_wo_bias + rhs_bias`; `cfm_factor = 1` when `dist > 0`. No slop dead-zone.
- **Warm-starting**: stored per manifold point (`ContactData::warmstart_impulse`, `warmstart_tangent_impulse`,
  3D: world-space `warmstart_tangent_world` re-projected on the current basis). Every substep: `impulse *= warmstart_coefficient`,
  then re-applied. `impulse_accumulator` tracks the total impulse for events.
- **Friction, 2D**: one tangent `t = (-n.y, n.x)`; `new = clamp(impulse - r*dvel, -mu*lambda_n, +mu*lambda_n)`; tangent rhs carries
  a positional bias `(p1 - p2).t * inv_dt` (anchors drift => friction "remembers" the anchor). `tangent_velocity` = conveyor-belt term.
- **Friction, 3D Coulomb**: two tangents from the Pixar branchless orthonormal basis (`utils/orthonormal_basis.rs`, deterministic, **not** velocity-aligned);
  coupled 2x2 solve then `cap_magnitude(limit)` (needs a sqrt) — `ContactConstraintTangentPart::solve`.
- **Friction, 3D Simplified (default)**: `contact_with_twist_friction.rs` (981 lines): one 2-DOF tangent constraint at the manifold's friction centre
  + one angular **twist** constraint about the normal, limit weighted by `twist_dists`.
- **Block solver** (default on): manifold points are solved in pairs by a 2x2 MLCP enumeration (4 cases), falls back to sequential if
  the 2x2 matrix is ill-conditioned (`BLOCK_SOLVER_MIN_CONDITION = 0.01`).
- **Restitution**: box2d-style, once at end of step: target `-restitution * approach_velocity` captured at generate time, only for new/bouncy
  contacts that actually applied an impulse (`solve_restitution`).
- **Dominance**: `relative_dominance = group1 - group2`; the dominated side's solver-body id is replaced by `u32::MAX` (treated as infinite mass).
  Fixed bodies and sleeping "frontier" bodies are also `u32::MAX` => identity pose, zero velocity.
- **Joints**: rows are rebuilt **every substep** from current solver poses (`JointConstraintBuilder::update` ->
  `JointConstraint::update`), no warm start by default, solved **before** contacts in every pass ("contacts must win").
- Gauss–Seidel order = colour order then bucket order. The Rust result therefore depends on the colouring;
  **a sequential Cairo port that solves in insertion order will not reproduce Rust numbers bit-for-bit even in floats** —
  cross-validation must be tolerance-based (§8).

### 2.6 Sleeping, events, hooks

- Sleep candidacy per body (`RigidBodyActivation::update_energy`, §4.7), decision per persistent island, wake-up on
  contact start with a moving body, user modification, joint insertion (`sleep.rs`).
- `EventHandler`: `handle_collision_event(Started|Stopped, flags SENSOR|REMOVED)`, `handle_contact_force_event`,
  `handle_soft_body_tear_event`. Dispatched synchronously from inside the narrow phase / CCD / end of solve.
  `ChannelEventCollector` is the std helper. Gated per collider by `ActiveEvents`.
- `PhysicsHooks`: `filter_contact_pair -> Option<SolverFlags>`, `filter_intersection_pair -> bool`,
  `modify_solver_contacts(&mut ContactModificationContext)` (one-way platforms, conveyor belts). Gated by `ActiveHooks`.

Cairo: no `dyn` callbacks across the step. Events => append to an output `Array<CollisionEvent>` returned by `step`;
hooks => a generic trait parameter (monomorphised) or simply omitted in the MVP (collision groups cover most game needs).

---

## 3. Data structures and Cairo mapping

### 3.1 Arena / handles (`src/data/arena.rs`, 1,191 lines)

```rust
pub struct Arena<T> { items: Vec<Entry<T>>, generation: u32, free_list_head: Option<u32>, len: usize }
enum Entry<T> { Free { next_free: Option<u32> }, Occupied { generation: u32, value: T } }
pub struct Index { index: u32, generation: u32 }       // RigidBodyHandle(Index), ColliderHandle(Index), ImpulseJointHandle(Index)
```

`Coarena<T>` (`coarena.rs`, 160): `Vec<(generation, T)>` indexed by another arena's index — side tables
(graph node ids per collider, etc.). `ModifiedObjects<H, O>` = dirty list. `VecMap` (parry) = sparse vec.

**Cairo mapping.** Random-access mutation is the core difficulty: Cairo `Array<T>` is append-only and
immutable. Options: (a) `Felt252Dict<Nullable<T>>` keyed by index with a separate `len`/free-list — each
access costs a dict read + (for structs) a `Nullable` box deref, and every dict costs a squash at destruction
proportional to the number of accesses; (b) SoA of `Felt252Dict<felt252>` per scalar field (cheapest per
access, no boxing, but many dicts); (c) rebuild arrays functionally each step (O(n) per pass, fine for hot
data that is touched every step anyway — e.g. solver bodies). Recommendation: **handles = `(u32 index, u32 generation)`
packed in one felt**; persistent sets in dicts; per-step solver data in dense `Felt252Dict` SoA keyed by
`body_slot * K + field`. Generations can be dropped in an MVP where bodies are never removed mid-game, but
keep the type so it can be added.

### 3.2 Rigid body (`rigid_body.rs` 2,208; `rigid_body_components.rs` 1,623)

```rust
pub struct RigidBody {
    ids: RigidBodyIds,                 // active_island_id, active_set_id, island_id, island_index (u32 x4)
    pos: RigidBodyPosition,            // position: Pose, next_position: Pose
    damping: RigidBodyDamping<Real>,   // linear_damping, angular_damping
    vels: RigidBodyVelocity<Real>,     // linvel: Vector, angvel: AngVector
    forces: RigidBodyForces,           // force, torque (effective), gravity_scale, user_force, user_torque, gyroscopic_forces_enabled
    mprops: RigidBodyMassProps,        // world_com, effective_inv_mass: Vector, effective_world_inv_inertia: AngularInertia,
                                       // local_mprops: MassProperties, flags: LockedAxes, additional_local_mprops, max_extent
    ccd_vels, ccd: RigidBodyCcd,       // ccd_thickness, ccd_active, ccd_enabled, soft_ccd_prediction, allow_fast_rotation
    colliders: RigidBodyColliders,     // Vec<ColliderHandle>
    activation: RigidBodyActivation,   // normalized_linear_threshold 0.05, angular_threshold 0.5, time_until_sleep 0.5,
                                       // time_since_can_sleep, sleeping, sleep_prev_pose
    changes: RigidBodyChanges,         // bitflags
    body_type: RigidBodyType,          // Dynamic, Fixed, KinematicPositionBased, KinematicVelocityBased, SoftFrame
    dominance: RigidBodyDominance(i8),
    enabled: bool, additional_solver_iterations, additional_pgs_iterations, soft_body.., user_data: u128,
}
```

`effective_inv_mass` is a **Vector** (per-axis, so translation locks zero a component and everything uses
`component_mul`). `AngularInertia` is `Real` in 2D and `SdpMatrix3<Real>` (6 floats, symmetric) in 3D.

Cairo: one struct per component group works (value semantics: read struct, modify, write back). 2D body is
about 35 scalars; 3D about 70. Hard parts: `Vec<ColliderHandle>` per body (use a linked list through
colliders: `next_sibling` field, or a fixed max), `Option<Box<..>>` additional mass props (drop),
`u128 user_data` (use `felt252`).

### 3.3 Collider (`collider.rs` 1,517; `collider_components.rs` 427)

`Collider { coll_type: Solid|Sensor, shape: SharedShape (Arc<dyn Shape>), mprops: Density|Mass|MassProperties,
changes, parent: Option<{handle, pos_wrt_parent: Pose}>, pos: Pose, material: {friction 0.5, restitution 0.0, combine rules},
flags: {active_collision_types, collision_groups, solver_groups: InteractionGroups{memberships u32, filter u32, test_mode}, active_hooks, active_events, enabled},
contact_skin, contact_force_event_threshold, user_data }`.

Cairo: `Arc<dyn Shape>` => a closed `enum Shape { Ball, Cuboid, Capsule, ConvexPolygon(Span<Vec2>), ... }`; the
pair dispatch becomes a `match (s1, s2)`. Variable-size shapes (polygons, polylines) need an indirection
(shape table + `Span`). `InteractionGroups::test` is two ANDs — trivial and very useful for games.

### 3.4 Contact data (`contact_pair.rs`, 1,063)

- `ContactPair { collider1, collider2, event_status, contacts: PairContacts::Rigid(RigidPairContacts { manifolds: Vec<ContactManifold>, solver_clusters, workspace, recycle_state, solver_color, .. }) | Soft{..} }`
- `ContactManifold` (parry generic over `ContactManifoldData`, `ContactData`): `points: [TrackedContact {local_p1, local_p2, dist, fid1, fid2, data: ContactData}]` (<= 2 in 2D), `local_n1`, `local_n2`, `subshape1/2`, `subshape_pos1/2`.
- `ContactData { impulse, tangent_impulse, warmstart_impulse, warmstart_tangent_impulse, (3D) warmstart_twist_impulse, warmstart_tangent_world, solver_dp1, solver_dp2 }`
- `ContactManifoldData { rigid_body1/2, solver_flags, solver_color, solver_body_ids: [u32;2], graph_pos, normal, solver_contacts: ArrayVec<SolverContact,2> (2D) | Vec (3D), relative_dominance: i16, user_data, friction, restitution }`
- `SolverContact { contact_id (bit31 = is_new), anchor1, anchor2, dist, tangent_velocity }`

**Warm-start persistence needs feature-id matching** across frames (done inside parry's manifold update via
`fid1/fid2`). In Cairo: key the persistent impulse store by `(pair_key, feature_id_pair)` in a `Felt252Dict`.
2D makes this easy (<= 2 points, feature = vertex/edge index).

### 3.5 Interaction graph (`data/graph.rs` 786, `geometry/interaction_graph.rs` 284)

A petgraph-derived adjacency-list graph: `nodes: Vec<Node{weight, next:[EdgeIndex;2]}>`,
`edges: Vec<Edge{weight, next:[EdgeIndex;2], node:[NodeIndex;2]}>`, swap-remove on deletion (edge ids are
unstable; side tables mirror the swap-removes). Two instances in `NarrowPhase` (contacts, intersections), one
in `ImpulseJointSet`.

Cairo: for an MVP **drop the graph**. Keep `pairs: Felt252Dict<pair_key -> pair slot>` plus a dense list of
live pair slots rebuilt each step. Per-collider adjacency is only needed for efficient removal and
"contacts_with(collider)" queries.

### 3.6 Island manager (2,686 lines)

`Island { bodies: Vec<RigidBodyHandle> }` chunks in a `VecMap`; one is the awake island; `PersistentIslands`
holds per-island `bodies`, `contact_links`, `joint_links`, sleeping flag, split cooldowns;
union-by-size merges on new touching contacts/joints, deferred splits. `active_set_epoch` invalidates caches.

Cairo: **rebuild per step with union-find** (`data/union_find.rs`, 106 lines, is already the classic array
version) over touching pairs + joints — O((n + m) alpha) with dict-backed parent array. Persistent/incremental
islands are a CPU optimisation for 40k-body scenes and not worth their complexity in a provable setting.
Or, in the MVP, **no sleeping at all** (every proven step pays full cost anyway unless sleeping bodies are
skipped — sleeping is actually a big gas saver for mostly-static game worlds, so plan for it in phase 2).

### 3.7 Solver-side structures

`SolverBodies { vels: Vec<SolverVel>, poses: Vec<SolverPose{rotation, translation, ii, im}>, flags }` (AoS
with SIMD gather/scatter), `ContactWithCoulombFriction<SimdReal>` (4 manifolds x up to 2/4 points),
`JointConstraint<N, LANES>` rows, `SolverContactGraph` colour buckets, `ManifoldStore` raw-pointer view.
All the colouring/chunking/`unsafe` pointer sharing (319 `unsafe` occurrences in `src/`) exists only for
SIMD + threads. **In Cairo: one scalar constraint per manifold, sequential Gauss–Seidel, no colouring.**

---

## 4. Dynamics math

### 4.1 Mass properties

`parry::mass_properties::MassProperties { local_com, inv_mass, inv_principal_inertia (Real | Vector), principal_inertia_local_frame (3D: Rotation) }`.

- 2D: angular inertia is a scalar; `world_inv_inertia(rot) = inv_principal_inertia` (rotation-invariant).
- 3D: `world_inv_inertia = R * diag(inv_I) * R^T` with `R = from_quat(rot * principal_frame)` — a quat->mat3 and a
  mat3 product per body per step (`parry/src/mass_properties/mass_properties.rs:260`). Computing `principal_inertia_local_frame`
  from a general tensor needs a **symmetric 3x3 eigendecomposition** (`glamx::SymmetricEigen3`) — only when
  combining several colliders / convex meshes; for primitive shapes the frame is identity.
- The old `inv_principal_inertia_sqrt` / "world inv inertia sqrt" formulation is **gone**; the solver uses plain inverse inertia
  (`poses.ii.transform_vector(torque_dir)`).
- `RigidBodyMassProps::update_world_mass_properties`: world CoM, splat inv mass, zero out locked axes (`LockedAxes`), zero all for non-dynamic.
- Per-shape formulas live in parry (`ball`, `cuboid`, `capsule`, `convex_polygon`…): ball 2D `I = m r^2 / 2`, cuboid 2D `m (w^2+h^2)/12`, etc. `pi` constant needed for ball/capsule density->mass.

Complexity: 2D **trivial**; 3D **medium** (quat->mat, SDP matrix ops); compound bodies **medium-hard** (parallel-axis + eigen in 3D).

### 4.2 Force / velocity integration

Semi-implicit (symplectic) Euler, split across substeps: `v += inv_mass (*) force * dt_sub`, `w += II_world * torque * dt_sub`
(`worker.rs:75-77`, `286-291`). `forces.compute_effective_force_and_torque`: gravity enters as a force
`gravity * mass * gravity_scale` (so `effective_mass = 1/inv_mass` per axis, via `utils::inv` which maps ~0 to 0).
Impulses: `apply_impulse` => `linvel += impulse (*) inv_mass`; `apply_impulse_at_point` adds `II * ((p - com) x impulse)`.
Complexity: **trivial**.

### 4.3 Gyroscopic term (3D, opt-in per body)

`gyroscopic_corrected_angvel` (`rigid_body.rs:2185`): explicit term in the principal frame
`L' = L - (w x L) dt`, rescaled to preserve `|L|` (needs one `sqrt` of a ratio), back to world. Default off. Complexity: **low**, skip in MVP.

### 4.4 Damping

`apply_damping` (`rigid_body_components.rs:859`): `v *= 1 / (1 + dt * damping)` — a Padé approximation,
**no `exp`/`pow`**. Applied once per step at writeback with the full `dt`. Complexity: **trivial** (one division per body).

### 4.5 Position integration

`integrate_linearized` (`rigid_body_components.rs:884-923`), applied to the CoM-centred solver pose each substep:

- 2D: `(cos, sin) += dang * (-sin, cos)` then **normalise** the unit complex (`sqrt` + division or `rsqrt`), `t += v dt`.
- 3D: `q = (w*dt/2, 1) * q` then **normalise** the quaternion.

No `sin`/`cos`. The exact version (`RigidBodyVelocity::integrate`, using `Rotation::new(angle)` /
`from_scaled_axis`, i.e. sin/cos) is only used for kinematic prediction (`predict_position_using_velocity*`),
soft-CCD and user-facing helpers. Angular speed is clamped to `(pi/4)/dt` per step unless `allow_fast_rotation`.
Final: `next_position = solver_pose.prepend_translation(-local_com)`.
Complexity: **low**; the normalisation is the single most frequent `sqrt` in the engine (bodies x substeps).

### 4.6 Contact constraints — see §2.5. Complexity ranking

| Piece | 2D | 3D |
|---|---|---|
| normal row build/solve | low | low-medium (SDP mat-vec) |
| friction | low (1 clamp) | medium (2x2 coupled + cap magnitude) / high (twist model) |
| block solver | medium | medium |
| anchor freezing / recycling / clustering | skip | skip (clustering matters for trimesh/compound only) |

### 4.7 Sleeping math

`update_energy`: dynamic body can sleep if `sq_angvel < (pi/2)^2` and
`relative_pose_drift(prev_pose, pose, max_extent) * 0.5 < linear_threshold * dt`, where drift = translation length +
rotation chord `2 * max_extent * sin(dtheta/2)`; 2D uses `sin(dtheta/2) = |sin| / sqrt(2(1+cos))` — explicitly written to
avoid atan2/acos. Needs `length()` (sqrt) — can be replaced by squared comparisons in Cairo.

### 4.8 Joints (impulse joints)

Everything is a `GenericJoint` (`src/dynamics/joint/generic_joint.rs`, 858 lines):
`local_frame1/2: Pose`, `locked_axes`, `limit_axes`, `motor_axes`, `coupled_axes` (bitmasks over LinX..AngZ),
`limits[SPATIAL_DIM]{min,max,impulse}`, `motors[SPATIAL_DIM]{target_vel,target_pos,stiffness,damping,max_force,impulse,model}`,
`softness`, `contacts_enabled`, `enabled`. Typed wrappers only set masks:

| Joint | Locked axes | Notes | Complexity |
|---|---|---|---|
| Fixed (`fixed_joint.rs` 180) | all | | low (2D: 3 rows) |
| Revolute (387) | LIN_* (+ANG_Y,ANG_Z in 3D) | limits/motor on AngX; `angle()` helper uses `asin`/`atan2` | low 2D / medium 3D |
| Prismatic (317) | all but LinX | limits/motor on LinX | low |
| Spherical (340, 3D) | LIN_* | per-axis angular limits/motors | medium |
| PinSlot (297, 2D) | LinY | | low |
| Rope (280) | none; `coupled_axes = LIN_AXES`, limit max dist | `limit_linear_coupled` — needs `length` + division | low-medium |
| Spring (184) | none; coupled LIN + position motor, `MotorModel::ForceBased` | `motor_linear_coupled` | low-medium |
| Generic 6-DOF | any | coupled angular limits (3D) `limit_angular_coupled` | high |

Row construction (`joint_constraint_helper.rs`, 847): `JointConstraintHelper::new` builds `basis = frame1.rotation.to_mat()`,
`lin_err = t2 - t1`, `cmat_i = [r_i]x * basis`, `ang_err = R1^-1 R2` (+ 3D `diff_conj1_2_tr` angular basis and quaternion sign fix via `copysign`).
Rows: `lock_linear` (`rhs_bias = lin_jac.lin_err * erp_inv_dt`), `lock_angular` (`rhs_bias = ang_err.imag[axis] * erp_inv_dt` — the **sine** of the error,
no trig), `limit_linear`, `limit_angular` (**`atan2`** via `recentered_angle`, plus `sin_cos` of the limit centre once per assembly),
`motor_linear`, `motor_angular` (2D: `ang_err.angle()` = **atan2**; 3D: **`asin`**). Then
`finalize_constraints`: **modified Gram–Schmidt** orthogonalisation of the rows of a joint w.r.t. the mass metric
and `inv_lhs = 1/(J M^-1 J^T + cfm_gain)`. Solve: `JointConstraint::solve_generic` — one clamp per row.
`MotorModel::{AccelerationBased, ForceBased}::combine_coefficients` — divisions only.

So: **locked-axis joints (fixed / revolute / prismatic without limits or position motors) need no trig at all.**
Angular limits and angular position motors need `atan2` (2D) / `asin`,`atan2` (3D).

### 4.9 Multibody joints (reduced coordinates) — 4,329 lines + 1,067 (generic constraints) + 742/584 (generic contacts)

`multibody.rs` (2,003): Featherstone-style generalized coordinates with dense `DMatrix`/`DVector`, body jacobians
(`Matrix3xX`/`Matrix6xX`), augmented mass matrix assembled per step and **LU-factorised** (`na::LU<Real, Dyn, Dyn>`),
Coriolis terms, energy guard, IK (`multibody_ik.rs`). The only place rapier truly needs nalgebra's dynamic
linear algebra. Complexity: **very high**; O(n^3) dense LU in a VM is prohibitive. **Cut.**

### 4.10 Soft bodies — ~26k lines (soft_body 11,770 + soft_constraint 7,956 + soft_fem 2,506 + soft_contacts 3,422). **Cut.**

### 4.11 Controllers (`src/control`, 2,761 lines)

`KinematicCharacterController` (1,496; shape-casts + slope/stairs/snap logic, uses `QueryPipeline`),
`DynamicRayCastVehicleController` (823), `PidController` (422). Out of MVP; character controller is a likely phase-3 item for games.

---

## 5. Floating-point function inventory (drives the fixed-point library)

Scope: core rigid-body path (soft bodies, multibody, debug render, controllers excluded unless noted). parry has its own, larger list.

### 5.1 `sqrt` / length / normalise

| Where | What |
|---|---|
| `dynamics/rigid_body_components.rs:900` (2D), `:920` (3D) `integrate_linearized` | rotation renormalisation — **hot: bodies x substeps** |
| `dynamics/solver/staged_island_solver/worker.rs:662,674` | `linear.length()`, `angular.length()` for velocity caps — hot, replaceable by squared compare + sqrt only when exceeded |
| `dynamics/solver/contact_constraint/contact_constraint_element.rs:159,692`, `generic_contact_constraint_element.rs:290` | 3D friction `cap_magnitude` — hot (3D only) |
| `dynamics/solver/joint_constraint/joint_constraint_helper.rs:246,367` | `lin_jac.simd_length()` for coupled (rope/spring) rows |
| `dynamics/rigid_body.rs:2203` | gyroscopic momentum rescale (3D opt-in) |
| `dynamics/rigid_body_components.rs:1117-1175` | CCD fast-body test: `linvel.length()`, 3D `angvel.length()`, `(com2-com1).length()` |
| `geometry/contact_pair.rs:390-409` `relative_pose_drift` | translation length, 2D `sqrt(2(1+cos))`, 3D quaternion vector length (sleep + recycling) |
| `dynamics/island_manager/local_split.rs:286` | `sq_linvel.sqrt() + sq_angvel.sqrt() * max_extent` |
| `geometry/narrow_phase/pair_update.rs:350` | `(linvel1 - linvel2).length()` for effective prediction distance; `:716` aabb extents length |
| `dynamics/coefficient_combine_rule.rs:88` | `GeometricMean` combine rule (optional) |
| `dynamics/rigid_body_components.rs:527` | `max_extent` from collider bounding spheres (on collider change only) |
| `utils/dot_product.rs:24` `simd_length`, `utils/mod.rs:70` `try_normalize_and_get_length` | helpers |
| `dynamics/rigid_body.rs:1250` | `predict_position_using_velocity_and_forces_with_max_dist` (soft CCD) |

`rsqrt` is never called directly; normalisation goes through glam `normalize()`.

### 5.2 Trigonometry

| Function | Where | When |
|---|---|---|
| `sin_cos` | `joint_constraint_helper.rs:62,64` (`AngularLimitParams::new`) | once per joint assembly with angular limits |
| `atan2` | `joint_constraint_helper.rs:479,493` (`recentered_angle`) | per substep per angular-limit row |
| `asin` | `joint_constraint_helper.rs:593` (3D angular motor), `joint/revolute_joint.rs:104-106` (3D `angle()` helper) | 3D only |
| `Rot2::angle()` (= atan2) | `joint_constraint_helper.rs:586` (2D angular position motor), `rigid_body_components.rs:199` (`pose_errors` -> kinematic position-based bodies, PID), `revolute_joint.rs:111`, `utils/rotation_ops.rs:190,218` | per kinematic body per step; per motor row |
| `to_scaled_axis` (acos/atan2 + sqrt) | `rigid_body_components.rs:203` (3D `pose_errors`) | 3D kinematic position-based bodies |
| `Rotation::new(angle)` / `from_scaled_axis` (sin, cos) | `lib.rs:173-181` `rotation_from_angle`, `utils/pos_ops.rs:129,137` `append_rotation` used by `RigidBodyVelocity::integrate` | body construction from an angle; kinematic/CCD prediction; user API. **Not in the solver loop** |
| `sin`/`cos` of delta rotation | `rigid_body_components.rs:1170`, `contact_pair.rs:396` | these read the stored complex components (`Rot2::sin()` is a field), no trig evaluated |
| `sin(angle/2)` | `generic_joint_constraint_builder.rs:766,769` | multibody only |
| `acos` | none in core (only `FRAC_PI_4`/`FRAC_PI_2`/`PI`/`TWO_PI` constants: `worker.rs:29`, `rigid_body_components.rs:1483`, `joint_constraint_helper.rs:494-495`, `utils/mod.rs:225`, `integration_parameters.rs` `simd_two_pi`) | constants |

`exp`, `pow`, `ln`, `tan`, `floor`/`ceil`/`round`: **not used** in the core engine (damping is rational, §4.4).

### 5.3 Division / reciprocal

`utils::inv` / `utils::simd_inv` (`utils/mod.rs:132-146`): `1/x` with `|x| <= 1e-20 -> 0`. Used for every effective mass
(`contact_with_coulomb_friction.rs:232,285,341`, `contact_with_twist_friction.rs:267,334,409`, `contact_constraint_element.rs:149,319-338,682`,
`joint_constraint_helper.rs:692-693`, `motor_model.rs:45-51`, `joint_constraint_builder.rs:430-432`, `rigid_body_components.rs:382,389`,
`geometry/mod.rs:267`, `pair_update.rs:333`, `solver_graph.rs:469`), plus `inv_dt`, `SpringCoefficients` (4-5 divisions per
`cfm_factor`, computed per constraint per substep in Rust — **hoist to once per substep in Cairo**), damping, orthonormal basis (3D `-1/(sign+z)`).
In fixed point the epsilon test becomes `x == 0`. Fixed-point division is the dominant arithmetic cost: count divisions, not multiplications.

### 5.4 min / max / clamp / abs / sign / select

Everywhere in the solver: `simd_max(0)` (normal impulse), `simd_clamp(-limit, limit)` (2D friction, joint bounds),
`simd_clamp(-max_corrective_velocity, 0)`, `simd_min`, `abs`, `signum`/`copysign` (`worker.rs:670`, `utils/mod.rs:210,220`,
`orthonormal_basis.rs:78,89`, `joint_constraint_helper.rs:135,495`), lane `select` (becomes `if`).

### 5.5 Sentinels and epsilon comparisons

- `Real::MAX` as "unbounded": `JointLimits` default, `JointMotor::max_force`, `impulse_bounds`, `normalized_max_*` checks
  (`integration_parameters.rs`), `Real::INFINITY` (`joint_constraint_helper.rs:260`). Fixed point: use `Option` or a saturating `MAX`, and
  make sure `MAX * x` never happens — Rust does compute `max_impulse: self.max_force * dt` with `max_force = Real::MAX`
  (`generic_joint.rs:246`, harmless in floats since `dt < 1`, an overflow/garbage hazard in fixed point).
- `INV_EPSILON = 1e-20`, `denom > 1e-6` (`contact_pair.rs:398`), `COS_MERGE_ANGLE = 0.996`, `BLOCK_SOLVER_MIN_CONDITION = 0.01`,
  `DEFAULT_EPSILON = f32::EPSILON` (parry), exact `== 0.0` tests for "is new contact" (`impulse == 0.0`), kinematic sleep (`sq_linvel == 0.0`).
- `is_finite` checks (`quarantine.rs`, `substep.rs`) — irrelevant in fixed point; the analogue is **overflow detection** (panic = unprovable step, so
  velocity caps and input validation become correctness-critical).
- `canonicalize_zero` (`utils/mod.rs:90`): `-0.0` handling — irrelevant.

### 5.6 Resulting fixed-point library requirements

Must-have for 2D MVP: `add, sub, mul, div, neg, abs, min, max, clamp, sign, sqrt, cmp`, constants `PI, TWO_PI, FRAC_PI_2, FRAC_PI_4`,
`Vec2 {dot, perp, cross (scalar), length, length_squared, normalize}`, `Rot2 {mul, inverse, rotate vec, normalize, from_cos_sin}`,
`Pose2 {mul, inverse, transform_point, inverse_transform_point}`. Add `sin_cos` (body creation from angle, limit centres) and
`atan2` (angular limits, position motors, kinematic position-based bodies) for phase 2.
3D adds: `Vec3 cross`, `Quat {mul, conj, rotate, normalize, to_mat3}`, `Mat3`/`SdpMatrix3` mat-vec and `R D R^T`, `asin`, `copysign`.

Range/precision: velocities up to 400 units/s, `inv_dt` 240-960, `erp_inv_dt` ~ 9-240, inverse inertia of small bodies ~1e3-1e5, penetration
depths ~1e-3, `cfm_coeff` down to 1e-9 for rigid joints (special-case). Products like `inv_inertia * torque_dir * impulse` and
`dist * inv_dt` argue for **at least Q32.32 with 128-bit intermediates** (or Q64.64 on `u256`/felt arithmetic); Q16.16 is not enough.

---

## 6. Determinism

What `enhanced-determinism` does today (it is much thinner than it used to be):

1. `simba/libm_force`: all `ComplexField`/`RealField` transcendentals (`sin`, `cos`, `atan2`, `asin`, `sqrt`...) go through `libm` software
   implementations instead of platform intrinsics.
2. `parry/enhanced-determinism` => `glamx/libm` + `glamx/scalar-math` (glam without SSE/NEON paths; "glam's shared scalar core") + `indexmap`.
3. `parry::utils::hashmap::HashMap` / `hashset::HashSet` become `IndexMap`/`IndexSet` (insertion-ordered iteration) instead of `hashbrown`
   (used by: broad-phase `pairs`, `ImpulseJointSet::to_wake_up`/`to_join` — hence the `drain(..)` vs `drain()` cfg in `substep.rs:291-300,360-369`,
   persistent-island joint link map, soft self-contact map). `utils/mod.rs:168-181` wraps the choice.
4. `canonicalize_zero` on stored impulses (signed-zero snapshot equality).
5. Incompatible with `simd8` (`lib.rs:19`); compatible with `parallel` (results are independent of thread count because the Gauss–Seidel
   order is defined by the scene-dependent colouring, `LAYOUT_REF_WORKERS = 8` constant, ordered chunk reductions).
6. Workspace note in `Cargo.toml`: glam `fast-math` must never be enabled (FMA contraction breaks cross-platform equality).

Remaining sources of nondeterminism in Rust that the features guard against: hash-map iteration order, platform libm differences, FMA
contraction / SIMD lane-width differences, thread scheduling, NaN propagation, snapshot restore ordering (arena free lists and graph
swap-removes are serialised to keep handle allocation identical — see test `rigid_body_removal_snapshot_handle_determinism`).

Relevance to Cairo: integer fixed point on a deterministic VM makes items 1, 2, 4, 5, 6 moot. What **still matters** is *algorithmic
order*: iteration order over pairs/bodies/joints must be a pure function of state (use index order / insertion order, never anything
derived from dict squashing internals), and the engine state must be fully serialisable so that "state_n + inputs -> state_n+1" is the
provable transition. Rapier's lesson to keep: persist warm-start impulses and contact ids in the state, or replays diverge.

---

## 7. Dependencies

From `crates/rapier2d/Cargo.toml` + `src/`:

| Crate | Used for | Port impact |
|---|---|---|
| **parry2d/3d 0.31** | shapes (`SharedShape`, `Shape` trait, `Ball, Cuboid, Capsule, Segment, Triangle, ConvexPolygon/Polyhedron, TriMesh, Polyline, HeightField, Compound, Voxels, HalfSpace, Round*`), `query::PersistentQueryDispatcher::contact_manifolds`, `intersection_test`, `ContactManifold`/`TrackedContact`, shape casting (`cast_shapes`, `cast_shapes_nonlinear`, `NonlinearRigidMotion`), ray casting, point projection, `bounding_volume::{Aabb, BoundingSphere}`, `partitioning::{Bvh, BvhWorkspace, BvhNode, BvhLeafUpdateStatus}` + `traverse_bvtt_single_tree`, `mass_properties::MassProperties`, `utils::{SdpMatrix2/3, VecMap, hashmap, hashset, PoseOpt}`, `math::*` | the geometry agent's domain; rapier.cairo needs: AABB, MassProperties, a manifold generator for the chosen shape pairs, optionally a BVH |
| **glamx 0.3** (re-exported by parry; wraps **glam 0.33**) | **all concrete math types**: `Vec2/Vec3/DVec2/DVec3`, `Mat2/Mat3`, `Rot2`/`Rot3` (unit complex `re/im` / quaternion), `Pose2`/`Pose3` (`translation` + `rotation`), `SymmetricEigen2/3`, `MatExt` | **primary math dependency -> glam.cairo**. `crate::math::{Real, Vector, AngVector, Rotation, Pose, Matrix, AngularInertia, SpatialVector, ...}` are aliases defined in `parry/src/math/mod.rs` |
| **nalgebra 0.35** | only: (a) generic-over-`SimdReal` SoA types in the solver (`SimdVector<N> = na::Vector2/3<N>`, `na::UnitComplex/UnitQuaternion<N>`, `na::Isometry2/3<N>`, `na::Matrix2/3<N>`), (b) `TangentImpulse<N> = na::Vector1/Vector2<N>`, `na::Vector2` in the block solver, (c) multibody: `DVector`, `DMatrix`, `LU`, `Matrix3xX/6xX` jacobians, `SVector/SMatrix`; (d) `vector!`/`point!` macros re-exported in the prelude | **not needed** for a scalar port without multibody. nalgebra.cairo only becomes relevant if multibody is ever ported |
| **simba 0.10** (`wide` feature) | `SimdReal = WideF32x4`, `SimdValue/SimdBool/SimdPartialOrd/SimdRealField` traits used pervasively in solver code (`simd_max`, `simd_clamp`, `select`, `splat`), `ComplexField/RealField` for scalar transcendentals | collapses to scalar ops |
| **wide** | SIMD backend | cut |
| **num-traits** | `Zero`, `One`, `FloatConst` (`Real::PI()`), `Float` | constants |
| **approx** | test assertions | tests only |
| **arrayvec** | `SolverContacts = ArrayVec<SolverContact, 2>` in 2D | fixed-size pairs in Cairo |
| **bitflags** | `RigidBodyChanges`, `ColliderChanges`, `LockedAxes`, `AxesMask`, `JointAxesMask`, `ActiveEvents`, `ActiveHooks`, `ActiveCollisionTypes`, `SolverFlags`, `CollisionEventFlags`, `Group` | `u8`/`u32` + bit ops |
| log, thiserror, profiling, static_assertions | diagnostics/layout asserts | cut |
| rayon, web-time, serde, bytemuck (optional) | parallel / profiler / serialisation | cut |

---

## 8. Testing, examples, golden scenarios

How rapier tests itself:

- **Unit tests in `src/`**: 94 `#[test]` functions; the largest clusters are pipeline behaviour (`physics_pipeline/test.rs`, 740 lines:
  removal before step, snapshot handle determinism, CCD hook, body type toggling, `dt = 0`, user force persistence, contact-force events),
  `test_staged.rs` (staged solver vs thread counts), `quarantine.rs` (NaN containment), multibody regression tests, `union_find.rs`,
  `coefficient_combine_rule.rs`, `rigid_body_components.rs::test_interpolate_velocity` (interpolate-then-integrate round trip),
  `joint_constraint_helper.rs::recentered_angle_slope_is_one_everywhere`, contact element tests.
- **Integration/regression tests**: `crates/rapier2d/tests/*.rs` (23 files in 2D alone: `ccd_*`, `issue_NNN_*` e.g. `issue_499_angular_limits`,
  `issue_629_large_ground_stability`, `issue_772_pin_slot_joint`, `issue_818_nan_tangent_impulse`). These are short headless scenes with
  assertions on final poses — **excellent templates** for Cairo tests.
- **Testbed examples** (`examples2d/`, `examples3d/`): visual scenes; `b2d_*`/`b3d_*` are benchmark ports of Box2D v3 samples, `s2d_*` are
  Solver2D samples (arch, bridge, card house, high mass ratio, far pyramid…). `examples3d/stress_tests/` (balls, boxes, capsules, ccd, joints…).
  CI: `.github/workflows/rapier-ci-build.yml`, `rapier-ci-bench.yml`.
- No built-in golden-file mechanism; determinism is checked by stepping two worlds and comparing (`debug_rollback3`, snapshot tests).

Recommended golden scenarios (generate with a small Rust harness against `rapier2d` with `enhanced-determinism`, **f64 build** to minimise
float noise, dump `(step, body, x, y, angle, vx, vy, w)` as JSON/CSV; compare Cairo fixed-point output with tolerances that grow with step count):

1. **Free fall, no contact** (ball, 120 steps): validates gravity, substep integration, damping. Analytic check also available
   (note the substepped semi-implicit Euler: `y_n` differs from `gt^2/2` in a known way).
2. **Ball dropped on fixed ground**, restitution 0 and 0.7 (`examples2d/restitution2.rs`): normal row, speculative contact, restitution pass.
3. **Box resting on ground** then **box sliding on an incline** with friction below/above `tan(theta)`: friction clamp + tangent bias.
4. **Box stack of 5-10** (`examples2d/pyramid2.rs`, `s2d_pyramid.rs`, `debug_vertical_column2.rs`): warm-starting, soft contact, block solver; metric = max penetration and drift after N steps, not exact positions.
5. **Pendulum / chain** (`multi_pendulum2.rs`, `s2d_ball_and_chain.rs`, `joints2.rs`): revolute lock rows + Gram–Schmidt; metric = energy and anchor error.
6. **Prismatic with limits, revolute with angular limits** (`debug_angular_limits2.rs`, test `issue_499_angular_limits`): `atan2` path.
7. **Rope and spring joints** (`rope_joints2.rs`, `spring_joints3.rs`).
8. **Kinematic platform** (`platform2.rs`), **locked rotations** (`locked_rotations2.rs`), **damping** (`damping2.rs`), **collision groups** (`collision_groups2.rs`), **sensor** (`sensor2.rs`).
9. **High mass ratio** (`s2d_high_mass_ratio_*.rs`) as a fixed-point precision stress test.
10. **Fast bullet vs thin wall** (`ccd2.rs`, `crates/rapier2d/tests/ccd_semantics.rs`) if CCD is ported.

To make Rust and Cairo comparable, configure Rust with the features the port drops: `contact_recycling = false`, `contact_clustering = false`,
default-features off for `block-solver` if the port omits it, `max_ccd_substeps = 0`, sleeping disabled (`can_sleep(false)`). Expect chaotic
divergence in piles after ~100 steps regardless; compare invariants (rest height, no tunnelling, energy bounds) there.

---

## 9. Proposed decomposition for `rapier.cairo`

### 9.1 What to cut, and why

| Cut | Lines saved | Reason |
|---|---|---|
| Soft bodies (+ FEM, tearing, soft contacts) | ~26,000 | new, huge, per-particle constraints => gas explosion |
| Multibody joints + generic (jacobian) constraints | ~6,700 | dense LU, dynamic matrices, O(n^3) |
| `parallel`, staged workers, `StageSync`, colouring, solver contact graph, `ManifoldStore`, SIMD gather/scatter | ~6,000 | single-threaded VM; sequential Gauss–Seidel replaces all of it. The *math* inside `contact_constraint_element.rs` and `joint_constraint_helper.rs` is what gets ported |
| Persistent islands (local/global split) | ~1,700 | replace with per-step union-find, or no sleeping in MVP |
| BVH incremental optimiser, deferred optimisation, change-detection flags | most of parry `partitioning` | start with O(n^2) AABB sweep or a uniform grid keyed in a dict; games on-chain have tens of bodies, not 40k |
| Contact recycling, anchor freezing, clustering, manifold reduction | ~600 | CPU cache optimisations / 3D mesh quality features |
| serde, bytemuck, debug-render, counters/profiler, quarantine, `PhysicsWorld` thread-pool API | ~3,500 | tooling; NaN cannot occur (overflow policy instead) |
| Controllers (character, vehicle, PID) | ~2,800 | phase 3 |
| CCD | ~1,100 + parry shape casting | phase 3; mitigate with velocity caps, thick walls, speculative margin (`prediction_distance` scaled by relative velocity is already in the narrow phase and is cheap) |
| 3D twist friction model, gyroscopic | ~1,100 | use plain Coulomb in 3D |
| `PhysicsHooks` (dyn), `EventHandler` (dyn) | — | events => returned array; hooks => omitted or generic param |

What remains is roughly **6-8k lines of Rust-equivalent logic**, of which the numerically essential part is under 2k.

### 9.2 Modules and dependency order

```
L0  fixed (Real)            glam.cairo: Vec2/Rot2/Pose2 (Vec3/Quat/Mat3/Pose3)        [sibling projects]
L1  math_ext                utils traits: gcross, gcross_matrix, orthonormal_vector, inv(), AngularInertia ops (SdpMatrix3 in 3D)
    data                    Handle(index, generation), Arena-over-dict, dirty lists, union_find, bitflag helpers
L2  mass_properties         MassProperties per shape, combine, world inv inertia          (parry-side, shared with geometry agent)
    shapes + aabb           closed Shape enum, compute_aabb                               (geometry agent)
L3  rigid_body              components (§3.2), builder, forces/impulses API, update_world_mass_properties, apply_damping, integrate_linearized
    collider                components (§3.3), material + CoefficientCombineRule, InteractionGroups
    integration_parameters  struct + SpringCoefficients (erp_inv_dt, cfm_factor)  — pure functions, unit-testable against Rust immediately
L4  broad_phase             AABB pairs (n^2 -> grid/BVH later), pair add/remove events
    narrow_phase            pair table, manifold generation dispatch (geometry agent), SolverContact build, warm-start matching by feature id,
                            collision events, intersection (sensor) pairs
    joints                  GenericJoint + typed builders (fixed, revolute, prismatic, rope, spring)
L5  solver/contact          scalar ContactConstraint {normal parts, tangent parts}: generate, update, warmstart, solve, update_rhs_wo_bias,
                            apply_restitution, writeback      (port of contact_with_coulomb_friction.rs + contact_constraint_element.rs)
    solver/joint            JointConstraintHelper rows + finalize (Gram-Schmidt) + solve_generic
    solver/island           substep loop of §2.5 over dense solver bodies
L6  islands/sleep           union-find islands, activation timers, wake-up rules
    pipeline                World struct, user-change handling, step(), event output
L7  query                   ray cast / point / AABB queries over the broad phase; (later) CCD, character controller
```

Parallelisable work streams once L0/L1 interfaces are frozen:
**(A)** `integration_parameters` + solver/contact (only needs math + a mock manifold),
**(B)** rigid_body/collider/data sets,
**(C)** broad phase + narrow-phase bookkeeping (needs geometry agent's manifold API),
**(D)** joints + solver/joint,
**(E)** Rust golden-data harness (independent, should start first).
`pipeline` integrates last.

### 9.3 MVP (2D) vs 3D

**2D MVP** (targets: platformer/pinball/pool-like provable games):
dynamic/fixed/kinematic-velocity bodies; ball, cuboid, capsule (+ convex polygon) colliders; gravity, forces, impulses, damping, locked axes;
AABB broad phase (n^2 or grid); manifolds <= 2 points; soft-contact substep solver exactly as §2.5 with `num_solver_iterations = 4`
(expose it — it is the main gas/quality knob), warm starting, 1 tangent friction, restitution pass; collision groups; sensors;
events as returned array; fixed + revolute + prismatic joints (no limits) ; no sleeping, no CCD.
Math needed: `+ - * / sqrt min max clamp abs` only — **no trig in the step** (trig only to build a body from an angle).

**Phase 2 (2D complete):** joint limits + motors (`atan2`, `sin_cos`), rope/spring, kinematic position-based bodies, sleeping with
union-find islands, block solver, dominance, contact-force events, `modify_solver_contacts`-style one-way platforms (as a built-in flag), query pipeline.

**3D:** everything angular becomes 3x heavier: `SdpMatrix3` world inertia per body per step (quat->mat3 + `R D R^T`), quaternion normalise,
4-point manifolds with reduction, 2 tangents with coupled 2x2 solve + `cap_magnitude` sqrt per point per iteration, 6-row joints with
quaternion error bases (`diff_conj1_2_tr`), SAT/GJK-EPA in parry for boxes/convex. Estimate **4-6x the 2D gas per contact**. Do it only after 2D is benchmarked.

### 9.4 Hot loops — where Cairo steps will go

Let B = awake bodies, P = broad-phase pairs, M = solver manifolds, k = points per manifold (2 in 2D), J = joint rows, S = substeps (4).

1. **Solver sweeps**: per step `S * (1 biased + 1 relax)` passes over all M manifolds, plus `S` updates + warm starts. Per 2D manifold per substep:
   ~2x(update: 2 point transforms, 3 dots) + warmstart + 2 normal solves + (relax) 2 normal + 2 friction solves => roughly 150-250 fixed-point
   mul and a handful of `max/clamp`; **zero divisions in the sweep itself** if `cfm_factor`, `erp_inv_dt`, `inv_dt` are hoisted (the block solver
   adds ~5 divisions per pair per pass — consider leaving it off). Each solve also does 2 body velocity reads + 2 writes => **dict accesses
   dominate**: ~8 dict ops per constraint solve. Mitigation: gather both bodies' velocities once per manifold (as Rust does per chunk), solve all
   its points, scatter once. Cost ~ `O(S * M * k)`. This is the #1 gas consumer for stacked scenes.
2. **Constraint generation** (`generate`): per point 1 division for the normal effective mass + 1 per tangent; lever arms, cross products. `O(M * k)` once per step.
3. **Narrow phase manifold generation** (parry): cuboid-cuboid SAT + clipping in 2D is the expensive pair; ball pairs are nearly free (one sqrt).
   `O(P)`; on par with or above the solver for few-contact scenes. Contact recycling is the Rust answer; a cheap Cairo analogue is skipping
   pairs whose two bodies are both asleep/fixed.
4. **Broad phase**: n^2 AABB tests are 4 comparisons each — fine up to ~50-100 colliders; beyond that a dict-backed uniform grid (hash of cell
   coords => felt key) is simpler to prove than a BVH with refit/rotation.
5. **Integration**: `S * B` rotation normalisations (1 sqrt + 1 div, or 1 rsqrt via Newton) — small but non-negligible; consider normalising
   once per step instead of per substep (error stays O(dang^2)).
6. **Joint rows**: rebuilt every substep (Rust does this for stability): `S * joints * (helper build + Gram–Schmidt O(rows^2))`.
   A 2D revolute = 2 rows => cheap; a 3D fixed joint = 6 rows => 15 row-pair orthogonalisations per substep.
7. **State (de)serialisation**: not in Rust's profile at all, but on Starknet reading/writing the world from storage will rival compute.
   Keep the persistent state minimal: poses, velocities, warm-start impulses per contact point, sleep timers. Everything in
   §3.7 is per-step scratch and must never be stored.

### 9.5 Porting risks specific to fixed point

- Soft-contact coefficients are tuned for `dt = 1/60`, S = 4; changing S changes `dt_sub` and therefore `erp`/`cfm` — keep the formula, not the constants.
- `utils::inv` semantics (0 -> 0) must be preserved: infinite-mass bodies rely on `inv_mass = 0` and `inv(0) = 0`.
- Overflow: `dist * inv_dt` with a deep penetration and `inv_dt = 240` is fine; `Real::MAX` sentinels are not — replace before porting any formula that multiplies them.
- Rust compares `impulse == 0.0` to detect new contacts; in fixed point tiny impulses truncate to 0 more often — acceptable (only affects restitution gating).
- Determinism of iteration order replaces colouring order; document the chosen order (pair slot ascending) because it is part of the state-transition function.
