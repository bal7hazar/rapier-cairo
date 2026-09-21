# DD — `rapier_dynamics2d::{rigid_body_set, collider_set, narrow_phase}`: sets and narrow-phase bookkeeping

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D8 pair order, D9 persistent state, wave-1 outcome: arena for
persistent sets, dense arrays for per-step scratch); `docs/interfaces/geometry-dynamics.md` §3, §5;
`docs/research/01-rapier-analysis.md` §2.3 (narrow phase: pair table, `SolverContact` build, events);
on `main`: `crates/rapier_core/src/data/{arena,handle}.cairo`, `interaction_groups.cairo`,
`crates/rapier_core/src/collider/*.cairo` (`ActiveCollisionTypes`, `ActiveEvents`, combine rules),
`crates/rapier_dynamics2d/src/{rigid_body.cairo,collider.cairo}` (DA components, DB `Collider`),
`crates/rapier_geometry2d/src/{contact.cairo,manifold.cairo,broad_phase.cairo,dispatch.cairo}`
(frozen types, GE persistence, GA `find_pairs`, GG `contact_manifold` dispatch). Upstream
(`UP=/home/claude/git/refs`):
`$UP/rapier/src/dynamics/rigid_body_set.rs`, `$UP/rapier/src/geometry/collider_set.rs`,
`$UP/rapier/src/geometry/narrow_phase/` (`process_pair`: pair filtering by body types, collision
groups, `ActiveCollisionTypes`; manifold generation; `SolverContact` localisation with
prediction skipping; friction/restitution combination; `ContactManifoldData` filling; collision
events), `$UP/rapier/src/geometry/contact_pair.rs` (`ContactPair`, `PairEventStatus`),
`$UP/rapier/src/pipeline/event_handler.rs` (`CollisionEvent`, `ContactForceEvent` — events as
returned arrays here, D9).

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/rigid_body_set.cairo`, `crates/rapier_dynamics2d/src/collider_set.cairo`,
`crates/rapier_dynamics2d/src/narrow_phase.cairo` (+ `src/narrow_phase/*.cairo`),
`crates/rapier_dynamics2d/src/events.cairo`, `crates/rapier_dynamics2d/tests/narrow_phase_scenes.cairo`,
`gas/rapier_dynamics2d/{rigid_body_set,collider_set,narrow_phase,events}.snap`,
`gas/rapier_dynamics2d_integrationtest/narrow_phase_scenes.snap`. Modules pre-declared.

## 3. Expected API and semantics
`RigidBody` (assembled from DA components + `rapier_core` scalar components: `body_type`,
`activation`, `damping`, `dominance`, `changes`, `colliders: Array<Handle>` or a fixed small array
— pick and justify), `RigidBodySet` over `rapier_core::data::arena::Arena<RigidBody>` with
`insert`, `get`, `get_mut`-style `update(handle, f)` or `set`, `remove`, `len`, `iter` in ascending
index order, `propagate_modified_body_positions_to_colliders`; `ColliderSet` likewise with
`insert_with_parent(collider, body, ref bodies)` maintaining `body.colliders`;
`NarrowPhase { pairs: Array<ContactPair> }` rebuilt every step from `find_pairs` (stateless, D7)
but **carrying the previous step's manifolds forward by `(collider1, collider2)` key for warm
start** — implement the carry-over with a `Felt252Dict` keyed by the packed handle pair and
measure it against a sorted-merge on the ascending pair order (both candidates; the sorted merge
needs no dict); `compute_contacts(prediction, ref bodies, ref colliders, pairs) -> Array<CollisionEvent>`
doing upstream's `process_pair`: filters (`ActiveCollisionTypes::test`, collision groups,
sensors, both bodies non-dynamic → skip), `contact_manifold` (GG) with `match_contacts` (GE),
`SolverContact` build (world anchors relative to each body's COM via DA `world_com`, skip points
with `dist > prediction`, `NEW_CONTACT_BIT` for unmatched points), `friction`/`restitution` via
`CoefficientCombineRule::combine`, `relative_dominance`, `solver_flags`, `rigid_body1/2`;
`CollisionEvent::{Started, Stopped}` derived from `PairEventStatus` transitions, returned as an
array.
**Dispatch dependency:** GG is not merged yet (`rapier_geometry2d::dispatch` is an empty stub and the
GF generators land in parallel with you). Mirror upstream, where `NarrowPhase` receives a
`&dyn PersistentQueryDispatcher`: declare in `narrow_phase.cairo`
`pub trait ContactDispatcher { fn contact_manifold(pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold) -> bool; }`
(the frozen GG signature, interface doc §2) and make `compute_contacts` generic over an impl of it
(static dispatch, no runtime cost). Tests use a small mock impl written in your test module/file
(exact ball–ball and axis-aligned cuboid–halfspace manifolds are enough for the scenes of §5);
P1 will plug GG's `dispatch::contact_manifold` in. Do not touch `rapier_geometry2d`.
DEFER: sensors' intersection events (keep the filter), contact-force events, hooks,
modification flags (`RigidBodyChanges`) propagation beyond positions, islands.

## 4. Efficiency and variants
The per-step cost is `O(pairs)`: keep the pair loop free of dict access except the carry-over
lookup; bench `compute_contacts` for 2, 8, 32 bodies in a stack and a sparse layout, with and
without warm-start carry-over, both carry-over candidates. `Arena` operations are measured
already (C1); use `Array` scratch for per-step data (D9).

## 5. Tests
Scenes: two boxes on a halfspace (manifolds, anchors, friction combined by Max rule as upstream
default, `Started` events on first contact, `Stopped` when separated beyond prediction), a sensor
producing no manifold, collision groups excluding a pair, kinematic vs fixed pair skipped,
warm-start carried over across two steps (impulses set on step 1 are visible in step 2's
`SolverContact.contact_id` without `NEW_CONTACT_BIT`). Table-driven, `fuzz_*` ≤ 4, `gas_*` per
public function and candidate. ≤ 800 lines/file.

## 6. Definition of done
Foreground gate (`scarb fmt --workspace && scarb lint --workspace --deny-warnings && scarb build
--workspace && snforge test --workspace`); snapshot filters for each owned module; conventional
commits + trailer; push; `gh pr create` per template; `gh pr checks --watch` until green; never
merge; `REPORT.md` (Summary · API · Gas table · Deviations · Deferred · Requested re-exports ·
Escalations · PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.
