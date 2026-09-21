# P1 — `rapier2d`: `World`, `step()`, events as a returned array

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D7–D10, wave-4 status and findings: Sierra gas is path-insensitive,
narrow-phase and proxy costs); `docs/interfaces/geometry-dynamics.md`;
`docs/research/01-rapier-analysis.md` §2 (order of operations in `PhysicsPipeline::step`: §2.1 user
changes, §2.2 collision detection, §2.4–2.5 solver, position update). On `main`, the pieces you
assemble and must NOT modify:
- `rapier_geometry2d`: `broad_phase::{BroadPhaseProxy, find_pairs}` (pairs are indices into the
  proxy span, ascending), `dispatch::contact_manifold` (GG; if `dispatch.cairo` is still a stub when
  you start, `git fetch && git rebase origin/main` — it is being merged — and if it is still missing,
  write the glue against the frozen signature of the interface doc §2 and test with DD's mock);
- `rapier_dynamics2d`: `RigidBodySet`/`RigidBody`, `ColliderSet` (`insert_with_parent`,
  `broad_phase_proxies(ref bodies, prediction)`), `NarrowPhase::compute_contacts::<D>(prediction, ref
  bodies, ref colliders, pairs) -> Array<CollisionEvent>` generic over `ContactDispatcher`,
  `propagate_modified_body_positions_to_colliders`, `joint::ImpulseJoint`,
  `solver::body_store::SolverBodyStoreTrait::{from_bodies(ref bodies, gravity, params), to_bodies}`,
  `solver::island::solve_island(params, ref store, ref contact_set, ref joint_set)` — read DD's and
  DF's module docs for what they leave to "the pipeline" (clearing `changes` flags, advancing
  `position` to `next_position`, kinematic velocities);
- `rapier_core::integration_parameters`, `rapier_core::data::{arena, handle}`.
Upstream (`UP=/home/claude/git/refs`): `$UP/rapier/src/pipeline/physics_pipeline/{mod.rs,substep.rs,solve.rs}`,
`$UP/rapier/src/pipeline/user_changes.rs`, `$UP/rapier/src/pipeline/physics_world.rs` (the bundle API
to mirror), `$UP/rapier/src/pipeline/event_handler.rs`. Golden: `rapier_golden::scenes`
(`BALL_DROP`, `BALL_BOUNCE`, `BOX_SLOPE_STICK/SLIDE`, `BOX_STACK3`, `PENDULUM`) and
`tools/golden/README.md` (scene settings: recycling, clustering, block solver, CCD, sleeping off).

## 2. Scope (file allowlist)
`crates/rapier2d/src/world.cairo`, `crates/rapier2d/src/pipeline.cairo` (+ `src/pipeline/*.cairo`),
`crates/rapier2d/src/dispatcher.cairo`, `crates/rapier2d/tests/world_step.cairo`,
`gas/rapier2d/{world,pipeline,dispatcher}.snap`, `gas/rapier2d_integrationtest/world_step.snap`.
The crate, its `Scarb.toml`, `lib.cairo` and the test stubs are pre-declared. `tests/golden_scenes.cairo`
and `tests/gas_scenes.cairo` belong to P2/P3: leave them untouched. Everything else is forbidden —
if a merged API blocks you, use it as-is and escalate.

## 3. Expected API and semantics
```cairo
pub struct World { gravity, integration_parameters, bodies: RigidBodySet, colliders: ColliderSet,
                   impulse_joints: Array<ImpulseJoint> (or a small set type), narrow_phase: NarrowPhase }
WorldTrait::{new(gravity, params), insert_body, insert_collider(collider, parent), insert_joint,
             remove_body / remove_collider (detaching what upstream detaches), body, set_body,
             collider, step(ref self) -> Array<CollisionEvent>}
```
mirroring upstream's `PhysicsWorld` names where they exist. `dispatcher.cairo`: a zero-size
`DefaultDispatcher` implementing `rapier_dynamics2d::narrow_phase::ContactDispatcher` by calling
`rapier_geometry2d::dispatch::contact_manifold`. `pipeline::step` in upstream order: (1) user
changes — propagate modified body positions to colliders, recompute mass properties from colliders
when a collider was attached/changed, `update_world_mass_properties`, clear change flags;
(2) `broad_phase_proxies` → `find_pairs` → `compute_contacts::<DefaultDispatcher>` with `prediction =
params.prediction_distance()`; (3) gather the manifolds with `num_solver_contacts > 0` in pair
order (D8) → `from_bodies` → `solve_island` → `to_bodies`; write the solved `ContactData`
(impulses) back into the narrow-phase pairs so that the next step warm-starts; (4) advance
`position ← next_position`, propagate to colliders; (5) return the collision events. State kept
between steps = D9 only (poses, velocities, warm-start impulses inside `narrow_phase.pairs`).
Determinism: every iteration in ascending handle/pair order; no dict iteration.
DEFER: sleeping/islands, CCD, hooks, contact-force events, sensors' intersection events, query
pipeline, kinematic position-based interpolation beyond what DF exposes, multi-world.

## 4. Efficiency and variants
`step` is the product's headline number. Report, in **Sierra gas and Cairo steps** (`snforge test
<name> --detailed-resources --tracked-resource cairo-steps`): one step of ball-on-ground (resting),
free fall (no pair), `BOX_STACK3`, `PENDULUM`; and the split user-changes / broad phase / narrow
phase / solver / position update for `BOX_STACK3`. Candidates to measure: (a) rebuilding the
proxies every step vs (b) skipping fixed bodies' proxies that did not change (cache in `World`, D9
allows it only if measured cheaper including storage — report, ship the cheaper); passing only
touching manifolds to the solver vs all of them. Put each computing `match` arm behind an
`#[inline(never)]` helper (AGENTS.md §7).

**Dispatcher cost model (GG, PR #48 — read its REPORT in the PR body):** `dispatch::contact_manifold`
is `#[inline(always)]`; inlined in a test, each pair pays only its own generator (ball–ball 41k,
cuboid–segment 528k Sierra gas), but behind ANY outlined function the `match` is charged its most
expensive arm on every call (flat 556k) — and DD's `process_pair`/`update_manifold` are outlined
functions called from the pair loop. You must therefore MEASURE, through the real `step`, the
narrow-phase cost per pair of a ball-only world vs a cuboid-only world (same pair count). If a
ball–ball pair pays cuboid prices, that is this package's main finding: report it with numbers, try
within your allowlist (a) an `#[inline(always)]` `DefaultDispatcher::contact_manifold`, (b) per-kind
pair buckets (group pairs by shape-pair kind, one loop per kind so that each loop body reaches a
single generator), and escalate what would have to change in DD (`narrow_phase.cairo` is not yours)
with the measured gain of each option. Same question for Cairo steps (expected: no penalty).

## 5. Tests
`tests/world_step.cairo`, table-driven: free fall matches DA's closed form; a ball dropped on a
halfspace comes to rest and emits exactly one `Started`; removing the ball emits `Stopped`; a
collider attached later changes the body mass; a fixed body never moves; warm start: the second
resting step's `contact_id` has no `NEW_CONTACT_BIT`; `BALL_DROP` replayed for its first 30 sampled
steps within README tolerances (the full scene matrix is P2's job). `fuzz_*` ≤ 4; `gas_*` per
public function and candidate; ≤ 800 lines per file.

## 6. Definition of done
Foreground gate (`scarb fmt --workspace && scarb lint --workspace --deny-warnings && scarb build
--workspace && snforge test --workspace`); `python3 scripts/gas.py snapshot --filter rapier2d::world`,
`…rapier2d::pipeline`, `…rapier2d::dispatcher`, `…rapier2d_integrationtest::world_step`;
`python3 scripts/gas.py check`; conventional commits + trailer; push; `gh pr create` per template;
`gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · API · Gas table in gas and
steps incl. the per-stage split · Deviations · Deferred · Requested re-exports (a `prelude`) ·
Escalations · PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.
