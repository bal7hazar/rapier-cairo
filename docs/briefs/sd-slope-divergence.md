# SD — why `box_slope_stick` / `box_slope_slide` diverge from upstream at the first contact step

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D8 sequential solver order, D11 tolerance-based validation, wave-1
outcomes: f64 cuboid feature-id bug, `ambiguous` cases); `tools/golden/README.md` (scenes, tolerances,
settings table); on `main`: `crates/rapier2d/tests/golden_scenes.cairo` and `golden_scenes/builder.cairo`
(P2: the two `#[ignore]`d slope tests and their measured gaps — first divergence at **step 3**, the
first contact step: stick linvel 2.46e6 / 1.58e6 ulp, angvel 4.5e6; slide vx 4.9e5; by step 4–5
vy 5.2e8, angvel 6.0e8; warm start off makes it worse; late step-end velocities agree to 2e3 ulp
while positions drift 3.4e4 ulp per step), `crates/rapier2d/src/pipeline.cairo` (P1 step order),
`crates/rapier_dynamics2d/src/solver/island.cairo` + `island/sweeps.cairo` (DF substep loop),
`crates/rapier_dynamics2d/src/solver/contact/*.cairo` (DC constraint rows, bias, restitution),
`crates/rapier_dynamics2d/src/narrow_phase.cairo` (DD `solver_contact`: anchors, prediction skip),
`crates/rapier_geometry2d/src/contact_generators/cuboid_cuboid.cairo` (GF2), `polygonal_feature.cairo`,
`clip.cairo` (GD). `tools/golden/src/{scenes.rs,manifolds.rs,q.rs}` (the Rust harness).
Upstream (`UP=/home/claude/git/refs`): `$UP/rapier/src/dynamics/solver/contact_constraint/`,
`$UP/rapier/src/dynamics/solver/staged_island_solver/worker.rs`, `$UP/rapier/src/geometry/narrow_phase/pair_update.rs`
(`SolverContact` build, `dist` vs prediction), `$UP/parry/src/query/contact_manifolds/contact_manifolds_cuboid_cuboid.rs`.

## 2. Scope (file allowlist)
Rust harness: `tools/golden/src/scenes.rs` (new per-step diagnostics for the two slope scenes ONLY:
after each of steps 1–10, the contact manifolds of the box–slope pair — `local_n1/2`, every point's
`local_p1/2`, `dist`, `fid1/2`, `impulse`, `tangent_impulse` — and the body velocities after each
substep are NOT available from the public API; emit what `NarrowPhase::contact_pair` and
`RigidBody` expose after `step`, plus the solver contacts if reachable), `tools/golden/vectors/scenes.json`
(additions only), `crates/rapier_golden/src/types.cairo` (new types only, never change existing),
`crates/rapier_golden/src/generated/scenes.cairo` (generated), `tools/golden/README.md` (scenes
section). Cairo side: `crates/rapier2d/tests/golden_scenes.cairo` + `golden_scenes/*.cairo` (the
comparison at step 3, un-`#[ignore]` when fixed), `gas/rapier2d_integrationtest/golden_scenes.snap`,
`gas/rapier_golden_integrationtest/sanity.snap` if it drifts. **Engine files are not in the
allowlist**: when you have located the bug, describe the exact fix (file, function, lines,
before/after, numbers) under Escalations and STOP; the orchestrator briefs the fix as its own
lot in the owning package. Exception: if the fix is ≤ 10 lines in one engine file and all existing
tests of that crate still pass, you may include it in a SEPARATE commit and say so — the reviewer
decides.

## 3. Expected result
The step-3 state of both slope scenes explained, quantity by quantity: which of (a) manifold
geometry (points, order, `dist`, feature ids), (b) `SolverContact` build (anchors, prediction
skip, `NEW_CONTACT_BIT`), (c) constraint coefficients (bias, `erp`, `cfm`, friction combine, the
2-point sequential order vs upstream's block/ordering), (d) substep integration or position
update differs first, with the upstream value, the port value and the delta in ulps and in SI.
Then the fix (or its precise description). Also explain the `slide` oddity (positions drift while
step-end velocities agree): report the per-substep velocities of the port and reason about what
upstream's `integrate_linearized` + `next_position` do differently.

## 4. Efficiency
Not a gas package. Do not regress any snapshot outside your allowlist.

## 5. Tests
The two slope replays pass within the README tolerance (or the tolerance is justified per
quantity in the README with the upstream root cause, if the divergence is a legitimate f64-vs-Q32.32
effect — prove it with the numbers, e.g. by showing the same gap in an f32 run of the harness).
Golden regeneration idempotent (`cargo run --release --locked` twice, no diff on existing cases).
≤ 800 lines per file, no fuzz.

## 6. Definition of done
Foreground gate; snapshot filters `rapier2d_integrationtest::golden_scenes` (+ `rapier_golden_integrationtest::sanity`
if touched); `gas.py check`; conventional commits + trailer; push; `gh pr create` per template; `gh pr
checks --watch` until green; never merge; `REPORT.md` (Summary · Root cause per quantity · Fix ·
Gas table · Deviations · Deferred · Escalations · PR URL). Memory rules of the system prompt apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
