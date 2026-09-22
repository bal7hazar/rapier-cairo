# P2 — end-to-end golden scenes through `rapier2d::World::step`

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D11 tolerance-based validation, wave-1 outcomes); `tools/golden/README.md`
(`scenes` section and the settings table: recycling, clustering, block solver, CCD, sleeping off;
tolerances); on `main`: `crates/rapier_golden/src/types.cairo` (`SceneCase`, `SceneBodyRaw`,
`SceneColliderRaw`, `SceneJointRaw`, `SceneSampleRaw`, `BodyStateRaw`), `crates/rapier_golden/src/generated/scenes.cairo`
(`BALL_DROP`, `BALL_BOUNCE`, `BOX_SLOPE_STICK`, `BOX_SLOPE_SLIDE`, `BOX_STACK3`, `PENDULUM`),
`crates/rapier2d/src/{world.cairo,pipeline.cairo}` and `crates/rapier2d/tests/world_step.cairo` (P1:
how `World` is built from a scene, how `BALL_DROP`/`PENDULUM` were replayed for 30 / 10 steps),
`crates/rapier_dynamics2d/tests/substep_scenes.cairo` (DF's replays with analytic manifolds and its
note on the VM step limit: 120 continuous steps needed a raised limit, CI uses two 60-step windows
re-seeded from the upstream sample at step 60), `crates/rapier2d/src/lib.cairo` (`prelude`).

## 2. Scope (file allowlist)
`crates/rapier2d/tests/golden_scenes.cairo` (+ `crates/rapier2d/tests/golden_scenes/*.cairo` if
you split: a shared scene builder module is expected), `gas/rapier2d_integrationtest/golden_scenes.snap`.
Everything else is forbidden: a divergence is reported, not patched in the engine (escalate with the
scene, the step, the quantity and the delta).

## 3. Expected content and semantics
One table-driven test per scene replaying **all 120 steps** through `World::step` and comparing every
sampled state (`translation`, `rotation`, `linvel`, `angvel`) with the trace within the README
tolerances; keep DF's windowing (re-seed from the sample at step 60) if the VM step limit forces it,
say so per scene. A generic `build_world(scene: SceneCase) -> World` from the fixture (bodies in
insertion order, colliders with material, joints; gravity and `dt` from the case; integration
parameters = upstream defaults with `num_solver_iterations` as the README states). Report per scene
the max deviation in ulps for each quantity and the step where it occurs, and the trend (drift vs
bounded). A scene that exceeds the tolerance is a **finding, not a failure to hide**: mark the
test `#[ignore]` with the reason, keep the assertion code, and put the numbers under Escalations.
DEFER: new scenes (need the Rust harness), sleeping, CCD.

## 4. Efficiency
Not a gas package, but every test doubles as a probe: keep the six scene tests as the only
`gas_*`-relevant entries (no per-step probes here, P3 owns budgets). Compile budget: ≤ 800 lines
per file, no fuzz.

## 5. Tests
The six scenes; plus invariants over the whole run: fixed bodies never move, energy never increases
in `ball_drop` / `box_stack3` (no restitution), `pendulum` rod length constant within tolerance.

## 6. Definition of done
Foreground gate; `python3 scripts/gas.py snapshot --filter rapier2d_integrationtest::golden_scenes`;
`gas.py check`; conventional commits + trailer; push; `gh pr create` per template; `gh pr checks
--watch` until green; never merge; `REPORT.md` (Summary · Scene table (max ulps per quantity, step,
windowed?) · Gas table · Deviations · Deferred · Requested re-exports · Escalations · PR URL).
Memory rules of the system prompt apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
