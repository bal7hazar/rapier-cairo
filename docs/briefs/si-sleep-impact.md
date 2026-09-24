# SI — why `ball_drop_sleep` diverges after the ball-on-ball impact

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D8, D9, D11; SL findings; how SD and SO located divergences: per-step
harness diagnostics + Cairo counterfactual tests, engine read-only); `tools/golden/README.md` (scenes, the
sleep scenes SL added, SD/SO diagnostics sections); SL's REPORT in PR #95's body (divergences 1–6:
per-step islands, **mixed islands wake strongly as a whole where upstream wakes the toucher weakly**,
partner wake-ups one step later, …; `ball_drop_sleep`: samples in tolerance up to step 80, sleep flags exact
at every step (sleep 66, wake 87), then after the ≈ 13 m/s ball-on-ball impact the samples diverge — 0.37 m
at step 110 — so the test judges `SamplesUntil(80)` + invariants). On `main`: `crates/rapier2d/src/pipeline/{islands.cairo,sleeping.cairo,user_changes.cairo}`,
`crates/rapier2d/tests/golden_scenes.cairo` + `golden_scenes/*`, `tools/golden/src/{scenes.rs,cairo.rs,cairo/scenes.rs}`.
Upstream (`UP=/home/claude/git/refs`): `$UP/rapier/src/dynamics/island_manager/{sleep.rs,manager.rs}`,
`$UP/rapier/src/dynamics/rigid_body_components.rs` (`RigidBodyActivation::wake_up(strong)`), the narrow
phase / pipeline code that wakes bodies when a contact starts, restitution handling in the contact solver.

## 2. Scope (file allowlist)
Harness diagnostics for `ball_drop_sleep` only (`tools/golden/src/**`, `tools/golden/vectors/scenes.json`
additions only, `crates/rapier_golden/src/types.cairo` new types only, generated fixtures, README scenes
section); Cairo diagnostics in `crates/rapier2d/tests/golden_scenes/sleep_diagnostics.cairo` (new) and
`golden_scenes.cairo`; the snapshots that move. **Engine files are read-only**: describe the exact fix
(file, function, before/after, numbers) under Escalations; exception: a fix ≤ 10 lines in one engine file
whose crate's tests all still pass may go in a SEPARATE commit, flagged.

## 3. Expected result
The first quantity that differs around the impact (steps ~85–95), among: (a) wake-up timing and strength
(which bodies are awake at which substep, their timers — SL's divergence 2), (b) the ball–ball manifold at
first contact (speculative contact, `dist`, NEW status), (c) restitution (first-contact bias vs restitution
pass, relative velocity threshold), (d) velocity carried by the previously sleeping ball (zeroed at sleep,
upstream too?), (e) anything else — each with a counterfactual test measuring how much of the gap it closes.
Then the recommendation (engine fix, or a documented legitimate divergence with numbers).

## 4. Efficiency
Not a gas package; do not regress snapshots outside your allowlist.

## 5. Tests
Counterfactual diagnostics pass and assert the measured recovery; harness regeneration idempotent; ≤ 800
lines per file; no fuzz.

## 6. Definition of done
`cargo run --release --locked` twice in `tools/golden` (no lock needed), zero diff on the second run; foreground
Cairo gate (tool timeout 3600000 ms; crate-scoped runs via `scripts/build-shims/snforge` while iterating);
module-filtered snapshots; `gas.py check`; conventional commits + trailer; push; `gh pr create` per template;
wait for the checks to be registered, then `gh pr checks --watch` until green; never merge; `REPORT.md`
(Summary · First divergence · Causes with counterfactual recovery · Recommendation · Escalations · PR URL).
Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
