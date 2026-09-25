# G0 — level-shaped golden scene and the cost of a level

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` ("Programme target", "Cost of a level" paragraphs; SL sleeping, SE sensors, KD kinematic,
EV force events); `docs/BUDGETS.md`; `tools/golden/README.md` (scenes, tolerances); `crates/rapier2d/tests/{golden_scenes.cairo,golden_scenes/*,gas_scenes.cairo}`;
the programme's specification `~/projects/pm/messages/to-rapier/2026-09-25-level-budget-g0-and-rapier2d.md` §2 and
`~/projects/pm/research/R2-game-design-and-client.md` (read-only). Upstream (`UP=/home/claude/git/refs`): rapier's
scene settings in the harness.

## 2. Scope (file allowlist)
The harness `tools/golden/src/**` (new level scenes; existing vectors byte-identical), `tools/golden/vectors/scenes.json`
(additions), generated fixtures, `crates/rapier_golden/src/types.cairo` (additions), `tools/golden/README.md`,
`crates/rapier2d/tests/golden_scenes.cairo` + `golden_scenes/*` (replays), `crates/rapier2d/tests/gas_scenes.cairo`
(level probes + `steps_*` twins, +10 % ceilings) — or a new `crates/rapier2d/tests/level_budget.cairo` if the per-tick
matrix does not fit the P3 file's budget —, and the snapshots that move. **No engine change**: if the scene exposes a
bug, reproduce it with a failing `#[ignore]`d test and escalate (as KD did).

## 3. The scene (programme spec)
Half-space ground; **8–12 cuboid / convex-polygon blocks** in a pre-settled structure, **asleep at t = 0**; 2–3 small
"core" bodies (ball or small cuboid) on / in the structure (sensor targets: one sensor collider per core, or the core
itself as the target — follow the spec, say which); a "pebble" ball r = 0.25 m, density 4, launched with `set_linvel`
at 15–25 m/s from a point 6–10 m away; 60 Hz, 4 substeps, **300 ticks**; plus a **20-block variant**. Golden trace
against rapier-rs with the usual tolerance bands (sleeping ON in these scenes, as the game will run).

## 4. Measurements (the product of this lot)
Per tick and for the whole run, in **Sierra gas AND exact Cairo steps**: the 10-block and 20-block levels at substeps
4, 2 and 1 and at 30 Hz (4 substeps); the number of awake bodies per tick (average and max); the tick at which
everything sleeps; the share of the step spent in broad phase / narrow phase / solver / sleeping bookkeeping for the
first 60 ticks (the impact). Put the matrix in REPORT.md (the orchestrator copies it into `docs/BUDGETS.md` and writes
the "cost of a level" paragraph). Golden fidelity at substeps 2 / 1 and 30 Hz: say whether the traces stay within
tolerance (they are generated at 4 substeps 60 Hz; generate matching upstream traces for the other settings if that is
the only honest comparison). Also compare the programme's "calm" rule (all awake bodies below a velocity threshold for
20 ticks, then `sleep()` all) with the engine's own sleeping: ticks and gas saved.

## 5. Tests
The replays (strict where possible, invariants otherwise, with reasons); the probes; ≤ 800 lines per file; no fuzz.

## 6. Definition of done
Harness twice → zero diff (cargo takes no lock); crate-scoped local gate only (AGENTS §6; foreground, tool timeout
3600000 ms, through `scripts/build-shims/`): `scarb fmt --workspace`, `scarb lint -p` / `snforge test -p` on rapier_golden
and rapier2d; snapshots with `snforge test -p rapier2d --tracked-resource sierra-gas > gas-rapier2d.log` and
`python3 scripts/gas.py snapshot --filter rapier2d_integrationtest::<module> --from-log gas-rapier2d.log`; never a
workspace-wide run (CI is the full gate); `api_parity.py --check`;
conventional commits + trailer; push; `gh pr create --base main --title "<what ships>" --body-file …`; wait for the
checks to be registered, then `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · Scene · Golden
results · Budget matrix · Calm rule vs engine sleeping · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
