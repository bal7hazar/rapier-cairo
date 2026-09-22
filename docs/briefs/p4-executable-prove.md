# P4 — `examples/`: an `#[executable]` physics step for `scarb execute` and `scarb prove`

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D10 pure Cairo, P4 milestone "a proven physics step", open question 2);
on `main`: `crates/rapier2d/src/lib.cairo` (`prelude`), `crates/rapier2d/tests/world_step.cairo`
(building a world), `Scarb.toml` (workspace members = `crates/*`; the example lives OUTSIDE the
workspace members list on purpose, see §2). Scarb docs for executables: `scarb execute` /
`scarb prove` / `scarb verify` (scarb 2.19.4 ships `scarb-execute`, `scarb-prove`, `scarb-verify`;
run `scarb execute --help` and `scarb prove --help`), the `executable` target and
`enable-gas = false` requirement, `cairo_execute` dependency.

## 2. Scope (file allowlist)
`examples/ball_drop/Scarb.toml`, `examples/ball_drop/src/lib.cairo`, `examples/ball_drop/README.md`,
`examples/ball_drop/.gitignore` (target/), `scripts/prove-example.sh`. The example is a standalone
package depending on `rapier2d` by path (`../../crates/rapier2d`); do NOT add it to the workspace
members (executable targets need `enable-gas = false`, incompatible with the snforge crates) —
if scarb refuses a nested package outside the workspace, put it under `examples/` with its own
`[workspace]` table and escalate what the root `Scarb.toml` would need. No CI change (orchestrator
adds the nightly job from your script), no engine change.

## 3. Expected content
`#[executable] fn main(args) -> ...`: build a small scene (a ball dropped on a half-space, or the
scene chosen by an input argument among ball drop / box stack 3 / pendulum), run `n` steps from the
input, return the final poses/velocities as felts (raw `i64` values) plus the number of collision
events. `scripts/prove-example.sh [scene] [steps]`: `scarb execute` (print the output), then `scarb
prove` and `scarb verify`, timing each stage, exiting non-zero on failure. README: how to run,
measured timings on this machine for 1 / 10 / 60 steps, proof size, and which stage dominates.
DEFER: on-chain verification, Starknet contract wrapper (`rapier_starknet`), inputs from JSON.

## 4. Efficiency
Report `scarb execute --print-resource-usage` (steps, builtins) for 1 / 10 / 60 steps of each scene
next to the `snforge` numbers of P1/P3 for the same scene: they must agree on Cairo steps within
a few %; explain any gap. Note whether proving time scales with steps.

## 5. Tests
The script run for real is the test (execute + prove + verify green for the default scene, 10
steps); add a `#[cfg(test)]` unit test in the example that `main` returns the expected number of
outputs for 1 step. No fuzz.

## 6. Definition of done
Foreground: `scarb build` in the example, `scripts/prove-example.sh` green, root workspace gate
untouched (`scarb fmt --check --workspace` must still pass — format the example with `scarb fmt`
inside its directory); conventional commits + trailer; push; `gh pr create` per template; `gh pr
checks --watch` until green; never merge; `REPORT.md` (Summary · Commands · Timings and resources
table · Deviations · Deferred · Escalations (incl. the CI job you recommend) · PR URL). Memory rules
apply: `scarb prove` may use several GB — run nothing else meanwhile.

## 7. Work autonomously, do not ask questions, do not widen the scope.
