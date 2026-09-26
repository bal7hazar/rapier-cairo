# CC2 — continuous collision detection in the step (rapier's `CCDSolver`)

## 1. Read first
`AGENTS.md` (§7, incl. the SH1 lesson on new `match` arms); `docs/PLAN.md` (the CC row: CCD guards thin planks if the
game drops to 2 or 1 substeps; D7 / D9 as amended by BT2; BT1–BT4 results); CC1's report and code (#180 — the casts
you call: `cast_shapes`, `cast_shapes_nonlinear`, the sweep TOI); `docs/adr/0001-upstream-divergences.md`; on `main`:
`crates/rapier2d/src/{pipeline.cairo,pipeline/**,world.cairo,world/**}`, `crates/rapier_dynamics2d/src/{rigid_body_set.cairo,rigid_body_set/**,rigid_body.cairo,rigid_body/**}`
(the CCD flags RB stored), `crates/rapier_core/src/integration_parameters*` (`max_ccd_substeps`). Upstream
(`UP=/home/claude/git/refs`): `$UP/rapier/src/dynamics/ccd/{ccd_solver.rs,toi_entry.rs,mod.rs}` and whatever they
call, `$UP/rapier/src/pipeline/physics_pipeline/**` (where `solve_continuous` / `update_ccd_active_flags` /
`find_first_impact` run), `RigidBodyCcd` in `$UP/rapier/src/dynamics/rigid_body_components.rs`, the CCD members of
`RigidBody` / `RigidBodyBuilder` (`ccd_enabled`, `soft_ccd_prediction`, `enable_ccd`, …).

## 2. Scope (file allowlist)
`crates/rapier2d/src/{pipeline.cairo,pipeline/**,world.cairo,world/**}` (a new `pipeline/ccd/**` for the solver),
`crates/rapier_dynamics2d/src/{rigid_body_set.cairo,rigid_body_set/**,rigid_body.cairo,rigid_body/**}` (CCD
components and flags), `crates/rapier_core/src/{integration_parameters.cairo,integration_parameters/**}`, their
tests, the harness `tools/golden/src/**` (new CCD scenes; existing vectors byte-identical), vectors / fixtures / types
(additions only), `tools/golden/README.md`, `docs/API_PARITY.md` (regenerated), `scripts/api_parity.py` (entries you
justify), and the snapshots that move. Forbidden: the casts' kernels (CC1's; report defects under Escalations), the
contact solver, narrow-phase generators, `Scarb.toml`/`lib.cairo`.

## 3. Expected result
Upstream semantics: `CCDSolver` (`update_ccd_active_flags`, `find_first_impact`, `solve_continuous` with
`max_ccd_substeps`), `RigidBodyCcd` (`ccd_thickness`, `ccd_max_dist`, `ccd_active`, `ccd_enabled`,
`soft_ccd_prediction`), the `RigidBody` / `RigidBodyBuilder` CCD members, the pipeline wiring in upstream's order.
**Owner's guideline — parity unless it costs Cairo steps:** the pinned upstream makes `ccd_active` automatic for every
fast dynamic body (it sweeps fixed colliders even without `ccd_enabled`). Measure what that automatic mode costs on
G0's level windows (flight, impact, load) and on P3. If it costs steps, keep upstream's behaviour available behind an
explicit switch (an `IntegrationParameters` field or equivalent, default documented) and make the default the one
that costs nothing for bodies without `ccd_enabled`; register the choice in REPORT.md for ADR 0001. Bodies with
`ccd_enabled` get the full upstream behaviour. New persistent state goes into `WorldState` with a version bump and
WS's chunked round-trip tests.

## 4. Golden and constraints
Golden CCD scenes from rapier2d-f64 0.35.3 (the vendored copy): a small fast ball against a thin fixed plank at 1 and 4
substeps, with and without `ccd_enabled` (tunnels / does not tunnel exactly as upstream: impact step and positions
within bands), a fast body against a dynamic box with `ccd_enabled`, and a slow scene where CCD must not change
anything. **Worlds without an active CCD body keep their exact Cairo steps** (P3, level windows, golden replays:
before / after table); Sierra gas ≤ +1 %, explained; `gas/bytecode.size` regenerated. Measure a CCD step (gas and
exact steps) for one fast body vs 1 and 10 fixed colliders. ≤ 800 lines per file; ≤ 4 fuzz per module.

## 5. Tests
The golden scenes; table-driven unit tests (activation thresholds, TOI clamping, substep limit, soft CCD); WS round
trips with CCD state; existing tests unchanged; `python3 scripts/api_parity.py --check`.

## 6. Definition of done
Harness twice → zero diff. Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through
`scripts/build-shims/`): `scarb fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on rapier_core,
rapier_dynamics2d, rapier_golden and rapier2d (one at a time); snapshots with `--from-log`; `python3
scripts/bytecode_size.py snapshot`; `python3 scripts/api_parity.py` then `--check`. Never a workspace-wide run: CI is
the full gate. Rebase on `origin/main` before the PR. Conventional commits + trailer; push; `gh pr create --base main
--title "<what ships>" --body-file …`; wait for the checks to be registered, then `gh pr checks --watch` until green;
never merge; `REPORT.md` (Summary · Items · API · Automatic-CCD measurement and the default chosen · Golden results
and bands · Steps unchanged proof · CCD step costs · `WorldState` version · Deviations · Requested re-exports ·
Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
