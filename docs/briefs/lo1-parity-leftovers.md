# LO1 — parity leftovers: rigid-body, collider and sensor packages (AP, ≈ 64 items)

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (programme decisions: free parity items welcome, nothing that costs Cairo steps);
`docs/API_PARITY.md` (sections "WP: Rigid-body API completion", "WP: Collider API completion", "WP: Sensors and
intersection events" and their owner tables — the exact items; CW #135's and QY1 #152's reports list why some stayed
`missing`: keep those reasons unless you can close them); `scripts/api_parity.py` (`OWNER_ALIASES`,
`METHOD_RENAMES`, closed exclusion reasons); on `main`: `crates/rapier_dynamics2d/src/{rigid_body_set.cairo,rigid_body_set/**,rigid_body.cairo,rigid_body/**,collider_set.cairo,collider/**}`,
`crates/rapier_geometry2d/src/{query.cairo,query/**,dispatch/intersection.cairo}`, `crates/rapier_core/src/{data.cairo,data/**}`.

## 2. Scope (file allowlist)
API-only additions in the files above, their tests, `scripts/api_parity.py` (`OWNER_ALIASES` / `METHOD_RENAMES` /
closed-reason patterns you justify), `docs/API_PARITY.md` (regenerated), and the snapshots that move. Forbidden:
layouts of the structs the step copies, the pipeline, narrow phase, solver, `crates/rapier2d/src/**`,
`crates/rapier_geometry2d/src/{shape.cairo,shape/**}` and `crates/rapier_core/src/{interaction_groups.cairo,integration_parameters*,rigid_body*}`
(PO1 works there in parallel), `Scarb.toml`/`lib.cairo`.

## 3. Expected result
For each item of the three packages: implement it (upstream name and semantics: `RigidBodySet` / `RigidBodyActivation`
/ `RigidBodyHandle` / `RigidBodyIds` / `RigidBodyChanges` members, `ColliderSet` / `ColliderBuilder` members, parry's
`intersection_test_*` entry points as thin wrappers over the existing kernels with upstream's signatures, …), or map it
(`METHOD_RENAMES` when the semantics match), or leave it `missing` with a one-line reason (no interaction graph,
persistent islands, BVH, parry 0.31-only wrappers, …). A triage table in REPORT.md.

## 4. Constraints
No step cost: every P3, level and scene probe identical in Sierra gas and exact Cairo steps; `gas/bytecode.size`
unchanged. `#[inline(always)]` trivial accessors. ≤ 800 lines per file; ≤ 4 fuzz per module.

## 5. Tests
Table-driven per owner; existing tests unchanged; `python3 scripts/api_parity.py --check` passes.

## 6. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/`):
`scarb fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on the crates you touch; snapshots with
`snforge test -p <crate> --tracked-resource sierra-gas > gas-<crate>.log` and `python3 scripts/gas.py snapshot --filter
<crate>::<module> --from-log gas-<crate>.log`; `python3 scripts/api_parity.py` then `--check`; `python3
scripts/bytecode_size.py check`. Never a workspace-wide run: CI is the full gate. Before the PR, rebase on `origin/main`
(PO1 may have merged; regenerate `docs/API_PARITY.md` if it conflicts). Conventional commits + trailer; push; `gh pr
create --base main --title "<what ships>" --body-file …`; wait for the checks to be registered, then `gh pr checks
--watch` until green; never merge; `REPORT.md` (Summary · Triage table · API · Deviations · Requested re-exports ·
Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
