# LO2 — five trivial parity leftovers (LO1's escalation)

## 1. Read first
`AGENTS.md`; LO1's report in PR #170 ("Escalations": the items below were outside its allowlist); `docs/API_PARITY.md`
(owner tables `Aabb`, `NarrowPhase`, `MotorModel`, `RayIntersection`); on `main`: `crates/rapier_geometry2d/src/{aabb.cairo,aabb/**,ray.cairo,ray/**}`,
`crates/rapier_dynamics2d/src/{narrow_phase.cairo,narrow_phase/**,joint.cairo,joint/config.cairo}`. Upstream
(`UP=/home/claude/git/refs`): the source files named in the items' rows.

## 2. Scope (file allowlist)
API-only additions in the files above, their tests, `docs/API_PARITY.md` (regenerated), and the snapshots that move.
Forbidden: anything the step reaches beyond adding functions, `crates/rapier_geometry2d/src/query/**` and
`crates/rapier2d/src/queries/**` (CC1 works there), `Scarb.toml`/`lib.cairo`.

## 3. Expected result
Upstream names and semantics for: `Aabb::aligned_intersections`, `Aabb::intersects_moving_aabb`,
`NarrowPhase::{intersection_pair_unknown_gen, intersection_pairs_with_unknown_gen}`, `MotorModel::combine_coefficients`,
`RayIntersection::with_subshape` (sub-shape ids exist only for composites: implement it as upstream, storing the id).
Each `ported`, or left `missing` with a one-line reason.

## 4. Constraints
No step cost: every P3, level and scene probe identical; `gas/bytecode.size` unchanged. ≤ 800 lines per file.

## 5. Tests
Table-driven per item; existing tests unchanged; `python3 scripts/api_parity.py --check`.

## 6. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, through `scripts/build-shims/`): `scarb fmt --workspace`; `scarb
lint -p` / `scarb build -p` / `snforge test -p` on rapier_geometry2d and rapier_dynamics2d; snapshots with `--from-log`;
`python3 scripts/api_parity.py` then `--check`; `python3 scripts/bytecode_size.py check`. Never a workspace-wide run.
Rebase on `origin/main` before the PR. Conventional commits + trailer; push; `gh pr create --base main --title "<what
ships>" --body-file …`; `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · Items · API ·
Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
