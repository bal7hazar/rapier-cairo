# PK1 — package-size gates: measure the published crates, CI report, and what tests out of the sources would save

## 1. Read first
`AGENTS.md`; the owner's rule (via the programme, 2026-09-28): `/home/claude/projects/pm/decisions/2026-09-28-package-granularity-rule.md`
and the measurements `/home/claude/projects/pm/research/R7-package-granularity.md`. A published crate has ≤ 40,000
library lines (inline tests excluded); an empty consumer of the crate alone adds ≤ 5 s and ≤ 1 GB (peak RSS, cold) to
the no-dependency build; the typical closure of a product stays under 15 s and 3 GB; scopes follow the upstream module
tree, a facade keeps names and paths; zero extra Cairo steps; CI-checked. Programme measurements on alpha.6 (empty
consumer, cold): rapier_core 4.6 s / 0.9 GB, rapier_math 5.6 s / 1.4 GB, rapier_geometry2d 7.5 s / 1.9 GB,
rapier_dynamics2d 9.9 s / 2.4 GB, rapier2d 10.4 s / 2.5 GB. The shared script: `/home/claude/projects/nalgebra-cairo/scripts/consumer_cost.py`
(nalgebra-cairo #61, bff3462; its docstring is the spec) and nalgebra-cairo's `consumer_cost.toml` and CI jobs
(`.github/workflows/ci.yml`: a lines-only report step and a `consumer-cost` report-only job). Published crates here:
`scripts/release.sh` (`CRATES`): rapier_math, rapier_core, rapier_geometry2d, rapier_dynamics2d, rapier2d,
rapier2d_classes.

**Gate definitions (programme, 2026-09-28):** gate 2 (5 s / 1 GB) is the crate's **marginal** cost: cost(empty consumer
of the crate) − cost(empty consumer of its direct dependencies together); gate 3 (15 s / 3 GB) is a product's closure
over the no-dependency baseline (closures `rapier2d`, `game_classes`); a facade or a product is judged on gate 3. If the
script reports only the cost over the baseline, compute the marginal figure in your report (measure the
direct-dependency consumer with `--closure deps_of_<crate>=a,b`) and do not change the script (the programme asks
nalgebra-cairo to add a `marginal` column to the shared script).

## 2. Work
1. Copy `scripts/consumer_cost.py` **unchanged**; add `consumer_cost.toml` at the root (the rule's gates; closures:
   `rapier2d = ["rapier2d"]`, `game_classes = ["rapier2d_classes"]`, and any other product closure you can justify).
2. `publish = false` in `crates/{rapier_golden,rapier_testing,rapier_sink}/Scarb.toml` (orchestrator exception for
   these three lines), so the script sees exactly the six published crates; a CI step failing if a published crate lists
   `rapier_golden`, `rapier_testing` or `rapier_sink` under `[dependencies]` (not `[dev-dependencies]`).
3. CI (orchestrator exception for `.github/workflows/ci.yml`): the lines-only report step in an existing fast job and a
   `consumer-cost` job, **report only** (non-blocking; it becomes enforcing with alpha.7 by the orchestrator).
4. Measure locally, cold, under the project lock (`flock ~/orchestrator/locks/rapier-cairo.lock python3
   scripts/consumer_cost.py --modules --repeat 3 --json …`): every published crate and closure (lines, added s, added
   GB, verdict, per top-level module lines).
5. **What moving inline tests out of the published sources would save** (alexandria's layout: a `tests/` directory per
   package, excluded from the package), measured on throwaway copies (`/tmp`, never committed): strip the test-only code
   from each crate's sources and re-measure the consumer cost — rapier_geometry2d first (the largest test share), then the others. Also estimate the work: how many inline test modules use
   crate-private items (they cannot move to `tests/` as they are), and whether gas-snapshot keys would change (a key is
   the test's module path). **Do not move tests in this lot.**
6. If a crate fails a gate even with tests out: the smallest cut that follows the upstream module tree (e.g.
   rapier_geometry2d into shapes / queries / contact manifolds; what the facade re-exports), with the measured or
   estimated cost of each part — a plan only, for the programme.

## 3. Scope (file allowlist)
`scripts/consumer_cost.py` (copy), `consumer_cost.toml`, `.github/workflows/ci.yml` (the two additions only), the
`publish = false` line of the three internal crates, `docs/research/package-cost.md` (new: the tables, the test-move
gain and cost, the cut plan if any). Forbidden: any Cairo source, other `Scarb.toml` edits, `Scarb.lock` changes beyond
what scarb regenerates.

## 4. Definition of done
No Cairo change, so no snforge run is needed; `scarb build -p` of one crate to confirm the manifests; the script's
runs (under the lock, one at a time). Rebase on `origin/main` before the PR. Conventional commits + trailer; push; `gh pr
create --base main --title "<what ships>" --body-file …`; `gh pr checks --watch` until green; never merge; `REPORT.md`
(Summary · Per-crate table (lines, s, GB, verdict) · Closures · Tests-out gain per crate and cost of the move · Cut plan
if needed · CI · PR URL). Memory rules apply.

## 5. Work autonomously, do not ask questions, do not widen the scope.
