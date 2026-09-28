# CX1 — cheaper crossings for solve-and-advance and islands (class split: the step optimisation)

## 1. Read first
`AGENTS.md`; `docs/research/class-split.md` (CS3–CS5); CS5's report figures: on the 151-tick pile10 shot every stage out
costs 32.91M steps (+46.7 % over 22.43M in process), of which solve-and-advance is 37,960 steps × 152 calls (≈ 5.8M) and
islands 13,741 × 67; `SolveAdvanceClass` is 70,402 CASM felts, `IslandsClass` 16,908. On `main`:
`crates/rapier2d_classes/src/{advance,islands,arena,config}.cairo`, `tests/steps.cairo`,
`crates/rapier2d/src/pipeline/stages.cairo` (the slot traits), the stage functions they call in `crates/rapier2d/src/pipeline/**`
and `crates/rapier_dynamics2d/src/solver/**`; ADR 0001 entries 39, 40.

**Programme decision (2026-09-28):** go, in parallel with CS6 (which owns the caller's size). Size is the gate (every
class ≤ 73,728 Sierra and CASM, SNIP-36 clean), steps are this lot's objective (target +25 % over in process on the
shot).

## 2. Work
1. **Awake bodies only and compact deltas** across solve-and-advance: send only what the stage reads (awake / touched
   bodies and their colliders, the manifolds it solves), return only what changed; measure the felts and steps per call
   before / after.
2. **Fewer crossings per step:** fold stages whose calls happen on the same ticks where the class stays under the limit
   **with the 10 % margin** — islands into `SolveAdvanceClass` only if the result is ≤ 73,728 in both Sierra and CASM,
   otherwise no; other merges (e.g. broad phase with contacts) are in scope if they save steps and fit.
3. **Coverage the pile10 shot lacks** (codex audit of CS5): a bit-identity test `crates/rapier2d_classes/tests/removals.cairo`
   comparing every split layout with in process at every tick on a scenario with despawns mid-run (arena holes, reused
   slots with new generations), a body with several colliders, a kinematic body, a standalone collider (no parent) and a
   sleeping island woken by an impact; fix any divergence it finds in the crossings / remapping (`advance`, `islands`,
   `arena`).
4. Report the pile10 shot per layout (CS5's all-out, and yours): total steps, Δ, transactions of ≤ 10M steps, per-call
   steps and felts per class.

## 3. Scope (file allowlist)
`crates/rapier2d_classes/src/{advance,islands,arena}.cairo` and new files of yours in `crates/rapier2d_classes/src/`,
`crates/rapier2d_classes/tests/**` (your probes; keep CS6's tests passing), the solve-and-advance / islands slot traits
in `crates/rapier2d/src/pipeline/stages.cairo` (only those two; their in-process impls must stay forwarding unchanged),
`pub` / `Serde` seams in `crates/rapier_dynamics2d/src/solver/**` and `crates/rapier2d/src/pipeline/**` (no logic change),
`gas/bytecode.size` (your classes' lines), the snapshots that move. Forbidden: `crates/rapier2d_classes/src/config.cairo`
beyond wiring your classes, the caller fixtures and `scripts/bytecode_size.py` (CS6's; ask under Escalations), the
`WorldState` bytes, `Scarb.toml` / `lib.cairo` of the engine crates, `.github/**`.

## 4. Gate
Every declared class ≤ 73,728 Sierra and CASM (`bytecode_size.py check` green, SNIP-36 checks included); bit-identical
to in process on the pile10 shot and on `tests/removals.cairo` at every tick; in-process users unchanged (`program.basic` 231,196; every `steps_*`
probe of rapier2d and the CCD tests identical in exact Cairo steps).

## 5. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/`): `scarb fmt
--workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on the crates you touch (one at a time); snapshots
with `--from-log`; `python3 scripts/bytecode_size.py snapshot` then `check`. Never a workspace-wide run. Rebase on
`origin/main` before the PR (if CS6 merged first, re-measure on top of it). Commit wip states early. Conventional commits +
trailer; push; `gh pr create --base main --title "<what ships>" --body-file …`; `gh pr checks --watch` until green; never
merge; `REPORT.md` (Summary · Per-call felts and steps before / after · Folds tried (sizes) · Pile10 per layout · In-process
unchanged proof · Proposed ADR text · Escalations · PR URL). Memory rules apply.

## 6. Work autonomously, do not ask questions, do not widen the scope.
