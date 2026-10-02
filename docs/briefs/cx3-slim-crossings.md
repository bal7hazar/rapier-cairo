# CX3 — the slim layout's class crossings (IT1 step 2, lever X1)

Runner: `impl-opus` (step path, declared classes), on the Mac (whole-shot suites). Lot id `cx3-slim-crossings`.

## 1. Goal, context, files to read first

**Goal.** Cut the Cairo steps the slim layout pays for crossing between declared classes, with results
**bit-identical**, so that the game's owner's shot may lose a proof.

**Context.** IT1 step 1 (`docs/research/impact-tick.md`, #249) measured, on Scarb 2.19.4: the slim shot pays
+8,344,423 steps (owner's shot, +37.3 %) and +3,803,112 (reference shot, +44.0 %) over the in-process run; crossings
cost 74–84k per collapse tick, 82.7k at the impact, 12.6k per flight tick; `NarrowPhaseClass` generation +19–23k per
collapse tick. Lever X1, estimated −2 to −4M on the owner's shot (−0.6 to −1.2M on the reference shot): a CX3-like
lot — an awake-only narrow-phase wire and the advance write-back. One proof on the owner's shot needs ≈ −2.89M (game
basis 154 gas per step, cost sheet on alpha.8). Approved by the project manager on 2026-10-02 (IT1 step 2, option 3:
this lot first, then the in-scope engine levers as a lot of their own).

Read first: `AGENTS.md`, `CLAUDE.md`, `docs/research/impact-tick.md` (§1–§4, X1), `docs/research/class-split.md`,
the briefs and research of CX1 / CX2 (`docs/briefs/cx1-crossing-cost.md`, `docs/briefs/cx2-contact-crossings.md`),
`docs/adr/0001-upstream-divergences.md`, `crates/rapier2d_classes/src/**`, `crates/rapier2d_classes/tests/{steps,slim,
pile10,game_ticks,hashes,split,edits,removals}.cairo`, `scripts/bytecode_size.py`, `.github/workflows/ci.yml` (the
`bytecode-snapshot` and `class-hashes` artefacts added by TC1).

## 2. Scope: file allowlist

- `crates/rapier2d_classes/src/**` (the stages, their wires and codecs, the slim layout, the caller).
- `crates/rapier2d_classes/tests/hashes.cairo`: the pinned class-hash constants and their comments only, from CI's
  `class-hashes` artefact (§5).
- `gas/**/*.snap` of the crates touched (by `scripts/gas.py` only); `gas/bytecode.size` from CI's `bytecode-snapshot`
  artefact only (§5); `docs/PACKAGES.md` from CI's `consumer-cost` artefact with `scripts/packages_table.py`.
- `docs/BUDGETS.md` (one new dated section), `docs/research/class-split.md` or `docs/research/impact-tick.md` (a new
  section), `docs/adr/0001-upstream-divergences.md` (a new entry only, if any), this brief
  (`docs/briefs/cx3-slim-crossings.md`, your first commit, as given).
- Not in scope: `crates/rapier2d/**`, `crates/rapier_dynamics2d/**`, `crates/rapier_geometry2d/**` sources (the engine
  levers are the next lot), `WorldState` / its codec, public signatures of the engine, CI. Needs go to Escalations.

## 3. Interfaces and conditions (the project manager's, 2026-10-02)

- **Bit-identical results** on every reference shot and probe: goldens, every `*_bit_identical` test, digests, the slim
  and in-process `pile10` runs of the owner's and the reference shots, `game_ticks`, `edits`, `removals`, `split`.
  **A numeric change stops the lot**: push what you have, stop, and report it under Escalations with the failing tests
  and values; the orchestrator takes it to the project manager. Never adjust a test's expected value (the hash
  constants of §5 are not results).
- **Every declared class stays under 73,728** Sierra and CASM felts, the margins printed (`bytecode_size.py`), the CI
  gate (≥ 1,000) green. **If the slim caller's margin falls under 2,000 felts, say so at the top of the report and in
  the PR description**: the orchestrator tells the project manager before any merge.
- The class layout's public surface (the class names a game declares, `ClassHashes`, the `README` list of classes to
  declare) changes only if the lever needs it; say so and list what a game must re-pin.
- Losers of a benched choice stay under `mod alternatives` (`AGENTS.md` §2).

## 4. Targets

- Steps: the slim layout's crossings per tick (impact, collapse, flight) and the whole slim shot, owner's and reference.
- Report, before (`origin/main` after TC1, Scarb 2.20.1) and after, **exact** Cairo steps and the **proofs per shot**
  on the game's basis (owner's and reference), in the report and the PR description, with the in-process figures
  next to them for scale.

## 5. Machines, pins and measurement rules

- `RAYON_NUM_THREADS=1` for every build and measurement; whole-shot suites (`rapier2d_classes`, `rapier_sink`) with
  `--max-threads 2`, one at a time; never `snforge test --workspace`.
- Steps, gas and felt counts are path-free: measure them on the Mac, naming machine and absolute build path.
- **Class hashes and `gas/bytecode.size` are path-bound: they come from CI only.** Push, then take them from the PR's
  run artefacts (`gh run download`): `bytecode-snapshot` → commit `gas/bytecode.size` as generated; `class-hashes` →
  set the changed constants of `hashes.cairo`, each block with a comment naming CI's root path and the run id. The
  uncommitted local hashes you may need to run the `steps_slim_*` probes on the Mac stay uncommitted.
- Run `scripts/prepush.sh` before each push if it exists on `main` (lot PP1), and `scarb fmt --check` in any case.

## 6. Acceptance criteria

1. Before / after tables (§4), exact steps and proofs per shot, every figure with its toolchain and machine.
2. Every result test passes unchanged (`git diff --stat origin/main -- 'crates/**/tests/**'` shows only the hash
   constants of `hashes.cairo`).
3. The declared-class table with margins (from CI's `bytecode` log), the slim caller first; every class under 73,728.
4. `gas/bytecode.size`, the hash constants and `docs/PACKAGES.md` from CI's artefacts; every CI check green.
5. `docs/BUDGETS.md`: a section "CX3 — slim crossings (<date>)".
6. A pull request; **never merge**.

## 7. Report expected

Your thread report as your rules say. Lead with the slim caller's margin (and the 2,000 warning if it applies), then
the exact steps and proofs per shot before / after, the bit-identity verdict, and what a game must re-pin.

## 8. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.
