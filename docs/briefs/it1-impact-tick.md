# IT1 — the impact tick: where its Cairo steps go, and the cut

Runner: `impl-opus` (step path, numerics), on the Mac (whole-shot suites). Lot id `it1-impact-tick`.

## 1. Goal, context, files to read first

**Goal.** Lower the Cairo steps of the **impact tick** of the game's reference shots, with results **bit-identical**.
A gain there cuts the proofs per shot (game cost sheet: 3 proofs for the reference shot, 6 for the owner's shot).
Nothing waits on it; it is a game need relayed by the project manager on 2026-10-02, which reopens the parked item
"(d) further step levers" of `docs/PLAN.md` for this lot only. The other parked items stay parked.

Two steps:
1. **Measure** where the impact tick's steps go, on the game's reference shots: a per-stage, per-function profile.
2. **Cut**: implement the levers that are clear and keep results bit-identical; propose the others with an estimate.
   Step 2 starts only when the orchestrator says so (it waits for TC1, the toolchain bump, to merge: the step figures
   and the gas snapshots move with the compiler).

**History of this tick** (`docs/BUDGETS.md`, `docs/PLAN.md` row BT): G0 → BT1 → BT3 → BT4 took the L10 impact tick
from 811,866 to ≈ 370k steps. BT3 / BT4 left as levers: narrow phase ≈ 101k and island solve ≈ 194k per impact tick,
≈ 17 steps per fixed-point rescale as the arithmetic's floor, six `remove_body` ≈ 90k on pile10, L20's mixed-tick
excess ≈ 45k (dormant pairs out of `narrow_phase.pairs`, `WorldState` v3: **a codec change, out of scope**).

Read first: `AGENTS.md` (§3 the report, §6 validation, §7 the steps rule: before / after tables on the P3, level **and**
game-shaped probes), `CLAUDE.md`, `docs/BUDGETS.md` (the BT sections), `docs/adr/0001-upstream-divergences.md`,
`crates/rapier2d/tests/game_path.cairo`, `crates/rapier2d/tests/level_budget.cairo`,
`crates/rapier2d_classes/tests/{steps,pile10,game_ticks,slim}.cairo`, `scripts/gas.py`.

**Reference shots.** The game's reference shot is pile10 (`docs/PLAN.md` § Phase 2: impact tick ≈ 1.02M steps on the
game path, six `remove_body` ≈ 90k). The owner's shot: find it in the game's cost sheet (repository
`bal7hazar/slingfall`, `docs/PLAN.md` / `docs/proving.md` / `docs/levels.md`, read-only through `gh api` or a clone
outside your worktree); if it is not reachable or has no rapier-side probe, say so and use the L10 / L20 level probes
and `game_path`. Name in the report which probe stands for which shot.

## 2. Scope: file allowlist

**Step 1** (measurement): `docs/research/impact-tick.md` (new) and this brief, `docs/briefs/it1-impact-tick.md` (your
first commit, as given). Nothing else is committed: probes, traces and scripts you write to measure stay uncommitted,
or are listed in the report.

**Step 2** (cut), on the orchestrator's word, on a new branch from `origin/main` after TC1: the Cairo sources of the
step path that the step-1 report names (`crates/rapier_dynamics2d/src/**`, `crates/rapier_geometry2d/src/**`,
`crates/rapier2d/src/**`), their inline tests, `gas/**/*.snap` of the crates touched (by `scripts/gas.py` only),
`docs/BUDGETS.md` (one new dated section), `docs/adr/0001-upstream-divergences.md` (a new divergence only), and
`docs/research/impact-tick.md`. Not in scope: `WorldState` / codecs, public signatures, `lib.cairo` re-exports, crate
`Scarb.toml`, CI, `gas/bytecode.size` (Linux only, see §5), `crates/rapier2d_classes/**` and `crates/rapier_sink/**`
sources. Needs go to Escalations.

## 3. Interfaces

No public item changes. Results are API: goldens, `WorldState` codecs, step results, events, digests. **A numeric
change stops the lot**: any failing result test (goldens, `*_bit_identical`, digests, any expected value) means push
what you have, stop, and report under Escalations with the failing tests and values; the orchestrator takes it to
the project manager. Do not adjust a test. Parked levers (`docs/PLAN.md`: (c) solver-graph order, (A) the solver's
scalar API, (B) composites, (E) sub-shape widening, (F) `contact_skin`, (e) `core_witness` precision, (f) the sweep
normal) are not touched, even if the profile points at them: name them with their estimate.

## 4. Efficiency rules and targets

- Step 1: the profile of the impact tick of each reference shot, per pipeline stage (user changes, broad phase,
  narrow phase: generation / bookkeeping, islands, solver: velocity / position / per substep, CCD, events,
  `remove_body`, codec crossings in the class layout) and the top functions, in exact Cairo steps
  (`--tracked-resource cairo-steps`; snforge's trace data and `cairo-profiler` if useful). The flight tick next to
  it for scale. Then the levers, ranked by steps saved per tick × ticks per shot, each with: the mechanism, the
  estimate (called one), whether it is bit-identical by construction, the files it touches, its risk.
- Step 2: implement the bit-identical levers that are clear, one commit each, measured each. Losers of a benched
  choice stay under `mod alternatives` (`AGENTS.md` §2).

## 5. Machines and measurement rules

- Every build and measurement with `RAYON_NUM_THREADS=1` exported; whole-shot suites (`rapier2d_classes`,
  `rapier_sink`) with `--max-threads 2`, one such suite of this lot at a time; never `snforge test --workspace`.
- Steps and gas are measured on the Mac. **Hash, class-bytes, class-size or margin figures are never produced from a
  Mac build** (programme rule, 2026-10-02): if the cut moves class sizes, take them from the PR's CI `bytecode` log
  and list the VPS regeneration of `gas/bytecode.size` under Escalations. Every hash or size you report names its
  machine and its absolute build path.
- The toolchain is the one `.tool-versions` pins on your branch (step 1: `main` before TC1 merges is 2.19.4 / 0.61.0;
  say which in every table).

## 6. Acceptance criteria

Step 1:
1. `docs/research/impact-tick.md`: the profile tables (§4) of each reference shot's impact tick, the command that
   produced each, the toolchain and machine, and the ranked levers.
2. A pull request with the brief and the research document, checks green. **Never merge.**

Step 2:
3. Before / after exact-steps tables on the P3 probes (`steps_step_*`), the level probes, the game-shaped probes
   (`game_path`, `steps_game_*`) and the reference shots' impact tick and whole shot (`rapier2d_classes` `steps_`,
   `pile10`), in the report and the PR description.
4. Every result test passes unchanged (`git diff --stat origin/main -- 'crates/**/tests/**'` shows no edit of an
   expected value); bit-identity on the reference shots shown by the existing `*_bit_identical` tests.
5. `gas/**/*.snap` regenerated by `scripts/gas.py`; every CI check green except `bytecode` if class sizes moved (§5).
6. `docs/BUDGETS.md`: one section "IT1 — impact tick (<date>)" with the summary table.

## 7. Verification

Step 1: crate-scoped runs of the existing probes (`snforge test -p rapier2d steps_ --tracked-resource cairo-steps`,
`snforge test -p rapier2d_classes steps_ --include-ignored --tracked-resource cairo-steps --max-threads 2`, the
`game_path` and `pile10` tests), plus any uncommitted probe you need. Per crate `scarb lint -p <crate>
--deny-warnings` for anything committed. Step 2: the same, before and after each lever, plus `snforge test -p <crate>`
crate by crate for every crate touched.

User-local tools: you may `asdf plugin add` / `asdf install` a profiling tool (user-local); a system package is the
owner's: report the need.

## 8. Report expected

Your thread report as your rules say, title `# [<model you run as>] IT1 — impact tick (step <n>)`. Lead with the
impact tick's steps per reference shot and where they go (step 1), or the before / after table (step 2); then the
levers, the bit-identity verdict, the machine and toolchain of every figure.

## 9. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.
