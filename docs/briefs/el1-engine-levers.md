# EL1 — the engine's step levers (IT1 step 2, the in-scope levers)

Runner: `impl-opus` (step path), on the Mac (whole-shot suites; no heavy lock there). Lot id `el1-engine-levers`.

## 1. Goal, context, files to read first

**Goal.** Lower the Cairo steps of every awake contact tick with the levers IT1 measured, with results
**bit-identical**. The value does not depend on saving a proof: the lot lowers gas per shot and shortens the client's
post-impact slow motion (the owner's path (d)). Approved by the project manager on 2026-10-02 (IT1 step 2, after CX3).

**Context.** `docs/research/impact-tick.md` §4 ranks the levers and estimates them; CX3 (lever X1) is merged (#253),
and since CI2 (#256) a class-hash pin that differs from CI's build fails CI ("Pinned class hashes match CI's build"):
after the levers push, take the new pins from that run's `class-hashes` artefact and push them. The owner's
physics-rate study is done: the rate stays at 60 Hz. The levers of this lot, with IT1's estimates (owner's shot, in process; estimates, not targets):
- **N1** narrow-phase pair-loop glue (in process only: in the slim layout `NarrowPhaseClass` runs its own loop since
  CX3): −0.45 to −0.95M;
- **F1** force events without whole-collider reads: −0.35 to −0.55M;
- **S1** solver sweep data movement (`Hot` / `Bank` pop and re-append): −0.3 to −0.4M;
- **W1** fewer walks of `narrow_phase.pairs` in the step glue: −0.25 to −0.45M;
- **U1** the dense step's user-change scan on all-awake ticks: −0.2 to −0.3M;
- **R1** `remove_body` in one walk: −0.03 to −0.06M;
- **G1** constraint generation: −0.1M.

Read first: `AGENTS.md`, `CLAUDE.md`, `docs/research/impact-tick.md` (all of it), `docs/briefs/it1-impact-tick.md`,
`docs/briefs/cx3-slim-crossings.md` and CX3's section of `docs/BUDGETS.md`, `docs/adr/0001-upstream-divergences.md`,
the BT1–BT4 sections of `docs/BUDGETS.md` (variants already benched and their losers under `mod alternatives`).

The IT1 probes (`it1.cairo`, `stages.py`, the census) are uncommitted in the Mac worktree
`/Users/bal7hazar/.herdr/worktrees/rapier-cairo/hp-slingfall-rapier-t-0008-it1-impact-tick-measure`, and CX3's in
`/Users/bal7hazar/.herdr/worktrees/rapier-cairo/hp-slingfall-rapier-t-0017-cx3-slim-crossings-x1`: read and copy them
with the Read tool, never edit those worktrees. If reading them is refused, rewrite the probes you need from
`docs/research/impact-tick.md` §6–§7 (it names them) and say so.

## 2. Scope: file allowlist

- The engine sources the levers name: `crates/rapier_dynamics2d/src/**` (narrow phase, `ColliderSet` accessor, solver
  sweeps, generation), `crates/rapier2d/src/**` (pipeline, force events, sleeping, active set, user changes, world).
  Their inline tests may be added, never edited.
- `gas/**/*.snap` of the crates touched (by `scripts/gas.py` only); `gas/bytecode.size` from CI's `bytecode-snapshot`
  artefact only; the changed constants of `crates/rapier2d_classes/tests/hashes.cairo` from CI's `class-hashes`
  artefact only, CI's root path and run id beside them; `docs/PACKAGES.md` from CI's `consumer-cost` artefact with
  `scripts/packages_table.py`.
- `docs/BUDGETS.md` (one new dated section), `docs/research/impact-tick.md` (a new section with the measured result
  of each lever), `docs/adr/0001-upstream-divergences.md` (a new entry only, if any), this brief
  (`docs/briefs/el1-engine-levers.md`, your first commit, as given).
- Not in scope: `crates/rapier2d_classes/src/**`, `crates/rapier_sink/**`, `crates/rapier_geometry2d/**` (unless a lever
  needs it: escalate), `WorldState` / its codec, public signatures, `lib.cairo` re-exports, crate `Scarb.toml`, CI.
- Parked items stay parked (`docs/PLAN.md`: (c), (A), (B), (E), (F), (e), (f), dormant pairs out of
  `narrow_phase.pairs`, sleep thresholds).

## 3. Conditions (the project manager's)

- **Bit-identical results** on every reference shot and probe: goldens, every `*_bit_identical` test, digests, the
  `pile10` owner's and reference shots in process and slim, `game_path`, `game_ticks`, `edits`, `removals`, `split`.
  **A numeric change stops the lot**: push what you have, stop, report under Escalations; the orchestrator takes it to
  the project manager. Never edit a test's expected value (hash constants are not results).
- **Declared classes**: every class under 73,728 Sierra and CASM felts, the margins printed (CI's `bytecode` log), the
  slim caller first; if the slim caller's margin falls under 2,000 felts, say so at the top of the report and the PR.
- **Hashes from CI only**, with CI's root path (the build-path rule).
- **Steps and proofs**, before (`origin/main` after CX3) and after, exact, per shot (owner's and reference), in process
  and slim, plus the impact tick and an average post-impact tick, and the proof count on the game's basis.

## 4. How

One lever per commit, measured each (exact steps before / after on the probes it moves); a lever that does not pay,
or that cannot be kept bit-identical, is dropped and reported with its figure. Losers of a benched choice stay under
`mod alternatives` (`AGENTS.md` §2). `RAYON_NUM_THREADS=1`; whole-shot suites with `--max-threads 2`, one at a time;
never `snforge test --workspace`. Run `scripts/prepush.sh` before each push if it is on `main`, else `scarb fmt --check`.

**CI is starved**: push rarely. Push once when the levers are done (open the PR then), once more with the pins from
that run's artefacts. Each review's fixes go in one push.

## 5. Acceptance criteria

1. The before / after tables of §3, every figure with its toolchain, machine and build path.
2. Every result test passes unchanged (`git diff --stat origin/main -- 'crates/**/tests/**'` shows only hash constants).
3. The declared-class table with margins from CI's `bytecode` log; every class under 73,728.
4. The pins from CI's artefacts; every CI check green.
5. `docs/BUDGETS.md`: a section "EL1 — engine levers (<date>)"; `docs/research/impact-tick.md`: each lever's measured
   result against its estimate.
6. A pull request; **never merge**. Prefer words over links to external issues in PR text and commit messages.

## 6. Report expected

Your thread report as your rules say. Lead with the slim caller's margin, then the steps and proofs per shot before /
after, each lever's measured gain (or why it was dropped), the bit-identity verdict, and what a game must re-pin.

## 7. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.
