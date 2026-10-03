# WS3 — dormant pairs out of `narrow_phase.pairs`, and the stepped-layout items

Runner: `impl-opus` (step path, codec), on the Mac (whole-shot suites). Lot id `ws3-dormant-pairs`.

## 1. Goal and context

**Goal.** Lower the Cairo steps of every tick that has sleeping contact pairs, by keeping dormant pairs out of the
pair list that the step walks, with results **bit-identical**. The `WorldState` codec changes, so it gets a new
version with a migration.

**Context.**
- **Why it was parked:** BT4 (`docs/PLAN.md` row BT) left L20's mixed-tick excess of about 45k steps per tick. Its
  lever was dormant pairs out of `narrow_phase.pairs`, a `WorldState` change. It was parked because the codec faced
  the game.
- **Why now:** the game is on hold and adopts library changes only at its next re-pin. The project manager approved
  WS3 on 2026-10-03, after EL1 (merged, #259).
- **The figures to beat:** IT1 and EL1 (`docs/research/impact-tick.md` §1–§9) measured the walks of
  `narrow_phase.pairs`: about 12 walks per impact tick, and the sleeping pairs walked and copied on every tick.
- **Also in this lot:** two parity items that need a new field on a stepped or serialised struct:
  - `ColliderSet::take_removed` (`crates/rapier_dynamics2d/src/collider_set.cairo`);
  - `GenericJointBuilder::user_data`.

Read first:
- `AGENTS.md`, `CLAUDE.md`;
- `docs/research/impact-tick.md`;
- `docs/BUDGETS.md` (the BT, EL1 and CX3 sections);
- `docs/adr/0001-upstream-divergences.md` (the sleeping and active-set entries, BT2, D7 / D9);
- the `WorldState` codec and its version policy (find them under `crates/rapier2d/src/world*`; state the current
  version: v3 since CC2, if so);
- `CHANGELOG.md`;
- the pipeline files EL1 changed (`pipeline/{sleeping,active_set,ordering,fused}.cairo`, `world.cairo`);
- `crates/rapier2d_classes/src/**` (the class layout crosses `WorldState` and the pair lists).

## 2. Scope: file allowlist

- **Engine sources:** `crates/rapier_dynamics2d/src/**`, `crates/rapier2d/src/**`, their inline tests (added, never
  edited).
- **Classes:** `crates/rapier2d_classes/src/**`, where the class layout carries the pair list or the codec.
- **Golden and probe files:** `crates/rapier2d/tests/world_state.cairo` and any golden or codec test file, **only to
  add** the new version's vectors and the migration test. An existing expected value is never edited.
- **Generated files:**
  - `gas/**/*.snap` of the crates touched, by `scripts/gas.py` only;
  - `gas/bytecode.size`, the changed hash constants of `crates/rapier2d_classes/tests/hashes.cairo` (with CI's root
    path and run id beside them), and `docs/PACKAGES.md`, all from CI's artefacts only;
  - `docs/API_PARITY.md`, by its script only.
- **Docs:**
  - `docs/BUDGETS.md` and `docs/research/impact-tick.md`: one new section each;
  - `docs/adr/0001-upstream-divergences.md`: new entries only;
  - `CHANGELOG.md`, `## Unreleased`: **required**. Say the codec version, the migration, the steps, and which classes
    a game re-pins.
- **This brief:** `docs/briefs/ws3-dormant-pairs.md`, your first commit, as given.
- **Not in scope:** public signatures of existing items (new items are fine), CI, scripts, the solver's arithmetic.

## 3. Conditions

- **Bit-identical results** on every reference shot and probe: goldens, every `*_bit_identical` test, digests, the
  `pile10` owner's and reference shots in process and slim, `game_path`, `game_ticks`, `edits`, `removals`, `split`,
  the level probes. **A numeric change stops the lot:** push what you have, stop, and report it under Escalations.
- **Codec:**
  - a new `WorldState` version (the next number);
  - the previous version still decodes, through a migration whose result is bit-identical to a world built and
    stepped the same way;
  - tests prove decode, encode and migration, and that a migrated world steps bit-identically to a fresh one.

  Follow the version policy of the codec and of `CHANGELOG.md`, and say in the report which bump it is.
- **Declared classes:** every one under 73,728 felts, with margins printed (CI's `bytecode` log). If the slim caller's
  margin falls under 2,000, say so at the top of the report and in the PR.
- **Class hashes:** from CI only. The CI2 gate fails until the pins match.
- **Steps:** before (`origin/main`, alpha.9 / `51dd962` or later) and after, exact, on the P3, level (L10 / L20:
  load, flight, impact, and the mixed ticks), game-shaped and pile10 probes, in process and slim. Also the proof count
  on the game's basis (154 L2 gas per step), as an estimate.
- **The two parity items:** port them faithfully, or escalate one with the reason it cannot be added without moving
  results.

## 4. How

- One lever or item per commit, each measured.
- A variant that loses stays under `mod alternatives`.
- `RAYON_NUM_THREADS=1`; whole-shot suites with `--max-threads 2`, one at a time; never `snforge test --workspace`.
- `scripts/prepush.sh` before each push.
- Push rarely: once when done (open the PR), once with the pins from that run's artefacts, then one push per review's
  fixes.

## 5. Programme rules

- **CI logs:** wait for the run to complete, then `gh run view <run-id> --log-failed` alone in its call. Never
  `gh api …/logs`. If you need a log earlier, the orchestrator relays it.
- **Git:**
  - your own git commands run normally, as separate commands;
  - to pick up `main` after a push, `git merge origin/main` alone in its call; never another rebase, never a force
    push;
  - a git command on another repository runs only in a sanitised environment (`env -u GIT_DIR -u GIT_WORK_TREE -u
    GIT_INDEX_FILE -u GIT_COMMON_DIR -u GIT_PREFIX`, or `env -i`).
- **Processes:** signal only processes you started, by the pid you recorded.
- **Sibling checkouts:** read files of another worktree with the Read tool, and run no git command on it.
- **Text:** prefer words over links to external issues in PR text and commit messages.
- **Refusals:** if a command is refused by the permission system, put its exact text and the time in your report and
  stop that part.

## 6. Acceptance criteria

1. The before / after tables of §3, every figure with its toolchain, machine and build path.
2. Every result test passes unchanged (`git diff --stat origin/main -- 'crates/**/tests/**'` shows only additions and
   the hash constants).
3. The codec tests of §3 pass.
4. The declared-class table with margins (CI's `bytecode` log).
5. The pins from CI's artefacts; every CI check green, `ci-ok` included.
6. The CHANGELOG entry, the ADR entries and the BUDGETS section.
7. A pull request; **never merge**.

## 7. Report

Your thread report as your rules say. Lead with the slim caller's margin, then:
- the steps and proofs per shot, before and after;
- the codec version and the migration;
- the bit-identity verdict;
- the two parity items;
- what a game must re-pin.

## 8. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.
