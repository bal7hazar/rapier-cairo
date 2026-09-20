# AGENTS.md

Canonical working agreement for every agent (and human) contributing to `rapier.cairo`.
`CLAUDE.md` only adds context; when the two disagree, this file wins.

## 1. Mission

Port the [Rapier](https://github.com/dimforge/rapier) physics engine (and the part of
[Parry](https://github.com/dimforge/parry) it needs) to Cairo, so that a whole game can run on
provable physics. Gas is a first-class requirement, on par with correctness: a feature that is
correct but unmeasured is not done.

Scope, phases and work packages live in [`docs/PLAN.md`](docs/PLAN.md). The research that
justifies them lives in [`docs/research/`](docs/research).

## 2. Operating principles

1. **Measure, never guess.** Expected cost order in Cairo is
   `felt / integer arithmetic  <  DivRem  <  bitwise builtin  <  loop`.
   When the cheapest form is not obvious, implement the candidates, benchmark them with `gas_*`
   tests and keep the winner. The ranking can flip between compiler versions, so losers stay
   in the tree (see §5).
2. **Small, testable diffs.** One module or one optimisation class per PR.
3. **Frozen interfaces first.** Public types and trait signatures are agreed (and merged) before
   work that depends on them is parallelised.
4. **Determinism is part of the spec.** Iteration order, rounding direction and iteration bounds
   are explicit and documented; nothing may depend on dict iteration order.
5. **Upstream is the reference.** Port the maths of Rapier/Parry faithfully; diverge only where
   the research reports say so (fixed point, no SIMD/parallel, closed shape enum, …) and record
   new divergences in `docs/adr/`.

## 3. Roles

| Role | Does | Does not |
|---|---|---|
| **Orchestrator** | Owns `docs/PLAN.md`, writes task briefs with acceptance checks, freezes interfaces, owns `Scarb.toml`, `.tool-versions`, `scripts/**`, `.github/**`, regenerates `.gas-snapshot` after merging executor work, reviews and merges PRs | Large implementation work |
| **Executor** (sub-agent) | Implements exactly one work package inside the files it was given, with tests and gas probes, in its own git worktree/branch | Touch files outside its brief, edit shared config, change frozen interfaces |
| **Reviewer** | Checks correctness against upstream, gas deltas, conventions | First implementation |

### Launching executors

Executors run through the terminal `claude` CLI (logged in with a separate account) rather than
inside the orchestrator's session: `scripts/executor.sh <id> <model> <brief.md>` creates the
worktree and branch `feat/<id>` from `origin/main`, frames the run with
`scripts/executor/system-prompt.md`, restricts tools to file edits plus the toolchain, git and
python, and writes the transcript and final report under `.executor-logs/`. Briefs live in
`docs/briefs/<id>.md`.

Model policy — the cheapest model that can do the job, with the frame above to prevent drift:

| Task | Model |
|---|---|
| Mechanical, fully specified (generated fixtures, wiring, docs, straightforward ports validated by golden vectors) | `sonnet` |
| Algorithmic or design-heavy (solver, SAT/clipping, numeric hazards, candidate design) | `opus` |
| Orchestration, review, interface freezes, merges | the orchestrator itself |

### Task brief template (orchestrator → executor)

```
Work package: <id and title from docs/PLAN.md>
Goal: <one paragraph>
Files owned: <paths the executor may create/modify>
Frozen interfaces: <types/traits it must use as-is>
Upstream reference: <rapier/parry file paths + functions>
Acceptance: <tests that must exist and pass, gas budget if any, golden vectors to match>
Out of scope: <explicit list>
```

### Escalation template (executor → orchestrator)

```
Blocker: <what prevents completion>
Options: <A / B / C with cost>
Recommendation: <one of them, and why>
```

## 4. Parallelisation rules

- **Safe in parallel:** disjoint modules/crates on top of frozen interfaces; each executor in an
  isolated worktree and branch.
- **Must be serialised (orchestrator only):** changes to public types of `rapier_math`, any
  `Scarb.toml`, `.tool-versions`, `scripts/**`, `.github/**`, `.gas-snapshot` regeneration,
  `docs/PLAN.md`.
- **Conflict protocol:** land the interface change first with its tests, then rebase dependents.
- Executors never commit `.gas-snapshot`; the orchestrator regenerates it once per merge so that
  parallel branches do not conflict on it.

## 5. Feature pipeline and definition of done

`author → test → benchmark alternatives → pick winner → snapshot → review`

A feature is done when:

- [ ] `test_*` unit tests cover behaviour, edge cases (zero, negative, extremes, overflow policy)
      and every bug fixed gets a regression test;
- [ ] where upstream golden vectors exist, results match within the documented tolerance;
- [ ] `fuzz_*` tests (fixed seed) prove alternative implementations equivalent, when there are any;
- [ ] `gas_*` probes exist for every public function **and every candidate implementation**
      (`gas_mul_math`, `gas_mul_bitwise`, …), with inputs routed through
      `rapier_testing::opaque` and one `gas_baseline` per test module;
- [ ] rejected candidates live in a `#[cfg(test)] mod alternatives` next to the winner;
- [ ] public items carry `///` docs (arguments, returns, panics, rounding direction, value range);
- [ ] validation passes (§6).

## 6. Required validation

```
scarb fmt --check --workspace
scarb lint --workspace --deny-warnings
scarb build --workspace
snforge test --workspace
python3 scripts/gas.py check      # orchestrator; executors run `diff` and report the table
```

## 7. Coding conventions

- Shifts and masks are `*` and `DivRem::div_rem` by `NonZero` **constants**; never `/` and `%` on
  the same operands; never `pow()` at runtime (use a `const` table or `match`).
- Smallest integer type that fits; no `u256` in hot paths; widening multiplication for
  fixed-point products.
- Fixed-size maths is fully unrolled over struct fields; no `Array`/`Span` inside vectors or
  matrices. Unavoidable loops use exact-trip conditions (`while i != n`), a cached length, or
  `pop_front` iteration.
- Value types derive `Copy, Drop, Serde, PartialEq, Debug` and are passed by value.
- `#[inline(always)]` on leaf arithmetic only; avoid `unwrap()` and panics inside inlined hot code.
- Traits: `FooTrait` / `FooImpl` (via `#[generate_trait]` when there is a single impl); operators
  through core traits; `Zero`, `One`, `Default` where meaningful.
- Errors: `pub mod errors { pub const X: felt252 = 'Type: reason'; }` with `assert(cond, errors::X)`;
  `Option` for expected absence. No `ByteArray` panics in library code.
- `lib.cairo` contains only module declarations and re-exports.
- Core crates stay pure Cairo: no `starknet` dependency.
- Tests are inline (`#[cfg(test)] mod tests`) at the bottom of each file; `crates/<c>/tests/`
  holds only cross-module scenarios and golden-vector comparisons.

## 8. Rationalisations to reject

"It is obviously cheaper" (measure it) · "tests later" (no) · "just this once I will edit the
snapshot by hand" (never) · "the interface needs a tiny change" (escalate).
