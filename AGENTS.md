# AGENTS.md

Canonical working agreement for every agent (and human) contributing to `rapier-cairo`.
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

The orchestrator-side strategy (CLIs, model tiers, brief format, parallelism) is
[`docs/ORCHESTRATOR.md`](docs/ORCHESTRATOR.md); this section is the porter-side view of it.

| Role | Does | Does not |
|---|---|---|
| **Orchestrator** | Owns `docs/PLAN.md`, `docs/interfaces/**`, root `Scarb.toml`, `.tool-versions`, `scripts/**`, `.github/**`, every `lib.cairo` and crate `Scarb.toml`; pre-declares the stubs of a wave (modules, test files, snapshot files) before launching it; writes the briefs; reviews `REPORT.md` + CI; merges; updates re-exports/status/decisions after each merge | Large implementation work |
| **Executor** (headless CLI agent: `claude -p` or `codex exec`, own account, own worktree and branch `feat/<id>`) | Implements exactly one brief inside its file allowlist, with tests and gas probes; regenerates the gas snapshot of **its own modules**; opens its PR and drives it to green CI; writes `REPORT.md` (uncommitted) | Edit shared files (it lists needs under "Escalations" in the report), change frozen interfaces, merge, ask questions |
| **Reviewer** | Checks API parity with upstream, deviations, gas table, conventions | First implementation |

### Launching executors

`scripts/executor.sh <id> <runner> <brief.md>` with `runner` = `claude:sonnet|opus|fable` or
`codex:<model>[:<effort>]`; `scripts/executor.sh resume <id> <runner> "<follow-up>"` continues an
interrupted agent in the same worktree. The launcher prepends `scripts/executor/system-prompt.md`
(the frame every executor must obey) to the brief. Logs go to `.executor-logs/<id>.log`; the
orchestrator reads `REPORT.md` and the log, never the transcript.

Model choice by difficulty (`docs/ORCHESTRATOR.md`):

| difficulty | claude | codex |
|---|---|---|
| mechanical, well framed (generated code, spec alignment, benching already-identified variants) | `sonnet` | `gpt-5.5` / `gpt-5.6-*`, effort `medium` |
| standard port with numerics (a new module: kernels, tests, golden vectors, benches) | `opus` | `gpt-5.6-*`, effort `high` |
| genuinely complex (novel numerics, hard debugging, cross-module design) | `fable` | `gpt-6-astra`, effort `xhigh` |

The smaller the model, the tighter the brief. On the current codex account only `gpt-5.5` and
`gpt-6-astra` are accepted (`gpt-5.6-*` is refused): use `gpt-5.5` at `high` effort for the middle tier.

### Brief (mandatory sections, in this order)

1. Files to read first (`AGENTS.md`, `docs/PLAN.md`, `docs/interfaces/**`, style precedents on `main`).
2. Strict scope: file allowlist; everything else is forbidden (needs go to "Escalations").
3. Expected API (exact upstream names), numeric semantics, what is explicitly deferred (DEFER).
4. Efficiency rules and gas targets; variants to bench (winner in the library, losers under `mod alternatives`).
5. Tests: table-driven, compile budget (≤ 800 lines per file, ≤ 4 `fuzz_*` per module), golden vectors, exact panic messages.
6. Definition of done: full gate in the foreground, `scripts/gas.py snapshot --filter <crate>::<module>` per owned module, conventional commits with trailer, push, `gh pr create`, `gh pr checks --watch` until green, never merge, `REPORT.md` (Summary · API · Gas table · Deviations · Deferred · Requested re-exports · Escalations · PR URL).
7. "Work autonomously, do not ask questions, do not widen the scope."

### Escalation (executor → orchestrator, in `REPORT.md`)

```
Blocker: <what prevents completion>
Options: <A / B / C with cost>
Recommendation: <one of them, and why>
```

## 4. Parallelisation rules

- **Safe in parallel:** disjoint modules on top of frozen interfaces; each executor in its own
  worktree and branch; one gas snapshot file per module (`gas/<crate>/<module>.snap`, or
  `<module>.<child>.snap` for the parents listed in `scripts/gas.py` `SPLIT_MODULES`), so parallel
  PRs never touch a common file.
- **Orchestrator only, serialised:** public types of `rapier_math`, any `Scarb.toml`, every
  `lib.cairo` (stubs are pre-declared before the wave), `.tool-versions`, `scripts/**`,
  `.github/**`, `docs/PLAN.md`, `docs/interfaces/**`.
- **Conflict protocol:** land the interface change first with its tests, then rebase dependents.
- Waves follow the dependency graph; a wave starts when its dependencies are merged.

## 5. Feature pipeline and definition of done

`author → test → benchmark alternatives → pick winner → snapshot → review`

A feature is done when:

- [ ] `test_*` unit tests cover behaviour, edge cases (zero, negative, extremes, overflow policy)
      and every bug fixed gets a regression test;
- [ ] where upstream golden vectors exist, results match within the documented tolerance;
- [ ] `fuzz_*` tests (fixed seed) prove alternative implementations equivalent, when there are any;
- [ ] `gas_*` probes exist for every public function **and every candidate implementation**
      (`gas_mul_math`, `gas_mul_bitwise`, …), with inputs routed through
      `rapier_testing::opaque` and one `gas_baseline` per test module, and the module's
      `gas/<crate>/<module>.snap` is regenerated and committed with the code;
- [ ] rejected candidates live in a `#[cfg(test)] mod alternatives` next to the winner;
- [ ] public items carry `///` docs (arguments, returns, panics, rounding direction, value range);
- [ ] validation passes (§6).

## 6. Required validation

```
scarb fmt --check --workspace
scarb lint --workspace --deny-warnings
scarb build --workspace
snforge test --workspace
python3 scripts/gas.py snapshot --filter <crate>::<module>   # executors, per owned module
python3 scripts/gas.py check                                  # CI and orchestrator
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
- Sierra gas charges a loop-free function its **most expensive path**, not the path taken (GF2:
  482k on every regime; GG: an outlined dispatcher costs 556k for every pair). Only Cairo steps
  follow the executed path, so early-exit variants are compared with `--tracked-resource
  cairo-steps` too. Practical rules (measured in DB, GG, P1): keep a dispatching `match`
  `#[inline(always)]` in its caller; a computing arm that must stay inside an outlined function
  goes behind a one-iteration `while pending { …; pending = false; }` ("metered" call, ~265 steps),
  because a `while` body is only charged when it runs (`loop { …; break; }` is not); an
  `#[inline(never)]` helper per arm is the cheap fix only when the `match` itself is inlined. A third tool (JM, #109):
  calling a `#[inline(never)]` function that contains a (zero-trip) loop — a "gas wallet", see
  `rapier_dynamics2d::solver::joint::row::gas_wallet` — makes the caller's unused branch gas refundable;
  measure it (it saved 115k of 359k on joint row generation, but cost +20k in the solve loops).
- Traits: `FooTrait` / `FooImpl` (via `#[generate_trait]` when there is a single impl); operators
  through core traits; `Zero`, `One`, `Default` where meaningful.
- Errors: `pub mod errors { pub const X: felt252 = 'Type: reason'; }` with `assert(cond, errors::X)`;
  `Option` for expected absence. No `ByteArray` panics in library code.
- `lib.cairo` contains only module declarations and re-exports.
- Core crates stay pure Cairo: no `starknet` dependency.
- Tests are inline (`#[cfg(test)] mod tests`) at the bottom of each file; `crates/<c>/tests/`
  holds only cross-module scenarios and golden-vector comparisons.
- Compile budget: no file over 800 lines, at most 4 `fuzz_*` tests per module, table-driven tests
  rather than one function per case — test-crate compile time is the first cause of CI failures.

## 8. Rationalisations to reject

"It is obviously cheaper" (measure it) · "tests later" (no) · "just this once I will edit the
snapshot by hand" (never) · "the interface needs a tiny change" (escalate) · "I will run the gate in
the background and finish" (the headless session stops; run it in the foreground).
