# AS — bring `rapier_core::data::{arena, union_find}` under the 800-line file budget

## 1. Read first
`AGENTS.md` (§7 compile budget: no source or test file over 800 lines); on `main`:
`crates/rapier_core/src/data/arena.cairo` (1 409 lines: `ArenaTrait`, the dict / split / array arenas,
`mod alternatives` in `data/arena/`, an inline `mod tests` from line ≈ 408), `crates/rapier_core/src/data/union_find.cairo`
(822 lines: inline `mod alternatives` from ≈ 125, inline `mod tests` from ≈ 340), and how other modules of
the workspace move their tests into sibling files (e.g. `crates/rapier2d/src/pipeline/tests.cairo`,
`crates/rapier_geometry2d/src/dispatch/tests.cairo`).

## 2. Scope (file allowlist)
`crates/rapier_core/src/data/arena.cairo`, `crates/rapier_core/src/data/arena/*.cairo`,
`crates/rapier_core/src/data/union_find.cairo`, `crates/rapier_core/src/data/union_find/*.cairo` (new),
and `gas/rapier_core/data.snap` (regenerate with `--filter rapier_core::data` if test paths change).
Nothing else: no API change, no behaviour change, no new tests.

## 3. Expected result
A pure move: the inline test modules (and `union_find`'s inline `alternatives`) become sibling files
declared with `#[cfg(test)] mod tests;` / `mod alternatives;`, every file ≤ 800 lines, every test and probe
keeps its name suffix so the gas snapshot entries only change by module path if at all. Report the
snapshot diff: entries must keep their values (a renamed path with an identical value is fine).

## 4. Efficiency
Nothing to optimise; no gas value may change.

## 5. Tests
All existing tests pass unchanged.

## 6. Definition of done
Foreground gate (tool timeout 3600000 ms, never background); `python3 scripts/gas.py snapshot --filter
rapier_core::data`; `python3 scripts/gas.py check`; conventional commit + trailer; push; `gh pr create` per
template; wait until the PR's checks are registered, then `gh pr checks --watch` until green; never merge;
`REPORT.md` (Summary · Files and line counts · Snapshot diff (paths only) · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
