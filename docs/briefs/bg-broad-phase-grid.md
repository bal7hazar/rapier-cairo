# BG — sub-quadratic `find_pairs` for large worlds (grid / radix), deterministic

## 1. Read first
`AGENTS.md` (§7 gas cost model; loops pay per iteration run; `Felt252Dict` ops are measured in C1/DF);
`docs/PLAN.md` (D7 stateless broad phase, D9 no persisted scratch; BP's findings: the shipped tail scan
costs **5.4k Sierra gas per non-overlapping pair test** on shuffled proxies, i.e. ≈ 2.7M of the 16.1M
`free_fall32` step; struct-of-arrays lost on construction cost; sort-and-prune with a real merge sort won
only on sparse sets and collapsed on stacks); `docs/BUDGETS.md`; on `main`:
`crates/rapier_geometry2d/src/broad_phase.cairo` (+ its `alternatives`, probes and the shuffled-layout
suite BP built), `crates/rapier_geometry2d/tests/aabb_overlap_golden.cairo`, `crates/rapier_core/src/data/arena.cairo`
(dict usage precedent).

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/broad_phase.cairo` (+ `src/broad_phase/*.cairo`),
`crates/rapier_geometry2d/tests/aabb_overlap_golden.cairo`, and the snapshots that move (module filters
after `gas.py check`). Everything else is forbidden; `find_pairs`'s signature and output contract are
frozen (ascending `(i, j)` with `i < j`, static–static skipped, closed-interval overlap).

## 3. Expected result
A `find_pairs` whose cost grows ~linearly with n for spread-out worlds, bit-identical output, and no
regression for small or dense worlds. Candidates (measure each):
1. **Uniform grid** keyed in a `Felt252Dict` (cell size from the median AABB extent, or a fixed parameter
   documented), each proxy inserted in the cells it covers, candidate pairs deduplicated and emitted in
   ascending order (no dict iteration order may leak: e.g. collect `(i, j)` per `i` then sort, or walk `i`
   ascending and test only candidates `j > i` gathered from its cells);
2. **Radix / counting sort on quantised min-x** (O(n) integer sort) feeding a sweep, then order the output;
3. the shipped tail scan as the small-n branch; ship a size threshold if the crossover is clear.
Inputs in arena order (shuffled), layouts: sparse grid of falling bodies, a stack, a dense pile, a mixed
world with a big static ground (the ground's AABB covers every cell — handle it without O(n) cells per
static, e.g. statics in a separate list tested against dynamic cells).

## 4. Efficiency and variants
Report at n = 8, 32, 64, 128, 256 (Sierra gas and exact Cairo steps, per pair test and total), and the P3
scenes before/after. Target: `free_fall32` broad phase ≤ 1M gas (from ≈ 2.7M) and linear growth to 256;
no regression on `cuboid_stack*`, `mixed_pile8`, `balls_halfspace*`. Losers under `mod alternatives`;
≤ 4 fuzz per module (one comparing every candidate with brute force on random AABB sets incl. statics and
huge statics); ≤ 16 probes; ≤ 800 lines per file. No benchmark-shaped shortcuts (the orchestrator rejects
paths that only fire on the probe's construction order).

## 5. Tests
Golden overlap cases unchanged; equivalence fuzz; `gas_*` per candidate × size × layout.

## 6. Definition of done
Foreground gate (tool timeout 3600000 ms, never background); `gas.py check` then module-filtered
snapshots; if another lot merged meanwhile, rebase and regenerate; conventional commits + trailer (subject
= what ships); push; `gh pr create` per template; `gh pr checks --watch` until green; never merge;
`REPORT.md` (Summary · Candidates × sizes × layouts (gas | exact steps) · Winner / threshold · Scene
budgets · Deviations (none) · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
