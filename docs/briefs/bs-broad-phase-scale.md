# BS — a scale-free cell size for the strip/grid broad phase

## 1. Read first
`AGENTS.md` (§7; "never `pow()` at runtime: const table or `match`"; `DivRem` by a `NonZero` constant is
cheap, by a runtime value is not — measure); `docs/PLAN.md` (BP, BG findings); BG's REPORT in PR #81's body;
on `main`: `crates/rapier_geometry2d/src/broad_phase.cairo` (`find_pairs` dispatches: tail scan < 32,
x strips 32–63, 2D `Felt252Dict` grid ≥ 64) and `broad_phase/{grid,strip,ordering,alternatives,benches,tests}.cairo`
— `grid::cell(raw)` divides by the constant `0x400000000` (4.0 in Q32.32): the cell is 4 world units
whatever the scale; wide boxes (grounds) are kept out of the cells; crowded cells fall back to a scan.
`crates/rapier_core/src/integration_parameters.cairo` (`length_unit`).

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/broad_phase.cairo` (+ `broad_phase/*.cairo`),
`crates/rapier_geometry2d/tests/aabb_overlap_golden.cairo`, and the snapshots that move (module filters after
`gas.py check`). `find_pairs(Span<BroadPhaseProxy>) -> Array<(u32, u32)>` keeps its signature and contract
(the cell size must therefore come from the proxies themselves). Everything else is forbidden (lot CL runs in
parallel on dispatch / pipeline / narrow phase).

## 3. Expected result
The strip and grid perform the same on a world scaled by 1/16, 1, 16 and 256 (same layout, coordinates and
extents multiplied): today a ×16 world puts every body in its own crowded-or-wide situation and a ×1/16
world puts many bodies per cell. Candidates: (a) cell = a power of two derived from the median (or a cheap
robust statistic, e.g. the max of a sample) of the dynamic proxies' extents, applied through a `match` on the
exponent over constant divisors (no runtime `pow`, no runtime divisor); (b) quantise with a precomputed
`RecipNearest`-like multiply-shift; (c) keep 4.0 but choose the exponent per call from `length_unit`-like
information carried by the proxies — only if (a)/(b) lose. Output bit-identical to brute force for every
scale (fuzz with scale factors), deterministic, no dict order leak.

## 4. Efficiency and variants
Report at n = 32, 64, 256 × scale 1/16, 1, 16, 256 × layouts (sparse, stack, dense, sparse + huge ground),
shuffled arena order, Sierra gas and exact Cairo steps; plus the P3 scenes (scale 1) before/after. Target:
at every scale within 25 % of today's scale-1 cost; at scale 1, P3 scenes not worse than +2 %. ≤ 4 fuzz per
module, ≤ 16 probes, ≤ 800 lines per file, losers under `mod alternatives`.

## 5. Tests
Golden overlap cases unchanged; scale-sweep equivalence fuzz against brute force.

## 6. Definition of done
Foreground gate (tool timeout 3600000 ms, never background); `gas.py check` then module-filtered snapshots;
if lot CL merged meanwhile, rebase and regenerate; conventional commits + trailer; push; `gh pr create` per
template; `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · Scale × size × layout table ·
Winner · Scene budgets · Deviations (none) · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
