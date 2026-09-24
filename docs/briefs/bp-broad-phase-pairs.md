# BP — make `find_pairs` cheap: per-pair cost and asymptotics

## 1. Read first
`AGENTS.md` (§7 gas cost model: a loop-free body is charged its most expensive path on every iteration;
a one-iteration `while pending { …; pending = false; }` "meters" an expensive arm so it is only paid when
taken — measured on the dispatcher in GM/P1); `docs/PLAN.md` (D7 stateless broad phase, GA's ranking:
brute force won up to n = 64 at ~7k gas per pair test; OP's escalation: at 32 falling bodies `find_pairs`
costs 3.1M gas, 97k per body, ≈ 6.2k per pair test); on `main`: `crates/rapier_geometry2d/src/broad_phase.cairo`
(`find_pairs` → `find_pairs_brute`, and `alternatives::find_pairs_sort_and_prune`), its golden test
`crates/rapier_geometry2d/tests/aabb_overlap_golden.cairo`, `crates/rapier2d/tests/gas_scenes.cairo`
(P3 scene probes: `free_fall{8,32}`, `balls_halfspace32`, `mixed_pile8`).

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/broad_phase.cairo` (+ `src/broad_phase/*.cairo` if you split),
`crates/rapier_geometry2d/tests/aabb_overlap_golden.cairo`, and the snapshots that move
(`gas/rapier_geometry2d/broad_phase.snap`, `gas/rapier_geometry2d_integrationtest/aabb_overlap_golden.snap`,
`gas/rapier2d*/**` — module filters after `gas.py check`). Forbidden: every other file; `find_pairs`'s
signature and its output contract (pairs of proxy indices `(i, j)`, `i < j`, ascending lexicographic
order, static–static pairs skipped, closed-interval overlap as upstream's `Aabb::intersects`).

## 3. Expected result
The same pair list, bit for bit, cheaper. Candidates to write and measure:
1. **Metered brute force**: the inner body pays `pairs.append` only on overlap (metered append), the
   four raw comparisons without short-circuit branches or with, whichever measures cheaper; iterate
   with `pop_front` on span tails instead of `at(j)`; hoist `a`'s fields.
2. Static/dynamic split: dynamic × dynamic and dynamic × static loops (fixed bodies never pair with
   each other) — must still emit pairs in the global ascending order (merge or index trick).
3. Sort-and-prune re-ranked with the same metering (its sort is a loop: measure where it overtakes).
4. Optional: a coarse uniform grid keyed in a `Felt252Dict` — only if 1–3 leave a clear gap at n ≥ 64;
   deterministic output order is mandatory (sort the result), no dict iteration order observable.
Ship `find_pairs` as the winner, or as a size threshold between two winners if the crossover is clear
(document the threshold and the measurement).

## 4. Efficiency and variants
Measure every candidate at n = 8, 32, 64, 128 on three layouts (sparse grid with no overlap, a stack,
a dense pile) in **Sierra gas and Cairo steps**, plus the P3 scenes before/after. Target: ≤ 2k Sierra gas
per non-overlapping pair test at n = 32, and `gas_step_free_fall32` at least 2M lower. Losers stay under
`mod alternatives` with probes; ≤ 4 fuzz per module (one fuzz comparing every candidate with brute force on
random AABB sets, statics included); ≤ 800 lines per file.

## 5. Tests
Existing golden overlap cases unchanged; the equivalence fuzz; `gas_*` per candidate × size × layout
(keep the matrix small: ≤ 16 probes).

## 6. Definition of done
Foreground gate (tool timeout 3600000 ms, never background); `gas.py check` then module-filtered
snapshots; if another lot merged meanwhile, rebase and regenerate; conventional commits + trailer;
push; `gh pr create` per template (table in the body); `gh pr checks --watch` until green; never merge;
`REPORT.md` (Summary · Candidates × sizes × layouts (gas | steps) · Winner / threshold · Scene budgets ·
Deviations (none) · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
