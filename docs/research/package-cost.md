# Package cost of the published crates (PK1)

Toolchain: scarb / Cairo 2.19.4 (`.tool-versions`). Measured on `590f612` (after CX1, version `0.1.0-alpha.6`) with
`scripts/consumer_cost.py`, a verbatim copy of nalgebra-cairo's shared script (#61, `bff3462`; its docstring is the
spec), and `consumer_cost.toml`. Rule: programme decision of 2026-09-28 (package granularity), gates as defined by the
programme the same day: **gate 1** ≤ 40,000 library lines (inline tests excluded); **gate 2** the crate's *marginal*
cost, cost(empty consumer of the crate) − cost(empty consumer of its direct dependencies together), ≤ 5 s and ≤ 1 GB;
**gate 3** a product closure over the no-dependency baseline ≤ 15 s and ≤ 3 GB (facades and products are judged on
gate 3 only).

Method: cold `scarb build` of an empty consumer (`SCARB_INCREMENTAL=false`, fresh `target/`), median of 3, minus the
no-dependency baseline (1.86 s, 0.54 GB); peak RSS from `/usr/bin/time -v`, GB = 10⁹ bytes. Local machine: 8 vCPU,
shared (load average 4–8 during the runs), under the project build lock with the machine's build policy
(`RAYON_NUM_THREADS=4`, `nice -n 10`). Memory is reproducible to ±0.01 GB; wall time to about ±0.5 s at this load.
The published crates are the six of `scripts/release.sh`; `rapier_golden`, `rapier_testing` and `rapier_sink` are
`publish = false`, so the script sees exactly those six.

## 1. Per crate

"Over baseline" is what the script reports; "marginal" subtracts the consumer of the crate's direct dependencies
(measured with `--closure deps_of_rapier_geometry2d=rapier_core,rapier_math`; for the other crates the direct
dependencies' consumer is an existing row, or a throwaway wrapper crate for the registry dependencies `fixed`, `glam`
and `starknet`, since the script's closures accept workspace crates only).

| crate | lines | over baseline s / GB | direct deps consumer | deps s / GB | **marginal s / GB** | verdict |
|---|--:|--:|---|--:|--:|---|
| rapier_math | 935 | 4.5 / 0.71 | fixed + glam | 2.9 / 0.70 | **1.7 / 0.00** | ok |
| rapier_core | 3,200 | 1.4 / 0.31 | fixed | 1.3 / 0.23 | **0.1 / 0.09** | ok |
| rapier_geometry2d | 22,013 | 5.5 / 1.28 | rapier_core + rapier_math | 3.4 / 0.81 | **2.1 / 0.47** | ok |
| rapier_dynamics2d | 14,333 | 7.8 / 1.67 | rapier_geometry2d ¹ | 5.5 / 1.28 | **2.3 / 0.38** | ok |
| rapier2d (facade) | 9,966 | 8.3 / 1.85 | rapier_dynamics2d ¹ | 7.8 / 1.67 | 0.6 / 0.18 | gate 3 |
| rapier2d_classes (product) | 3,474 | 8.1 / 1.94 | rapier2d + starknet | 8.7 / 1.86 | ≈0 / 0.09 ² | gate 3 |

¹ Its closure contains every other direct dependency. ² Negative time delta (−0.6 s): noise.

Every crate passes gates 1 and 2. The script's own verdict column (cost over the baseline) marks geometry2d,
dynamics2d, rapier2d and rapier2d_classes as failing 5 s / 1 GB: that column is not gate 2 (see §5).

Lines per top-level module (inline tests excluded):

| crate | modules |
|---|---|
| rapier_math | math_ext 560, consts 126, pose2 112, rot2 108, root 29 |
| rapier_core | collider 956, data 775, rigid_body 703, integration_parameters 488, interaction_groups 242, root 36 |
| rapier_geometry2d | query 5,444, shape 4,242, point 2,534, ray 2,357, contact_generators 2,272, dispatch 1,891, aabb 814, broad_phase 777, mass 380, contact 222, polygonal_feature 219, manifold 214, closest_points 183, sat 160, feature_id 141, clip 112, root 51 |
| rapier_dynamics2d | solver 5,717, joint 2,241, rigid_body_set 1,865, narrow_phase 1,595, rigid_body 1,204, collider 1,173, collider_set 299, events 211, root 28 |
| rapier2d | pipeline 6,533, control 1,328, world 1,073, queries 857, dispatcher 89, root 86 |
| rapier2d_classes | advance 1,108, islands 804, contact 319, narrow 207, active_set 174, forces 159, solver 152, orchestrator 125, config 112, mass 63, broad_phase 64, arena 87, hashes 47, root 53 |

## 2. Closures (gate 3)

| closure (`consumer_cost.toml`) | members | over baseline s / GB | verdict |
|---|---|--:|---|
| `rapier2d` (in-process step: a game or a harness) | rapier2d | 8.9 / 1.85 | ok (15 s / 3 GB) |
| `game_classes` (Starknet game library-calling the step classes) | rapier2d_classes | 9.0 / 1.95 | ok |

`game_classes` contains every published crate, so it is also the whole-repository closure. No other product closure
is justified today: a consumer of `rapier_geometry2d` alone is the crate row (5.5 s / 1.28 GB). These numbers match the
programme's alpha.6 measurements (rapier2d 10.4 s / 2.5 GB absolute, here 10.2 s / 2.39 GB).

## 3. What moving the inline tests out would save

A throwaway copy of the repository (`/tmp`, not committed) had every test-only item removed from the six crates: the
inline `#[cfg(test)]` blocks and the files of the `#[cfg(test)] mod x;` declarations. It builds (`scarb build -p
rapier2d`, `-p rapier2d_classes`). Consumers of the original and of the stripped sources were then built interleaved
(baseline, original, stripped; 5 rounds; medians), so load drift hits both sides alike.

| crate | source lines (all) | library | inline test code | test-only files | original s / GB | stripped s / GB | **gain s / GB** |
|---|--:|--:|--:|--:|--:|--:|--:|
| rapier_math | 2,599 | 935 | 1,664 | 0 | 5.27 / 1.260 | 4.85 / 1.253 | 0.4 / 0.006 |
| rapier_core | 9,355 | 3,200 | 4,031 | 2,124 | 3.63 / 0.854 | 4.29 / 0.848 | −0.7 ³ / 0.006 |
| rapier_geometry2d | 42,061 | 22,013 | 8,867 | 11,181 | 7.12 / 1.829 | 6.86 / 1.801 | 0.3 / 0.029 |
| rapier_dynamics2d | 30,811 | 14,333 | 5,108 | 11,370 | 9.20 / 2.207 | 8.86 / 2.164 | 0.3 / 0.043 |
| rapier2d | 22,489 | 9,966 | 1,122 | 11,401 | 9.85 / 2.392 | 9.63 / 2.353 | 0.2 / 0.039 |
| rapier2d_classes | 3,508 | 3,474 | 34 | 0 | 10.37 / 2.481 | 9.75 / 2.451 | 0.6 / 0.030 |

Absolute consumer builds (baseline ≈ 1.6–1.8 s, 0.54 GB). Each row strips the crate *and* its dependencies.
³ Noise: time deltas under ~0.5 s are not significant at this load.

**The gain is small: at most ~0.04 GB (≈ 2 % of a closure) and a few tenths of a second.** The reason: a file
declared by `#[cfg(test)] mod x;` is never read by a non-test build, so the 36k test-only file lines (the bulk of
R7's "40 % of geometry") already cost a consumer nothing; only the 21k lines of inline `#[cfg(test)]` blocks are
parsed and then dropped. Moving tests out of the published sources is worth it for package hygiene (tarball size,
alexandria's layout), not for the consumer-cost gates.

Cost of the move (estimated, not attempted):

| crate | test modules (inline + file) | of which name crate-private items ⁴ | `mod alternatives` (of which private) | in-crate gas keys that would change |
|---|--:|--:|--:|--:|
| rapier_math | 11 | 0 | 4 (0) | 132 |
| rapier_core | 33 | 6 | 14 (4) | 490 |
| rapier_geometry2d | 111 | 29 | 31 (13) | 1,306 |
| rapier_dynamics2d | 78 | ≤ 46 | 21 (13) | 909 |
| rapier2d | 45 | 20 | 14 (8) | 561 |
| rapier2d_classes | 1 | 0 | 0 | 0 |

⁴ Heuristic: names imported through `use super::…` / `use crate::…` or `super::x` paths that are only declared
without `pub` (or in a private module). dynamics2d is an upper bound (48 `use super::*` globs, name-matched). About 100
modules (a third) cannot move as they are: each needs its helpers made `pub` (API growth, visible in
`docs/API_PARITY.md`) or stays in-crate. The `mod alternatives` modules test the losers against private helpers by design
(AGENTS.md §5); moving them means publishing those helpers. **Every gas-snapshot key changes**: a key is the test's
module path, so `rapier_geometry2d::aabb::bounding_volume::tests::gas_ball_aabb` would become
`rapier_geometry2d_integrationtest::…` (3,398 keys, 48 snapshot files rewritten; values should be identical since
inlining crosses crates, which the diff would have to prove).

## 4. Cut plan

None needed: with or without tests, every crate passes gate 2 on its marginal cost (largest: dynamics2d 2.3 s /
0.38 GB, geometry2d 2.1 s / 0.47 GB) and both closures pass gate 3 with ≥ 5.9 s / 1.05 GB of headroom. For reference,
should the programme judge gate 2 on the cost over the baseline instead, `rapier_geometry2d` (5.5 s / 1.28 GB) would
need about 0.3 GB less; the upstream-shaped cut would be `shape` + `mass` + `aabb` (≈ 5.4k lines), `query` + `point` +
`ray` + `closest_points` (≈ 10.5k), contact manifolds (`contact_generators`, `dispatch`, `sat`, `clip`, `manifold`,
`polygonal_feature`, `contact`, `feature_id`, `broad_phase`: ≈ 6.0k) behind a `rapier_geometry2d` facade; at the measured
slope (≈ 0.1 s and 21 MB per 1,000 lines for geometry2d's marginal) each part would add ≈ 0.5 / 1.0 / 0.6 s and
0.11 / 0.22 / 0.13 GB to its dependencies, but dynamics2d and the facades still pull all of them, so no closure gets
cheaper. Not recommended.

## 5. CI and open points

* `lint` job: the lines-only report (`--lines-only --report-only --modules`) and a check that no published crate lists
  `rapier_golden`, `rapier_testing` or `rapier_sink` under `[dependencies]`.
* `consumer-cost` job: report only (step summary + `consumer_cost.json` artifact), with
  `deps_of_rapier_geometry2d` so geometry2d's marginal can be read from the report.
* Before making the job enforcing (alpha.7): the script's verdict is the cost over the baseline, which fails four
  crates that pass gate 2. It needs the programme's `marginal` column (asked of nalgebra-cairo), and closures of
  registry crates (`fixed`, `glam`, `starknet`) to measure rapier_math's and rapier_core's direct dependencies.
* Script finding for nalgebra-cairo: `--manifest-path` is passed after `scarb metadata`, which scarb 2.19.4 rejects
  (`unexpected argument '--manifest-path'`); it must precede the subcommand.

## Update (after 0.1.0-alpha.7): gate 2 enforced

The shared script (nalgebra-cairo 7177cf3, copied unchanged) computes each crate's marginal cost itself (a consumer of
the crate minus a consumer of its direct dependencies together, registry crates included) and judges a facade
(`facades = ["rapier2d"]` in `consumer_cost.toml`) on gate 3 only; CI enforces gates 1–3. Local run (load ≈ 10 on 8
vCPU, so the times are noisy; the memory figures are stable), marginal cost: rapier_math 3.0 s / 0.00 GB, rapier_core
1.1 / 0.10, rapier_geometry2d 3.4 / 0.49, rapier_dynamics2d 1.4 / 0.38, rapier2d_classes 1.9 / 0.09; closures
`rapier2d` 1.85 GB, `game_classes` 1.93 GB over the baseline. Every gate passes.
