# Step Budgets

## Since the matrix below (2026-09-25)

- SE #127 (sensors): every P3 scene ≤ +0.21 %; ceilings unchanged.
- RB #121 (rigid-body API, cold data boxed): contact and joint scenes +0.05 % to +0.29 % net; `free_fall1/8/32`
  −20.5 / −8.5 / −6.5 % net through a no-contact fast path (`pipeline/free_path.cairo`) that fires only when the world
  has no pair and no joint, so the free-fall probes no longer measure the general per-body path (+13 % net on
  `free_fall32` without the fast path); `gas_setup_*` (world construction) +3.7 %. Both are BT items (`docs/PLAN.md`).
  Exact steps after RB: `free_fall32` 535,449; `balls_halfspace32` 1,856,887; `cuboid_stack10` 754,143;
  `mixed_pile8` 810,808. The full matrix is refreshed with G0.

## Current (2026-09-24 evening, after OS #68, OP #69, OI #73, BP #72, OJ #78, DO #80, BG #81)

One settled `World::step`, net of setup. Sierra gas from `gas_step_* − gas_setup_*`
(`gas/rapier2d_integrationtest/gas_scenes.snap`); Cairo steps from the uncapped `steps_step_*` twins minus
`gas_setup_*` (`snforge test -p rapier2d gas_scenes --detailed-resources --tracked-resource cairo-steps`).
Every `#[available_gas]` ceiling (on the gross `gas_step_*` tests) is reset to +10 % of the gross value
measured here.

| scene | Sierra gas | Cairo steps | Δ gas vs 09-24 morning | Δ gas vs 09-22 |
|---|---:|---:|---:|---:|
| free fall 1 | 629,514 | 5,573 | +0 % | -50 % |
| free fall 8 | 3,592,502 | 31,200 | -1 % | -44 % |
| free fall 32 | 13,950,428 | 120,746 | -13 % | -47 % |
| balls on half-space 1 | 3,976,369 | 31,637 | +1 % | -23 % |
| balls on half-space 8 | 27,211,322 | 210,893 | +1 % | -26 % |
| balls on half-space 32 | 108,685,038 | 842,894 | -0 % | -26 % |
| cuboid stack 1 | 4,234,319 | 37,434 | +1 % | -30 % |
| cuboid stack 3 | 12,916,927 | 105,871 | +1 % | -30 % |
| cuboid stack 5 | 21,638,655 | 174,664 | +1 % | -30 % |
| cuboid stack 10 | 43,614,125 | 348,204 | +1 % | -30 % |
| mixed pile 8 | 48,471,917 | 386,326 | +1 % | -29 % |
| pendulum chain 1 joint | 3,703,566 | 32,970 | -23 % | -27 % |
| pendulum chain 3 joints | 9,987,414 | 88,697 | -25 % | -27 % |

Since the morning matrix: joints −23 to −25 % (OJ), free fall 32 −13 % (grid broad phase BG), contact
scenes +1 % (D8 fixed-last partition, DO). Open targets: narrow phase per pair (lot ON), per-contact solver
cost (a resting ball still costs ≈ 3.4M per step), world-scale-aware broad-phase cell size.

## Morning matrix (2026-09-24, after OS, OP, OI)

One settled `World::step`, net of setup. Sierra gas from `gas_step_*`; Cairo steps from the uncapped
`steps_step_*` twins (`snforge test -p rapier2d gas_scenes --detailed-resources --tracked-resource
cairo-steps`). Ceilings (`#[available_gas]` on the gross `gas_step_*` tests) are +10 % of the measured
gross value. Δ is against the 2026-09-22 matrix below.

| scene | Sierra gas | Cairo steps | Δ gas since 09-22 |
|---|---:|---:|---:|
| free fall 1 | 627,054 | 5,556 | −50 % |
| free fall 8 | 3,620,842 | 31,596 | −44 % |
| free fall 32 | 16,072,618 | 141,708 | −39 % |
| balls on half-space 1 | 3,941,139 | 31,296 | −23 % |
| balls on half-space 8 | 27,010,752 | 209,005 | −26 % |
| balls on half-space 32 | 109,145,808 | 847,309 | −26 % |
| cuboid stack 1 | 4,199,089 | 37,093 | −31 % |
| cuboid stack 3 | 12,808,177 | 104,823 | −30 % |
| cuboid stack 5 | 21,462,065 | 172,977 | −30 % |
| cuboid stack 10 | 43,292,785 | 345,217 | −30 % |
| mixed pile 8 | 48,060,297 | 382,345 | −30 % |
| pendulum chain 1 joint | 4,823,046 | 39,120 | −5 % |
| pendulum chain 3 joints | 13,351,774 | 107,204 | −3 % |

Marginals now (least squares, gas | steps): free fall ≈ 495k | 4.3k per body; balls on half-space ≈ 3.4M |
26k per contact body; cuboid stack ≈ 4.3M | 34k per box (2 points); joints still ≈ 4.3M | 34k per joint.
Open targets: `find_pairs` O(n²) (lot BP), joints (untouched by OS), narrow phase per pair.

## History: first matrix (2026-09-22, P3)

Measured 2026-09-22 with Scarb 2.19.4 / snforge 0.61.0. Sierra gas is
`gas_step_* - gas_setup_*` from `gas/rapier2d_integrationtest/gas_scenes.snap`; Cairo steps use
the same subtraction from
`snforge test -p rapier2d gas_scenes --detailed-resources --tracked-resource cairo-steps`.

Contact scenes are exact-touching, warm-started synthetic states. The contact families use zero
gravity so the budget isolates steady contact maintenance; free fall and pendulum use the default
gravity.

| scene | bodies | active pairs | solver points | Sierra gas | Cairo steps |
|---|---:|---:|---:|---:|---:|
| free fall 1 | 1 | 0 | 0 | 1,260,534 | 11,314 |
| free fall 8 | 8 | 0 | 0 | 6,419,422 | 57,367 |
| free fall 32 | 32 | 0 | 0 | 26,294,398 | 236,095 |
| balls on half-space 1 | 1 | 1 | 1 | 5,147,979 | 42,655 |
| balls on half-space 8 | 8 | 8 | 8 | 36,543,252 | 301,032 |
| balls on half-space 32 | 32 | 32 | 32 | 147,223,428 | 1,215,912 |
| cuboid stack 1 | 1 | 1 | 2 | 6,067,649 | 54,359 |
| cuboid stack 3 | 3 | 3 | 6 | 18,411,257 | 157,339 |
| cuboid stack 5 | 5 | 5 | 10 | 30,799,665 | 260,743 |
| cuboid stack 10 | 10 | 10 | 20 | 61,966,685 | 521,108 |
| mixed pile 8 | 8 | 15 | 20 | 68,514,067 | 578,765 |
| pendulum chain 1 joint | 2 | 0 | 0 | 5,052,186 | 41,544 |
| pendulum chain 3 joints | 4 | 0 | 0 | 13,697,834 | 111,414 |

## Marginals

Least-squares fits over the sized families:

| family | marginal | Sierra gas | Cairo steps |
|---|---|---:|---:|
| free fall | per dynamic body | 812,838 | 7,301 |
| balls on half-space | per body/pair/point | 4,590,435 | 37,917 |
| cuboid stack | per body/pair | 6,213,444 | 51,884 |
| cuboid stack | per manifold point | 3,106,722 | 25,942 |
| pendulum chain | per joint | 4,322,824 | 34,935 |

Sensor pairs (SE #127, `pipeline::sensor_benches`, narrow phase per pair per step, net): ball–ball 111,309 gas /
1,010 steps; cuboid–ball 117,959 / 1,066; cuboid–cuboid 133,829 / 1,188 (the same pair as a contact pair: 721,015 /
5,120); triangle–cuboid 288,929 / 2,605. P3 scenes (no sensor) moved ≤ +0.21 % with SE; ceilings unchanged.

`scripts/gas.py rank rapier2d_integrationtest::gas_scenes` ranks the gross probes from
`gas_baseline` (14,120) through `gas_step_balls_halfspace32` (312,754,634); the net table above is
the budget source because each scene subtracts its matching warm-up/setup probe.

## Targets

Using P1's settled `BOX_STACK3` split, whole step 17,895,799 Sierra gas:

| target | share | expected gain for a brief |
|---|---:|---|
| solver contact sweeps | 80.7% | A 20-25% solver reduction saves 2.9-3.6M gas on stack 3. |
| narrow phase per pair | 13.5% | A 25-30% dispatcher/generator reduction saves 0.6-0.7M gas on stack 3. |
| broad-phase proxies | 1.9% | A 50% proxy rebuild reduction saves ~0.17M gas on stack 3; prioritize only if it also helps many-static scenes. |

## Notes

The complete required matrix needs 13 setup/step pairs plus `gas_baseline` (27 tests). This exceeds
the brief's 24-probe target, but preserves exact per-step Sierra gas instead of dropping scenes or
reporting setup-inclusive costs.
