# Step Budgets

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
