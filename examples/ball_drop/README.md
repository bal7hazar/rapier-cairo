# Ball Drop Executable

Standalone `#[executable]` package for P4: build a small `rapier2d` scene, run
`World::step` a fixed number of times, and emit final body state as raw Q32.32 felts.

This package is intentionally outside the root workspace. Executable targets require
`[cairo] enable-gas = false`, which is not compatible with the snforge-tested workspace crates.

## Run

From the repository root:

```sh
scripts/prove-example.sh [scene] [steps]
```

Scenes:

| name | id | dynamic bodies |
| --- | ---: | ---: |
| `ball_drop` | 0 | 1 |
| `box_stack3` | 1 | 3 |
| `pendulum` | 2 | 1 |

The default is `scripts/prove-example.sh ball_drop 10`. The script always runs
`scarb build` and `scarb execute --print-program-output --print-resource-usage`. It runs
`scarb prove` and `scarb verify` only when `PROVE=1` is set:

```sh
PROVE=1 scripts/prove-example.sh ball_drop 1
```

`PROVE=1` is for larger machines only. The orchestrator measured the Stwo prover being
OOM-killed above a 22 GB cgroup cap even for one physics step (measured on an early wip executable of 4,781 Cairo
steps; the one-tick run of the table below is 19,587 steps including world construction),
`prover_input.json` = 105 MB, and `memory.address_to_id` = 2.7M cells. No proof was
produced on this machine, so the proof size is unavailable here.

## Output

`main(scene: u8, steps: u32) -> Array<felt252>` returns:

```text
[
  num_collision_events,
  num_dynamic_bodies,
  per dynamic body in scene order:
    translation.x,
    translation.y,
    rotation.re,
    rotation.im,
    linvel.x,
    linvel.y,
    angvel,
]
```

The pose and velocity entries are raw `i64` Q32.32 values serialized as felts.

## Measurements

Measured on this shared executor machine with `PROVE` unset, after the package had already
been built:

| scene | steps | execute wall | Cairo steps | range_check | bitwise | output | events |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `ball_drop` | 1 | 21.103s | 19,587 | 760 | 33 | 10 | 0 |
| `ball_drop` | 10 | 23.103s | 143,976 | 5,998 | 132 | 10 | 0 |
| `ball_drop` | 60 | 28.007s | 1,578,926 | 90,381 | 684 | 10 | 1 |

P1's `snforge` net step probes in `docs/PLAN.md` report free fall at 14,654 Cairo steps
per step and a resting ball at 42,654 Cairo steps per step. The 10-step executable averages
14,398 Cairo steps per requested step, within 2% of the free-fall probe even though it also
includes scene construction and output serialization. The 60-step run is higher because the
ball reaches the half-space and spends later steps in contact; its average sits between the
free-fall and resting-contact P1 probes.

The stage that dominates locally is proof generation, not execution: execution is seconds,
while proving did not complete under the 22 GB cgroup cap. Proving-time scaling with step
count is therefore deferred until a machine with enough RAM is available.

## Deferred

On-chain verification, a Starknet contract wrapper (`rapier_starknet`), and JSON inputs are
intentionally deferred.
