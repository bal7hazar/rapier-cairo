# Step Budgets

## WS3 — `WorldState` v4 with the v3 migration (2026-10-03)

`WorldState` version 4 (new fields reserved and written empty, version 3 migrated); WS3's lever and the parity items
`take_removed` / `user_data` measured and not shipped (`docs/research/impact-tick.md` §10, with their figures). Results
bit-identical: every test unchanged, the per-tick digests of the version-3 felts of the state equal alpha.9's (pile10
both shots in process and slim, 520 ticks; levels 10 and 20, 180 ticks).

**The step is unchanged** (exact Cairo steps, Scarb 2.20.1 / snforge 0.64.0, `RAYON_NUM_THREADS=1`, the Mac, build path
`/Users/bal7hazar/.herdr/worktrees/rapier-cairo/hp-slingfall-rapier-t-0047-ws3-dormant-pairs-codec`, before = `main`
at `b1670ed`): of the 164 committed `steps_*` probes, 156 are identical, among them the P3 scenes, the level windows
(`steps_{load,flight,impact}_level{10,20}`, `steps_asleep*`, `steps_flight1*`), `steps_game_{step,force,reads,despawn}`
and the pile10 shots (`steps_basic_151` 22,508,552, `steps_slim_151` 29,501,744, every tick probe). The 8 that move
are the ones that encode or decode a state: `steps_game_{chunked,serde_trips}` and `steps_game_basic_chunked` +764
(three round trips), `steps_game_state_trips` +104, `test_basic_codec_steps_the_same` +403, `steps_edit_*_class`
+426 (`WorldEditClass` takes a `BasicWorldState`). Sierra gas: no existing entry of the step moves; the changed
entries of `gas/rapier2d*/**` are all tests that serialize or restore a state (`world::state`, `world::basic_state`,
`world_state`, the chunked `game_path` probes, `level_budget`'s impact digests, three force-event checks that round
trip their scene).

Declared classes (felt counts, path-free, `scripts/bytecode_size.py table`; CI's `bytecode` log in the PR): the ten
library classes unchanged (no hash changes); `SlimSplitStep` 66,890 → 67,700 CASM felts (margin 6,028),
`SlimEditStep` 68,632 → 69,467 (margin 4,261), `WorldEditClass` 51,380 → 52,205: the codec's reserved fields and the
version check.

## EL1 — engine levers (2026-10-03)

IT1's in-scope step levers (`docs/research/impact-tick.md` §8). R1: `remove_body` wakes and releases a collider's pairs
in one walk. F1: the force-event pass keeps the pair list when no status bit changes, and reads three collider fields
instead of whole colliders (`ColliderSetTrait::get_field`). W1: `body_status` reads the slot's entry before walking.
N1's single unbox measured 0 and was dropped; S1, U1, G1 and the rest of W1 found no bit-identical candidate
(reasons and figures in §8). Results bit-identical: every test unchanged, and the per-tick digests of both shots in
both layouts equal before and after. Exact Cairo steps, Scarb 2.20.1 / snforge 0.64.0, `RAYON_NUM_THREADS=1`, on the
Mac (Apple silicon arm64), build path
`/Users/bal7hazar/.herdr/worktrees/rapier-cairo/hp-slingfall-rapier-t-0035-el1-engine-levers`, before = `main` after
CX3 (`665e999`):

| pile10 shot | in process before → after | Δ | slim before → after | Δ |
|---|--:|--:|--:|--:|
| owner's (−1022, −63), 151 ticks | 22,867,951 → 22,507,444 | −360,507 (−1.58 %) | 29,849,153 → 29,500,630 | −348,523 (−1.17 %) |
| reference (−604, −392), 107 ticks | 8,752,430 → 8,585,434 | −166,996 (−1.91 %) | 12,065,485 → 11,906,622 | −158,863 (−1.32 %) |
| owner's impact tick (42) | 473,902 → 445,477 | −28,425 (−6.00 %) | 525,985 → 498,535 | −27,450 (−5.22 %) |
| reference impact tick (82) | 478,016 → 449,595 | −28,421 (−5.95 %) | 529,995 → 502,549 | −27,446 (−5.18 %) |
| owner's tick after the impact, average of 108 | 199,371.9 → 196,464.5 | −2,907 (−1.46 %) | 255,961.9 → 253,152.6 | −2,809 (−1.10 %) |
| reference tick after the impact, average of 24 | 288,849.1 → 284,205.3 | −4,644 (−1.61 %) | 368,790.4 → 364,427.6 | −4,363 (−1.18 %) |

`rapier2d` `steps_*` probes: game path −1.07 to −1.49 % (`steps_game_step` 2,758,024 → 2,726,098), asleep levels −1.5 to
−1.6 %, level impacts −0.14 / −0.19 %, P3 contact scenes −0.05 to −0.15 %, free fall and joints unchanged; none rose.
Sierra gas (`gas/rapier2d/**`): median −0.08 %, −8.8 % to +0.62 % (the rises are on force-event and panic paths: Sierra
gas charges the costliest path, now including the rebuild branch). Proofs (estimate, the game's basis of 154 L2 gas
per step): owner's shot 5.28e9 → 5.226e9 virtual L2 gas, still 6 proofs (5 need ≈ 1.47M more steps); reference shot
2.57e9 → 2.546e9, still 3. Classes whose hash changes: `IslandsClass` (W1) and `ForceEventsClass` (F1).

## CX3 — slim crossings (2026-10-02)

`NarrowPhaseClass` runs its own pair loop on the previous pairs as they cross, each pair's contacts generated where the
loop reaches it (a pair with a ball by one `ContactBallClass` call), instead of CX2's jobs then loop. Results
bit-identical (every test and `*_bit_identical` probe unchanged; the reference shot checked at every tick, slim against
in process). Exact Cairo steps, Scarb 2.20.1 / snforge 0.64.0, `RAYON_NUM_THREADS=1`, on the Mac (Apple silicon arm64),
build path `/Users/bal7hazar/.herdr/worktrees/rapier-cairo/hp-slingfall-rapier-t-0017-cx3-slim-crossings-x1`, before =
`main` after TC1 (`5bfc4d3`):

| pile10 shot | in process | slim before | slim after | Δ |
|---|--:|--:|--:|--:|
| owner's (−1022, −63), 151 ticks | 22,867,951 | 30,941,790 (+35.3 %) | 29,849,153 (+30.5 %) | −1,092,637 (−3.53 %) |
| reference (−604, −392), 107 ticks | 8,752,430 | 12,548,826 (+43.4 %) | 12,065,485 (+37.9 %) | −483,341 (−3.85 %) |

Per tick (slim, before → after): owner's flight tick 30 26,554 → 25,119, impact tick 42 527,257 → 525,985, collapse tick
50 275,932 → 264,857; reference impact tick 82 531,265 → 529,995, collapse tick 90 383,852 → 369,291; build and settle
497,850 → 484,856. `NarrowPhaseClass` 22,027 / 68,470 → 22,667 / 67,179 Sierra / CASM felts (margin 51,061 / 6,549); the
caller and every other class unchanged (`SlimSplitStep` 26,844 / 67,108, margin 6,620 CASM). Proofs (estimate, the game's
basis of 154 L2 gas per step): the owner's shot 5.445e9 − 0.168e9 = 5.28e9 virtual L2 gas, still 6 proofs (5 need
≤ 5.0e9, ≈ 1.8M more steps); the reference shot 2.645e9 − 0.074e9 = 2.57e9, still 3. The absolute figures are the
game's cost sheet on alpha.8 (Scarb 2.19.4, before TC1), not re-measured after TC1 (+1.25 to +1.38 % steps on the game
path): they are lower bounds. Details:
`docs/research/class-split.md`, section 13.

## Toolchain 2.20.1 / 0.64.0 (TC1, 2026-10-02)

Scarb 2.19.4 / snforge 0.61.0 → Scarb 2.20.1 (Cairo 2.20.0) / snforge 0.64.0; results bit-identical (every test and
`*_bit_identical` probe passes unchanged). Every figure with `RAYON_NUM_THREADS=1`. Felt counts are path-free: measured
on the Mac (`scripts/bytecode_size.py table`, worktree
`/Users/bal7hazar/.herdr/worktrees/rapier-cairo/hp-slingfall-rapier-t-0007-tc1-toolchain-bump`); the same run on 2.19.4
equals the committed `gas/bytecode.size` (Linux) for all 32 classes. Declared classes against 73,728 felts:

| declared class | Sierra felts | CASM felts | margin (Sierra / CASM) |
|---|--:|--:|--:|
| `SlimSplitStep` (caller) | 26,834 → 26,844 | 67,076 → 67,108 | 46,884 / 6,620 (−32) |
| `SlimEditStep` (caller) | 27,135 → 27,145 | 68,818 → 68,850 | 46,583 / 4,878 (−32) |
| `NarrowPhaseClass` | 22,008 → 22,027 | 68,372 → 68,470 | 51,701 / 5,258 (−98) |
| `IslandsClass` | 7,952 → 7,970 | 19,083 → 19,228 | 65,758 / 54,500 (−145) |
| the 9 others | unchanged | unchanged | unchanged (smallest: `SolveAdvanceClass` CASM 15,181) |

Programs: `full` 565,378 → 566,269 felts, `basic` 231,196 → 231,554. Exact Cairo steps of the `steps_*` probes: rapier2d
50 probes, 44 moved, +0 to +1.84 % (median +1.33 %; game path +1.25 to +1.38 %, `steps_impact_level20` 3,401,223 →
3,463,774, free fall unchanged); `rapier2d_classes` 111 probes, 110 moved, median +1.61 %, max +3.78 % on a shot
(`steps_batched_in_process_0`), `steps_install_stored` 6,246 → 7,746, `steps_install` unchanged. Sierra gas: the snforge
harness −7,710 per test, net of it median 0, whole shots up to +1.8 %.

## Composite grounds (SH2a #184, 2026-09-27)

Per step, one body resting on the ground (Sierra gas / Cairo steps): ball on a 10- / 50-segment polyline 3.58M / 30.8k,
3.89M / 32.7k; box 5.12M / 44.0k, 5.89M / 49.0k; on a 10- / 50-cell heightfield: ball 3.83M / 4.20M gas, box 5.33M /
6.14M gas; the half-space references are ball 2.16M / 19.0k, box 2.51M / 22.3k. The implicit-tree prefilter beats a
linear scan from ~50 parts (1.87M vs 2.13M gas at 50, 6.80M vs 8.20M at 200). Existing pairs pay +2 steps per pair per
step for the group check (level windows +107 to +139 steps on ≈ 2M).

## The game on 0.1.0-alpha.6 (slingfall #35, 2026-09-27)

Six levels pass unchanged; Cairo steps −1.9 % to −4.0 % on its 11 goldens; program 582k → 272k felts
(`step_with_force_events_with::<BasicStepConfig>`). CS2's saving scales with contact pairs: −551 steps on a flight step
(11,918 → 11,367), −6,853 on a pile10 impact step (398,167 → 391,314). SF1 (the numeric change) on the owner's pile10
shot (−1022, −63): the score (5300) still wins, but the collapse settles later — 151 ticks instead of 106, 22.0M steps
instead of 16.1M. `SlingfallSim` contract class 140,568 Sierra felts (1.72× the limit; the in-class path is closed).

## Program size (CS2 #193, 2026-09-27)

A game-shaped executable's program (felts): full step 565,378; `BasicStepConfig` 231,196 (−59.1 %; no joints −29.4 %,
no composites −11.1 %, no sensors −6.8 %, basic dispatcher −1.4 %). Basic vs full: −1,612 Cairo steps per step,
results bit-identical. Details: `docs/research/class-size.md` §7.

## SF1 #191 (contact-separation rebase, 2026-09-27; results change)

Exact Cairo steps, before → after: `cuboid_stack10` 421,484 → 421,780; `mixed_pile8` 403,413 → 402,755;
`impact_level10` 2,685,306 → 2,685,623; `steps_game_*` +30 to +317 (≤ +0.03 %); free fall and joints unchanged. The
rebase costs 137 / 172 steps per manifold (ground / body pairs) and seeds the first substep's separations.

## Game-shaped path (RG1 #189, 2026-09-27)

`crates/rapier2d/tests/game_path.cairo`, level 10, 30 ticks, force events on (exact Cairo steps): alpha.4 → alpha.5 →
RG1: `step_with_force_events` 2,723,719 → 2,764,257 (+1.49 %, SH2a's collect) → 2,720,701; with reads, despawn and 3
`WorldState` round trips 2,924,944 → 2,964,071 → 2,924,116. Remaining: `ShapeSerde` +676 steps per round trip (RG2),
the composite hook +2 steps per pair per step (≤ 0.009 % on P3 / levels, accepted).

## Compound shapes (SH2b #187, 2026-09-27)

Per warm step (Sierra gas | Cairo steps), a compound resting on a half-space / on a polyline: 2 parts 5.84M | 51.7k /
7.78M | 66.3k; 4 parts 9.86M | 86.8k / 13.25M | 113k; 8 parts 18.89M | 166k / 22.89M | 194k — each extra part ≈ 2.26M gas,
close to one cuboid on a half-space. `GameStep` class 577,435 → 600,420 CASM felts.

## Cost of a level (G0 #133, 2026-09-25)

Half-space, 10 pre-settled sleeping blocks (cuboids + 2 polygons) + 3 cores, pebble r = 0.25, density 4, (18, 4) m/s
from 7.2 m; level 20 adds a copy of the blocks. Whole runs include the load (one setup step without gravity, then
`sleep()`) and the despawn. Sierra gas / exact Cairo steps.

| level, setting | ticks | gas | steps | gas per tick avg / max | awake avg / max | all asleep (port / upstream) |
|---|---:|---:|---:|---|---|---|
| 10, 60 Hz × 4 | 300 | 24,058,098,795 | 203,657,538 | 79.3M / 101.2M | 12.3 / 14 | — / 278 |
| 10, 60 Hz × 2 | 300 | 15,515,727,742 | 130,498,852 | 50.9M / 70.8M | 12.0 / 14 | — / — |
| 10, 60 Hz × 1 | 300 | 12,265,353,729 | 102,643,616 | 40.1M / 50.9M | 11.6 / 14 | — / — |
| 10, 30 Hz × 4 | 150 | 9,576,253,056 | 79,472,837 | 62.6M / 100.2M | 9.8 / 14 | 130 / — |
| 20, 60 Hz × 4 | 150 (prefix) | 19,115,223,376 | 164,005,950 | 125.9M / 184.0M | 17.8 / 24 | — / — |
| 20, 60 Hz × 2 | 150 (prefix) | 13,811,914,419 | 117,046,734 | 90.8M / 130.8M | 18.2 / 24 | — / — |
| 20, 60 Hz × 1 | 300 | 22,795,671,297 | 183,601,385 | 75.2M / 99.1M | 20.0 / 24 | 286 / — |
| 20, 30 Hz × 4 | 150 | 22,458,221,991 | 193,814,539 | 148.2M / 182.3M | 20.9 / 24 | — / — |

CI windows (`crates/rapier2d/tests/level_budget.cairo`): load 88.3M / 722,788 steps (L10), 160.9M / 1,321,457 (L20);
flight tick 6.59M / 61,042 (L10), 10.99M / 102,182 (L20); impact tick 89.9M / 773,821 (L10), 97.5M / 845,534 (L20).
Awake body-tick at impact: 5.8M gas (× 4 substeps), 3.8M (× 2), 2.8M (× 1). All-asleep tick: 3.37M (L10), 8.14M (L20).
Stage shares, ticks 1–60 (L10 / L20): solver 76.9 / 68.9 %, narrow phase 16.5 / 14.7 %, broad phase 3.2 / 7.5 %, user
changes 2.0 / 3.1 %, islands + sleeping 1.3 / 5.7 %; flight ticks 1–25: broad phase 38 / 42 %, islands 31 / 31 %,
user changes 19 / 19 %, solver 9 / 7 %. Engine sleep timer 0.5 s; the programme's calm rule (all awake bodies below a
velocity threshold for 20 ticks) fires within 1–4 ticks of the engine where either fires, at 0.15M gas per tick.

## Since the 2026-09-24 matrix below

- SE #127 (sensors): every P3 scene ≤ +0.21 %; ceilings unchanged.
- RB #121 (rigid-body API, cold data boxed): contact and joint scenes +0.05 % to +0.29 % net; `free_fall1/8/32`
  −20.5 / −8.5 / −6.5 % net through a no-contact fast path (`pipeline/free_path.cairo`) that fires only when the world
  has no pair and no joint, so the free-fall probes no longer measure the general per-body path (+13 % net on
  `free_fall32` without the fast path); `gas_setup_*` (world construction) +3.7 %. Both are BT items (`docs/PLAN.md`).
  Exact steps after RB: `free_fall32` 535,449; `balls_halfspace32` 1,856,887; `cuboid_stack10` 754,143;
  `mixed_pile8` 810,808. The full matrix is refreshed with G0.
- WS #131 (world state save / restore, G0-like pile after one step, net; Sierra gas | Cairo steps): pile10 1,864 felts,
  round trip (`to_state` + `serialize` + `deserialize` + `from_state`) 6,621,940 | 51,974 (`into_state` instead of
  `to_state` saves ≈ 290k | 2.9k); pile20 4,896 felts, 14,451,140 | 114,814. `deserialize` is ≈ 70 % of the steps and
  the narrow-phase pairs ≈ 70 % of the felts (next lever: a hand-written `Serde` for the pairs).
- **BT1 #139** (split contact sweeps; results bit-identical, impact-window digests pinned): exact Cairo steps, impact
  window L10 3,869,103 → 2,283,360 (−41.0 %), L20 4,227,669 → 2,641,926 (−37.5 %); load windows −30.9 %; flight
  unchanged; P3 contact scenes −32 % to −46 % net steps (`cuboid_stack10` 335,761 → 209,830, `mixed_pile8`
  377,700 → 205,048, `balls_halfspace32` 833,669 → 477,791), Sierra gas −27 % to −44 % gross; ceilings lowered to
  +10 %. L10 impact tick 811,866 → 475,498 steps: narrow phase 111.7k (generation 78.9k, bookkeeping 32.8k),
  `solve_island` 290.6k (generation ≈ 47k + ≈ 54k per substep), glue 51.5k, other stages ≈ 21.6k. Per point per
  substep ≈ 4.2k → 2.6k steps.
- **BT2 #143** (persistent active set; results bit-identical, `WorldState` v2): exact Cairo steps, all-asleep engine step
  L10 / L20 36,141 / 64,511 → 1,000 / 1,000 (a sleeping body costs 0 steps per tick; 108k Sierra gas); flight tick
  incl. the probe's despawn scan 60,230 / 100,848 → 16,489 / 20,426 (an awake flying body ≈ 11.2k steps per tick with
  its live pair against the ground); impact windows −0.09 % / −0.47 %; load windows +4.2 % / +5.1 %; P3 `steps_step_*`
  +0.8 % to +3.7 % (≈ 85 steps per step from the sets' `modified` field → BT4). Activation reads: `World::body` 269
  steps, `is_sleeping` / `linvel` / `angvel` 146 / 150 / 146 (an arena read copies the whole body: BT4's field
  accessor).
- **BT3 #146** (narrow phase, constraint generation, per-substep solve; results bit-identical): exact Cairo steps,
  impact windows L10 / L20 2,278,494 / 2,640,861 → 1,849,193 / 2,211,560 (−18.8 % / −16.3 %); load −12.3 % / −13.0 %;
  flight −0.2 %; P3 contact scenes −5.5 % to −15.1 % gross (`cuboid_stack10` 511,800 → 434,585, `mixed_pile8`
  475,224 → 415,434), free fall and joints identical. L10 impact tick 476,178 → 369,855: narrow phase 101.3k,
  `solve_island` 194.4k (≈ 35.0k per substep), glue 52.2k. Per point per substep 2,202 → 1,473 steps; a non-touching
  half-space–cuboid pair 1,359 → 321. Since G0 (before BT1) the L10 impact tick went 811,866 → 369,855 (−54 %).
- **BT4 #151** (arena `modified` bit and field accessor, pipeline glue, mixed ticks; results bit-identical,
  `WorldState` still v2): impact windows L10 / L20 1,849,193 / 2,211,560 → 1,796,672 / 2,012,504 (−2.8 % / −9.0 %;
  L20's excess over L10 +19.6 % → +12.0 %); flight windows 470,522 / 612,278 → 447,011 / 587,357; load 450,067 /
  815,721 → 441,445 / 800,744; P3 `steps_step_*` −1.2 % to −3.8 % (BT2's +1–3 % recovered); activation reads
  `is_sleeping` / `linvel` / `angvel` 146 / 150 / 146 → 88 / 92 / 88 steps (the floor is the `Felt252Dict` access);
  `GameStep` class 435,454 → 440,868 CASM felts.

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
