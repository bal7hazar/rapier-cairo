# IT1 — the impact tick: where its Cairo steps go

Lot IT1, step 1 (measurement; brief `docs/briefs/it1-impact-tick.md`). Nothing in the engine changed: the
probes, the profiles and the scripts that produced the figures below are uncommitted (listed in §7).

**Every figure is exact Cairo steps** (`--tracked-resource cairo-steps`), measured on the Mac (Apple silicon arm64,
macOS 26.6.2, 64 GB), absolute build path
`/Users/bal7hazar/.herdr/worktrees/rapier-cairo/hp-slingfall-rapier-t-0008-it1-impact-tick-measure`, branch base
`main` @ `cc5f88c`, **Scarb 2.19.4 / snforge 0.61.0** (the toolchain `.tool-versions` pins before TC1), cairo-profiler
0.17.0, `RAYON_NUM_THREADS=1`, `--max-threads 2`. Steps are path-free; no hash, class size or Sierra byte count is
reported here. Estimates are called estimates.

## 1. The shots and their probes

The game's cost sheet (slingfall `docs/proving.md` "Cost sheet on alpha.8", lot B6; `fixtures/golden/cases.json`)
names two pile10 shots. Both are reproduced by `crates/rapier2d_classes/tests/pile10.cairo` (slingfall's `pile10.json`,
checked identical: poses, materials, settle, damage rule D6; the calm, bounds and scoring rules are not reproduced, so
a run is a fixed number of ticks):

| shot | pull | ticks (game) | proofs (game, alpha.8) | probe |
|---|---|---:|---:|---|
| **reference shot** | (-604, -392) | 107 | 3 | `pile10::{build, launch, tick}` with `REFERENCE`, uncommitted `tests/it1.cairo` (no committed probe has this pull) |
| **owner's shot** | (-1022, -63) | 151 | 6 | `pile10::run` (`PULL`), committed probes `steps_basic_*` / `steps_slim_*` |

Two layouts are measured: **in process** (`InProcess<BasicStepConfig>`, the executable / client path) and **slim**
(`Staged<BasicStepConfig, SlimSplitStages<PinnedHashes>>`, the declared-class layout the game proves on SNIP-36: the
proof count follows its steps). The `level_budget` L10 / L20 probes and `game_path` were not needed: both game shots
have a rapier-side probe.

### Steps per tick

Tick `t` is the `(t+1)`-th tick after the launch; its cost is `steps(t+1 ticks) − steps(t ticks)` of two otherwise
identical tests (`it1_<layout>_<shot>_<NNN>`, the shot built, launched and run `NNN` ticks, then a digest).

| | owner, in process | owner, slim | reference, in process | reference, slim |
|---|---:|---:|---:|---:|
| build + settle (+ class declarations, slim) | 291,140 | 488,727 | 291,140 | 488,727 |
| launch tick (tick 0) | 66,584 | | 66,584 | |
| flight tick (owner tick 30) | 11,844 | 26,357 | 11,844–11,932 | |
| flight ticks, all | 41 × 11,844 | | 81, 959,839 | |
| **impact tick** | **466,039** (tick 42) | **523,785** (tick 42) | **470,153** (tick 82) | **527,793** (tick 82) |
| a collapse tick | 193,163 (tick 50) | 273,472 (tick 50) | 286,462 (tick 90) | 380,927 (tick 90) |
| ticks after the impact | 108: 21,056,632 (avg 194,969) | 108: 28,454,528 (avg 263,468) | 24: 6,856,250 (avg 285,677) | 24: 9,132,421 (avg 380,518) |
| **whole shot** | **22,365,999** | **30,710,422** | **8,643,966** | **12,447,078** |
| impact tick / shot | 2.1 % | 1.7 % | 5.4 % | 4.2 % |
| ticks after the impact / shot | 94.1 % | 92.7 % | 79.3 % | 73.4 % |

The slim owner's shot (30,710,422) matches the plan's alpha.8 figure (30,773,277 − CS7's 61,741 = 30,711,536 within
1,114 steps; the difference is the probe's digest).

**What the impact tick does** (census, uncommitted `it1_census_*`, after the step): the pebble meets the sleeping pile,
**11 bodies awake, 30 contact pairs, 23 touching manifolds, 43 solver points, 8–9 force events, 3 bodies destroyed**
(damage rule, then three `World::remove_body`), identical on both shots. The owner's shot removes 2 more bodies at
tick 81; the reference shot none. After the impact, 8 then 6 bodies stay awake to the end of both runs (11–19 pairs,
15–30 points): the pile never falls asleep again within the shot (G0's finding), so the game ends the shot by its calm /
spent-pebble rules.

**Headline: the impact tick is 1.7–5.4 % of a shot; the ticks after it are 73–94 %.** A lever that only acts at the
impact moves the proof count by at most a few hundred thousand steps; a lever that acts on every awake contact tick is
multiplied by 108 (owner) or 24 (reference).

## 2. Profile of the impact tick, per stage

Command (each probe pair; the profile of `NNN+1` ticks minus that of `NNN` ticks):

```sh
export RAYON_NUM_THREADS=1 PATH=$HOME/.asdf/installs/cairo-profiler/0.17.0/bin:$PATH
snforge test -p rapier2d_classes it1::it1_basic_owner_042 --include-ignored --tracked-resource cairo-steps \
    --build-profile --max-threads 2          # and _043; slim_*, reference_* likewise
python3 stages.py <profile of 042>.pb.gz <profile of 043>.pb.gz   # per-stage table (§7)
go tool pprof -sample_index=steps -top -diff_base=<042>.pb.gz <043>.pb.gz        # top functions
```

The build had `unstable-add-statements-{functions,code-locations}-debug-info = true` under `[profile.dev.cairo]`
(root `Scarb.toml`, uncommitted): debug info only, the steps are identical (owner 042 / 043: 843,328 / 1,309,367 with
and without). Functions are Sierra functions (`--show-inlined-functions` misattributes: it put the whole solver,
270k, on an inlined frame whose callee carries 95 steps), so an inlined callee counts in its caller. Stage tables sum
exactly to the tick.

| stage (impact tick) | owner, in process | % | owner, slim | reference, in process | reference, slim |
|---|---:|---:|---:|---:|---:|
| solver: stabilization sweeps (stage 2, refresh + relax, 4 substeps) | 94,041 | 20.2 | 94,041 | 94,914 | 94,914 |
| solver: update / warm-start sweeps (stages 5 / 0) | 52,481 | 11.3 | 52,481 | 53,261 | 53,261 |
| solver: PGS sweeps (stage 1) | 42,260 | 9.1 | 42,260 | 42,691 | 42,691 |
| solver: constraint generation | 46,826 | 10.0 | 46,826 | 46,986 | 46,986 |
| solver: bodies (forces, integrate, damp, finish) | 17,431 | 3.7 | 17,431 | 17,431 | 17,431 |
| solver: final writeback (stage 4) + `impulses_of` | 5,952 | 1.3 | 5,952 | 5,952 | 5,952 |
| solve glue (gather, advance, write bodies, mass, body status) | 42,855 | 9.2 | 35,362 | 44,683 | 37,115 |
| solve glue: `scatter_impulses` | 12,927 | 2.8 | (in the crossing) | 12,927 | (in the crossing) |
| `remove_body` × 3 (wake partners 40,976, release pairs 25,896) | 71,988 | 15.4 | 71,988 | 71,772 | 71,772 |
| islands | 20,365 | 4.4 | 12,888 | 20,400 | 12,923 |
| force events (`collect_body`) | 20,107 | 4.3 | 20,107 | 20,330 | 20,330 |
| sleeping / dormant pairs (`split_dormant`, `merge_pairs`, wakes) | 14,620 | 3.1 | 14,620 | 15,018 | 15,018 |
| active set (`split_at_positions`, statics) | 6,239 | 1.3 | 6,239 | 6,239 | 6,239 |
| narrow phase | 3,395 | 0.7 | 6,274 | 3,101 | 5,977 |
| broad phase | 1,367 | 0.3 | 1,491 | 1,442 | 1,566 |
| class crossings (codec, library calls): advance | — | | 57,643 | — | 57,643 |
| class crossings: islands | — | | 12,880 | — | 12,880 |
| class crossings: narrow phase | — | | 9,384 | — | 9,356 |
| class crossings: broad phase | — | | 2,760 | — | 2,760 |
| game: damage rule (probe) | 6,260 | 1.3 | 6,260 | 6,099 | 6,099 |
| other (step glue, probe loop) | 6,925 | 1.5 | 6,898 | 6,907 | 6,880 |
| **total** | **466,039** | 100 | **523,785** | **470,153** | **527,793** |

Grouped: **solver 259,000 (55.6 %)**, solve glue 55.8k (12.0 %), `remove_body` 72.0k (15.4 %), the rest of the step
(islands, force events, dormant pairs, active set, narrow and broad phase) 66.1k (14.2 %), probe 13.2k. The narrow
phase is almost free at the impact (3.4k): the pile's pairs come back from the dormant list with their manifolds, and
only the pebble's pair is generated; the solver then solves the whole woken pile (43 points). The slim layout adds
**+57,746** steps (owner) / **+57,640** (reference) to the impact tick, 82.7k of crossings less what leaves the caller.

### Top functions (owner, in process; flat, a Sierra function with what is inlined in it)

| function | steps | role |
|---|---:|---|
| `solver::island::sweeps::split::banked` | 148,895 | stages 5 / 0 / 2 / 4 sweep loop and kernels |
| `sweeps::split::generation::generate` | 45,636 | constraint generation (2.0k per manifold) |
| `sweeps::split::sweep` | 41,618 | stage 1 sweep loop and kernels |
| `core::array::SpanIterator::next` | 27,895 | ≈ 12 walks of the 30-pair `narrow_phase.pairs`: wake ×3 and release ×3 (6,513 each), `link_pairs`, solve, `scatter_impulses`, `split_at_positions`, `split_dormant` (≈ 2.1–2.2k each) |
| `sleeping::release_removed_pairs` | 19,383 (25,896 cum) | one full copy of the pair list per removal |
| `sleeping::wake_partners` | 27,080 (40,976 cum) | one more walk of the pair list per removal |
| `Felt252Dict::squash` | 14,936 | solver velocity dictionary 9,510, islands 1,956 |
| `Arena::get` | 13,596 | force events 5,940 (two collider reads per pair), `wake_parent` 5,552 |
| `force_events::collect_body` | 12,896 (20,044 cum) | |
| `ordering::scatter_impulses` | 8,462 (12,927 cum) | |

Per point and substep, the three sweeps cost (94,041 + 52,481 + 42,260 + 3,175) / (43 × 4) = **1,116 steps** (≈ 370
per point per sweep), generation 2,036 per manifold: BT3's floor (≈ 17 steps per fixed-point rescale) is what is left.

## 3. The ticks around it, for scale

| stage | flight (owner tick 30), in process | flight, slim | collapse (owner tick 50), in process | collapse, slim | collapse (reference tick 90), in process | collapse, slim |
|---|---:|---:|---:|---:|---:|---:|
| solver sweeps (stages 0–5) | — | — | 61,826 | 61,826 | 133,041 | 133,041 |
| constraint generation | — | — | 17,162 | 17,162 | 30,895 | 30,895 |
| solver bodies + `impulses_of` | — | — | 12,416 | 12,416 | 13,193 | 13,193 |
| solve glue | 3,520 | 3,063 | 34,099 | 21,375 | 40,644 | 27,776 |
| narrow phase | 1,685 | 3,960 | 43,296 | 62,130 | 43,055 | 66,040 |
| force events | 682 | 682 | 9,707 | 9,707 | 11,653 | 11,653 |
| user changes (dense step) | — | — | 6,431 | 6,446 | 6,431 | 6,446 |
| broad phase | 1,353 | 1,477 | 2,295 | 2,381 | 2,454 | 2,540 |
| active set / islands | 1,525 | 1,525 | — | 32 | — | 32 |
| class crossings (all) | — | 12,572 | — | 74,074 | — | 84,223 |
| other (step glue, probe) | 3,079 | 3,078 | 5,931 | 5,923 | 5,096 | 5,088 |
| **total** | **11,844** | **26,357** | **193,163** | **273,472** | **286,462** | **380,927** |

After the impact every body of the pile is awake, so the step takes the dense whole-step path (`active_set::usable` is
false), with its full user-change scan (6.4k). The narrow phase is now the second stage (43k for 15–19 pairs; the pair
loop's own glue `compute_contacts_from_scratch_with[930-4612]` is 22.5k flat of it, the generators the rest). On the
slim layout the collapse tick pays **+80,309** (owner) / **+94,465** (reference): crossings 74–84k (narrow phase 39–42k,
advance 32–39k) and `NarrowPhaseClass`'s own contact generation 62–66k instead of 43k in process.

## 4. The levers, ranked

Ranked by steps saved per shot = per tick × ticks of the shot (owner: 1 impact tick, 108 ticks after it, 5 removals;
reference: 1 / 24 / 3). **Every saving is an estimate** from the profiles above unless stated; "bit-identical by
construction" means the same arithmetic in the same order on the same values (only copies, walks or reads removed).

| # | lever | mechanism | impact tick (est.) | owner shot (est.) | reference shot (est.) | bit-identical | files | risk | in IT1 step 2 scope |
|---|---|---|---:|---:|---:|---|---|---|---|
| X1 | **class crossings of the slim layout** | the slim shot pays +8,344,423 (owner, +37.3 %) / +3,803,112 (reference, +44.0 %) over in process, measured: crossings 74–84k per collapse tick, 82.7k at the impact, 12.6k per flight tick; `NarrowPhaseClass` generation +19–23k per collapse tick. A CX3-like lot (awake-only narrow-phase wire, advance write-back) | −20 to −40k | −2 to −4M | −0.6 to −1.2M | yes (codec only) | `crates/rapier2d_classes/src/**` | medium (caller size, 73,728 gate) | **no** (classes are out of scope) |
| N1 | narrow-phase pair loop glue | per pair, `*found.unbox().manifold` copied, `solver_data_supported` rebuilt, `ContactPair` appended: 22.5k flat over 15–19 pairs per collapse tick (inlined dispatch code included, to attribute with an `inlining-strategy = "avoid"` build first) | ≈ 0 (3.4k narrow phase) | −0.45 to −0.95M (−4 to −9k × 108) | −0.1 to −0.2M | yes, if the manifold operations are unchanged | `rapier_dynamics2d/src/narrow_phase.cairo` | medium | yes (in process; the slim layout only if `NarrowPhaseClass` calls it) |
| F1 | force events without whole-collider reads | `collect_body` reads both colliders whole (`Arena::get`, 5.9k at impact) and rebuilds the pair list for every pair; read the flags / threshold through a field accessor or the step's `PairCollider` scratch, rebuild only pairs whose status bit changes | −6 to −10k | −0.35 to −0.55M (−3 to −5k × 108) | −0.08 to −0.13M | yes (same values read, same bits written) | `rapier2d/src/pipeline/force_events.cairo` (+ a `ColliderSet` accessor in `rapier_dynamics2d`) | low | yes |
| S1 | solver sweep data movement | `banked` / `sweep` pop and re-append `Hot` (10 felts) and `Bank` (8 felts) of every constraint on each of 13 calls per step; a tighter state layout or fewer rebuilds | −8 to −12k (≈ 5 % of 192k) | −0.3 to −0.4M | −0.15 to −0.2M | yes if the kernels' arithmetic is untouched | `rapier_dynamics2d/src/solver/island/sweeps/split.cairo` | medium (BT1 / BT3 benched most variants; losers under `alternatives`) | yes |
| W1 | fewer walks of `narrow_phase.pairs` in the step glue | ≈ 12 walks per impact tick (`SpanIterator::next` 27.9k), `scatter_impulses` 12.9k, `split_dormant` 9.9k, `split_at_positions` 5.0k, `link_pairs` 7.5k: fuse the scatter into the solve's write-back, the dormant split into the active-set split | −8 to −15k | −0.25 to −0.45M (−2 to −4k × 108) | −0.06 to −0.1M | yes if the write order is kept | `rapier2d/src/pipeline/{fused,ordering,sleeping,active_set}.cairo` | medium | yes |
| U1 | dense step's user-change scan on all-awake ticks | 6.4k per collapse tick (`Arena::to_array` 3.3k, census, body infos) although nothing changed between steps | 0 | −0.2 to −0.3M (−2 to −3k × 108) | −0.05 to −0.07M | yes | `rapier2d/src/pipeline.cairo`, `pipeline/user_changes.cairo` | low–medium | yes |
| R1 | `remove_body` in one walk | `wake_contact_partners` walks the pair list twice per removal (`wake_partners` 13.7k, `release_removed_pairs` 8.6k per removal): one fused walk that wakes and releases | **−25 to −33k** (−5 to −7 % of the impact tick) | −0.05M (5 removals) | −0.03M (3 removals) | yes (body writes and pair rewrites are independent and keep their order) | `rapier2d/src/world.cairo`, `rapier2d/src/pipeline/sleeping.cairo` | low | yes |
| G1 | constraint generation | 2.0k per manifold (46.8k at impact, 17–31k per collapse tick); BT3 floor | −2 to −3k | −0.1M | −0.03M | depends | `sweeps/split/generation.cairo` | medium | yes |
| — | dormant pairs out of `narrow_phase.pairs` | removes most walks and copies of sleeping pairs (L20's mixed-tick excess) | — | — | — | yes | `WorldState` v3 | — | **no** (codec change, parked) |
| — | a pile that sleeps again | 6–8 bodies stay awake to the end of both shots; every collapse tick is 190–380k | — | the bulk of the shot | | **no** (sleep thresholds change results) | engine parameters / game rules | — | **no** (numeric; the game's calm rule) |
| — | (c) solver-graph order, (A) scalar API, (B) composites, (E) sub-shapes, (F) `contact_skin`, (e) `core_witness`, (f) sweep normal | parked by `docs/PLAN.md`; none shows in these profiles as a step cost of the reference shots | — | — | — | — | — | — | **no** (parked) |

**In-scope levers together (N1, F1, S1, W1, U1, R1, G1), estimate:** the impact tick −50 to −75k (−11 to −16 %); the
owner's shot −1.7 to −2.8M in process (−7.6 to −12.5 %), the same absolute amount on the slim shot (−5.5 to −9.1 % of
30.7M) wherever the lever runs in the caller (F1, W1, U1, R1, and N1 / S1 / G1 only if the classes call the same
functions); the reference shot −0.5 to −0.75M.

**Proof count (estimate, from the cost sheet).** The game packs chunks into proofs of ≤ 1.0e9 virtual L2 gas at about
150 L2 gas per step on heavy ticks: the owner's shot is 5.445e9 in 6 proofs, the reference shot 2.645e9 in 3. Dropping
the owner's shot to 5 proofs needs ≈ −8 % of its virtual L2 gas (≈ −2.9M steps at 150 gas per step), and chunk
boundaries make it non-linear: the in-scope levers reach it only at their upper estimate. The reference shot needs
≈ −24 % for 2 proofs: out of reach of the in-scope levers. X1 (crossings) is the lever that can move a proof; it is
outside IT1's step-2 allowlist.

## 5. What the profile rules out

- The narrow phase is **not** a lever at the impact tick (3.4k: dormant manifolds are reused); BT3's "narrow phase ≈ 101k
  per impact tick" was the G0 level, whose pebble wakes a pile that has to regenerate its pairs.
- Six `remove_body` ≈ 90k (alpha.1) are now three at 72k on the reference shots' impact tick.
- The solver kernels themselves (fused wide sums, skipped exact zeros, unit warm start) are at BT3's floor: what is left
  in the sweeps is data movement (S1), not arithmetic.

## 6. Reproduce

```sh
export RAYON_NUM_THREADS=1
# per-tick curve (192 tests, ~100 s on the Mac)
snforge test -p rapier2d_classes it1_basic_ --include-ignored --tracked-resource cairo-steps --detailed-resources \
    --max-threads 2
# census of the impact
snforge test -p rapier2d_classes it1_census --include-ignored --tracked-resource cairo-steps --max-threads 2
# whole shots, one at a time
snforge test -p rapier2d_classes it1::it1_slim_owner_151 --include-ignored --tracked-resource cairo-steps \
    --detailed-resources --max-threads 2
```

## 7. Measurement material (uncommitted)

- `crates/rapier2d_classes/tests/it1.cairo` (+ `mod it1;` in `tests/lib.cairo`): `it1_basic_{owner,reference}_NNN`
  (owner 0–80, reference 0–110), `it1_slim_*` pairs around ticks 30, 42, 50, 82, 90, the whole shots, `it1_census_*`.
- Root `Scarb.toml`: `[profile.dev.cairo] unstable-add-statements-functions-debug-info = true`,
  `unstable-add-statements-code-locations-debug-info = true` (debug info; steps unchanged).
- `stages.py`: per-stage table of two cairo-profiler profiles (`go tool pprof -raw`, first match from the root of each
  sampled stack; the solver refined by sweep stage; codec frames under a library call counted as crossing).
- cairo-profiler 0.17.0 (asdf, user-local, already installed), `go tool pprof`.
