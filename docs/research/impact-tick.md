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

**Against the earlier impact figures.** BUDGETS ("The game on 0.1.0-alpha.6", CS2) records a pile10 impact *step* of
391,314 steps. The step alone of this impact tick is 466,039 − 71,988 (`remove_body` × 3, after the step) − 13,185 (the
probe's damage rule and loop) = **380,866**, i.e. −10,448 since alpha.6, in line with the CX / CS7 / DU1 lots since
(step path untouched, bit-identical). PLAN's "impact tick 1.02M (six `remove_body` ≈ 90k)" and the game's 652k (alpha.2)
/ 557k (alpha.3) are game-side ticks (rules included) measured before BT1–BT4 or before CS2, on the game's own
reference shot of the time: not comparable with these figures.

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
| solver: final restitution pass (stage 4: 3,175) + impulse write-back `impulses_of` (2,777) | 5,952 | 1.3 | 5,952 | 5,952 | 5,952 |
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
| `solver::island::sweeps::split::banked` | 148,895 | stages 5 / 0 / 2 and the final stage-4 (restitution) pass: loop and kernels |
| `sweeps::split::generation::generate` | 45,636 | constraint generation (2.0k per manifold) |
| `sweeps::split::sweep` | 41,618 | stage 1 sweep loop and kernels |
| `core::array::SpanIterator::next` | 27,895 | ≈ 12 walks of the 30-pair `narrow_phase.pairs`: wake ×3 and release ×3 (6,513 each), `link_pairs`, solve, `scatter_impulses`, `split_at_positions`, `split_dormant` (≈ 2.1–2.2k each) |
| `sleeping::release_removed_pairs` | 19,383 (25,896 cum) | one full copy of the pair list per removal |
| `sleeping::wake_partners` | 27,080 (40,976 cum) | one more walk of the pair list per removal |
| `Felt252Dict::squash` | 14,936 | solver velocity dictionary 9,510, islands 1,956 |
| `Arena::get` | 13,596 | force events 5,940 (two collider reads per pair), `wake_parent` 5,552 |
| `force_events::collect_body` | 12,896 (20,044 cum) | |
| `ordering::scatter_impulses` | 8,462 (12,927 cum) | |

Per point and substep, the sweeps cost (94,041 stage 2 + 52,481 stages 5 / 0 + 42,260 stage 1 + 3,175 for the one
stage-4 restitution pass after the last substep) / (43 × 4) = **1,116 steps** (≈ 370 per point per sweep). This
**excludes** constraint generation (2,036 per manifold) and the bodies; the whole solver of the tick, the quantity of
BT3's "per point per substep 1,473" (L10 level impact tick, `solve_island`), is 258,991 / (43 × 4) = **1,506**, on
another scene (43 points of an 11-body pile against BT3's L10 pile). BT3's floor (≈ 17 steps per fixed-point rescale)
is what is left in the kernels.

## 3. The ticks around it, for scale

| stage | flight (owner tick 30), in process | flight, slim | collapse (owner tick 50), in process | collapse, slim | collapse (reference tick 90), in process | collapse, slim |
|---|---:|---:|---:|---:|---:|---:|
| solver sweeps (stages 0, 1, 2, 5 and the stage-4 pass) | — | — | 61,826 | 61,826 | 133,041 | 133,041 |
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
| N1 | narrow-phase pair loop glue | per pair, `*found.unbox().manifold` copied, `solver_data_supported` rebuilt, `ContactPair` appended: 22.5k flat over 15–19 pairs per collapse tick (inlined dispatch code included, to attribute with an `inlining-strategy = "avoid"` build first) | ≈ 0 (3.4k narrow phase) | −0.45 to −0.95M (−4 to −9k × 108) | −0.1 to −0.2M | yes, if the manifold operations are unchanged | `rapier_dynamics2d/src/narrow_phase.cairo` | medium | yes, **in process only** (`NarrowPhaseClass` runs `stages::narrow::compute_contacts_with_results`, not this function) |
| F1 | force events without whole-collider reads | `collect_body` reads both colliders whole (`Arena::get`, 5.9k at impact) and rebuilds the pair list for every pair; read the flags / threshold through a field accessor or the step's `PairCollider` scratch, rebuild only pairs whose status bit changes | −6 to −10k | −0.35 to −0.55M (−3 to −5k × 108) | −0.08 to −0.13M | yes (same values read, same bits written) | `rapier2d/src/pipeline/force_events.cairo` (+ a `ColliderSet` accessor in `rapier_dynamics2d`) | low | yes |
| S1 | solver sweep data movement | `banked` / `sweep` pop and re-append `Hot` (10 felts) and `Bank` (8 felts) of every constraint on each of 13 calls per step; a tighter state layout or fewer rebuilds | −8 to −12k (≈ 5 % of 192k) | −0.3 to −0.4M | −0.15 to −0.2M | yes if the kernels' arithmetic is untouched | `rapier_dynamics2d/src/solver/island/sweeps/split.cairo` | medium (BT1 / BT3 benched most variants; losers under `alternatives`) | yes |
| W1 | fewer walks of `narrow_phase.pairs` in the step glue | ≈ 12 walks per impact tick (`SpanIterator::next` 27.9k), `scatter_impulses` 12.9k, `split_dormant` 9.9k, `split_at_positions` 5.0k, `link_pairs` 7.5k: fuse the scatter into the solve's write-back, the dormant split into the active-set split | −8 to −15k | −0.25 to −0.45M (−2 to −4k × 108) | −0.06 to −0.1M | yes if the write order is kept | `rapier2d/src/pipeline/{fused,ordering,sleeping,active_set}.cairo` | medium | yes |
| U1 | dense step's user-change scan on all-awake ticks | 6.4k per collapse tick (`Arena::to_array` 3.3k, census, body infos) although nothing changed between steps | 0 | −0.2 to −0.3M (−2 to −3k × 108) | −0.05 to −0.07M | yes | `rapier2d/src/pipeline.cairo`, `pipeline/user_changes.cairo` | low–medium | yes |
| R1 | `remove_body` in one walk | `wake_contact_partners` walks the pair list twice per removal; one fused walk that wakes and releases. Retained: `release_removed_pairs` whole (25,896: the copy and rebuild of every pair and its `== removed` test) and `wake_parent` (7,383: the reads and writes of the woken bodies). Removed: the rest of `wake_partners`, 33,593 for 3 removals (its walk 6,513, its inner loop over `touched` 8,014, its own per-pair code 19,066). Lower bound: only the walk and the inner loop go (14.5k), if the 19k of per-pair code (≈ 212 steps per pair visit, not attributed further) has to be repeated in the fused loop; upper bound: all of it, since the fused loop's involvement test is release's existing `== removed` test | **−15 to −34k** (−3 to −7 % of the impact tick) | −0.03 to −0.06M (5 removals, ≈ 5–11k each) | −0.015 to −0.034M (3 removals) | yes (body writes and pair rewrites are independent and keep their order) | `rapier2d/src/world.cairo`, `rapier2d/src/pipeline/sleeping.cairo` | low | yes |
| G1 | constraint generation | 2.0k per manifold (46.8k at impact, 17–31k per collapse tick); BT3 floor | −2 to −3k | −0.1M | −0.03M | depends | `sweeps/split/generation.cairo` | medium | yes |
| — | dormant pairs out of `narrow_phase.pairs` | removes most walks and copies of sleeping pairs (L20's mixed-tick excess) | — | — | — | yes | `WorldState` v3 | — | **no** (codec change, parked) |
| — | a pile that sleeps again | 6–8 bodies stay awake to the end of both shots; every collapse tick is 190–380k | — | the bulk of the shot | | **no** (sleep thresholds change results) | engine parameters / game rules | — | **no** (numeric; the game's calm rule) |
| — | (c) solver-graph order, (A) scalar API, (B) composites, (E) sub-shapes, (F) `contact_skin`, (e) `core_witness`, (f) sweep normal | parked by `docs/PLAN.md`; none shows in these profiles as a step cost of the reference shots | — | — | — | — | — | — | **no** (parked) |

**In-scope levers together, estimate.** In process (N1, F1, S1, W1, U1, R1, G1): the impact tick −39 to −74k (−8 to
−16 %), the owner's shot −1.7 to −2.8M (−7.5 to −12.5 % of 22.37M), the reference shot −0.5 to −0.75M.

**On the slim layout (what the game proves), N1 does not apply:** `NarrowPhaseClass` runs
`rapier2d::pipeline::stages::narrow::compute_contacts_with_results` / `contact_jobs` (owner tick 50 profile), not
`rapier_dynamics2d::narrow_phase::compute_contacts_from_scratch_with`. S1 and G1 do apply (the solver rows of the slim
profiles are the same `rapier_dynamics2d::solver` functions with the same steps, run inside `SolveAdvanceClass`; the
class is rebuilt from them), F1, U1 and R1 run in the caller, W1 only partly (its `scatter_impulses` part runs in the
advance class's write-back on the slim layout). Without N1: **owner's slim shot −1.2 to −1.85M (−4.0 to −6.0 % of
30.71M)**, reference slim shot −0.4 to −0.55M (−3.1 to −4.5 % of 12.45M).

**Proof count (estimate, from the cost sheet).** Source: slingfall `docs/proving.md` "Cost sheet on alpha.8" (lot B6,
`origin/main` `d0d0eaa`, read in a local clone): the game packs chunks into proofs of ≤ 1.0e9 virtual L2 gas; the
owner's shot is 5,445,013,440 virtual L2 gas in 6 proofs for **35.29M snforge steps of the game's chunk** (rapier's
slim step plus the game's rules, codec round trips and bindings: more than rapier's 30.71M here), the reference shot
2.645e9 in 3 proofs (the sheet gives no step count for it). **Basis: 5.445e9 / 35.29M = 154 L2 gas per step**, the
whole-shot average of the game's own figures, consistent with the sheet's "about 150 L2 gas per step on the heavy
ticks" (W3 measured ≈ 111 on a light chunk). Dividing by rapier's 30.71M instead (177) would charge the game's own
steps to the engine.

Five proofs for the owner's shot need ≤ 5.0e9, i.e. ≥ −0.445e9 = **≥ −2.89M steps** at 154 (a lower bound: chunk
boundaries must also fall right). **The in-scope levers do not reach 5 proofs on their own numbers:** −1.2 to −1.85M on
the slim shot without N1 (−2.8M even with N1, which does not apply there) is short of −2.89M. The reference shot needs
≈ −24 % (−0.645e9, ≈ −4.2M steps at 154) for 2 proofs: out of reach. The in-scope levers lower the steps and the L2 gas
of every shot (the proofs' gas cost, the client's run, the local proofs), not the proof count. X1 (crossings, −2 to
−4M estimate) is the only lever of this list that can move a proof, alone or with the in-scope ones; it is outside
IT1's step-2 allowlist.

## 5. What the profile rules out

- The narrow phase is **not** a lever at the impact tick (3.4k: dormant manifolds are reused); BT3's "narrow phase ≈ 101k
  per impact tick" was the G0 level, whose pebble wakes a pile that has to regenerate its pairs.
- Removals got **dearer per body**, not cheaper: PLAN's six `remove_body` ≈ 90k (alpha.1, the game's reference shot
  of the time, pre-BT) are 15k per removal; here three cost 71,988, **24.0k per removal**. Each removal walks the whole
  pair list twice and copies it once (`wake_partners`, `release_removed_pairs`), and since BT2 the list keeps the
  sleeping pile's dormant pairs (30 pairs at the impact), so the cost grows with the pile, not with the removed body's
  own contacts. This supports R1 (and the parked "dormant pairs out of `narrow_phase.pairs`").
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

Kept uncommitted in the lot's worktree (`hp/slingfall-rapier/t-0008-it1-impact-tick-measure`) for step 2, which
recommits the probes it needs for its before / after tables:


- `crates/rapier2d_classes/tests/it1.cairo` (+ `mod it1;` in `tests/lib.cairo`): `it1_basic_{owner,reference}_NNN`
  (owner 0–80, reference 0–110), `it1_slim_*` pairs around ticks 30, 42, 50, 82, 90, the whole shots, `it1_census_*`.
- Root `Scarb.toml`: `[profile.dev.cairo] unstable-add-statements-functions-debug-info = true`,
  `unstable-add-statements-code-locations-debug-info = true` (debug info; steps unchanged).
- `stages.py`: per-stage table of two cairo-profiler profiles (`go tool pprof -raw`, first match from the root of each
  sampled stack; the solver refined by sweep stage; codec frames under a library call counted as crossing).
- cairo-profiler 0.17.0 (asdf, user-local, already installed), `go tool pprof`.

## 8. EL1 — the in-scope levers, measured (2026-10-03)

Lot EL1 (`docs/briefs/el1-engine-levers.md`). Exact Cairo steps, Scarb 2.20.1 / snforge 0.64.0, on the Mac (Apple
silicon arm64), build path `/Users/bal7hazar/.herdr/worktrees/rapier-cairo/hp-slingfall-rapier-t-0035-el1-engine-levers`,
`RAYON_NUM_THREADS=1`, before = `main` after CX3 and CI2 (`665e999`). Probes: CX3's `cx3.cairo` copied as an uncommitted
`el1.cairo` (the pile10 shots built, launched, run `NNN` ticks, digest), plus two uncommitted probes that print the
digest of every tick of both shots in both layouts. Every lever's per-tick digests are equal to the before run's (520
lines), and every result test passes unchanged.

| lever | estimate (owner's shot, IT1 §4) | measured: owner's shot, in process / slim | reference shot, in process / slim | kept |
|---|---|--:|--:|---|
| R1 `remove_body` in one walk (`sleeping::wake_and_release_removed`) | −0.03 to −0.06M | −31,289 / −31,289 | −23,461 / −23,461 | yes |
| F1 force events: pair list kept when no status bit changes | −0.35 to −0.55M (F1 as a whole) | −207,332 / −207,332 | −89,141 / −89,141 | yes |
| F1 force events: three collider fields read (`ColliderSetTrait::get_field`) | (in the line above) | −107,744 / −107,744 | −44,104 / −44,104 | yes |
| W1 `body_status` reads the slot's entry before walking | −0.25 to −0.45M (W1 as a whole) | −14,142 / −2,158 | −10,290 / −2,157 | yes |
| N1 pair loop: one unbox of the previous pair | −0.45 to −0.95M (in process) | 0 / 0 | 0 / 0 | no (the compiler already shares it) |
| S1, U1, G1, the rest of N1 and W1 | see below | not implemented | | |

The impact tick loses 28,421–28,425 steps in process (−6.0 %) and 27,446–27,450 slim (−5.2 %). Each tick after the
impact loses 2.8–4.6k (−1.1 to −1.6 %), which is about ¼ of IT1's estimate (−8 to −16 k per collapse tick for all
in-scope levers). Why the rest was not implemented (reference shot, in process, from the HP profile below):

- **S1.** Of the 2,605,823 steps of `split::banked`, the arrays' own movement is `array_append` 61,776 and
  `array_pop_front` 13,728. The rest is the kernels' arithmetic and its `store_temp` (1,365,229). Rebuilding `Hot` /
  `Bank` is inherent to immutable arrays, and BT1 / BT3 benched the layouts (`split/alternatives.cairo`). No
  bit-identical candidate is left.
- **N1.** The pair loop costs 1,364,656: generators 679,476, `solver_data_supported` 236,601 (combine rules 60,576,
  in `rapier_core`, out of scope), `append` 72,704, `SpanImpl::at` 56,800 (the two `PairCollider` copies, which the
  callees take by value). Sharing the unbox changed nothing. N1 does not apply to the slim layout.
- **W1.** `scatter_impulses` (224,063) rebuilds the pair list after the solve, and the force-event pass walks it
  again. Fusing them would cross the boundary between `SolveAdvanceClass` and the force-event stage in the slim
  layout (classes are out of scope). The `body_status` part above is the part found inside the allowed files.
- **U1.** `user_changes_bodies_for_step` costs 229,461 over the shot. 98,320 of it is the two set snapshots
  (`Arena::to_array`) the rest of the step reads, and 84,621 is the body loop that builds them. Skipping the
  change-flag scan when the arena is unmodified is not safe: a `WorldState` round trip (which the game does every
  tick) restores an unmodified set whose flags may be raised.
- **G1.** Generation (789,915) is BT3's direct split; no step was found to remove without changing an operation.

Proofs (estimate, the game's basis of 154 L2 gas per step, CX3's figures after TC1): owner's shot 5.28e9 − 348,523 ×
154 = 5.226e9, still 6 proofs (5 need ≈ 1.47M more steps); reference shot 2.57e9 − 158,863 × 154 = 2.546e9, still 3.

## 9. HP — hot-path profile of the `fixed` and `glam_core` operations (2026-10-03)

For the glam track: the operations of the `fixed` and `glam_core` dependencies ranked by their Cairo steps.

**How.** The setup is that of §8, at commit `2a80232` (main + R1; R1 changes no `fixed` / `glam_core` call). The
tests were run with `--save-trace-data` on a build with `[profile.dev.cairo]
unstable-add-statements-functions-debug-info = true` and `unstable-add-statements-code-locations-debug-info = true`
(uncommitted; I checked that the steps are identical with and without them). The profiles come from
**cairo-profiler 0.17.0** with `--show-inlined-functions --show-libfuncs --max-function-stack-trace-depth 1000`, each
on a frozen copy of the Sierra file the trace was made with.

- **Stack depth.** The default depth (100) truncates the deep stacks of the game and shot probes, and their steps
  land on outer frames: the reference shot showed 1.4 % `fixed` + `glam_core` at depth 100 against 38.8 % at
  depth 1000.
- **Inlined code.** Every `fixed` / `glam_core` function is `#[inline(always)]`, so a step goes to the inlined
  frames of its statement. A row is the **outermost** `fixed::` / `glam_core::` frame, i.e. the operation as rapier
  calls it, inclusive of what it inlines.
- **Call counts.** Exact, but derived: cairo-profiler counts only contract calls, and cairo-coverage 0.6.1's line
  hits are not call counts. Each operation is counted through a libfunc of fixed CASM cost that it runs a known
  number of times per call: `bounded_int_div_rem` inside `narrow32` (7 steps, once per rescale; twice for a `Vec2`
  component-wise product), `u128_sqrt` (9), or `i64_overflowing_add` / `sub` (6). A libfunc's cost is the gcd of its
  steps over all stacks. Comparisons (`gt`, `eq`) have no such libfunc: "—".
- **Coverage gaps.** The owner's shot (22.8M steps) could not be profiled on the Mac: the profiler needs about 41 GB
  for the reference shot and swapped on the owner's. The slim shots could not be profiled either: on snforge 0.64's
  traces, cairo-profiler 0.17.0 stops with "Failed to map function to SyscallSelector … VariantNotFound". The classes
  run the same engine functions, so the in-process reference shot stands for the pile10 family.

| family (probes) | Cairo steps | `fixed` + `glam_core` | of which `narrow32` (the Q64.64 → Q32.32 rescale) |
|---|--:|--:|--:|
| P3 `steps_step_*` (17) | 4,401,279 | 1,171,569 (26.6 %) | 588,294 (13.4 %) |
| levels `steps_{flight,impact,load}_level{10,20}` (6) | 9,754,110 | 2,463,439 (25.3 %) | 1,291,117 (13.2 %) |
| game `game_path::steps_game_*` (12) | 31,429,657 | 9,027,677 (28.7 %) | 4,809,742 (15.3 %) |
| pile10 reference shot, in process (1) | 8,728,969 | 3,387,651 (38.8 %) | 2,003,956 (23.0 %) |
| **total** (36) | **54,314,015** | **16,050,336 (29.6 %)** | **8,693,109 (16.0 %)** |

Top 10, all families together (each operation's steps per call is close to its per-family value):

| # | operation | calls | steps / call | steps | share |
|---|---|--:|--:|--:|--:|
| 1 | `fixed::wide::mul_add` | 201,841 | 15.2 | 3,067,293 | 5.65 % |
| 2 | `fixed::fixed::FixedMul::mul` | 106,849 | 13.8 | 1,469,881 | 2.71 % |
| 3 | `fixed::wide::dot2_add` | 73,560 | 17.8 | 1,311,688 | 2.42 % |
| 4 | `fixed::wide::mul_sub` | 50,016 | 15.6 | 781,835 | 1.44 % |
| 5 | `fixed::wide::dot2` | 45,451 | 15.3 | 696,625 | 1.28 % |
| 6 | `fixed::fixed::FixedPartialOrd::gt` | — | — | 628,693 | 1.16 % |
| 7 | `fixed::internal::acc::W3Narrow::narrow` | 40,520 | 14.5 | 589,232 | 1.08 % |
| 8 | `fixed::fixed::FixedSub::sub` | 86,405 | 6.5 | 560,896 | 1.03 % |
| 9 | `glam_core::vec2::Vec2Sub::sub` | 42,983 | 12.7 | 546,194 | 1.01 % |
| 10 | `glam_core::vec2::Vec2Mul::mul` | 19,420 | 26.4 | 513,200 | 0.94 % |

Per family (top 10 each):

| P3 | calls | steps / call | steps | share | | levels | calls | steps / call | steps | share |
|---|--:|--:|--:|--:|---|---|--:|--:|--:|--:|
| `wide::dot2_add` | 7,404 | 17.9 | 132,604 | 3.01 % | | `wide::mul_add` | 23,000 | 15.2 | 350,502 | 3.59 % |
| `FixedMul::mul` | 9,484 | 13.4 | 127,292 | 2.89 % | | `wide::dot2_add` | 13,572 | 17.8 | 242,194 | 2.48 % |
| `Vec2Mul::mul` | 3,375 | 26.5 | 89,394 | 2.03 % | | `FixedMul::mul` | 14,895 | 13.7 | 204,549 | 2.10 % |
| `wide::mul_sub` | 4,632 | 15.6 | 72,370 | 1.64 % | | `wide::dot2` | 9,000 | 15.3 | 137,847 | 1.41 % |
| `wide::mul_add` | 4,093 | 15.6 | 63,735 | 1.45 % | | `wide::mul_sub` | 8,749 | 15.6 | 136,665 | 1.40 % |
| `wide::normalize2` | 1,166 | 48.0 | 55,968 | 1.27 % | | `Vec2Sub::sub` | 8,506 | 12.6 | 106,995 | 1.10 % |
| `Vec2Sub::sub` | 4,167 | 13.1 | 54,532 | 1.24 % | | `W3Narrow::narrow` | 6,790 | 14.5 | 98,486 | 1.01 % |
| `wide::dot2` | 3,171 | 15.4 | 48,935 | 1.11 % | | `FixedPartialOrd::gt` | — | — | 95,154 | 0.98 % |
| `Vec2Add::add` | 3,531 | 13.3 | 46,830 | 1.06 % | | `Vec2Add::add` | 6,004 | 13.1 | 78,872 | 0.81 % |
| `FixedImpl::recip` | 1,120 | 40.2 | 45,001 | 1.02 % | | `FixedSub::sub` | 12,135 | 6.5 | 78,603 | 0.81 % |

| game | calls | steps / call | steps | share | | reference shot | calls | steps / call | steps | share |
|---|--:|--:|--:|--:|---|---|--:|--:|--:|--:|
| `wide::mul_add` | 111,527 | 15.2 | 1,696,213 | 5.40 % | | `wide::mul_add` | 63,221 | 15.1 | 956,843 | 10.96 % |
| `FixedMul::mul` | 57,842 | 13.8 | 797,061 | 2.54 % | | `FixedMul::mul` | 24,628 | 13.8 | 340,979 | 3.91 % |
| `wide::dot2_add` | 39,908 | 17.8 | 711,574 | 2.26 % | | `wide::dot2_add` | 12,676 | 17.8 | 225,316 | 2.58 % |
| `wide::mul_sub` | 28,497 | 15.6 | 445,569 | 1.42 % | | `FixedSub::sub` | 21,397 | 6.5 | 139,666 | 1.60 % |
| `wide::dot2` | 26,771 | 15.3 | 409,764 | 1.30 % | | `FixedPartialOrd::gt` | — | — | 137,540 | 1.58 % |
| `FixedPartialOrd::gt` | — | — | 361,830 | 1.15 % | | `W3Narrow::narrow` | 8,956 | 14.4 | 128,672 | 1.47 % |
| `W3Narrow::narrow` | 22,522 | 14.6 | 328,406 | 1.04 % | | `wide::mul_sub` | 8,138 | 15.6 | 127,231 | 1.46 % |
| `FixedSub::sub` | 49,887 | 6.5 | 323,385 | 1.03 % | | `W7Narrow::narrow` | 8,012 | 13.0 | 104,156 | 1.19 % |
| `Vec2Sub::sub` | 23,486 | 12.6 | 297,001 | 0.94 % | | `wide::dot2` | 6,509 | 15.4 | 100,079 | 1.15 % |
| `Vec2Mul::mul` | 9,818 | 26.4 | 259,206 | 0.82 % | | `Vec2Mul::mul` | 3,379 | 26.5 | 89,380 | 1.02 % |

**Reading.** More than half of the dependency steps are the rescale `narrow32` (`div_rem` by 2^64 plus the `u128`
range check): every fused multiply (`mul_add`, `mul`, `dot2*`, `mul_sub`) is about 13–18 steps, of which `narrow32` is
about 12. `mul_add` alone is 5.7 % of all probe steps and 11 % of the pile10 shot (the solver's `apply`: six
`mul_add` per impulse). A cheaper rescale in `fixed` would move every family; the `Vec2` component-wise operations are
two scalar operations each (no extra cost).

## 10. WS3 — dormant pairs out of the pair list: measured, back to the plan (2026-10-03)

Lot WS3 (`docs/briefs/ws3-dormant-pairs.md`) implemented IT1's parked lever (§4, "dormant pairs out of
`narrow_phase.pairs`") and two parity items, measured them, and shipped none of them: the project manager's rule of
2026-10-03 is zero cost on the default path (no existing `gas/**/*.snap` entry and no default probe may change), and
none of the three met it. What shipped is `WorldState` version 4 with the migration of version 3; its new fields are
reserved for these features and written empty. The lever goes back to `docs/PLAN.md` with what follows.

**Figures** (PR #266 at `2eade5e`, exact Cairo steps, Scarb 2.20.1 / snforge 0.64.0, the Mac, before = alpha.9 at
`b1670ed`; uncommitted probes `crates/rapier2d_classes/tests/ws3.cairo`, `crates/rapier2d/tests/ws3_probes.cairo`):

| probe | before | default (the switch off) | opted in |
|---|--:|--:|--:|
| pile10 owner's shot, in process | 22,507,444 | +40,571 (+0.18 %) | −2,470 (−0.01 %) |
| pile10 owner's shot, slim | 29,500,630 | +44,457 (+0.15 %) | +1,416 (+0.00 %) |
| pile10 reference shot, in process | 8,585,434 | +20,139 (+0.23 %) | −64,902 (−0.76 %) |
| pile10 reference shot, slim | 11,906,622 | +22,429 (+0.19 %) | −62,612 (−0.53 %) |
| level 20, 60 ticks (mixed ticks from 26) | 15,237,617 | +32,840 (+0.22 %) | −678,846 (−4.46 %) |
| `steps_impact_level20` | 3,457,133 | +10,713 (+0.31 %) | −97,320 (−2.82 %) |
| `steps_asleep_level20` (every block asleep) | 968,713 | +2,971 (+0.31 %) | +11,341 (+1.17 %) |
| `steps_game_step` | 2,726,098 | +6,318 (+0.23 %) | −26,878 (−0.99 %) |

Bit-identical in both modes: the per-tick digests of the version-3 felts of the whole state (520 pile10 ticks, 180
level ticks) equal alpha.9's.

**What was learned.**

- **The layout breaks no result but breaks the readers of the list.** With the dormant pairs out of
  `narrow_phase.pairs` after every step, 14 tests fail: golden scenes, the SI sleep diagnostics and the
  staged-against-fused comparisons read `world.narrow_phase.pairs` as the whole list or drive the public stage
  functions on it. The layout must be opt-in (a switch on the world, or a wrapper type), or those tests must read the
  world's queries.
- **Where it pays.** The sparse step stops gathering the live pairs by position, comparing them and writing the whole
  list back (`write_live` / `merge_live`); the island fallback stops splitting it. Level 20's mixed ticks (an awake
  structure next to a sleeping one) lose ≈ 20k each; a pile10 flight tick ≈ 0.9k, the impact tick 4.3k. After the
  pile10 impact every body is awake and nothing is dormant: the owner's shot (108 of its 151 ticks after the impact)
  gains nothing.
- **Where it costs.** On fully asleep ticks the opt-in costs more than it saves (`steps_asleep_level20` +1.17 %): the
  pebble's removal merges the pairs back and the next whole step splits them again.
- **The default path pays for a runtime switch.** A field on `World` (and the removed-collider list on `ColliderSet`)
  rides along every `ref` of the world or the set: two unboxed fields cost +0.05 to +0.96 % per probe, one boxed cell
  each +0.02 to +0.75 %, and any extra branch moves the Sierra gas of the functions that hold it. Zero default cost
  needs the layout outside `World`: a wrapper type holding the world and the dormant list, its own step entry points
  (the sparse step duplicated in `pipeline/active_set`, the default one untouched), removals and queries, and the
  reserved `WorldState` fields. The slim callers would not compile it (their margin, option (A) of the project
  manager): only in-process callers would gain.
- **The positions stay.** The active set must keep the positions of the live pairs in the whole list in both modes:
  the migration, the version-3 felts and the stale positions of an invalid set depend on them.
- **The parity items.** `ColliderSet::take_removed` needs a list on the collider set (+0.1 to +0.2 % per tick, boxed:
  the set rides along every stage) and `GenericJointBuilder::user_data` a field on `GenericJoint` (+0.05 to +0.11 % on
  joint scenes: the solver copies the joint). Neither can reach zero default cost while it lives on a struct the step
  passes or copies; both are back to `missing` in `docs/API_PARITY.md`, with `WorldState` v4's reserved fields ready
  for them.
