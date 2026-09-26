# Class size of the 2D step (CS1)

Toolchain: scarb / Cairo 2.19.4, snforge 0.61.0 (`.tool-versions`). Measured on `main` at `c84eeda` (after BT2); the
fixture table (section 1), the headline combinations (section 3, "after BT3") and the floor were re-measured on
`bc309f0` (after BT3, #146), the base of this lot's PR. Sizes come from
`scripts/bytecode_size.py` (`table`, `attribution`) on the `crates/rapier_sink` fixtures, in the release profile (the dev
profile compiles the same CASM). Steps are exact Cairo steps of G0's level-10 run (`World::step` + despawn, 60 Hz, 4
iterations; the copy of `tests/level_budget.cairo` described in the appendix). **Impact window** = ticks 26–30
(`steps_impact − steps_flight`; 2,277,979 at `c84eeda`, BUDGETS after BT1: 2,283,360; 1,848,678 after BT3).
**Flight window** = ticks 1–25 (`steps_flight − steps_load`; 468,779 at `c84eeda`, 467,947 after BT3).

## Summary

* **CASM is the binding limit.** The game's configuration (`GameStep`: ball, cuboid, convex polygon, half-space; no
  joint, no sensor; `step_with_force_events`) compiles to **435,454 CASM felts after BT3 (5.32× the 81,920 limit)**,
  215,894 Sierra felts (2.64×) and 12.6 MB (3.07× 4,089,446 bytes). At `c84eeda` it was 431,808 CASM, which matches
  slingfall G7b (`SplitSim`: 433,601 CASM).
* **54 % of it is code the game never runs**, which a caller cannot exclude: the joint solver (−37.5 %), sensor tests
  (−3.1 %), the capsule / segment arms of the dispatcher (−8.4 %), plus the pair-free fast path `free_path` (−4.3 %).
  Removing the four (throwaway builds) gives **201,239 CASM after BT3 (2.46×)** at unchanged steps (bit-identical
  run, −0.2 %).
* **No lever combination gets one class under 81,920 CASM felts.** The smallest class measured is **137,247 CASM
  felts after BT3 (1.68×)**. It needs every cut above, dispatch dedup, `#[inline(never)]` on 25 bodies, engine
  `inline(always)` off, no sparse step and `inlining-strategy = 20`, and it costs +72 % steps on the impact window
  and +232 % on the flight window. What remains is flat: no part is above 20 %.
  Sierra felts (≈ 51–70k) and both class byte sizes (2.7–3.9 MB) fit once the unused code is gone. Only CASM does not.
* A **multi-class layout** does not rescue it cheaply. Every class pays ≈ 27k CASM of fixed cost for the
  `WorldState` crossing. The step does not split at a natural stage boundary into pieces under ≈ 55k. Each crossing
  costs **76,942 steps** on the level-10 impact state (3,374 felts of calldata). A 4–5-class chain would cost ≈ +310–385k
  steps per tick: +83–104 % on an impact tick after BT3, ×17–22 on a flight tick.

## 1. Fixtures and limits (`gas/bytecode.size` after BT3, checked by CI's `bytecode` job)

| contract | what it reaches | Sierra felts | CASM felts | × CASM limit | Sierra class bytes | CASM class bytes |
|---|---|--:|--:|--:|--:|--:|
| `WorldOnly` | the game world built, no step | 7,756 | 22,743 | 0.28 | 389,581 | 527,750 |
| `StateRoundTrip` | `WorldState` in, `from_state`, `into_state`, out | 8,460 | 26,602 | 0.32 | 441,067 | 563,938 |
| `GameStep` | game world + `step_with_force_events` | 215,894 | 435,454 | 5.32 | 12,570,031 | 9,781,034 |
| `StateStep` | `WorldState` in, K steps, out (one chunk class) | 221,830 | 450,729 | 5.50 | 12,907,634 | 10,117,645 |
| `Sink` | game + full world (6 shapes, a sensor, 5 joint kinds), `step` and `step_with_force_events` | 223,251 | 455,197 | 5.56 | 13,005,519 | 10,170,274 |

Limits and sources: `scripts/bytecode_size.py` (`LIMITS`, from glam-cairo R1: 81,920 Sierra felts at the gateway,
81,920 CASM felts at Sierra→CASM compilation, 4,089,446 bytes per declared and per compiled class object). The first
step in a class costs 435,454 − 22,743 = **+413k CASM**. A second monomorphisation of `step_internal` (`step` next to
`step_with_force_events`) costs ≈ +12k CASM (`Sink` − `GameStep`, net of the full scene's ≈ 7k).

## 2. Decomposition (`scripts/bytecode_size.py attribution --class <C>`)

The CASM of a compiled class is split per Sierra function (`bytecode_segment_lengths`). A function is assigned to the
first matching part (`PARTS`). Inlined code counts in its caller: the dispatcher is inlined into the narrow-phase
loop, so "dispatch table" shows 0 there.

| part (`GameStep`, `c84eeda`) | CASM felts | share |
|---|--:|--:|
| joint solver (`solver::joint`; `generate_coupled` ×2 at 13.9k, `generate` ×2 at 12.5k / 12.2k, `finalize3*` 22.6k) | 149,987 | 34.7 % |
| contact constraints and solver sweeps (`solver::island` 45.5k: five `split::sweep` kernels 15.5k; `contact`, `body_store`) | 51,719 | 12.0 % |
| geometry kernels (convex polygon 9.2k, SAT, clipping, projections, polygonal features) | 37,474 | 8.7 % |
| pipeline glue (user changes, stages, `step_internal`, `solve_and_advance_sleeping` 9.9k) | 25,968 | 6.0 % |
| contact generators: polygon–polygon 15.1k, capsule–capsule 9.1k, cuboid–segment 7.6k, cuboid–capsule 6.7k, half-space–PFM 6.3k, convex–ball 4.6k, polygon–segment 2.6k, cuboid–cuboid 2.4k, ball–ball 1.5k | 55,843 | 12.9 % |
| pair-free fast path (`free_path`) | 17,526 | 4.1 % |
| sensor intersection tests | 13,334 | 3.1 % |
| active set (BT2 sparse step) | 13,193 | 3.1 % |
| narrow phase (pair loop with the dispatcher inlined, solver data) | 12,585 | 2.9 % |
| mass properties | 11,385 | 2.6 % |
| sleeping / islands | 11,358 | 2.6 % |
| broad phase | 8,467 | 2.0 % |
| sets, arena, dicts, world API | 6,478 | 1.5 % |
| fixed-point and vector maths | 6,774 | 1.6 % |
| events | 2,008 | 0.5 % |
| fixture, corelib | 7,633 | 1.8 % |
| `WorldState` save / restore and Serde (`StateStep` only) | 17,031 | 3.8 % of `StateStep` |

Removal (throwaway builds, section 3) confirms the attribution's exclusive cuts. Joint solver: −162,004 measured vs
151,784 exclusive. Sensors: −13,390 vs 13,339. `free_path`: −18,422 vs 17,526. Sparse step: −17,856 vs 13,266.
Capsule / segment arms: −36,479 measured. The exclusive cut only counts the generators, 33,800.

## 3. Lever estimates (throwaway builds; `GameStep`; steps on level 10; `c84eeda` unless marked BT3)

Every build below ran level 10 bit-identically: the same digest of every pose and velocity after tick 30.

| lever | Sierra | CASM | Δ CASM | × limit | Sierra bytes | impact window | Δ | flight window | Δ |
|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| `main` | 209,007 | 431,808 | | 5.27 | 12,147,938 | 2,277,979 | | 468,779 | |
| no joint solver | 111,689 | 269,804 | −37.5 % | 3.29 | 6,269,971 | 2,275,569 | −0.1 % | 468,779 | 0 |
| no sensor tests | 201,892 | 418,418 | −3.1 % | 5.11 | 11,745,354 | 2,277,979 | 0 | 468,779 | 0 |
| game dispatcher (ball / cuboid / polygon / half-space arms) | 201,326 | 395,329 | −8.4 % | 4.83 | 11,700,766 | 2,277,510 | 0 | 468,805 | 0 |
| no `free_path` | 196,403 | 413,386 | −4.3 % | 5.05 | 11,399,718 | 2,277,831 | 0 | 468,742 | 0 |
| no sparse step (active set) | 201,179 | 413,952 | −4.1 % | 5.05 | 11,696,212 | 2,287,500 | +0.4 % | 1,196,366 | **+155 %** |
| `inlining-strategy = "avoid"` | 161,690 | 358,620 | −16.9 % | 4.38 | 9,271,681 | 5,297,603 | +133 % | 942,207 | +101 % |
| `inlining-strategy = 40` | 156,882 | 368,037 | −14.8 % | 4.49 | 9,018,634 | 2,621,403 | +15.1 % | 514,677 | +9.8 % |
| `#[inline(never)]` on the 10 heaviest inlined bodies | 185,183 | 400,542 | −7.2 % | 4.89 | 10,727,453 | 2,442,613 | +7.2 % | 479,033 | +2.2 % |
| **cut3** = no joints + no sensors + game dispatcher (G7b ask 1) | 97,593 | 219,935 | −49.1 % | 2.68 | 5,473,682 | 2,275,100 | −0.1 % | 468,805 | 0 |
| **cut4** = cut3 + no `free_path` | 85,673 | 201,513 | −53.3 % | 2.46 | 4,769,874 | 2,274,952 | −0.1 % | 468,768 | 0 |
| cut4 + dispatch dedup | 84,855 | 198,348 | −54.1 % | 2.42 | 4,722,313 | 2,273,360 | −0.2 % | 468,872 | 0 |
| cut4 + `inline(never)` ×10 | 71,288 | 182,772 | −57.7 % | 2.23 | 3,906,956 | 2,439,586 | +7.1 % | 479,022 | +2.2 % |
| cut4 + `inline(never)` ×25 | 68,034 | 180,626 | −58.2 % | 2.20 | 3,708,398 | 2,599,101 | +14.1 % | 486,193 | +3.7 % |
| cut4 + engine `inline(always)` off (721 attributes) | 66,056 | 184,984 | −57.2 % | 2.26 | 3,577,383 | 2,781,711 | +22.1 % | 494,847 | +5.6 % |
| cut4 + strategy `avoid` / 40 / 20 | 78,189 / 75,519 / 74,544 | 178,512 / 180,292 / 167,233 | −58.7 / −58.2 / −61.3 % | 2.18 / 2.20 / 2.04 | 4.27 / 4.17 / 4.10 MB | +132 / +14.9 / +41.5 % | | +101 / +9.8 / +29.8 % | |
| **F1** = cut4 + dedup + `inline(never)` ×10 | 70,470 | 179,607 | −58.4 % | 2.19 | 3,859,791 | 2,437,994 | +7.0 % | 479,126 | +2.2 % |
| **F2** = F1 + strategy 20 | 61,527 | 150,976 | −65.0 % | 1.84 | 3,318,367 | 3,390,214 | +48.8 % | 618,742 | +32.0 % |
| **F3** = F1 + no sparse step + `inline(always)` off + `inline(never)` ×25 + strategy 20 | 50,874 | **140,873** | −67.4 % | **1.72** | 2,686,072 | 3,872,920 | +70.0 % | 1,555,962 | +232 % |
| `unsafe-panic = true` | — | — | | | | | | | |
| **after BT3** `main` (`bc309f0`) | 215,894 | 435,454 | | 5.32 | 12,570,031 | 1,848,678 | | 467,947 | |
| BT3 cut3 | 103,172 | 219,661 | −49.6 % | 2.68 | 5,815,300 | 1,845,839 | −0.2 % | 467,973 | 0 |
| BT3 cut4 | 91,253 | 201,239 | −53.8 % | 2.46 | 5,111,168 | 1,845,691 | −0.2 % | 467,936 | 0 |
| BT3 cut4 + dedup | 90,436 | 198,073 | −54.5 % | 2.42 | 5,064,093 | 1,844,099 | −0.2 % | 468,040 | 0 |
| BT3 F1 | 76,330 | 179,184 | −58.9 % | 2.19 | 4,219,924 | 1,999,352 | +8.2 % | 478,294 | +2.2 % |
| BT3 F2 | 67,096 | 151,074 | −65.3 % | 1.84 | 3,660,783 | 2,657,696 | +43.8 % | 616,818 | +31.8 % |
| BT3 F3 | 52,024 | **137,247** | −68.5 % | **1.68** | 2,752,093 | 3,179,022 | +72.0 % | 1,555,936 | +232 % |

BT3's dispatcher calls the generators' `*_fresh` entries; the BT3 rows use a game dispatcher that keeps them (with the
pre-BT3 entries it cost +0.6 % steps). `unsafe-panic` does not compile as a Starknet contract ("Invalid entry point signature"), so it is not a lever.
`cut4` + strategy 0 and the other `max_*` variants were no smaller than F3.

Reading:

* **The cuts are free in steps.** Joints, sensors, capsule / segment arms and `free_path` never run on level 10. The
  step is generic over its output (`StepOutput`) but hard-wires `DefaultDispatcher`, and it always compiles the joint
  solve and the sensor path. `free_path` does not even pay for itself on level 10: dropping it gives −1.6k steps at
  load and −0.1k at impact. (Its P3 free-fall gain is RB's open follow-up.)
* **Dispatch dedup** (every `flipped` literal handed to a generator routed through an outlined identity, so that
  `polygon_polygon::finish`, `convex_ball::finish`, … are not const-specialised twice) saves 3.2k (−1.6 %) with no
  step cost. Const specialisation duplicates ≈ 16.6k CASM in cut4 in total (loop bodies included).
* **Targeted `#[inline(never)]`** beats the global strategies. ×10 saves 18.7k (−9.3 % of cut4) for +7 % steps.
  Strategy 40 saves 21k for +15 %, `avoid` 23k for +132 %. The bodies are ranked by size × extra call sites in a build
  with engine `inline(always)` off: `math_ext::scalar::inv` (30 sites), `island::integrate_body`,
  `split::solve_normal`, `split::apply`, `island::add_force`, `ShapeImpl::compute_aabb`, `split::update_point`,
  `split::solve_both`, `AabbImpl::loosened`, `apply_damping`.
* **Feature gating vs generics.** The cuts above are what `cfg(feature: …)` gates on the dispatcher arms and on the joint
  and sensor stages would give: the closed `Shape` enum keeps its variants, and only the arms go. A step generic over the
  dispatcher and over joint / sensor strategies with no-op impls (G7b ask 1) gives the same code without Scarb
  feature unification across a game's dependency graph. The capsule / segment branches *inside* `halfspace_pfm` and
  `convex_ball` (and `point::capsule` / `point::segment`, ≈ 2.2k) stay reachable either way: they match on `Shape`.

## 4. The floor

After BT3, the smallest single class stepping the game's shapes is **F3: 137,247 CASM felts, 1.68× the limit** (52,024
Sierra felts: fits; 2.75 MB Sierra / 2.71 MB CASM class: fit). It costs +72 % steps on impact ticks and ×3.3 on flight
ticks. The steps-neutral floor is **cut4 + dedup: 198,073 (2.42×)**. The best trade-off is **F1: 179,184 (2.19×) for
+8.2 %**. (At `c84eeda`: 140,873 / 198,348 / 179,607.)
**No combination fits one class.** The remainder of F3 (`c84eeda`) is flat: solver sweeps and constraints 19.7 %, geometry
kernels 15.7 %, glue 10.4 %, polygon–polygon 6.6 %, islands 6.2 %, arena / dicts 5.5 %, broad phase 4.9 %, maths 4.8 %.
Reaching 81,920 would need a further −42 %, i.e. rewriting the solver and narrow phase for size (e.g. one generic sweep
instead of five monomorphised kernels, ≈ 15.5k → ≈ 4k, against BT1's −41 % steps).

## 5. Multi-class layout (`library_call_syscall`, the world crossing as `WorldState`)

* **Fixed cost per class:** `StateRoundTrip` is 26,602 CASM (in F1's `StateStep`: `WorldState` code 17.0k, plus
  arena / maths / corelib shared by every stage). That leaves ≈ 55k CASM per class for physics.
* **Split:** F1's physics is ≈ 160k CASM. Stage "collision" (user changes, broad phase, narrow phase, generators, geometry
  kernels, mass) is ≈ 94k. Stage "solve" (constraints and sweeps, islands, active set, glue, events) is ≈ 66k. Neither
  fits, so the narrow phase and the solver would have to be cut internally (per-pair-type generator classes, solver
  constraints crossing too). That makes a chain of **≥ 4–5 classes per tick**.
* **Steps:** one crossing (`into_state`, `Serde` out and in, `from_state`) of the level-10 world after the impact
  window costs **76,942 steps** for 3,374 felts. G7b measured a library call itself at ≈ 6.8k steps. A 4–5-class chain
  costs ≈ +310–385k steps per tick: +83–104 % on an impact tick after BT3 (369.7k) and ×17–22 on a flight tick (18.7k). The flight
  and sleeping ticks are most of a level, so the run would cost several times its current steps.
* **Verified here:** the sizes and steps above. **Open:** whether the SNIP-36 virtual OS executes `library_call_syscall`
  (it runs the Starknet OS, and slingfall G7b ran the call under snforge only; nothing on this machine runs the
  virtual OS). Every stage class must be declared on the reference block, so each must fit the limits.

## 6. Recommendation for CS2

1. `step_internal` generic over the contact dispatcher and over joint / sensor strategies, with no-op impls and a
   `GameDispatcher`-style narrow table: −49 % CASM, 0 steps.
2. Drop `free_path` (−4.3 %, 0 steps on level 10; confirm on P3's contactless worlds, the open RB follow-up).
3. Dispatch dedup (−1.6 %, 0 steps).
4. Optional: `#[inline(never)]` on the ~10 ranked bodies (−9 %, +8 % steps after BT3), subject to the owner's steps
   criterion.

This gets the game's class to ≈ 180–198k CASM (2.2–2.4×), which is still not declarable. Stone + Integrity on the
standalone executable remains the proof path (programme decision 2026-09-26). Keep the `bytecode` CI job as the budget
that tracks CS2's progress.

## Appendix: throwaway protocol

Each lever: `git archive HEAD` into `/tmp`, a Python patch, then `SCARB_PROFILE=release scarb build -p rapier_sink` and a
`zz_steps` package holding a copy of G0's level-10 loader (`tests/golden_scenes/levels.cairo`) with the level-10 data.
Its tests are `steps_{load,flight,impact}_level10` (0 / 25 / 30 ticks) and a digest of every body after 30 ticks, run
with `snforge test --tracked-resource cairo-steps --detailed-resources`. The patches:

* no joints: `prepare_joints` / `rebuild_joints` return `array![]`, `write_joints` and `array_joint::joints` are empty;
* no sensors: `intersection_pair_step` is empty;
* game dispatcher: `contact_manifold_step` keeps only the ball–ball, cuboid–cuboid, ball–any, any–ball, half-space–{polygon,
  cuboid} in both orders, polygon–polygon and polygon–cuboid in both orders arms;
* no `free_path`: `if free_candidate {..} else {B}` → `B`;
* no sparse step: `step_internal` without the `active_set::usable` early return and the `refresh` loop;
* dedup: `flipped` literals through `#[inline(never)] fn rt(b: bool) -> bool`;
* `inline(never)` ×N: attribute set on the N ranked functions; strategies: `[cairo] inlining-strategy` at the copy's root.
