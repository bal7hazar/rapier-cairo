# Splitting the game's step across declared classes (CS3)

Toolchain: scarb / Cairo 2.19.4, snforge 0.61.0. Base: `main` at `0c053ea` (after CN1). Class sizes come from
`scripts/bytecode_size.py` (`table`, `attribution --by phases`, release profile) on the `crates/rapier_sink` fixtures.
Steps are exact Cairo steps from `snforge test -p rapier_sink --tracked-resource cairo-steps --detailed-resources`.
snforge counts the steps of the library-called classes and of the syscalls in the caller's test. Engine changes the
measurements needed live on the unmerged branch `proto/cs3-phase-dispatch` (section 6). Target (programme decision
2026-09-27, SNIP-36 path): every class ≤ 73,728 felts in Sierra **and** in CASM, ≤ +25 % steps on the game-shaped path,
≤ 10M Cairo steps per transaction.

## Summary

* **The game's step as one class** (`BasicGameStep`: `WorldState` in, `step_with_force_events_with::<BasicStepConfig>`,
  `WorldState` out) is **133,280 Sierra / 274,232 CASM felts**, 1.63× / 3.35× the limit.
* **Contact generation and the solve split off cleanly** through `StepConfig`'s existing strategy slots. The
  contacts go in two family classes (ball pairs **39,561** CASM, polygon / cuboid / half-space pairs **54,634** CASM)
  with no engine change. The solve goes in one class (**45,552** CASM); it needs `Serde` on the solver's input and
  output (prototype, 0 steps, same `program.basic`). Every result is bit-identical over the whole reference shot.
* **The class that keeps the world does not fit.** In the 4-class layout the caller is **76,745 Sierra / 162,902 CASM
  felts** (1.99× in CASM, 4.23 MB, over the 4.09 MB class-object limit too). Throwaway lever builds bring it down in
  steps: 114,083 CASM with free_path, non-basic shape arms, mass, islands and broad phase gone. It reaches
  **86,357 CASM** only once the fused solve-and-advance and the narrow-phase loop are also out. What remains is the
  step's orchestration: the `WorldState` codec, the active set, glue, user changes and set access. **No 2-, 3- or
  4-class cut of today's step meets 73,728 CASM felts in every class.**
* **Steps on the local pile10 reference shot** (in process 22,432,379, slingfall 22.0M), 151 ticks:
  * contacts out: +16.0 % (compact crossing);
  * solve out: +9.3 %;
  * 4-class layout (both out): **+24.2 %**, 27,859,685 steps.

  A contact call costs ≈ 2.5k steps and a solve call ≈ 16.6k steps. The shot takes 3 transactions of ≤ 10M steps
  either way.
* **API:** yes. `StepConfig` already is the phase-dispatch generic for these two phases. A contract supplies a
  `ContactDispatcher` / `JointStrategy` impl that library-calls; in-process users compile the same code. On the
  prototype, the four `steps_game_basic_*` probes and `program.basic` are unchanged.
* **Recommendation:** build the contact and solver classes (CS4, small), then make the caller fit. That takes new
  strategy slots for the stages that must leave the caller (CS5), then slimming of its own code (CS6), each gated on
  measured sizes. Verify first that the SNIP-36 virtual OS executes `library_call_syscall` (SN1). Nothing on this
  machine runs the virtual OS.

## 1. Fixtures and the pile10 reproduction

`crates/rapier_sink/src/split.cairo`, `family.cairo` (merged): `BasicGameStep`, `ContactClass` / `ContactClassFull` (the
game's `BasicShapesDispatcher` behind one entry point), `ContactBallClass` / `ContactPolygonClass` (one class per
shape-pair family), the caller `SplitNarrowStep`, and `Echo` (a bare library call). Proto only (`split_solve.cairo`):
`SolverClass`, `SplitSolveStep`, `Split3Step`, `Split4Step`. The dispatchers read each class hash from a storage slot of
the executing contract (under `library_call` the caller's storage). Since CS4, `family.cairo` and its `steps_family_*`
probes are gone: the family classes, `SolverClass` and their strategies live in `crates/rapier2d_classes` (constant class
hashes), and `Split3Step` / `Split4Step` are `rapier_sink` fixtures (`src/classes.cairo`).

`crates/rapier_sink/tests/pile10.cairo` rebuilds slingfall's `pile10` level the way slingfall's `GameTrait::new` does:
level order, dynamic bodies asleep, events on blocks and cores, `settle` (a `dt = 0` step, then back to sleep). It
launches the pebble with `sling::launch` and the owner's pull `(-1022, -63)`, and applies slingfall's damage rule (D6)
after each tick. It skips the calm / out-of-bounds / scoring rules and runs a fixed 151 ticks. **The in-process shot
costs 22,432,379 steps, +2.0 % over slingfall's 22.0M** (B4), so it stands in for the game. First contact is at tick
43. Before it, the pile's 29 pairs are dormant and each flight tick meets one pair: pebble against the half-space,
whose AABB is unbounded.

## 2. Size by phase (question 1)

`attribution --class BasicGameStep --by phases` (the table `PHASES` in `scripts/bytecode_size.py`). The attribution
works on CASM felts per Sierra function (exact). Each function goes to the first matching stage, and inlined code
counts in its caller: `step_internal` and `solve_and_advance_sleeping_with` are glue. Shared code keeps its own rows:
fixed-point maths, arena and dicts, corelib. Sierra felts are estimated pro rata of Sierra statements.

| stage | CASM felts | share | Sierra felts (est.) |
|---|--:|--:|--:|
| contact generators: `polygon_polygon` 13,805, `halfspace_pfm` 8,303, `convex_ball` 5,112, `cuboid_cuboid` 1,981, `ball_ball` 1,527 | 30,728 | 11.2 % | 7,396 |
| geometry kernels (SAT, clipping, point projections, features) | 36,951 | 13.5 % | 14,135 |
| `WorldState` encode / decode | 23,021 | 8.4 % | 7,534 |
| solve (sweeps) | 21,969 | 8.0 % | 16,401 |
| constraint build | 9,175 | 3.3 % | 11,422 |
| integrate (bodies, free bodies, damping) | 7,060 | 2.6 % | 3,380 |
| broad phase (proxies, AABBs, pairs) | 20,993 | 7.7 % | 10,220 |
| user changes, mass properties | 18,644 | 6.8 % | 8,360 |
| step glue (`step_internal`, fused solve and advance) | 17,747 | 6.5 % | 11,794 |
| active set (sparse step) | 17,435 | 6.4 % | 7,534 |
| pair-free fast path (`free_path`) | 15,475 | 5.6 % | 11,828 |
| narrow phase (pair loop, solver contacts) | 12,009 | 4.4 % | 7,563 |
| islands / sleep | 11,826 | 4.3 % | 5,803 |
| shapes (other methods) | 11,371 | 4.1 % | 3,845 |
| sets, arena, dicts, world API | 7,718 | 2.8 % | 1,923 |
| fixed-point and vector maths | 5,267 | 1.9 % | 2,114 |
| corelib | 2,898 | 1.1 % | 793 |
| force events | 2,272 | 0.8 % | 952 |
| fixture | 1,608 | 0.6 % | 283 |
| **total** | **274,232** | | **133,280** |

Exclusive cuts give a lower bound on what a class loses: the matched functions, plus everything reachable only through
them.

| cut | exclusive CASM |
|---|--:|
| contact generation (dispatch, generators, kernels) | 75,142 |
| solver (build, sweeps, integrate) | 37,486 |
| mass properties | 20,556 |
| `free_path` | 17,543 |
| islands / sleep | 16,152 |
| broad phase | 8,652 |
| non-basic shape arms (triangle, round, composite, capsule, segment code the `Shape` matches reach) | 30,630 |
| all of these together | 175,922 |

What is left after all of them is ≈ 98k CASM: world, codec, active set, glue and narrow loop.

## 3. Candidate cuts, sizes and crossing data (question 2)

Real builds (release):

| class | Sierra | CASM | class bytes (Sierra / CASM) | fits 73,728? |
|---|--:|--:|--:|---|
| `BasicGameStep` (1 class) | 133,280 | 274,232 | 7.54 / 6.26 MB | no |
| `ContactClass` (all generators, compact crossing) | 25,266 | 86,650 | 1.39 / 1.88 MB | no (CASM) |
| `ContactClassFull` (whole-manifold crossing) | 25,532 | 89,036 | 1.41 / 1.93 MB | no |
| **`ContactBallClass`** (ball–ball, ball–convex) | 14,182 | **39,561** | 0.74 / 0.93 MB | **yes** |
| **`ContactPolygonClass`** (cuboid, polygon, half-space pairs) | 13,659 | **54,634** | 0.72 / 1.13 MB | **yes** |
| **`SolverClass`** (proto; `solve` and `solve_compact`) | 29,973 | **45,552** | 1.66 / 1.15 MB | **yes** |
| `SplitNarrowStep` (caller, contacts out) | 102,313 | 197,982 | 5.74 / 4.60 MB | no |
| `SplitSolveStep` (proto, caller, solve out) | 100,902 | 239,081 | 5.57 / 5.35 MB | no |
| `Split3Step` (proto, caller, contacts and solve out) | 76,616 | 162,688 | 4.22 / 3.68 MB | no |
| `Split4Step` (proto, caller, 2 contact families and solve out) | 76,745 | 162,902 | 4.23 / 3.68 MB | no |

| cut | classes (CASM) | all fit? |
|---|---|---|
| 2: caller + contacts | 197,982 + 86,650 | no |
| 2: caller + solve | 239,081 + 45,552 | no |
| 3: caller + contacts + solve | 162,688 + 86,650 + 45,552 | no |
| 4: caller + ball contacts + polygon contacts + solve | 162,902 + 39,561 + 54,634 + 45,552 | no: the caller is 2.2× the target |

Data crossing each boundary. Felts come from the `Serde` layouts; calls are counted by snforge on the reference shot.

* **Contacts** (`ContactDispatcher::contact_manifold`, once per pair whose AABBs overlap):
  * in: `pos12` 4 felts, two `Shape`s (ball 2, cuboid / half-space 3, pentagon ≈ 40), `prediction` 1, and the manifold
    geometry `ManifoldGeometry` 29 (points, count, normals, subshapes);
  * out: `(bool, ManifoldGeometry)` 30.

  The solver data (`ContactManifoldData`, 30 felts) stays in the caller: no generator reads it. The whole manifold
  would be 59 each way (`ContactClassFull`). Calls: 1 per flight tick (≈ 40 felts in, 30 out). An impact tick makes 2
  (tick 43) to 17 (tick 44) calls, ≈ 0.7k–1.2k felts at 17 pairs. The shot makes 1,428 calls.
* **Solve** (`JointStrategy::solve`, once per step with a touching manifold, never on a flight tick):
  * in: `IntegrationParameters`, per member body a `SolverBody` (12) and `BodyStep` (8), and per manifold a
    `SolverManifold` (35: solver data, point count, the two warm-start impulses: all the solve reads);
  * out: per body a `SolverBody` (12), per manifold a `ManifoldImpulses` (11).

  The frozen `BodyStep`s stay in the caller. Tick 44 (≈ 12 bodies, 17 manifolds): ≈ 0.85k felts in, 0.33k out. The
  shot makes 110 calls; the settle step makes the first.
* **World:** it never crosses inside a transaction. The caller keeps it. Between transactions it crosses once as a
  `WorldState`: 45,339 steps at tick 60 and 37,009 at tick 100 (`into_state`, `Serde` out and in, `from_state`).

## 4. Crossing costs and the pile10 shot (question 3)

`Echo`: a library call costs **≈ 990 steps** with no data, plus ≈ 17 steps per felt echoed. Reading the class hash from
storage adds **204 steps** per call. Per layout, in exact steps:

| layout | load | flight tick (1–30 avg) | tick 43 | tick 44 | ticks 43–151 avg | **shot (151)** | Δ | calls | tx |
|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| in process (`BasicStepConfig`) | 291,129 | 13,451 | 466,988 | 229,113 | 198,149 | **22,432,379** | | 0 | 3 |
| contacts out, whole manifold | 386,890 | 16,893 | 473,781 | 288,074 | 241,346 | 27,381,194 | +22.1 % | 1,428 | |
| contacts out, compact (1 class) | 362,210 | 15,973 | 472,013 | 271,904 | 229,524 | 26,029,300 | +16.0 % | 1,428 | 3 |
| contacts out, 2 family classes | 362,214 | 15,887 | 471,841 | 271,668 | 229,599 | 26,033,870 | +16.1 % | 1,428 | |
| solve out, whole crossing (proto) | 323,860 | 13,467 | 505,767 | 250,282 | 217,064 | 24,527,574 | +9.3 % | 110 | 3 |
| contacts (whole) + solve (whole) (proto) | 419,607 | 16,902 | 512,552 | 309,236 | 260,254 | 29,475,277 | +31.4 % | 1,538 | 4 |
| **4 classes: families + solve, compact (proto)** | 390,232 | 15,896 (+18.2 %) | 505,616 (+8.3 %) | 290,126 (+26.6 %) | 246,089 | **27,859,685** | **+24.2 %** | 1,538 | 3 |
| same, whole solver crossing (proto) | 394,931 | — | — | 292,830 | — | 28,127,953 | +25.4 % | 1,538 | |

(—: not probed; the whole-crossing row has probes at ticks 0, 43, 44, 60, 100 and 151 only.)

* **Per call:** a compact contact call is (26,029,300 − 22,432,379) / 1,428 = **2,519 steps**; the whole-manifold one is
  3,466. A compact solve call is (27,859,685 − 26,033,870) / 110 = **16,598 steps**; the whole crossing is 19,047. The
  compact crossings were measured against the whole ones and won (whole: `split::alternatives`,
  `FullLibraryCallSolver`).
* **Transactions** (greedy, 10-tick granularity, a `WorldState` crossing at every boundary):
  * in process: 3 transactions, ≤ 80 / 130 / 151 ticks at 9.10M / 9.56M / 3.96M steps;
  * 4 classes: 3 transactions, ≤ 70 / 110 / 151 ticks at 8.79M / 9.73M / 9.53M.

  The heaviest 10-tick window is 2.13M in process and 2.69M in 4 classes, so an impact tick (≤ 0.51M) never threatens
  the limit alone. A transaction holds about 40–50 impact ticks.
* **Remaining step levers, derived rather than built:**
  * constant class hashes instead of storage reads: −204 × 1,538 = −0.31M (−1.4 pts);
  * one batched narrow-phase call per family and step instead of one per pair: about −1.4k fixed cost × ≈ 1,270 calls
    ≈ −1.8M (−8 pts). This needs the narrow-phase loop out (section 5).

## 5. The caller's floor (throwaway lever builds)

`scripts/cs3_levers.py` (proto branch) applies cumulative stubs to a `git archive` copy and rebuilds. A lever "off" is a
panicking stub on a branch the game never takes. A stage "out" is replaced by a wrapper that serialises its inputs and
deserialises its result, which is what a library-call wrapper compiles, minus the syscall. The rest of the step stays
reachable. These are sizes only: the stubbed builds do not step.

| `Split4Step` after | Sierra | CASM | Sierra / CASM bytes |
|---|--:|--:|--:|
| 4-class caller (proto) | 76,745 | 162,902 | 4.23 / 3.68 MB |
| `free_path` off | 64,977 | 144,575 | 3.54 / 3.24 MB |
| + non-basic shape arms off (SH1 / SH2 AABB, mass, `Shape` Serde helpers) | 61,349 | 136,553 | 3.35 / 3.05 MB |
| + per-shape mass properties out | 57,693 | 128,230 | 3.15 / 2.82 MB |
| + islands out (`update_islands`, `islands_after_insertions`) | 54,888 | 122,799 | 2.99 / 2.71 MB |
| + broad phase out (`find_pairs`, `find_pairs_sparse`, `near_statics`) | 50,381 | **114,083** | 2.75 / 2.51 MB |
| + fused solve and advance out (bodies and colliders written back) | 36,775 | 95,990 | 2.04 / 2.07 MB |
| + narrow-phase pair loop out (batched) | 32,226 | **86,357** | 1.77 / 1.87 MB |

At 114,083 CASM, what remains of the caller is orchestration:

| part | CASM |
|---|--:|
| `WorldState` codec | 20,419 |
| active set | 17,521 |
| glue | 16,657 |
| user changes | 9,991 |
| narrow loop | 9,191 |
| proxies and AABBs | 8,067 |
| sets | 6,756 |
| island remnants (sleep timers) | 6,652 |
| maths, corelib, events | 8,625 |

Exclusive: user changes 21.3k, active set 20.9k, codec 20.3k, fused solve and advance 19.3k, narrow loop 14.4k. Where
the stages go after 86k:

* a solve-and-advance class ≈ solver 45.6k + 19.3k;
* narrow classes ≈ family class + 14.4k loop, i.e. 54k and 69k. Every one of them would fit.

**Closing the last 12.6k (86,357 → 73,728)** needs changes to the caller's own code, not to its cut:

* a `WorldState` codec for the basic configuration (no joint or non-basic `Shape` serde, out of 20.4k);
* one step skeleton instead of `step_internal` plus `sparse_step`;
* dropping the one-way platform filter, kinematic preparation and `atan2` (≈ 4k) from `BasicStepConfig`;
* or CS1's targeted `#[inline(never)]` (−9 % CASM for +7–8 % steps).

None of these was built here.

## 6. API (question 4) and the prototype

**`StepConfig` already carries the phase dispatch.** Its `Dispatcher` (`ContactDispatcher`) is the per-pair contact
phase, and its `Joints` (`JointStrategy::solve`) is the island solve. A contract writes impls that library-call:
`split::LibraryCallDispatcher`, `family::FamilyDispatcher` (since CS4 `rapier2d_classes::FamilyDispatcher`),
`split_solve::LibraryCallSolver` (since CS4 `rapier2d_classes::LibraryCallSolver`). A program that names
`BasicStepConfig` compiles exactly what it did. No new generic is needed for these two phases.

The contact impls use only public API and live on `main` (this PR). The solve needs `#[derive(Serde)]` on
`SolverInput`, `SolvedIsland`, `ManifoldImpulses`, `PointImpulses` (proto commit `3c30953`). It also needs `pub` fields
on `SolverInput` / `SolvedIsland` and a `pub` `BodyStep` (proto, for the compact crossing that keeps `steps` in the
caller, `dd10298`). Before / after on the proto branch (`e1ccb04`), exact Cairo steps:

| probe (`crates/rapier2d/tests/game_path.cairo`) | `main` | proto |
|---|--:|--:|
| `steps_game_basic_step` | 2,708,888 | 2,708,888 |
| `steps_game_basic_force` | 2,709,273 | 2,709,273 |
| `steps_game_basic_despawn` | 2,693,233 | 2,693,233 |
| `steps_game_basic_chunked` | 2,912,401 | 2,912,401 |

`program.*` and every pre-existing `gas/bytecode.size` class line are identical on the proto branch
(`program.basic` 231,196). All split layouts are bit-identical to the in-process step: the digest of every live body's
pose and velocities, and every entity's hit points, after 151 ticks (`test_*_bit_identical`).

The stages in section 5 have no slot yet: mass properties, islands, broad phase, fused solve-and-advance and the
narrow-phase loop. They would need new strategies, i.e. a new associated impl of `StepConfig` (a breaking change for
custom configurations). The alternative is a second generic on new entry points
(`step_with_force_events_with::<C, P: StepPhases>`, with `InProcessPhases` the in-process default).

## 7. Recommendation and implied lots (question 5)

Build it in this order. Each lot keeps results bit-identical, and in process its `steps_game_basic_*` probes and
`program.basic` stay unchanged:

1. **SN1 (verify, before any engine work):** one `library_call_syscall` from a declared class to another inside the
   SNIP-36 virtual OS, on a machine that runs it. Also check how it charges calldata felts and storage reads there.
   Nothing here executed the virtual OS. The step figures are snforge's; they include its syscall costs.
2. **CS4 (small; engine: derives and field visibility on the solver I/O, 0 steps):**
   * `ContactBallClass`, `ContactPolygonClass` and the `SolverClass` crossing (compact), promoted from fixtures to a
     supported module;
   * constant class hashes in the dispatchers (−0.31M steps on the shot).

   Expected: three classes at 39.6k / 54.6k / ≈ 45k CASM; +24 % steps on the shot, ≈ +23 % with constant class hashes.
3. **CS5 (engine refactor, the step path's shape changes):** strategy slots, in-process by default, for the fused
   solve-and-advance, a batched narrow phase per family, islands, broad phase and mass properties.
   * Measured floor with all of them out: caller **86,357 CASM** (section 5).
   * Steps: batching removes ≈ 1.8M; moving the advance out adds one call per flight tick (not measured).
4. **CS6 (caller slimming to ≤ 73,728):** `free_path` as a config switch, basic-shape `Shape` methods (both already in
   the 86k), a basic `WorldState` codec, one step skeleton, the unused filters out of `BasicStepConfig`. This is the lot
   that decides the target, and no build here demonstrates it.

**Risks:**

* The virtual OS and `library_call` (unverified).
* Its charge per felt of calldata and per storage read (unmeasured).
* The caller may not reach 73,728 without step-costing inlining levers.
* Rounds of engine refactors under a bit-identity constraint (CS5 moves code the P3 / level / game probes all cover).
* Every new `Shape` or strategy grows the family classes.

## 8. Verified vs open

* **Verified (built and run here):**
  * every class size above;
  * the bit-identity of every split layout on pile10 over 151 ticks;
  * the exact steps of every layout (snforge);
  * the game-shaped probes and `program.basic` equal on the prototype;
  * the `WorldState` boundary cost;
  * the transaction counts derived from the probes.
* **Open:**
  * `library_call_syscall` and storage reads under the SNIP-36 virtual OS;
  * the steps of CS5's wrappers (only their sizes were measured, on stub builds);
  * the caller below 86k;
  * the pile10 reproduction runs no calm rule (a fixed 151 ticks, +2.0 % steps against slingfall).

## Appendix: reproduction

```
python3 scripts/bytecode_size.py table
python3 scripts/bytecode_size.py attribution --class BasicGameStep --by phases [--cut LABEL=REGEX ...]
snforge test -p rapier_sink --tracked-resource cairo-steps --detailed-resources
# the family layout (CS3's `steps_family_*`, removed from rapier_sink by CS4): the split step of rapier2d_classes
snforge test -p rapier2d_classes steps_ --include-ignored --tracked-resource cairo-steps --detailed-resources
# CS3's own family probes: `git switch --detach 8fe9398`, then `snforge test -p rapier_sink family`
git switch proto/cs3-phase-dispatch
snforge test -p rapier_sink split4 --tracked-resource cairo-steps --detailed-resources
python3 scripts/cs3_levers.py
```
