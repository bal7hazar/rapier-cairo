# Splitting the game's step across declared classes (CS3–CS7, CX1–CX3)

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

## 9. CS5: a stage slot per stage, every stage out (#213)

Drawn from CS5's report. CS5 gave `StepConfig` five stage slots (narrow phase's pair loop, broad phase, islands, fused
solve-and-advance, mass properties) with in-process impls that forward to the stage functions unchanged; CS6 moved them
to a separate `StageConfig` (section 10). `rapier2d_classes` gained `SolveAdvanceClass` (41,186 Sierra / 70,402 CASM),
`IslandsClass` (6,497 / 16,908), `BroadPhaseClass` (4,525 / 9,841), `MassClass` (13,297 / 35,686) and a batched contact
entry point, and the layouts that library-call every stage. Each stage class runs the step's own functions on sets
rebuilt from compact crossings, handles renumbered densely (ADR 0001 entry 40).

| layout (pile10, 151 ticks) | steps | Δ |
|---|--:|--:|
| in process | 22,432,677 | |
| CS4 (contacts per pair, solve out) | 27,545,488 | +22.8 % |
| every stage out, contacts per pair (`StagesSplitStep`) | 32,910,544 | +46.7 % |
| every stage out, contacts batched | 32,927,632 | +46.8 % |
| every stage out, hybrid solve | 32,578,478 | +45.2 % |

Per call: contact 2,318 steps (1,428 calls), broad phase 2,801 (152), islands 13,741 (67), solve and advance 37,960 (152),
mass 5,254 (12). Every stage class fits; the caller with every stage out does not: **120,402 CASM** (55,410 Sierra). CS5's
throwaway lever builds (panicking stubs, sizes only) brought it to 85,882 with CS6's levers 1 and 2 and to 73,369 with the
force events, the active-set rebuild and the pair loop out; the steps of those three were estimated (+0.4M, ≤ +2.7M,
+3.3M).

## 10. CS6: the caller class under 73,728 felts

Toolchain as above. Base: `main` at `07dc0b7` (after CS5 and its docs). Sizes: `scripts/bytecode_size.py` (release);
steps: `snforge test -p rapier2d_classes <filter> --include-ignored --tracked-resource cairo-steps --detailed-resources`
(`tests/{slim,route_b,steps}.cairo`, constant class hashes `PinnedHashes` except where noted). **Gate met:** the caller
`SlimSplitStep` is **28,413 Sierra / 73,181 CASM** felts and every declared class fits (`bytecode_size.py check`, SNIP-36
checks included); the shot is bit-identical to in process at every tick (and with user changes mid-shot); in-process users
are unchanged (section 10.5). Every layout of this section is bit-identical over the whole shot (`--include-ignored`); CI
runs the whole shot for the shipped layouts and 50 ticks for the measured ones (its 16 GB runners cannot hold every
whole-shot trace at once: CS5's variants test alone peaks at 9.0 GB). **Steps: +65.9 %**, 5 transactions of ≤ 10M: inside the programme's first-shot envelope
(≤ +75 %, ≤ 5 transactions), above the +25 % target.

### 10.1 The API change undone

`StepConfig` is back to its released shape (0.1.0-alpha.6: dispatcher, sensor / composite / joint strategies). The stage
slots live in `rapier2d::pipeline::stages::StageConfig`, taken by new entry points `step_with_stages::<C, S>` and
`step_with_force_events_with_stages::<C, S>` (`WorldTrait` and `pipeline`); `step_with::<C>` is
`step_with_stages::<C, InProcessStages<C>>`. `InProcessStages<C>` names the in-process impls, built from `C`'s strategies.
`rapier2d_classes`' CS5 layouts follow as stage configurations (`SplitStages`, `SplitBatchedStages`, `SplitHybridStages`;
`ContactSolveStepConfig`, CS4's layout, stays a `StepConfig`). Cost: zero (section 10.5).

CS6 adds five members to `StageConfig`, in-process by default: `Shapes: ShapeStage` (proxy bounding boxes;
`BasicShapeKernels` compiles the four basic shapes only and rejects the others with `'Step: not a basic shape'`),
`Forces: ForceEventStage`, `Active: ActiveSetStage` (the active-set rebuild), `Free: FreePathStage` (`NoFreePath`: no
pair-free fast path, the ordinary path gives the same results) and `const KINEMATIC: bool` (`false` rejects a
position-based kinematic body with `'Step: kinematic disabled'`). The `free_path` switch is a trailing `&& S::Free::ENABLED`
on the original condition: placed first (or as an `if`, or as a function of the slot), it moved the in-process classes by
+63 to +178 CASM and their steps by a few per step.

### 10.2 Levers 1 and 2, for real

`rapier2d::world::basic_state::BasicWorldState`: `WorldState`'s felts (version 3, no new version), serialized field by
field with the basic shapes' tags and payloads and no joint entry; `from_basic_state` rejects a joint arena that was ever
used, `into_basic_state` writes the real arena bookkeeping and rejects a joint (`'State: joints disabled'`). The one-way
filter lives in `rapier_dynamics2d`'s pair loop (`solver_data_supported`): no configuration can drop it, it leaves with
the pair loop (lever 5).

| caller (`SlimSplitStep` family), CASM felts | alone reverted | all reverted |
|---|--:|--:|
| levers 1 and 2 (`Levers12Step`: free path off, basic shape kernels, no kinematic preparation / `atan2`, basic codec) | 83,573 | 120,402 (`StagesSplitStep`) |
| free path back | 105,591 (+22,018) | |
| every shape's bounding box back | 86,939 (+3,366) | |
| kinematic preparation back | 87,727 (+4,154) | |
| `WorldState`'s codec back | 90,507 (+6,934) | |

Steps: 32,907,106 on the shot (−3,873 against every stage out: the flight ticks take the ordinary path with the broad
phase out). The basic codec's round trip is 221 steps cheaper than `WorldState`'s on a stepped level
(`rapier2d::world::basic_state::tests::gas_*_round_trip`, 479,304 against 479,525).

### 10.3 The remaining ≈ 10k: route (a) against route (b)

**Route (a), stage slots** (`rapier2d_classes`, all measured on the shot):

| caller after | Sierra | CASM | steps | Δ vs in process |
|---|--:|--:|--:|--:|
| levers 1 and 2 | 33,361 | 83,573 | 32,907,106 | +46.7 % |
| + pair loop out (`NarrowPhaseClass`, lever 5) | 29,236 | 75,209 | 37,196,261 | +65.8 % |
| **+ active-set rebuild out (`ActiveSetClass`, lever 4): shipped `SlimSplitStages`** | **28,413** | **73,181** | **37,219,191** | **+65.9 %** |
| + force events out (`ForceEventsClass`, lever 3) | 28,211 | 72,609 | 39,619,796 | +76.6 % |

* `NarrowPhaseClass` (11,319 / 23,597) runs the batched narrow phase (`stages::narrow`, whose last pass is now public:
  `contact_jobs`, `compute_contacts_with_results`) and calls each contact family once per step; the family hashes cross
  with the call (a declared class cannot be compiled with the game's constants). The pair loop reads the collider set only
  for the `Stopped` flag of a dropped pair whose `Started` was emitted: the caller sends those pairs' live colliders and the
  class answers from a set holding exactly them (none on pile10). Crossing: previous pairs, pair colliders (basic shape
  tags), candidate pairs; back: the pairs and the collision events. **+28,218 steps per call** (152 calls) against the
  per-pair contact calls.
* `ActiveSetClass` (4,381 / 10,930) re-runs `active_set::rebuild` on placeholders holding what it reads (five fields of a
  body, five of a collider, the pairs' handles). One call on the shot (+22,930 steps).
* `ForceEventsClass` (2,832 / 6,202) re-runs `collect_convex` on placeholders (the colliders' flags and thresholds, eleven
  fields of each pair) and writes the statuses back: 572 CASM felts for **+2.4M steps**. Measured, not in
  `SlimSplitStages` (kept as a slot a caller can name).

**Route (b), a second orchestration class** (`rapier2d_classes::orchestrator`, `tests/route_b.cairo`): the caller keeps the
world between steps (a game's rules run there) and hands it to `OrchestratorClass` at every step, which runs the step
and returns the world and the force events. The orchestrator reads the stage classes' hashes from the calling contract's
storage (it cannot be compiled with the game's constants).

| route (b) | Sierra | CASM | steps | Δ |
|---|--:|--:|--:|--:|
| orchestrator with levers 1 and 2, pair loop inside (built at `6dabc68`, removed: over Starknet's 81,920) | 33,539 | 84,291 | 47,845,475 | +113.3 % |
| orchestrator with route (a)'s stages (`OrchestratorClass`) | 28,673 | 73,699 | 51,952,331 | +131.6 % |
| its caller (`OrchestratedStep`) | 8,375 | 26,855 | | |

* Persisted codec: the world crosses twice per step with the basic codec (the persisted bytes). The codec's round trip
  alone costs 7,151,663 steps over the shot (`steps_route_b_codec_151` against `steps_basic_151`), 47,365 per tick.
* In-memory layout: whatever the layout, the same felts must reach the orchestrator and come back. Their transfer alone,
  without any codec, costs **9,095,752 steps** over the shot (`steps_route_b_echo_151` − `steps_route_b_codec_151`). With
  the orchestrator's own step (≥ 32.9M, the levers-1-and-2 layout) the floor of any in-memory layout carrying the same
  felts is ≈ 42.0M, above route (a)'s 37.2M; to beat it, a layout would have to carry fewer than ≈ 47 % of the world's
  felts (1,422 to 2,978 per crossing on the shot) at zero codec cost. The orchestrator also needs route (a)'s levers to
  fit.
* **Route (a) is kept** (−14.7M steps against route (b) at the same sizes).

### 10.4 Per layout: classes, shot, transactions, calldata

Transactions: greedy, 10-tick granularity, ≤ 10M steps each; each pays one world crossing (decode in, encode out: the
basic codec's measured round trip, 47,365 steps; CS3 / CS5 charged a crossing to both sides of a boundary, hence their
slightly higher figures). Calldata per transaction: the world in (its felts at the first tick: 2,831 after the settle
step, 2,978 in flight, 1,839 after the first destructions, 1,422 at the end; the same for every layout and codec), the
inputs (the step count; route (b) also the orchestrator's hash) and the world out.

| layout | classes (Sierra / CASM felts; Sierra / CASM class bytes) | shot | Δ | transactions (ticks: steps; calldata in + inputs → out) |
|---|---|--:|--:|---|
| in process | `BasicGameStep` 133,280 / 274,232; 7.54 / 6.26 MB | 22,432,677 | | 0–80: 9.10M (2,831 + 1 → 1,839); 80–130: 9.51M (1,839 + 1 → 1,422); 130–151: 3.96M (1,422 + 1 → 1,422) |
| CS4 | `Split4Step` 76,525 / 162,173; 4.22 / 3.66 MB · `ContactBallClass` 14,950 / 41,560; 0.78 / 0.97 MB · `ContactPolygonClass` 14,515 / 57,299; 0.77 / 1.18 MB · `SolverClass` 29,500 / 43,726; 1.63 / 1.11 MB | 27,545,923 | +22.8 % | 0–70: 8.68M (2,831 + 1 → 1,839); 70–110: 9.58M (1,839 + 1 → 1,422); 110–151: 9.43M (1,422 + 1 → 1,422) |
| CS5, every stage out | `StagesSplitStep` 55,410 / 120,402; 3.03 / 2.61 MB · families as CS4 · `SolveAdvanceClass` 41,186 / 70,402; 2.34 / 1.73 MB · `IslandsClass` 6,497 / 16,908; 0.33 / 0.36 MB · `BroadPhaseClass` 4,525 / 9,841; 0.23 / 0.22 MB · `MassClass` 13,297 / 35,686; 0.69 / 0.87 MB | 32,910,979 | +46.7 % | 0–60: 7.47M (2,831 + 1 → 1,839); 60–90: 8.86M (1,839 + 1 → 1,422); 90–120: 8.25M; 120–151: 8.53M (1,422 + 1 → 1,422) |
| **CS6 route (a), kept** | **`SlimSplitStep` 28,413 / 73,181; 1.55 / 1.49 MB** · `NarrowPhaseClass` 11,319 / 23,597; 0.55 / 0.50 MB · `ActiveSetClass` 4,381 / 10,930; 0.22 / 0.24 MB · families, solve-and-advance, islands, broad phase, mass as CS5 (`ForceEventsClass` 2,832 / 6,202; 0.13 / 0.14 MB, optional) | **37,219,191** | **+65.9 %** | 0–60: 8.57M (2,831 + 1 → 1,839); 60–80: 6.95M (1,839 + 1 → 1,839); 80–110: 9.28M (1,839 + 1 → 1,422); 110–140: 9.26M; 140–151: 3.40M (1,422 + 1 → 1,422) |
| CS6 route (b) | `OrchestratedStep` 8,375 / 26,855; 0.44 / 0.56 MB · `OrchestratorClass` 28,673 / 73,699; 1.57 / 1.52 MB · the classes of route (a) | 51,952,331 | +131.6 % | 7 transactions: 0–40: 7.41M (2,831 + 2 → 2,978); 40–60: 8.65M; 60–80: 8.80M; 80–100: 7.78M; 100–120: 7.69M; 120–140: 7.71M; 140–151: 4.24M (1,422 + 2 → 1,422) |

The shot's steps include the settle step and the launch (`steps_*_0`: 291,126 in process, 581,855 route (a)) and the
class declarations of the test (`steps_install`: 1,533).

### 10.5 In-process users unchanged

`snforge test -p rapier2d --tracked-resource cairo-steps --detailed-resources` before (`main`) and after: all 871
pre-existing tests have identical steps and builtins except the three `stages::tests::test_batched_*` equality tests of
the in-process batched narrow phase (+5 to +156 steps: its last pass is now a separate function), which no shipped
configuration uses. That covers `game_path` (`steps_game_basic_*`), the P3, level, sleep and CCD probes. `program.*`: all
six identical (`program.basic` 231,196). Class sizes: every pre-existing line of `gas/bytecode.size` identical but
`StagesBatchedStep` (+33 CASM, the same batched pass).

### 10.6 Open

* The pair loop's crossing (+4.3M steps) is route (a)'s main cost after CS5's stage crossings: `solver_data_supported`
  rewrites every solver-data field but `user_data`, so the previous pairs could cross as geometry, status, contact count
  and `user_data` (≈ 37 instead of ≈ 64 felts each); not built (the caller's margin is 547 CASM felts).
* The caller's margin is thin: CX1's changes to the solve-and-advance and island crossings move the caller's wrappers.

## 11. CX2: the narrow phase's crossings

Toolchain as above. Base: `main` at `a7d7392` (0.1.0-alpha.7, CX1's slim layout: 32,965,606 steps on the shot, +47.0 %).
Steps: whole-shot probes with `--tracked-resource cairo-steps --detailed-resources`, one at a time; sizes:
`scripts/bytecode_size.py`. The variants' strategies and class entries are at commit `b225f42` of the PR branch (a
throwaway test module, not committed, measured them against CS6's crossing at 32,965,603 steps); the shipped one is `SlimSplitStages`' `LibraryCallNarrowPhase`.

**Programme gate (measurement first) met:** the caller `SlimSplitStep` stays at **73,204 CASM felts** (≤ 73,728; +121)
and the shot gains **2,192,329 steps** (≥ 2M): **30,773,277 steps, +37.2 %** over in process (22,432,677), 4
transactions of ≤ 10M with more headroom. Bit-identical at every tick; in-process users unchanged (no engine file changed).

### 11.1 Where the crossing's steps go

Felts over the shot (152 narrow-phase calls): previous pairs 90,768 (1,417 pairs, 64 felts each), pair colliders 29,162
(942, 31 each), candidate pairs 1,428, contact jobs 55,611 (1,428 jobs, 39 felts each; 233 with a ball) and their results
(30 felts each), new pairs 91,544 (1,428), collision events 152. An extra `Serde` round trip of each crossing added to the
shot costs: previous pairs 1,626,826 steps, new pairs 1,641,700, colliders 579,178, jobs and results 2,153,376:
≈ 18 steps per felt (serialize and deserialize), ≈ 6.0M for the narrow phase's crossings.

### 11.2 Packing loses, dropping felts wins

| variant (whole shot, bit-identical unless noted) | steps | Δ vs CS6's crossing |
|---|--:|--:|
| CS6's crossing (whole previous pairs, both families library-called) | 32,965,603 | |
| every crossing packed in 64-bit lanes (colliders 5 felts + shape, previous pairs 10, jobs 10 + shapes, results 9, new pairs 17) | 33,187,017 | +221,414 |
| the same, the new pairs back by their `Serde` (identity not checked) | 33,952,027 | +986,424 |
| the polygon family in `NarrowPhaseClass`, CS6's felts otherwise | 31,095,744 | −1,869,859 |
| **and the previous pairs as `PreviousPair` (shipped)** | **30,773,277** | **−2,192,329** |

* A packed felt (three lanes: `a + b · 2^64 + c · 2^128`, a `Fixed` biased by `2^63`, two `u32` per lane) is decoded with a
  `u256` conversion and two bounded divisions: ≈ 70 steps to write and read, about the price of the three plain felts it
  replaces. On 15 pairs of the shot (tick 60): a whole pair's `Serde` round trip 1,143 steps, packed 1,314; a geometry
  587 by `Serde` (29 felts), 654 packed (9 felts). The packed wires stay measured in `narrow::alternatives` (round
  trips and `gas_*` probes: net of baseline on the three fixture pairs, whole pairs 3,404 steps, `PreviousPair` 2,895,
  previous pairs packed 2,631, whole pairs packed 3,950).
* **Polygon family merged** (`crate::contact::family_local_polygon`): 84 % of the jobs are pairs without a ball; their
  geometry crossed four times (caller → `NarrowPhaseClass` → `ContactPolygonClass` → back twice). Computing them in
  `NarrowPhaseClass` removes their job and result crossings and 109 of the 797 library calls; the pairs with a ball still
  call `ContactBallClass`, whose hash crosses with the call (`contact_polygon()` is no longer read by the slim layout).
  `NarrowPhaseClass` goes from 11,319 / 23,597 to **22,008 / 68,372** Sierra / CASM felts (margin 5,356 CASM); both
  families' classes are unchanged (the other layouts call them).
* **`PreviousPair`** (`narrow::{PreviousPair, previous_of, pair_of}`): a previous pair crosses as its handles, event status,
  solver contact count, user data and geometry, 36 felts instead of 64. The pair loop rewrites every other field of the
  solver data (`solver_data_supported`: bodies, flags, friction, restitution, dominance, normal, both solver contacts) and
  reads only the contact count (`had_contact`); the class rebuilds the pair with the default solver data but those two
  fields (`pair_of`), so its results are those of the whole pair. −322,533 steps for +121 CASM felts in the caller.

### 11.3 Per layout: classes, shot, transactions

Transactions as in section 10.4 (greedy, 10-tick granularity, ≤ 10M steps, one world crossing of 47,365 steps each):

| layout | classes changed (Sierra / CASM felts) | shot | Δ | transactions (ticks: steps; calldata in + inputs → out) |
|---|---|--:|--:|---|
| CX1 (`main`) | `SlimSplitStep` 28,518 / 73,083 · `NarrowPhaseClass` 11,319 / 23,597 | 32,965,606 | +47.0 % | 4 |
| **CX2** | **`SlimSplitStep` 28,638 / 73,204** · **`NarrowPhaseClass` 22,008 / 68,372** · `OrchestratorClass` 28,819 / 73,570 | **30,773,277** | **+37.2 %** | 0–60: 7.22M (2,831 + 1 → 1,839); 60–90: 8.41M (1,839 + 1 → 1,422); 90–120: 7.54M (1,422 + 1 → 1,422); 120–151: 7.79M (1,422 + 1 → 1,422) |

Per narrow-phase call (152): ≈ 800 fewer felts cross (over the shot, the previous pairs go from 90,768 to 51,164 felts,
1,417 × 36 plus the counts; the 1,195 polygon jobs, ≈ 46,600 felts, and their results, ≈ 35,850, no longer cross) and
14,423 fewer steps.

### 11.4 Bit-identity and in-process users

`snforge test -p rapier2d_classes` (every default test, single-threaded): the slim layout against `BasicStepConfig` at
every tick of the shot and with user changes (`tests/slim.cairo`), the removals scene (`tests/removals.cairo`: holes and
reused slots, kinematic bodies, a parentless collider, an impact wake-up) and the game's ticks with force events every
step and basic-codec save / restore mid-collapse (`tests/game_ticks.cairo`); `snforge test -p rapier_sink` (58 tests).
No file of `rapier2d` or of its dependencies changed: every `steps_*` probe of `rapier2d`, the CCD tests, the game-shaped
probes and `program.basic` (231,196 felts, `bytecode_size.py check`) are unchanged by construction.

### 11.5 Open

* The previous pairs packed (10 felts) would save ≈ 90 more steps per pair in process (the `gas_*` probes above), ≈ 0.13M
  on the shot, for a lane encoder in the caller (margin 524 CASM felts): not built.
* The new pairs (64 felts, 1.64M steps per round trip over the shot) cross whole: their handles and parents are
  derivable in the caller, at the price of caller code.
* The pairs with a ball (233 jobs) still cross to `ContactBallClass` (41,560 CASM felts: it does not fit next to the pair
  loop and the polygon family).

## 12. CS7: the slim caller to 67,076 CASM felts, and a World-edits class

Toolchain as above. Base: `main` at `642ee93` (after DU1: `SlimSplitStep` 28,638 / 73,204 felts, the pile10 shot
30,773,277 steps, both re-measured there). Programme request (2026-09-28): `SlimSplitStep` ≤ 69,891 CASM felts for at
most +1.5M steps on the shot, same results. Sizes: `scripts/bytecode_size.py` (release) and its `attribution`; steps:
`snforge test -p rapier2d_classes slim::steps_slim_151 --include-ignored --tracked-resource cairo-steps`.

**Go: the caller is 26,834 / 67,076 felts (−6,128, margin 6,652 under 73,728) and the shot 30,711,536 steps (−61,741,
+36.9 % over in process)**, bit-identical at every tick; no stage class changed (every pinned hash is the same).

### 12.1 Candidates (measurement first)

Exclusive CASM felts of each part of the caller (`attribution --cut`, what leaves with it) and, when built, the caller
and the shot. The throwaway builds (A1, A2: the sparse step switched off for `SlimSplitStages`, not committed) were measured at
`9c7372b` (DU1 left the shot's steps unchanged).

| candidate | exclusive | caller | Δ felts | shot | Δ steps | verdict |
|---|--:|--:|--:|--:|--:|---|
| sparse step off, set never refreshed (A1) | 15,614 | 57,578 | −15,626 | 32,180,777 | +1,407,500 | no: the saved `WorldState` carries an invalid active set (not bit-identical) |
| sparse step off, set rebuilt in `ActiveSetClass` each flight tick (A2) | | not measured | | 33,678,063 | +2,904,786 | no: over +1.5M (bit-identical) |
| user changes (`body_changes`, `collider_changes`) | 5,039 | | | | | not built: of it, 3,885 is the mass crossing (below); the rest needs a crossing on every tick with a change |
| the whole-step skeleton (`step_internal`: 110 of the shot's 152 steps) | 3,237 own | | | | | not built: a world crossing costs ≈ 105k steps (12.3), ≈ +11.5M |
| arenas (`ArenaStateImpl`, `ArenaImpl`) | 4,924 own | | | | | out of scope (`rapier_core`), used by every stage |
| force events out (CS6, `ForceEventsClass`) | 2,277 | 72,609 (CS6) | −572 | | +2.4M | no |
| **the codec's reader through outlined leaf readers** | 12,360 (decode) | 68,471 | **−4,733** | 30,776,132 | +2,855 | **shipped** |
| **the mass crossing by the codec's basic collider writer** | 3,885 (mass call) | 67,076 | **−1,395** | 30,711,536 | −64,596 | **shipped** |
| the contact pairs by the same readers (codec and `NarrowPhaseClass`'s answer) | | 64,978 | −2,098 | 31,703,816 | +927,684 | rejected (`decode::alternatives`) |

The shot takes the sparse step on 42 flight ticks only (2 steps before the flight, 108 after the impact take the whole
step), so neither skeleton can leave cheaply.

* **The reader** (`rapier2d::world::basic_state::decode`): the derived `Serde` inlines the conversion and the `None`
  branch of every field (a `Fixed` is a range check), and reads a `[Vec2; 8]` through seven tuple splits (1,255 felts
  for the polygons alone). One `#[inline(never)]` reader per leaf (`Fixed`, `u32`, `bool`, `Vec2`, handle, pose, mass
  properties), called by every field, and loops for the arenas' entries: same felts, same values, `None` on malformed
  input as before. `ActiveSetClass`'s and `MassClass`'s answers use the same readers (`read_active_set`,
  `read_body_mass_props`), otherwise the derived code stays in the caller next to them. The round trip costs +5,065
  steps (484,369 against 479,304, `gas_basic_state_round_trip`), once per transaction.
* **The mass crossing** serialized the colliders with the derived `Serde` (every shape's serializer); the codec's
  `serialize_collider` writes the same felts for the basic shapes (`MassClass` unchanged).
* **The pairs** cross 1,428 times on the shot: a call per field costs ≈ 650 steps per pair over the inline derived code.

### 12.2 The caller and the shot

| | Sierra | CASM | shot | Δ vs in process | transactions (≤ 10M, 10-tick granularity) |
|---|--:|--:|--:|--:|---|
| CX2 / DU1 (`main`) | 28,638 | 73,204 | 30,773,277 | +37.2 % | 4 |
| **CS7** | **26,834** | **67,076** | **30,711,536** | **+36.9 %** | 4: 0–60 7.19M; 60–90 8.34M; 90–120 7.47M; 120–151 7.72M (+ one world crossing each) |

Other lines of `gas/bytecode.size` that move: `Levers12Step` 78,382 (−5,084) and `OrchestratorClass` 67,442 (−6,128),
the same reader; CS5's `Stages*Step` +937 to +992 (their full `WorldState` codec shared the derived collider serializer
with the mass crossing). Every declared stage class is unchanged.

### 12.3 The World-edits class

`rapier2d_classes::edits`: `WorldEdit` (`Insert` a body with one basic-shape collider and its velocities, `Remove`,
`Sleep`), `apply_edits` (in process, the `World` methods), `edit_world(class_hash, world, edits)` (the caller's side:
the world crosses in and out with the basic codec, the edits as the felts of a `Span<WorldEdit>`, forwarded) and
`WorldEditClass`. Pile10 at tick 60 (a world of 1,839 felts), `tests/edits.cairo`:

| | Sierra | CASM | per call (steps, class − in process) |
|---|--:|--:|--:|
| `WorldEditClass` | 17,610 | 51,865 | |
| `SlimEditStep`: `SlimSplitStep` + `edit_world` | 27,135 | 68,818 (+1,742) | launch 110,590; removal 104,970; the end's sleeps and the pebble's removal 106,357 |
| `SlimInCallerEditStep`: the same edits in process (loser) | 35,721 | 96,657 (+29,581) | |

The crossing fits and the in-caller edits do not. Bit-identical to the same edits in process on the whole shot (the
settle's sleeps, the launch, every destruction, the end's sleeps and the pebble's removal) and to the reference shot.

### 12.4 Only declarable classes in the published crate

`OrchestratorClass`, `StoredClassHashes` and `orchestrated_step` (route (b)) moved to `rapier_sink::orchestrator` (still
built and tracked in `gas/bytecode.size`). Route (b)'s whole-shot tests stayed at `9c7372b`: they declare the stage
classes, which only `rapier2d_classes`' own tests can. The measured-only stage configurations (`SplitStages`,
`SplitBatchedStages`, `SplitHybridStages`) stay: they compile no class and the crate's tests run them. The README lists
the classes a game declares for `SlimSplitStages` and for CS4's layout.

### 12.5 Bit-identity and in-process users

`snforge test -p rapier2d_classes` (58 tests: the slim layout at every tick of the shot and with user changes,
`removals`, `game_ticks`, the edits), `-p rapier_sink` (58), `test_pinned_class_hashes` (unchanged). `snforge test -p
rapier2d --include-ignored --tracked-resource cairo-steps` on `642ee93` and on CS7: of the 936 common tests, only five
codec tests differ (`gas_basic_state_round_trip` and four `test_basic_codec_*`); every `steps_*` probe, `game_path`
(`steps_game_basic_step` 2,708,888, `_force` 2,709,273, `_despawn` 2,693,233, `_chunked` 2,912,401), the P3, level,
sleep and CCD tests are identical in steps and builtins. `program.basic` 231,196.

### 12.6 Open

* The game's world class: S36a measured it 3,837 felts over the slim caller (76,920); with this caller it would be
  ≈ 70,913 if that difference holds (not measured here).
* `WorldEditClass` is not in `bytecode_size.py`'s `DECLARED` (its SNIP-36 interface was checked by hand: builtins
  `bitwise`, `range_check`, `segment_arena`; no syscall).
* The pairs could still cross the caller's boundary in fewer felts (CX2's open items).

## 13. CX3: the slim layout's crossings (IT1 step 2, lever X1)

Toolchain: Scarb 2.20.1 / snforge 0.64.0 (TC1). Base: `main` at `5bfc4d3`. Exact Cairo steps
(`--tracked-resource cairo-steps`), `RAYON_NUM_THREADS=1`, `--max-threads 2`, on the Mac (Apple silicon arm64), build
path `/Users/bal7hazar/.herdr/worktrees/rapier-cairo/hp-slingfall-rapier-t-0017-cx3-slim-crossings-x1`; profiles with
cairo-profiler 0.17.0 and IT1's `stages.py`. The probes (pile10's owner's and reference shots built, launched and run
`N` ticks, then a digest, as IT1's) stayed uncommitted.

**Shipped: the owner's slim shot 30,941,790 → 29,849,153 steps (−1,092,637, +30.5 % over in process instead of
+35.3 %), the reference shot 12,548,826 → 12,065,485 (−483,341)**, bit-identical, `NarrowPhaseClass` 68,470 → 67,179
CASM felts, the caller unchanged. The estimate of IT1 (−2 to −4M on the owner's shot) is not reached: one proof of the
owner's shot needs ≈ −2.89M. On the game's basis (154 L2 gas per step) the owner's shot goes from 5.445e9 to ≈ 5.28e9
virtual L2 gas, still 6 proofs, and the reference shot from 2.645e9 to ≈ 2.57e9, still 3; those absolute figures are the
game's cost sheet on alpha.8 (Scarb 2.19.4, before TC1), not re-measured after TC1 (+1.25 to +1.84 % steps on the game
path), so they are lower bounds.

### 13.1 Where a collapse tick's crossings go

Owner's tick 50 (all bodies awake, 15–19 pairs), slim 275,932 steps against 197,135 in process (`main`):

| | steps |
|---|--:|
| narrow phase: the caller's side (previous pairs and colliders written, the new pairs read: `ContactPairSerde` 14,448) | 22,924 |
| narrow phase: the class's arguments read (`PreviousPair` 7,792, `PairCollider` 3,553) and its answer written | 15,889 |
| narrow phase: the class's own work (`NarrowPhaseClass::compute_contacts`) | 64,866 |
| in process, the whole narrow phase | 44,672 |
| solve and advance: the caller's side (`encode` ≈ 5.5k, `write_back` 13,545, the answer read) | 24,278 |
| solve and advance: the class's arguments read (`MotionBody` 3,240, `TouchingManifold` 3,051) and its answer written | 8,529 |

The class's narrow phase cost 20k more than in process: CX2's batched body rebuilt every previous pair whole
(`pair_of`), built a job per pair (`contact_jobs`: the pose, both shapes and the previous geometry copied, 11.8k), ran
the generators over the jobs (`family_local_polygon`), then walked the pairs again over the results.

### 13.2 Candidates

| candidate | owner's shot | Δ | reference shot | Δ | verdict |
|---|--:|--:|--:|--:|---|
| `main` (CX2's crossing) | 30,941,790 | | 12,548,826 | | |
| **the class's own pair loop** (`narrow::pair_loop`) | **29,849,153** | **−1,092,637** | **12,065,485** | **−483,341** | **shipped** |
| and the new pairs back without what the caller derives (`narrow::alternatives::NewPair`) | 30,040,823 | +191,670 | 12,146,515 | +81,030 | rejected |
| the pairs with a ball batched again (one call per step) | not built | | | | estimate ≈ 0 |
| the touching manifolds forwarded from the narrow phase to the advance class | not built | | | | estimate ≤ 0 |

* **The pair loop** walks the `PreviousPair`s as they cross (no whole pair rebuilt) and generates a pair's contacts where
  it reaches it: the polygon family in the class, a pair with a ball by one call of `ContactBallClass`'s
  `contact_geometry` entry (the per-pair entry CS4 already declares). Each generator gets what its job carried (the pose
  of collider 2 in collider 1's frame, the shapes, the previous geometry found by the same walk, default solver data) and
  the manifold gets the previous solver data back before `solver_data_supported`, so pairs and events are those of
  `compute_contacts_with_results`. The collapse tick: 275,932 → 264,857 (class work 64,866 → 53,801). The ball calls cost
  632,321 steps over the owner's shot (≈ 233 pairs, of which ≈ 337k is crossing); CX2's batched body is kept as
  `narrow::alternatives::batched_in_class`.
* **The new pairs' wire** (one record per candidate pair of two solid colliders: no handles, no bodies, an unused solver
  contact slot as `None` when it is the default, 40 + 8 felts per used slot instead of 64): the caller's decoding loop
  costs 17,330 steps at the collapse tick against 14,448 for the whole pairs, and the class pays for the slot tests.
  Rebuilding a pair costs about what deserializing it does: dropping derivable felts pays only where nothing has to be
  rebuilt (CX2's previous pairs).
* **Not built.** Batching the pairs with a ball again saves ≈ 1k per extra call on the ticks with two or more of them but
  needs a pre-pass on every step (≈ 233 pairs on 152 steps: the two cancel, estimate). Forwarding the touching manifolds
  as felts from `NarrowPhaseClass` to `SolveAdvanceClass` saves only the caller's `touching_manifold` writes (≈ 2.5k per
  collapse tick), moves the work into the narrow class and breaks when the island stage revives dormant pairs between
  the two calls. Dropping the solver contacts' tangent velocities (always zero from the narrow phase) is not exact for a
  restored world whose dormant pairs carry other values. The advance write-back (`write_back`, 13.5k per collapse tick)
  is the in-process stage's own write-back (`scatter_impulses`, the bodies' world mass properties, the colliders'
  poses), not crossing.

### 13.3 Classes, bit-identity, in-process users

| class | Sierra | CASM | margin (Sierra / CASM) |
|---|--:|--:|--:|
| `SlimSplitStep` (caller) | 26,844 | 67,108 | 46,884 / 6,620 (unchanged) |
| `NarrowPhaseClass` | 22,027 → 22,667 | 68,470 → 67,179 | 51,061 / 6,549 |

Every other declared class is unchanged; only `NarrowPhaseClass`'s hash moves. `snforge test -p rapier2d_classes` (59
tests: the 58 of `main` and this lot's `test_new_pair_round_trip`) and its 20 ignored `*_bit_identical` tests pass unchanged (the slim layout at every tick of the owner's shot, with
user changes, `removals`, `game_ticks`, `edits`, `split`); the reference shot was checked at every tick, slim against in
process, by an uncommitted probe (the whole world's digest). No engine file changed: the in-process shots are identical
in steps (22,867,951 and 8,752,430).

### 13.4 Open

* The crossings left on a collapse tick, ≈ 39k (narrow phase) and ≈ 32k (solve and advance), are per-felt reads and
  writes of what the classes need (CX2: ≈ 18 steps per felt both ways); the one large cut left is fewer crossings (the
  narrow phase, the islands and the solve in fewer classes), bounded by the 73,728 limit (`NarrowPhaseClass` margin
  6,549 CASM felts, `SolveAdvanceClass` 15,181).
* The in-scope engine levers (IT1 §4, the next lot) apply to the slim layout but N1.

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
# CS6: route (a) per lever, route (b), the state felts, the call counts
snforge test -p rapier2d_classes slim --include-ignored --tracked-resource cairo-steps --detailed-resources
snforge test -p rapier2d_classes route_b --include-ignored --tracked-resource cairo-steps --detailed-resources
snforge test -p rapier2d_classes steps_cs4_ --include-ignored --tracked-resource cairo-steps --detailed-resources
snforge test -p rapier2d_classes test_state_felts --include-ignored
# CX2: the narrow phase's crossings (the variants' code: `git switch --detach b225f42`)
snforge test -p rapier2d_classes narrow:: --tracked-resource cairo-steps --detailed-resources
snforge test -p rapier2d_classes slim::steps_slim_ --include-ignored --max-threads 1 --tracked-resource cairo-steps --detailed-resources
# CS7: the caller's parts, the edits' calls, the readers
python3 scripts/bytecode_size.py attribution --class SlimSplitStep --cut 'sparse=active_set::sparse_step' --cut 'decode=basic_state::(BasicWorldStateSerde::deserialize|from_basic_state)'
snforge test -p rapier2d_classes edits::steps_edit --include-ignored --max-threads 1 --tracked-resource cairo-steps --detailed-resources
snforge test -p rapier2d basic_state --tracked-resource cairo-steps --detailed-resources
# CX3: IT1's pile10 probes (uncommitted) before / after, profiles with cairo-profiler 0.17.0
snforge test -p rapier2d_classes <probe> --include-ignored --tracked-resource cairo-steps --build-profile --max-threads 2
```
