# FU0 — fewer fixed-point rescales on the step path

Lot FU0, research only (brief `docs/briefs/fu0-fused-rescales.md`). Nothing in the engine changed: the probes and the
prototype are uncommitted (§7). The study proposes; the project manager decides.

**Figures.** Steps are exact Cairo steps (`--tracked-resource cairo-steps`) measured on the VPS (x86_64), Scarb 2.20.1 /
snforge 0.64.0, `RAYON_NUM_THREADS=1`, base `main` @ `0e009d7`, every build under
`prlimit --as=8589934592`. Call counts come from HP (`docs/research/impact-tick.md` §9, the Mac, `2a80232`).
Everything marked *(est.)* is an estimate: a measured per-call saving times a call count that is derived or
extrapolated, as each line says. "ulp" is one raw unit of Q32.32 (2^-32).

## 0. Summary

| # | formula (file) | rescales today → fused | steps / call, measured | calls, reference shot | reference shot *(est.)* | owner's shot *(est.)* | bit-identical |
|---|---|---|---|--:|--:|--:|---|
| P2 | refresh separation: two `transform` then `separation` (`split.cairo` `refresh_point`; same function in generation's `round_trip`) | 6 → 2 | 131 → 72 (−59) | ≈ 3,120 (derived) | **−0.18M** | **−0.54M** | no (≤ 2 ulp per call, 37 % of outputs) |
| P1 | row solve `impulse − r · (jv + rhs)` (`split.cairo` `solve_normal`, `solve_tangent`) | 2 → 1 | 55 → 38 (−17) | 8,012 (HP, exact) | **−0.14M** | **−0.40M** | no (1–3 ulp, every output) |
| P3 | effective mass `dir · (im_sum ∘ dir) + g1 ig1 + g2 ig2` (`generation.cairo` `coefficients`) | 3 → 1 | 57 → 32 (−25) | ≈ 1,560 (derived) | −0.04M | −0.12M | no (≤ 2 ulp, 16 % of outputs) |
| C5 | manifold update `update_candidate` + `normal_matches` (`rapier_geometry2d/src/manifold.cairo`) | 5 → 3 per point, 3 → 1 per manifold | not prototyped (≈ −45 per point, −24 per manifold) | ≈ 550 points | −0.03M | −0.09M | no |
| C7 | `Rot2::integrate` (`rapier_math/src/rot2.cairo`) | 3 → 2 (+ renormalise) | not prototyped (≈ −12) | ≈ 800 | −0.01M | −0.04M | no |
| C6 | half-space vertex `add_vertex` (`halfspace_pfm.cairo`) | 5 → 3 per vertex | not prototyped (≈ −37) | ≈ 225 | −0.01M | −0.03M | no |
| B1 | solver contact anchors: `transform(..) − world_com` folded into the wide sum (`narrow_phase.cairo` `solver_contact`) | 2 → 2 (two checked subtractions go) | not prototyped (≈ −10 per anchor) | ≈ 1,560 anchors | −0.02M | −0.05M | **yes** (`floor(x) − c = floor(x − c)` for an integer `c`) |

**Prototype of P1 + P2 + P3 in the real solver** (patched copy of the crates, §5): the 21 golden scene tests of
`crates/rapier2d/tests/golden_scenes.cairo` that fit the 8 GB cap all pass within their bands; the contact scenes take **2.8–4.1 % fewer steps** (box_stack3 windows −4.0 to −4.1 %,
≈ −530 steps per contact point and tick, above the probes' sum), and the positions stay within 62 ulp of today's
over 120 steps. Against the f64 oracle the fused solver is closer on some scenes and farther on others (§4).

**Recommendation (§6).** One result-changing lot, **FU1** (`impl-opus`): P1 + P2 (refresh and generation's round trip
together) + P3, with the unfused forms kept under `mod alternatives`; estimate **−0.36 to −0.41M steps on the reference shot and
−1.06 to −1.22M on the owner's shot**, the same in process and slim. Bundle it with C5 / C6 / C7 if they are taken, in **one**
numeric release, so that the game re-pins once. B1 is bit-identical and can ship on its own. FU1 alone does not reach
5 proofs for the owner's shot on these estimates (EL1: ≈ −1.47M more needed); with C5–C7 and B1 it comes within the
estimates' uncertainty.

## 1. Method

**Where the rescales are.** HP (§9) counted each `fixed` / `glam_core` operation through a libfunc it runs a known
number of times; every fused kernel already rescales once (`mul_add`, `dot2_add`, `W3Narrow`, `W7Narrow`, …). What is
left is a *chain* of kernels: an output of one rescale feeds the wide sum of the next (`dv = narrow(..)` then
`r * dv`). The table below follows HP's operations to their call sites on the pile10 step path (solver sweeps and
generation, narrow phase, integration) and keeps the chains whose intermediate value is used nowhere else, so that it
can stay wide.

**How a chain is fused, without changing `fixed`.** `fixed` 0.4.0's `wide` module already has every piece: a Q64.64
sum `Wn` times a `Fixed` is a Q96.96 sum `Tn` (`WideMul`, one step), `Wn.lift()` moves a Q64.64 term to Q96.96
(`WideLift`), and `Tn.narrow()` is `narrow64`, a `div_rem` by 2^64 that is as cheap as `narrow32` (it skips the
`* 2^32`). The type bound (`n ≤ 16`) holds for every candidate below.

**Call counts on the reference shot (in process).** HP's counts are per operation, not per call site; they are
attributed as follows.

- `W7Narrow::narrow`, 8,012 calls: only `split::jv_add` sums seven terms on the pile10 path (six products and `rhs`).
  So P1 runs **8,012** times (exact).
- `W3Narrow::narrow`, 8,956 calls: the `x` component of `split::transform` (refresh, generation's round trip) and of
  `narrow_phase::transform` (`solver_contact`). Per solver point and tick that is 4 refreshes × 2 transforms, ≤ 2 round-
  trip transforms (none for a world end) and 2 anchors: 11–12 per point-tick, so **a ≈ 750–810 point-ticks**, and
  **≈ 3,120 refreshes** (4a) for P2.
- Cross-checks: 8,012 row solves are 86 % of the 12a ≈ 9,360 rows that 4 substeps × (one biased normal row + one normal
  and one tangent relaxation row) allow (`friction_in_bias_pass` is false); the rest are skipped by `idle` and the zero
  friction limit. `Vec2Mul::mul` (3,379) ≈ 2a (P3's `im_sum * dir`) + 4 weights per manifold-tick.
- P3 runs twice per point-tick (normal and tangent rows): **≈ 1,560**.

**The owner's shot** could not be profiled (HP: the profiler needs more than the Mac's memory). Its counts are
extrapolated by the ratio of the steps after the impact, 21.06M / 6.86M ≈ 3.07 (IT1 §1), applied to the reference's
737 point-ticks after the impact: a ≈ 43 + 737 × 3.07 ≈ 2,300, **≈ 2.95 × the reference** *(est.; the owner's ticks
after the impact are lighter, 195k against 286k on average, so the ratio may be lower)*.

**In process and slim.** The solver runs inside `SolveAdvanceClass` on the slim layout and the narrow phase's
`solver_data_supported` / manifold update inside `NarrowPhaseClass` (`pipeline/stages/narrow.cairo` calls the same
functions), so every saving below applies to both layouts with the same absolute value; only the percentage differs
(reference 8.64M in process / 12.45M slim, owner 22.37M / 30.71M, IT1 §1).

## 2. The formulas

Notation: `⌊·⌋` is one rescale (floor toward −∞, `narrow32` or `narrow64`); everything inside one `⌊·⌋` is an exact
wide sum.

### P2 — the refresh separation (`split.cairo`: `refresh_point`, `update_point` without reuse; `generation.cairo`: `round_trip`)

Today, per contact point and refresh:

```
a  = (⌊re1·l1x − im1·l1y + t1x⌋, ⌊im1·l1x + re1·l1y + t1y⌋)       transform(p1, l1): 2 rescales
b  = (⌊re2·l2x − im2·l2y + t2x⌋, ⌊im2·l2x + re2·l2y + t2y⌋)       transform(p2, l2): 2 rescales
dist   = ⌊(ax − bx)·dx + (ay − by)·dy + dist0⌋                     separation: 1 rescale
t_dist = ⌊(ax − bx)·tx + (ay − by)·ty⌋                             1 rescale
```

Six rescales; `a` and `b` are used nowhere else. Fused: the difference `p1·l1 − p2·l2` stays an exact Q64.64 sum
per component (`W6`: four products and two lifted translations), projected on `dir` and `t` at Q96.96:

```
DX = re1·l1x − im1·l1y + t1x − re2·l2x + im2·l2y − t2x                (exact, W6)
DY = im1·l1x + re1·l1y + t1y − im2·l2x − re2·l2y − t2y                (exact, W6)
dist   = ⌊DX·dx + DY·dy + dist0⌋    (T13)        t_dist = ⌊DX·tx + DY·ty⌋    (T12)
```

Two rescales, and fewer products than today (8 wide products and 4 `Wn·Fixed` against 16 wide products). **Saves 4
rescales per call: 131 → 72 steps (−59), measured.** Calls: 4 per point-tick (stage 2 of each substep; stage 5
reuses the cached separations, stage 0 never runs with the default `num_internal_stabilization_iterations = 1`).

**Generation's round trip must use the same function.** `split_point` stores `dist: sc.dist − n0`, where `n0` is the
separation of the generation pose computed by `round_trip`, so that the first refresh at that pose gives back
`sc.dist` exactly. SF1 (#191) was the defect of a round trip that did not match the refresh (exactly closed gaps went
soft). Fusing the refresh alone would bring that defect back; the prototype fuses both, with an identity pose for a
world end (exact: `⌊ONE·l⌋ = l`). For a world end the round trip costs a little more than today (today it skips the
transform), so the generation side of P2 is not counted in the estimates.

### P1 — the row solve (`split.cairo`: `solve_normal`, `solve_tangent`)

```
dv  = ⌊dx·(v1x − v2x) + dy·(v1y − v2y) + g1·w1 + g2·w2 + rhs⌋     jv_add, W7: 1 rescale
new = impulse − ⌊r·dv⌋                                             1 rescale, 1 checked subtraction
```

then `max(0, new)` (normal) or `clamp(new, −limit, limit)` (tangent). `dv` is used nowhere else. Fused:

```
new = ⌊impulse − r·(jv + rhs)⌋        lift(lift(impulse)) − W7·r: T8, 1 rescale
```

**Saves 1 rescale and the checked subtraction: 55 → 38 steps (−17), measured.** Calls: 8,012 on the reference shot
(HP, exact).

Note on rounding direction: today `impulse − ⌊r·dv⌋` rounds the new impulse *up* (a floored value is subtracted), the
fused form rounds it *down*. Both are within one ulp of the exact value of `impulse − r·(jv + rhs)` for the given
`dv`; the fused one is within one ulp of the exact value of the formula. A variant `impulse − ⌊r·(jv + rhs)⌋` keeps
today's direction and saves the same rescale but not the subtraction (≈ −11, not measured).

### P3 — the effective mass (`generation.cairo`: `coefficients`)

```
md = (⌊imx·dx⌋, ⌊imy·dy⌋)                          im_sum * dir: 2 rescales
k  = ⌊dx·mdx + dy·mdy + g1·ig1 + g2·ig2⌋            dot4: 1 rescale
r  = inv(k)
```

`md` is used nowhere else. Fused: `k = ⌊dx·dx·imx + dy·dy·imy + g1·ig1 + g2·ig2⌋` (two `W1·Fixed` and two lifted
products, T4). **Saves 2 rescales: 57 → 32 steps (−25), measured.** Calls: 2 per point-tick. (`ig1 = ii1·g1` could be
fused with `g1`'s `mul_sub` too, but `g1` is stored, so the count of rescales would not fall.)

### C5 — the manifold update (`manifold.cairo`: `update_candidate`, `normal_matches`), not prototyped

Per point, today: `local_p2 = pos12 · p2` (2 rescales), `dist = ⌊(local_p2 − p1) · n1⌋` (1), `new_p1 = local_p2 −
n1·dist` (2), then `delta = new_p1 − p1` and a wide norm test. `dist` is stored, `local_p2` and `new_p1` are not.
Fused: `dist` from the exact `pos12 · p2 − p1` at Q96.96 (1 rescale) and `delta` as one W5 sum per component
(2 rescales): 5 → 3 rescales and six checked subtractions absorbed, ≈ −45 steps per point *(est.)*. `normal_matches`:
`⌊n1 · (R12 · n2)⌋` against a tolerance, 3 → 1 rescale, ≈ −24 per manifold *(est.)*. Runs on every cuboid–cuboid pair
whose manifold is reused (the half-space and ball generators regenerate). Calls ≈ 0.7a points and ≈ 0.4a manifolds
*(est.)*. The accept / reject decision can flip at its thresholds, so this one changes results like the others.

### C7 — `Rot2::integrate` (`rapier_math/src/rot2.cairo`), not prototyped

`d = ⌊ω·dt⌋`, `re' = ⌊re − d·im⌋`, `im' = ⌊im + d·re⌋`, then `renormalize`. Fused: `re' = ⌊re − ω·dt·im⌋` (T2),
same for `im'`: saves `d`'s rescale, ≈ −12 steps per moving body and substep *(est.)*; calls ≈ 8 bodies × 4 substeps ×
25 ticks ≈ 800 on the reference shot *(est.)*. `Rot2Trait::integrate` is a public function of `rapier_math`.

### C6 — the half-space vertex (`halfspace_pfm.cairo`: `add_vertex`), not prototyped

`v = pos12 · vertex2` (2), `d = ⌊v · n1⌋` (1), `local_p1 = v − n1·d` (2): 5 → 3 the same way as C5, ≈ −37 steps per
vertex *(est.)*, two vertices per half-space–cuboid pair-tick (the ground), ≈ 225 on the reference shot *(est.)*. Also
bit-identical and separate: `vertex2 − n1_2 · border_radius2` computes two products by an exact zero for a cuboid.

### B1 — the solver contact anchors (`narrow_phase.cairo`: `solver_contact`), bit-identical, not prototyped

`anchor1 = transform(co1.pose, p1) − co1.world_com`: the subtraction of an exact `Fixed` after the floor equals its
subtraction inside the wide sum (`⌊x⌋ − c = ⌊x − c⌋` for an integer `c`), so folding `world_com` into `transform`'s sum
gives the **same bits** and drops one checked `Vec2` subtraction (HP: 12.7 steps) for ≈ 2 steps of lift and add, per
anchor, 2 anchors per solver contact-tick. Only the overflow panic moves (on an intermediate that today overflows and
whose difference would not), outside any real world.

### Ruled out or left out

- **The impulse application** (`split::apply`, six `mul_add` per impulse change, the first operation of HP's table):
  one rescale per output already. Keeping the two bodies' velocities wide across the rows of one kernel (2–4 rows) would
  save most of them (V1, a rough −0.2 to −0.45M on the reference shot), but the next row's `jv` would then sum
  `Wn·Fixed` terms beyond `fixed`'s 16-term bound, and P1's `r·(jv + rhs)` would become a quadruple product that does
  not fit a felt252. It needs a `fixed` change (an unbounded Q96.96 accumulator, glam track) and is not ranked here.
- **`generation::midpoint`**: `⌊(s − dir·shift)·½⌋` with `shift = ⌊d·dir⌋ − dist`; fusing `shift` makes the `½`
  a fourth factor. Narrowing at Q96.96 then halving saves nothing (the halving is a multiplication).
- **Restitution seed** `restitution · jv(..)` (2 → 1): new contacts only, negligible.
- **`pair_pose`, `Pose2` ops, the warm start, `jv_add` itself, `transform` alone**: one rescale per output already
  (BT3, M2).

## 3. Estimates per shot

Per call: the measured probe saving. Calls: §1. Per shot = per call × calls; owner = 2.95 × reference *(est.)*.

| # | Δ steps / call | reference calls | reference shot | % in process / slim | owner's shot | % in process / slim |
|---|--:|--:|--:|--:|--:|--:|
| P2 (refresh only) | −59 | 3,120 | −184k | −2.1 / −1.5 | −543k | −2.4 / −1.8 |
| P1 | −17 | 8,012 | −136k | −1.6 / −1.1 | −402k | −1.8 / −1.3 |
| P3 | −25 | 1,560 | −39k | −0.5 / −0.3 | −115k | −0.5 / −0.4 |
| **FU1 = P1 + P2 + P3** | | | **−359k** | **−4.2 / −2.9** | **−1.06M** | **−4.7 / −3.5** |
| C5 | ≈ −45 / point, −24 / manifold | ≈ 550 / 310 | ≈ −32k | | ≈ −95k | |
| C7 | ≈ −12 | ≈ 800 | ≈ −10k | | ≈ −37k | |
| C6 | ≈ −37 | ≈ 225 | ≈ −8k | | ≈ −25k | |
| B1 (bit-identical) | ≈ −10 / anchor | ≈ 1,560 | ≈ −16k | | ≈ −46k | |
| **all** | | | **≈ −0.42M** | ≈ −4.9 / −3.4 | **≈ −1.26M** | ≈ −5.6 / −4.1 |

**Calibration in the real solver.** The prototype of FU1 in the solver (§5) saves 190–192k steps on each 60-step
box_stack3 window: three boxes on the ground, three manifolds, ≈ 6 points, ≈ 360 point-ticks, so **≈ −530 steps per
point-tick** *(est.: the point count is not measured)*, against ≈ −460 to −490 predicted by the probes (10.3 row
solves × 17 + 4 refreshes × 59 + 2 × 25; the generation round trip and the compiler's handling of the inlined kernels
are not in the probes). With 530 per point-tick, FU1 is **−0.41M** on the reference shot (a ≈ 780) and **−1.22M** on
the owner's (a ≈ 2,300). The FU1 line above is the lower end, this the upper end *(est.)*.

**Proofs** (the game's basis, 154 L2 gas per step, IT1 §4 / EL1 §8): the owner's shot needs ≈ −1.47M more steps for 5
proofs after EL1. FU1 (−1.06 to −1.22M *(est.)*) does not reach it alone; FU1 with C5–C7 and B1 (≈ −1.26 to −1.42M)
comes within the uncertainty of the owner's extrapolation, still short on its central value. Only the Mac's whole-shot
run (§5) can say. The reference shot (3 proofs) does not
move. Every saving lowers the L2 gas of every shot.

## 4. Bit-identity

**None of P1, P2, P3, C5, C6, C7 is bit-identical**: one rounding instead of two or more moves the last bit whenever
the intermediate was not an integer. No intermediate of these formulas is exact in general (rotations, directions,
lever arms and velocities are arbitrary Q32.32 values). Two special cases are exact today and stay exact: a world end
(identity pose) in P2, and `cfm == ONE` (already skipped). **B1 and the zero-radius shortcut of C6 are bit-identical.**

### Size of the change, per formula

Measured on the scratch probes: 64 cases per formula checked bit for bit against an exact integer simulation (the
Cairo tests assert the simulated raw outputs), then 50,000 cases simulated with the same code. Inputs are realistic
pile10 ranges (unit directions, lever arms ±0.75, velocities ±15, translations 17–22 / 0–4, rotations ±0.5 rad, a
quarter of P2's cases against a world end). "exact" is the exact rational value of the formula on the same Q32.32
inputs; f64 rapier evaluates the same formula within 0.00006 ulp of it (measured on P1 over 20,000 cases), so it is the
f64 oracle of the formula.

| # | outputs | today vs exact: max / mean | fused vs exact: max / mean | fused vs today: max, share that differs | fused closer / farther (of those that differ) |
|---|--:|--:|--:|--:|--:|
| P1 | 50,000 | 2.46 / 0.89 ulp | **1.00 / 0.50** | 3 ulp, 100 % | 79 % / 21 % |
| P2 | 100,000 | 2.26 / 0.60 | **1.00 / 0.50** | 2 ulp, 37 % | 59 % / 41 % |
| P3 | 50,000 | 2.32 / 0.56 | **1.00 / 0.50** | 2 ulp, 16 % | 63 % / 37 % |

Per call, the fused result is always within one ulp of the oracle and closer to it on average; today's chain is up to
2.3–2.5 ulp off. P1 differs on every call because of the rounding direction (above).

### In a whole simulation

Prototype of FU1 in the solver (§5), every step of five golden scenes, 120 steps each, fused against today:

| scene | first step that differs | max \|fused − today\| over 120 steps: tx / ty / re / im (ulp) | vx / vy / ω (ulp) |
|---|--:|--:|--:|
| box_stack3 | 3 | 36 / 4 / 0 / 2 | 358 / 293 / 141 |
| box_slope_stick | 3 | 26 / 16 / 3 / 6 | 762 / 895 / 2,971 |
| box_slope_slide | 3 | 62 / 35 / 1 / 2 | 95 / 145 / 243 |
| ball_bounce | 34 | 0 / 2 / 0 / 0 | 0 / 2 / 0 |
| pendulum (joint, no contact) | — (identical) | 0 | 0 |

The change is small against the port's own distance to the oracle and against the bands (`golden_scenes.cairo`:
4,096 ulp per step for poses, twice that for velocities): 62 ulp is 1.4e-8 m. Against the f64 oracle (the scene
tests' own report, maximum over the samples, today → fused):

| scene window | tx | ty | im | vx | vy | ω |
|---|--:|--:|--:|--:|--:|--:|
| box_stack3 [0..60] | 405 → 377 | 15 → 16 | 41 → 41 | 545 → 390 | 852 → 628 | 346 → 223 |
| box_stack3 [60..120] (warm re-seed) | 3,092 → 3,088 | 405 → 408 | 428 → 427 | 21,099 → 21,084 | 1,297 → 1,311 | 10,044 → 10,039 |
| box_stack3_sleep [0..120] | 240 → 220 | 15 → 16 | 41 → 41 | 545 → 390 | 852 → 628 | 346 → 223 |
| box_slope_stick [0..120] | 192 → 200 | 108 → 113 | 8 → 14 | 1,850 → 2,612 | 1,597 → 2,492 | 5,105 → 8,076 |
| box_slope_slide [0..120] | 2,337 → 2,399 | 1,352 → 1,387 | 9 → 8 | 2,141 → 2,229 | 1,229 → 1,289 | 863 → 1,106 |
| ball_bounce, ball_drop, ball_drop_sleep, pendulum | equal (ball_bounce ty 128 → 130) | | | | | |

**Is the fused result closer to the oracle?** Per formula, yes (always within one ulp, today up to 2.5). Over a whole
simulation, not systematically: closer on the stack's first window (velocities −26 to −36 %), farther on the sticking
slope (velocities +41 to +58 %), about equal elsewhere. The scenes' distance to f64 rapier is dominated by the
trajectory's divergence (contact events, sleeping, friction cones), not by the last bit of each operation, so a
last-bit improvement does not carry through. Every sample stays within its band.

### What would move, and what it means here

- **Golden vectors** (rapier2d-f64 0.35.3 / parry2d-f64 0.30.2, `crates/rapier_golden`): the oracle does not change, so
  nothing is regenerated on its side; the port's results move against it and must stay within the documented bands. The
  21 scene tests run here pass; the other golden suites (levels, tilted landing, sleep / slope / stack diagnostics,
  composite, compound, CCD, kinematic scenes, `rapier_dynamics2d` `substep_scenes`, the contact-manifold families for
  C5 / C6) are to be run by the lot (CI's golden job).
- **Pinned digests** re-pinned by the lot: `rapier2d/tests/level_budget.cairo` `test_impact_digest_level{10,20}`,
  `rapier2d/tests/game_path.cairo` `GAME_DIGEST`, `rapier_sink/tests/split.cairo`, and any other test that pins a
  world-state digest (SF1 re-pinned the same set). Per-tick digest comparisons between layouts stay equal (both layouts
  run the same functions).
- **The solver's own equivalence checks**: `sweeps/checks.cairo`, `zero_checks.cairo` and `split/tests.cairo` compare
  the split sweeps bit for bit with `contact`'s reference sweeps (`contact::cached`). FU1 must fuse the reference the
  same way or turn those checks into comparisons with a fused reference.
- **Steps and sizes**: every `gas/**` snapshot of the touched modules, `gas/bytecode.size`, the class hashes
  (`rapier2d_classes/tests/hashes.cairo`, pinned from CI) and the declared-class felts (gate 73,728, to be re-measured:
  the fused forms have fewer instructions).
- **Versioning** (the domain rule: a last-bit change is a MINOR bump and a golden regeneration from the upstream
  oracle): the crates are `0.1.0-alpha.9`, and `crates/rapier_golden/README.md` says an alpha has no numeric stability
  and that from `0.1.0` on a numeric change is a MINOR bump. So FU1 ships in the next alpha with a CHANGELOG entry that
  says results change (as SF1 did in alpha.6), or in `0.2.0` if `0.1.0` is out by then. `fixed` and `glam_core` do not
  change (FU1 uses their existing API). P3 and C7 change public functions only through results (`Rot2Trait::integrate`
  is public in `rapier_math`).

## 5. The prototype

**Probes** (`fu0probe`, scratch package outside the repository, depending on `rapier_math` and `rapier_testing` by path
and on `fixed` 0.4.0 / `glam_core` 0.4.1): each formula's current form copied from `main` and its fused form, both
`#[inline(always)]`, one call on inputs routed through `rapier_testing::opaque`, net of a baseline test that reads the
same inputs. 12 tests, peak 1.35 GB.

| probe | baseline | current | fused | current net | fused net | Δ |
|---|--:|--:|--:|--:|--:|--:|
| P1 row solve | 89 | 144 | 127 | 55 | 38 | −17 |
| P2 refresh separation | 97 | 228 | 169 | 131 | 72 | −59 |
| P3 effective mass | 79 | 136 | 111 | 57 | 32 | −25 |

```sh
cd <scratch>/fu0probe && python3 gen.py 50000       # writes src/cases.cairo, prints the ulp statistics
RAYON_NUM_THREADS=1 prlimit --as=8589934592 -- /usr/bin/time -v \
    snforge test --tracked-resource cairo-steps --detailed-resources
```

**In the solver** (`fusedws`, a copy of `crates/{rapier_math, rapier_core, rapier_geometry2d, rapier_dynamics2d,
rapier2d, rapier_golden, rapier_testing}` with P1 + P2 + P3 applied to `split.cairo` and `generation.cairo`, ≈ 60
lines): two scratch packages hold `crates/rapier2d/tests/golden_scenes.cairo` and its `builder` / `stack_diagnostics`
modules (the other modules removed) plus a probe that prints every step's raw state; `scene_cur` depends on the
worktree's crates, `scene_fused` on the copy. `rapier_golden` is replaced by a copy holding only `compare`, `types`
and `generated::scenes`: the full crate (4.4 MB of generated vectors) made the build exceed the 8 GB cap (measured:
allocation failure at 5.1 GB resident). Peak with the reduced crate: 4.1 GB.

```sh
cd <scratch>/scene_cur   && RAYON_NUM_THREADS=1 prlimit --as=8589934592 -- /usr/bin/time -v \
    snforge test --max-threads 2 --detailed-resources
cd <scratch>/scene_fused && RAYON_NUM_THREADS=1 prlimit --as=8589934592 -- /usr/bin/time -v \
    snforge test --max-threads 2 --detailed-resources
```

Exact Cairo steps of each scene test, today → fused (the 21 tests; the four `gas_order_*` probes and
`test_stack_solve_order_is_colour_order` do not run the solver and are unchanged):

| test | today | fused | Δ | Δ % |
|---|--:|--:|--:|--:|
| test_box_stack3_first_window | 4,720,319 | 4,529,946 | −190,373 | −4.03 % |
| test_box_stack3_second_window | 4,670,865 | 4,479,128 | −191,737 | −4.10 % |
| test_box_stack3_sleep | 3,709,616 | 3,592,631 | −116,985 | −3.15 % |
| test_stack_first_steps | 3,139,867 | 3,047,927 | −91,940 | −2.93 % |
| test_stack_engine_is_colour_order | 1,514,556 | 1,453,384 | −61,172 | −4.04 % |
| test_stack_colour_order | 223,371 | 214,899 | −8,472 | −3.79 % |
| test_stack_colour_order_first_window | 4,776,239 | 4,585,819 | −190,420 | −3.99 % |
| test_stack_second_window_cold_colour | 4,756,231 | 4,563,829 | −192,402 | −4.05 % |
| test_stack_second_window_warm_colour | 4,771,633 | 4,579,896 | −191,737 | −4.02 % |
| test_stack_second_window_warm_pair | 4,653,857 | 4,461,803 | −192,054 | −4.13 % |
| test_box_slope_stick | 3,784,463 | 3,669,518 | −114,945 | −3.04 % |
| test_box_slope_slide | 4,108,959 | 3,992,101 | −116,858 | −2.84 % |
| test_ball_drop_sleep | 5,564,581 | 5,476,229 | −88,352 | −1.59 % |
| test_ball_drop | 2,764,439 | 2,720,674 | −43,765 | −1.58 % |
| test_ball_bounce | 1,185,389 | 1,182,430 | −2,959 | −0.25 % |
| test_pendulum | 4,504,980 | 4,504,980 | 0 | 0 |

Peak resident memory 3.4–4.1 GB per run. All 21 tests pass on both sides (the band checks included).

**To run on the Mac** (the whole-shot effect, which does not fit the VPS cap): the pile10 shots with FU1 applied,
in both layouts, against `main`, to measure the real steps saved, the digests and the per-tick drift:

```sh
export RAYON_NUM_THREADS=1
# FU1 applied in a worktree; probes of IT1 / EL1 (uncommitted it1.cairo / el1.cairo in rapier2d_classes/tests)
snforge test -p rapier2d_classes el1::el1_basic_owner_151 --include-ignored --tracked-resource cairo-steps \
    --detailed-resources --max-threads 2
snforge test -p rapier2d_classes el1::el1_slim_owner_151 --include-ignored --tracked-resource cairo-steps \
    --detailed-resources --max-threads 2
# reference shot likewise (_reference_107); the full golden suites:
snforge test -p rapier2d golden_scenes --max-threads 2
snforge test -p rapier_dynamics2d --max-threads 2
```

## 6. Recommendation

1. **FU1 — fuse P1, P2 and P3** (`impl-opus`, numerics). Files: `rapier_dynamics2d/src/solver/island/sweeps/split.cairo`,
   `split/generation.cairo`, the reference sweeps in `solver/contact/**` (or their checks), the pinned digests of §4,
   the snapshots, CHANGELOG (Unreleased: results change). Size: ≈ 60 lines of engine code (the prototype) plus the
   reference path, the re-pins and the probes; the unfused forms stay under `mod alternatives`. Gate: the Mac's
   whole-shot table (§5) before / after, in process and slim, with the class felts against 73,728. Estimate: **−0.36 to
   −0.41M steps on the reference shot, −1.06 to −1.22M on the owner's shot**, both layouts.
2. **Bundle the result-changing ones in one release.** C5 (manifold update), C6 (half-space vertex) and C7
   (`Rot2::integrate`) are smaller (≈ −0.16M together on the owner's shot *(est.)*) and touch `rapier_geometry2d` and
   `rapier_math`; if they are wanted, take them in FU1 or in a lot that lands before the same release, so that the game
   re-pins once. If not, FU1 alone.
3. **B1 (and C6's zero-radius shortcut) as a bit-identical lot** (`impl-sonnet` is enough): no digest moves; it can
   ship in any release.
4. **V1** (velocities kept wide across a kernel's rows) is the largest remaining lever of this kind but needs a
   `fixed` accumulator beyond 16 terms: a question for the glam track, after FU1.

**What a game re-pins** after FU1: the class hashes of every declared class whose code includes the solver or the
generation (`SolveAdvanceClass`, and the world / caller classes that embed them; C5 / C6 add `NarrowPhaseClass`),
its `WorldState` digests and replay digests (slingfall's golden cases), and its cost sheet (steps and proofs per
shot). The `WorldState` format does not change.

## 7. Measurement material (uncommitted)

In the session scratchpad, never committed: `fu0probe/` (`gen.py`, `src/lib.cairo`, `src/tests.cairo`,
`src/cases.cairo`), `fusedws/` (the patched crates), `scene_cur/`, `scene_fused/`, `golden_mini/`, and the logs
`scene_cur*.log`, `scene_fused*.log`.
