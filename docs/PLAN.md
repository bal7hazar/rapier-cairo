# rapier.cairo — execution plan

Status: **v2.24, 2026-09-24** (v2: scalar delegated to glam.cairo's `fixed`; v2.1: wave 1 merged; v2.2: `fixed` consumed, C2 + M3 merged; v2.3: C3 + G2 merged; v2.4: glam `Vec2` consumed, M2 + F3 merged, wave 3 launched; v2.5: wave 3 merged, wave 4 in progress; v2.6: DB, DE, GH merged, orchestrator moved to a new machine, rest of wave 4 launched; v2.7: G3, GF1, GF2, GF4, DD merged, GF3 running, DF launched; v2.8: GF3, DF merged, GG running, wave-5 stubs + P1 brief; v2.9: GG merged, wave 4 complete, P1 launched; v2.10: P1 merged, prelude, GM/P2/P3/P4 launched; v2.11: GM, P2, P3 merged, SD launched, prover finding; v2.12: SD, P4 merged, DM launched, nightly execute job; v2.13: paused, DM wip pushed; v2.14: resumed, BX/GS briefs, glam 0.3.0 plan; v2.15: DM, BX merged, GS + OS running; v2.16: GS merged, OP launched; v2.17: OS merged, BP launched; v2.18: OP merged, OI launched; v2.19: OI merged, budgets refreshed, SO launched; v2.20: BP merged, OJ launched; v2.21: SO merged, D8 amended, DO launched; v2.22: OJ merged, BG launched; v2.23: DO merged, ON launched; v2.24: BG merged, budgets and ceilings refreshed). Owner of this file: the orchestrator session (see [`AGENTS.md`](../AGENTS.md)).

Goal: a Cairo port of [Rapier](https://github.com/dimforge/rapier) good enough to build a complete
game whose physics is provable, with gas tracked per feature from the first line of code.

## 1. What the research established

Four reports back this plan. Numbers below are measured unless marked *(est.)*.

| Report | Headline findings |
|---|---|
| [01 — Rapier](research/01-rapier-analysis.md) (v0.35.3, 80k lines) | Maths is now **glam** (`Vec2`, `Rot2` = unit complex, `Pose2`), nalgebra only for multibody/SIMD. Solver is a substepped soft-constraint PGS ("TGS-soft", 4 substeps). **No trig, `exp` or `pow` anywhere in the step**: only `+ - * / sqrt min max clamp abs`. After cutting soft bodies (26k), multibody (6.7k), parallel/SIMD (6k), persistent islands, serde, debug… **6–8k lines of logic remain, < 2k numerically essential.** Solver sweeps are the #1 cost: `O(substeps × manifolds × points)`. |
| [02 — Parry](research/02-parry-analysis.md) (v0.31.1, 80k lines) | The step consumes only `Aabb`, `compute_aabb`, `mass_properties`, `contact_manifolds` and the manifold types. With Ball, Cuboid, Capsule, HalfSpace, Segment (+ convex polygon via SAT) **the whole 2D pair matrix needs neither GJK nor EPA** (EPA is the biggest fixed-point risk upstream). The BVH broad phase is a poor fit for Cairo; brute-force AABB (≤ ~40 bodies) then stateless sort-and-prune, no `Felt252Dict`. MVP ≈ 3.5–4k Cairo lines *(est.)*. |
| [03 — Cairo ecosystem](research/03-cairo-ecosystem-architecture.md) | alexandria: only repo with gas regression, but non-blocking and noisy. origami: best efficiency idioms (mul/div packing, const tables, unrolling), no gas tracking, its algebra crate is not reusable. starknet-agentic: optimisation/testing rules and the `AGENTS.md` coordinator/executor model. snforge has **no built-in gas snapshot** → `scripts/gas.py`. |
| [04 — Numeric benchmark](research/04-numeric-benchmark.md) (415 probes) | Owner heuristic **confirmed**: DivRem ≈ bitwise in steps but bitwise costs 12–55 % more gas; loops cost 13–140× more; BoundedInt is cheaper still. **Q32.32 in a native `i64`** wins: constant-cost add/sub/lt (6 steps), mul 16 steps, no negative zero (cubit has one). **Fused multiply-accumulate** (one rescale per output): dot2 18 steps vs 69 (cubit), mat3·vec3 82 vs 344, Vec2 length 17 vs 109. Core `u128_sqrt` = 10 steps vs Newton 231. Polynomial sin 144 steps vs cubit 1051. **Orion rejected**: its FP32x32 *is* cubit, its tensors cost 3–16× struct code. Q16.16 and Q8.23 overflow; Q64.64 costs 3–12×. |

The `glam.cairo` and `nalgebra.cairo` sessions reached the same scalar conclusion independently
(Q32.32 in `i64`, bias-trick multiplication, fused accumulation).

## 2. Decisions

Each becomes a short ADR in `docs/adr/` when first implemented.

| # | Decision | Why |
|---|---|---|
| D1 | Scalar = signed **Q32.32 in `struct { raw: i64 }`**, provided by glam.cairo's `fixed` (D12); mul via BoundedInt bias trick; `sqrt` via core `u128_sqrt`; `inv(0) = 0` as upstream | Report 04; range ±2.1e9, resolution 2.3e-10 |
| D2 | **Fused kernels are the unit of design**, not scalar ops: dot, cross, `Rot2·Vec2`, `Pose2·Point`, constraint rows `J·v` accumulate in wide form and rescale once | 3–6× cheaper than composing scalar ops |
| D3 | ~~Dedicated scale for extreme-range quantities~~ **Not needed for the spring coefficients**: C2 reproduces every `IntegrationParameters`-derived value within 1 ulp in plain Q32.32 (joint `cfm_coeff` ≈ 6 ulp of resolution but correct). Re-evaluate only for inverse inertia in DA | Measured in C2 (PR #10) |
| D4 | `Real::MAX` sentinels: kept as `fixed::MAX` where the consumer guards against multiplication (velocity caps, C2), replaced by `Option`/flags elsewhere; rigid joints (`cfm ≈ 1.5e-9` upstream) are special-cased | Would overflow or underflow in fixed point |
| D5 | **2D first**, 3D only after 2D is benchmarked (3D ≈ 4–6× gas per contact *(est.)*). Dimension-specific crates (`*2d`, later `*3d`) over a shared dimension-agnostic core — no mutually exclusive Scarb features | Keeps `--workspace` builds and the snapshot simple |
| D6 | Shapes are a **closed enum**; no `dyn`, no `SharedShape` | Cairo has no trait objects; `match` dispatch is cheap |
| D7 | Broad phase is **stateless**: brute-force AABB with static/dynamic split, sort-and-prune as the measured alternative | Nothing to persist, no dict, trivially deterministic |
| D8 | Solver = upstream soft-step algorithm, scalar, sequential Gauss–Seidel; contact order = **stable partition of the ascending pair list: pairs of two non-fixed bodies first, pairs with a fixed/world body last** (amended 2026-09-24 after SO: upstream colours touching pairs — non-fixed pairs take the lowest free colour, pairs with a fixed body the highest — and solves colours in ascending order; the partition reproduces that order whenever the non-fixed pairs' colours ascend in pair order, e.g. every golden scene; exact parity needs persisted colours, deferred); `num_solver_iterations` exposed (main gas/quality knob); velocities gathered once per manifold, scattered once | Order is part of the state-transition function; SO measured +27k gas per step for the partition vs +209k for greedy colouring per step |
| D9 | Per-step scratch is never persisted. Persistent world state = poses, velocities, warm-start impulses, (later) sleep timers | On Starknet, state I/O rivals compute |
| D10 | Core crates are **pure Cairo** (no `starknet` dep); Starknet/Dojo storage packing lives in a separate adapter crate | Usable from contracts, Dojo and `scarb execute` / `scarb prove` |
| D11 | Validation against upstream through **golden vectors** generated by a Rust harness (`rapier2d-f64` / `parry2d-f64`, with recycling, clustering, block solver, CCD and sleeping disabled); tolerance-based, never bit-exact | Rust's solve order comes from graph colouring |
| D12 | **The scalar is not implemented here.** `glam.cairo` ships a `fixed` package (Q32.32 `i64`, wide accumulators, trig) explicitly shared by the glam, nalgebra and rapier ports; rapier.cairo consumes it (git dependency pinned by rev) together with glam's `Vec2`. `rapier_math` plays the role of upstream's `glamx`: `Rot2`, `Pose2` and `math_ext` on top of glam, contributed back if glam.cairo wants them (its plan lists them as late item P1) | Three ports with three scalars would diverge in rounding and make golden vectors incomparable |

## 3. Architecture

```
rapier_testing     dev-only: opaque(), approx asserts, fixtures
rapier_math        (glamx) Rot2, Pose2, fused kernels, math_ext — on glam.cairo `fixed` + `Vec2` (D12)
rapier_core        dimension-agnostic: handles + arena, union-find, interaction groups,
                   IntegrationParameters, SpringCoefficients, event types
rapier_geometry2d  (Parry) aabb, shape enum, mass properties, point/segment queries, clip, SAT,
                   contact manifolds, per-pair generators, dispatch, broad phase
rapier_dynamics2d  rigid body + collider sets, narrow-phase bookkeeping, joints,
                   contact + joint solver, substep loop, (later) islands/sleep
rapier2d           World, step(), events, queries; public facade and prelude
rapier_starknet    (phase 2) storage packing / Dojo models for the persistent state
tools/golden       Rust harness producing golden vectors → generated Cairo fixtures
examples/          `#[executable]` scenes for `scarb execute` + `scarb prove`
```

Strict DAG: `testing ← math ← core ← geometry2d ← dynamics2d ← rapier2d`. `rapier_geometry2d`
never depends on dynamics, so it can be extracted as `parry.cairo` later at no cost.

## 4. Phases and work packages

Notation: **[P]** parallelisable inside its wave, **[S]** serial (orchestrator). Every package
ships `test_*`, `gas_*` (one per candidate implementation) and docs per `AGENTS.md` §5.

### Phase 0 — Foundations *(done in PR #1 unless noted)*

| ID | Package | Output |
|---|---|---|
| F0 [S] | Workspace, toolchain pin, `scripts/gas.py`, CI (fmt, lint, build, test, gas check) | ✅ |
| F1 [S] | `AGENTS.md`, `CLAUDE.md`, research reports, this plan | ✅ |
| F2 [S] | Agree the `fixed`/`Vec2` API with glam.cairo (§6) and freeze `rapier_math` signatures (`Rot2`, `Pose2`, `math_ext`) | next |

### Phase 1 — 2D MVP: "a box stack settles, and the step is proven"

**Wave 1 — independent of the scalar** ✅ merged (PR #3, #4)

| ID | Package | Upstream reference | Acceptance |
|---|---|---|---|
| C1 [P] | `rapier_core::data`: generational handles, arena (candidates: `Felt252Dict` vs `Array` rebuild), union-find, interaction groups (math vs bitwise candidates) | `src/data`, `geometry/interaction_groups.rs` | handle reuse/generation tests, candidates ranked |
| G1 [P] ✅ (PR #7) | Scene traces as Cairo fixtures | — | 10 sanity tests replaying the traces |
| C3 [P] ✅ (PR #13) | Scalar-only body/collider components: `RigidBodyType/Damping/Dominance/Activation`, `ColliderMaterial`, `CoefficientCombineRule` (6 rules), `ActiveCollisionTypes`, flags | `rigid_body_components.rs`, `collider_components.rs` | damping ≤ 1 ulp; flag ops ranked |
| G2 [P] ✅ (PR #14) | Leaf golden vectors: `pose2`, `aabb_overlap`, `sat2d`, `clip2d`, `point_projection`, `segment_segment` | parry `query::{sat,details}` | 63 sanity tests; upstream quirks in `tools/golden/README.md` |
| G0 [P] | `tools/golden`: Rust harness + fixture generator; leaf vectors (mass props, AABB, spring coefficients, each contact pair over 6 regimes: separated, within prediction, touching, shallow, deep, degenerate) and scene traces. Inputs quantised to Q32.32, values emitted as raw `i64` so fixtures do not depend on the scalar crate | report 01 §8, report 02 §8 | fixtures committed, regeneration documented |

**Wave 1 outcomes that constrain later packages**

- Golden vectors pin `rapier2d-f64 =0.35.3` with `parry2d-f64 =0.30.2` (the Parry that the published
  Rapier actually depends on; report 02 analysed 0.31.1, differences are not relevant to the MVP).
- Upstream cuboid **feature ids are wrong in f64 builds** (bit 31 of the float is read); fixtures carry
  ids from an f32 run. The Cairo port must follow the f32 semantics.
- Convex pairs always return one manifold, possibly with 0 points; clipped points beyond the
  prediction distance are kept; ball–ball tests `<` where other generators test `<=`; normals of
  exactly-degenerate configurations are unreliable upstream (cases tagged `ambiguous`, excluded from
  strict comparison).
- Joint `cfm_coeff` is ~6 ulp in Q32.32 → confirms D4 (rigid joints special-cased) and D3.
- No rotation other than multiples of 90° is exactly unit in Q32.32 → `Rot2` renormalisation policy
  is part of M2's acceptance.
- `rapier_core`: `Felt252Dict` arena is O(1) per mutation (array rebuild is O(capacity)), but arrays
  are 15–25 % cheaper for read-only and bulk passes → solver scratch data should be dense arrays
  built once per step, the arena is for the persistent sets.

**Gate X1 — external:** ✅ `fixed` F1 + F2 merged in glam.cairo and consumed (PR #8). **Still open: `Vec2` (glam item V2, status todo)** — the only remaining blocker for M2 and wave 3. Original wording: `glam.cairo` merges `fixed` F1 + F2 (scalar + wide accumulators) and `Vec2`
(its items F1, F2, V2). Its `docs/DESIGN.md` already fixes what we need: `fixed::Fixed { raw: i64 }`,
**floor** rounding on every rescale, panicking `recip(0)` (so `inv(0) = 0` stays in
`rapier_math::math_ext`), public wide accumulators. Only these three items block rapier.cairo; nalgebra.cairo blocks nothing.
If the gate slips, fallback: vendor a snapshot of `fixed` from the glam.cairo branch under
`crates/` and swap it for the git dependency later (same code, so no semantic drift).

**Wave 2** (needs X1) — C2 ✅ (PR #10), M3 ✅ (PR #11), M2 ✅ (PR #21, codex gpt-6-astra: fused kernels win everywhere, renormalise once per substep), F3 ✅ as code (PR #19)

| ID | Package | Acceptance |
|---|---|---|
| M2 [P] | `rapier_math`: `Rot2`, `Pose2` with fused kernels (rotate, transform, inverse-transform, `inv_mul`/`pos12`, renormalise) on glam's `Vec2` and `fixed::wide` | fused vs composed candidates ranked |
| M3 [P] | `math_ext`: `gcross` family, `inv` (0 → 0), tolerances expressed in ulps, **wide comparisons of squared quantities** (compare the unscaled product against a pre-scaled constant) | report 02 §6 hazards covered by tests |
| C2 [P] | `IntegrationParameters`, `SpringCoefficients` (`erp_inv_dt`, `cfm_factor`, …); settles D3 (dedicated scale for inverse inertia / `cfm`) by measurement | matches G0 golden values |
| F3 [S] | Freeze geometry↔dynamics interface: `ContactManifold`, `TrackedContact`, feature ids, `SolverContact`, `MassProperties` | merged before wave 3 |

Trig (`sin_cos`, `atan2`) comes from `fixed::trig` (glam item F3); it is only needed in phase 2.

**Wave 3** ✅ merged 2026-09-20 (GE #23, GA #24, GD #25, DC #26, DA #27, GB #28, GC #30; 1 091 tests on main). Findings: brute-force broad phase wins up to n = 64 but costs ~7k gas per pair test (optimisation candidate); `try_update_contacts` fast path 75k; SAT+clip contact ≈ 150k; solver 153k per manifold per pass (1 point), 233k (2 points) with gather/scatter; DC warns that scattering into an immutable `Array` is O(bodies) per manifold → DF benches a dict-backed store; free-fall step of one body 222k. Launched 2026-09-20 (stubs pre-declared in PR #20, briefs in `docs/briefs/`): GA codex gpt-5.5, GB claude sonnet, GC claude opus, GD codex gpt-6-astra, GE codex gpt-5.5, DA claude opus, DC codex gpt-6-astra xhigh. DB moved to wave 4 (needs GB's `Shape`).

| ID | Package | Upstream reference |
|---|---|---|
| GA [P] | `Aabb` + broad phase (brute force vs sort-and-prune candidates, static/dynamic split) | `bounding_volume/aabb.rs` |
| GB [P] | Shape enum (Ball, Cuboid, Capsule, HalfSpace, Segment), `compute_aabb`, mass properties | `shape/`, `mass_properties/` |
| GC [P] | Point projection, segment–segment closest points | `query/point`, `query/closest_points` |
| GD [P] | Segment clipping, 2D SAT (cuboid/cuboid, support scans) | `query/clip`, `query/sat` |
| GE [P] | Manifold persistence: `try_update_contacts` fast path, `match_contacts` by feature id | `contact_manifolds/contact_manifold.rs` |
| DA [P] | Rigid body components, forces/impulses API, world mass properties, damping `1/(1+dt·d)`, `integrate_linearized`, locked axes | `dynamics/rigid_body_components.rs` |
| DB [P] | Collider components, material + combine rules, sensors flag | `geometry/collider_components.rs` |
| DC [P] | Contact solver on **mock manifolds**: generate, update, warm start, solve (biased + relax), friction, restitution pass, writeback | `solver/contact_constraint/*` |

**Wave 4** (needs wave 3) — briefs in `docs/briefs/`. Merged: DE ✅ (PR #32, fixed/revolute/prismatic
joint solver), DB ✅ (PR #33, collider components + builder; finding: an inlined computing `match` arm
is charged to every arm, 49k vs 21k → `AGENTS.md` §7), GH ✅ (PR #35, wave-3 shims deleted: one `Aabb`,
one shape set, one `PolygonalFeature`); 1 355 tests on main. Launch plan from 2026-09-21, in parallel: GF1
(claude sonnet), GF2 (codex gpt-6-astra high), GF3 (claude opus), GF4 (codex gpt-5.5 high), DD (claude
opus). Each GF package owns its own
`gas/rapier_geometry2d/contact_generators.<child>.snap` (`SPLIT_MODULES` in `scripts/gas.py`), and DD
is generic over a `ContactDispatcher` trait (upstream's `&dyn PersistentQueryDispatcher`) so it does
not wait for GG. Then GG after GF1–4 and DF after DD (DF must bench a `Felt252Dict`-backed body store against
the immutable-`Array` scatter, see wave-3 findings). Still-empty pre-declared stubs:
`rapier_geometry2d::{contact_generators::*, dispatch}`, `rapier_dynamics2d::{collider_set,
rigid_body_set, narrow_phase, events, solver::{island, body_store}}`.

G3 (added 2026-09-21, brief `g3-manifold-golden-gaps.md`, codex gpt-5.5 high): GF4 found no golden
vectors for halfspace–capsule, halfspace–segment and cuboid–segment (analytic tests only); G3 appends
21 cases to the `contact_manifolds` family, then GF4's golden test file is extended.

DF ✅ (#46, codex gpt-6-astra xhigh, 2026-09-22): `solve_island` in upstream's `run_worker` order over a
dense `Felt252Dict` body store (wins every size: stack 5 = 22.8M gas | 206k steps per step vs 27.6M |
253k with the array store; stack 16 = 72.6M vs 121.6M). One step, 4 substeps: ball drop 4.7M | 39k,
pendulum 5.4M | 45k, slope 5.6M | 50k, stack 3 = 13.8M | 124k; contact sweeps ≈ 80 % of a stack step.
Wave 5 prepared: crate `rapier2d` pre-declared (`world`, `pipeline`, `dispatcher`, three test stubs),
brief `p1-world-step.md`; P2–P4 briefs follow P1's API.

BG ✅ (#81, codex then claude opus after the codex quota ran out): `find_pairs` dispatches on n — tail
scan below 32, four-unit x strips for 32–63, a four-unit 2D `Felt252Dict` grid from 64 (wide/static boxes
tested separately, descending insertion keeps the ascending output without a global sort; crowded cells
fall back to a scan); 354 gas per pair test at 256 sparse bodies (tail scan 4 171), linear growth; free
fall 32 −1.41M. Caveat: the 4-unit cell suits metre-scale worlds — follow-up: derive it from
`IntegrationParameters::length_unit`. Budgets refreshed and every P3 ceiling reset to +10 % of the
current gross (`docs/BUDGETS.md`): since 09-22 stacks −30 %, free fall −44 to −50 %, joints −27 %.

DO ✅ (#80, claude sonnet): D8 as amended — touching manifolds solved as a stable partition (non-fixed
pairs first, pairs with a fixed or absent body last; kinematic bodies count as non-fixed, as upstream's
`RigidBody::is_fixed`); `box_stack3` now passes the **strict** per-sample comparison in both windows (max
404 / 909 ulp over 0–60); measured cost +0.85 % per step (stack 3 +109k, higher than SO's +27k probe:
per-pair visits). Codex quota exhausted on 2026-09-24 until 09-27 15:14 → lots run on claude meanwhile
(BG handed over). Launched **ON** (brief `on-narrow-phase.md`, claude opus): dedupe the narrow-phase
loop copied into `rapier2d` by OP, then −25 % per pair.

OJ ✅ (#78, codex gpt-6-astra high): two-row joints no longer pay the three-row Gram–Schmidt (222k → 83k),
inlined scalar row kernels, impulse-only copies on seeding/writeback: pendulum chain 3 −25.2 % gas / −17.3 %
steps (13.35M → 9.99M), chain 1 4.83M → 3.70M; every other scene unchanged; bit-identical. Follow-up
(orchestrator): lower the chain P3 ceilings. Launched **BG** (brief `bg-broad-phase-grid.md`): grid / radix
broad phase for large worlds.

SO ✅ (#76, claude opus): `box_stack3`'s strict divergence is the **solve order** — manifolds, feature ids,
solver contacts and NEW status agree at the first diverging step (step 4); only the ground impulse differs
(1 622 403 951 vs 1 682 864 324 raw) because upstream solves the ground pair last (colour 127) and the
port first. Upstream's order: 0 violations over steps 0–60 (max 404 / 909 ulp), 0 over 60–120 once the
re-seed restores upstream impulses. Decision: D8 amended (fixed-last stable partition, +27k gas), lot
**DO** (brief `do-solve-order.md`) implements it and moves `box_stack3` to strict comparison.

BP ✅ (#72, codex, after one review round): `find_pairs` = tail scan (`pop_front`, hoisted `a`, static-static
skip, short-circuit raw overlap), ranked on shuffled proxies: 5.4k gas per non-overlapping pair test (was
~7k: each test copied a whole proxy and bounds-checked); free fall 32 −2.15M; struct-of-arrays and metered
append lose (construction cost), sort-and-prune with a real merge sort wins only on sparse sets and
collapses on stacks. The 2k/pair target needs a grid or radix (later). Launched **OJ** (brief
`oj-joint-solver.md`): joints cost ≈ 4.3M per joint per step.

OI ✅ (#73, claude opus): bodies referenced by no manifold/joint are solved alone (same expressions, no
dict, 4 substeps unrolled) and the solve is fused with the position update: free fall −31 % gas / −33 %
steps at 32 bodies (16.1M | 142k); every scene cheaper; P3 probes now have uncapped `steps_step_*` twins
(`docs/BUDGETS.md` refreshed: since 09-22, stacks −30 %, free fall −39 to −50 %, joints −3 to −5 %).
Rejected: carrying world mass across steps (inexact after a flagless `World::set_body`; would need
change flags on every body edit). BP (#72) sent back: its winner was a benchmark-shaped fast path
(sorted-and-disjoint input); rework on shuffled inputs and per-pair-test cost. Launched **SO** (brief
`so-stack-divergence.md`, claude opus): why `box_stack3` still fails the strict comparison.

OP ✅ (#69, claude opus): `compute_aabb` inlined (outlined, every collider paid the costliest shape arm)
and one walk per set per step; free fall −10 to −12 % per body (32 bodies 26.2M → 23.3M), stack 3 −2.7 %
(12.87M). Note: `rapier2d::pipeline::contacts_from_scratch` duplicates DD's `compute_contacts_with`
(narrow_phase was out of OP's scope) → dedupe in a later lot. Launched **OI** (brief
`oi-idle-body-solver.md`, claude opus): the solver's 430k per unconstrained body + P3 step companions.

OS ✅ (#68, codex gpt-6-astra xhigh): solver sweeps −28.5 % on a settled stack-3 step (18.50M → 13.23M
Sierra gas, 108k steps); stack 10 62.3M → 44.5M, mixed pile 8 68.8M → 49.0M, balls 8 36.7M → 28.1M;
2-point contact over a full step 1.05M → 0.63M. Winners: cached `dir * im` products, exact-zero guards,
separate joint stages, an inert-contact driver; rejected (kept under alternatives): metered arms, dict
constraint storage, hot/cold split. Bit-identical. Open: P3's `#[available_gas]` ceilings block
`--tracked-resource cairo-steps` runs of some probes (need step companions and lower ceilings).
OP (#69, per-body overhead, −11 % per free body, rebasing on OS) escalated: the solver still costs 430k per
body with no constraint (fuse `to_bodies` with the position update, reuse world mass) and `find_pairs`
is O(n²) at ~6.2k per pair test → lot **BP** (brief `bp-broad-phase-pairs.md`, metered brute force /
static split / sort-and-prune re-rank).

GS ✅ (#66, claude opus): scene traces regenerated from a vendored `parry2d-f64 0.30.2` with correct f64
cuboid feature ids (`>> 63 / >> 62`; 2.9 MB under `tools/golden/vendor/`); ball_drop, ball_bounce, pendulum
unchanged; **box_slope_stick passes** all 22 samples; **box_slope_slide** stays `#[ignore]`d on an exact tie:
at step 4, substep 1 the second point's refreshed gap is exactly `0` raw, so the port takes upstream's
`dist <= 0` soft branch where the f64 residue fell on the rigid side (solving that one row rigidly passes
every sample) — a legitimate fixed-vs-float divergence; **box_stack3** still fails the strict per-sample
comparison (passes the rest invariants): candidate cause = constraint order (D8 pair order vs upstream's
interaction-graph order), to investigate later. Launched: **OP** (per-body pipeline overhead, brief
`op-pipeline-per-body.md`, claude opus) next to OS.

DM ✅ (#63, claude opus): common-midpoint lever arms as upstream — slope step-3 tangent inverse effective
mass 1708349406 vs upstream 1708349407 raw (was 1717986916); +5.5k gas per generated point. BX ✅ (#64,
codex): `fixed`/`glam` 0.3.0 from scarbs.xyz; numeric fallout = 11 expectations moved by 1 ulp (damping,
springs, ratio, averages), `CoefficientCombineRule::Average` no longer halves with a truncating `DivRem`,
flaky mass fuzz fixed; step budgets +0.4 % (contacts) to +1.4 % (joints), division-heavy probes +25 %.
Launched: GS (slope traces, claude opus) and **OS** (solver sweeps, brief `os-solver-sweeps.md`, codex
gpt-6-astra xhigh: per-pass `match stage`, per-manifold `Array` scratch and per-pass constraint-array
rebuild are the measured suspects; bit-identical results, target −20 % on stack 3).

**Resumed 2026-09-23 evening.** Sibling news: glam.cairo released `fixed`, `glam`, `glamx` 0.3.0 on
scarbs.xyz (`fixed`: `/`, `recip`, `from_ratio` now round to nearest-even; `wide::Acc`, `wide::RecipNearest`);
nalgebra.cairo dropped its own scalar and is generic over `simba::Real` for `fixed::Fixed` (rapier needs
nothing from it before its 0.1.0; multibody stays out of scope). `glamx` 0.3.0 has `Rot2`, `Pose3`,
`SdpMatrix2/3` but **not `Pose2`** yet → `rapier_math::{rot2, pose2}` stay for now (escalation to
glam.cairo: port `Pose2`, rapier's M2 kernels available). Sequence: DM (resumed) → **BX** (registry
0.3.0, every snapshot regenerated, delta reviewed alone; brief `bx-fixed-0.3.md`, includes the flaky
mass fuzz) → **GS** (slope traces from a patched f64 parry with correct cuboid feature ids; brief
`gs-slope-traces.md`) → optimisation lots from `docs/BUDGETS.md`. At most two rapier executors at a
time (shared machine hit its CPU ceiling on 2026-09-23).

**Paused 2026-09-23 (owner's decision: the shared machine hit its CPU ceiling overnight; glam.cairo has
priority).** No executor running, no open PR. Resume point: lot DM (`feat/dm-midpoint-anchors`, three
`wip:` commits pushed: midpoint lever arms in `solver/contact.cairo`, substep mock witnesses, slope
diagnostics) — resume with `scripts/executor-unit.sh resume dm-midpoint-anchors claude:opus "<follow-up>"`
(the worktree `.claude/worktrees/exec-dm-midpoint-anchors` still exists) or relaunch from the branch;
then GS (regenerate slope traces with correct feature ids, un-ignore the slope tests), MF (flaky
`mass.cairo:520` fuzz), then optimisation lots from `docs/BUDGETS.md`.

P4 ✅ (#57, execute-only): `examples/ball_drop` (`#[executable]`, scenes ball_drop / box_stack3 / pendulum,
standalone package outside the workspace because of `enable-gas = false`), `scripts/prove-example.sh`
(execute always; prove + verify behind `PROVE=1` and the build lock), nightly job `.github/workflows/execute.yml`.
`scarb execute`: 10 steps of ball drop = 143 976 Cairo steps (14.4k per step, matches P1's 14.7k), 60 steps
= 1.58M (contact from step ~50). **Milestone "a proven step" is blocked on hardware** (Stwo > 22 GB).
SD ✅ (#56, codex gpt-6-astra xhigh): slope divergence explained — (1) the port takes each contact's lever
arms from its own surface point while upstream freezes a **common midpoint** for both bodies
(`pair_update.rs` "Localize solver contacts"): tangent inverse effective mass 1717986916 vs 1708349407
raw; the midpoint alone brings step-3 velocity errors from ~2.5e6 to < 700 ulp → lot DM (brief
`dm-midpoint-anchors.md`); (2) from step 4 the **upstream f64 trace is itself wrong** (duplicate cuboid
feature ids warm-start both regenerated points): the slope references must be regenerated with correct
ids (f32 engine or patched ids) → lot GS after DM. Also: one flaky `mass.cairo:520` fuzz
counterexample (30447 vs 30441 raw) to investigate.

Wave 5 (2026-09-22/23): GM ✅ (#54, one metered dispatcher in `rapier_geometry2d::dispatch`, P1's numbers
hold: 4 ball pairs 1.49M, 4 cuboid pairs 3.20M). P3 ✅ (#52, `docs/BUDGETS.md`: one settled step = free
fall 0.81M gas | 7.3k steps per body, ball on ground 4.6M | 38k per body, cuboid stack 3.1M | 26k per
manifold point, joint 4.3M | 35k; `#[available_gas]` ceilings at +10 %; targets: solver sweeps 81 %,
narrow phase 13.5 %, proxies 1.9 %). P2 ✅ (#53, six traces through `World::step` over 120 steps:
ball_drop, ball_bounce, pendulum, box_stack3 pass — stack judged on rest invariants, two 60-step
windows because of the VM step budget; **box_slope_stick / box_slope_slide diverge at step 3, the
first contact step** (stick: constant offset along the slope; slide: +3.4e4 ulp per step while
step-end velocities agree) → `#[ignore]`d, lot SD (brief `sd-slope-divergence.md`) instruments the
Rust harness and locates the cause). P4 (execute-only, see below) in progress. **Prover finding:**
`scarb prove` (Stwo) is OOM-killed above a 22 GB cap even for one step (4 781 Cairo steps,
`prover_input.json` 105 MB): the proven-step milestone needs a ≥ 64 GB machine; CI runs
`scarb execute` only. Compile budget: CI `test` job reached 5 min with P2/P3 — no new integration
test file without removing one.

P1 ✅ (#50, claude opus, 116 turns): `rapier2d::World` + `step()` in upstream order, events as an array,
`prelude`. One step: free fall 1.76M gas | 14.7k steps · ball resting 5.15M | 42.7k · BOX_STACK3
17.9M | 151k (solver 81 %, narrow phase 13.5 %) · PENDULUM 5.06M | 41.6k. **Finding: the "metered"
dispatcher** — each generator call wrapped in a one-iteration `while` inside the `match` — makes an
outlined caller pay only the reached generator (4 ball pairs 1.49M vs 3.36M with GG's plain inlined
match, cuboids 3.20M), at ~265 steps per pair; rule recorded in `AGENTS.md` §7. Fused user-changes
pass 262k vs 500k; touching-only manifolds to the solver; static-proxy cache rejected (D9, +5 % on a
stack, −28 % only with many statics). Follow-ups: GM (move the metering into `dispatch`, P1's
escalation A), P2/P3/P4 briefs written 2026-09-22.

GG ✅ (#48, claude sonnet): dispatcher = one inlined typed `match`, 87 golden cases pass in both orders;
overhead 1.2–6.9k gas | 12–69 steps per pair. **Finding: the dispatcher must stay `#[inline(always)]`**
— outlined, every pair is charged the most expensive arm (flat 556k, ball–ball included), and
`#[inline(never)]` per-arm helpers do not fix an outlined `match` (they only help inside an inlined
one, at +7k). Open issue handed to P1: DD's `process_pair` is outlined and called from the pair loop,
so a ball-only world may pay cuboid prices per pair — P1 measures it and proposes per-kind pair
buckets if so. Wave 4 is complete.

Wave-4 status (2026-09-22): merged G3 ✅ (#38, golden manifolds 66 → 87), GF2 ✅ (#39), GF4 ✅ (#41), GF1 ✅
(#42), DD ✅ (#43); GF3 running; DF launched after DD; GG waits for GF3. Measured (Sierra gas net | Cairo
steps): ball–ball 39k | 277–411; convex–ball 92k | 466–754; cuboid–cuboid 482k | 767 separated, 1 090
cached, 3 165 full; halfspace–cuboid 212k, halfspace–segment/capsule ~140k; cuboid–segment 524k | 716
separated, 3 082 touching. Narrow phase (DD, mock dispatcher): `process_pair` 275k, `compute_contacts`
≈ 0.5M per body in a stack (18.9M for 32), `broad_phase_proxies` 290k per collider — optimisation
candidates for P3; warm-start carry-over by sorted merge beats `Felt252Dict` by 1.5–3.4 %.
Findings: (1) G3's vectors caught a real gap — vertex–vertex cuboid–segment needs an endpoint–corner
fallback after clipping (face-normal SAT cannot produce the diagonal normal); upstream (GJK path)
emits that point twice, the port reproduces it, and a 2-point manifold costs the solver 233k vs 153k
per pass → candidate: collapse duplicates. (2) Any test iterating `contact_manifolds::cases()` drifts
in gas when the golden table grows: merge golden extensions before launching their consumers.
(3) Executors now run in systemd user units with a machine-wide build lock (`scripts/executor-unit.sh`,
PR #40) after two OOM-induced mass kills.

Wave-4 finding (GF2, measured 2026-09-21): **Sierra gas is path-insensitive for loop-free code.**
`contact_manifold_cuboid_cuboid` costs 482k Sierra gas in every regime (separated early exit, cached
`try_update_contacts` hit, full SAT + clip) because `branch_align` charges each branch up to the most
expensive one, whereas Cairo steps follow the executed path: 767 (separated, short-circuit) / 968
(separated, eager two-SAT) / 1 090 (cached hit) / 3 165 (full). Consequences: (1) `gas_*` probes rank
whole functions, not early exits — candidates that differ only by an early exit must also be compared
with `--tracked-resource cairo-steps --detailed-resources`; (2) Starknet execution pays the worst
case, `scarb prove` pays the path; P3 records both resources per scene (ties into open question 2).

Upstream read-only clones for the briefs live at `UP=/home/claude/git/refs/{rapier,parry}`
(rapier `28d0ba9` = v0.35.3 + 4 commits, parry `3383f51` = v0.31.1; golden vectors still pin
`parry2d-f64 =0.30.2`). Briefs of already-merged packages keep the path of the previous machine.

| ID | Package |
|---|---|
| GF×N [P] | One task per pair cell: ball–ball, ball–cuboid, ball–capsule, cuboid–cuboid, cuboid–capsule (re-enable upstream's analytic generator), capsule–capsule, halfspace–\*, segment–\*; each validated on G0 vectors |
| GG [S] | `dispatch` (`match` on the shape pair) |
| DD [P] | Narrow-phase bookkeeping: pair table, `SolverContact` build, warm-start transfer, collision events |
| DE [P] | Joints: `GenericJoint` data + fixed, revolute, prismatic (no limits); joint rows, Gram–Schmidt, substep rebuild |
| DF [P] | Substep island loop integrating DC + DE over dense solver bodies |

**Wave 5** (integration, mostly serial)

| ID | Package | Acceptance |
|---|---|---|
| P1 [S] | `rapier2d::World`, user-change handling, `step()`, events as returned array | upstream step order (report 01 §2) |
| P2 [P] | Golden scenes: ball drop, bouncing ball, box on slope (friction), box stack (3, 5, 10), pendulum, Newton-cradle-like chain | within tolerance of Rust traces over N steps |
| P3 [P] | `gas_scene_*` benchmarks (per step, by body/contact count) + hot-path `#[available_gas]` ceilings | budgets recorded in this file |
| P4 [P] | `examples/`: `#[executable]` scene, `scarb execute` + `scarb prove` in CI (nightly job) | **milestone: a proven physics step** |

### Phase 2 — 2D complete (game-ready)

Joint limits and motors (uses `fixed::trig`), rope/spring joints, kinematic position-based bodies, sleeping
with per-step union-find islands, dominance, convex polygon (≤ 8 vertices, polygon SAT) and round
shapes, query pipeline (ray, point, AABB), one-way platforms as a built-in flag, contact-force
events, optional 2×2 block solver (measure first), `rapier_starknet` storage packing (D9/D10), a
demo game contract.

### Phase 3 — Robustness and controllers

CCD substitute (velocity caps + speculative margin) then trig-free `sweep_toi` if budgets allow,
character controller, flat compound shapes, GJK-2D as a test oracle, 2D heightfield.

### Phase 4 — 3D

`Vec3`/`Quat`/`Mat3` from glam.cairo, `SdpMatrix3` world inertia, 4-point manifolds with reduction,
15-axis SAT + polygonal feature clipping, two-tangent friction, 6-row joints. Started only once
the 2D gas profile is known.

### Explicitly out of scope

Soft bodies, multibody joints, parallel/SIMD/graph colouring, BVH, trimesh/voxels/3D heightfield,
EPA, mesh `transformation/`, serde/rkyv, debug-render, profiling counters, `dyn` hooks.

## 5. Orchestration protocol

The full strategy is [`docs/ORCHESTRATOR.md`](ORCHESTRATOR.md) (shared with the glam/nalgebra
orchestrators). In short:

1. Before a wave, the orchestrator merges the interface it depends on and **pre-declares the stubs**
   (module lines in `lib.cairo`, empty test files, `gas/<crate>/<module>.snap` targets) so that
   parallel PRs never touch a common file.
2. One brief per package in the mandatory 7-section format (`docs/briefs/<id>.md`), launched with
   `scripts/executor.sh <id> <claude:model|codex:model:effort> <brief>` in the background — local
   CLIs on accounts distinct from the session; model tier by difficulty; the in-session Agent tool
   only for short read-only research.
3. Each executor runs the gate in the foreground, regenerates the snapshot of its own modules,
   pushes, opens its PR, drives CI to green, never merges, and writes `REPORT.md`.
4. The orchestrator reads `REPORT.md` + the log, merges on green CI + review (API parity,
   deviations, gas table), then alone updates re-exports, status, decisions and this file.
5. An interrupted executor is resumed (`scripts/executor.sh resume …`), not relaunched.

Executor runs so far (claude CLI, second account; before the per-module snapshot and self-opened
PR flow): G1 Sonnet 50 turns $1.6 · C2 Sonnet 72 turns $3.5 · M3 Opus 121 turns $15.2 ·
C3 Sonnet 97 turns $5.1 · G2 Sonnet 130 turns $8.5.

Parallel width: wave 1 = 2, wave 2 = 3, wave 3 = 8, wave 4 ≈ 11, wave 5 = 3.

## 6. Contract expected from glam.cairo (D12)

Needed for gate X1, all on the shared Q32.32 `i64` scalar:

- `fixed`: arithmetic, comparisons, `abs/min/max/clamp/copysign`, `sqrt`, `recip`, conversions, and
  the **wide accumulator API** (`dot2`, `mul_add`, `norm2`, wide sums) so that engine-side kernels
  (`Rot2·Vec2`, `Pose2·Point`, constraint rows `J·v`) can fuse across types with one rescale;
- `glam::Vec2`: add, sub, neg, scale, dot, `perp`, `perp_dot`, length, length², `try_normalize`,
  min/max/abs, with public fields.

Two semantics to align before freezing, because Rapier relies on them: `inv(0) = 0` (lives in
`rapier_math::math_ext`, not in `fixed::recip`) and the rounding direction of `mul` (must be the
one used to quantise golden vectors in G0).

## 7. Risks

| Risk | Mitigation |
|---|---|
| Precision of soft-contact coefficients and inverse inertia in Q32.32 | D3 dedicated scale, measured in C2 against Rust values |
| Squared-length comparisons lose half their bits | M3 wide comparisons; tolerances in ulps |
| Divergence from Rust traces misread as bugs | D11 comparable settings, tolerance bands per scene, leaf-level vectors first |
| Solver cost on stacks exceeds transaction budgets | Expose substeps/iterations, gather/scatter per manifold, hoist `erp`/`cfm`, `try_update_contacts` fast path, measure in P3 before optimising |
| Snapshot conflicts between parallel branches | Orchestrator-only regeneration |
| Compiler upgrades flip candidate rankings | Losers kept under `mod alternatives`; toolchain bumps in their own PR |
| cairo-lang 2.19.4 incremental-cache panic seen on a large bench workspace | set `incremental = false` if it reproduces |
| Schedule coupling with glam.cairo (gate X1) | Only `fixed` F1+F2 and `Vec2` block; wave 1 is independent; fallback = vendored snapshot of the same code |

## 8. Open questions for the owner

1. Should the Parry port eventually live in its own `parry.cairo` repository (the crate layout already allows it)?
2. Target runtime for the first game: Starknet contract, Dojo world, or client-side proving with `scarb prove`? It decides how early `rapier_starknet` is needed.
3. Is 2D-first acceptable, or is there a 3D game already in sight?
