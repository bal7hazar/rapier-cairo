# rapier.cairo — execution plan

Status: **v1, 2026-09-20.** Owner of this file: the orchestrator session (see [`AGENTS.md`](../AGENTS.md)).

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
| D1 | Scalar `Real` = signed **Q32.32 in `struct { raw: i64 }`**; mul via BoundedInt bias trick; `sqrt` via core `u128_sqrt`; `inv(0) = 0` as upstream | Report 04; range ±2.1e9, resolution 2.3e-10 |
| D2 | **Fused kernels are the unit of design**, not scalar ops: dot, cross, `Rot2·Vec2`, `Pose2·Point`, constraint rows `J·v` accumulate in wide form and rescale once | 3–6× cheaper than composing scalar ops |
| D3 | Quantities with extreme range (inverse inertia, `cfm`) get a **dedicated scale** (e.g. Q16.48) — to be benchmarked in M1 before freezing | Report 04 §precision; Report 01 §9.5 |
| D4 | `Real::MAX` sentinels are replaced by `Option`/flags; rigid joints (`cfm ≈ 1.5e-9` upstream) are special-cased | Would overflow or underflow in fixed point |
| D5 | **2D first**, 3D only after 2D is benchmarked (3D ≈ 4–6× gas per contact *(est.)*). Dimension-specific crates (`*2d`, later `*3d`) over a shared dimension-agnostic core — no mutually exclusive Scarb features | Keeps `--workspace` builds and the snapshot simple |
| D6 | Shapes are a **closed enum**; no `dyn`, no `SharedShape` | Cairo has no trait objects; `match` dispatch is cheap |
| D7 | Broad phase is **stateless**: brute-force AABB with static/dynamic split, sort-and-prune as the measured alternative | Nothing to persist, no dict, trivially deterministic |
| D8 | Solver = upstream soft-step algorithm, scalar, sequential Gauss–Seidel in **pair-slot ascending order**; `num_solver_iterations` exposed (main gas/quality knob); velocities gathered once per manifold, scattered once | Order is part of the state-transition function |
| D9 | Per-step scratch is never persisted. Persistent world state = poses, velocities, warm-start impulses, (later) sleep timers | On Starknet, state I/O rivals compute |
| D10 | Core crates are **pure Cairo** (no `starknet` dep); Starknet/Dojo storage packing lives in a separate adapter crate | Usable from contracts, Dojo and `scarb execute` / `scarb prove` |
| D11 | Validation against upstream through **golden vectors** generated by a Rust harness (`rapier2d-f64` / `parry2d-f64`, with recycling, clustering, block solver, CCD and sleeping disabled); tolerance-based, never bit-exact | Rust's solve order comes from graph colouring |
| D12 | `rapier_math` is an **adapter crate**: it defines what the engine needs (`Real`, `Vec2`, `Rot2`, `Pose2`, `math_ext`). It is implemented in-repo first, then re-exports `glam.cairo` once that library ships a compatible API (contract in §6) | Unblocks the engine without coupling release schedules |

## 3. Architecture

```
rapier_testing     dev-only: opaque(), approx asserts, fixtures
rapier_math        Real (Q32.32), Vec2, Rot2, Pose2, fused kernels, math_ext, trig      → glam.cairo later (D12)
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
| F2 [S] | Freeze `rapier_math` public signatures (types + trait signatures, no bodies) | next |

### Phase 1 — 2D MVP: "a box stack settles, and the step is proven"

**Wave 1** (no dependency between them)

| ID | Package | Upstream reference | Acceptance |
|---|---|---|---|
| M1 [P] | `Real`: add/sub/neg/abs/cmp/min/max/clamp, mul (candidates: bias-trick BoundedInt, `i128` widening, sign-magnitude), div, `inv`, sqrt, rsqrt, conversions, consts, overflow policy. Settles D3 by measurement | cubit tests as vectors | winner ≤ figures of report 04; `fuzz_` equivalence of candidates |
| C1 [P] | `rapier_core::data`: generational handles, arena (candidates: `Felt252Dict` vs `Array` rebuild), union-find, interaction groups (math vs bitwise candidates) | `src/data`, `geometry/interaction_groups.rs` | handle reuse/generation tests |
| G0 [P] | `tools/golden`: Rust harness + fixture generator; leaf vectors (mass props, AABB, each contact pair over 6 regimes: separated, within prediction, touching, shallow, deep, degenerate) and scene traces | report 01 §8, report 02 §8 | fixtures committed, regeneration documented |

**Wave 2** (needs M1)

| ID | Package | Acceptance |
|---|---|---|
| M2 [P] | `Vec2`, `Rot2`, `Pose2` with fused kernels (dot, perp-dot, length, `try_normalize`, rotate, transform, inverse-transform, `pos12`) | fused vs composed candidates ranked |
| M3 [P] | `math_ext`: `gcross` family, `copysign`, tolerances expressed in ulps, **wide comparisons of squared quantities** (compare the unscaled product against a pre-scaled constant) | report 02 §6 hazards covered by tests |
| M4 [P] | Trig: `sin_cos`, `atan2` (polynomial, degree 7 vs 9 candidates) | max error documented over a sweep |
| C2 [P] | `IntegrationParameters`, `SpringCoefficients` (`erp_inv_dt`, `cfm_factor`, …) | matches Rust golden values |
| F3 [S] | Freeze geometry↔dynamics interface: `ContactManifold`, `TrackedContact`, feature ids, `SolverContact`, `MassProperties` | merged before wave 3 |

**Wave 3** (needs M2–M3, F3) — geometry and dynamics streams run side by side

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

**Wave 4** (needs wave 3)

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

Joint limits and motors (uses M4), rope/spring joints, kinematic position-based bodies, sleeping
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

1. The orchestrator session picks the next wave, writes one task brief per package (template in
   `AGENTS.md` §3) and launches one sub-agent per brief **in an isolated worktree**, all in parallel.
2. Interfaces a wave depends on are frozen and merged to `main` *before* the wave starts (F2, F3).
3. Each executor returns: branch name, test results, `scripts/gas.py diff` table, candidate ranking.
4. The orchestrator reviews, opens one PR per package, merges in dependency order, then regenerates
   `.gas-snapshot` once on `main` for the wave. Executors never commit the snapshot.
5. CI is the gate: fmt, lint, build, tests, gas check. A red `main` stops the wave.
6. After each wave this file is updated: status column, measured budgets, new ADRs.

Parallel width: wave 1 = 3, wave 2 = 4, wave 3 = 8, wave 4 ≈ 11, wave 5 = 3.

## 6. Contract expected from glam.cairo (D12)

`rapier_math` will re-export glam.cairo when it provides, on the **same `Real`** (Q32.32, `i64`):
`Vec2` (add, sub, neg, scale, dot, perp-dot, length, length², `try_normalize`, min/max/abs),
`Rot2` as unit complex (`from_angle`, mul, inverse, rotate, inverse-rotate, renormalise),
`Pose2` (mul, inverse, transform point/vector, inverse transform, `inv_mul`), all as fused kernels,
plus access to the raw/wide form so engine-side kernels (constraint rows) can fuse across types.
Until then the in-repo implementation is the reference and can be contributed upstream.

## 7. Risks

| Risk | Mitigation |
|---|---|
| Precision of soft-contact coefficients and inverse inertia in Q32.32 | D3 dedicated scale, measured in M1/C2 against Rust values |
| Squared-length comparisons lose half their bits | M3 wide comparisons; tolerances in ulps |
| Divergence from Rust traces misread as bugs | D11 comparable settings, tolerance bands per scene, leaf-level vectors first |
| Solver cost on stacks exceeds transaction budgets | Expose substeps/iterations, gather/scatter per manifold, hoist `erp`/`cfm`, `try_update_contacts` fast path, measure in P3 before optimising |
| Snapshot conflicts between parallel branches | Orchestrator-only regeneration |
| Compiler upgrades flip candidate rankings | Losers kept under `mod alternatives`; toolchain bumps in their own PR |
| cairo-lang 2.19.4 incremental-cache panic seen on a large bench workspace | set `incremental = false` if it reproduces |
| Schedule coupling with glam.cairo | D12 adapter crate |

## 8. Open questions for the owner

1. Should the Parry port eventually live in its own `parry.cairo` repository (the crate layout already allows it)?
2. Target runtime for the first game: Starknet contract, Dojo world, or client-side proving with `scarb prove`? It decides how early `rapier_starknet` is needed.
3. Is 2D-first acceptable, or is there a 3D game already in sight?
