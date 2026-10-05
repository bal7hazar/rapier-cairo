# rapier-cairo — execution plan

Status: **v2.101, 2026-10-05** (v2: scalar delegated to glam-cairo's `fixed`; v2.1: wave 1 merged; v2.2: `fixed` consumed, C2 + M3 merged; v2.3: C3 + G2 merged; v2.4: glam `Vec2` consumed, M2 + F3 merged, wave 3 launched; v2.5: wave 3 merged, wave 4 in progress; v2.6: DB, DE, GH merged, orchestrator moved to a new machine, rest of wave 4 launched; v2.7: G3, GF1, GF2, GF4, DD merged, GF3 running, DF launched; v2.8: GF3, DF merged, GG running, wave-5 stubs + P1 brief; v2.9: GG merged, wave 4 complete, P1 launched; v2.10: P1 merged, prelude, GM/P2/P3/P4 launched; v2.11: GM, P2, P3 merged, SD launched, prover finding; v2.12: SD, P4 merged, DM launched, nightly execute job; v2.13: paused, DM wip pushed; v2.14: resumed, BX/GS briefs, glam 0.3.0 plan; v2.15: DM, BX merged, GS + OS running; v2.16: GS merged, OP launched; v2.17: OS merged, BP launched; v2.18: OP merged, OI launched; v2.19: OI merged, budgets refreshed, SO launched; v2.20: BP merged, OJ launched; v2.21: SO merged, D8 amended, DO launched; v2.22: OJ merged, BG launched; v2.23: DO merged, ON launched; v2.24: BG merged, budgets and ceilings refreshed; v2.25: ON merged, CL and BS launched; v2.26: CL, BS, parallel CI merged, AS launched; v2.27: phase 2 wave 6 (SL, QP) planned and launched; v2.28: QP merged, JL launched; v2.29: SL merged, SC launched; v2.30: JL merged, SI launched; v2.31: SI merged, ADR 0001; v2.32: CP1 launched; v2.33: SC merged, JM launched; v2.34: CP1 merged, CP2 prepared; v2.35: CP2 merged, wave 8 KD launched; v2.36: JM merged, KD bug found, RJ launched; v2.37: KD merged; v2.38: EV launched; v2.39: EV merged, ADR 18–19; v2.40: RJ merged, repo renames, parry-split proposal, paused; v2.41: resumed for feature parity, AP + SE; v2.42: AP merged, parity waves 9–14, api-parity CI job, RB; v2.43: programme target (game), G0, machine rule; v2.44: cost of a level, BT wave, release assessment; v2.45: crate-scoped local gate for executors (CI is the full gate), CW brief; v2.46: SE merged, codex for audits only; v2.47: WS world-state lot (programme G1b), client-side step figures; v2.48: RB merged, WS launched; v2.49: WS merged, CW launched; v2.50: `0.1.0-alpha.1` published on scarbs.xyz, G0 merged, BT decided and briefed, parry split after M6; v2.51: CW merged, BT1 launched; v2.52: BT2 launched, slingfall G3 findings folded into BT2; v2.53: BT1 merged (−41 % steps at impact), BT3 briefed; v2.54: BT3 launched, class-size blocker (CS1 / CS2); v2.55: BT2 merged (sleeping bodies free, D7 / D9 amended, `WorldState` v2), CS1 launched; v2.56: `0.1.0-alpha.2` published, BT3 merged, BT4 launched; v2.57: CS1 merged — one class cannot hold the step; v2.58: SNIP-36 in-class path closed for the MVP, CS2 to the backlog, QY1 launched; v2.59: BT4 merged, alpha.3 go requested, JA1 briefed; v2.60: `0.1.0-alpha.3` published, QY1 merged; v2.61: JA1 merged, MH1 queued; v2.62: MH1 merged, parity inventory refreshed (dim3-only methods, `pub type`), 54.1 %; v2.63: QY2 merged, CHANGELOG `Unreleased`; v2.64: PO1 merged, LO1 / SH1 launched; v2.65: LO1 merged; v2.66: programme order CC → SH2 (KC deferred), CC1 briefed, alpha.4 go after SH1; v2.67: SH1 merged, CC1 launched, parity inventory follows the module tree; v2.68: `0.1.0-alpha.4` published, LO2 merged; v2.69: CC1 merged, CC2 briefed; v2.70: CC2 merged, SH2a briefed; v2.71: SH2a merged, SH2b briefed, alpha.5 = CC1 + CC2 + LO2 + SH2a; v2.72: alpha.5 published, SH2b merged, RG1 / SF1 running, CS2 back; v2.73: RG1 merged, CS2 launched, RG2 briefed; v2.74: SF1 merged (a port defect, results change), alpha.6 = SH2b + RG1 + RG2 + SF1; v2.75: CS2 merged (−59.1 % program), alpha.6 += CS2; v2.76: RG2 merged, `0.1.0-alpha.6` published, parity closeout options; v2.77: programme decisions — PX1 closeout, prelude selectors, KC1; (c) / (d) parked; v2.78: PX1 merged — raw 74.0 %, in scope 81.0 %; v2.79: KC1 merged, parity inventory drops dim3-only module declarations — raw 77.2 %, in scope 84.2 %, CN1 briefed; v2.80: PX2 merged — raw 78.2 %, in scope 85.2 %; CN1 launched; v2.81: CN1 merged; v2.82: CS3 class-split study launched, parity matcher finds the contact generators — raw 79.0 %, in scope 86.1 %; v2.83: CS3 merged — no cut fits yet, SN1 / CS4–CS6 proposed; v2.84: SN1 positive, CS4 merged, CS5 launched; v2.85: CS5 merged, CS6 + CX1 running; v2.86: CS6 merged — the caller fits; v2.87: CX1 merged, CX2 + PK1 running; v2.88: PK1 merged, `0.1.0-alpha.7` published, package gates 1–3 enforced; v2.89: CX2 merged — the slim shot at +37.2 %, DU1 running; v2.90: DU1 and CS7 merged, alpha.8 go; v2.91: `0.1.0-alpha.8` published, PX3 merged; v2.92: parity exclusions, CP3 merged; v2.93: IG1 merged; v2.94: PX4 merged, the rapier queue idle; v2.95: PX5 merged; v2.96: resume point, the owner's pause; v2.97: new orchestrator session under the Nexus standard, orchestration documents aligned; v2.98: TC1, IT1 step 1, CX3 and PP1 merged, the orchestrator moves to herdr, documents aligned; v2.99: alpha.9 prepared; v2.100: alpha.10 prepared; FU1's whole-shot steps, measured after the release: `docs/releases/0.1.0-alpha.10.md`; v2.101: status at the pause). Owner of this file: the orchestrator session (see [`AGENTS.md`](../AGENTS.md)).

Goal: a Cairo port of [Rapier](https://github.com/dimforge/rapier) good enough to build a complete
game whose physics is provable, with gas tracked per feature from the first line of code.

## Status of the track (2026-10-02)

**Orchestrator.** The coordinator of the herdr project `slingfall-rapier`, successor of the Nexus session "[Opus 5.5]
Orchestrateur rapier — slingfall", from 2026-10-02. Implementers, reviewers and auditors are herdr threads
(`docs/ORCHESTRATOR.md`, `slingfall/OPERATIONS.md`).

**Merged on 2026-10-02:**
- #249 IT1 step 1: the impact tick's steps, profile and levers (`docs/research/impact-tick.md`).
- #250 TC1: toolchain Scarb 2.20.1 (Cairo 2.20.0) / snforge 0.64.0, results bit-identical, steps: game path +1.25 to +1.38 %, `rapier2d` probes up to +1.84 % (median +1.33 %), `rapier2d_classes` up to +3.78 % (median +1.61 %).
- #251 PP1 step A: `scripts/prepush.sh` and its pre-push hook.
- #252 PP1 step B: scarb / snforge download retries, PR-only cancel of superseded CI runs.
- #253 CX3: `NarrowPhaseClass` runs its own pair loop, the slim shot −3.53 % (owner's) / −3.85 % (reference).

**State.** No release is pending. Held: EL1, the in-scope engine levers of `docs/research/impact-tick.md` §4, until the
owner's physics-rate study reports. A release (alpha.9) needs the project manager's written go per package
(`docs/ORCHESTRATOR.md`, Releases).

**Published: `0.1.0-alpha.8`** (tag `v0.1.0-alpha.8`, release commit 5a3c0fc; six crates: rapier_math, rapier_core,
rapier_geometry2d, rapier_dynamics2d, rapier2d, rapier2d_classes). It holds DU1 (`fixed` 0.4.0, `glam_core` 0.4.1),
CX2 (polygon family in `NarrowPhaseClass`, trimmed previous pairs), CS7 (slim caller 73,204 → 67,076 CASM felts, 6,652
under 73,728; `WorldEditClass`; measured-only classes moved to `rapier_sink`) and the README list of classes to declare.
Step results unchanged since alpha.7 (full `WorldState` codec included); the slim layout's pile10 shot +36.9 % Cairo
steps in 4 transactions of ≤ 10M. The game (slingfall #43) is on alpha.8: world class 71,076 CASM, pile10 35.29M steps
in 6 transactions.

**On `main` since alpha.8 (unreleased, step results unchanged; TC1 and CX3 move Cairo steps only):** prelude exports (`RigidBodyType`,
`ShapeTrait`, the basic shapes, contact-pair / interaction-graph views, set `Index`, …), PX3 (Parry geometry utilities),
CP3 (`ContactPairView` read API), IG1 (read-only `InteractionGraph`), PX4 (cheap API items), PX5
(`map_attached_joints_mut`), TC1 (toolchain 2.20.1 / 0.64.0), CX3 (`NarrowPhaseClass` pair loop),
`tests/finite_state.cairo`, `docs/PACKAGES.md`, and CI: the `execute` job also runs on release / dependency PRs, `sink`
jobs per crate with 2 threads, package gates 1–3 enforced, declared classes gated with a 1,000-felt margin.

**Parity:** raw 84.6 %, in scope 94.5 % (1,527 ported, 89 missing, 976 excluded of 2,592). Package sizes
(`docs/PACKAGES.md`): every gate passes, tightest `rapier_geometry2d` lines at +42 % margin.

**Parked, with reasons (reopened only by a game need or the programme):**
- (c) solver-graph order for new pairs, and (A) the solver's scalar API (`SolverBodies`, `SolverPose`, `VelocitySolver`,
  `ContactRef` / `GraphPos`): a numeric change on the step; PX1 keeps them open by record.
- (d) further step levers; (e) `core_witness` short-pair precision (step path); (f) the latent sweep normal.
- (B) composite machinery (composite traits, compound flags / pseudo-normals, `NormalConstraints`, sub-shape poses):
  changes contact results for composite worlds.
- (E) sub-shape result widening (`with_subshape(s)`, `ShapeDistance`): costs steps on every query (ADR 35).
- (F) `contact_skin`: changes the collider layout and the step for users who set it.
- (G) CE follow-ups (#271; deferred by the orchestrator, 2026-10-04), for a later lot: (a) the +10 Sierra gas on
  `gas_scale_dyn_ball_polygon` and `gas_scale_dyn_cuboid` (code layout, no mechanism found); (b) the lever for the
  +200 gas per composite pair step of worlds without a compound: `#[inline(always)]` on `composite_pair_inner` for the
  plain arm, measured with `--tracked-resource cairo-steps` on one composite world; (c) the `vee/p5` tie of
  `composite_queries_golden` should also assert `(subshape 0, Vertex(1))`.
- (H) WS3, the dormant-pair layout of `WorldState` (project manager's decision, 2026-10-03; #266 closed, not merged,
  `WorldState` stays v3), parked: the opt-in dormant-pair layout gains −0.53 % on the reference shot (slim) and −4.46 %
  on L20 over 60 ticks; the default path costs +0.15 to +2.16 % (P3 probes), because two boxed cells ride along every
  `ref`; `take_removed` alone costs +0.1 to +0.2 %; the v4 codec costs about 7.8k steps per state round trip, paid at
  every chunk boundary. Design note for a later lot: boxed cells only when opted in. 14 diagnostic tests read
  `narrow_phase.pairs` as the whole list, and a default layout would need them rewritten.
- Missing by decision: V-HACD / voxelisation ("not ported yet", low priority), `solver_contact_world_points` (no exact
  answer), `IndexMut`, `take_removed` (closed by PX9 with WS3's measured cost, ADR 53). `GenericJointBuilder::user_data`
  is ported by PX9 beside the joint set (`JointUserData`, ADR 54), not as a field of the stepped joint.
- (V1) wide velocities across the rows of one solver kernel (project manager's decision, 2026-10-04), parked: keep the two
  bodies' velocities wide (Q64.64) across the rows of one kernel (2 to 4 rows), instead of narrowing after each impulse
  application. Estimate: FU0's rough −0.2 to −0.45M steps on the reference shot (`docs/research/fused-rescales.md` §2,
  "Ruled out or left out"). Prerequisite: an unbounded Q96.96 accumulator in `fixed`, a glam-track change (the next row's
  `jv` would exceed `fixed`'s 16-term bound, and P1's product would no longer fit a felt252). Reopens only if the game's
  proofs need it after its re-pin. Also out, with their figures: C6 (1.95 ulp > 1) and C7 (out of the `box_slope_slide`
  band), each under about 10k steps on the reference shot.

**Launch rule (programme, 2026-10-02; procedure in `docs/ORCHESTRATOR.md`):** a thread starts only after `machine-capacity` and the programme's placement rule (the Mac for builds that write no pin, the VPS for the rest, one heavy suite at a time there); the conditions of the programme go into the brief.

## Status at the pause (2026-10-05)

The owner paused the whole Slingfall programme on 2026-10-05, libraries included: no new lot, thread, release or
publication until `hp resume`.

**Published:** `0.1.0-alpha.10`, all six crates, on 2026-10-04: FU1 (fused rescales) and CE (compound internal edges
and the parry 0.31.1 query answers). It is tagged, has a GitHub pre-release, and its record is
`docs/releases/0.1.0-alpha.10.md`, which includes the whole-shot steps measured after the release.

**Merged since alpha.9:**
- #267 FU0 (study)
- #268 B1
- #269 OB (golden oracle on parry 0.31.1)
- #270 FU1
- #271 CE
- #272 alpha.10 bump
- #273 the release commit (on `release/0.1.0-alpha.10`)
- #274, #276 and #277: the release record
- #275 PX9: the parity table closed at 100 % in scope (`Compound::bvh` and `take_removed` closed, joint `user_data`
  ported beside the joint set)

**Open:** no pull request. No thread is running.
- DEP5 had just started when the pause came, and was stopped before any commit or push.
- WS3 (#266) stays closed, parked in entry H.

**Merged on `main` after alpha.10, unreleased:** PX9 (#275).

**Next lot, on resume: DEP5.** Move to `fixed` 0.5.0 and `glam_core` 0.5.0. Both are published, and rapier depends on
no other glam crate.
- First step: check whether a consumer must enable `glam_core`'s experimental `associated_item_constraints`. If it
  must, the project manager decides before the bump.
- Expected: bit-identical, with class hashes re-pinned from CI if they move.
- Brief: `docs/briefs/dep5-fixed-glam-05.md`, to be committed by the DEP5 thread.

**Parked:**
- V1, wide velocities across a kernel's rows (entry in this plan, with the Q96.96 prerequisite);
- WS3, the dormant-pair layout (entry H);
- C6 and C7 stay out.

Resumed on 2026-10-05, owner's decision, only to finish `0.1.0-alpha.11` (DEP5 + PX9).

## 1. What the research established

Four reports back this plan. Numbers below are measured unless marked *(est.)*.

| Report | Headline findings |
|---|---|
| [01 — Rapier](research/01-rapier-analysis.md) (v0.35.3, 80k lines) | Maths is now **glam** (`Vec2`, `Rot2` = unit complex, `Pose2`), nalgebra only for multibody/SIMD. Solver is a substepped soft-constraint PGS ("TGS-soft", 4 substeps). **No trig, `exp` or `pow` anywhere in the step**: only `+ - * / sqrt min max clamp abs`. After cutting soft bodies (26k), multibody (6.7k), parallel/SIMD (6k), persistent islands, serde, debug… **6–8k lines of logic remain, < 2k numerically essential.** Solver sweeps are the #1 cost: `O(substeps × manifolds × points)`. |
| [02 — Parry](research/02-parry-analysis.md) (v0.31.1, 80k lines) | The step consumes only `Aabb`, `compute_aabb`, `mass_properties`, `contact_manifolds` and the manifold types. With Ball, Cuboid, Capsule, HalfSpace, Segment (+ convex polygon via SAT) **the whole 2D pair matrix needs neither GJK nor EPA** (EPA is the biggest fixed-point risk upstream). The BVH broad phase is a poor fit for Cairo; brute-force AABB (≤ ~40 bodies) then stateless sort-and-prune, no `Felt252Dict`. MVP ≈ 3.5–4k Cairo lines *(est.)*. |
| [03 — Cairo ecosystem](research/03-cairo-ecosystem-architecture.md) | alexandria: only repo with gas regression, but non-blocking and noisy. origami: best efficiency idioms (mul/div packing, const tables, unrolling), no gas tracking, its algebra crate is not reusable. starknet-agentic: optimisation/testing rules and the `AGENTS.md` coordinator/executor model. snforge has **no built-in gas snapshot** → `scripts/gas.py`. |
| [04 — Numeric benchmark](research/04-numeric-benchmark.md) (415 probes) | Owner heuristic **confirmed**: DivRem ≈ bitwise in steps but bitwise costs 12–55 % more gas; loops cost 13–140× more; BoundedInt is cheaper still. **Q32.32 in a native `i64`** wins: constant-cost add/sub/lt (6 steps), mul 16 steps, no negative zero (cubit has one). **Fused multiply-accumulate** (one rescale per output): dot2 18 steps vs 69 (cubit), mat3·vec3 82 vs 344, Vec2 length 17 vs 109. Core `u128_sqrt` = 10 steps vs Newton 231. Polynomial sin 144 steps vs cubit 1051. **Orion rejected**: its FP32x32 *is* cubit, its tensors cost 3–16× struct code. Q16.16 and Q8.23 overflow; Q64.64 costs 3–12×. |

The `glam-cairo` and `nalgebra-cairo` sessions reached the same scalar conclusion independently
(Q32.32 in `i64`, bias-trick multiplication, fused accumulation).

## 2. Decisions

Each becomes a short ADR in `docs/adr/` when first implemented; deliberate divergences from upstream are registered in `docs/adr/0001-upstream-divergences.md`.

| # | Decision | Why |
|---|---|---|
| D1 | Scalar = signed **Q32.32 in `struct { raw: i64 }`**, provided by glam-cairo's `fixed` (D12); mul via BoundedInt bias trick; `sqrt` via core `u128_sqrt`; `inv(0) = 0` as upstream | Report 04; range ±2.1e9, resolution 2.3e-10 |
| D2 | **Fused kernels are the unit of design**, not scalar ops: dot, cross, `Rot2·Vec2`, `Pose2·Point`, constraint rows `J·v` accumulate in wide form and rescale once | 3–6× cheaper than composing scalar ops |
| D3 | ~~Dedicated scale for extreme-range quantities~~ **Not needed for the spring coefficients**: C2 reproduces every `IntegrationParameters`-derived value within 1 ulp in plain Q32.32 (joint `cfm_coeff` ≈ 6 ulp of resolution but correct). Re-evaluate only for inverse inertia in DA | Measured in C2 (PR #10) |
| D4 | `Real::MAX` sentinels: kept as `fixed::MAX` where the consumer guards against multiplication (velocity caps, C2), replaced by `Option`/flags elsewhere; rigid joints (`cfm ≈ 1.5e-9` upstream) are special-cased | Would overflow or underflow in fixed point |
| D5 | **2D first**, 3D only after 2D is benchmarked (3D ≈ 4–6× gas per contact *(est.)*). Dimension-specific crates (`*2d`, later `*3d`) over a shared dimension-agnostic core — no mutually exclusive Scarb features | Keeps `--workspace` builds and the snapshot simple |
| D6 | Shapes are a **closed enum**; no `dyn`, no `SharedShape` | Cairo has no trait objects; `match` dispatch is cheap |
| D7 | Broad phase is **stateless**: brute-force AABB with static/dynamic split, sort-and-prune as the measured alternative. **Amended by BT2 (#143):** the proxies of sleeping or fixed colliders persist between steps (in the active set) and only awake proxies are tested against them | Nothing to persist, no dict, trivially deterministic; BT2: a sleeping body costs 0 steps per tick |
| D8 | Solver = upstream soft-step algorithm, scalar, sequential Gauss–Seidel; contact order = **stable partition of the ascending pair list: pairs of two non-fixed bodies first, pairs with a fixed/world body last** (amended 2026-09-24 after SO: upstream colours touching pairs — non-fixed pairs take the lowest free colour, pairs with a fixed body the highest — and solves colours in ascending order; the partition reproduces that order whenever the non-fixed pairs' colours ascend in pair order, e.g. every golden scene; exact parity needs persisted colours, deferred); `num_solver_iterations` exposed (main gas/quality knob); velocities gathered once per manifold, scattered once | Order is part of the state-transition function; SO measured +27k gas per step for the partition vs +209k for greedy colouring per step |
| D9 | Per-step scratch is never persisted. Persistent world state = poses, velocities, warm-start impulses, (later) sleep timers. **Amended by BT2 (#143):** plus the active set (awake bodies and their colliders, static proxies, live-pair positions; derived data, invalidated by any set write) — `WorldState` v2 | On Starknet, state I/O rivals compute; the game proves client-side runs, and the active set pays for itself on every flight tick |
| D10 | Core crates are **pure Cairo** (no `starknet` dep); Starknet/Dojo storage packing lives in a separate adapter crate | Usable from contracts, Dojo and `scarb execute` / `scarb prove` |
| D11 | Validation against upstream through **golden vectors** generated by a Rust harness (`rapier2d-f64` / `parry2d-f64`, with recycling, clustering, block solver, CCD and sleeping disabled); tolerance-based, never bit-exact | Rust's solve order comes from graph colouring |
| D12 | **The scalar is not implemented here.** `glam-cairo` ships a `fixed` package (Q32.32 `i64`, wide accumulators, trig) explicitly shared by the glam, nalgebra and rapier ports; rapier-cairo consumes it (git dependency pinned by rev) together with glam's `Vec2`. `rapier_math` plays the role of upstream's `glamx`: `Rot2`, `Pose2` and `math_ext` on top of glam, contributed back if glam-cairo wants them (its plan lists them as late item P1) | Three ports with three scalars would diverge in rounding and make golden vectors incomparable |

## 3. Architecture

```
rapier_testing     dev-only: opaque(), approx asserts, fixtures
rapier_math        (glamx) Rot2, Pose2, fused kernels, math_ext — on glam-cairo `fixed` + `Vec2` (D12)
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
never depends on dynamics, so it can be extracted as `parry-cairo` later at no cost.

## 4. Phases and work packages

Notation: **[P]** parallelisable inside its wave, **[S]** serial (orchestrator). Every package
ships `test_*`, `gas_*` (one per candidate implementation) and docs per `AGENTS.md` §5.

### Phase 0 — Foundations *(done in PR #1 unless noted)*

| ID | Package | Output |
|---|---|---|
| F0 [S] | Workspace, toolchain pin, `scripts/gas.py`, CI (fmt, lint, build, test, gas check) | ✅ |
| F1 [S] | `AGENTS.md`, `CLAUDE.md`, research reports, this plan | ✅ |
| F2 [S] | Agree the `fixed`/`Vec2` API with glam-cairo (§6) and freeze `rapier_math` signatures (`Rot2`, `Pose2`, `math_ext`) | next |

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

**Gate X1 — external:** ✅ `fixed` F1 + F2 merged in glam-cairo and consumed (PR #8). **Still open: `Vec2` (glam item V2, status todo)** — the only remaining blocker for M2 and wave 3. Original wording: `glam-cairo` merges `fixed` F1 + F2 (scalar + wide accumulators) and `Vec2`
(its items F1, F2, V2). Its `docs/DESIGN.md` already fixes what we need: `fixed::Fixed { raw: i64 }`,
**floor** rounding on every rescale, panicking `recip(0)` (so `inv(0) = 0` stays in
`rapier_math::math_ext`), public wide accumulators. Only these three items block rapier-cairo; nalgebra-cairo blocks nothing.
If the gate slips, fallback: vendor a snapshot of `fixed` from the glam-cairo branch under
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

CL ✅ (#86, claude opus): one dispatch table in geometry — `dispatch::contact_manifold_step` (plain match,
`try_update_contacts` hoisted; cheap only inlined in a loop body, e.g. `DefaultDispatcher` in the pair loop)
next to the metered `dispatch::contact_manifold` (for direct, non-inlined calls: ball–ball 79k vs 649k);
`CoefficientCombineRuleTrait::apply` inlined (the reached rule only is paid); `pipeline.cairo` 706 lines;
P3 ceilings lowered. BS ✅ (#87, claude sonnet): the strip/grid cell is `2^k` from the median extent of five
sampled proxies — cost identical at world scales ×1/16 … ×256 (the fixed 4-unit cell cost up to 15× more
at ×256), +0.22 % on free fall 32. CI (#88): the test job runs as 4 parallel crate groups + a `gas` job on
the merged logs (7.5 → ~2.5 min). Next: **AS** (brief `as-core-file-budget.md`, the last two hand-written
files over 800 lines), then phase 2.

ON ✅ (#84, claude opus): one narrow-phase loop (`narrow_phase::compute_contacts_from_scratch::<D>`, fully
inlined per pair; OP's copy in `rapier2d` deleted) with a step dispatcher that hoists `try_update_contacts`:
narrow-phase stage −43 % (stack 3), −27 % (balls 8), −32 % (mixed 8); resting cuboid–cuboid pair 758k → 274k;
stack 3 step 12.93M → 11.69M, stack 10 −11.9 %; bit-identical. Left for **CL** (brief `cl-consolidate-step.md`):
the step dispatcher copies the geometry dispatch table, the combine rule is outlined (costliest arm charged),
`pipeline.cairo` is 843 lines, P3 ceilings. **BS** (brief `bs-broad-phase-scale.md`): BG's 4-unit cell is not
scale-free.

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

**Resumed 2026-09-23 evening.** Sibling news: glam-cairo released `fixed`, `glam`, `glamx` 0.3.0 on
scarbs.xyz (`fixed`: `/`, `recip`, `from_ratio` now round to nearest-even; `wide::Acc`, `wide::RecipNearest`);
nalgebra-cairo dropped its own scalar and is generic over `simba::Real` for `fixed::Fixed` (rapier needs
nothing from it before its 0.1.0; multibody stays out of scope). `glamx` 0.3.0 has `Rot2`, `Pose3`,
`SdpMatrix2/3` but **not `Pose2`** yet → `rapier_math::{rot2, pose2}` stay for now (escalation to
glam-cairo: port `Pose2`, rapier's M2 kernels available). Sequence: DM (resumed) → **BX** (registry
0.3.0, every snapshot regenerated, delta reviewed alone; brief `bx-fixed-0.3.md`, includes the flaky
mass fuzz) → **GS** (slope traces from a patched f64 parry with correct cuboid feature ids; brief
`gs-slope-traces.md`) → optimisation lots from `docs/BUDGETS.md`. At most two rapier executors at a
time (shared machine hit its CPU ceiling on 2026-09-23).

**Paused 2026-09-23 (owner's decision: the shared machine hit its CPU ceiling overnight; glam-cairo has
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
`scarb prove` (Stwo) is OOM-killed above a 22 GB cap even for one step (a wip executable of 4 781 Cairo steps;
the final `ball_drop` one-tick run is 19 587 steps including world construction,
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

**Programme target (2026-09-25, from the project-manager session "Angry Birds Cairo orchestration", `~/projects/pm/`;
owner confirmation pending in `pm/decisions/PENDING-*.md`).** The first product is a 2D Angry Birds-like game on
rapier-cairo: shots replayed through a Cairo `#[executable]`, proved locally (`scarb execute` / `scarb prove`), the
proof verified on Starknet. Consequences for this plan: open question 2 → client-side execution + local proof +
on-chain verification, so **ST / `rapier_starknet` leaves the near plan** (the contract stores level hashes and
results, not worlds); question 3 → 2D confirmed; question 1 (parry split) → "yes, after the game's critical path",
prerequisites only when the machine is free, no repository cut before the owner answers. Game-driven priorities:
RB and SE (running) → **CW** (collider / world API) → **G0**, a level-shaped golden scene (a ~20–30 m/s "slingshot"
ball into a stack of ~20 cuboids / convex polygons on a half-space with 3 sensor targets, 300 steps at 60 Hz, 4
substeps, traced against rapier-rs; per-step and whole-scene gas and Cairo steps in `docs/BUDGETS.md` — it sizes the
proof budget of one level) → the rest of the parity waves (RS, SH1/SH2, CC later; measure whether a speculative
margin suffices before CCD). `step_with_force_events` is the game's damage source: keep its overhead. Machine rule
(pm `OPERATIONS.md` §3): at most ~4 sub-agents machine-wide, 2 per orchestrator, **1 when another orchestrator runs
2**. Escalations to fixed / glam / nalgebra go through the project-manager session.

**Cost of a level (programme research R1/R2, 2026-09-25; owner confirmation pending).** The production proving path
is SNIP-36: the level logic runs as a contract entry point in the virtual OS, proof attached to the transaction, capped
at **1.1B L2 gas per transaction**. At today's ≈ 6M Sierra gas per awake contacting body per tick (4 substeps), a small
level (20 bodies, 200 ticks, ~8 awake on average) costs ≈ 10B gas, i.e. ~9 proven transactions. Hence, **after RB/SE
and pending the owner's confirmation, a "gas per awake body-tick" wave goes ahead of parity waves 11–14**: solver
sweeps (still ~80 % of a stack step), the cost of sleeping / settled structures, `find_pairs`, and the substep count
(1–2 substeps and 30 Hz if golden fidelity allows) — sized by **G0** (brief `g0-level-golden-scene.md`: 8–12 pre-settled
sleeping blocks + 2–3 cores + a 15–25 m/s pebble, 300 ticks, and a 20-block variant, measured at substeps 4 / 2 / 1 and
30 Hz). The game crate must also compile as a Starknet contract with gas enabled (snforge already does). Release: the
game consumes `rapier2d` by registry version — assessment in `docs/proposals/release-0.1.0-alpha.md` (one PR on the
owner's go). Checked for the programme: `World::remove_body` wakes the contact partners of every removed collider and
the joint partners (as upstream); the "4 781 Cairo steps per step" of the prover note was the first wip executable (before
the scene builder) — the measured figures are 19 587 steps for one tick **including world construction** and ≈ 13.8k per
additional free-fall tick (143 976 for 10), see `examples/ball_drop/README.md`.

**Programme decisions (2026-09-25, project-manager session under the owner's delegation, `~/projects/pm/decisions/`).**
The owner's guideline: as close to the Rust API as possible **unless it costs Cairo steps** (steps, not gas: the prover
pays the executed path); ordering is chosen only for speed to the target. Hence BT right after G0, ahead of waves
11–14; parity items that cost steps are not wanted, free ones (accessors, builders) may be interleaved when they unblock
the game (CW is one). The parry split waits until after the end-to-end demo (M6). The game is `bal7hazar/slingfall`; it
pins `rapier2d` by registry version: **`0.1.0-alpha.1` is published** (tag `v0.1.0-alpha.1`, `scripts/release.sh`), a
new alpha follows each wave the game needs. G0 (#133) measured the level: 10 blocks at 60 Hz × 4 substeps = 24B gas /
204M Cairo steps over 300 ticks (≈ 7× the 3e7-steps-per-shot target); × 1 substep 103M, 30 Hz × 4 79M over 150 ticks;
the settled structure does not fall asleep again within 300 ticks (12.3 of 14 bodies awake on average); upstream
fidelity is lost at the impact tick because upstream leaves the bodies woken that tick out of its solve (ADR entry 9).

**Game findings (slingfall G3 on `0.1.0-alpha.1`, 2026-09-25).** (1) Bodies built `.sleeping(true)` wake at the first
step: the port wakes the parent of every flagged collider, including a collider inserted since the last step, which
upstream does not (its narrow phase wakes the parent and contact partners of a modified collider only when the collider
already has a contact-graph index). A sleeping 10-block pile then costs ~920k Cairo steps per flight tick instead of
~56k; the game settles with a `dt = 0` step + `sleep()` meanwhile. (2) Reading activation / velocities through
`World::body` copies the whole `RigidBody` (~190 steps per read). Both are added to BT2 (upstream-exact insertion
asleep, confirmed on rapier2d-f64 in the harness; `WorldTrait::is_sleeping` and velocity reads) and ship in BT's first
alpha. The game's reference shot (pile10): flight tick 56k steps, impact tick 1.02M (six `remove_body` ≈ 90k), whole
shot to a calm end at tick 334 = 43.2M steps (target 3e7, interim 1e8); the engine is 92–98 % of it.
**Client-side execution (programme spike G1b, 2026-09-25).** The browser client runs the Cairo executable through
cairo-vm in WASM, capped by memory at ~5e7 Cairo steps per run, so the game runs the physics in chunks (state in,
K ticks, state out) — hence **WS**, a versioned `WorldState` save / restore that stays exact after removals (arena
generation and free list), blocking the game's G3/G4. Measured on `pile12` (ground, 12 unit cuboids in 3 columns, ball
r = 0.5 fired at (12, 3) m/s; 60 Hz, 4 substeps, 120 ticks): **40.0M Cairo steps**, 333k per tick on average, 540k per
tick during the first 40 ticks (impact), 200–300k afterwards. The client's 10 s gate passes with 1.3× fewer steps per
tick; the SNIP-36 budget (1.1B gas ≈ 9M steps per transaction) needs far more — both point at BT. A full state round
trip costs 1.2–2k felts and 73–110k steps (2–5 % of a shot); WS measures a compact form.

**State after `0.1.0-alpha.6` (2026-09-27).** Published alphas 1–6 (tags `v0.1.0-alpha.1` … `v0.1.0-alpha.6`, `CHANGELOG.md`).
API parity 73.9 % of the in-scope inventory (1,360 ported, 480 missing, 827 excluded). What is missing is almost only
upstream internals — contact / joint constraint builders and parts (`ContactWithCoulombFrictionBuilder`,
`JointConstraintHelper`, …), persistent islands and the BVH broad phase (no Cairo counterpart by design, ADR 6 / D7),
manifold workspaces — plus `SharedShape` (an `Arc` wrapper whose role `Shape` fills) and the controllers (KC, deferred).
Closeout options for the programme: (a) a closed exclusion reason "solver / island internals not exposed" (mirroring
"EPA/GJK internals not exposed") and a `SharedShape` → `Shape` mapping, which would bring the in-scope parity near 90 %;
(b) KC (PID controller free; the kinematic character controller now possible on CC1's shape casts); (c) fidelity: the
persisted solver-graph order at a new pair (the SO / DO class, SF1's step 94); (d) step levers left (dormant pairs out
of `narrow_phase.pairs` for mixed ticks — `WorldState` v4; the ≈ 17 steps per fixed-point rescale floor); (e) whatever the
game needs next. RG2 (#195): the old shapes' serde gap is the wider enum's cost, measured and accepted.
**Programme decisions (2026-09-27, owner's delegation):** (a) approved — PX1 (brief `px1-parity-closeout.md`: the
closed reason "solver / island internals not exposed" by explicit item list, `SharedShape` → `Shape`, the report keeps
the raw and the in-scope figures); the prelude exports the body field selectors (the game's B2 escalation); (b) KC1
next (brief `kc1-controllers.md`: PID / PD, then the kinematic character controller on CC1's casts; nothing may enter
the `BasicStepConfig` program or move a step); (c) parked — a numeric change, only bundled with another one (ADR 0001
entry 38); (d) parked until a game budget needs it. No release without a new written go.
**PX1 ✅ #199:** 159 upstream internals excluded by exact name under "solver / island internals not exposed", `SharedShape`
→ `Shape` (the type; its constructors stay missing: Cairo's `Shape` has none of those names — a cheap future item);
`docs/API_PARITY.md` reports both figures: **raw 74.0 %** (the closed reasons that existed before PX1) and **in scope
81.0 %** (every closed reason). **KC1 ✅ #202:** PD / PID controllers and the kinematic character controller on
CC1's casts (golden `pid_corrections`, `character_moves`; `BasicStepConfig` program and every step probe unchanged). The
parity walker now skips modules declared behind a dim3-only cfg (rapier's ray-cast vehicle controller, 3D contact
clustering / manifold reduction, …): 2,592 items, **raw 77.2 %, in scope 84.2 %**, `control` 100 %. `gas.py`: a
`--from-log` snapshot no longer drops entries absent from a partial log. Next: CN1 (brief `cn1-cast-start-normals.md`:
exact start normals of casts on face contacts, KC1's `wall_slide` knife edge) and PX2 (running). **PX2 ✅ #204:** the 17
`SharedShape::*` constructors on `ShapeTrait` (thin delegations, `shape.snap` +22 probes, game program and steps
unchanged): **raw 78.2 %, in scope 85.2 %**, `parry::shape` 83.1 %. **CN1 ✅ #205** (programme go
2026-09-27, not bundled with (c) / (d)): a support-map cast that starts on a face reports the face's exact normal
(`on_face`, start-geometry block only; the slab slack scales with the core distance after the codex audit); start
golden 16 pairs × 3 directions × 5 option sets, worst normal 0 raw (was up to 327,680), KC golden 900 → 3 raw and
`wall_slide` strict. Touching starts +670 Cairo steps per cast, non-touching 0; the 52 step probes (`game_path`, P3,
levels, sleep), the 51 CCD tests and every program size identical. Left open, each a numeric change for the programme
to schedule: (e) `core_witness` normalises a very short closest pair at that length's precision (a corner pair 2^-16
apart: normal 72 raw from upstream) — shared with the step's contacts, so parked like (c); (f) `query/sweep/proxy.cairo`
normalises its rounded pair the same way — latent (CCD reads only the fraction), to fix if `SweepToiOutput.normal`
gets a consumer. (e) and (f) are parked with (c) and (d) (programme, 2026-09-27). The parity matcher now finds Parry's
`query::contact_manifold_*` generators (`contact_generators/*.cairo`) and `clip_segment_segment` (`clip.cairo`), ported
long ago but unmatched: **raw 79.0 %, in scope 86.1 %** (1,424 ported, 229 missing).

**CS3 running (programme priority, 2026-09-27; parity lots only as filler beside it).** The game's contract is live on
Sepolia but a settled record takes 73 min through SHARP; SNIP-36 needs the physics in declared classes of ≤ 81,920
Sierra **and** CASM felts each, called through `library_call`, ≤ 10M steps per transaction (`SlingfallSim` is 140,568
Sierra felts, 1.72×). CS3 (brief `cs3-class-split.md`, Opus) measures the `BasicStepConfig` step's size by phase,
candidate 2–4-class cuts with ≥ 10 % margin and their crossing data, the steps they cost on the game-shaped path and on
the pile10 reference shot (target ≤ +25 %), and whether `StepConfig` can carry the phase dispatch (in-process by default,
bit-identical, 0 steps for non-contract users). Deliverable: `docs/research/class-split.md`; any engine change stays on
an unmerged prototype branch until the programme's go.

**CS3 ✅ #209 (2026-09-28): no 2-, 3- or 4-class cut of today's step fits.** The game's step as one class
(`BasicGameStep`) is 133,280 Sierra / 274,232 CASM felts. Contacts split into two family classes (ball pairs 39,561,
polygon / cuboid / half-space pairs 54,634 CASM) and the solve into one (45,552; needs `Serde` / `pub` on the solver's
I/O, prototype `proto/cs3-phase-dispatch`), all through `StepConfig`'s existing slots, bit-identical over the local
pile10 reference shot (22.43M steps in process, slingfall 22.0M). The caller that keeps the world stays at 162,902 CASM
in the 4-class layout, 86,357 at best once free_path, non-basic arms, mass, islands, broad phase, solve-and-advance and
the narrow loop are out (throwaway builds). Steps: contacts out +16.0 %, solve out +9.3 %, 4 classes +24.2 % (a library
call ≈ 990 steps + ≈ 17 per felt; 3 transactions of ≤ 10M either way). Proposed, each on the programme's go: **SN1**
(does the SNIP-36 virtual OS execute `library_call_syscall`), **CS4** (family + solver classes, constant class hashes),
**CS5** (strategy slots so the remaining stages can leave the caller), **CS6** (slim the caller from 86k to ≤ 73,728 —
not demonstrated). CI now runs `rapier_sink`'s tests (job `sink`). **Programme decisions (2026-09-28):** SN1 is the
programme's (a negative verdict stops CS5 / CS6 at once); **CS4 go — running** (brief `cs4-declared-classes.md`: new
crate `rapier2d_classes` with the three classes, library-calling strategies and constant class hashes; results
bit-identical on the pile10 shot, in-process steps and `program.basic` unchanged, declared classes ≤ 73,728 guarded by
`bytecode_size.py check`); CS5 go once CS4 is merged (every new slot defaults to in-process; the batched narrow phase
measured both ways before it becomes a default); CS6 waits for CS5's caller size and SN1 (a codec that changes the
serialized bytes is a breaking `WorldState` v4: ranked last). One codex audit at the end of CS5; alpha.7 proposed to the
programme with the measured tables once CS4 + CS5 are in. **SN1 (programme, source evidence):** the SNIP-36 virtual OS
runs `library_call` (874 steps), `storage_read` (240), `call_contract`, `get_execution_info`, `send_message_to_l1`; it
fails on deploy / replace_class / get_block_hash / meta_tx_v0, and the prover rejects the ecdsa / range_check96 /
add_mod / mul_mod builtins; state between transactions travels as calldata in and an L2→L1 message out. **CS4 ✅ #211:**
crate `rapier2d_classes` (`ContactBallClass` 39,561, `ContactPolygonClass` 54,634, `SolverClass` 43,726 CASM felts;
`ClassHashes` constants, `SplitStepConfig<H>`), bit-identical over the 151-tick pile10 shot at +22.8 % steps (constant
hashes −320,892 vs storage); `bytecode_size.py check` guards 73,728 and the SNIP-36 syscalls / builtins; the caller
`Split4Step` is 162,173 CASM; in-process steps and `program.basic` unchanged; solver I/O `Serde` / `pub` = ADR 0001
entry 39. `rapier2d_classes` joins the release order after `rapier2d`. **CS5 ✅ #213:** five stage slots in
`StepConfig` (in-process by default: every `steps_*` probe and `program.basic` unchanged) and four stage classes
(`SolveAdvanceClass` 70,402, `MassClass` 35,686, `IslandsClass` 16,908, `BroadPhaseClass` 9,841 CASM); every stage out,
the caller is 120,402 CASM and the pile10 shot costs 32.91M steps (+46.7 %, 4 transactions ≤ 10M; solve-and-advance
37,960 steps × 152 calls); the batched narrow phase ties across the call (+3.5 % in process: a default nowhere). Codex
audit: merge as is (`is_modified` bookkeeping only; pile10 lacks removals / multi-collider / kinematic coverage; the
API break is avoidable with a separate stage trait). ADR 0001 entry 40. **Programme decisions:** size is the gate,
steps the optimisation (≤ +75 % and ≤ 5 transactions accepted for the first SNIP-36 shot). **CS6 running** (brief
`cs6-caller-under-limit.md`: `StepConfig` back to its released shape with the stages in a separate trait; free_path /
non-basic arms / kinematic preparation / one-way filter / joint-free codec out of the basic configs with the same
`WorldState` bytes; levers 3–5 vs a second orchestration class, both built; the per-layout tables the game needs).
**CX1 running** in parallel (brief `cx1-crossing-cost.md`: awake-only compact deltas across solve-and-advance, folds that
fit with the 10 % margin, the removal / multi-collider / kinematic bit-identity scenario).

**CS6 ✅ #215 — the size gate is met.** The caller `SlimSplitStep` is 28,413 Sierra / **73,181 CASM** felts (547
under 73,728): `StageConfig` stages out (route (a): stage slots, the active-set rebuild out; route (b), an orchestration
class crossing the world once per step, measured at 73,699 CASM but +131.6 % steps: rejected), levers free_path off
(−22,018), basic codec (−6,934, the same `WorldState` felts), kinematic preparation off (−4,154; a kinematic body is
rejected), basic shape kernels (−3,366). New classes `NarrowPhaseClass` 23,597, `ActiveSetClass` 10,930,
`ForceEventsClass` 6,202 CASM. Pile10: 37,219,191 steps (+65.9 %), 5 transactions of ≤ 10M; calldata per transaction:
the world in (1,422–2,978 felts), one input, the world out. `StepConfig` keeps its alpha.6 shape (no break); every
`program.*` and in-process step unchanged. ADR 0001 entry 41. The CI job `sink` is split per crate (a whole-shot run
peaks near the runner's 16 GB). **CX1 running** (resumed after an OOM kill next to CS6; rebases on CS6 and re-measures
the 547-felt margin). **Next (programme):** PK1 package-size rule after CX1 (owner rule 2026-09-28).

**CX1 ✅ #217:** solve-and-advance runs on value arrays with a packed wire (37,960 → ≈ 14,256 steps per call,
`SolveAdvanceClass` 58,547 CASM), islands return decisions the caller replays (13,741 → 2,615, `IslandsClass` 19,083);
folding islands into the advance class is 76,343 CASM (over). The slim layout's pile10 shot: 32,965,606 steps (+47.0 %,
was +65.9 %), 4 transactions of ≤ 10M; the caller 73,083 CASM (645 under the gate; a first pass at 73,662 was brought
back under CS6's 73,181 for +0.14M steps, programme rule). New coverage, bit-identical on every layout:
`tests/game_ticks.cairo` (pile10 as the game plays it: force events every step, seven codec round trips mid-collapse,
damage removals, a second pebble with a velocity) and `tests/removals.cairo` (holes, reused slots, multi-collider,
kinematic, parentless collider, impact wake-up). ADR 0001 entry 42. `bytecode_size.py` prints every declared class's
margin. **Programme decisions:** the +25 % stays a target (ceiling +75 %); **CX2 running** (brief
`cx2-contact-crossings.md`: contact / narrow-phase crossings compacted, measurement first — implemented only if the
caller stays ≤ 73,728 and the shot gains ≥ 2M steps); **PK1 running** (brief `pk1-package-cost.md`: consumer_cost.py
from nalgebra-cairo, gates 1–3 measured — gate 2 is a crate's marginal cost over its direct dependencies, gate 3 the
closures `rapier2d` / `game_classes` —, the test-move gain measured on throwaway copies, CI report-only until alpha.7);
alpha.7 proposed after PK1, not waiting for CX2.

**PK1 ✅ #219:** every published crate passes the package rule (lines ≤ 22,013; marginal ≤ 2.3 s / 0.47 GB; closures
`rapier2d` 8.9 s / 1.85 GB, `game_classes` 9.0 s / 1.95 GB); tests out of the sources would save ≤ 0.04 GB (PK2 dropped);
no cut. **`0.1.0-alpha.7` published** (2026-09-28, programme's written go; #220, release commit a7d7392, tag
`v0.1.0-alpha.7`): the six crates incl. `rapier2d_classes` for the first time; step results unchanged since alpha.6,
shape-cast results changed for touching starts (CN1). The shared `consumer_cost.py` (nalgebra-cairo 7177cf3) now computes
the marginal cost; CI enforces gates 1–3 (`rapier2d` a facade, gate 3 only). **Queued:** DU1 (brief
`du1-dependency-update.md`, Sonnet, measurement only: fixed 0.4.0 / glam 0.4.1, `glam_core` instead of the facade) after
CX2. **Game side:** slingfall B5 (pin alpha.7) and S36a (the game's chunk on `rapier2d_classes`; the slim caller has 645
felts of margin before any game code: rules proposed as declared classes — a per-tick rules class, or the damage rule
folded into `ForceEventsClass` — and a possible `TickHook` stage slot on our side if the spike asks).

**CX2 ✅ #222 (go: caller ≤ 73,728 and shot −2.19M ≥ 2M):** `NarrowPhaseClass` computes the polygon family itself
(11,319 / 23,597 → 22,008 / 68,372 Sierra / CASM) and previous pairs cross as `PreviousPair` (36 felts instead of 64);
lane packing measured and rejected (a packed felt costs about what it saves). The slim layout's pile10 shot 32,965,606 →
**30,773,277 steps (+37.2 %)**, 4 transactions (7.22 / 8.41 / 7.54 / 7.79M); the caller 73,204 CASM (524 under the
gate). Bit-identical (pile10, removals, game ticks). ADR 0001 entry 43. **DU1 running** (Sonnet, measurement only).

**DU1 ✅ #225** (programme go for (a)+(b)): `fixed` 0.4.0 and `glam_core` 0.4.1 instead of `fixed` 0.3.0 / the `glam`
0.3.0 facade; bit-identical in every probe, felts unchanged, class hashes change (type paths), closures −0.19 GB.
**CS7 ✅ #226** (programme request after the game's spike S36a, whose world class was 76,920 CASM): the slim caller
73,204 → **67,076 CASM** (outlined basic-codec readers −4,733, the mass crossing by the basic collider writer −1,395),
the pile10 shot −61,741 steps (+36.9 %); `WorldEditClass` (51,865 CASM: the World edits between steps; a forwarding
caller 68,818, the edits in the caller 96,657); route (b) and the measured-only classes moved to `rapier_sink`. ADR 0001
entry 44. `bytecode_size.py check` gates every class a game declares (incl. `WorldEditClass`, `SlimEditStep`) with at
least 1,000 felts of margin under 73,728. **alpha.8** (programme's written go, 2026-09-29): DU1 + CX2 + CS7. After it,
the class-split queue is empty unless the game's bump reports a need; parity filler one executor at a time, step path
untouched; (c)–(f) parked.

**`0.1.0-alpha.8` published** (2026-09-29, programme's written go; #228, release commit 5a3c0fc, tag `v0.1.0-alpha.8`);
the `execute` job now also runs on every release and dependency PR (#229: `examples/ball_drop` still pinned `fixed`
0.3.0 after DU1). The game is on alpha.8 (slingfall #43: main path bit-identical; the split layout's world class 71,076
CASM, pile10 35.29M steps in 6 transactions). Prelude: `RigidBodyType`, `ShapeTrait` and the basic shapes (#230).
**PX3 ✅ #231** (parity filler, Sonnet): 32 Parry geometry utilities in `rapier_geometry2d`, additions only, golden
family `geometry_utils` (254 cases), step path unchanged — **raw 80.8 %, in scope 88.1 %** (197 missing). **Parity
exclusions (programme decision 2026-09-29):** three new closed reasons, items by exact name, in-scope only — "soft bodies
are not part of the port" (19, incl. `SoftEdgePass` / `SoftVolumePatch` that used to match the EPA pattern), "Q32.32
state cannot become NaN or infinite; nothing to contain" (6; proof `tests/finite_state.cairo`), "static dispatch through
StepConfig / StageConfig (D10)" (7); convex decomposition / voxelisation stay missing ("not ported yet"); the
interaction graph stays in scope (a read-only view is to be assessed after CP3); PX1's deliberately open solver items
(`SolverBodies` API, `SolverPose`, `VelocitySolver`, `ContactRef`, `GraphPos` — the (c) class) stay open. **Raw 80.7 %,
in scope 89.7 %** (167 missing). **CP3 ✅ #235** (contact-pair read API through `ContactPairView`, one view per
collider pair; step path unchanged); the parity matcher reads derived `Default` / `Debug`: **raw 81.9 %, in scope
91.1 %** (145 missing). Launches now wait on `~/orchestrator/capacity.json` (programme rule, one rapier executor at a
time). Interaction graph: 12 of its 16 items can be a read-only view over the pair list (IG1 candidate); `raw_graph`,
`interaction_pair_mut`, `interactions_with_mut`, `InteractionsWithMut` need a persistent, mutable graph. **IG1 ✅
#238:** the 12-item read-only view (`InteractionGraph<E>`, contact and intersection graphs); the 4 mutable / raw-graph
items closed (#237, "persistent mutable graph not ported (D7)"). **Raw 82.7 %, in scope 92.2 %** (127 missing). **PX4 ✅
#240 (the last filler lot):** cheap API items off the step path (set `Index`, change tracking read, contact-pair aliases,
joint graph view, composite ray / point queries, the dyn-shape API's value meanings); `make_mut`, `as_shape_mut` and the
`Shape` trait object closed ("closed value enum replaces Arc / dyn shapes (SH2a)"); the parity parser now reads `impl`
headers wrapped by `scarb fmt`. **PX5 ✅ #242:** `map_attached_joints_mut`
as a real closure-taking function, `handle_user_changes` matched by an owner alias. **Final: raw 84.6 %, in scope
94.5 %** (1,527 ported, 89 missing, 976 excluded of 2,592). **The rapier queue is idle** (programme decision): parked until a game need or the programme's request — the
solver scalar API (tied to (c)), the composite machinery, the sub-shape result widening, `contact_skin`; V-HACD /
voxelisation and `solver_contact_world_points` stay missing; (c)–(f) parked; no release for PX1–PX4 alone.

**Feature-parity waves (from AP #119, `docs/API_PARITY.md`: 24.3 % of 1 997 in-scope items ported, 1 511 missing).**
Every lot closes one AP work package (or part of it) and regenerates the inventory; CI checks it (`api-parity` job).

| Wave | ID | AP package (items) | Notes |
|---|---|---|---|
| 9 | SE | Sensors and intersection events (53) | ✅ #127: 81 golden `intersection_test` cases and `sensor_trigger` exact; P3 ≤ +0.21 %; ADR 20–24 |
| 10 | RB | Rigid-body API completion (123), incl. additional mass properties | ✅ #121: cold data behind `Box<Option<RigidBodyCold>>`; contact scenes +0.05–0.29 %; follow-ups for BT below |
| 10 | WS | Versioned world state save / restore (`WorldState`, `ArenaState` on the three sets), requested by the programme (spike G1b), blocking the game's G3/G4 | ✅ #131: chunked runs bit-exact (K = 1, 7; removals keep handles; sleep + sensor); round trip pile10 1 864 felts / 52k steps, pile20 4 896 / 115k; compact form rejected (−15 %) |
| 10 | CW | Collider API completion (59) + Pipeline and world facade (67) | ✅ #135: 55 ported, 26 excluded (closed reasons), 49 missing with reasons; coverage 30.3 → 33.5 %; removed-collider pairs upstream-exact at zero per-step cost; `CollisionPipeline`, `active_bodies`, `convex_hull` (gift wrap, ≤ 8 vertices). Follow-up (scripts): count `#[derive(Default)]` and `pub type` aliases, exclude `convex_mesh` (dim3), drop the `IslandManager` → `World` owner alias's false positives (`new`) |
| 10 | G0 | Level-shaped golden scene (8–12 sleeping blocks + cores + 15–25 m/s pebble, 300 ticks, 20-block variant; substeps 4/2/1, 30 Hz) + cost of a level | ✅ #133: matrix in `docs/BUDGETS.md` (level 10, 60 Hz × 4: 204M steps / 300 ticks); upstream fidelity breaks at the impact tick (ADR entry 9) |
| 10½ | BT | **Cairo steps** per awake body-tick — decided (programme, 2026-09-25): right after G0, ahead of waves 11–14. BT1 (brief `bt1-awake-contact-steps.md`): solver + narrow phase of an awake contact tick (77 % + 16.5 % at impact), −30 % steps on the impact windows. BT1 ✅ #139: −41.0 % / −37.5 % Cairo steps on the L10 / L20 impact windows, bit-identical. BT3 ✅ #146: −18.8 % / −16.3 % more on the impact windows (goal −25 % not reached: the rest is pipeline-side or ≈ 17 steps per fixed-point rescale), bit-identical. `0.1.0-alpha.2` published (BT1 + BT2 + CW, tag `v0.1.0-alpha.2`, `CHANGELOG.md`). BT4 ✅ #151: impact windows −2.8 % / −9.0 %, P3 −1.2 % to −3.8 %, activation reads 146 → 88 steps, bit-identical, `WorldState` still v2. The L10 impact tick is ≈ 360k steps (812k at G0). Remaining step levers: narrow phase ≈ 101k and island solve ≈ 194k per impact tick (BT3 reports ≈ 17 steps per fixed-point rescale as the floor of the arithmetic), L20's mixed-tick excess ≈ 45k per tick (dormant pairs out of `narrow_phase.pairs`: `WorldState` v3). `0.1.0-alpha.3` published (BT3 + BT4, tag `v0.1.0-alpha.3`, bit-identical to alpha.2, `WorldState` v2). BT2 ✅ #143 (all-asleep engine step 36k → 1k steps, flight tick 60k → 16.5k on L10, bit-identical; bodies inserted asleep and activation reads per G3; P3 +1–3 % from the sets' `modified` field → BT4: a modified bit and a field accessor in `rapier_core::Arena`): the flight / all-asleep tick (broad phase 38 %, islands 31 %, user changes 19 % of a 61k-step flight tick; an all-asleep level-10 tick still costs 3.37M gas) — sleeping bodies should cost ≈ nothing. Also RB's follow-ups: its no-contact fast path (`pipeline/free_path.cairo`) makes the P3 free-fall probes 6.5–20 % cheaper but hides a +13 % net cost of the general path on contactless worlds (`free_fall32` without it: 16.81M vs 14.88M on main) — find it, then keep or drop the fast path; world construction +3.7 % (≈ +59k gas per inserted body) | sized by G0 |
| 10¾ | CS | **Contract class size of the 2D step** — blocker of the on-chain path (programme, 2026-09-26): the game's `Slingfall` class stepping a world is 201,974 Sierra felts (limit 81,920) and 11.7 MB (limit 4,089,446 bytes), so it cannot be declared nor run in the SNIP-36 virtual OS. CS1 (brief `cs1-class-size-budget.md`): a `Sink` fixture contract + `scripts/bytecode_size.py` + CI budget, the size decomposition, lever estimates in size AND steps (inlining strategy, `#[inline(never)]` on the largest bodies, feature-gated shape pairs / joints, dispatch dedup, a multi-class layout), the floor. CS2: the levers, after BT3 | CS1 ✅ #148 (`crates/rapier_sink`, `scripts/bytecode_size.py`, CI `bytecode` job, `docs/research/class-size.md`): the game's step is 435,454 CASM felts (5.32×); **no lever combination fits one class** — floor 137,247 CASM (1.68×) at +72 % impact steps; CS2's zero-step levers (step generic over the dispatcher and joint / sensor strategies −49 %, no `free_path`, dispatch dedup) leave ≈ 2.2–2.4×; a 4–5-class chain costs ≈ 77k steps per `WorldState` crossing (+83–104 % per impact tick). Proof path: Stone + Integrity on the standalone executable. **Programme decision (2026-09-26):** the in-class SNIP-36 path is closed for the MVP; the proof path is Stone + Integrity on the standalone executable. CS2 goes to the backlog (revisit if Starknet raises the class limits, if SNIP-36 accepts a cheap multi-class layout, or when a game needs the generic dispatcher — also the upstream-faithful shape, so a candidate free parity item) |
| 11 | LO1 | Parity leftovers of the rigid-body, collider and sensor packages (≈ 64 items, mostly internals already triaged by CW / QY1) | ✅ #170: 25 of 60 closed (activation defaults, `Default` impls, `ShapeIntersection`, parry `intersection_test_*` wrappers), 35 left with reasons (no interaction graph / persistent islands, dict-backed `Index`, V-HACD, triangles → SH1, composites → SH2, dead upstream code); five trivial items outside its allowlist (`Aabb::{aligned_intersections, intersects_moving_aabb}`, `NarrowPhase::*_unknown_gen`, `MotorModel::combine_coefficients`, `RayIntersection::with_subshape`) → LO2 ✅ #178 (4 ported; `RayIntersection::with_subshape` waits for composites: the field would widen every ray result). `0.1.0-alpha.4` published (tag `v0.1.0-alpha.4`: the free parity round and SH1; breaking `QueryFilter.flags`) |
| 11 | SH1 | Additional 2D shapes, part 1: triangle, round shapes | ✅ #174: `Triangle`, `RoundCuboid` (inline), `RoundTriangle` / `RoundConvexPolygon` (boxed): `Shape` stays six felts; every query and manifold; golden `triangle_contacts` / `round_shape_contacts`; old shapes' exact steps unchanged (before / after table); ADR 27–29 |
| 11 | QY | Query completion (342), part 1: per-shape point/ray/distance/contact/closest-points queries | QY1 ✅ #152: `rapier_geometry2d::query::{distance, closest_points, contact, intersection_test}` for every pair of the closed set (analytic kernels where parry uses GJK / EPA), `PointQuery` / `RayCast` / `PointQueryWithLocation` impls, `Aabb` queries, manifold utilities; golden `shape_queries` (81 cases × 6 queries, both orders); a parry defect fixed (ADR 26); step, snapshots and class size unchanged; parity 38.8 %. QY2 ✅ #163 (`intersect_shape`, `project_point_and_get_feature`, `intersect_aabb_conservative`, `QueryPipeline` view, `QueryFilterFlags`, `exclude_solids`; breaking: `QueryFilter.flags` is typed). Left in QY: composite shapes with SH2, shape casts with CC. After BT4: free parity items and the remaining step levers (impact tick: narrow phase 101k, island solve 194k) |
| 12 | CC | CCD and shape casts (91) — **first** (programme, 2026-09-26: CCD on the pebble guards thin planks if slingfall drops to 2 or 1 substeps) | CC1 ✅ #180 (linear / nonlinear casts and swept TOI for the closed set, `cast_shape*` on `World` / `QueryPipeline`; golden `shape_casts`, `nonlinear_shape_casts`, `sweep_toi`; ADR 30–32; parity 66.5 %); CC2 ✅ #182: `World::step_with_ccd(ref CCDSolver)`; `World::step` unchanged (609 step probes identical); automatic CCD off by default (+22 % flight steps on level 10 if on; ADR 33–34); a bullet pebble costs +22.7 % flight / +14.1 % impact steps on level 10; `WorldState` v3 |
| 12 | SH2 | Additional 2D shapes, part 2: polyline, compound, 2D heightfield (multi-manifold pairs) | SH2a ✅ #184 (polyline + 2D heightfield, boxed; composite pairs as consecutive `ContactPair` entries; golden `composite_contacts` + 3 scenes; existing pairs +2 steps per pair per step; ADR 35). SH2b ✅ #187 (compound shapes; golden `compound` families + 3 scenes; P3 / level steps identical; ADR 36; parity 73.9 %). `0.1.0-alpha.5` published (CC1 + CC2 + LO2 + SH2a). **RG1** ✅ #189: SH2a's force-event collect (+1.49 % with force events; P3 / levels have none) — fixed, the game-shaped probe is −0.11 % vs alpha.4; the `game_path` probes are now part of every steps-unchanged proof (executor frame, AGENTS §7); RG2 (brief `rg2-shape-serde.md`): `ShapeSerde` +676 steps per round trip. **SF1** ✅ #191: a **port defect** — the frozen contact separation was not rebased on the anchors' floored round trip, so exactly closed gaps went soft; fixed, `box_slope_slide` un-ignored (step 4: 26 184 → 5 ulps), `ell_topple` and a new `tilted_landing` scene within bands; results change (alpha.6 carries it, programme decision). Left: the persisted graph order at a new pair (SO / DO class, documented). RG2 running. **CS2** ✅ #193 (`StepConfig`, `BasicStepConfig`: game program 565k → 231k felts, −59.1 %, bit-identical, ≤ steps; ADR 37; joins alpha.6 by programme decision) (brief `cs2-reachable-step.md`, programme decision 2026-09-27: the settled path Pedersen-hashes the whole program, 582k felts ≈ 4.6M steps for slingfall) |
| 13 | JA | Joint API completion (239) | JA1 ✅ #157 (claude sonnet): 170 of 239 items — typed joints (`FixedJoint`, `RevoluteJoint`, `PrismaticJoint`, `PinSlotJoint`, `RopeJoint`, `SpringJoint`) as views over `GenericJoint`, accessors, setters, builders, `ImpulseJointSet` queries, `World::impulse_joints(_with)`; joint scenes' exact steps and the class size unchanged; the 69 left are solver internals, the joint graph, `user_data` (layout rule) and MH1 items — no JA2 planned |
| 13 | KC | Character controller (12), PID / vehicle controllers (35) | deferred (no game need; PID may come as a free item) |
| 14 | MH, PO | Mass/AABB/shape helpers (101), API polish (179) | MH1 ✅ #160: 91 of 100 items + 9 neighbours (`Aabb` members, `BoundingSphere` / `BoundingVolume`, `SupportMap` / `PolygonalFeatureMap`, feature ids, per-shape bounding volumes, `MassProperties` members; exact sphere tests kept over cheaper inexact ones); step and class size unchanged. The parity script now drops upstream methods behind `#[cfg(feature = "dim3")]` and reads Cairo `pub type` (inventory refreshed from the pinned checkouts: 2,667 items, **54.1 %** ported). PO1 ✅ #167: 168 items triaged — +26 ported (`AxesMask`, `BodyStatus`, `set_dt`, `is_convex`, `Into<Shape>`, capsule / segment / polygon helpers), +89 excluded through closed reasons (72 soft-body internals, mesh converters, serde, SIMD lanes), the rest left with reasons (no interaction graph, composites with SH2, `dyn Shape`, Parry's dead `polygon.rs`, the scalar solver-body API). Script follow-up: follow Parry's module tree, read `pub const fn` |

**Resumed 2026-09-25 (owner: "continue towards feature parity with rapier-rs").** Wave 9 starts with **AP** (a
generated API-parity inventory against rapier-rs 0.35.3 2D + the parry2d subset it exposes, `scripts/api_parity.py`
→ `docs/API_PARITY.md`, brief `ap-api-parity.md`) — the remaining lots are cut from its "missing" groups — and
**SE** (sensors: intersection pairs and events, brief `se-sensors.md`). Known gaps before the inventory: sensors,
triangle / round / polyline / compound shapes and 2D heightfield, segment–segment and segment–capsule contacts,
`cast_shape` / `intersect_shape` queries, CCD, character controller, pin-slot joint, additional mass properties.

**PAUSED 2026-09-25 (owner's decision), resume point below.** RJ ✅ (#116, claude opus; squash commit `a015c65`
carries a wrong title from `--fill-first` on a stale local `main` — content verified): upstream's
`limit_linear_coupled` / `motor_linear_coupled`, `RopeJointBuilder`, `SpringJointBuilder`; golden `rope_pendulum`,
`spring_mass`, `spring_mass_accel` pass 120 steps; one rope/spring joint step ≈ 4.1–4.2M gas; existing joints
bit-identical. Repositories renamed to `bal7hazar/{rapier,glam,nalgebra}-cairo`; `fixed`, `glam`, `glamx` split into
`fixed-cairo`, `glam-cairo`, `glamx-cairo` (registry packages unchanged). Open for the owner: the parry split
(`docs/proposals/parry-split.md`). **Resume point:** wave 8 is done except RS (round shapes); then wave 9 (ST,
`rapier_starknet`, gated on the target-runtime question) or the parry split; small follow-ups: pipeline per-body
hoist (sleep overhead +4.6 % on free fall), `joints()` empty-loop guard (JM), prelude re-exports for rope/spring.

EV ✅ (#114, codex gpt-6-astra high): contact-force events as upstream (normal impulses / dt, strict threshold, the
minimum enabled threshold, ascending pair order, `WorldTrait::step_with_force_events`; `step` unchanged) and one-way
platforms as a built-in collider flag implementing upstream's example rule (ADR 18); golden `one_way_jump` and
`force_event_drop` pass 120 steps (event at step 34 exactly); P3 overhead ≤ +1.05 % (free fall 1 body).

KD ✅ (#111, codex gpt-6-astra high): **bug fixed** — kinematic contact endpoints keep their velocity and substep poses
(fixed and dominance-superior endpoints stay WORLD, as upstream); `set_next_kinematic_position/translation/rotation`
with velocity interpolation before contacts and joints, kinematic velocity-based bodies, `set_dominance_group`,
`RigidBodyBuilder`; golden `kinematic_platform`, `kinematic_pusher`, `dominance_stack` pass; P3 overhead ≤ +0.73 % gas.

JM ✅ (#109, claude opus): joint kinds specialised once per step (`Plain` / `Controlled` / `Legacy`), one frame
construction, interior limits with zero impulse not emitted, a "gas wallet" (AGENTS §7): inactive limit +7.7 % over
a plain joint (was +63 %), active limit +29 %, motors +21–25 %, plain chains −2.3 / −2.8 % below their pre-JL cost;
new probes `pendulum_limited`, `wheel_motor`. KD (running) found a **bug**: the contact solver turned every
zero-inverse-mass endpoint into WORLD, so kinematic bodies could neither push nor carry (upstream passenger
1.0046 m/s, port 0); KD fixes it first. Launched **RJ** (brief `rj-rope-spring-joints.md`).

CP2 ✅ (#107, codex gpt-6-astra high): polygon–polygon, polygon–cuboid, polygon–segment, polygon–capsule contacts by
SAT + clipping (no GJK/EPA), both dispatch tables; 48 golden cases within 64 raw units, feature ids exact where
unambiguous; cold pair 0.77–0.99M gas; P3 scenes ≤ +0.18 %. Every pair of the closed shape set now has a
generator except segment–segment / segment–capsule / capsule–segment (unsupported as in phase 1). Launched **KD**
(brief `kd-kinematic-dominance.md`).

CP1 ✅ (#105, codex gpt-6-astra high): `Shape::ConvexPolygon(Box<ConvexPolygon>)` (3–8 vertices, CCW, strict
construction), AABB, mass, analytic point projection and ray cast, `ColliderBuilder::convex_polygon`; polygon–ball
and polygon–half-space contacts already work through `convex_ball` / `halfspace_pfm`; 54 golden cases; every P3
scene within +0.72 % (several cheaper). ADR entries 13–16. Next **CP2** (brief `cp2-polygon-contacts.md`,
generator modules pre-declared).

SC ✅ (#103, claude sonnet): the sleep timer decides fast-moving bodies without square roots (felt252 path),
bit-identical; free-fall overhead of sleeping +6.9 % → +4.6 % (8 bodies) / +4.8 % (32); the remaining ~2.5 %
lives in the pipeline's per-body walks (census, eligibility) → a later pipeline lot can hoist per-step
constants. Launched **JM** (brief `jm-joint-row-cost.md`): limit/motor rows paid only when enabled.

SI ✅ (#100, codex gpt-6-astra high): `ball_drop_sleep`'s post-impact gap is an **upstream defect** — on the wake step
upstream leaves the revived dormant ground pair out of the solver, so the woken lower ball sinks ~12 cm for one
frame; the port supports it immediately. Delaying that pair in the port closes the gap from 0.37 m to 50 ulp; a
reverse Rust control (upstream with a pre-wake) matches the port. Kept as a deliberate divergence: first ADR,
`docs/adr/0001-upstream-divergences.md`, registers the 12 divergences found so far.

JL ✅ (#96, codex gpt-6-astra high, second run after the exporter was added to its allowlist): revolute angle
limits (`atan2` from `fixed::trig`), prismatic limits, velocity/position motors with `max_force` and both motor
models, upstream setters/builders; golden `pendulum_limited`, `wheel_motor`, `slider_limited`, `servo` pass 120
steps with zero violations (max 2 921 ulp). Cost: a one-joint step 4.34M plain, 7.07M with a limit (inactive or
active — the row is charged either way), 6.89M with a motor; plain pendulum frames +1.6–1.8 % → optimisation
lot later. Launched **SI** (brief `si-sleep-impact.md`): the `ball_drop_sleep` post-impact divergence.

SL ✅ (#95, claude fable): sleeping on by default with upstream's thresholds — per-step union-find islands over
touching pairs and enabled joints, upstream's `update_energy` timer (no persisted previous pose, D9), sleeping
bodies' proxies static and their dormant pairs skip narrow phase and solver; wake-ups on contact with an awake
body, joints, user changes, forces/impulses (new `RigidBodyTrait::{wake_up, sleep, add_force, apply_impulse,
…}`). A sleeping `cuboid_stack10` step costs **2.64M gas | 25k steps = 6.9 % of the awake step**. Golden
`box_stack3_sleep` and `ball_drop_sleep` sleep and wake at upstream's steps. Divergences (documented): islands
per step, mixed islands wake strongly, partner wake-ups one step later. Open: awake overhead +1–2 % on contact
and joint scenes but +7 % on free fall (lot **SC**, brief `sc-sleep-timer-cost.md`); `ball_drop_sleep` samples
diverge after a 13 m/s ball-on-ball impact (0.37 m at step 110, flags still exact) → investigation lot after JL.

QP ✅ (#93, claude opus): ray casts on the five shapes (64 golden cases: times of impact to the ulp, capsule
analytic where upstream uses GJK — upstream's hollow support-map cast mixes length and direction units, the
port returns the true exit), `World::{cast_ray, cast_ray_and_get_normal, intersect_ray, project_point,
intersect_point, intersect_aabb}` with `QueryFilter` (brute force: `cast_ray` 5.3M gas at 32 colliders; the
grid variant loses below 128). Build locks reworked (#92): per-project lock + shared heavy lock only for
workspace-wide runs. Launched **JL** (brief `jl-joint-limits-motors.md`, codex).

**Status 2026-09-24:** phase 1 is complete except the proven-step milestone (blocked on hardware:
Stwo > 22 GB). Phase 2 starts with wave 6; later waves are refined after each merge.

| Wave | ID | Package | Notes |
|---|---|---|---|
| 6 | SL ✅ #95 | Sleeping: per-step union-find islands, sleep timers (D9), upstream wake-up points, golden sleep scenes | brief `sl-sleeping.md`; biggest gas win left for resting games |
| 6 | QP ✅ #93 | Scene queries: per-shape ray casts, `World::{cast_ray, intersect_ray, project_point, intersect_point, intersect_aabb}`, `QueryFilter`, golden `ray_casts` family | brief `qp-queries.md`; modules `rapier_geometry2d::ray`, `rapier2d::queries` pre-declared |
| 7 | JL ✅ #96 | Joint limits and motors (revolute angle via `fixed::trig`, prismatic linear), motor models | needs a golden joint-limit scene |
| 7 | CP1 ✅ #105 | Convex polygon shape (≤ 8 vertices): `Shape::ConvexPolygon` (approved interface change), AABB, mass, analytic point projection and ray cast (upstream: GJK), golden cases | brief `cp1-convex-polygon-shape.md` |
| 7 | CP2 ✅ #107 | Polygon contact generators: SAT + clipping vs polygon/cuboid/segment/capsule, ball and half-space through CP1, dispatch, golden manifolds (upstream: PFM–PFM with GJK/EPA) | after CP1 |
| 8 | KD ✅ #111 | Kinematic position-/velocity-based bodies (`set_next_kinematic_*`, velocity interpolation), dominance end to end, golden scenes | brief `kd-kinematic-dominance.md` |
| 8 | EV ✅ #114 | One-way platforms (built-in collider flag, no hooks), contact-force events (threshold, total/max force) | brief `ev-events-one-way.md` |
| 8 | RJ ✅ #116 | Coupled limits/motors, rope and spring joints | brief `rj-rope-spring-joints.md` |
| 8 | RS | Round shapes (round cuboid / polygon) | |
| 9 | ST | `rapier_starknet`: storage packing of the persistent state (D9/D10), demo game contract | out of the near plan: the game verifies proofs on-chain, it does not store worlds |

### Phase 3 — Robustness and controllers

CCD substitute (velocity caps + speculative margin) then trig-free `sweep_toi` if budgets allow,
character controller, flat compound shapes, GJK-2D as a test oracle, 2D heightfield.

### Phase 4 — 3D

`Vec3`/`Quat`/`Mat3` from glam-cairo, `SdpMatrix3` world inertia, 4-point manifolds with reduction,
15-axis SAT + polygonal feature clipping, two-tangent friction, 6-row joints. Started only once
the 2D gas profile is known.

### Explicitly out of scope

Soft bodies, multibody joints, parallel/SIMD/graph colouring, BVH, trimesh/voxels/3D heightfield,
EPA, mesh `transformation/`, serde/rkyv, debug-render, profiling counters, `dyn` hooks.

## 5. Orchestration protocol

The procedure is [`docs/ORCHESTRATOR.md`](ORCHESTRATOR.md), on top of the standard of the organisation and
`slingfall/OPERATIONS.md`; since 2026-10-02 the orchestrator is a herdr coordinator and the tasks are herdr threads. In short:

1. Before a wave, the orchestrator merges the interface it depends on and **pre-declares the stubs**
   (module lines in `lib.cairo`, empty test files, `gas/<crate>/<module>.snap` targets) so that
   parallel PRs never touch a common file.
2. One brief per package in the mandatory format of `AGENTS.md` §3 (`docs/briefs/<id>.md`), committed,
   then started as a herdr thread with a profile (`impl-sonnet`, `impl-opus`) on the machine that
   `machine-capacity` and the placement rule give; the in-session Agent tool only for short read-only research.
3. Each thread runs the gate in the foreground, regenerates the snapshot of its own modules, runs
   `scripts/prepush.sh`, pushes, opens its PR, drives CI to green, never merges, and writes its report.
4. The orchestrator reads the report, reviews it (API parity, deviations, gas and steps tables), has the PR
   reviewed by a review thread on another model, has it merged on its line `Merge the PR: ...`, then alone
   updates re-exports, status, decisions and this file.
5. A stopped thread is prompted to resume, not started again.

Executor runs so far (claude CLI, second account; before the per-module snapshot and self-opened
PR flow): G1 Sonnet 50 turns $1.6 · C2 Sonnet 72 turns $3.5 · M3 Opus 121 turns $15.2 ·
C3 Sonnet 97 turns $5.1 · G2 Sonnet 130 turns $8.5.

Parallel width: wave 1 = 2, wave 2 = 3, wave 3 = 8, wave 4 ≈ 11, wave 5 = 3.

## 6. Contract expected from glam-cairo (D12)

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
| Schedule coupling with glam-cairo (gate X1) | Only `fixed` F1+F2 and `Vec2` block; wave 1 is independent; fallback = vendored snapshot of the same code |

## 8. Open questions for the owner

1. Should the Parry port eventually live in its own `parry-cairo` repository (the crate layout already allows it)?
2. Target runtime for the first game: Starknet contract, Dojo world, or client-side proving with `scarb prove`? It decides how early `rapier_starknet` is needed.
3. Is 2D-first acceptable, or is there a 3D game already in sight?
