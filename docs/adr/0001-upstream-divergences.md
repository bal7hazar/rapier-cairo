# ADR 0001 — Deliberate divergences from upstream Rapier / Parry

Status: accepted (2026-09-24). Owner: orchestrator. Updated whenever a lot finds a new one.

## Context

`AGENTS.md` §2.5: upstream is the reference, and we diverge only for a reason that is written down.
Validation is tolerance-based against golden vectors generated from the real `rapier2d-f64 0.35.3` /
`parry2d-f64 0.30.2` (D11). Some divergences are forced by fixed point, some are upstream defects the
port does not reproduce, some are design decisions of this port. Each entry names the lot that measured
it and where the evidence lives.

## Decision — the registry

| # | Area | Upstream behaviour | Port behaviour | Why | Evidence |
|---|---|---|---|---|---|
| 1 | Cuboid feature ids (f64 builds) | `vertex_feature_id` reads f32 sign bits → every f64 cuboid vertex id 0, face id `0b110000`; warm start of both regenerated points shares one point's data | correct ids (the f32 scheme) | upstream defect | wave-1 outcome; GS #66 regenerates scene traces with a vendored parry patched `>> 63 / >> 62` (`tools/golden/vendor/README.md`) |
| 2 | Contact solve order (D8) | touching pairs get persistent colours (non-fixed pairs lowest free colour, fixed pairs highest), solved by colour | stable partition of the ascending pair list: non-fixed pairs first, fixed-body pairs last | stateless (D9); equals upstream's order whenever the non-fixed colours ascend in pair order (every golden scene) | SO #76, DO #80; a 4-box chain differs |
| 3 | Exact zero gap (`box_slope_slide`) | f64 residue lands a contact on the rigid side of `dist <= 0` | exactly `0` in Q32.32 → soft side | fixed-vs-float tie, not a defect | GS #66 (`test_slide_zero_gap_counterfactual`) |
| 4 | Capsule hollow ray cast | support-map cast mixes length and direction units (`|dir| ≠ 1` gives a wrong exit time) | analytic capsule, true exit | upstream defect | QP #93 (`ray_golden`, `capsule/inside`) |
| 5 | Capsule ray/contact normals | GJK search direction | analytic normal (≤ 285 ulp apart) | no GJK in the port (phase 1 scope) | QP #93, GF3 #44 |
| 6 | Islands | persistent island manager | islands rebuilt every step by union-find; only per-body activation persists | D9 (persisted state minimal) | SL #95 |
| 7 | Island wake strength | the awake toucher is woken weakly | a mixed island wakes strongly as a whole | per-step islands | SL #95 |
| 8 | Contact-partner wake-ups of modified colliders | same step | next step's user changes | stateless pipeline order | SL #95 |
| 9 | Revived dormant pairs on the wake step | left out of the solver for one step (a woken ball resting on the ground sinks ~12 cm for a frame) | solved in the wake step | upstream defect; delaying that one pair in the port reproduces upstream within 50 ulp | SI #100 (`sleep_diagnostics`, reverse Rust control) |
| 10 | Division rounding | f64 | `fixed` 0.3.0: `/`, `recip`, `from_ratio` round to nearest-even; products and wide kernels floor once | Q32.32 | BX #64 |
| 11 | Rigid joints / tiny CFM (D4) | f64 `cfm ≈ 1.5e-9` | rigid rows special-cased below the representable cfm | Q32.32 | C2 #10, DE #32, JL #96 |
| 12 | Revolute limit angle | f64 `atan2` | `fixed::trig::atan2` (≈ ±1 ulp) | Q32.32 | JL #96 |
| 13 | Convex polygon construction | `from_convex_polyline` tolerates some degenerate input | strict: rejects duplicates, redundant collinear vertices, clockwise, concave, self-intersecting (`None`) | fixed-point robustness | CP1 #105 |
| 14 | Convex polygon point projection / ray cast | GJK (search-direction features, EPA inside) | analytic: nearest edge/vertex, half-plane clipping; ties pick the first edge | no GJK/EPA in the port | CP1 #105 |
| 15 | EPA depth | EPA can return an approximate penetration (a pentagon-origin case reports −1.5) | exact nearest distance (−1) | upstream approximation | CP1 #105 (`polygon_point` regression) |
| 16 | `Shape::ConvexPolygon` storage | inline shape | `Box<ConvexPolygon>` so `Shape` stays six felts (inline storage measured +7 % on a one-body free-fall step) | Cairo value-type cost | CP1 #105 |
| 17 | Polygon contact manifolds | PFM–PFM: GJK/EPA + polygonal-feature clipping | SAT over face normals + the same clipping; exact penetration; first face on exact ties | no GJK/EPA in the port | CP2 #107 |
| 18 | One-way platforms | no built-in: users write a `PhysicsHooks::modify_solver_contacts` hook (example `one_way_platforms2`, test `issue_752`) | built-in collider flag `ColliderBuilder::one_way(local_up, allowed_angle)` implementing the example's three-state rule (unknown / allowed / forbidden, persisted in the manifold `user_data`; both platforms must accept when both are one-way) | D10: no `dyn` hooks in Cairo | EV #114 |
| 19 | `ContactForceEvent::started` | absent from the published `rapier2d-f64 0.35.3` | present (from the pinned clone `28d0ba9`), with an independent force-start bit | reference clone is 0.35.3 + 4 commits | EV #114 |
| 20 | Intersection-pair storage | a separate intersection graph | the same `NarrowPhase.pairs` list, marked by status bits, empty manifold (a second array measured +4.5k gas per step on every scene) | Cairo value-type cost (D9) | SE #127 |
| 21 | Event order within a step | contact events, then intersection events | one sequence ascending by pair key (D8 determinism) | stateless pipeline order | SE #127 |
| 22 | Sensor pair on a removed collider | `Stopped \| SENSOR \| REMOVED` emitted by `handle_user_changes` without waking the partner | the partner's parent is woken so that the dormant pair ends at the next step (same event); to be made upstream-exact by CW | dormant pairs are skipped by the step | SE #127 |
| 23 | Solid ↔ sensor switch; dropped pairs | the pair keeps its graph edge; `Stopped` decided from `intersecting` and the current event flags | the pair ends (`Stopped` if started) and restarts as the other kind; `Stopped` decided from the stored start-emitted bit | one list for both kinds | SE #127 |
| 24 | `intersection_test` kernels | GJK for most pairs; the query dispatcher is pluggable | analytic distance or SAT per pair (touching ⇒ intersecting, as upstream on the 81 golden cases); not pluggable | no GJK/EPA in the port; D10 | SE #127 |

## Consequences

Golden comparisons that cross one of these entries are either regenerated from a corrected reference (1),
judged on invariants or strict up to the divergence point with a counterfactual proving the cause (3, 9),
or compared on the quantities the divergence does not touch (4, 5). Entry 9 decides the fidelity of level-shaped
scenes (G0 #133): a level's traces match upstream up to the impact tick and diverge from it, because the impact wakes
the structure; pre-waking the structure in the upstream run brings the pebble within 0.003 m/s (`level_checks::
prewake_before_impact`). The port keeps its behaviour (upstream defect); level replays are judged on invariants after
the impact. A new divergence found by a lot is
added here by the orchestrator in the PR that merges the lot's findings.
