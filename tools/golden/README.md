# tools/golden — golden vectors from upstream Rapier / Parry

Work package **G0**, decision **D11** of `docs/PLAN.md`. This Rust tool runs the *real* Rapier
and Parry (2D, `f64`) on deterministic inputs and records what they answer. The Cairo port is
validated against these records with tolerances, never bit for bit.

```
tools/golden/
  Cargo.toml, Cargo.lock      exact upstream pins
  src/                        the harness (one module per vector family) + the Cairo generator
  vectors/*.json              committed reference vectors (source of truth)
crates/rapier_golden/
  src/types.cairo             hand-written data carriers (raw i64 only)
  src/compare.cairo           hand-written tolerance helpers
  src/generated.cairo         generated module index
  src/generated/*.cairo       generated fixtures — do not edit
  tests/sanity.cairo          closed-form checks of the fixtures
  tests/scenes.cairo          shape and physics checks of the scene traces
  tests/<leaf family>.cairo   closed-form / consistency checks of each G2 family (see below)
```

## Regenerating

```sh
cd tools/golden
cargo run --release            # vectors/*.json, then the Cairo fixtures, then `scarb fmt`
cargo run --release -- vectors # JSON only
cargo run --release -- cairo   # Cairo fixtures only, from the committed JSON
```

Requirements: a Rust toolchain (built with cargo 1.97) and `scarb` on the `PATH` (the last step
runs `scarb fmt --package rapier_golden`; `GOLDEN_SKIP_FMT=1` skips it, in which case run
`scarb fmt --workspace` yourself).

Regeneration is **idempotent**: a second run must leave `git status` clean. There is no clock, no
RNG and no hash-map iteration anywhere in the tool; JSON keys keep their insertion order; cases are
listed in source order. Rapier is built with `enhanced-determinism` (libm instead of the platform
`sin`/`cos`), so the vectors should not depend on the host either. If a run does produce a diff,
either an upstream pin moved or the platform differs: investigate before committing.

Changing a pinned version is its own PR: regenerate, review the vector diff, re-run the Cairo
tests.

## Pinned versions

| crate | version | why |
|---|---|---|
| `rapier2d-f64` | `=0.35.3` | latest release on crates.io (2026-09). Built with `default-features = false` and `dim2, f64, std, enhanced-determinism`, i.e. **without `block-solver`** |
| `parry2d-f64` | `=0.30.2` | the version `rapier2d-f64 0.35.3` depends on (`^0.30.2`). `parry2d-f64 0.31.1` exists on crates.io but no published Rapier uses it; pinning 0.31 would silently put two Parry copies in the build. Built from `vendor/parry2d-f64` through `[patch.crates-io]`, with the one-line feature-id fix of [`vendor/README.md`](vendor/README.md); Rapier links the same patched copy |
| `parry2d` (f32) | `=0.30.2` | feature ids only, see [Feature ids](#feature-ids-f64-upstream-bug) |

Note that the local upstream clones used by the research reports are Rapier `master` (0.35.x +
soft bodies) and Parry 0.31.x: the vectors come from the **published** pair above (Parry with the
one-line feature-id patch of `vendor/`), not from the clones.

## Quantisation rule

The port computes in signed Q32.32 stored as a raw `i64`: `value = raw / 2^32`.

- **Inputs** (shape parameters, poses, densities, friction, gravity, `dt`, prediction distance …)
  are built from a raw `i64` and converted with `raw as f64 / 2^32`, which is exact. Decimal
  wishes such as `0.3` or `9.81` are first snapped to the nearest raw. Both engines therefore start
  from *identical* numbers, and no comparison tolerance is spent on input rounding.
- **Rotations** are unit complex numbers `(re, im)` whose two components are Q32.32 values:
  `(round(cos θ · 2^32), round(sin θ · 2^32))`. `2^64` is not a sum of two non-zero squares, so no
  rotation other than the multiples of 90° is exactly unit in Q32.32: the norm is `1 ± 2^-32`. The
  pair is handed to upstream **as is, without renormalisation**, and that exact pair is what the
  vectors record. The 30° slope uses `im = 0.5` (exact) and `re = 3719550787 / 2^32`.
- **Outputs** are emitted twice: the upstream `f64` (JSON number, shortest round-trip form) and
  `raw = round(f64 · 2^32)` with ties away from zero. `-0.0` is normalised to `0.0`. The tool panics
  on a non-finite or out-of-range output instead of saturating.
- Two upstream defaults are not Q32.32 numbers: the prediction distance `0.02` and `dt = 1/60`.
  The manifold family uses `prediction = 85899346 / 2^32`, the scenes use
  `dt = 71582788 / 2^32` (so the substep `dt / 4 = 17895697 / 2^32` is exact too).
  `integration_parameters.json` carries the derived quantities for both the `f64` and the
  quantised `dt`.

JSON encoding: a scalar is `{"f64": x, "raw": n}`, a vector or rotation is
`{"f64": [a, b], "raw": [ra, rb]}`. `raw` values are JSON integers and can exceed 2^53 (e.g. the
joint angular frequency): parse them as 64-bit integers, not as doubles.

## Vector families and case ids

A case id is `<group>/<name>`, ASCII, at most 31 characters (it must fit a Cairo short string),
stable across regenerations. The Cairo constant of a case is its id in upper case with every
non-alphanumeric character replaced by `_` (`cuboid/rot-135` → `CUBOID_ROT_135`).

| file | cases | id convention | content |
|---|---|---|---|
| `integration_parameters.json` | defaults + 2 | `dt_f64`, `dt_q32` | every `IntegrationParameters` field; for `dt` and 4 solver iterations: `inv_dt`, substep `dt` / `inv_dt`, length-scaled limits, and `erp_inv_dt`, `erp`, `cfm_coeff`, `cfm_factor` of the contact, static-contact and joint springs **at the substep length**, all from the upstream functions |
| `mass_properties.json` | 11 + 1 | `<shape>/<params>_d<density>`, `compound/<parts>` | `Shape::mass_properties(density)` for ball, cuboid, capsule (incl. oblique and zero-length); a Rapier body with two colliders (local properties, world COM, effective inverse mass / inertia), cross-checked against the Parry sum |
| `aabb.json` | 32 | `<shape>/<pose>` | `Shape::compute_aabb(pose)`, 4 shapes × 8 poses (identity, translation, exact 90°/180°, 30°, 45°, −135°, 1° far from the origin) |
| `contact_manifolds.json` | 87 | `<shape1>_<shape2>/<regime>` | `DefaultQueryDispatcher::contact_manifolds` with the default prediction distance; see below |
| `scenes.json` | 6 | scene name | full-engine traces (also exported to Cairo, see [Scene fixtures](#scene-fixtures)) |
| `pose2.json` | 7 + 3 | `pair/<what>`, `chain/deg<angle>` | **G2** 2D pose algebra and rotation drift, see [Leaf-level families](#leaf-level-families-g2) |
| `aabb_overlap.json` | 5 | `set/<what>` | **G2** overlapping pairs of 8–32 AABBs |
| `sat2d.json` | 22 | `<cuboid>_<other>/<regime>` | **G2** separating-axis helpers, both directions |
| `clip2d.json` | 16 | `clip/<what>` | **G2** segment-against-segment clipping |
| `point_projection.json` | 33 | `<shape>/<what>` | **G2** point projection on ball, cuboid, capsule, segment |
| `segment_segment.json` | 24 | `seg/<what>` | **G2** closest points between two segments |
| `ray_casts.json` | 64 | `<shape>/<regime>` | **QP** world-space ray casts on the five shapes, solid and hollow, see [ray_casts](#ray_casts) |

### contact_manifolds

Pairs (shape 1 first): `ball_ball`, `ball_cuboid`, `ball_capsule`, `cuboid_cuboid`,
`cuboid_capsule`, `capsule_capsule`, `halfspace_ball`, `halfspace_cuboid`, `segment_ball`,
`halfspace_capsule`, `halfspace_segment`, `cuboid_segment`. Regimes: `separated` (beyond
prediction), `within_pred` (gap 0.01 < prediction), `touching` (distance exactly 0), `shallow`,
`deep`, `degenerate`. That is 12 × 6 = 72 canonical cases, plus

- 7 extra degenerate cases named `degen_<what>` (regime `degenerate`): ball centre exactly on a
  face / plane / segment, corner against corner, exact quarter turn, crossing capsule segments,
  zero-length capsule;
- 8 flipped-order cases (`cuboid_ball`, `capsule_ball`, `ball_halfspace`, `ball_segment`,
  `capsule_cuboid`, `capsule_halfspace`, `segment_halfspace`, `segment_cuboid`, regime `shallow`)
  because upstream swaps the arguments and the outputs.

Upstream routes `halfspace_capsule` and `halfspace_segment` through the halfspace-PFM generator.
It routes `cuboid_segment` through the generic PFM-PFM generator (GJK/EPA plus feature clipping);
the Cairo port uses SAT plus clipping for that pair, so normals and points must agree but exact
tie-breaking may differ. The `ambiguous` tag marks those tied outputs.

Each case records both shapes, `pos12` (pose of shape 2 in the frame of shape 1) and, per
manifold, `local_n1`, `local_n2`, the number of points and for each point `local_p1`, `local_p2`,
`dist`, `fid1`, `fid2` (packed value + decoded kind and code).

`"ambiguous": true` (20 cases) marks a case whose discrete outputs — point count, feature ids,
even the sign of the normal — hinge on an exact tie or on a fallback branch upstream. A port may
legitimately answer differently there; compare `dist` and treat the rest as informative.

### scenes

JL adds four joint-only scenes (sleeping disabled): `pendulum_limited` falls onto and remains
at its −0.5 rad stop; `wheel_motor` drives a pinned free wheel to 4 rad/s with a force cap;
`slider_limited` falls along a vertical prismatic axis onto its 0.5 upper stop; `servo` uses a
ForceBased position motor to converge to 0.75 rad. Motor targets, stiffness/damping, force caps,
model, prismatic axis and limit settings are exported from the same quantized inputs used upstream.
The new `JOINT_CASES` / `joint_cases()` table contains `JointSceneCase { scene, joint }` wrappers;
the original `ALL` table and every original fixture remain byte-identical. Replays use the same
`4096 * step` ulp pose tolerance and twice that velocity tolerance as `pendulum`.

The Cairo joint solver uses `fixed::trig` sin/cos/atan2 for the range-centered revolute angle;
Q32.32 rounding can move an exact limit boundary by a few ulps. Center and half-range are formed
in i128 and halved toward zero, so `[MIN, MAX]` disables angular limits without overflow. A range
of at least one full turn is disabled, as upstream. Limits inherit the existing D4 cutoff that
zeros CFM below eight ulps; motors retain their computed CFM. Motor coefficients and force caps use
the substep duration; motor position error remains in the relaxed pass. Fixed extrema stand in
for floating unbounded impulses, and `MAX` force saturates at `MAX` impulse when dt exceeds one.
Uncoupled free axes are supported; coupled limits/motors remain deferred. Other unrepresentable
fixed-point intermediates panic under the existing numeric policy.

JL one-joint whole-world step costs, subtracting each matching `gas_setup_*` probe from
`gas_step_*` (opaque scene inputs, metered one-iteration call, four substeps per quantized dt=1/60 frame).
Active limits start from the upstream step-30 pose with cold impulses; other probes start at
step zero. These include world bookkeeping, and are not isolated row-solve costs.

| Configuration | Sierra gas | Exact Cairo steps |
|---|---:|---:|
| Plain revolute | 4,336,896 | 38,296 |
| Inactive revolute limit | 7,068,716 | 61,645 |
| Active revolute limit | 7,068,206 | 61,658 |
| Velocity motor (AccelerationBased) | 6,887,156 | 56,515 |
| Position motor (ForceBased) | 6,887,156 | 58,283 |
| Active prismatic limit | 6,987,446 | 57,972 |

Reproduce with `scripts/build-shims/snforge test -p rapier2d golden_scenes::joint_controls::gas_`;
add `--tracked-resource cairo-steps --detailed-resources` for exact steps. The module also supplies
`gas_baseline` (14,120 Sierra gas / 63 Cairo steps); paired subtraction cancels this overhead.

`ball_drop`, `ball_bounce` (restitution 0.7), `box_slope_stick` (μ = 0.7 > tan 30°),
`box_slope_slide` (μ = 0.25), `box_stack3`, `pendulum` (revolute joint), and two scenes with
sleeping **on** (work package SL): `box_stack3_sleep` (the stack of `box_stack3`, which settles
and falls asleep as one island) and `ball_drop_sleep` (the ball of `ball_drop` plus a second ball
released at `y = 11.5`, which lands on the sleeping first ball and wakes it up). 120 steps at the
quantised `dt`, gravity `(0, -9.81)` snapped to Q32.32. The file describes each scene completely
(bodies in insertion order, colliders, material, joints, `can_sleep`) and samples every dynamic
body at step 0 (initial state), steps 1–10, then every 10th step: `translation`, `rotation`
`(re, im)`, `linvel`, `angvel`, read after `PhysicsPipeline::step`; for the two sleep scenes also
`sleeping` (`RigidBody::is_sleeping`), and the scene lists every flip of that flag in
`sleep_transitions` (`step`, `body`, `sleeping`), the step being the one whose `step()` produced
it. The Cairo fixture (`SceneCase`) does not carry the sleep flags: the generator's scene mapping
is field by field, so `crates/rapier2d/tests/golden_scenes.cairo` transcribes the transitions from
the JSON by hand. Upstream decides an island's sleep before solving the step (the bodies keep the
pose of the previous step, velocities zeroed) and wakes a sleeping body in the narrow phase of the
step where a contact starts touching it; the port does both at the same steps
(`rapier2d::pipeline::islands`).

### Sleeping-ball impact diagnosis (SI)

`ball_drop_sleep.impact_diagnostics` records steps 65, 66 and 80–120: both balls' vertical
states and activation timers, every retained manifold, solver contact IDs (including NEW), and
post-solve impulses. `prewake_steps` is a separate Rust counterfactual: the lower ball receives
`RigidBody::wake_up(true)` immediately before step 87. All original scene samples and settings
are unchanged. `generated::sleep_impact` exports selected diagnostic steps and five prewake
checkpoints; there are no hand-transcribed reference numbers in the Cairo diagnostics.

**Cause: one frame without the ground constraint upstream.** Both engines sleep the lower ball
at step 66, zero its velocity, and wake it at step 87. Before that impact the incoming ball has
velocity −14.061 m/s; the first ball–ball contact is already penetrating (distance
−456,446,172 raw, about −0.10627 m), with NEW set. The normal and sphere witnesses agree; the
uninterrupted Cairo distance differs by only 270 ulps. Every restitution coefficient and hence
every restitution seed is zero. The subsequent upward motion comes from penetration correction.

At step 87 upstream leaves the ground impulse at its dormant value, 551,527,914 raw; Cairo
applies 54,919,242,958 raw. Upstream ends that frame with **both** balls moving downward at
−7.194 m/s and the lower centre at 0.375396 m. Cairo keeps the lower velocity at zero and its
centre at 0.498255 m. Upstream restores ground support at step 88, whose large penetration
correction launches the upper ball. The retained ground manifold's `active` flag describes
contact geometry, not membership in that frame's solver selection.

This agrees with the read-only upstream clone's selection mechanism:
`narrow_phase/contacts.rs::compute_contacts` freezes the awake mask/candidate list before
`apply_pair_transitions` wakes the sleeping side; sleeping clears pair solver-hint counts
(`solver_graph.rs::clear_asleep_pair_solver_hint_counts_of`), and `reconcile_pair` explicitly
honours those hints even when they lag the live awake state. These source details are supporting
explanation from the clone; the measured behaviour comes from the pinned published 0.35.3.
Internal upstream substep velocities/selection are not exposed by the public API. In Cairo,
`pipeline::step` explicitly merges revived dormant pairs into the solver input before solving.
Both balls are awake before all four substeps; activation is not changed inside the solver.

The Cairo counterfactual removes **only** the dormant ground pair from the step-87 solver input,
then merges it back unchanged. Starting from the generated step-80 body state and ground cache:

| Intervention | Max position error, samples 90/100/110/120 | Max velocity error | Samples outside tolerance |
|---|---:|---:|---:|
| Immediate ground support (engine) | 1,591,205,862 ulps (0.37048 m) | 8,061,226,037 ulps | 4 |
| Ground deferred for step 87 only | 50 ulps | 81 ulps | 0 |

The reverse Rust experiment (wake before collision detection) matches the uninterrupted Cairo
trajectory within the existing scene tolerance at steps 87, 88, 90, 110 and 120. Weak-waking the
already-awake toucher preserves its already-zero timer; restoring the sleeping velocity to
upstream's zero and clearing NEW (zero restitution seed and zero warm-start impulse) are exact
solver-state no-ops. Tests also replace the incident distance and the dormant ground cache
independently: reference geometry leaves 1,591,205,863 ulps maximum position error (one ulp
worse than the seeded control); restoring the ground cache in the uninterrupted replay reduces
its 1,591,207,411-ulp maximum by only 1,550 ulps. Delaying ground support in that uninterrupted
replay instead passes every later sample (max 53,057 position / 52,326 velocity ulps). Cairo's
ground cache first differs at the sleep step 66 (warm-start 122,738,298
versus upstream 137,881,979 raw), even though both bodies' poses/velocities remain in tolerance;
that small cache difference is distinct from wake-step solver membership.

Upstream updates eligibility timers at the start of a step; Cairo updates them after motion.
At wake step 87 upstream reports 71,582,788 raw for the lower ball, while Cairo ends with zero
because the impact displacement exceeds the sleep threshold. Comparing these post-step timers
without accounting for phase would misidentify the cause.

Recommendation: preserve immediate ground support as a documented divergence instead of
reproducing the upstream one-frame loss of support. `SamplesUntil(80)` remains appropriate for
the original trace; SI adds strict recovery and reverse-control assertions. An exact-parity
change would remove the revived-pair merge in `pipeline::step` and retain those pairs dormant
until after the solve, with matching staged-pipeline semantics and regression tests. This
requires an orchestrator decision and an ADR; SI leaves engine files untouched.

Reproduce with `scripts/build-shims/snforge test -p rapier2d sleep_diagnostics` and
`cargo run --release --locked` in `tools/golden`. This is a diagnosis package, with no production
performance candidate or new gas API; test costs are recorded in `golden_scenes.snap`.

### Slope divergence diagnosis (SD)

The two slope scenes add `contact_diagnostics.steps` (steps 1–10; `box_stack3` too, see SO).
These records come from the pinned **published** Rapier 0.35.3 API after `step`: all manifold points,
normals, raw f64 feature IDs, accumulated and warm-start impulses, solver contacts,
`ContactData::solver_dp1/2`, and the dynamic body's state. Geometry belongs to the
pre-solve collision pass; impulses and body state belong to the completed step.
`SolverContact::anchor*` are CoM-local upstream (world for the fixed side), whereas
`solver_dp*` are the world lever arms consumed by constraint generation. Compare the
port's `SolverContact::anchor*` with **`solver_dp*`**, not with upstream's localized anchors.
Substep velocities are private upstream and are deliberately not presented as observations.
Existing scene samples and tolerances are unchanged.

The measurements of this section were taken against the **pre-GS references** (published
`parry2d-f64`, duplicate feature ids); see "GS update" below for the current state.

**GS update (corrected feature ids, after DM).** DM moved the common midpoint into the engine and
GS regenerated the traces with correct ids; the duplicate-id emulation is gone from the tests.
`box_slope_stick` now passes every sample (max 221 / 121 ulp on translation, 10 on rotation,
2014 / 1948 / 6014 on velocities). `box_slope_slide` passes steps 1–3 and 9–120 but samples 4–8
exceed the tolerance (step 4: 12 539 / 26 184 translation, 39 582 / 68 561 rotation,
147 107 / 514 352 / 452 015 velocity ulps), then reconverge (step 120: 2479 / 1431). Cause: in
step 4, substep 0 solves the speculative second point (gap 52 raw) so that it closes exactly;
at substep 1 its refreshed gap is exactly `0` raw. Both engines solve a row softly
(`cfm_factor`) when `dist <= 0` and rigidly otherwise; upstream's f64 gap at that point is a
rounding residue whose sign decided "rigid" there. Solving that single row rigidly in a
public-API trace (`test_slide_zero_gap_counterfactual`, one row in 120 steps) passes every
sample: step 4 falls to 4 / 0 / 1 / 0 / 25 / 18 / 59 ulps, step 120 to 2382 / 1376 translation
ulps. This is a tie on a discontinuity, not a porting defect; the replay stays ignored with its
tolerance unchanged.

Reproduce with `snforge test -p rapier2d slope_diagnostics`: the engine mode traces the pair with
the public constraint API and asserts the pipeline solve reproduces it bit for bit; the zero-gap
counterfactual writes the traced state back. There are no production engine edits.

**First response, step 3.** Steps 1–2 already have two speculative manifold points;
step 3 is the first step with nonzero impulse. Both materials have the same initial
geometry and coefficients. In the following table numbers are raw Q32.32 and signed
deltas mean **port minus upstream**. SI deltas are raw deltas divided by `2^32`;
feature IDs and indices are discrete and have no SI interpretation.

| Quantity | Upstream | Port | Delta (ulp; SI) |
|---|---|---|---|
| `local_n1`, `local_n2` | `(0,4294967296)`, `(0,-4294967296)` | same | 0 |
| world normal | `(-2147483648,3719550787)` | same | 0 |
| first `local_p1.x` | 2134316890 | 2134316900 | +10; +2.33e-9 m |
| second `local_p1.x` | -2160650407 | -2160650400 | +7; +1.63e-9 m |
| both `local_p1.y` | 2147483648 | same | 0 |
| `local_p2` in point order | `(2147483648,-2147483648)`, `(-2147483648,-2147483648)` | same | 0 |
| both distances | 20144178 | 20144174 | -4; -9.31e-10 m |
| `(fid1,fid2)`, point 0 | `(3221225520,1073741824)` | `(3221225524,1073741826)` | known f64 sign-bit bug |
| `(fid1,fid2)`, point 1 | `(3221225520,1073741824)` | `(3221225524,1073741827)` | known f64 sign-bit bug |
| solver point count / IDs | 2 / `2147483648,2147483649` | same | no prediction skip; both NEW |
| first world arm on slope | `(769594778,2935656523)` | `(774630831,2926933843)` | `(+5036053,-8722680)`; `(+0.00117255,-0.00203091)` m |
| first world arm on box | `(2938553262,-794756254)` | `(2933517217,-786033570)` | `(-5036045,+8722684)`; `(-0.00117255,+0.00203091)` m |
| second world arm on slope | `(-2949956009,788172875)` | `(-2944919960,779450193)` | `(+5036049,-8722682)`; `(+0.00117255,-0.00203091)` m |
| second world arm on box | `(-780997525,-2942239902)` | `(-786033570,-2933517218)` | `(-5036045,+8722684)`; `(-0.00117255,+0.00203091)` m |
| first box normal cross coefficient | 2147483648 | 2147483647 | -1; -2.33e-10 m |
| first normal inverse effective mass | 1717986918 | 1717986918 | 0 |
| box tangent cross coefficient, both points | -2157555737 | -2147483649 | +10072088; +0.00234509 m |
| tangent inverse effective mass, both points | 1708349407 | 1717986916 | +9637509; +0.00224391 kg^-1 |
| substep inverse dt | 1030792154880 | same | 0 |
| static `erp_inv_dt` | 75062807161 | 75062807160 | -1; -2.33e-10 s^-1 |
| static `erp` (`substep_dt * erp_inv_dt`) | 312761695 | same | 0 |
| static `cfm_factor` | 4171843512 | same | 0 |
| friction: stick / slide | 3006477107 / 1073741824 | same | 0; average combine |
| restitution | 0 | 0 | 0 |

Upstream row coefficients above are calculated from its recorded `solver_dp2`, world
normal, inverse mass 1 and inverse inertia 6, using its scalar row formulas. ERP/CFM
come from `integration_parameters.json` (`dt_q32.static_contact`). Initial speculative
normal RHS is `max(dist,0)/substep_dt`; penetration bias is zero. Both implementations
solve normal points 0,1 then tangents 0,1, with friction in the relaxation pass only.
The block solver is disabled and a single pair has no graph-order ambiguity.

The first substantive defect is in the port's constraint builder: it uses the two
separated surface points as lever arms and material anchors. Upstream freezes **one
shared midpoint** for both. Its tangent arm length is `0.5 + dist/2`, rather than 0.5;
this changes both friction effective mass and angular response. Upstream's
`pair_update.rs` localization pass computes that midpoint, stores `solver_dp1/2`, and
`ContactWithCoulombFrictionBuilder::generate` uses them for both rows and substep anchors.

| Step-3 velocity | Upstream raw | Port raw | Signed delta (ulp; SI) |
|---|---|---|---|
| stick vx | -180922494 | -178467458 | +2455036; +0.000571608 m/s |
| stick vy | 34900322 | 36476938 | +1576616; +0.000367085 m/s |
| stick angular | 277166668 | 272637761 | -4528907; -0.00105447 rad/s |
| slide vx | -502431356 | -501940666 | +490690; +0.000114248 m/s |
| slide vy | -350302641 | -352018006 | -1715365; -0.000399390 m/s |
| slide angular | -624046678 | -626383038 | -2336360; -0.000543976 rad/s |

| Step-3 pose | Upstream raw | Port raw | Signed delta (ulp; SI) |
|---|---|---|---|
| stick x / y | -2169527138 / 3708194310 | -2169511332 / 3708204096 | +15806 / +9786; +3.68e-6 / +2.28e-6 m |
| stick re / im | 3718363471 / 2149538828 | 3718362901 / 2149539815 | -570 / +987; -1.33e-7 / +2.30e-7 |
| slide x / y | -2170201782 / 3706973093 | -2170198175 / 3706966844 | +3607 / -6249; +8.40e-7 / -1.45e-6 m |
| slide re / im | 3719084234 / 2148291539 | 3719091454 / 2148279040 | +7220 / -12499; +1.68e-6 / -2.91e-6 |

Replacing only the anchors with the common midpoint reduces step-3 absolute errors:

| Scene | x/y position ulps | re/im rotation ulps | vx/vy/angular velocity ulps |
|---|---|---|---|
| stick | 4 / 0 | 1 / 2 | 673 / 599 / 559 |
| slide | 2 / 3 | 0 / 0 | 298 / 1017 / 1640 |

**Step 4 and subsequent drift.** Midpoint alone does not recover step 4 because upstream
regenerates the manifold with **duplicate f64 IDs**. Parry 0.30.2 `match_contacts` keeps
iterating after a match; both new points receive the *last* old point's data. For stick,
that last point has warm normal impulse 714093889 and tangent -462965448 at step 3,
while the first point's warm impulses are zero. The port correctly keeps the distinct
point data. Reproducing upstream's erroneous last-match copy *only in the diagnostic*
recovers every sample through step 10: maximum position / rotation / velocity errors
are 30 / 28 / 3000 ulps for stick and 26 / 5 / 1640 for slide. This establishes a
second, independent root cause; increasing a rounding tolerance cannot repair it.

**Substeps and the slide position/velocity oddity.** These are measured **port**
step-3 velocities, as `(vx,vy,angular)` raw; divide by `4294967296` for m/s and rad/s.
Substeps are numbered 0–3. “Integrated” is the velocity just before pose integration;
“relaxed” is the velocity after the subsequent no-bias solve.

| Scene | Substep | Integrated | Relaxed |
|---|---|---|---|
| both | 0 | `(0,-1580011092,0)` | same |
| both | 1 | `(0,-1755567880,0)` | same |
| stick | 2 | `(0,-1931124668,0)` | `(-222884590,-638332879,1024380310)` |
| stick | 3 | `(-132683402,-63378354,569913031)` | `(-178467458,36476938,272637761)` |
| slide | 2 | `(0,-1931124668,0)` | `(-348989990,-711139865,587538390)` |
| slide | 3 | `(-297525818,-360318806,220439496)` | `(-501940666,-352018006,-626383038)` |

At slide step 120 the port's integrated velocities are:

| Substep | vx raw | vy raw | angular raw |
|---|---|---|---|
| 0 | -20559380031 | -11869963559 | 41 |
| 1 | -20602481457 | -11894848181 | 36 |
| 2 | -20645582882 | -11919732801 | 37 |
| 3 | -20688684317 | -11944617407 | 39 |

The final relaxed velocity is `(-20688684298,-11944617442,9)`.
The linear components differ from upstream by only 1921 / 1828 ulps, but angular
velocity differs by 158982 ulps (3.70e-5 rad/s); the earlier “velocities agree to
2e3 ulps” observation applies to **linear** velocity. Position errors are 4450151 /
2569242 ulps at step 120 (0.001036 / 0.000598 m).

Both engines integrate each substep's *biased* velocity and then relax velocity.
Upstream `RigidBodyVelocity::integrate_linearized` normalizes the first-order complex
rotation and adds `linvel * substep_dt` to translation. The port does the same;
`next_position` is the accumulated solver pose (minus rotated local CoM, zero here),
not a reintegration using the final relaxed velocity. The duplicate-ID warm start
changes transient substep velocities even when their final linear values converge.
Consequently their integrals differ each step. This is a solver-input difference,
not an independent defect in `integrate_linearized` or `next_position`.

The midpoint-plus-ID counterfactual also passes **all 22 sampled states through 120
steps** for both scenes. At slide step 120 it differs from upstream by only 2500 /
1443 position ulps, 4 / 6 rotation ulps, and 2281 / 1314 / 2 velocity ulps. Its
integrated substep velocities, measured in the test (not read from upstream), are:

| Substep | vx raw | vy raw | angular raw |
|---|---|---|---|
| 0 | -20550629413 | -11866170347 | -1508757 |
| 1 | -20603635768 | -11892920071 | 3829244 |
| 2 | -20644928878 | -11920714278 | -2365515 |
| 3 | -20688740093 | -11944626077 | 45545 |

The current port moves `(-343733870,-198454844)` raw during step 120; the
counterfactual moves `(-343699727,-198435130)`. The difference is
`(-34143,-19714)` ulps (`-7.95e-6,-4.59e-6` m) **in one step**, while their final
linear velocities differ by just `(360,-514)` ulps. This directly reproduces the
reported drift mechanism without inventing access to upstream's private substeps.

**Required follow-up.** In `crates/rapier_dynamics2d/src/solver/contact.cairo`,
`generate_element` (lines 352, 368, 379–386 at the SD base), reconstruct the two world
points from `original*.position.translation + sc.anchor*`, compute their midpoint,
and use midpoint-minus-original-CoM for **both** calls to `coefficients`. Use the same
midpoint for `local_p1/2`, and keep `sc.dist` as the base separation. Leave the frozen
DD/F3 `SolverContact` interface unchanged. This belongs with the owning package's
unit tests and affected gas snapshots. Separately obtain corrected-ID upstream scene
references (or a controlled f32 engine cross-check); do not port the f64 ID bug.
The two original 120-step replays remain ignored pending these follow-ups, with their
original tolerances intact. No wider tolerance is claimed or justified by this diagnosis.

### Stack divergence diagnosis (SO)

`box_stack3` also carries `contact_diagnostics.steps`: after steps 1–8 and after step 60, every
contact pair in upstream's contact-graph edge order (`collider1`, `collider2`, `active`) with
the same per-manifold fields as the slope records. Upstream's **solve order** is not observable
through the public API (`ContactPair::solver_color` is `pub(crate)`); it is derived from the
0.35.3 sources and confirmed by the counterfactual below.

**Upstream's order.** Rapier 0.35.3 has no per-step island ordering: the narrow phase keeps a
persistent solver colour per pair (`narrow_phase/mod.rs`, `assign_pair_solver_color`), assigned
when the pair starts touching, greedily in ascending `(min, max)` body index. Pairs of two
non-fixed bodies take the lowest free colour in `0..120`, pairs with a fixed body the highest
free colour below 128 (the constant's comment: "fixed geometry the final say each sweep").
`staged_island_solver` solves the colour buckets in ascending colour; small colours all land in
worker 0's serial overflow, still in ascending colour. In the stack all three pairs touch from
step 1 on (gap 0.01 < prediction 0.02), so the colours never change: `(box0, box1)` 0,
`(box1, box2)` 1, `(ground, box0)` 127. Upstream solves pairs `[1, 2, 0]` of the port's pair
order; the port (D8) solves `[0, 1, 2]`, ground first. The edge order `(2,3), (1,2), (0,1)` is
neither.

**First divergence: step 4, solver impulses.** Steps 1–3 match (max 85 ulp): only the ground
pair carries impulse. At step 4 box1 lands on box0, the first step with two loaded manifolds on
one body. Quantity by quantity at step 4 (raw; port minus upstream):

| Quantity | Result |
|---|---|
| (a) pair set | same 3 pairs, all active from step 1; order differs as above |
| (b) manifolds | same normals, point order and feature ids; `dist` within 8 ulp (`(0,1)`: 629 543 / −8 both) |
| (c) solver contacts | same count and NEW/matched ids (`(0,1)`: 0, 1; `(1,2)`: NEW 0, NEW 1) |
| (d) solve order | **differs**: ground pair impulse 1 622 403 951 vs 1 682 864 324 (−60.5M), warm-start 1 096 872 324 vs 788 065 815 |
| (e) exact-zero gaps | none needed: the order alone recovers every sample |

**Counterfactuals** (`crates/rapier2d/tests/golden_scenes/stack_diagnostics.cairo`; the step is
replayed with the public stage functions and only the manifold order changes; pair order
reproduces `World::step` bit for bit):

| Order / window | Violations | Max ulps tx / ty / im / vx / vy / w |
|---|---|---|
| pair (port), 0–60 | 12 | 7 542 318 / 4 357 583 / 4 711 768 / 159 391 583 / 218 119 261 / 153 858 956 |
| pair, steps 1–10 | 7 | |
| reversed (edge order), steps 1–10 | 6 | |
| **colour (upstream), 0–60** | **0** | 404 / 12 / 43 / 631 / 909 / 361 |
| colour, step 4 | 0 | ground impulse 1 682 865 100 (+776), warm-start 788 065 591 (−224) |
| colour, 60–120, cold re-seed | 2 | 313 801 / 879 137 / 251 280 / 10 409 380 / 4 101 317 / 4 786 826 |
| **colour, 60–120, re-seed + upstream impulses** | **0** | 3 095 / 404 / 428 / 21 102 / 1 298 / 10 043 |
| pair, 60–120, re-seed + upstream impulses | 0 | 11 082 / 3 819 / 3 777 / 47 561 / 58 353 / 63 673 |

The second window's residual failures are a re-seed artefact (the port starts with an empty
contact cache while upstream warm-starts); with upstream's step-60 impulses written on the
matching feature ids, both orders pass, the colour order 2–6× closer. The stack at rest is far
less order-sensitive than the landing.

## Leaf-level families (G2)

Work package **G2**. The families above only validate end results (a manifold, a body trace). The
six below validate the *internal* functions the port implements in waves 2–3, so that a wrong
`inv_mul`, a wrong SAT tie-break or a wrong clipping feature is caught where it is written, not
three layers up. Same quantisation rule, same fixture pattern (`const` per case + `ALL` +
`cases()`); sanity tests in `crates/rapier_golden/tests/<family>.cairo`.

Public entry points: everything below is called through the published API of
`parry2d-f64 0.30.2` re-exported by `rapier2d-f64`. Two things differ from the 0.31 clone the
algorithms were read from: `query::clip` and `query::closest_points` are **private modules** in
0.30.2, so `clip_segment_segment*` and `closest_points_segment_segment*` are reached through
`query::details`; `query::sat::*` is public. Nothing had to be replaced by a "nearest public API".
The 3D-only SAT helpers (`*_edge_twoway`, `cuboid_cuboid_compute_separation_wrt_local_line`) do not
exist in 2D and are not used.

Case lists are deliberately short (≈ 3 200 generated Cairo lines in total): each case is there
for a reason written in its `note` field in the JSON.

### pose2

Upstream: `Pose * Pose`, `Pose::inverse`, `Pose::inv_mul` (Parry's `pos12`),
`Pose::{transform_point, inverse_transform_point, transform_vector, inverse_transform_vector}`,
`Rotation * Rotation`, `Rotation::inverse`, on `parry2d_f64::math::{Pose, Rotation}` (glam's
`Pose2` / `Rot2`). 7 pairs `a`, `b` (identity first, exact quarter / half turns, generic angles,
a far pair with a 1° rotation and a 100–250 translation, `a == b`, `b = a⁻¹` snapped) and 3
points `(1, 0)`, `(0.5, -2)`, `(-3, 4.25)` on which the point / vector operations are recorded.

- **Upstream never renormalises.** `Pose::from_parts` keeps the `(re, im)` pair as passed
  (`a_rotation_used` in the JSON equals the input for every case), and `Rotation * Rotation` is
  the plain complex product (`rot_mul_is_plain_complex_product` is `true` everywhere).
  `Rotation::inverse` is the conjugate, so `R · R⁻¹ = |R|² = 1 ± 2^-31`, not 1: the port must not
  "fix" that either. The norm error of a rotation multiplies every coordinate it is applied to.
- `chain/deg0p1`, `chain/deg1`, `chain/deg5`: `acc = acc · step` 1 000 times from the identity, in
  `f64`, sampled after 1, 10, 100 and 1 000 products (`norm_squared` = `re² + im²`, `drift` =
  `norm_squared − 1`). The drift is **the input-norm term alone**: `|step|² − 1` is
  `−1.83e-10`, `+1.51e-10`, `+1.24e-10` and grows linearly (drift after 1 000 products =
  1 000 × drift after 1, to 1e-13; that is −786, +649, +535 raw). `f64` rounding is invisible. A
  Q32.32 chain adds the bias of its truncating products on top: report the two contributions
  separately in the M2 study, and compare the *growth* (linear in the number of steps) rather than
  the value.

Tolerances: rotation products / inverse 2 ulp (two products and a sum; the conjugate is exact);
translations and transformed points `4 + 2·(|p|₁ + |t|₁)` ulp with `|·|₁` in whole units (the
norm error above scales with the magnitude: the far pair sits at ≈ 350, i.e. ≈ 700 ulp).

### aabb_overlap

Upstream: `BoundingVolume::intersects` and `BoundingVolume::merged` on `parry::bounding_volume::Aabb`.
Five sets, each an ordered list of AABBs with an `is_static` flag: `grid_touching` (4 × 3 unit
boxes sharing edges and corners), `nested` (nested, identical, edge- and corner-touching),
`ulp_boundary` (gaps and overlaps of exactly one raw unit along x, y and diagonally), `mixed_20`
and `random_32` (boxes on a 1/16 grid from a fixed LCG, some forced to touch or nest). The
recorded `pairs` are every `i < j` that intersects, sorted by `i` then `j`.

- **Convention: closed.** `intersects` is `mins ≤ other.maxs ∧ maxs ≥ other.mins` on every axis,
  so AABBs that merely touch (along an edge, or at a corner) overlap, and so does an AABB with
  itself. The `ulp_boundary` set pins the decision at the last raw unit.
- `intersects` knows nothing about static bodies. `is_static` is carried so that the port can
  test the Rapier rule (no pair of two fixed colliders): `both_static` marks those pairs, they are
  still listed. Rapier's broad phase may inflate the AABBs (prediction distance) before this test;
  that inflation is not part of this family.

Tolerance: **none, exact.** The inputs are exact Q32.32 numbers and the test is a comparison, so
the pair list is reproduced bit for bit (the Cairo sanity test recomputes it with integer
comparisons). `merged` is exact too.

### sat2d

Upstream (`query::sat`): `cuboid_cuboid_find_local_separating_normal_oneway`,
`cuboid_support_map_find_local_separating_normal_oneway` (cuboid vs segment and vs triangle: the
four face normals of the cuboid), `segment_cuboid_find_local_separating_normal_oneway` and
`triangle_cuboid_find_local_separating_normal_oneway` (the one normal of the segment, the three
edge normals of the triangle). Each case calls the helper in both directions: `sep1` tests the
normals of shape 1 (cuboid) against shape 2 with `pos12`, `sep2` those of shape 2 against shape 1
with `pos21`. 22 cases: cuboid–cuboid 9, cuboid–segment 6, cuboid–triangle 7, over the six regimes
of `contact_manifolds` (`separated`, `within_pred`, `touching`, `shallow`, `deep`, `degenerate`),
the cuboid–cuboid ones with the same poses as `contact_manifolds/cuboid_cuboid/*`, plus
`degen_corner`, `degen_rot90`, `sep_diagonal` and the triangle `degen_edge`.

- `pos21` is the inverse of `pos12` **snapped to Q32.32** (the rotation conjugate is exact, the
  translation is rounded) and is what the second call was given, so both calls have exact inputs;
  `pos12` and `pos21` are mutually consistent to 1–2 ulp only.
- The axis of `sep1` is expressed in the frame of shape 1, the axis of `sep2` in the frame of
  shape 2, both pointing from the tested shape towards the other one.
- **Tie-breaking.** All helpers keep the first axis on an exact tie (strict `>`): the cuboid
  helpers scan `-x, +x, -y, +y` (support-map version) or `x, y` (cuboid–cuboid version, oriented
  by `copysign(1, translation_i)`, so an exact `+0` orients towards `+`). Cases where this decides
  the answer carry `"ambiguous": true` (`cuboid_cuboid/{degenerate, degen_corner}`,
  `cuboid_segment/degenerate`, `cuboid_triangle/{touching, degenerate, degen_edge}`): compare
  `separation`, treat `axis` as informative.
- **Weighted diagonal.** `cuboid_cuboid…oneway` does not return the axis with the largest
  separation when at least two face axes have `separation ≥ 0`: it returns the direction of
  `Σ sign_i · max(separation_i, ε) · e_i` and the separation measured along it. For two boxes
  apart along x and y that is the corner-to-corner distance, **larger** than any face gap
  (`sep_diagonal`: 1.2806 vs 1.0), and for exact corner contact it is the 45° axis with
  separation 0 (`degen_corner`), not `+x`. The manifold generators rely on it.
- A zero-length segment has no normal: `segment_cuboid…oneway` answers `(-f64::MAX, 0)`. It cannot
  be quantised, so it is recorded (`non_finite_probes`) but not exported.

Tolerances: separation 1 ulp for axis-aligned face axes (a difference of inputs), 4 ulp with
rotations (the support point goes through one pose product); axis components 4 ulp for face axes,
8 ulp for a normalised diagonal or segment / triangle normal (one square root and a division).

### clip2d

Upstream: `query::details::clip_segment_segment(seg1, seg2)` and
`clip_segment_segment_with_normal(seg1, seg2, normal)` (2D), the clipping Parry runs on the
reference and incident edges of a polygonal manifold. Each returns `None` or two clipping points
`(p1, p2, f1, f2)`; `p1` lies on segment 1, `p2` on segment 2, and a feature is **`0` = first
vertex as passed, `1` = interior, `2` = second vertex** (not the `PackedFeatureId` convention, the
manifold code translates it). 16 cases: partial overlap, containment both ways, identical,
reversed, collinear overlap / disjoint, parallel disjoint, two end-to-end single-point cases,
perpendicular crossing, oblique, zero-length segment 2 in the interior, a one-raw-unit sliver,
vertical segments with an `x` normal, and a slanted segment 2.

- `plain` projects on the direction of segment 1 and returns the points sorted along it;
  `with_normal` projects on the tangent `(-normal.y, normal.x)` (a `+y` normal gives a `-x`
  tangent), so its two points come out in the **opposite order** for horizontal segments.
- **Single point.** Touching end to end yields two clipping points at the same position with
  *different* features (`(1, 0)` then `(2, 1)`), not one point.
- **Ties** (`range2[0] == range1[0]`, e.g. `identical`): the strict `>` picks the vertex of
  segment 1 and the *interior* feature of segment 2 (`(0, 1)`), although the point coincides with
  a vertex of segment 2. Features are exact except at such ties.
- **Division by zero.** `clip_segment_segment` divides by the length of segment 1 or 2: a
  zero-length segment 1, a zero-length segment 2 starting where segment 1 starts, or two points
  give non-finite points (recorded in `non_finite_probes`, not exported). `with_normal` stays
  finite there (its reciprocal is guarded). A zero-length segment 2 in the interior (case
  `seg2_point_inside`) is fine.

Tolerance: clip points `4 + 2·length` ulp (`a + (b − a)·t`, `t` a ratio of two projections: one
division and one product, so the error scales with the segment length); features exact except at
the ties above.

### point_projection

Upstream: `PointQuery::{project_local_point(pt, solid), project_local_point_and_get_feature,
distance_to_local_point}` on `Ball`, `Cuboid`, `Capsule`, `Segment`, and
`PointQueryWithLocation::project_local_point_and_get_location` on `Segment`. All in the local
frame of the shape. 33 cases: inside / outside / on the boundary / on a vertex / on an edge
extension / at the centre for each shape (see `note`). Recorded: the projection for `solid = false`
and `solid = true`, `distance` (`solid = false`: negative inside), the feature and, for segments,
the location (`OnVertex(i)` or `OnEdge(u)` with the point at `a + u (b − a)`).

- **Solid vs non-solid.** `solid = true` projects an inside point to itself; `solid = false`
  pushes it to the boundary (nearest face for the cuboid, the surface for ball and capsule, the
  point itself for a segment, which has no interior). Outside points are identical in both.
- **Boundary counts as inside** (`<=`): a ball point at distance exactly `r`, a cuboid point on a
  face or on a vertex, a capsule point on its side, a segment point on the segment.
- **Features.** Ball and capsule always report `Face(0)`. Cuboid (`Aabb` projection): outside a
  face `Face(i)` with `i = 0, 1` for `+x, +y` and `2, 3` for `-x, -y`; outside a vertex region
  `Vertex(code)` with bit `i` set when the coordinate is below the centre (`+,+` = 0, `-,+` = 1,
  `+,-` = 2, `-,-` = 3); a point on the boundary or inside (zero shift) reports the first `Face`
  it touches, so a point *on a vertex* is `Face(0)`, not a vertex. Segment: `Vertex(0|1)` in a
  vertex region (`ab·ap ≤ 0` and `≥ |ab|²` belong to the vertex), otherwise `Face(0)` when
  `perp_dot(pt − proj, ab) ≥ 0` and `Face(1)` on the other side. These are the codes of the
  point-projection scheme, different from the SAT / support-face scheme of the manifolds.
- **Cuboid ties.** For an inside point the axis with the smaller distance to a face wins and
  `diff.x <= diff.y` picks `x` on a tie (`inside_tie`); an exact zero coordinate has sign `+1`
  (`center` projects to `+y`).
- **Capsule on its core segment** (distance 0, also the end points): the projection uses the
  segment normal `(dir.y, -dir.x)`, or `+y` for a zero-length core. Upstream's "distance ≥
  `f64::EPSILON`" threshold is `2.2e-16`, far below 1 ulp: in Q32.32 read it as "distance ≠ 0".
- **Ball centre.** `pt · (r / sqrt(|pt|²))` is `0 · ∞ = NaN` at the exact centre (recorded in
  `non_finite_probes`, not exported); the port needs a rule. `ball/near_center` is 2^-20 from the
  centre: its squared length `2^-40` is below one raw unit, so a Q32.32 `length_squared` underflows
  to 0 and the projection needs wide arithmetic (or a fallback) there.

Tolerances: cuboid projection exact (≤ 1 ulp: sums and differences of inputs); segment 4 ulp (one
division), `u` 2 ulp; ball 4 ulp away from the centre; capsule 8 ulp (normalisation); `distance`
8 ulp; discrete outputs (`is_inside`, feature, location kind) exact, except that a point within a few
ulp of a region boundary may legitimately flip `OnVertex` / `OnEdge`: compare the points.

### ray_casts

Upstream: `RayCast::{cast_ray, cast_ray_and_get_normal}(pose, ray, max_time_of_impact, solid)`
on `Ball`, `Cuboid`, `Capsule`, `Segment`, `HalfSpace`, parry `0.30.2` (the vendored patched
copy). Each case places the shape at a pose (the identity except for the `*/posed` cases) and
casts a world-space ray with a **non-normalised** `dir`, for `solid = true` and `false`. Both entry
points are recorded (`toi` from `cast_ray`, `hit` = time, world normal, feature from
`cast_ray_and_get_normal`) because they do not always agree. Regimes per shape: hit from
outside, grazing / tangent, from inside (solid and hollow), parallel miss, pointing away,
`max_toi` cut (and, for the ball, a hit exactly at `max_toi`, kept: `<=`), zero direction, and a
posed cast. A `None` upstream result is `has_toi = false` / `hit = false` with zeroed fields.

- **Two cuboid algorithms.** `cast_local_ray` is the slab loop with `tmin = 0`, `tmax = max`:
  a hollow ray whose entry is 0 answers `tmax`, i.e. the exit clipped to `max` (`inside_max_cut`
  answers `max` itself; `zero_dir_inside` hollow answers `max = 100`). `cast_local_ray_and_get_normal`
  goes through `clip_aabb_line`: from inside, the exit only within `max`; from the boundary
  pointing in (`boundary_in`), `t = 0`. Cuboid faces are `Face(0|1)` for `-x|-y` and `Face(3|4)`
  for `+x|+y` (upstream's `+ 3`), `Unknown` for a zero `dir` inside; a solid ray from inside has a
  zero normal and the feature of the exit face; a corner tie has the normal `-dir / |dir|`.
- **Capsule = GJK upstream.** The feature is `Unknown`; a zero `dir` is a miss even inside; a
  solid ray from inside answers `-dir / |dir|`. The normal is GJK's last search direction, up to
  ~300 ulp from the exact one (`hit_cap`, `posed`).
- **Upstream bug, `capsule/inside` hollow.** The hollow support-map cast shifts the origin by a
  *length* along `dir / |dir|`, casts back along `-dir` (times in units of `|dir|`) and returns
  `shift - toi_back`: correct only for a unit `dir`. With `|dir| = 1.118` upstream answers
  `0.18122` where the exit is at `0.15` (the point it reports is outside the capsule). The case
  is kept and the Cairo test checks that upstream's number is exactly that unit mix applied to the
  port's exit; a port should answer the true exit.
- **Half-space.** A solid ray strictly inside answers `t = 0` with a zero normal; a ray parallel
  to the plane divides by zero upstream (`±inf` / `NaN`, rejected by the comparisons).
- The exact centre of a ball as a solid origin gives a `NaN` normal (`non_finite_probes`).

Tolerances: time of impact 4 ulp (every non-capsule case matched to the ulp), normals 8 ulp (max
2 observed); capsule time of impact 16 ulp (0 observed) and normal 1024 ulp (GJK, 285 observed);
hit / miss, `has_toi` and features exact.

### segment_segment

Upstream: `query::details::closest_points_segment_segment_with_locations(pos12, seg1, seg2)` (Ericson's
routine) and `closest_points_segment_segment(pos12, seg1, seg2, margin)` (asserted equal: same
points). `pos12` places segment 2 in the frame of segment 1; `p1` is reported in the frame of
segment 1, `p2` in the frame of segment 2, and `dist_sq` is `|p1 − pos12·p2|²` computed **by the
harness** in `f64` from those points (upstream returns no distance). 24 cases: crossing, oblique
crossing, parallel (both orientations), collinear overlapping / disjoint / touching / identical,
endpoint to interior, endpoint to endpoint, skew, nearly parallel (slope 2^-10), zero-length first /
second / both, three posed cases, and 6 **swapped copies** (`*_swap`) for the symmetry check.

- **Non-unique pairs.** Parallel and collinear-overlapping segments have infinitely many closest
  pairs; upstream picks one by falling back to `s = 0` and clamping `t`: for `parallel` it answers
  the start of segment 2 and its projection on segment 1, for `parallel_reversed` the *same
  geometric point* through the other end point of segment 2 (`OnVertex(1)`). These cases carry
  `"ambiguous": true`: compare `dist_sq`, not points or locations.
- Collinearity is decided by `denom > eps && !ulps_eq(ae, bb)` with `eps = f64::EPSILON`; in Q32.32
  the port needs its own threshold (`nearly_parallel` has `denom = 4·2^-20`, i.e. 16 384 raw, and
  goes through the regular branch).
- A location within an ulp of `0` or `1` may legitimately be `OnVertex` or `OnEdge`.

Tolerances: `dist_sq` 16 ulp (distances ≤ 2); points and `u` 4 ulp (a division scaled by the segment
length); locations exact unless `ambiguous`.

## Settings deviating from Rapier's defaults

Everything D11 asks for could be disabled through the public API or a cargo feature:

| setting | default | here | how | why |
|---|---|---|---|---|
| block solver | on | **off** | cargo feature `block-solver` not enabled | the port solves manifold points sequentially (MVP) |
| contact recycling | on | **off** | `IntegrationParameters::contact_recycling = false` | the port recomputes every manifold every step |
| contact clustering | on | **off** | `IntegrationParameters::contact_clustering = false` | only acts on pairs with several manifolds; never the case for the convex pairs here, disabled for clarity |
| CCD | 1 substep | **off** | `max_ccd_substeps = 0` (skips the CCD branch entirely) and `ccd_enabled(false)`, no soft-CCD | not in the MVP |
| sleeping | on | **off** except `box_stack3_sleep`, `ball_drop_sleep` | `RigidBodyBuilder::can_sleep(false)` on every body of the six original scenes; `can_sleep(true)` (default thresholds) in the two SL scenes | the original traces predate sleeping (SL) and stay unchanged; the two SL scenes validate the sleep and wake-up steps |
| `dt` | `1/60` (f64) | `71582788 / 2^32` | field | quantisation rule |

Left at their defaults, and **not** removable, so they remain sources of legitimate divergence:

- the **solver order**: contacts are solved colour by colour from Rapier's incremental graph
  colouring, joints before contacts; a sequential port that solves in insertion order will not
  reproduce multi-contact scenes exactly, even in floats;
- **warm starting** (`warmstart_coefficient = 1`), 4 solver iterations, 1 PGS + 1 stabilisation
  iteration, friction outside the bias pass, the manifold-level temporal-coherence fast path
  inside Parry (`try_update_contacts`), the BVH broad phase (irrelevant for results as long as
  all overlapping pairs are found);
- linear and angular damping are 0, gravity scale 1, friction / restitution combine rule
  `Average` — defaults, recorded in `scenes.json`.

## Recommended comparison tolerances

1 ulp = `2^-32 ≈ 2.3e-10`. `rapier_golden::compare::{within, vec2_within}` take a tolerance in
ulps. These are starting points, to be tightened once the port exists; when a tolerance must be
raised, write down why.

| family | tolerance | justification |
|---|---|---|
| integration parameters, plain values | 1 ulp | one rounding of an `f64` constant |
| … spring coefficients (`erp_inv_dt`, `erp`, `cfm_*`) | 16 ulp, absolute | a division and up to 4 products on values of magnitude ≤ 240. `cfm_coeff` of joints is `1.46e-9` = **6 ulp**: only an absolute tolerance makes sense, and the port may as well treat it as a constant |
| mass, inertia, centre of mass | 16 ulp | ≤ 4 chained products, each ≤ 1 ulp of truncation, scaled by factors ≤ 2^3 |
| inverse mass / inertia | `4 + 2·inv²` ulp (`inv` in real units) | `δ(1/m) = δm / m²`: a 1-ulp error on a small mass is amplified by `inv²` (the `r = 0.05` ball has `1/I ≈ 1.0e5`) |
| AABB | exact for axis-aligned poses, 4 ulp otherwise | `|R|·half_extents` is 2 products and a sum per axis; exact quarter turns must give exact boxes |
| manifolds, analytic pairs (all but `cuboid_capsule`) | 64 ulp on `dist` and points, 64 ulp per normal component | one `sqrt`, one division, a few products on magnitudes ≤ 4 |
| manifolds, `cuboid_capsule` / `capsule_cuboid` | 2^16 ulp (1.5e-5) | upstream runs GJK/EPA, which stops on its own epsilon; the port plans an analytic generator (report 02 §4.1), so the two agree only up to GJK's convergence threshold |
| manifolds, discrete outputs | exact unless `ambiguous` | point count, feature ids, which point comes first |
| ray casts | 4 ulp time of impact, 8 ulp normal; capsule 16 / 1024 ulp | one correctly rounded quotient per time; the capsule normal is GJK's search direction upstream. Hit / miss and features exact. `capsule/inside` hollow is an upstream bug, see [ray_casts](#ray_casts) |
| scenes `ball_drop`, `ball_bounce`, `pendulum`, `box_slope_*` | `2^12 · step` ulp on positions (≈ 1e-6 per step), twice that on velocities | single-contact or joint-only scenes have no solver-order ambiguity; the error is rounding accumulated over ~10³ operations per step, growing at most linearly while the motion is not chaotic. After the first bounce of `ball_bounce`, compare bounce apex and impact step rather than samples |
| scene `box_stack3` | invariants, not samples | rest heights within `allowed_linear_error` (0.005) of `0.5 + i`, `|x| < 0.01`, final speeds `< 1e-3`; multi-contact ordering differs from upstream by construction (SO: in upstream's colour order the samples pass the scene tolerance) |

## Upstream behaviours worth knowing

Found while building the vectors; all visible in the JSON.

### Feature ids (f64 upstream bug)

`Cuboid::vertex_feature_id` extracts "sign bits" with `to_bits() >> 31` / `>> 30`. That is the sign
bit of an `f32`, but a **mantissa** bit of an `f64` (upstream left a `TODO: is this still correct
with the f64 version?`). In the published `parry2d-f64` every cuboid vertex id therefore collapses
to `0` and every face id to `0b110000`, which defeats contact matching (warm starting) for cuboids
in f64 builds: both regenerated points of a cuboid manifold receive the data of the same old point.

Since work package GS the harness builds a vendored `parry2d-f64 0.30.2` that reads bits `63` /
`62` instead ([`vendor/README.md`](vendor/README.md)), so **every vector, the scene traces
included, comes from an f64 engine with correct feature ids**. The manifold vectors still expose
the ids of an **f32 run of the same case** as `fid1` / `fid2` — the scheme the port implements —
and the f64 build's ids as `fid1_f64_build` / `fid2_f64_build`; with the patch the two agree on
all 216 ids (108 points). When the two builds disagree on the points themselves (only allowed for
`ambiguous` cases) the ids are `null` in JSON and `0` (`PackedFeatureId::UNKNOWN`) in Cairo.

Packed encoding: `0b01 << 30 | code` vertex, `0b11 << 30 | code` face, `0` unknown. Three
different code schemes coexist and are only comparable within one pair type:

- cuboid through SAT / support face: vertex `code = (x < 0) | (y < 0) << 1`, face
  `code = max(v1, v2) << 2 | min(v1, v2) | 0b110000`;
- cuboid through point projection (the `*_ball` pairs): ids of `Aabb::project_local_point…`;
- segment: point projection gives `Vertex(0|1)` and `Face(0|1)` (the side the point is on), while
  capsule–capsule and clipping use `0` / `2` for the end points and `1` for the interior, all tagged
  as faces. Ball, capsule (point projection) and half-space always report `Face(0)`.

### Manifold conventions

- `contact_manifolds` always leaves **one** manifold for a convex pair, possibly with 0 points;
  its normals are then meaningless (zero, or GJK's cached separating direction for PFM pairs).
- `local_n1` points from shape 1 to shape 2 in the frame of shape 1;
  `local_n2 = -R12⁻¹ · local_n1`; `local_p1` / `local_p2` are each in their own shape's frame;
  `dist < 0` means penetration.
- The prediction test is `dist < prediction` for ball–ball but `dist <= prediction` elsewhere.
- Cuboid–cuboid and PFM manifolds keep clipped points **beyond** the prediction distance
  (`cuboid_cuboid/shallow` holds a point at `dist = 0.42`); Rapier filters them later, when it
  builds solver contacts.
- Exactly-on-boundary ball centres (`*/degen_on_*`, `halfspace_ball/degenerate`): the
  zero-length projection makes upstream take the normal from `normalize(pos12.translation)` (or
  `+Y`) and then flip it because the point counts as inside. `halfspace_ball/degenerate` answers
  `local_n1 = (0, -1)`, pointing *into* the half-space. Do not copy this blindly.
- Crossing capsule segments (`capsule_capsule/degenerate`, `degen_cross30`): the closest points
  coincide; upstream normalises the float residue (`degen_cross30` answers a normal of
  `(0.894, 0.447)` out of pure noise) or falls back to `+Y`.
- Ball first (`ball_cuboid`, `ball_capsule`, …) runs the convex–ball routine with the arguments
  swapped and swaps the outputs back.

### Integration parameters

- In 0.35.3 contacts use a spring of 30 Hz, damping ratio 10; contacts touching a fixed body use
  60 Hz (`static_contact_softness`); joints 1e6 Hz, ratio 1. All `erp` / `cfm` are evaluated at the
  **substep** length `dt / num_solver_iterations`.
- `normalized_prediction_distance = 0.02`, `normalized_allowed_linear_error = 0.005`,
  `normalized_max_corrective_velocity = 3`, `warmstart_coefficient = 1`.
- The joint angular frequency (`2π · 1e6`) has a raw of 2.7e16: it fits an `i64` but squaring it
  does not. Port the closed forms, not the intermediate products.

## Cairo fixture format

`crates/rapier_golden/src/generated/<family>.cairo` holds one `pub const <CASE>: <CaseType>` per
case, a `pub const ALL: [<CaseType>; N]` table in JSON order and `pub fn cases() ->
Span<<CaseType>>` (plus `DEFAULTS`, `PREDICTION`, `ALL_BODIES` / `body_cases()` where relevant).
The case types live in `src/types.cairo`: plain `#[derive(Copy, Drop, Serde, PartialEq, Debug)]`
structs of raw `i64`, one `ShapeRaw` enum, `felt252` short-string ids, `[T; 2]` fixed-size arrays
for contact points and colliders (unused point slots are zeroed, `num_points` says how many count).
Nothing depends on a fixed-point type: consumers wrap the raws into theirs.

`const` was chosen over constructor functions because Cairo 2.19.4 accepts all of it in constant
position (nested structs, enum variants with payloads, fixed-size arrays, negative `i64` literals,
hexadecimal `u32`), and a constant costs nothing until it is used: no code is generated to build
it, and `ALL.span()` is a pointer to a constant segment.

### Scene fixtures

`generated/scenes.cairo` holds one `pub const <SCENE>: SceneCase` per scene (`BALL_DROP`,
`BALL_BOUNCE`, `BOX_SLOPE_STICK`, `BOX_SLOPE_SLIDE`, `BOX_STACK3`, `PENDULUM`), the `ALL` table and
`cases()`. A `SceneCase` carries the whole description (`gravity`, `dt`, `num_steps`, bodies with
kind, initial pose, damping, gravity scale and colliders with shape / pose / density / friction /
restitution, the revolute joints) and the 22 samples of the trace, so that the engine can replay a
scene and compare every sampled step.

- Variable-length lists are **fixed-size arrays with a count**, the same way manifold points are:
  `bodies: [SceneBodyRaw; 4]` + `num_bodies`, `colliders: [_; 1]` + `num_colliders`, `joints: [_; 1]`
  + `num_joints`, `states: [BodyStateRaw; 3]` + `num_dynamic`. Unused slots are zeroed. The generator
  panics when a scene outgrows a capacity; then widen the array in `types.cairo` and the
  `SCENE_MAX_*` constants in `src/cairo.rs` together.
- **Body order** is the JSON `bodies` order (insertion order upstream). A sample lists the
  *dynamic* bodies only, in that same order, and refers to a body by its index into
  `SceneCase::bodies` (`BodyStateRaw::body`); fixed bodies never move and are not sampled. Joints
  refer to bodies by index too.
- `SceneCase::samples` follows the sampling schedule: index `i <= 10` is step `i`, then steps 20,
  30, …, 120 (`SceneSampleRaw::step` says so explicitly).
- `SceneCase` derives `Copy, Drop` only: Cairo 2.19.4 has no `Serde` / `PartialEq` / `Debug` for a
  fixed-size array of 22 elements. Compare fields, not whole cases.
- The free-text `note` of the JSON is not exported (it does not fit a short string).

Read a scene with `scenes::cases().at(i)`, then walk `samples.span()` and, per sample,
`states.span()`; `crates/rapier_golden/tests/scenes.cairo` has the accessors.

### Leaf-level fixtures

`generated/{pose2, aabb_overlap, sat2d, clip2d, point_projection, segment_segment, ray_casts}.cairo` follow
the same pattern (`pub const <CASE>`, `ALL`, `cases()`); `pose2` also has `ALL_CHAINS` /
`chain_cases()` for the rotation chains. Case types are in `types.cairo` (`Pose2Case`,
`RotChainCase`, `AabbOverlapCase`, `SatCase`, `ClipCase`, `ProjectionCase`, `SegmentPairCase`,
`RayCase`).
Deviations from the scene format worth knowing:

- `AabbOverlapCase` is a fixed-size array with a count (`aabbs: [OverlapBoxRaw; 32]` +
  `num_aabbs`, `pairs: [OverlapPairRaw; 32]` + `num_pairs`, unused slots zeroed); the generator
  panics when a set outgrows a capacity (`OVERLAP_MAX_*` in `src/cairo/leaf_families.rs`, widen
  them with `types.cairo`). It derives `Copy, Drop` only (no `Serde` for arrays that long).
- A `None` upstream result (`clip2d`) is `clipped: false` with zeroed points; a location on a
  segment is `SegmentLocationRaw::{OnVertex(i), OnEdge(u)}` (`NoLocation` for the other shapes), a
  point-projection feature `PointFeatureRaw::{Unknown, Vertex(i), Face(i)}`.
- Inputs that make upstream answer NaN / infinity / `-f64::MAX` (zero-length segments in
  `clip_segment_segment`, the exact centre of a ball, a zero-length segment normal) are recorded
  under `non_finite_probes` in the JSON and **not** exported: the generator refuses to quantise a
  non-finite output, and the port needs a rule of its own there.
- `a_rotation_used`, `b_rotation_used`, `step_norm_squared_minus_one`, the SAT / clip / projection
  `note`s stay in the JSON only.
