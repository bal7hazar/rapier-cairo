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
| `parry2d-f64` | `=0.30.2` | the version `rapier2d-f64 0.35.3` depends on (`^0.30.2`). `parry2d-f64 0.31.1` exists on crates.io but no published Rapier uses it; pinning 0.31 would silently put two Parry copies in the build |
| `parry2d` (f32) | `=0.30.2` | feature ids only, see [Feature ids](#feature-ids-f64-upstream-bug) |

Note that the local upstream clones used by the research reports are Rapier `master` (0.35.x +
soft bodies) and Parry 0.31.x: the vectors come from the **published** pair above, not from the
clones.

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
| `contact_manifolds.json` | 66 | `<shape1>_<shape2>/<regime>` | `DefaultQueryDispatcher::contact_manifolds` with the default prediction distance; see below |
| `scenes.json` | 6 | scene name | full-engine traces (also exported to Cairo, see [Scene fixtures](#scene-fixtures)) |

### contact_manifolds

Pairs (shape 1 first): `ball_ball`, `ball_cuboid`, `ball_capsule`, `cuboid_cuboid`,
`cuboid_capsule`, `capsule_capsule`, `halfspace_ball`, `halfspace_cuboid`, `segment_ball`.
Regimes: `separated` (beyond prediction), `within_pred` (gap 0.01 < prediction), `touching`
(distance exactly 0), `shallow`, `deep`, `degenerate`. That is 9 × 6 = 54 canonical cases, plus

- 7 extra degenerate cases named `degen_<what>` (regime `degenerate`): ball centre exactly on a
  face / plane / segment, corner against corner, exact quarter turn, crossing capsule segments,
  zero-length capsule;
- 5 flipped-order cases (`cuboid_ball`, `capsule_ball`, `ball_halfspace`, `ball_segment`,
  `capsule_cuboid`, regime `shallow`) because upstream swaps the arguments and the outputs.

Each case records both shapes, `pos12` (pose of shape 2 in the frame of shape 1) and, per
manifold, `local_n1`, `local_n2`, the number of points and for each point `local_p1`, `local_p2`,
`dist`, `fid1`, `fid2` (packed value + decoded kind and code).

`"ambiguous": true` (16 cases) marks a case whose discrete outputs — point count, feature ids,
even the sign of the normal — hinge on an exact tie or on a fallback branch upstream. A port may
legitimately answer differently there; compare `dist` and treat the rest as informative.

### scenes

`ball_drop`, `ball_bounce` (restitution 0.7), `box_slope_stick` (μ = 0.7 > tan 30°),
`box_slope_slide` (μ = 0.25), `box_stack3`, `pendulum` (revolute joint). 120 steps at the
quantised `dt`, gravity `(0, -9.81)` snapped to Q32.32. The file describes each scene completely
(bodies in insertion order, colliders, material, joints) and samples every dynamic body at step 0
(initial state), steps 1–10, then every 10th step: `translation`, `rotation` `(re, im)`,
`linvel`, `angvel`, read after `PhysicsPipeline::step`.

## Settings deviating from Rapier's defaults

Everything D11 asks for could be disabled through the public API or a cargo feature:

| setting | default | here | how | why |
|---|---|---|---|---|
| block solver | on | **off** | cargo feature `block-solver` not enabled | the port solves manifold points sequentially (MVP) |
| contact recycling | on | **off** | `IntegrationParameters::contact_recycling = false` | the port recomputes every manifold every step |
| contact clustering | on | **off** | `IntegrationParameters::contact_clustering = false` | only acts on pairs with several manifolds; never the case for the convex pairs here, disabled for clarity |
| CCD | 1 substep | **off** | `max_ccd_substeps = 0` (skips the CCD branch entirely) and `ccd_enabled(false)`, no soft-CCD | not in the MVP |
| sleeping | on | **off** | `RigidBodyBuilder::can_sleep(false)` on every body | not in the MVP; removes the 0.5 s sleep timer from traces |
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
| scenes `ball_drop`, `ball_bounce`, `pendulum`, `box_slope_*` | `2^12 · step` ulp on positions (≈ 1e-6 per step), twice that on velocities | single-contact or joint-only scenes have no solver-order ambiguity; the error is rounding accumulated over ~10³ operations per step, growing at most linearly while the motion is not chaotic. After the first bounce of `ball_bounce`, compare bounce apex and impact step rather than samples |
| scene `box_stack3` | invariants, not samples | rest heights within `allowed_linear_error` (0.005) of `0.5 + i`, `|x| < 0.01`, final speeds `< 1e-3`; multi-contact ordering differs from upstream by construction |

## Upstream behaviours worth knowing

Found while building the vectors; all visible in the JSON.

### Feature ids (f64 upstream bug)

`Cuboid::vertex_feature_id` extracts "sign bits" with `to_bits() >> 31` / `>> 30`. That is the sign
bit of an `f32`, but a **mantissa** bit of an `f64` (upstream left a `TODO: is this still correct
with the f64 version?`). In `parry2d-f64` every cuboid vertex id therefore collapses to `0` and
every face id to `0b110000`, which defeats contact matching (warm starting) for cuboids in f64
builds. The vectors expose the ids of an **f32 run of the same case** as `fid1` / `fid2` — the
scheme the port should implement — and keep the f64 build's ids as `fid1_f64_build` /
`fid2_f64_build`. When the two builds disagree on the points themselves (only allowed for
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
