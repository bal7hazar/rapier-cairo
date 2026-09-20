# M2 — `rapier_math::{Rot2, Pose2}` (the `glamx` layer)

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D2, D12, wave 2); `docs/interfaces/geometry-dynamics.md` (how `Pose2`
is consumed: `inv_mul` → `pos12`, transforms of points/normals); style precedents on `main`:
`crates/rapier_math/src/math_ext/vec2.cairo` (fused kernels, candidates under `mod alternatives`,
`gas_*` probes) and `crates/rapier_core/src/integration_parameters.cairo` (golden comparisons).
Upstream (read-only clones at /private/tmp/claude-501/-Users-bal7hazar-git-rapier-cairo--claude-worktrees-rapier-physics-cairo-benchmark-2d3317/239b7c62-97d7-47ea-9d79-74f81363555f/scratchpad/refs/):
`parry/src/math/mod.rs` (`Rot2`/`Pose2` aliases and the `glamx` types they wrap — find `glamx` in
`~/.cargo/registry/src/*/glamx-*` if present), `rapier/src/dynamics/rigid_body_components.rs`
(`integrate_linearized` / rotation update by an angular velocity).

## 2. Scope (file allowlist)
- `crates/rapier_math/src/rot2.cairo`, `crates/rapier_math/src/pose2.cairo` (submodules allowed)
- `crates/rapier_math/tests/pose2_golden.cairo`
- `gas/rapier_math/rot2.snap`, `gas/rapier_math/pose2.snap`, `gas/rapier_math_integrationtest/pose2_golden.snap`
`lib.cairo` already declares `pub mod rot2; pub mod pose2;` (pre-declared stubs). Everything else
is forbidden; needs go under "Escalations" in `REPORT.md`.

## 3. Expected API and semantics
Frozen inputs: `fixed::{Fixed, FixedTrait, …}`, `fixed::wide::{wide_mul, dot2, dot2_add, mul_sub,
mul_add, norm2, normalize2, Recip, W*}` (`*` floors, `/` truncates toward zero, overflow panics);
`glam::Vec2` (public `x`, `y`, glam-rs method names); `rapier_math::consts`
(`COS_1_DEGREES`, `UNIT_TOL_SQ_RAW`, …); `rapier_math::math_ext` (`is_unit2_raw`,
`try_normalize2`, `gcross_*`, `perp`). Golden fixtures: `rapier_golden::pose2` (7 pose pairs, 3
rotation chains with the f64 `re² + im²` drift), `rapier_golden::aabb` poses, `rapier_golden::scenes`
per-step rotations; helpers in `rapier_golden::compare`.

```cairo
#[derive(Copy, Drop, Serde, PartialEq, Debug)] pub struct Rot2 { pub re: Fixed, pub im: Fixed }
#[derive(Copy, Drop, Serde, PartialEq, Debug)] pub struct Pose2 { pub translation: Vec2, pub rotation: Rot2 }
```
`Rot2`: `IDENTITY`, `Default`, `from_cos_sin(re, im)` (renormalising), `mul`, `inverse` (conjugate),
`rotate(v)`, `inverse_rotate(v)`, `is_unit()` (`is_unit2_raw`), `renormalize()` (`normalize2` on
the wide sum of squares), `integrate(angvel: Fixed, dt: Fixed)` (upstream's small-angle update
then renormalise), `Mul` operator. `Pose2`: `IDENTITY`, `Default`, `new(translation, rotation)`,
`mul`, `inverse`, `inv_mul(other)` (= `pos12`, one fused kernel — the hot path), `transform_point`,
`transform_vector`, `inverse_transform_point`, `inverse_transform_vector`, `Mul` operator.
Upstream never renormalises after `mul` (G2 finding); the port exposes renormalisation explicitly
and documents at type level when it is expected (see §4 study). DEFER: `Rot2::angle()`,
`from_angle` (need trig, glam item F3): leave a doc note, no panicking stub. DEFER: `lerp_slerp`.

## 4. Efficiency and variants to bench
One rescale per output component (D2). For `rotate`, `inverse_rotate`, `mul`, `inv_mul`,
`transform_point`, `inverse_transform_point`: implement the fused kernel and the composed-from-
`Vec2`-ops candidate, measure both, ship the winner, keep the loser under `mod alternatives`.
Drift study (report table): after 1 000 `mul`s by a small rotation, `|re² + im² − 1|` in raw
units without renormalisation vs renormalising every k ∈ {1, 4, 16} steps; compare with the f64
drift recorded in `rapier_golden::pose2` chains; recommend a policy for the solver (renormalise once
per step vs per substep) in `REPORT.md`.

## 5. Tests
Table-driven `test_*`: identity/inverse round-trips, `inv_mul` consistency (`a.inv_mul(a)` is
identity within `UNIT_TOL_SQ_RAW` and 2 ulp of translation), transforms against every
`rapier_golden::pose2` case within the tolerances its README recommends, scene sample rotations
stay unit within `UNIT_TOL_SQ_RAW`; `fuzz_*` (fixed seed, ≤ 4 per module) equivalence of
candidates; `gas_*` for every public function and candidate (`rapier_testing::opaque` inputs, one
`gas_baseline` per test module). Compile budget: ≤ 800 lines per file. Panics: document each
(`'Fixed: overflow'`, `'Rot2: zero'` for `from_cos_sin(0, 0)` and `renormalize` of zero).

## 6. Definition of done
Foreground gate from the worktree root: `scarb fmt --workspace && scarb lint --workspace
--deny-warnings && scarb build --workspace && snforge test --workspace`; then
`python3 scripts/gas.py snapshot --filter rapier_math::rot2`, `… --filter rapier_math::pose2`,
`… --filter rapier_math_integrationtest::pose2_golden`; conventional commits with the trailer;
push; `gh pr create` following `.github/PULL_REQUEST_TEMPLATE.md`; `gh pr checks --watch` until
green; never merge; `REPORT.md` (Summary · API · Gas table · Deviations · Deferred · Requested
re-exports · Escalations · PR URL) with the drift table and the renormalisation recommendation.

## 7. Work autonomously, do not ask questions, do not widen the scope.
