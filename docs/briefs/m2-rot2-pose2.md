Work package: M2 — `rapier_math::{Rot2, Pose2}` (the `glamx` layer)
Goal: the 2D rotation (unit complex) and rigid pose types that Rapier/Parry use everywhere, as fused kernels on glam.cairo's `Vec2` and `fixed::wide`, with a renormalisation policy that survives Q32.32 (no rotation other than multiples of 90° is exactly unit).

Files owned:
- `crates/rapier_math/src/rot2.cairo`, `crates/rapier_math/src/pose2.cairo` (new; submodules allowed)
- `crates/rapier_math/src/lib.cairo` — only `pub mod` lines and re-exports
- `crates/rapier_math/tests/pose2_golden.cairo` (new)
Do not touch `consts.cairo`, `math_ext/**` (use them), any `Scarb.toml`.

Frozen interfaces:
- `fixed::{Fixed, FixedTrait, …}`, `fixed::wide::{wide_mul, W1.., dot2, dot2_add, mul_sub, mul_add, norm2, normalize2, Recip, …}` (`*` floors, `/` truncates, overflow panics).
- `glam::Vec2` from glam.cairo (public `x`, `y`; `dot`, `perp_dot`, `length`, `length_squared`, `try_normalize`, operators, `mul_scalar`) — mirror glam-rs names; read its README/source in the dependency checkout under `target/` or the Scarb cache.
- `rapier_math::consts` (`COS_1_DEGREES`, `UNIT_TOL_SQ_RAW`, …) and `rapier_math::math_ext` (`is_unit2_raw`, `try_normalize2`, `gcross_*`, `perp`).
- Golden fixtures: `rapier_golden::aabb` cases carry poses (`PoseRaw { translation: Vec2Raw, rotation: RotRaw { re, im } }`) recorded exactly as passed to upstream (unnormalised); `rapier_golden::scenes` samples carry per-step rotations. Use `rapier_golden::compare`.

Upstream reference (read-only clones at /private/tmp/claude-501/-Users-bal7hazar-git-rapier-cairo--claude-worktrees-rapier-physics-cairo-benchmark-2d3317/239b7c62-97d7-47ea-9d79-74f81363555f/scratchpad/refs/): `parry/src/math/mod.rs` (the `Rot2`/`Pose2` aliases and what they wrap), the `glamx` crate as vendored in `~/.cargo/registry` if present or its source referenced from `parry/Cargo.toml` (`Rot2 { re, im }`, `Pose2 { translation, rotation }`, `mul`, `inverse`, `inv_mul`, `transform_point`, `transform_vector`, `inverse_transform_point`, `from_angle`, `angle`, `renormalize`, `lerp_slerp` if any). `rapier/src/dynamics/rigid_body_components.rs` `integrate_linearized`/`integrate` for how the rotation is advanced by an angular velocity (small-angle update then renormalise) — port that helper here as `Rot2::integrate(angvel, dt)`.

Requirements:
1. `Rot2 { re: Fixed, im: Fixed }` (unit complex; `IDENTITY`), `Pose2 { translation: Vec2, rotation: Rot2 }` (`IDENTITY`), `#[derive(Copy, Drop, Serde, PartialEq, Debug)]`, `Default = IDENTITY`.
2. Fused kernels, one rescale per output component: `Rot2::mul`, `Rot2::inverse` (conjugate, free), `Rot2::rotate(v)` / `inverse_rotate(v)` (`mul_sub`/`dot2`), `Pose2::mul`, `Pose2::inverse`, `Pose2::inv_mul(other)` (the `pos12` relative pose Parry computes per pair — this is the hot path, make it one kernel), `transform_point` (`dot2_add`), `transform_vector`, `inverse_transform_point`, `inverse_transform_vector`. For each, implement the composed-from-`Vec2`-ops candidate too, measure, ship the winner, keep the loser under `mod alternatives`.
3. Angle I/O only where trig is not needed: `Rot2::from_cos_sin(re, im)` (renormalising), `Rot2::angle()` is out of scope (needs `atan2`, glam item F3) — leave a documented `todo` note, do not stub with a panic.
4. **Renormalisation policy** (must be documented at the type level): a `Rot2` is considered unit when `is_unit2_raw(re, im)` holds; `renormalize()` uses `normalize2` on the wide sum of squares; `integrate(angvel, dt)` performs upstream's update then renormalises. Show with a test how far `|re² + im²|` drifts after 1 000 `mul`s by a small rotation without renormalisation vs with renormalisation every k steps (k = 1, 4, 16) and report the numbers; recommend a policy for the solver (renormalise once per step vs per substep) in the report.
5. Golden tests: for every `rapier_golden::aabb` case, rebuild the pose from `PoseRaw` and check `transform_point` of the shape's local extreme points against what the fixture's AABB implies (or at minimum that `inv_mul` of a pose with itself is identity within tolerance and that `mul(inverse)` round-trips); for scene samples, check that consecutive rotations stay unit within `UNIT_TOL_SQ_RAW`.
6. `gas_*` for every public function and every candidate (`opaque` inputs, `gas_baseline` per module); `fuzz_*` fixed-seed equivalence between candidates.

Acceptance: fmt, lint, build, `snforge test --workspace` green; `gas.py diff` table and the drift table in the report; the renormalisation recommendation.

Out of scope: trig, 3D, `Vec2` itself, shapes/AABB, `Scarb.toml`, CI.
