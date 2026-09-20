Work package: M3 — `rapier_math::math_ext`
Goal: the scalar helpers Rapier/Parry rely on that do not belong to `fixed` (glam.cairo's shared Q32.32 scalar), implemented on `fixed::Fixed` in the new `rapier_math` crate, with the fixed-point hazards of squared-quantity comparisons solved once, here, for every later package.

Files owned:
- `crates/rapier_math/src/math_ext.cairo` (new; submodules under `crates/rapier_math/src/math_ext/` allowed)
- `crates/rapier_math/src/consts.cairo` (new)
- `crates/rapier_math/src/lib.cairo` — only to add `pub mod` lines and re-exports (the crate skeleton exists; do not touch its `Scarb.toml`)
Do not create `Rot2`/`Pose2` (package M2, another executor).

Frozen interfaces: `fixed::{Fixed, FixedTrait, …}`, `fixed::wide::*` (`wide_mul`, `W1..W16`, `dot2`, `mul_sub`, `norm2`, `normalize2`, `Recip`, `narrow`, …). Read the `fixed` README and `wide.cairo` from the dependency checkout under `target/` or the Scarb cache. Semantics: `*` floors, `/` truncates toward zero, `recip(0)` panics, overflow panics.

Upstream reference (read-only clone at /private/tmp/claude-501/-Users-bal7hazar-git-rapier-cairo--claude-worktrees-rapier-physics-cairo-benchmark-2d3317/239b7c62-97d7-47ea-9d79-74f81363555f/scratchpad/refs/):
- `rapier/src/utils.rs`: `inv` (returns 0 for 0 — Rapier relies on it for infinite-mass bodies), `simd_*` helpers you can ignore, `gcross`/`gdot`/`perp` family for 2D (`WCross`, `WDot`, `WBasis`), `cap_magnitude`, `smallest_abs_component`, `orthonormal_vector`, `WSign::copy_sign_to`, `WReal` epsilon helpers.
- `parry/src/math/mod.rs`, `parry/src/utils/*.rs` (`DEFAULT_EPSILON`, `COS_1_DEGREES`, `COS_5_DEGREES`, `SIN_10_DEGREES`, `COS_10_DEGREES`, `COS_45_DEGREES`, `SIN_45_DEGREES`, `COS_FRAC_PI_8`, `SIN_FRAC_PI_8`, `try_normalize`-style helpers), and `docs/research/02-parry-analysis.md` §5.10 and §6 in this repository (tolerances, squared-quantity hazard).

Requirements:
1. `consts`: every upstream angular/epsilon constant expressed in **raw Q32.32** (`Fixed` consts) with the exact decimal it approximates in the doc comment; `DEFAULT_EPSILON` re-derived for Q32.32 and justified (upstream uses f32/f64 machine epsilon; here the resolution is 2^-32 and the relevant tolerance for "is this vector zero / normalised" must be argued in ulps — write that argument down in the module doc).
2. `math_ext` scalar helpers: `inv` (0 → 0, otherwise `recip`), `copysign` wrappers if `fixed` lacks the exact upstream semantics, `cap_magnitude`-style clamps, `smallest_abs_component`.
3. **Wide comparisons of squared quantities** — the core of this package. Rapier/Parry compare `length_squared` against thresholds (`eps²`, `prediction²`, `1 - 1e-6` for normalisation checks). In Q32.32 a squared length that is rescaled loses half its bits and underflows for small vectors. Provide comparison helpers that keep the raw product wide (`fixed::wide::W*`, Q64.64) and compare against a pre-scaled constant: e.g. `is_norm2_lt(x, y, threshold)`, `is_norm2_between(...)`, `is_unit2(x, y, tol_ulps)`, `is_zero2(x, y)`, `norm2_cmp(a_x, a_y, b_x, b_y)` for comparing two lengths without a sqrt. Implement at least two candidates for the main helpers (wide comparison vs rescale-then-compare vs sqrt-then-compare) and measure them; the wide version is expected to win on both precision and gas — prove it with `test_*` cases that the naive version gets wrong (tiny vectors, large vectors near the overflow boundary) and `gas_*` probes.
4. 2D vector helpers that Rapier uses beyond glam's `Vec2` API and that M2 will build on: `gcross(scalar, (x, y))`, `gcross((x, y), scalar)`, `gcross((ax, ay), (bx, by))` (= perp-dot), `gdot`, `perp`, `orthonormal_vector`, `try_normalize2` returning `Option<(Fixed, Fixed)>` using the wide zero test (component tuples for now — do not introduce a `Vec2` type in this package; M2 owns types).
5. Fuzz tests (fixed seed) proving candidate equivalence on the domain where both are valid; edge cases: zero, ±1 ulp, ±`MAX`, mixed signs; document every panic.
6. `gas_*` probes for every public function and every candidate (inputs through `rapier_testing::opaque`, one `gas_baseline` per test module); losers under `#[cfg(test)] mod alternatives`.

Acceptance: from the repo root, `scarb fmt --check --workspace`, `scarb lint --workspace --deny-warnings`, `scarb build --workspace`, `snforge test --workspace` pass; `python3 scripts/gas.py diff` table and the candidate ranking (net gas) in the report; the written argument for `DEFAULT_EPSILON` and the unit/zero tolerances in the module docs.

Out of scope: `Rot2`, `Pose2`, any `Vec2` struct, trig, changes to `fixed`, `Scarb.toml` files, CI.
