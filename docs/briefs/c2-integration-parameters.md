Work package: C2 — `IntegrationParameters` and `SpringCoefficients`
Goal: port Rapier's `IntegrationParameters` (all fields, defaults) and the spring/softness maths the solver derives from them (`SpringCoefficients`: `erp_inv_dt`, `cfm_factor`, `cfm_coeff`, and whatever the current upstream computes at the substep `dt`) into `rapier_core`, on the shared `fixed::Fixed` scalar, and match the golden fixtures bit-for-bit where the maths is rational (or within the documented tolerance where a `sqrt`/division rounding differs).

Files owned:
- `crates/rapier_core/src/integration_parameters.cairo` (new; new submodules under `crates/rapier_core/src/integration_parameters/` if you need them)
- `crates/rapier_core/src/lib.cairo` — only to add `pub mod integration_parameters;` and a re-export
- `crates/rapier_core/tests/integration_parameters.cairo` (new; golden comparisons)
Do not touch any `Scarb.toml`: the `fixed` and `rapier_golden` dependencies are already declared.

Frozen interfaces:
- `fixed::{Fixed, FixedTrait, ONE, ZERO, …}` and `fixed::wide::*` (Q32.32 in `i64`, `*` floors, `/` truncates toward zero, overflow panics). Read `fixed`'s README in the Scarb cache or the dependency checkout under `target/` — do not modify it.
- Golden fixtures: `rapier_golden::integration_parameters::{DEFAULTS, DT_F64, DT_Q32, ALL}` and the raw struct types in `rapier_golden::types` (`IntegrationDefaultsRaw`, `IntegrationDerivedRaw`, `SpringDefaultsRaw`, `SpringDerivedRaw`). `DT_Q32` is the case computed with `dt` already quantised to Q32.32; that is the one the port must match most closely. Comparison helpers: `rapier_golden::compare::{within, abs_diff}`.

Upstream reference (read-only clone):
/private/tmp/claude-501/-Users-bal7hazar-git-rapier-cairo--claude-worktrees-rapier-physics-cairo-benchmark-2d3317/239b7c62-97d7-47ea-9d79-74f81363555f/scratchpad/refs/rapier/src/dynamics/integration_parameters.rs (struct, `Default`, `dt`/`inv_dt`, `substep_dt`, `contact_softness_coefficients`, `joint_softness_coefficients`, the `SpringCoefficients`/`SpringDampingRatio`-style types and their math), plus `tools/golden/src/params.rs` in this repository to see exactly which upstream functions produced each fixture field.

Requirements:
1. `IntegrationParameters` as a `#[derive(Copy, Drop, Serde, PartialEq, Debug)]` struct with every upstream field that survives the plan's cuts (keep the CCD/recycling/clustering fields as plain data so defaults match the fixtures; document that they are unused for now). `Default` impl reproducing upstream defaults exactly (compare each field to `DEFAULTS` in a test).
2. Derived quantities as methods mirroring upstream names: `inv_dt`, `substep_dt`, `substep_inv_dt`, `allowed_linear_error`, `max_corrective_velocity`, `prediction_distance`, `max_linear_velocity`, `contact_recycle_distance`, `contact_softness_coefficients`, `static_contact_softness_coefficients` (or whatever upstream calls it), `joint_softness_coefficients`. Study the fixture `SpringDerivedRaw` fields to know what to output.
3. Numeric care (decision D3/D4 in `docs/PLAN.md`): joint softness upstream uses a 1e6 Hz natural frequency, giving `cfm_coeff` of only a few ulp in Q32.32. Implement the general formula, compare against the fixture, and report the error in ulps for every derived field of `DT_Q32` (table in your final report). If a field cannot be reproduced within 8 ulp by the straightforward formula, try reordering the operations (e.g. keep the product `w·dt` wide via `fixed::wide` before dividing) and report both. Do not special-case values to hit the fixture.
4. Prefer `fixed::wide` kernels over chains of `*` and `/` (one rescale per output); keep divisions out of anything called per substep — precompute in the coefficient struct.
5. Tests: `test_defaults_match_golden` (field by field), `test_derived_match_golden_dt_q32` (each field within a per-field tolerance you justify in a comment), `test_derived_dt_f64_within_tolerance`, edge cases (`dt = 0` behaviour — check what upstream does and mirror or panic with an `errors` const), and `gas_*` probes for `Default::default`, `substep_dt`, and each `*_softness_coefficients` call (inputs through `rapier_testing::opaque`, one `gas_baseline`).
6. `///` docs on every public item, including the rounding direction and value range.

Acceptance: from the repo root, `scarb fmt --check --workspace`, `scarb lint --workspace --deny-warnings`, `scarb build --workspace`, `snforge test --workspace` all pass; `python3 scripts/gas.py diff` table in the report; error-in-ulps table in the report.

Out of scope: the solver, bodies, any change to `fixed`, `rapier_golden` or `Scarb.toml` files, CI.
