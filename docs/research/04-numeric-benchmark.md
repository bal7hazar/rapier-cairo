# 04 — Numeric benchmark: choosing the fixed-point scalar of `rapier.cairo`

> Empirical study. Every number in the result tables below was **measured** with `snforge` on the
> bench project described in §1; anything that is an estimate or an argument rather than a
> measurement is explicitly marked *(estimate)* or *(not measured)*.
> Bench project (kept compiling, `snforge test -p numbench` green, 415 tests):
> `/private/tmp/claude-501/-Users-bal7hazar-git-rapier-cairo--claude-worktrees-rapier-physics-cairo-benchmark-2d3317/239b7c62-97d7-47ea-9d79-74f81363555f/scratchpad/numbench`

## 0. TL;DR

| Question | Answer (measured) |
|---|---|
| Format | **Q32.32 signed**, stored as a native **`i64`** in a one-field struct (1 felt per scalar). |
| Representation | **Not** sign-magnitude (cubit/Orion style). `i64` add/sub/lt cost 6 steps, constant, vs 4–27 data-dependent steps for `{mag, sign}`; one felt instead of two. |
| mul / rescale | `core::internal::bounded_int`: `mul(i64,i64)` (0 range checks) → `+2^126` → `div_rem` by the constant `2^32` → `-2^94` → **one** `downcast` to `i64`. **16 steps / 5 RC / 1 950 gas**, vs 18 / 5 / 2 250 for cubit f64 and 33 / 7 / 4 010 for the naive `i128` route. |
| Fused kernels | The decisive win. Accumulate products as `BoundedInt` and rescale **once**: dot2 **18** steps (cubit f64: 69), mat3·vec3 **82** (cubit f64: 344, cubit f128: 959), mat3·mat3 **214** (Orion tensor matmul: 3 038), 6-term constraint row **26** (naive: 134). Bounds are proven by the type system at compile time: no soundness argument to maintain by hand. |
| sqrt / length | core `u128_sqrt` on the widened magnitude: **10–17 steps**. `length = sqrt(Σ wide squares)` directly: **17 steps**, and the squared length never has to fit Q32.32 (no overflow above 46 340 units). Never Newton (231–356 steps and not converged after 8 fixed iterations for extreme inputs). |
| trig | Odd minimax polynomial + arithmetic range reduction: `sin` deg 7 = **144 steps, 5.9e-7 max error**; deg 9 = **162 steps, 7.7e-9**. cubit Taylor: 1 051 steps (1.6e-8); cubit LUT: 206 steps (4.6e-6) plus a 1 331-line table. |
| div | abs/sign split + `bounded_int::div_rem`: 44 steps. cubit's sign-magnitude div is cheaper (18) — the one significant op where sign-magnitude wins (it also wins abs/neg by a few steps and sqrt by 7). Physics code should store inverses (as Rapier already does) and multiply. |
| Orion | FP32x32 *is* cubit f64 (type alias), FP16x16/FP8x23 are the cubit algorithms at lower width (±3 steps). Tensor linalg is **3.2×** slower than unrolled struct code on the *same* scalar and **14×** slower than the recommended fused `Mat3`. Nothing to reuse for physics. |
| Owner heuristic | Confirmed with one nuance: in **steps**, bitwise AND/OR tie with DivRem (7 vs 8, 11 vs 12); in **l2_gas** bitwise is 12–55 % dearer (one bitwise builtin use is billed ≈ 483 gas vs 70 for a range check, derived from the measured rows at 100 gas/step); loops are 13–140× dearer. With `BoundedInt`, packing two `u32` costs **1 step / 0 RC**. |

## 1. Methodology

### 1.1 Toolchain

| Tool | Version |
|---|---|
| scarb | 2.19.4 (b45b74c03 2026-07-21) — cairo 2.19.4, sierra 1.9.3, aarch64-apple-darwin |
| starknet-foundry (`snforge`, `snforge_std`) | 0.61.0 |
| cubit (influenceth) | shallow clone `8007a30`, v1.4.0 |
| orion (gizatechxyz) | shallow clone `bac0b42`, v0.2.5 (declares `cairo-version = 2.5.3`, pins cubit `6275608` and alexandria `800f5ad`) |
| alexandria | `800f5ad` for the three crates Orion needs (cloned separately); HEAD `6d2cfcc` inspected only |
| origami | `1ddafcb`, inspected only |

Managed by asdf through `.tool-versions` in the bench directory.

### 1.2 Project layout

A scarb workspace: `packages/cubit`, `packages/orion`, `packages/alexandria_*` are **source copies**
(not git dependencies) compiled with their original edition `2023_10`; `packages/bench`
(`numbench`, edition `2024_07`) contains my representations, kernels and all tests. See the
`README.md` in the bench directory for the rerun commands
(`python3 scripts/run.py && python3 scripts/sweep.py`).

### 1.3 How a number is produced

* One `#[test]` per operation. `scripts/run.py` runs the suite twice:
  `snforge test -p numbench --detailed-resources --tracked-resource cairo-steps` → **steps**,
  **builtin counters**, memory holes; and `--tracked-resource sierra-gas` → **l2_gas**. (In
  `cairo-steps` mode snforge clamps `l2_gas` to a 40 000 floor, useless for micro-benchmarks; in
  `sierra-gas` mode no step count is printed. Hence two runs.)
* **Anti constant-folding**: every input goes through `#[inline(never)] fn bb<T>(x: T) -> T`, every
  result is consumed by `#[inline(never)] fn sink<T>(x: T)`. The control experiment is in table G:
  the same multiplication with literal inputs costs **0 steps** (fully folded), with black-boxed
  inputs 16.
* **Baseline subtraction**: every module has `base_<k>` tests that build exactly the same
  black-boxed inputs and call the same `sink` but perform no operation; `<k>_<op>` tests are
  reported **net** of their baseline. A bare snforge test costs ≈ 65–116 steps / 14 300–19 400 gas
  depending on the number of inputs (up to 391 steps when two Orion tensors are built), so this matters. Resolution of the subtraction is about
  **±3 steps** (the compiler lays out the baseline and the benchmark slightly differently — e.g.
  sign-magnitude `lt` with mixed signs nets to 0).
* Execution is deterministic, so a single run per test is exact; there is no statistical noise.
* Same numeric inputs for every library/representation: a = 3.5, b = −1.25, c = 1.25,
  sqrt input 1234.5678 (200.5678 for FP8x23 whose range is ±256), angle 0.7 rad (and 2.5 rad),
  vectors (3.5, −1.25)·(0.75, 2.125), a rotation-like 3×3 matrix and (3.5, −1.25, 2.0).
  Sign-magnitude costs are data dependent, so both same-sign and mixed-sign cases are reported.
* Every representation has a `check_correctness` test (results compared with Python-computed
  expectations, and fused vs unfused kernels compared with each other).
* Cell format in the pivot tables: **steps / range_check count / l2_gas** (net).
* What “steps” does not capture: builtin usage is billed on top. `l2_gas` (sierra gas) already
  folds both in and is the number to use for budgeting; steps + RC are given to reason about
  prover cost.

### 1.4 Changes made to third-party code (all marked `NUMBENCH PATCH` in the sources)

**cubit** (algorithms untouched)
1. All trailing `#[cfg(test)] mod … {}` unit-test modules stripped (one-off script
   `strip_tests.py` kept in the bench root): they no longer parse — `assert(a < b == false, …)` is now
   `error[E1028]: Consecutive comparison operators are not allowed`.
2. Unreferenced legacy files `src/math/`, `src/types/` (not in `lib.cairo`) and `*.js` removed.
3. `impl … of AddAssign/SubAssign/MulAssign/DivAssign<Fixed, Fixed>` replaced by the legacy
   `AddEq/SubEq/MulEq/DivEq` impls (f64 and f128). Reason: Orion's generic tensor code requires
   `+AddEq<T>`; corelib bridges `AddEq → AddAssign` but not the reverse, and having both is
   `E2313 multiple implementations`. Bodies are identical (`self = Add::add(self, rhs)`).
4. Manifest: `starknet = "2.19.4"`, edition kept at `2023_10`.

**orion**
1. Only `numbers` (all fixed-point implementations, complex numbers, signed ints),
   `operators::{tensor, matrix, vec}`, `utils`, `test_helper` are compiled.
   `operators::{nn, ml, sequence}` were dropped: `ml/tree_ensemble` produces 17 ×
   `E3002 Variable not dropped`, `nn/functional/conv.cairo` hits the `Div<i32>` ambiguity below;
   none of it is relevant here.
2. `numbers.cairo`: removed `I8Div … I128Div` (corelib now implements `Div` for signed integers →
   `E2313`) and `I8IntoFP32x32` / `I8IntoFP64x64` (duplicate of impls now shipped by cubit).
   Import lists in `tensor_i8/i32/fp32x32/fp64x64.cairo` adjusted accordingly.
3. `operators/tensor/core.cairo`: `TensorSerde` removed (corelib `SpanSerde` now carries a
   negative impl bound `-TypeEqual<felt252, T>` that a generic `T` cannot satisfy);
   `*(*self).shape.at(i)` → `*(*self.shape).at(i)` (`E3003 Cannot desnap a non copyable type`).
4. `operators/matrix.cairo`: `let mut sum_exp: T = NumberTrait::zero();` (type annotation, `E2314`).
5. Unit-test modules stripped (17 files).
6. Built against cubit **HEAD** instead of the pinned rev `6275608` (hence cubit patch 3).

**alexandria @ 800f5ad** — `data_structures` reduced to `array_ext` + `vec` (drops the
`encoding → math → numeric` dependency chain); `integer::u32_wrapping_add(x, 1)` → `x + 1`;
`nullable_from_box(BoxTrait::new(v))` → `NullableTrait::new(v)`; `concat_span` trait bound aligned
(`+Destruct<T>`); `merkle_tree`: `use hash::…`/`pedersen::…`/`poseidon::…` → `core::…`, storage-proof
module dropped; test modules stripped.

So **Orion could be compiled almost whole** (≈ 100 errors, 6 categories); no extraction of a
fixed-point core was necessary.

### 1.5 What was not done
* `atan2` does not exist in cubit or Orion; `atan` is benchmarked for the libraries, and my own
  `atan2` (polynomial) is benchmarked in D.
* Orion `FP64x64`, `FP16x16W`, `FP8x23W` were not benchmarked individually (FP64x64 is an alias of
  cubit f128, which *is* benchmarked; the “wide” variants only widen the intermediate type).
* No native `i128` Q64.64 representation was written (only sign-magnitude `u128`).
* Newton inverse-sqrt iterations were not implemented (the `2^96/mag → u128_sqrt` formulation at
  21 steps made them pointless).
* alexandria and origami were inspected, not benchmarked: alexandria has no fixed-point type
  (`linalg` = loop-based generic `dot/kron/norm` over `Span<T>`; `math::trigonometry` = degree-based
  table lookups with 1e8 decimal scaling; `wad_ray_math` = unsigned decimals); origami's
  `algebra::Vec2<T>` is a thin generic wrapper over a cubit f128 fork.
* Costs are from the snforge VM accounting, not from an actual proof; Starknet fee weights can change.

## 2. Raw results

### A. Library scalar operations (steps / RC / l2_gas)

| op | cubit f64 (32.32) | cubit f128 (64.64) | orion FP16x16 | orion FP8x23 | orion FP32x32 |
|---|---:|---:|---:|---:|---:|
| add_same_sign | 4 / 1 / 470 | 19 / 1 / 2130 | 4 / 1 / 470 | 4 / 1 / 470 | 4 / 1 / 470 |
| add_mixed_sign | 15 / 2 / 1640 | 27 / 2 / 2960 | 15 / 2 / 1640 | 15 / 2 / 1640 | 15 / 2 / 1640 |
| sub | 27 / 1 / 4450 | 26 / 1 / 4550 | 27 / 1 / 4450 | 27 / 1 / 4450 | 27 / 1 / 4450 |
| mul | 18 / 5 / 2250 | 91 / 24 / 11580 | 15 / 4 / 1780 | 15 / 4 / 1780 | 18 / 5 / 2250 |
| div | 18 / 5 / 2250 | 93 / 24 / 11880 | 15 / 4 / 1780 | 15 / 4 / 1780 | 18 / 5 / 2250 |
| neg | 5 / 0 / 500 | 5 / 0 / 500 | 5 / 0 / 500 | 5 / 0 / 500 | 5 / 0 / 500 |
| abs | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| lt | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| lt_same_sign | 15 / 1 / 1580 | 15 / 1 / 1580 | 15 / 1 / 1580 | 15 / 1 / 1580 | 15 / 1 / 1580 |
| sqrt | 35 / 13 / 4410 | 74 / 21 / 8880 | 13 / 5 / 1650 | 13 / 5 / 1650 | 35 / 13 / 4410 |
| sin | 1051 / 197 / 130270 | 3642 / 945 / 460230 | 976 / 172 / 118670 | 976 / 172 / 118670 | 1051 / 197 / 130270 |
| sin_fast_lut | 206 / 29 / 30320 | 386 / 79 / 52600 | 172 / 26 / 25290 | 172 / 26 / 25290 | 206 / 29 / 30320 |
| cos | 1084 / 199 / 134820 | 3676 / 947 / 464880 | 1011 / 174 / 123270 | 1011 / 174 / 123270 | 1084 / 199 / 134820 |
| cos_fast_lut | 263 / 31 / 34870 | 444 / 81 / 57250 | 229 / 28 / 29840 | 229 / 28 / 29840 | 263 / 31 / 34870 |
| atan | 436 / 68 / 83740 | 1116 / 258 / 188720 | 380 / 56 / 73680 | 406 / 58 / 77630 | 436 / 68 / 83740 |
| atan_fast_lut | 319 / 18 / 57290 | 304 / 60 / 85890 | 313 / 16 / 54940 | 313 / 16 / 54940 | 319 / 18 / 57290 |
| exp | 398 / 63 / 64870 | 1091 / 262 / 157820 | 326 / 48 / 53300 | 362 / 53 / 58830 | 398 / 63 / 64870 |
| floor | 16 / 4 / 1880 | 41 / 13 / 5110 | 16 / 4 / 1880 | 16 / 4 / 1880 | 16 / 4 / 1880 |
| round | 31 / 5 / 3570 | 55 / 14 / 7580 | 31 / 5 / 3570 | 31 / 5 / 3570 | 31 / 5 / 3570 |
| from_int | 4 / 1 / 470 | 26 / 9 / 3230 | 4 / 1 / 470 | 4 / 1 / 470 | 4 / 1 / 470 |
| to_int_u32 | 12 / 4 / 1480 | 16 / 6 / 2120 | 9 / 3 / 1110 | 9 / 3 / 1110 | 12 / 4 / 1480 |

Observations
* **Orion FP32x32 is literally `cubit::f64::Fixed`** (`use cubit::f64::Fixed as FP32x32`), FP64x64 is
  cubit f128; identical numbers, as expected. A side effect on the current compiler: importing
  `FP32x32Add`, `FP32x32Mul`, … next to the type is an `E2313` ambiguity with cubit's own impls, so
  the Orion impls had to be called by path in the benchmark.
* FP16x16 / FP8x23 are the same algorithms on `u32`: 3 steps / 1 RC cheaper on mul/div, otherwise identical.
* cubit f128 (Q64.64) is **5×** dearer on mul/div (u256 division), **3.5×** on trig.
* `sub` costs 25–27 steps although `add` costs 4–15: `sub` = `add(a, -b)`, and the gas (4 450 vs 1 640) suggests a real call rather than an inlined body.
* cubit f128 `sqrt` = `sqrt(mag) * 2^64 / 2^32`: only 32 fractional bits are meaningful.
* Transcendentals are dominated by the number of fixed-point multiplications: Taylor `sin` = 8
  loop iterations × (2 mul + 1 div) ≈ 1 050 steps.

### B. Representation experiments (steps / RC / l2_gas)

Representations (all in `packages/bench/src/reprs/`):
`sm64` = `{mag: u64, sign: bool}` with NonZero-constant divisors; `i64 naive` = `i64`, `i64_wide_mul`
→ `i128`, corelib signed `/`; `i64+BoundedInt` = `i64` storage, `bounded_int` arithmetic (floor
rounding for mul, truncation for div); `felt lazy` = `felt252`, raw field add/sub, one
`felt252 → u128` conversion per mul/lt/div/sqrt; `sm128` = `{mag: u128, sign}` Q64.64 with
`u128 wide_mul` + `BoundedInt` recombination and an exact `u256_sqrt`. cubit f64 / f128 run through
the same generic kernels for reference. “fused” = products accumulated before a single rescale.

| op | cubit f64 | sm64 | i64 naive | i64+BoundedInt | felt lazy | cubit f128 | sm128 |
|---|---:|---:|---:|---:|---:|---:|---:|
| add_same_sign | 4 / 1 / 470 | 4 / 1 / 470 | 6 / 2 / 740 | 6 / 2 / 740 | 0 / 0 / 0 | 19 / 1 / 2130 | 19 / 1 / 2130 |
| add_mixed_sign | 15 / 2 / 1640 | 15 / 2 / 1640 | 6 / 2 / 740 | 6 / 2 / 740 | 0 / 0 / 0 | 27 / 2 / 2960 | 27 / 2 / 2960 |
| sub | 27 / 1 / 4450 | 25 / 1 / 4250 | 6 / 2 / 740 | 6 / 2 / 740 | 0 / 0 / 0 | 26 / 1 / 4550 | 24 / 1 / 4250 |
| mul | 18 / 5 / 2250 | 18 / 5 / 2250 | 33 / 7 / 4010 | 16 / 5 / 1950 | 16 / 5 / 1950 | 91 / 24 / 11580 | 53 / 15 / 6570 |
| div | 18 / 5 / 2250 | 18 / 5 / 2250 | 88 / 20 / 12880 | 44 / 7 / 5730 | 61 / 11 / 7050 | 93 / 24 / 11880 | 93 / 24 / 11880 |
| neg | 5 / 0 / 500 | 2 / 0 / 200 | 3 / 0 / 300 | 3 / 0 / 300 | 1 / 0 / 100 | 5 / 0 / 500 | 2 / 0 / 200 |
| lt_mixed_sign | 0 / 0 / 0 | 0 / 0 / 0 | 6 / 1 / 670 | 6 / 1 / 670 | 11 / 2 / 1240 | 0 / 0 / 0 | 0 / 0 / 0 |
| lt_same_sign | 15 / 1 / 1580 | 15 / 1 / 1580 | 6 / 1 / 670 | 6 / 1 / 670 | 11 / 2 / 1240 | 15 / 1 / 1580 | 15 / 1 / 1580 |
| sqrt | 35 / 13 / 4410 | 10 / 4 / 1280 | 16 / 6 / 2020 | 17 / 6 / 2120 | 12 / 5 / 1550 | 74 / 21 / 8880 | 65 / 16 / 7740 |
| dot2 | 69 / 12 / 9270 | 69 / 12 / 9270 | 66 / 16 / 8750 | 51 / 12 / 6670 | 42 / 10 / 5020 | 214 / 50 / 27710 | 122 / 32 / 15850 |
| dot2_fused | – | – | – | 18 / 5 / 2150 | 18 / 5 / 2150 | – | – |
| cross2 | 71 / 11 / 9880 | 67 / 11 / 9470 | 66 / 16 / 8750 | 51 / 12 / 6670 | 42 / 10 / 5020 | 209 / 49 / 27810 | 122 / 31 / 16350 |
| cross2_fused | – | – | – | 18 / 5 / 2150 | 18 / 5 / 2150 | – | – |
| length2 | 109 / 24 / 14480 | 79 / 15 / 10840 | 78 / 22 / 10640 | 65 / 18 / 8660 | 58 / 15 / 7510 | 275 / 70 / 34780 | 174 / 47 / 21660 |
| length2_fused | – | – | – | 42 / 11 / 5090 | 38 / 10 / 4620 | – | – |
| length2_wide_sqrt | – | – | – | 17 / 6 / 2120 | – | – | – |
| normalize2 | 143 / 34 / 19560 | 114 / 25 / 16030 | 254 / 62 / 36230 | 151 / 32 / 19120 | 178 / 37 / 20850 | 468 / 118 / 59240 | 362 / 95 / 45620 |
| normalize2_fused | – | – | – | 131 / 25 / 16630 | – | – | – |
| normalize2_wide_sqrt | – | – | – | 112 / 20 / 14500 | – | – | – |
| normalize2_fused_rsqrt | – | – | – | 87 / 25 / 11480 | – | – | – |
| mat3_mul_vec3 | 344 / 55 / 47370 | 344 / 55 / 47370 | 270 / 75 / 34280 | 202 / 57 / 24920 | 177 / 45 / 20850 | 959 / 226 / 121890 | 545 / 145 / 68520 |
| mat3_mul_vec3_fused | – | – | – | 82 / 15 / 10180 | 82 / 15 / 9810 | – | – |

Reading guide
* add/sub/lt: `i64` = 6 steps flat. Sign-magnitude = 0–27 depending on the signs (and 2 felts).
  felt = 0 (no check at all).
* mul: `i64+BoundedInt` = felt lazy = 16 < sm64/cubit 18 < naive i128 route 33 (the corelib signed
  division branches on both signs and range-checks each branch).
* div: sign-magnitude 18 < `i64+BoundedInt` 44 < felt 61 < naive 88 < Q64.64 93.
* Unfused kernels track the scalar costs; **fused kernels are 2.5–4× cheaper again** and cost the
  same for `i64+BoundedInt` and lazy felt — i.e. the lazy-felt trick buys nothing that
  `BoundedInt` does not already give, and `BoundedInt` needs no manual soundness argument.
* `length2_wide_sqrt` (17 steps): `u128_sqrt(x² + y²)` with the squares kept at scale 2^64 gives the
  Q32.32 length directly — cheaper than even one multiplication + sqrt, exact, overflow-free.
* `normalize2`: 2 divisions dominate (112–151). Inverse-sqrt + 2 mul = 87 steps but see §3.4 for its precision caveat.

| case | steps | range_check | bitwise | l2_gas |
|---|---:|---:|---:|---:|
| mulshift_bounded_int | 12 | 4 | 0 | 1480 |
| mulshift_div_operator_literal | 15 | 5 | 0 | 1950 |
| mulshift_div_runtime_divisor | 15 | 5 | 0 | 1950 |
| mulshift_divrem_const_nonzero | 15 | 5 | 0 | 1950 |
| mulshift_felt | 17 | 6 | 0 | 2220 |
| u128_checked_mul | 25 | 9 | 0 | 3130 |
| u128_div_const | 13 | 4 | 0 | 1580 |
| u128_div_runtime | 12 | 4 | 0 | 1480 |
| u128_wide_mul_only | 24 | 9 | 0 | 3030 |
| u256_div_runtime | 57 | 15 | 0 | 7070 |
| u64_checked_mul | 4 | 1 | 0 | 470 |
| u64_div_const | 8 | 3 | 0 | 1010 |
| u64_div_runtime | 8 | 3 | 0 | 1010 |
| u64_wide_mul_only | 0 | 0 | 0 | 0 |

* `u64_wide_mul` is free (0 steps). A `u128` checked multiplication is **25 steps / 9 RC** — this is
  what makes cubit's `sqrt` (35) 3.5× dearer than necessary (10).
* **`DivRem` by a `NonZero` constant vs the generic `/` with a literal vs a run-time divisor: no
  difference** (15 / 15 / 15 for the mul-shift, 13 vs 12 for bare u128, 8 vs 8 for u64). The
  compiler already folds the literal into a `NonZero` const and the zero-check of a run-time
  divisor is ~free. So this is *not* an optimisation lever in 2.19.
* `BoundedInt` mul + `div_rem` + single downcast: 12 steps / 4 RC (−20 %) because the product and
  the quotient need no intermediate `u128` range check. Going through `felt252` is the worst (17 / 6).
* `u256` division: 57 steps — the reason every 64.64 design is slow.

### C. Square roots (unsigned Q32.32 magnitude, input 1234.5678)

| case | steps | range_check | bitwise | l2_gas |
|---|---:|---:|---:|---:|
| inv_sqrt_core_then_div | 26 | 9 | 0 | 3330 |
| inv_sqrt_div_then_core | 21 | 8 | 0 | 2760 |
| sqrt_core_u128_widemul | 10 | 4 | 0 | 1280 |
| sqrt_core_u64_lowprec | 14 | 5 | 0 | 1750 |
| sqrt_cubit_style_checked_mul | 35 | 13 | 0 | 4410 |
| sqrt_newton8_loop | 356 | 87 | 0 | 43510 |
| sqrt_newton8_unrolled | 231 | 78 | 0 | 31160 |

* core `u128_sqrt(mag << 32)` is exact (`floor`) and costs 10 steps. cubit's version returns the
  same value for 35 steps.
* `u64_sqrt(mag) << 16` is *dearer* (14) and only has 16 fractional bits: no reason to use it.
* Newton/Heron with 8 fixed iterations from `x0 = (x + 1)/2`: 231 (unrolled) – 356 (loop) steps, and
  8 iterations are **not enough** away from 1.0: measured `sqrt(1e-5)` → 14 691 217 vs exact
  13 581 930 (8 % off), `sqrt(1e6)` → 2.1× too large. A safe iteration count would be ~40
  (*estimate*: ≈ 1 100–1 800 steps). Loop overhead alone is +54 % over unrolled.
* Inverse sqrt: `u128_sqrt(2^96 / mag)` = 21 steps, `2^64 / sqrt` = 26 steps; both returned
  122 236 912 for an exact value of 122 236 917 (5 ulp ≈ 1.2e-9).

### D. Trigonometry

Cost at 0.7 rad (and 2.5 rad where shown):

| case | steps | range_check | bitwise | l2_gas |
|---|---:|---:|---:|---:|
| bhaskara_sin_0p7 | 70 | 15 | 0 | 9430 |
| cubit_atan_fast_lut_0p7 | 319 | 18 | 0 | 57290 |
| cubit_atan_poly_0p7 | 436 | 68 | 0 | 83740 |
| cubit_cos_fast_lut_0p7 | 263 | 31 | 0 | 34870 |
| cubit_cos_taylor_0p7 | 1084 | 199 | 0 | 134820 |
| cubit_sin_fast_lut_0p7 | 206 | 29 | 0 | 30320 |
| cubit_sin_fast_lut_2p5 | 221 | 30 | 0 | 30320 |
| cubit_sin_taylor_0p7 | 1051 | 197 | 0 | 130270 |
| cubit_sin_taylor_2p5 | 1051 | 197 | 0 | 130270 |
| cubit_tan_taylor_0p7 | 2169 | 401 | 0 | 268940 |
| poly_atan2_deg11 | 186 | 44 | 0 | 23110 |
| poly_cos7_0p7 | 155 | 38 | 0 | 19240 |
| poly_sin5_0p7 | 126 | 30 | 0 | 16250 |
| poly_sin7_0p7 | 144 | 35 | 0 | 18400 |
| poly_sin7_2p5 | 148 | 36 | 0 | 18400 |
| poly_sin9_0p7 | 162 | 40 | 0 | 20550 |

Accuracy over a sweep of 201 points on [−7, 7] (raw results printed by `tests/d_sweep.cairo`,
compared with libm by `scripts/sweep.py`):

| function | points | max abs error | mean abs error |
|---|---:|---:|---:|
| bhaskara_sin | 201 | 1.632e-03 | 8.446e-04 |
| cubit_atan | 201 | 1.620e-09 | 2.632e-10 |
| cubit_atan_fast | 201 | 3.970e-06 | 1.628e-06 |
| cubit_cos | 201 | 1.947e-08 | 8.915e-10 |
| cubit_cos_fast | 201 | 4.614e-06 | 2.086e-06 |
| cubit_sin | 201 | 1.646e-08 | 9.501e-10 |
| cubit_sin_fast | 201 | 4.622e-06 | 1.902e-06 |
| poly_atan | 201 | 1.663e-06 | 1.026e-06 |
| poly_cos7 | 201 | 5.916e-07 | 3.752e-07 |
| poly_sin5 | 201 | 6.768e-05 | 4.228e-05 |
| poly_sin7 | 201 | 5.896e-07 | 3.713e-07 |
| poly_sin9 | 201 | 7.699e-09 | 2.448e-09 |

* My polynomials (`packages/bench/src/trig.cairo`) reduce the angle with `DivRem` by π, parity of
  the quotient with `DivRem` by 2, reflect around π/2, then evaluate an odd minimax polynomial in
  Horner form (coefficients fitted offline, max theoretical errors 6.8e-5 / 5.9e-7 / 3.3e-9 for
  degree 5 / 7 / 9; the measured 7.7e-9 for degree 9 includes fixed-point rounding).
* **deg-9 polynomial: 6.5× cheaper than cubit's Taylor `sin` and 2× more accurate; 1.3× cheaper
  than cubit's LUT `sin_fast` and 600× more accurate** — without the 1 331-line LUT in the bytecode.
* Bhaskara I: 70 steps but 1.6e-3 error — only acceptable for cosmetic uses.
* `atan2` (1 division + degree-11 polynomial + octant fix-up): 186 steps, 1.7e-6. cubit `atan`: 436
  steps (1.6e-9), `atan_fast` 319 steps (4.0e-6); neither offers `atan2`.
* `cos(x) = sin(x + π/2)` costs one extra `i64` add (155 vs 144). A joint `sin_cos` sharing the
  range reduction would save ≈ 25 steps *(estimate, not implemented)*.

### E. Orion linear algebra vs struct-based `Mat3`

| case | steps | range_check | bitwise | l2_gas |
|---|---:|---:|---:|---:|
| orion_tensor_dot_3_fp16x16 | 294 | 24 | 0 | 35080 |
| orion_tensor_dot_3_fp32x32 | 303 | 27 | 0 | 36490 |
| orion_tensor_matmul_3x3_fp16x16 | 2957 | 313 | 0 | 344300 |
| orion_tensor_matmul_3x3_fp32x32 | 3038 | 340 | 0 | 356990 |
| orion_tensor_matvec_3x3_3_fp16x16 | 1262 | 117 | 0 | 142660 |
| orion_tensor_matvec_3x3_3_fp32x32 | 1289 | 126 | 0 | 146890 |
| struct_mat3_mul_cubit_f64 | 937 | 161 | 0 | 134250 |
| struct_mat3_mul_i64b | 574 | 171 | 0 | 70100 |
| struct_mat3_mul_i64b_fused | 214 | 45 | 0 | 25480 |
| struct_mat3_vec3_i64b | 202 | 57 | 0 | 24920 |
| struct_mat3_vec3_i64b_fused | 82 | 15 | 0 | 10180 |

(Tensor construction is in the baseline; only `matmul` is measured.)

* 3×3 · 3×3: Orion tensor = 3 038 steps; unrolled struct on the *same scalar* (cubit f64) = 937
  → the generic tensor machinery (shape spans, index arithmetic, array appends, loops) costs **3.2×**.
  Against the recommended scalar: 574 unfused (**5.3×**), 214 fused (**14.2×**, 14× in gas).
* 3×3 · 3: 1 289 vs 82 fused (**15.7×**). Dot of two 3-vectors: 303 vs ≈ 27 (**11×**; 27 = one fused row of the 82-step mat3·vec3, a 3-term dot was not measured on its own).
* Orion is an ONNX runtime; dynamic shapes are the wrong abstraction for fixed-size physics math.

### F. “math < bitwise < loops”

| case | steps | range_check | bitwise | l2_gas |
|---|---:|---:|---:|---:|
| low32_bitwise_and | 7 | 0 | 1 | 1183 |
| low32_math_divrem | 8 | 3 | 0 | 1010 |
| pack_bitwise_or | 9 | 1 | 1 | 1453 |
| pack_math_bounded_int | 1 | 0 | 0 | 100 |
| pack_math_checked | 8 | 2 | 0 | 940 |
| pack_math_felt | 6 | 2 | 0 | 740 |
| parity_bitwise_and | 11 | 0 | 1 | 1583 |
| parity_math_divrem | 12 | 3 | 0 | 1410 |
| shl13_loop_doubling | 211 | 27 | 0 | 22990 |
| shl13_math_core_pow | 127 | 23 | 0 | 14300 |
| shl13_math_table_mul | 16 | 2 | 0 | 1740 |
| split_bitwise_and | 19 | 3 | 2 | 3176 |
| split_loop_32_halvings | 1127 | 223 | 0 | 128310 |
| split_math_bounded_int | 8 | 3 | 0 | 1010 |
| split_math_divrem | 8 | 3 | 0 | 1010 |

* **math vs bitwise**: step counts are a tie (8 vs 7, 12 vs 11), but DivRem pays in range checks
  (3 RC ≈ 210 gas) while `&`/`|` pay one bitwise builtin (≈ 483 gas, derived: 1 183 = 7 × 100 + 483): l2_gas is 12–17 % higher
  for bitwise on single ops, 55 % for pack, and **3.1×** for the split (two ANDs + a division anyway, since Cairo
  has no shift operator — a “bitwise shift” is always arithmetic in disguise). The bitwise builtin
  is also a much heavier trace cell for the prover than a range check. Heuristic confirmed, with
  the nuance that it does not show in the raw step count.
* **loops**: 13× (shl by doubling vs table lookup + mul) to 140× (bit-by-bit split). Even corelib
  `Pow::pow(2, k)` (square-and-multiply recursion) is 8× the table lookup.
* New tier *below* “math”: **`BoundedInt` math**. Packing two `u32`: 1 step / 0 RC (vs 8 / 2 checked,
  9 / 1 + 1 bitwise for `|`), because the bounds are known statically and nothing needs checking.
  Ordering: `BoundedInt` ≤ plain integer math < bitwise builtin ≪ loops.

### G. Methodology controls and extra kernels

| case | steps | range_check | bitwise | l2_gas |
|---|---:|---:|---:|---:|
| dot6_fused_single_rescale | 26 | 5 | 0 | 2950 |
| dot6_naive_6mul_5add | 134 | 40 | 0 | 16320 |
| felt_storage_boundary_check | 6 | 2 | 0 | 740 |
| mul_blackboxed_inputs | 16 | 5 | 0 | 1950 |
| mul_literal_inputs_no_blackbox | 0 | 0 | 0 | 0 |
| mul_x4_chain_default_inlining | 72 | 20 | 0 | 8720 |
| mul_x4_chain_inline_always | 72 | 20 | 0 | 8720 |
| mul_x4_chain_inline_never | 108 | 20 | 0 | 15120 |

* Literal inputs → 0 steps: the whole fixed-point multiplication is constant-folded. Any benchmark
  without black-boxing is meaningless.
* `#[inline(never)]` on `mul`: 27 steps per multiplication instead of 18 in a chain (+50 %), +73 % in gas
  (call/return + implicits shuffling). Default inlining already inlines this small body; `#[inline(always)]`
  changes nothing here. In a chain a multiplication costs 18 rather than the isolated 16.
* 6-term accumulation (a 2D two-body constraint row `J·v`): **26 steps fused vs 134 naive (5.2×)**.
  The `BoundedInt` sum reaches 2^130 before the `div_rem`; the compiler accepts it.
* Normalising a lazy felt at a storage boundary: 6 steps / 2 RC.

## 3. Precision and range analysis for game physics

Units: 1.0 = 1 m, dt = 1/60 s.

| | Q8.23 (u32) | Q16.16 (u32) | **Q32.32 (i64)** | Q64.64 (u128) |
|---|---|---|---|---|
| Range | ±256 | ±32 768 | **±2.147e9** | ±9.2e18 |
| Resolution | 1.2e-7 | 1.5e-5 | **2.3e-10** | 5.4e-20 |
| Significant digits at 1.0 | 6.9 | 4.8 | **9.6** | 19.3 |
| dt = 1/60, relative error of the constant | 9.5e-7 | **2.4e-4** (raw 1092) | 3.7e-9 (raw 71 582 788) | < 2e-18 |
| Largest \|v\| whose squared length fits | 16 | 181 | **46 340** (no limit for `length` with the wide sqrt) | 3.0e9 |
| mul cost (steps, best measured) | 15 | 15 | **16** | 53 (own) / 91 (cubit) |
| mat3·vec3 (best measured) | – | – | **82** | 545 (own) / 959 (cubit) |
| Memory cells per scalar | 2 (sign-mag.) | 2 | **1** | 2 |

### 3.1 World size, velocities, squared lengths
* Q16.16 overflows `v·v` at 181 units and Q8.23 at 16: every broad-phase distance test, every
  normalisation and every kinetic-energy term is at risk. Disqualified for a general engine.
* Q32.32 holds ±2.1e9 m. The practical limit is the **squared length: 46 340 m** (checked:
  `x*x + y*y` with (30 000, 40 000) panics — test `check_length_naive_overflows`). Two mitigations,
  both measured: (a) the fused kernels only range-check the *final* result, so intermediate sums may
  exceed the range (2^127–2^130 headroom); (b) `length` via the wide sqrt never materialises the
  squared length (test `check_length_wide_no_overflow`: exact 50 000.0). Velocities up to 46 km/s
  can be squared. This is ample for game worlds; squared-distance comparisons beyond 46 km should
  compare wide values (`BoundedInt`/`u128`) instead of Q32.32.
### 3.2 dt, small impulses, accumulation
* `x += v·dt`: each multiplication truncates by < 2.3e-10 m. With floor rounding the bias is
  systematic: ≤ 2.3e-10 × 216 000 steps/hour ≈ **5e-5 m/hour** worst case. Round-to-nearest is free
  in the `BoundedInt` scheme (the added constant becomes `2^126 + 2^31`; *not measured separately —
  it only changes a constant*), giving an unbiased error (random-walk ≈ 1e-7 m/hour, *estimate*).
  In Q16.16 the same drift is 1.5e-5 × 216 000 ≈ 3.3 m/hour plus a 2.4e-4 relative error on dt itself.
* Solver impulses of 1e-6 N·s still have 12 significant bits (raw 4 295); contact slop (1e-3),
  Baumgarte/ERP factors (0.2), restitution thresholds are all comfortably representable.
  Q16.16 represents 1e-4 as raw 7 (14 % error).
* Both rounding modes are **deterministic**, which is what provable physics needs; floor (toward
  −∞) is not sign-symmetric (−a·b ≠ −(a·b) by 1 ulp), truncation/sign-magnitude is. If mirror
  symmetry of simulations matters, use round-half-away or document the asymmetry.
### 3.3 Inertia values
* Rapier stores *inverse* mass / inverse inertia (and their square roots in 3D). For a 10 g,
  1 cm disc I = 5e-7 → raw 2 147 (11 bits) but 1/I = 2e6 fits with 53 bits: fine. The problem
  is the other end: a 10 t, 10 m body has 1/I ≈ 2e-6 → raw 8 590 (13 bits, 1e-4 relative).
* Recommendation: keep Q32.32 as *the* scalar, but give inverse inertia (and similar very
  small/very large quantities) a **dedicated scale** (e.g. Q16.48) with its own `rescale` constant —
  in the `BoundedInt` scheme a mixed-scale multiply costs exactly the same 16 steps
  (*design estimate; the mechanism is the measured `div_rem` by a constant*). Q64.64 everywhere would
  cost 3.3–12× on every kernel to fix a problem confined to a handful of fields.
### 3.4 sqrt / inverse sqrt / trig precision
* `u128_sqrt(mag << 32)` is exact to the last bit for Q32.32.
* Inverse sqrt has 32 fractional bits *absolute*: for |v| = 1 000 the factor 1e-3 has only 22
  significant bits → normalised vectors are off by ~2e-7 relative. Use it for |v| ≲ 100 or
  renormalise; otherwise use the wide sqrt + 2 divisions (112 steps, exact to 1 ulp).
* Rotations: Rapier 2D keeps a unit complex (cos, sin). With the deg-9 polynomial (7.7e-9) a
  rotation composed every frame drifts from unit norm by ≤ 1e-8 per step *(estimate)*; renormalise
  periodically with the 17-step wide length. Prefer incremental rotation updates
  (`rot += ω·dt·perp(rot)` + renormalise) that need no trig at all; keep trig for user-facing
  `from_angle`/`angle()`.

## 4. Sign handling trade-offs

| | sign-magnitude `{mag, sign}` (cubit, Orion) | native `i64` (+ `BoundedInt`) | lazy `felt252` |
|---|---|---|---|
| add / sub | 4–27 steps, **data dependent**, branchy | 6 steps flat | 0 |
| lt | 0–15, data dependent | 6 flat | 11 |
| neg / abs | 2–5 / 0 | 3 / abs not measured (a `constrain` + negate, *estimate* ≈ 10) | 1 / abs not measured |
| mul | 18 (sign = XOR, free) | 16 (offset trick) | 16 |
| div | **18** | 44 (abs/sign split) | 61 |
| memory / calldata / storage | **2 felts** per scalar (a `Vec3` = 6 cells, `Mat3` = 18) | 1 felt | 1 felt (must be normalised first: +6 steps) |
| canonical form | **No: negative zero.** Measured on cubit: `0 * -1` → `{mag: 0, sign: true}`, which is `!= ZERO` and `< ZERO` (test `check_cubit_negative_zero`). A contact solver comparing against zero will misbehave. | unique | unique mod P, but the *type* carries no bound |
| rounding | truncation toward zero (symmetric) | floor for mul (or nearest, free), truncation for div | floor |
| overflow detection | per op | per op on add; **once per fused kernel** on products | only at mul/lt/div/sqrt/check |
| cost predictability | varies with the signs of the data | constant | constant |

**Soundness of the lazy-felt representation** (asked for explicitly). The value x is stored as
`x mod P`. add/sub/neg are field operations with no check. The representation stays faithful as
long as no intermediate true integer reaches P/2 ≈ 2^250. Rule that guarantees it: every value is
a signed sum of at most 2^30 *leaves*, a leaf being either a checked input (|x| < 2^63) or the
output of mul/div/sqrt (|x| < 2^96 by construction: the quotient of a u128 by 2^32). Then |x| < 2^126,
any product is < 2^252 < P/2, and the single `felt252 → u128` conversion of `p + 2^127` performed by
mul (and the analogous one in lt/div/sqrt) **is** the overflow check: it panics when |p| ≥ 2^127
instead of wrapping. Checks are therefore needed (1) at every mul/div/lt/sqrt (built in),
(2) when a value is stored, serialised, used as a key or handed to non-lazy code (`check`: 6 steps),
(3) in any loop that could accumulate more than 2^30 leaves or *double* a value repeatedly
(`x = x + x` multiplies the leaf count by 2 each time — 30 iterations break the invariant).
The catch is that (3) is a whole-program property the type system does not see. `BoundedInt`
gives the same fused costs (tables B, G) with the bound **proved per expression at compile
time**, so lazy felt is only recommended *inside* small audited kernels whose magnitudes are
statically obvious (the Horner loop of `trig.cairo` is exactly that).

## 5. Recommendation

### 5.1 Scalar type (shared with the glam / nalgebra ports)

```cairo
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Fixed { pub v: i64 }   // Q32.32, two's-complement-free native signed integer
```

* **add / sub / neg / lt / eq**: native `i64` ops (6 / 6 / 3 / 6 steps, constant time, overflow-checked).
* **mul**: `bounded_int::mul(i64, i64)` → add `2^126` (+`2^31` for round-to-nearest) →
  `bounded_int::div_rem` by `NonZero<UnitInt<2^32>>` → subtract `2^94` → `downcast` to `i64`
  (the only range check, = overflow check). Reference implementation:
  `packages/bench/src/reprs/i64b.cairo` (`wide`, `rescale1`).
* **Fused multiply-accumulate as the primary API of the vector layer**: `wide(a, b) -> Prod`,
  typed sums `Prod2 … Prod6` (extend as needed: 9 for 3×3 determinants, 12 for 3D constraint
  rows), one `rescaleN` each. glam/nalgebra ports should implement `dot`, `cross`/`perp_dot`,
  `Mat2/Mat3 · Vec`, `Mat · Mat`, `length_squared`, quaternion products and the solver's `J·v`
  rows on these, never as `a*b + c*d` on the scalar. Measured gains: 2.8× (dot2), 2.5× (mat3·vec3
  vs unfused i64), 4.2× vs cubit f64, 5.2× (6-term row).
* **div**: abs/sign split with `bounded_int::constrain::<i64, 0>` + `div_rem` on non-negative
  `BoundedInt`s (44 steps, `I64bDiv`). Design the engine around stored inverses (inverse mass,
  inverse inertia, `inv_dt`) so that division stays out of hot loops.
* **sqrt**: `constrain` ≥ 0 → `× 2^32` as `BoundedInt` → `upcast` to `u128` → core `u128_sqrt` → `i64` (17 steps).
  **length**: `u128_sqrt` of the wide sum of squares (17 steps for 2D). **normalize**: wide length +
  divisions (112 steps 2D), or inverse sqrt `u128_sqrt(2^96/mag)` + multiplications (87 steps) where
  22+ significant bits suffice.
* **trig**: arithmetic range reduction + odd minimax polynomial, degree 9 for `sin`/`cos` (162
  steps, 7.7e-9), degree 7 when 6e-7 is enough (144 steps); `atan2` = 1 division + degree-11 polynomial
  (186 steps, 1.7e-6; raise the degree or add cubit's √3/3 argument shift if more precision is needed).
  Coefficients are in `packages/bench/src/trig.cairo`. No LUTs.
* **floor / round / from-int / to-int**: `DivRem`/`bounded_int::div_rem` by `2^32`; `from_int` is a
  `BoundedInt` multiply by the constant `2^32` + downcast *(pattern measured in F: pack = 1 step + a downcast)*.
* Rounding: choose round-to-nearest for mul (free) unless bit-exact parity with a reference
  implementation using truncation is required. Document it; it is part of the determinism contract.
* Special scales: allow a second type (e.g. Q16.48) for inverse inertia-like quantities; same machinery.

### 5.2 Reuse vs rewrite

| Source | Verdict |
|---|---|
| cubit core ops (`add/sub/mul/div/lt/sqrt`) | **Rewrite.** Sign-magnitude, 2 felts, negative-zero bug, checked-u128-mul sqrt, non-inlined `sub`. |
| cubit trig / `lut.cairo` | **Rewrite** (polynomials are cheaper *and* more accurate). Keep cubit's `atan` range-reduction idea (invert above 1, shift above 0.7) if higher `atan` accuracy is wanted. |
| cubit `exp/ln/pow/hyp` | Not needed by Rapier's core. If ever needed, port the algorithms (they are fine) onto the new scalar; cost is ~400 steps for `exp`. |
| cubit tests | **Reuse as test vectors** (after fixing the `a < b == false` syntax). |
| cubit `Vec2/3/4`, origami `Vec2<T>` | Rewrite on fused kernels. |
| Orion fixed point | Nothing to reuse: it is cubit (FP32x32/FP64x64 aliases; FP16x16/FP8x23 narrower copies) and the narrow formats are unusable for physics (§3). |
| Orion tensors / matrix | **Do not use**: 3–16× overhead on 3×3 sizes, dynamic shapes, `Span` allocation per op. |
| alexandria | Nothing relevant (no fixed point; loop-based generic `dot`). |
| corelib | **Reuse heavily**: `i64`, `i64_wide_mul`, `u128_sqrt`/`u256_sqrt`, `DivRem`, and above all `core::internal::bounded_int` (`mul/add/sub/div_rem/constrain/downcast/upcast`). Note it lives under `core::internal` (not a stability promise) — isolate it in one module. |

## 6. Pitfalls found

1. **Constant folding**: literal inputs make a fixed-point multiplication cost 0 steps (table G).
   Always black-box inputs (`#[inline(never)]` identity) and sink outputs.
2. **Baseline**: an empty snforge test is 65–116 steps / ≥ 14 320 gas; a single op is 5–20 steps. Always
   subtract a baseline with the same inputs; expect ±3 steps of layout noise.
3. **`l2_gas` is clamped to 40 000 with `--tracked-resource cairo-steps`**, and steps/builtins are
   not printed with `sierra-gas`: two runs are required.
4. **Compiler crash**: cairo-lang-lowering 2.19.4 panics in its incremental cache
   (`cache/mod.rs:437: called Option::unwrap() on a None value`) when compiling the snforge test
   target of this workspace. Workaround: `[profile.dev.cairo] incremental = false` in the workspace
   manifest (or `SCARB_INCREMENTAL=false`).
5. **`snforge test` without `-p`** also builds test targets for every vendored workspace member.
6. **Old-edition libraries**: visibility is not enforced in `2023_10`, so copying cubit/Orion into
   a package that keeps that edition is far less work than porting them to `2024_07`.
7. **Language drift breaking cubit/Orion/alexandria**: consecutive comparison operators (E1028);
   `AddEq`→`AddAssign` bridge causing duplicate impls (E2313); corelib now implementing `Div` for
   signed ints and cubit now implementing `Into<i8, Fixed>` (E2313 against Orion's own impls);
   `SpanSerde` negative impl; stricter desnap and drop checking; `hash::`/`integer::` root paths
   gone; `nullable_from_box`, `u32_wrapping_add` removed.
8. **Type aliases + operator impls**: because Orion's FP32x32 *is* cubit's `Fixed`, importing both
   libraries' operator impls makes `a + b` ambiguous.
9. **Inlining**: forcing `#[inline(never)]` on a small scalar op costs +50 % steps / +73 % gas per
   call; cubit's `sub` pays this today (27 steps for an op that should cost ≤ 15). Small ops of the
   new scalar should be `#[inline(always)]`; large ones (trig) should not, to keep bytecode size in check.
10. **Negative zero** in sign-magnitude libraries (measured, §4).
11. **Checked `u128` multiplication is expensive** (25 steps / 9 RC); `u64_wide_mul`/`i64_wide_mul` and
    `bounded_int::mul` are free. Never scale by multiplying `u128`s when a widening or bounded multiply exists.
12. **`NonZero` constants are not an optimisation** in 2.19: `/ literal`, `DivRem` by a const
    `NonZero` and even a run-time divisor cost the same (±1 step).
13. **No shift operators** in corelib: “bitwise” code still divides; bitwise builtin gas ≈ 7× a range check.
14. **Fixed-iteration Newton sqrt is a trap**: slow *and* wrong outside a narrow input band unless the
    iteration count is sized for the full range.
15. **`BoundedInt` ergonomics**: every distinct range needs a helper impl (`MulHelper`, `AddHelper`,
    `DivRemHelper`, …) with exact bounds — the compiler rejects wrong ones, which is the point, but
    it is boilerplate (generated by a Python snippet here; a small macro/codegen is advisable).
    Some corelib items are `pub(crate)` (`IsZeroResult`), so NonZero-ness of a `BoundedInt` divisor
    must be obtained via `TryInto<i64, NonZero<i64>>` + `constrain::<NonZero<i64>, 0>`.
    `bounded_int::div_rem` accepted dividends up to 2^130 here; its upper limit was not explored.
16. `use some::function;` / `use some::Trait;` inside a function body is rejected
    (`E2075 Unsupported use item in statement`) — import at module level.
