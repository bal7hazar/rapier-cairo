# 03 — Cairo ecosystem benchmark: architecture, efficiency idioms, engineering practices

Scope: what `rapier.cairo` should copy (and avoid) from three reference repositories, plus
what the current toolchain (Scarb 2.19.4 / snforge 0.61.0) actually offers for gas tracking.

| Repo | Commit studied | Role in this benchmark |
|---|---|---|
| `keep-starknet-strange/alexandria` | `6d2cfcc` (2026-03-05) | workspace architecture, CI, gas report, docs |
| `dojoengine/origami` | `1ddafcb` (2025-09-23) | code-efficiency idioms, API/trait style |
| `keep-starknet-strange/starknet-agentic` | `c7e1c9e` (2026-09-14) | Cairo authoring/optimisation/testing skills, agent workflow |

All paths below are relative to each repo root. Local experiments were run in a scratch
package with `scarb 2.19.4` + `starknet-foundry 0.61.0` (section 3.4); numbers quoted from
those runs are measured, not estimated.

---

## 1. Repository / workspace architecture

### 1.1 alexandria

- **Root `Scarb.toml`** is a *virtual-ish* workspace (it has `name`/`version` keys under
  `[workspace]`, but no root package). 17 members under `packages/<short_name>`; package
  names are prefixed: `alexandria_math`, `alexandria_linalg`, `alexandria_numeric`, ...
- Shared config:
  ```toml
  [workspace.dependencies]
  starknet = "2.16.0"
  cairo_test = "2.16.0"
  snforge_std = "0.56.0"

  [workspace.tool.fmt]
  sort-module-level-items = true

  [workspace.package]
  version = "0.10.0"

  [profile.coverage]
  sierra = true

  [scripts]
  all = "scarb build && snforge test"
  ```
- Per-package `Scarb.toml` (`packages/math/Scarb.toml`): `version.workspace = true`,
  `edition = "2023_11"` (they have not migrated to `2024_07`), `cairo-version = "2.16.0"`, `[tool] fmt.workspace = true`,
  `[dev-dependencies] snforge_std.workspace = true`, and
  `experimental-features = ["user_defined_inline_macros"]` where declarative macros are used
  (`packages/math/src/pow_macro.cairo`).
- Each package ships `README.md`, `Scarb.toml`, `snfoundry.toml`
  (`[snforge.default] sierra = true, casm = true`), `src/`, `tests/`.
- **Toolchain pin**: `.tool-versions` = `scarb 2.16.0`, `starknet-foundry 0.56.0`.
- **`lib.cairo`**: flat list of `pub mod x;` with a `//!` crate doc header. `alexandria_math`
  also defines generic helpers directly in `lib.cairo` (`pow`, `BitShift`, `BitRotate`,
  `WrappingMath`) — this makes `lib.cairo` a 250-line grab bag; not a pattern to copy.
- **Tests**: *integration-style only*. One file per source module in
  `packages/<pkg>/tests/<module>_test.cairo`, no `tests/lib.cairo`, so snforge compiles each
  package's `tests/` as `alexandria_<pkg>_integrationtest::<file>::<test>`. Almost no inline
  `#[cfg(test)]`. Consequence: only the `pub` API is testable; private helpers are not.
  A stale `src/tests.cairo` (list of `mod x_test;`) is still present but unreferenced.
- A proc-macro package lives in the same workspace (`packages/macros`, Rust/Cargo +
  `Scarb.toml`) with a separate `macros_tests` Cairo package and its own CI workflow. It
  provides `#[derive(Add, Sub, Mul, Div, AddAssign..., Zero)]` and `pow!` — relevant if we
  want derive-based vector operators, but it requires a Rust toolchain for consumers' builds.

### 1.2 origami

- **Root `Scarb.toml`**:
  ```toml
  [workspace]
  members = ["crates/contracts", "crates/algebra", "crates/defi", "crates/map",
             "crates/random", "crates/rating", "crates/security"]

  [workspace.package]
  version = "1.1.2"
  edition = "2024_07"

  [workspace.dependencies]
  cubit = { git = "https://github.com/bengineer42/cubit", branch = "bump-cairo-gt-2.8" }
  starknet = "^2.12.2"
  cairo_test = "^2.12.2"
  ```
  (`crates/contracts` is listed but no longer exists; a leftover `crates/Scarb.toml`
  references `dojo.workspace` that is not defined — the repo is in light-maintenance mode.)
- Packages: `origami_<name>` in `crates/<name>`, all with `version.workspace = true`,
  `edition.workspace = true`, a `description` and a `homepage` pointing at the crate dir.
  Only real dependency is `cubit` (fixed point) for `algebra` and `defi`; `map`, `random`,
  `rating`, `security` are dependency-free.
- **Toolchain pin**: `.tool-versions` = `scarb 2.12.2` only. **Test runner is `cairo-test`**
  (`scarb test`), not snforge.
- **`lib.cairo`**: nested inline module tree, grouping by role
  (`crates/map/src/lib.cairo`):
  ```cairo
  pub mod hex;
  pub mod map;
  pub mod types { pub mod direction; pub mod node; }
  pub mod finders { pub mod astar; pub mod bfs; ... }
  pub mod generators { pub mod caver; ... }
  pub mod helpers {
      pub mod asserter; pub mod bitmap; pub mod heap; pub mod power;
      #[cfg(target: "test")]
      pub mod printer;
      pub mod seeder;
  }
  ```
  Note `#[cfg(target: "test")]` to compile debug-only modules out of the library.
- **Tests**: 100 % inline `#[cfg(test)] mod tests { ... }` at the bottom of each source file;
  no `tests/` dir. Private functions are testable; every file is self-contained.
- README lists "Physics (WIP)" as a crate — it never shipped. There is no physics code in
  the tree. `origami_algebra` (see 4.7) is the only physics-adjacent package.

### 1.3 starknet-agentic

Not a Cairo library: a pnpm monorepo (TS packages, website, skills) with four *independent*
Scarb packages under `contracts/` (no Scarb workspace). Each pins deps literally
(`contracts/agent-account/Scarb.toml`):

```toml
edition = "2024_07"
[dependencies]
starknet = "2.14.0"
[dev-dependencies]
snforge_std = "0.54.1"
assert_macros = "2.14.0"
[scripts]
test = "snforge test"
[tool.scarb]
allow-prebuilt-plugins = ["snforge_std"]
```

Tests are in `tests/` (integration, dispatcher-based) because everything is a contract.
Its value for us is the guidance content (section 6), not its layout.

---

## 2. CI/CD and contributor tooling

### 2.1 alexandria (`.github/workflows/`)

- `test.yml` (on push + PR), four jobs: `build` (`scarb build`) → `test` (`snforge test`,
  via `software-mansion/setup-scarb@v1.3.2` which reads `.tool-versions`, and
  `foundry-rs/setup-snfoundry@v3` with explicit `starknet-foundry-version: '0.56.0'`),
  `check-format` (`scarb fmt --check`), and **`gas-report`**
  (`./scripts/generate_gas_report.sh`, see 3.1). No caching, no per-package matrix, no
  `scarb lint`.
- `macros.yml`: path-filtered workflow for the Rust proc-macro crate (cargo build, clippy
  with `-Dwarnings`, `cargo fmt --check`, `scarb fmt --check`).
- `mdbook.yml`: manual (`workflow_dispatch`) docs deploy to GitHub Pages.
  `scripts/generate_doc.sh` runs
  `scarb doc --workspace --exclude alexandria_macros --remote-base-url <repo>`, injects
  `docs/intro.md` into the generated `SUMMARY.md`, strips the `core` section, copies
  `docs/book.toml`, and runs `mdbook build target/doc`.
- **Publishing**: `scripts/update_registry.sh` loops `scarb publish --package <name>` over a
  hand-ordered package list (dependency order). Manual, not in CI. No changelog file;
  version is bumped in `[workspace.package]` and stated in the README
  ("Current version is 0.10.0 compatible with Cairo 2.16.0").
- Housekeeping: PR template (type-of-change checkboxes + breaking-change section), three
  issue templates, `CODEOWNERS`, `labels.yml`, `stale.yml`, `lock.yml`,
  `.all-contributorsrc`. `docs/CONTRIBUTING.md` is an unfinished template ("2. TODO").
- No pre-commit hooks.

### 2.2 origami (`.github/workflows/`)

- `ci.yml`: `SCARB_VERSION: 2.12.2` env, jobs `check` (`scarb fmt --check`) → `build`
  (`scarb build`) → **one hand-written job per package**
  (`scarb test --package origami_algebra`, ...). Parallelism by copy-paste instead of a
  matrix; PR template even has a checkbox "Add a dedicated CI job for new examples".
- `release.yaml`: on `v*` tag, creates a GitHub release via curl. No registry publish.
- No gas tracking, no docs generation, no changelog, no contributor doc.

### 2.3 starknet-agentic

- `ci.yml`: a `detect-changes` job (`dorny/paths-filter`, outputs such as
  `cairo-erc8004`, `cairo-agent-account`) gates per-contract jobs; all actions are
  **SHA-pinned** (`software-mansion/setup-scarb@2a96b74... # v1.6.2`,
  `scarb-version: "2.14.0"`), and a final aggregate job `needs:` everything so branch
  protection has one required check.
- Extra workflows: CodeQL, OSSF scorecard, dependency-review, dependabot automerge,
  secret scanning (`.githooks/pre-commit` runs `scripts/secret_scan.sh`, `.gitleaks.toml`),
  changesets (`.changeset/`), `CHANGELOG.md`, `VERSIONING.md` (pre-1.0 SemVer policy:
  PATCH = fixes/internal, MINOR = any externally visible change), `CONTRIBUTING.md`
  (small PRs, acceptance test mandatory, conventional commits), AI reviewers config
  (`.coderabbit.yaml`, `greptile.json`, `.pr_agent.toml`).

---

## 3. Gas tracking

### 3.1 alexandria — the only repo with a regression mechanism

- Committed snapshot: **`gas_report.json`** at the repo root, 1 427 entries, a flat JSON map
  `"<crate>_integrationtest::<file>::<test>": <l2_gas>`, sorted by gas descending.
- `scripts/generate_gas_report.sh` runs `snforge test [-p alexandria_<pkg>]`, extracts with
  ```bash
  grep -E '\[PASS\] .* \(l1_gas: ~[0-9]+, l1_data_gas: ~[0-9]+, l2_gas: ~[0-9]+\)' \
    | sed -E 's/\[PASS\] ([^(]*) \(l1_gas: ~[0-9]+, l1_data_gas: ~[0-9]+, l2_gas: ~([0-9]+)\)/\1 \2/'
  ```
  compares with the previous `gas_report.json`, prints `INCREASE:`/`DECREASE:` lines,
  overwrites `gas_report.json` and writes `gas_report_diff.json` (git-ignored).
- Weaknesses to avoid:
  1. **It never fails** — both branches end in `exit 0`, so the CI job is informational.
  2. It overwrites the snapshot during the check, so CI compares against the committed
     file but nothing forces contributors to commit the refreshed file.
  3. Uses BSD-only `sed -i ''` — breaks on the Ubuntu runner it runs on.
  4. Sorted by gas value, so any change reshuffles lines and produces noisy diffs.
  5. No `#[available_gas]` anywhere (0 occurrences); tests measure "whatever the test does",
     with constant inputs (risk of constant folding, see 3.4).

### 3.2 origami

No gas tracking at all: 0 `#[available_gas]`, runner is `cairo-test`, no snapshot, no
benchmarks. Efficiency is achieved by idiom/discipline only (section 4).

### 3.3 starknet-agentic

- `skills/cairo-testing/references/legacy-full.md` "Gas Reporting": only
  `snforge test --detailed-resources > gas-report.txt` and "diff manually".
- `skills/cairo-optimization/` is the serious part: profile-driven workflow built on
  `snforge test --save-trace-data` → `cairo-profiler` → `pprof`
  (`scripts/profile.py profile --mode snforge --package P --test T --metric steps|rc|sierra-gas|l2-gas`),
  outputs `profiles/<ts>_<pkg>_<name>_<metric>_<commit>.{pb.gz,png,summary.txt}`.
  Regressions are locked via regex-based eval cases
  (`evals/cases/contract_skill_benchmark.jsonl`, e.g. `must_match: DivRem::div_rem\(amount, 2\)`),
  i.e. *pattern* regression, not *gas number* regression.

### 3.4 What the toolchain offers today (verified locally, scarb 2.19.4 / snforge 0.61.0)

- **`scarb cairo-test` is deprecated**: running it prints
  `warn: scarb cairo-test is deprecated and will be removed in a future version. help: please migrate to snforge`.
  It still supports `--print-resource-usage` (prints `gas usage est.`, `steps`,
  `memory holes`, `builtins` per test), and `#[available_gas(N)]`. Do not build on it.
- **`snforge test`** prints for every passing test
  `[PASS] pkg::mod::test (l1_gas: ~0, l1_data_gas: ~0, l2_gas: ~13620)`. For a pure library
  `l2_gas` equals Sierra gas exactly (`--detailed-resources` shows `sierra gas: 13620`).
  Useful flags: `--detailed-resources`, `--tracked-resource cairo-steps|sierra-gas`
  (also `[tool.snforge] tracked_resource` in Scarb.toml), `--save-trace-data`,
  `--build-profile` (cairo-profiler), `--coverage` (cairo-coverage), `--max-n-steps`,
  `--partition I/N` and `--max-threads` (CI sharding), `-p/--package`, `-w/--workspace`,
  `--exact`, `--skip`, `--fuzzer-runs/--fuzzer-seed`, `--features`.
  - `--gas-report` is **contract-only** ("No contract gas usage data to display, no
    contract calls made") — useless for a pure library.
  - `snforge optimize-inlining --contracts ...` searches the `inlining-strategy` threshold —
    contract-only as well, but the underlying `[cairo] inlining-strategy` knob applies to us.
  - There is **no built-in gas snapshot** (no `forge snapshot` equivalent) and no JSON
    output; text parsing is the only option.
- With `--tracked-resource cairo-steps` the detailed view gives `steps`, `memory holes`
  and per-builtin counts (`range_check`, `bitwise`), but `l2_gas` becomes coarse
  (rounded: 40 000 for all small tests). **Use `sierra-gas` for the snapshot, `cairo-steps`
  for diagnosis** (it is the only view that shows *which* builtin got more expensive).
- `#[available_gas(l2_gas: N)]` works as a hard per-test budget:
  `[FAIL] ... Test cost exceeded the available gas. Consumed l2_gas: ~107410`.
- Fuzz tests print a distribution instead of a number
  (`l2_gas: {max: ~306593, min: ~293273, mean: ~303679, std deviation: ~3507}`) —
  exclude them from snapshots.
- `core::internal::bounded_int` is importable directly in 2.19.4 (no `corelib_imports`
  dependency needed) but is gated: `warn[E2065]: Usage of unstable feature
  "bounded-int-utils"` unless the `use` carries `#[feature("bounded-int-utils")]`.
- `scarb lint` and `scarb doc` both ship with Scarb 2.19.4.

**Measured micro-benchmark** (extract bits 32..63 of a `u128`; `bench_baseline` is the same
test without the call; all values Sierra gas):

| Variant | Code | Test total | Net of baseline |
|---|---|---:|---:|
| baseline | `opaque(x)` only | 15 540 | 0 |
| BoundedInt | 2 × `bounded_int::div_rem` by `UnitInt<2^32>` | 17 460 | **1 920** |
| math | 2 × `DivRem::div_rem(x, NZ_2_POW_32)` | 18 400 | 2 860 |
| bitwise | `(x / 2^32) & 0xffffffff` | 18 503 | 2 963 |
| loop | 32 × `q / 2`, `while i != 32` | 109 490 | 93 950 |
| loop | same with `while i < 32` | 124 230 | 108 690 (+15.7 %) |

Two methodological lessons from this experiment:

1. **Constant folding silently invalidates benchmarks.** With a `const` input the "math"
   variant cost 13 620 for the whole test — *less than the baseline* — because the compiler
   evaluated it at compile time (`cairo-test` reported `steps: 4`). Benchmark inputs must
   pass through an `#[inline(never)] fn opaque<T>(x: T) -> T` (or come from a fuzzer/arg).
2. **Always subtract a baseline.** ~15.5k of every number above is test harness +
   `assert_eq!`. Relative differences between variants (2 860 vs 2 963) are invisible
   without a baseline test in the same module.

The owner's ordering (arithmetic < bitwise < loop) holds; BoundedInt is cheaper still
(no overflow/range re-checks), and `!=` loop conditions beat `<` by a wide margin.

### 3.5 Recommended gas-regression mechanism for rapier.cairo

The worktree already contains the right skeleton (`.gas-snapshot`, `scripts/gas.py`,
`[workspace.tool.snforge] tracked_resource = "sierra-gas"`, `scarb run gas` /
`scarb run gas-check`). Keep it, and harden it with the following rules:

1. **Snapshot file**: `.gas-snapshot` at the root, one line per test
   `<fully::qualified::test> <sierra_gas>`, **sorted by name** (stable diffs), generated only
   by the script. Fuzz tests and `#[ignore]` tests are excluded (regex only matches the
   single-number `[PASS]` form).
2. **Two commands**: `snapshot` (rewrite file) and `check` (run tests, diff against file,
   print `name: old -> new (+x.xx%)`, **exit 1 on any difference** — increase, decrease,
   added or removed test). Decreases also fail so the snapshot is always refreshed in the
   same PR as the code change, and the diff of `.gas-snapshot` is the reviewable gas report.
   A prototype of this script was validated against snforge 0.61.0 output (it caught the
   `!=` → `<` change above as `+13.46%`).
3. **CI**: `gas-check` is a required job. Optionally post the diff as a PR comment.
4. **Benchmark tests convention** (what makes numbers comparable):
   - name prefix `gas_` (or module `bench`), one per public function *and per candidate
     implementation*: `gas_mul_math`, `gas_mul_bitwise`, `gas_mul_bounded`;
   - one `gas_baseline` test per module with the identical harness minus the call;
   - inputs always via `opaque()` from a shared `rapier_testing` dev-crate;
   - one call per test (or a fixed N-iteration loop for very cheap ops, documented);
   - optional `#[available_gas(l2_gas: N)]` budget on hot-path ops (solver step, SAT test)
     as a hard ceiling independent of the snapshot.
5. **Keeping losing alternatives**: put rejected implementations under
   `#[cfg(test)] mod alternatives` (or a `bench` feature) next to the winner, so the
   snapshot permanently documents *why* the chosen variant won and re-evaluates them on
   every compiler upgrade (the ranking can flip between Cairo versions).
6. **Diagnosis, not gating**: `snforge test <name> --detailed-resources --tracked-resource
   cairo-steps` for builtin breakdown; `--build-profile` + `cairo-profiler view` for
   hotspots in composite functions (the agentic `profile.py` flow).
7. Toolchain bumps are their own PR: regenerate `.gas-snapshot`, review the delta alone.

---

## 4. Code-efficiency idioms (origami, with alexandria where relevant)

### 4.1 Shifts and masks as mul / div / mod by powers of two

origami never uses a shift helper; packing is pure arithmetic on a precomputed power:

```cairo
// origami crates/map/src/helpers/bitmap.cairo
fn get(x: felt252, index: u8) -> u8 {
    let x: u256 = x.into();
    let offset: u256 = TwoPower::pow(index);
    (x / offset % 2).try_into().unwrap()
}
fn set(x: felt252, index: u8) -> felt252 {
    let x: u256 = x.into();
    let offset: u256 = TwoPower::pow(index);
    let bit = x / offset % 2;
    let offset: u256 = offset * (1 - bit);      // branchless: add 2^i only if bit is 0
    (x + offset).try_into().unwrap()
}
```

Branch-free `set`/`unset` via `offset * (1 - bit)` / `offset * bit` replaces `|` and `& !`.
Packed queues are consumed with `%` and `/=`:

```cairo
// origami crates/map/src/types/direction.cairo
pub const DIRECTION_SIZE: u32 = 0x10;
fn pop_front(ref directions: u32) -> Direction {
    let direciton: u8 = (directions % DIRECTION_SIZE).try_into().unwrap();
    directions /= DIRECTION_SIZE;
    direciton.into()
}
```

SWAR popcount keeps `&` only where masks are irreducible and replaces every shift by a
division (`crates/map/src/helpers/bitmap.cairo`):

```cairo
fn _popcount(mut x: u32) -> u8 {
    x -= ((x / 2) & 0x55555555);
    x = (x & 0x33333333) + ((x / 4) & 0x33333333);
    x = (x + (x / 16)) & 0x0f0f0f0f;
    x += (x / 256);
    x += (x / 65536);
    return (x % 64).try_into().unwrap();
}
```

Room for improvement we should exploit: origami does `x / offset % 2` as two separate
operations on **`u256`** (the most expensive integer type) and never uses `DivRem`. For
rapier: keep packed words ≤ `u128` where possible, and use one
`DivRem::div_rem(x, NZ_CONST)` with a `NonZero` *constant* divisor (no runtime zero check):
`const NZ_TWO_POW_32: NonZero<u128> = 0x100000000;` compiles and was used in 3.4.

alexandria's equivalent (`packages/math/src/lib.cairo`) is the slow generic version —
`fn shr(x, n) { x / pow(2_u8.into(), n) }` with a recursive generic `pow` — and its
"optimised" module replaces it with tables (4.2).

### 4.2 Lookup tables: const fixed-size arrays and `match`

Both repos converge on `const TABLE: [T; N]` + `.span()` indexing:

```cairo
// origami crates/map/src/helpers/power.cairo
const TWO_POWER: [u256; 256] = [0x1, 0x2, 0x4, ...];
fn pow(exp: u8) -> u256 { *TWO_POWER.span().at(exp.into()) }

// alexandria packages/math/src/opt_math.cairo
const SHIFT_TABLE128: [u128; 128] = [0x1, 0x2, ...];
#[inline(always)]
pub fn shr128(a: u8, b: u128) -> u128 { b / *SHIFT_TABLE128.span()[a.into()] }
#[inline(always)]
pub fn shl128(a: u8, b: u128) -> u128 { overflowing_mul128(b, *SHIFT_TABLE128.span()[a.into()]) }
```

The file header of `opt_math.cairo` states the trade-off explicitly: "Runtime optimized math
utils (n steps / gas). Might increase contract size". `alexandria_math::const_pow` does the
same for `pow2`, `pow2_felt252`, `pow10` (tables declared *inside* the function body).
alexandria's trig (`packages/math/src/trigonometry.cairo`) is also table-driven
(`sin_table: [u64; 10]` at 10° steps + `cos_table` for 0–9°, angle-addition formula,
1e8 scaling) — precision is poor, but the structure (quadrant reduction by `%` and
subtraction, then table + one interpolation) is the right shape for a physics `sin/cos`.

`match` on a small integer is used for non-numeric tables — e.g. the 24 permutations of 4
directions packed as nibbles (`0 => 0x2468, 1 => 0x2486, ...`,
`crates/map/src/types/direction.cairo`) and enum ↔ integer conversions. Agentic rule 3
recommends `match` for small pow2 tables too. Rule of thumb for rapier: `match` for ≤ ~16
dense cases or enum conversion, `const [T; N]` + `span()` beyond that — and benchmark both
for each table, since this is exactly the math-vs-lookup comparison the gas tests exist for.

### 4.3 Loop avoidance by unrolling

origami unrolls every fixed-count iteration by hand. A* evaluates its 4 neighbours with the
block repeated 4 times (`crates/map/src/finders/astar.cairo`), the cave generator has one
`if` per neighbour direction (`count_direct_floor` / `count_indirect_floor` in
`generators/caver.cairo`), and `least_significant_bit` is an 8-step unrolled binary search:

```cairo
if (x & 0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF) > 0 { r -= 128; } else { x /= 0x100000000000000000000000000000000; }
if (x & 0xFFFFFFFFFFFFFFFF) > 0 { r -= 64; } else { x /= 0x10000000000000000; }
...
```

Where a loop is unavoidable: countdown with `while index != 0` (`caver.cairo`),
`while let Option::Some(x) = heap.pop_front()` (`astar.cairo`), and
`span.pop_front()` iteration instead of indexing (`algebra/src/vector.cairo`,
alexandria `linalg/src/dot.cairo`). For rapier: `Vec2/Vec3/Mat2/Mat3` ops must be fully
unrolled struct-field arithmetic — never `Span`-based like `origami_algebra::vector` or
`alexandria_linalg::dot` (section 4.7).

### 4.4 `#[inline]` usage

- origami `map`: plain `#[inline]` (hint) on practically every method, including large ones
  (`Astar::search`). `random`/`algebra`: `#[inline(always)]` on constructors and one-liners.
  Noted limitation in `algebra/src/vec2.cairo`:
  `// #[inline(always)] is not allowed for functions with impl generic parameters.`
- alexandria: 260 inline attributes, systematically `#[inline(always)]` on thin wrappers
  (`OptWrapping`, `shl*/shr*`, `I257Impl::new`).
- Guidance for rapier: `#[inline(always)]` on leaf arithmetic (fixed-point add/mul, vector
  ops — call overhead dominates there); leave big functions to the compiler; evaluate
  `[cairo] inlining-strategy` once there is a representative benchmark; and note from the
  agentic skill that panicking paths inside inlined/unrolled code bloat Sierra
  quadratically (4.6).

### 4.5 `Felt252Dict`, `Span`, snapshots

- `Felt252Dict` is origami's only mutable random-access structure: the binary heap
  (`crates/map/src/helpers/heap.cairo`) stores items in `Felt252Dict<Nullable<T>>` and —
  explicitly "to save gas" — folds two logical maps into **one** dict with a key offset:
  ```cairo
  const KEY_OFFSET: felt252 = 252;
  self.keys.insert(index.into(), key);                 // index -> key
  self.keys.insert(key.into() + KEY_OFFSET, index);    // key   -> index
  ```
  `Deck` (`crates/random/src/deck.cairo`) uses the sparse Fisher–Yates trick: only swapped
  positions are stored, `0` means "identity", so an N-card deck costs O(draws) not O(N).
  Structs containing dicts derive `Destruct` (not `Drop`) and are passed `ref self`.
  Relevance: broad-phase grids, contact caches and island/union-find structures in rapier
  will be dict-backed; every dict access has a squash cost at destruction, so merge maps
  and keep keys small/dense.
- `Span<T>` for read-only inputs and outputs (`Astar::search(...) -> Span<u8>`,
  `fn remove(ref self: Deck, cards: Span<u8>)`); `Array` only as a local builder.
- Small `Copy` structs are passed **by value** (`self: Map`, `self: Node`, `lhs: Node`),
  `@` snapshots are used only for non-Copy owners (`fn is_empty(self: @Heap<T>)`) and where
  core traits force it (`PartialEq::eq(lhs: @Node, rhs: @Node)`). For rapier's `Fixed`,
  `Vec2`, `Rot`, `Isometry`, `Aabb`: derive `Copy, Drop` and pass by value.
- State that fits in one felt is kept in one felt: the whole map grid is a `felt252` bitmap
  (≤ 252 cells) and `Map` is `{ width: u8, height: u8, grid: felt252, seed: felt252 }`.

### 4.6 Integer types, BoundedInt, panics

- Neither origami nor alexandria use `BoundedInt`/`downcast`/`upcast` (0 occurrences).
  origami over-relies on `u256` intermediates (`seed.into() % 24_u256`,
  `random % self.remaining.into()`); that is a known inefficiency, not a pattern.
- alexandria's `opt_math.cairo` shows the `WideMul` idiom for wrapping multiplication
  without overflow panic: `x.wide_mul(y).low` (u128) or
  `(x.wide_mul(y) & 0xffffffff).try_into().unwrap()` (u32), and `lib.cairo` builds
  rotate-left from one `wide_mul` + one `DivRem::div_rem(word, 0x100_u16.try_into().unwrap())`
  then `quotient + remainder`. `WideMul` (u64×u64→u128, u128×u128→u256) is exactly what a
  Q-format fixed-point multiply needs.
- The BoundedInt playbook comes from starknet-agentic
  (`skills/cairo-optimization/references/legacy-full.md`, by feltroidprime/garaga):
  - limb assembly `4×u32 → u128`: 28 340 gas with `u128` arithmetic vs 13 840 with
    `bounded_int::{mul, add}` + `upcast`;
  - **"BoundedInt in, BoundedInt out"**: `downcast` costs a range check (4 steps) and
    erases the gain if done at every call; `upcast` is free; convert only at system
    boundaries;
  - `felt252 → u128` via `u128s_from_felt252` (2 steps) rather than `try_into` (4 steps);
  - `try_into().unwrap()` in unrolled hot code causes O(N²) Sierra bloat (each panic site
    drops all live variables) — use non-panicking `match` or types that make the
    conversion infallible;
  - negative dividends: add `SHIFT = ceil(|min| / m) * m` before `bounded_int_div_rem`;
  - bounds are capped at 2^128; helper impls (`AddHelper`, `MulHelper`, `DivRemHelper`)
    must be declared per type pair — the skill ships `scripts/bounded_int_calc.py` to
    generate them and forbids computing bounds by hand.
  Our own measurement (3.4) confirms the gain on a simple limb extraction (1 920 vs 2 860).
  A signed fixed-point `BoundedInt<-2^63, 2^63-1>`-style core type is worth prototyping
  against `{ mag: u64, sign: bool }` (cubit style) and native `i64`/`i128`.
- `felt252` arithmetic: origami uses felts as opaque containers (bitmaps, seeds, dict
  keys) and does the math in integers. Raw felt add/mul is the cheapest operation in Cairo
  (no range check) but has no ordering/division; legitimate uses for rapier are packing
  (`a + b * 2^k` when bounds are already proven), hashing and dict keys — not dynamics.

### 4.7 origami packages relevant to a physics engine

- **`origami_algebra`** (`crates/algebra/src/{vec2,vector,matrix}.cairo`) is what is left
  of the cubit-derived algebra. `Vec2<T>` is a generic struct ported from `glam`
  (`new`, `splat`, `select(mask: Vec2<bool>, ...)`, `dot`, `dot_into_vec`, swizzles
  `xx/xy/yx/yy`), tested with `cubit::f128::types::fixed::{FixedTrait, ONE_u128}`
  (sign-magnitude `Fixed { mag: u128, sign: bool }`, 64.64). There is no `Add/Sub/Mul` for
  `Vec2`, no length/normalize/cross/rotation, no `Vec3`, and — telling — the modules are
  declared **private** in `lib.cairo` (`mod matrix; mod vec2; mod vector;` without `pub`),
  so the package exports nothing under edition 2024_07. `Vector<T>`/`Matrix<T>` are
  `Span<T>`-backed with loops, `array![]` rebuilding on every add, recursive cofactor
  determinant — unusable for a hot path. Conclusion: **nothing to reuse; use only as a
  naming reference (`Vec2Trait::new/splat/dot`)**. rapier needs its own `rapier_math`.
- `cubit` itself is consumed from a personal fork branch
  (`bengineer42/cubit`, `bump-cairo-gt-2.8`) — a supply-chain smell. Do not depend on it;
  the fixed-point type is core IP for this project and must be benchmark-driven.
- **`origami_map`**: bitmap-in-a-felt grid, dict-backed binary heap, unrolled neighbour
  visits → direct inspiration for broad-phase (uniform grid / sweep-and-prune cells packed
  in felts), priority structures, and island traversal.
- **`origami_random`**: deterministic Poseidon-based PRNG `hash(seed, nonce)` with `ref self`
  nonce bump — the pattern for any deterministic tie-breaking/jitter we need.
- **`origami_rating`** (`crates/rating/src/elo.cairo`): uses core
  `core::num::traits::Sqrt` chained (`u256 → u128 → u64 → u32 → u16`) for a 16th root, and
  a generic `round_div`. `Sqrt` is a native libfunc-backed integer sqrt — the baseline any
  custom fixed-point `sqrt` (Newton, as in alexandria `fast_root.cairo`) must beat.

---

## 5. API / trait design conventions

| Topic | origami | alexandria |
|---|---|---|
| Trait/impl naming | `FooTrait` + `FooImpl` (`MapTrait`/`MapImpl`); for stateless helper namespaces the impl takes the bare noun so call sites read `Bitmap::get(..)`, `Astar::search(..)`, `TwoPower::pow(..)`, `Seeder::shuffle(..)` | `I257Impl of I257Trait`; many free functions (`fast_sqrt`, `dot`, `pow2`) |
| `#[generate_trait]` | default for everything non-generic; explicit trait only when generic or when docs must live on the trait (`HeapTrait<T>`, `DeckTrait`) | used on struct impls (`I257Impl`), generic `Bitmap<T, ...>` |
| Private helpers | second impl in same file: `#[generate_trait] impl Private of PrivateTrait { fn _popcount(..) }` (not `pub`) | module-private free fns |
| Operators | core traits implemented by hand: `NodePartialEq of PartialEq<Node>`, `NodePartialOrd of PartialOrd<Node>`, `VectorAdd of Add<Vector<T>>`; conversions via `Into` (`DirectionIntoFelt252`, `DirectionFromU8 of Into<u8, Direction>`) | hand-written `Add/Sub/Mul/Div/Rem` + `*Assign`, `Zero`, `One`, `Default`, `Display` on `i257`; derive macros `#[derive(Add, Sub, Mul, Div, Zero)]` from `alexandria_macros` |
| Generic bounds | anonymous `+Trait<T>` impls: `impl HeapImpl<T, +ItemTrait<T>, +PartialOrd<T>, +Copy<T>, +Drop<T>>` | same; very long bound lists on numeric generics (`fast_power` has 9) |
| Errors | `pub mod errors { pub const NO_CARDS_LEFT: felt252 = 'Deck: no cards left'; }` per file, prefixed `'<Type>: <reason>'`, used with `assert(cond, errors::X)`; `Option` for expected absence (`Heap::get`, `pop_front`) and a panicking twin (`Heap::at`); dedicated `Asserter` impl with `assert_*` functions | mix of `assert!(.., "fast_power: invalid input")` (ByteArray — more expensive panic data) and `.expect('felt msg')`; no `Result` |
| Derives | `#[derive(Copy, Drop)]` (+`Serde` when public data), `Destruct` for dict owners | `#[derive(Serde, Copy, Drop, Hash)]` |
| Doc style | `//!` module header, `///` with `# Arguments` / `# Returns` / `# Panics` / `# Effects`; section comments `// Core imports` / `// Internal imports` / `// Constants`; inline step tags `// [Check]`, `// [Effect]`, `// [Compute]`, `// [Return]` | `//!` crate + module headers; `///` with `#### Arguments` / `#### Returns` / `#### Panics` (h4 because it renders better in `scarb doc`/mdBook) |
| Tests | inline `#[cfg(test)] mod tests`, `test_<type>_<behaviour>`, `assert_eq!` | `tests/<mod>_test.cairo`, `assert!(x == y)` |
| README | root only | root (table of packages) + one README per package listing modules with links |
| Consts | `SCREAMING_CASE`, typed, hex for masks/powers | same; some lowercase consts (`sin_table`) — avoid |

Take-aways for rapier: origami's conventions are the more coherent set (namespaced impls,
felt error constants, step-tag comments, inline tests). Borrow from alexandria the `//!`
crate headers, per-package READMEs, `Zero/One/Default` + full operator-trait coverage on
numeric types, and `scarb doc`-friendly doc headings. Prefer short-string felt panics over
`assert!` with ByteArray formatting in hot paths (format machinery is pulled into the
Sierra of every call site).

---

## 6. starknet-agentic: guidance worth adopting

### 6.1 Cairo optimisation rules (`skills/cairo-optimization/`)

Twelve rules (`references/legacy-full.md`), with our verdict for a physics library:

| # | Rule | Adopt? |
|---|---|---|
| 1 | `DivRem::div_rem` instead of separate `/` and `%` | yes — and with `NonZero` const divisors |
| 2 | `while i != n` instead of `i < n` (exact-trip loops only) | yes (measured +15.7 % for `<`) |
| 3 | No `pow()` for 2^k — lookup table (`match`) | yes (origami/alexandria use const arrays) |
| 4 | Iterate with `pop_front` / `for` / `multi_pop_front::<N>()`, never `at(i)` | yes |
| 5 | Cache `.len()` outside loops | yes |
| 6 | `span.slice()` instead of copy loops | yes |
| 7 | Parity/halving via `div_rem(x, 2)`, not `& 1` | yes ("bitwise AND is more expensive than div_rem") |
| 8 | Smallest integer type (`u128` over `u256`) | yes — strongly; design Q-format so products fit `u128`/`WideMul` |
| 9 | `StorePacking` for storage | n/a for the lib; relevant to the example contracts/Dojo models |
| 10 | BoundedInt for limb split/assembly and modular arithmetic | prototype in `rapier_math`, gated by gas tests |
| 11 | `hades_permutation(x, y, 2)` for 2-input Poseidon | yes where hashing pairs (body-pair keys) |
| 12 | `u128s_from_felt252` + `upcast` rather than `downcast`/`try_into` | yes for unpacking felts |

Process rules ("Security-Critical Rules" in `SKILL.md`): optimise only after tests pass;
**one optimisation class per commit**; re-profile after every change; never keep a change
that reduces readability without a measured gain; never hand-compute BoundedInt bounds
(use the calculator script); "Rationalizations to Reject" list (e.g. "We can skip
re-profiling — the change is obviously better").

### 6.2 Testing rules (`skills/cairo-testing/`)

Applicable subset: inline `#[cfg(test)]` for unit tests, `tests/` for integration;
`test_<function>_<scenario>` naming; shared helpers module; every public function gets a
success test plus a negative test per rejection condition;
`#[should_panic(expected: '...')]` always with the expected message, never bare;
fuzzing with a **fixed seed** (`#[fuzzer(runs: 256, seed: 12345)]`) and bounded inputs via
a custom `Fuzzable` impl (`snforge_std::fuzzable::{Fuzzable, generate_arg}`) instead of
discarding invalid inputs; turn every bug into a failing-before/fixed-after regression test.
For rapier, fuzzing is the natural way to do differential testing between alternative
implementations (`fuzz_mul_math_eq_mul_bounded`) and against reference vectors generated
from Rust Rapier.

### 6.3 Security/auditor material (`skills/cairo-auditor/`)

Mostly contract-centric (access control, upgrades, sessions). The math-relevant vectors
(`references/attack-vectors/attack-vectors-3.md`, `-4.md`): rounding bias that
systematically favours one side (#43), underflow guarded only by assumption (#45),
division-before-multiplication precision loss (#53), multiply-chain overflow before divide
(#108), felt252 range violations on packing/bit ops (#145), unbounded user-controlled
iteration (#80), truncating narrow conversions (#170). For a deterministic physics engine
these translate to: document rounding direction of every fixed-point op, multiply before
divide with a wide intermediate, prove packing bounds, cap iteration counts (solver
iterations, contact counts) by construction.

### 6.4 Agent-driven development conventions

From `AGENTS.md` / `CLAUDE.md`:

- **`AGENTS.md` is canonical; `CLAUDE.md` is a thin adapter** ("Canonical behavioral
  instructions live in AGENTS.md"). `CLAUDE.md` carries *context* in XML-ish sections:
  `<identity>`, `<stack>` (pinned versions table), `<structure>` (annotated tree),
  `<commands>` (task → command → working dir), `<conventions>`, `<workflows>` (step lists
  such as "Adding a new Cairo contract"), `<boundaries>` (**DO NOT modify** /
  **Require human review** / **Safe for agents**), `<references>` (path → use when),
  `<implementation_status>` (component → status → location), `<troubleshooting>`.
- **Roles**: Coordinator (scope, plan, sequencing, status accuracy — *not* large
  implementation), Executors per area, Reviewer (correctness, regressions, release gates —
  *not* initial implementation).
- **Task lifecycle** `todo → inprogress → inreview → done` (+ `blocked`); acceptance checks
  are defined by the coordinator *before* implementation.
- **Parallelisation rules**: safe = disjoint directories with stable interfaces; must
  serialise = any shared-interface change, `scripts/**`, CI workflows, concurrent edits to
  the same directory. Conflict protocol: detect overlap early, land the interface change
  first with tests, rebase dependents.
- **Required validation per change type** (for us: `scarb fmt --check`, `scarb build`,
  `snforge test`, gas check) and an **escalation template** (Blocker / Options /
  Recommendation).
- **Skill pipeline with explicit handoffs**: `authoring → testing → optimization →
  auditor`; each skill has "When to use / When NOT to use", a 4-turn orchestration
  (Understand → Plan ≤ 30 lines and wait → Implement → Verify), error codes with recovery,
  and eval cases that lock the rules. (The referenced `skills/references/skill-handoff.md`
  does not exist in the tree — a reminder to keep cross-references tested.)
- Skills are packaged as `skills/<name>/{SKILL.md, references/, scripts/, workflows/}` with
  YAML frontmatter and mirrored for Codex via `.agents/skills/` symlinks and for Claude via
  `.claude-plugin/plugin.json`; slash-command aliases live in `commands/`.

---

## 7. Comparison and blueprint

### 7.1 Comparison table

| Dimension | alexandria | origami | starknet-agentic |
|---|---|---|---|
| Layout | workspace, `packages/<n>` → `alexandria_<n>` (17) | workspace, `crates/<n>` → `origami_<n>` (6) | standalone packages under `contracts/` |
| Shared config | `[workspace.dependencies]`, `[workspace.package] version`, `[workspace.tool.fmt]` | `[workspace.package] version+edition`, `[workspace.dependencies]` | none |
| Edition | `2023_11` | `2024_07` | `2024_07` |
| Toolchain | scarb 2.16.0, snforge 0.56.0 (`.tool-versions`) | scarb 2.12.2 | scarb 2.14.0, snforge 0.54.1 (CI only) |
| Test runner | snforge | cairo-test (now deprecated) | snforge |
| Test layout | `tests/<mod>_test.cairo` only | inline `#[cfg(test)]` only | `tests/` integration |
| Fuzzing | none | none | recommended in skill (fixed seed) |
| CI | build → test / fmt → gas-report; no cache, no matrix | fmt → build → one job per package | path-filtered, SHA-pinned actions, aggregate gate, CodeQL/scorecard/secret-scan |
| Gas tracking | `gas_report.json` + script; **non-blocking** | none | profiling skill (cairo-profiler); manual diff |
| Docs | `scarb doc` → mdBook → Pages; README per package | root README only | extensive docs/, skills, llms.txt |
| Release | manual `scarb publish` loop script | GitHub release on tag | changesets, CHANGELOG, VERSIONING.md |
| Efficiency idioms | lookup tables, `WideMul`, heavy `#[inline(always)]`; but slow generic defaults | arithmetic bit-packing, const tables, unrolling, dict merging; but `u256`-heavy, no `DivRem` | the rule set: DivRem, BoundedInt, `!=` loops, no `pow`, no bitwise parity |
| API style | free functions + some traits, `assert!` ByteArray errors | `FooTrait/FooImpl`, `#[generate_trait]`, `errors` module with felt consts | OZ-component contracts |
| Agent guidance | none | none | AGENTS.md canonical + CLAUDE.md context + skills |

### 7.2 Blueprint for rapier.cairo

**Workspace layout** (origami-style `crates/`, already started in the worktree):

```
rapier.cairo/
├── Scarb.toml              # [workspace] members = ["crates/*"]; shared package/deps/tool config
├── .tool-versions          # scarb 2.19.4 / starknet-foundry 0.61.0
├── .gas-snapshot           # generated, sorted by test name, committed
├── scripts/gas.py          # snapshot | check (exit 1 on any diff)
├── AGENTS.md  CLAUDE.md  CONTRIBUTING.md  CHANGELOG.md  README.md
├── docs/{research/, adr/}  # research reports, architecture decision records
├── .github/{workflows/ci.yml, PULL_REQUEST_TEMPLATE.md}
└── crates/
    ├── rapier_testing/     # dev-only helpers: opaque(), approx-eq asserts, Fuzzable impls, fixtures
    ├── rapier_math/        # fixed-point scalar, Vec2/Vec3, Rot, Isometry, Mat, trig/sqrt tables
    ├── rapier_geometry/    # (parry) shapes, AABB, queries, contact manifolds, broad phase
    ├── rapier_dynamics/    # bodies, integration, joints, constraint solver, islands
    ├── rapier_pipeline/    # PhysicsPipeline step, world container, events
    └── rapier2d/ (rapier3d/)  # thin facade re-exporting a dimension-specific API (later)
```

Rules: package name == directory name, `rapier_` prefix (scarbs.xyz names are global),
strict DAG `math ← geometry ← dynamics ← pipeline`; no `starknet` dependency in the core
crates (pure Cairo, so they are usable from contracts, Dojo and `scarb execute`/provers);
every crate has `README.md`, `//!` header in `lib.cairo`, and
`version/edition/license/repository.workspace = true`. `lib.cairo` contains only `pub mod`
declarations (nested by role like `origami_map`), plus a curated `pub use` prelude —
no logic.

**Toolchain**: `scarb 2.19.4`, `starknet-foundry 0.61.0`, edition `2024_07`,
`cairo-version = "2.19.4"`, snforge only (cairo-test is deprecated),
`[workspace.tool.snforge] tracked_resource = "sierra-gas"`,
`[workspace.tool.fmt] sort-module-level-items = true, max-line-length = 100`,
`allow-prebuilt-plugins = ["snforge_std"]`. `#[feature("bounded-int-utils")]` on
`core::internal::bounded_int` imports, confined to `rapier_math` internals.

**CI jobs** (single `ci.yml`, SHA-pinned actions, `setup-scarb` + `setup-snfoundry` reading
`.tool-versions`, Scarb cache enabled, `concurrency` cancel-in-progress):
1. `fmt` — `scarb fmt --check`
2. `lint` — `scarb lint --workspace` (deny warnings once clean)
3. `build` — `scarb build --workspace`
4. `test` — `snforge test --workspace` (use `--partition i/N` matrix when it gets slow —
   better than origami's per-package copy-paste)
5. `gas` — `python3 scripts/gas.py check` (**required**, fails on any diff; uploads the diff)
6. `docs` (main only) — `scarb doc --workspace` → mdBook → Pages (alexandria script as base)
7. `release` (tag `v*`) — GitHub release + ordered `scarb publish -p <crate>` loop
8. `ci-ok` aggregate job as the single required status.

**Test layout**: inline `#[cfg(test)] mod tests` at the bottom of every source file
(origami style; private helpers testable, agents touch one file per feature);
`crates/<c>/tests/` only for cross-module scenarios (e.g. "box stack settles") and
reference-vector comparisons against Rust Rapier. Three test families by name prefix:
`test_*` (behaviour), `fuzz_*` (fixed-seed properties / differential equivalence between
implementations), `gas_*` (benchmarks feeding `.gas-snapshot`, with `gas_baseline` per
module and `opaque()` inputs).

**Gas snapshot mechanism**: as specified in 3.5.

**Coding conventions checklist**
- [ ] Cost order: felt/int arithmetic < `DivRem` < bitwise builtin < loop. Every non-trivial
      function lands with `gas_*` tests for each candidate; losers kept under
      `#[cfg(test)] mod alternatives`.
- [ ] Shifts/masks = `*`, `DivRem::div_rem` by `NonZero` **constants**; never `/` and `%` on
      the same operands; never `pow()` at runtime — `const [T; N]` table or `match`.
- [ ] Smallest integer type; no `u256` in hot paths; `WideMul` for fixed-point products.
- [ ] BoundedInt end-to-end inside `rapier_math` where it wins; `upcast` free, `downcast`
      only at API boundaries; bounds generated by script.
- [ ] Fixed-size math fully unrolled (struct fields, no `Span`/`Array` for Vec/Mat);
      unavoidable loops: `while i != n`, cached `len`, `pop_front`/`for`/`multi_pop_front`.
- [ ] Value types `#[derive(Copy, Drop, Serde, PartialEq, Debug)]`, passed by value; `@`
      only for non-Copy owners; dict owners derive `Destruct`, take `ref self`, merge maps
      with key offsets.
- [ ] `#[inline(always)]` on leaf arithmetic only; avoid `unwrap()`/panics inside
      inlined/unrolled hot code (Sierra bloat) — prefer infallible types.
- [ ] Naming: `FooTrait`/`FooImpl` via `#[generate_trait]`; stateless namespaces use the
      bare noun (`Sat::test(..)`); private impl `Private of PrivateTrait`.
- [ ] Operators through core traits (`Add, Sub, Mul, Div, Neg, PartialEq, PartialOrd`,
      `AddAssign`…), plus `Zero`, `One`, `Default`, `Into/TryInto`.
- [ ] Errors: `pub mod errors { pub const X: felt252 = 'Type: reason'; }`, `assert(c, errors::X)`;
      `Option` for expected absence with a panicking twin only when ergonomic; no ByteArray
      `assert!`/`panic!` in library code paths.
- [ ] Docs: `//!` per module, `///` with `# Arguments / # Returns / # Panics`; rounding
      direction and value range documented on every fixed-point op; step tags
      `// [Check] / [Compute] / [Effect] / [Return]`.
- [ ] Determinism: no dependence on dict iteration order, explicit rounding, bounded
      iteration counts.
- [ ] One optimisation class per commit, snapshot diff in the same commit.

**AGENTS.md outline** (canonical, behavioural)
1. Mission and scope (port of Rapier to provable Cairo; gas is a first-class requirement)
2. Operating principles (small testable diffs; measure, don't guess; single source of truth)
3. Roles: Orchestrator (plans, defines acceptance checks, owns `Scarb.toml`/CI/scripts/
   shared interfaces, merges), Executor sub-agents (one crate/module each), Reviewer
4. Task lifecycle + task brief template (goal, files owned, interfaces frozen, acceptance:
   tests + gas budget, out of scope)
5. Feature pipeline: author → test (`test_`, `fuzz_`) → benchmark alternatives (`gas_`) →
   pick winner → snapshot → review
6. Parallelisation rules: parallel across crates/modules with frozen interfaces; serialise
   changes to `rapier_math` public types, `Scarb.toml`, `.gas-snapshot` regeneration
   (orchestrator regenerates once after merging executors' work), CI, scripts
7. Required validation: `scarb fmt --check && scarb build && snforge test --workspace &&
   scarb run gas-check`
8. Definition of done (tests, fuzz equivalence, gas tests + snapshot, docs, README entry)
9. Escalation template; rationalisations to reject ("obviously faster", "tests later")

**CLAUDE.md outline** (context adapter, points to AGENTS.md)
`<identity>`, `<stack>` (pinned versions), `<structure>` (crate tree + dependency DAG),
`<commands>` (build / test one crate / test one name / gas snapshot / gas check / detailed
resources / profile / doc), `<conventions>` (the checklist above, condensed),
`<workflows>` ("add a math primitive", "port a Rapier module", "compare implementations",
"bump toolchain"), `<boundaries>` (do not hand-edit `.gas-snapshot`/`Scarb.lock`; human
review for toolchain bumps, public API changes, new dependencies), `<references>`
(`docs/research/*`, upstream rapier/parry paths), `<implementation_status>` table,
`<troubleshooting>` (const-folded benchmarks, `bounded-int-utils` warning, snforge plugin
prebuilt, asdf versions). Optionally vendor the `cairo-optimization` and `cairo-testing`
skills (frontmatter `license: Apache-2.0`, upstream feltroidprime/cairo-skills; repo is MIT) under `.claude/skills/`, trimmed of contract-specific content, and add
a project skill `gas-benchmark` encoding section 3.5.
