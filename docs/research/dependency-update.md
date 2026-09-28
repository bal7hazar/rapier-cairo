# DU1 — dependency update: `fixed` 0.4.0 / `glam` 0.4.1, and `glam_core` instead of the `glam` facade

Programme go 2026-09-28, measurement lot: **rapier-cairo depends on `fixed ^0.3.0` / `glam ^0.3.0`
while the current registry generation is `fixed 0.4.0` / `glam 0.4.1`.** This report measures two
independent options against `main` at `v0.1.0-alpha.7`; it does not decide. Branch
`feat/du1-dependency-update`; every command below ran crate-scoped, through
`scripts/build-shims/` (the project lock), one crate at a time.

## 1. What changed upstream between 0.3.0 and the current release

`fixed` and `glam` were split out of `glam-cairo` into their own repositories (`fixed-cairo`,
`glam-cairo`) on the way to 0.4.0; package names and the registry are unchanged. Between 0.3.0 and
now:

- `fixed` 0.4.0 (2026-09-25): pure addition, `ExpTrait::{sinh, cosh, tanh, sinhc, coshc}`; "no
  numeric result of 0.3.0 changes" (upstream CHANGELOG).
- `glam` 0.4.0 (2026-09-25): repository move, depends on `fixed` 0.4.0; "no numeric result of
  `glam` changes, every gas snapshot is identical" (upstream CHANGELOG).
- `glam` 0.4.1 (2026-09-28, PK-G / package-size gate): `glam` is cut into `glam_core` (float
  vectors/matrices, `Quat`, affines, Euler, camera, `BVec*`, and the integer vector *types* with
  their operator/conversion impls), `glam_int` (integer vector methods and constants),
  `glam_swizzles` (float swizzles), `glam_int_swizzles` (integer swizzles); `glam` becomes a
  facade re-exporting all four under the 0.4.0 paths. Upstream CHANGELOG: "non-breaking … no
  numeric result changes, every gas snapshot is identical".

Nothing between 0.3.0 and 0.4.1 changes a numeric result reachable from rapier-cairo's usage
(`fixed::{+,-,*,/,sqrt}`, `glam::Vec2`); the division rounding change (0.3.0, already the floor for
rapier's current pin) is untouched.

## 2. (a) Move the workspace to `fixed 0.4.0` / `glam 0.4.1`

`Scarb.toml` `[workspace.dependencies]`: `fixed = "0.4.0"`, `glam = "0.4.1"`, `Scarb.lock`
regenerated. **No source line needed an API move** (`scarb build -p <crate>` clean on every
touched crate: `rapier_math`, `rapier_core`, `rapier_geometry2d`, `rapier_dynamics2d`, `rapier2d`,
`rapier2d_classes`, `rapier_sink`). Only fixup needed: `crates/rapier2d_classes/tests/hashes.cairo`
pins the ten declared classes' `ClassHashSHA` constants; declaring the same source against a new
`glam`/`fixed` checksum changes the class hash (routes through the library, not through source), so
`test_pinned_class_hashes` re-pinned the ten constants to their newly declared values (mechanical,
no behaviour change).

**Results, every probe named by the brief, before / after, in exact Cairo steps
(`--tracked-resource cairo-steps --detailed-resources`) and Sierra gas
(`--tracked-resource sierra-gas`): bit-identical, test by test.**

| probe set | tests | steps (before → after) |
|---|--:|---|
| `rapier2d steps_*` (`game_path`, `ccd_budget`, `composite_budget`, `compound_budget`, `level_budget`, `sleep_budget`) | 53 | 60,121,051 → 60,121,051 |
| `rapier2d ccd` | 51 | 31,689,785 → 31,689,785 |
| `rapier_geometry2d golden` | 127 | 53,612,057 → 53,612,057 |
| `rapier_dynamics2d golden` | 24 | 30,770,856 → 30,770,856 |
| `rapier2d golden` | 106 | 197,380,564 → 197,380,564 |
| `rapier2d scene` (`gas_scenes` + golden scenes) | 205 | 238,630,101 → 238,630,101 |
| `rapier_sink` (whole crate) | 58 | 505,391,770 → 505,391,770 |
| `rapier2d_classes` (whole crate, non-ignored) | 46 | 977,640,395 → 977,640,395 |
| `rapier2d_classes` `#[ignore]`d 151-tick / bit-identity probes (`steps_cs4_151`, `steps_slim_151`, `steps_split_151`, `steps_batched_151`, `steps_hybrid_151`, `steps_route_b_151`, `steps_advance_values_151`, `steps_islands_values_151`, `steps_forces_out_151`, `steps_levers12_151`, `test_slim_levers_bit_identical_151`, `test_route_b_bit_identical_151`, `test_split_variants_bit_identical_151`, `test_crossing_values_bit_identical_151`, `test_call_counts_*`, `test_state_felts`) | 17 runs | every one bit-identical (steps and per-builtin counts) |

Every test's exact step count, memory-hole count and per-builtin count (`range_check`, `bitwise`)
matches to the unit; `gas.py diff --from-log … --filter <crate>` against the committed
`gas/<crate>/*.snap` reports **"gas snapshot up to date"** for `rapier_math`, `rapier_core`,
`rapier_geometry2d`, `rapier_dynamics2d`, `rapier2d` (no snapshot file needed a rewrite, so none is
committed on this branch, per the brief's "commit only if (a) is identical" — since it is
identical, there is nothing to commit).

`gas/bytecode.size`: every `program.*` line (`full`, `basic`, `no_joints`, `no_sensors`,
`no_composites`, `basic_dispatcher`) and every declared/fixture class's Sierra felts and CASM felts
(`sierra_felts`, `casm_felts` columns) are bit-identical. The JSON **byte** size of the ten declared
classes (`sierra_bytes`) moves by a small constant per class (+15 to +167 bytes on
`gas/bytecode.size`'s ≈216k–16.1M byte classes, e.g. `ActiveSetClass` 216,342 → 216,427,
`GameStep` 15,643,767 → 15,643,772); `casm_bytes` is untouched. Cause: `glam` 0.4.1's `Vec2` is
canonically declared in `glam_core::vec2::Vec2` (facade re-export), so the Starknet class ABI (JSON
type names, not Sierra felts) already spells `glam_core::…` for every class that reaches `Vec2`
through the plain `glam` facade — before rapier-cairo's own source names change at all. No cost
impact (felts, steps and gas are what gates and provers charge); not proposed as a snapshot update
since (a) is otherwise identical.

**Verdict (a): bit-identical.** A PR is opened per the brief's gate.

## 3. (b) Depend on `glam_core` instead of the `glam` facade

rapier-cairo's entire `glam` surface is `Vec2` / `Vec2Trait` / `vec2()` from `glam::vec2` (260
occurrences across `crates/`, `grep`-verified: no `Mat*`, `Quat`, `Affine*`, integer vector or
swizzle use anywhere in the workspace). `glam_core` alone provides that surface unchanged
(`glam_core::vec2::{Vec2, Vec2Trait, vec2}`, `glam_core::{Vec2, Vec2Trait}`); `glam_int` and
`glam_swizzles` are not needed.

Change: `Scarb.toml` `glam = "0.4.1"` → `glam_core = "0.4.1"`; every crate's
`glam.workspace = true` → `glam_core.workspace = true` (`rapier_math`, `rapier_geometry2d`,
`rapier_dynamics2d`, `rapier2d`, `rapier2d_classes`, `rapier_sink`; `rapier_core` has no `glam`
dependency); every `glam::` path in source rewritten to `glam_core::` (mechanical, same identifiers).

Every crate builds clean; **every probe from §2 re-run against this state and is bit-identical
again** (steps, gas snapshots, declared/fixture class felts, `program.*` felts — same tables as
above, one more time, zero deltas). The declared classes' Sierra/CASM felts and the slim caller's
margin under the 73,728-felt limit are therefore unchanged by (b) too (see §4).

Consumer cost (`scripts/consumer_cost.py --repeat 3`, project lock held, GB figures documented as
the stable measure — wall-clock is noisy under this machine's load, see
`docs/research/package-cost.md`):

| closure | over baseline GB, `glam` facade (alpha.7) | over baseline GB, `glam_core` | gain |
|---|--:|--:|--:|
| `rapier2d` (facade closure) | 1.84 | 1.65 | 0.19 GB |
| `game_classes` (`rapier2d_classes` closure) | 1.93 | 1.75 | 0.18 GB |

Every crate's marginal cost stays well inside gate 2 (5 s / 1 GB) either way; no crate's line count
or verdict changes. Nothing else moves: no API rapier-cairo re-exports names a `glam` path (checked
against `docs/API_PARITY.md`, unaffected — rapier-cairo does not re-export glam-cairo items), no
test, golden vector, or gas key changes.

**Verdict (b): ~0.18–0.19 GB (≈10 %) closure-cost gain, zero step/gas/size cost, mechanical source
change (260 `glam::` → `glam_core::` occurrences), no API-visible effect for rapier-cairo's own
consumers.**

## 4. (c) Declared class sizes after (a) and after (b)

Unchanged by both (a) and (b) — same Sierra/CASM felt counts as `main`'s `gas/bytecode.size` for
all ten declared classes and the slim caller, hence the same margin under 73,728 felts:

| class | Sierra felts | margin (73,728 − Sierra) | CASM felts |
|---|--:|--:|--:|
| `ContactBallClass` | 14,950 | 58,778 | 41,560 |
| `ContactPolygonClass` | 14,515 | 59,213 | 57,299 |
| `SolverClass` | 29,500 | 44,228 | 43,726 |
| `SolveAdvanceClass` | 37,044 | 36,684 | 58,547 |
| `IslandsClass` | 7,952 | 65,776 | 19,083 |
| `BroadPhaseClass` | 4,525 | 69,203 | 9,841 |
| `MassClass` | 13,297 | 60,431 | 35,686 |
| `NarrowPhaseClass` | 11,319 | 62,409 | 23,597 |
| `ActiveSetClass` | 4,381 | 69,347 | 10,930 |
| `ForceEventsClass` | 2,832 | 70,896 | 6,202 |
| `SlimSplitStep` (caller) | 28,518 | 45,210 | 73,083 |

Only the JSON *byte* size of the class object moves, by the constant-per-class amount described in
§2 (`glam_core` type-path length), with no felt, gas or margin impact.

## 5. Table for the programme

| option | results identical to `main`? | steps Δ | program / class felt sizes Δ | closure s / GB Δ | API changes for rapier users |
|---|---|---|---|---|---|
| (a) `fixed 0.4.0` / `glam 0.4.1` | **yes**, bit-identical (every `steps_*`, golden, scene, pile10/151, gas snapshot) | 0 | 0 (bytes only: +15…+167 B per declared class, cosmetic) | noise-level (±1 s; GB unchanged) | none |
| (a) + (b) `glam_core` instead of the `glam` facade | **yes**, bit-identical (same probes, re-run) | 0 | 0 | −0.18…−0.19 GB per closure (`rapier2d`, `game_classes`), time noise-level | none observable (rapier-cairo re-exports nothing from `glam`); a project that itself imports `glam::` paths directly (not through rapier-cairo) is unaffected either way since `glam` still resolves those paths as a facade |

## 6. Escalation

`scripts/bytecode_size.py` (`programs_package`, `temp_package`, orchestrator-owned) hardcodes
`pins[d] for d in ("fixed", "glam")` when building its two throwaway packages (the executable
`programs()` fixtures and the `attribution` dev-profile package); neither
`crates/rapier_sink/programs/lib.cairo` nor `crates/rapier_sink/src/scene.cairo` references `glam`
at all, so the dependency is unused there. After (b), `Scarb.toml` no longer has a `glam` key
(`glam_core` instead), so `python3 scripts/bytecode_size.py check` (and `table`, `snapshot`,
`attribution`) raises `KeyError: 'glam'` before it reaches the comparison. One-line fix at both call
sites (`scripts/bytecode_size.py:374` and `:418`): drop `"glam"` from the tuple (it is not needed by
either throwaway package). Verified locally with a scratch-patched copy of the script (not
committed, orchestrator-owned file): with the tuple reduced to `("fixed",)`, `table` reproduces
exactly the numbers of §2–§4 (all felt counts identical to `gas/bytecode.size`, only the same
per-class byte deltas). Until the orchestrator applies that one-liner, CI's `bytecode` job will fail
on this branch (and on any future branch that moves off the `glam` facade) with the `KeyError`
above, not with a real size regression.

## 7. Branch / PR

Branch `feat/du1-dependency-update` (commits: workspace bump, class-hash re-pin, `glam_core`
switch). PR opened per the brief's gate ((a) is bit-identical); see `REPORT.md` for the URL and CI
status. **Not merged**: per the programme's 2026-09-28 go, this is a measurement lot — the
programme decides between shipping (a) alone, (a)+(b), or neither.
