# AP — generated API parity inventory against rapier-rs 0.35.3 (2D) and the parry2d subset it exposes

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (scope, "Explicitly out of scope", phase 2/3 tables); `docs/adr/0001-upstream-divergences.md`;
the precedents `/home/claude/projects/nalgebra-cairo/scripts/api_parity.py` + `docs/API_PARITY.md` and
`/home/claude/projects/glam-cairo/scripts/api_parity.py` (read-only: dependency-free parser that masks comments,
finds balanced blocks, embeds the Rust inventory in the generated Markdown so `--check` needs no Rust checkout).
Upstream (`UP=/home/claude/git/refs`, read-only): `$UP/rapier/src/**` (commit `28d0ba9`, rapier 0.35.3 + 4) and
`$UP/parry/src/**` (0.31.1; the golden vectors pin `parry2d-f64 0.30.2` — note it). The Cairo side: `crates/**/src`.

## 2. Scope (file allowlist)
`scripts/api_parity.py` (new; the orchestrator explicitly delegates this `scripts/**` file to you),
`docs/API_PARITY.md` (generated). Nothing else; CI wiring is the orchestrator's (say what `--check` needs).

## 3. Expected result
A dependency-free Python 3 script: `--refresh --rapier <checkout> --parry <checkout>` re-reads the Rust sources
and embeds the inventory; plain run regenerates `docs/API_PARITY.md` from the embedded inventory + the Cairo
sources; `--check` fails when the file is stale. Items: `(owner, kind, name)` for the public API of rapier's
`dynamics`, `geometry`, `pipeline`, `control` modules and the parry2d items rapier re-exports or users need
(`shape`, `query` entry points, `bounding_volume::Aabb`, mass properties, `Ray`, …): types, inherent `pub fn`
(associated functions included), builder methods, consts, trait impls normalised, free functions. **2D only**:
honour `#[cfg(feature = "dim3")]` / `dim2` (drop dim3-only items, keep dim2 and shared). Owners map to Cairo
candidates (e.g. `RigidBody` → `RigidBody` + `RigidBodyTrait`, `RigidBodyBuilder`, `Collider` + `ColliderTrait`,
`ColliderBuilder`, `PhysicsPipeline`/`PhysicsWorld` → `World`/`WorldTrait` + `pipeline`, `QueryPipeline` →
`queries` + `WorldTrait`, …) with a `RENAMES` table for documented renames (read ADR 0001 and the plan). Status:
ported / partial / missing / excluded, with a **closed list of exclusion reasons**: dim3-only, soft bodies,
multibody, SIMD/parallel, debug render, serde/rkyv/bytemuck, profiling counters, `dyn` hooks (`PhysicsHooks`,
`EventHandler` as traits — note the port's built-in replacements), trimesh/voxels/3D heightfield, EPA/GJK internals
not exposed, f32/f64 conversions and `approx` traits. Anything else defaults to **missing**. Coverage summary per
module and total, then per-owner tables, then the "missing" list grouped into candidate work packages (the
orchestrator turns them into briefs): name each group and estimate its size.

## 4. Efficiency
Not a gas package. The script must run in < 30 s.

## 5. Tests
`python3 scripts/api_parity.py --check` passes after generation; a small self-test mode (`--self-test`) on
embedded snippets (cfg dim3 skip, builder chain methods, trait impl normalisation) is welcome.

## 6. Definition of done
Generate, check; conventional commits + trailer; push; `gh pr create --base main --title "<what ships>" --body-file …`
per template; wait for the checks to be registered, then `gh pr checks --watch` until green; never merge; `REPORT.md`
(Summary · Coverage table · Top missing groups with sizes · Exclusion rules applied · What CI needs · PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.
