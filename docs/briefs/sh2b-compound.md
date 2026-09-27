# SH2b — additional 2D shapes, part 2b: compound shapes

## 1. Read first
`AGENTS.md` (§7, incl. the SH1 lesson on new `match` arms); `docs/PLAN.md` (the SH2 row); **SH2a's report and code
(#184)**: the composite design this lot extends — boxed variants, per-part queries (`*_composite` in
`{point,ray,query,dispatch}/composite.cairo`), the implicit tree prefilter, composite pairs as consecutive
`ContactPair` entries led by the one holding the event status, force events per collider pair, CCD against
composite targets, sub-shape ids returned by `*_part` functions; `docs/adr/0001-upstream-divergences.md` (entry 35);
`docs/API_PARITY.md` (owner tables `Compound`, `CompoundFlags`, `CompoundPseudoNormals`, `ColliderBuilder::compound`,
`Shape::as_compound`, the remaining composite items); on `main`: the geometry, narrow-phase and builder files SH2a
touched. Upstream (`UP=/home/claude/git/refs`): `$UP/parry/src/shape/compound.rs`, the compound cases of
`$UP/parry/src/query/**`, `$UP/rapier/src/geometry/collider.rs` (`ColliderBuilder::compound`, mass properties of
compounds).

## 2. Scope (file allowlist)
The same files as SH2a's allowlist (`crates/rapier_geometry2d/src/**` for the shape, queries, dispatch and generators;
`crates/rapier_dynamics2d/src/{narrow_phase.cairo,narrow_phase/**,collider/builder.cairo}` and siblings;
`crates/rapier2d/src/{pipeline.cairo,pipeline/**,world/state.cairo}` only where pairs / events / state must follow),
their tests, the harness `tools/golden/src/**` (new families; existing vectors byte-identical), vectors / fixtures /
types (additions only), `tools/golden/README.md`, `scripts/api_parity.py` (entries you justify), `docs/API_PARITY.md`
(regenerated), and the snapshots that move. Forbidden: the contact and joint solvers, `Scarb.toml`/`lib.cairo`.

## 3. Expected result
- **`Compound`**: a list of `(Pose2, Shape)` parts (convex parts: ball, cuboid, capsule, segment, triangle, convex
  polygon, round shapes; nested composites rejected as upstream does, or documented), a boxed `Shape` variant.
- Every query SH2a gives polylines / heightfields, for compounds, with the part's sub-shape id: AABB, **mass
  properties (sum of the parts', upstream)**, point / ray / intersection / distance / contact / closest points, shape
  casts, swept TOI; contacts against every convex shape and against half-spaces, polylines and heightfields where
  upstream supports the pair (composite–composite: follow upstream's support matrix; say what is unsupported).
- `ColliderBuilder::compound`, `Shape::as_compound`, `Compound` members; every other item: implemented, excluded
  (closed reason), or `missing` with a reason.

## 4. Constraints, golden and efficiency
Existing shapes and pairs: no Cairo step moves beyond SH2a's composite hook (before / after table on P3, level windows,
golden replays, generator probes); Sierra gas ≤ +1 %, explained; `gas/bytecode.size` regenerated; `WorldState` version
bumped only if its layout changes. Golden families from parry2d-f64 0.30.2 / rapier2d-f64 0.35.3: compound vs every
supported shape (manifolds per part, queries, mass properties) and a scene (an L-shaped compound block toppling on a
ground, events exact). Measure the per-step cost of a 2-, 4- and 8-part compound resting on a half-space and on a
polyline. ≤ 800 lines per file; ≤ 4 fuzz per module.

## 5. Tests
The golden families and scene; table-driven unit tests; existing tests unchanged; `python3 scripts/api_parity.py --check`.

## 6. Definition of done
Harness twice → zero diff. Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through
`scripts/build-shims/`): `scarb fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on
rapier_geometry2d, rapier_golden, rapier_dynamics2d and rapier2d (one at a time); snapshots with `--from-log`; `python3
scripts/bytecode_size.py snapshot`; `python3 scripts/api_parity.py` then `--check`. Never a workspace-wide run: CI is
the full gate. Rebase on `origin/main` before the PR. Commit wip states early (the lot is large). Conventional commits +
trailer; push; `gh pr create --base main --title "<what ships>" --body-file …`; wait for the checks to be registered,
then `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · Items · API · Golden results and bands ·
Steps proof · Costs of compound pairs · `WorldState` · Deviations · Requested re-exports · Escalations · PR URL).
Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
