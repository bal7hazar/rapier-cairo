# GG — `rapier_geometry2d::dispatch`: the contact-manifold dispatcher

## 1. Read first
`AGENTS.md`; `docs/interfaces/geometry-dynamics.md` §2 (`contact_manifold` entry point);
`docs/research/02-parry-analysis.md` §4.1 (dispatch order: ball–ball, cuboid–cuboid,
capsule–capsule, anything–ball, halfspace–pfm, then the pfm fallback); on `main`:
`crates/rapier_geometry2d/src/contact_generators/*.cairo` (GF1–GF4: `*_shapes` wrappers returning
`bool`), `shape.cairo` (`Shape`, `ShapeType`). Upstream
(`UP=/home/claude/git/refs`):
`$UP/parry/src/query/default_query_dispatcher.rs` (`contact_manifolds`, `contact_manifold_convex_convex`).
Golden: the whole `rapier_golden::contact_manifolds` family (every case through the dispatcher,
both argument orders).

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/dispatch.cairo`, `crates/rapier_geometry2d/tests/dispatch_golden.cairo`,
`gas/rapier_geometry2d/dispatch.snap`, `gas/rapier_geometry2d_integrationtest/dispatch_golden.snap`.
Module pre-declared.

## 3. Expected API and semantics
```cairo
/// Parry's `contact_manifolds` for one convex pair; `true` when a generator exists for the pair.
pub fn contact_manifold(pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold) -> bool;
```
One `match` on `(shape1, shape2)` variants in upstream's priority order; flipped pairs handled by
calling the generator with swapped arguments and `pos12.inverse()` then swapping the manifold
sides (`local_n1/2`, `local_p1/2`, `fid1/2`, `subshape1/2`) exactly as upstream's `flip` does —
implement `ContactManifold::flip` here if GE did not. Unsupported pairs (segment–segment,
halfspace–halfspace, halfspace–ball is supported via convex–ball) return `false` and leave the
manifold cleared. DEFER: compound/heightfield/trimesh branches, `ContactManifoldsWorkspace`.

Generators on `main` (GF1 #42, GF2 #39, GF3, GF4 #41), each with a typed entry point and a
`*_shapes(pos12, shape1, shape2, prediction, ref manifold) -> bool` wrapper:
`ball_ball::contact_manifold_ball_ball`, `convex_ball::{contact_manifold_convex_ball,
contact_manifold_ball_convex}` (the second is upstream's `flipped = true`),
`cuboid_cuboid::contact_manifold_cuboid_cuboid`, `capsule_capsule`/`cuboid_capsule` (GF3),
`halfspace_pfm::contact_manifold_halfspace_pfm(.., flipped: bool)`,
`cuboid_segment::contact_manifold_cuboid_segment`. Pitfall reported by GF1:
`contact_manifold_convex_ball_shapes` also accepts ball–ball, so the ball–ball arm must come first,
as in upstream's dispatcher. Match on the variants and call the **typed** entry points (the
`_shapes` wrappers re-match the enum: bench one against the other before choosing). Golden: the
family now has 87 cases (G3, #38), including `cuboid_segment`, `halfspace_capsule`,
`halfspace_segment` and their flipped singles.

## 4. Efficiency and variants
The dispatcher must add nothing measurable on top of the generator: `match` on two enum
discriminants, no allocation. Bench the dispatch overhead per pair (dispatcher cost minus the
generator's own `gas_*` figure) and report it.

## 5. Tests
Golden: every `rapier_golden::contact_manifolds` case through `contact_manifold` (both orders),
same tolerances as the GF tests; a table of unsupported pairs returning `false`; `gas_*` for
each pair kind. ≤ 800 lines/file.

## 6. Definition of done
Foreground gate (`scarb fmt --workspace && scarb lint --workspace --deny-warnings && scarb build
--workspace && snforge test --workspace`); `python3 scripts/gas.py snapshot --filter
rapier_geometry2d::dispatch`, `… rapier_geometry2d_integrationtest::dispatch_golden`; conventional
commits + trailer; push; `gh pr create` per template; `gh pr checks --watch` until green; never
merge; `REPORT.md` (Summary · API · Gas table · Deviations · Deferred · Requested re-exports ·
Escalations · PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.
