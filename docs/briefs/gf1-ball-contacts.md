# GF1 — contact generators: ball–ball and convex–ball

## 1. Read first
`AGENTS.md`; `docs/interfaces/geometry-dynamics.md` §2 and `crates/rapier_geometry2d/src/contact.cairo`
(frozen `ContactManifold`, `TrackedContact`, `FeatureId`); `docs/PLAN.md` wave-1 outcomes (empty
manifolds are legal, points beyond prediction are kept, ball–ball tests `<` where others test
`<=`); on `main`: `crates/rapier_geometry2d/src/{shape.cairo,point.cairo,manifold.cairo}`
(GB shapes, GC `project_local_point_and_get_feature_*`, GE `try_update_contacts`/`match_contacts`)
and `src/sat.cairo` as a style precedent for golden tests. Upstream
(`UP=/private/tmp/claude-501/-Users-bal7hazar-git-rapier-cairo--claude-worktrees-rapier-physics-cairo-benchmark-2d3317/239b7c62-97d7-47ea-9d79-74f81363555f/scratchpad/refs`):
`$UP/parry/src/query/contact_manifolds/{contact_manifolds_ball_ball.rs,contact_manifolds_convex_ball.rs}`.
Golden: `rapier_golden::contact_manifolds` cases whose pair is ball–ball, ball–cuboid,
ball–capsule, halfspace–ball, segment–ball (and the flipped-order cases); `tools/golden/README.md`
for tolerances, `ambiguous` tags and the f32 feature-id convention.

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/contact_generators/ball_ball.cairo`,
`crates/rapier_geometry2d/src/contact_generators/convex_ball.cairo`,
`crates/rapier_geometry2d/tests/contact_ball_golden.cairo`,
`gas/rapier_geometry2d/contact_generators.snap` (entries of your two modules only; regenerate with
`--filter rapier_geometry2d::contact_generators::ball_ball` and `…::convex_ball`),
`gas/rapier_geometry2d_integrationtest/contact_ball_golden.snap`. `lib.cairo` and
`contact_generators.cairo` already declare the modules. Everything else is forbidden.

## 3. Expected API and semantics
```cairo
pub fn contact_manifold_ball_ball(pos12: Pose2, ball1: Ball, ball2: Ball, prediction: Fixed, ref manifold: ContactManifold);
pub fn contact_manifold_convex_ball(pos12: Pose2, shape1: Shape, ball2: Ball, prediction: Fixed, ref manifold: ContactManifold);
```
Upstream semantics exactly: ball–ball keeps the point when `dist < prediction` (strict), normal
from the centre difference (upstream's fallback normal when centres coincide: reproduce it —
the golden `degenerate` case records what f64 did), one point with `fid = FEATURE_UNKNOWN` for
balls; convex–ball projects the ball centre on shape 1 (`project_local_point_and_get_feature`),
`dist = |p - c| - r` with the sign from `is_inside`, `fid1` = the projected feature, `fid2 =
UNKNOWN`, normal flipped when inside, point kept when `dist <= prediction`. Both call
`manifold.clear()` then push, and the caller handles `match_contacts` (GE) — do not call it here
unless upstream's `*_shapes` wrapper does; mirror the wrapper split (`_shapes` takes `Shape`
values and downcasts, the inner function takes the concrete shapes).
DEFER: compound/subshape positions, `ContactManifoldsWorkspace`.

## 4. Efficiency and variants
One `normalize2` per call at most; `dist` from the wide length; no `Fixed` division. Bench
`contact_manifold_ball_ball` and `convex_ball` per concrete shape; for convex–ball, compare (a)
projection then separate normal computation with (b) reusing the projection's direction; ship the
winner.

## 5. Tests
Golden: every applicable `rapier_golden::contact_manifolds` case (num_points, local points, dist,
normals within README tolerances; feature ids exact; `ambiguous` cases only checked for
num_points and dist). Table-driven: separated beyond prediction (0 points), touching, deep,
coincident centres; ball vs each convex shape inside/outside. `fuzz_*` ≤ 4; `gas_*` per public
function and candidate. ≤ 800 lines per file.

## 6. Definition of done
Foreground gate (`scarb fmt --workspace && scarb lint --workspace --deny-warnings && scarb build
--workspace && snforge test --workspace`); the snapshot filters above; conventional commits +
trailer; push; `gh pr create` per template; `gh pr checks --watch` until green; never merge;
`REPORT.md` (Summary · API · Gas table · Deviations · Deferred · Requested re-exports · Escalations
· PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.
