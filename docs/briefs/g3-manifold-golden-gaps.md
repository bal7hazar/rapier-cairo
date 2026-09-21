# G3 — golden vectors for the contact pairs GF4 could only test analytically

## 1. Read first
`AGENTS.md`; `tools/golden/README.md` (quantisation rule, `contact_manifolds` section, `ambiguous`
tags, f32 feature-id convention, idempotent regeneration); `tools/golden/src/{manifolds.rs,shapes.rs,q.rs,cairo.rs}`
(the `Case` table you extend, `ShapeSpec::{halfspace_up, segment, capsule_x, capsule_y, cuboid}`);
`crates/rapier_golden/src/generated/contact_manifolds.cairo` and `crates/rapier_golden/tests/sanity.cairo`
(the `const` + `ALL` + `cases()` pattern, `cases().len()` assertion); `docs/briefs/gg-dispatch.md` §3
(the pair matrix of the port). Upstream (`UP=/home/claude/git/refs`, parry clone is 0.31.1 while
the pinned crate is `parry2d-f64 =0.30.2`: use the published crate's API, the clone only to read
algorithms): `$UP/parry/src/query/default_query_dispatcher.rs`,
`$UP/parry/src/query/contact_manifolds/{contact_manifolds_halfspace_pfm.rs,contact_manifolds_pfm_pfm.rs,contact_manifolds_cuboid_capsule.rs}`.

## 2. Scope (file allowlist)
`tools/golden/src/manifolds.rs`, `tools/golden/src/shapes.rs` (new constructors only),
`tools/golden/vectors/contact_manifolds.json`, `tools/golden/README.md` (the `contact_manifolds`
section and the families table row only), `crates/rapier_golden/src/generated/contact_manifolds.cairo`
(generated, never by hand), `crates/rapier_golden/tests/sanity.cairo` (the case-count assertion and
the per-regime lists), `gas/rapier_golden_integrationtest/sanity.snap`. Everything else is
forbidden: no engine crate, no `types.cairo` change, no `Cargo.toml`/`Cargo.lock`/`Scarb.toml`, no CI.

## 3. Expected content and semantics
Add to the `contact_manifolds` family, with the same six regimes as the existing pairs (`separated`,
`within_pred`, `touching`, `shallow`, `deep`, `degenerate`) and the same id scheme
`<shape1>_<shape2>/<regime>`:
- `halfspace_capsule` (6): capsule upright, lying flat on the plane (2 points), and tilted 30°
  across the regimes; `degenerate` = capsule axis exactly parallel to the plane with both ends at
  `dist == prediction`;
- `halfspace_segment` (6): same layout with a segment; `degenerate` = segment lying exactly in the plane;
- `cuboid_segment` (6): segment facing a cuboid face (2 points), tilted towards a corner (1 point),
  crossing the cuboid (`deep`); `degenerate` = segment collinear with a cuboid edge;
- flipped-order singles, as the family already does for other pairs: `capsule_halfspace`,
  `segment_halfspace`, `segment_cuboid` (one `shallow` case each).
**Hard constraint: the diff on every existing case must be empty** (append new constants; `ALL`
and `cases()` grow; existing names, order and values unchanged). All inputs exactly representable
in Q32.32 (`q.rs`); feature ids from the f32 run like the rest of the family; tag `ambiguous` with
a `note` wherever a discrete output hinges on an exact tie. If upstream routes one of these pairs
through the GJK-based `pfm_pfm` generator, say so in the README (the Cairo port uses SAT + clipping
for cuboid–segment, so normals/points must agree but tie-breaking may not → `ambiguous`).
DEFER: segment–segment, capsule–segment, convex polygons, any new family.

## 4. Efficiency
Not a gas package. Keep the generated file growth under ~900 lines (21 cases); no sweeps.

## 5. Tests
`sanity.cairo`: update `cases().len()` (66 → 87), add the new pairs to the per-regime lists the file
already checks (`separated` ⇒ 0 points, `touching`/`shallow`/`deep` ⇒ ≥ 1 point, unit normals within
the file's tolerance, `local_n2` consistent with `pos12`). Table-driven, no new test file, no fuzz.

## 6. Definition of done
In `tools/golden`: `cargo run --release --locked` twice → zero `git diff` the second time, and zero
diff on pre-existing cases the first time (`git diff` must show additions only in the generated
file and the JSON). Then from the repository root the foreground gate (`scarb fmt --workspace &&
scarb lint --workspace --deny-warnings && scarb build --workspace && snforge test --workspace`);
`python3 scripts/gas.py snapshot --filter rapier_golden_integrationtest::sanity`; conventional
commits + trailer; push; `gh pr create` per template; `gh pr checks --watch` until green (the
`golden` CI job regenerates and diffs); never merge; `REPORT.md` (Summary · API (new constants) ·
Gas table (sanity tests only) · Deviations (what upstream does for each new pair, which generator
it routes to) · Deferred · Requested re-exports · Escalations · PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.
