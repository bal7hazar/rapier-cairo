# GE — `rapier_geometry2d::manifold`: manifold persistence and warm-start matching

## 1. Read first
`AGENTS.md`; `docs/interfaces/geometry-dynamics.md` §2–3 (frozen types, already in
`crates/rapier_geometry2d/src/contact.cairo` — read it); `docs/research/02-parry-analysis.md` §4.3;
`crates/rapier_math/src/consts.cairo` (`COS_1_DEGREES`, squared thresholds) and
`math_ext/norm2.cairo`. Upstream (`UP=/private/tmp/claude-501/-Users-bal7hazar-git-rapier-cairo--claude-worktrees-rapier-physics-cairo-benchmark-2d3317/239b7c62-97d7-47ea-9d79-74f81363555f/scratchpad/refs`):
`$UP/parry/src/query/contact_manifolds/contact_manifold.rs` (`try_update_contacts`,
`try_update_contacts_eps`, `match_contacts`, `match_contacts_using_positions`, `find_deepest_contact`,
`clear`), `$UP/rapier/src/geometry/narrow_phase/` (how `NEW_CONTACT_BIT` and warm-start impulses are
consumed — read only).

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/manifold.cairo`, `gas/rapier_geometry2d/manifold.snap`. `lib.cairo`
already declares `pub mod manifold;`. The types in `contact.cairo` are frozen: do not edit them;
needed additions go under "Escalations".

## 3. Expected API and semantics
On `ContactManifold` (extension trait `ManifoldTrait`):
`try_update_contacts(ref self, pos12: Pose2) -> bool` (upstream: recompute `dist` and `local_p2`
from the stored points if the normal rotated less than 1° — compare `COS_1_DEGREES` on the wide
dot of the new and old normals — and points moved less than `1e-3` (scaled by the length unit);
return `false` to force full regeneration), `try_update_contacts_eps(ref self, pos12, angle_cos_tol,
dist_sq_tol)`, `match_contacts(ref self, old: @ContactManifold)` (transfer `ContactData` where
`(fid1, fid2)` match; unmatched points keep default data), `match_contacts_using_positions(ref self,
old, dist_threshold)`, `find_deepest_contact(self) -> Option<u8>`, `max_dist(self)`. Upstream
quirks to keep: unknown feature ids never match; points beyond prediction may remain.
DEFER: `subshape_pos` handling for compounds.

## 4. Efficiency and variants
`match_contacts` is called for every pair every step: at most 2×2 id comparisons, no dict.
Bench `try_update_contacts` as (a) direct port and (b) with the position/normal checks fused into
one wide accumulation; ship the winner.

## 5. Tests
Table-driven: normal rotated 0.5° vs 2°, point moved below/above threshold, matching with 0/1/2
common ids, swapped order, unknown ids, empty manifold; `fuzz_*` ≤ 4; `gas_*` for every public
function and candidate. ≤ 800 lines per file.

## 6. Definition of done
Foreground gate (`scarb fmt --workspace && scarb lint --workspace --deny-warnings && scarb build
--workspace && snforge test --workspace`); `python3 scripts/gas.py snapshot --filter
rapier_geometry2d::manifold`; conventional commits + trailer; push; `gh pr create` per template;
`gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · API · Gas table ·
Deviations · Deferred · Requested re-exports · Escalations · PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.
