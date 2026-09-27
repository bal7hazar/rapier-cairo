# PX1 — parity closeout: "solver / island internals not exposed", `SharedShape` → `Shape`, two coverage figures

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` ("State after 0.1.0-alpha.6", option (a)); the programme's decision (2026-09-27, under the
owner's delegation): approved with three conditions — (1) the new closed reason **lists every excluded item by name**
in the inventory, (2) `SharedShape` maps to `Shape`, (3) the report keeps **both figures**, the raw parity (today's
73.9 %) and the in-scope parity after the new reason, so the higher one never hides the lower one;
`scripts/api_parity.py` (`EXCLUSIONS`, `exclusion_reason`, `OWNER_ALIASES`, `METHOD_RENAMES`, the summary and
work-package rendering); `docs/API_PARITY.md` (the missing items of "Query completion", "Joint API completion",
"Pipeline and world facade", "Additional 2D shapes", "API polish", … — the candidates).

## 2. Scope (file allowlist)
`scripts/api_parity.py`, `docs/API_PARITY.md` (regenerated, never by hand). Nothing else.

## 3. Expected result
1. A new closed reason `solver / island internals not exposed` in `EXCLUSIONS`, applied through an **explicit,
   commented list of `(owner, kind, name)`** (or owner names when a whole owner is internal) — never a loose pattern:
   rapier's contact / joint constraint builders and parts (`ContactWith*FrictionBuilder`, `ContactWith*Friction`,
   `ContactConstraint*Part`, `GenericContactConstraint`, `ContactConstraintsSet`, `JointConstraint*`,
   `JointGeneric*ConstraintBuilder`, `JointConstraintHelper`, `GenericJointConstraint`, `AnyJointConstraintMut`,
   `CoulombContactPointInfos`, `StagedIslandSolver`, …), the persistent islands (`PersistentIslands`, `Island`,
   `IslandManager` internals), the BVH broad phase (`BroadPhaseBvh`, `BvhOptimizationStrategy`), the manifold
   workspaces (`*ContactManifoldsWorkspace`), and whatever else of the same nature you find — each justified in a
   comment (why it is an internal, which ADR / decision says there is no Cairo counterpart). Anything a user of rapier
   calls stays in scope.
2. `OWNER_ALIASES`: `SharedShape` → `Shape` (constructors and accessors map by name where the semantics match; list
   what does not match and leave it `missing` with a reason).
3. The coverage summary shows **two columns**: "raw" = ported / (items − excluded by the reasons that existed before
   PX1) and "in scope" = ported / (items − all excluded), per module and in total; the header explains both. The
   work-package tables keep counting only in-scope missing items.
4. A list, in REPORT.md, of every item that changed status (owner, kind, name, old → new).

## 4. Definition of done
`python3 scripts/api_parity.py --self-test`, `python3 scripts/api_parity.py`, `--check`. Conventional commits + trailer;
push; `gh pr create --base main --title "<what ships>" --body-file …`; `gh pr checks --watch` until green; never merge;
`REPORT.md` (Summary · Both figures before / after · The internal list by owner · SharedShape mapping · Status changes ·
Escalations · PR URL). Memory rules apply.

## 5. Work autonomously, do not ask questions, do not widen the scope.
