# PO1 — API polish and miscellaneous parity: triage and completion (AP work package, 168 items)

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (programme decisions: free parity items welcome, nothing that costs Cairo steps; the PO
row); `docs/API_PARITY.md` (section "WP: API polish and miscellaneous parity" and the owner tables it names — the
exact items); `scripts/api_parity.py` (`exclusion_reason` and its **closed** reasons, `OWNER_ALIASES`,
`METHOD_RENAMES`, `SKIPPED_CRATES`; how CW #135 and MH1 #160 tightened patterns); on `main`: the Cairo modules owning
each item (`crates/rapier_core/src/**`, `crates/rapier_geometry2d/src/{shape.cairo,shape/**}`, …). Upstream
(`UP=/home/claude/git/refs`): the source file named in each item's row.

## 2. Scope (file allowlist)
`scripts/api_parity.py` (`exclusion_reason` patterns within the closed reasons, `OWNER_ALIASES`, `METHOD_RENAMES` —
each entry justified), `docs/API_PARITY.md` (regenerated), and API-only additions in
`crates/rapier_core/src/{interaction_groups.cairo,integration_parameters.cairo,integration_parameters/**,rigid_body.cairo,rigid_body/**}`,
`crates/rapier_geometry2d/src/{shape.cairo,shape/**}` (new functions / impls only), their tests, and the snapshots that
move. Forbidden: anything the step reaches beyond adding functions (layouts of `Shape`, `RigidBody`, `Collider`,
solver, pipeline, narrow phase), `crates/rapier2d/src/**` (QY2 works there in parallel), `Scarb.toml`/`lib.cairo`.

## 3. Expected result
1. **Triage first**, written into REPORT.md as a table (owner, item, decision, reason): each item is (a) a real 2D
   public API → implement it (upstream name and semantics), (b) covered by an existing **closed** exclusion reason
   whose pattern misses it → tighten the pattern (soft-body constraints such as `NeoHookeanConstraint` /
   `SoftAttachmentConstraint` → `soft bodies`; multibody / soft linear-algebra internals such as `BlockMatrix`,
   `SkylineCholesky`, `ConjugateGradient` → the reason that owns their source file; mesh converters → `trimesh/voxels/3D
   heightfield`; …), never inventing a new reason, or (c) no Cairo counterpart by design (e.g. `InteractionGraph`: the
   port has no interaction graph, ADR 0001 entry 6) → left `missing` with that one-line reason.
2. **Implement (a)**: e.g. `Shape` members of the list, `InteractionGroups` / `AxesMask` / `BodyStatus` /
   `IntegrationParameters` items, `Capsule` / `Segment` / `Cuboid` / `ConvexPolygon` / `HalfSpace` helpers not covered
   by MH1 — free API only (`#[inline(always)]` trivial accessors).
3. Every change of status listed; nothing else moves.

## 4. Constraints
No step cost: every P3, level and scene probe identical; `gas/bytecode.size` unchanged. ≤ 800 lines per file; ≤ 4 fuzz
per module.

## 5. Tests
Table-driven for the implemented items; `python3 scripts/api_parity.py --self-test` and `--check` pass.

## 6. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/`):
`scarb fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on the crates you touch; snapshots with
`snforge test -p <crate> --tracked-resource sierra-gas > gas-<crate>.log` and `python3 scripts/gas.py snapshot --filter
<crate>::<module> --from-log gas-<crate>.log`; `python3 scripts/api_parity.py` then `--check`; `python3
scripts/bytecode_size.py check`. Never a workspace-wide run: CI is the full gate. Conventional commits + trailer; push;
`gh pr create --base main --title "<what ships>" --body-file …`; wait for the checks to be registered, then `gh pr
checks --watch` until green; never merge; `REPORT.md` (Summary · Triage table · Items closed / excluded / left with
reasons · API · Deviations · Requested re-exports · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
