# CN1 — exact start normals of shape casts on face contacts (KC1's escalation)

## 1. Read first
`AGENTS.md`; KC1's report (#202, Escalation 2): the port's shape-cast start witness normal is tilted 10–20 raw for
ball–cuboid (up to 900 for cuboid–cuboid) where upstream's is exact, so a character move exactly parallel to the floor
with `stop_at_penetration = false` counts as a hit (`character_moves/wall_slide`: one extra ground event, a 1e-4 nudge
up); CC1's report (#180) and ADR 0001 entries 30–32 (exact Minkowski ray-cast casts); on `main`:
`crates/rapier_geometry2d/src/query/{shape_cast.cairo,shape_cast/**,support_map*,sweep/**}`, KC1's
`crates/rapier2d/src/control/character_controller.cairo` and `tests/control_golden.cairo`. Upstream
(`UP=/home/claude/git/refs`): parry's `cast_shapes` start-penetration / touching handling.

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/query/{shape_cast.cairo,shape_cast/**,support_map.cairo,support_map/**}`, their tests,
`crates/rapier2d/tests/control_golden.cairo` (re-judge `wall_slide` strictly), the harness `tools/golden/src/**` only to
add start-contact cases to the `shape_casts` family in a new file (existing vectors byte-identical), and the snapshots
that move. Forbidden: the step, CCD's sweep (`query/sweep/**`: read-only unless the same defect lives there — then say so
under Escalations), `Scarb.toml`/`lib.cairo`.

## 3. Expected result
When two shapes touch or overlap at the start of a cast along a face (ball on a cuboid face, cuboid on a cuboid face,
polygon faces, round shapes), the start witness normal is the exact face normal, as upstream's, on both
`stop_at_penetration` paths; `wall_slide` produces upstream's events and positions; every CC1 golden case stays within
its band or improves. Queries only: the step and CCD results do not change (prove it: P3, level, game-shaped probes and
the CCD golden scenes identical in results and exact Cairo steps; `gas/bytecode.size`'s `program.*` unchanged).

## 4. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, through `scripts/build-shims/`): `scarb fmt --workspace`; `scarb lint
-p` / `scarb build -p` / `snforge test -p` on rapier_geometry2d and rapier2d; snapshots with `--from-log`; `python3
scripts/bytecode_size.py check`. Never a workspace-wide run. Rebase on `origin/main` before the PR. Conventional commits
+ trailer; push; `gh pr create --base main --title "<what ships>" --body-file …`; `gh pr checks --watch` until green;
never merge; `REPORT.md` (Summary · Cause · Fix · Golden before / after · Step / CCD unchanged proof · PR URL). Memory rules
apply.

## 5. Work autonomously, do not ask questions, do not widen the scope.
