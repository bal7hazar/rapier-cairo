# RG2 — `ShapeSerde` round-trip cost (RG1's second cause)

## 1. Read first
`AGENTS.md`; RG1's report (#189): since SH2a (#184), a `WorldState` round trip (`serialize` + `deserialize`) costs +676
Cairo steps more than at `0.1.0-alpha.4` on the game-shaped probe, from `ShapeSerde`'s tag dispatch in
`crates/rapier_geometry2d/src/shape.cairo` (SH1 and SH2a / SH2b added variants with tags 6–12); the game pays it at
every chunk boundary. `crates/rapier2d/tests/game_path.cairo` (RG1's `steps_game_*` probes and `test_game_digest`).

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/{shape.cairo,shape/**}` (serde only), its tests, the snapshots that move. Forbidden:
the serialized format (tags and payloads stay byte-identical: states saved before load after), `WorldState`'s
version, everything else. CS2 works in `rapier2d` / dispatch in parallel.

## 3. Expected result
The round trip of states holding only the old shapes (ball, cuboid, capsule, segment, half-space, convex polygon) back
to alpha.4's Cairo steps (the game-shaped probe's "+ 3 WorldState round trips" row ≤ its alpha.4 value), the new
shapes' serde unchanged or cheaper, serialized bytes identical, measured candidates (tag order, a two-level dispatch
for the new tags, per-variant `#[inline(never)]` helpers) with the losers under `alternatives`.

## 4. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, through `scripts/build-shims/`): `scarb fmt --workspace`; `scarb lint
-p` / `scarb build -p` / `snforge test -p` on rapier_geometry2d and rapier2d (for `game_path` and `world_state`);
snapshots with `--from-log`; `python3 scripts/bytecode_size.py snapshot`. Never a workspace-wide run. Rebase on
`origin/main` before the PR. Conventional commits + trailer; push; `gh pr create --base main --title "<what ships>"
--body-file …`; `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · Candidates · Before / after on
`game_path` and `world_state` · Bytes-identical proof · PR URL). Memory rules apply.

## 5. Work autonomously, do not ask questions, do not widen the scope.
