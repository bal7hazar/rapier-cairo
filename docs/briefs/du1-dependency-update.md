# DU1 — dependency update to fixed 0.4 / glam 0.4.1 (measurement first)

## 1. Read first
`AGENTS.md`; `Scarb.toml` (`[workspace.dependencies]`: `fixed = "0.3.0"`, `glam = "0.3.0"`), `docs/PLAN.md` D12 (the
scalar and vector types come from the registry; a MINOR bump may change numeric results, so it is its own PR);
`docs/research/package-cost.md` (PK1) and `consumer_cost.toml`. The sibling repositories, read-only (fetch them first:
`git -C ~/projects/glam-cairo fetch`, `git -C ~/projects/fixed-cairo fetch`, and read `origin/main`): their
CHANGELOGs between 0.3.0 and the current releases (fixed 0.4.0; glam 0.4.1, now split — a `glam_core` crate and the
`glam` facade), and the registry (`scarbs.xyz`).

**Programme go (2026-09-28), measurement lot only: nothing merges into `main` without the programme's answer.**
**Programme request (2026-09-28):** rapier depends on fixed ^0.3.0 and glam ^0.3.0 while the current generation is
fixed 0.4.0 / glam 0.4.1: a project that uses rapier with the current glam compiles both generations. Measure, do not
decide: a numeric change would cost the game a re-pin, the programme decides on the table.

## 2. Work (on a branch; open a PR only if (a) is bit-identical — otherwise push the branch, no PR)
(a) Move the workspace to fixed 0.4.0 / glam 0.4.1 (source changes only where an API moved), compared with `main` at
    alpha.7 (`v0.1.0-alpha.7`). The probes, each before / after in exact Cairo steps (`--tracked-resource cairo-steps
    --detailed-resources`) and results:
    - `snforge test -p rapier2d steps_` (every `steps_*` probe: `game_path` `steps_game_*`, P3 contact scenes, level
      windows, sleep) and `snforge test -p rapier2d ccd`;
    - every golden replay and scene test: `snforge test -p rapier_geometry2d golden`, `snforge test -p rapier_dynamics2d
      golden`, `snforge test -p rapier2d golden`, `snforge test -p rapier2d scene`;
    - `snforge test -p rapier2d_classes` (pile10 bit-identity, `game_ticks`, `removals`) and `snforge test -p rapier_sink`;
    - the full crates' gas snapshots (`python3 scripts/gas.py diff --from-log …` per crate, in a scratch copy);
    - `python3 scripts/bytecode_size.py table` against `gas/bytecode.size`: every `program.*` line and every class line.
    Are the step results, every `steps_*` probe and every `program.*` bit-identical to alpha.7? If not, list what
    changes (which tests, by how much, the first diverging quantity and its cause in the dependency's CHANGELOG / diff),
    and the Sierra-gas and exact-Cairo-step deltas of the probes that move.
(b) Depend on `glam_core` (and whichever of `glam_swizzles` / `glam_int` is really used — say which items need them)
    instead of the `glam` facade: the gain on the `rapier2d`
    and `game_classes` closures and on each crate's marginal cost (`python3 scripts/consumer_cost.py --repeat 3`, under
    the project lock), and whether anything else changes.
(c) The declared classes' sizes (Sierra, CASM) and the slim caller's margin under 73,728 after the change (and after
    (b)).
(d) The table for the programme: option × {results identical?, steps Δ, program / class sizes Δ, closure s / GB Δ,
    API changes for rapier users}.

## 3. Scope (file allowlist)
Measurement branch: `Scarb.toml` `[workspace.dependencies]` and the crates' dependency lines (orchestrator exception for
this lot), the source lines an API move requires, `Scarb.lock` (regenerated), the snapshots that move, and
`docs/research/dependency-update.md` (new: the table and findings). Forbidden: any numeric workaround to hide a
difference; `.github/**`.

Order on this machine: run the heavy suites (rapier2d, rapier2d_classes) one at a time under the project lock.

## 4. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/` — the
project lock; one crate at a time; never two whole-shot runs at once): `scarb build -p` / `snforge test -p` per crate
(rapier_math, rapier_core, rapier_geometry2d, rapier_dynamics2d, rapier2d, rapier2d_classes, rapier_sink), snapshots with
`--from-log` in a scratch copy to diff (do not commit changed snapshots unless (a) is identical), `python3
scripts/bytecode_size.py check`, `python3 scripts/api_parity.py --check`. Commit wip states early. Conventional commits +
trailer; push; PR only if (a) is bit-identical (`gh pr create --base main …`, `gh pr checks --watch` until green, never
merge); `REPORT.md` (Summary · (a) results and sizes · (b) closure gain · Table · Branch / PR URL · Escalations). Memory
rules apply.

## 5. Work autonomously, do not ask questions, do not widen the scope.
