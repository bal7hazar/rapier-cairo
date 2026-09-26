# CS1 — contract class size of the 2D step: fixture, CI budget, decomposition, lever estimates

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` ("Programme decisions", "Game findings", the CS row); the request
`~/projects/pm/messages/to-rapier/2026-09-26-class-size.md` (read-only); glam-cairo's precedent, read-only:
`~/projects/glam-cairo/scripts/bytecode_size.py`, `~/projects/glam-cairo/packages/consumer/`,
`~/projects/glam-cairo/docs/audits/R1-bytecode-size.md` (limits and their sources, the attribution protocol, the
`inlining-strategy` finding); on `main`: `crates/rapier2d/src/{lib.cairo,world.cairo,pipeline.cairo,pipeline/**,dispatcher.cairo}`,
`crates/rapier_geometry2d/src/{dispatch.cairo,dispatch/**,contact_generators.cairo}`, `crates/rapier_dynamics2d/src/solver.cairo`,
`crates/rapier2d/src/world/state.cairo` (WS: the cost of moving a world across a call boundary).

**Why:** the game's `Slingfall` class, whose `simulate` entry point steps a `rapier2d` world through
`World::step_with_force_events` (0.1.0-alpha.1), is **201,974 Sierra felts (limit 81,920) and 11.7 MB (limit
4,089,446 bytes)**: it cannot be declared, so it cannot run in the SNIP-36 virtual OS (programme milestones M3 / M5).
This lot **measures** and **estimates**; the levers that change engine code ship in CS2, after BT3 (which is changing
the same hot code).

## 2. Scope (file allowlist)
A new fixture crate `crates/rapier_sink/` (a Starknet contract, never published: `scripts/release.sh` lists the
published crates), `scripts/bytecode_size.py` (new), a `bytecode` job in `.github/workflows/ci.yml` (added next to the
existing jobs, nothing else changed there), `scripts/api_parity.py` only to skip `rapier_sink`, the snapshot
`gas/bytecode.size` (new), a report `docs/research/class-size.md` (new). **No change to any engine crate**: lever
estimates are measured on throwaway copies (a temporary package outside the workspace, as glam's `attribution` does)
and described, not committed. Forbidden: engine sources, `Scarb.toml` of other crates, the root `Scarb.toml` beyond
what a new member needs (list it under Escalations if anything is needed).

## 3. Deliverables
1. **Fixture:** `rapier_sink` with a `Sink` contract whose entry points build a small world with each of the five shape
   types (ball, cuboid, capsule, half-space, convex polygon; a segment if cheap) and joints, then call
   `step_with_force_events` (one entry point per configuration below), compiled as a `starknet-contract` target with
   the release profile the game would use.
2. **Script and CI:** `scripts/bytecode_size.py [table|snapshot|check|attribution]` modelled on glam's (Sierra felts,
   CASM felts, Sierra and CASM bytes as a declare transaction carries them; limits with their sources), the snapshot
   `gas/bytecode.size`, and a CI `bytecode` job running `check` (equality, deterministic toolchain).
3. **Decomposition:** the class's size attributed to its parts: contact generators per shape pair, the dispatch table,
   the narrow phase, constraint generation and the solver (contacts; joints), sleeping / islands, the broad phase,
   events, `WorldState` Serde; by removal (a variant of the fixture without the part, when the closed enums allow it)
   or by attribution probes.
4. **Lever estimates** (throwaway builds, each with its size AND its exact Cairo steps on G0's level-10 impact window,
   so that the owner's steps criterion can arbitrate): `inlining-strategy` (`default`, `avoid`, numeric) set by the
   top-level package; `#[inline(never)]` on the N largest inlined bodies; feature-gating shape pairs / joints a game
   does not use (Scarb features + `cfg`: which enum arms stay reachable); dispatch dedup; a **multi-class layout**
   (the step split across classes called with `library_call_syscall`, the world crossing each boundary via
   `WorldState` Serde: its steps cost per call, from WS's round-trip figures, and whether SNIP-36's virtual OS allows
   it — say what you could verify and what stays open).
5. **The floor:** the smallest class that steps a world with the game's shapes (ball, cuboid, convex polygon,
   half-space; no joints) under each lever combination, and whether it fits 81,920 Sierra felts / the CASM limit /
   4,089,446 bytes. If no combination fits one class, say so plainly with the numbers (the programme then weighs the
   multi-class layout against Stone + Integrity on the standalone executable).

## 4. Constraints
Build memory: a contract build of the whole engine is heavy — one build at a time, through the build shims, never in
the background; delete throwaway targets after use. No engine change, no snapshot of engine crates moves.

## 5. Definition of done
Crate-scoped local gate on `rapier_sink` (`scarb fmt --workspace`, `scarb lint -p rapier_sink --deny-warnings`,
`scarb build -p rapier_sink`), `python3 scripts/bytecode_size.py check`, `python3 scripts/api_parity.py --check`.
Never a workspace-wide test run locally: CI is the full gate. Conventional commits + trailer; push; `gh pr create --base
main --title "<what ships>" --body-file …`; wait for the checks to be registered, then `gh pr checks --watch` until
green; never merge; `REPORT.md` (Summary · Fixture and CI · Size table vs limits · Decomposition · Lever estimates
(size | steps) · Floor and verdict · Multi-class assessment · Recommendation for CS2 · Escalations · PR URL). Memory
rules apply.

## 6. Work autonomously, do not ask questions, do not widen the scope.
