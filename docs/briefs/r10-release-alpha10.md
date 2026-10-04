# R10 — release 0.1.0-alpha.10, step 1: the version bump on main

Runner: `impl-sonnet` (a documented, mechanical release step), on the VPS. Lot id `r10-release-alpha10`.

Prepare `0.1.0-alpha.10` on `main`, as `0.1.0-alpha.9` was prepared by #260 (`7c494f2`): `Scarb.toml`, `Scarb.lock`,
`CHANGELOG.md` and `docs/PLAN.md`. It carries two MINOR result changes shipped together (FU1, #270; CE, #271) and
everything else merged since `v0.1.0-alpha.9` (OB, B1, SW1, PX8, the FU0 study).

1. Version: every published crate and the workspace `Scarb.toml` from `0.1.0-alpha.9` to `0.1.0-alpha.10`;
   `Scarb.lock` through Scarb, never by hand.
2. `CHANGELOG.md`: `## Unreleased` becomes `## 0.1.0-alpha.10 (2026-10-04)`, an empty `## Unreleased` above it, a
   **Results** line naming the classes a game re-pins.
3. `docs/PLAN.md`: the version line and one "Parked" entry for WS3 (the project manager's decision of 2026-10-03;
   #266 closed, `WorldState` stays v3).

Allowlist: `Scarb.toml`, `Scarb.lock`, `CHANGELOG.md`, `docs/PLAN.md` and this brief. No source, test, gas or CI file.
Nothing is published, tagged or released; the PR is never merged by its thread.
