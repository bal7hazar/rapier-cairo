# BT4 — Cairo steps of the pipeline glue, dormant pairs in mixed ticks, and arena reads

## 1. Read first
`AGENTS.md` (§7); `docs/PLAN.md` (D7 / D9 as amended by BT2, BT row); `docs/BUDGETS.md` (BT1, BT2, BT3 entries); the
reports' pipeline-side findings reproduced below; on `main`: `crates/rapier_core/src/{data.cairo,data/**}` (the
arena), `crates/rapier_dynamics2d/src/{rigid_body_set.cairo,rigid_body_set/**,collider_set.cairo,solver/island.cairo,solver/island/**}`,
`crates/rapier2d/src/{pipeline.cairo,pipeline/**,world.cairo,world/**}`, the probes `crates/rapier2d/tests/{level_budget,sleep_budget,gas_scenes}.cairo`.

**Why (measured by BT2 / BT3):**
1. BT2's sets carry a `modified` field through every pass: ≈ 85 steps per step on every scene (P3 +1–3 %); and an
   activation read (`WorldTrait::is_sleeping` / `linvel` / `angvel`) costs ≈ 146 steps because `Arena::get` copies the
   whole body (the game reads every dynamic body each tick).
2. The pipeline glue around `solve_island` is 52.2k steps per impact tick: `SweepBodies` could be built straight from
   the entries (≈ 6–8k: the `DenseBodies` round trip), and a narrower `solve_island` interface returning impulses only
   would remove `scatter_touching_split` (≈ 8k).
3. In a mixed tick (some bodies awake, others asleep) the sleeping part still costs: level 20's impact window is ≈ 360k
   steps above level 10's, all pipeline-side — dormant split and merge 108k, pair iteration 62k, islands 42k, broad
   phase 54k — although its second structure sleeps.
**Cairo steps are the measure.**

## 2. Scope (file allowlist)
`crates/rapier_core/src/{data.cairo,data/**}`, `crates/rapier_dynamics2d/src/{rigid_body_set.cairo,rigid_body_set/**,collider_set.cairo,solver/island.cairo,solver/island/**}`
(the `solve_island` interface only, not the sweeps), `crates/rapier2d/src/{pipeline.cairo,pipeline/**,world.cairo,world/**}`,
their tests / benches, probes in `crates/rapier2d/tests/{level_budget,sleep_budget}.cairo`, and the snapshots that
move. Forbidden: contact generators, narrow-phase contact computation, the sweep kernels, `Scarb.toml`/`lib.cairo`.
(CS1 is measuring class size in parallel without touching engine code.)

## 3. Levers (the profile decides; one measured commit each)
1. **Arena:** a `modified` bit packed into an existing arena field (no extra field carried by the sets), and a field
   accessor that reads one component of an entry without copying the entry (activation, velocities, pose). Target:
   P3 back to BT3's `main` minus BT2's +1–3 %; activation reads ≤ 50 steps.
2. **Glue:** `SweepBodies` from the entries; `solve_island` returning impulses only; any other copy between stages the
   profile shows.
3. **Dormant pairs in mixed ticks:** keep the pairs of sleeping islands out of the per-step pair walk (persist them
   beside the live pairs, touch them only when an island wakes), and let islands / the broad phase skip sleeping
   structures the way BT2 did for all-asleep ticks. Target: level 20's impact window within +25 % of level 10's (≈ +19 %
   today at the tick level, +360k over the window).

## 4. Constraints
Results **bit-identical** (impact digests, every golden and scene test, WS round trips). New persistent state goes into
`WorldState` with a version bump (3) and the chunked round-trip tests. P3 may only go down; impact, flight and load
windows may only go down. No benchmark-shaped fast path. Do not run the `#[ignore]`d whole-level replays locally
(≈ 15 GB each; check your test filters). Run crate test suites one at a time in the foreground.

## 5. Tests
Existing tests unchanged; the randomized equivalence test of BT2 extended to mixed ticks (one structure awake, one
asleep, wake-ups through contact); ≤ 800 lines per file; ≤ 4 fuzz per module.

## 6. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/`):
`scarb fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on rapier_core, rapier_dynamics2d,
rapier2d (and rapier_geometry2d if touched); snapshots with `snforge test -p <crate> --tracked-resource sierra-gas >
gas-<crate>.log` and `python3 scripts/gas.py snapshot --filter <crate>::<module> --from-log gas-<crate>.log`; exact
steps with `--detailed-resources --tracked-resource cairo-steps`. Never a workspace-wide run: CI is the full gate.
Rebase on `origin/main` before the PR. Commit each kept lever separately; push; `gh pr create --base main --title
"<what ships>" --body-file …`; wait for the checks to be registered, then `gh pr checks --watch` until green; never
merge; `REPORT.md` (Summary · Profile · Levers kept / rejected · Before / after: impact / flight / load / all-asleep L10
/ L20, P3, activation reads · `WorldState` version · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
