# BT2 — Cairo steps of sleeping bodies: the flight tick and the all-asleep tick

## 1. Read first
`AGENTS.md` (§7 gas cost model); `docs/PLAN.md` (D7 stateless broad phase, D8 determinism and pair order, D9 persistent
state, SL sleeping, "Cost of a level", BT row); `docs/BUDGETS.md` ("Cost of a level": stage shares);
`docs/adr/0001-upstream-divergences.md` (entries 6–9: islands, wake-ups); G0's probes
`crates/rapier2d/tests/level_budget.cairo`; WS's `crates/rapier2d/src/world/state.cairo` and
`crates/rapier2d/tests/world_state.cairo`; on `main`: `crates/rapier2d/src/{pipeline.cairo,pipeline/**,world.cairo}`,
`crates/rapier_geometry2d/src/{broad_phase.cairo,broad_phase/**}`, `crates/rapier_dynamics2d/src/{collider_set.cairo,rigid_body_set.cairo}`;
the earlier briefs `docs/briefs/{op-pipeline-per-body,bg-broad-phase-grid,bs-broad-phase-scale,sc-sleep-timer-cost}.md`.
Upstream (`UP=/home/claude/git/refs`): `$UP/rapier/src/dynamics/island_manager/*` (active sets), the broad phase's
handling of sleeping colliders, `$UP/rapier/src/pipeline/user_changes.rs` (modified lists).

**Why:** in a level the structure sleeps before the shot and most of a shot's ticks have few awake bodies. G0: a
level-10 flight tick (pebble in the air, structure asleep) costs 6.59M gas / 61,042 steps, of which broad phase 38 %,
islands 31 %, user changes 19 %, solver 9 %; an all-asleep tick still costs 3.37M gas (level 10) and 8.14M (level
20). Every stage walks every body and collider each step (D7: nothing persists between steps but D9's list). Target:
**a sleeping body costs ≈ nothing per tick** — the tick's cost scales with the awake bodies and their contacts.
**Cairo steps are the measure** (Sierra gas reported next to them).

## 2. Scope (file allowlist)
`crates/rapier2d/src/{pipeline.cairo,pipeline/**,world.cairo,world/**}`, `crates/rapier_geometry2d/src/{broad_phase.cairo,broad_phase/**}`,
`crates/rapier_dynamics2d/src/{collider_set.cairo,rigid_body_set.cairo,rigid_body_set/**}` (bookkeeping only),
their tests and benches, new probes in `crates/rapier2d/tests/level_budget.cairo`, and the snapshots that move.
Forbidden: solver, narrow-phase contact computation, contact generators (BT1 works there in parallel), frozen
interface types, `Scarb.toml`/`lib.cairo`.

## 3. Method and constraints
1. **Profile first** the flight and all-asleep ticks of levels 10 and 20 per stage (user changes, proxies, pair
   finding, narrow-phase bookkeeping of dormant pairs, islands, sleep timers, advance), exact steps.
2. **Levers** (the profile decides): walk only awake / changed bodies (a persistent awake or modified set, as
   upstream's island manager and modified lists do); keep the broad-phase proxies of sleeping colliders from one step
   to the next instead of rebuilding them (they do not move) and test only awake proxies against them; skip islands
   work for bodies that cannot wake; make dormant pairs cost nothing until a partner wakes.
3. **Persistent state is allowed** where it pays (it amends D7 / D9: say which): everything that persists goes into
   `WorldState` with a `WORLD_STATE_VERSION` bump, and WS's chunked round-trip tests must still pass bit for bit.
4. **Results bit-identical**: the same pairs, in the same order (D8: pair order drives the solver), the same events,
   the same wake-ups, on every golden and scene test, including the level replays and the sleep / sensor scenes.
5. No benchmark-shaped wins: measure on levels 10 and 20 (flight, all-asleep, impact, load windows) and on P3
   (`free_fall*`, `balls_halfspace*`, `cuboid_stack*`, `mixed_pile8`); the impact tick must not regress beyond +1 %;
   P3 may only go down (lower their ceilings to +10 % of the new gross values). Do not run G0's `#[ignore]`d whole
   replays locally (≈ 15 GB each).

## 4. Targets
Level-10 all-asleep tick **≤ 10 % of today** (≤ 0.34M gas); level-10 flight tick **≤ 40 % of today**; the per-tick
cost of the level-20 all-asleep tick within 1.5× level 10's (it is 2.4× today). Report per-unit costs: per sleeping
body per tick, per awake body per tick.

## 5. Tests
Existing tests unchanged; WS round trips with the new persistent state; a randomized equivalence test of the new
bookkeeping against the stateless reference (bodies falling asleep, waking through contacts, user changes, removals,
sensors) — raw equality of pairs, events and bodies every step; ≤ 800 lines per file; ≤ 4 fuzz per module.

## 6. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/`):
`scarb fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on rapier_geometry2d,
rapier_dynamics2d and rapier2d; snapshots with `snforge test -p <crate> --tracked-resource sierra-gas > gas-<crate>.log`
and `python3 scripts/gas.py snapshot --filter <crate>::<module> --from-log gas-<crate>.log`; exact steps with
`--detailed-resources --tracked-resource cairo-steps`. Never a workspace-wide run: CI is the full gate. Commit each
kept lever separately (conventional commits + trailer); push; `gh pr create --base main --title "<what ships>"
--body-file …`; wait for the checks to be registered, then `gh pr checks --watch` until green; never merge;
`REPORT.md` (Summary · Profile table · Levers kept / rejected · Before / after: flight, all-asleep, impact, load for
L10 / L20, P3, per-unit costs · Persistent state added and `WorldState` version · D7 / D9 amendments · Escalations ·
PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
