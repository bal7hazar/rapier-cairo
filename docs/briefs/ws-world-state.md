# WS — versioned world state: save / restore (requested by the programme, spike G1b; blocking the game's G3/G4)

## 1. Read first
`AGENTS.md` (§6 crate-scoped local gate, §7 gas cost model); `docs/PLAN.md` (D9 persistent state, D7 stateless broad
phase, SL sleeping, SE sensors); the request `~/projects/pm/messages/to-rapier/2026-09-25-world-state-api.md` and the
spike report `~/projects/pm/spikes/wasm-vm/REPORT-G1b.md` (read-only: why the client runs the physics in chunks, what
broke after a removal); on `main`: `crates/rapier_core/src/data/arena.cairo` + `arena/*` (`ArenaState`, its Serde
round-trip tests), `crates/rapier_dynamics2d/src/{rigid_body_set.cairo,rigid_body_set/*,collider_set.cairo,joint/set.cairo,narrow_phase.cairo}`,
`crates/rapier2d/src/world.cairo`, `crates/rapier2d/tests/{golden_scenes.cairo,golden_scenes/*,world_step.cairo}`.

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/{rigid_body_set.cairo,rigid_body_set/**,collider_set.cairo,joint/set.cairo,narrow_phase.cairo}`
(state accessors and `Serde` derives only, no behaviour change), `crates/rapier_core/src/data/arena.cairo` + `arena/**`
only if `ArenaState` misses something, `crates/rapier2d/src/world.cairo` + a new child module `world/state.cairo`
(declared from `world.cairo`), a new test file `crates/rapier2d/tests/world_state.cairo`, and the snapshots that
move. Forbidden: solver, narrow/broad-phase logic, geometry, `Scarb.toml`/`lib.cairo` (re-exports → Escalations).

## 3. Expected API and semantics
1. `RigidBodySetTrait`, `ColliderSetTrait`, `ImpulseJointSetTrait`: `to_state(self: @…) -> ArenaState<T>` and
   `from_state(state: ArenaState<T>) -> …`, wrapping `ArenaState` (generation counter, free list, capacity, entries):
   handles issued after a restore are exactly the ones the original would issue, **including after removals**.
   Any extra field of a set (indices, caches, change lists) is either in the state or rebuilt identically.
2. `WorldState` (`Drop`, `Serde`, `PartialEq`, `Debug`), versioned: `version: u32` checked against
   `WORLD_STATE_VERSION` (= 1); it owns D9's whole persistent list — gravity, integration parameters, the three set
   states (bodies with activation / sleep state and change flags, colliders, joints with accumulated impulses), the
   narrow-phase pairs (manifolds with warm-start impulses, event status, sensor `intersecting` bits) — and every other
   field `World` holds, so that a future persistent piece (islands, broad-phase cache, CCD) is added in one place.
   `WorldTrait::to_state(self: @World) -> WorldState`; `WorldTrait::from_state(state: WorldState) -> World`, which
   panics with `'world state: version'` on a mismatch. `from_state(to_state(w)) == w` field for field.
3. RigidBody storage is whatever RB (#121) merged (hot fields + one boxed cold struct): make it round-trip through
   `Serde` (implement `Serde` for the box if the corelib has none).
4. Optional, only once 1–3 are done and green: a compact form (dynamic fields only: poses, velocities, activation,
   joint impulses, pairs; the static parts rebuilt by the level code with identical handles). Measure it against the
   full form; ship it only if it saves ≥ 40 % of the felts and steps, otherwise keep it under `#[cfg(test)]` with the
   numbers.

## 4. Golden and efficiency
The lot's acceptance test, bit for bit (every field of `World` and every event of every step):
`step^n ≡ (from_state ∘ deserialize ∘ serialize ∘ to_state ∘ step)^n` on the golden scenes, with chunk sizes
K = 1 and K = 7, **including** a scene with removals (remove a body and a collider mid-run, then insert new ones:
same handles as the uninterrupted run) and a scene where bodies fall asleep and wake up (sensor pair included).
Report, for 10- and 20-body worlds (G0-like: pile of cuboids + ball): serialized length in felts, and Sierra gas and
exact Cairo steps of `to_state`, `from_state`, `serialize`, `deserialize` (the spike measured 73–110k steps per round
trip). P3 scenes bit-identical in gas (nothing in `step` changes). ≤ 800 lines per file; ≤ 4 fuzz per module.

## 5. Tests
The round-trip equivalence above (table-driven over scenes and K); version mismatch panics with the exact message;
set-level round trips after removals (next handle equality); existing tests unchanged.

## 6. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/`):
`scarb fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on rapier_core (if touched),
rapier_dynamics2d, rapier2d; snapshots with `snforge test -p <crate> --tracked-resource sierra-gas > gas-<crate>.log`
and `python3 scripts/gas.py snapshot --filter <crate>::<module> --from-log gas-<crate>.log`; `python3
scripts/api_parity.py --check`. Never a workspace-wide run: CI is the full gate. Conventional commits + trailer;
push; `gh pr create --base main --title "<what ships>" --body-file …`; wait for the checks to be registered, then
`gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · API · State layout and version policy ·
Round-trip results · Size | gas | steps table · Compact form verdict · Requested re-exports · Escalations · PR URL).
Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
