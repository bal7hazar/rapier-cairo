# CS5 — strategy slots so the caller's remaining stages can leave it (class split, step 2)

## 1. Read first
`AGENTS.md` (§6, §7 incl. the game-shaped probes); `docs/research/class-split.md` (CS3: §2 size by phase, §5 the
caller's floor — 162,902 CASM in the 4-class layout, 114,083 with free_path / non-basic arms off and mass, islands,
broad phase out, **86,357** with the fused solve-and-advance and the narrow-phase loop out too; §4 the derived saving of
a batched narrow phase, ≈ −1.8M steps on the pile10 shot); CS4's crate `crates/rapier2d_classes` (`ClassHashes`,
`FamilyDispatcher<H>`, `LibraryCallSolver<H>`, `SplitStepConfig<H>`, the declared-class guard and the SNIP-36 checks in
`scripts/bytecode_size.py`), its tests (pile10 bit-identity on stored hashes, the `#[ignore]` constant-hash probes);
`docs/adr/0001-upstream-divergences.md` entries 37 and 39; the throwaway lever script `scripts/cs3_levers.py` on
`origin/proto/cs3-phase-dispatch`; on `main`: `crates/rapier2d/src/{pipeline.cairo,pipeline/**,world.cairo}`,
`crates/rapier_dynamics2d/src/{narrow_phase.cairo,narrow_phase/**,solver/**,broad_phase*,mass*,island*}` (as they
exist).

**Programme go (2026-09-28), conditions:** every new slot defaults to the in-process implementation; results
bit-identical on every step probe and on the 151-tick pile10 shot; `program.basic` and the Cairo steps of the
`game_path` probes (and every other step probe) unchanged for in-process users; the batched narrow phase is measured
both ways (in process and across the call) before it becomes a default anywhere; SNIP-36 limits (no deploy /
replace_class / get_block_hash / meta_tx_v0, no ecdsa / range_check96 / add_mod / mul_mod builtins, no panic on a valid
level's path) in every declared class.

## 2. Scope (file allowlist)
- `crates/rapier2d/src/pipeline/**` and `crates/rapier2d/src/world.cairo`: new associated items of `StepConfig` (one
  slot per stage: fused solve-and-advance, narrow phase — per pair and batched per family, islands / sleep, broad phase,
  mass properties), their in-process defaults wired into `DefaultStepConfig` and `BasicStepConfig`.
- `crates/rapier_dynamics2d/src/**`: only the seams the slots need (inputs / outputs as values with `Serde`, `pub`
  where a class must call a stage), no logic change.
- `crates/rapier2d_classes/**`: the stage classes (each ≤ 73,728 Sierra and CASM felts, in `DECLARED`), their
  library-calling strategies, `SplitStepConfig<H>` extended, the tests (bit-identity on the pile10 shot; per-call
  steps; `#[ignore]` constant-hash probes).
- `crates/rapier_sink/**` (caller fixtures), `scripts/bytecode_size.py`, `gas/bytecode.size`, the snapshots that move.
- Forbidden: any change of results or in-process steps; `Scarb.toml` / `lib.cairo` of the engine crates (new
  re-exports → Escalations); `.github/**`.

## 3. Expected result
1. A contract caller with every stage out through `SplitStepConfig<H>`: its size (Sierra, CASM, bytes) against the
   CS3 floor (86,357 CASM), and each stage class's size; every declared class ≤ 73,728.
2. Pile10 shot: bit-identical per tick; total steps and Δ vs in process for (a) CS4's layout, (b) every stage out with a
   per-pair narrow phase, (c) the same with the batched narrow phase — and the batched narrow phase **in process** vs
   per pair (steps both ways). Per-call steps per class; ticks per transaction under 10M.
3. The **CS6 candidate list**: what still keeps the caller above 73,728, each candidate with its measured or estimated
   CASM saving and its step cost (throwaway builds as CS3 §5): e.g. free_path off, non-basic shape arms out of
   `BasicStepConfig`, one step skeleton, a basic-only `WorldState` codec (**ranked last**: changing the serialized
   bytes is a breaking `WorldState` v4 for the game).
4. Proposed ADR text for the new Cairo-only seams.

## 4. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/`): `scarb
fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on the crates you touch (one at a time: rapier_dynamics2d,
rapier2d, rapier2d_classes, rapier_sink); snapshots with `--from-log`; `python3 scripts/bytecode_size.py snapshot` then
`check`; `python3 scripts/api_parity.py --check`. Step-probe proof: every `steps_*` probe of rapier2d (game_path, P3,
levels, sleep) and the CCD tests before / after, identical. Never a workspace-wide run. Rebase on `origin/main` before
the PR. Commit wip states early. Conventional commits + trailer; push; `gh pr create --base main --title "<what ships>"
--body-file …`; `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · API · Class sizes (incl.
builtins) · Pile10 bit-identity and steps · Batched narrow phase both ways · In-process unchanged proof · CS6 candidate
list · Proposed ADR text · Escalations · PR URL). Memory rules apply.

## 5. Work autonomously, do not ask questions, do not widen the scope.
