# CS6 — the caller class under 73,728 felts (class split, step 3: the size gate)

## 1. Read first
`AGENTS.md` (§6, §7 incl. the game-shaped probes); `docs/research/class-split.md` (CS3 §5 levers, CS4 / CS5 sections);
CS5's REPORT section "CS6 candidates" (reproduced below) and its throwaway lever script, kept at
`/home/claude/orchestrator/rapier-cairo-tools/cs5_levers.py` (not in the repo: copy it, never commit it);
`crates/rapier2d/src/{pipeline.cairo,pipeline/**,world.cairo,state*}`, `crates/rapier2d_classes/**`,
`crates/rapier_sink/**`, `scripts/bytecode_size.py`; ADR 0001 entries 37, 39, 40.

CS5's caller with every stage out (`StagesSplitStep`) is **120,402 CASM**; cumulative throwaway levers measured:
free_path off 98,088 (−5k steps); + non-basic shape arms, kinematic preparation / `atan2`, one-way filter, joint-free
codec with the same bytes 85,882 (−52k steps with free_path); + force-event collection out 84,384 (+0.4M est.); +
active-set rebuild out 81,596 (≤ +2.7M est.); + pair loop out 73,369 (+3.3M est.).

**Programme decision (2026-09-28):** size is the gate, steps are the optimisation (≤ +75 % over in process and ≤ 5
transactions of ≤ 10M steps accepted for the first end-to-end SNIP-36 shot; +25 % stays the target).

## 2. Work
0. **Undo CS5's API break at zero cost** (codex audit of CS5, answer 6): `StepConfig` goes back to its released shape
   (0.1.0-alpha.6: dispatcher, sensor / composite / joint strategies), the five stage slots move to a separate trait (e.g.
   `StageConfig`) taken by new entry points (e.g. `step_with_stages::<C, S>` and the force-event variant); the existing
   entry points instantiate the in-process stages. Prove zero cost: every `steps_*` probe identical, `program.basic`
   231,196. `SplitStepConfig` & co. in `rapier2d_classes` follow.
1. **Levers 1 and 2 first, for real** (not stubs): free_path off, the non-basic shape arms, kinematic preparation /
   `atan2`, the one-way filter and a joint-free codec **out of what `BasicStepConfig` / the split configs compile**, each
   behind the configuration (in-process `DefaultStepConfig` users keep everything). **Hard condition: the serialized
   `WorldState` bytes are unchanged** (a v4 codec is out of scope). A world that uses a removed feature is rejected by
   the configuration with a documented panic, as CS2's.
2. For the remaining ≈ 12k felts, **build and measure both routes**, keep the cheaper in steps:
   (a) CS5's levers 3–5 as stage slots (force-event collection, active-set rebuild, pair loop out);
   (b) a second orchestration class: the caller runs the first half of the step skeleton and library-calls an
   orchestrator class for the second half, the world crossing once per step — measure with the persisted `WorldState`
   codec and with a basic-only **in-memory** layout (not the persisted codec, which must not change).
3. The deliverable table, **one per layout** (in process; CS4; CS5 all out; the CS6 route kept; the other route): every
   class with Sierra / CASM felts and bytes, the pile10 shot's total steps and Δ, the transactions of ≤ 10M steps (ticks
   per transaction, steps each), and **calldata felts per transaction** (world in, inputs, message out). Put it in
   `docs/research/class-split.md` (a CS6 section; the file is in scope), after a short CS5 section drawn from CS5's
   report (`/home/claude/orchestrator/rapier-cairo-tools/cs5-report.md`).

## 3. Scope (file allowlist)
`crates/rapier2d/src/**` (configuration-level removals and new slots only; no result change), `crates/rapier2d_classes/**`
**except** `src/{advance,islands,arena}.cairo` (the solve-and-advance / islands crossing, owned by the parallel lot CX1:
read them, do not change them; if CS6 needs a change there, say so under Escalations), `crates/rapier_sink/**`,
`scripts/bytecode_size.py`, `gas/bytecode.size`, the snapshots that move, `docs/research/class-split.md`. Forbidden:
`crates/rapier_dynamics2d/src/**` and `crates/rapier_geometry2d/src/**` beyond `pub` / `Serde` seams (Escalations),
`Scarb.toml` / `lib.cairo` of the engine crates, `.github/**`, the `WorldState` bytes.

## 4. Gate (all must hold)
Caller and every declared class ≤ 73,728 Sierra **and** CASM felts (in `DECLARED`, `bytecode_size.py check` green incl.
the SNIP-36 syscall / builtin checks); bit-identical to in process on the pile10 shot at every tick, and on CX1's
removal scenario (`rapier2d_classes/tests/removals.cairo`) once it is on `main`; in-process users
unchanged: `program.basic` 231,196 (other `program.*` lines may only shrink if a removal applies to them — say which),
every `steps_*` probe of rapier2d (game_path, P3, levels, sleep) and the CCD tests identical in exact Cairo steps.
If the gate cannot be met, stop at the best measured state and say exactly what is missing.

## 5. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/`): `scarb fmt
--workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on rapier2d, rapier2d_classes, rapier_sink (one
at a time); snapshots with `--from-log`; `python3 scripts/bytecode_size.py snapshot` then `check`; `python3
scripts/api_parity.py --check`. Never a workspace-wide run. Rebase on `origin/main` before the PR (if CX1 merged first,
re-measure the tables on top of it). Commit wip states early. Conventional commits + trailer; push; `gh pr create --base
main --title "<what ships>" --body-file …`; `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · Gate
table · Levers 1–2 · Route (a) vs (b) · Per-layout tables · In-process unchanged proof · API changes (breaking?) ·
Proposed ADR text · Escalations · PR URL). Memory rules apply.

## 6. Work autonomously, do not ask questions, do not widen the scope.
