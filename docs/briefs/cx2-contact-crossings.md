# CX2 — compact contact and narrow-phase crossings (measurement first)

## 1. Read first
`AGENTS.md`; `docs/research/class-split.md` (CS3–CS6 sections, CX1's if present); CX1's merged work
(`crates/rapier2d_classes/src/{advance,islands}.cairo`: value-array walks, packed wires serialized straight into the
calldata, decisions replayed by the caller — the pattern to reuse); on `main`: `crates/rapier2d_classes/src/{contact,narrow_phase,config,classes*}.cairo`
(as they exist), `crates/rapier2d/src/pipeline/stages/**`, `scripts/bytecode_size.py`, `gas/bytecode.size`.
CX1's escalation: the shipped slim layout is at +46.3 % on the pile10 shot (32.82M steps vs 22.43M in process, 4
transactions ≤ 10M); what remains is the per-pair contact crossings (+3.31M), CS6's narrow-phase and active-set classes
and the broad phase (+0.43M); per-felt deserialization (≈ 20 steps per felt) dominates.

**Programme conditions (2026-09-28), all binding:**
- **Measurement first:** build the compacted contact / narrow-phase crossing on a throwaway branch or copy, measure
  `SlimSplitStep`'s size (Sierra, CASM) and the pile10 shot's steps. **Implement only if** the caller stays ≤ 73,728
  CASM **and** the shot gains **≥ 2M steps**; otherwise stop, report the measurement, open no PR.
- Every declared class ≤ 73,728 Sierra and CASM, SNIP-36 clean (`bytecode_size.py check`).
- Bit-identical to in process at every tick on the pile10 shot and on `crates/rapier2d_classes/tests/removals.cairo`
  (incl. force events every step and basic-codec save / restore mid-collapse), on the slim layout first.
- In-process users unchanged: `program.basic` 231,196, every `steps_*` probe of rapier2d and the CCD tests identical.
- Target +25 %, accepted ceiling +75 %: this lot is an optimisation, it must not change results or anything
  in-process.

## 2. Scope (file allowlist)
`crates/rapier2d_classes/**` (contact / narrow-phase classes, their strategies and wires, tests), the narrow-phase slot
traits in `crates/rapier2d/src/pipeline/stages/**` (in-process impls forwarding unchanged), `pub` / `Serde` seams in
`crates/rapier2d/src/pipeline/**`, `crates/rapier_dynamics2d/src/narrow_phase/**`, `crates/rapier_geometry2d/src/{contact,manifold}*`
(no logic change), `gas/bytecode.size`, the snapshots that move, `docs/research/class-split.md` (a CX2 section).
Forbidden: `scripts/**`, `.github/**`, `Scarb.toml` / `lib.cairo` of the engine crates, the `WorldState` bytes.

## 3. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/` — the
project lock; never two whole-shot runs at once; test-name filters while iterating): `scarb fmt --workspace`; `scarb
lint -p` / `scarb build -p` / `snforge test -p` on the crates you touch (one at a time); snapshots with `--from-log`;
`python3 scripts/bytecode_size.py snapshot` then `check`; `python3 scripts/api_parity.py --check`. Rebase on
`origin/main` before the PR. Commit wip states early. Conventional commits + trailer; push; `gh pr create --base main
--title "<what ships>" --body-file …`; `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary ·
Measurement (caller size, shot steps, go / no-go against the two thresholds) · Per-call felts and steps before / after ·
Pile10 per layout and transactions · Bit-identity · In-process unchanged proof · Proposed ADR text · Escalations · PR URL
or "no PR"). Memory rules apply.

## 4. Work autonomously, do not ask questions, do not widen the scope.
