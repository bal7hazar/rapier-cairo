# DEP5 — move to `fixed` 0.5.0 and `glam_core` 0.5.0

Runner: `impl-sonnet` (a dependency bump with a fixed proof). On the VPS. Lot id `dep5-fixed-glam-05`.

## 1. Goal and context

Rapier depends on `fixed` "0.4.0" and `glam_core` "0.4.1" (the workspace `Scarb.toml`; every crate takes
`.workspace = true`). The glam track has published both 0.5.0 releases on scarbs.xyz:

- `fixed` 0.5.0 is a pure addition (`sinh_cosh`, `asinh`, `acosh`, `atanh`): no 0.4.0 result changes.
- `glam_core` 0.5.0 (read back `sha256:a5e6e4863acfc4be492e06f8f965a16982b8a8975a5179a903d64092a9f05975`) depends on
  `fixed ^0.5.0`. It brings:
  - additions: `Sum` / `Product`, `from_span` / `write_to`, `map`;
  - one MINOR result change, in `Quat::to_axis_angle` / `to_scaled_axis`. Rapier is 2D and does not call either;
  - the **experimental feature `associated_item_constraints`**, enabled inside `glam_core`.

The goal: the whole stack resolves on one `fixed`. The project manager approved this on 2026-10-04.

## 2. Step 1, first and alone: the experimental feature

Before any bump is committed, find out whether a **consumer** of `glam_core` 0.5.0 must enable
`associated_item_constraints` in its own manifest.

- In a scratch copy, change the two workspace requirements to `fixed = "0.5.0"` and `glam_core = "0.5.0"`.
- Run `scarb build -p rapier_math` through the shim, under the cap. Then `scarb build -p rapier2d`.
- If the build needs the feature enabled in rapier's own `Scarb.toml`: stop, commit nothing, report the exact error and
  the manifest line that fixes it. The project manager decides first.
- If the build passes with rapier's manifests unchanged apart from the two versions, go on to step 2. Quote the build
  output's last lines in the PR.

## 3. Step 2: the bump

- In the workspace `Scarb.toml`, set `fixed = "0.5.0"` and `glam_core = "0.5.0"`.
- Update `Scarb.lock` through Scarb, never by hand.
- **Expected: bit-identical.** The proof is the CI rule: every golden, digest and `*_bit_identical` test passes
  unchanged; `gas` is green with no existing `.snap` entry changed; `bytecode` is green with `gas/bytecode.size`
  unchanged.
- **Class hashes:** if the CI2 gate shows they moved, re-pin them from that run's `class-hashes` artifact, with CI's
  root path and the run id beside them. List each moved hash in the PR. Take `gas/bytecode.size` and
  `docs/PACKAGES.md` from CI's artifacts only if they move.
- **If any result or step moves:** stop, push what you have, and report it under Escalations.
- `CHANGELOG.md`, `## Unreleased`, "Changed": one line naming the two versions and saying no result changes (with the
  CI proof), plus the moved class hashes if any.

## 4. Scope: allowlist

- the workspace `Scarb.toml` (the two requirements) and `Scarb.lock`;
- `CHANGELOG.md`;
- if, and only if, CI shows they moved: the hash constants of `crates/rapier2d_classes/tests/hashes.cairo`,
  `gas/bytecode.size` and `docs/PACKAGES.md`, all from CI's artifacts;
- this brief, `docs/briefs/dep5-fixed-glam-05.md`.

## 5. Acceptance criteria

1. Step 1's answer, with its build output.
2. One PR, with the two versions and the lock, and the CI proof line: "no existing `.snap` entry changed;
   `gas/bytecode.size` unchanged; every golden and digest passes unchanged".
3. Moved class hashes, if any, re-pinned from CI and listed.
4. Every CI check green, `ci-ok` included.
5. Never merge.
