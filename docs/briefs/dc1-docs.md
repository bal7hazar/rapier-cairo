# DC1 — the track's documents after TC1, IT1, CX3 and PP1

Runner: `impl-sonnet`, on the VPS (documents and Python, no Cairo build). Lot id `dc1-docs`.

## 1. Goal and context

Bring the track's documents up to date with what merged on 2026-10-02, and fix two known script issues.
- **Merged on 2026-10-02:**
  - #249 IT1 step 1, the impact-tick research;
  - #250 TC1, toolchain Scarb 2.20.1 / snforge 0.64.0;
  - #251 / #252 PP1, the pre-push check, download retries and PR-only cancel;
  - #253 CX3, the slim crossings.
- **The organisation's change:** it now runs in herdr. The orchestrator is a herdr coordinator, and implementers,
  reviewers and auditors are herdr threads. Nexus, its `nexus` commands and the old launcher
  (`scripts/executor-unit.sh`, units `rapier-exec-*`) are no longer used, and Codex is not used. The programme's
  operating document `slingfall/OPERATIONS.md` (already updated) describes this.

Read first: `AGENTS.md`, `docs/ORCHESTRATOR.md`, `docs/PLAN.md` (top status section), `CHANGELOG.md`,
`scripts/executor/system-prompt.md`, `scripts/bytecode_size.py`, `scripts/prepush.sh`, `docs/BUDGETS.md` (the TC1 and
CX3 sections), `docs/research/impact-tick.md`. Read `/home/claude/projects/slingfall/OPERATIONS.md` (read only; it is
another repository: do not copy it, refer to it as `slingfall/OPERATIONS.md`).

## 2. Scope: file allowlist

`CHANGELOG.md`, `docs/PLAN.md` (the status line and the top "Status of the track" section only; the history in the
status line gains one entry), `docs/ORCHESTRATOR.md`, `AGENTS.md` (§3 Roles, "Launching executors", and the brief's
verification wording only), `scripts/executor/system-prompt.md`, `scripts/bytecode_size.py` (the two fixes below only),
`scripts/prepush.sh` (the header comment only), this brief `docs/briefs/dc1-docs.md` (your first commit, as given).
Everything else: Escalations.

## 3. Changes

1. **`CHANGELOG.md`, `## Unreleased`:**
   - toolchain Scarb 2.20.1 (Cairo 2.20.0) / starknet-foundry 0.64.0 (TC1): results bit-identical, steps +1.25 to
     +1.84 %, class hashes re-pinned from CI;
   - CX3: `NarrowPhaseClass` runs its own pair loop. The slim layout's whole shot is −3.53 % (owner's shot) and
     −3.85 % (reference shot), bit-identical; a game re-pins `NarrowPhaseClass` only;
   - CI: the scarb / snforge download retries and the PR-only cancel of superseded runs; `scripts/prepush.sh` with its
     hook.

   Take the figures from `docs/BUDGETS.md`, never invent them.
2. **`docs/PLAN.md`:** status line v2.98 (2026-10-02), and a short "Status of the track" section:
   - **the orchestrator:** the herdr coordinator of the project `slingfall-rapier`, successor of the Nexus session,
     2026-10-02;
   - **what merged today:** the PR numbers above, one line each;
   - **published:** unchanged, `0.1.0-alpha.8`;
   - **`main` since alpha.8:** add TC1 and CX3;
   - **held:** EL1, the in-scope engine levers of `docs/research/impact-tick.md` §4, held until the owner's
     physics-rate study reports;
   - **parked items:** unchanged.

   Keep the rest of the file as it is.
3. **`docs/ORCHESTRATOR.md` and `AGENTS.md` (§3 and "Launching executors"):** replace the Nexus and launcher
   procedure with the herdr one, in the existing style and brevity:
   - a task is a herdr thread started by the orchestrator, with a profile (`impl-sonnet` by default, `impl-opus` for
     the step path and numerics), on the machine `machine-capacity` and the programme's placement rule give;
   - every PR gets a review thread on another model than its writer (`review` for Opus or Fable, `review-opus` for
     Sonnet), and an audit only as the exception;
   - the thread opens its own PR and never merges;
   - the build-path rule: class hashes, Sierra bytes and `gas/bytecode.size` come from CI's artefacts
     (`bytecode-snapshot`, `class-hashes`) with CI's root path recorded; felt counts, CASM, gas and steps are
     path-free;
   - `scripts/prepush.sh` before every push.

   Keep the rapier-specific content (model tiers with rapier examples, conflict-free parallelism, closing a lot,
   releases). In "Releases", say that publishing follows the standard's "Publishing a package": a go from the
   project manager per package, naming package, version, commit and archive sha256; the orchestrator publishes each
   with `scarb publish -p <package>` by hand; `scripts/release.sh publish`, which publishes all six at once, is not
   used for a publication. Remove the report archive under `~/orchestrator/logs/` and the `nexus` commands.
4. **`scripts/executor/system-prompt.md`:** mark it as historical (the old launcher's frame, not used since
   2026-10-02) in one line at the top. Do not delete it.
5. **`AGENTS.md` §3 verification wording, owed since 2026-10-01:** an implementer may run `asdf install <tool>
   <version>` and `asdf plugin add` (user-local); a system package stays the owner's.
6. **`scripts/bytecode_size.py`:**
   - the temporary packages it generates (around lines 389 and 432) pin `cairo_execute` / `starknet` 2.19.4: take the
     version from the workspace instead (the root `Scarb.toml` or `.tool-versions`), not a new hardcoded number;
   - it fails under asdf with "No version is set for command scarb" because those packages are built outside the
     repository: make the temporary directory resolve the repository's `.tool-versions` (e.g. copy it in, or set
     `ASDF_SCARB_VERSION` from it), and say which.

   Show with a light command that the generated manifests now read 2.20.0. **Do not run `bytecode_size.py
   table|check|snapshot` here:** it is a heavy build, and its figures come from CI. CI's `bytecode` job is the check.
7. **`scripts/prepush.sh`:** the header comment at lines 21-22 says a warning is printed when `scarb` is not the shim.
   Since #251 it skips the compile with the busy line instead: fix the comment only.

## 4. Acceptance criteria

1. Each change of §3 made, in the files of §2 only (`git diff --stat origin/main`).
2. `scripts/prepush.sh` passes before each push. Run it as `scripts/prepush.sh` and push with
   `git -c core.hooksPath=.githooks push`; write no git config.
3. Every CI check green, `bytecode` included: it shows that `bytecode_size.py` still produces the same sizes.
4. A pull request; **never merge**. Prefer words over links to external issues in PR text and commit messages.
5. Push rarely (CI is starved): one push when done, then one per review's fixes.

## 5. Report

Your thread report as your rules say: each §3 item with what changed, the light command of item 6, the CI result.

## 6. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.
