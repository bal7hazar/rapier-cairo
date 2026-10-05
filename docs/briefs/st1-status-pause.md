# ST1 — the pause status into the plan

Runner: `impl-sonnet` (copy given text into the plan), on the VPS. Lot id `st1-status-pause`.

## 1. Goal

The owner paused the programme on 2026-10-05, and the track's status was written to project memory because no thread
could start. Copy it into `docs/PLAN.md` now. That is item (1) of the owner's decision to resume, which covers only the
planned versions.

## 2. The edit

- Add a new section to `docs/PLAN.md`, in the plan's existing format, near the other status and "Parked" entries. Its
  heading is `## Status at the pause (2026-10-05)`, and its text is exactly the section below, from "The owner paused…"
  to the "Parked" bullets.
- Add one line under it: "Resumed on 2026-10-05, owner's decision, only to finish `0.1.0-alpha.11` (DEP5 + PX9)."
- Add nothing about the Mac or about DEP5's local branch. Those are internal to the orchestrator.
- Bump the plan's version line by one step (v2.101), with "status at the pause".

The status text, word for word:


The owner paused the whole Slingfall programme on 2026-10-05, libraries included: no new lot, thread, release or
publication until `hp resume`.

**Published:** `0.1.0-alpha.10`, all six crates, on 2026-10-04: FU1 (fused rescales) and CE (compound internal edges
and the parry 0.31.1 query answers). It is tagged, has a GitHub pre-release, and its record is
`docs/releases/0.1.0-alpha.10.md`, which includes the whole-shot steps measured after the release.

**Merged since alpha.9:**
- #267 FU0 (study)
- #268 B1
- #269 OB (golden oracle on parry 0.31.1)
- #270 FU1
- #271 CE
- #272 alpha.10 bump
- #273 the release commit (on `release/0.1.0-alpha.10`)
- #274, #276 and #277: the release record
- #275 PX9: the parity table closed at 100 % in scope (`Compound::bvh` and `take_removed` closed, joint `user_data`
  ported beside the joint set)

**Open:** no pull request. No thread is running.
- DEP5 had just started when the pause came, and was stopped before any commit or push.
- WS3 (#266) stays closed, parked in entry H.

**Merged on `main` after alpha.10, unreleased:** PX9 (#275).

**Next lot, on resume: DEP5.** Move to `fixed` 0.5.0 and `glam_core` 0.5.0. Both are published, and rapier depends on
no other glam crate.
- First step: check whether a consumer must enable `glam_core`'s experimental `associated_item_constraints`. If it
  must, the project manager decides before the bump.
- Expected: bit-identical, with class hashes re-pinned from CI if they move.
- Brief: `docs/briefs/dep5-fixed-glam-05.md`, to be committed by the DEP5 thread.

**Parked:**
- V1, wide velocities across a kernel's rows (entry in this plan, with the Q96.96 prerequisite);
- WS3, the dormant-pair layout (entry H);
- C6 and C7 stay out.

## 3. Scope: allowlist

- `docs/PLAN.md`;
- this brief, `docs/briefs/st1-status-pause.md`, your first commit, as given.

## 4. Rules

- `scripts/prepush.sh` before the push, and push once.
- `git merge origin/main` alone in its call; never a rebase, never a force push.
- If a command is refused, put its exact text in your report and stop that part.
- **Never merge.** The orchestrator tells you how to merge.

## 5. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.
