# PX9 — close the parity table: `Compound::bvh`, `take_removed`, joint `user_data`

Runner: `impl-sonnet` (three named items, clear acceptance criteria). On the VPS, with crate-scoped builds only. Lot id
`px9-parity-close`.

## 1. Goal and context

`docs/API_PARITY.md` on `main` has **3** missing in-scope items:
- `Compound::bvh` (parry::shape);
- `ColliderSet::take_removed` (geometry);
- `GenericJointBuilder::user_data` (dynamics).

The project manager decided on 2026-10-04 how each one ends:

- **`Compound::bvh`: close it** with the existing no-BVH reason, the one `CompositeShape::bvh` already carries (ADR 36,
  SH2b: `parts_in_aabb` scans `aabbs`, which beats a tree up to about 6 parts). Write no code for it.
- **`take_removed` and `user_data`: port each one only if every existing probe shows 0 steps by the CI rule.**
  - The CI rule: CI `gas` is green with **no existing `.snap` entry changed**. Added lines for new tests are fine.
    `bytecode` is green with `gas/bytecode.size` unchanged.
  - Otherwise **close it**, with its measured cost as the reason.

Background, read first:
- WS3 (#266, closed, never merged) ported both items at head `2eade5e` of the branch
  `hp/slingfall-rapier/t-0047-ws3-dormant-pairs-codec`. Read that head's diff for these two items
  (`git show 2eade5e -- <paths>` and `git log`).
- WS3's lesson is in `docs/PLAN.md`, the "Parked" entry H. Boxed cells carried on every `ref` cost the default path. For
  `take_removed` alone that was +0.1 to +0.2 %.
- So the port must keep these items **out of everything the step carries**. Candidates:
  - for `take_removed`: a removal log that only the removal path writes, kept where the step never copies it, or an
    `Option` / `Nullable` that costs nothing when empty;
  - for `user_data`: data kept on the builder and beside the joint set, keyed by handle, not a new field of the stepped
    `GenericJoint`.
- The design is yours. Say which you chose and why.

Also read:
- `AGENTS.md`;
- `docs/adr/0001-upstream-divergences.md`;
- `scripts/api_parity.py` (how items are matched and closed), with `docs/API_PARITY.md`;
- the upstream sources: `rapier/src/geometry/collider_set.rs`, `rapier/src/dynamics/joint/generic_joint.rs` and parry
  `compound.rs`, at the pins in `tools/golden`.

## 2. Conditions

- **Results bit-identical:** no existing test's expected value is edited.
- **0 steps on existing probes, by the CI rule above.**
  - First measure each item crate-scoped on the VPS, before and after, on the step probes:
    `snforge test -p rapier2d steps_ --tracked-resource cairo-steps`, plus the `rapier_dynamics2d` tests that step
    joints and remove colliders.
  - If an item moves any existing probe or `.snap` entry, try at most one more layout.
  - If it still moves something: revert that item, close it with the measured figure as the reason ("costs +N steps
    on <probe>: …"), and record it in an ADR entry.
- **Class sizes and hashes:** `gas/bytecode.size` unchanged, and no class hash moves. If a hash moves, that item failed
  the rule: close it.
- **Closing an item:** use the parity script's existing mechanism for closed reasons (as PX7 and PX8 did). Never
  hand-edit the table. Each closed reason is one sentence with its figure.

## 3. Scope: allowlist

- **Sources:**
  - `crates/rapier_dynamics2d/src/**`, for the collider set and the joint builder or set;
  - `crates/rapier2d/src/**`, the prelude only, for re-exports;
  - inline tests may be added.
- **Parity:**
  - `scripts/api_parity.py`, only its closed-reason rules for these three items;
  - `docs/API_PARITY.md`, by the script only.
- **Generated:** `gas/**/*.snap`, by `scripts/gas.py` only, and only added lines for new tests.
- **Docs:**
  - `docs/adr/0001-upstream-divergences.md`: new entries;
  - `CHANGELOG.md`, `## Unreleased`: **required**, either "Added" for what is ported or a line saying the parity closes;
  - `docs/PLAN.md`: **one new "Parked" entry for V1**, worded below.
- **This brief:** `docs/briefs/px9-parity-close.md`, your first commit, as given.
- **Not in scope:**
  - `crates/rapier_geometry2d/**` and `crates/rapier2d_classes/**`;
  - CI, and any other script;
  - public signatures of existing items.

The V1 "Parked" entry for `docs/PLAN.md`, in the plan's existing entry format. It is the project manager's decision of
2026-10-04:
- **What V1 is:** keep the two bodies' velocities wide (Q64.64) across the rows of one solver kernel (2 to 4 rows),
  instead of narrowing after each impulse application.
- **The estimate:** FU0's rough −0.2 to −0.45M steps on the reference shot (`docs/research/fused-rescales.md` §2,
  "Ruled out or left out").
- **The prerequisite:** an unbounded Q96.96 accumulator in `fixed`, a glam-track change. The next row's `jv` would
  exceed `fixed`'s 16-term bound, and P1's product would no longer fit a felt252.
- **When it reopens:** only if the game's proofs need it after its re-pin.
- **Also out, with their figures:** C6 (1.95 ulp > 1) and C7 (out of the `box_slope_slide` band), each under about 10k
  steps on the reference shot.

## 4. Machines (VPS)

- Every Cairo build goes through the `scarb` / `snforge` shims (the heavy lock), with `RAYON_NUM_THREADS=1`.
- Run tests under `prlimit --as=8589934592 -- /usr/bin/time -v snforge test -p <crate> ...`, crate-scoped, one at a
  time.
- If a run hits the cap, report it. Never raise the cap and never run uncapped.
- Never use `--workspace`, and never run the whole-shot suites (`rapier2d_classes`, `rapier_sink`) here.

## 5. Programme rules

- **CI logs:** wait for the run to complete, then `gh run view <run-id> --log-failed` alone in its call. Never
  `gh api …/logs`.
- **Git:**
  - your own git commands run normally, as separate commands;
  - to pick up `main`, `git merge origin/main` alone in its call; never another rebase, never a force push;
  - remove an untracked file only with `git clean -f -- <exact path>`; never `-d`, `-x` or `-X`;
  - reading WS3's branch in this repository is fine (`git show`, `git log`).
- **Text:** prefer words over links to external issues. `gh pr edit` works.
- **Processes:** signal only processes you started, by the pid you recorded.
- **Refusals:** if a command is refused, put its exact text and the time in your report and stop that part.
- **Pushes:**
  - `scripts/prepush.sh` before each push;
  - one push when done, then one with the gas lines from CI if new tests need them, then one per review's fixes.

## 6. Acceptance criteria

1. The three items are each `ported` or closed in `docs/API_PARITY.md`. In scope reads 100 %.
2. For each ported item: the before / after step comparison, with its commands, and the CI rule's proof line in the
   PR: "no existing `.snap` entry changed; `gas/bytecode.size` unchanged".
3. For each closed item: its measured cost, in the reason and in an ADR entry.
4. No existing test's expected value edited.
5. The CHANGELOG line, and the V1 plan entry.
6. Every CI check green, `ci-ok` included.
7. A pull request; **never merge**.

## 7. Report

Your thread report as your rules say. Lead with:
- each item's outcome, ported or closed, with its figure;
- then the layout chosen, and the proof line.

## 8. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.
