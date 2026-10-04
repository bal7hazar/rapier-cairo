# R10b — the release record of 0.1.0-alpha.10

Runner: `impl-sonnet` (a documented record in a fixed shape), on the VPS. Lot id `r10b-release-record-alpha10`.

## 1. Goal and context

`0.1.0-alpha.10` is published: all six crates, read back. The tag `v0.1.0-alpha.10` is pushed and the GitHub pre-release
exists. What remains is the record on `main`, in exactly the shape of `docs/releases/0.1.0-alpha.9.md` (read it first),
and `docs/PACKAGES.md`, which still says alpha.9.

## 2. What to write

1. **`docs/releases/0.1.0-alpha.10.md`**, in the shape of alpha.9's record, with these facts.

   **The release commit:**
   - `f7a91363838a99e987dffb0c77fdd168114d6911` on `release/0.1.0-alpha.10` (kept, never merged);
   - cut from `4d8e14a9a287eea3b18344b9c60f27d0f31cb735`, the merge of the release PR #272;
   - it removes only the six crates' `[dev-dependencies]`;
   - it was landed by fast-forward (#273, review PASS, Opus), never squash-merged;
   - CI: every non-test job is green on `f7a9136`, and the test jobs and both sinks are green on `4d8e14a` on `main`
     (run 37171579462).

   **Packages**, in the order published, all from commit `f7a9136`. The sha256 is the registry read-back:

   | package | sha256 | bytes |
   |---|---|--:|
   | rapier_math | 76e30030167448cc42017628e48708ca06dda5018b1bb6b978d096314d81e271 | 24,742 |
   | rapier_core | 3167bf9587303f4b0b4f8e41976842a84046658cd78d1b66880c03bdb75070a5 | 59,708 |
   | rapier_geometry2d | 950d7a75762b53b6cdb92af010f133677e4aa7e3896a9483a6563860f03436e8 | 319,653 |
   | rapier_dynamics2d | 6d4c5848f556e874400da59e7e31fb11b5d41e4a1ea830dc1880278166a1f933 | 231,505 |
   | rapier2d | 2b0056723120098634aeb1a056b1ce8ef4c375c28d7f7f4f175ae829649edccb | 200,989 |
   | rapier2d_classes | f27f9797bdd763f0315581711801bf540d7738266595720c767bb0ea9497c0b4 | 53,412 |

   **How it was published:**
   - five staged goes of the project manager, as for alpha.9, with the same commands;
   - tag `v0.1.0-alpha.10` on `f7a9136` (annotated);
   - pre-release `https://github.com/bal7hazar/rapier-cairo/releases/tag/v0.1.0-alpha.10`.

   **One more section, "Measurements still owed":**
   - FU1's whole-shot steps: `rapier2d_classes`, and the pile10 owner's and reference shots, in process and slim;
   - not taken because the Mac was draining;
   - the project manager's decision of 2026-10-04: the go did not wait for them, and they are added to this record and to
     the plan when the Mac is back.
2. **`docs/PACKAGES.md`**, taken ONLY from CI's `consumer-cost` artifact of `main`'s run 37171579462 on `4d8e14a`
   (`gh run download 37171579462 -n consumer-cost`).
   - Use the table that artifact carries, as the file's header says.
   - Do not run `consumer_cost.py` or `packages_table.py` yourself.
   - If the artifact is missing or expired, stop and report.

## 3. Scope: allowlist

- `docs/releases/0.1.0-alpha.10.md` (new);
- `docs/PACKAGES.md`;
- this brief, `docs/briefs/r10b-release-record-alpha10.md`, your first commit, as given.

Nothing else.

## 4. Programme rules

- **Git:**
  - your own git commands run normally, as separate commands;
  - `git merge origin/main` alone in its call; never a rebase, never a force push.
- **CI logs:** wait for the run to complete, then `gh run view <run-id> --log-failed` alone in its call. Never
  `gh api …/logs`.
- **Refusals:** if a command is refused, put its exact text and the time in your report and stop that part.
- **Pushes:** `scripts/prepush.sh` before each push. One push when done, then one per review's fixes.

## 5. Acceptance criteria

1. One PR to `main` touching only the allowlist.
2. Every CI check green, `ci-ok` included.
3. **Never merge.**

## 6. Report

Your thread report as your rules say: the PR, its head, and what `docs/PACKAGES.md` changed.

## 7. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.

