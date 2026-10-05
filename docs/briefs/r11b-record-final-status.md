# R11b — the alpha.11 release record, the leftover version texts, and the track's final status

Runner: `impl-sonnet` (documents in fixed shapes, with given facts), on the VPS. Lot id `r11b-record-final-status`.

## 1. Context

`0.1.0-alpha.11` is published: all six crates, read back. The annotated tag `v0.1.0-alpha.11` is on `1c79832`, and the
GitHub pre-release exists. The owner resumed the track on 2026-10-05 only to finish this version. After this PR, the
track goes idle.

## 2. What to write

### 2.1 `docs/releases/0.1.0-alpha.11.md` (new)

Write it in exactly the shape of `docs/releases/0.1.0-alpha.10.md`, up to and including its "How it was published"
section. Do not add a "Whole-shot steps" section: nothing on the step path changed.

**The release commit:**
- `1c79832dd92aa2238bc0e9754e0cd0a4381f3bd4` on `release/0.1.0-alpha.11` (kept, never merged);
- cut from `7c3c8c869672176cd0e4dda0bbdf097198731bb1`, the merge of the release PR #280;
- it removes only the six crates' `[dev-dependencies]`;
- it was landed by fast-forward (#281, review PASS, Opus), never squash-merged;
- CI: every non-test job is green on `1c79832`, and the test jobs and both sinks are green on `7c3c8c8` on `main` (run
  37299536942).

**Packages,** in the order published, all from commit `1c79832`. The sha256 is the registry read-back:

| package | sha256 | bytes |
|---|---|--:|
| rapier_math | 417b9e1a8c2718e2231765430dbff6fd75502aa49eb90a3c21f34063ead370bd | 24,741 |
| rapier_core | 99039e3e6628397d4f4cf17f0834448b285eddf6564aec608eaed91009a707ca | 59,682 |
| rapier_geometry2d | efc0d3e0a7b82653f9881e5772503d7bd5b47a52055410581b8bcd3e4b5e2f7f | 319,743 |
| rapier_dynamics2d | 8127db0eeb817522d834111ac41cf5004095168131dceacb66db7bb0c7081fb8 | 232,852 |
| rapier2d | 5b094187dfa9fadd8d98c2f59ee87e86c2f32e2afed07534357b89a8d8258290 | 201,030 |
| rapier2d_classes | 96398b62194c833a96fdb0f534eda6b0f638489035bc5c22be52e0299ce5c7af | 53,389 |

**How it was published:**
- five staged goes of the project manager, with the same commands as alpha.10;
- tag `v0.1.0-alpha.11` on `1c79832` (annotated);
- pre-release `https://github.com/bal7hazar/rapier-cairo/releases/tag/v0.1.0-alpha.11`.

**One more section, "Upgrading from 0.1.0-alpha.10",** word for word:
- "Dependencies: rapier-cairo now depends on `fixed` 0.5.0 and `glam_core` 0.5.0 (DEP5, #279). A consumer that uses
  `Fixed` or `Vec2` itself must move to `fixed` 0.5 and `glam_core` 0.5 too, or it compiles two generations of the same
  types."
- "One source break (PX9, #275): `GenericJointBuilder` gained a crate-private field, so a struct literal
  `GenericJointBuilder { data }` written outside the crate no longer compiles. Use `GenericJointBuilderTrait::new`."
- "Results: unchanged. No step result, no `WorldState` format and no class hash moves."

### 2.2 The leftover version texts

The review of #280 found these lines still naming `0.1.0-alpha.8`. Bring them to the current version:

- the status line of `README.md` (about line 6, "`0.1.0-alpha.8` on scarbs.xyz"): it becomes `0.1.0-alpha.11`;
- the "`0.1.0-alpha.8` is an **alpha**" lines, in `README.md` (about line 51) and in each `crates/*/README.md` (about
  line 13; in `rapier2d`, about line 29): they become `0.1.0-alpha.11`. Find them with
  `git grep -n "0\.1\.0-alpha\.8" -- README.md 'crates/*/README.md'`;
- leave the dependency-history sentences that name alpha.8 to alpha.10 as history; they are correct.

Also regenerate `examples/ball_drop/Scarb.lock` through Scarb in that directory (the shim, under the cap), so that its
path crates read `0.1.0-alpha.11`. Only the version lines may change.

### 2.3 `docs/PACKAGES.md`

Take it from the `consumer-cost` artifact of `main`'s run 37299536942, on `7c3c8c8`, the alpha.11 bump
(`gh run download 37299536942 -n consumer-cost`), using the table it carries. Do not run the scripts yourself. If the
artifact is missing or expired, stop and report it.

### 2.4 `docs/PLAN.md`: the final dated status

Add a section `## Final status (2026-10-05)` beside "Status at the pause", in the plan's format, and bump the version
line by one step ("alpha.11 released, final status"). Its content:

- **Published:** `0.1.0-alpha.11` on 2026-10-05, all six crates. It carries DEP5 (`fixed` 0.5.0, `glam_core` 0.5.0:
  one `fixed` in the stack) and PX9 (parity 100 % in scope). Its record is `docs/releases/0.1.0-alpha.11.md`.
- **Merged since the pause:** #278 (status at the pause), #279 (DEP5), #280 (the alpha.11 bump), #281 (the release
  commit, on `release/0.1.0-alpha.11`) and this PR.
- **Open:** no pull request and no thread. The track is idle by the owner's decision of 2026-10-05: no new lot until
  the owner says otherwise.
- **Next lot, if the track resumes:** none is planned. Parked: V1 (wide velocities, needs a Q96.96 accumulator in
  `fixed`), WS3 (the dormant-pair layout, entry H), and C6 / C7 (out).

## 3. Scope: allowlist

- `docs/releases/0.1.0-alpha.11.md` (new);
- `README.md` and `crates/*/README.md`: the alpha.8 version lines only;
- `examples/ball_drop/Scarb.lock`: the version lines, by Scarb;
- `docs/PACKAGES.md`: from the artifact;
- `docs/PLAN.md`;
- this brief, `docs/briefs/r11b-record-final-status.md`, your first commit, as given, word for word.

## 4. Rules

- Every `scarb` call goes through the shim (the heavy lock), with `RAYON_NUM_THREADS=1` and under
  `prlimit --as=8589934592`.
- **Git:**
  - before your FIRST push, `git rebase origin/main` in exactly that form, alone, is allowed;
  - after a push, `git merge origin/main`;
  - never a force push;
  - remove an untracked file only with `git clean -f -- <exact path>`.
- **CI logs:** wait for the run to complete, then `gh run view <id> --log-failed` alone.
- **Pushes:** `scripts/prepush.sh` before each push. One push, then one per review's fixes.
- **Refusals:** if a command is refused, put its exact text in your report and stop that part.

## 5. Acceptance criteria

1. One PR to `main`, touching only the allowlist.
2. Every CI check green, `ci-ok` included.
3. **Never merge.** It is reviewed first: the record holds checksums.

## 6. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.
