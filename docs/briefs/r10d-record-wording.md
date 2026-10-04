# R10d — the alpha.10 record: say the sleep explanation as an inference

Runner: `impl-sonnet` (a two-sentence docs fix), on the VPS. Lot id `r10d-record-wording`.

## 1. What and why

The post-merge review of #276 (c6ba881) found that `docs/releases/0.1.0-alpha.10.md`, in "Trajectory of the owner's
shot", states an inference as a finding. No per-tick steps were measured. Its first bullet also overstates what WS2
measured.

## 2. The edit

In that section, replace the first two bullets with these, word for word:

- "Contact counts, live bodies and force-event counts are identical at every tick. Apart from small pose differences
  (at most about 0.03 m at the end, entity 6), the only difference is sleep: from tick 118 the pile sleeps after FU1,
  and it never sleeps before."
- "That is 33 ticks of a sleeping pile. It is the likely cause of the owner's shot saving 4.4 to 6 times FU0's per-call
  estimate, but it is an inference, not a measurement: no per-tick step figures were taken. The reference shot's saving
  matches the estimate."

Then delete the separate "Final poses differ by at most about 0.03 m (entity 6)." bullet, since the first bullet now
carries it. Change nothing else in the file; the checksums and the rest of the record stay byte-identical.

## 3. Scope: allowlist

- `docs/releases/0.1.0-alpha.10.md`: only that section;
- this brief, `docs/briefs/r10d-record-wording.md`, your first commit, as given, word for word.

## 4. Rules

- **Git:**
  - your own git commands run normally, as separate commands;
  - `git merge origin/main` alone in its call; never a rebase, never a force push.
- **Pushes:** `scripts/prepush.sh` before the push. Push once.
- **Refusals:** if a command is refused, put its exact text and the time in your report and stop that part.

## 5. Acceptance criteria

1. One PR to `main`, touching only the allowlist.
2. CI green, `ci-ok` included.
3. **Never merge.** A short review comes first.

## 6. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.
