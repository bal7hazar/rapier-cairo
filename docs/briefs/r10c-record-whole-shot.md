# R10c — add FU1's whole-shot steps to the alpha.10 release record

Lot id `r10c-record-wholeshot`. Runner: `impl-sonnet` (a docs edit with given figures), on the VPS.

Goal: replace the "Measurements still owed" section of `docs/releases/0.1.0-alpha.10.md` with "Whole-shot steps
(measured after the release)", with the WS1 figures measured on the Mac (before `62cd1b6`, after `4d8e14a`; owner's 151
and reference 107, in process and slim), compared with FU0's estimates in plain words (estimates labelled as such; the
owner's-shot excess over the estimate is a hypothesis, not a finding). Add one line to `docs/PLAN.md`'s version line
pointing to the record.

Allowlist: `docs/releases/0.1.0-alpha.10.md`, one line of `docs/PLAN.md`, this brief. A PR to `main`, CI green with
`ci-ok`, never merged by the thread.
