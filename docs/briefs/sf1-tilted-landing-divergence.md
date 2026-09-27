# SF1 — solver fidelity: a tilted landing with an off-centre centre of mass diverges from upstream

## 1. Read first
`AGENTS.md`; `docs/adr/0001-upstream-divergences.md` (every entry: a divergence is either a documented choice or an
upstream defect, never an unexplained drift); the precedents `docs/briefs/{dm-midpoint-anchors,sd-slope-divergence,so-stack-divergence,si-sleep-impact}.md`
and their reports (how a divergence was isolated: counterfactual upstream runs in the harness, a minimal scene, the
first diverging quantity); SH2b's report (#187): the golden scene `ell_topple` (an L-shaped compound on a half-space,
`tools/golden/src/sh2b.rs`, replay in `crates/rapier2d/tests/golden_scenes/compound.cairo`) deviates from upstream
from step 7, and **the same body built from two plain colliders reproduces the port's trajectory within 40 ulp**, so
the cause is not the compound: it is the solver / mass properties on a tilted landing with an off-centre centre of mass.
It comes to rest at upstream's rotation and height (16 / 3 ulp) but 0.0017 further along x. On `main`:
`crates/rapier_dynamics2d/src/{solver.cairo,solver/**,rigid_body*,narrow_phase/**}`, `crates/rapier2d/src/pipeline/**`.
Upstream (`UP=/home/claude/git/refs`): the solver (`dynamics/solver/**`), contact constraint construction.

## 2. Scope (file allowlist)
Investigation anywhere (throwaway copies in `/tmp`; counterfactual upstream runs in the harness). Fix: the solver /
contact-constraint / mass-property files the cause lives in, their tests, a new minimal golden scene in the harness
(`tools/golden/src/**`, existing vectors byte-identical), vectors / fixtures (additions only), and the snapshots that
move. Forbidden: the pipeline glue RG1 is changing (`crates/rapier2d/src/pipeline/**` beyond reading), geometry
kernels, `Scarb.toml`/`lib.cairo`.

## 3. Method and expected result
1. Reduce to the smallest scene that diverges (two plain colliders on one body, or one off-centre collider), print the
   first diverging quantity per substep against upstream (velocities, impulses, anchors, effective masses, the order
   of constraints).
2. Decide: a port defect (fix it, bit-identical everywhere else unless the fix is the point), an upstream defect
   (demonstrate it with a counterfactual upstream run, register it for ADR 0001 in REPORT.md, keep the port), or an
   accepted numeric divergence (quantify it and explain why).
3. If fixed: the minimal scene and `ell_topple` within bands; every other golden replay and scene test unchanged or
   within its band (list each that moves and why); P3 and level windows' Cairo steps unchanged (before / after table).

## 4. Definition of done
Harness twice → zero diff. Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through
`scripts/build-shims/`): `scarb fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on the crates
you touch; snapshots with `--from-log`; `python3 scripts/bytecode_size.py check` (or `snapshot` if code moved). Never a
workspace-wide run: CI is the full gate. Rebase on `origin/main` before the PR. Conventional commits + trailer; push;
`gh pr create --base main --title "<what ships>" --body-file …`; `gh pr checks --watch` until green; never merge;
`REPORT.md` (Summary · Minimal scene · First divergence · Cause and evidence (counterfactuals) · Fix or ADR proposal ·
Golden / scene deltas · Steps proof · Escalations · PR URL). Memory rules apply.

## 5. Work autonomously, do not ask questions, do not widen the scope.
