# DM — contact constraint lever arms from upstream's common midpoint (SD's root cause 2)

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D8, wave-5 SD finding); `docs/interfaces/geometry-dynamics.md` §3
(`SolverContact.anchor1/2` are world-space offsets from each body's COM — frozen, do not change);
`tools/golden/README.md` "scenes" diagnostics section written by SD (raw values at step 3 of the slope
scenes, the tangent inverse effective mass 1717986916 raw in the port vs 1708349407 upstream, the
counterfactual that recovers stick/slide velocities to < 700 ulp); on `main`:
`crates/rapier2d/tests/golden_scenes/slope_diagnostics.cairo` (SD's counterfactual: the exact
midpoint arithmetic that passes, in test code — port it into the engine, do not copy the f64
feature-id emulation, which is a test-only artefact), `crates/rapier_dynamics2d/src/solver/contact.cairo`
(`generate_element`, the `coefficients(dir, sc.anchor1, sc.anchor2, b1, b2)` calls and the
`wp1`/`wp2`/`local_p1`/`local_p2`/`dist` block), `crates/rapier_dynamics2d/src/narrow_phase.cairo`
(`solver_contact`: `anchor1 = world1 - co1.world_com`, `anchor2 = world2 - co2.world_com`).
Upstream (`UP=/home/claude/git/refs`): `$UP/rapier/src/geometry/narrow_phase/pair_update.rs` lines
~655–700 ("Localize solver contacts": `shift = (anchor2 - anchor1)·normal - dist`, `p1 = anchor1 +
normal * shift`, `point = (p1 + anchor2) * 0.5`, `solver_dp1/2 = point - com`), and
`$UP/rapier/src/dynamics/solver/contact_constraint/two_body_constraint.rs` (how `solver_dp1/2`
feed `gcross`, and how `local_p1/2` / `dist` are derived).

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/solver/contact.cairo` (+ `src/solver/contact/*.cairo`),
`crates/rapier_dynamics2d/tests/contact_solver_scenes.cairo`, and the snapshots that legitimately
drift because the constraint numbers change: `gas/rapier_dynamics2d/solver.snap`,
`gas/rapier_dynamics2d_integrationtest/{contact_solver_scenes,substep_scenes}.snap`,
`gas/rapier2d/{pipeline,world}.snap`, `gas/rapier2d_integrationtest/{world_step,golden_scenes,gas_scenes}.snap`
(regenerate with the module filters only after `gas.py check` names them). Everything else is
forbidden: the frozen `SolverContact` layout stays (the narrow phase keeps producing per-surface
anchors; the midpoint is computed where upstream computes its frozen arms, i.e. when the constraint
element is generated), `golden_scenes.cairo` stays `#[ignore]`d for the slopes (lot GS un-ignores
them against corrected traces).

## 3. Expected change and semantics
In `generate_element` (and wherever the tangent/normal `coefficients` are computed from anchors):
reconstruct the world points from the anchors and the original COMs, compute upstream's `p1` (anchor
1 shifted along the normal so that the pair is `dist` apart) and the common midpoint
`point = (p1 + wp2) / 2`, and use `point - com1` / `point - com2` as the lever arms of BOTH the normal
and tangent coefficients; derive `local_p1/2` by inverse-transforming that same midpoint; keep `dist`
as upstream keeps it. Halving = `* HALF` (never `/ 2`), all in one fused wide expression per output
where possible. Every other contact-solver semantic is unchanged (warm start, bias, restitution seed,
friction, `NEW_CONTACT_BIT`, sequential order).

## 4. Efficiency and variants
Measure the constraint generation probe before/after (Sierra gas and Cairo steps); the midpoint costs
a few adds and one halving per point — report the delta. If two formulations are plausible (midpoint
in world space vs in each body frame), bench both, ship the cheaper, keep the loser under `mod
alternatives`.

## 5. Tests
Existing DC scenes must still pass (mock manifolds — adjust expected numbers only where the midpoint
legitimately changes them, and say which). Add a table-driven test reproducing SD's step-3 slope
numbers: with the fix, stick velocity errors vs the diagnostics recorded in
`rapier_golden::scenes` (SD's new fields) must be ≤ 1 000 ulp (SD measured 673/599/559) and slide ≤
2 000 ulp. `fuzz_*` ≤ 4, ≤ 800 lines per file.

## 6. Definition of done
Foreground gate; `python3 scripts/gas.py check` then the snapshot filters of the modules it names
(within the allowlist); conventional commits + trailer; push; `gh pr create` per template; `gh pr
checks --watch` until green; never merge; `REPORT.md` (Summary · API · Gas table before/after ·
Deviations · Deferred · Requested re-exports · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
