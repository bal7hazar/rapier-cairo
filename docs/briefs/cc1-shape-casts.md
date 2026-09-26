# CC1 — shape casts: linear, nonlinear and sweeps (parry's `shape_cast`, `nonlinear_shape_cast`, `sweep_toi`)

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (programme order 2026-09-26: CC first — CCD on the pebble is the guard against tunneling
through thin planks if the game drops to 2 or 1 substeps; CC1 = the casts, CC2 = the CCD solver in the step);
`docs/adr/0001-upstream-divergences.md` (entries 14, 17, 24, 26: analytic kernels instead of GJK / EPA);
`docs/API_PARITY.md` (section "WP: CCD and shape casts" and the owner tables `ShapeCastOptions`, `ShapeCastHit`,
`ShapeCastStatus`, `NonlinearRigidMotion`, `NonlinearShapeCastMode`, `Sweep*`, `QueryDispatcher`, `QueryPipeline`
(`cast_shape`, `cast_shape_nonlinear`) — CC2 owns `CCDSolver`, `RigidBodyCcd`, `ToiProxy`, the body / builder CCD
items); on `main`: `crates/rapier_geometry2d/src/{query.cairo,query/**,ray.cairo,ray/**,point/**,shape.cairo,shape/**}`
(QY1's queries, SH1's shapes), `crates/rapier2d/src/{queries.cairo,queries/**}` (QY2's `QueryPipeline`). Upstream
(`UP=/home/claude/git/refs`): `$UP/parry/src/query/{shape_cast,nonlinear_shape_cast,sweep_toi}/**`, the casts of
`$UP/parry/src/query/default_query_dispatcher.rs`, `$UP/rapier/src/pipeline/query_pipeline/**` (`cast_shape*`).

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/query.cairo` + `query/**` (new `query/shape_cast/**`, `query/nonlinear_shape_cast/**`,
`query/sweep/**` declared from `query.cairo`), `crates/rapier2d/src/{queries.cairo,queries/**}` (`cast_shape`,
`cast_shape_nonlinear` on `World` / `QueryPipeline`), their tests, the harness `tools/golden/src/**` (new families
`shape_casts`, `nonlinear_shape_casts`; existing vectors byte-identical, new cases in new generated files), vectors /
fixtures / types (additions only), `tools/golden/README.md`, `scripts/api_parity.py` (entries you justify),
`docs/API_PARITY.md` (regenerated), and the snapshots that move. Forbidden: the step (pipeline, narrow phase, solver:
CC2's scope), shapes' layout, `Scarb.toml`/`lib.cairo`.

## 3. Expected result
Upstream names and semantics for every pair of the closed shape set (ball, cuboid, capsule, segment, half-space,
convex polygon, triangle, round shapes):
- `cast_shapes(pos1, vel1, g1, pos2, vel2, g2, options: ShapeCastOptions) -> Option<ShapeCastHit>` (time of impact,
  witnesses, normals, `ShapeCastStatus`; `max_time_of_impact`, `target_distance`, `stop_at_penetration`,
  `compute_impact_geometry_on_penetration`), with parry's dispatcher cases (ball–ball analytic, half-space–support map,
  support map–support map; cheapest exact formulation per pair — conservative advancement or analytic TOI —
  measured).
- `cast_shapes_nonlinear(motion1: NonlinearRigidMotion, g1, motion2, g2, start_time, end_time, stop_at_penetration)`
  (rotating motions; conservative advancement with parry's termination rules) and `NonlinearRigidMotion` members.
- The sweep TOI of `$UP/parry/src/query/sweep_toi` if it is part of the public API used by rapier's CCD (read it: CC2
  will need what rapier's `ccd_solver` calls).
- `World::cast_shape` / `cast_shape_nonlinear` and the `QueryPipeline` versions (filters, deterministic order, as QY2).
- Every item: `ported`, `excluded` (closed reason), or `missing` with a one-line reason (composites → SH2).

## 4. Golden, determinism and efficiency
Golden families from parry2d-f64 0.30.2 (the vendored copy): every supported pair × {hit, miss, touching at start,
penetrating at start, grazing} × {linear, rotating}, strict or within documented bands (TOI within a stated number of
ulp of the reference time, witnesses on both surfaces). Iterative methods: bounded iteration counts, deterministic
results, no data-dependent early exits that differ from parry's except where documented. Measure Sierra gas and exact
Cairo steps per cast kind (the step is untouched: P3, level probes and `gas/bytecode.size` unchanged). ≤ 800 lines per
file; ≤ 4 fuzz per module.

## 5. Tests
The golden families; table-driven unit tests (symmetry, `max_time_of_impact` cut, `target_distance`, penetration
flags); existing tests unchanged; `python3 scripts/api_parity.py --check`.

## 6. Definition of done
Harness twice → zero diff. Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through
`scripts/build-shims/`): `scarb fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on
rapier_geometry2d, rapier_golden and rapier2d (one at a time); snapshots with `snforge test -p <crate>
--tracked-resource sierra-gas > gas-<crate>.log` and `python3 scripts/gas.py snapshot --filter <crate>::<module>
--from-log gas-<crate>.log`; `python3 scripts/api_parity.py` then `--check`; `python3 scripts/bytecode_size.py check`.
Never a workspace-wide run: CI is the full gate. Rebase on `origin/main` before the PR. Conventional commits + trailer;
push; `gh pr create --base main --title "<what ships>" --body-file …`; wait for the checks to be registered, then `gh
pr checks --watch` until green; never merge; `REPORT.md` (Summary · Items closed / excluded / left · API · Golden
results and bands · Gas | steps per cast kind · Deviations · What CC2 can call · Requested re-exports · Escalations ·
PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
