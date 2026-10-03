# SC2 — study of the parked API families: what opt-in API they allow, and at what cost

Runner: `impl-opus` (design and measurement), on the VPS (no whole-shot suite needed). Lot id `sc2-parked-families`. **Research only: no library
change is committed.**

## 1. Goal and context

The programme now focuses on the Cairo primitives and their API coverage (the owner, 2026-10-03). Four families of
upstream API were parked, because adding them moved results or Cairo steps that the game depended on. The game is now
on hold, but the library's rule stands: existing results stay bit-identical and existing step paths do not slow down.

The question, for each family: what can be added as **opt-in API**? That means new types, functions or variants that
existing users never call, so their results and steps stay as they are. And what does it cost?

The four families (39 missing in-scope items, `docs/API_PARITY.md`):
1. **The solver's scalar API** (26): `SolverBodies` (8), `SolverVel` (8), `SolverPose` (5), `SolverTransform` (2),
   `SolverPoseRepr::identity`, `SolverVelRepr::zero`, `VelocitySolver::new`. Parked item (A). Lives in
   `crates/rapier_dynamics2d/src/solver/**`, which lot EL1 is changing now: read `origin/main` and EL1's branch, and
   change nothing.
2. **Sub-shape result widening** (7): `Contact::with_subshapes`, `PointProjection::with_subshape`,
   `RayIntersection::with_subshape`, `ContactManifold::{subshape_pos1, subshape_pos2, set_subshape_pos1,
   set_subshape_pos2}`. Parked item (E), ADR 35: a field on these structs costs steps on every query.
3. **`contact_skin`** (3): `Collider::{contact_skin, set_contact_skin}`, `ColliderBuilder::contact_skin`. Parked item
   (F): a field on `Collider` and a solver change.
4. **Compound internal edges** (8): `Compound::{DEFAULT_WELD_TOLERANCE, flags, set_flags, with_flags,
   part_normal_constraints, bvh}`, `CompoundFlags`, `CompoundPseudoNormals`. Parked item (B). PX6 added
   `NormalConstraints`, pseudo-normal cones and `TrianglePseudoNormals`. The goldens are pinned to parry 0.30.2, and
   this family is in parry 0.31.

Read first: `AGENTS.md`, `docs/PLAN.md` (the parked items and their reasons), `docs/adr/0001-upstream-divergences.md`
(entries 35, 45 and the ones the families touch), `docs/API_PARITY.md`, `docs/research/impact-tick.md` (how steps are
measured), `tools/golden/` (the golden oracle and its pinned upstream versions), the upstream sources (parry 0.31.1 and
rapier 0.35.3 are in the cargo registry copy on this Mac, `~/.cargo/registry/src/index.crates.io-*/`).

## 2. What to produce

One document, `docs/research/parked-families.md`, and this brief, `docs/briefs/sc2-parked-families.md` (your first
commit, as given), in a pull request that changes nothing else. For each family:

1. **Opt-in designs.** One to three options, each named concretely. For example: a widened copy of a type beside the
   existing one, a generic or strategy parameter that defaults to today's behaviour, a separate query function, a
   builder flag that existing worlds never set. Give the exact upstream names it would port, and the Cairo signatures.
2. **Cost, measured where a prototype is cheap and estimated otherwise (called an estimate):**
   - library lines, against the package-size gate (`docs/PACKAGES.md`: `rapier_geometry2d` +42 %, `rapier_dynamics2d`
     +62 % margin);
   - the steps of the existing probes, which must stay identical (say how that is guaranteed);
   - the steps of the opt-in path itself;
   - class sizes, if the declared classes would compile it in.

   A prototype stays uncommitted in your worktree. Its figures go in the document with the command that produced
   them.
3. **Results:** prove that existing results cannot move. If an option can move them, say so and reject it.
4. **Family 4 only:** does it need the golden oracle bumped to parry 0.31? What would that bump move for the existing
   goldens? Check what `tools/golden` pins, and whether 0.31 changes any existing golden.
5. **A recommendation:** port it now (as which lot, which profile, what size), port it later, or exclude it with a
   closed reason. The project manager decides on this report.

## 3. Rules

- **Nothing committed but the two documents.** No change to `crates/**`, scripts or CI.
- **Builds and measurements:** `RAYON_NUM_THREADS=1`; crate-scoped runs; whole-shot suites with `--max-threads 2`, one
  at a time; never `snforge test --workspace`; `--tracked-resource cairo-steps` for steps.
- **Programme rules:**
  - read CI logs only with `gh run view <run-id> --log-failed`, alone in its call, after the run completes;
  - your own git commands run normally, as separate commands;
  - signal only processes you started, by their recorded pid;
  - prefer words over links to external issues in PR text and commit messages.
- One push when done, then one per review's fixes. `scripts/prepush.sh` before each push. **Never merge.**

## 4. Report

Your thread report as your rules say. Lead with one line per family: the recommendation and its cost. Then the open
questions for the project manager.

## 5. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.
