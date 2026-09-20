Work package: G2 — leaf-level golden vectors for pose algebra and the narrow-phase building blocks
Goal: extend `tools/golden` (Rust, runs the real Parry/Rapier in f64) with vector families that validate the *internal* functions the Cairo port will implement in waves 2–3, not only the end results: 2D pose algebra, AABB overlap sets, SAT separating axes, segment clipping, point projection and segment–segment closest points. Emit them as Cairo `const` fixtures in `crates/rapier_golden` like the existing families.

Files owned:
- `tools/golden/src/*.rs` (add new modules; `main.rs` wiring; `cairo.rs` emitters), `tools/golden/README.md` (add the families; keep existing text), `tools/golden/vectors/*.json` (new files only)
- `crates/rapier_golden/src/types.cairo` (add types; never change existing ones), `crates/rapier_golden/src/generated.cairo` (declare new modules), `crates/rapier_golden/src/generated/<family>.cairo` (generated), `crates/rapier_golden/src/lib.cairo` (re-exports only), `crates/rapier_golden/tests/<family>.cairo` (sanity tests)
Do not change pinned Rust versions, existing fixture output (diff on existing generated files must be empty), any `Scarb.toml`, CI.

Frozen interfaces: existing `rapier_golden::types` (`Vec2Raw`, `RotRaw`, `PoseRaw`, `ShapeRaw`, …), the quantisation rule of `tools/golden/src/q.rs` (inputs exactly representable in Q32.32; outputs as f64 and raw `i64` round-to-nearest), the `const` + `ALL` + `cases()` fixture pattern, `scarb fmt` as the generator's last step, idempotent regeneration (CI job `golden` enforces zero diff).

Upstream reference: `parry2d-f64` and `rapier2d-f64` as pinned in `tools/golden/Cargo.toml`; source clones at /private/tmp/claude-501/-Users-bal7hazar-git-rapier-cairo--claude-worktrees-rapier-physics-cairo-benchmark-2d3317/239b7c62-97d7-47ea-9d79-74f81363555f/scratchpad/refs/{parry,rapier} (note: the clone is parry 0.31, the pinned crate is 0.30.2 — use the published crate's API, consult the clone for algorithms). Public entry points to prefer: `parry2d_f64::math::{Pose2, Rot2, Vec2}` (or the glam/glamx types they alias), `bounding_volume::Aabb::{intersects, merged, …}`, `query::sat::*` (check which functions are `pub`), `query::clip::*`, `query::PointQuery::project_local_point`, `query::closest_points::closest_points_segment_segment`, `shape::Segment::project_local_point_and_get_location`. If a needed function is private, say so in the README and derive the vector from the nearest public API.

Families (stable string ids per case; cover regular and degenerate inputs):
1. `pose2`: pairs of poses with exactly-representable translations and rotations given as `(re, im)` **as passed** (record the normalised value upstream actually uses, if it normalises) → `mul`, `inverse`, `inv_mul` (`pos12`), `transform_point`, `inverse_transform_point`, `transform_vector` on a few points; plus rotation `mul`, `inverse`, and a chain of 1 000 small-rotation multiplications reporting `re² + im²` drift (f64) to compare with the Cairo drift study of M2.
2. `aabb_overlap`: sets of 8–32 AABBs (some static, some dynamic, some touching exactly on an edge, some nested) → the sorted list of overlapping index pairs as upstream's `Aabb::intersects` decides (document the closed/open boundary convention it uses).
3. `sat2d`: cuboid–cuboid and cuboid–segment/triangle configurations → the local separating axis and separation distance returned by Parry's SAT helpers (both directions), over the 6 regimes used by the manifold family (separated, within prediction, touching, shallow, deep, degenerate: parallel edges, corner–corner).
4. `clip2d`: segment-vs-segment clipping cases (the polygon-feature clipping Parry uses for cuboid manifolds) → clipped points and their feature ids; include parallel, collinear, disjoint and single-point results.
5. `point_projection`: project points onto ball, cuboid, capsule, segment (inside, outside, on boundary, on a vertex, on an edge extension) → projected point, `is_inside`, and the feature/location upstream reports.
6. `segment_segment`: closest points between segment pairs (crossing, parallel, collinear overlapping, endpoint-to-interior, degenerate zero-length) → the two closest points and the squared distance.

Requirements:
- Sanity tests in Cairo per family (e.g. overlap symmetry, `inv_mul` consistency, projection of an inside point is itself, closest-point distance is symmetric).
- README: per family, the upstream functions called, tolerances recommended for the Cairo port and why, any ambiguity found (tie-breaking in SAT axes, feature-id conventions, boundary conventions).
- Keep total generated Cairo under ~6 000 lines: prefer fewer, well-chosen cases over sweeps.

Acceptance: `cargo run --release --locked` twice → zero git diff; from the repo root `scarb fmt --check --workspace`, `scarb lint --workspace --deny-warnings`, `scarb build --workspace`, `snforge test --workspace` pass; `python3 scripts/gas.py diff` table (new tests only) in the report.

Out of scope: any engine code, changing existing families, CI, `Scarb.toml`.
