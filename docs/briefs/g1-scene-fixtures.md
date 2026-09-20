Work package: G1 — Cairo fixtures for the golden scene traces
Goal: `tools/golden` already emits `tools/golden/vectors/scenes.json` (6 scenes: ball drop, ball bounce, box on slope sticking and sliding, stack of 3, pendulum) but only JSON. Extend the Cairo generator so the scenes are also available as `const` fixtures in `crates/rapier_golden`, in the same style as the four existing families (`generated/aabb.cairo`, `generated/mass_properties.cairo`, …), so that the future engine can replay a scene and compare every sampled step.

Files owned:
- `tools/golden/src/cairo.rs` (and `tools/golden/src/main.rs` only if wiring is needed)
- `crates/rapier_golden/src/types.cairo` (add types; do not change existing ones)
- `crates/rapier_golden/src/generated.cairo` (declare the new module)
- `crates/rapier_golden/src/generated/scenes.cairo` (generated)
- `crates/rapier_golden/tests/scenes.cairo` (new sanity tests)
- `tools/golden/README.md` (document the new family; keep the existing text)

Frozen interfaces: everything already in `crates/rapier_golden/src/types.cairo` (`Vec2Raw`, `ShapeRaw`, pose/rotation structs…) — reuse them. Raw values are Q32.32 `i64` (`raw = round(value · 2^32)`), the same `q.rs` helpers as the other families.

Upstream reference: none needed; the JSON is the source of truth. Read `tools/golden/README.md` first for the fixture conventions, then `tools/golden/src/cairo.rs` to see how the other families are emitted.

Requirements:
1. Scene fixture types (raw `i64`, `#[derive(Copy, Drop)]`, usable in `const` position): a scene description (id as `felt252` short string, gravity, dt, bodies with type, initial pose, damping, gravity scale, colliders with shape / pose wrt parent / density / friction / restitution; joints if the JSON has any — the pendulum has a revolute joint) and the sampled trace (step index, per-body translation, rotation `(re, im)`, linvel, angvel). Body order must match `body_order` in the JSON. Represent variable-length lists with fixed-size arrays `[T; N]` in `const` position only if Cairo 2.19.4 accepts them for the sizes needed; otherwise generate one `const` per body/step and a `fn steps() -> Span<...>` accessor, whichever compiles. Check what the existing families do for `ALL` tables and follow the same pattern.
2. Keep the generator deterministic and idempotent: running `cargo run --release` twice in `tools/golden` must leave `git status` clean (the CI job `golden` enforces this). Run `scarb fmt` as the generator already does so the output passes `scarb fmt --check --workspace`.
3. Sanity tests in `crates/rapier_golden/tests/scenes.cairo`: the ball in `ball_drop` has strictly decreasing `y` during free fall over the first sampled steps and ends at rest (|linvel| below a small raw tolerance) above the slab; every scene has the expected number of bodies; the fixed ground never moves. Use `rapier_golden::compare` helpers.
4. Do not add dependencies to either the Rust crate or the Cairo crate. Do not change existing fixture output (the diff on the other generated files must be empty).

Acceptance:
- `cargo run --release --locked` in `tools/golden`: zero git diff on a second run
- from the repo root: `scarb fmt --check --workspace`, `scarb lint --workspace --deny-warnings`, `scarb build --workspace`, `snforge test --workspace` all pass
- `python3 scripts/gas.py diff` table in the report (new tests only; do not run `snapshot`)

Out of scope: any engine code, changing existing families, changing pinned Rust versions, CI files.
