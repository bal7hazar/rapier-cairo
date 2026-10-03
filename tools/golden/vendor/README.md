# tools/golden/vendor — patched upstream crates

Two published crates, wired in by `[patch.crates-io]` in `../Cargo.toml`. Each copy keeps the
normalised `Cargo.toml` of the `.crate` and the `src/` files the build reads: the files `cargo`
never opens for this feature set (3D-only, test-only and debug-render modules behind `cfg`) were
deleted, using the dep-info files of the release build (`target/release/deps/<crate>-*.d`). No
other file differs from the published crate except the patches below.

| crate | patch, one line each |
|---|---|
| `parry2d-f64 0.31.1` | `Cuboid::vertex_feature_id` reads the f64 sign bits (`>> 63` / `>> 62`), not the f32 ones |
| `rapier2d-f64 0.35.3` | its `parry2d-f64` requirement raised from `0.30.2` to `0.31.1`, and 6 call sites adapted to parry 0.31's API |

## parry2d-f64 0.31.1

A copy of the published crate (`parry2d-f64 = "=0.31.1"`, crates.io checksum
`4f0d7c5731bcbff3cefdca822e64693f9654598377475030f25946a3417be856`, upstream git
`3383f51cbbe9af70565427e7a66c605e1557c1fc`, Apache-2.0). Its `src/` equals the `v0.31.1` tag of the
reference clone except `shape/cuboid.rs` (the patch). Rapier links this copy too: the scene traces
run with the fix. 281 files, 2.3M. (Until OB this folder held `parry2d-f64 0.30.2`, checksum
`cc4dc9d407475690de5f87740b33254163d11e98df650690d8ae6ce690c42fc2`, with the same patch.)

**Why.** `Cuboid::vertex_feature_id` extracts sign bits with `to_bits() >> 31` / `>> 30`, the
sign bit of an `f32` but a mantissa bit of an `f64` (upstream's own `TODO: is this still correct
with the f64 version?`, still there in 0.31.1). In f64 every cuboid vertex id is `0` and every face
id `0b110000`, so contact matching gives both regenerated points of a cuboid manifold the same
warm-start data (diagnosed by SD, see `../README.md`). That TODO covers only this function: no
other feature id of the crate is computed from float bits.

**The patch** (the whole diff against the published crate):

```diff
--- parry2d-f64-0.31.1/src/shape/cuboid.rs (crates.io)
+++ vendor/parry2d-f64/src/shape/cuboid.rs
@@ -162,10 +162,11 @@
     /// the dot product with `dir`.
     #[cfg(feature = "dim2")]
     pub fn vertex_feature_id(vertex: Vector) -> u32 {
-        // TODO: is this still correct with the f64 version?
+        // rapier.cairo golden harness patch: read the f64 sign bits (63 / 62), not the f32 ones
+        // (31 / 30, mantissa bits of an f64). See tools/golden/vendor/README.md.
         #[allow(clippy::unnecessary_cast)] // Unnecessary for f32 but necessary for f64.
         {
-            ((vertex.x.to_bits() >> 31) & 0b001 | (vertex.y.to_bits() >> 30) & 0b010) as u32
+            ((vertex.x.to_bits() >> 63) & 0b001 | (vertex.y.to_bits() >> 62) & 0b010) as u32
         }
     }
 
```

With it, the f64 ids equal those of the f32 `parry2d` build on all 216 ids (108 manifold points) of
`contact_manifolds.json`.

## rapier2d-f64 0.35.3

A copy of the published crate (`rapier2d-f64 = "=0.35.3"`, crates.io checksum
`60d083d6e7bebb42db54d1a82c2e8db19c7adffbe6bf2bf0c8ebfb4cde78bae0`, upstream git
`b82079ac41310a8af438af95b49b8fa551ce650f`, Apache-2.0). 133 files, 1.8M.

**Why.** No published Rapier depends on parry 0.31: `rapier2d-f64 0.35.3` asks for `^0.30.2`, so
pinning parry 0.31.1 next to it would build two Parry copies (Rapier on 0.30.2, the oracle's own
queries on 0.31.1). Raising the requirement keeps one Parry, the patched 0.31.1 above.

**The patch.** `Cargo.toml`: `[dependencies.parry2d-f64] version = "0.31.1"` (was `"0.30.2"`).
Six call sites, none numeric, follow two API changes of parry 0.31:

- `QueryDispatcher::intersection_test` returns a `ShapeIntersection` instead of a `bool`: read
  `.map(|i| i.intersecting)` in `dynamics/ccd/ccd_solver.rs` (2 sites),
  `geometry/narrow_phase/intersections.rs` (1), `pipeline/physics_world.rs` (1) and
  `pipeline/query_pipeline.rs` (1);
- `CompositeShapeRef::project_local_point_and_get_feature` returns a flat
  `(id, projection, feature)` instead of `(id, (projection, feature))`:
  `pipeline/query_pipeline.rs` (1).

`diff -ru` against the published crate (pruned files aside) shows exactly these lines.

## Updating

Changing a pin means re-vendoring: copy `src/` and `Cargo.toml` of the new published crate,
re-apply the patches above (or drop them if upstream fixed them, or drop the Rapier copy when a
published Rapier asks for the Parry pin), build once, delete the `src/` files absent from the
dep-info, rebuild, and regenerate the vectors in the same PR. If values move, apply the policy of
`../README.md` ("Frozen parry 0.30.2 values") to each moved field.
