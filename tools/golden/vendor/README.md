# tools/golden/vendor — patched upstream crates

## parry2d-f64 0.30.2

A copy of the published crate (`parry2d-f64 = "=0.30.2"`, crates.io checksum
`cc4dc9d407475690de5f87740b33254163d11e98df650690d8ae6ce690c42fc2`, upstream git
`1be4b1a7cd0a090bd7efb1207b7bc0d453f4132e`, Apache-2.0), wired in by `[patch.crates-io]` in
`../Cargo.toml`. `rapier2d-f64 0.35.3` depends on `parry2d-f64 ^0.30.2`, so Rapier links this
copy too: the scene traces run with the fix.

**Why.** `Cuboid::vertex_feature_id` extracts sign bits with `to_bits() >> 31` / `>> 30`, the
sign bit of an `f32` but a mantissa bit of an `f64` (upstream's own `TODO: is this still correct
with the f64 version?`). In f64 every cuboid vertex id is `0` and every face id `0b110000`, so
contact matching gives both regenerated points of a cuboid manifold the same warm-start data
(diagnosed by SD, see `../README.md`). That TODO covers only this function: no other feature id
of the crate is computed from float bits.

**The patch** (the whole diff against the published crate):

```diff
--- parry2d-f64-0.30.2/src/shape/cuboid.rs (crates.io)
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

With it, the f64 ids equal those of the f32 `parry2d 0.30.2` build on all 216 ids (108 manifold points) of
`contact_manifolds.json`.

**What was kept.** `Cargo.toml` (the normalised one from the `.crate`) and the `src/` files the
build reads: the 62 files that `cargo` never opens for this feature set (3D-only and test-only
modules behind `cfg`) were deleted, using the dep-info file of the release build
(`target/release/deps/parry2d_f64-*.d`). No other file differs from the published crate.
276 files, 2.9M.

**Updating.** Changing the Parry pin means re-vendoring: copy `src/` and `Cargo.toml` of the new
published crate, re-apply the diff above (or drop it if upstream fixed it), build once, delete the
`src/` files absent from the dep-info, rebuild, and regenerate the vectors in the same PR.
