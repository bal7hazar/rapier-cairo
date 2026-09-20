# Parry analysis — input for the `rapier.cairo` geometry port

Source analysed: `dimforge/parry` @ `3383f51` (v0.31.1, 2026-09-18) and `dimforge/rapier` @ `28d0ba9` (2026-09-19), shallow clones, read-only.
All paths below are relative to `parry/src/` unless prefixed with `rapier/`. Line counts are `wc -l` (docs included; parry is ~50 % rustdoc) unless marked "code LOC" (comments/blank lines stripped).

Cairo constraints assumed throughout: no floats (fixed point), value semantics, append-only `Array`, `Felt252Dict` as the only mutable map, no `dyn` (enum dispatch), cost = VM steps, loops/dict accesses expensive, plain arithmetic/comparisons cheap.

---

## 0. Headline findings

1. **Parry no longer uses nalgebra.** `math/mod.rs` wraps **glam via the `glamx` crate** (`glamx 0.3`: `Pose2/3`, `Rot2/3`, `MatExt`, `SymmetricEigen2/3`). `simba` is only kept for the `RealField/ComplexField` scalar traits and SIMD lanes. There is zero `nalgebra` import in the tree (three stale TODO comments only). The sibling *glam* port is therefore the relevant math dependency; the *nalgebra* port is not needed for parry/rapier geometry.
2. **`Qbvh` is gone.** The only acceleration structure is the binary `Bvh` (`partitioning/bvh/`, 6 847 lines). Rapier's only broad phase is `BroadPhaseBvh` (`rapier/src/geometry/broad_phase_bvh/`, 939 lines); the old SAP multi-layer broad phase has been removed.
3. **Core collision code is almost trig-free.** Outside tests/docs, runtime geometry needs only `sqrt`, division, `abs`, `min/max`, `copysign`, `clamp`. `sin/cos` appear only at rotation construction (`Rot2::new(angle)` in glamx), `atan2` only in `Cone::ccd_angular_thickness`, `acos` only in `VectorExt::angle` and mesh-intersection. Angle thresholds are pre-baked cosine constants (`utils/consts.rs`).
4. **The 2D contact path is small.** A 2D manifold holds at most 2 points (`ArrayVec<TrackedContact, 2>`). Ball/cuboid/capsule/halfspace pairs are analytic (SAT + 1D segment clipping). GJK+EPA is only reached for the generic "PFM-PFM" fallback (capsule–cuboid, anything with convex polygon / triangle / segment, round shapes) and for point projection / ray cast on `ConvexPolygon`.
5. Parry carries an `improved_fixed_point_support` feature and a vestigial `FixedI40F24` import in `crates/parry2d/tests/geometry/ball_cuboid_contact.rs` — upstream once ran this code on Q40.24 fixed point; three code sites are still switched by that feature (listed in §5).

---

## 1. Workspace layout, math aliases, features

### 1.1 Workspace

```
parry/
  Cargo.toml            workspace, version 0.31.1, members = crates/parry{2d,3d}{,-f64}
  src/                  single shared source tree (79 506 lines)
  crates/parry2d/       Cargo.toml ([lib] path = "../../src/lib.rs"), examples/ (21), tests/
  crates/parry3d/       same + benches/, examples/ (25), tests/
  crates/parry2d-f64/, crates/parry3d-f64/   same lib path, different required features
```

The four crates compile the *same* `src/lib.rs`; dimension and scalar are selected by mutually exclusive features `dim2|dim3` and `f32|f64` and consumed through `#[cfg(feature = "dim2")]` blocks everywhere. `lib.rs` is `#![no_std]` with optional `alloc`/`std`.

Top-level modules (lines): `query/` 26 514 · `shape/` 21 340 · `transformation/` 13 454 · `partitioning/` 6 847 · `utils/` 4 849 · `mass_properties/` 3 055 · `bounding_volume/` 2 774 · `math/` 468.

### 1.2 `math/mod.rs` (314 lines) — exact state today

- Scalar: `pub use f32 as Real; pub use i32 as Int;` (or `f64/i64`).
- Re-exports from **glamx**: `Pose2, Pose3, Rot2, Rot3, SymmetricEigen2/3, MatExt`, and glam `Mat2, Mat3, Vec2, Vec3, Vec3A, Vec4` (the `D*` variants under `f64`).
- Re-exports from **simba**: `ComplexField, RealField` (scalar function traits), SIMD `SimdReal/SimdBool` (`WideF32x4`, `x8` with `simd8`), `SIMD_WIDTH`.
- `DEFAULT_EPSILON = Real::EPSILON` (1.19e-7 in f32).
- Dimension aliases:

| alias | 2D | 3D |
|---|---|---|
| `Vector` | `Vec2` | `Vec3` |
| `IVector` | `IVec2` | `IVec3` |
| `AngVector`, `Orientation`, `AngularInertia`, `PrincipalAngularInertia` | `Real` | `Vec3` (`AngularInertia = SdpMatrix3`) |
| `Matrix` | `Mat2` | `Mat3` |
| `Pose` | `Pose2 {rotation: Rot2, translation: Vec2}` | `Pose3 {rotation: Rot3 (quat), translation: Vec3}` |
| `Rotation` | `Rot2 {re, im}` = unit complex (cos, sin) | `Rot3` (glam quaternion) |
| `SpatialVector` | `Vec3` | `[Real; 6]` |
| `CrossMatrix` | `Vec2` | `Mat3` |
| `SdpMatrix` | `utils::SdpMatrix2` | `utils::SdpMatrix3` |

- `math/vector_ext.rs` (154 lines): `VectorExt { ith, angle (uses acos), kronecker, vget, vset }`, `IVectorExt`.
- `orthonormal_subspace_basis` (3D only), `ivect_to_vect`/`vect_to_ivect`.

Port note: `Rot2` being (cos, sin) means every pose transform in 2D is 4 mul + 2 add, no trig. Trig is only required by whoever *integrates* the angle (rapier side), and even there one can integrate the complex number directly and renormalise (sqrt + div).

### 1.3 Feature flags (`crates/parry2d/Cargo.toml`; 3D identical unless noted)

| flag | effect | port relevance |
|---|---|---|
| `dim2`/`dim3`, `f32`/`f64` | required selectors | replace by separate Cairo packages or a generic `Vector` trait |
| `std`, `alloc` | `alloc` gates everything heap-based: Compound, Polyline, TriMesh, HeightField, Voxels, ConvexPolygon/Polyhedron, SharedShape, all `contact_manifolds_*`, EPA, `transformation/` | — |
| `enhanced-determinism` | libm + scalar glam, `indexmap`; incompatible with `simd8` | conceptually what Cairo gives for free |
| `simd8`, `parallel` (rayon) | SIMD width, parallel BVH ops | cut |
| `serde-serialize`, `rkyv`, `bytemuck-serialize`, `encase` | serialisation / GPU layouts | cut |
| `spade` | Delaunay for `transformation::volume_mesh` | cut |
| `wavefront` (3D) | `TriMesh::to_obj_file` | cut |
| `improved_fixed_point_support` | switches 3 code sites for fixed-point robustness | **read these sites** (§5) |

---

## 2. Shapes

### 2.1 Trait machinery

- `shape/shape.rs` (1 729 lines): `enum ShapeType` (Ball, Cuboid, Capsule, Segment, Triangle, Voxels, TriMesh, Polyline, HalfSpace, HeightField, Compound, ConvexPolygon[2D], ConvexPolyhedron[3D], Cylinder[3D], Cone[3D], RoundCuboid, RoundTriangle, RoundCylinder/RoundCone/RoundConvexPolyhedron[3D], RoundConvexPolygon[2D], Custom); `enum TypedShape<'a>` = same variants holding `&'a T`.
- `trait Shape: RayCast + PointQuery + Any`: `compute_local_aabb`, `compute_local_bounding_sphere`, `compute_aabb(pos)`, `compute_swept_aabb(start,end)` (= merge of two AABBs), `mass_properties(density)`, `shape_type`, `as_typed_shape`, `ccd_thickness`, `ccd_angular_thickness`, `is_convex`, `as_support_map -> Option<&dyn SupportMap>`, `as_composite_shape`, `as_polygonal_feature_map -> Option<(&dyn PolygonalFeatureMap, border_radius)>`, `feature_normal_at_point`, `clone_dyn`, `scale_dyn`. `impl dyn Shape` adds `as_ball()/as_cuboid()/…` downcasts via `Any`.
- `shape/shared_shape.rs` (715 lines): `SharedShape(pub Arc<dyn Shape>)` + constructors (`ball`, `cuboid`, `capsule`, `convex_hull`, `trimesh`, …). In Cairo this entire layer collapses into `enum Shape { Ball, Cuboid, Capsule, … }` with `match`; `TypedShape` *is* the Cairo representation. No `Arc` needed (value semantics); compounds would hold `Span<(Pose, Shape)>`.
- `shape/support_map.rs` (447 lines, ~20 code): `trait SupportMap { local_support_point(dir); local_support_point_toward(unit_dir); support_point(pose, dir) = pose * local(rot⁻¹ * dir); support_point_toward }`.
- `shape/polygonal_feature_map.rs` (207): `trait PolygonalFeatureMap: SupportMap { local_support_feature(dir, &mut PolygonalFeature); local_support_feature_toward(dir, hint, out) }`, implemented for Segment, Triangle, Cuboid, Cylinder, Cone (caps approximated by a square oriented by the contact azimuth), ConvexPolygon, ConvexPolyhedron. Capsule → `(segment, radius)`, `RoundShape<S>` → `(inner, border_radius)`.
- `shape/polygonal_feature2d.rs` (148): `PolygonalFeature { vertices: [Vector;2], vids: [PackedFeatureId;2], fid, num_vertices }` + `contacts()` → `face_face_contacts` (1D clipping) / `face_vertex_contacts`. 3D version `polygonal_feature3d.rs` (418): up to 4 vertices, `contacts_edge_edge`, `contacts_face_face` (projects both faces to a 2D plane, vertex-in-polygon tests + edge/edge intersections: O(4×4) double loops).
- `shape/feature_id.rs` (752): `enum FeatureId { Vertex(u32), Edge(u32)[3D], Face(u32), Unknown }`; `PackedFeatureId(u32)` with 2-bit header (`01` vertex, `10` edge, `11` face, `0` = UNKNOWN) and 30-bit code. Trivial in Cairo as a `u32`.

### 2.2 Per-shape table

`SM` = support map; `PFM` = polygonal feature map; difficulty is for a Cairo fixed-point port (E easy / M medium / H hard / X cut).

| Shape (file, lines) | Data | AABB (`bounding_volume/aabb_*.rs`) | Bounding sphere | Support point | Mass props (`mass_properties/*`) | `ccd_thickness` / angular | Diff. |
|---|---|---|---|---|---|---|---|
| **Ball** (`ball.rs` 248) | `radius` | `center ± r` (no rotation) | `(center, r)` | `normalize(dir) * r` (1 sqrt+div) | 2D: `V=πr²`, `I=r²/2·m`; 3D: `V=4/3πr³`, `I=2/5 r² m` | `r` / π | E |
| **Cuboid** (`cuboid.rs` 821) | `half_extents: Vector` | `center ± |R|·he` (`Pose::absolute_transform_vector`) | `|he|` (sqrt) | `copysign(he, dir)` — **no mul** | `V=4·hx·hy`, `I=(hx²+hy²)/3·m` (3D per-axis) | `min(he)` / π/2 | E |
| **Capsule** (`capsule.rs` 404) | `segment: Segment{a,b}`, `radius` | transform segment, `min/max(a,b) ± r` | centre mid, `|ab|/2 + r` | pick a or b by dot, `+ normalize_or(dir,Y)·r` | cylinder + ball + parallel-axis term `(h²/4 + 3hr/8)·V_ball` (`mass_properties_capsule.rs`); 3D needs `rotation_wrt_y` | `r` / π/2 | E |
| **Segment** (`segment.rs` 695) | `a, b` | min/max of 2 transformed pts | — | argmax of 2 dots | zero | 0 / π/2 | E |
| **Triangle** (`triangle.rs` 943) | `a, b, c` | per-axis min/max via 3 dots (`aabb_triangle.rs`) | — | argmax of 3 dots | 2D: area = `|perp_dot|/2`, Box2D-style `unit_angular_inertia`; 3D zero-volume | 0 / π/2 | E (2D) |
| **HalfSpace** (`half_space.rs` 323) | `normal: Vector` (unit), plane through local origin | `±Vector::MAX/2` (infinite) | infinite | none (not a SM) | zero / infinite | `Real::MAX` | E — but needs a special case in the broad phase (always-overlap list), since an "infinite" AABB in fixed point must be a sentinel |
| **RoundShape\<S\>** (`round_shape.rs` 345) | `inner_shape: S`, `border_radius` | inner AABB loosened by radius | +radius | `inner.support(dir) + normalize(dir)·radius` | same as inner (radius ignored) | inner | E once inner + PFM path exists |
| **ConvexPolygon** [2D] (`convex_polygon.rs` 672) | `points: Vec<Vector>`, `normals: Vec<Vector>` (unit edge normals, precomputed with sqrt) | `point_cloud_aabb`: transform all n pts | centroid + max dist | **O(n) linear scan** `utils::point_cloud_support_point`; PFM: O(n) scan over normals → edge | triangle fan around centroid: area, COM, inertia (`mass_properties_convex_polygon.rs`, O(n)) | `min(half_extents)` / π/4 | M (hot O(n) loops; constructors need convex hull → cut, accept pre-validated CCW vertex lists only via `from_convex_polyline_unmodified`) |
| **ConvexPolyhedron** [3D] (`convex_polyhedron.rs` 1 076) | points + `vertices/faces/edges` + 4 adjacency index arrays | point cloud | — | O(n) scan (no hill climbing) | tetrahedra decomposition (`mass_properties_convex_polyhedron.rs` → trimesh3d 342 lines, needs 3×3 symmetric eigen-decomposition for principal axes) | min half-extent / π/4 | H (construction requires `transformation::convex_hull3` 1 273 lines; eigen solver) |
| **Cylinder** [3D] (`cylinder.rs` 206) | `half_height, radius` | via 6 support-point calls (`aabb_utils::support_map_aabb`) | `sqrt(h²+r²)` | normalise xz part × r, `copysign(h, dir.y)` | closed form | `r` / π/2 | M (always goes through GJK/EPA for contacts) |
| **Cone** [3D] (`cone.rs` 174) | `half_height, radius` | same | same | branch apex vs base rim | closed form, COM at −h/2 | `r` / uses **atan2** | M |
| **Tetrahedron** [3D] (`tetrahedron.rs` 722) | `a,b,c,d` | — | — | — | — | — | not a rapier collider shape; used by volume meshes — X |
| **Compound** (`compound.rs` 2 050) | `shapes: Vec<(Pose, SharedShape)>`, `bvh: Bvh`, `aabbs`, `aabb`, flags, optional pseudo-normals | merged child AABBs | — | — | sum of children with parallel-axis (`mass_properties_compound.rs`) | min over children | M if reimplemented as a flat list without BVH and without `FIX_INTERNAL_EDGES`; otherwise H |
| **Polyline** (`polyline.rs` 1 210) | `bvh`, `vertices`, `indices: Vec<[u32;2]>`, pseudo-normals, flags | bvh root | — | — | zero | 0 / π/4 | X (MVP); static level geometry can be a list of Segments/Cuboids |
| **TriMesh** (`trimesh.rs` 2 303) | bvh, vertices, indices, optional topology (half-edge), connected components, pseudo-normals, flags | bvh root | — | — | `mass_properties_trimesh{2d,3d}.rs` | 0 / π/4 | X |
| **HeightField** (`heightfield2.rs` 256, `heightfield3.rs` 940) | 2D: `heights: Vec<Real>`, `status`, `scale`, `aabb`; 3D: `Array2<Real>` + cell status bitflags | stored | — | — | zero | 0 / π/4 | X (2D version is actually easy: cell lookup by `floor(x/scale)`; candidate post-MVP) |
| **Voxels** (`shape/voxels/` 2 546) | chunked sparse grid: `chunk_bvh`, `HashMap<IVector, header>`, chunks, `voxel_size` | — | — | — | `mass_properties_voxels.rs` | — | X |

Shape-level cut candidates inside kept files: `to_outline`/`to_polyline`/`to_trimesh` methods (in `transformation/`), `scaled()` variants (non-uniform scaling returns `Either<Shape, ConvexPolygon>`), `From<[Vector; N]>` sugar, pseudo-normal files (`*_pseudo_normals.rs`, only for internal-edge fixing on meshes/compounds).

---

## 3. Bounding volumes and partitioning

### 3.1 `Aabb` (`bounding_volume/aabb.rs`, 1 031 lines, ~300 code)

`struct Aabb { mins: Vector, maxs: Vector }`. Needed subset: `new`, `new_invalid`, `from_half_extents`, `from_points`, `center`, `half_extents`, `extents`, `volume`, `half_perimeter`/`half_area`, `take_point`, `transform_by` (centre transform + `|R|·half_extents`), `translated`, `merged`/`merge`, `intersects` (`mins ≤ other.maxs && maxs ≥ other.mins`, all axes), `contains`, `loosened(margin)`/`tightened`, `contains_local_point`, `intersects_moving_aabb` (Minkowski-sum + ray slab test, used by CCD). Cut: `intersects_spiral` (uses `sin_cos`, nonlinear CCD only), `difference*`, `split_at_center`, `aligned_intersections`, `scaled*`.

Every one of these is comparisons + add/sub; `transform_by` is the only one with multiplications (4 in 2D, 9 in 3D). `BoundingVolume` trait (`bounding_volume.rs` 45 lines): `center, intersects, contains, merge(d), loosen(ed), tighten(ed)`.

### 3.2 `BoundingSphere` (590 lines, mostly docs)

`{ center, radius }`. Rapier does not use bounding spheres in its pipeline (only `Aabb`). **Cut from MVP**; if desired as a cheap pre-filter it costs a squared-distance (2–3 mul) vs. the AABB's 4–6 comparisons — comparisons are cheaper in Cairo, so AABB wins.

### 3.3 `Bvh` (`partitioning/bvh/`, 6 847 lines, 3 253 code LOC)

Data (`bvh_tree.rs` 2 480):
- `BvhNode { mins: Vector, children: u32, maxs: Vector, data: BvhNodeData(u32) }` — 32 bytes; `data` packs `leaf_count` (30 bits) + 2 change bits (`CHANGED=0b01`, `CHANGE_PENDING=0b11`). `leaf_count == 1` ⇒ leaf, `children` = user leaf id.
- `BvhNodeWide { left: BvhNode, right: BvhNode }` — 64-byte cache-line-aligned pair; the tree is `Vec<BvhNodeWide>`, root at index 0. `BvhNodeIndex(usize)` = `(wide_id << 1) | is_right`.
- `Bvh { nodes, parents: Vec<BvhNodeIndex>, leaf_node_indices: VecMap<BvhNodeIndex>, optimization: BvhIncrementalOptimizationState, free_wide_nodes: Vec<u32> }`; `BvhWorkspace` holds scratch (refit buffer, rebuild leaves, `BinaryHeap` priority queue, traversal stack).

Algorithms:
- **Build** (`bvh_binned_build.rs` 200, `bvh_ploc_build.rs` 94): binned SAH (uses `floor`, `sqrt`) or PLOC (Morton-sorted locally-ordered clustering).
- **Insert** (`bvh_insert.rs` 730): SAH-cost descent (choose child minimising merged half-area/perimeter × leaf count), ancestors' AABBs/leaf-counts fixed on the way, `maybe_apply_rotation` tree rotations on the way up. `insert_or_update_partially(aabb, leaf, margin)` implements **fat-AABB change detection**: if the new AABB is still inside the stored (fattened by `margin`) leaf AABB → `BvhLeafUpdateStatus::Unchanged`, nothing touched; otherwise the leaf is rewritten in place and flagged `CHANGE_PENDING` (ancestors stale until refit). `reinsert_or_update_if_present` = remove + SAH re-insert for low change volumes.
- **Refit** (`bvh_refit.rs` 766): `refit` = full bottom-up recomputation that *also* rewrites the node array in depth-first order into `refit_tmp` (cache locality) and resolves change flags (pending → changed, propagate "changed" to ancestors); `refit_partial(prev_changed, newly_changed)` walks only ancestors of changed leaves via `parents`; `refit_without_resolve` preserves flags.
- **Rebalance** (`bvh_optimize.rs` 578): `optimize_incremental` — each call rebuilds ~5 % of leaves: picks `target_optimized_subtree_count` subtrees of ≈ `4·√n` leaves (tracked by a rolling cursor in `BvhIncrementalOptimizationState`), rebuilds each with binned SAH; every other frame optimises the root region (top ≈ `√n` nodes) either breadth-first or by priority queue (`BinaryHeap` on node cost). Uses `sqrt`, `log2`, `ceil`, `round` on leaf counts.
- **Traversal**: `bvh_traverse.rs` (445) — stack-based `leaves(check)`, `traverse(check) -> TraversalAction`, `find_best` (best-first with pruning cost, used by ray cast/point projection/shape cast on composites); `bvh_traverse_bvtt.rs` (528) — `traverse_bvtt_single_tree::<CHANGE_DETECTION>` self-intersection: recursive `self_intersect_node` + `intersect_nodes`, skipping subtrees whose `is_changed()` flag is clear; `leaf_pairs` for two-tree BVTT (composite vs composite); `bvh_queries.rs` (319) — `intersect_aabb`, `project_point`, `cast_ray`.

### 3.4 How rapier uses it (`rapier/src/geometry/broad_phase_bvh/{mod.rs 333, update.rs 606}`)

`BroadPhaseBvh { tree: Bvh, workspace, pairs: HashMap<(ColliderHandle, ColliderHandle), u32>, pair_adjacency: Coarena<Vec<ColliderHandle>>, prev/curr_updated_leaves, changes_since_optimize, reinsert_leaf_updates, frame_index, … }`.

Per step (`update`):
1. Remove leaves of removed colliders; force remove+reinsert for colliders whose filter inputs changed.
2. For each modified collider: `aabb = collider.compute_broad_phase_aabb()` (swept AABB if CCD/soft-CCD), fatten by skin = `CHANGE_DETECTION_FACTOR (0.04) × length_unit` (adaptive variant: `clamp(min_extent/8, 0.04, 0.25)`), then `insert_or_update_partially` (bulk regime) or `reinsert_or_update_if_present` (when `num_updated·16 < leaf_count`).
3. `optimize_incremental` when enough in-place updates accumulated (may be deferred to run concurrently with the solver), then `refit` (full) or `refit_partial` (when few leaves changed).
4. `traverse_bvtt_single_tree::<true>` → candidate pairs where at least one leaf changed; filter (same parent body, collision groups, `ActiveCollisionTypes`), insert new pairs into `pairs` + adjacency, emit `BroadPhasePairEvent::AddPair`.
5. Stale pairs: for every updated collider scan its adjacency list, re-test leaf AABB overlap, emit `DeletePair`.

The broad phase is thus **incremental and event-based** (add/delete pair events into the narrow phase's interaction graph). All of this relies on in-place mutable node arrays, hash maps, parent pointers and a heap — the worst possible fit for Cairo.

### 3.5 Cairo-friendly broad phase for N ≤ ~100

Context specific to Starknet: state that survives between steps lives in contract storage (very expensive per felt) or must be re-supplied as calldata. A persistent BVH / SAP endpoint list would have to be serialised each step; a **stateless** broad phase recomputed from the body list is strongly favoured. Pair persistence (needed for warm-starting) should live in the narrow-phase contact cache keyed by pair id, not in the broad phase.

Step-cost estimates below are order-of-magnitude reasoning, not measurements; the project's benchmark harness should confirm them. Rough unit costs assumed: fixed-point compare ≈ 3–6 steps (range-check based), array `at` ≈ 3–5, loop iteration overhead ≈ 10–20 (recursion-compiled loops, gas/withdraw bookkeeping), `Felt252Dict` access ≈ 50–100+ amortised including squash (each access adds an entry that must be range-checked and sorted at squash time; first access per key costs more).

**(a) Brute-force O(n²) AABB.** Pre-compute `aabbs: Span<Aabb>` once (N shape-dependent evaluations: ball 0 mul, cuboid 4 mul in 2D). Inner body: 4 comparisons in 2D (6 in 3D) with early-out after the first failing axis, plus 2 span reads. ≈ 40–60 steps/pair including loop overhead. N=32 → 496 pairs ≈ 25 k steps; N=100 → 4 950 pairs ≈ 250 k steps. Cheap reductions with pure arithmetic: split bodies into `dynamic[]` and `static[]` and only test dynamic×dynamic (triangular) + dynamic×static (skips static×static, which dominates typical game levels: 80 static + 20 dynamic → 190 + 1 600 = 1 790 pairs ≈ 90 k); skip sleeping×sleeping; collision-group mask test before AABB. No dicts, no state, deterministic output order (i<j) — which also gives a canonical pair id `i·N + j` for the contact cache.

**(b) Stateless single-axis sort-and-prune.** Sort indices by `mins.x` (merge sort rebuilding arrays: ~n·log₂n ≈ 700 element appends for n=100, ≈ 15–25 k steps; or insertion sort which is O(n) on coherent input but needs array rebuilding per shift in Cairo, so merge sort is the safer choice), then for each i scan j>i while `mins.x[j] ≤ maxs.x[i]`, testing the remaining axis. Work = sort + (number of x-overlapping pairs) × ~40 steps. In a side-scroller or top-down world spread along x this prunes 80–95 % of pairs: N=100 ≈ 30–70 k steps. Worst case (vertical stack, everything overlapping in x) degenerates to (a) plus the sort cost; choosing the axis of greatest variance (2 sums of squares, cheap) mitigates. Still dict-free and stateless. **Recommended above N ≈ 40–50.**

**(c) Uniform grid / spatial hash on `Felt252Dict`.** Key = `cx + cy·2^k` felt; each body is inserted into every overlapped cell (1–4 in 2D if cell ≥ max object size), cell contents need an intrusive linked list (second dict `next[slot]`) because dict values are single felts; pairs sharing several cells must be de-duplicated (arithmetic rule: report only in the cell containing the min corner of the AABB intersection). Cost ≈ N × cells × 2–3 dict ops × ~80 ≈ 50–100 k steps for N=100 *before* pair tests, plus squash of two dicts, plus `floor(x / cell)` divisions. Only wins for N in the several hundreds with homogeneous object sizes; large static platforms spanning many cells blow up insertion cost (parry/rapier avoid grids for the same reason). **Not recommended for the target range.**

**(d) BVH port.** Rebuild-per-step costs O(n log n) with heavy constant factors (SAH binning, node array rebuild); incremental version needs mutable nodes → dict-backed node store, ≥ 6 felts per node per access. Roughly an order of magnitude more steps than (b) at N=100. **Cut.**

Recommendation: implement (a) first behind a `BroadPhase` function signature `fn(aabbs: Span<Aabb>, flags: Span<BodyFlags>) -> Array<(u32,u32)>`, add (b) as a drop-in when benchmarks show the crossover. Keep rapier's *fat AABB* idea in a different form: enlarge AABBs by `prediction_distance/2` (rapier does exactly this in `pair_update.rs:378`) so that the narrow phase sees pairs slightly before contact.

---

## 4. Narrow phase

### 4.1 Dispatcher (`query/query_dispatcher.rs` 668, `query/default_query_dispatcher.rs` 823)

`trait QueryDispatcher { intersection_test, distance, contact, closest_points, cast_shapes, cast_shapes_nonlinear }` and `trait PersistentQueryDispatcher<ManifoldData, ContactData>: QueryDispatcher { contact_manifolds(pos12, shape1, shape2, prediction, &mut Vec<ContactManifold>, &mut Option<ContactManifoldsWorkspace>); contact_manifold_convex_convex(pos12, s1, s2, normal_constraints1/2, prediction, &mut ContactManifold) }`. `DefaultQueryDispatcher` is a unit struct; dispatch is an if-chain on downcasts / `match (shape_type1, shape_type2)`. All queries work in **shape-1 local space** with `pos12 = pos1⁻¹ · pos2`.

`contact_manifolds` order: composite×composite → TriMesh → HeightField → Voxels → composite×shape → `contact_manifold_convex_convex`.

`contact_manifold_convex_convex` match order (first hit wins):

| pair | function | algorithm |
|---|---|---|
| Ball–Ball | `contact_manifold_ball_ball` (57 lines) | analytic: `dist = |c| − r1 − r2`; normal `c/|c|` (fallback `Y`) |
| Cuboid–Cuboid | `contact_manifold_cuboid_cuboid` (100) | SAT (`sat_cuboid_cuboid.rs`) + `PolygonalFeature::contacts` clipping |
| Capsule–Capsule | `contact_manifold_capsule_capsule` (180) | segment–segment closest points; in 2D a second point by segment clipping when axes nearly parallel (`COS_FRAC_PI_8`, `SIN_FRAC_PI_8`) |
| X–Ball / Ball–X | `contact_manifold_convex_ball` (145) | `shape.project_local_point_and_get_feature(ball_centre)` + radius. Analytic for cuboid/capsule/segment/triangle/halfspace; **GJK/EPA for ConvexPolygon/Polyhedron/Cylinder/Cone** |
| Triangle–Cuboid | `contact_manifold_cuboid_triangle` (174) | SAT + clipping |
| HalfSpace–PFM | `contact_manifold_halfspace_pfm` (93) | support feature toward `−normal`, keep vertices with `dist ≤ prediction` |
| otherwise PFM–PFM | `contact_manifold_pfm_pfm` (165) | GJK (→ EPA if penetrating) for normal, then support features + clipping |
| (disabled) | `contact_manifolds_cuboid_capsule.rs` (175) — commented out in the dispatcher | SAT cuboid–segment + clipping. Present and analytic; **re-enable in the Cairo port** to keep capsule–cuboid off GJK |

Note HalfSpace–Ball is caught by the Ball arm (via `PointQuery for HalfSpace`), not by `halfspace_pfm` (Ball has no PFM).

### 4.2 Data structures (`contact_manifolds/contact_manifold.rs`, 915 lines, ~250 code)

```rust
struct TrackedContact<Data> { local_p1: Vector, local_p2: Vector, dist: Real,
                              fid1: PackedFeatureId, fid2: PackedFeatureId, data: Data }
struct ContactManifold<ManifoldData, ContactData> {
    points: ArrayVec<TrackedContact, 2> /*2D*/ | Vec<TrackedContact> /*3D, ≤4 after clipping (+1 GJK point)*/,
    local_n1: Vector, local_n2: Vector,           // normal in each shape's local frame
    subshape1: u32, subshape2: u32,
    subshape_poses: Option<Box<SubshapePoses>>,   // composite children only
    data: ManifoldData }
```

`local_p1` is in shape-1 space, `local_p2` in shape-2 space; `dist < 0` = penetration. Rapier instantiates `ContactData { impulse, tangent_impulse, warmstart_impulse, warmstart_tangent_impulse, warmstart_twist_impulse, warmstart_tangent_world, … }` (`rapier/src/geometry/contact_pair.rs`).

### 4.3 Persistence, contact ids, warm-start matching

- **Temporal coherence fast path** — `try_update_contacts_eps(pos12, COS_1_DEGREES = 0.99984769515, dist_sq_threshold = 1e-6)`: if `−n1 · (R12·n2) ≥ cos 1°`, and every point keeps its penetration sign and its re-projected `local_p1` moved by < 1e-3 (squared 1e-6), just `update_separations` (`dist = (pos12·p2 − p1)·n1`) and **skip the whole narrow phase**. Used by cuboid–cuboid and PFM–PFM. In Cairo this is ~6 mul per point vs. a full SAT/GJK — keep it, but it requires the previous manifold as input (calldata/storage); it is optional for correctness.
- **Matching** — `match_contacts(old)`: O(new×old) (≤ 2×2 in 2D) equality on `(fid1, fid2)`, copy `data` (impulses). `match_contacts_using_positions(old, dist_threshold)` is the fallback for shapes without stable feature ids. Ball–ball / capsule–capsule(3D) / convex–ball keep the single point and only `copy_geometry_from`, so impulses persist implicitly.
- **Feature-id schemes**: cuboid 2D face id = `(max(vid1,vid2) << 2) | min(vid1,vid2) | 0b11_00_00`, vertex id from sign bits (`Cuboid::vertex_feature_id`); segment: vertices 0/2, face 1; clipping returns feature index `0|1|2` (first vertex / face interior / second vertex) per side (`clip/clip_segment_segment.rs`); cylinder/cone ids are documented in `polygonal_feature_map.rs`; the extra GJK point pushed by PFM–PFM carries `PackedFeatureId::UNKNOWN` (never matched → no warm start).
- **Manifold normal as GJK cache**: PFM–PFM seeds GJK with `manifold.local_n1` and stores the separating direction back into `local_n1` on `NoIntersection`.

### 4.4 Prediction distance

`prediction` is a parameter of every manifold function: contacts are generated while `dist ≤ prediction` (+ border radii). Rapier defaults (`rapier/src/dynamics/integration_parameters.rs:412-417`): `prediction_distance = normalized_prediction_distance (0.02) × length_unit`, `allowed_linear_error = 0.005 × length_unit`; per pair `effective_prediction = max(prediction, dt·|v1−v2|)` under soft-CCD, plus `contact_skin` sums (`narrow_phase/pair_update.rs:328-354`). AABBs are loosened by `prediction/2` before the pair's AABB re-check. In fixed point, `prediction` must stay ≫ resolution; with Q32.32 and metre units 0.02 ≈ 8.6e7 ulp — fine.

### 4.5 Composite handling (cut for MVP, documented for completeness)

`contact_manifolds_composite_shape_shape.rs` (208): BVH `intersect_aabb` of the other shape's AABB (loosened by prediction) in composite space → for each hit child, a persistent `SubDetector { manifold_id, timestamp }` in `HashMap<u32, SubDetector>` inside `ContactManifoldsWorkspace` (type-erased `Box<dyn WorkspaceData>`), recursive `dispatcher.contact_manifolds` on the child with `pos12 = child_pose⁻¹·pos12`, stale sub-detectors pruned by timestamp flip. `composite_composite` uses two-tree `leaf_pairs`. TriMesh (`contact_manifolds_trimesh_shape.rs` 246) additionally applies pseudo-normal `NormalConstraints` (`normals_constraint.rs` 125) for internal-edge fixing; HeightField (198) iterates cells under the AABB; Voxels ×4 files (1 251 lines).

Cairo equivalent if compounds are wanted later: flat child list, brute-force child AABB test, manifold key `(pair_id, child_idx)`.

---

## 5. Core algorithms

Common tolerances: `DEFAULT_EPSILON = f32::EPSILON ≈ 1.19e-7`; `gjk::eps_tol() = 10·ε ≈ 1.19e-6`; `eps_rel = sqrt(eps_tol) ≈ 1.09e-3`; EPA `_eps_tol = 100·ε ≈ 1.19e-5`.

Fixed-point consequences (general): with Q32.32 (ulp 2.3e-10) all of these are representable (eps_tol ≈ 5 100 ulp). With Q16.16 (ulp 1.5e-5) `eps_tol` and the `1e-6` squared-distance thresholds underflow to 0, so tolerances must be re-derived in ulps. The main hazard is **comparisons on squared quantities** (`length_squared() < eps_tol`, `dist_sq_threshold = 1e-6`, `denom > eps` in segment–segment): a product of two small fixed-point numbers loses half its significant bits when rescaled. Cairo-specific remedy: for *comparisons only*, keep the double-width unscaled product (`i64×i64 → i128`, no shift) and compare against a pre-scaled constant — this is both exact and cheaper than the rescaling multiply.

### 5.1 GJK (`query/gjk/`, 1 656 lines, 920 code; 2D-only subset ≈ 624 code)

- `CsoPoint { point, orig1, orig2 }` (`cso_point.rs` 99): Minkowski-difference support `orig1 − pos12·orig2` with `from_shapes(pos12, g1, g2, dir)` = 2 support calls + 1 inverse rotation + 1 pose transform.
- `VoronoiSimplex` 2D (`voronoi_simplex2.rs` 218): `vertices: [CsoPoint; 3]`, `proj: [Real; 2]` barycentrics, `dim`, plus `prev_*` copies. `add_point` rejects points with `|v − p|² < eps_tol`. `project_origin_and_reduce` delegates to `Segment::project_local_point_and_get_location` / `Triangle::…` (Voronoi-region tests, Ericson) and swaps vertices to shrink the simplex. 3D (`voronoi_simplex3.rs` 345): up to 4 vertices, extra degeneracy checks (`|ab×ac|² < eps_tol`, `|n·ap| < eps_tol`), delegates to `Tetrahedron` projection (`point_tetrahedron.rs` 337).
- `closest_points(pos12, g1, g2, max_dist, exact_dist, simplex) -> GJKResult { Intersection | ClosestPoints(p1, p2, n) | Proximity(n) | NoIntersection(n) }` (`gjk.rs:370-495`):
  - loop bounded by `niter == 100` (returns `NoIntersection(X)` on exhaustion);
  - per iteration: 1 `normalize_and_length` (sqrt+div), 1 CSO support, 1 simplex projection;
  - exits: `dist ≤ eps_tol` → Intersection; `min_bound > max_dist` → NoIntersection (this is how `prediction` prunes early); `max_bound − min_bound ≤ eps_rel·max_bound` → converged; simplex full (`dim == DIM`) → Intersection/ClosestPoints; stagnation (`max_bound ≥ old_max_bound` or duplicate support) → classify with `best_min_bound` against `−eps_rel·support_scale`, else up to `MAX_PERTURBATIONS = 2·DIM` direction perturbations of `1e-2` (`perturbed_dir`).
  - Typical convergence for polygon pairs: 3–6 iterations in 2D, 4–10 in 3D.
- `minkowski_ray_cast` (`gjk.rs:700-855`): GJK ray cast on the CSO, basis of `cast_local_ray` (ray vs support map) and `directional_distance` (linear shape cast). Same 100-iteration cap. Contains the `improved_fixed_point_support` switch: when `max_bound − min_bound ≤ eps_rel·max_bound`, return the current hit instead of `None` — **enable this behaviour in the port**.
- `special_support_maps.rs` (84): `ConstantOrigin`, `ConstantPoint`, `DilatedShape`.

Fixed-point sensitivity: GJK is tolerant as long as (1) termination is relative (`eps_rel·max_bound`) with `eps_rel` ≥ ~2^-10, (2) duplicate-vertex detection uses the unscaled wide product, (3) the iteration cap is lowered (20 is ample in 2D) since every iteration costs on the order of 1–2 k steps (two support maps, a sqrt, a division, a triangle projection with ~10 dot products). Flat/degenerate simplices are the failure mode: barycentric denominators (`va+vb+vc`) become tiny → division blow-up. Guard denominators with an ulp floor and fall back to the previous simplex (`result(simplex, prev=true)` already exists for this).

Analytic alternatives: in 2D every MVP pair can avoid GJK — Box2D-style polygon SAT (`b2FindMaxSeparation`: for each edge normal of A take the support vertex of B; O(n·m) dot products, or O(n+m) with hill-climbing) gives separation *and* penetration axis in one pass, which also removes EPA.

### 5.2 EPA (`query/epa/epa2.rs` 594 / 302 code; `epa3.rs` 781 / 431 code)

- 2D: `vertices: Vec<CsoPoint>`, `faces: Vec<Face { pts: [usize;2], normal, proj, bcoords, deleted }>`, `heap: BinaryHeap<FaceId { id, neg_dist }>`. Start from GJK's simplex (special cases: `dim == 0` vertex–vertex → two ≤100-iteration loops walking tangent cones; `dim == 1` two opposite faces). Loop: pop closest face, support along its normal, stop when `max_dist − curr_dist < 100ε` or stalled (`|curr − old| < ε`); else split the face in two. Cap `niter > 100`. `dist_tol = eps_tol × max(1, max|vertex|)` rejects degenerate faces (issue #415). Returns `None` on failure → caller reports `NoIntersection(X)` (contact silently dropped for that step).
- 3D: triangle faces with adjacency `adj: [usize;3]`, recursive `compute_silhouette` flood fill, face deletion flags, new fan of faces per iteration; topology checks.

Fixed-point sensitivity: **high.** Face normals come from normalising edges/cross products of nearly coincident CSO points (precision collapses as the polytope refines); termination tolerance `100ε` is absolute; the heap ordering compares distances that differ by less than the rounding error, so the expansion order — and the result — become format-dependent. Cairo structural cost: needs a priority queue and mutable `deleted` flags → dict-backed or linear rescans of an append-only face array (acceptable in 2D where face count ≤ ~10; painful in 3D).

Cheap alternative: SAT penetration for polygons/boxes/capsules/segments (2D MVP needs **no EPA at all**). For 3D round shapes (cylinder/cone), MPR or "GJK on shrunken cores + margin" (Bullet style: run GJK on shapes eroded by a margin so that shallow penetration is still a separated-core query) avoids EPA in the common case.

### 5.3 SAT (`query/sat/`, 1 548 lines, 415 code)

- `cuboid_cuboid_find_local_separating_normal_oneway(c1, c2, pos12)` (`sat_cuboid_cuboid.rs`): for each of `DIM` face axes of c1 (sign chosen by `copysign(1, translation[i])`), `separation = (pos12·support2(−axis))[i]·sign − he1[i]`. No normalisation; ~4 mul per axis in 2D. When ≥ 2 axes are positive it accumulates a weighted axis and **normalises** (vertex/edge Voronoi region; 1 sqrt) — only when separated. Called twice (1→2, 2→1).
- 3D adds `cuboid_cuboid_find_local_separating_edge_twoway`: 9 cross-product axes, each normalised (`length`, skip if `≤ ε`) → 9 sqrt + 9 div. Fixed-point-friendly variant: compare `separation²·sign` against `best²·|axis|²` (cross-multiplication) to avoid all 9 sqrt; normalise only the winner.
- Others: `sat_cuboid_point.rs` (92), `sat_cuboid_segment.rs` (155; used by the disabled cuboid–capsule generator), `sat_cuboid_support_map.rs` (274), `sat_cuboid_triangle.rs` (332), `sat_triangle_segment.rs` (207), `sat_support_map_support_map.rs` (88; `support_map_support_map_compute_separation(sm1, sm2, pos12, dir)`). A `sat_polygon_polygon` module is referenced but commented out in `sat/mod.rs:96,106`; `contact_manifolds/polygon_polygon_contact_generator.rs` (150) is dead code left from the pre-parry rapier — usable as a reading reference for the polygon SAT the Cairo port should implement.

All SAT loops are statically bounded (DIM, 9, n). Numeric sensitivity: low — sums of products, comparisons. Tie-breaking between nearly equal axes (face-1 vs face-2) flips under rounding → manifold normal jitter; parry picks `sep2` only if strictly greater than `sep1` and `sep3` (bias toward shape 1's faces); keep that bias, and consider Box2D's relative+absolute tolerance (`0.98·s + 0.001`) for stability.

### 5.4 Clipping (`query/clip/`, 470 lines)

- `clip_segment_segment_with_normal(seg1, seg2, normal)` [2D] (`clip_segment_segment.rs`): project both segments on `tangent = perp(normal)`, sort 1D ranges, intersect, interpolate the clipped endpoints with `utils::inv` (safe reciprocal: `inv(0) = 0`). Returns two `ClippingPoints = (p1, p2, feature1, feature2)`. Cost ≈ 8 mul + 2 div. `clip_segment_segment` (any-D, no normal) uses seg1's own un-normalised tangent.
- `PolygonalFeature::face_vertex_contacts` [2D]: 1 division `(f0 − v)·n1 / (−n1·sep_axis)`.
- 3D `contacts_face_face`: see §2.1 — two 4×4 loops of 2D orientation tests + segment–segment line intersections (`closest_points_line2d`, eps-guarded determinant).
- `clip_aabb_line.rs` (219): slab clipping for ray–AABB / ray–cuboid. `clip_halfspace_polygon`, `clip_aabb_polygon`: mesh tooling, cut.

Bounded, division-light, robust in fixed point provided `inv` is preserved (zero-length range ⇒ bcoord 0).

### 5.5 Closest points / distance / contact (one-shot)

`query/closest_points/` (1 251), `query/distance/` (407), `query/contact/` (875). Rapier's pipeline does **not** call these in the step loop (only `KinematicCharacterController` calls `contact`). The one essential kernel is `closest_points_segment_segment_with_locations_nD` (`closest_points_segment_segment.rs` 118; Ericson §5.1.9): 5 dots, ≤ 3 divisions, `clamp`, collinearity guard `denom > ε && !ulps_eq!(ae, bb)` — in fixed point replace by `denom > k·ulp·(ae)` relative test on the wide product. Returns `SegmentPointLocation::{OnVertex(i), OnEdge([1−s, s])}`. Used by capsule–capsule and by rapier's joints/debug code.

### 5.6 Point projection (`query/point/`, 2 299 lines)

`trait PointQuery { project_local_point(pt, solid) -> PointProjection { is_inside, point, subshape }; project_local_point_and_get_feature -> (proj, FeatureId); distance_to_local_point; contains_local_point; + world-space wrappers }`.

| shape | file | algorithm |
|---|---|---|
| Ball | `point_ball.rs` 42 | `normalize_and_length` (1 sqrt, 1 div) |
| Cuboid | `point_cuboid.rs` 34 → `point_aabb.rs` 145 | per-axis shift/clamp, inside case picks min penetration axis; **no sqrt, no div** |
| Capsule | `point_capsule.rs` 47 | segment projection + ball |
| Segment | `point_segment.rs` 77 | 1 dot ratio (1 div) |
| Triangle | `point_triangle.rs` 318 | Voronoi regions (Ericson); 3D branch switched by `improved_fixed_point_support` (normalises `n` before the `n·(ab×ap)` triple products to avoid overflow/underflow of 4th-degree terms — a direct fixed-point lesson: **degree-4 expressions overflow/underflow; normalise intermediate vectors**) |
| HalfSpace | `point_halfspace.rs` 39 | 1 dot |
| ConvexPolygon/Polyhedron, Cylinder, Cone, RoundShape | `point_support_map.rs` 99 | **GJK `project_origin` then EPA if inside.** Cairo alternative for ConvexPolygon: loop edges, max signed distance to edge lines (precomputed unit normals) → inside test + closest edge, clamp on that edge; O(n), no iteration |

`contact_manifold_convex_ball` depends on this trait, so Ball×{Cuboid, Capsule, Segment, HalfSpace, Triangle} are fully analytic; Ball×ConvexPolygon needs the analytic replacement above.

### 5.7 Ray casting (`query/ray/`, 2 340 lines)

`Ray { origin, dir }`, `RayIntersection { time_of_impact, normal, feature, subshape }`, `trait RayCast { cast_local_ray(ray, max_toi, solid) -> Option<Real>; cast_local_ray_and_get_normal; intersects_local_ray }`.
Ball: quadratic, 1 sqrt (`ray_ball.rs`). Cuboid/Aabb: slab method (`ray_aabb.rs`, `clip_aabb_line.rs`; divisions by `dir[i]` with zero guard). HalfSpace: 1 division (`ray_halfspace.rs::ray_toi_with_halfspace`, also used inside GJK ray cast). Capsule: analytic since 0.31 (`ray_capsule.rs` 440). Triangle: Möller-style (`ray_triangle.rs`). Segment in 2D: analytic line–line parameters (`ray_support_map.rs:146`, `closest_points_line_line_parameters_eps`, with a collinear special case). Segment[3D]/ConvexPolygon/Polyhedron/Cylinder/Cone: `ray_support_map.rs` → `gjk::cast_local_ray` (iterative). Cairo alternative for ConvexPolygon: Cyrus–Beck clipping against edge half-planes, O(n), 1 division per non-parallel edge.
Not needed by the simulation step itself (scene queries, vehicle controller). **Post-MVP work package.**

### 5.8 Intersection tests (`query/intersection_test/`, 623 lines)

Ball–ball (squared distance vs `(r1+r2)²`, no sqrt), cuboid–cuboid (SAT sign only), cuboid–triangle, cuboid–segment, ball–X via `distance_to_local_point`, halfspace–SM (1 support point), SM–SM via GJK with `exact_dist = false` (exits at first separating axis — `Proximity`). Used by rapier for sensors (`narrow_phase/intersections.rs`) and CCD pre-checks. For the MVP, sensors can reuse `manifold.points.len() > 0 && dist ≤ 0` or dedicated boolean SAT variants (cheaper: early-out on first separating axis).

### 5.9 Shape casting / TOI

- **Linear** (`query/shape_cast/`, 1 120 lines): `cast_shapes(pos12, vel12, g1, g2, ShapeCastOptions { max_time_of_impact, target_distance, stop_at_penetration, compute_impact_geometry_on_penetration }) -> Option<ShapeCastHit { time_of_impact, witness1/2, normal1/2, status }>`. Ball–ball analytic (ray vs inflated ball: 1 sqrt); halfspace–SM analytic (support point + ray–plane); SM–SM = `gjk::directional_distance` (Minkowski ray cast, ≤ 100 iterations; `target_distance > 0` wraps shapes in `DilatedShape`). Used by the character controller and `QueryPipeline::cast_shape`.
- **Nonlinear, velocity-based** (`query/nonlinear_shape_cast/`, 1 881 lines): `NonlinearRigidMotion { start, local_center, linvel, angvel }` (`position_at_time` needs rotation from `angvel·t` → **sin/cos per evaluation**), conservative advancement with `bisect` (loop until `range < eps_tol`, no hard cap besides convergence), `Aabb::intersects_spiral`, `utils/interval.rs` interval arithmetic incl. interval `sin_cos`. Still called by rapier (`dynamics/ccd/sweeps.rs:508`) for some pairs.
- **Sweep TOI, endpoint-interpolated** (`query/sweep_toi/`, 2 681 lines, 2 052 code; new, ported from Box2D): `Sweep::from_poses(start, end, local_com)` (linear COM lerp + rotation **nlerp** → sqrt, no trig), `ToiProxy` (≤ 4/8 inline points + radius), own GJK distance with simplex cache (`proxy_distance.rs` 1 027, `MAX_GJK_ITERATIONS = 32`, `MAX_ITERATIONS = 20`), separation function (`separation.rs` 597), outer loop `MAX_DISTANCE_ITERATIONS = 20` (2D) / 25 (3D), root finder `MAX_ROOT_ITERATIONS = 50` hybrid bisection/false position, tolerance `0.25·linear_slop`. All bounded, trig-free — the better CCD candidate if CCD is ever ported — but worst case is 20 × (GJK 32 + vertices × 50 root iterations × 2 pose interpolations): tens to hundreds of k steps per fast pair.

**CCD is cut from the MVP.** Provable-game mitigation: clamp velocities so that `|v|·dt < ccd_thickness` of the thinnest dynamic shape, thick static walls, and/or substeps; optionally add analytic ball-vs-static sweeps (ray vs inflated AABB/segment) for bullets.

### 5.10 Bounded-iteration summary

| algorithm | loop bound | sqrt / div per iteration | fixed-point risk | analytic alternative |
|---|---|---|---|---|
| SAT cuboid 2D | 2×2 axes | 0 / 0 (1 sqrt in corner region) | low | — (is the alternative) |
| SAT cuboid 3D | 6 + 9 axes | 9 / 9 | low–medium (near-parallel edges → tiny cross products; ε skip) | cross-multiplied comparison, normalise winner only |
| segment clipping 2D | none | 0 / 2 | low | — |
| seg–seg closest pts | none | 0 / ≤ 3 | medium (collinearity guard) | — |
| GJK distance | 100 (+ 2·DIM perturbations) | 1 / 1 + projection divs | medium (flat simplices, squared-eps tests) | polygon SAT in 2D |
| EPA 2D / 3D | 100 (+ 2×100 in vertex case) | 1–2 / 1 per new face | **high** | SAT depth; core-shape margins; MPR |
| GJK ray cast | 100 | 1 / 2 | medium–high | ray vs inflated primitives, Cyrus–Beck |
| nonlinear shape cast | convergence-based | trig per eval | high | cut |
| sweep TOI | 20 × 32 × 50 | 1 per pose eval | medium | speed clamp / substeps |
| BVH ops | tree depth / n | sqrt, log2 in optimiser | n/a (structure, not numerics) | brute force / sort-and-prune |

---

## 6. Floating-point function inventory

Counts are occurrences in `src/` excluding comment-only lines (they still include doc-tests inside `///` blocks on the same files for a few entries; treat as upper bounds).

| function | occurrences | where it matters (runtime paths) | Cairo note |
|---|---|---|---|
| `length()` / `normalize()` / `try_normalize` / `normalize_or*` / `normalize_and_length` (**sqrt + div**) | 80 / 73 / 41 / 31 / 19 | `shape/ball.rs:231,241` (support), `shape/capsule.rs:393`, `shape/round_shape.rs`, `query/gjk/gjk.rs:397,717` (every iteration), `query/epa/epa2.rs` via `utils/ccw_face_normal.rs`, `query/sat/sat_cuboid_cuboid.rs` (9 edge axes; corner case), `contact_manifolds_ball_ball.rs:30`, `contact_manifolds_convex_ball.rs` (explicit `dpos / dist` — comment explains they avoid `1/len` multiply for exactness), `contact_manifolds_capsule_capsule.rs`, `query/point/point_ball.rs`, `shape/convex_polygon.rs` (edge normals at construction), `bounding_volume/bounding_sphere_*.rs` | one `try_normalize(v) -> Option<(unit, len)>` primitive returning both; implement via integer sqrt of the wide `length_squared` (Cairo has `u128_sqrt`/`u256_sqrt` libfuncs — sqrt is comparatively cheap, a hint + verification) then one division per component or one reciprocal + DIM mul |
| explicit `sqrt(` | 24 | `gjk.rs:376,716` (`eps_rel`, constant → precompute), `ray_ball.rs:58`, `ray_capsule.rs`, `sat_cuboid_triangle.rs`, `sweep_toi.rs`, `bvh_optimize.rs`, `bvh_binned_build.rs`, `bounding_sphere_{cone,cylinder,utils}.rs`, `mass_properties_triangle.rs` | — |
| inverse sqrt | 0 explicit | hidden in glam's `normalize` (`v * length_recip`) | decide once in the glam port: `x / len` (parry's own preference for exactness) |
| `utils::inv` (safe `1/x`, `inv(0)=0`) | 32 | `utils/inv.rs`; clipping, mass properties (`inv_mass`, `inv_principal_inertia`) | keep semantics |
| `abs()` | 60 | cuboid support face (`local_dir.abs().min_position()`), `absolute_transform_vector`, SAT, tolerances | trivial on signed fixed |
| `copy_sign_to` (`utils/wops.rs::WSign`) | 32 | `shape/cuboid.rs:490` support point, SAT sign selection, cylinder/cone support | sign-bit select; **no multiply** |
| `min`/`max`/`clamp` | many / 20 clamp | AABB, seg–seg | trivial |
| `sin`, `cos`, `sin_cos` | 10 / 10 / 7 | runtime: `bounding_volume/aabb.rs::intersects_spiral`, `utils/interval.rs` (both nonlinear CCD only); `glamx::Rot2::new(angle)`. Others are doc-tests/tests (`convex_polygon.rs`, `mass_properties_convex_polygon.rs`, `bvh_*`) | **not needed by the MVP collision code**; needed by whoever builds a `Rot2` from an angle (scene setup) |
| `atan2` | 1 | `shape/shape.rs:1462` (`Cone::ccd_angular_thickness`); `glamx::Rot2::angle()` | cut with Cone/CCD |
| `acos` | 2 | `math/vector_ext.rs:55` (`VectorExt::angle`), `transformation/mesh_intersection` | cut; compare cosines instead |
| `floor`/`ceil`/`round` | 10 / 17 / 1 | heightfield cell lookup, voxels, BVH binning/optimiser, voxelisation | only if 2D heightfield is ported |
| `log2` | 2 | `bvh_optimize.rs` | cut |
| `pi()`, `frac_pi_2/4` | 22 / 21 | ball/cylinder/cone volume, `ccd_angular_thickness` | constants |
| `EPSILON` / `DEFAULT_EPSILON` / `default_epsilon()` / `eps_tol()` | 43 / 27 / 10 / 14 | GJK, EPA, seg–seg, SAT edge skip, capsule second point (`ε·100`), triangle degeneracy | centralise in one `consts` module expressed in ulps |
| `relative_eq!` / `ulps_eq!` (approx crate) | 48 (mostly tests) | runtime: `gjk.rs:719` zero ray, `closest_points_segment_segment.rs:67` | replace by explicit abs/rel tests |
| `Real::MAX` / `max_value()` / `is_finite` | 84 / 27 / 6 | sentinels (`-Real::MAX` best separation, halfspace AABB, `max_dist`) ; `assert!(min_bound.is_finite())` in GJK | use `Option` or a dedicated sentinel; fixed point has no NaN/inf (overflow panics instead → a *liveness* risk in a provable game: a panicking step cannot be proven; saturating ops or world-bounds clamping are advisable) |
| hard-coded constants | — | `utils/consts.rs`: `COS_1_DEGREES = 0.99984769515`, `COS_FRAC_PI_8 = 0.92387953251`, `SIN_FRAC_PI_8 = 0.38268343236`; `contact_manifold.rs`: `DIST_SQ_THRESHOLD = 1e-6`; `gjk.rs`: perturbation `OFFSET = 1e-2`; `polygonal_feature3d.rs:131`: eps `1e-3`-ish for edge–edge; `polygonal_feature_map.rs`: `dir.y.abs() < 0.5` cylinder side/cap switch | note `cos 1°` needs ≥ 14 fractional bits to be distinguishable from 1.0 (1 − cos1° = 1.5e-4) |

---

## 7. Parry API surface consumed by rapier

Derived from `grep parry::` over `rapier/src` plus call-site grep. Rapier re-exports `parry::shape::*`, `parry::math::*`, `parry::glamx` wholesale (`geometry/mod.rs:281`, `lib.rs:158`), so the *public* surface is larger than what the engine itself calls; below is what the engine calls.

**math**: `Real, Vector, AngVector, Pose, Rotation, Matrix, VectorExt, SimdReal, SIMD_WIDTH`, `glamx`.

**bounding_volume**: `Aabb`, `BoundingVolume` (`intersects`, `merged`, `loosened`, `contains`).

**partitioning**: `Bvh` (`insert_or_update_partially`, `reinsert_or_update_if_present`, `remove`, `refit`, `refit_partial`, `refit_without_resolve`, `optimize_incremental`, `traverse_bvtt_single_tree`, `leaf_node`, `leaf_count`, `intersect_aabb`, `find_best`, `cast_ray`, `project_point`), `BvhWorkspace`, `BvhNode`, `BvhLeafUpdateStatus`, `BvhBuildStrategy` — broad phase, `QueryPipeline`, soft bodies.

**shape**: `Shape` (trait methods: `compute_aabb`, `compute_swept_aabb`, `mass_properties`, `ccd_thickness`, `ccd_angular_thickness`, `shape_type`, `as_typed_shape`, `as_composite_shape`), `TypedShape`, `SharedShape`, `Ball, Cuboid, Capsule, Segment, Triangle, HalfSpace, HeightField, Cylinder, Cone, Tetrahedron`, `Polyline/PolylineFlags`, `TriMeshFlags/TriMeshBuilderError`, `Voxels/VoxelState/VoxelType`, `CompositeShape/CompositeShapeRef/TypedCompositeShape`, `FeatureId`, `PackedFeatureId`, `SegmentPointLocation`.

**mass_properties**: `MassProperties { local_com, inv_mass, inv_principal_inertia, principal_inertia_local_frame[3D] }` with `+`/`-`, `transform_by`, `from_ball/cuboid/capsule/…`, `world_com`, `world_inv_inertia_sqrt`-style helpers (`mass_properties.rs` 789).

**query**:
- step loop: `PersistentQueryDispatcher::contact_manifolds` (`narrow_phase/pair_update.rs:438` — the single narrow-phase entry point), `QueryDispatcher::intersection_test` (sensors `narrow_phase/intersections.rs`, CCD `ccd_solver.rs`), `ContactManifold`, `TrackedContact`, `ContactManifoldsWorkspace`, `DefaultQueryDispatcher`.
- CCD: `QueryDispatcher::cast_shapes_nonlinear`, `NonlinearRigidMotion`, `sweep_toi::{Sweep, SweepCompositeFastShape, SweepToiStatus, ToiProxy, sweep_time_of_impact, CORE_FRACTION}`, `ShapeCastHit`.
- scene queries / controllers: `Ray`, `RayIntersection`, `RayCast::{cast_ray, cast_ray_and_get_normal, cast_local_ray_and_get_normal}`, `PointQuery::{project_point, project_local_point}`, `PointQueryWithLocation`, `PointProjection`, `details::ShapeCastOptions`, `QueryDispatcher::{cast_shapes, contact}` (character controller), `details::NormalConstraints`.
- misc: `details::closest_points_segment_segment_with_locations{,_nD}` (soft-body edge pass), `utils::{SdpMatrix, SdpMatrix3, hashmap::HashMap, hashset::HashSet, VecMap, PoseOpt, Array2, obb}`.
- tooling: `transformation::{vhacd::VHACDParameters, voxelization::FillMode, VolumeMesh, VolumeMeshParameters}` (collider builders, soft bodies).

**Minimum port surface for a rapier-like rigid-body step**: `Aabb` + `Shape::{compute_aabb, mass_properties}` + `contact_manifolds` (convex–convex subset) + `ContactManifold/TrackedContact/PackedFeatureId` + `MassProperties`. Everything else (BVH, rays, point queries as public API, shape casts, CCD, composites, transformation) is optional.

---

## 8. Test strategy in parry and golden vectors

- **Unit tests in `src/`**: 130 `#[test]` (SAT, clipping, BVH `bvh_tests.rs` 460 lines, EPA regression cases, mass properties, `utils`).
- **Integration tests**: `crates/parry2d/tests/` (79 tests) — `geometry/{ball_ball_toi, ball_cuboid_contact, epa2, epa_convergence, ray_cast, time_of_impact2, aabb_scale, convex_polygons_intersection}.rs`, `query/{closest_points_cuboid_cuboid, point_triangle, point_composite_shape}.rs`, plus `issue_*.rs` regression files (e.g. `issue_315_touching_cuboids_contact`, `issue_431_cuboid_distance_asymmetry`, `issue_180_shape_cast_grazing_none`, `issue_76_point_degenerate_triangle`). `crates/parry3d/tests/` (113 tests) similar, with many GJK/EPA degeneracy regressions (`issue_396_cylinder_intersection_false_negative`, `issue_193_shape_cast_penetrating_normals`, `issue_429_shape_cast_toi_accuracy`, `issue_70_capsule_cuboid_false_negatives`).
- **Doc-tests**: ≈ 1 184 fenced examples across `src/` — nearly every public shape method has a tiny numeric example with asserted values (good seeds for Cairo unit tests: e.g. `Cuboid::local_support_point`, `Aabb::*`, `MassProperties::from_*`).
- **Examples**: 21 (2D) + 25 (3D) runnable programs (`contact_query2d`, `distance_query2d`, `time_of_impact_query2d`, `solid_ray_cast2d`, `convex2d`, …), several with kiss3d visualisation.
- **Benches** (3D only): `crates/parry3d/benches/{bounding_volume, query, support_map}` — useful list of "what upstream considers hot".

**Golden-vector generation plan.** Tests assert mostly on inequalities/approximate equality, so they are not directly golden data. Build a small Rust harness crate (outside the Cairo repo tree or under `tools/golden/`) depending on `parry2d` (f64 variant `parry2d-f64` for reference precision, plus `enhanced-determinism`):
1. Deterministic generators (`oorandom`, as parry's own tests use) produce shape pairs + `pos12` for each cell of the MVP pair matrix, stratified by regime: separated beyond prediction, within prediction, touching, shallow penetration, deep penetration, degenerate (parallel edges, vertex–vertex, coincident centres, zero-length capsule).
2. Call the *leaf* functions directly (`contact_manifold_cuboid_cuboid`, `contact_manifold_ball_ball`, `sat::cuboid_cuboid_find_local_separating_normal_oneway`, `clip_segment_segment_with_normal`, `closest_points_segment_segment_with_locations_nD`, `Shape::compute_aabb`, `mass_properties`, `project_local_point`) and dump inputs/outputs as JSON → generate Cairo test files with fixed-point literals.
3. Compare in Cairo with a tolerance of a few hundred ulps for positions/distances and an angular tolerance for normals; compare **discrete outputs exactly** (number of points, feature ids, which SAT axis won) but tag cases whose float margin between alternatives is below a threshold as "ambiguous" and accept either outcome.
4. Replay parry's issue regressions that fall inside the MVP matrix (2D cuboid/ball/capsule/segment ones) as named tests.
5. End-to-end: record `rapier2d` trajectories for a few canonical scenes (stack of boxes, ball pit, capsule on slope) as coarse envelopes — not bit-exact targets, since fixed point will diverge chaotically.

---

## 9. Proposed Cairo decomposition

### 9.1 MVP shape set and pair matrix — 2D first

Shapes: **Ball, Cuboid, Capsule, HalfSpace** (static ground/walls), **Segment** (static edges), then **ConvexPolygon** (≤ 8 vertices, pre-validated CCW, precomputed unit normals) as phase 2; **RoundCuboid** is nearly free once Cuboid clipping exists (shift points by `border_radius`, subtract from dist).

`enum Shape { Ball: Fixed, Cuboid: Vec2, Capsule: (Vec2, Vec2, Fixed), HalfSpace: Vec2, Segment: (Vec2, Vec2), ConvexPolygon: Span<Vec2> + Span<Vec2> }`.

Pair matrix (upper triangle; lower = flip with `TrackedContact::flipped` semantics):

| | Ball | Cuboid | Capsule | Segment | HalfSpace | ConvexPolygon (phase 2) |
|---|---|---|---|---|---|---|
| **Ball** | analytic (parry `ball_ball`) | point-on-AABB projection (parry `convex_ball` + `point_aabb`) | point-on-segment + radius | point-on-segment | 1 dot | edge-loop projection (**deviation**: parry uses GJK/EPA) |
| **Cuboid** | | SAT + clip (parry `cuboid_cuboid`) | SAT cuboid–segment + clip + radius (**re-enable** parry's `cuboid_capsule`) | same, radius 0 | support face vertices vs plane (parry `halfspace_pfm`) | polygon SAT + clip (**deviation**: parry uses PFM/GJK/EPA) |
| **Capsule** | | | seg–seg closest + parallel clipping (parry `capsule_capsule` 2D) | same, radius2 = 0 | 2 endpoints vs plane − r | polygon SAT with segment as 2-gon + radius |
| **Segment** | | | | static–static: skip | skip / endpoints vs plane | polygon SAT |
| **HalfSpace** | | | | | skip | all-vertices-below-prediction (support feature) |
| **ConvexPolygon** | | | | | | polygon SAT + clip |

With this matrix the 2D MVP contains **no GJK and no EPA**. A generic `Cuboid`-as-polygon path can serve as a differential-testing oracle for the specialised cuboid SAT.

### 9.2 3D extension

Ball, Cuboid, Capsule, HalfSpace first: ball–X analytic; cuboid–cuboid = 15-axis SAT + `polygonal_feature3d` face/edge clipping (≤ 4+4 vertices, the main new complexity, ~330 code LOC); capsule–capsule single point (parry 3D does exactly that); capsule–cuboid via SAT cuboid–segment (`sat_cuboid_segment.rs`) + clipping; manifold up to 4 points → needs rapier's manifold reduction. Cylinder/Cone/ConvexPolyhedron require the GJK(+EPA or margin-core) package and the 3×3 symmetric eigen solver for inertia → separate, later milestone.

### 9.3 Modules, dependency order, work packages

Sizes are estimates of Cairo LOC for the 2D MVP, informed by the Rust code LOC of the corresponding parry files.

```
L0  fixed / glam port (external sibling): Fixed, Vec2, Rot2{cos,sin}, Pose2, Mat2, sqrt, div, wide-mul compare
L1  parry::consts        eps in ulps, COS_1_DEG, COS/SIN_PI_8, prediction defaults            (~50)
    parry::math_ext      try_normalize -> Option<(Vec2, Fixed)>, inv, copysign, perp, abs_rot  (~150)
    parry::feature_id    PackedFeatureId(u32) encode/decode                                   (~60)
L2  parry::aabb          Aabb + BoundingVolume ops, transform_by                               (~200)
    parry::shape         enum Shape, per-shape structs, support_point, support_face (PFM 2D)   (~400)
    parry::mass          MassProperties + from_ball/cuboid/capsule/convex_polygon, add, transform (~300)
L3  parry::shape_aabb    compute_aabb(shape, pose), swept aabb                                  (~120)
    parry::point         project_local_point(+feature) for ball/cuboid/capsule/segment/halfspace/polygon (~350)
    parry::seg_seg       closest_points_segment_segment_with_locations                         (~120)
    parry::clip          clip_segment_segment_with_normal, face_face / face_vertex contacts    (~200)
    parry::sat           cuboid_cuboid oneway, cuboid_segment, polygon_polygon max-separation  (~350)
L4  parry::manifold      TrackedContact, ContactManifold (≤2 pts), try_update_contacts, match_contacts, update_separations (~250)
    parry::contact::*    one file per matrix cell (ball_ball, convex_ball, cuboid_cuboid, capsule_capsule,
                         cuboid_capsule, halfspace_x, polygon_polygon)                          (~700)
L5  parry::dispatch      match (Shape, Shape) -> manifold, flip handling                        (~150)
    parry::broad_phase   brute force (static/dynamic split), optional sort-and-prune            (~200–350)
L6  (post-MVP)           ray casts; intersection tests for sensors; GJK2D+VoronoiSimplex2; compound (flat);
                         2D heightfield; linear shape cast; 3D packages
```

Parallelisable work packages once L0–L1 interfaces are frozen:
- **WP-A** `aabb` + `shape_aabb` + `broad_phase` (independent of all narrow-phase code; benchmark-driven).
- **WP-B** `shape` (support points/faces, feature ids) + `mass`.
- **WP-C** `point` + `seg_seg` (pure kernels, golden-vector heavy).
- **WP-D** `clip` + `sat`.
- **WP-E** `manifold` (data + persistence logic; can be developed against mocked generators).
- **WP-F** `contact::*` cells — each cell is an independent task after B/C/D/E land; `dispatch` last.
- **WP-G** Rust golden-vector harness (can start immediately, no Cairo dependency).
- **WP-H** (optional, later) GJK 2D + simplex, as oracle/fallback for exotic pairs and ray casts.

### 9.4 Hot loops (step-cost hotspots to benchmark first)

1. Broad-phase pair loop (N² or sweep) — runs every step over all bodies.
2. `pose` algebra: `pos12 = pos1⁻¹·pos2`, `pos12 * point`, `rot⁻¹ * dir` — executed several times per pair; in 2D compute `(c, s)` of the relative rotation once and reuse; avoid re-deriving `pos21`.
3. SAT axis loops + support-point evaluation (cuboid: `copysign`, no mul — keep it that way; polygon: O(n) dot scans → cap n, or exploit CCW ordering for hill-climbing from the previous best vertex).
4. `try_normalize` (sqrt + DIM divisions): ball–ball, convex–ball, capsule–capsule, SAT corner region. One per pair at most in the MVP matrix — ensure no hidden duplicates (parry's `Ball::support_point` normalises `dir` on every call; irrelevant once GJK is out).
5. `match_contacts` / `try_update_contacts` — tiny (≤ 2×2) but per pair per step; `try_update_contacts` is the big saver for resting stacks (skips SAT+clipping entirely).
6. If GJK is ever enabled: the GJK main loop (2 support maps + sqrt + simplex projection per iteration) and EPA face expansion.
7. ConvexPolygon `compute_aabb` transforms all n vertices each step → cache local AABB and use `Aabb::transform_by` (conservative, 4 mul) instead.

### 9.5 Explicit cut list

| cut | lines (approx.) | reason |
|---|---|---|
| `transformation/` (convex hull 2D/3D, VHACD, voxelization, mesh intersection, volume mesh, `to_trimesh/to_polyline/to_outline`) | 13 454 | offline asset tooling; hash maps, heaps, Delaunay; run in Rust off-chain and feed validated convex data |
| `shape/trimesh.rs`, `ray_trimesh`, `contact_manifolds_trimesh_shape`, pseudo-normals, `mass_properties_trimesh*` | ~4 000 | BVH + half-edge topology + internal-edge fixing; unbounded data size on-chain |
| `shape/voxels/*`, `contact_manifolds_voxels_*`, `*_voxels_*` queries | ~4 500 | hash-map chunk store, 3D-oriented |
| `shape/heightfield3.rs` (+ 3D queries) | ~1 800 | `Array2`, 3D only; (2D heightfield is a cheap post-MVP option) |
| `shape/polyline.rs`, `shape/compound.rs` (BVH-backed), composite manifolds/workspace | ~4 000 | depend on `Bvh`, `HashMap` workspaces, `dyn` recursion; replace later by flat child lists |
| `partitioning/bvh/*` | 6 847 | mutable node arrays, parent pointers, heaps; stateless broad phase is cheaper at N ≤ 100 (§3.5) |
| `bounding_volume/bounding_sphere*`, `simd_aabb.rs` | ~1 300 | unused by rapier's step; SIMD |
| `query/epa/*` | 1 472 | high fixed-point risk, needs priority queue; unnecessary with SAT-based matrix |
| `query/gjk/*` (MVP) | 1 656 | not needed by the 2D matrix; keep as WP-H option |
| `query/nonlinear_shape_cast/*`, `query/sweep_toi/*`, `utils/interval.rs`, `Aabb::intersects_spiral` | ~5 100 | CCD: trig/interval arithmetic or deep nested iteration; replace by velocity clamping |
| `query/shape_cast/*`, `query/ray/*` (MVP), `query/distance/*`, `query/closest_points/*` (except seg–seg), `query/contact/*` (one-shot) | ~6 000 | scene queries, not part of the step |
| `query/split/*` | 1 233 | mesh/shape splitting tooling |
| Cylinder, Cone, ConvexPolyhedron, Tetrahedron, Round* in 3D | ~2 500 | 3D-only, need GJK/EPA and eigen-decomposition |
| serde / rkyv / bytemuck / encase derives, `parallel` (rayon), SIMD (`simd8`, `SimdReal`, `array!` macro), `wavefront`, `spade` | — | no Cairo equivalent / irrelevant; Cairo `Serde`/`Store` derives replace serialisation |
| `dyn Shape`, `SharedShape(Arc)`, `Any` downcasts, `QueryDispatcherChain`, `Custom` shape type | ~1 000 | replaced by `enum Shape` + `match` |
| `utils/{hashmap, hashset, fx_hasher, vec_map, sort, median, morton, z_order, spade, as_bytes, obb, cov, eigen*}` | ~3 000 | support code for the cut modules |

Kept-but-simplified: `NormalConstraints` (drop), `subshape1/2` + `subshape_poses` (drop until compounds), `ContactManifoldsWorkspace` (drop), generic `ManifoldData/ContactData` parameters (monomorphise to rapier.cairo's concrete structs).
