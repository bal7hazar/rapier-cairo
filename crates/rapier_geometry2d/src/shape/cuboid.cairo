//! `Cuboid` (Parry `shape/cuboid.rs`, `bounding_volume/aabb_cuboid.rs`,
//! `mass_properties_cuboid.rs`).

use fixed::{Fixed, FixedTrait};
use glam::Vec2;
use rapier_math::pose2::Pose2;
use rapier_math::{copy_sign_to, smallest_abs_component_index};
use crate::feature_id::{FeatureId, FeatureIdTrait};
use crate::mass::{MassProperties, MassPropertiesTrait};
use crate::shape::aabb_shim::{Aabb, AabbTrait, absolute_transform_vector};

/// Raw of `1 / sqrt(2)`, the components of a normalised diagonal.
const FRAC_1_SQRT_2_RAW: i64 = 3037000500;
/// `0b11_0000`: the fixed bits of a cuboid face code (see [`CuboidTrait::support_feature`]).
const FACE_CODE_BASE: u32 = 48;

/// A box centred on the local origin, aligned with the local axes.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Cuboid {
    /// Half of the extent along each axis (`>= 0`, not checked, as upstream).
    pub half_extents: Vec2,
}

/// The two-vertex face of a cuboid that supports a direction (the 2D `PolygonalFeature` that the
/// clipping code of work package GD consumes; same fields, `num_vertices` is always 2 here).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SupportFeature {
    pub vertices: [Vec2; 2],
    pub vids: [FeatureId; 2],
    pub fid: FeatureId,
    pub num_vertices: u8,
}

#[generate_trait]
pub impl CuboidImpl of CuboidTrait {
    #[inline(always)]
    fn new(half_extents: Vec2) -> Cuboid {
        Cuboid { half_extents }
    }

    /// `[-h, h]`.
    /// #### Panics
    /// * `'i64_neg Underflow'` for a half extent of `fixed::MIN`.
    #[inline(always)]
    fn compute_local_aabb(self: Cuboid) -> Aabb {
        AabbTrait::new(-self.half_extents, self.half_extents)
    }

    /// Box centred on the translation with half extents `|R| * h`: two fused kernels
    /// (`dot2` per axis), no division. Exact for quarter turns.
    #[inline(always)]
    fn compute_aabb(self: Cuboid, pose: Pose2) -> Aabb {
        AabbTrait::from_half_extents(
            pose.translation, absolute_transform_vector(pose, self.half_extents),
        )
    }

    /// Mass properties for `density` (`from_cuboid`).
    fn mass_properties(self: Cuboid, density: Fixed) -> MassProperties {
        MassPropertiesTrait::from_cuboid(density, self.half_extents)
    }

    /// The vertex code of the **f32** build of Parry: bit 0 is the sign of `x`, bit 1 the sign of
    /// `y` (`0b00` = `(+, +)`, `0b01` = `(-, +)`, `0b10` = `(+, -)`, `0b11` = `(-, -)`). A zero
    /// coordinate counts as positive (Q32.32 has no `-0`).
    #[inline(always)]
    fn vertex_feature_id(vertex: Vec2) -> FeatureId {
        let code: u32 = if vertex.x.raw < 0 {
            1
        } else {
            0
        } + if vertex.y.raw < 0 {
            2
        } else {
            0
        };
        FeatureIdTrait::vertex(code)
    }

    /// The face of the cuboid that is most aligned with `local_dir` (upstream `support_feature`
    /// / `support_face`): the axis along which `local_dir` is smallest supplies the two vertices,
    /// the other axis picks the side. Face code `= (max(v1, v2) << 2) | min(v1, v2) | 0b11_0000`
    /// where `v1`, `v2` are the vertex codes; vertex ids are the plain vertex codes.
    fn support_feature(self: Cuboid, local_dir: Vec2) -> SupportFeature {
        let he = self.half_extents;
        let (v1, v2) = if smallest_abs_component_index(local_dir.x, local_dir.y) == 0 {
            let y = copy_sign_to(local_dir.y, he.y);
            (Vec2 { x: he.x, y }, Vec2 { x: -he.x, y })
        } else {
            let x = copy_sign_to(local_dir.x, he.x);
            (Vec2 { x, y: he.y }, Vec2 { x, y: -he.y })
        };
        let vid1 = Self::vertex_feature_id(v1);
        let vid2 = Self::vertex_feature_id(v2);
        let (c1, c2) = (vid1.code(), vid2.code());
        let (hi, lo) = if c1 > c2 {
            (c1, c2)
        } else {
            (c2, c1)
        };
        SupportFeature {
            vertices: [v1, v2],
            vids: [vid1, vid2],
            fid: FeatureIdTrait::face(hi * 4 + lo + FACE_CODE_BASE),
            num_vertices: 2,
        }
    }

    /// Outward normal of a feature, `None` when the id does not name one. Faces use the simple
    /// scheme (`0`, `1` = `+x`, `+y`; `2`, `3` = `-x`, `-y`); the face codes produced by
    /// `support_feature` (>= 48) are not in it (upstream would index out of bounds). Vertices give
    /// the normalised diagonal of the vertex code.
    fn feature_normal(self: Cuboid, feature: FeatureId) -> Option<Vec2> {
        let zero = FixedTrait::from_raw(0);
        let one = FixedTrait::from_raw(0x100000000);
        let diag = FixedTrait::from_raw(FRAC_1_SQRT_2_RAW);
        let code = feature.code();
        if feature.is_face() {
            match code {
                0 => Some(Vec2 { x: one, y: zero }),
                1 => Some(Vec2 { x: zero, y: one }),
                2 => Some(Vec2 { x: -one, y: zero }),
                3 => Some(Vec2 { x: zero, y: -one }),
                _ => None,
            }
        } else if feature.is_vertex() {
            match code {
                0 => Some(Vec2 { x: diag, y: diag }),
                1 => Some(Vec2 { x: -diag, y: diag }),
                2 => Some(Vec2 { x: diag, y: -diag }),
                3 => Some(Vec2 { x: -diag, y: -diag }),
                _ => None,
            }
        } else {
            None
        }
    }

    /// The vertex farthest along `dir`: the half extents with the signs of `dir` (zero counts as
    /// positive). Needs no normalisation.
    /// #### Panics
    /// * `'Fixed: overflow'` for a half extent of `fixed::MIN`.
    #[inline(always)]
    fn local_support_point(self: Cuboid, dir: Vec2) -> Vec2 {
        Vec2 {
            x: copy_sign_to(dir.x, self.half_extents.x),
            y: copy_sign_to(dir.y, self.half_extents.y),
        }
    }

    /// Same as `local_support_point`: the cuboid support map ignores the length of `dir`.
    #[inline(always)]
    fn local_support_point_toward(self: Cuboid, dir: Vec2) -> Vec2 {
        Self::local_support_point(self, dir)
    }
}

/// Rejected candidates for `compute_aabb`, kept for the `gas_*` ranking and as oracles.
#[cfg(test)]
mod alternatives {
    use fixed::{Fixed, FixedTrait};
    use glam::Vec2;
    use rapier_math::pose2::Pose2;
    use rapier_math::rot2::Rot2Trait;
    use crate::shape::aabb_shim::{Aabb, AabbTrait};
    use super::Cuboid;

    fn abs_max(p: Fixed, q: Fixed) -> Fixed {
        if p.abs() > q.abs() {
            p.abs()
        } else {
            q.abs()
        }
    }

    /// `|R| * h` with two rounded products per axis instead of one fused rescale (up to 1 ulp
    /// off the winner).
    pub fn compute_aabb_composed(c: Cuboid, pose: Pose2) -> Aabb {
        let (cos, sin) = (pose.rotation.re.abs(), pose.rotation.im.abs());
        let h = c.half_extents;
        let ws = Vec2 { x: cos * h.x + sin * h.y, y: sin * h.x + cos * h.y };
        AabbTrait::from_half_extents(pose.translation, ws)
    }

    /// Upstream-independent oracle: rotate the four corners and take the min / max.
    pub fn compute_aabb_corners(c: Cuboid, pose: Pose2) -> Aabb {
        let h = c.half_extents;
        let a = pose.rotation.rotate(h);
        let b = pose.rotation.rotate(Vec2 { x: h.x, y: -h.y });
        let ws = Vec2 { x: abs_max(a.x, b.x), y: abs_max(a.y, b.y) };
        AabbTrait::from_half_extents(pose.translation, ws)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_golden::contact_manifolds::cases;
    use rapier_golden::types::{ShapeRaw, Vec2Raw};
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::{Rot2, Rot2Trait};
    use rapier_testing::opaque;
    use crate::feature_id::{FEATURE_UNKNOWN, FeatureId, FeatureIdTrait};
    use crate::shape::aabb_shim::Aabb;
    use super::{Cuboid, CuboidTrait, alternatives};

    const DIAG: i64 = 3037000500;

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn pose(x: Fixed, y: Fixed, re: Fixed, im: Fixed) -> Pose2 {
        Pose2Trait::new(v(x, y), Rot2Trait::from_cos_sin(re, im))
    }

    fn cuboid() -> Cuboid {
        CuboidTrait::new(v(TWO, ONE))
    }

    /// A 45 degree pose built from literals: no runtime normalisation inside a gas probe.
    fn probe_pose() -> Pose2 {
        Pose2 {
            translation: v(ONE, ONE),
            rotation: Rot2 { re: Fixed { raw: 3037000500 }, im: Fixed { raw: 3037000500 } },
        }
    }

    #[test]
    fn test_aabb_table() {
        let n = -ONE;
        // (pose, mins, maxs) of the box (2, 1): exact for quarter turns, translated by (3, -2).
        let cases: Span<(Pose2, Vec2, Vec2)> = array![
            (pose(ZERO, ZERO, ONE, ZERO), v(-TWO, n), v(TWO, ONE)),
            (pose(ZERO, ZERO, ZERO, ONE), v(n, -TWO), v(ONE, TWO)),
            (pose(ZERO, ZERO, n, ZERO), v(-TWO, n), v(TWO, ONE)),
            (
                pose(FixedTrait::from_int(3), FixedTrait::from_int(-2), ZERO, -ONE),
                v(FixedTrait::from_int(2), FixedTrait::from_int(-4)),
                v(FixedTrait::from_int(4), FixedTrait::from_int(0)),
            ),
        ]
            .span();
        for (p, mins, maxs) in cases {
            assert_eq!(cuboid().compute_aabb(*p), Aabb { mins: *mins, maxs: *maxs });
        }
        // 45 degrees: half extent (2 + 1) / sqrt(2) on both axes, within 2 ulp.
        let rotated = cuboid().compute_aabb(pose(ZERO, ZERO, ONE, ONE));
        let expected = Fixed { raw: 3 * DIAG };
        assert!(rotated.maxs.x.abs_diff_eq(expected, Fixed { raw: 2 }));
        assert!(rotated.maxs.y.abs_diff_eq(expected, Fixed { raw: 2 }));
        assert_eq!(rotated.mins, -rotated.maxs);
        // Local box and degenerate (zero) half extents.
        let local = cuboid().compute_local_aabb();
        assert_eq!((local.mins, local.maxs), (v(-TWO, n), v(TWO, ONE)));
        let flat = CuboidTrait::new(v(ZERO, ZERO)).compute_aabb(pose(ONE, ONE, ONE, ONE));
        assert_eq!((flat.mins, flat.maxs), (v(ONE, ONE), v(ONE, ONE)));
    }

    #[test]
    fn test_vertex_feature_id_table() {
        let n = -ONE;
        // Sign bits of (x, y): bit 0 = x < 0, bit 1 = y < 0; zero is positive.
        let cases: Span<(Vec2, u32)> = array![
            (v(ONE, ONE), 0), (v(n, ONE), 1), (v(ONE, n), 2), (v(n, n), 3), (v(ZERO, ZERO), 0),
            (v(ZERO, n), 2),
        ]
            .span();
        for (vertex, code) in cases {
            assert_eq!(CuboidTrait::vertex_feature_id(*vertex), FeatureIdTrait::vertex(*code));
        }
    }

    #[test]
    fn test_support_feature_table() {
        let t = HALF / FixedTrait::from_int(5);
        let n = -ONE;
        // (direction, vertices, vertex codes, face code): he = (2, 1). The axis with the smaller
        // |direction| gives the two vertices; face code = (max << 2) | min | 0b11_0000.
        let cases: Span<(Vec2, [Vec2; 2], [u32; 2], u32)> = array![
            (v(ONE, t), [v(TWO, ONE), v(TWO, n)], [0, 2], 56),
            (v(t, ONE), [v(TWO, ONE), v(-TWO, ONE)], [0, 1], 52),
            (v(n, t), [v(-TWO, ONE), v(-TWO, n)], [1, 3], 61),
            (v(t, n), [v(TWO, n), v(-TWO, n)], [2, 3], 62),
            (v(ONE, ONE), [v(TWO, ONE), v(-TWO, ONE)], [0, 1], 52),
            (v(ZERO, ZERO), [v(TWO, ONE), v(-TWO, ONE)], [0, 1], 52),
        ]
            .span();
        for (dir, vertices, vids, face) in cases {
            let f = cuboid().support_feature(*dir);
            let [a, b] = f.vertices;
            let [va, vb] = *vertices;
            assert_eq!((a, b), (va, vb));
            let [ia, ib] = f.vids;
            let [ca, cb] = *vids;
            assert_eq!((ia, ib), (FeatureIdTrait::vertex(ca), FeatureIdTrait::vertex(cb)));
            assert_eq!(f.fid, FeatureIdTrait::face(*face));
            assert_eq!(f.num_vertices, 2);
        }
    }

    #[test]
    fn test_feature_normal_table() {
        let zero = ZERO;
        let d = Fixed { raw: DIAG };
        let cases: Span<(FeatureId, Option<Vec2>)> = array![
            (FeatureIdTrait::face(0), Some(v(ONE, zero))),
            (FeatureIdTrait::face(1), Some(v(zero, ONE))),
            (FeatureIdTrait::face(2), Some(v(-ONE, zero))),
            (FeatureIdTrait::face(3), Some(v(zero, -ONE))), (FeatureIdTrait::face(56), None),
            (FeatureIdTrait::vertex(0), Some(v(d, d))), (FeatureIdTrait::vertex(1), Some(v(-d, d))),
            (FeatureIdTrait::vertex(2), Some(v(d, -d))),
            (FeatureIdTrait::vertex(3), Some(v(-d, -d))), (FeatureIdTrait::vertex(4), None),
            (FEATURE_UNKNOWN, None),
        ]
            .span();
        for (feature, expected) in cases {
            assert_eq!(cuboid().feature_normal(*feature), *expected);
        }
    }

    #[test]
    fn test_support_points() {
        let n = -ONE;
        let cases: Span<(Vec2, Vec2)> = array![
            (v(ONE, ONE), v(TWO, ONE)), (v(n, ONE), v(-TWO, ONE)), (v(ONE, n), v(TWO, n)),
            (v(n, n), v(-TWO, n)), (v(ZERO, ZERO), v(TWO, ONE)),
            (v(Fixed { raw: 1 }, n), v(TWO, n)),
        ]
            .span();
        for (dir, expected) in cases {
            assert_eq!(cuboid().local_support_point(*dir), *expected);
            assert_eq!(cuboid().local_support_point_toward(*dir), *expected);
        }
    }

    /// The feature ids of the f32 upstream (`rapier_golden::contact_manifolds`) for every
    /// non-ambiguous cuboid-cuboid case: a contact point's ids are the vertex ids or the face id of
    /// the support feature along the manifold normal of its shape.
    #[test]
    fn test_feature_ids_match_golden_manifolds() {
        let mut checked = 0_u32;
        for c in cases() {
            let (h1, h2) = match (*c.shape1, *c.shape2) {
                (ShapeRaw::Cuboid(a), ShapeRaw::Cuboid(b)) => (a, b),
                _ => { continue; },
            };
            if *c.ambiguous || *c.num_points == 0 {
                continue;
            }
            let f1 = CuboidTrait::new(vec(h1)).support_feature(vec(*c.local_n1));
            let f2 = CuboidTrait::new(vec(h2)).support_feature(vec(*c.local_n2));
            let [p1, p2] = *c.points;
            let points = array![p1, p2];
            let mut i = 0_u32;
            for p in points.span() {
                if i < *c.num_points {
                    assert!(is_of(f1, *p.fid1), "fid1 of {}", *c.id);
                    assert!(is_of(f2, *p.fid2), "fid2 of {}", *c.id);
                    checked += 1;
                }
                i += 1;
            }
        }
        assert!(checked >= 10, "golden cases checked");
    }

    fn vec(r: Vec2Raw) -> Vec2 {
        Vec2 { x: Fixed { raw: r.x }, y: Fixed { raw: r.y } }
    }

    fn is_of(f: super::SupportFeature, packed: u32) -> bool {
        let [a, b] = f.vids;
        packed == a.packed || packed == b.packed || packed == f.fid.packed
    }

    fn close(a: Aabb, b: Aabb) {
        let tol = Fixed { raw: 2 };
        assert!(a.mins.x.abs_diff_eq(b.mins.x, tol) && a.mins.y.abs_diff_eq(b.mins.y, tol));
        assert!(a.maxs.x.abs_diff_eq(b.maxs.x, tol) && a.maxs.y.abs_diff_eq(b.maxs.y, tol));
    }

    #[test]
    #[fuzzer(runs: 128, seed: 20260920)]
    fn fuzz_aabb_candidates(hx: u16, hy: u16, re: i32, im: i32, tx: i32) {
        if re == 0 && im == 0 {
            return;
        }
        let half = v(Fixed { raw: hx.into() * 65536 }, Fixed { raw: hy.into() * 65536 });
        let p = pose(
            Fixed { raw: tx.into() }, ZERO, Fixed { raw: re.into() }, Fixed { raw: im.into() },
        );
        let c = CuboidTrait::new(half);
        let fused = c.compute_aabb(p);
        close(fused, alternatives::compute_aabb_composed(c, p));
        close(fused, alternatives::compute_aabb_corners(c, p));
    }

    #[test]
    fn gas_baseline() {}
    #[test]
    fn gas_compute_local_aabb() {
        let _ = CuboidTrait::new(opaque(v(TWO, ONE))).compute_local_aabb();
    }
    #[test]
    fn gas_compute_aabb_fused() {
        let _ = CuboidTrait::new(opaque(v(TWO, ONE))).compute_aabb(opaque(probe_pose()));
    }
    #[test]
    fn gas_compute_aabb_composed() {
        let _ = alternatives::compute_aabb_composed(
            CuboidTrait::new(opaque(v(TWO, ONE))), opaque(probe_pose()),
        );
    }
    #[test]
    fn gas_compute_aabb_corners() {
        let _ = alternatives::compute_aabb_corners(
            CuboidTrait::new(opaque(v(TWO, ONE))), opaque(probe_pose()),
        );
    }
    #[test]
    fn gas_vertex_feature_id() {
        let _ = CuboidTrait::vertex_feature_id(opaque(v(-ONE, ONE)));
    }
    #[test]
    fn gas_support_feature() {
        let _ = CuboidTrait::new(opaque(v(TWO, ONE))).support_feature(opaque(v(ONE, HALF)));
    }
    #[test]
    fn gas_feature_normal() {
        let _ = CuboidTrait::new(opaque(v(TWO, ONE)))
            .feature_normal(opaque(FeatureIdTrait::vertex(3)));
    }
    #[test]
    fn gas_local_support_point() {
        let _ = CuboidTrait::new(opaque(v(TWO, ONE))).local_support_point(opaque(v(-ONE, HALF)));
    }
    #[test]
    fn gas_mass_properties() {
        let _ = CuboidTrait::new(opaque(v(TWO, ONE))).mass_properties(opaque(ONE));
    }
}
