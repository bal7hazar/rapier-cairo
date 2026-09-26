//! Half-space against polygonal-feature-map contact manifolds.
//!
//! Port of Parry's `contact_manifolds_halfspace_pfm`: ask the feature-map shape for the support
//! feature toward `-normal`, then emit one contact for each feature vertex whose
//! `dist_to_plane - border_radius <= prediction`. Convex polygons and compounds are deferred.
//!
//! BT3: a cuboid whose support distance is surely beyond the prediction leaves early with what
//! the full path would leave (`cuboid_beyond`; exact, the full path decides every doubtful case).
//! The ground's AABB overlaps every body, so most half-space pairs of a level are such pairs: a
//! far pair costs 321 Cairo steps instead of 1 359 (`gas_halfspace_cuboid_far*`), the level-10
//! impact tick's contact generation −4.8k.

use fixed::wide::{WideAdd, WideNarrow, WideSub, dot2, wide_mul};
use fixed::{Fixed, FixedTrait, ZERO};
use glam::Vec2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::contact::{ContactManifold, ContactManifoldTrait, TrackedContact};
use crate::feature_id::FeatureIdTrait;
use crate::manifold::ManifoldTrait;
use crate::polygonal_feature::PolygonalFeature;
use crate::shape::{ConvexPolygonTrait, Cuboid, CuboidTrait, HalfSpace, Segment, Shape};

fn dot(a: Vec2, b: Vec2) -> Fixed {
    dot2(a.x, b.x, a.y, b.y)
}

fn segment_feature(segment: Segment) -> PolygonalFeature {
    PolygonalFeature {
        vertices: [segment.a, segment.b],
        vids: [FeatureIdTrait::vertex(0), FeatureIdTrait::vertex(2)],
        fid: FeatureIdTrait::face(1),
        num_vertices: 2,
    }
}

fn push(
    ref manifold: ContactManifold,
    local_p1: Vec2,
    local_p2: Vec2,
    fid2: crate::feature_id::FeatureId,
    dist: Fixed,
    flipped: bool,
) {
    let contact = if flipped {
        TrackedContact {
            local_p1: local_p2,
            local_p2: local_p1,
            fid1: fid2,
            fid2: FeatureIdTrait::face(0),
            dist,
            ..Default::default(),
        }
    } else {
        TrackedContact {
            local_p1, local_p2, fid1: FeatureIdTrait::face(0), fid2, dist, ..Default::default(),
        }
    };
    let [p0, _] = manifold.points;
    manifold
        .points =
            if manifold.num_points == 0 {
                [contact, Default::default()]
            } else {
                [p0, contact]
            };
    manifold.num_points += 1;
}

fn generate_from_feature(
    pos12: Pose2,
    halfspace1: HalfSpace,
    normal1_2: Vec2,
    feature2: PolygonalFeature,
    border_radius2: Fixed,
    prediction: Fixed,
    ref manifold: ContactManifold,
    flipped: bool,
) {
    let [v0, v1] = feature2.vertices;
    let [fid0, fid1] = feature2.vids;
    if feature2.num_vertices != 0 {
        add_vertex(
            pos12,
            halfspace1.normal,
            normal1_2,
            v0,
            fid0,
            border_radius2,
            prediction,
            ref manifold,
            flipped,
        );
    }
    if feature2.num_vertices > 1 {
        add_vertex(
            pos12,
            halfspace1.normal,
            normal1_2,
            v1,
            fid1,
            border_radius2,
            prediction,
            ref manifold,
            flipped,
        );
    }
}

fn add_vertex(
    pos12: Pose2,
    normal1: Vec2,
    normal1_2: Vec2,
    vertex2: Vec2,
    fid2: crate::feature_id::FeatureId,
    border_radius2: Fixed,
    prediction: Fixed,
    ref manifold: ContactManifold,
    flipped: bool,
) {
    let vertex2_1 = pos12.transform_point(vertex2);
    let dist_to_plane = dot(vertex2_1, normal1);
    let dist = dist_to_plane - border_radius2;
    if dist <= prediction {
        push(
            ref manifold,
            vertex2_1 - Vec2 { x: normal1.x * dist_to_plane, y: normal1.y * dist_to_plane },
            vertex2 - Vec2 { x: normal1_2.x * border_radius2, y: normal1_2.y * border_radius2 },
            fid2,
            dist,
            flipped,
        );
    }
}

fn finish_normals(
    pos12: Pose2,
    halfspace1: HalfSpace,
    normal1_2: Vec2,
    ref manifold: ContactManifold,
    flipped: bool,
) {
    if flipped {
        manifold.local_n1 = -normal1_2;
        manifold.local_n2 = halfspace1.normal;
    } else {
        manifold.local_n1 = halfspace1.normal;
        manifold.local_n2 = -normal1_2;
    }
    let _ = pos12;
}

/// Margin of [`cuboid_beyond`], raw: `2^-16`, above its error bound while the half extents sum
/// to less than [`BEYOND_EXTENTS`].
const BEYOND_MARGIN: i64 = 0x10000;
/// Largest `hx + hy` (raw, 2^14 units) for which [`cuboid_beyond`] may conclude.
const BEYOND_EXTENTS: i64 = 0x400000000000;

/// `true` when no vertex of `cuboid2` can reach `prediction` (BT3: exact early-out of a
/// non-touching pair; the ground's AABB overlaps every body). The support distance
/// `n . t - |m.x| hx - |m.y| hy` (`m = R^T n`, here the floored `normal1_2`) is summed exactly
/// and floored once; it is below every vertex distance of the full path by at most
/// `(hx + hy) * 2^-32` (the flooring of `normal1_2`) plus four ulps (the floors of the vertex and
/// of its dot product), which [`BEYOND_MARGIN`] covers for extents below [`BEYOND_EXTENTS`].
/// Otherwise `false`, and the full path decides. Never changes a result.
#[inline(always)]
fn cuboid_beyond(
    pos12: Pose2, normal1: Vec2, normal1_2: Vec2, cuboid2: Cuboid, prediction: Fixed,
) -> bool {
    let h = cuboid2.half_extents;
    if h.x.raw + h.y.raw >= BEYOND_EXTENTS {
        return false;
    }
    let t = pos12.translation;
    let support = wide_mul(normal1.x, t.x)
        .add(wide_mul(normal1.y, t.y))
        .sub(wide_mul(normal1_2.x.abs(), h.x))
        .sub(wide_mul(normal1_2.y.abs(), h.y))
        .narrow();
    support.raw > prediction.raw + BEYOND_MARGIN
}

fn halfspace_cuboid_direct(
    pos12: Pose2,
    halfspace1: HalfSpace,
    cuboid2: Cuboid,
    prediction: Fixed,
    ref manifold: ContactManifold,
    flipped: bool,
) {
    let normal1_2 = pos12.inverse_transform_vector(halfspace1.normal);
    if cuboid_beyond(pos12, halfspace1.normal, normal1_2, cuboid2, prediction) {
        // What the full path leaves: no point pushed, the normals set, `match_contacts` on
        // an empty manifold changes nothing.
        manifold.clear();
        finish_normals(pos12, halfspace1, normal1_2, ref manifold, flipped);
        return;
    }
    let old = manifold;
    manifold.clear();
    generate_from_feature(
        pos12,
        halfspace1,
        normal1_2,
        cuboid2.support_feature(-normal1_2),
        ZERO,
        prediction,
        ref manifold,
        flipped,
    );
    finish_normals(pos12, halfspace1, normal1_2, ref manifold, flipped);
    manifold.match_contacts(@old);
}

fn halfspace_pfm_generic(
    pos12: Pose2,
    halfspace1: HalfSpace,
    pfm2: Shape,
    prediction: Fixed,
    ref manifold: ContactManifold,
    flipped: bool,
) {
    let normal1_2 = pos12.inverse_transform_vector(halfspace1.normal);
    let maybe = match pfm2 {
        Shape::Cuboid(c) => Some((c.support_feature(-normal1_2), ZERO)),
        Shape::ConvexPolygon(c) => Some((c.unbox().support_feature(-normal1_2), ZERO)),
        Shape::Segment(s) => Some((segment_feature(s), ZERO)),
        Shape::Capsule(c) => Some((segment_feature(c.segment), c.radius)),
        _ => None,
    };
    let old = manifold;
    manifold.clear();
    if let Some((feature, radius)) = maybe {
        generate_from_feature(
            pos12, halfspace1, normal1_2, feature, radius, prediction, ref manifold, flipped,
        );
        finish_normals(pos12, halfspace1, normal1_2, ref manifold, flipped);
    }
    manifold.match_contacts(@old);
}

/// Computes the manifold between `halfspace1` and a cuboid/segment/capsule `pfm2`.
///
/// `pos12` places shape 2 in the half-space frame. The half-space normal is assumed unit and no
/// normalization is performed. Contacts are retained when
/// `dist_to_plane - border_radius <= prediction`, matching upstream's inclusive comparison.
/// `flipped` swaps the stored local points, feature ids and manifold normals for reversed dispatch.
pub fn contact_manifold_halfspace_pfm(
    pos12: Pose2,
    halfspace1: HalfSpace,
    pfm2: Shape,
    prediction: Fixed,
    ref manifold: ContactManifold,
    flipped: bool,
) {
    match pfm2 {
        Shape::Cuboid(c) => {
            halfspace_cuboid_direct(pos12, halfspace1, c, prediction, ref manifold, flipped);
        },
        Shape::ConvexPolygon(_) => {
            halfspace_pfm_generic(pos12, halfspace1, pfm2, prediction, ref manifold, flipped);
        },
        Shape::Segment(_) => {
            halfspace_pfm_generic(pos12, halfspace1, pfm2, prediction, ref manifold, flipped);
        },
        Shape::Capsule(_) => {
            halfspace_pfm_generic(pos12, halfspace1, pfm2, prediction, ref manifold, flipped);
        },
        _ => {
            let old = manifold;
            manifold.clear();
            manifold.match_contacts(@old);
        },
    }
}

/// Dispatches the supported half-space/PFM pairs.
///
/// Returns `true` for halfspace-cuboid, halfspace-segment and halfspace-capsule in either order;
/// returns `false` for deferred shapes. Reversed pairs use `pos12.inverse()` and `flipped = true`.
pub fn contact_manifold_halfspace_pfm_shapes(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    match (shape1, shape2) {
        (
            Shape::HalfSpace(h), Shape::ConvexPolygon(_),
        ) => {
            contact_manifold_halfspace_pfm(pos12, h, shape2, prediction, ref manifold, false);
            true
        },
        (
            Shape::ConvexPolygon(_), Shape::HalfSpace(h),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12.inverse(), h, shape1, prediction, ref manifold, true,
            );
            true
        },
        (
            Shape::HalfSpace(h), Shape::Cuboid(c),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12, h, Shape::Cuboid(c), prediction, ref manifold, false,
            );
            true
        },
        (
            Shape::HalfSpace(h), Shape::Segment(s),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12, h, Shape::Segment(s), prediction, ref manifold, false,
            );
            true
        },
        (
            Shape::HalfSpace(h), Shape::Capsule(c),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12, h, Shape::Capsule(c), prediction, ref manifold, false,
            );
            true
        },
        (
            Shape::Cuboid(c), Shape::HalfSpace(h),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12.inverse(), h, Shape::Cuboid(c), prediction, ref manifold, true,
            );
            true
        },
        (
            Shape::Segment(s), Shape::HalfSpace(h),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12.inverse(), h, Shape::Segment(s), prediction, ref manifold, true,
            );
            true
        },
        (
            Shape::Capsule(c), Shape::HalfSpace(h),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12.inverse(), h, Shape::Capsule(c), prediction, ref manifold, true,
            );
            true
        },
        _ => false,
    }
}

#[cfg(test)]
mod alternatives {
    use fixed::{Fixed, ZERO};
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use crate::contact::{ContactManifold, ContactManifoldTrait};
    use crate::manifold::ManifoldTrait;
    use crate::shape::{Capsule, Cuboid, CuboidTrait, HalfSpace, Segment, Shape};
    use super::{finish_normals, generate_from_feature, halfspace_pfm_generic, segment_feature};

    /// The half-space–cuboid path before BT3's `cuboid_beyond` early-out (loser on the level's
    /// impact ticks: every non-touching ground pair built the support feature, transformed its
    /// two vertices and matched an empty manifold; see `gas_halfspace_cuboid_far*`).
    pub fn contact_manifold_halfspace_cuboid_full(
        pos12: Pose2,
        halfspace1: HalfSpace,
        cuboid2: Cuboid,
        prediction: Fixed,
        ref manifold: ContactManifold,
        flipped: bool,
    ) {
        let normal1_2 = pos12.inverse_transform_vector(halfspace1.normal);
        let old = manifold;
        manifold.clear();
        generate_from_feature(
            pos12,
            halfspace1,
            normal1_2,
            cuboid2.support_feature(-normal1_2),
            ZERO,
            prediction,
            ref manifold,
            flipped,
        );
        finish_normals(pos12, halfspace1, normal1_2, ref manifold, flipped);
        manifold.match_contacts(@old);
    }

    pub fn contact_manifold_halfspace_pfm_generic(
        pos12: Pose2,
        halfspace1: HalfSpace,
        pfm2: Shape,
        prediction: Fixed,
        ref manifold: ContactManifold,
        flipped: bool,
    ) {
        halfspace_pfm_generic(pos12, halfspace1, pfm2, prediction, ref manifold, flipped);
    }

    pub fn contact_manifold_halfspace_segment_direct(
        pos12: Pose2,
        halfspace1: HalfSpace,
        segment2: Segment,
        prediction: Fixed,
        ref manifold: ContactManifold,
        flipped: bool,
    ) {
        let normal1_2 = pos12.inverse_transform_vector(halfspace1.normal);
        let old = manifold;
        manifold.clear();
        generate_from_feature(
            pos12,
            halfspace1,
            normal1_2,
            segment_feature(segment2),
            ZERO,
            prediction,
            ref manifold,
            flipped,
        );
        finish_normals(pos12, halfspace1, normal1_2, ref manifold, flipped);
        manifold.match_contacts(@old);
    }

    pub fn contact_manifold_halfspace_capsule_direct(
        pos12: Pose2,
        halfspace1: HalfSpace,
        capsule2: Capsule,
        prediction: Fixed,
        ref manifold: ContactManifold,
        flipped: bool,
    ) {
        let normal1_2 = pos12.inverse_transform_vector(halfspace1.normal);
        let old = manifold;
        manifold.clear();
        generate_from_feature(
            pos12,
            halfspace1,
            normal1_2,
            segment_feature(capsule2.segment),
            capsule2.radius,
            prediction,
            ref manifold,
            flipped,
        );
        finish_normals(pos12, halfspace1, normal1_2, ref manifold, flipped);
        manifold.match_contacts(@old);
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
    use glam::Vec2;
    use rapier_math::consts::UNIT_TOL_SQ_RAW;
    use rapier_math::math_ext::norm2::is_unit2_raw;
    use rapier_math::pose2::{IDENTITY, Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::contact::{ContactManifold, ContactManifoldTrait};
    use crate::feature_id::FeatureIdTrait;
    use crate::shape::{Capsule, Cuboid, HalfSpace, Segment, Shape};
    use super::{
        alternatives, contact_manifold_halfspace_pfm, contact_manifold_halfspace_pfm_shapes,
    };

    const H: HalfSpace = HalfSpace { normal: Vec2 { x: ZERO, y: ONE } };
    const C: Cuboid = Cuboid { half_extents: Vec2 { x: ONE, y: HALF } };
    const N_ONE: Fixed = Fixed { raw: -4294967296 };
    const N_HALF: Fixed = Fixed { raw: -2147483648 };
    const S: Segment = Segment { a: Vec2 { x: N_ONE, y: N_HALF }, b: Vec2 { x: ONE, y: HALF } };
    const R: Rot2 = Rot2 { re: Fixed { raw: 3719550787 }, im: Fixed { raw: 2147483648 } };

    fn pose(x: Fixed, y: Fixed) -> Pose2 {
        Pose2 { translation: Vec2 { x, y }, ..IDENTITY }
    }

    #[test]
    fn test_cuboid_flat_tilted_and_flipped() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_halfspace_pfm(
            pose(ZERO, FixedTrait::from_ratio(2, 5)), H, Shape::Cuboid(C), ZERO, ref m, false,
        );
        assert_eq!(m.num_points, 2);
        assert_eq!(m.local_n1, H.normal);
        assert_eq!(m.local_n2, -H.normal);
        assert_eq!(m.point(0).dist, Fixed { raw: -429496730 });
        assert_eq!(m.point(0).fid1, FeatureIdTrait::face(0));
        assert_eq!(m.point(0).fid2, FeatureIdTrait::vertex(2));
        assert_eq!(m.point(1).fid2, FeatureIdTrait::vertex(3));

        let p = Pose2 {
            translation: Vec2 { x: ZERO, y: FixedTrait::from_ratio(9, 10) }, rotation: R,
        };
        contact_manifold_halfspace_pfm(p, H, Shape::Cuboid(C), ZERO, ref m, false);
        assert_eq!(m.num_points, 1);
        assert_eq!(m.point(0).fid2, FeatureIdTrait::vertex(3));
        assert!(is_unit2_raw(m.local_n1.x, m.local_n1.y, UNIT_TOL_SQ_RAW));

        let mut flipped: ContactManifold = Default::default();
        assert!(
            contact_manifold_halfspace_pfm_shapes(
                p.inverse(), Shape::Cuboid(C), Shape::HalfSpace(H), ZERO, ref flipped,
            ),
        );
        assert_eq!(flipped.local_n1, m.local_n2);
        assert_eq!(flipped.local_n2, m.local_n1);
        assert_eq!(flipped.point(0).local_p1, m.point(0).local_p2);
        assert_eq!(flipped.point(0).fid2, FeatureIdTrait::face(0));
    }

    #[test]
    fn test_capsule_radius_and_segment_prediction() {
        let cap = Capsule {
            segment: Segment { a: Vec2 { x: -ONE, y: ZERO }, b: Vec2 { x: ONE, y: ZERO } },
            radius: HALF,
        };
        let mut m: ContactManifold = Default::default();
        contact_manifold_halfspace_pfm(
            pose(ZERO, ZERO), H, Shape::Capsule(cap), ZERO, ref m, false,
        );
        assert_eq!(m.num_points, 2);
        assert_eq!(m.point(0).dist, -HALF);
        assert_eq!(m.point(0).local_p2.y, -HALF);
        assert_eq!(m.point(1).local_p2.y, -HALF);

        contact_manifold_halfspace_pfm(pose(ZERO, ZERO), H, Shape::Segment(S), ZERO, ref m, false);
        assert_eq!(m.num_points, 1);
        assert_eq!(m.point(0).fid2, FeatureIdTrait::vertex(0));
        contact_manifold_halfspace_pfm(pose(ZERO, ZERO), H, Shape::Segment(S), HALF, ref m, false);
        assert_eq!(m.num_points, 2);
    }

    #[test]
    #[fuzzer(runs: 64, seed: 20260921)]
    fn fuzz_direct_matches_generic(x: i16, y: i16) {
        let p = Pose2 {
            translation: Vec2 {
                x: Fixed { raw: x.into() * 65536 }, y: Fixed { raw: y.into() * 65536 },
            },
            rotation: R,
        };
        for shape in array![Shape::Cuboid(C), Shape::Segment(S)].span() {
            let mut a: ContactManifold = Default::default();
            let mut b: ContactManifold = Default::default();
            contact_manifold_halfspace_pfm(p, H, *shape, HALF, ref a, false);
            alternatives::contact_manifold_halfspace_pfm_generic(p, H, *shape, HALF, ref b, false);
            assert_eq!(a.num_points, b.num_points);
            assert_eq!(a.points, b.points);
            assert_eq!(a.local_n1, b.local_n1);
            assert_eq!(a.local_n2, b.local_n2);
        }
    }

    /// BT3's early-out against the full path, from a warm manifold (its stale points must
    /// survive exactly), on both sides of the prediction boundary, rotated, flipped.
    #[test]
    fn test_cuboid_early_out_matches_full() {
        let mut warm: ContactManifold = Default::default();
        contact_manifold_halfspace_pfm(
            pose(ZERO, HALF), H, Shape::Cuboid(C), HALF, ref warm, false,
        );
        // (y raw, rotated, flipped): the lowest vertex reaches the prediction (1/2) at y = 1.
        // Far above; beyond the early-out margin (2^-16); within it (full path, no point); one
        // ulp beyond; exactly at the prediction (a point); inside; penetrating.
        let cases = array![
            (0x500000000_i64, false, false), (0x500000000, true, true), (0x100010001, false, false),
            (0x100010000, false, true), (0x100000001, true, false), (0x100000000, false, false),
            (0xc0000000, true, true), (-0x10000, false, false),
        ];
        for (y, rotated, flipped) in cases {
            let mut p = pose(ZERO, Fixed { raw: y });
            if rotated {
                p.rotation = R;
            }
            let mut a = warm;
            let mut b = warm;
            contact_manifold_halfspace_pfm(p, H, Shape::Cuboid(C), HALF, ref a, flipped);
            alternatives::contact_manifold_halfspace_cuboid_full(p, H, C, HALF, ref b, flipped);
            assert_eq!(a, b);
        }
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(IDENTITY);
    }

    /// A cuboid one unit above the prediction: the early-out, then the full path.
    #[test]
    fn gas_halfspace_cuboid_far() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_halfspace_pfm(
            opaque(pose(ZERO, FixedTrait::from_int(3))), H, Shape::Cuboid(C), HALF, ref m, false,
        );
    }

    #[test]
    fn gas_halfspace_cuboid_far_full() {
        let mut m: ContactManifold = Default::default();
        alternatives::contact_manifold_halfspace_cuboid_full(
            opaque(pose(ZERO, FixedTrait::from_int(3))), H, C, HALF, ref m, false,
        );
    }

    /// A touching cuboid through the full path (`gas_halfspace_cuboid_direct` is the winner).
    #[test]
    fn gas_halfspace_cuboid_touching_full() {
        let mut m: ContactManifold = Default::default();
        alternatives::contact_manifold_halfspace_cuboid_full(
            opaque(IDENTITY), H, C, ZERO, ref m, false,
        );
    }

    #[test]
    fn gas_halfspace_cuboid_direct() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_halfspace_pfm(opaque(IDENTITY), H, Shape::Cuboid(C), ZERO, ref m, false);
    }

    #[test]
    fn gas_halfspace_segment_winner() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_halfspace_pfm(opaque(IDENTITY), H, Shape::Segment(S), HALF, ref m, false);
    }

    #[test]
    fn gas_halfspace_capsule_winner() {
        let mut m: ContactManifold = Default::default();
        let cap = Capsule { segment: S, radius: HALF };
        contact_manifold_halfspace_pfm(
            opaque(IDENTITY), H, Shape::Capsule(cap), ZERO, ref m, false,
        );
    }

    #[test]
    fn gas_halfspace_cuboid_generic() {
        let mut m: ContactManifold = Default::default();
        alternatives::contact_manifold_halfspace_pfm_generic(
            opaque(IDENTITY), H, Shape::Cuboid(C), ZERO, ref m, false,
        );
    }

    #[test]
    fn gas_halfspace_segment_generic() {
        let mut m: ContactManifold = Default::default();
        alternatives::contact_manifold_halfspace_pfm_generic(
            opaque(IDENTITY), H, Shape::Segment(S), HALF, ref m, false,
        );
    }

    #[test]
    fn gas_halfspace_capsule_generic() {
        let mut m: ContactManifold = Default::default();
        let cap = Capsule { segment: S, radius: HALF };
        alternatives::contact_manifold_halfspace_pfm_generic(
            opaque(IDENTITY), H, Shape::Capsule(cap), ZERO, ref m, false,
        );
    }

    #[test]
    fn gas_halfspace_segment_direct_candidate() {
        let mut m: ContactManifold = Default::default();
        alternatives::contact_manifold_halfspace_segment_direct(
            opaque(IDENTITY), H, S, HALF, ref m, false,
        );
    }

    #[test]
    fn gas_halfspace_capsule_direct_candidate() {
        let mut m: ContactManifold = Default::default();
        let cap = Capsule { segment: S, radius: HALF };
        alternatives::contact_manifold_halfspace_capsule_direct(
            opaque(IDENTITY), H, cap, ZERO, ref m, false,
        );
    }
}
