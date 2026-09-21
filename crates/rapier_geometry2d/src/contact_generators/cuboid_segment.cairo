//! Cuboid-segment contact manifolds.
//!
//! This is the 2D polygonal-feature-map path specialized to cuboid/segment: use the GD SAT
//! helpers for the separating normal, then clip the two polygonal features exactly like Parry.

use fixed::{Fixed, MAX, ZERO};
use glam::Vec2;
use rapier_math::consts::{COS_1_DEGREES, DIST_SQ_THRESHOLD_RAW};
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::contact::{ContactManifold, ContactManifoldTrait};
use crate::feature_id::FeatureIdTrait;
use crate::manifold::ManifoldTrait;
use crate::polygonal_feature::{PolygonalFeature, PolygonalFeatureTrait};
use crate::sat::{
    cuboid_segment_find_local_separating_normal_oneway,
    segment_cuboid_find_local_separating_normal_oneway,
};
use crate::shape::{Cuboid, CuboidTrait, Segment, Shape};

fn segment_feature(segment: Segment) -> PolygonalFeature {
    PolygonalFeature {
        vertices: [segment.a, segment.b],
        vids: [FeatureIdTrait::vertex(0), FeatureIdTrait::vertex(2)],
        fid: FeatureIdTrait::face(1),
        num_vertices: 2,
    }
}

fn choose_normal(pos12: Pose2, sep1: (Fixed, Vec2), sep2: (Fixed, Vec2)) -> (Fixed, Vec2) {
    let (dist1, normal1) = sep1;
    let (dist2, normal2) = sep2;
    if dist2 > dist1 {
        (dist2, pos12.transform_vector(-normal2))
    } else {
        (dist1, normal1)
    }
}

fn contact_manifold_cuboid_segment_with_flip(
    pos12: Pose2,
    pos21: Pose2,
    cuboid1: Cuboid,
    segment2: Segment,
    prediction: Fixed,
    ref manifold: ContactManifold,
    flipped: bool,
) {
    if (!flipped && manifold.try_update_contacts_eps(pos12, COS_1_DEGREES, DIST_SQ_THRESHOLD_RAW))
        || (flipped
            && manifold.try_update_contacts_eps(pos21, COS_1_DEGREES, DIST_SQ_THRESHOLD_RAW)) {
        return;
    }

    let sep1 = cuboid_segment_find_local_separating_normal_oneway(cuboid1, segment2, pos12);
    let (sep1_dist, _) = sep1;
    if sep1_dist > prediction {
        manifold.clear();
        return;
    }

    let sep2 = segment_cuboid_find_local_separating_normal_oneway(segment2, cuboid1, pos21);
    let (sep2_dist, _) = sep2;
    if sep2_dist > prediction {
        manifold.clear();
        return;
    }

    let sep3 = (-MAX, Vec2 { x: fixed::ONE, y: ZERO });
    let (sep3_dist, _) = sep3;
    let (dist, normal1) = if sep3_dist > sep1_dist && sep3_dist > sep2_dist {
        sep3
    } else {
        choose_normal(pos12, sep1, sep2)
    };
    let _ = dist;
    let normal2 = pos21.transform_vector(-normal1);
    let feature1 = cuboid1.support_feature(normal1);
    let feature2 = segment_feature(segment2);
    let old = manifold;
    manifold.clear();
    PolygonalFeatureTrait::contacts(
        pos12, pos21, normal1, normal2, feature1, feature2, ref manifold, flipped,
    );
    if flipped {
        manifold.local_n1 = normal2;
        manifold.local_n2 = normal1;
    } else {
        manifold.local_n1 = normal1;
        manifold.local_n2 = normal2;
    }
    manifold.match_contacts(@old);
}

/// Computes the manifold between a cuboid and a segment.
///
/// `pos12` places the segment in the cuboid frame. The fast path refreshes existing contacts
/// with upstream's 1-degree and `1e-6` squared-distance tolerances. If either one-way SAT
/// separation is greater than `prediction`, the manifold is cleared; otherwise the axis with the
/// larger separation is clipped. Points beyond prediction are not filtered after clipping.
pub fn contact_manifold_cuboid_segment(
    pos12: Pose2,
    cuboid1: Cuboid,
    segment2: Segment,
    prediction: Fixed,
    ref manifold: ContactManifold,
) {
    contact_manifold_cuboid_segment_with_flip(
        pos12, pos12.inverse(), cuboid1, segment2, prediction, ref manifold, false,
    );
}

/// Dispatches cuboid-segment in either order.
///
/// Returns `true` for cuboid/segment and segment/cuboid, `false` otherwise. Reversed pairs use
/// `pos12.inverse()` internally and flip the stored points, ids and normals.
pub fn contact_manifold_cuboid_segment_shapes(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    match (shape1, shape2) {
        (
            Shape::Cuboid(c), Shape::Segment(s),
        ) => {
            contact_manifold_cuboid_segment(pos12, c, s, prediction, ref manifold);
            true
        },
        (
            Shape::Segment(s), Shape::Cuboid(c),
        ) => {
            contact_manifold_cuboid_segment_with_flip(
                pos12.inverse(), pos12, c, s, prediction, ref manifold, true,
            );
            true
        },
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{IDENTITY, Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::contact::{ContactData, ContactManifold, ContactManifoldTrait};
    use crate::feature_id::FeatureIdTrait;
    use crate::shape::{Cuboid, Segment, Shape};
    use super::{contact_manifold_cuboid_segment, contact_manifold_cuboid_segment_shapes};

    const C: Cuboid = Cuboid { half_extents: Vec2 { x: ONE, y: ONE } };
    const N_HALF: Fixed = Fixed { raw: -2147483648 };
    const S: Segment = Segment { a: Vec2 { x: N_HALF, y: N_HALF }, b: Vec2 { x: HALF, y: N_HALF } };
    const R: Rot2 = Rot2 { re: Fixed { raw: 3037000500 }, im: Fixed { raw: 3037000500 } };

    fn pose(x: Fixed, y: Fixed) -> Pose2 {
        Pose2 { translation: Vec2 { x, y }, ..IDENTITY }
    }

    #[test]
    fn test_flat_crossing_separated_and_flipped() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_cuboid_segment(
            pose(ZERO, FixedTrait::from_ratio(13, 10)), C, S, ZERO, ref m,
        );
        assert_eq!(m.num_points, 2);
        assert_eq!(m.local_n1, Vec2 { x: ZERO, y: ONE });
        assert_eq!(m.point(0).dist, Fixed { raw: -858993460 });
        assert_eq!(m.point(0).fid2, FeatureIdTrait::vertex(2));
        assert_eq!(m.point(1).fid2, FeatureIdTrait::vertex(0));

        contact_manifold_cuboid_segment(pose(ZERO, FixedTrait::from_int(3)), C, S, ZERO, ref m);
        assert_eq!(m.num_points, 0);

        let tilted = Pose2 {
            translation: Vec2 { x: ZERO, y: FixedTrait::from_ratio(13, 10) }, rotation: R,
        };
        contact_manifold_cuboid_segment(tilted, C, S, ZERO, ref m);
        assert!(m.num_points != 0);
        let mut flipped: ContactManifold = Default::default();
        assert!(
            contact_manifold_cuboid_segment_shapes(
                tilted.inverse(), Shape::Segment(S), Shape::Cuboid(C), ZERO, ref flipped,
            ),
        );
        assert_eq!(flipped.num_points, m.num_points);
        assert_eq!(flipped.local_n1, m.local_n2);
        assert_eq!(flipped.point(0).local_p1, m.point(0).local_p2);
    }

    #[test]
    fn test_fast_path_keeps_warmstart_data() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_cuboid_segment(
            pose(ZERO, FixedTrait::from_ratio(13, 10)), C, S, ZERO, ref m,
        );
        let [mut p0, p1] = m.points;
        p0.data = ContactData { impulse: ONE, ..Default::default() };
        m.points = [p0, p1];
        contact_manifold_cuboid_segment(
            pose(ZERO, FixedTrait::from_ratio(13, 10)), C, S, ZERO, ref m,
        );
        assert_eq!(m.point(0).data.impulse, ONE);
    }

    #[test]
    #[fuzzer(runs: 64, seed: 20260922)]
    fn fuzz_dispatch_matches_direct(x: i16, y: i16) {
        let p = Pose2 {
            translation: Vec2 {
                x: Fixed { raw: x.into() * 65536 }, y: Fixed { raw: y.into() * 65536 },
            },
            rotation: R,
        };
        let mut a: ContactManifold = Default::default();
        let mut b: ContactManifold = Default::default();
        contact_manifold_cuboid_segment(p, C, S, HALF, ref a);
        assert!(
            contact_manifold_cuboid_segment_shapes(
                p, Shape::Cuboid(C), Shape::Segment(S), HALF, ref b,
            ),
        );
        assert_eq!(a.num_points, b.num_points);
        assert_eq!(a.points, b.points);
        assert_eq!(a.local_n1, b.local_n1);
        assert_eq!(a.local_n2, b.local_n2);
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(IDENTITY);
    }

    #[test]
    fn gas_cuboid_segment_touching() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_cuboid_segment(
            opaque(pose(ZERO, FixedTrait::from_ratio(3, 2))), C, S, ZERO, ref m,
        );
    }

    #[test]
    fn gas_cuboid_segment_tilted() {
        let mut m: ContactManifold = Default::default();
        let p = Pose2 {
            translation: Vec2 { x: ZERO, y: FixedTrait::from_ratio(13, 10) }, rotation: R,
        };
        contact_manifold_cuboid_segment(opaque(p), C, S, ZERO, ref m);
    }

    #[test]
    fn gas_cuboid_segment_flipped_shapes() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_cuboid_segment_shapes(
            opaque(pose(ZERO, FixedTrait::from_ratio(-13, 10))),
            Shape::Segment(S),
            Shape::Cuboid(C),
            ZERO,
            ref m,
        );
    }
}
