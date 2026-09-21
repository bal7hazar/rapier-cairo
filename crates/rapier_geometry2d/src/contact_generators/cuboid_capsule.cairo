//! Cuboid–capsule contact manifold (Parry `contact_manifolds_cuboid_capsule.rs`, 2D branch).
//!
//! Upstream ships this analytic generator but comments it out of its dispatcher, which sends the
//! pair to the generic PFM–PFM (GJK/EPA) path instead; this port re-enables it (report 02 §4.1).
//! Steps: the persistence fast path, the cuboid's four face axes against the capsule, the
//! capsule segment's normal against the cuboid, then the cuboid face most aligned with the best
//! axis clipped against the capsule segment (`PolygonalFeature::contacts`), the radius applied
//! last on the capsule side.
//!
//! Upstream's 2D branch names a `CuboidFeature::face_face_contacts` that no longer exists; its
//! behaviour is `PolygonalFeature::face_face_contacts`, which `PolygonalFeatureTrait::contacts`
//! reaches for two 2-vertex features.
//!
//! Fixed point: `cuboid_support_map_find_local_separating_normal_oneway` (GD) already subtracts
//! the radius; the subtraction is exact, so `sep1 > prediction` is upstream's
//! `sep1 > prediction + radius` and `sep1 + radius` is upstream's core separation. The only
//! normalisation is the segment normal inside `segment_cuboid_find_local_separating_normal_oneway`.
use fixed::Fixed;
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::contact::{ContactManifold, ContactManifoldTrait};
use crate::feature_id::FeatureIdTrait;
use crate::manifold::ManifoldTrait;
use crate::polygonal_feature::{PolygonalFeature, PolygonalFeatureTrait};
use crate::sat::{
    cuboid_support_map_find_local_separating_normal_oneway,
    segment_cuboid_find_local_separating_normal_oneway,
};
use crate::shape::{Capsule, Cuboid, CuboidTrait, Segment, Shape};

/// Upstream `PolygonalFeature::from(Segment)`: vertices `a`, `b` with ids `vertex(0)`,
/// `vertex(2)`, face id `face(1)`.
#[inline(always)]
fn segment_feature(s: Segment) -> PolygonalFeature {
    PolygonalFeature {
        vertices: [s.a, s.b],
        vids: [FeatureIdTrait::vertex(0), FeatureIdTrait::vertex(2)],
        fid: FeatureIdTrait::face(1),
        num_vertices: 2,
    }
}

/// Moves every point of the capsule side by `offset` and subtracts `radius` from `dist`.
#[inline(always)]
fn inflate(ref manifold: ContactManifold, offset: Vec2, radius: Fixed, flipped: bool) {
    let [mut p0, mut p1] = manifold.points;
    if flipped {
        p0.local_p1 = p0.local_p1 + offset;
        p1.local_p1 = p1.local_p1 + offset;
    } else {
        p0.local_p2 = p0.local_p2 + offset;
        p1.local_p2 = p1.local_p2 + offset;
    }
    p0.dist = p0.dist - radius;
    p1.dist = p1.dist - radius;
    manifold.points = [p0, p1];
}

/// Upstream `contact_manifold_cuboid_capsule(pos12, pos21, cube1, capsule2, ..., flipped)`:
/// `pos21 = pos12.inverse()`; with `flipped`, the manifold's shape 1 is the capsule and its
/// relative pose is `pos21`.
#[inline(never)]
fn cuboid_capsule(
    pos12: Pose2,
    pos21: Pose2,
    cube1: Cuboid,
    capsule2: Capsule,
    prediction: Fixed,
    ref manifold: ContactManifold,
    flipped: bool,
) {
    let manifold_pos12 = if flipped {
        pos21
    } else {
        pos12
    };
    if manifold.try_update_contacts(manifold_pos12) {
        return;
    }
    let radius = capsule2.radius;
    let segment2 = capsule2.segment;
    // Vertex–face cases.
    let (sep1, axis1) = cuboid_support_map_find_local_separating_normal_oneway(
        cube1, capsule2, pos12,
    );
    if sep1 > prediction {
        manifold.clear();
        return;
    }
    let (sep2, axis2) = segment_cuboid_find_local_separating_normal_oneway(segment2, cube1, pos21);
    if sep2 > prediction + radius {
        manifold.clear();
        return;
    }
    // No edge–edge case in 2D. Best axis, ties to the cuboid's.
    let normal1 = if sep2 > sep1 + radius {
        pos12.rotation.rotate(-axis2)
    } else {
        axis1
    };
    let normal2 = pos21.rotation.rotate(-normal1);
    let feature1 = cube1.support_feature(normal1);
    let feature2 = segment_feature(segment2);

    let old = manifold;
    manifold.clear();
    PolygonalFeatureTrait::contacts(
        pos12, pos21, normal1, normal2, feature1, feature2, ref manifold, flipped,
    );
    inflate(ref manifold, normal2.mul_scalar(radius), radius, flipped);
    if flipped {
        manifold.local_n1 = normal2;
        manifold.local_n2 = normal1;
    } else {
        manifold.local_n1 = normal1;
        manifold.local_n2 = normal2;
    }
    manifold.match_contacts(@old);
}

/// Contact manifold between a cuboid and a capsule (Parry `contact_manifold_cuboid_capsule`,
/// analytic 2D branch).
///
/// `pos12` is the pose of `capsule2` in the frame of `cuboid1` (unit rotation). Starts with the
/// `try_update_contacts` fast path; clears the manifold when either SAT axis separates by more
/// than `prediction` (radius included). Otherwise 0 to 2 clipped points, which may lie beyond
/// `prediction`. Feature ids: the cuboid's support-face vertex / face ids (`vertex(0..3)`,
/// `face(48 + ..)`), the capsule segment's `vertex(0)`, `face(1)`, `vertex(2)`.
/// #### Panics
/// * As `PolygonalFeatureTrait::contacts` and the SAT helpers (coordinates beyond Q32.32).
pub fn contact_manifold_cuboid_capsule(
    pos12: Pose2,
    cuboid1: Cuboid,
    capsule2: Capsule,
    prediction: Fixed,
    ref manifold: ContactManifold,
) {
    cuboid_capsule(pos12, pos12.inverse(), cuboid1, capsule2, prediction, ref manifold, false);
}

/// [`contact_manifold_cuboid_capsule`] for two [`Shape`]s, in either order: a cuboid–capsule
/// pair, or a capsule–cuboid pair handled by swapping the shapes and flipping the output (as
/// upstream). Returns `false` (manifold untouched) for any other pair.
pub fn contact_manifold_cuboid_capsule_shapes(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    match shape1 {
        Shape::Cuboid(cuboid1) => {
            if let Shape::Capsule(capsule2) = shape2 {
                cuboid_capsule(
                    pos12, pos12.inverse(), cuboid1, capsule2, prediction, ref manifold, false,
                );
                return true;
            }
            false
        },
        Shape::Capsule(capsule1) => {
            if let Shape::Cuboid(cuboid2) = shape2 {
                cuboid_capsule(
                    pos12.inverse(), pos12, cuboid2, capsule1, prediction, ref manifold, true,
                );
                return true;
            }
            false
        },
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
    use glam::Vec2;
    use rapier_golden::contact_manifolds::{
        CAPSULE_CUBOID_SHALLOW, CUBOID_CAPSULE_DEEP, CUBOID_CAPSULE_DEGENERATE,
        CUBOID_CAPSULE_SEPARATED, CUBOID_CAPSULE_SHALLOW, CUBOID_CAPSULE_TOUCHING,
        CUBOID_CAPSULE_WITHIN_PRED, PREDICTION,
    };
    use rapier_golden::types::{ManifoldCase, ShapeRaw, Vec2Raw};
    use rapier_math::pose2::{IDENTITY, Pose2, Pose2Trait};
    use rapier_math::rot2::{Rot2, Rot2Trait};
    use rapier_testing::opaque;
    use crate::contact::{ContactData, ContactManifold, ContactManifoldTrait, TrackedContact};
    use crate::feature_id::FeatureIdTrait;
    use crate::shape::{Ball, Capsule, CapsuleTrait, Cuboid, CuboidTrait, Shape};
    use super::{contact_manifold_cuboid_capsule, contact_manifold_cuboid_capsule_shapes};

    fn f(n: i64, d: i64) -> Fixed {
        FixedTrait::from_raw(n * 0x1_0000_0000 / d)
    }
    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }
    fn at(x: Fixed, y: Fixed) -> Pose2 {
        Pose2 { translation: v(x, y), ..IDENTITY }
    }
    fn quarter(x: Fixed, y: Fixed) -> Pose2 {
        Pose2 { translation: v(x, y), rotation: Rot2 { re: ZERO, im: ONE } }
    }
    fn vr(r: Vec2Raw) -> Vec2 {
        v(FixedTrait::from_raw(r.x), FixedTrait::from_raw(r.y))
    }
    fn shape_of(s: ShapeRaw) -> Shape {
        match s {
            ShapeRaw::Capsule(c) => Shape::Capsule(
                CapsuleTrait::new(vr(c.a), vr(c.b), FixedTrait::from_raw(c.radius)),
            ),
            ShapeRaw::Cuboid(h) => Shape::Cuboid(CuboidTrait::new(vr(h))),
            _ => panic!("unexpected shape"),
        }
    }
    fn input(c: ManifoldCase) -> (Pose2, Shape, Shape) {
        let p = c.pos12;
        let pose = Pose2 {
            translation: vr(p.translation),
            rotation: Rot2 {
                re: FixedTrait::from_raw(p.rotation.re), im: FixedTrait::from_raw(p.rotation.im),
            },
        };
        (pose, shape_of(c.shape1), shape_of(c.shape2))
    }
    fn cuboid() -> Cuboid {
        CuboidTrait::new(v(ONE, HALF))
    }
    fn capsule() -> Capsule {
        CapsuleTrait::new_y(HALF, f(1, 4))
    }
    fn run(pos12: Pose2, c: Capsule, prediction: Fixed) -> ContactManifold {
        let mut m: ContactManifold = Default::default();
        contact_manifold_cuboid_capsule(pos12, cuboid(), c, prediction, ref m);
        m
    }

    fn near(a: Vec2, b: Vec2) -> bool {
        (a.x - b.x).abs().raw <= 4 && (a.y - b.y).abs().raw <= 4
    }
    fn min_dist(m: ContactManifold) -> Fixed {
        if m.num_points == 2 {
            m.point(0).dist.min(m.point(1).dist)
        } else {
            m.point(0).dist
        }
    }

    /// `(pose of the capsule, capsule, points, deepest dist, normal)` against the 2 x 1 box.
    #[test]
    fn test_regimes_table() {
        let p = f(1, 10);
        let lying = CapsuleTrait::new_x(HALF, f(1, 4));
        let cases: Span<(Pose2, Capsule, u8, Fixed, Vec2)> = array![
            // Lying on the top face, penetrating by 0.05: two points.
            (at(ZERO, f(7, 10)), lying, 2, f(-1, 20), v(ZERO, ONE)),
            // Standing on the top-right corner, end cap first: both clip points coincide.
            (at(ONE, f(5, 4) - f(1, 20)), capsule(), 2, f(-1, 20), v(ZERO, ONE)),
            // Right of the box, parallel to the face, within prediction.
            (at(f(5, 4) + f(1, 20), ZERO), capsule(), 2, f(1, 20), v(ONE, ZERO)),
            // Beyond prediction.
            (at(f(3, 1), ZERO), capsule(), 0, ZERO, v(ZERO, ZERO)),
            // Lying across the right face (quarter turn), deep: face +x.
            (quarter(f(3, 2), ZERO), capsule(), 2, f(-1, 4), v(ONE, ZERO)),
        ]
            .span();
        for (pose, c, n, dist, normal) in cases {
            let m = run(*pose, *c, p);
            assert_eq!(m.num_points, *n);
            if *n != 0 {
                assert!((min_dist(m) - *dist).abs().raw <= 2, "dist {:?}", min_dist(m));
                assert_eq!(m.local_n1, *normal);
                assert_eq!(m.local_n2, pose.rotation.inverse_rotate(-*normal));
            }
        }
    }

    #[test]
    fn test_lying_points_and_ids() {
        let m = run(at(ZERO, f(7, 10)), CapsuleTrait::new_x(HALF, f(1, 4)), ZERO);
        assert_eq!(m.num_points, 2);
        // Clipped against the top face (vertices (1, 1/2) and (-1, 1/2)): the capsule's ends.
        let top = FeatureIdTrait::face(48 + 1 * 4 + 0);
        let expected = array![
            (v(HALF, HALF), v(HALF, f(-1, 4)), top, FeatureIdTrait::vertex(2)),
            (v(-HALF, HALF), v(-HALF, f(-1, 4)), top, FeatureIdTrait::vertex(0)),
        ];
        let mut i = 0;
        for (p1, p2, f1, f2) in expected.span() {
            let c = m.point(i);
            assert_eq!((c.local_p1, c.local_p2, c.fid1, c.fid2), (*p1, *p2, *f1, *f2));
            i += 1;
        }
    }

    #[test]
    fn test_flipped_matches_swapped_output() {
        let pose = Pose2 {
            translation: v(f(13, 10), f(1, 5)), rotation: Rot2 { re: f(4, 5), im: f(3, 5) },
        };
        let mut a: ContactManifold = Default::default();
        let (s1, s2) = (Shape::Cuboid(cuboid()), Shape::Capsule(capsule()));
        assert!(contact_manifold_cuboid_capsule_shapes(pose, s1, s2, ONE, ref a));
        let mut b: ContactManifold = Default::default();
        assert!(contact_manifold_cuboid_capsule_shapes(pose.inverse(), s2, s1, ONE, ref b));
        assert_eq!(a.num_points, b.num_points);
        assert_eq!((a.local_n1, a.local_n2), (b.local_n2, b.local_n1));
        let mut i = 0;
        while i != a.num_points {
            let (p, q) = (a.point(i), b.point(i));
            assert_eq!((p.fid1, p.fid2), (q.fid2, q.fid1));
            assert!((p.dist - q.dist).abs().raw <= 4);
            assert!(near(p.local_p1, q.local_p2) && near(p.local_p2, q.local_p1));
            i += 1;
        }
    }

    #[test]
    fn test_fast_path_warm_start_and_dispatch() {
        let pose = at(ZERO, f(7, 10));
        let lying = CapsuleTrait::new_x(HALF, f(1, 4));
        let mut m = run(pose, lying, ZERO);
        let data = ContactData { impulse: ONE, ..Default::default() };
        let [p0, p1] = m.points;
        m.points = [TrackedContact { data, ..p0 }, p1];
        // A tiny move keeps the points (fast path): only `dist` changes.
        let moved = at(ZERO, f(7, 10) + FixedTrait::from_raw(1000));
        contact_manifold_cuboid_capsule(moved, cuboid(), lying, ZERO, ref m);
        assert_eq!(m.point(0).data, data);
        assert!((m.point(0).dist - f(-1, 20) - FixedTrait::from_raw(1000)).abs().raw <= 2);
        // A full regeneration (moved sideways by 1/4) keeps the impulse through the feature ids.
        contact_manifold_cuboid_capsule(at(f(1, 4), f(7, 10)), cuboid(), lying, ZERO, ref m);
        assert_eq!(m.num_points, 2);
        assert_eq!(m.point(0).data, data);
        assert_eq!(m.point(0).local_p2, v(HALF, f(-1, 4)));
        let ball = Shape::Ball(Ball { radius: ONE });
        let before = m;
        assert!(
            !contact_manifold_cuboid_capsule_shapes(pose, ball, Shape::Capsule(lying), ZERO, ref m),
        );
        assert!(
            !contact_manifold_cuboid_capsule_shapes(
                pose, Shape::Cuboid(cuboid()), ball, ZERO, ref m,
            ),
        );
        assert_eq!(m, before);
    }

    fn probe(c: ManifoldCase) {
        let (pose, s1, s2) = input(opaque(c));
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold_cuboid_capsule_shapes(
            pose, s1, s2, FixedTrait::from_raw(PREDICTION), ref m,
        );
    }

    #[test]
    fn gas_baseline() {
        let (pose, s1, s2) = input(opaque(CUBOID_CAPSULE_DEEP));
        let _ = (pose, s1, s2);
        let _: ContactManifold = Default::default();
    }
    #[test]
    fn gas_cuboid_capsule_separated() {
        probe(CUBOID_CAPSULE_SEPARATED);
    }
    #[test]
    fn gas_cuboid_capsule_within_pred() {
        probe(CUBOID_CAPSULE_WITHIN_PRED);
    }
    #[test]
    fn gas_cuboid_capsule_touching() {
        probe(CUBOID_CAPSULE_TOUCHING);
    }
    #[test]
    fn gas_cuboid_capsule_shallow() {
        probe(CUBOID_CAPSULE_SHALLOW);
    }
    #[test]
    fn gas_cuboid_capsule_deep() {
        probe(CUBOID_CAPSULE_DEEP);
    }
    #[test]
    fn gas_cuboid_capsule_degenerate() {
        probe(CUBOID_CAPSULE_DEGENERATE);
    }
    #[test]
    fn gas_capsule_cuboid_shallow() {
        probe(CAPSULE_CUBOID_SHALLOW);
    }
    #[test]
    fn gas_contact_manifold_cuboid_capsule() {
        let (pose, _, _) = input(opaque(CUBOID_CAPSULE_SHALLOW));
        let mut m: ContactManifold = Default::default();
        contact_manifold_cuboid_capsule(
            pose, cuboid(), capsule(), FixedTrait::from_raw(PREDICTION), ref m,
        );
    }
}
