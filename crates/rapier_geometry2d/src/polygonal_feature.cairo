//! Two-vertex features and contact generation. Both features passed to `clip`
//! are in the same frame; `contacts` handles distinct local frames and flipping.
//! Coordinates/displacements must fit Q32.32; all dot/pose products floor once.
use fixed::wide::dot2;
use fixed::{Fixed, ZERO};
use glam::Vec2;
use rapier_math::pose2::{IDENTITY, Pose2, Pose2Trait};
use crate::clip::{ClippingPoints, clip_segment_segment_with_normal};
use crate::contact::{ContactManifold, TrackedContact};
use crate::feature_id::FeatureId;

/// Local vertex or face, with stable packed identifiers. `num_vertices` is 0..2.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct PolygonalFeature {
    pub vertices: [Vec2; 2],
    pub vids: [FeatureId; 2],
    pub fid: FeatureId,
    pub num_vertices: u8,
}

pub mod errors {
    pub const MANIFOLD_FULL: felt252 = 'Feature: manifold full';
    pub const VERTEX_COUNT: felt252 = 'Feature: vertex count';
}
fn dot(a: Vec2, b: Vec2) -> Fixed {
    dot2(a.x, b.x, a.y, b.y)
}
fn feature(f: PolygonalFeature, i: u32) -> FeatureId {
    let [a, b] = f.vids;
    if i == 0 {
        a
    } else if i == 2 {
        b
    } else {
        f.fid
    }
}
fn push(
    ref m: ContactManifold,
    p1: Vec2,
    p2: Vec2,
    f1: FeatureId,
    f2: FeatureId,
    dist: Fixed,
    flipped: bool,
) {
    assert(m.num_points < 2, errors::MANIFOLD_FULL);
    let c = if flipped {
        TrackedContact {
            local_p1: p2, local_p2: p1, fid1: f2, fid2: f1, dist, ..Default::default(),
        }
    } else {
        TrackedContact {
            local_p1: p1, local_p2: p2, fid1: f1, fid2: f2, dist, ..Default::default(),
        }
    };
    let [a, _] = m.points;
    m.points = if m.num_points == 0 {
        [c, Default::default()]
    } else {
        [a, c]
    };
    m.num_points += 1;
}
fn clipped_contact(
    ref m: ContactManifold,
    c: ClippingPoints,
    a: PolygonalFeature,
    b: PolygonalFeature,
    p: Pose2,
    n: Vec2,
    flipped: bool,
) {
    let (p1, p2, f1, f2) = c;
    push(
        ref m,
        p1,
        p.inverse_transform_point(p2),
        feature(a, f1),
        feature(b, f2),
        dot(p2 - p1, n),
        flipped,
    );
}

#[generate_trait]
pub impl PolygonalFeatureImpl of PolygonalFeatureTrait {
    /// Transforms both slots in place, including an unused second slot, as upstream.
    /// Floors pose products; panics if an output coordinate leaves Q32.32.
    fn transform_by(ref self: PolygonalFeature, pose: Pose2) {
        let [a, b] = self.vertices;
        self.vertices = [pose.transform_point(a), pose.transform_point(b)];
    }

    /// Appends contacts in a common frame. `prediction` is deliberately unused:
    /// upstream retains points beyond prediction for the solver to filter later.
    /// Empty features give no contacts; vertex pairs use their signed normal gap.
    /// Same rounding/range as `contacts`; panics if the manifold has insufficient space.
    fn clip(
        self: PolygonalFeature,
        other: PolygonalFeature,
        normal: Vec2,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) {
        let _ = prediction;
        Self::contacts(IDENTITY, IDENTITY, normal, -normal, self, other, ref manifold, false);
    }

    /// Appends contacts for local features using mutually inverse poses and their
    /// local separating normals; `flipped` swaps point frames and feature ids.
    /// First clipping point stays first. Floors products/ratios; bounds and panics
    /// of `clip_segment_segment_with_normal` apply. Count >2 or capacity overflow panics.
    /// Parallel face/vertex projection (zero denominator) produces no contact.
    fn contacts(
        pos12: Pose2,
        pos21: Pose2,
        sep_axis1: Vec2,
        sep_axis2: Vec2,
        feature1: PolygonalFeature,
        feature2: PolygonalFeature,
        ref manifold: ContactManifold,
        flipped: bool,
    ) {
        assert(feature1.num_vertices <= 2 && feature2.num_vertices <= 2, errors::VERTEX_COUNT);
        if feature1.num_vertices == 0 || feature2.num_vertices == 0 {
            return;
        }
        if feature1.num_vertices == 2 {
            if feature2.num_vertices == 2 {
                face_face(pos12, feature1, sep_axis1, feature2, ref manifold, flipped);
            } else {
                face_vertex(pos12, feature1, sep_axis1, feature2, ref manifold, flipped);
            }
        } else if feature2.num_vertices == 2 {
            face_vertex(pos21, feature2, sep_axis2, feature1, ref manifold, !flipped);
        } else {
            // Upstream leaves this case unimplemented. Deterministic point-point
            // contact is useful for degenerate features and requires no division.
            let [a, _] = feature1.vertices;
            let [b, _] = feature2.vertices;
            let [fa, _] = feature1.vids;
            let [fb, _] = feature2.vids;
            push(ref manifold, a, b, fa, fb, dot(pos12.transform_point(b) - a, sep_axis1), flipped);
        }
    }
}

fn face_face(
    p: Pose2,
    a: PolygonalFeature,
    n: Vec2,
    b: PolygonalFeature,
    ref m: ContactManifold,
    flipped: bool,
) {
    let [a0, a1] = a.vertices;
    let [b0, b1] = b.vertices;
    if let Some((ca, cb)) =
        clip_segment_segment_with_normal(
            (a0, a1), (p.transform_point(b0), p.transform_point(b1)), n,
        ) {
        clipped_contact(ref m, ca, a, b, p, n, flipped);
        clipped_contact(ref m, cb, a, b, p, n, flipped);
    }
}
fn face_vertex(
    p: Pose2,
    a: PolygonalFeature,
    sep: Vec2,
    b: PolygonalFeature,
    ref m: ContactManifold,
    flipped: bool,
) {
    let [a0, a1] = a.vertices;
    let [b0, _] = b.vertices;
    let [fid, _] = b.vids;
    let v = p.transform_point(b0);
    let t = a1 - a0;
    let n = Vec2 { x: -t.y, y: t.x };
    let denom = -dot(n, sep);
    if denom == ZERO {
        return;
    }
    let dist = dot(a0 - v, n) / denom;
    // Preserve upstream's unnormalized normal in the point reconstruction.
    let p1 = v - Vec2 { x: dist * n.x, y: dist * n.y };
    push(ref m, p1, p.inverse_transform_point(v), a.fid, fid, dist, flipped);
}

#[cfg(test)]
mod tests {
    use fixed::{ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{IDENTITY, Pose2};
    use rapier_testing::opaque;
    use crate::contact::{ContactManifold, ContactManifoldTrait};
    use crate::feature_id::FeatureIdTrait;
    use super::{PolygonalFeature, PolygonalFeatureTrait};
    const O: Vec2 = Vec2 { x: ZERO, y: ZERO };
    const B: Vec2 = Vec2 { x: TWO, y: ZERO };
    const Y: Vec2 = Vec2 { x: ZERO, y: ONE };
    fn face() -> PolygonalFeature {
        PolygonalFeature {
            vertices: [O, B],
            vids: [FeatureIdTrait::vertex(0), FeatureIdTrait::vertex(2)],
            fid: FeatureIdTrait::face(1),
            num_vertices: 2,
        }
    }
    #[test]
    fn test_faces_order_ids_prediction_and_transform() {
        let a = face();
        let mut b = a;
        b.transform_by(Pose2 { translation: Y, ..IDENTITY });
        let mut m: ContactManifold = Default::default();
        a.clip(b, Y, ZERO, ref m);
        assert_eq!(m.num_points, 2);
        assert_eq!(m.point(0).local_p1, B);
        assert_eq!(m.point(1).local_p1, O);
        assert_eq!(m.point(0).dist, ONE);
        assert_eq!(m.point(0).fid1, FeatureIdTrait::vertex(2));
        assert_eq!(m.point(0).fid2, a.fid);
    }
    #[test]
    fn test_contacts_frames_flip_and_face_vertex() {
        let a = face();
        let b = PolygonalFeature { vertices: [Y, O], num_vertices: 1, ..a };
        let mut m: ContactManifold = Default::default();
        a.clip(b, Y, ZERO, ref m);
        assert_eq!(m.num_points, 1);
        assert_eq!(m.point(0).dist, ONE);
        // Upstream uses the length-2 face normal to reconstruct p1.
        assert_eq!(m.point(0).local_p1, -Y);
        let mut flipped: ContactManifold = Default::default();
        PolygonalFeatureTrait::contacts(IDENTITY, IDENTITY, Y, -Y, a, b, ref flipped, true);
        assert_eq!(flipped.point(0).local_p1, m.point(0).local_p2);
        assert_eq!(flipped.point(0).fid2, m.point(0).fid1);
        let mut reversed: ContactManifold = Default::default();
        b.clip(a, -Y, ZERO, ref reversed);
        assert_eq!(reversed.points, flipped.points);
        let p = Pose2 { translation: Y, ..IDENTITY };
        let q = Pose2 { translation: -Y, ..IDENTITY };
        let mut posed: ContactManifold = Default::default();
        PolygonalFeatureTrait::contacts(p, q, Y, -Y, a, a, ref posed, false);
        assert_eq!(posed.point(0).local_p2, B);
        assert_eq!(posed.point(0).dist, ONE);
    }
    #[test]
    fn test_empty_parallel_and_vertex_pairs() {
        let a = face();
        let mut m: ContactManifold = Default::default();
        Default::<PolygonalFeature>::default().clip(a, Y, ZERO, ref m);
        assert_eq!(m.num_points, 0);
        let b = PolygonalFeature { num_vertices: 1, ..a };
        a.clip(b, Vec2 { x: ONE, y: ZERO }, ZERO, ref m);
        assert_eq!(m.num_points, 0);
        b.clip(b, Y, ZERO, ref m);
        assert_eq!(m.num_points, 1);
        assert_eq!(m.point(0).dist, ZERO);
    }
    #[test]
    #[should_panic(expected: 'Feature: manifold full')]
    fn test_capacity_panics() {
        let mut m: ContactManifold = Default::default();
        m.num_points = 2;
        face().clip(face(), Y, ZERO, ref m);
    }
    #[test]
    #[should_panic(expected: 'Feature: vertex count')]
    fn test_invalid_count_panics() {
        let a = PolygonalFeature { num_vertices: 3, ..face() };
        let mut m: ContactManifold = Default::default();
        a.clip(face(), Y, ZERO, ref m);
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque(face());
    }
    #[test]
    fn gas_transform_by() {
        let mut f = opaque(face());
        f.transform_by(IDENTITY);
    }
    #[test]
    fn gas_clip() {
        let mut m: ContactManifold = Default::default();
        opaque(face()).clip(face(), Y, ZERO, ref m);
    }
    #[test]
    fn gas_contacts() {
        let mut m: ContactManifold = Default::default();
        PolygonalFeatureTrait::contacts(
            IDENTITY, IDENTITY, Y, -Y, opaque(face()), face(), ref m, false,
        );
    }
}
