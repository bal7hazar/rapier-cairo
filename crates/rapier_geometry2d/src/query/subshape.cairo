//! Query results with the sub-shape ids upstream stores in their fields (Parry 0.31
//! `Contact::subshape1 / 2`, `PointProjection::subshape`, `RayIntersection::subshape`; lot SW1,
//! `docs/briefs/sw1-subshape-widening.md`).
//!
//! The port keeps every result at its width (ADR 35), so the ids travel in twin values built by
//! the upstream method names: `ContactTrait::with_subshapes`, `PointProjectionTrait::with_subshape`
//! and `RayIntersectionTrait::with_subshape`. The `*_part` queries of the composite shapes already
//! answer `(SubShapeId, T)`: call `with_subshape` on the `T`. See ADR 46 for the manifold poses.

use crate::feature_id::SubShapeId;
use crate::point::PointProjection;
use crate::ray::RayIntersection;
use super::Contact;

/// A [`Contact`] with the sub-shapes of its two sides (`0` for a simple shape).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SubshapeContact {
    pub contact: Contact,
    pub subshape1: SubShapeId,
    pub subshape2: SubShapeId,
}

/// A [`PointProjection`] with the sub-shape it landed on (`0` for a simple shape).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SubshapePointProjection {
    pub projection: PointProjection,
    pub subshape: SubShapeId,
}

/// A [`RayIntersection`] with the sub-shape that was hit (`0` for a simple shape).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SubshapeRayIntersection {
    pub intersection: RayIntersection,
    pub subshape: SubShapeId,
}

#[cfg(test)]
mod tests {
    use fixed::{ONE, TWO, ZERO};
    use glam_core::vec2::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::contact::{ContactManifold, ContactManifoldTrait, SubshapePoses};
    use crate::feature_id::FeatureIdTrait;
    use crate::point::{PointProjection, PointProjectionTrait};
    use crate::ray::{RayIntersection, RayIntersectionTrait};
    use crate::shape::{BallTrait, CuboidTrait, Shape, ShapeTrait};
    use super::super::{Contact, ContactTrait};

    fn v(x: fixed::Fixed, y: fixed::Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn pose(x: fixed::Fixed, y: fixed::Fixed) -> Pose2 {
        Pose2Trait::new(v(x, y), Rot2 { re: ONE, im: ZERO })
    }

    fn compound() -> Shape {
        ShapeTrait::compound(
            array![
                (pose(ZERO, ZERO), Shape::Ball(BallTrait::new(ONE))),
                (pose(TWO, ONE), Shape::Cuboid(CuboidTrait::new(v(ONE, ONE)))),
            ]
                .span(),
        )
    }

    #[test]
    fn test_with_subshape_builders_keep_the_result() {
        let c = ContactTrait::new(v(ONE, ZERO), v(TWO, ZERO), v(ONE, ZERO), v(ZERO, ONE), ONE);
        let sc = c.with_subshapes(opaque(3_u32), opaque(5_u32));
        assert_eq!(sc.contact, c);
        assert_eq!((sc.subshape1, sc.subshape2), (3, 5));

        let p = PointProjectionTrait::new(true, v(ONE, TWO));
        let sp = p.with_subshape(opaque(7_u32));
        assert_eq!(sp.projection, p);
        assert_eq!(sp.subshape, 7);

        let r = RayIntersectionTrait::new(ONE, v(ZERO, ONE), FeatureIdTrait::face(1));
        let sr = r.with_subshape(opaque(2_u32));
        assert_eq!(sr.intersection, r);
        assert_eq!(sr.subshape, 2);
    }

    #[test]
    fn test_manifold_subshape_pos_reads_the_part_pose() {
        let c = compound();
        let ball = Shape::Ball(BallTrait::new(ONE));
        let m = ContactManifold { subshape1: 1, subshape2: 0, ..Default::default() };
        assert_eq!(m.subshape_pos1(@c), Some(pose(TWO, ONE)));
        assert_eq!(m.subshape_pos2(@c), Some(pose(ZERO, ZERO)));
        assert_eq!(m.subshape_pos1(@ball), None);
        assert_eq!(m.subshape_pos2(@ball), None);
    }

    #[test]
    #[should_panic(expected: 'Compound: part index')]
    fn test_manifold_subshape_pos_out_of_range_panics() {
        let m = ContactManifold { subshape1: opaque(2_u32), ..Default::default() };
        let _ = m.subshape_pos1(@compound());
    }

    #[test]
    fn test_set_subshape_pos_writes_the_poses() {
        let mut poses: SubshapePoses = Default::default();
        ContactManifoldTrait::set_subshape_pos1(ref poses, Some(pose(ONE, ONE)));
        ContactManifoldTrait::set_subshape_pos2(ref poses, Some(pose(TWO, ZERO)));
        assert_eq!(poses.pos1, Some(pose(ONE, ONE)));
        assert_eq!(poses.pos2, Some(pose(TWO, ZERO)));
        ContactManifoldTrait::set_subshape_pos1(ref poses, None);
        assert_eq!(poses.pos1, None);
        assert_eq!(poses.pos2, Some(pose(TWO, ZERO)));
    }
}
