//! The contact dispatcher of the step: `rapier_dynamics2d::ContactDispatcher` over
//! `rapier_geometry2d::dispatch::contact_manifold` (upstream: the `DefaultQueryDispatcher` a
//! `PhysicsPipeline` hands to its narrow phase).
//!
//! [`DefaultDispatcher`] is an impl, not a type: `ContactDispatcher` has no `Self`, so the
//! dispatcher is zero-size by construction and selected statically
//! (`compute_contacts::<DefaultDispatcher>`).
//!
//! # Cost model
//!
//! The metered typed match lives in `rapier_geometry2d::dispatch`. The one-iteration loops around
//! each generator make an outlined narrow-phase caller pay only the reached generator while keeping
//! the dispatcher as the single source of truth. P1 measured the accepted real-pipeline target at
//! 1 486 460 Sierra gas for four warm ball-ball pairs and 3 197 940 for four warm cuboid-cuboid
//! pairs, with about 265 Cairo steps per pair for the loop frames.

use fixed::Fixed;
use rapier_dynamics2d::narrow_phase::ContactDispatcher;
use rapier_geometry2d::contact::ContactManifold;
use rapier_geometry2d::dispatch::contact_manifold as dispatch_contact_manifold;
use rapier_geometry2d::shape::Shape;
use rapier_math::pose2::Pose2;

/// The default dispatcher: every convex pair of the closed `Shape` enum, delegated to
/// `rapier_geometry2d::dispatch::contact_manifold` (unsupported pairs clear the manifold and return
/// `false`).
///
/// # Panics
/// As `rapier_geometry2d::dispatch::contact_manifold`.
pub impl DefaultDispatcher of ContactDispatcher {
    #[inline(always)]
    fn contact_manifold(
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) -> bool {
        dispatch_contact_manifold(pos12, shape1, shape2, prediction, ref manifold)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, HALF, ONE, ZERO};
    use glam::Vec2;
    use rapier_geometry2d::contact::ContactManifold;
    use rapier_geometry2d::dispatch::contact_manifold;
    use rapier_geometry2d::shape::{
        BallTrait, CapsuleTrait, CuboidTrait, HalfSpaceTrait, Segment, SegmentTrait, Shape,
    };
    use rapier_golden::contact_manifolds::{self, PREDICTION};
    use rapier_golden::types::ManifoldCase;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::pipeline::alternatives::InlineDispatcher;
    use crate::pipeline::fixtures::{pose, shape};
    use super::DefaultDispatcher;

    #[inline(never)]
    fn run_default(pos12: Pose2, shape1: Shape, shape2: Shape, ref m: ContactManifold) -> bool {
        DefaultDispatcher::contact_manifold(pos12, shape1, shape2, Fixed { raw: PREDICTION }, ref m)
    }

    #[inline(never)]
    fn run_dispatch(pos12: Pose2, shape1: Shape, shape2: Shape, ref m: ContactManifold) -> bool {
        contact_manifold(pos12, shape1, shape2, Fixed { raw: PREDICTION }, ref m)
    }

    /// Both dispatchers on the same inputs, twice (the second call starts from the first
    /// manifold: warm-start fast paths). Returns `true` when every output is identical.
    fn same(pos12: Pose2, shape1: Shape, shape2: Shape) -> bool {
        let mut a: ContactManifold = Default::default();
        let mut b: ContactManifold = Default::default();
        let first = run_default(
            pos12, shape1, shape2, ref a,
        ) == run_dispatch(pos12, shape1, shape2, ref b)
            && a == b;
        let second = run_default(
            pos12, shape1, shape2, ref a,
        ) == run_dispatch(pos12, shape1, shape2, ref b)
            && a == b;
        first && second
    }

    /// One shape of each variant, sized to touch the others at the poses of [`poses`].
    fn shapes() -> Array<Shape> {
        let half = Vec2 { x: HALF, y: HALF };
        array![
            Shape::Ball(BallTrait::new(HALF)), Shape::Cuboid(CuboidTrait::new(half)),
            Shape::Capsule(
                CapsuleTrait::new(Vec2 { x: -HALF, y: ZERO }, Vec2 { x: HALF, y: ZERO }, HALF),
            ),
            Shape::HalfSpace(HalfSpaceTrait::new(Vec2 { x: ZERO, y: ONE })),
            Shape::Segment(SegmentTrait::new(Vec2 { x: -ONE, y: ZERO }, Vec2 { x: ONE, y: ZERO })),
        ]
    }

    /// Separated, touching-within-prediction, shallow-rotated and deep poses of shape 2.
    fn poses() -> Array<Pose2> {
        let turn = Rot2 { re: Fixed { raw: 3719550787 }, im: HALF }; // 30°
        array![
            Pose2Trait::new(
                Vec2 { x: ZERO, y: Fixed { raw: 12884901888 } }, Rot2 { re: ONE, im: ZERO },
            ),
            Pose2Trait::new(
                Vec2 { x: ZERO, y: ONE + Fixed { raw: 4294967 } }, Rot2 { re: ONE, im: ZERO },
            ),
            Pose2Trait::new(
                Vec2 { x: Fixed { raw: 858993459 }, y: Fixed { raw: 3865470566 } }, turn,
            ),
            Pose2Trait::new(Vec2 { x: Fixed { raw: 429496729 }, y: HALF }, turn),
        ]
    }

    /// Every ordered pair of shape variants at every pose: `DefaultDispatcher` is bit-identical
    /// to `rapier_geometry2d::dispatch::contact_manifold` (supported and unsupported arms).
    #[test]
    fn test_matches_dispatch_on_every_arm() {
        let shapes = shapes();
        let mut supported: u32 = 0;
        for pos12 in poses().span() {
            for s1 in shapes.span() {
                for s2 in shapes.span() {
                    assert!(same(*pos12, *s1, *s2));
                    let mut m: ContactManifold = Default::default();
                    if run_default(*pos12, *s1, *s2, ref m) {
                        supported += 1;
                    }
                }
            }
        }
        // 25 ordered pairs, 4 without generator (segment–segment, segment–capsule both ways,
        // half-space–half-space), at 4 poses.
        assert_eq!(supported, 4 * 21);
    }

    /// The pairs without generator: `false` and a cleared manifold, as `dispatch`.
    #[test]
    fn test_unsupported_pairs_match_dispatch() {
        let segment = Shape::Segment(
            Segment { a: Vec2 { x: -ONE, y: ZERO }, b: Vec2 { x: ONE, y: ZERO } },
        );
        let pos12 = pose(contact_manifolds::HALFSPACE_CUBOID_SHALLOW.pos12);
        let halfspace = shape(contact_manifolds::HALFSPACE_CUBOID_SHALLOW.shape1);
        let capsule = shape(contact_manifolds::CAPSULE_CAPSULE_SHALLOW.shape1);
        for (s1, s2) in array![
            (segment, segment), (segment, capsule), (capsule, segment), (halfspace, halfspace),
        ]
            .span() {
            assert!(same(pos12, *s1, *s2));
            let mut m = contact_manifolds_live();
            assert!(!DefaultDispatcher::contact_manifold(pos12, *s1, *s2, ONE, ref m));
            assert_eq!(m.num_points, 0);
        }
    }

    fn contact_manifolds_live() -> ContactManifold {
        let case = contact_manifolds::BALL_BALL_SHALLOW;
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold(
            pose(case.pos12), shape(case.shape1), shape(case.shape2), ONE, ref m,
        );
        assert!(m.num_points != 0);
        m
    }

    /// One dispatch behind a call (as in `update_manifold`): `DefaultDispatcher`.
    #[inline(never)]
    fn probe(case: ManifoldCase) -> bool {
        let mut m: ContactManifold = Default::default();
        let (pos12, s1, s2) = (opaque(pose(case.pos12)), shape(case.shape1), shape(case.shape2));
        DefaultDispatcher::contact_manifold(pos12, s1, s2, opaque(Fixed { raw: PREDICTION }), ref m)
    }

    /// The same through the pipeline candidate that now reaches the shared metered dispatcher.
    #[inline(never)]
    fn probe_inline(case: ManifoldCase) -> bool {
        let mut m: ContactManifold = Default::default();
        let (pos12, s1, s2) = (opaque(pose(case.pos12)), shape(case.shape1), shape(case.shape2));
        InlineDispatcher::contact_manifold(pos12, s1, s2, opaque(Fixed { raw: PREDICTION }), ref m)
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(1_u32);
    }

    #[test]
    fn gas_default_dispatcher_ball_ball() {
        assert!(probe(contact_manifolds::BALL_BALL_SHALLOW));
    }

    #[test]
    fn gas_default_dispatcher_cuboid_cuboid() {
        assert!(probe(contact_manifolds::CUBOID_CUBOID_SHALLOW));
    }

    #[test]
    fn gas_default_dispatcher_halfspace_cuboid() {
        assert!(probe(contact_manifolds::HALFSPACE_CUBOID_SHALLOW));
    }

    #[test]
    fn gas_inline_dispatcher_ball_ball() {
        assert!(probe_inline(contact_manifolds::BALL_BALL_SHALLOW));
    }

    #[test]
    fn gas_inline_dispatcher_cuboid_cuboid() {
        assert!(probe_inline(contact_manifolds::CUBOID_CUBOID_SHALLOW));
    }

    #[test]
    fn gas_inline_dispatcher_halfspace_cuboid() {
        assert!(probe_inline(contact_manifolds::HALFSPACE_CUBOID_SHALLOW));
    }
}
