//! The contact dispatcher of the step: `rapier_dynamics2d::ContactDispatcher` over
//! `rapier_geometry2d::dispatch::contact_manifold_step` (upstream: the `DefaultQueryDispatcher` a
//! `PhysicsPipeline` hands to its narrow phase).
//!
//! [`DefaultDispatcher`] is an impl, not a type: `ContactDispatcher` has no `Self`, so the
//! dispatcher is zero-size by construction and selected statically
//! (`compute_contacts::<DefaultDispatcher>`).
//!
//! # Cost model
//!
//! The dispatch table lives in `rapier_geometry2d::dispatch`, in two formulations of the same
//! arms: the metered `contact_manifold` (one-iteration loop around each generator, for loop-free
//! outlined callers) and `contact_manifold_step` (plain arms, the persistence fast path of the
//! cuboid pairs hoisted in front of their generators). The narrow phase's pair loop
//! (`rapier_dynamics2d::narrow_phase::compute_contacts_from_scratch`) inlines its dispatcher into
//! a loop body, where each arm is charged only when taken: the metering loops only cost frames
//! there, and resting cuboid pairs skip their generator's max-path charge.
//!
//! Narrow-phase stage of one P3 step (`pipeline::narrow_benches`, Sierra gas | Cairo steps, net,
//! measured after CL) on `cuboid_stack(3)` / `balls_on_halfspace(8)` / `mixed_pile`:
//! * shipped, `compute_contacts_from_scratch::<DefaultDispatcher>`: 1 640 685 | 11 680 /
//!   2 114 000 | 12 357 / 4 577 705 | 32 156;
//! * same loop with `pipeline::narrow_alternatives::PersistentDispatcher` (this fast path in
//!   front of the metered table): 1 724 535 | 12 437 / 2 349 140 | 14 682 / 4 955 615 | 35 821;
//! * same loop with the metered table (`pipeline::alternatives::InlineDispatcher`, the
//!   `DefaultDispatcher` before CL): 2 466 285 | 13 067 / 2 252 120 | 14 159 / 5 469 945 | 36 103;
//! * `narrow_alternatives::compute_contacts_inlined` (ON's first step) with the metered table:
//!   2 611 915 | 14 644 / 2 479 950 | 16 630 / 5 891 075 | 40 675;
//! * before ON (`narrow_alternatives::compute_contacts_outlined`): 2 844 745 | 16 872 /
//!   2 835 410 | 20 022 / 6 605 945 | 47 522.
//!
//! One dispatch in a one-iteration loop body (gross Sierra gas of the `gas_*` probes below,
//! ball–ball / cuboid–cuboid / half-space–cuboid): `DefaultDispatcher` 129 640 / 535 420 /
//! 304 060, the metered table 157 770 / 563 910 / 333 270. Behind a loop-free call the metered
//! table costs 121 880 / 528 020 / 297 480, and `contact_manifold_step` its costliest arm
//! (`rapier_geometry2d::dispatch::tests::gas_step_outlined_ball_ball`).

use fixed::Fixed;
use rapier_dynamics2d::narrow_phase::ContactDispatcher;
use rapier_geometry2d::contact::ContactManifold;
use rapier_geometry2d::dispatch::contact_manifold_step;
use rapier_geometry2d::shape::Shape;
use rapier_math::pose2::Pose2;

/// The default dispatcher: every convex pair of the closed `Shape` enum, delegated to
/// `rapier_geometry2d::dispatch::contact_manifold_step` (unsupported pairs clear the manifold and
/// return `false`). Bit-identical to `rapier_geometry2d::dispatch::contact_manifold`.
///
/// Only for callers that inline it into a loop body (the narrow phase's pair loop): a loop-free
/// outlined caller pays the costliest arm for every pair (use the metered
/// `rapier_geometry2d::dispatch::contact_manifold` there).
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
        contact_manifold_step(pos12, shape1, shape2, prediction, ref manifold)
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

    /// `DefaultDispatcher` in a one-iteration loop body, its context in the pair loop (outlined
    /// and loop-free, it would be charged its costliest arm on every call).
    #[inline(never)]
    fn run_default(pos12: Pose2, shape1: Shape, shape2: Shape, ref m: ContactManifold) -> bool {
        let mut supported = false;
        let mut pending = true;
        while pending {
            supported =
                DefaultDispatcher::contact_manifold(
                    pos12, shape1, shape2, Fixed { raw: PREDICTION }, ref m,
                );
            pending = false;
        }
        supported
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

    /// One dispatch inlined into a one-iteration loop body, as in the narrow phase's pair loop
    /// (where each arm is charged only when taken): `DefaultDispatcher`.
    #[inline(never)]
    fn probe(case: ManifoldCase) -> bool {
        let mut m: ContactManifold = Default::default();
        let (pos12, s1, s2) = (opaque(pose(case.pos12)), shape(case.shape1), shape(case.shape2));
        let prediction = opaque(Fixed { raw: PREDICTION });
        let mut supported = false;
        let mut pending = true;
        while pending {
            supported = DefaultDispatcher::contact_manifold(pos12, s1, s2, prediction, ref m);
            pending = false;
        }
        supported
    }

    /// The same with the metered table (`InlineDispatcher`, the `DefaultDispatcher` before CL).
    #[inline(never)]
    fn probe_inline_loop(case: ManifoldCase) -> bool {
        let mut m: ContactManifold = Default::default();
        let (pos12, s1, s2) = (opaque(pose(case.pos12)), shape(case.shape1), shape(case.shape2));
        let prediction = opaque(Fixed { raw: PREDICTION });
        let mut supported = false;
        let mut pending = true;
        while pending {
            supported = InlineDispatcher::contact_manifold(pos12, s1, s2, prediction, ref m);
            pending = false;
        }
        supported
    }

    /// The metered table behind a loop-free call (as in an outlined `update_manifold`): the
    /// context it is built for, where it is charged the reached generator only.
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

    #[test]
    fn gas_inline_dispatcher_loop_ball_ball() {
        assert!(probe_inline_loop(contact_manifolds::BALL_BALL_SHALLOW));
    }

    #[test]
    fn gas_inline_dispatcher_loop_cuboid_cuboid() {
        assert!(probe_inline_loop(contact_manifolds::CUBOID_CUBOID_SHALLOW));
    }

    #[test]
    fn gas_inline_dispatcher_loop_halfspace_cuboid() {
        assert!(probe_inline_loop(contact_manifolds::HALFSPACE_CUBOID_SHALLOW));
    }
}
