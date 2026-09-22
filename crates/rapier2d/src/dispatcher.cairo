//! The contact dispatcher of the step: `rapier_dynamics2d::ContactDispatcher` over
//! `rapier_geometry2d`'s contact generators (upstream: the `DefaultQueryDispatcher` a
//! `PhysicsPipeline` hands to its narrow phase).
//!
//! [`DefaultDispatcher`] is an impl, not a type: `ContactDispatcher` has no `Self`, so the
//! dispatcher is zero-size by construction and selected statically
//! (`compute_contacts::<DefaultDispatcher>`).
//!
//! # Cost model and candidates
//!
//! Sierra gas charges a loop-free function its most expensive path. DD's narrow phase calls the
//! dispatcher from `update_manifold`, an outlined loop-free function, inside its pair loop: with
//! `rapier_geometry2d::dispatch::contact_manifold` there (inlined or not), **every pair pays the
//! cuboid–cuboid generator** — measured through `compute_contacts`, 4 ball–ball pairs cost
//! exactly as much as 4 cuboid–cuboid pairs (3 362 856 gas, 841k per pair). A loop, on the other
//! hand, withdraws its gas when an iteration starts: [`DefaultDispatcher`] is GG's `match`, arm
//! for arm and in the same order, with each generator call wrapped in a one-iteration `while`
//! loop, so the caller's static cost holds no generator and each pair pays its own.
//!
//! Narrow-phase stage, second step, Sierra gas | Cairo steps (`crate::pipeline::benches`):
//!
//! | candidate | 4 ball–ball | 4 cuboid–cuboid | ball on half-space | `BOX_STACK3` |
//! |---|---|---|---|---|
//! | **`DefaultDispatcher`** | **1 486 460** \| 12 066 | **3 197 940** \| 14 894 | **509 505** \| 3
//! 783 | **2 422 715** \| 11 403 |
//! | GG's `dispatch::contact_manifold` inlined | 3 362 856 \| 11 014 | 3 362 856 \| 13 822 | 916
//! 344 \| 3 500 | 2 546 002 \| 10 596 |
//! | the same `#[inline(never)]` | 3 394 056 \| 11 578 | 3 394 056 \| 14 386 | — | — |
//!
//! One dispatch behind a call (this module's probes, net of `gas_baseline`): ball–ball 107 560
//! vs 553 100, half-space–cuboid 283 160 vs 553 100, cuboid–cuboid 508 060 vs 553 100.
//!
//! The price is ~265 Cairo steps per pair (+8 to +10 % of the narrow phase) for the loop frames;
//! the same arms written `loop { …; break; }` measured identical to GG's match (no gain). The
//! pipeline-level candidates (per-kind `match` in the pair loop, per-kind bucket loops, per-kind
//! one-iteration loops around `process_pair`) are in `crate::pipeline::alternatives`, all
//! beaten.

use fixed::Fixed;
use rapier_dynamics2d::narrow_phase::ContactDispatcher;
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldTrait};
use rapier_geometry2d::contact_generators::ball_ball::contact_manifold_ball_ball;
use rapier_geometry2d::contact_generators::capsule_capsule::contact_manifold_capsule_capsule;
use rapier_geometry2d::contact_generators::convex_ball::{
    contact_manifold_ball_convex, contact_manifold_convex_ball,
};
use rapier_geometry2d::contact_generators::cuboid_capsule::{
    contact_manifold_cuboid_capsule, contact_manifold_cuboid_capsule_shapes,
};
use rapier_geometry2d::contact_generators::cuboid_cuboid::contact_manifold_cuboid_cuboid;
use rapier_geometry2d::contact_generators::cuboid_segment::{
    contact_manifold_cuboid_segment, contact_manifold_cuboid_segment_shapes,
};
use rapier_geometry2d::contact_generators::halfspace_pfm::contact_manifold_halfspace_pfm;
use rapier_geometry2d::shape::Shape;
use rapier_math::pose2::{Pose2, Pose2Trait};

/// The default dispatcher: every convex pair of the closed `Shape` enum, with exactly the
/// generators, arm order and results of `rapier_geometry2d::dispatch::contact_manifold`
/// (unsupported pairs clear the manifold and return `false`); see the module documentation for
/// why the match is repeated here.
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
        match (shape1, shape2) {
            (
                Shape::Ball(ball1), Shape::Ball(ball2),
            ) => {
                let mut pending = true;
                while pending {
                    contact_manifold_ball_ball(pos12, ball1, ball2, prediction, ref manifold);
                    pending = false;
                }
                true
            },
            (
                Shape::Cuboid(cuboid1), Shape::Cuboid(cuboid2),
            ) => {
                let mut pending = true;
                while pending {
                    contact_manifold_cuboid_cuboid(
                        pos12, cuboid1, cuboid2, prediction, ref manifold,
                    );
                    pending = false;
                }
                true
            },
            (
                Shape::Capsule(capsule1), Shape::Capsule(capsule2),
            ) => {
                let mut pending = true;
                while pending {
                    contact_manifold_capsule_capsule(
                        pos12, capsule1, capsule2, prediction, ref manifold,
                    );
                    pending = false;
                }
                true
            },
            (
                Shape::Ball(ball1), _,
            ) => {
                let mut pending = true;
                while pending {
                    contact_manifold_ball_convex(pos12, ball1, shape2, prediction, ref manifold);
                    pending = false;
                }
                true
            },
            (
                _, Shape::Ball(ball2),
            ) => {
                let mut pending = true;
                while pending {
                    contact_manifold_convex_ball(pos12, shape1, ball2, prediction, ref manifold);
                    pending = false;
                }
                true
            },
            (
                Shape::Cuboid(cuboid1), Shape::Capsule(capsule2),
            ) => {
                let mut pending = true;
                while pending {
                    contact_manifold_cuboid_capsule(
                        pos12, cuboid1, capsule2, prediction, ref manifold,
                    );
                    pending = false;
                }
                true
            },
            (
                Shape::Capsule(_), Shape::Cuboid(_),
            ) => {
                let mut supported = false;
                let mut pending = true;
                while pending {
                    supported =
                        contact_manifold_cuboid_capsule_shapes(
                            pos12, shape1, shape2, prediction, ref manifold,
                        );
                    pending = false;
                }
                supported
            },
            (
                Shape::Cuboid(cuboid1), Shape::Segment(segment2),
            ) => {
                let mut pending = true;
                while pending {
                    contact_manifold_cuboid_segment(
                        pos12, cuboid1, segment2, prediction, ref manifold,
                    );
                    pending = false;
                }
                true
            },
            (
                Shape::Segment(_), Shape::Cuboid(_),
            ) => {
                let mut supported = false;
                let mut pending = true;
                while pending {
                    supported =
                        contact_manifold_cuboid_segment_shapes(
                            pos12, shape1, shape2, prediction, ref manifold,
                        );
                    pending = false;
                }
                supported
            },
            (Shape::HalfSpace(halfspace1), Shape::Cuboid(_)) |
            (Shape::HalfSpace(halfspace1), Shape::Segment(_)) |
            (
                Shape::HalfSpace(halfspace1), Shape::Capsule(_),
            ) => {
                let mut pending = true;
                while pending {
                    contact_manifold_halfspace_pfm(
                        pos12, halfspace1, shape2, prediction, ref manifold, false,
                    );
                    pending = false;
                }
                true
            },
            (Shape::Cuboid(_), Shape::HalfSpace(halfspace2)) |
            (Shape::Segment(_), Shape::HalfSpace(halfspace2)) |
            (
                Shape::Capsule(_), Shape::HalfSpace(halfspace2),
            ) => {
                let mut pending = true;
                while pending {
                    contact_manifold_halfspace_pfm(
                        pos12.inverse(), halfspace2, shape1, prediction, ref manifold, true,
                    );
                    pending = false;
                }
                true
            },
            _ => {
                manifold.clear();
                false
            },
        }
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

    /// The same with the rejected `InlineDispatcher`.
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
