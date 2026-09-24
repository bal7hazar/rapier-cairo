//! The contact dispatcher of the step: the table of `rapier_geometry2d::dispatch::contact_manifold`
//! as a plain typed `match`, with the persistence fast path of the generators hoisted out of
//! them (work package ON).
//!
//! `rapier_geometry2d::dispatch` puts each generator behind a one-iteration `while` (GM), so
//! that a loop-free outlined caller is charged only the reached generator. The step's pair loop
//! (`rapier_dynamics2d::narrow_phase::compute_contacts_from_scratch`) inlines its dispatcher in
//! a loop body, where each arm is charged only when taken anyway: the loops only cost their
//! frames (about 265 Cairo steps per pair), so [`StepDispatcher`] drops them.
//!
//! The cuboid–cuboid, cuboid–capsule and cuboid–segment generators (both orders) start with
//! `try_update_contacts(pos12)` on the same `pos12` and return when it succeeds. Sierra gas
//! charges an outlined generator its costliest path (SAT and clipping) even when that fast path
//! returns, so [`StepDispatcher`] runs the check itself, in the loop body, and calls the
//! generator only when it fails (the generator repeats the check; `try_update_contacts` leaves
//! the manifold untouched when it fails). Same results on every pair: the only difference is
//! that a capsule–cuboid or segment–cuboid pair whose fast path succeeds skips the generator's
//! `pos12.inverse()`, whose only effect is a panic on an unrepresentable pose.
//!
//! # Cost
//!
//! Narrow-phase stage of one P3 step (`narrow_benches`, Sierra gas | Cairo steps, net) on
//! `cuboid_stack(3)` / `balls_on_halfspace(8)` / `mixed_pile`:
//! * shipped, `compute_contacts_from_scratch::<StepDispatcher>`: 1 642 485 | 11 680 /
//!   2 116 880 | 12 357 / 4 583 105 | 32 156;
//! * same loop with `narrow_alternatives::PersistentDispatcher` (this fast path in front of the
//!   metered `DefaultDispatcher`): 1 726 335 | 12 437 / 2 352 020 | 14 682 / 4 961 015 | 35 821;
//! * same loop with `DefaultDispatcher`: 2 468 085 | 13 067 / 2 255 000 | 14 159 /
//!   5 475 345 | 36 103;
//! * `narrow_alternatives::compute_contacts_inlined` (first step: composition inlined, carry-over
//!   and pair copies unchanged) with `DefaultDispatcher`: 2 613 715 | 14 644 / 2 482 830 |
//!   16 630 / 5 896 475 | 40 675;
//! * before ON (`narrow_alternatives::compute_contacts_outlined`): 2 883 645 | 16 952 /
//!   2 897 650 | 20 150 / 6 722 645 | 47 762.

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
use rapier_geometry2d::manifold::ManifoldTrait;
use rapier_geometry2d::shape::Shape;
use rapier_math::pose2::{Pose2, Pose2Trait};

/// `rapier_geometry2d::dispatch::contact_manifold` as a plain typed `match` (same arms, same
/// order, same generators) with the persistence fast path in front of the cuboid pairs (see the
/// module documentation). Only for callers that inline it in a loop body: in a loop-free
/// outlined caller every pair would be charged the costliest arm.
///
/// # Panics
/// As `rapier_geometry2d::dispatch::contact_manifold`.
pub impl StepDispatcher of ContactDispatcher {
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
                contact_manifold_ball_ball(pos12, ball1, ball2, prediction, ref manifold);
                true
            },
            (
                Shape::Cuboid(cuboid1), Shape::Cuboid(cuboid2),
            ) => {
                if !manifold.try_update_contacts(pos12) {
                    contact_manifold_cuboid_cuboid(
                        pos12, cuboid1, cuboid2, prediction, ref manifold,
                    );
                }
                true
            },
            (
                Shape::Capsule(capsule1), Shape::Capsule(capsule2),
            ) => {
                contact_manifold_capsule_capsule(
                    pos12, capsule1, capsule2, prediction, ref manifold,
                );
                true
            },
            (
                Shape::Ball(ball1), _,
            ) => {
                contact_manifold_ball_convex(pos12, ball1, shape2, prediction, ref manifold);
                true
            },
            (
                _, Shape::Ball(ball2),
            ) => {
                contact_manifold_convex_ball(pos12, shape1, ball2, prediction, ref manifold);
                true
            },
            (
                Shape::Cuboid(cuboid1), Shape::Capsule(capsule2),
            ) => {
                if !manifold.try_update_contacts(pos12) {
                    contact_manifold_cuboid_capsule(
                        pos12, cuboid1, capsule2, prediction, ref manifold,
                    );
                }
                true
            },
            (
                Shape::Capsule(_), Shape::Cuboid(_),
            ) => {
                if manifold.try_update_contacts(pos12) {
                    return true;
                }
                contact_manifold_cuboid_capsule_shapes(
                    pos12, shape1, shape2, prediction, ref manifold,
                )
            },
            (
                Shape::Cuboid(cuboid1), Shape::Segment(segment2),
            ) => {
                if !manifold.try_update_contacts(pos12) {
                    contact_manifold_cuboid_segment(
                        pos12, cuboid1, segment2, prediction, ref manifold,
                    );
                }
                true
            },
            (
                Shape::Segment(_), Shape::Cuboid(_),
            ) => {
                if manifold.try_update_contacts(pos12) {
                    return true;
                }
                contact_manifold_cuboid_segment_shapes(
                    pos12, shape1, shape2, prediction, ref manifold,
                )
            },
            (Shape::HalfSpace(halfspace1), Shape::Cuboid(_)) |
            (Shape::HalfSpace(halfspace1), Shape::Segment(_)) |
            (
                Shape::HalfSpace(halfspace1), Shape::Capsule(_),
            ) => {
                contact_manifold_halfspace_pfm(
                    pos12, halfspace1, shape2, prediction, ref manifold, false,
                );
                true
            },
            (Shape::Cuboid(_), Shape::HalfSpace(halfspace2)) |
            (Shape::Segment(_), Shape::HalfSpace(halfspace2)) |
            (
                Shape::Capsule(_), Shape::HalfSpace(halfspace2),
            ) => {
                contact_manifold_halfspace_pfm(
                    pos12.inverse(), halfspace2, shape1, prediction, ref manifold, true,
                );
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
    use rapier_dynamics2d::narrow_phase::ContactDispatcher;
    use rapier_geometry2d::contact::ContactManifold;
    use rapier_geometry2d::dispatch::contact_manifold;
    use rapier_geometry2d::shape::{
        BallTrait, CapsuleTrait, CuboidTrait, HalfSpaceTrait, SegmentTrait, Shape,
    };
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use super::StepDispatcher;

    const PREDICTION: Fixed = Fixed { raw: 8589934 }; // 0.002

    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: Fixed { raw: x }, y: Fixed { raw: y } }
    }

    /// One shape of each variant, sized to touch the others at the poses of `poses`.
    fn shapes() -> Array<Shape> {
        array![
            Shape::Ball(BallTrait::new(HALF)),
            Shape::Cuboid(CuboidTrait::new(Vec2 { x: HALF, y: HALF })),
            Shape::Capsule(
                CapsuleTrait::new(Vec2 { x: -HALF, y: ZERO }, Vec2 { x: HALF, y: ZERO }, HALF),
            ),
            Shape::HalfSpace(HalfSpaceTrait::new(Vec2 { x: ZERO, y: ONE })),
            Shape::Segment(SegmentTrait::new(Vec2 { x: -ONE, y: ZERO }, Vec2 { x: ONE, y: ZERO })),
        ]
    }

    /// Separated, resting (within prediction), shallow-rotated and deep poses of shape 2.
    fn poses() -> Array<Pose2> {
        let level = Rot2 { re: ONE, im: ZERO };
        let turn = Rot2 { re: Fixed { raw: 3719550787 }, im: HALF }; // 30°
        array![
            Pose2Trait::new(v(0, 12884901888), level), Pose2Trait::new(v(0, 4294967296), level),
            Pose2Trait::new(v(858993459, 3865470566), turn),
            Pose2Trait::new(v(429496729, HALF.raw), turn),
        ]
    }

    /// `pos12` moved by `(dx, dy)` raw.
    fn moved(pos12: Pose2, dx: i64, dy: i64) -> Pose2 {
        Pose2Trait::new(pos12.translation + v(dx, dy), pos12.rotation)
    }

    /// `StepDispatcher` and `dispatch::contact_manifold` from the same manifold: same result,
    /// same manifold.
    fn same(
        pos12: Pose2, shape1: Shape, shape2: Shape, ref a: ContactManifold, ref b: ContactManifold,
    ) -> bool {
        let got = StepDispatcher::contact_manifold(pos12, shape1, shape2, PREDICTION, ref a);
        let expected = contact_manifold(pos12, shape1, shape2, PREDICTION, ref b);
        got == expected && a == b
    }

    /// Every ordered pair of shape variants at every pose, three calls on the same manifold:
    /// cold, warm at the same pose (persistence fast path taken), and moved beyond the
    /// persistence tolerance (fast path fails, the generator runs): bit-identical to
    /// `rapier_geometry2d::dispatch::contact_manifold` on every call.
    #[test]
    fn test_matches_dispatch_cold_warm_and_moved() {
        let shapes = shapes();
        for pos12 in poses().span() {
            for s1 in shapes.span() {
                for s2 in shapes.span() {
                    let mut a: ContactManifold = Default::default();
                    let mut b: ContactManifold = Default::default();
                    assert!(same(*pos12, *s1, *s2, ref a, ref b));
                    assert!(same(moved(*pos12, 4294, 0), *s1, *s2, ref a, ref b));
                    assert!(same(moved(*pos12, 42949672, -4294967), *s1, *s2, ref a, ref b));
                }
            }
        }
    }
}
