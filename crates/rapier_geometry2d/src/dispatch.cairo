//! Contact-manifold dispatcher (Parry `DefaultQueryDispatcher::contact_manifolds` restricted to
//! one convex pair, i.e. `contact_manifold_convex_convex`).
//!
//! One `match` on the two [`Shape`] variants, in upstream's priority order, calling the **typed**
//! generator of each pair (the `*_shapes` wrappers of `contact_generators` re-match the enum):
//!
//! 1. ball – ball: `ball_ball::contact_manifold_ball_ball`;
//! 2. cuboid – cuboid: `cuboid_cuboid::contact_manifold_cuboid_cuboid`;
//! 3. capsule – capsule: `capsule_capsule::contact_manifold_capsule_capsule`;
//! 4. ball – anything, anything – ball: `convex_ball::{contact_manifold_ball_convex,
//!    contact_manifold_convex_ball}`;
//! 5. cuboid – capsule and capsule – cuboid: `cuboid_capsule`;
//! 6. cuboid – segment and segment – cuboid: `cuboid_segment`;
//! 7. half-space – cuboid / segment / capsule and the reverse:
//!    `halfspace_pfm::contact_manifold_halfspace_pfm`.
//!
//! Rows 5 and 6 are a Rapier-2D specialisation: upstream sends those pairs to its generic
//! polygonal-feature-map generator (`contact_manifold_pfm_pfm`, GJK/EPA based, not ported) and
//! keeps the analytic cuboid–capsule generator commented out. Every other pair of the closed
//! [`Shape`] enum (segment–segment, segment–capsule, capsule–segment,
//! half-space–half-space) has no generator and is reported unsupported.
//!
//! # Flipped pairs
//!
//! Upstream's `flipped = true` branches (ball first, half-space second, capsule–cuboid,
//! segment–cuboid) swap the shapes, invert `pos12` and swap the two sides of the manifold
//! (`local_n1/2`, `local_p1/2`, `fid1/2`). Each generator of `contact_generators` does that
//! itself, on the manifold it is handed (which also keeps the warm-start fast paths correct), so
//! the dispatcher only chooses the entry point and needs no `ContactManifold::flip`.
//!
//! # Deferred
//!
//! Compound, heightfield and triangle-mesh shapes, `ContactManifoldsWorkspace`, normal
//! constraints, and the generic PFM–PFM generator.
//!
//! # Candidates
//!
//! Ranked by the `gas_*` probes of this module (Sierra gas net of `gas_baseline`, Cairo steps in
//! parentheses; ball–ball / cuboid–segment / unsupported pair). The losers live in
//! `dispatch/alternatives.cairo`.
//!
//! 1. **`#[inline(always)]` typed `match`** (this module): 40 570 (362) / 527 900 (3 019) / 6 720
//!    (66), i.e. 1.2k to 6.9k gas and 12 to 70 steps above the bare generator, whatever the pair.
//! 2. `alternatives::contact_manifold_helpers`: the same with one `#[inline(never)]` helper per
//!    arm: 47 470 / 535 200 / 6 720, about 7k more than the winner for nothing.
//! 3. `alternatives::contact_manifold_shapes_chain`: upstream's shape, the `*_shapes` wrappers
//!    tried in turn (each re-matches the enum): 54 670 / 1 966 870 / 2 198 450.
//! 4. `alternatives::contact_manifold_outlined`: the winner behind a call: 556 300 gas for every
//!    pair (the most expensive arm), 565 / 3 221 / 208 steps.
//!
//! Why the attribute: Sierra gas charges a called function its worst-case path, so an outlined
//! dispatcher costs the most expensive generator on every pair (ball–ball would cost 14 times the
//! generator). Inlined into its caller, the arms are separate paths and the caller pays only the
//! generator it reaches; the generators themselves stay real calls. Capsule–cuboid and
//! segment–cuboid cost 6.8k more than the others because only the `*_shapes` wrapper of their
//! generator reaches the flipped implementation (it re-matches the pair).

use fixed::Fixed;
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::contact::{ContactManifold, ContactManifoldTrait};
use crate::contact_generators::ball_ball::contact_manifold_ball_ball;
use crate::contact_generators::capsule_capsule::contact_manifold_capsule_capsule;
use crate::contact_generators::convex_ball::{
    contact_manifold_ball_convex, contact_manifold_convex_ball,
};
use crate::contact_generators::cuboid_capsule::{
    contact_manifold_cuboid_capsule, contact_manifold_cuboid_capsule_shapes,
};
use crate::contact_generators::cuboid_cuboid::contact_manifold_cuboid_cuboid;
use crate::contact_generators::cuboid_segment::{
    contact_manifold_cuboid_segment, contact_manifold_cuboid_segment_shapes,
};
use crate::contact_generators::halfspace_pfm::contact_manifold_halfspace_pfm;
use crate::shape::Shape;

/// Computes the contact manifold of one convex pair (Parry `contact_manifolds` for two convex
/// shapes).
///
/// `pos12` is the pose of `shape2` in the local frame of `shape1` (`pose1.inv_mul(pose2)`, unit
/// rotation); `manifold` is the previous step's manifold of the pair (or a default one) and is
/// updated in place, its `ContactData` following the feature ids where the generator supports
/// warm starting. Returns `true` when a generator exists for the pair (upstream `Ok`), `false`
/// for an unsupported pair (upstream `Err(Unsupported)`), in which case `manifold` is cleared.
/// #### Panics
/// * The panics of the selected generator (see `crate::contact_generators`), and those of
///   `Pose2::inverse` for a half-space in second position.
/// #### Deviations
/// * Upstream returns `Result<(), Unsupported>` and pushes the manifold into a `Vec`; the `bool`
///   and the `ref` manifold replace both. Compound shapes, heightfields, triangle meshes, the
///   workspace and the normal constraints are deferred.
/// * Cuboid–capsule, capsule–cuboid, cuboid–segment and segment–cuboid use the analytic
///   generators instead of upstream's generic PFM–PFM one; segment–segment and
///   segment–capsule (also PFM–PFM upstream) are unsupported until that generator is ported.
/// * `#[inline(always)]`: the dispatcher must be inlined into its caller, otherwise Sierra gas
///   charges every pair the most expensive arm (see the candidates above).
#[inline(always)]
pub fn contact_manifold(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
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
            contact_manifold_cuboid_cuboid(pos12, cuboid1, cuboid2, prediction, ref manifold);
            true
        },
        (
            Shape::Capsule(capsule1), Shape::Capsule(capsule2),
        ) => {
            contact_manifold_capsule_capsule(pos12, capsule1, capsule2, prediction, ref manifold);
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
            contact_manifold_cuboid_capsule(pos12, cuboid1, capsule2, prediction, ref manifold);
            true
        },
        (
            Shape::Capsule(_), Shape::Cuboid(_),
        ) => contact_manifold_cuboid_capsule_shapes(
            pos12, shape1, shape2, prediction, ref manifold,
        ),
        (
            Shape::Cuboid(cuboid1), Shape::Segment(segment2),
        ) => {
            contact_manifold_cuboid_segment(pos12, cuboid1, segment2, prediction, ref manifold);
            true
        },
        (
            Shape::Segment(_), Shape::Cuboid(_),
        ) => contact_manifold_cuboid_segment_shapes(
            pos12, shape1, shape2, prediction, ref manifold,
        ),
        (
            Shape::HalfSpace(halfspace1), Shape::Cuboid(_),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12, halfspace1, shape2, prediction, ref manifold, false,
            );
            true
        },
        (
            Shape::HalfSpace(halfspace1), Shape::Segment(_),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12, halfspace1, shape2, prediction, ref manifold, false,
            );
            true
        },
        (
            Shape::HalfSpace(halfspace1), Shape::Capsule(_),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12, halfspace1, shape2, prediction, ref manifold, false,
            );
            true
        },
        (
            Shape::Cuboid(_), Shape::HalfSpace(halfspace2),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12.inverse(), halfspace2, shape1, prediction, ref manifold, true,
            );
            true
        },
        (
            Shape::Segment(_), Shape::HalfSpace(halfspace2),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12.inverse(), halfspace2, shape1, prediction, ref manifold, true,
            );
            true
        },
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

#[cfg(test)]
pub mod alternatives;

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::contact::{ContactData, ContactManifold, ContactManifoldTrait};
    use crate::contact_generators::ball_ball::contact_manifold_ball_ball;
    use crate::contact_generators::capsule_capsule::contact_manifold_capsule_capsule;
    use crate::contact_generators::convex_ball::{
        contact_manifold_ball_convex, contact_manifold_convex_ball,
    };
    use crate::contact_generators::cuboid_capsule::{
        contact_manifold_cuboid_capsule, contact_manifold_cuboid_capsule_shapes,
    };
    use crate::contact_generators::cuboid_cuboid::contact_manifold_cuboid_cuboid;
    use crate::contact_generators::cuboid_segment::{
        contact_manifold_cuboid_segment, contact_manifold_cuboid_segment_shapes,
    };
    use crate::contact_generators::halfspace_pfm::contact_manifold_halfspace_pfm;
    use crate::shape::{Ball, Capsule, Cuboid, HalfSpace, Segment, Shape};
    use super::alternatives::{
        contact_manifold_helpers, contact_manifold_outlined, contact_manifold_shapes_chain,
    };
    use super::contact_manifold;

    const PREDICTION: Fixed = Fixed { raw: 85899346 };
    /// 30 degrees.
    const R_30: Rot2 = Rot2 { re: Fixed { raw: 3719550787 }, im: HALF };
    const QUARTER: Fixed = Fixed { raw: 1073741824 };

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn ball_s() -> Ball {
        Ball { radius: HALF }
    }
    fn cuboid_s() -> Cuboid {
        Cuboid { half_extents: v(ONE, HALF) }
    }
    fn capsule_s() -> Capsule {
        Capsule { segment: segment_s(), radius: QUARTER }
    }
    fn segment_s() -> Segment {
        Segment { a: v(-HALF, ZERO), b: v(HALF, ZERO) }
    }
    fn halfspace_s() -> HalfSpace {
        HalfSpace { normal: v(ZERO, ONE) }
    }

    /// Shape 2 slightly below-right of shape 1 (half-space normal +Y): every pair overlaps.
    fn pose_direct() -> Pose2 {
        Pose2 { translation: v(HALF, FixedTrait::from_ratio(1, 8)), rotation: R_30 }
    }
    /// The same scene, seen from the other side (used when shape 1 is the half-space's partner).
    fn pose_reversed() -> Pose2 {
        Pose2 { translation: v(HALF, FixedTrait::from_ratio(-1, 8)), rotation: R_30 }
    }

    /// Every ordered pair of the closed enum: `(shape1, shape2, supported)`.
    fn all_pairs() -> Array<(Shape, Shape, bool)> {
        let shapes = array![
            Shape::Ball(ball_s()), Shape::Cuboid(cuboid_s()), Shape::Capsule(capsule_s()),
            Shape::Segment(segment_s()), Shape::HalfSpace(halfspace_s()),
        ];
        let mut pairs = array![];
        for s1 in shapes.span() {
            for s2 in shapes.span() {
                let unsupported = match (*s1, *s2) {
                    (Shape::Segment(_), Shape::Segment(_)) => true,
                    (Shape::Segment(_), Shape::Capsule(_)) => true,
                    (Shape::Capsule(_), Shape::Segment(_)) => true,
                    (Shape::HalfSpace(_), Shape::HalfSpace(_)) => true,
                    _ => false,
                };
                pairs.append((*s1, *s2, !unsupported));
            }
        }
        pairs
    }

    /// Sets the solver impulse of both point slots.
    fn with_impulse(m: ContactManifold, raw: i64) -> ContactManifold {
        let mut out = m;
        let [mut p0, mut p1] = m.points;
        let data = ContactData { impulse: FixedTrait::from_raw(raw), ..Default::default() };
        p0.data = data;
        p1.data = data;
        out.points = [p0, p1];
        out
    }

    /// Dispatcher vs chain on one pair and one pose, on a fresh then a warm manifold; `true`
    /// when the pair produced contacts.
    #[inline(never)]
    fn check_pair(pose: Pose2, shape1: Shape, shape2: Shape, supported: bool) -> bool {
        let mut a: ContactManifold = Default::default();
        let mut b: ContactManifold = Default::default();
        assert_eq!(contact_manifold(pose, shape1, shape2, PREDICTION, ref a), supported);
        assert_eq!(
            contact_manifold_shapes_chain(pose, shape1, shape2, PREDICTION, ref b), supported,
        );
        assert_eq!(a, b);
        if a.num_points == 0 {
            return false;
        }
        // Second pass over the warm manifold, with a solver impulse to carry over.
        a = with_impulse(a, 7);
        b = with_impulse(b, 7);
        contact_manifold(pose, shape1, shape2, PREDICTION, ref a);
        contact_manifold_shapes_chain(pose, shape1, shape2, PREDICTION, ref b);
        assert_eq!(a, b);
        assert_eq!(a.point(0).data.impulse.raw, 7);
        true
    }

    /// The dispatcher gives, for the 25 ordered pairs, the same result and the same manifold as
    /// upstream's chain of `*_shapes` wrappers, on a fresh manifold and on a warm one; every
    /// supported pair produces contacts under one of the two poses.
    #[test]
    fn test_dispatch_matches_shapes_chain_on_every_pair() {
        let mut supported = 0_u32;
        for (s1, s2, ok) in all_pairs().span() {
            let hit = check_pair(pose_direct(), *s1, *s2, *ok)
                | check_pair(pose_reversed(), *s1, *s2, *ok);
            assert_eq!(hit, *ok);
            if *ok {
                supported += 1;
            }
        }
        assert_eq!(supported, 21);
    }

    /// Ball–ball goes to the ball–ball generator, not to the convex–ball one (which accepts
    /// it too), and the flipped ball-first pair mirrors the ball-second one.
    #[test]
    fn test_ball_ball_is_matched_first() {
        let p = pose_direct();
        let mut m: ContactManifold = Default::default();
        let mut e: ContactManifold = Default::default();
        assert!(
            contact_manifold(p, Shape::Ball(ball_s()), Shape::Ball(ball_s()), PREDICTION, ref m),
        );
        contact_manifold_ball_ball(p, ball_s(), ball_s(), PREDICTION, ref e);
        assert_eq!(m, e);
        assert_eq!(m.num_points, 1);
    }

    /// Unsupported pairs return `false` and clear a manifold that held points.
    #[test]
    fn test_unsupported_pairs_clear_the_manifold() {
        let mut live: ContactManifold = Default::default();
        contact_manifold(
            pose_direct(), Shape::Ball(ball_s()), Shape::Ball(ball_s()), PREDICTION, ref live,
        );
        assert_eq!(live.num_points, 1);
        let unsupported = array![
            (Shape::Segment(segment_s()), Shape::Segment(segment_s())),
            (Shape::Segment(segment_s()), Shape::Capsule(capsule_s())),
            (Shape::Capsule(capsule_s()), Shape::Segment(segment_s())),
            (Shape::HalfSpace(halfspace_s()), Shape::HalfSpace(halfspace_s())),
        ];
        for (s1, s2) in unsupported.span() {
            let mut m = live;
            assert!(!contact_manifold(pose_direct(), *s1, *s2, PREDICTION, ref m));
            assert_eq!(m.num_points, 0);
        }
    }

    // Opaque inputs of the probes.
    fn pd() -> Pose2 {
        opaque(pose_direct())
    }
    fn pr() -> Pose2 {
        opaque(pose_reversed())
    }
    fn ball() -> Shape {
        opaque(Shape::Ball(ball_s()))
    }
    fn cuboid() -> Shape {
        opaque(Shape::Cuboid(cuboid_s()))
    }
    fn capsule() -> Shape {
        opaque(Shape::Capsule(capsule_s()))
    }
    fn segment() -> Shape {
        opaque(Shape::Segment(segment_s()))
    }
    fn halfspace() -> Shape {
        opaque(Shape::HalfSpace(halfspace_s()))
    }

    #[test]
    fn gas_baseline() {
        let _ = pd();
    }

    #[test]
    fn gas_dispatch_ball_ball() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold(pd(), ball(), ball(), PREDICTION, ref m);
    }

    #[test]
    fn gas_generator_ball_ball() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_ball_ball(pd(), opaque(ball_s()), opaque(ball_s()), PREDICTION, ref m);
    }

    #[test]
    fn gas_outlined_ball_ball() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold_outlined(pd(), ball(), ball(), PREDICTION, ref m);
    }

    #[test]
    fn gas_helpers_ball_ball() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold_helpers(pd(), ball(), ball(), PREDICTION, ref m);
    }

    #[test]
    fn gas_chain_ball_ball() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold_shapes_chain(pd(), ball(), ball(), PREDICTION, ref m);
    }

    #[test]
    fn gas_dispatch_cuboid_cuboid() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold(pd(), cuboid(), cuboid(), PREDICTION, ref m);
    }

    #[test]
    fn gas_generator_cuboid_cuboid() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_cuboid_cuboid(
            pd(), opaque(cuboid_s()), opaque(cuboid_s()), PREDICTION, ref m,
        );
    }

    #[test]
    fn gas_dispatch_capsule_capsule() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold(pd(), capsule(), capsule(), PREDICTION, ref m);
    }

    #[test]
    fn gas_generator_capsule_capsule() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_capsule_capsule(
            pd(), opaque(capsule_s()), opaque(capsule_s()), PREDICTION, ref m,
        );
    }

    #[test]
    fn gas_dispatch_ball_cuboid() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold(pd(), ball(), cuboid(), PREDICTION, ref m);
    }

    #[test]
    fn gas_generator_ball_cuboid() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_ball_convex(pd(), opaque(ball_s()), cuboid(), PREDICTION, ref m);
    }

    #[test]
    fn gas_dispatch_cuboid_ball() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold(pd(), cuboid(), ball(), PREDICTION, ref m);
    }

    #[test]
    fn gas_generator_cuboid_ball() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_convex_ball(pd(), cuboid(), opaque(ball_s()), PREDICTION, ref m);
    }

    #[test]
    fn gas_dispatch_cuboid_capsule() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold(pd(), cuboid(), capsule(), PREDICTION, ref m);
    }

    #[test]
    fn gas_generator_cuboid_capsule() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_cuboid_capsule(
            pd(), opaque(cuboid_s()), opaque(capsule_s()), PREDICTION, ref m,
        );
    }

    #[test]
    fn gas_dispatch_capsule_cuboid() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold(pd(), capsule(), cuboid(), PREDICTION, ref m);
    }

    #[test]
    fn gas_generator_capsule_cuboid() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold_cuboid_capsule_shapes(
            pd(), capsule(), cuboid(), PREDICTION, ref m,
        );
    }

    #[test]
    fn gas_dispatch_cuboid_segment() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold(pd(), cuboid(), segment(), PREDICTION, ref m);
    }

    #[test]
    fn gas_generator_cuboid_segment() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_cuboid_segment(
            pd(), opaque(cuboid_s()), opaque(segment_s()), PREDICTION, ref m,
        );
    }

    #[test]
    fn gas_outlined_cuboid_segment() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold_outlined(pd(), cuboid(), segment(), PREDICTION, ref m);
    }

    #[test]
    fn gas_helpers_cuboid_segment() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold_helpers(pd(), cuboid(), segment(), PREDICTION, ref m);
    }

    #[test]
    fn gas_chain_cuboid_segment() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold_shapes_chain(pd(), cuboid(), segment(), PREDICTION, ref m);
    }

    #[test]
    fn gas_dispatch_segment_cuboid() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold(pd(), segment(), cuboid(), PREDICTION, ref m);
    }

    #[test]
    fn gas_generator_segment_cuboid() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold_cuboid_segment_shapes(
            pd(), segment(), cuboid(), PREDICTION, ref m,
        );
    }

    #[test]
    fn gas_dispatch_halfspace_cuboid() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold(pd(), halfspace(), cuboid(), PREDICTION, ref m);
    }

    #[test]
    fn gas_generator_halfspace_cuboid() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_halfspace_pfm(
            pd(), opaque(halfspace_s()), cuboid(), PREDICTION, ref m, false,
        );
    }

    #[test]
    fn gas_dispatch_halfspace_segment() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold(pd(), halfspace(), segment(), PREDICTION, ref m);
    }

    #[test]
    fn gas_generator_halfspace_segment() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_halfspace_pfm(
            pd(), opaque(halfspace_s()), segment(), PREDICTION, ref m, false,
        );
    }

    #[test]
    fn gas_dispatch_halfspace_capsule() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold(pd(), halfspace(), capsule(), PREDICTION, ref m);
    }

    #[test]
    fn gas_generator_halfspace_capsule() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_halfspace_pfm(
            pd(), opaque(halfspace_s()), capsule(), PREDICTION, ref m, false,
        );
    }

    #[test]
    fn gas_dispatch_cuboid_halfspace() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold(pr(), cuboid(), halfspace(), PREDICTION, ref m);
    }

    #[test]
    fn gas_generator_cuboid_halfspace() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_halfspace_pfm(
            pr().inverse(), opaque(halfspace_s()), cuboid(), PREDICTION, ref m, true,
        );
    }

    #[test]
    fn gas_dispatch_segment_halfspace() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold(pr(), segment(), halfspace(), PREDICTION, ref m);
    }

    #[test]
    fn gas_generator_segment_halfspace() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_halfspace_pfm(
            pr().inverse(), opaque(halfspace_s()), segment(), PREDICTION, ref m, true,
        );
    }

    #[test]
    fn gas_dispatch_capsule_halfspace() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold(pr(), capsule(), halfspace(), PREDICTION, ref m);
    }

    #[test]
    fn gas_generator_capsule_halfspace() {
        let mut m: ContactManifold = Default::default();
        contact_manifold_halfspace_pfm(
            pr().inverse(), opaque(halfspace_s()), capsule(), PREDICTION, ref m, true,
        );
    }

    #[test]
    fn gas_outlined_capsule_halfspace() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold_outlined(pr(), capsule(), halfspace(), PREDICTION, ref m);
    }

    #[test]
    fn gas_helpers_capsule_halfspace() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold_helpers(pr(), capsule(), halfspace(), PREDICTION, ref m);
    }

    #[test]
    fn gas_chain_capsule_halfspace() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold_shapes_chain(pr(), capsule(), halfspace(), PREDICTION, ref m);
    }

    #[test]
    fn gas_dispatch_segment_segment() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold(pd(), segment(), segment(), PREDICTION, ref m);
    }

    #[test]
    fn gas_outlined_segment_segment() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold_outlined(pd(), segment(), segment(), PREDICTION, ref m);
    }

    #[test]
    fn gas_helpers_segment_segment() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold_helpers(pd(), segment(), segment(), PREDICTION, ref m);
    }

    #[test]
    fn gas_chain_segment_segment() {
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold_shapes_chain(pd(), segment(), segment(), PREDICTION, ref m);
    }
}
