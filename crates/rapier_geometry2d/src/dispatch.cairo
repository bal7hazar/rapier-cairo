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
//!    `halfspace_pfm::contact_manifold_halfspace_pfm`;
//! 8. polygon – polygon / cuboid / segment / capsule (both orders): SAT + PFM clipping;
//! 9. triangle or round shape – anything but a ball or a half-space (SH1):
//!    `pfm_pfm::contact_manifold_pfm_pfm`, SAT + PFM clipping on the cores, border radii added.
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
//! # Intersection tests
//!
//! [`intersection_test`] (`intersection`, work package SE) is Parry's
//! `DefaultQueryDispatcher::intersection_test`, the boolean query behind sensor pairs: its own
//! table, with exact analytic or SAT kernels per pair (no clipping, no prediction), every pair of
//! the closed enum but half-space–half-space. Deriving the answer from this module's contact
//! generators at zero prediction costs 1.4 to 20 times more gas (`intersection::alternatives`).
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
//! 1. **metered `#[inline(always)]` typed `match`** (this module): each generator call is inside a
//!    one-iteration `while`, so an outlined caller is charged only the reached generator.
//! 2. `alternatives::contact_manifold_plain`: GG's original typed `match`, still cheapest when
//!    fully inlined into a caller, but a loop-free outlined caller pays its costliest arm for every
//!    pair.
//! 3. `alternatives::contact_manifold_helpers`: the plain match with one `#[inline(never)]` helper
//!    per arm: no help for an outlined caller, and about 7k more when inlined.
//! 4. `alternatives::contact_manifold_shapes_chain`: upstream's shape, the `*_shapes` wrappers
//!    tried in turn (each re-matches the enum): 54 670 / 1 966 870 / 2 198 450.
//! 5. `alternatives::contact_manifold_plain_outlined`: the plain match behind a call: 556 300 gas
//!    for every pair (the most expensive arm), 565 / 3 221 / 208 steps.
//!
//! Why the metering: Sierra gas charges a loop-free called function its worst-case path, so a plain
//! outlined dispatcher costs the most expensive generator on every pair (ball–ball would cost 14
//! times the generator). The one-iteration loops defer the generator charge until the reached arm's
//! loop body runs; that costs about 265 Cairo steps per pair and preserves the real pipeline's gas
//! ranking when the dispatcher is called from outlined narrow-phase code.
//!
//! # The step's variant
//!
//! [`contact_manifold_step`] is the same table for the narrow phase's pair loop
//! (`rapier_dynamics2d::narrow_phase::compute_contacts_from_scratch`, through
//! `rapier2d::dispatcher::DefaultDispatcher`), which inlines its dispatcher into a loop body where
//! each arm is charged only when taken: the metering loops only cost frames there, so it drops
//! them, and it hoists the persistence fast path of the cuboid pairs in front of their generators.
//! Neither formulation wins everywhere (work package CL): outlined, [`contact_manifold`] pays the
//! reached arm only and [`contact_manifold_step`] the costliest one; inlined in the pair loop,
//! [`contact_manifold_step`] is cheaper on every P3 scene (narrow-phase stage, Sierra gas | Cairo
//! steps, `rapier2d::pipeline::narrow_benches`, `cuboid_stack(3)` / `balls_on_halfspace(8)` /
//! `mixed_pile`: 1 640 685 | 11 680 / 2 114 000 | 12 357 / 4 577 705 | 32 156 against
//! 2 466 285 | 13 067 / 2 252 120 | 14 159 / 5 469 945 | 36 103 for [`contact_manifold`]).
//! Probes of this module (gross): ball–ball through [`contact_manifold_step`] 57 370 inlined
//! (79 060 for [`contact_manifold`]), 648 620 behind a loop-free call
//! (`gas_step_outlined_ball_ball`); a warm cuboid–cuboid pair 617 230 against 1 039 730 for
//! [`contact_manifold`] (`gas_*_cuboid_cuboid_warm`, common setup).

use fixed::{Fixed, ZERO};
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
use crate::contact_generators::cuboid_cuboid::{contact_manifold_cuboid_cuboid, cuboid_cuboid_fresh};
use crate::contact_generators::cuboid_segment::{
    contact_manifold_cuboid_segment, contact_manifold_cuboid_segment_shapes,
};
use crate::contact_generators::halfspace_pfm::contact_manifold_halfspace_pfm;
use crate::contact_generators::pfm_pfm::contact_manifold_pfm_pfm;
use crate::contact_generators::polygon_polygon::{
    contact_manifold_polygon_cuboid, contact_manifold_polygon_polygon, polygon_cuboid_fresh,
    polygon_polygon_fresh,
};
use crate::contact_generators::polygon_segment::{
    contact_manifold_polygon_capsule, contact_manifold_polygon_segment,
    generate_fresh as polygon_segment_fresh,
};
use crate::manifold::ManifoldTrait;
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
                contact_manifold_cuboid_cuboid(pos12, cuboid1, cuboid2, prediction, ref manifold);
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
        // SH1 arms never share an old arm: that would move the old arms' code layout.
        (Shape::Ball(ball1), Shape::Triangle(_)) | (Shape::Ball(ball1), Shape::RoundCuboid(_)) |
        (Shape::Ball(ball1), Shape::RoundTriangle(_)) |
        (
            Shape::Ball(ball1), Shape::RoundConvexPolygon(_),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_ball_convex(pos12, ball1, shape2, prediction, ref manifold);
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
        (Shape::Triangle(_), Shape::Ball(ball2)) | (Shape::RoundCuboid(_), Shape::Ball(ball2)) |
        (Shape::RoundTriangle(_), Shape::Ball(ball2)) |
        (
            Shape::RoundConvexPolygon(_), Shape::Ball(ball2),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_convex_ball(pos12, shape1, ball2, prediction, ref manifold);
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
                contact_manifold_cuboid_capsule(pos12, cuboid1, capsule2, prediction, ref manifold);
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
                contact_manifold_cuboid_segment(pos12, cuboid1, segment2, prediction, ref manifold);
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
        (Shape::HalfSpace(halfspace1), Shape::ConvexPolygon(_)) |
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
        (Shape::ConvexPolygon(_), Shape::HalfSpace(halfspace2)) |
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
        (Shape::HalfSpace(halfspace1), Shape::Triangle(_)) |
        (Shape::HalfSpace(halfspace1), Shape::RoundCuboid(_)) |
        (Shape::HalfSpace(halfspace1), Shape::RoundTriangle(_)) |
        (
            Shape::HalfSpace(halfspace1), Shape::RoundConvexPolygon(_),
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
        (Shape::Triangle(_), Shape::HalfSpace(halfspace2)) |
        (Shape::RoundCuboid(_), Shape::HalfSpace(halfspace2)) |
        (Shape::RoundTriangle(_), Shape::HalfSpace(halfspace2)) |
        (
            Shape::RoundConvexPolygon(_), Shape::HalfSpace(halfspace2),
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
        (
            Shape::ConvexPolygon(a), Shape::ConvexPolygon(b),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_polygon_polygon(
                    pos12, a.unbox(), b.unbox(), prediction, ref manifold,
                );
                pending = false;
            }
            true
        },
        (
            Shape::ConvexPolygon(a), Shape::Cuboid(b),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_polygon_cuboid(
                    pos12, a.unbox(), b, prediction, false, ref manifold,
                );
                pending = false;
            }
            true
        },
        (
            Shape::Cuboid(b), Shape::ConvexPolygon(a),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_polygon_cuboid(
                    pos12.inverse(), a.unbox(), b, prediction, true, ref manifold,
                );
                pending = false;
            }
            true
        },
        (
            Shape::ConvexPolygon(a), Shape::Segment(b),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_polygon_segment(
                    pos12, a.unbox(), b, prediction, false, ref manifold,
                );
                pending = false;
            }
            true
        },
        (
            Shape::Segment(b), Shape::ConvexPolygon(a),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_polygon_segment(
                    pos12.inverse(), a.unbox(), b, prediction, true, ref manifold,
                );
                pending = false;
            }
            true
        },
        (
            Shape::ConvexPolygon(a), Shape::Capsule(b),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_polygon_capsule(
                    pos12, a.unbox(), b, prediction, false, ref manifold,
                );
                pending = false;
            }
            true
        },
        (
            Shape::Capsule(b), Shape::ConvexPolygon(a),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_polygon_capsule(
                    pos12.inverse(), a.unbox(), b, prediction, true, ref manifold,
                );
                pending = false;
            }
            true
        },
        // SH1: always supported (balls and half-spaces are matched above). Shape 2 is rebuilt
        // from its payload: using it whole would keep a copy alive in the rows of the old shapes.
        (Shape::Triangle(_), _) | (Shape::RoundCuboid(_), _) | (Shape::RoundTriangle(_), _) |
        (
            Shape::RoundConvexPolygon(_), _,
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_pfm_pfm(pos12, shape1, shape2, prediction, ref manifold);
                pending = false;
            }
            true
        },
        (
            _, Shape::Triangle(t2),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_pfm_pfm(
                    pos12, shape1, Shape::Triangle(t2), prediction, ref manifold,
                );
                pending = false;
            }
            true
        },
        (
            _, Shape::RoundCuboid(r2),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_pfm_pfm(
                    pos12, shape1, Shape::RoundCuboid(r2), prediction, ref manifold,
                );
                pending = false;
            }
            true
        },
        (
            _, Shape::RoundTriangle(r2),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_pfm_pfm(
                    pos12, shape1, Shape::RoundTriangle(r2), prediction, ref manifold,
                );
                pending = false;
            }
            true
        },
        (
            _, Shape::RoundConvexPolygon(r2),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_pfm_pfm(
                    pos12, shape1, Shape::RoundConvexPolygon(r2), prediction, ref manifold,
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

/// [`contact_manifold`] for the step's pair loop: the same table (same arms, same order, same
/// generators) as a plain typed `match`, with the persistence fast path of the cuboid pairs
/// hoisted in front of their generators.
///
/// The cuboid–cuboid, cuboid–capsule and cuboid–segment generators (both orders) start with
/// `manifold.try_update_contacts(pos12)` and return when it succeeds. This function runs that
/// check itself and calls the generator only when it fails (`try_update_contacts` leaves the
/// manifold untouched when it fails), so a resting pair skips the generator's call, which Sierra
/// gas charges its costliest path (SAT and clipping). Since BT3 the cuboid–cuboid and the
/// non-reversed polygon arms call the generator's `*_fresh` entry, which does not repeat the
/// check (a moving warm cuboid pair: 7 326 → 6 845 Cairo steps, `gas_step_*cuboid_cuboid_moved`;
/// the level-10 impact tick's contact generation −3.7k steps); the capsule and segment arms keep
/// their generator's own check (the segment one uses other thresholds).
/// Results are bit-identical to [`contact_manifold`] on every pair; the only difference is that a
/// capsule–cuboid or segment–cuboid pair whose fast path succeeds skips the generator's
/// `pos12.inverse()`, whose only effect is a panic on an unrepresentable pose.
///
/// Polygon pairs also use persistence. Reversed polygon pairs keep the check inside their
/// generator: moving it before the rounded double inversion changes distances by one ulp.
/// Arguments, returned value and panics: as [`contact_manifold`].
/// #### Cost
/// Only for callers that inline it into a loop body (the narrow phase's pair loop), where each
/// arm is charged only when taken: there, the metered loops of [`contact_manifold`] only cost
/// their frames (about 265 Cairo steps per pair). A loop-free outlined caller would pay the
/// costliest arm for every pair: use [`contact_manifold`] there.
#[inline(always)]
pub fn contact_manifold_step(
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
            if !manifold.try_update_contacts(pos12) {
                cuboid_cuboid_fresh(pos12, cuboid1, cuboid2, prediction, ref manifold);
            }
            true
        },
        (
            Shape::Capsule(capsule1), Shape::Capsule(capsule2),
        ) => {
            contact_manifold_capsule_capsule(pos12, capsule1, capsule2, prediction, ref manifold);
            true
        },
        // SH1 arms never share an old arm (see [`contact_manifold`]).
        (Shape::Ball(ball1), Shape::Triangle(_)) | (Shape::Ball(ball1), Shape::RoundCuboid(_)) |
        (Shape::Ball(ball1), Shape::RoundTriangle(_)) |
        (
            Shape::Ball(ball1), Shape::RoundConvexPolygon(_),
        ) => {
            contact_manifold_ball_convex(pos12, ball1, shape2, prediction, ref manifold);
            true
        },
        (
            Shape::Ball(ball1), _,
        ) => {
            contact_manifold_ball_convex(pos12, ball1, shape2, prediction, ref manifold);
            true
        },
        (Shape::Triangle(_), Shape::Ball(ball2)) | (Shape::RoundCuboid(_), Shape::Ball(ball2)) |
        (Shape::RoundTriangle(_), Shape::Ball(ball2)) |
        (
            Shape::RoundConvexPolygon(_), Shape::Ball(ball2),
        ) => {
            contact_manifold_convex_ball(pos12, shape1, ball2, prediction, ref manifold);
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
                contact_manifold_cuboid_capsule(pos12, cuboid1, capsule2, prediction, ref manifold);
            }
            true
        },
        (
            Shape::Capsule(_), Shape::Cuboid(_),
        ) => {
            if manifold.try_update_contacts(pos12) {
                return true;
            }
            contact_manifold_cuboid_capsule_shapes(pos12, shape1, shape2, prediction, ref manifold)
        },
        (
            Shape::Cuboid(cuboid1), Shape::Segment(segment2),
        ) => {
            if !manifold.try_update_contacts(pos12) {
                contact_manifold_cuboid_segment(pos12, cuboid1, segment2, prediction, ref manifold);
            }
            true
        },
        (
            Shape::Segment(_), Shape::Cuboid(_),
        ) => {
            if manifold.try_update_contacts(pos12) {
                return true;
            }
            contact_manifold_cuboid_segment_shapes(pos12, shape1, shape2, prediction, ref manifold)
        },
        (Shape::HalfSpace(halfspace1), Shape::ConvexPolygon(_)) |
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
        (Shape::ConvexPolygon(_), Shape::HalfSpace(halfspace2)) |
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
        (Shape::HalfSpace(halfspace1), Shape::Triangle(_)) |
        (Shape::HalfSpace(halfspace1), Shape::RoundCuboid(_)) |
        (Shape::HalfSpace(halfspace1), Shape::RoundTriangle(_)) |
        (
            Shape::HalfSpace(halfspace1), Shape::RoundConvexPolygon(_),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12, halfspace1, shape2, prediction, ref manifold, false,
            );
            true
        },
        (Shape::Triangle(_), Shape::HalfSpace(halfspace2)) |
        (Shape::RoundCuboid(_), Shape::HalfSpace(halfspace2)) |
        (Shape::RoundTriangle(_), Shape::HalfSpace(halfspace2)) |
        (
            Shape::RoundConvexPolygon(_), Shape::HalfSpace(halfspace2),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12.inverse(), halfspace2, shape1, prediction, ref manifold, true,
            );
            true
        },
        (
            Shape::ConvexPolygon(a), Shape::ConvexPolygon(b),
        ) => {
            if !manifold.try_update_contacts(pos12) {
                polygon_polygon_fresh(pos12, a.unbox(), b.unbox(), prediction, ref manifold);
            }
            true
        },
        (
            Shape::ConvexPolygon(a), Shape::Cuboid(b),
        ) => {
            if !manifold.try_update_contacts(pos12) {
                polygon_cuboid_fresh(pos12, a.unbox(), b, prediction, false, ref manifold);
            }
            true
        },
        (
            Shape::Cuboid(b), Shape::ConvexPolygon(a),
        ) => {
            contact_manifold_polygon_cuboid(
                pos12.inverse(), a.unbox(), b, prediction, true, ref manifold,
            );
            true
        },
        (
            Shape::ConvexPolygon(a), Shape::Segment(b),
        ) => {
            if !manifold.try_update_contacts(pos12) {
                polygon_segment_fresh(pos12, a.unbox(), b, ZERO, prediction, false, ref manifold);
            }
            true
        },
        (
            Shape::Segment(b), Shape::ConvexPolygon(a),
        ) => {
            contact_manifold_polygon_segment(
                pos12.inverse(), a.unbox(), b, prediction, true, ref manifold,
            );
            true
        },
        (
            Shape::ConvexPolygon(a), Shape::Capsule(b),
        ) => {
            if !manifold.try_update_contacts(pos12) {
                polygon_segment_fresh(
                    pos12, a.unbox(), b.segment, b.radius, prediction, false, ref manifold,
                );
            }
            true
        },
        (
            Shape::Capsule(b), Shape::ConvexPolygon(a),
        ) => {
            contact_manifold_polygon_capsule(
                pos12.inverse(), a.unbox(), b, prediction, true, ref manifold,
            );
            true
        },
        // SH1: as in [`contact_manifold`].
        (Shape::Triangle(_), _) | (Shape::RoundCuboid(_), _) | (Shape::RoundTriangle(_), _) |
        (
            Shape::RoundConvexPolygon(_), _,
        ) => {
            contact_manifold_pfm_pfm(pos12, shape1, shape2, prediction, ref manifold);
            true
        },
        (
            _, Shape::Triangle(t2),
        ) => {
            contact_manifold_pfm_pfm(pos12, shape1, Shape::Triangle(t2), prediction, ref manifold);
            true
        },
        (
            _, Shape::RoundCuboid(r2),
        ) => {
            contact_manifold_pfm_pfm(
                pos12, shape1, Shape::RoundCuboid(r2), prediction, ref manifold,
            );
            true
        },
        (
            _, Shape::RoundTriangle(r2),
        ) => {
            contact_manifold_pfm_pfm(
                pos12, shape1, Shape::RoundTriangle(r2), prediction, ref manifold,
            );
            true
        },
        (
            _, Shape::RoundConvexPolygon(r2),
        ) => {
            contact_manifold_pfm_pfm(
                pos12, shape1, Shape::RoundConvexPolygon(r2), prediction, ref manifold,
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
pub mod intersection;
pub use intersection::intersection_test;

#[cfg(test)]
mod tests;
