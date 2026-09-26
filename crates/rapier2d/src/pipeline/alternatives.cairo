//! Rejected (or not yet adopted) pipeline candidates, kept for re-ranking (AGENTS.md §5). The
//! measured ranking is in the pipeline module documentation and in `REPORT.md`.

use fixed::{Fixed, HALF};
use glam::Vec2;
use kind::{
    BALL_BALL, BALL_CONVEX, BallBallDispatcher, BallConvexDispatcher, CAPSULE_CAPSULE, CONVEX_BALL,
    CUBOID_CUBOID, CapsuleCapsuleDispatcher, ConvexBallDispatcher, CuboidCuboidDispatcher,
    OtherDispatcher, pair_kind,
};
use rapier_core::collider::ColliderChangesTrait;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_core::rigid_body::changes::{COLLIDERS, LOCAL_MASS_PROPERTIES, POSITION};
use rapier_core::rigid_body::{RigidBodyChanges, RigidBodyChangesTrait};
use rapier_dynamics2d::collider::ColliderTrait;
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::events::CollisionEvent;
use rapier_dynamics2d::joint::{ImpulseJointSet, ImpulseJointSetTrait};
use rapier_dynamics2d::narrow_phase::{
    CarryOver, ContactDispatcher, ContactPair, NarrowPhase, NarrowPhaseTrait, PairCollider,
    SortedMerge, SortedMergeCarryOver, dropped_events, pair_colliders, process_pair,
};
use rapier_dynamics2d::rigid_body::RigidBodyMassPropsTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBodySet, RigidBodySetTrait, RigidBodyTrait};
use rapier_dynamics2d::solver::body_store::SolverBodyStoreTrait;
use rapier_dynamics2d::solver::island::solve_island;
use rapier_geometry2d::aabb::AabbTrait;
use rapier_geometry2d::broad_phase::{BroadPhaseProxy, find_pairs};
use rapier_geometry2d::contact::ContactManifold;
use rapier_geometry2d::dispatch::contact_manifold;
use rapier_geometry2d::shape::Shape;
use rapier_math::pose2::Pose2;
use crate::world::World;
use super::sleeping::wake_touched_partners;
use super::{
    advance_to_final_positions, body_changes, collider_changes, joint_values,
    recompute_mass_properties_from_colliders, solve, write_joints,
};

/// The metered `rapier_geometry2d::dispatch::contact_manifold` as a plain `#[inline(always)]`
/// call: GG's option (a), and `DefaultDispatcher` until work package CL moved it to
/// `dispatch::contact_manifold_step` (inlined in the pair loop, each metered arm costs a loop
/// frame the plain arms of `contact_manifold_step` do not).
pub impl InlineDispatcher of ContactDispatcher {
    #[inline(always)]
    fn contact_manifold(
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) -> bool {
        contact_manifold(pos12, shape1, shape2, prediction, ref manifold)
    }
}

/// GG's dispatcher behind a call.
pub impl OutlinedDispatcher of ContactDispatcher {
    #[inline(never)]
    fn contact_manifold(
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) -> bool {
        contact_manifold(pos12, shape1, shape2, prediction, ref manifold)
    }
}

/// Option (b): DD's `compute_contacts` loop rebuilt from its public pieces, with the pair kind
/// resolved in the loop so that each kind runs a `process_pair` instantiation that reaches one
/// generator only. Same pairs, events and order as `compute_contacts::<InlineDispatcher>`.
pub fn compute_contacts_by_kind(
    ref narrow_phase: NarrowPhase,
    prediction: Fixed,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    pairs: Span<(u32, u32)>,
) -> Array<CollisionEvent> {
    let scratch = pair_colliders(ref bodies, ref colliders).span();
    let mut carry: SortedMerge = SortedMergeCarryOver::begin(narrow_phase.pairs.span());
    let mut current = array![];
    let mut transitions = array![];
    for (i, j) in pairs {
        let co1 = *scratch.at(*i);
        let co2 = *scratch.at(*j);
        if !(co1.solid && co2.solid) {
            continue;
        }
        let previous = carry.take(co1.handle, co2.handle);
        let (pair, event) = process_by_kind(prediction, co1, co2, previous);
        current.append(pair);
        if let Some(event) = event {
            transitions.append(event);
        }
    }
    let mut events = dropped_events(carry.finish(), ref colliders);
    events.append_span(transitions.span());
    narrow_phase.pairs = current;
    events
}

fn process_by_kind(
    prediction: Fixed, co1: PairCollider, co2: PairCollider, previous: Option<ContactPair>,
) -> (ContactPair, Option<CollisionEvent>) {
    let kind = pair_kind(co1.shape, co2.shape);
    if kind == BALL_BALL {
        process_ball_ball(prediction, co1, co2, previous)
    } else if kind == CUBOID_CUBOID {
        process_cuboid_cuboid(prediction, co1, co2, previous)
    } else if kind == CAPSULE_CAPSULE {
        process_capsule_capsule(prediction, co1, co2, previous)
    } else if kind == BALL_CONVEX {
        process_ball_convex(prediction, co1, co2, previous)
    } else if kind == CONVEX_BALL {
        process_convex_ball(prediction, co1, co2, previous)
    } else {
        process_other(prediction, co1, co2, previous)
    }
}

#[inline(never)]
fn process_ball_ball(
    prediction: Fixed, co1: PairCollider, co2: PairCollider, previous: Option<ContactPair>,
) -> (ContactPair, Option<CollisionEvent>) {
    process_pair::<BallBallDispatcher>(prediction, co1, co2, previous)
}

#[inline(never)]
fn process_cuboid_cuboid(
    prediction: Fixed, co1: PairCollider, co2: PairCollider, previous: Option<ContactPair>,
) -> (ContactPair, Option<CollisionEvent>) {
    process_pair::<CuboidCuboidDispatcher>(prediction, co1, co2, previous)
}

#[inline(never)]
fn process_capsule_capsule(
    prediction: Fixed, co1: PairCollider, co2: PairCollider, previous: Option<ContactPair>,
) -> (ContactPair, Option<CollisionEvent>) {
    process_pair::<CapsuleCapsuleDispatcher>(prediction, co1, co2, previous)
}

#[inline(never)]
fn process_ball_convex(
    prediction: Fixed, co1: PairCollider, co2: PairCollider, previous: Option<ContactPair>,
) -> (ContactPair, Option<CollisionEvent>) {
    process_pair::<BallConvexDispatcher>(prediction, co1, co2, previous)
}

#[inline(never)]
fn process_convex_ball(
    prediction: Fixed, co1: PairCollider, co2: PairCollider, previous: Option<ContactPair>,
) -> (ContactPair, Option<CollisionEvent>) {
    process_pair::<ConvexBallDispatcher>(prediction, co1, co2, previous)
}

#[inline(never)]
fn process_other(
    prediction: Fixed, co1: PairCollider, co2: PairCollider, previous: Option<ContactPair>,
) -> (ContactPair, Option<CollisionEvent>) {
    process_pair::<OtherDispatcher>(prediction, co1, co2, previous)
}

/// Option (c): the per-kind `match` of [`compute_contacts_by_kind`], but each arm runs its
/// `process_pair` instantiation inside a one-iteration loop. A loop withdraws its gas when an
/// iteration starts, so the caller's static cost no longer includes the generator and only the
/// arm that runs pays for it.
pub fn compute_contacts_metered(
    ref narrow_phase: NarrowPhase,
    prediction: Fixed,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    pairs: Span<(u32, u32)>,
) -> Array<CollisionEvent> {
    let scratch = pair_colliders(ref bodies, ref colliders).span();
    let mut carry: SortedMerge = SortedMergeCarryOver::begin(narrow_phase.pairs.span());
    let mut current = array![];
    let mut transitions = array![];
    for (i, j) in pairs {
        let co1 = *scratch.at(*i);
        let co2 = *scratch.at(*j);
        if !(co1.solid && co2.solid) {
            continue;
        }
        let previous = carry.take(co1.handle, co2.handle);
        let kind = pair_kind(co1.shape, co2.shape);
        let (pair, event) = if kind == BALL_BALL {
            metered::<BallBallDispatcher>(prediction, co1, co2, previous)
        } else if kind == CUBOID_CUBOID {
            metered::<CuboidCuboidDispatcher>(prediction, co1, co2, previous)
        } else if kind == CAPSULE_CAPSULE {
            metered::<CapsuleCapsuleDispatcher>(prediction, co1, co2, previous)
        } else if kind == BALL_CONVEX {
            metered::<BallConvexDispatcher>(prediction, co1, co2, previous)
        } else if kind == CONVEX_BALL {
            metered::<ConvexBallDispatcher>(prediction, co1, co2, previous)
        } else {
            metered::<OtherDispatcher>(prediction, co1, co2, previous)
        };
        current.append(pair);
        if let Some(event) = event {
            transitions.append(event);
        }
    }
    let mut events = dropped_events(carry.finish(), ref colliders);
    events.append_span(transitions.span());
    narrow_phase.pairs = current;
    events
}

/// `process_pair::<D>` behind a one-iteration loop (see [`compute_contacts_metered`]).
#[inline(never)]
fn metered<impl D: ContactDispatcher>(
    prediction: Fixed, co1: PairCollider, co2: PairCollider, previous: Option<ContactPair>,
) -> (ContactPair, Option<CollisionEvent>) {
    loop {
        break process_pair::<D>(prediction, co1, co2, previous);
    }
}

/// One pair of the bucketed narrow phase: its rank among the solid pairs, its colliders and the
/// previous step's pair of the same key.
#[derive(Copy, Drop)]
pub struct PairJob {
    pub rank: u32,
    pub co1: PairCollider,
    pub co2: PairCollider,
    pub previous: Option<ContactPair>,
}

/// A processed [`PairJob`].
#[derive(Copy, Drop)]
pub struct PairDone {
    pub rank: u32,
    pub pair: ContactPair,
    pub event: Option<CollisionEvent>,
}

/// Option (b) proper: one pass classifies the pairs by kind (carry-over lookup in ascending key,
/// as DD), one loop per kind runs `process_pair` with the dispatcher of that kind only, one pass
/// merges the results back in pair order. Same pairs, events and order as
/// `compute_contacts::<InlineDispatcher>`.
pub fn compute_contacts_bucketed(
    ref narrow_phase: NarrowPhase,
    prediction: Fixed,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    pairs: Span<(u32, u32)>,
) -> Array<CollisionEvent> {
    let scratch = pair_colliders(ref bodies, ref colliders).span();
    let mut carry: SortedMerge = SortedMergeCarryOver::begin(narrow_phase.pairs.span());
    let (mut b0, mut b1, mut b2, mut b3, mut b4, mut b5) = (
        array![], array![], array![], array![], array![], array![],
    );
    let mut rank = 0;
    for (i, j) in pairs {
        let co1 = *scratch.at(*i);
        let co2 = *scratch.at(*j);
        if !(co1.solid && co2.solid) {
            continue;
        }
        let previous = carry.take(co1.handle, co2.handle);
        let job = PairJob { rank, co1, co2, previous };
        let kind = pair_kind(co1.shape, co2.shape);
        if kind == BALL_BALL {
            b0.append(job);
        } else if kind == CUBOID_CUBOID {
            b1.append(job);
        } else if kind == CAPSULE_CAPSULE {
            b2.append(job);
        } else if kind == BALL_CONVEX {
            b3.append(job);
        } else if kind == CONVEX_BALL {
            b4.append(job);
        } else {
            b5.append(job);
        }
        rank += 1;
    }
    let mut r0 = run_bucket::<BallBallDispatcher>(b0.span(), prediction).span();
    let mut r1 = run_bucket::<CuboidCuboidDispatcher>(b1.span(), prediction).span();
    let mut r2 = run_bucket::<CapsuleCapsuleDispatcher>(b2.span(), prediction).span();
    let mut r3 = run_bucket::<BallConvexDispatcher>(b3.span(), prediction).span();
    let mut r4 = run_bucket::<ConvexBallDispatcher>(b4.span(), prediction).span();
    let mut r5 = run_bucket::<OtherDispatcher>(b5.span(), prediction).span();
    let mut current = array![];
    let mut transitions = array![];
    let mut next = 0;
    while next != rank {
        let done = if next_is(r0, next) {
            *r0.pop_front().unwrap()
        } else if next_is(r1, next) {
            *r1.pop_front().unwrap()
        } else if next_is(r2, next) {
            *r2.pop_front().unwrap()
        } else if next_is(r3, next) {
            *r3.pop_front().unwrap()
        } else if next_is(r4, next) {
            *r4.pop_front().unwrap()
        } else {
            *r5.pop_front().unwrap()
        };
        current.append(done.pair);
        if let Some(event) = done.event {
            transitions.append(event);
        }
        next += 1;
    }
    let mut events = dropped_events(carry.finish(), ref colliders);
    events.append_span(transitions.span());
    narrow_phase.pairs = current;
    events
}

#[inline(always)]
fn next_is(bucket: Span<PairDone>, rank: u32) -> bool {
    match bucket.get(0) {
        Some(done) => done.unbox().rank == @rank,
        None => false,
    }
}

/// `process_pair::<D>` over one bucket: a loop of its own, so that each iteration is charged
/// the generator of `D` only.
#[inline(never)]
fn run_bucket<impl D: ContactDispatcher>(
    jobs: Span<PairJob>, prediction: Fixed,
) -> Array<PairDone> {
    let mut out = array![];
    for job in jobs {
        let (pair, event) = process_pair::<D>(prediction, *job.co1, *job.co2, *job.previous);
        out.append(PairDone { rank: *job.rank, pair, event });
    }
    out
}

/// Solver stage handing every manifold (touching or not) to `solve_island`, zipped back.
pub fn solve_all_manifolds(
    gravity: Vec2,
    params: IntegrationParameters,
    ref bodies: RigidBodySet,
    ref narrow_phase: NarrowPhase,
    ref impulse_joints: ImpulseJointSet,
) {
    let mut manifolds = array![];
    for pair in narrow_phase.pairs.span() {
        manifolds.append(*pair.manifold);
    }
    let joint_entries = impulse_joints.to_array();
    let mut joints = joint_values(joint_entries.span());
    let (mut store, _) = SolverBodyStoreTrait::from_bodies(ref bodies, gravity, params);
    solve_island(params, ref store, ref manifolds, ref joints);
    store.to_bodies(ref bodies);
    let mut solved = manifolds.span();
    let mut out = array![];
    for pair in narrow_phase.pairs.span() {
        let mut pair = *pair;
        pair.manifold = *solved.pop_front().unwrap();
        out.append(pair);
    }
    narrow_phase.pairs = out;
    write_joints(joint_entries.span(), joints.span(), ref impulse_joints);
}

/// User changes through DD's `propagate_modified_body_positions_to_colliders` (one more body
/// scan, raises collider `POSITION` flags that a third pass clears).
pub fn handle_user_changes_propagate(
    ref bodies: RigidBodySet, ref colliders: ColliderSet, pairs: Span<ContactPair>,
) {
    let mut touched = array![];
    let mut fresh = array![];
    for (handle, collider) in colliders.iter() {
        if !collider.changes.is_empty() {
            collider_changes(handle, collider, ref bodies, ref colliders, ref touched, ref fresh);
        }
    }
    bodies.propagate_modified_body_positions_to_colliders(ref colliders);
    for (handle, body) in bodies.iter() {
        if !body.changes.is_empty() {
            let mut body = body;
            let changes = body.changes;
            if changes.intersects(TOUCHING_CHANGES)
                && super::user_changes::has_known_collider(body.colliders, fresh.span()) {
                body.wake_up(true);
                touched.append_span(body.colliders);
            }
            if changes.contains(POSITION) || changes.contains(COLLIDERS) {
                body
                    .mprops = body
                    .mprops
                    .update_world_mass_properties(body.body_type, body.pos.position);
            }
            if changes.intersects(LOCAL_MASS_PROPERTIES | COLLIDERS) {
                recompute_mass_properties_from_colliders(ref body, ref colliders);
            }
            body.changes = RigidBodyChangesTrait::empty();
            let _ = bodies.set(handle, body);
        }
    }
    for (handle, collider) in colliders.iter() {
        if !collider.changes.is_empty() {
            let mut collider = collider;
            collider.changes = ColliderChangesTrait::empty();
            let _ = colliders.set(handle, collider);
        }
    }
    if !touched.is_empty() && !pairs.is_empty() {
        let _ = wake_touched_partners(touched.span(), pairs, ref bodies, ref colliders);
    }
}

/// The body changes whose colliders the wake-up pass touches (as `pipeline::user_changes`).
const TOUCHING_CHANGES: RigidBodyChanges = RigidBodyChanges { bits: 0xba };

/// Stage 1 as shipped, returning whether any flag was set (for the proxy cache).
pub fn handle_user_changes_flagged(
    ref bodies: RigidBodySet, ref colliders: ColliderSet, pairs: Span<ContactPair>,
) -> bool {
    let mut any = false;
    let mut touched = array![];
    let mut fresh = array![];
    for (handle, collider) in colliders.iter() {
        if !collider.changes.is_empty() {
            any = true;
            collider_changes(handle, collider, ref bodies, ref colliders, ref touched, ref fresh);
        }
    }
    for (handle, body) in bodies.iter() {
        if !body.changes.is_empty() {
            any = true;
            let _ = body_changes(
                handle, body, ref bodies, ref colliders, ref touched, fresh.span(),
            );
        }
    }
    if !touched.is_empty() && !pairs.is_empty() {
        let _ = wake_touched_partners(touched.span(), pairs, ref bodies, ref colliders);
    }
    any
}

/// Candidate (b) of the broad phase: the previous step's proxies, reused for static colliders
/// when no user change happened since and the collider at that index is the same handle.
#[derive(Drop)]
pub struct ProxyCache {
    pub proxies: Array<BroadPhaseProxy>,
    pub valid: bool,
}

#[generate_trait]
pub impl ProxyCacheImpl of ProxyCacheTrait {
    fn new() -> ProxyCache {
        ProxyCache { proxies: array![], valid: false }
    }

    /// The proxies of this step (same contract as `ColliderSetTrait::broad_phase_proxies`);
    /// refreshes the cache.
    fn proxies(
        ref self: ProxyCache,
        ref bodies: RigidBodySet,
        ref colliders: ColliderSet,
        prediction: Fixed,
    ) -> Array<BroadPhaseProxy> {
        let out = if self.valid {
            let margin = prediction * HALF;
            let mut cached = self.proxies.span();
            let mut out = array![];
            for (handle, collider) in colliders.iter() {
                let proxy = match cached.pop_front() {
                    Some(proxy) => if *proxy.collider == handle && *proxy.is_static {
                        *proxy
                    } else {
                        fresh_proxy(handle, collider, margin, ref bodies)
                    },
                    None => fresh_proxy(handle, collider, margin, ref bodies),
                };
                out.append(proxy);
            }
            out
        } else {
            colliders.broad_phase_proxies(ref bodies, prediction)
        };
        let mut copy = array![];
        copy.append_span(out.span());
        self.proxies = copy;
        self.valid = true;
        out
    }
}

#[inline(never)]
fn fresh_proxy(
    handle: rapier_core::Handle,
    collider: rapier_dynamics2d::collider::Collider,
    margin: Fixed,
    ref bodies: RigidBodySet,
) -> BroadPhaseProxy {
    let is_static = match collider.parent {
        Some(parent) => match bodies.get(parent.handle) {
            Some(body) => body.is_fixed(),
            None => true,
        },
        None => true,
    };
    BroadPhaseProxy { collider: handle, aabb: collider.compute_aabb().loosened(margin), is_static }
}

/// A whole step with the proxy cache (invalidated by any user change).
pub fn step_with_cache(ref world: World, ref cache: ProxyCache) -> Array<CollisionEvent> {
    if handle_user_changes_flagged(
        ref world.bodies, ref world.colliders, world.narrow_phase.pairs.span(),
    ) {
        cache.valid = false;
    }
    let prediction = world.integration_parameters.prediction_distance();
    let proxies = cache.proxies(ref world.bodies, ref world.colliders, prediction);
    let pairs = find_pairs(proxies.span());
    let events = world
        .narrow_phase
        .compute_contacts::<
            InlineDispatcher,
        >(prediction, ref world.bodies, ref world.colliders, pairs.span());
    solve(
        world.gravity,
        world.integration_parameters,
        ref world.bodies,
        ref world.narrow_phase,
        ref world.impulse_joints,
    );
    advance_to_final_positions(ref world.bodies, ref world.colliders, world.integration_parameters);
    events
}

/// Shape-pair kinds, one per generator reached by `dispatch::contact_manifold`, and one
/// dispatcher per kind that reaches that generator only.
pub mod kind {
    use fixed::Fixed;
    use rapier_dynamics2d::narrow_phase::ContactDispatcher;
    use rapier_geometry2d::contact::ContactManifold;
    use rapier_geometry2d::contact_generators::ball_ball::contact_manifold_ball_ball;
    use rapier_geometry2d::contact_generators::capsule_capsule::contact_manifold_capsule_capsule;
    use rapier_geometry2d::contact_generators::convex_ball::{
        contact_manifold_ball_convex, contact_manifold_convex_ball,
    };
    use rapier_geometry2d::contact_generators::cuboid_cuboid::contact_manifold_cuboid_cuboid;
    use rapier_geometry2d::dispatch::contact_manifold;
    use rapier_geometry2d::shape::Shape;
    use rapier_math::pose2::Pose2;

    /// Ball – ball.
    pub const BALL_BALL: u8 = 0;
    /// Cuboid – cuboid.
    pub const CUBOID_CUBOID: u8 = 1;
    /// Capsule – capsule.
    pub const CAPSULE_CAPSULE: u8 = 2;
    /// Ball first, anything but a ball second.
    pub const BALL_CONVEX: u8 = 3;
    /// Anything but a ball first, ball second.
    pub const CONVEX_BALL: u8 = 4;
    /// Every other pair: the full dispatcher (polygonal pairs, half-spaces, unsupported pairs).
    pub const OTHER: u8 = 5;

    /// The kind of the pair `(shape1, shape2)`, in `dispatch::contact_manifold`'s priority order.
    #[inline(always)]
    pub fn pair_kind(shape1: Shape, shape2: Shape) -> u8 {
        match (shape1, shape2) {
            (Shape::Ball(_), Shape::Ball(_)) => BALL_BALL,
            (Shape::Cuboid(_), Shape::Cuboid(_)) => CUBOID_CUBOID,
            (Shape::Capsule(_), Shape::Capsule(_)) => CAPSULE_CAPSULE,
            (Shape::Ball(_), _) => BALL_CONVEX,
            (_, Shape::Ball(_)) => CONVEX_BALL,
            _ => OTHER,
        }
    }

    /// Ball – ball only; `false` (unsupported) for any other pair.
    pub impl BallBallDispatcher of ContactDispatcher {
        #[inline(always)]
        fn contact_manifold(
            pos12: Pose2,
            shape1: Shape,
            shape2: Shape,
            prediction: Fixed,
            ref manifold: ContactManifold,
        ) -> bool {
            if let (Shape::Ball(ball1), Shape::Ball(ball2)) = (shape1, shape2) {
                contact_manifold_ball_ball(pos12, ball1, ball2, prediction, ref manifold);
                return true;
            }
            false
        }
    }

    /// Cuboid – cuboid only; `false` (unsupported) for any other pair.
    pub impl CuboidCuboidDispatcher of ContactDispatcher {
        #[inline(always)]
        fn contact_manifold(
            pos12: Pose2,
            shape1: Shape,
            shape2: Shape,
            prediction: Fixed,
            ref manifold: ContactManifold,
        ) -> bool {
            if let (Shape::Cuboid(cuboid1), Shape::Cuboid(cuboid2)) = (shape1, shape2) {
                contact_manifold_cuboid_cuboid(pos12, cuboid1, cuboid2, prediction, ref manifold);
                return true;
            }
            false
        }
    }

    /// Capsule – capsule only; `false` (unsupported) for any other pair.
    pub impl CapsuleCapsuleDispatcher of ContactDispatcher {
        #[inline(always)]
        fn contact_manifold(
            pos12: Pose2,
            shape1: Shape,
            shape2: Shape,
            prediction: Fixed,
            ref manifold: ContactManifold,
        ) -> bool {
            if let (Shape::Capsule(capsule1), Shape::Capsule(capsule2)) = (shape1, shape2) {
                contact_manifold_capsule_capsule(
                    pos12, capsule1, capsule2, prediction, ref manifold,
                );
                return true;
            }
            false
        }
    }

    /// Ball first, any shape second; `false` (unsupported) when shape 1 is not a ball.
    pub impl BallConvexDispatcher of ContactDispatcher {
        #[inline(always)]
        fn contact_manifold(
            pos12: Pose2,
            shape1: Shape,
            shape2: Shape,
            prediction: Fixed,
            ref manifold: ContactManifold,
        ) -> bool {
            if let Shape::Ball(ball1) = shape1 {
                contact_manifold_ball_convex(pos12, ball1, shape2, prediction, ref manifold);
                return true;
            }
            false
        }
    }

    /// Any shape first, ball second; `false` (unsupported) when shape 2 is not a ball.
    pub impl ConvexBallDispatcher of ContactDispatcher {
        #[inline(always)]
        fn contact_manifold(
            pos12: Pose2,
            shape1: Shape,
            shape2: Shape,
            prediction: Fixed,
            ref manifold: ContactManifold,
        ) -> bool {
            if let Shape::Ball(ball2) = shape2 {
                contact_manifold_convex_ball(pos12, shape1, ball2, prediction, ref manifold);
                return true;
            }
            false
        }
    }

    /// The full dispatcher, for the pairs of kind [`OTHER`].
    pub impl OtherDispatcher of ContactDispatcher {
        #[inline(always)]
        fn contact_manifold(
            pos12: Pose2,
            shape1: Shape,
            shape2: Shape,
            prediction: Fixed,
            ref manifold: ContactManifold,
        ) -> bool {
            contact_manifold(pos12, shape1, shape2, prediction, ref manifold)
        }
    }
}
