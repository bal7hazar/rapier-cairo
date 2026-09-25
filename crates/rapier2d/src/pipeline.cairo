//! The step pipeline (upstream `pipeline/physics_pipeline/{mod,substep,solve}.rs`,
//! `PhysicsPipeline::step`, and `pipeline/user_changes.rs`), in upstream order:
//!
//! 1. [`handle_user_changes`]: colliders first (a new parent moves the collider; a new shape,
//!    mass, parent or enabled state marks the parent's mass for recomputation), then bodies (a
//!    moved body or a new collider list moves the colliders and refreshes the world mass
//!    properties; a changed collider list or local mass recomputes the mass from the colliders);
//!    every change flag is cleared;
//! 2. collision detection: `ColliderSetTrait::broad_phase_proxies` → `find_pairs` →
//!    `NarrowPhaseTrait::compute_contacts::<DefaultDispatcher>` with the prediction distance
//!    (see [`detect_collisions`]);
//! 3. [`solve`]: the manifolds that have solver contacts, in [`solve_order`] (D8: pairs of two
//!    non-fixed bodies first, then pairs with a fixed body or none, each in pair order), and the
//!    impulse joints in slot order go to `solve_island` over a `SolverBodyStore`; velocities and
//!    `next_position` go back to the bodies, the solved impulses back into the narrow-phase pairs
//!    (next step's warm start) and into the joint set;
//! 4. [`advance_to_final_positions`]: `position ← next_position` for every enabled non-fixed
//!    body, world mass properties refreshed, attached colliders moved;
//! 5. optional normal-force events are collected, then returned alongside collision events.
//!
//! [`step`] runs these stages fused (work package OP), so that each set is walked once before
//! the solver: [`user_changes_snapshot`] is stage 1 plus the collider walk (reused, retaken only
//! when a change flag was raised) and one [`BodyInfo`] per body; [`collision_inputs`] builds the
//! proxies and the narrow-phase scratch (`narrow_phase::pair_colliders`) from them in one walk
//! with no set read (the parent is found at `infos[handle.index]` when the set has no free slot
//! before it, read otherwise); `narrow_phase::compute_contacts_from_scratch` runs on it;
//! [`advance_with_snapshot`] takes the colliders from the snapshot (stages 2–3 write none). The
//! public stage functions stay, with the same results (`fused_alternatives::step_staged`, raw
//! equivalence on scenes and random worlds in `tests`).
//!
//! Stages 3–4 are fused too (work package OI), in [`solve_and_advance`]: only the bodies a
//! touching manifold or an enabled joint references go through the `SolverBodyStore`; every
//! other body (nothing acts on it) is solved alone by `FreeBodySolverTrait::solve`, the same
//! expressions in the same order, and each moving body is read from the [`user_changes_bodies`]
//! walk and written once. Same results as [`solve`] then [`advance_to_final_positions`] (raw
//! equivalence in `tests`, candidate by candidate and on random worlds).
//!
//! Position-based kinematic velocities are interpolated in the fused user-change stage,
//! after mass updates and before collision detection/islands. Separate-stage callers use
//! `kinematic::interpolate_kinematic_velocities` at the same boundary. Targets stay exact.
//!
//! Every loop runs in ascending slot / pair order; no dict is iterated (determinism, AGENTS.md
//! §2.4). Change flags are only raised by user mutations: stages 3–4 move bodies and colliders
//! without raising them, as upstream's internal motion.
//!
//! Sleeping (work package SL, `islands`, `sleeping`, `user_changes`): stage 1 wakes up the
//! parents and contact partners of the colliders a user change touched; the proxies of sleeping
//! bodies are static and the previous step's dormant pairs (both sides fixed, absent or asleep)
//! bypass the narrow phase; between stages 2 and 3 [`update_islands`] rebuilds the islands
//! (union-find over touching pairs and enabled joints) and applies upstream's wake-up and sleep
//! rules; the solver takes the active pairs only, sleeping bodies enter the store as immovable
//! copies when a joint or a pair still references them and are neither advanced nor written;
//! the sleep timer of every moved body is updated in the position update
//! (`islands::update_sleep_timer`).
//!
//! Deviations from upstream: one step = one CCD substep (CCD is deferred); islands are rebuilt
//! every step (upstream persists them, see `islands`); no user hooks or sensor events,
//! a body whose enabled state changes does not propagate it
//! to its colliders (disable the colliders).
//!

use core::dict::{Felt252Dict, Felt252DictTrait};
use fixed::{Fixed, HALF};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::collider::{ActiveEventsTrait, ColliderChangesTrait};
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_core::rigid_body::{
    RigidBodyChangesTrait, RigidBodyDominance, RigidBodyDominanceTrait, RigidBodyType,
};
use rapier_dynamics2d::collider::{Collider, ColliderTrait};
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::events::{CollisionEvent, ContactForceEvent};
use rapier_dynamics2d::joint::{ImpulseJoint, ImpulseJointSet, ImpulseJointSetTrait, JointEnabled};
use rapier_dynamics2d::narrow_phase::{
    ContactPair, NarrowPhase, PairCollider, compute_contacts_from_scratch,
};
use rapier_dynamics2d::rigid_body::RigidBodyMassPropsTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet, RigidBodySetTrait};
use rapier_dynamics2d::solver::body_store::SolverBodyStoreTrait;
use rapier_dynamics2d::solver::island::{FreeBodySolverTrait, solve_island};
use rapier_geometry2d::aabb::AabbTrait;
use rapier_geometry2d::broad_phase::{BroadPhaseProxy, find_pairs};
use rapier_geometry2d::shape::ShapeTrait;
use crate::dispatcher::DefaultDispatcher;
use crate::world::World;

#[cfg(test)]
pub(crate) mod alternatives;
#[cfg(test)]
mod benches;
#[cfg(test)]
pub(crate) mod fixtures;

pub mod force_events;
use force_events::{CollisionOnly, StepOutput, WithForces};
#[cfg(test)]
pub(crate) mod fused_alternatives;
pub mod islands;
pub mod kinematic;
#[cfg(test)]
pub(crate) mod narrow_alternatives;
#[cfg(test)]
mod narrow_benches;
#[cfg(test)]
mod narrow_tests;
mod ordering;
pub mod sleeping;
#[cfg(test)]
pub(crate) mod solve_alternatives;
#[cfg(test)]
mod solve_benches;
#[cfg(test)]
mod tests;
mod user_changes;
pub use islands::{SleepCensus, SleepCensusTrait, update_islands};
pub(crate) use ordering::{dormant_of, fixed_last_flag, link_status};
pub use ordering::{scatter_touching, scatter_touching_split, solve_order, touching_manifolds};
pub use sleeping::{any_sleeping, merge_pairs, split_dormant};
mod staged;
pub use staged::{advance_to_final_positions, detect_collisions, solve};
pub(crate) use user_changes::{body_changes, collider_changes};
pub use user_changes::{handle_user_changes, recompute_mass_properties_from_colliders};


/// One step of `world` (see the module documentation for the stages); returns the collision
/// events.
///
/// # Panics
/// As the stages: fixed-point overflow, zero solver iterations, negative parameters.
pub fn step(ref world: World) -> Array<CollisionEvent> {
    step_internal::<Array<CollisionEvent>, CollisionOnly>(ref world)
}

/// Same step, also returning post-solver normal-force events in ascending pair order.
/// Empty force array and no post-solver scan when no collider enables force events.
/// Panics as `step`.
pub fn step_with_force_events(
    ref world: World,
) -> (Array<CollisionEvent>, Array<ContactForceEvent>) {
    step_internal::<(Array<CollisionEvent>, Array<ContactForceEvent>), WithForces>(ref world)
}

fn step_internal<T, impl Output: StepOutput<T>, +Drop<T>>(ref world: World) -> T {
    let (snapshot, infos, entries, census) = user_changes_bodies_for_step(
        ref world.bodies,
        ref world.colliders,
        world.narrow_phase.pairs.span(),
        Some(world.integration_parameters.dt),
    );
    let prediction = world.integration_parameters.prediction_distance();
    let (proxies, scratch, sleeping, force_events) = collision_inputs_with_events(
        snapshot, infos, ref world.bodies, prediction,
    );
    let mut dormant = array![];
    if sleeping {
        let (active, asleep) = split_dormant(world.narrow_phase.pairs.span(), entries);
        world.narrow_phase.pairs = active;
        dormant = asleep;
    }
    let pairs = find_pairs(proxies.span());
    let events = compute_contacts_from_scratch::<
        DefaultDispatcher,
    >(ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders);
    let joint_entries = world.impulse_joints.to_array();
    let (entries, sleeping, woken) = update_islands(
        ref world.bodies,
        world.narrow_phase.pairs.span(),
        dormant.span(),
        joint_entries.span(),
        entries,
        census,
    );
    if woken && !dormant.is_empty() {
        // The dormant pairs of the woken bodies join the solver input of this step.
        let (revived, asleep) = split_dormant(dormant.span(), entries);
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), revived.span());
        dormant = asleep;
    }
    solve_and_advance_sleeping(
        world.gravity,
        world.integration_parameters,
        ref world.bodies,
        ref world.colliders,
        ref world.narrow_phase,
        ref world.impulse_joints,
        entries,
        snapshot,
        joint_entries.span(),
        sleeping,
    );
    // Dormant manifolds retain old impulses; emit only from this step's active pairs.
    let output = Output::finish(
        events,
        force_events,
        world.integration_parameters.dt,
        ref world.narrow_phase,
        ref world.colliders,
    );
    if !dormant.is_empty() {
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), dormant.span());
    }
    output
}

/// What the collision stages read from a parent body, after the user changes.
#[derive(Copy, Drop, PartialEq, Debug)]
pub struct BodyInfo {
    pub handle: Handle,
    pub body_type: RigidBodyType,
    pub world_com: Vec2,
    /// Effective dominance group (upstream `effective_group`).
    pub dominance: i16,
    /// The body sleeps (SL): its colliders' proxies are static.
    pub sleeping: bool,
}

/// The [`BodyInfo`] of a missing parent or of no parent: a fixed body at the origin.
#[inline(always)]
fn no_body_info() -> (RigidBodyType, Vec2, i16, bool) {
    let dominance: RigidBodyDominance = Default::default();
    (
        RigidBodyType::Fixed,
        Default::default(),
        dominance.effective_group(RigidBodyType::Fixed),
        false,
    )
}

/// The `(body_type, world_com, dominance, sleeping)` of the body `handle`: `infos[handle.index]`
/// when that entry has the handle (always the case in a set without free slot), a set read
/// otherwise. The read is inlined: in the loop body it is only paid when reached
/// (`fused_alternatives::collision_inputs_outlined_fallback` puts it behind a call).
#[inline(always)]
fn body_info(
    infos: Span<BodyInfo>, handle: Handle, ref bodies: RigidBodySet,
) -> (RigidBodyType, Vec2, i16, bool) {
    if let Some(info) = infos.get(handle.index) {
        let info = *info.unbox();
        if info.handle == handle {
            return (info.body_type, info.world_com, info.dominance, info.sleeping);
        }
    }
    match bodies.get(handle) {
        Some(body) => (
            body.body_type,
            body.mprops.world_com,
            body.dominance.effective_group(body.body_type),
            body.activation.sleeping,
        ),
        None => no_body_info(),
    }
}

/// One [`BodyInfo`] per entry of `entries` (ascending slot), and their [`SleepCensus`].
pub fn body_infos(entries: Span<(Handle, RigidBody)>) -> (Array<BodyInfo>, SleepCensus) {
    let mut infos = array![];
    let mut census: SleepCensus = Default::default();
    for (handle, body) in entries {
        census.count(body);
        infos
            .append(
                BodyInfo {
                    handle: *handle,
                    body_type: *body.body_type,
                    world_com: *body.mprops.world_com,
                    dominance: body.dominance.effective_group(*body.body_type),
                    sleeping: *body.activation.sleeping,
                },
            );
    }
    (infos, census)
}

/// The collider `handle`: `snapshot[handle.index]` when that entry has the handle, a set read
/// otherwise.
#[inline(always)]
fn snapshot_collider(
    snapshot: Span<(Handle, Collider)>, handle: Handle, ref colliders: ColliderSet,
) -> Option<Collider> {
    if let Some(entry) = snapshot.get(handle.index) {
        let (h, collider) = *entry.unbox();
        if h == handle {
            return Some(collider);
        }
    }
    colliders.get(handle)
}

/// [`handle_user_changes`] that also returns what the rest of the step reads from the sets:
/// every `(handle, collider)` after the changes in ascending slot (as `ColliderSetTrait::iter`)
/// and one [`BodyInfo`] per body in ascending slot. The collider walk is reused when no change
/// flag was raised, taken again otherwise. [`user_changes_bodies`] without the bodies. `pairs`
/// are the previous step's contact pairs (wake-up pass, `sleeping::wake_touched_partners`).
pub fn user_changes_snapshot(
    ref bodies: RigidBodySet, ref colliders: ColliderSet, pairs: Span<ContactPair>,
) -> (Span<(Handle, Collider)>, Span<BodyInfo>) {
    let (snapshot, infos, _, _) = user_changes_bodies(ref bodies, ref colliders, pairs);
    (snapshot, infos)
}

/// [`user_changes_snapshot`] that also returns every `(handle, body)` after the changes in
/// ascending slot (as `RigidBodySetTrait::iter`) and the bodies' [`SleepCensus`] (the island
/// stage's fast check); both walks are reused when no change flag was raised, taken again
/// otherwise (and after a wake-up of the pass over `pairs`).
pub fn user_changes_bodies(
    ref bodies: RigidBodySet, ref colliders: ColliderSet, pairs: Span<ContactPair>,
) -> (Span<(Handle, Collider)>, Span<BodyInfo>, Span<(Handle, RigidBody)>, SleepCensus) {
    user_changes_bodies_for_step(ref bodies, ref colliders, pairs, None)
}

fn user_changes_bodies_for_step(
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    pairs: Span<ContactPair>,
    dt: Option<Fixed>,
) -> (Span<(Handle, Collider)>, Span<BodyInfo>, Span<(Handle, RigidBody)>, SleepCensus) {
    let mut snapshot = colliders.iter().span();
    let mut dirty = false;
    let mut touched = array![];
    for (handle, collider) in snapshot {
        if !collider.changes.is_empty() {
            collider_changes(*handle, *collider, ref bodies, ref colliders, ref touched);
            dirty = true;
        }
    }
    let mut entries = bodies.iter().span();
    let mut bodies_dirty = false;
    let mut infos = array![];
    let mut census: SleepCensus = Default::default();
    let mut has_kinematic = false;
    for (handle, body) in entries {
        let (body_type, world_com, dominance, sleeping) = if body.changes.is_empty() {
            census.count(body);
            (
                *body.body_type,
                *body.mprops.world_com,
                body.dominance.effective_group(*body.body_type),
                *body.activation.sleeping,
            )
        } else {
            bodies_dirty = true;
            let body = body_changes(*handle, *body, ref bodies, ref colliders, ref touched);
            census.count(@body);
            (
                body.body_type,
                body.mprops.world_com,
                body.dominance.effective_group(body.body_type),
                body.activation.sleeping,
            )
        };
        if body_type == RigidBodyType::KinematicPositionBased {
            has_kinematic = true;
        }
        infos.append(BodyInfo { handle: *handle, body_type, world_com, dominance, sleeping });
    }
    // Only a world with position-based bodies pays for the extra walk. The ordinary
    // body's hot path keeps snapshot field reads and no per-body loop dispatch.
    while has_kinematic {
        if let Some(dt) = dt {
            kinematic::prepare_existing(ref bodies, entries, dt);
            bodies_dirty = true;
        }
        has_kinematic = false;
    }
    if !touched.is_empty()
        && !pairs.is_empty()
        && sleeping::wake_touched_partners(touched.span(), pairs, ref bodies, ref colliders) {
        bodies_dirty = true;
        entries = bodies.iter().span();
        let (fresh, recount) = body_infos(entries);
        infos = fresh;
        census = recount;
    } else if bodies_dirty {
        entries = bodies.iter().span();
    }
    if dirty || bodies_dirty {
        snapshot = colliders.iter().span();
    }
    (snapshot, infos.span(), entries, census)
}

/// The broad-phase proxies (as `ColliderSetTrait::broad_phase_proxies`, plus SL: a sleeping
/// body's proxies are static) and the narrow-phase scratch (as `narrow_phase::pair_colliders`)
/// of `snapshot`, in one walk without set read (except for a parent absent from `infos`' dense
/// layout). [`collision_inputs_sleeping`] without its flag.
pub fn collision_inputs(
    snapshot: Span<(Handle, Collider)>,
    infos: Span<BodyInfo>,
    ref bodies: RigidBodySet,
    prediction: Fixed,
) -> (Array<BroadPhaseProxy>, Span<PairCollider>) {
    let (proxies, scratch, _) = collision_inputs_sleeping(snapshot, infos, ref bodies, prediction);
    (proxies, scratch)
}

/// [`collision_inputs`] that also tells whether a collider belongs to a sleeping body (then the
/// previous pairs must be split, `sleeping::split_dormant`).
pub fn collision_inputs_sleeping(
    snapshot: Span<(Handle, Collider)>,
    infos: Span<BodyInfo>,
    ref bodies: RigidBodySet,
    prediction: Fixed,
) -> (Array<BroadPhaseProxy>, Span<PairCollider>, bool) {
    let (proxies, scratch, sleeping, _) = collision_inputs_with_events(
        snapshot, infos, ref bodies, prediction,
    );
    (proxies, scratch, sleeping)
}

/// Step-only census: the last flag enables post-solver force-event collection.
pub(crate) fn collision_inputs_with_events(
    snapshot: Span<(Handle, Collider)>,
    infos: Span<BodyInfo>,
    ref bodies: RigidBodySet,
    prediction: Fixed,
) -> (Array<BroadPhaseProxy>, Span<PairCollider>, bool, bool) {
    let margin = prediction * HALF;
    let mut proxies = array![];
    let mut scratch = array![];
    let mut any_sleeping = false;
    let mut force_events = false;
    for (handle, collider) in snapshot {
        let collider = *collider;
        if collider.flags.active_events.bits != 0
            && collider
                .flags
                .active_events
                .contains(rapier_core::collider::events::CONTACT_FORCE_EVENTS) {
            force_events = true;
        }
        let body = collider.parent();
        let (body_type, world_com, dominance, sleeping) = match body {
            Some(parent) => body_info(infos, parent, ref bodies),
            None => no_body_info(),
        };
        if sleeping {
            any_sleeping = true;
        }
        let pose = collider.pos.pose;
        proxies
            .append(
                BroadPhaseProxy {
                    collider: *handle,
                    aabb: collider.shape.compute_aabb(pose).loosened(margin),
                    is_static: body_type == RigidBodyType::Fixed || sleeping,
                },
            );
        scratch
            .append(
                PairCollider {
                    handle: *handle,
                    solid: collider.is_enabled() && !collider.is_sensor(),
                    shape: collider.shape,
                    pose,
                    friction: collider.material.friction,
                    restitution: collider.material.restitution,
                    friction_combine_rule: collider.material.friction_combine_rule,
                    restitution_combine_rule: collider.material.restitution_combine_rule,
                    active_collision_types: collider.flags.active_collision_types,
                    collision_groups: collider.flags.collision_groups,
                    solver_groups: collider.flags.solver_groups,
                    active_events: collider.flags.active_events,
                    one_way: collider.one_way,
                    body,
                    body_type,
                    world_com,
                    dominance,
                },
            );
    }
    (proxies, scratch.span(), any_sleeping, force_events)
}


/// [`advance_to_final_positions`] reading the colliders from `snapshot` (the colliders as
/// [`user_changes_snapshot`] left them; stages 2–3 do not write colliders).
pub fn advance_with_snapshot(
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    snapshot: Span<(Handle, Collider)>,
    params: IntegrationParameters,
) {
    for (handle, body) in bodies.iter().span() {
        if moving(body) {
            advance_body_with_snapshot(*handle, *body, ref bodies, ref colliders, snapshot, params);
        }
    }
}

/// The step moves `body`: enabled, not fixed, awake.
#[inline(always)]
pub(crate) fn moving(body: @RigidBody) -> bool {
    *body.enabled && *body.body_type != RigidBodyType::Fixed && !*body.activation.sleeping
}

/// `body` with its `enabled` flag cleared: how a sleeping body enters the solver store when a
/// pair or a joint still references it (immovable, zero velocity), see the module documentation.
#[inline(always)]
pub(crate) fn immovable(body: RigidBody) -> RigidBody {
    let mut body = body;
    body.enabled = false;
    body
}

/// The position update of one moving body (`position ← next_position`, world mass properties,
/// attached colliders moved) after its sleep timer (`islands::update_sleep_timer`, on the
/// displacement of the step).
#[inline(always)]
pub(crate) fn advance_body_with_snapshot(
    handle: Handle,
    body: RigidBody,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    snapshot: Span<(Handle, Collider)>,
    params: IntegrationParameters,
) {
    let mut body = body;
    let previous = body.pos.position;
    body.pos.position = body.pos.next_position;
    islands::update_sleep_timer(ref body, previous, params);
    body.mprops = body.mprops.update_world_mass_properties(body.body_type, body.pos.position);
    let _ = bodies.set(handle, body);
    for co_handle in body.colliders {
        if let Some(mut collider) = snapshot_collider(snapshot, *co_handle, ref colliders) {
            if let Some(parent) = collider.parent {
                collider.pos.pose = body.pos.position * parent.pos_wrt_parent;
                let _ = colliders.set(*co_handle, collider);
            }
        }
    }
}

/// The joints of `entries` that are not dormant (`ordering::dormant_of`: both bodies fixed,
/// absent or asleep, one asleep): the solver input when a body sleeps.
pub(crate) fn active_joints(
    joints: Span<(Handle, ImpulseJoint)>, entries: Span<(Handle, RigidBody)>,
) -> Array<(Handle, ImpulseJoint)> {
    let mut out = array![];
    for entry in joints {
        let (_, joint) = entry;
        let (s1, s2) = link_status(entries, Some(*joint.body1), Some(*joint.body2));
        if !dormant_of(s1, s2) {
            out.append(*entry);
        }
    }
    out
}

/// Stages 3 and 4 fused (work package OI), with the same results as [`solve`] then
/// [`advance_with_snapshot`]: only the bodies a touching manifold or an enabled joint references
/// enter the `SolverBodyStore`; every other body is solved alone by `FreeBodySolverTrait::solve`
/// (bit-identical: nothing else acts on it), and each moving body is written once, after its
/// velocities, `next_position`, position, world mass properties and collider poses.
/// `entries` and `snapshot` are the bodies and colliders as [`user_changes_bodies`] left them.
/// [`solve_and_advance_sleeping`] on the whole pair list (split around the solver when a body
/// sleeps).
pub fn solve_and_advance(
    gravity: Vec2,
    params: IntegrationParameters,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    ref narrow_phase: NarrowPhase,
    ref impulse_joints: ImpulseJointSet,
    entries: Span<(Handle, RigidBody)>,
    snapshot: Span<(Handle, Collider)>,
) {
    let sleeping = any_sleeping(entries);
    let mut dormant = array![];
    if sleeping {
        let (active, asleep) = split_dormant(narrow_phase.pairs.span(), entries);
        narrow_phase.pairs = active;
        dormant = asleep;
    }
    let joint_entries = impulse_joints.to_array();
    solve_and_advance_sleeping(
        gravity,
        params,
        ref bodies,
        ref colliders,
        ref narrow_phase,
        ref impulse_joints,
        entries,
        snapshot,
        joint_entries.span(),
        sleeping,
    );
    if !dormant.is_empty() {
        narrow_phase.pairs = merge_pairs(narrow_phase.pairs.span(), dormant.span());
    }
}

/// [`solve_and_advance`] on the active pairs of `narrow_phase` (the dormant pairs of sleeping
/// bodies split out by the caller) and the given joint entries. `sleeping` tells whether any
/// body of `entries` sleeps (after [`update_islands`]): then dormant joints are left out, the
/// sleeping bodies a constraint references enter the store as immovable copies, and sleeping
/// bodies are neither advanced nor written; with `false` the stage is the pre-SL one.
pub fn solve_and_advance_sleeping(
    gravity: Vec2,
    params: IntegrationParameters,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    ref narrow_phase: NarrowPhase,
    ref impulse_joints: ImpulseJointSet,
    entries: Span<(Handle, RigidBody)>,
    snapshot: Span<(Handle, Collider)>,
    joint_entries: Span<(Handle, ImpulseJoint)>,
    sleeping: bool,
) {
    let mut constrained: Felt252Dict<bool> = Default::default();
    let mut first = array![];
    let mut last = array![];
    let mut flags = array![];
    for pair in narrow_phase.pairs.span() {
        if *pair.manifold.data.num_solver_contacts != 0 {
            let manifold = *pair.manifold;
            if let Some(h) = manifold.data.rigid_body1 {
                constrained.insert(h.into(), true);
            }
            if let Some(h) = manifold.data.rigid_body2 {
                constrained.insert(h.into(), true);
            }
            let fixed_last = fixed_last_flag(
                entries, manifold.data.rigid_body1, manifold.data.rigid_body2,
            );
            flags.append(fixed_last);
            if fixed_last {
                last.append(manifold);
            } else {
                first.append(manifold);
            }
        }
    }
    let n_first = first.len();
    first.append_span(last.span());
    let mut manifolds = first;
    let joint_entries = if sleeping {
        active_joints(joint_entries, entries).span()
    } else {
        joint_entries
    };
    let mut joints = joint_values(joint_entries);
    for joint in joints.span() {
        if *joint.data.enabled == JointEnabled::Enabled {
            constrained.insert((*joint.body1).into(), true);
            constrained.insert((*joint.body2).into(), true);
        }
    }
    let any = !manifolds.is_empty() || !joints.is_empty();
    let mut members = array![];
    let mut has_free = false;
    if any {
        for entry in entries {
            let (handle, body) = entry;
            if constrained.get((*handle).into()) {
                if sleeping && *body.activation.sleeping {
                    members.append((*handle, immovable(*body)));
                } else {
                    members.append(*entry);
                }
            } else if moving(body) {
                has_free = true;
            }
        }
    }
    let mut store = SolverBodyStoreTrait::from_entries(members.span(), gravity, params);
    // With no manifold and no joint, `solve_island` would only validate the parameters, which
    // `FreeBodySolverTrait::new` does with the same panics; with constraints it is only built
    // when a moving body is free.
    let free = if !any || has_free {
        FreeBodySolverTrait::new(params, gravity)
    } else {
        Default::default()
    };
    if any {
        solve_island(params, ref store, ref manifolds, ref joints);
        if !manifolds.is_empty() {
            narrow_phase
                .pairs =
                    scatter_touching_split(
                        narrow_phase.pairs.span(), manifolds.span(), flags.span(), n_first,
                    );
        }
        write_joints(joint_entries, joints.span(), ref impulse_joints);
    }
    let mut dense: u32 = 0;
    for (handle, body) in entries {
        let member = any && constrained.get((*handle).into());
        if moving(body) {
            let body = if member {
                let mut body = *body;
                store.write_body(dense, ref body);
                body
            } else {
                free.solve(*handle, *body)
            };
            advance_body_with_snapshot(*handle, body, ref bodies, ref colliders, snapshot, params);
        }
        if member {
            dense += 1;
        }
    }
}

pub(crate) fn joint_values(entries: Span<(Handle, ImpulseJoint)>) -> Array<ImpulseJoint> {
    let mut out = array![];
    for (_, joint) in entries {
        out.append(*joint);
    }
    out
}

pub(crate) fn write_joints(
    entries: Span<(Handle, ImpulseJoint)>,
    solved: Span<ImpulseJoint>,
    ref impulse_joints: ImpulseJointSet,
) {
    let mut solved = solved;
    for (handle, _) in entries {
        let _ = impulse_joints.set(*handle, *solved.pop_front().unwrap());
    }
}

#[cfg(test)]
mod joint_glue_probes {
    use rapier_dynamics2d::joint::RevoluteJointBuilderTrait;
    use rapier_testing::opaque;
    use super::*;
    #[inline(always)]
    fn probe(stage: u8) {
        let mut set = ImpulseJointSetTrait::new();
        let mut i = 0;
        while i != 3 {
            let _ = set
                .insert(
                    opaque(Handle { index: i, generation: 1 }),
                    opaque(Handle { index: i + 1, generation: 1 }),
                    opaque(RevoluteJointBuilderTrait::new().build()),
                );
            i += 1;
        }
        let entries = set.to_array();
        if stage != 0 {
            let values = joint_values(opaque(entries.span()));
            if stage == 2 {
                write_joints(opaque(entries.span()), opaque(values.span()), ref set);
            }
            let _ = opaque(values.span());
        }
        let _ = opaque(set.len());
    }
    #[test]
    fn gas_baseline() {
        probe(0);
    }
    #[test]
    fn gas_joint_values3() {
        probe(1);
    }
    #[test]
    fn gas_write_joints3() {
        probe(2);
    }
}
