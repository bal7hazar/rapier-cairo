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
//! 5. the collision events of stage 2 are returned.
//!
//! [`step`] runs these stages fused (work package OP), so that each set is walked once before
//! the solver: [`user_changes_snapshot`] is stage 1 plus the collider walk (reused, retaken only
//! when a change flag was raised) and one [`BodyInfo`] per body; [`collision_inputs`] builds the
//! proxies and the narrow-phase scratch (`narrow_phase::pair_colliders`) from them in one walk
//! with no set read (the parent is found at `infos[handle.index]` when the set has no free slot
//! before it, read otherwise); [`contacts_from_scratch`] is `compute_contacts` on that scratch;
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
//! Every loop runs in ascending slot / pair order; no dict is iterated (determinism, AGENTS.md
//! §2.4). Change flags are only raised by user mutations: stages 3–4 move bodies and colliders
//! without raising them, as upstream's internal motion.
//!
//! Deviations from upstream: one step = one CCD substep (CCD is deferred); no islands, sleeping,
//! hooks, contact-force or sensor events, kinematic velocity interpolation (a position-based
//! kinematic body moves to its `next_position` with the velocity the user gave it); a body whose
//! enabled state changes does not propagate it to its colliders (disable the colliders).
//!
//! # Cost
//!
//! Stages of one step, Sierra gas | Cairo steps, from `benches` (cumulative probes,
//! differences), staged on `main` before OP → fused:
//!
//! * `free_fall(32)`: user changes 1 797 620 → 1 883 490, broad phase 5 138 200 → 4 070 100,
//!   narrow phase 1 652 840 → 26 600, solver 13 625 704 (unchanged), position update
//!   4 010 344 → 3 665 624;
//! * `BOX_STACK3` settled: user changes 261 540 → 269 110, broad phase 337 680 → 222 210,
//!   narrow phase 2 440 235 → 2 219 595, solver 13 416 918 (unchanged), position update
//!   418 076 → 386 276.
//!
//! Whole step (solver after OS): `free_fall(32)` 26 224 708 | 235 363 → 23 270 228 | 210 191,
//! `free_fall(8)` 6 299 812 | 56 299 → 5 540 612 | 49 799, `BOX_STACK3` 16 874 449 | 141 347 →
//! 16 512 819 | 138 088. What is left per falling body at 32 bodies: solver 426k (`from_bodies`,
//! `solve_island`, `to_bodies`), `find_pairs` 97k (O(n²) pair tests), position update 115k
//! (body write 22k, world mass 17k, collider pose and write), user changes 59k (the two walks).
//!
//! OI (`solve_benches`, stages 3–4 of one step, Sierra gas | Cairo steps): `free_fall(32)`
//! 17 284 958 | 153 262 → 10 080 648 | 84 638 (540k → 315k per falling body), `BOX_STACK3`
//! 13 800 934 | 124 124 → 13 696 574 | 123 072, `PENDULUM` 4 644 766 | 37 413 → 4 602 816 |
//! 36 990, one resting ball 4 193 914 | 34 792 → 4 183 474 | 34 662. Whole `free_fall(32)` step
//! 23 281 428 | 210 294 → 16 072 618 | 141 708 (P3 `gas_scenes`).
//!
//! Candidates (`alternatives`, `fused_alternatives` and `solve_alternatives`, equivalence in
//! `tests`):
//! * stages 3–4 fused (shipped) vs staged (`solve` then `advance_with_snapshot`): above; vs the
//!   free-body arm behind a one-iteration `while` (`solve_and_advance_metered`), vs marking the
//!   constrained bodies in a second walk over `touching_manifolds`
//!   (`solve_and_advance_separate_marking`), vs matching the next member handle instead of a
//!   second dict read (`solve_and_advance_member_handles`), vs building the free-body constants
//!   on the first free body (`solve_and_advance_lazy`), vs calling `solve_island` with nothing
//!   to solve (`solve_and_advance_island_always`), gas on `free_fall(32)` / `BOX_STACK3` /
//!   resting ball: 10 080 648 / 13 696 574 / 4 183 474 shipped vs 10 679 588 / 13 730 814 /
//!   4 194 954, 10 090 038 / 13 730 174 / 4 200 934, 10 092 608 / 13 700 864 / 4 186 474,
//!   10 388 488 / 13 719 024 / 4 192 744, 10 286 388 / 13 696 964 / 4 183 864 (Cairo steps
//!   rank the same);
//! * fused step (shipped) vs the staged stage functions (`step_staged`): above;
//! * parent read of a sparse set inlined in the proxy loop (shipped) vs behind a call
//!   (`collision_inputs_outlined_fallback`), vs field-by-field reads through the snapshot
//!   instead of one collider copy (`collision_inputs_field_reads`): broad phase 4 070 100 vs
//!   4 154 280 / 4 095 700 on `free_fall(32)`, 222 210 vs 232 470 / 225 410 on `BOX_STACK3`;
//! * per-body position update inlined in the body loop (shipped) vs `#[inline(never)]`
//!   (`advance_with_snapshot_outlined`): 3 665 624 vs 3 918 324 on `free_fall(32)`, 386 276 vs
//!   409 776 on `BOX_STACK3`;
//! * proxy AABB: `ShapeTrait::compute_aabb` inlined (shipped, OP) vs out of line: see
//!   `rapier_geometry2d::shape`; the staged broad phase of `free_fall(32)` drops from 5 138 200 to
//!   4 609 880 with it;
//! * user changes: one fused pass per set (shipped) vs DD's
//!   `propagate_modified_body_positions_to_colliders` plus a flag-clearing pass: 261 540 vs
//!   500 162 on a settled stack, 1 461 064 vs 1 777 058 on the first step;
//! * broad phase: rebuilding every proxy (shipped) vs reusing the static proxies of the previous
//!   step while no user change happened: 337 680 vs 353 770 on `BOX_STACK3` (1 static collider
//!   of 4), 816 370 vs 584 350 with 8 static platforms; the cache would be persisted state (D9)
//!   and loses on the reference scene, so it is not shipped;
//! * solver input: touching manifolds only (shipped) vs every pair: 5 156 516 vs 6 768 856 with
//!   one non-touching pair (1.6M per inert manifold), 14 444 688 vs 14 424 288 when all touch;
//! * narrow phase: see `crate::dispatcher` (shipped dispatcher) and, rejected, a per-kind
//!   `match` in the pair loop (`compute_contacts_by_kind`), per-kind bucket loops
//!   (`compute_contacts_bucketed`) and a one-iteration loop around each per-kind `process_pair`
//!   (`compute_contacts_metered`): 4 ball pairs 3 477 356 / 1 778 206 / 1 651 356 vs 1 486 460
//!   shipped, 4 cuboid pairs 3 477 356 / 3 548 006 / 3 252 676 vs 3 197 940 shipped.

use core::dict::{Felt252Dict, Felt252DictTrait};
use fixed::{Fixed, HALF};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::collider::ColliderChangesTrait;
use rapier_core::collider::changes::{
    ENABLED_OR_DISABLED, LOCAL_MASS_PROPERTIES as CO_LOCAL_MASS_PROPERTIES, PARENT, SHAPE,
};
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_core::rigid_body::changes::{COLLIDERS, LOCAL_MASS_PROPERTIES, POSITION};
use rapier_core::rigid_body::{
    RigidBodyChangesTrait, RigidBodyDominance, RigidBodyDominanceTrait, RigidBodyType,
};
use rapier_dynamics2d::collider::{Collider, ColliderTrait};
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::events::CollisionEvent;
use rapier_dynamics2d::joint::{ImpulseJoint, ImpulseJointSet, ImpulseJointSetTrait, JointEnabled};
use rapier_dynamics2d::narrow_phase::{
    CarryOver, ContactPair, NarrowPhase, NarrowPhaseTrait, PairCollider, SortedMerge,
    dropped_events, process_pair,
};
use rapier_dynamics2d::rigid_body::RigidBodyMassPropsTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet, RigidBodySetTrait};
use rapier_dynamics2d::solver::body_store::SolverBodyStoreTrait;
use rapier_dynamics2d::solver::island::{FreeBodySolverTrait, solve_island};
use rapier_geometry2d::aabb::AabbTrait;
use rapier_geometry2d::broad_phase::{BroadPhaseProxy, find_pairs};
use rapier_geometry2d::contact::ContactManifold;
use rapier_geometry2d::mass::{MassProperties, MassPropertiesTrait};
use rapier_geometry2d::shape::ShapeTrait;
use crate::dispatcher::DefaultDispatcher;
use crate::world::World;

#[cfg(test)]
pub(crate) mod alternatives;
#[cfg(test)]
mod benches;
#[cfg(test)]
pub(crate) mod fixtures;
#[cfg(test)]
pub(crate) mod fused_alternatives;
#[cfg(test)]
pub(crate) mod solve_alternatives;
#[cfg(test)]
mod solve_benches;
#[cfg(test)]
mod tests;


/// One step of `world` (see the module documentation for the stages); returns the collision
/// events.
///
/// # Panics
/// As the stages: fixed-point overflow, zero solver iterations, negative parameters.
pub fn step(ref world: World) -> Array<CollisionEvent> {
    let (snapshot, infos, entries) = user_changes_bodies(ref world.bodies, ref world.colliders);
    let prediction = world.integration_parameters.prediction_distance();
    let (proxies, scratch) = collision_inputs(snapshot, infos, ref world.bodies, prediction);
    let pairs = find_pairs(proxies.span());
    let events = contacts_from_scratch(
        ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders,
    );
    solve_and_advance(
        world.gravity,
        world.integration_parameters,
        ref world.bodies,
        ref world.colliders,
        ref world.narrow_phase,
        ref world.impulse_joints,
        entries,
        snapshot,
    );
    events
}

/// What the collision stages read from a parent body, after the user changes.
#[derive(Copy, Drop, PartialEq, Debug)]
pub struct BodyInfo {
    pub handle: Handle,
    pub body_type: RigidBodyType,
    pub world_com: Vec2,
    /// Effective dominance group (upstream `effective_group`).
    pub dominance: i16,
}

/// The [`BodyInfo`] of a missing parent or of no parent: a fixed body at the origin.
#[inline(always)]
fn no_body_info() -> (RigidBodyType, Vec2, i16) {
    let dominance: RigidBodyDominance = Default::default();
    (RigidBodyType::Fixed, Default::default(), dominance.effective_group(RigidBodyType::Fixed))
}

/// The `(body_type, world_com, dominance)` of the body `handle`: `infos[handle.index]` when that
/// entry has the handle (always the case in a set without free slot), a set read otherwise.
/// The read is inlined: in the loop body it is only paid when reached (`fused_alternatives::
/// collision_inputs_outlined_fallback` puts it behind a call).
#[inline(always)]
fn body_info(
    infos: Span<BodyInfo>, handle: Handle, ref bodies: RigidBodySet,
) -> (RigidBodyType, Vec2, i16) {
    if let Some(info) = infos.get(handle.index) {
        let info = *info.unbox();
        if info.handle == handle {
            return (info.body_type, info.world_com, info.dominance);
        }
    }
    match bodies.get(handle) {
        Some(body) => (
            body.body_type, body.mprops.world_com, body.dominance.effective_group(body.body_type),
        ),
        None => no_body_info(),
    }
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
/// flag was raised, taken again otherwise. [`user_changes_bodies`] without the bodies.
pub fn user_changes_snapshot(
    ref bodies: RigidBodySet, ref colliders: ColliderSet,
) -> (Span<(Handle, Collider)>, Span<BodyInfo>) {
    let (snapshot, infos, _) = user_changes_bodies(ref bodies, ref colliders);
    (snapshot, infos)
}

/// [`user_changes_snapshot`] that also returns every `(handle, body)` after the changes in
/// ascending slot (as `RigidBodySetTrait::iter`); both walks are reused when no change flag was
/// raised, taken again otherwise.
pub fn user_changes_bodies(
    ref bodies: RigidBodySet, ref colliders: ColliderSet,
) -> (Span<(Handle, Collider)>, Span<BodyInfo>, Span<(Handle, RigidBody)>) {
    let mut snapshot = colliders.iter().span();
    let mut dirty = false;
    for (handle, collider) in snapshot {
        if !collider.changes.is_empty() {
            collider_changes(*handle, *collider, ref bodies, ref colliders);
            dirty = true;
        }
    }
    let mut entries = bodies.iter().span();
    let mut bodies_dirty = false;
    let mut infos = array![];
    for (handle, body) in entries {
        let (body_type, world_com, dominance) = if body.changes.is_empty() {
            (*body.body_type, *body.mprops.world_com, *body.dominance)
        } else {
            bodies_dirty = true;
            let body = body_changes(*handle, *body, ref bodies, ref colliders);
            (body.body_type, body.mprops.world_com, body.dominance)
        };
        infos
            .append(
                BodyInfo {
                    handle: *handle,
                    body_type,
                    world_com,
                    dominance: dominance.effective_group(body_type),
                },
            );
    }
    if dirty || bodies_dirty {
        snapshot = colliders.iter().span();
    }
    if bodies_dirty {
        entries = bodies.iter().span();
    }
    (snapshot, infos.span(), entries)
}

/// The broad-phase proxies (as `ColliderSetTrait::broad_phase_proxies`) and the narrow-phase
/// scratch (as `narrow_phase::pair_colliders`) of `snapshot`, in one walk without set read
/// (except for a parent absent from `infos`' dense layout).
pub fn collision_inputs(
    snapshot: Span<(Handle, Collider)>,
    infos: Span<BodyInfo>,
    ref bodies: RigidBodySet,
    prediction: Fixed,
) -> (Array<BroadPhaseProxy>, Span<PairCollider>) {
    let margin = prediction * HALF;
    let mut proxies = array![];
    let mut scratch = array![];
    for (handle, collider) in snapshot {
        let collider = *collider;
        let body = collider.parent();
        let (body_type, world_com, dominance) = match body {
            Some(parent) => body_info(infos, parent, ref bodies),
            None => no_body_info(),
        };
        let pose = collider.pos.pose;
        proxies
            .append(
                BroadPhaseProxy {
                    collider: *handle,
                    aabb: collider.shape.compute_aabb(pose).loosened(margin),
                    is_static: body_type == RigidBodyType::Fixed,
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
                    body,
                    body_type,
                    world_com,
                    dominance,
                },
            );
    }
    (proxies, scratch.span())
}


/// `NarrowPhaseTrait::compute_contacts::<DefaultDispatcher>` on a prebuilt scratch (see
/// [`collision_inputs`]): same pair loop, carry-over and events.
pub fn contacts_from_scratch(
    ref narrow_phase: NarrowPhase,
    prediction: Fixed,
    scratch: Span<PairCollider>,
    pairs: Span<(u32, u32)>,
    ref colliders: ColliderSet,
) -> Array<CollisionEvent> {
    let mut carry: SortedMerge = CarryOver::begin(narrow_phase.pairs.span());
    let mut current = array![];
    let mut transitions = array![];
    for (i, j) in pairs {
        let co1 = *scratch.at(*i);
        let co2 = *scratch.at(*j);
        if !(co1.solid && co2.solid) {
            continue;
        }
        let previous = carry.take(co1.handle, co2.handle);
        let (pair, event) = process_pair::<DefaultDispatcher>(prediction, co1, co2, previous);
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

/// [`advance_to_final_positions`] reading the colliders from `snapshot` (the colliders as
/// [`user_changes_snapshot`] left them; stages 2–3 do not write colliders).
pub fn advance_with_snapshot(
    ref bodies: RigidBodySet, ref colliders: ColliderSet, snapshot: Span<(Handle, Collider)>,
) {
    for (handle, body) in bodies.iter().span() {
        if *body.enabled && *body.body_type != RigidBodyType::Fixed {
            advance_body_with_snapshot(*handle, *body, ref bodies, ref colliders, snapshot);
        }
    }
}

#[inline(always)]
fn advance_body_with_snapshot(
    handle: Handle,
    body: RigidBody,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    snapshot: Span<(Handle, Collider)>,
) {
    let mut body = body;
    body.pos.position = body.pos.next_position;
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

/// Stage 1 (upstream `handle_user_changes_to_colliders` then
/// `handle_user_changes_to_rigid_bodies`, and the clearing of the modified sets): see the
/// module documentation. Scans every collider and every body in ascending slot; only the flagged
/// ones are rewritten.
pub fn handle_user_changes(ref bodies: RigidBodySet, ref colliders: ColliderSet) {
    for (handle, collider) in colliders.iter() {
        if !collider.changes.is_empty() {
            collider_changes(handle, collider, ref bodies, ref colliders);
        }
    }
    for (handle, body) in bodies.iter() {
        if !body.changes.is_empty() {
            let _ = body_changes(handle, body, ref bodies, ref colliders);
        }
    }
}

/// The user changes of one flagged collider, flags cleared.
#[inline(never)]
fn collider_changes(
    handle: Handle, collider: Collider, ref bodies: RigidBodySet, ref colliders: ColliderSet,
) {
    let mut collider = collider;
    let changes = collider.changes;
    if let Some(parent) = collider.parent {
        if let Some(mut body) = bodies.get(parent.handle) {
            if changes.contains(PARENT) {
                collider.pos.pose = body.pos.position * parent.pos_wrt_parent;
            }
            if changes.intersects(SHAPE | CO_LOCAL_MASS_PROPERTIES | ENABLED_OR_DISABLED | PARENT) {
                body.changes.insert(LOCAL_MASS_PROPERTIES);
                let _ = bodies.set(parent.handle, body);
            }
        }
    }
    collider.changes = ColliderChangesTrait::empty();
    let _ = colliders.set(handle, collider);
}

/// The user changes of one flagged body, flags cleared.
#[inline(never)]
fn body_changes(
    handle: Handle, body: RigidBody, ref bodies: RigidBodySet, ref colliders: ColliderSet,
) -> RigidBody {
    let mut body = body;
    let changes = body.changes;
    if changes.contains(POSITION) || changes.contains(COLLIDERS) {
        move_colliders(body, ref colliders);
        body.mprops = body.mprops.update_world_mass_properties(body.body_type, body.pos.position);
    }
    if changes.intersects(LOCAL_MASS_PROPERTIES | COLLIDERS) {
        recompute_mass_properties_from_colliders(ref body, ref colliders);
    }
    body.changes = RigidBodyChangesTrait::empty();
    let _ = bodies.set(handle, body);
    body
}

/// Upstream `RigidBodyMassProps::recompute_mass_properties_from_colliders`: the local mass
/// properties become the sum, in attachment order, of the enabled colliders' mass properties
/// expressed in the body frame; the world ones are refreshed.
pub fn recompute_mass_properties_from_colliders(ref body: RigidBody, ref colliders: ColliderSet) {
    let mut local: MassProperties = Default::default();
    for co_handle in body.colliders {
        if let Some(collider) = colliders.get(*co_handle) {
            if collider.is_enabled() {
                if let Some(parent) = collider.parent {
                    local = local + collider.mass_properties().transform_by(parent.pos_wrt_parent);
                }
            }
        }
    }
    body.mprops.local_mprops = local;
    body.mprops = body.mprops.update_world_mass_properties(body.body_type, body.pos.position);
}

/// Sets the world pose of the colliders of `body` to `body.position * pos_wrt_parent`, without
/// raising change flags (upstream `RigidBodyColliders::update_positions`).
fn move_colliders(body: RigidBody, ref colliders: ColliderSet) {
    for co_handle in body.colliders {
        if let Some(mut collider) = colliders.get(*co_handle) {
            if let Some(parent) = collider.parent {
                collider.pos.pose = body.pos.position * parent.pos_wrt_parent;
                let _ = colliders.set(*co_handle, collider);
            }
        }
    }
}

/// Stage 2 (upstream `detect_collisions`): stateless broad phase over proxies loosened by half
/// the prediction distance, then the narrow phase with [`DefaultDispatcher`]. Returns the
/// collision events.
pub fn detect_collisions(
    params: IntegrationParameters,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    ref narrow_phase: NarrowPhase,
) -> Array<CollisionEvent> {
    let prediction = params.prediction_distance();
    let proxies = colliders.broad_phase_proxies(ref bodies, prediction);
    let pairs = find_pairs(proxies.span());
    narrow_phase
        .compute_contacts::<DefaultDispatcher>(prediction, ref bodies, ref colliders, pairs.span())
}

/// Stage 3 (upstream `build_islands_and_solve_velocity_constraints`, one island): gathers the
/// touching manifolds and the joints, runs `solve_island`, writes velocities / `next_position`
/// back to the bodies and the impulses back to the pairs and the joints.
pub fn solve(
    gravity: Vec2,
    params: IntegrationParameters,
    ref bodies: RigidBodySet,
    ref narrow_phase: NarrowPhase,
    ref impulse_joints: ImpulseJointSet,
) {
    let all = bodies.iter();
    let (mut manifolds, flags) = solve_order(narrow_phase.pairs.span(), all.span());
    let joint_entries = impulse_joints.to_array();
    let mut joints = joint_values(joint_entries.span());
    let (mut store, _) = SolverBodyStoreTrait::from_bodies(ref bodies, gravity, params);
    solve_island(params, ref store, ref manifolds, ref joints);
    store.to_bodies(ref bodies);
    if !manifolds.is_empty() {
        narrow_phase
            .pairs = scatter_touching(narrow_phase.pairs.span(), manifolds.span(), flags.span());
    }
    write_joints(joint_entries.span(), joints.span(), ref impulse_joints);
}

/// Stages 3 and 4 fused (work package OI), with the same results as [`solve`] then
/// [`advance_with_snapshot`]: only the bodies a touching manifold or an enabled joint references
/// enter the `SolverBodyStore`; every other body is solved alone by `FreeBodySolverTrait::solve`
/// (bit-identical: nothing else acts on it), and each moving body is written once, after its
/// velocities, `next_position`, position, world mass properties and collider poses.
/// `entries` and `snapshot` are the bodies and colliders as [`user_changes_bodies`] left them.
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
    let joint_entries = impulse_joints.to_array();
    let mut joints = joint_values(joint_entries.span());
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
                members.append(*entry);
            } else if *body.enabled && *body.body_type != RigidBodyType::Fixed {
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
        write_joints(joint_entries.span(), joints.span(), ref impulse_joints);
    }
    let mut dense: u32 = 0;
    for (handle, body) in entries {
        let member = any && constrained.get((*handle).into());
        if *body.enabled && *body.body_type != RigidBodyType::Fixed {
            let body = if member {
                let mut body = *body;
                store.write_body(dense, ref body);
                body
            } else {
                free.solve(*handle, *body)
            };
            advance_body_with_snapshot(*handle, body, ref bodies, ref colliders, snapshot);
        }
        if member {
            dense += 1;
        }
    }
}

/// The manifolds of the pairs that have at least one solver contact, in pair order. The solver
/// input is [`solve_order`] of this list.
pub fn touching_manifolds(pairs: Span<ContactPair>) -> Array<ContactManifold> {
    let mut out = array![];
    for pair in pairs {
        if *pair.manifold.data.num_solver_contacts != 0 {
            out.append(*pair.manifold);
        }
    }
    out
}

/// D8: whether a manifold between `body1` and `body2` goes to the second solve group: it has a
/// fixed body or no body. Upstream colours a touching pair with the lowest free colour when both
/// bodies are non-fixed (kinematic included: `RigidBody::is_fixed` is `body_type == Fixed`) and
/// with the highest free colour otherwise ("fixed geometry the final say each sweep"), and solves
/// the colours in ascending order. `entries` holds every body in ascending arena slot.
#[inline(always)]
fn fixed_last_flag(
    entries: Span<(Handle, RigidBody)>, body1: Option<Handle>, body2: Option<Handle>,
) -> bool {
    match (body1, body2) {
        (Some(h1), Some(h2)) => is_fixed_body(entries, h1) || is_fixed_body(entries, h2),
        _ => true,
    }
}

/// Whether the body of `handle` is fixed; a handle `entries` lacks counts as non-fixed. The
/// position of a body in `entries` is at most its arena slot (equal while no body was removed),
/// so the lookup starts at the slot and walks down over the holes.
fn is_fixed_body(entries: Span<(Handle, RigidBody)>, handle: Handle) -> bool {
    if entries.is_empty() {
        return false;
    }
    let mut position = handle.index;
    if position >= entries.len() {
        position = entries.len() - 1;
    }
    let mut fixed = false;
    loop {
        let (candidate, body) = entries.at(position);
        if candidate.index == @handle.index {
            fixed = *body.body_type == RigidBodyType::Fixed;
            break;
        }
        if candidate.index < @handle.index || position == 0 {
            break;
        }
        position -= 1;
    }
    fixed
}

/// The stable partition of `manifolds` (pair order): the entries flagged `false` (in order), then
/// those flagged `true`.
fn partition(manifolds: Span<ContactManifold>, flags: Span<bool>) -> Array<ContactManifold> {
    let mut first = array![];
    let mut last = array![];
    let mut next = flags;
    for manifold in manifolds {
        if *next.pop_front().unwrap() {
            last.append(*manifold);
        } else {
            first.append(*manifold);
        }
    }
    first.append_span(last.span());
    first
}

/// D8: the touching manifolds of `pairs` (as [`touching_manifolds`]) as the stable partition of
/// the ascending pair order into the manifolds between two non-fixed bodies, then the manifolds
/// with a fixed body or no body (see [`fixed_last_flag`]). `entries` holds every body
/// (`user_changes_bodies`, ascending arena slot). Returns the manifolds in solve order and, per
/// touching pair in pair order, whether it went to the second group (the argument of
/// [`scatter_touching`]).
pub fn solve_order(
    pairs: Span<ContactPair>, entries: Span<(Handle, RigidBody)>,
) -> (Array<ContactManifold>, Array<bool>) {
    let mut first = array![];
    let mut last = array![];
    let mut flags = array![];
    for pair in pairs {
        if *pair.manifold.data.num_solver_contacts != 0 {
            let manifold = *pair.manifold;
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
    first.append_span(last.span());
    (first, flags)
}

/// `pairs` with the manifold of each touching pair replaced by its solved manifold. `solved` is
/// the output of `solve_island` on [`solve_order`]'s manifolds and `last` its flags: the pairs
/// flagged `false` take `solved` from the front in pair order, the others from the point where
/// that group ends.
pub fn scatter_touching(
    pairs: Span<ContactPair>, solved: Span<ContactManifold>, last: Span<bool>,
) -> Array<ContactPair> {
    let mut n_first = 0;
    for fixed_last in last {
        if !*fixed_last {
            n_first += 1;
        }
    }
    scatter_touching_split(pairs, solved, last, n_first)
}

/// [`scatter_touching`] with the size of the first group given.
pub fn scatter_touching_split(
    pairs: Span<ContactPair>, solved: Span<ContactManifold>, last: Span<bool>, n_first: u32,
) -> Array<ContactPair> {
    let mut first = solved.slice(0, n_first);
    let mut rest = solved.slice(n_first, solved.len() - n_first);
    let mut flags = last;
    let mut out = array![];
    for pair in pairs {
        let mut pair = *pair;
        if pair.manifold.data.num_solver_contacts != 0 {
            pair
                .manifold =
                    if *flags.pop_front().unwrap() {
                        *rest.pop_front().unwrap()
                    } else {
                        *first.pop_front().unwrap()
                    };
        }
        out.append(pair);
    }
    out
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

/// Stage 4 (upstream `advance_to_final_positions`): for every enabled non-fixed body,
/// `position ← next_position`, world mass properties refreshed, attached colliders moved. No
/// change flag is raised.
pub fn advance_to_final_positions(ref bodies: RigidBodySet, ref colliders: ColliderSet) {
    for (handle, body) in bodies.iter() {
        if body.enabled && body.body_type != RigidBodyType::Fixed {
            advance_body(handle, body, ref bodies, ref colliders);
        }
    }
}

#[inline(never)]
fn advance_body(
    handle: Handle, body: RigidBody, ref bodies: RigidBodySet, ref colliders: ColliderSet,
) {
    let mut body = body;
    body.pos.position = body.pos.next_position;
    body.mprops = body.mprops.update_world_mass_properties(body.body_type, body.pos.position);
    let _ = bodies.set(handle, body);
    move_colliders(body, ref colliders);
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
