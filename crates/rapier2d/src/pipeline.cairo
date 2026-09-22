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
//! 3. [`solve`]: the manifolds that have solver contacts, in pair order (D8), and the impulse
//!    joints in slot order go to `solve_island` over a `SolverBodyStore`; velocities and
//!    `next_position` go back to the bodies, the solved impulses back into the narrow-phase pairs
//!    (next step's warm start) and into the joint set;
//! 4. [`advance_to_final_positions`]: `position ← next_position` for every enabled non-fixed
//!    body, world mass properties refreshed, attached colliders moved;
//! 5. the collision events of stage 2 are returned.
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
//! One settled `BOX_STACK3` step (3 touching cuboid pairs), Sierra gas | Cairo steps, from
//! `benches` (cumulative probes, differences): user changes 261 540 | 2 591, broad phase
//! 337 680 | 2 807, narrow phase 2 422 715 | 11 403, solver 14 444 688 | 130 119, position
//! update 418 076 | 3 929; whole step 17 895 799 | 150 948 (`tests/world_step.cairo`).
//!
//! Candidates (`alternatives`, equivalence in `tests`):
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

use glam::Vec2;
use rapier_core::Handle;
use rapier_core::collider::ColliderChangesTrait;
use rapier_core::collider::changes::{
    ENABLED_OR_DISABLED, LOCAL_MASS_PROPERTIES as CO_LOCAL_MASS_PROPERTIES, PARENT, SHAPE,
};
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_core::rigid_body::changes::{COLLIDERS, LOCAL_MASS_PROPERTIES, POSITION};
use rapier_core::rigid_body::{RigidBodyChangesTrait, RigidBodyType};
use rapier_dynamics2d::collider::{Collider, ColliderTrait};
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::events::CollisionEvent;
use rapier_dynamics2d::joint::{ImpulseJoint, ImpulseJointSet, ImpulseJointSetTrait};
use rapier_dynamics2d::narrow_phase::{ContactPair, NarrowPhase, NarrowPhaseTrait};
use rapier_dynamics2d::rigid_body::RigidBodyMassPropsTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet, RigidBodySetTrait};
use rapier_dynamics2d::solver::body_store::SolverBodyStoreTrait;
use rapier_dynamics2d::solver::island::solve_island;
use rapier_geometry2d::broad_phase::find_pairs;
use rapier_geometry2d::contact::ContactManifold;
use rapier_geometry2d::mass::{MassProperties, MassPropertiesTrait};
use crate::dispatcher::DefaultDispatcher;
use crate::world::World;

#[cfg(test)]
pub(crate) mod alternatives;
#[cfg(test)]
mod benches;
#[cfg(test)]
pub(crate) mod fixtures;
#[cfg(test)]
mod tests;

/// One step of `world` (see the module documentation for the stages); returns the collision
/// events.
///
/// # Panics
/// As the stages: fixed-point overflow, zero solver iterations, negative parameters.
pub fn step(ref world: World) -> Array<CollisionEvent> {
    handle_user_changes(ref world.bodies, ref world.colliders);
    let events = detect_collisions(
        world.integration_parameters, ref world.bodies, ref world.colliders, ref world.narrow_phase,
    );
    solve(
        world.gravity,
        world.integration_parameters,
        ref world.bodies,
        ref world.narrow_phase,
        ref world.impulse_joints,
    );
    advance_to_final_positions(ref world.bodies, ref world.colliders);
    events
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
            body_changes(handle, body, ref bodies, ref colliders);
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
) {
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
    let mut manifolds = touching_manifolds(narrow_phase.pairs.span());
    let joint_entries = impulse_joints.to_array();
    let mut joints = joint_values(joint_entries.span());
    let (mut store, _) = SolverBodyStoreTrait::from_bodies(ref bodies, gravity, params);
    solve_island(params, ref store, ref manifolds, ref joints);
    store.to_bodies(ref bodies);
    if !manifolds.is_empty() {
        narrow_phase.pairs = scatter_touching(narrow_phase.pairs.span(), manifolds.span());
    }
    write_joints(joint_entries.span(), joints.span(), ref impulse_joints);
}

/// The manifolds of the pairs that have at least one solver contact, in pair order (D8).
pub fn touching_manifolds(pairs: Span<ContactPair>) -> Array<ContactManifold> {
    let mut out = array![];
    for pair in pairs {
        if *pair.manifold.data.num_solver_contacts != 0 {
            out.append(*pair.manifold);
        }
    }
    out
}

/// `pairs` with the manifold of each touching pair replaced, in order, by the next entry of
/// `solved` (the output of `solve_island` on [`touching_manifolds`]).
pub fn scatter_touching(
    pairs: Span<ContactPair>, solved: Span<ContactManifold>,
) -> Array<ContactPair> {
    let mut solved = solved;
    let mut out = array![];
    for pair in pairs {
        let mut pair = *pair;
        if pair.manifold.data.num_solver_contacts != 0 {
            pair.manifold = *solved.pop_front().unwrap();
        }
        out.append(pair);
    }
    out
}

fn joint_values(entries: Span<(Handle, ImpulseJoint)>) -> Array<ImpulseJoint> {
    let mut out = array![];
    for (_, joint) in entries {
        out.append(*joint);
    }
    out
}

fn write_joints(
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
