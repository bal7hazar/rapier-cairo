//! The staged stage functions of the step (upstream's `detect_collisions`,
//! `build_islands_and_solve_velocity_constraints`, `advance_to_final_positions`), moved out of
//! `crate::pipeline` by work package SL (file budget). `step` runs their fused versions
//! (`collision_inputs_sleeping` + `compute_contacts_from_scratch`, `solve_and_advance_sleeping`);
//! these give the same results (`fused_alternatives::step_staged`, equivalence in `tests`) and
//! are what the diagnostics and the candidates call.

use fixed::Fixed;
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::events::CollisionEvent;
use rapier_dynamics2d::joint::{ImpulseJointSet, ImpulseJointSetTrait};
use rapier_dynamics2d::narrow_phase::{NarrowPhase, compute_contacts_from_scratch};
use rapier_dynamics2d::rigid_body::RigidBodyMassPropsTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet, RigidBodySetTrait};
use rapier_dynamics2d::solver::body_store::SolverBodyStoreTrait;
use rapier_dynamics2d::solver::island::solve_island;
use rapier_geometry2d::broad_phase::find_pairs;
use crate::dispatcher::DefaultDispatcher;
use super::{
    active_joints, any_sleeping, body_infos, collision_inputs_with_events, immovable, islands,
    joint_values, merge_pairs, moving, scatter_touching, solve_order, split_dormant,
    split_dormant_existing, user_changes, write_joints,
};

/// Stage 2 (upstream `detect_collisions`): stateless broad phase over proxies loosened by half
/// the prediction distance (sleeping bodies static), then the narrow phase with
/// [`DefaultDispatcher`] on the active pairs; the dormant pairs of sleeping bodies are carried
/// over unchanged (`sleeping`). Returns the collision events.
pub fn detect_collisions(
    params: IntegrationParameters,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    ref narrow_phase: NarrowPhase,
) -> Array<CollisionEvent> {
    detect_collisions_with_prediction(
        params.prediction_distance(), ref bodies, ref colliders, ref narrow_phase,
    )
}

/// [`detect_collisions`] for an explicit prediction distance (the collision stages of
/// `facade::CollisionPipelineTrait::step`).
pub fn detect_collisions_with_prediction(
    prediction: Fixed,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    ref narrow_phase: NarrowPhase,
) -> Array<CollisionEvent> {
    detect_collisions_fresh(
        prediction, ref bodies, ref colliders, ref narrow_phase, array![].span(),
    )
}

/// [`detect_collisions_with_prediction`] with the colliders inserted since the last step
/// (`user_changes::handle_user_changes_fresh`), whose proxies are not static on a sleeping parent.
pub(crate) fn detect_collisions_fresh(
    prediction: Fixed,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    ref narrow_phase: NarrowPhase,
    fresh: Span<Handle>,
) -> Array<CollisionEvent> {
    let snapshot = colliders.iter().span();
    let entries = bodies.iter().span();
    let (infos, _) = body_infos(entries);
    let (proxies, scratch, sleeping, _) = collision_inputs_with_events(
        snapshot, infos.span(), ref bodies, prediction,
    );
    let proxies = super::sleeping::unstatic_fresh(proxies, fresh, sleeping, snapshot, entries);
    let mut dormant = array![];
    if sleeping {
        let (active, asleep) = split_dormant_existing(narrow_phase.pairs.span(), entries, snapshot);
        narrow_phase.pairs = active;
        dormant = asleep;
    }
    let pairs = find_pairs(proxies.span());
    let events = compute_contacts_from_scratch::<
        DefaultDispatcher,
    >(ref narrow_phase, prediction, scratch, pairs.span(), ref colliders);
    if !dormant.is_empty() {
        narrow_phase.pairs = merge_pairs(narrow_phase.pairs.span(), dormant.span());
    }
    events
}

/// Stage 3 (upstream `build_islands_and_solve_velocity_constraints`, one island): gathers the
/// touching manifolds of the active pairs and the joints, runs `solve_island` over every body
/// (sleeping ones as immovable copies, `from_entries`), writes velocities / `next_position`
/// back to the moving bodies and the impulses back to the pairs and the joints. Run
/// [`super::kinematic::interpolate_kinematic_velocities`] before collision detection and
/// [`update_islands`] before this stage.
pub fn solve(
    gravity: Vec2,
    params: IntegrationParameters,
    ref bodies: RigidBodySet,
    ref narrow_phase: NarrowPhase,
    ref impulse_joints: ImpulseJointSet,
) {
    let all = bodies.iter();
    let entries = all.span();
    let sleeping = any_sleeping(entries);
    let mut dormant = array![];
    if sleeping {
        let (active, asleep) = split_dormant(narrow_phase.pairs.span(), entries);
        narrow_phase.pairs = active;
        dormant = asleep;
    }
    let (mut manifolds, flags) = solve_order(narrow_phase.pairs.span(), entries);
    let joint_entries = impulse_joints.to_array();
    let joint_entries = if sleeping {
        active_joints(joint_entries.span(), entries)
    } else {
        joint_entries
    };
    let mut joints = joint_values(joint_entries.span());
    let members = if sleeping {
        let mut members = array![];
        for entry in entries {
            let (handle, body) = *entry;
            members
                .append(
                    if body.activation.sleeping {
                        (handle, immovable(body))
                    } else {
                        (handle, body)
                    },
                );
        }
        members.span()
    } else {
        entries
    };
    let mut store = SolverBodyStoreTrait::from_entries(members, gravity, params);
    solve_island(params, ref store, ref manifolds, ref joints);
    store.to_bodies(ref bodies);
    if !manifolds.is_empty() {
        narrow_phase
            .pairs = scatter_touching(narrow_phase.pairs.span(), manifolds.span(), flags.span());
    }
    write_joints(joint_entries.span(), joints.span(), ref impulse_joints);
    if !dormant.is_empty() {
        narrow_phase.pairs = merge_pairs(narrow_phase.pairs.span(), dormant.span());
    }
}

/// Stage 4 (upstream `advance_to_final_positions`): for every enabled, awake, non-fixed body,
/// the sleep timer update then `position ← next_position`, world mass properties refreshed,
/// attached colliders moved. No change flag is raised.
pub fn advance_to_final_positions(
    ref bodies: RigidBodySet, ref colliders: ColliderSet, params: IntegrationParameters,
) {
    for (handle, body) in bodies.iter() {
        if moving(@body) {
            advance_body(handle, body, ref bodies, ref colliders, params);
        }
    }
}

#[inline(never)]
fn advance_body(
    handle: Handle,
    body: RigidBody,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    params: IntegrationParameters,
) {
    let mut body = body;
    let previous = body.pos.position;
    body.pos.position = body.pos.next_position;
    islands::update_sleep_timer(ref body, previous, params);
    body.mprops = body.mprops.update_world_mass_properties(body.body_type, body.pos.position);
    let _ = bodies.set(handle, body);
    user_changes::move_colliders(body, ref colliders);
}
