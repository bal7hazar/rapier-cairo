//! Stages 3 and 4 of the fused step (work package OI): the solver and the position update in
//! one walk (see the parent module). Generic over the joint strategy since CS2 (`config`).

use core::dict::{Felt252Dict, Felt252DictTrait};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_dynamics2d::collider::Collider;
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::joint::{ImpulseJoint, ImpulseJointSet, ImpulseJointSetTrait};
use rapier_dynamics2d::narrow_phase::NarrowPhase;
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet, RigidBodySetTrait};
use rapier_dynamics2d::solver::island::{
    FreeBodySolverTrait, SolvedIsland, SolvedIslandTrait, SolverInputTrait,
};
use super::config::{ImpulseJointSolver, JointStrategy};
use super::{
    advance_body_with_snapshot, any_sleeping, dormant_of, fixed_last_flag, immovable, link_status,
    merge_pairs, moving, scatter_impulses, split_dormant,
};

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
    solve_and_advance_sleeping_with::<
        ImpulseJointSolver,
    >(
        gravity,
        params,
        ref bodies,
        ref colliders,
        ref narrow_phase,
        ref impulse_joints,
        entries,
        snapshot,
        joint_entries,
        sleeping,
    );
}

/// [`solve_and_advance_sleeping`] with the joints handled by `J` (CS2, `config`): with
/// `config::NoJoints` the joint solver is not compiled.
pub(crate) fn solve_and_advance_sleeping_with<impl J: JointStrategy>(
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
    let manifolds = first;
    let (joint_entries, mut joints) = J::constrain(
        joint_entries, entries, sleeping, ref constrained,
    );
    let any = !manifolds.is_empty() || !joints.is_empty();
    // BT4: the members gathered straight into the solver input (no `SolverBodyStore`, no copy
    // of the entries), their flags kept for the write-back walk.
    let mut input = SolverInputTrait::new();
    let mut member_flags = array![];
    let mut has_free = false;
    if any {
        let dt = params.substep_dt();
        for (handle, body) in entries {
            let member = constrained.get((*handle).into());
            member_flags.append(member);
            if member {
                if sleeping && *body.activation.sleeping {
                    input.push(*handle, immovable(*body), gravity, dt);
                } else {
                    input.push(*handle, *body, gravity, dt);
                }
            } else if moving(body) {
                has_free = true;
            }
        }
    }
    // With no manifold and no joint, `solve_island` would only validate the parameters, which
    // `FreeBodySolverTrait::new` does with the same panics; with constraints it is only built
    // when a moving body is free.
    let free = if !any || has_free {
        FreeBodySolverTrait::new(params, gravity)
    } else {
        Default::default()
    };
    let mut solved: SolvedIsland = Default::default();
    if any {
        solved = J::solve(params, input, manifolds.span(), ref joints);
        if !manifolds.is_empty() {
            // BT4: the impulses go straight into the pairs (no solved manifold array).
            narrow_phase
                .pairs =
                    scatter_impulses(narrow_phase.pairs.span(), @solved, flags.span(), n_first);
        }
        J::write(joint_entries, joints.span(), ref impulse_joints);
    }
    let mut dense: u32 = 0;
    let mut member_flags = member_flags.span();
    for (handle, body) in entries {
        let member = match member_flags.pop_front() {
            Some(member) => *member,
            None => false,
        };
        if moving(body) {
            let body = if member {
                let mut body = *body;
                solved.write_body(dense, ref body);
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
    bodies.mark_modified();
    colliders.mark_modified();
}
