//! Rejected candidates of `solve_and_advance` (work package OI), kept for re-ranking
//! (AGENTS.md §5). Probes in `solve_benches`, ranking in the pipeline module documentation;
//! equivalence in `tests`.

use core::dict::{Felt252Dict, Felt252DictTrait};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_core::rigid_body::RigidBodyType;
use rapier_dynamics2d::collider::Collider;
use rapier_dynamics2d::collider_set::ColliderSet;
use rapier_dynamics2d::joint::{ImpulseJointSet, ImpulseJointSetTrait, JointEnabled};
use rapier_dynamics2d::narrow_phase::NarrowPhase;
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet};
use rapier_dynamics2d::solver::body_store::SolverBodyStoreTrait;
use rapier_dynamics2d::solver::island::{FreeBodySolverTrait, solve_island};
use super::{
    advance_body_with_snapshot, joint_values, scatter_touching, solve_order, touching_manifolds,
    write_joints,
};

/// The free-body arm behind a one-iteration `while` (AGENTS.md §7 "metered" call).
pub fn solve_and_advance_metered(
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
    let mut manifolds = array![];
    for pair in narrow_phase.pairs.span() {
        if *pair.manifold.data.num_solver_contacts != 0 {
            let manifold = *pair.manifold;
            if let Some(h) = manifold.data.rigid_body1 {
                constrained.insert(h.into(), true);
            }
            if let Some(h) = manifold.data.rigid_body2 {
                constrained.insert(h.into(), true);
            }
            manifolds.append(manifold);
        }
    }
    let joint_entries = impulse_joints.to_array();
    let mut joints = joint_values(joint_entries.span());
    for joint in joints.span() {
        if *joint.data.enabled == JointEnabled::Enabled {
            constrained.insert((*joint.body1).into(), true);
            constrained.insert((*joint.body2).into(), true);
        }
    }
    let any = !manifolds.is_empty() || !joints.is_empty();
    let (mut manifolds, last) = if manifolds.is_empty() {
        (manifolds, array![])
    } else {
        solve_order(narrow_phase.pairs.span(), entries)
    };
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
                .pairs = scatter_touching(narrow_phase.pairs.span(), manifolds.span(), last.span());
        }
        write_joints(joint_entries.span(), joints.span(), ref impulse_joints);
    }
    let mut dense: u32 = 0;
    for (handle, body) in entries {
        let member = any && constrained.get((*handle).into());
        if *body.enabled && *body.body_type != RigidBodyType::Fixed {
            let mut body = *body;
            if member {
                store.write_body(dense, ref body);
            }
            let mut pending = !member;
            while pending {
                body = free.solve(*handle, body);
                pending = false;
            }
            advance_body_with_snapshot(*handle, body, ref bodies, ref colliders, snapshot, params);
        }
        if member {
            dense += 1;
        }
    }
}

/// `touching_manifolds`, then a second walk over them to mark the constrained bodies.
pub fn solve_and_advance_separate_marking(
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
    let mut manifolds = touching_manifolds(narrow_phase.pairs.span());
    for manifold in manifolds.span() {
        if let Some(h) = manifold.data.rigid_body1 {
            constrained.insert((*h).into(), true);
        }
        if let Some(h) = manifold.data.rigid_body2 {
            constrained.insert((*h).into(), true);
        }
    }
    let joint_entries = impulse_joints.to_array();
    let mut joints = joint_values(joint_entries.span());
    for joint in joints.span() {
        if *joint.data.enabled == JointEnabled::Enabled {
            constrained.insert((*joint.body1).into(), true);
            constrained.insert((*joint.body2).into(), true);
        }
    }
    let any = !manifolds.is_empty() || !joints.is_empty();
    let (mut manifolds, last) = if manifolds.is_empty() {
        (manifolds, array![])
    } else {
        solve_order(narrow_phase.pairs.span(), entries)
    };
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
                .pairs = scatter_touching(narrow_phase.pairs.span(), manifolds.span(), last.span());
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
            advance_body_with_snapshot(*handle, body, ref bodies, ref colliders, snapshot, params);
        }
        if member {
            dense += 1;
        }
    }
}

/// The body walk matches the next member handle (members keep `entries` order) instead of
/// reading the dict again.
pub fn solve_and_advance_member_handles(
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
    let mut manifolds = array![];
    for pair in narrow_phase.pairs.span() {
        if *pair.manifold.data.num_solver_contacts != 0 {
            let manifold = *pair.manifold;
            if let Some(h) = manifold.data.rigid_body1 {
                constrained.insert(h.into(), true);
            }
            if let Some(h) = manifold.data.rigid_body2 {
                constrained.insert(h.into(), true);
            }
            manifolds.append(manifold);
        }
    }
    let joint_entries = impulse_joints.to_array();
    let mut joints = joint_values(joint_entries.span());
    for joint in joints.span() {
        if *joint.data.enabled == JointEnabled::Enabled {
            constrained.insert((*joint.body1).into(), true);
            constrained.insert((*joint.body2).into(), true);
        }
    }
    let any = !manifolds.is_empty() || !joints.is_empty();
    let (mut manifolds, last) = if manifolds.is_empty() {
        (manifolds, array![])
    } else {
        solve_order(narrow_phase.pairs.span(), entries)
    };
    let mut members = array![];
    let mut member_handles = array![];
    let mut has_free = false;
    if any {
        for entry in entries {
            let (handle, body) = entry;
            if constrained.get((*handle).into()) {
                members.append(*entry);
                member_handles.append(*handle);
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
                .pairs = scatter_touching(narrow_phase.pairs.span(), manifolds.span(), last.span());
        }
        write_joints(joint_entries.span(), joints.span(), ref impulse_joints);
    }
    let mut dense: u32 = 0;
    let mut pending = member_handles.span();
    for (handle, body) in entries {
        let member = match pending.get(0) {
            Some(next) => *next.unbox() == *handle,
            None => false,
        };
        if member {
            let _ = pending.pop_front();
        }
        if *body.enabled && *body.body_type != RigidBodyType::Fixed {
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

/// The free-body constants built on the first free moving body (an `Option` carried by the
/// body walk) instead of from the membership walk.
pub fn solve_and_advance_lazy(
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
    let mut manifolds = array![];
    for pair in narrow_phase.pairs.span() {
        if *pair.manifold.data.num_solver_contacts != 0 {
            let manifold = *pair.manifold;
            if let Some(h) = manifold.data.rigid_body1 {
                constrained.insert(h.into(), true);
            }
            if let Some(h) = manifold.data.rigid_body2 {
                constrained.insert(h.into(), true);
            }
            manifolds.append(manifold);
        }
    }
    let joint_entries = impulse_joints.to_array();
    let mut joints = joint_values(joint_entries.span());
    for joint in joints.span() {
        if *joint.data.enabled == JointEnabled::Enabled {
            constrained.insert((*joint.body1).into(), true);
            constrained.insert((*joint.body2).into(), true);
        }
    }
    let any = !manifolds.is_empty() || !joints.is_empty();
    let (mut manifolds, last) = if manifolds.is_empty() {
        (manifolds, array![])
    } else {
        solve_order(narrow_phase.pairs.span(), entries)
    };
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
    let mut free = if any {
        None
    } else {
        Some(FreeBodySolverTrait::new(params, gravity))
    };
    if any {
        solve_island(params, ref store, ref manifolds, ref joints);
        if !manifolds.is_empty() {
            narrow_phase
                .pairs = scatter_touching(narrow_phase.pairs.span(), manifolds.span(), last.span());
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
                let solver = match free {
                    Some(solver) => solver,
                    None => {
                        let solver = FreeBodySolverTrait::new(params, gravity);
                        free = Some(solver);
                        solver
                    },
                };
                solver.solve(*handle, *body)
            };
            advance_body_with_snapshot(*handle, body, ref bodies, ref colliders, snapshot, params);
        }
        if member {
            dense += 1;
        }
    }
}

/// `solve_island` called even with no manifold and no joint.
pub fn solve_and_advance_island_always(
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
    let mut manifolds = array![];
    for pair in narrow_phase.pairs.span() {
        if *pair.manifold.data.num_solver_contacts != 0 {
            let manifold = *pair.manifold;
            if let Some(h) = manifold.data.rigid_body1 {
                constrained.insert(h.into(), true);
            }
            if let Some(h) = manifold.data.rigid_body2 {
                constrained.insert(h.into(), true);
            }
            manifolds.append(manifold);
        }
    }
    let joint_entries = impulse_joints.to_array();
    let mut joints = joint_values(joint_entries.span());
    for joint in joints.span() {
        if *joint.data.enabled == JointEnabled::Enabled {
            constrained.insert((*joint.body1).into(), true);
            constrained.insert((*joint.body2).into(), true);
        }
    }
    let any = !manifolds.is_empty() || !joints.is_empty();
    let (mut manifolds, last) = if manifolds.is_empty() {
        (manifolds, array![])
    } else {
        solve_order(narrow_phase.pairs.span(), entries)
    };
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
    solve_island(params, ref store, ref manifolds, ref joints);
    if !manifolds.is_empty() {
        narrow_phase
            .pairs = scatter_touching(narrow_phase.pairs.span(), manifolds.span(), last.span());
    }
    write_joints(joint_entries.span(), joints.span(), ref impulse_joints);
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
            advance_body_with_snapshot(*handle, body, ref bodies, ref colliders, snapshot, params);
        }
        if member {
            dense += 1;
        }
    }
}
