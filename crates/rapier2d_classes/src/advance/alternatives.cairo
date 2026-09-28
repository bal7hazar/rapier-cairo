//! CS5's crossing of the fused solve and position update, measured against CX1's
//! (`super`) and rejected: whole bodies (colliders included), each moving body's colliders as
//! `(handle, parent)`, the pairs' point impulses both ways, and in the class the sets rebuilt
//! (`crate::arena`, handles renumbered densely) and the stage run on them. Its class is not
//! declared any more; [`Cs5ValuesSolveAdvance`] runs its gather, class work and write-back in
//! process (`tests/steps.cairo`, `steps_advance_cs5_values_*`).

use core::dict::{Felt252Dict, Felt252DictTrait};
use fixed::Fixed;
use rapier2d::pipeline::config::errors::JOINTS;
use rapier2d::pipeline::stages::SolveAdvanceStage;
use rapier2d::pipeline::{moving, snapshot_collider};
use rapier2d::prelude::{
    Collider, ColliderSet, ColliderSetTrait, ContactPair, ContactPairTrait, Handle, ImpulseJoint,
    ImpulseJointSet, ImpulseJointSetTrait, IntegrationParameters, Pose2, RigidBody, RigidBodySet,
    RigidBodySetTrait, Shape, Vec2,
};
use rapier_core::rigid_body::RigidBodyActivation;
use rapier_dynamics2d::collider::components::ColliderParent;
use rapier_dynamics2d::narrow_phase::NarrowPhase;
use rapier_dynamics2d::rigid_body::{RigidBodyMassPropsTrait, RigidBodyPosition, RigidBodyVelocity};
use rapier_dynamics2d::solver::island::FreeBodySolverTrait;
use rapier_geometry2d::contact::{ContactManifoldData, TrackedContact};
use crate::arena::{dense, dense_option, partial_state, positions};


/// The impulses of a contact point the solve reads (warm start) and writes.
#[derive(Copy, Drop, Serde)]
pub struct PointImpulses {
    pub impulse: Fixed,
    pub tangent_impulse: Fixed,
    pub warmstart_impulse: Fixed,
    pub warmstart_tangent_impulse: Fixed,
}

/// What the solve reads of a touching pair: its solver data, its point count and the impulses
/// of its two points.
#[derive(Copy, Drop, Serde)]
pub struct ContactView {
    pub data: ContactManifoldData,
    pub num_points: u8,
    pub points: [PointImpulses; 2],
}

/// What the stage writes of a moved body, but its world mass properties (recomputed at the new
/// pose by the caller).
#[derive(Copy, Drop, Serde)]
pub struct BodyMotion {
    pub pos: RigidBodyPosition,
    pub vels: RigidBodyVelocity,
    pub activation: RigidBodyActivation,
}

#[inline(always)]
fn impulses_of(point: TrackedContact) -> PointImpulses {
    PointImpulses {
        impulse: point.data.impulse,
        tangent_impulse: point.data.tangent_impulse,
        warmstart_impulse: point.data.warmstart_impulse,
        warmstart_tangent_impulse: point.data.warmstart_tangent_impulse,
    }
}

#[inline(always)]
fn write_impulses(ref point: TrackedContact, impulses: PointImpulses) {
    point.data.impulse = impulses.impulse;
    point.data.tangent_impulse = impulses.tangent_impulse;
    point.data.warmstart_impulse = impulses.warmstart_impulse;
    point.data.warmstart_tangent_impulse = impulses.warmstart_tangent_impulse;
}

/// `pair`'s view.
fn view(pair: @ContactPair) -> ContactView {
    let [p0, p1] = *pair.manifold.points;
    ContactView {
        data: *pair.manifold.data,
        num_points: *pair.manifold.num_points,
        points: [impulses_of(p0), impulses_of(p1)],
    }
}

/// A pair the solve reads as it reads the original (default keys and geometry, which it does not
/// read).
fn from_view(view: ContactView) -> ContactPair {
    let [i0, i1] = view.points;
    let mut pair = ContactPairTrait::new(Default::default(), Default::default());
    let [mut p0, mut p1] = pair.manifold.points;
    write_impulses(ref p0, i0);
    write_impulses(ref p1, i1);
    pair.manifold.points = [p0, p1];
    pair.manifold.num_points = view.num_points;
    pair.manifold.data = view.data;
    pair
}

/// What crosses into [`SolveAdvanceClass`], gathered from the caller's sets.
#[derive(Drop)]
struct Crossing {
    /// The views of the touching pairs, in pair order.
    touching: Array<ContactView>,
    /// The bodies the stage reads, ascending slot.
    members: Array<(Handle, RigidBody)>,
    /// The colliders of the moving bodies that have a parent.
    colliders: Array<(Handle, Collider)>,
}

/// The touching pairs of `pairs`, the bodies of `entries` that move or that a touching pair
/// references, and the parented colliders of the moving ones (read from `snapshot` when it has
/// them, as the stage reads them).
fn gather(
    pairs: Span<ContactPair>,
    entries: Span<(Handle, RigidBody)>,
    snapshot: Span<(Handle, Collider)>,
    ref colliders: ColliderSet,
) -> Crossing {
    let mut touching = array![];
    let mut referenced: Felt252Dict<bool> = Default::default();
    for pair in pairs {
        if *pair.manifold.data.num_solver_contacts != 0 {
            touching.append(view(pair));
            if let Some(h) = *pair.manifold.data.rigid_body1 {
                referenced.insert(h.into(), true);
            }
            if let Some(h) = *pair.manifold.data.rigid_body2 {
                referenced.insert(h.into(), true);
            }
        }
    }
    let mut members = array![];
    let mut moved = array![];
    for (handle, body) in entries {
        let moves = moving(body);
        if moves || referenced.get((*handle).into()) {
            members.append((*handle, *body));
        }
        if moves {
            for co_handle in body.colliders {
                if let Some(collider) = snapshot_collider(snapshot, *co_handle, ref colliders) {
                    if collider.parent.is_some() {
                        moved.append((*co_handle, collider));
                    }
                }
            }
        }
    }
    Crossing { touching, members, colliders: moved }
}

/// The class's results written back: bodies and colliders untracked (as the stage writes them),
/// the impulses into the touching pairs of `narrow_phase.pairs`, both sets flagged modified.
fn write_back(
    motions: Span<BodyMotion>,
    poses: Span<Pose2>,
    impulses: Span<[PointImpulses; 2]>,
    crossing: @Crossing,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    ref narrow_phase: NarrowPhase,
) {
    let mut motions = motions;
    for (handle, body) in crossing.members.span() {
        if moving(body) {
            let motion = *motions.pop_front().unwrap();
            let mut body = *body;
            body.pos = motion.pos;
            body.vels = motion.vels;
            body.activation = motion.activation;
            body
                .mprops = body
                .mprops
                .update_world_mass_properties(body.body_type, body.pos.position);
            let _ = bodies.set_internal(*handle, body);
        }
    }
    let mut poses = poses;
    for (handle, collider) in crossing.colliders.span() {
        let mut collider = *collider;
        collider.pos.pose = *poses.pop_front().unwrap();
        let _ = colliders.set_internal(*handle, collider);
    }
    if !impulses.is_empty() {
        let mut impulses = impulses;
        let mut out = array![];
        for pair in narrow_phase.pairs.span() {
            let mut pair = *pair;
            if pair.manifold.data.num_solver_contacts != 0 {
                let [i0, i1] = *impulses.pop_front().unwrap();
                let [mut p0, mut p1] = pair.manifold.points;
                write_impulses(ref p0, i0);
                write_impulses(ref p1, i1);
                pair.manifold.points = [p0, p1];
            }
            out.append(pair);
        }
        narrow_phase.pairs = out;
    }
    bodies.mark_modified();
    colliders.mark_modified();
}

/// Measurement only: CS5's `LibraryCallSolveAdvance` with `solve_and_advance_values` called in
/// process instead of library-called (the crossing's gather, set rebuild and write-back without
/// the serialisation and the call).
pub impl Cs5ValuesSolveAdvance of SolveAdvanceStage {
    fn solve_and_advance(
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
        assert(joint_entries.is_empty(), JOINTS);
        let crossing = gather(narrow_phase.pairs.span(), entries, snapshot, ref colliders);
        if crossing.members.is_empty() {
            let _ = FreeBodySolverTrait::new(params, gravity);
            bodies.mark_modified();
            colliders.mark_modified();
            return;
        }
        let mut parents = array![];
        for (handle, collider) in crossing.colliders.span() {
            parents.append((*handle, collider.parent.unwrap()));
        }
        let (motions, poses, impulses) = solve_and_advance_values(
            gravity,
            params,
            crossing.members.span(),
            parents.span(),
            crossing.touching.span(),
            sleeping,
        );
        write_back(
            motions, poses, impulses, @crossing, ref bodies, ref colliders, ref narrow_phase,
        );
    }
}

/// A collider with only its parent: all the position update reads of it.
fn skeleton(parent: ColliderParent) -> Collider {
    Collider {
        co_type: rapier_core::collider::ColliderType::Solid,
        shape: Shape::Ball(rapier_geometry2d::shape::ball::Ball { radius: fixed::ZERO }),
        mprops: Default::default(),
        changes: rapier_core::collider::ColliderChangesTrait::empty(),
        parent: Some(parent),
        pos: Default::default(),
        material: Default::default(),
        flags: Default::default(),
        contact_force_event_threshold: fixed::ZERO,
        one_way: BoxTrait::new(None),
        user_data: 0,
    }
}

/// `solve_and_advance_sleeping_with::<NoJoints>` on the crossing's values (see the module
/// documentation): the motion of each moving body of `entries` (in order), the new pose of each
/// collider of `colliders` (in order) and the point impulses of each view of `touching`.
pub fn solve_and_advance_values(
    gravity: Vec2,
    params: IntegrationParameters,
    entries: Span<(Handle, RigidBody)>,
    colliders: Span<(Handle, ColliderParent)>,
    touching: Span<ContactView>,
    sleeping: bool,
) -> (Span<BodyMotion>, Span<Pose2>, Span<[PointImpulses; 2]>) {
    // Dense handles (`crate::arena`): bodies and colliders take the slots of their positions.
    let (n_bodies, n_colliders) = (entries.len(), colliders.len());
    let mut body_at = positions(entries);
    let mut collider_at = positions(colliders);
    let mut bodies = array![];
    let mut index: u32 = 0;
    for (_, body) in entries {
        let mut body = *body;
        let mut attached = array![];
        for co_handle in body.colliders {
            attached.append(dense(ref collider_at, n_colliders, *co_handle));
        }
        body.colliders = attached.span();
        bodies.append((Handle { index, generation: 0 }, body));
        index += 1;
    }
    let bodies = bodies.span();
    let mut skeletons = array![];
    index = 0;
    for (_, parent) in colliders {
        let mut parent = *parent;
        parent.handle = dense(ref body_at, n_bodies, parent.handle);
        skeletons.append((Handle { index, generation: 0 }, skeleton(parent)));
        index += 1;
    }
    let mut body_set = RigidBodySetTrait::from_state(partial_state(bodies));
    let mut collider_set = ColliderSetTrait::from_state(partial_state(skeletons.span()));
    let mut pairs = array![];
    for view in touching {
        let mut view = *view;
        view.data.rigid_body1 = dense_option(ref body_at, n_bodies, view.data.rigid_body1);
        view.data.rigid_body2 = dense_option(ref body_at, n_bodies, view.data.rigid_body2);
        pairs.append(from_view(view));
    }
    let mut narrow_phase = NarrowPhase { pairs };
    let mut joints = ImpulseJointSetTrait::new();
    rapier2d::pipeline::solve_and_advance_sleeping_with::<
        rapier2d::pipeline::config::NoJoints,
    >(
        gravity,
        params,
        ref body_set,
        ref collider_set,
        ref narrow_phase,
        ref joints,
        bodies,
        array![].span(),
        array![].span(),
        sleeping,
    );
    let mut motions = array![];
    for (handle, body) in bodies {
        if moving(body) {
            let body = body_set.get(*handle).unwrap();
            motions
                .append(BodyMotion { pos: body.pos, vels: body.vels, activation: body.activation });
        }
    }
    let mut poses = array![];
    for (handle, _) in skeletons.span() {
        poses.append(collider_set.get(*handle).unwrap().pos.pose);
    }
    let mut impulses = array![];
    for pair in narrow_phase.pairs.span() {
        let [p0, p1] = *pair.manifold.points;
        impulses.append([impulses_of(p0), impulses_of(p1)]);
    }
    (motions.span(), poses.span(), impulses.span())
}
