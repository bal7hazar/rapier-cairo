//! The fused solve and position update (`StepConfig::Advance`) in a declared class (CS5, CX1).
//!
//! [`SolveAdvanceClass`] runs the walk of the step's own stage
//! (`solve_and_advance_sleeping_with::<NoJoints>`) on the values that cross, with the stage's
//! own functions (`SolverInputTrait::push`, `solve_island_input_contacts`,
//! `FreeBodySolverTrait::solve`, `SolvedIslandTrait::write_body`, `update_sleep_timer`), and
//! returns what the stage changed. What crosses (CX1, "compact deltas"):
//! * in: gravity, the integration parameters, a [`MotionBody`] (all the stage reads of a body) of
//!   each body the stage reads (every moving body and every body a touching pair references,
//!   ascending slot: the others are neither solved nor written), a `SolverManifold` of each
//!   touching pair (all the solve reads of it: its solver data and warm-start impulses) and
//!   whether a body sleeps;
//! * out: a [`Motion`] per moved body (in the order of the bodies sent: its new pose, which is
//!   both `position` and `next_position` after the stage, its velocities and its sleep timer, the
//!   only activation field the stage writes) and the `ManifoldImpulses` of each touching pair.
//!
//! No collider crosses: the stage only writes `pose = body pose * pos_wrt_parent` of each
//! parented collider of a moved body, which the caller computes from the new pose. No set is
//! rebuilt in the class: the handles keep their caller values (the solve only matches them).
//!
//! The caller ([`LibraryCallSolveAdvance`]) writes the results back untracked, as the stage does:
//! each moved body gets its motion and its world mass properties at the new pose (the stage's own
//! `update_world_mass_properties`), its parented colliders their poses, each touching pair its
//! impulses (`SolvedIslandTrait::write_impulses`). [`HybridSolveAdvance`] makes no call on a step
//! without a touching pair: every moving body is then free, and
//! `rapier2d::pipeline::solve_and_advance_free` (the pair-free path's stage, in the caller anyway)
//! gives the same results.
//!
//! CS5's crossing (whole bodies, the colliders' parents, the sets rebuilt in the class and the
//! stage run on them) is the measured loser, in [`alternatives`].

use core::dict::{Felt252Dict, Felt252DictTrait};
use rapier2d::pipeline::config::errors::JOINTS;
use rapier2d::pipeline::islands::update_sleep_timer;
use rapier2d::pipeline::stages::SolveAdvanceStage;
use rapier2d::pipeline::{moving, snapshot_collider, solve_and_advance_free};
use rapier2d::prelude::{
    Collider, ColliderSet, ColliderSetTrait, ContactPair, Fixed, Handle, ImpulseJoint,
    ImpulseJointSet, IntegrationParameters, Pose2, RigidBody, RigidBodySet, RigidBodySetTrait, Vec2,
};
use rapier_core::integration_parameters::IntegrationParametersTrait;
use rapier_core::rigid_body::{RigidBodyActivation, RigidBodyDamping, RigidBodyType};
use rapier_dynamics2d::narrow_phase::NarrowPhase;
use rapier_dynamics2d::rigid_body::{
    LockedAxes, RigidBodyForces, RigidBodyMassProps, RigidBodyMassPropsTrait, RigidBodyPosition,
    RigidBodyVelocity,
};
use rapier_dynamics2d::solver::island::{
    FreeBodySolverTrait, ManifoldImpulses, PointImpulses, SolvedIsland, SolvedIslandTrait,
    SolverInputTrait, solve_island_input_contacts,
};
use rapier_geometry2d::contact::{
    ContactData, ContactManifold, ContactManifoldData, SolverContact, SolverFlags, TrackedContact,
};
use rapier_geometry2d::mass::MassProperties;
use starknet::SyscallResultTrait;
use starknet::syscalls::library_call_syscall;
use crate::hashes::{ClassHashes, errors};

/// Measured and rejected: CS5's crossing.
pub mod alternatives;

/// The rarely non-default fields of a moving body the solve and the sleep timer read.
#[derive(Copy, Drop, Serde, PartialEq)]
pub struct MotionExtras {
    pub damping: RigidBodyDamping,
    pub forces: RigidBodyForces,
    pub normalized_linear_threshold: Fixed,
    pub angular_threshold: Fixed,
}

/// The [`MotionExtras`] of a default body (sent as `None`).
#[inline(always)]
fn default_extras() -> MotionExtras {
    let activation: RigidBodyActivation = Default::default();
    MotionExtras {
        damping: Default::default(),
        forces: Default::default(),
        normalized_linear_threshold: activation.normalized_linear_threshold,
        angular_threshold: activation.angular_threshold,
    }
}

/// All the solve and the position update read of a body. `packed` is `slot · 2^13 + locked
/// axes · 32 + body type · 8 + constrained · 4 + enabled · 2 + sleeping` (the body type as
/// [`type_code`]; constrained: a touching pair references the body, the stage's `constrained`
/// set); the
/// world mass properties are recomputed from the local ones (`gather`), `next_position` is only
/// read for a position-based kinematic body (the solve writes it for the others), `extras` is
/// `None` for default damping, forces and sleep thresholds.
#[derive(Copy, Drop, Serde)]
pub struct MotionBody {
    pub packed: felt252,
    pub position: Pose2,
    pub next_position: Option<Pose2>,
    pub local_mprops: MassProperties,
    pub max_extent: Fixed,
    pub vels: RigidBodyVelocity,
    pub time_since_can_sleep: Fixed,
    pub extras: Option<MotionExtras>,
}

/// All the solve reads of a touching pair: its solver data but its bodies' handles (their
/// slots plus one in `bodies`, `slot1 · 2^32 + slot2`, 0 for a side without body) and user data,
/// the second solver contact only when there are two, its point count and the warm-start
/// impulses (normal, tangent) of its two points.
#[derive(Copy, Drop, Serde)]
pub struct TouchingManifold {
    pub bodies: felt252,
    pub solver_flags: SolverFlags,
    pub normal: Vec2,
    pub first: SolverContact,
    pub second: Option<SolverContact>,
    pub relative_dominance: i16,
    pub friction: Fixed,
    pub restitution: Fixed,
    pub num_points: u8,
    pub warmstart: [(Fixed, Fixed); 2],
}

/// What the stage writes of a moved body, but its world mass properties (recomputed at the new
/// pose by the caller): its pose (`position` and `next_position`), velocities and sleep timer
/// (`activation.time_since_can_sleep`).
#[derive(Copy, Drop, Serde)]
pub struct Motion {
    pub pose: Pose2,
    pub vels: RigidBodyVelocity,
    pub time_since_can_sleep: Fixed,
}

const TWO_POW_32: felt252 = 0x100000000;

/// `body_type` as two bits.
#[inline(always)]
fn type_code(body_type: RigidBodyType) -> u8 {
    match body_type {
        RigidBodyType::Dynamic => 0,
        RigidBodyType::Fixed => 1,
        RigidBodyType::KinematicPositionBased => 2,
        RigidBodyType::KinematicVelocityBased => 3,
    }
}

#[inline(always)]
fn type_of(code: u64) -> RigidBodyType {
    if code == 0 {
        RigidBodyType::Dynamic
    } else if code == 1 {
        RigidBodyType::Fixed
    } else if code == 2 {
        RigidBodyType::KinematicPositionBased
    } else {
        RigidBodyType::KinematicVelocityBased
    }
}

/// `body` at `handle` as it crosses.
#[inline(always)]
fn motion_body(handle: Handle, body: @RigidBody, constrained: bool) -> MotionBody {
    let body_type = *body.body_type;
    let mut packed: felt252 = handle.index.into() * 8192
        + (*body.mprops.flags.bits).into() * 32
        + type_code(body_type).into() * 8;
    if constrained {
        packed += 4;
    }
    if *body.enabled {
        packed += 2;
    }
    if *body.activation.sleeping {
        packed += 1;
    }
    let extras = MotionExtras {
        damping: *body.damping,
        forces: *body.forces,
        normalized_linear_threshold: *body.activation.normalized_linear_threshold,
        angular_threshold: *body.activation.angular_threshold,
    };
    MotionBody {
        packed,
        position: *body.pos.position,
        next_position: if body_type == RigidBodyType::KinematicPositionBased {
            Some(*body.pos.next_position)
        } else {
            None
        },
        local_mprops: *body.mprops.local_mprops,
        max_extent: *body.mprops.max_extent,
        vels: *body.vels,
        time_since_can_sleep: *body.activation.time_since_can_sleep,
        extras: if extras == default_extras() {
            None
        } else {
            Some(extras)
        },
    }
}

/// A body the stage reads as it reads the original, at its slot (generation 0): the fields
/// `MotionBody` leaves out are the defaults, which the stage does not read (colliders, change
/// flags, dominance, cold data, `time_until_sleep`) or recomputes (world mass properties, and
/// `next_position` but for a position-based kinematic body).
#[inline(always)]
fn from_motion_body(body: MotionBody) -> (Handle, RigidBody, bool) {
    let packed: u64 = body.packed.try_into().unwrap();
    let (rest, low) = DivRem::div_rem(packed, 32_u64.try_into().unwrap());
    let (slot, locked) = DivRem::div_rem(rest, 256_u64.try_into().unwrap());
    let (code, low) = DivRem::div_rem(low, 8_u64.try_into().unwrap());
    let (constrained, flags) = DivRem::div_rem(low, 4_u64.try_into().unwrap());
    let extras = body.extras.unwrap_or_else(|| default_extras());
    let mut activation: RigidBodyActivation = Default::default();
    activation.normalized_linear_threshold = extras.normalized_linear_threshold;
    activation.angular_threshold = extras.angular_threshold;
    activation.time_since_can_sleep = body.time_since_can_sleep;
    activation.sleeping = flags == 1 || flags == 3;
    // A literal rather than `RigidBody::default()`, which computes world mass properties.
    let out = RigidBody {
        pos: RigidBodyPosition {
            position: body.position, next_position: body.next_position.unwrap_or(body.position),
        },
        mprops: RigidBodyMassProps {
            flags: LockedAxes { bits: locked.try_into().unwrap() },
            local_mprops: body.local_mprops,
            world_com: Default::default(),
            effective_inv_mass: Default::default(),
            effective_world_inv_inertia: Default::default(),
            max_extent: body.max_extent,
        },
        vels: body.vels,
        damping: extras.damping,
        forces: extras.forces,
        colliders: array![].span(),
        activation,
        changes: Default::default(),
        body_type: type_of(code),
        dominance: Default::default(),
        enabled: flags >= 2,
        cold: BoxTrait::new(None),
    };
    (Handle { index: slot.try_into().unwrap(), generation: 0 }, out, constrained == 1)
}

/// A side without body, or whose body is among the `fixed` slots.
#[inline(always)]
fn side_fixed(fixed: Span<u32>, side: Option<Handle>) -> bool {
    match side {
        Some(h) => {
            let mut found = false;
            for slot in fixed {
                if *slot == h.index {
                    found = true;
                    break;
                }
            }
            found
        },
        None => true,
    }
}

/// `side`'s slot plus one, 0 without a body.
#[inline(always)]
fn slot_code(side: Option<Handle>) -> felt252 {
    match side {
        Some(h) => h.index.into() + 1,
        None => 0,
    }
}

/// The body at `code` (a [`slot_code`]), generation 0.
#[inline(always)]
fn side_of(code: u64) -> Option<Handle> {
    if code == 0 {
        None
    } else {
        Some(Handle { index: (code - 1).try_into().unwrap(), generation: 0 })
    }
}

/// `pair`'s touching manifold as it crosses.
#[inline(always)]
fn touching_manifold(pair: @ContactPair) -> TouchingManifold {
    let data = *pair.manifold.data;
    let [first, second] = data.solver_contacts;
    let [p0, p1] = *pair.manifold.points;
    TouchingManifold {
        bodies: slot_code(data.rigid_body1) * TWO_POW_32 + slot_code(data.rigid_body2),
        solver_flags: data.solver_flags,
        normal: data.normal,
        first,
        second: if data.num_solver_contacts == 2 {
            Some(second)
        } else {
            None
        },
        relative_dominance: data.relative_dominance,
        friction: data.friction,
        restitution: data.restitution,
        num_points: *pair.manifold.num_points,
        warmstart: [
            (p0.data.warmstart_impulse, p0.data.warmstart_tangent_impulse),
            (p1.data.warmstart_impulse, p1.data.warmstart_tangent_impulse),
        ],
    }
}

/// A manifold the solve reads as it reads the original (default geometry and user data, which
/// it does not read; the bodies at their slots, generation 0).
#[inline(always)]
fn from_touching(touching: TouchingManifold) -> ContactManifold {
    let bodies: u128 = touching.bodies.try_into().unwrap();
    let (code1, code2) = DivRem::div_rem(bodies, 0x100000000_u128.try_into().unwrap());
    let [(w0, t0), (w1, t1)] = touching.warmstart;
    let (second, num_solver_contacts) = match touching.second {
        Some(second) => (second, 2),
        None => (Default::default(), 1),
    };
    // A literal rather than `Default::default()` then the fields.
    ContactManifold {
        points: [tracked(w0, t0), tracked(w1, t1)],
        num_points: touching.num_points,
        local_n1: Default::default(),
        local_n2: Default::default(),
        subshape1: 0,
        subshape2: 0,
        data: ContactManifoldData {
            rigid_body1: side_of(code1.try_into().unwrap()),
            rigid_body2: side_of(code2.try_into().unwrap()),
            solver_flags: touching.solver_flags,
            normal: touching.normal,
            solver_contacts: [touching.first, second],
            num_solver_contacts,
            relative_dominance: touching.relative_dominance,
            user_data: 0,
            friction: touching.friction,
            restitution: touching.restitution,
        },
    }
}

/// A point with only its warm-start impulses (all the solve reads of it).
#[inline(always)]
fn tracked(warmstart_impulse: Fixed, warmstart_tangent_impulse: Fixed) -> TrackedContact {
    TrackedContact {
        local_p1: Default::default(),
        local_p2: Default::default(),
        dist: Default::default(),
        fid1: Default::default(),
        fid2: Default::default(),
        data: ContactData {
            impulse: Default::default(),
            tangent_impulse: Default::default(),
            warmstart_impulse,
            warmstart_tangent_impulse,
        },
    }
}

/// The impulses a solve left in a manifold (`ManifoldImpulses`) as they cross: its `count`
/// points.
#[inline(always)]
fn solved_points(impulses: ManifoldImpulses) -> Span<PointImpulses> {
    if impulses.count == 0 {
        array![].span()
    } else if impulses.count == 1 {
        array![impulses.a].span()
    } else {
        array![impulses.a, impulses.b].span()
    }
}

/// `island::write_point`: a solved point's impulses into its tracked contact.
#[inline(always)]
fn write_point(ref point: TrackedContact, impulses: PointImpulses) {
    point.data.impulse = impulses.impulse;
    point.data.tangent_impulse = impulses.tangent_impulse;
    point.data.warmstart_impulse = impulses.warmstart_impulse;
    point.data.warmstart_tangent_impulse = impulses.warmstart_tangent_impulse;
}

/// What crosses into [`SolveAdvanceClass`] from the caller's sets, appended to `out` as the
/// class's arguments `touching` then `entries` (each a count, then the values): the touching
/// pairs of `pairs` ([`TouchingManifold`], pair order) and the bodies of `entries` that move or
/// that a touching pair references ([`MotionBody`], ascending slot). Counted first, then
/// serialized straight into `out` (no intermediate array: a copy costs steps per felt).
fn encode(ref out: Array<felt252>, pairs: Span<ContactPair>, entries: Span<(Handle, RigidBody)>) {
    let mut n_touching: u32 = 0;
    for pair in pairs {
        if *pair.manifold.data.num_solver_contacts != 0 {
            n_touching += 1;
        }
    }
    out.append(n_touching.into());
    let mut referenced: Felt252Dict<bool> = Default::default();
    for pair in pairs {
        if *pair.manifold.data.num_solver_contacts != 0 {
            touching_manifold(pair).serialize(ref out);
            if let Some(h) = *pair.manifold.data.rigid_body1 {
                referenced.insert(h.into(), true);
            }
            if let Some(h) = *pair.manifold.data.rigid_body2 {
                referenced.insert(h.into(), true);
            }
        }
    }
    let mut n_members: u32 = 0;
    for (handle, body) in entries {
        if moving(body) || referenced.get((*handle).into()) {
            n_members += 1;
        }
    }
    out.append(n_members.into());
    for (handle, body) in entries {
        let constrained = referenced.get((*handle).into());
        if constrained || moving(body) {
            motion_body(*handle, body, constrained).serialize(ref out);
        }
    }
}

/// The class's results written back as the stage writes them: each moving body of `entries`
/// (untracked, in order) with
/// its world mass properties at the new pose, then its parented colliders (read from `snapshot`
/// when it has them) at `pose * pos_wrt_parent`; the impulses into the touching pairs of
/// `narrow_phase.pairs`; both sets flagged modified.
fn write_back(
    motions: Span<Motion>,
    impulses: Span<Span<PointImpulses>>,
    entries: Span<(Handle, RigidBody)>,
    snapshot: Span<(Handle, Collider)>,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    ref narrow_phase: NarrowPhase,
) {
    let mut motions = motions;
    for (handle, body) in entries {
        if !moving(body) {
            continue;
        }
        let motion = *motions.pop_front().unwrap();
        let mut body = *body;
        body.pos = RigidBodyPosition { position: motion.pose, next_position: motion.pose };
        body.vels = motion.vels;
        body.activation.time_since_can_sleep = motion.time_since_can_sleep;
        body.mprops = body.mprops.update_world_mass_properties(body.body_type, motion.pose);
        let _ = bodies.set_internal(*handle, body);
        for co_handle in body.colliders {
            if let Some(mut collider) = snapshot_collider(snapshot, *co_handle, ref colliders) {
                if let Some(parent) = collider.parent {
                    collider.pos.pose = motion.pose * parent.pos_wrt_parent;
                    let _ = colliders.set_internal(*co_handle, collider);
                }
            }
        }
    }
    if !impulses.is_empty() {
        // `SolvedIslandTrait::write_impulses` of each touching pair: its solved points, by
        // contact id.
        let mut impulses = impulses;
        let mut out = array![];
        for pair in narrow_phase.pairs.span() {
            let mut pair = *pair;
            if pair.manifold.data.num_solver_contacts != 0 {
                let [mut p0, mut p1] = pair.manifold.points;
                for point in *impulses.pop_front().unwrap() {
                    if *point.contact_id == 0 {
                        write_point(ref p0, *point);
                    } else {
                        write_point(ref p1, *point);
                    }
                }
                pair.manifold.points = [p0, p1];
            }
            out.append(pair);
        }
        narrow_phase.pairs = out;
    }
    bodies.mark_modified();
    colliders.mark_modified();
}

/// The stage library-called in `SolveAdvanceClass` (at `H::solve_advance()`), once per step with
/// a moving body or a touching pair.
fn call_class<impl H: ClassHashes>(
    gravity: Vec2,
    params: IntegrationParameters,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    ref narrow_phase: NarrowPhase,
    entries: Span<(Handle, RigidBody)>,
    snapshot: Span<(Handle, Collider)>,
    sleeping: bool,
) {
    // Also when nothing moves and nothing touches (the stage then only validates the
    // parameters, which the class does): the validation's code stays out of the caller.
    let mut calldata = array![];
    gravity.serialize(ref calldata);
    params.serialize(ref calldata);
    sleeping.serialize(ref calldata);
    encode(ref calldata, narrow_phase.pairs.span(), entries);
    let mut ret = library_call_syscall(
        H::solve_advance(), selector!("solve_and_advance"), calldata.span(),
    )
        .unwrap_syscall();
    let (motions, impulses): (Span<Motion>, Span<Span<PointImpulses>>) = Serde::deserialize(ref ret)
        .expect(errors::DECODE);
    write_back(motions, impulses, entries, snapshot, ref bodies, ref colliders, ref narrow_phase);
}

/// `InProcessSolveAdvance<NoJoints>` with the stage library-called in `SolveAdvanceClass` (at
/// `H::solve_advance()`) on every step that moves a body or has a touching pair.
///
/// # Panics
/// `rapier2d::pipeline::config::errors::JOINTS` when joint entries reach the stage;
/// `errors::DECODE` when the class returns something else than its result; as the stage.
pub impl LibraryCallSolveAdvance<impl H: ClassHashes> of SolveAdvanceStage {
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
        call_class::<
            H,
        >(
            gravity,
            params,
            ref bodies,
            ref colliders,
            ref narrow_phase,
            entries,
            snapshot,
            sleeping,
        );
    }
}

/// [`LibraryCallSolveAdvance`] without a call on the steps that have no touching pair (every
/// moving body is free: `solve_and_advance_free`, in the caller).
///
/// # Panics
/// As [`LibraryCallSolveAdvance`].
pub impl HybridSolveAdvance<impl H: ClassHashes> of SolveAdvanceStage {
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
        let mut touching = false;
        for pair in narrow_phase.pairs.span() {
            if *pair.manifold.data.num_solver_contacts != 0 {
                touching = true;
                break;
            }
        }
        if !touching {
            solve_and_advance_free(
                gravity, params, ref bodies, ref colliders, entries, snapshot, false,
            );
            colliders.mark_modified();
            return;
        }
        call_class::<
            H,
        >(
            gravity,
            params,
            ref bodies,
            ref colliders,
            ref narrow_phase,
            entries,
            snapshot,
            sleeping,
        );
    }
}

/// Measurement only: [`LibraryCallSolveAdvance`] with `solve_and_advance_values` called in
/// process instead of library-called (the crossing's gather and encoding, its decoding, the
/// class's work and the write-back, without the call and the results' encoding), to split a
/// call's cost.
pub impl ValuesSolveAdvance of SolveAdvanceStage {
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
        let mut felts = array![];
        encode(ref felts, narrow_phase.pairs.span(), entries);
        let mut felts = felts.span();
        let (touching, members): (Span<TouchingManifold>, Span<MotionBody>) = Serde::deserialize(
            ref felts,
        )
            .unwrap();
        let (motions, impulses) = solve_and_advance_values(
            gravity, params, members, touching, sleeping,
        );
        write_back(
            motions, impulses, entries, snapshot, ref bodies, ref colliders, ref narrow_phase,
        );
    }
}

/// `solve_and_advance_sleeping_with::<NoJoints>` on the crossing's values (see the module
/// documentation): the [`Motion`] of each moving body of `entries` (in order) and the impulses of
/// each manifold of `touching` (in order). The same walk as the stage's: the touching manifolds
/// partitioned (D8: those with a fixed body or no body last), the bodies they reference gathered
/// into the solver input (a sleeping one as an immovable copy when `sleeping`), every other
/// moving body solved alone, then each moving body advanced and its sleep timer updated.
pub fn solve_and_advance_values(
    gravity: Vec2,
    params: IntegrationParameters,
    entries: Span<MotionBody>,
    touching: Span<TouchingManifold>,
    sleeping: bool,
) -> (Span<Motion>, Span<Span<PointImpulses>>) {
    // `ordering::fixed_last_flag`: a side is fixed, or has no body (every body a manifold
    // references crossed; a fixed body is fixed whatever its other flags). The fixed members
    // are few (the ground, static props): a scan.
    let mut fixed = array![];
    let mut bodies = array![];
    let mut member_flags = array![];
    for body in entries {
        let (handle, body, constrained) = from_motion_body(*body);
        if body.body_type == RigidBodyType::Fixed {
            fixed.append(handle.index);
        }
        bodies.append((handle, body));
        member_flags.append(constrained);
    }
    let fixed = fixed.span();
    let entries = bodies.span();
    let mut first = array![];
    let mut last = array![];
    let mut flags = array![];
    for view in touching {
        let manifold = from_touching(*view);
        let fixed_last = side_fixed(fixed, manifold.data.rigid_body1)
            || side_fixed(fixed, manifold.data.rigid_body2);
        flags.append(fixed_last);
        if fixed_last {
            last.append(manifold);
        } else {
            first.append(manifold);
        }
    }
    let n_first = first.len();
    first.append_span(last.span());
    let manifolds = first;
    let any = !manifolds.is_empty();
    let mut input = SolverInputTrait::new();
    let mut has_free = false;
    if any {
        let dt = params.substep_dt();
        let mut constrained = member_flags.span();
        for (handle, body) in entries {
            let mut body = *body;
            if *constrained.pop_front().unwrap() {
                if sleeping && body.activation.sleeping {
                    // `immovable`.
                    body.enabled = false;
                }
                input.push(*handle, body, gravity, dt);
            } else if moving(@body) {
                has_free = true;
            }
        }
    }
    let free = if !any || has_free {
        FreeBodySolverTrait::new(params, gravity)
    } else {
        Default::default()
    };
    let mut impulses = array![];
    let mut solved: SolvedIsland = Default::default();
    if any {
        solved = solve_island_input_contacts(params, input, manifolds.span());
        // The impulses of each touching manifold, in pair order (`scatter_impulses`' indices).
        let (mut first, mut rest) = (0, n_first);
        for fixed_last in flags.span() {
            let index = if *fixed_last {
                rest += 1;
                rest - 1
            } else {
                first += 1;
                first - 1
            };
            impulses
                .append(
                    match solved.impulses.get(index) {
                        Some(found) => solved_points(*found.unbox()),
                        None => array![].span(),
                    },
                );
        }
    }
    let mut dense: u32 = 0;
    // The stage's members: a constraint references them, when there is one.
    let mut member_flags = if any {
        member_flags.span()
    } else {
        array![].span()
    };
    let mut motions = array![];
    for (handle, body) in entries {
        let member = match member_flags.pop_front() {
            Some(member) => *member,
            None => false,
        };
        let body = *body;
        if moving(@body) {
            let mut body = if member {
                let mut body = body;
                solved.write_body(dense, ref body);
                body
            } else {
                free.solve(*handle, body)
            };
            // `advance_body_with_snapshot`, but the world mass properties and the colliders.
            let previous = body.pos.position;
            body.pos.position = body.pos.next_position;
            update_sleep_timer(ref body, previous, params);
            motions
                .append(
                    Motion {
                        pose: body.pos.position,
                        vels: body.vels,
                        time_since_can_sleep: body.activation.time_since_can_sleep,
                    },
                );
        }
        if member {
            dense += 1;
        }
    }
    (motions.span(), impulses.span())
}

/// The fused solve and position update of the contacts (no joint).
#[starknet::contract]
pub mod SolveAdvanceClass {
    use rapier2d::prelude::{IntegrationParameters, Vec2};
    use rapier_dynamics2d::solver::island::PointImpulses;
    use super::{Motion, MotionBody, TouchingManifold};

    #[storage]
    struct Storage {}

    /// [`super::solve_and_advance_values`].
    #[external(v0)]
    fn solve_and_advance(
        self: @ContractState,
        gravity: Vec2,
        params: IntegrationParameters,
        sleeping: bool,
        touching: Span<TouchingManifold>,
        entries: Span<MotionBody>,
    ) -> (Span<Motion>, Span<Span<PointImpulses>>) {
        super::solve_and_advance_values(gravity, params, entries, touching, sleeping)
    }
}
