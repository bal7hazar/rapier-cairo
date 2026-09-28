//! The contact-force event collection (`StageConfig::Forces`) in a declared class (CS6, route
//! (a)).
//!
//! `rapier2d::pipeline::force_events::collect_convex` reads, of each pair, its colliders' event
//! flags and force thresholds, and of its manifold the solver contact count and flags, the point
//! count, the two normal impulses and the normal; it writes the pair's event status. Only those
//! cross ([`ForceCollider`], [`ForcePair`]); `ForceEventsClass` puts them back into placeholder
//! colliders and pairs, runs the same `collect_convex` and returns the events and the new
//! statuses, which the caller writes into its pairs.

use rapier2d::pipeline::stages::ForceEventStage;
use rapier2d::prelude::{ContactForceEvent, Fixed, Handle, Vec2};
use rapier_core::collider::ActiveEvents;
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::events::PairEventStatus;
use rapier_dynamics2d::narrow_phase::NarrowPhase;
use rapier_geometry2d::contact::SolverFlags;
use starknet::SyscallResultTrait;
use starknet::syscalls::library_call_syscall;
use crate::hashes::{ClassHashes, errors};

/// What `collect_convex` reads of a collider.
#[derive(Copy, Drop, Serde)]
pub struct ForceCollider {
    pub handle: Handle,
    pub active_events: ActiveEvents,
    pub threshold: Fixed,
}

/// What `collect_convex` reads of a pair.
#[derive(Copy, Drop, Serde)]
pub struct ForcePair {
    pub collider1: Handle,
    pub collider2: Handle,
    pub num_solver_contacts: u8,
    pub solver_flags: SolverFlags,
    pub num_points: u8,
    pub impulse1: Fixed,
    pub impulse2: Fixed,
    pub normal: Vec2,
    pub event_status: PairEventStatus,
}

/// Errors of the force-event stage.
pub mod forces_errors {
    /// A configuration with composite shapes (`groups`) library-called the convex collection.
    pub const GROUPS: felt252 = 'Forces: no composite groups';
}

/// The collection library-called in `ForceEventsClass` (at `H::force_events()`), once per step
/// when a collider enables contact-force events. Same results as `InProcessForceEvents` without
/// composite shapes.
///
/// # Panics
/// [`forces_errors::GROUPS`] when `groups` (a configuration with composite shapes);
/// `errors::DECODE` when the class returns something else than its result.
pub impl LibraryCallForceEvents<impl H: ClassHashes> of ForceEventStage {
    fn collect(
        groups: bool, dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
    ) -> Array<ContactForceEvent> {
        assert(!groups, forces_errors::GROUPS);
        let mut calldata = array![];
        dt.serialize(ref calldata);
        let all = colliders.iter();
        calldata.append(all.len().into());
        for (handle, collider) in all {
            ForceCollider {
                handle,
                active_events: collider.flags.active_events,
                threshold: collider.contact_force_event_threshold,
            }
                .serialize(ref calldata);
        }
        calldata.append(narrow.pairs.len().into());
        for pair in narrow.pairs.span() {
            let [a, b] = *pair.manifold.points;
            ForcePair {
                collider1: *pair.collider1,
                collider2: *pair.collider2,
                num_solver_contacts: *pair.manifold.data.num_solver_contacts,
                solver_flags: *pair.manifold.data.solver_flags,
                num_points: *pair.manifold.num_points,
                impulse1: a.data.impulse,
                impulse2: b.data.impulse,
                normal: *pair.manifold.data.normal,
                event_status: *pair.event_status,
            }
                .serialize(ref calldata);
        }
        let mut ret = library_call_syscall(H::force_events(), selector!("collect"), calldata.span())
            .unwrap_syscall();
        let (events, mut statuses): (Array<ContactForceEvent>, Span<PairEventStatus>) =
            Serde::deserialize(
            ref ret,
        )
            .expect(errors::DECODE);
        let mut pairs = array![];
        for pair in narrow.pairs.span() {
            let mut pair = *pair;
            pair.event_status = *statuses.pop_front().expect(errors::DECODE);
            pairs.append(pair);
        }
        narrow.pairs = pairs;
        events
    }
}

/// The contact-force events of the convex pairs (`force_events::collect_convex`).
#[starknet::contract]
pub mod ForceEventsClass {
    use rapier2d::pipeline::force_events::collect_convex;
    use rapier2d::prelude::{ContactForceEvent, Fixed, Handle};
    use rapier_dynamics2d::collider::{Collider, ColliderBuilderTrait};
    use rapier_dynamics2d::collider_set::ColliderSetTrait;
    use rapier_dynamics2d::events::PairEventStatus;
    use rapier_dynamics2d::narrow_phase::{ContactPair, ContactPairTrait, NarrowPhase};
    use super::{ForceCollider, ForcePair};

    #[storage]
    struct Storage {}

    /// `collect_convex` on placeholders holding what crossed: the force events and each pair's
    /// new event status, in pair order.
    #[external(v0)]
    fn collect(
        self: @ContractState, dt: Fixed, colliders: Span<ForceCollider>, pairs: Span<ForcePair>,
    ) -> (Array<ContactForceEvent>, Span<PairEventStatus>) {
        let placeholder = ColliderBuilderTrait::ball(fixed::ONE).build();
        let mut entries: Array<(Handle, Collider)> = array![];
        for collider in colliders {
            let mut value = placeholder;
            value.flags.active_events = *collider.active_events;
            value.contact_force_event_threshold = *collider.threshold;
            entries.append((*collider.handle, value));
        }
        let mut set = ColliderSetTrait::from_state(crate::arena::partial_state(entries.span()));
        let mut list: Array<ContactPair> = array![];
        for pair in pairs {
            let mut value = ContactPairTrait::new(*pair.collider1, *pair.collider2);
            let [mut a, mut b] = value.manifold.points;
            a.data.impulse = *pair.impulse1;
            b.data.impulse = *pair.impulse2;
            value.manifold.points = [a, b];
            value.manifold.num_points = *pair.num_points;
            value.manifold.data.num_solver_contacts = *pair.num_solver_contacts;
            value.manifold.data.solver_flags = *pair.solver_flags;
            value.manifold.data.normal = *pair.normal;
            value.event_status = *pair.event_status;
            list.append(value);
        }
        let mut narrow = NarrowPhase { pairs: list };
        let events = collect_convex(dt, ref narrow, ref set);
        let mut statuses = array![];
        for pair in narrow.pairs.span() {
            statuses.append(*pair.event_status);
        }
        (events, statuses.span())
    }
}
