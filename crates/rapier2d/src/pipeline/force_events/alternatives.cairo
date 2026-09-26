//! Rejected force-event dispatch candidates, including their complete step context.
use rapier_core::integration_parameters::IntegrationParametersTrait;
use rapier_dynamics2d::events::CollisionEvent;
use rapier_dynamics2d::joint::ImpulseJointSetTrait;
use rapier_dynamics2d::narrow_phase::compute_contacts_from_scratch;
use rapier_geometry2d::broad_phase::find_pairs;
use crate::dispatcher::DefaultDispatcher;
use crate::pipeline::{
    collision_inputs_with_events, force_events, merge_pairs, solve_and_advance_sleeping,
    split_dormant, update_islands, user_changes_bodies_for_step,
};
use crate::world::World;
use super::{ColliderSet, ContactForceEvent, Fixed, NarrowPhase, collect};
/// Keeps the metered branch's live state to the collider and pair sets, not the whole World.
#[inline(never)]
pub fn collect_if_enabled(
    enabled: bool, dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
) -> Array<ContactForceEvent> {
    let mut events = array![];
    let mut pending = enabled;
    while pending {
        events = collect(dt, ref narrow, ref colliders);
        pending = false;
    }
    events
}
/// Initial candidate: the metered loop carries the full World across its back edge.
#[inline(never)]
pub fn collect_world(ref world: World, enabled: bool) -> Array<ContactForceEvent> {
    let mut events = array![];
    let mut pending = enabled;
    while pending {
        events =
            collect(world.integration_parameters.dt, ref world.narrow_phase, ref world.colliders);
        pending = false;
    }
    events
}

/// Whole-world metering at the original call site.
pub fn step_world(ref world: World) -> (Array<CollisionEvent>, Array<ContactForceEvent>) {
    let (snapshot, infos, entries, census, _) = user_changes_bodies_for_step(
        ref world.bodies,
        ref world.colliders,
        world.narrow_phase.pairs.span(),
        Some(world.integration_parameters.dt),
        true,
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
    let mut forces = array![];
    let mut pending = force_events;
    while pending {
        forces =
            force_events::collect(
                world.integration_parameters.dt, ref world.narrow_phase, ref world.colliders,
            );
        pending = false;
    }
    if !dormant.is_empty() {
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), dormant.span());
    }
    (events, forces)
}

/// Outlined reduced-argument metering at the original call site.
pub fn step_reduced(ref world: World) -> (Array<CollisionEvent>, Array<ContactForceEvent>) {
    let (snapshot, infos, entries, census, _) = user_changes_bodies_for_step(
        ref world.bodies,
        ref world.colliders,
        world.narrow_phase.pairs.span(),
        Some(world.integration_parameters.dt),
        true,
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
    let forces = collect_if_enabled(
        force_events, world.integration_parameters.dt, ref world.narrow_phase, ref world.colliders,
    );
    if !dormant.is_empty() {
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), dormant.span());
    }
    (events, forces)
}

/// Direct conditional without an explicit refund boundary.
pub fn step_unwalleted(ref world: World) -> (Array<CollisionEvent>, Array<ContactForceEvent>) {
    let (snapshot, infos, entries, census, _) = user_changes_bodies_for_step(
        ref world.bodies,
        ref world.colliders,
        world.narrow_phase.pairs.span(),
        Some(world.integration_parameters.dt),
        true,
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
    let forces = if force_events {
        force_events::collect(
            world.integration_parameters.dt, ref world.narrow_phase, ref world.colliders,
        )
    } else {
        array![]
    };
    if !dormant.is_empty() {
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), dormant.span());
    }
    (events, forces)
}

/// Direct conditional with an explicit gas wallet.
pub fn step_wallet(ref world: World) -> (Array<CollisionEvent>, Array<ContactForceEvent>) {
    let (snapshot, infos, entries, census, _) = user_changes_bodies_for_step(
        ref world.bodies,
        ref world.colliders,
        world.narrow_phase.pairs.span(),
        Some(world.integration_parameters.dt),
        true,
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
    force_events::gas_wallet();
    let forces = if force_events {
        force_events::collect(
            world.integration_parameters.dt, ref world.narrow_phase, ref world.colliders,
        )
    } else {
        array![]
    };
    if !dormant.is_empty() {
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), dormant.span());
    }
    (events, forces)
}

/// Tied candidate: capture a unit payload before merging, assemble the return afterwards.
trait UnitOutput<T, Forces> {
    fn capture(
        enabled: bool, dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
    ) -> Forces;
    fn finish(events: Array<CollisionEvent>, forces: Forces) -> T;
}

impl UnitOnly of UnitOutput<Array<CollisionEvent>, ()> {
    #[inline(always)]
    fn capture(enabled: bool, dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet) {
        if enabled {
            let _ = collect(dt, ref narrow, ref colliders);
        }
    }
    fn finish(events: Array<CollisionEvent>, forces: ()) -> Array<CollisionEvent> {
        events
    }
}

fn step_unit_internal<T, Forces, impl Output: UnitOutput<T, Forces>, +Drop<Forces>>(
    ref world: World,
) -> T {
    let (snapshot, infos, entries, census, _) = user_changes_bodies_for_step(
        ref world.bodies,
        ref world.colliders,
        world.narrow_phase.pairs.span(),
        Some(world.integration_parameters.dt),
        true,
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
    let forces = Output::capture(
        force_events, world.integration_parameters.dt, ref world.narrow_phase, ref world.colliders,
    );
    if !dormant.is_empty() {
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), dormant.span());
    }
    Output::finish(events, forces)
}

pub fn step_unit_payload(ref world: World) -> Array<CollisionEvent> {
    step_unit_internal::<Array<CollisionEvent>, (), UnitOnly>(ref world)
}
