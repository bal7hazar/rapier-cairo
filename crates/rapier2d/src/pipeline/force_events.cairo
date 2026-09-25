//! Post-solver force events, in ascending collider-pair order.
use fixed::{Fixed, FixedTrait, ZERO};
use rapier_core::collider::ActiveEventsTrait;
use rapier_core::collider::events::CONTACT_FORCE_EVENTS;
use rapier_dynamics2d::collider::Collider;
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::events::{CollisionEvent, ContactForceEvent, ContactForceEventTrait};
use rapier_dynamics2d::narrow_phase::NarrowPhase;


/// Specialize only the return shape: both modes execute the same stages and event bookkeeping.
pub(crate) trait StepOutput<T> {
    fn finish(
        events: Array<CollisionEvent>,
        enabled: bool,
        dt: Fixed,
        ref narrow: NarrowPhase,
        ref colliders: ColliderSet,
    ) -> T;
}

pub(crate) impl CollisionOnly of StepOutput<Array<CollisionEvent>> {
    #[inline(always)]
    fn finish(
        events: Array<CollisionEvent>,
        enabled: bool,
        dt: Fixed,
        ref narrow: NarrowPhase,
        ref colliders: ColliderSet,
    ) -> Array<CollisionEvent> {
        if enabled {
            let _ = collect(dt, ref narrow, ref colliders);
        }
        events
    }
}

pub(crate) impl WithForces of StepOutput<(Array<CollisionEvent>, Array<ContactForceEvent>)> {
    #[inline(always)]
    fn finish(
        events: Array<CollisionEvent>,
        enabled: bool,
        dt: Fixed,
        ref narrow: NarrowPhase,
        ref colliders: ColliderSet,
    ) -> (Array<CollisionEvent>, Array<ContactForceEvent>) {
        let forces = dispatch(enabled, dt, ref narrow, ref colliders);
        (events, forces)
    }
}

/// Inlined dispatch with only the changed sets crossing the branch merge.
#[inline(always)]
pub(crate) fn dispatch(
    enabled: bool, dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
) -> Array<ContactForceEvent> {
    if enabled {
        collect(dt, ref narrow, ref colliders)
    } else {
        array![]
    }
}

#[cfg(test)]
/// Refund boundary for the optional event branch, as the JM solver gas-wallet pattern.
#[inline(never)]
pub(crate) fn gas_wallet() {
    let mut pending = false;
    while pending {
        pending = false;
    }
}

fn threshold(collider: Collider) -> Fixed {
    if collider.flags.active_events.contains(CONTACT_FORCE_EVENTS) {
        collider.contact_force_event_threshold
    } else {
        Fixed { raw: 9223372036854775807 }
    }
}

/// Emits normal-force events after impulse writeback, preserving the collision-event bit.
/// Only enabled sides contribute thresholds (minimum when both enable); strict `>`.
/// `dt == 0` means zero force, as upstream's safe inverse. Division rounds nearest-even.
/// Panics on fixed overflow. Called only when a collider has enabled force events.
pub fn collect(
    dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
) -> Array<ContactForceEvent> {
    let inv_dt = if dt == ZERO {
        ZERO
    } else {
        dt.recip()
    };
    let mut events = array![];
    let mut pairs = array![];
    for old in narrow.pairs.span() {
        let mut pair = *old;
        let co1 = colliders.get(pair.collider1).unwrap();
        let co2 = colliders.get(pair.collider2).unwrap();
        if (co1.flags.active_events | co2.flags.active_events).contains(CONTACT_FORCE_EVENTS) {
            let limit = threshold(co1).min(threshold(co2));
            let magnitude = if pair.manifold.data.num_solver_contacts == 0
                || pair.manifold.data.solver_flags.bits == 0 {
                ZERO
            } else {
                let [a, b] = pair.manifold.points;
                let first = if pair.manifold.num_points != 0 {
                    a.data.impulse
                } else {
                    ZERO
                };
                let second = if pair.manifold.num_points > 1 {
                    b.data.impulse
                } else {
                    ZERO
                };
                (first + second) * inv_dt
            };
            if magnitude > limit
                && pair.manifold.data.num_solver_contacts != 0
                && pair.manifold.data.solver_flags.bits != 0 {
                events.append(ContactForceEventTrait::from_contact_pair(dt, @pair, magnitude));
                pair.event_status.bits = pair.event_status.bits | 2;
            } else {
                pair.event_status.bits = pair.event_status.bits & 253;
            }
        } else {
            pair.event_status.bits = pair.event_status.bits & 253;
        }
        pairs.append(pair);
    }
    narrow.pairs = pairs;
    events
}

#[cfg(test)]
mod tests {
    use fixed::{HALF, ONE, ZERO};
    use glam::Vec2;
    use rapier_dynamics2d::collider::ColliderBuilderTrait;
    use rapier_dynamics2d::narrow_phase::{ContactPairTrait, NarrowPhaseTrait};
    use rapier_dynamics2d::rigid_body_set::RigidBodyTrait;
    use rapier_testing::opaque;
    use crate::world::{World, WorldTrait};
    use super::*;

    fn fixture(limit: Fixed) -> (NarrowPhase, ColliderSet) {
        let mut colliders = ColliderSetTrait::new();
        let a = colliders
            .insert(
                ColliderBuilderTrait::ball(HALF)
                    .active_events(CONTACT_FORCE_EVENTS)
                    .contact_force_event_threshold(limit)
                    .build(),
            );
        let b = colliders.insert(ColliderBuilderTrait::ball(HALF).build());
        let mut pair = ContactPairTrait::new(a, b);
        pair.manifold.data.normal = Vec2 { x: ZERO, y: ONE };
        pair.manifold.num_points = 1;
        pair.manifold.data.num_solver_contacts = 1;
        pair.manifold.data.solver_flags.bits = 1;
        let [mut point, other] = pair.manifold.points;
        point.data.impulse = ONE;
        point.data.tangent_impulse = ONE;
        pair.manifold.points = [point, other];
        let mut narrow = NarrowPhaseTrait::new();
        narrow.pairs.append(pair);
        (narrow, colliders)
    }

    #[test]
    fn test_strict_threshold_zero_dt_and_crossings() {
        let (mut narrow, mut colliders) = fixture(ONE);
        assert!(collect(ONE, ref narrow, ref colliders).is_empty());
        let event = collect(HALF, ref narrow, ref colliders);
        assert!(*event.at(0).started);
        assert_eq!(*event.at(0).total_force_magnitude, ONE + ONE);
        let event = collect(HALF, ref narrow, ref colliders);
        assert!(!*event.at(0).started);
        assert!(collect(ZERO, ref narrow, ref colliders).is_empty());
        let event = collect(HALF, ref narrow, ref colliders);
        assert!(*event.at(0).started);
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }
    #[test]
    fn gas_collect() {
        let (mut narrow, mut colliders) = fixture(opaque(ZERO));
        let _ = collect(opaque(ONE), ref narrow, ref colliders);
    }
    fn world_fixture() -> World {
        let (narrow, colliders) = fixture(opaque(ZERO));
        let mut world = WorldTrait::new(Vec2 { x: ZERO, y: ZERO }, Default::default());
        world.integration_parameters.dt = opaque(ONE);
        world.narrow_phase = narrow;
        world.colliders = colliders;
        world
    }
    #[test]
    fn gas_tail_reduced_off() {
        let mut w = world_fixture();
        let events = super::alternatives::collect_if_enabled(
            opaque(false), w.integration_parameters.dt, ref w.narrow_phase, ref w.colliders,
        );
        assert!(events.is_empty());
    }
    #[test]
    fn gas_tail_world_off() {
        let mut w = world_fixture();
        let events = super::alternatives::collect_world(ref w, opaque(false));
        assert!(events.is_empty());
    }
    #[test]
    fn gas_tail_reduced_on() {
        let mut w = world_fixture();
        let events = super::alternatives::collect_if_enabled(
            opaque(true), w.integration_parameters.dt, ref w.narrow_phase, ref w.colliders,
        );
        assert_eq!(events.len(), 1);
    }
    #[test]
    fn gas_tail_world_on() {
        let mut w = world_fixture();
        let events = super::alternatives::collect_world(ref w, opaque(true));
        assert_eq!(events.len(), 1);
    }
    fn falling() -> World {
        let mut w = WorldTrait::new(Vec2 { x: ZERO, y: opaque(-ONE) }, Default::default());
        w
            .insert(
                RigidBodyTrait::dynamic(Default::default()),
                ColliderBuilderTrait::ball(HALF).build(),
            );
        w.step();
        w.step();
        w
    }
    #[test]
    fn gas_step_dispatch_setup() {
        let _ = falling();
    }
    #[test]
    fn gas_step_dispatch_collision_only() {
        let mut w = falling();
        let _ = w.step();
    }
    #[test]
    fn gas_step_dispatch_shipped() {
        let mut w = falling();
        let _ = w.step_with_force_events();
    }
    #[test]
    fn gas_step_dispatch_wallet() {
        let mut w = falling();
        let _ = super::alternatives::step_wallet(ref w);
    }
    #[test]
    fn gas_step_dispatch_world() {
        let mut w = falling();
        let _ = super::alternatives::step_world(ref w);
    }
    #[test]
    fn gas_step_dispatch_reduced() {
        let mut w = falling();
        let _ = super::alternatives::step_reduced(ref w);
    }
    #[test]
    fn gas_step_dispatch_unwalleted() {
        let mut w = falling();
        let _ = super::alternatives::step_unwalleted(ref w);
    }
    #[test]
    fn test_step_dispatch_variants() {
        let mut a = falling();
        let mut b = falling();
        let mut c = falling();
        let mut d = falling();
        let _ = a.step_with_force_events();
        let _ = super::alternatives::step_world(ref b);
        let _ = super::alternatives::step_reduced(ref c);
        let _ = super::alternatives::step_unwalleted(ref d);
        let h = rapier_core::Handle { index: 0, generation: 0 };
        let expected = a.body(h);
        assert_eq!(b.body(h), expected);
        assert_eq!(c.body(h), expected);
        assert_eq!(d.body(h), expected);
    }
}

#[cfg(test)]
mod alternatives;
