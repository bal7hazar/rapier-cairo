//! Post-solver force events, in ascending collider-pair order.
use fixed::{Fixed, FixedTrait, ZERO};
use rapier_core::collider::ActiveEventsTrait;
use rapier_core::collider::events::CONTACT_FORCE_EVENTS;
use rapier_dynamics2d::collider::Collider;
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::events::{ContactForceEvent, ContactForceEventTrait};
use rapier_dynamics2d::narrow_phase::NarrowPhase;

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
    use rapier_testing::opaque;
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
}
