//! Collision events (upstream `geometry::CollisionEvent`) and the per-pair event bookkeeping
//! (upstream `geometry::contact_pair::PairEventStatus`).
//!
//! Upstream hands events to a `dyn EventHandler` as they happen; here the narrow phase returns
//! them as an array (`docs/PLAN.md` D9), in a deterministic order documented on
//! `NarrowPhaseTrait::compute_contacts`. Deferred: contact-force events and sensor intersection
//! events.

use rapier_core::Handle;
use rapier_core::collider::events::{REMOVED, SENSOR};
use rapier_core::collider::{CollisionEventFlags, CollisionEventFlagsTrait};

/// Two colliders started or stopped touching (upstream `CollisionEvent`).
///
/// The handles are the pair's `(collider1, collider2)`, in ascending slot index.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum CollisionEvent {
    /// The pair gained its first solver contact.
    Started: (Handle, Handle, CollisionEventFlags),
    /// The pair lost its last solver contact, stopped overlapping in the broad phase, or lost a
    /// collider (then with the `REMOVED` flag).
    Stopped: (Handle, Handle, CollisionEventFlags),
}

/// Accessors of [`CollisionEvent`] (upstream names).
#[generate_trait]
pub impl CollisionEventImpl of CollisionEventTrait {
    /// `true` for `Started`.
    #[inline(always)]
    fn started(self: CollisionEvent) -> bool {
        match self {
            CollisionEvent::Started(_) => true,
            CollisionEvent::Stopped(_) => false,
        }
    }

    /// `true` for `Stopped`.
    #[inline(always)]
    fn stopped(self: CollisionEvent) -> bool {
        !self.started()
    }

    /// The first collider of the pair.
    #[inline(always)]
    fn collider1(self: CollisionEvent) -> Handle {
        let (h, _, _) = self.parts();
        h
    }

    /// The second collider of the pair.
    #[inline(always)]
    fn collider2(self: CollisionEvent) -> Handle {
        let (_, h, _) = self.parts();
        h
    }

    /// `true` when one of the colliders is a sensor.
    #[inline(always)]
    fn sensor(self: CollisionEvent) -> bool {
        let (_, _, flags) = self.parts();
        flags.contains(SENSOR)
    }

    /// `true` when the event was caused by the removal of a collider.
    #[inline(always)]
    fn removed(self: CollisionEvent) -> bool {
        let (_, _, flags) = self.parts();
        flags.contains(REMOVED)
    }

    /// `(collider1, collider2, flags)`, whatever the variant.
    #[inline(always)]
    fn parts(self: CollisionEvent) -> (Handle, Handle, CollisionEventFlags) {
        match self {
            CollisionEvent::Started(parts) => parts,
            CollisionEvent::Stopped(parts) => parts,
        }
    }
}

/// Bit of [`PairEventStatus`]: a `CollisionEvent::Started` was emitted for the pair.
pub const START_EVENT_EMITTED: PairEventStatus = PairEventStatus { bits: 0x1 };

/// Event bookkeeping of a contact pair (upstream `PairEventStatus`). Only
/// `START_EVENT_EMITTED` is used; upstream's `INITIAL_FORCE_THRESHOLD_EVENT_EMITTED` (`0x2`)
/// belongs to the deferred contact-force events.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct PairEventStatus {
    pub bits: u8,
}

/// Set operations on [`PairEventStatus`].
#[generate_trait]
pub impl PairEventStatusImpl of PairEventStatusTrait {
    /// No bit set.
    #[inline(always)]
    fn empty() -> PairEventStatus {
        PairEventStatus { bits: 0 }
    }

    /// `true` when a `Started` event was emitted and no `Stopped` followed.
    #[inline(always)]
    fn start_event_emitted(self: PairEventStatus) -> bool {
        self.bits != 0
    }
}

/// `CollisionEvent::Started(collider1, collider2, empty)`.
#[inline(always)]
pub fn started(collider1: Handle, collider2: Handle) -> CollisionEvent {
    CollisionEvent::Started((collider1, collider2, CollisionEventFlagsTrait::empty()))
}

/// `CollisionEvent::Stopped(collider1, collider2, flags)`.
#[inline(always)]
pub fn stopped(collider1: Handle, collider2: Handle, flags: CollisionEventFlags) -> CollisionEvent {
    CollisionEvent::Stopped((collider1, collider2, flags))
}

#[cfg(test)]
mod tests {
    use rapier_core::Handle;
    use rapier_core::collider::events::{REMOVED, SENSOR};
    use rapier_core::collider::{CollisionEventFlags, CollisionEventFlagsTrait};
    use rapier_testing::opaque;
    use super::{
        CollisionEvent, CollisionEventTrait, PairEventStatus, PairEventStatusTrait,
        START_EVENT_EMITTED, started, stopped,
    };

    fn h(index: u32) -> Handle {
        Handle { index, generation: 0 }
    }

    #[test]
    fn test_accessors() {
        // (event, started, sensor, removed)
        let cases: Array<(CollisionEvent, bool, bool, bool)> = array![
            (started(h(1), h(2)), true, false, false),
            (stopped(h(1), h(2), CollisionEventFlagsTrait::empty()), false, false, false),
            (stopped(h(1), h(2), REMOVED), false, false, true),
            (CollisionEvent::Started((h(1), h(2), SENSOR)), true, true, false),
            (stopped(h(1), h(2), SENSOR | REMOVED), false, true, true),
        ];
        for (event, is_started, is_sensor, is_removed) in cases {
            assert_eq!(event.started(), is_started);
            assert_eq!(event.stopped(), !is_started);
            assert_eq!(event.sensor(), is_sensor);
            assert_eq!(event.removed(), is_removed);
            assert_eq!(event.collider1(), h(1));
            assert_eq!(event.collider2(), h(2));
        }
    }

    #[test]
    fn test_pair_event_status() {
        let status: PairEventStatus = Default::default();
        assert!(!status.start_event_emitted());
        assert!(START_EVENT_EMITTED.start_event_emitted());
        assert_eq!(PairEventStatusTrait::empty(), status);
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(h(1));
    }

    #[test]
    fn gas_started() {
        let _ = opaque(started(opaque(h(1)), h(2)));
    }

    #[test]
    fn gas_stopped_accessors() {
        let flags: CollisionEventFlags = opaque(REMOVED);
        let e = opaque(stopped(h(1), h(2), flags));
        assert!(e.stopped() && e.removed() && !e.sensor());
        assert_eq!(e.collider2(), h(2));
    }
}
