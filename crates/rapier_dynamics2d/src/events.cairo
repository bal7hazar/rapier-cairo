//! Collision events (upstream `geometry::CollisionEvent`) and the per-pair event bookkeeping
//! (upstream `geometry::contact_pair::PairEventStatus`).
//!
//! Upstream hands events to a `dyn EventHandler` as they happen; here the narrow phase returns
//! them as an array (`docs/PLAN.md` D9), in a deterministic order documented on
//! `NarrowPhaseTrait::compute_contacts`. Sensor intersection events remain deferred.

use fixed::{FixedTrait, ZERO};
use glam::Vec2Trait;
use rapier_core::Handle;
use rapier_core::collider::events::{REMOVED, SENSOR};
use rapier_core::collider::{CollisionEventFlags, CollisionEventFlagsTrait};

/// A post-solver normal-force event (upstream `ContactForceEvent`).
/// Tangential/friction impulses are intentionally excluded, as upstream.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct ContactForceEvent {
    /// First collider, in ascending pair order.
    pub collider1: Handle,
    /// Second collider, in ascending pair order.
    pub collider2: Handle,
    /// Vector sum of normal forces, in world coordinates.
    pub total_force: glam::Vec2,
    /// Sum of individual normal-force magnitudes, not the length of `total_force`.
    pub total_force_magnitude: fixed::Fixed,
    /// World unit normal at the strongest contact, or zero when every impulse is zero.
    pub max_force_direction: glam::Vec2,
    /// Strongest individual normal impulse divided by dt.
    pub max_force_magnitude: fixed::Fixed,
    /// True on the first step above threshold, reset at or below it or on separation.
    pub started: bool,
}

#[generate_trait]
pub impl ContactForceEventImpl of ContactForceEventTrait {
    /// Builds an event from solved normal impulses; `total_force_magnitude` is already
    /// a force. One convex manifold, at most two points. Zero dt gives zero force.
    /// Reciprocal rounds nearest-even, products floor; panics on fixed overflow.
    fn from_contact_pair(
        dt: fixed::Fixed,
        pair: @crate::narrow_phase::ContactPair,
        total_force_magnitude: fixed::Fixed,
    ) -> ContactForceEvent {
        let inv_dt = if dt == ZERO {
            ZERO
        } else {
            dt.recip()
        };
        let [a, b] = *pair.manifold.points;
        let first = if *pair.manifold.num_points != 0 {
            a.data.impulse
        } else {
            ZERO
        };
        let second = if *pair.manifold.num_points > 1 {
            b.data.impulse
        } else {
            ZERO
        };
        let maximum = first.max(second).max(ZERO);
        let normal = *pair.manifold.data.normal;
        ContactForceEvent {
            collider1: *pair.collider1,
            collider2: *pair.collider2,
            total_force: normal.mul_scalar(first + second).mul_scalar(inv_dt),
            total_force_magnitude,
            max_force_direction: if maximum > ZERO {
                normal
            } else {
                Default::default()
            },
            max_force_magnitude: maximum * inv_dt,
            started: *pair.event_status.bits & 2 == 0,
        }
    }
}

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

/// Bit of `PairEventStatus`: the previous step exceeded the force threshold.
pub const INITIAL_FORCE_THRESHOLD_EVENT_EMITTED: PairEventStatus = PairEventStatus { bits: 2 };

/// Event bookkeeping of a contact pair (upstream `PairEventStatus`):
/// bit 0 tracks collision starts; bit 1 tracks force-threshold crossings.
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
        let (_, bit) = DivRem::div_rem(self.bits, 2);
        bit != 0
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
    use fixed::{HALF, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_core::Handle;
    use rapier_core::collider::events::{REMOVED, SENSOR};
    use rapier_core::collider::{CollisionEventFlags, CollisionEventFlagsTrait};
    use rapier_testing::opaque;
    use crate::narrow_phase::ContactPairTrait;
    use super::{
        CollisionEvent, CollisionEventTrait, ContactForceEventTrait, PairEventStatus,
        PairEventStatusTrait, START_EVENT_EMITTED, started, stopped,
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
    #[test]
    fn test_force_event_normal_impulses_and_status() {
        let mut pair = ContactPairTrait::new(h(1), h(2));
        pair.manifold.num_points = 2;
        pair.manifold.data.normal = Vec2 { x: ZERO, y: ONE };
        let [mut a, mut b] = pair.manifold.points;
        a.data.impulse = ONE;
        b.data.impulse = TWO;
        a.data.tangent_impulse = TWO;
        pair.manifold.points = [a, b];
        let e = ContactForceEventTrait::from_contact_pair(HALF, @pair, TWO * (ONE + TWO));
        assert_eq!(e.total_force, Vec2 { x: ZERO, y: TWO * (ONE + TWO) });
        assert_eq!(e.max_force_magnitude, TWO * TWO);
        assert_eq!(e.max_force_direction, pair.manifold.data.normal);
        assert!(e.started);
        pair.event_status.bits = 2;
        assert!(!pair.event_status.start_event_emitted());
        assert!(!ContactForceEventTrait::from_contact_pair(ONE, @pair, ZERO).started);
        assert_eq!(
            ContactForceEventTrait::from_contact_pair(ZERO, @pair, ZERO).total_force,
            Default::default(),
        );
    }

    #[test]
    fn gas_from_contact_pair() {
        let mut pair = ContactPairTrait::new(h(1), h(2));
        pair.manifold.num_points = 1;
        pair.manifold.data.normal = Vec2 { x: ZERO, y: ONE };
        let [mut a, b] = pair.manifold.points;
        a.data.impulse = opaque(ONE);
        pair.manifold.points = [a, b];
        let _ = opaque(ContactForceEventTrait::from_contact_pair(opaque(HALF), @pair, ONE));
    }
}
