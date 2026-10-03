//! EL1 (F1) losers of the force-event pass: the shipped `collect_body` before EL1 (every pair
//! rebuilt) and with whole collider reads.
use fixed::{FixedTrait, ZERO};
use rapier_core::collider::ActiveEventsTrait;
use rapier_core::collider::events::CONTACT_FORCE_EVENTS;
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_geometry2d::shape::ShapeTrait;
use super::{
    ColliderSet, ContactForceEvent, Fixed, NarrowPhase, collect_groups, convex_event, convex_status,
    threshold,
};

/// EL1 (F1) loser: the shipped `collect_body` before EL1, which copies and appends every pair
/// whether its status bit changes or not.
pub fn collect_body_rebuilding(
    dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet, groups: bool,
) -> Array<ContactForceEvent> {
    let inv_dt = if dt == ZERO {
        ZERO
    } else {
        dt.recip()
    };
    let mut events = array![];
    let mut pairs = array![];
    let mut rest = narrow.pairs.span();
    let mut composite = false;
    while let Some(old) = rest.pop_front() {
        let mut pair = *old;
        let co1 = colliders.get(pair.collider1).unwrap();
        let co2 = colliders.get(pair.collider2).unwrap();
        if (co1.flags.active_events | co2.flags.active_events).contains(CONTACT_FORCE_EVENTS) {
            if co1.shape.is_composite() || co2.shape.is_composite() {
                composite = true;
                break;
            }
            let limit = threshold(co1).min(threshold(co2));
            convex_event(dt, inv_dt, limit, ref pair, ref events);
        }
        pairs.append(pair);
    }
    if composite {
        assert(groups, rapier_dynamics2d::narrow_phase::strategies::errors::COMPOSITE);
        let all = narrow.pairs.span();
        let start = all.len() - rest.len() - 1;
        collect_groups(
            dt, inv_dt, all.slice(start, rest.len() + 1), ref pairs, ref events, ref colliders,
        );
    }
    narrow.pairs = pairs;
    events
}

/// EL1 (F1) loser: the shipped `collect_body` with both colliders of every pair read whole
/// (`ColliderSetTrait::get`) instead of through `ForceEventReads`.
pub fn collect_body_whole_reads(
    dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet, groups: bool,
) -> Array<ContactForceEvent> {
    let inv_dt = if dt == ZERO {
        ZERO
    } else {
        dt.recip()
    };
    let mut events = array![];
    let mut pairs = array![];
    let mut rebuilt = false;
    let all = narrow.pairs.span();
    let mut rest = all;
    let mut composite = false;
    while let Some(old) = rest.pop_front() {
        let co1 = colliders.get(*old.collider1).unwrap();
        let co2 = colliders.get(*old.collider2).unwrap();
        let mut bits = *old.event_status.bits;
        if (co1.flags.active_events | co2.flags.active_events).contains(CONTACT_FORCE_EVENTS) {
            if co1.shape.is_composite() || co2.shape.is_composite() {
                composite = true;
                break;
            }
            let limit = threshold(co1).min(threshold(co2));
            bits = convex_status(dt, inv_dt, limit, old, ref events);
        }
        if !rebuilt && bits != *old.event_status.bits {
            pairs.append_span(all.slice(0, all.len() - rest.len() - 1));
            rebuilt = true;
        }
        if rebuilt {
            let mut pair = *old;
            pair.event_status.bits = bits;
            pairs.append(pair);
        }
    }
    if composite {
        assert(groups, rapier_dynamics2d::narrow_phase::strategies::errors::COMPOSITE);
        let start = all.len() - rest.len() - 1;
        if !rebuilt {
            pairs.append_span(all.slice(0, start));
            rebuilt = true;
        }
        collect_groups(
            dt, inv_dt, all.slice(start, rest.len() + 1), ref pairs, ref events, ref colliders,
        );
    }
    if rebuilt {
        narrow.pairs = pairs;
    }
    events
}
