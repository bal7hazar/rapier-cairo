//! Sleeping (work package SL), around the narrow phase and the user changes: the dormant contact
//! pairs of sleeping bodies and the wake-ups of user changes.
//!
//! Upstream only updates the contact pairs that have a modified collider or an awake body
//! (`collect_pairs_to_update`, `process_pair` with the awake mask); the pairs of a sleeping
//! body keep their manifold and never reach the solver. The port's broad phase is stateless
//! (D7), so a sleeping body's colliders are flagged static in their proxies (no pair between two
//! static proxies) and the previous step's *dormant* pairs — both sides fixed, absent or
//! asleep, one asleep (`ordering::dormant_of`) — are taken out of the previous list before the
//! narrow phase ([`split_dormant`]), which therefore neither regenerates them nor ends them, and
//! merged back in key order after the solver ([`merge_pairs`]). Their manifolds (warm-start
//! impulses, event status) survive the sleep unchanged, as upstream's do; when the island wakes
//! up, `pipeline::islands::update_islands` reports it and the step re-splits the dormant pairs
//! of the woken bodies into the solver input of the same step. **Deliberate divergence**
//! (`docs/adr/0001-upstream-divergences.md`, measured by lot SI): upstream leaves a revived dormant
//! pair out of the solver on the wake step (its awake mask and solver hints are frozen before the
//! wake), so a ball woken while resting on the ground sinks for one frame; the port supports it
//! immediately.
//!
//! Sensors (SE): intersection pairs live in the same list and are split the same way, so a
//! sleeping body's sensor pairs keep their `intersecting` state and emit nothing until a parent
//! wakes up. They wake nobody on a user change nor on a removal (upstream walks the contact graph
//! only).
//!
//! Removed colliders (CW, upstream-exact): the pairs of a removed collider end at the next step
//! with their `REMOVED` event, contact or sensor pair, whatever the sleep state of the bodies,
//! and the removal wakes the contact partners only (upstream `NarrowPhase::remove_collider`).
//! `World::remove_collider` clears the body links of the collider's pairs
//! ([`release_removed_pairs`]): a pair without body is never dormant, so the narrow phase sees it
//! and drops it (no per-step cost). The staged detection, which also serves raw sets
//! (`facade::CollisionPipelineTrait::step`), checks the colliders instead
//! ([`split_dormant_existing`]).
//!
//! User changes (upstream `pair_management::handle_user_changes`, `user_changes.rs`): a
//! modified collider wakes its parent and every body it is in contact with, strongly, whatever
//! the parents' types ("waking up the modified collider's parent isn't enough because it could
//! be a fixed or kinematic body"); a modified body (pose, colliders, type, dominance, enabled
//! state) does the same through its colliders. `pipeline::user_changes` wakes the parents in
//! place and [`wake_touched_partners`] is the pass over the previous step's pairs; a contact
//! that starts in the step and a joint linking an awake body are handled by the island stage.

use rapier_core::Handle;
use rapier_core::rigid_body::RigidBodyType;
use rapier_dynamics2d::collider::{Collider, ColliderTrait};
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::joint::ImpulseJoint;
use rapier_dynamics2d::narrow_phase::{ContactPair, ContactPairTrait, key_before};
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet, RigidBodySetTrait, RigidBodyTrait};
use rapier_geometry2d::broad_phase::BroadPhaseProxy;
use super::islands::{SleepCensus, update_islands, update_islands_slow};
use super::ordering::{BODY_SLEEPING, body_status, dormant_of, link_status};
use super::user_changes::is_fresh;

/// Whether any enabled non-fixed body of `entries` sleeps.
pub fn any_sleeping(entries: Span<(Handle, RigidBody)>) -> bool {
    let mut found = false;
    for (_, body) in entries {
        if *body.activation.sleeping && *body.enabled && *body.body_type != RigidBodyType::Fixed {
            found = true;
            break;
        }
    }
    found
}

/// Splits `pairs` (ascending key) into the active pairs and the dormant ones (see the module
/// documentation), both in key order, from the bodies the manifolds reference and their status
/// in `entries`. A pair whose manifold was cleared (no body) is active.
pub fn split_dormant(
    pairs: Span<ContactPair>, entries: Span<(Handle, RigidBody)>,
) -> (Array<ContactPair>, Array<ContactPair>) {
    let mut active = array![];
    let mut dormant = array![];
    for pair in pairs {
        let (s1, s2) = link_status(
            entries, *pair.manifold.data.rigid_body1, *pair.manifold.data.rigid_body2,
        );
        if dormant_of(s1, s2) {
            dormant.append(*pair);
        } else {
            active.append(*pair);
        }
    }
    (active, dormant)
}

/// [`split_dormant`] where a pair is dormant only when both of its colliders are still in
/// `colliders` (every `(handle, collider)` in ascending slot, `ColliderSetTrait::iter`): the
/// pairs of a removed collider stay active, so the narrow phase ends them with their `REMOVED`
/// event. The lookup runs for the dormant candidates only.
pub fn split_dormant_existing(
    pairs: Span<ContactPair>,
    entries: Span<(Handle, RigidBody)>,
    colliders: Span<(Handle, Collider)>,
) -> (Array<ContactPair>, Array<ContactPair>) {
    let mut active = array![];
    let mut dormant = array![];
    for pair in pairs {
        let (s1, s2) = link_status(
            entries, *pair.manifold.data.rigid_body1, *pair.manifold.data.rigid_body2,
        );
        if dormant_of(s1, s2)
            && collider_exists(colliders, *pair.collider1)
            && collider_exists(colliders, *pair.collider2) {
            dormant.append(*pair);
        } else {
            active.append(*pair);
        }
    }
    (active, dormant)
}

/// `pairs` with the body links of every pair of `removed` cleared (`manifold.data.rigid_body1`
/// and `rigid_body2` set to `None`), the other pairs unchanged, same order. Such a pair is never
/// dormant ([`split_dormant`]): the next step's narrow phase drops it, with its `REMOVED` event
/// once the collider is gone. O(pairs), at the removal only.
pub fn release_removed_pairs(pairs: Span<ContactPair>, removed: Handle) -> Array<ContactPair> {
    let mut out = array![];
    for pair in pairs {
        let mut pair = *pair;
        if pair.collider1 == removed || pair.collider2 == removed {
            pair.manifold.data.rigid_body1 = None;
            pair.manifold.data.rigid_body2 = None;
        }
        out.append(pair);
    }
    out
}

/// Whether `handle` is in `colliders` (ascending slot, dense): the entry of a live collider is
/// at most at its slot index, so the search starts there and walks down over the free slots.
pub fn collider_exists(colliders: Span<(Handle, Collider)>, handle: Handle) -> bool {
    let n = colliders.len();
    if n == 0 {
        return false;
    }
    let mut position = if handle.index < n {
        handle.index
    } else {
        n - 1
    };
    loop {
        let (candidate, _) = colliders.at(position);
        if candidate.index <= @handle.index {
            break *candidate == handle;
        }
        if position == 0 {
            break false;
        }
        position -= 1;
    }
}

/// The two ascending-key lists merged into one (their keys are disjoint).
pub fn merge_pairs(active: Span<ContactPair>, dormant: Span<ContactPair>) -> Array<ContactPair> {
    let mut active = active;
    let mut dormant = dormant;
    let mut out = array![];
    loop {
        match (active.get(0), dormant.get(0)) {
            (
                Some(a), Some(d),
            ) => {
                let a = a.unbox();
                let d = d.unbox();
                if key_before(*a.collider1, *a.collider2, *d.collider1, *d.collider2) {
                    out.append(*active.pop_front().unwrap());
                } else {
                    out.append(*dormant.pop_front().unwrap());
                }
            },
            (Some(_), None) => {
                out.append_span(active);
                break;
            },
            (None, Some(_)) => {
                out.append_span(dormant);
                break;
            },
            (None, None) => { break; },
        }
    }
    out
}

/// Wakes up (strongly) both non-fixed parents of every contact pair of `pairs` that involves a
/// collider of `touched` (upstream's modified-colliders pass of the narrow phase, which walks the
/// contact graph only: intersection pairs wake nothing; the touched colliders' own parents are
/// woken up by the caller). Returns `true` when a body was written.
pub fn wake_touched_partners(
    touched: Span<Handle>,
    pairs: Span<ContactPair>,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
) -> bool {
    wake_partners(touched, pairs, ref bodies, ref colliders)
}

fn wake_partners(
    touched: Span<Handle>,
    pairs: Span<ContactPair>,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
) -> bool {
    let mut written = false;
    for pair in pairs {
        let mut involved = false;
        for handle in touched {
            if *handle == *pair.collider1 || *handle == *pair.collider2 {
                involved = true;
                break;
            }
        }
        if involved && !pair.is_intersection_pair() {
            if wake_parent(*pair.collider1, ref bodies, ref colliders) {
                written = true;
            }
            if wake_parent(*pair.collider2, ref bodies, ref colliders) {
                written = true;
            }
        }
    }
    written
}

/// `RigidBody::wake_up(true)` on the non-fixed parent of collider `handle`, written back when
/// the body was asleep or its timer was running; `false` otherwise (also for a missing collider,
/// a standalone one or a fixed parent, which upstream's `IslandManager::wake_up` skips too).
fn wake_parent(handle: Handle, ref bodies: RigidBodySet, ref colliders: ColliderSet) -> bool {
    if let Some(collider) = colliders.get(handle) {
        if let Some(parent) = collider.parent() {
            if let Some(mut body) = bodies.get(parent) {
                if body.body_type != RigidBodyType::Fixed
                    && (body.activation.sleeping
                        || body.activation.time_since_can_sleep != fixed::ZERO) {
                    body.wake_up(true);
                    let _ = bodies.set(parent, body);
                    return true;
                }
            }
        }
    }
    false
}

/// Upstream's contact-start wake rule (`strong_wake_sleeping_side`: a contact that starts
/// strong-wakes whichever side is a sleeping dynamic body) for the pairs of the colliders
/// inserted since the last step (`fresh`, see `pipeline::user_changes`): every touching pair of
/// `pairs` (the step's narrow-phase output) with a collider of `fresh` is new, hence starting.
/// Every other start already involves an awake body, whose island the island stage wakes. Returns
/// `entries` with the woken bodies updated (the input span when nobody woke) and whether a body
/// woke.
pub fn wake_started_contacts(
    pairs: Span<ContactPair>,
    fresh: Span<Handle>,
    ref bodies: RigidBodySet,
    entries: Span<(Handle, RigidBody)>,
) -> (Span<(Handle, RigidBody)>, bool) {
    let mut woken = false;
    for pair in pairs {
        if *pair.manifold.data.num_solver_contacts == 0 {
            continue;
        }
        if is_fresh(fresh, *pair.collider1) || is_fresh(fresh, *pair.collider2) {
            let body1 = *pair.manifold.data.rigid_body1;
            let body2 = *pair.manifold.data.rigid_body2;
            // Only a sleeping side is read from the set (its type decides).
            let (s1, s2) = link_status(entries, body1, body2);
            if s1 == BODY_SLEEPING && wake_sleeping_dynamic(body1, ref bodies) {
                woken = true;
            }
            if s2 == BODY_SLEEPING && wake_sleeping_dynamic(body2, ref bodies) {
                woken = true;
            }
        }
    }
    if woken {
        (bodies.iter().span(), true)
    } else {
        (entries, false)
    }
}

/// `RigidBody::wake_up(true)` on `body` when it is a sleeping dynamic body.
fn wake_sleeping_dynamic(body: Option<Handle>, ref bodies: RigidBodySet) -> bool {
    if let Some(handle) = body {
        if let Some(mut rb) = bodies.get(handle) {
            if rb.body_type == RigidBodyType::Dynamic && rb.activation.sleeping {
                rb.wake_up(true);
                let _ = bodies.set(handle, rb);
                return true;
            }
        }
    }
    false
}

/// The island stage of a step whose user changes met colliders inserted since the last step
/// (`fresh`): upstream's contact-start wake-up first (`sleeping::wake_started_contacts`), then
/// `islands::update_islands`, forced through its slow path when a body woke so that its island
/// wakes with it.
#[inline(never)]
pub(crate) fn islands_after_insertions(
    ref bodies: RigidBodySet,
    pairs: Span<ContactPair>,
    dormant: Span<ContactPair>,
    joints: Span<(Handle, ImpulseJoint)>,
    entries: Span<(Handle, RigidBody)>,
    census: SleepCensus,
    fresh: Span<Handle>,
) -> (Span<(Handle, RigidBody)>, bool, bool) {
    let (entries, started) = wake_started_contacts(pairs, fresh, ref bodies, entries);
    if started {
        let (entries, sleeping, _) = update_islands_slow(
            ref bodies, pairs, dormant, joints, entries,
        );
        (entries, sleeping, true)
    } else {
        update_islands(ref bodies, pairs, dormant, joints, entries, census)
    }
}

/// The proxies of the step with those of the `fresh` colliders (inserted since the last step)
/// on a sleeping parent made non-static (upstream's broad phase pairs every modified collider);
/// `proxies` itself when there is none (`any_sleeping`: some collider has a sleeping parent).
/// `snapshot` and `entries`: the colliders and bodies after the user changes, ascending slot.
pub(crate) fn unstatic_fresh(
    proxies: Array<BroadPhaseProxy>,
    fresh: Span<Handle>,
    any_sleeping: bool,
    snapshot: Span<(Handle, Collider)>,
    entries: Span<(Handle, RigidBody)>,
) -> Array<BroadPhaseProxy> {
    if fresh.is_empty() || !any_sleeping {
        return proxies;
    }
    let mut out = array![];
    let mut colliders = snapshot;
    for proxy in proxies.span() {
        let (_, collider) = colliders.pop_front().unwrap();
        let mut proxy = *proxy;
        if proxy.is_static && is_fresh(fresh, proxy.collider) {
            let parent = collider.parent();
            if parent.is_some() && body_status(entries, parent.unwrap()) == BODY_SLEEPING {
                proxy.is_static = false;
            }
        }
        out.append(proxy);
    }
    out
}
