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
//! User changes (upstream `pair_management::handle_user_changes`, `user_changes.rs`): a
//! modified collider wakes its parent and every body it is in contact with, strongly, whatever
//! the parents' types ("waking up the modified collider's parent isn't enough because it could
//! be a fixed or kinematic body"); a modified body (pose, colliders, type, dominance, enabled
//! state) does the same through its colliders. `pipeline::user_changes` wakes the parents in
//! place and [`wake_touched_partners`] is the pass over the previous step's pairs; a contact
//! that starts in the step and a joint linking an awake body are handled by the island stage.

use rapier_core::Handle;
use rapier_core::rigid_body::RigidBodyType;
use rapier_dynamics2d::collider::ColliderTrait;
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::narrow_phase::{ContactPair, key_before};
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet, RigidBodySetTrait, RigidBodyTrait};
use super::ordering::{dormant_of, link_status};

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

/// Wakes up (strongly) both non-fixed parents of every pair of `pairs` that involves a collider
/// of `touched` (upstream's modified-colliders pass of the narrow phase; the touched colliders'
/// own parents are woken up by the caller). Returns `true` when a body was written.
pub fn wake_touched_partners(
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
        if involved {
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
