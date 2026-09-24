//! Rejected candidate of `update_islands` (AGENTS.md §5), kept for re-ranking: the slow path
//! behind a one-iteration `while` (metered call, AGENTS.md §7) instead of a plain `if`. The
//! loop machinery costs more than what it saves: the dict creations of the slow path are
//! runtime costs, not statically charged to the fast path (`tests::gas_islands_*`).

use rapier_core::Handle;
use rapier_dynamics2d::joint::ImpulseJoint;
use rapier_dynamics2d::narrow_phase::ContactPair;
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet};
use super::{SleepCensus, update_islands_slow};

/// `update_islands` with the slow path behind a one-iteration `while`.
pub fn update_islands_metered(
    ref bodies: RigidBodySet,
    pairs: Span<ContactPair>,
    dormant: Span<ContactPair>,
    joints: Span<(Handle, ImpulseJoint)>,
    entries: Span<(Handle, RigidBody)>,
    census: SleepCensus,
) -> (Span<(Handle, RigidBody)>, bool, bool) {
    let mut result = (entries, census.sleeping != 0, false);
    let mut pending = census.awake != 0 && (census.sleeping != 0 || census.eligible);
    while pending {
        result = update_islands_slow(ref bodies, pairs, dormant, joints, entries);
        pending = false;
    }
    result
}
