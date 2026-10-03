//! Indexing and change reads of the [`ColliderSet`] (work package PX4, off the step path):
//! upstream's `Index<ColliderHandle>` / `Index<data::Index>`, `take_modified`, `take_removed` and
//! `ModifiedColliders`.
//!
//! # Deviations
//!
//! * `Index` is Cairo's `core::ops::Index` (an arena read needs `ref self`, `IndexView` takes a
//!   snapshot): `colliders[handle]` copies the collider out and panics with
//!   [`errors::NOT_FOUND`] when the handle does not resolve, as upstream's `Index`. `IndexMut`
//!   stays missing: Cairo has no `IndexMut`, and a collider is a value; write a change back with
//!   `ColliderSetTrait::set`. Upstream's `data::Index` and `ColliderHandle` are both [`Handle`].
//! * `take_modified` reads, it does not drain: the set keeps no separate modified list (the step
//!   reads each collider's change flags, `docs/PLAN.md` D7/D9) and the step clears those flags, so
//!   draining them here would change what the next step sees. The returned
//!   [`ModifiedColliders`] lists, in ascending slot index, the colliders whose change flags are
//!   not empty; two calls without a step in between return the same list.
//! * `take_removed` (WS3) drains the handles that `ColliderSetTrait::remove` recorded (a body's
//!   removal with its colliders included), as upstream; the step drains them too (upstream's
//!   pipeline calls `take_removed`), so after a step the list holds the removals made since.

use core::ops::Index;
use rapier_core::Handle;
use crate::collider::Collider;
use super::{ColliderSet, ColliderSetTrait};

pub mod errors {
    /// `colliders[handle]` with a handle that does not resolve (upstream: index out of bounds).
    pub const NOT_FOUND: felt252 = 'ColliderSet: not found';
}

/// The colliders modified since their change flags were last cleared (upstream
/// `ModifiedColliders`, a set of handles): see the module documentation.
#[derive(Drop, Serde, PartialEq, Debug, Default)]
pub struct ModifiedColliders {
    /// The handles, in ascending slot index.
    pub handles: Array<Handle>,
}

/// The empty list of removed handles of a new or restored set.
#[inline(always)]
pub(crate) fn no_removal() -> Box<Array<Handle>> {
    BoxTrait::new(array![])
}

/// The empty set (`ColliderSetTrait::new`).
pub impl ColliderSetDefault of Default<ColliderSet> {
    #[inline(always)]
    fn default() -> ColliderSet {
        ColliderSetTrait::new()
    }
}

/// `colliders[handle]` (upstream `impl Index<ColliderHandle> for ColliderSet`).
pub impl ColliderSetIndexImpl of Index<ColliderSet, Handle> {
    type Target = Collider;

    /// The collider behind `handle`, copied out.
    /// #### Panics
    /// * [`errors::NOT_FOUND`] when the handle is stale or unknown.
    fn index(ref self: ColliderSet, index: Handle) -> Collider {
        self.get(index).expect(errors::NOT_FOUND)
    }
}

/// Change tracking reads of [`ColliderSet`] (upstream `take_modified`, `take_removed`).
#[generate_trait]
pub impl ColliderSetChangesImpl of ColliderSetChangesTrait {
    /// The colliders whose change flags are not empty (upstream `take_modified`, read-only: see
    /// the module documentation). Cost: one dict read per collider.
    fn take_modified(ref self: ColliderSet) -> ModifiedColliders {
        let mut handles = array![];
        for (handle, collider) in self.iter() {
            if collider.changes.bits != 0 {
                handles.append(handle);
            }
        }
        ModifiedColliders { handles }
    }

    /// The handles removed since the last call or the last step, in removal order, and the list
    /// emptied (upstream `take_removed`). The step drains the list as upstream's pipeline does, so
    /// a world's caller sees only the removals made since its last step.
    fn take_removed(ref self: ColliderSet) -> Array<Handle> {
        let removed = self.removed.unbox();
        self.removed = BoxTrait::new(array![]);
        removed
    }

    /// The handles [`take_removed`](ColliderSetChangesTrait::take_removed) would return, without
    /// emptying the list (the world state saves them, WS3).
    #[inline(always)]
    fn removed(self: @ColliderSet) -> Span<Handle> {
        self.removed.as_snapshot().unbox().span()
    }

    /// Replaces the list of removed handles (the world state restores it, WS3).
    fn restore_removed(ref self: ColliderSet, removed: Array<Handle>) {
        self.removed = BoxTrait::new(removed);
    }

    /// Records the removal of `handle` (`ColliderSetTrait::remove`, WS3).
    fn record_removed(ref self: ColliderSet, handle: Handle) {
        let mut removed = self.removed.unbox();
        removed.append(handle);
        self.removed = BoxTrait::new(removed);
    }

    /// Empties the list of removed handles when it is not empty (the step's drain, WS3).
    #[inline(always)]
    fn clear_removed(ref self: ColliderSet) {
        if !self.removed.as_snapshot().unbox().is_empty() {
            self.removed = BoxTrait::new(array![]);
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, HALF, ONE, ZERO};
    use glam_core::Vec2;
    use rapier_core::Handle;
    use rapier_core::collider::ColliderChangesTrait;
    use rapier_testing::opaque;
    use crate::collider::{Collider, ColliderBuilderTrait};
    use crate::rigid_body_set::RigidBodySetTrait;
    use super::super::{ColliderSet, ColliderSetTrait};
    use super::{ColliderSetChangesTrait, ColliderSetIndexImpl, ModifiedColliders};

    fn cuboid_at(x: Fixed, y: Fixed) -> Collider {
        ColliderBuilderTrait::cuboid(HALF, HALF).translation(Vec2 { x, y }).build()
    }

    fn two() -> (ColliderSet, Handle, Handle) {
        let mut colliders = ColliderSetTrait::new();
        let a = colliders.insert(cuboid_at(ZERO, ZERO));
        let b = colliders.insert(cuboid_at(ONE, ZERO));
        (colliders, a, b)
    }

    #[test]
    fn test_index_is_get() {
        let (mut colliders, a, b) = two();
        assert_eq!(colliders[a], colliders.get(a).unwrap());
        assert_eq!(colliders[b], colliders.get(b).unwrap());
        assert!(colliders[a] != colliders[b]);
    }

    #[test]
    #[should_panic(expected: ('ColliderSet: not found',))]
    fn test_index_of_a_removed_handle_panics() {
        let (mut colliders, a, _) = two();
        let mut bodies = RigidBodySetTrait::new();
        let _ = colliders.remove(a, ref bodies);
        let _ = colliders[a];
    }

    #[test]
    fn test_take_modified_lists_flagged_colliders_and_does_not_drain() {
        let (mut colliders, a, b) = two();
        let all = colliders.take_modified();
        assert_eq!(all, ModifiedColliders { handles: array![a, b] });
        // A second read answers the same: nothing was drained.
        assert_eq!(colliders.take_modified(), all);
        // The step clears the flags of `a`; only `b` stays modified.
        let mut quiet = colliders.get(a).unwrap();
        quiet.changes = ColliderChangesTrait::empty();
        assert!(colliders.set_internal(a, quiet));
        assert_eq!(colliders.take_modified(), ModifiedColliders { handles: array![b] });
    }

    #[test]
    fn test_take_modified_of_an_empty_set() {
        let mut colliders: ColliderSet = ColliderSetTrait::new();
        assert_eq!(colliders.take_modified(), Default::default());
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }

    /// `gas_index` and `gas_take_modified` − `gas_setup`: two inserted colliders.
    #[test]
    fn gas_setup() {
        let _ = two();
    }

    #[test]
    fn gas_index() {
        let (mut colliders, a, _) = two();
        let _ = colliders[opaque(a)];
    }

    #[test]
    fn gas_take_modified() {
        let (mut colliders, _, _) = two();
        let _ = colliders.take_modified();
    }
}

#[cfg(test)]
mod removed_tests {
    use fixed::{HALF, ZERO};
    use glam_core::Vec2;
    use rapier_core::Handle;
    use rapier_math::pose2::Pose2;
    use crate::collider::ColliderBuilderTrait;
    use crate::rigid_body_set::{RigidBodySetTrait, RigidBodyTrait};
    use super::ColliderSetChangesTrait;
    use super::super::ColliderSetTrait;

    /// Removals are recorded in order (a body's attached colliders included) and drained once.
    #[test]
    fn test_take_removed_drains_the_removals() {
        let mut colliders = ColliderSetTrait::new();
        let mut bodies = RigidBodySetTrait::new();
        let ball = ColliderBuilderTrait::ball(HALF).translation(Vec2 { x: ZERO, y: ZERO }).build();
        let a = colliders.insert(ball);
        let b = colliders.insert(ball);
        let body = bodies.insert(RigidBodyTrait::dynamic(Pose2 { ..Default::default() }));
        let c = colliders.insert_with_parent(ball, body, ref bodies);
        assert_eq!(colliders.take_removed(), array![]);
        assert!(colliders.remove(b, ref bodies).is_some());
        assert!(colliders.remove(b, ref bodies).is_none());
        assert!(bodies.remove(body, ref colliders, true).is_some());
        assert!(colliders.remove(a, ref bodies).is_some());
        assert_eq!(colliders.removed(), array![b, c, a].span());
        assert_eq!(colliders.take_removed(), array![b, c, a]);
        assert_eq!(colliders.take_removed(), array![]);
        colliders.restore_removed(array![Handle { index: 7, generation: 1 }]);
        assert_eq!(colliders.removed(), array![Handle { index: 7, generation: 1 }].span());
        colliders.clear_removed();
        assert_eq!(colliders.take_removed(), array![]);
    }
}
