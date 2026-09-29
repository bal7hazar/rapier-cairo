//! Indexing and change reads of the [`ColliderSet`] (work package PX4, off the step path):
//! upstream's `Index<ColliderHandle>` / `Index<data::Index>`, `take_modified` and
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
//! * `take_removed` is not ported: the set records no removals (`remove` returns the collider).

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

/// Change tracking reads of [`ColliderSet`] (upstream `take_modified`).
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
