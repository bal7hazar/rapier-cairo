//! Indexing of the [`RigidBodySet`] and the [`RigidBodyIds`] value (work package PX4, off the
//! step path): upstream's `Index<RigidBodyHandle>` / `Index<data::Index>` and `RigidBodyIds`.
//!
//! # Deviations
//!
//! * `Index` is Cairo's `core::ops::Index` (an arena read needs `ref self`, `IndexView` takes a
//!   snapshot): `bodies[handle]` copies the body out and panics with [`errors::NOT_FOUND`] when
//!   the handle does not resolve, as upstream's `Index`. `IndexMut` stays missing: Cairo has no
//!   `IndexMut`, and a body is a value; write a change back with `RigidBodySetTrait::set`.
//!   Upstream's `data::Index` and `RigidBodyHandle` are both [`Handle`].
//! * [`RigidBodyIds`] is a plain value with upstream's four fields and `Default`: the port keeps no
//!   persistent islands, so no body stores one and the step never reads it.

use core::ops::Index;
use rapier_core::Handle;
use super::{RigidBody, RigidBodySet, RigidBodySetTrait};

pub mod errors {
    /// `bodies[handle]` with a handle that does not resolve (upstream: index out of bounds).
    pub const NOT_FOUND: felt252 = 'RigidBodySet: not found';
}

/// Internal identifiers of the physics engine (upstream `RigidBodyIds`, whose `Default` is
/// `u32::MAX` in every field: no island, no active set slot).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RigidBodyIds {
    pub active_island_id: u32,
    pub active_set_id: u32,
    /// The persistent island of the body (`u32::MAX`: none, as for fixed or disabled bodies).
    pub island_id: u32,
    /// The index of the body in its island's body array.
    pub island_index: u32,
}

pub impl RigidBodyIdsDefault of Default<RigidBodyIds> {
    fn default() -> RigidBodyIds {
        RigidBodyIds {
            active_island_id: 0xffffffff,
            active_set_id: 0xffffffff,
            island_id: 0xffffffff,
            island_index: 0xffffffff,
        }
    }
}

/// `bodies[handle]` (upstream `impl Index<RigidBodyHandle> for RigidBodySet`).
pub impl RigidBodySetIndexImpl of Index<RigidBodySet, Handle> {
    type Target = RigidBody;

    /// The body behind `handle`, copied out.
    /// #### Panics
    /// * [`errors::NOT_FOUND`] when the handle is stale or unknown.
    fn index(ref self: RigidBodySet, index: Handle) -> RigidBody {
        self.get(index).expect(errors::NOT_FOUND)
    }
}

#[cfg(test)]
mod tests {
    use fixed::ONE;
    use rapier_math::pose2::IDENTITY;
    use rapier_testing::opaque;
    use crate::rigid_body_set::{RigidBodySetTrait, RigidBodyTrait};
    use super::{RigidBodyIds, RigidBodySetIndexImpl};

    #[test]
    fn test_index_is_get() {
        let mut bodies = RigidBodySetTrait::new();
        let a = bodies.insert(RigidBodyTrait::dynamic(IDENTITY));
        let b = bodies.insert(RigidBodyTrait::fixed(IDENTITY));
        assert_eq!(bodies[a], bodies.get(a).unwrap());
        assert_eq!(bodies[b], bodies.get(b).unwrap());
        assert!(bodies[a] != bodies[b]);
    }

    #[test]
    #[should_panic(expected: ('RigidBodySet: not found',))]
    fn test_index_of_a_removed_handle_panics() {
        let mut bodies = RigidBodySetTrait::new();
        let a = bodies.insert(RigidBodyTrait::dynamic(IDENTITY));
        let mut colliders = crate::collider_set::ColliderSetTrait::new();
        let _ = bodies.remove(a, ref colliders, false);
        let _ = bodies[a];
    }

    #[test]
    fn test_ids_default_is_all_invalid() {
        let ids: RigidBodyIds = Default::default();
        assert_eq!(
            ids,
            RigidBodyIds {
                active_island_id: 0xffffffff,
                active_set_id: 0xffffffff,
                island_id: 0xffffffff,
                island_index: 0xffffffff,
            },
        );
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }

    #[test]
    fn gas_index() {
        let mut bodies = RigidBodySetTrait::new();
        let a = bodies.insert(RigidBodyTrait::dynamic(IDENTITY));
        let _ = bodies[opaque(a)];
    }

    #[test]
    fn gas_ids_default() {
        let ids: RigidBodyIds = Default::default();
        let _ = opaque(ids);
    }
}
