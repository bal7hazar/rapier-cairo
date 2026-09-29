//! The names upstream's `geometry/contact_pair.rs` gives to the contents of a contact pair (work
//! package PX4, off the step path), over CP3's [`ContactPairView`].
//!
//! * [`RigidPairContacts`] is [`ContactPairView`]: its manifolds and `solver_manifolds` /
//!   `has_any_active_contact` are `ContactPairViewTrait`'s (there is no contact clustering, so
//!   `solver_manifolds` is `manifolds`).
//! * [`PairContacts`] keeps upstream's `Rigid` variant; the `Soft` variant belongs to the soft
//!   bodies, which are not part of the port.
//! * [`ContactId`] is the id of a solver contact (upstream: `u32` at `f32`, `u64` at `f64`). The
//!   port's solver contacts carry no id (`NEW_CONTACT_BIT` marks a manifold point), so no value of
//!   this type is produced by the step.

use super::ContactPairView;

/// The contact manifolds of a contact pair with the solver's view of them (upstream
/// `RigidPairContacts`): a [`ContactPairView`].
pub type RigidPairContacts = ContactPairView;

/// The contacts of a pair, in the form its two colliders call for (upstream `PairContacts`).
#[derive(Drop, Serde, PartialEq, Debug)]
pub enum PairContacts {
    /// Contact manifolds: every pair of the port.
    Rigid: RigidPairContacts,
}

/// The storage type of a solver contact id (upstream `ContactId`).
pub type ContactId = u32;

#[cfg(test)]
mod tests {
    use rapier_core::Handle;
    use rapier_testing::opaque;
    use crate::events::PairEventStatusTrait;
    use super::super::ContactPairViewTrait;
    use super::{ContactId, ContactPairView, PairContacts, RigidPairContacts};

    fn view() -> RigidPairContacts {
        ContactPairView {
            collider1: Handle { index: 1, generation: 0 },
            collider2: Handle { index: 2, generation: 0 },
            event_status: PairEventStatusTrait::empty(),
            manifolds: array![],
        }
    }

    #[test]
    fn test_rigid_pair_contacts_is_the_view() {
        let rigid: RigidPairContacts = view();
        assert!(!rigid.has_any_active_contact());
        assert_eq!(rigid.solver_manifolds().len(), rigid.manifolds().len());
        let contacts = PairContacts::Rigid(view());
        match contacts {
            PairContacts::Rigid(inner) => assert_eq!(inner, view()),
        }
        let id: ContactId = 7;
        assert_eq!(id, 7_u32);
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(1_u32);
    }

    #[test]
    fn gas_pair_contacts_rigid() {
        let _ = PairContacts::Rigid(opaque(view()));
    }
}
