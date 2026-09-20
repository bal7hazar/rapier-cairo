//! Physics-hook flags of a collider (upstream `pipeline::ActiveHooks`).
//!
//! Upstream's hooks are `dyn PhysicsHooks` callbacks, which are cut from the port (see
//! `docs/PLAN.md`); the flags are kept as plain data so that defaults and the collider layout
//! match upstream. Set operations are the raw `u32` `&` / `|`, see
//! [`events`](crate::collider::events) for the measured ranking of the candidates.

use core::traits::{BitAnd, BitOr};

/// Enables the contact pair filter hook for this collider.
pub const FILTER_CONTACT_PAIRS: ActiveHooks = ActiveHooks { bits: 0x1 };

/// Enables the intersection pair filter hook for this collider.
pub const FILTER_INTERSECTION_PAIR: ActiveHooks = ActiveHooks { bits: 0x2 };

/// Enables the solver contact modification hook for this collider.
pub const MODIFY_SOLVER_CONTACTS: ActiveHooks = ActiveHooks { bits: 0x4 };

/// Flags enabling custom collision filtering and contact modification callbacks for a collider.
/// Default: none.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct ActiveHooks {
    /// Raw mask, bit `i` is the flag `1 << i`.
    pub bits: u32,
}

/// Wraps a raw mask (upstream `from_bits_retain`).
pub impl U32IntoActiveHooks of Into<u32, ActiveHooks> {
    #[inline(always)]
    fn into(self: u32) -> ActiveHooks {
        ActiveHooks { bits: self }
    }
}

/// Unwraps the raw mask.
pub impl ActiveHooksIntoU32 of Into<ActiveHooks, u32> {
    #[inline(always)]
    fn into(self: ActiveHooks) -> u32 {
        self.bits
    }
}

/// Set union.
pub impl ActiveHooksBitOr of BitOr<ActiveHooks> {
    #[inline(always)]
    fn bitor(lhs: ActiveHooks, rhs: ActiveHooks) -> ActiveHooks {
        ActiveHooks { bits: lhs.bits | rhs.bits }
    }
}

/// Set intersection.
pub impl ActiveHooksBitAnd of BitAnd<ActiveHooks> {
    #[inline(always)]
    fn bitand(lhs: ActiveHooks, rhs: ActiveHooks) -> ActiveHooks {
        ActiveHooks { bits: lhs.bits & rhs.bits }
    }
}

/// Set operations on [`ActiveHooks`].
#[generate_trait]
pub impl ActiveHooksImpl of ActiveHooksTrait {
    /// No flag set.
    #[inline(always)]
    fn empty() -> ActiveHooks {
        ActiveHooks { bits: 0 }
    }

    /// Every defined flag set.
    #[inline(always)]
    fn all() -> ActiveHooks {
        ActiveHooks { bits: 0x7 }
    }

    /// Returns `true` when no flag is set.
    #[inline(always)]
    fn is_empty(self: ActiveHooks) -> bool {
        self.bits == 0
    }

    /// Returns `true` when every flag of `other` is in `self`.
    #[inline(always)]
    fn contains(self: ActiveHooks, other: ActiveHooks) -> bool {
        self.bits & other.bits == other.bits
    }

    /// Returns `true` when `self` and `other` share at least one flag.
    #[inline(always)]
    fn intersects(self: ActiveHooks, other: ActiveHooks) -> bool {
        self.bits & other.bits != 0
    }

    /// Sets every flag of `other`.
    #[inline(always)]
    fn insert(ref self: ActiveHooks, other: ActiveHooks) {
        self.bits = self.bits | other.bits;
    }

    /// Clears every flag of `other`.
    #[inline(always)]
    fn remove(ref self: ActiveHooks, other: ActiveHooks) {
        self.bits = self.bits & (0xffffffff - other.bits);
    }
}

#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use super::{
        ActiveHooks, ActiveHooksTrait, FILTER_CONTACT_PAIRS, FILTER_INTERSECTION_PAIR,
        MODIFY_SOLVER_CONTACTS,
    };

    #[test]
    fn test_default_is_empty() {
        let default: ActiveHooks = Default::default();
        assert_eq!(default, ActiveHooksTrait::empty());
        assert!(default.is_empty());
        assert!(!FILTER_CONTACT_PAIRS.is_empty());
    }

    #[test]
    fn test_flag_values() {
        assert_eq!(FILTER_CONTACT_PAIRS.bits, 0b001);
        assert_eq!(FILTER_INTERSECTION_PAIR.bits, 0b010);
        assert_eq!(MODIFY_SOLVER_CONTACTS.bits, 0b100);
        let union = FILTER_CONTACT_PAIRS | FILTER_INTERSECTION_PAIR | MODIFY_SOLVER_CONTACTS;
        assert_eq!(union, ActiveHooksTrait::all());
    }

    #[test]
    fn test_set_operations() {
        let mut hooks: ActiveHooks = Default::default();
        hooks.insert(MODIFY_SOLVER_CONTACTS);
        hooks.insert(FILTER_CONTACT_PAIRS);
        hooks.insert(MODIFY_SOLVER_CONTACTS);
        assert_eq!(hooks, FILTER_CONTACT_PAIRS | MODIFY_SOLVER_CONTACTS);
        assert!(hooks.contains(MODIFY_SOLVER_CONTACTS));
        assert!(!hooks.contains(FILTER_INTERSECTION_PAIR));
        assert!(hooks.intersects(FILTER_CONTACT_PAIRS | FILTER_INTERSECTION_PAIR));
        assert!(!hooks.intersects(FILTER_INTERSECTION_PAIR));
        hooks.remove(FILTER_CONTACT_PAIRS | FILTER_INTERSECTION_PAIR);
        assert_eq!(hooks, MODIFY_SOLVER_CONTACTS);
        assert_eq!(hooks & FILTER_CONTACT_PAIRS, ActiveHooksTrait::empty());
        let bits: u32 = hooks.into();
        assert_eq!(bits, 4);
        let back: ActiveHooks = 5_u32.into();
        assert_eq!(back, FILTER_CONTACT_PAIRS | MODIFY_SOLVER_CONTACTS);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_contains() {
        assert!(
            opaque(FILTER_CONTACT_PAIRS | MODIFY_SOLVER_CONTACTS).contains(MODIFY_SOLVER_CONTACTS),
        );
    }

    #[test]
    fn gas_insert() {
        let mut hooks = opaque(FILTER_CONTACT_PAIRS);
        hooks.insert(MODIFY_SOLVER_CONTACTS);
        assert!(hooks.bits == 5);
    }
}
