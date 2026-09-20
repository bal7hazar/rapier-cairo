//! Flags describing how a rigid-body has been modified by the user (upstream `RigidBodyChanges`).
//!
//! Same representation and set operations as the collider flags, see
//! [`collider::events`](crate::collider::events) for the measured ranking of the candidates
//! (`&` / `|` on the raw `u32` win).

use core::traits::{BitAnd, BitOr};

/// The rigid-body is in the modified rigid-body set.
pub const IN_MODIFIED_SET: RigidBodyChanges = RigidBodyChanges { bits: 0x1 };

/// The position component was modified.
pub const POSITION: RigidBodyChanges = RigidBodyChanges { bits: 0x2 };

/// The activation component was modified.
pub const SLEEP: RigidBodyChanges = RigidBodyChanges { bits: 0x4 };

/// The colliders component was modified.
pub const COLLIDERS: RigidBodyChanges = RigidBodyChanges { bits: 0x8 };

/// The body type was modified.
pub const TYPE: RigidBodyChanges = RigidBodyChanges { bits: 0x10 };

/// The dominance component was modified.
pub const DOMINANCE: RigidBodyChanges = RigidBodyChanges { bits: 0x20 };

/// The local mass properties must be recomputed.
pub const LOCAL_MASS_PROPERTIES: RigidBodyChanges = RigidBodyChanges { bits: 0x40 };

/// The rigid-body was enabled or disabled.
pub const ENABLED_OR_DISABLED: RigidBodyChanges = RigidBodyChanges { bits: 0x80 };

/// Flags describing how a rigid-body has been modified by the user. Default: none.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct RigidBodyChanges {
    /// Raw mask, bit `i` is the flag `1 << i`.
    pub bits: u32,
}

/// Wraps a raw mask (upstream `from_bits_retain`).
pub impl U32IntoRigidBodyChanges of Into<u32, RigidBodyChanges> {
    #[inline(always)]
    fn into(self: u32) -> RigidBodyChanges {
        RigidBodyChanges { bits: self }
    }
}

/// Unwraps the raw mask.
pub impl RigidBodyChangesIntoU32 of Into<RigidBodyChanges, u32> {
    #[inline(always)]
    fn into(self: RigidBodyChanges) -> u32 {
        self.bits
    }
}

/// Set union.
pub impl RigidBodyChangesBitOr of BitOr<RigidBodyChanges> {
    #[inline(always)]
    fn bitor(lhs: RigidBodyChanges, rhs: RigidBodyChanges) -> RigidBodyChanges {
        RigidBodyChanges { bits: lhs.bits | rhs.bits }
    }
}

/// Set intersection.
pub impl RigidBodyChangesBitAnd of BitAnd<RigidBodyChanges> {
    #[inline(always)]
    fn bitand(lhs: RigidBodyChanges, rhs: RigidBodyChanges) -> RigidBodyChanges {
        RigidBodyChanges { bits: lhs.bits & rhs.bits }
    }
}

/// Set operations on [`RigidBodyChanges`].
#[generate_trait]
pub impl RigidBodyChangesImpl of RigidBodyChangesTrait {
    /// No flag set.
    #[inline(always)]
    fn empty() -> RigidBodyChanges {
        RigidBodyChanges { bits: 0 }
    }

    /// Every defined flag set.
    #[inline(always)]
    fn all() -> RigidBodyChanges {
        RigidBodyChanges { bits: 0xff }
    }

    /// Returns `true` when no flag is set.
    #[inline(always)]
    fn is_empty(self: RigidBodyChanges) -> bool {
        self.bits == 0
    }

    /// Returns `true` when every flag of `other` is in `self`.
    #[inline(always)]
    fn contains(self: RigidBodyChanges, other: RigidBodyChanges) -> bool {
        self.bits & other.bits == other.bits
    }

    /// Returns `true` when `self` and `other` share at least one flag.
    #[inline(always)]
    fn intersects(self: RigidBodyChanges, other: RigidBodyChanges) -> bool {
        self.bits & other.bits != 0
    }

    /// Sets every flag of `other`.
    #[inline(always)]
    fn insert(ref self: RigidBodyChanges, other: RigidBodyChanges) {
        self.bits = self.bits | other.bits;
    }

    /// Clears every flag of `other`.
    #[inline(always)]
    fn remove(ref self: RigidBodyChanges, other: RigidBodyChanges) {
        self.bits = self.bits & (0xffffffff - other.bits);
    }
}

#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use super::{
        COLLIDERS, DOMINANCE, ENABLED_OR_DISABLED, IN_MODIFIED_SET, LOCAL_MASS_PROPERTIES, POSITION,
        RigidBodyChanges, RigidBodyChangesTrait, SLEEP, TYPE,
    };

    #[test]
    fn test_default_is_empty() {
        let default: RigidBodyChanges = Default::default();
        assert_eq!(default, RigidBodyChangesTrait::empty());
        assert!(default.is_empty());
        assert!(!POSITION.is_empty());
    }

    #[test]
    fn test_flag_values() {
        assert_eq!(IN_MODIFIED_SET.bits, 0x01);
        assert_eq!(POSITION.bits, 0x02);
        assert_eq!(SLEEP.bits, 0x04);
        assert_eq!(COLLIDERS.bits, 0x08);
        assert_eq!(TYPE.bits, 0x10);
        assert_eq!(DOMINANCE.bits, 0x20);
        assert_eq!(LOCAL_MASS_PROPERTIES.bits, 0x40);
        assert_eq!(ENABLED_OR_DISABLED.bits, 0x80);
        let union = IN_MODIFIED_SET
            | POSITION
            | SLEEP
            | COLLIDERS
            | TYPE
            | DOMINANCE
            | LOCAL_MASS_PROPERTIES
            | ENABLED_OR_DISABLED;
        assert_eq!(union, RigidBodyChangesTrait::all());
    }

    #[test]
    fn test_set_operations() {
        let mut changes: RigidBodyChanges = Default::default();
        changes.insert(POSITION);
        changes.insert(TYPE);
        changes.insert(POSITION);
        assert_eq!(changes, POSITION | TYPE);
        assert!(changes.contains(POSITION));
        assert!(changes.contains(POSITION | TYPE));
        assert!(!changes.contains(POSITION | SLEEP));
        assert!(changes.contains(RigidBodyChangesTrait::empty()));
        assert!(changes.intersects(POSITION | SLEEP));
        assert!(!changes.intersects(SLEEP | DOMINANCE));
        assert!(!changes.intersects(RigidBodyChangesTrait::empty()));
        changes.remove(SLEEP);
        assert_eq!(changes, POSITION | TYPE);
        changes.remove(POSITION | SLEEP);
        assert_eq!(changes, TYPE);
        changes.remove(TYPE);
        assert!(changes.is_empty());
        assert_eq!((POSITION | TYPE) & TYPE, TYPE);
        let bits: u32 = (POSITION | TYPE).into();
        assert_eq!(bits, 0x12);
        let back: RigidBodyChanges = 0x12_u32.into();
        assert_eq!(back, POSITION | TYPE);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_contains() {
        assert!(opaque(POSITION | TYPE).contains(TYPE));
    }

    #[test]
    fn gas_intersects() {
        assert!(opaque(POSITION | TYPE).intersects(TYPE));
    }

    #[test]
    fn gas_insert() {
        let mut changes = opaque(POSITION);
        changes.insert(TYPE);
        assert!(changes.bits == 0x12);
    }

    #[test]
    fn gas_remove() {
        let mut changes = opaque(POSITION | TYPE);
        changes.remove(TYPE);
        assert!(changes.bits == 0x02);
    }
}
