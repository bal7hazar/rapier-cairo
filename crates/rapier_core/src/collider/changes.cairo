//! Flags describing how a collider has been modified by the user (upstream `ColliderChanges`).
//!
//! Same representation and set operations as the other flag types, see
//! [`events`](crate::collider::events) for the measured ranking (`&` / `|` on the raw `u32`
//! win). `needs_narrow_phase_update` is upstream's `bits() > 2` integer comparison.
//!
//! Cut: upstream's `DEFORMED` (bit 9) belongs to soft-body surfaces; soft bodies are out of
//! scope, so bit 9 is unassigned and `needs_broad_phase_update` does not test it.

use core::traits::{BitAnd, BitOr};

/// `PARENT | POSITION | SHAPE | ENABLED_OR_DISABLED`: the changes a broad-phase update needs.
const BROAD_PHASE_MASK: u32 = 0x12c;

/// The collider handle is in the changed collider set.
pub const IN_MODIFIED_SET: ColliderChanges = ColliderChanges { bits: 0x1 };

/// The density or mass properties changed (rigid-body mass update).
pub const LOCAL_MASS_PROPERTIES: ColliderChanges = ColliderChanges { bits: 0x2 };

/// The parent component changed (broad-phase and narrow-phase update).
pub const PARENT: ColliderChanges = ColliderChanges { bits: 0x4 };

/// The position component changed (broad-phase and narrow-phase update).
pub const POSITION: ColliderChanges = ColliderChanges { bits: 0x8 };

/// The collision groups changed (narrow-phase update).
pub const GROUPS: ColliderChanges = ColliderChanges { bits: 0x10 };

/// The shape changed (broad-phase and narrow-phase update, pair workspace invalidation).
pub const SHAPE: ColliderChanges = ColliderChanges { bits: 0x20 };

/// The collider type changed (narrow-phase update, pair invalidation).
pub const TYPE: ColliderChanges = ColliderChanges { bits: 0x40 };

/// The effective dominance of the parent changed (narrow-phase update).
pub const PARENT_EFFECTIVE_DOMINANCE: ColliderChanges = ColliderChanges { bits: 0x80 };

/// The collider was enabled or disabled (broad-phase and narrow-phase update).
pub const ENABLED_OR_DISABLED: ColliderChanges = ColliderChanges { bits: 0x100 };

/// Flags describing how a collider has been modified by the user. Default: none.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ColliderChanges {
    /// Raw mask, bit `i` is the flag `1 << i`.
    pub bits: u32,
}

/// No flag set (upstream's bitflags `Default`, the empty set).
pub impl ColliderChangesDefault of Default<ColliderChanges> {
    #[inline(always)]
    fn default() -> ColliderChanges {
        ColliderChanges { bits: 0 }
    }
}

/// Wraps a raw mask (upstream `from_bits_retain`).
pub impl U32IntoColliderChanges of Into<u32, ColliderChanges> {
    #[inline(always)]
    fn into(self: u32) -> ColliderChanges {
        ColliderChanges { bits: self }
    }
}

/// Unwraps the raw mask.
pub impl ColliderChangesIntoU32 of Into<ColliderChanges, u32> {
    #[inline(always)]
    fn into(self: ColliderChanges) -> u32 {
        self.bits
    }
}

/// Set union.
pub impl ColliderChangesBitOr of BitOr<ColliderChanges> {
    #[inline(always)]
    fn bitor(lhs: ColliderChanges, rhs: ColliderChanges) -> ColliderChanges {
        ColliderChanges { bits: lhs.bits | rhs.bits }
    }
}

/// Set intersection.
pub impl ColliderChangesBitAnd of BitAnd<ColliderChanges> {
    #[inline(always)]
    fn bitand(lhs: ColliderChanges, rhs: ColliderChanges) -> ColliderChanges {
        ColliderChanges { bits: lhs.bits & rhs.bits }
    }
}

/// Set operations on [`ColliderChanges`].
#[generate_trait]
pub impl ColliderChangesImpl of ColliderChangesTrait {
    /// No flag set.
    #[inline(always)]
    fn empty() -> ColliderChanges {
        ColliderChanges { bits: 0 }
    }

    /// Every defined flag set.
    #[inline(always)]
    fn all() -> ColliderChanges {
        ColliderChanges { bits: 0x1ff }
    }

    /// Returns `true` when no flag is set.
    #[inline(always)]
    fn is_empty(self: ColliderChanges) -> bool {
        self.bits == 0
    }

    /// Returns `true` when every flag of `other` is in `self`.
    #[inline(always)]
    fn contains(self: ColliderChanges, other: ColliderChanges) -> bool {
        self.bits & other.bits == other.bits
    }

    /// Returns `true` when `self` and `other` share at least one flag.
    #[inline(always)]
    fn intersects(self: ColliderChanges, other: ColliderChanges) -> bool {
        self.bits & other.bits != 0
    }

    /// Sets every flag of `other`.
    #[inline(always)]
    fn insert(ref self: ColliderChanges, other: ColliderChanges) {
        self.bits = self.bits | other.bits;
    }

    /// Clears every flag of `other`.
    #[inline(always)]
    fn remove(ref self: ColliderChanges, other: ColliderChanges) {
        self.bits = self.bits & (0xffffffff - other.bits);
    }

    /// Do these changes justify a broad-phase update? True when `PARENT`, `POSITION`, `SHAPE`
    /// or `ENABLED_OR_DISABLED` is set.
    #[inline(always)]
    fn needs_broad_phase_update(self: ColliderChanges) -> bool {
        self.bits & BROAD_PHASE_MASK != 0
    }

    /// Do these changes justify a narrow-phase update? Upstream's `bits() > 2`, a plain integer
    /// comparison with no bitwise operation: false for the empty set, `IN_MODIFIED_SET` alone and
    /// `LOCAL_MASS_PROPERTIES` alone; true for every other value, including the two together
    /// (`3`), as upstream.
    #[inline(always)]
    fn needs_narrow_phase_update(self: ColliderChanges) -> bool {
        self.bits > 2
    }
}

#[cfg(test)]
mod alternatives {
    use super::{ColliderChanges, ENABLED_OR_DISABLED, PARENT, POSITION, SHAPE};

    /// Upstream's shape: `intersects(PARENT | POSITION | SHAPE | ENABLED_OR_DISABLED)` built
    /// at run time from the four flags.
    pub fn needs_broad_phase_update_or_chain(self: ColliderChanges) -> bool {
        self.bits & (PARENT.bits | POSITION.bits | SHAPE.bits | ENABLED_OR_DISABLED.bits) != 0
    }
}

#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use super::alternatives::needs_broad_phase_update_or_chain;
    use super::{
        ColliderChanges, ColliderChangesTrait, ENABLED_OR_DISABLED, GROUPS, IN_MODIFIED_SET,
        LOCAL_MASS_PROPERTIES, PARENT, PARENT_EFFECTIVE_DOMINANCE, POSITION, SHAPE, TYPE,
    };

    #[test]
    fn test_default_is_empty() {
        let default: ColliderChanges = Default::default();
        assert_eq!(default, ColliderChangesTrait::empty());
        assert!(default.is_empty());
        assert!(!POSITION.is_empty());
    }

    #[test]
    fn test_flag_values() {
        assert_eq!(IN_MODIFIED_SET.bits, 1);
        assert_eq!(LOCAL_MASS_PROPERTIES.bits, 2);
        assert_eq!(PARENT.bits, 4);
        assert_eq!(POSITION.bits, 8);
        assert_eq!(GROUPS.bits, 16);
        assert_eq!(SHAPE.bits, 32);
        assert_eq!(TYPE.bits, 64);
        assert_eq!(PARENT_EFFECTIVE_DOMINANCE.bits, 128);
        assert_eq!(ENABLED_OR_DISABLED.bits, 256);
        let union = IN_MODIFIED_SET
            | LOCAL_MASS_PROPERTIES
            | PARENT
            | POSITION
            | GROUPS
            | SHAPE
            | TYPE
            | PARENT_EFFECTIVE_DOMINANCE
            | ENABLED_OR_DISABLED;
        assert_eq!(union, ColliderChangesTrait::all());
    }

    #[test]
    fn test_set_operations() {
        let mut changes: ColliderChanges = Default::default();
        changes.insert(SHAPE);
        changes.insert(GROUPS);
        changes.insert(SHAPE);
        assert_eq!(changes, SHAPE | GROUPS);
        assert!(changes.contains(SHAPE | GROUPS));
        assert!(!changes.contains(SHAPE | TYPE));
        assert!(changes.intersects(SHAPE | TYPE));
        assert!(!changes.intersects(TYPE | PARENT));
        changes.remove(SHAPE | TYPE);
        assert_eq!(changes, GROUPS);
        assert_eq!(changes & GROUPS, GROUPS);
        let bits: u32 = changes.into();
        assert_eq!(bits, 16);
        let back: ColliderChanges = 16_u32.into();
        assert_eq!(back, GROUPS);
    }

    /// Broad phase: only `PARENT`, `POSITION`, `SHAPE` and `ENABLED_OR_DISABLED`.
    #[test]
    fn test_needs_broad_phase_update() {
        assert!(!ColliderChangesTrait::empty().needs_broad_phase_update());
        assert!(!IN_MODIFIED_SET.needs_broad_phase_update());
        assert!(!LOCAL_MASS_PROPERTIES.needs_broad_phase_update());
        assert!(PARENT.needs_broad_phase_update());
        assert!(POSITION.needs_broad_phase_update());
        assert!(!GROUPS.needs_broad_phase_update());
        assert!(SHAPE.needs_broad_phase_update());
        assert!(!TYPE.needs_broad_phase_update());
        assert!(!PARENT_EFFECTIVE_DOMINANCE.needs_broad_phase_update());
        assert!(ENABLED_OR_DISABLED.needs_broad_phase_update());
        assert!((GROUPS | POSITION).needs_broad_phase_update());
        assert!(ColliderChangesTrait::all().needs_broad_phase_update());
        assert!(!(IN_MODIFIED_SET | GROUPS | TYPE).needs_broad_phase_update());
    }

    /// Narrow phase: `bits > 2`: false for `0`, `IN_MODIFIED_SET` (1) and
    /// `LOCAL_MASS_PROPERTIES` (2) alone, true for their union (3), as upstream.
    #[test]
    fn test_needs_narrow_phase_update() {
        assert!(!ColliderChangesTrait::empty().needs_narrow_phase_update());
        assert!(!IN_MODIFIED_SET.needs_narrow_phase_update());
        assert!(!LOCAL_MASS_PROPERTIES.needs_narrow_phase_update());
        assert!((IN_MODIFIED_SET | LOCAL_MASS_PROPERTIES).needs_narrow_phase_update());
        assert!(PARENT.needs_narrow_phase_update());
        assert!(POSITION.needs_narrow_phase_update());
        assert!(GROUPS.needs_narrow_phase_update());
        assert!(SHAPE.needs_narrow_phase_update());
        assert!(TYPE.needs_narrow_phase_update());
        assert!(PARENT_EFFECTIVE_DOMINANCE.needs_narrow_phase_update());
        assert!(ENABLED_OR_DISABLED.needs_narrow_phase_update());
        assert!((IN_MODIFIED_SET | PARENT).needs_narrow_phase_update());
    }

    #[test]
    #[fuzzer(runs: 256, seed: 20260920)]
    fn fuzz_broad_phase_candidate_agrees(bits: u32) {
        let changes: ColliderChanges = bits.into();
        assert_eq!(changes.needs_broad_phase_update(), needs_broad_phase_update_or_chain(changes));
        assert_eq!(changes.needs_narrow_phase_update(), bits > 2);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_contains() {
        assert!(opaque(SHAPE | GROUPS).contains(GROUPS));
    }

    #[test]
    fn gas_insert() {
        let mut changes = opaque(SHAPE);
        changes.insert(GROUPS);
        assert!(changes.bits == 48);
    }

    #[test]
    fn gas_needs_broad_phase_update() {
        assert!(opaque(SHAPE | GROUPS).needs_broad_phase_update());
    }

    #[test]
    fn gas_needs_broad_phase_update_or_chain() {
        assert!(needs_broad_phase_update_or_chain(opaque(SHAPE | GROUPS)));
    }

    #[test]
    fn gas_needs_narrow_phase_update() {
        assert!(opaque(SHAPE | GROUPS).needs_narrow_phase_update());
    }
}
