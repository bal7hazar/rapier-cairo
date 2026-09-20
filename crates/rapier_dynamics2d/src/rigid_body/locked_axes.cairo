//! Axes a rigid-body may not move along (upstream `LockedAxes`), restricted to the 2D plane.
//!
//! Same representation and set operations as the flag masks of `rapier_core`, see
//! [`rapier_core::rigid_body::changes`] for the measured ranking of the candidates (`&` / `|` on
//! the raw integer win). The bit positions are upstream's, so a mask built for `rapier3d`
//! keeps its meaning here; the three axes that do not exist in 2D (`TRANSLATION_LOCKED_Z`,
//! `ROTATION_LOCKED_X`, `ROTATION_LOCKED_Y`) have no constant and are ignored by every consumer.

use core::traits::{BitAnd, BitOr};

/// Prevents translation along the world X axis.
pub const TRANSLATION_LOCKED_X: LockedAxes = LockedAxes { bits: 0x1 };

/// Prevents translation along the world Y axis.
pub const TRANSLATION_LOCKED_Y: LockedAxes = LockedAxes { bits: 0x2 };

/// Prevents any translation, i.e. both plane axes.
///
/// Upstream's constant also carries `TRANSLATION_LOCKED_Z` (`0x4`), which no 2D component reads.
pub const TRANSLATION_LOCKED: LockedAxes = LockedAxes { bits: 0x3 };

/// Prevents rotation, i.e. upstream's `ROTATION_LOCKED_Z`, the only rotation axis of the plane.
pub const ROTATION_LOCKED: LockedAxes = LockedAxes { bits: 0x20 };

/// Mask of the axes a rigid-body may not move along. Default: none, the body is free.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct LockedAxes {
    /// Raw mask, with upstream's bit positions; bits outside [`LockedAxesTrait::all`] are kept
    /// but never read in 2D.
    pub bits: u8,
}

/// Wraps a raw mask (upstream `from_bits_retain`).
pub impl U8IntoLockedAxes of Into<u8, LockedAxes> {
    #[inline(always)]
    fn into(self: u8) -> LockedAxes {
        LockedAxes { bits: self }
    }
}

/// Unwraps the raw mask.
pub impl LockedAxesIntoU8 of Into<LockedAxes, u8> {
    #[inline(always)]
    fn into(self: LockedAxes) -> u8 {
        self.bits
    }
}

/// Set union.
pub impl LockedAxesBitOr of BitOr<LockedAxes> {
    #[inline(always)]
    fn bitor(lhs: LockedAxes, rhs: LockedAxes) -> LockedAxes {
        LockedAxes { bits: lhs.bits | rhs.bits }
    }
}

/// Set intersection.
pub impl LockedAxesBitAnd of BitAnd<LockedAxes> {
    #[inline(always)]
    fn bitand(lhs: LockedAxes, rhs: LockedAxes) -> LockedAxes {
        LockedAxes { bits: lhs.bits & rhs.bits }
    }
}

/// Set operations on [`LockedAxes`].
#[generate_trait]
pub impl LockedAxesImpl of LockedAxesTrait {
    /// No axis locked.
    #[inline(always)]
    fn empty() -> LockedAxes {
        LockedAxes { bits: 0 }
    }

    /// Every axis of the plane locked: both translations and the rotation.
    #[inline(always)]
    fn all() -> LockedAxes {
        LockedAxes { bits: 0x23 }
    }

    /// Returns `true` when no axis is locked.
    #[inline(always)]
    fn is_empty(self: LockedAxes) -> bool {
        self.bits == 0
    }

    /// Returns `true` when every axis of `other` is locked in `self`.
    #[inline(always)]
    fn contains(self: LockedAxes, other: LockedAxes) -> bool {
        self.bits & other.bits == other.bits
    }

    /// Returns `true` when `self` and `other` share at least one axis.
    #[inline(always)]
    fn intersects(self: LockedAxes, other: LockedAxes) -> bool {
        self.bits & other.bits != 0
    }

    /// Locks every axis of `other`.
    #[inline(always)]
    fn insert(ref self: LockedAxes, other: LockedAxes) {
        self.bits = self.bits | other.bits;
    }

    /// Unlocks every axis of `other`.
    #[inline(always)]
    fn remove(ref self: LockedAxes, other: LockedAxes) {
        self.bits = self.bits & (0xff - other.bits);
    }
}

#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use super::{
        LockedAxes, LockedAxesTrait, ROTATION_LOCKED, TRANSLATION_LOCKED, TRANSLATION_LOCKED_X,
        TRANSLATION_LOCKED_Y,
    };

    #[test]
    fn test_flag_values_match_upstream_bits() {
        assert_eq!(TRANSLATION_LOCKED_X.bits, 0x01);
        assert_eq!(TRANSLATION_LOCKED_Y.bits, 0x02);
        assert_eq!(ROTATION_LOCKED.bits, 0x20);
        assert_eq!(TRANSLATION_LOCKED, TRANSLATION_LOCKED_X | TRANSLATION_LOCKED_Y);
        assert_eq!(TRANSLATION_LOCKED | ROTATION_LOCKED, LockedAxesTrait::all());
        let default: LockedAxes = Default::default();
        assert_eq!(default, LockedAxesTrait::empty());
        assert!(default.is_empty());
        assert!(!ROTATION_LOCKED.is_empty());
    }

    #[test]
    fn test_set_operations() {
        let mut axes: LockedAxes = Default::default();
        axes.insert(TRANSLATION_LOCKED_X);
        axes.insert(ROTATION_LOCKED);
        axes.insert(TRANSLATION_LOCKED_X);
        assert_eq!(axes, TRANSLATION_LOCKED_X | ROTATION_LOCKED);
        assert!(axes.contains(TRANSLATION_LOCKED_X));
        assert!(!axes.contains(TRANSLATION_LOCKED));
        assert!(axes.contains(LockedAxesTrait::empty()));
        assert!(axes.intersects(TRANSLATION_LOCKED));
        assert!(!axes.intersects(TRANSLATION_LOCKED_Y));
        assert!(!axes.intersects(LockedAxesTrait::empty()));
        axes.remove(TRANSLATION_LOCKED_Y);
        assert_eq!(axes, TRANSLATION_LOCKED_X | ROTATION_LOCKED);
        axes.remove(TRANSLATION_LOCKED);
        assert_eq!(axes, ROTATION_LOCKED);
        axes.remove(ROTATION_LOCKED);
        assert!(axes.is_empty());
        assert_eq!((TRANSLATION_LOCKED | ROTATION_LOCKED) & ROTATION_LOCKED, ROTATION_LOCKED);
        let bits: u8 = (TRANSLATION_LOCKED_Y | ROTATION_LOCKED).into();
        assert_eq!(bits, 0x22);
        let back: LockedAxes = 0x22_u8.into();
        assert_eq!(back, TRANSLATION_LOCKED_Y | ROTATION_LOCKED);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_contains() {
        assert!(opaque(TRANSLATION_LOCKED).contains(TRANSLATION_LOCKED_Y));
    }

    #[test]
    fn gas_intersects() {
        assert!(opaque(TRANSLATION_LOCKED).intersects(TRANSLATION_LOCKED_Y));
    }

    #[test]
    fn gas_insert() {
        let mut axes = opaque(TRANSLATION_LOCKED_X);
        axes.insert(opaque(ROTATION_LOCKED));
        assert_eq!(axes.bits, 0x21);
    }

    #[test]
    fn gas_remove() {
        let mut axes = opaque(LockedAxesTrait::all());
        axes.remove(opaque(ROTATION_LOCKED));
        assert_eq!(axes.bits, 0x03);
    }

    #[test]
    fn gas_bitor() {
        assert_eq!((opaque(TRANSLATION_LOCKED_X) | opaque(ROTATION_LOCKED)).bits, 0x21);
    }
}
