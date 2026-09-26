//! `AxesMask` (upstream `dynamics/rigid_body_components.rs`), 2D: the flags naming the axes a
//! constraint or a PD controller acts on.
//!
//! Upstream is a `bitflags` `u8` with `LIN_X = 1 << 0`, `LIN_Y = 1 << 1`, `ANG_Z = 1 << 5` in 2D
//! (`LIN_Z`, `ANG_X` and `ANG_Y` are 3D-only). The bit positions are kept, so `bits` is
//! upstream's `bits()`. `union` / `intersection` use the bitwise builtin: measured ~10x cheaper
//! than the `DivRem` candidates kept in `alternatives`. Only the three 2D bits are meaningful;
//! [`AxesMaskTrait::from_bits`] rejects the others, and the functions below assume a mask built
//! through it or the constants.

/// Flags of the axes a constraint acts on. Default: [`AxesMaskTrait::empty`].
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct AxesMask {
    /// Upstream's `bits()`: `LIN_X` 1, `LIN_Y` 2, `ANG_Z` 32.
    pub bits: u8,
}

/// The translational X axis.
pub const LIN_X: AxesMask = AxesMask { bits: 1 };
/// The translational Y axis.
pub const LIN_Y: AxesMask = AxesMask { bits: 2 };
/// The rotational Z axis (the only rotation in 2D).
pub const ANG_Z: AxesMask = AxesMask { bits: 32 };
/// Every 2D axis (upstream `AxesMask::all()`).
pub const ALL: AxesMask = AxesMask { bits: 35 };

/// The empty mask (upstream `AxesMask::empty()`, its `Default`).
pub impl AxesMaskDefault of Default<AxesMask> {
    #[inline(always)]
    fn default() -> AxesMask {
        AxesMask { bits: 0 }
    }
}

/// Set operations of [`AxesMask`] (the `bitflags` API the port needs).
#[generate_trait]
pub impl AxesMaskImpl of AxesMaskTrait {
    /// No axis.
    #[inline(always)]
    fn empty() -> AxesMask {
        AxesMask { bits: 0 }
    }

    /// Every 2D axis.
    #[inline(always)]
    fn all() -> AxesMask {
        ALL
    }

    /// The raw bits.
    #[inline(always)]
    fn bits(self: AxesMask) -> u8 {
        self.bits
    }

    /// The mask of `bits`, or `None` when it has a bit that is not a 2D axis (upstream
    /// `from_bits`, which also rejects the 3D-only bits here).
    fn from_bits(bits: u8) -> Option<AxesMask> {
        if bits < 36 && (bits < 4 || bits >= 32) {
            Some(AxesMask { bits })
        } else {
            None
        }
    }

    /// Is no axis set?
    #[inline(always)]
    fn is_empty(self: AxesMask) -> bool {
        self.bits == 0
    }

    /// Is every axis of `other` set in `self`?
    #[inline(always)]
    fn contains(self: AxesMask, other: AxesMask) -> bool {
        self.bits & other.bits == other.bits
    }

    /// The axes set in `self`, `other` or both (`self | other`).
    #[inline(always)]
    fn union(self: AxesMask, other: AxesMask) -> AxesMask {
        AxesMask { bits: self.bits | other.bits }
    }

    /// The axes set in both (`self & other`).
    #[inline(always)]
    fn intersection(self: AxesMask, other: AxesMask) -> AxesMask {
        AxesMask { bits: self.bits & other.bits }
    }
}

/// `DivRem` candidates for the set operations (rejected: ~10x the bitwise builtin's gas).
#[cfg(test)]
mod alternatives {
    use super::AxesMask;

    /// `bits` has the flag whose value is `flag` (`1`, `2` or `32`): `(bits / flag) % 2 == 1`.
    fn has(bits: u8, flag: NonZero<u8>) -> bool {
        let (quotient, _) = DivRem::div_rem(bits, flag);
        let (_, bit) = DivRem::div_rem(quotient, 2);
        bit == 1
    }

    /// `value` if the flag of `a` and `b` combine as wanted (`both`: AND, else OR), else `0`.
    fn pick(a: u8, b: u8, flag: NonZero<u8>, value: u8, both: bool) -> u8 {
        let (x, y) = (has(a, flag), has(b, flag));
        if (both && x && y) || (!both && (x || y)) {
            value
        } else {
            0
        }
    }

    pub fn union_divrem(a: AxesMask, b: AxesMask) -> AxesMask {
        let (x, y) = (a.bits, b.bits);
        AxesMask {
            bits: pick(x, y, 1, 1, false) + pick(x, y, 2, 2, false) + pick(x, y, 32, 32, false),
        }
    }

    pub fn intersection_divrem(a: AxesMask, b: AxesMask) -> AxesMask {
        let (x, y) = (a.bits, b.bits);
        AxesMask {
            bits: pick(x, y, 1, 1, true) + pick(x, y, 2, 2, true) + pick(x, y, 32, 32, true),
        }
    }
}

#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use super::alternatives::{intersection_divrem, union_divrem};
    use super::{ALL, ANG_Z, AxesMask, AxesMaskTrait, LIN_X, LIN_Y};

    #[test]
    fn test_default_is_empty() {
        let default: AxesMask = Default::default();
        assert_eq!(default, AxesMaskTrait::empty());
        assert!(default.is_empty());
        assert!(!ALL.is_empty());
    }

    #[test]
    fn test_constants_are_upstream_bits() {
        assert_eq!(LIN_X.bits(), 1);
        assert_eq!(LIN_Y.bits(), 2);
        assert_eq!(ANG_Z.bits(), 32);
        assert_eq!(AxesMaskTrait::all(), ALL);
        assert_eq!(ALL, LIN_X.union(LIN_Y).union(ANG_Z));
    }

    #[test]
    fn test_from_bits() {
        let valid = array![0, 1, 2, 3, 32, 33, 34, 35];
        for bits in valid {
            assert_eq!(AxesMaskTrait::from_bits(bits), Some(AxesMask { bits }));
        }
        let invalid = array![4, 8, 16, 31, 36, 64, 255];
        for bits in invalid {
            assert_eq!(AxesMaskTrait::from_bits(bits), None);
        }
    }

    #[test]
    fn test_set_operations() {
        let xy = LIN_X.union(LIN_Y);
        let yz = LIN_Y.union(ANG_Z);
        assert_eq!(xy, AxesMask { bits: 3 });
        assert_eq!(xy.union(yz), ALL);
        assert_eq!(xy.intersection(yz), LIN_Y);
        assert_eq!(LIN_X.intersection(ANG_Z), AxesMaskTrait::empty());
        assert!(ALL.contains(xy));
        assert!(xy.contains(LIN_Y));
        assert!(!xy.contains(yz));
        assert!(xy.contains(AxesMaskTrait::empty()));
        assert!(AxesMaskTrait::empty().contains(AxesMaskTrait::empty()));
    }

    #[test]
    fn test_alternatives_agree_with_shipped() {
        let masks = array![LIN_X, LIN_Y, ANG_Z, ALL, AxesMaskTrait::empty(), LIN_X.union(ANG_Z)];
        for a in masks.span() {
            for b in masks.span() {
                assert_eq!(union_divrem(*a, *b), (*a).union(*b));
                assert_eq!(intersection_divrem(*a, *b), (*a).intersection(*b));
            }
        }
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_union() {
        let (a, b) = (opaque(LIN_X), opaque(ANG_Z));
        a.union(b);
    }

    #[test]
    fn gas_union_divrem() {
        let (a, b) = (opaque(LIN_X), opaque(ANG_Z));
        union_divrem(a, b);
    }

    #[test]
    fn gas_intersection() {
        let (a, b) = (opaque(ALL), opaque(LIN_Y));
        a.intersection(b);
    }

    #[test]
    fn gas_intersection_divrem() {
        let (a, b) = (opaque(ALL), opaque(LIN_Y));
        intersection_divrem(a, b);
    }

    #[test]
    fn gas_contains() {
        let (a, b) = (opaque(ALL), opaque(LIN_Y));
        a.contains(b);
    }

    #[test]
    fn gas_from_bits() {
        let _ = AxesMaskTrait::from_bits(opaque(33_u8));
    }
}
