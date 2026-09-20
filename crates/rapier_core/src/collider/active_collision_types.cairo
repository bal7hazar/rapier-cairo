//! Which combinations of body types can collide (upstream `ActiveCollisionTypes`).
//!
//! # Encoding
//!
//! A `u16` of four nibbles, one per body type (`RigidBodyType::index`: dynamic 0, fixed 1,
//! kinematic-position 2, kinematic-velocity 3). Nibble `i` (bits `4i..4i+3`) holds the types
//! that type `i` may collide with, one bit per type index: bit `4 * t1 + t2` set means "`t1`
//! collides with `t2`". Two bodies collide when either direction is enabled, so
//! [`ActiveCollisionTypesTrait::test`] checks bits `4 * t1 + t2` **or** `4 * t2 + t1`.
//!
//! # Candidates for `test` (ranked by the `gas_*` probes)
//!
//! 1. `match` on the two types to the precomputed symmetric mask `2^(4 t1 + t2) | 2^(4 t2 + t1)`,
//!    then one `&` — **winner**;
//! 2. `alternatives::test_index_match` — the same 16 masks selected by `match` on the integer
//!    `4 * t1 + t2`;
//! 3. `alternatives::test_shift` — upstream's expression, `(bits >> 4 * t1) & 0xf & (1 << t2)`
//!    both ways, with the shifts done by `DivRem` by constants;
//! 4. `alternatives::test_arith` — no bitwise builtin: two `(bits / 2^k) % 2` bit tests.
//!
//! Cut: upstream maps the soft-frame body type to `Dynamic` first; the type does not exist here.

use core::traits::{BitAnd, BitOr};
use crate::rigid_body::body_type::RigidBodyType;

/// The default set: dynamic - dynamic, dynamic - kinematic and dynamic - fixed.
const DEFAULT_BITS: u16 = 0xf;

/// The bit mask of the two directed bits of a pair of body types, `2^(4 t1 + t2) | 2^(4 t2 + t1)`.
#[inline(always)]
fn pair_mask(type1: RigidBodyType, type2: RigidBodyType) -> u16 {
    match type1 {
        RigidBodyType::Dynamic => match type2 {
            RigidBodyType::Dynamic => 0x1,
            RigidBodyType::Fixed => 0x12,
            RigidBodyType::KinematicPositionBased => 0x104,
            RigidBodyType::KinematicVelocityBased => 0x1008,
        },
        RigidBodyType::Fixed => match type2 {
            RigidBodyType::Dynamic => 0x12,
            RigidBodyType::Fixed => 0x20,
            RigidBodyType::KinematicPositionBased => 0x240,
            RigidBodyType::KinematicVelocityBased => 0x2080,
        },
        RigidBodyType::KinematicPositionBased => match type2 {
            RigidBodyType::Dynamic => 0x104,
            RigidBodyType::Fixed => 0x240,
            RigidBodyType::KinematicPositionBased => 0x400,
            RigidBodyType::KinematicVelocityBased => 0x4800,
        },
        RigidBodyType::KinematicVelocityBased => match type2 {
            RigidBodyType::Dynamic => 0x1008,
            RigidBodyType::Fixed => 0x2080,
            RigidBodyType::KinematicPositionBased => 0x4800,
            RigidBodyType::KinematicVelocityBased => 0x8000,
        },
    }
}

/// Enables dynamic - dynamic collision detection.
pub const DYNAMIC_DYNAMIC: ActiveCollisionTypes = ActiveCollisionTypes { bits: 0x1 };

/// Enables dynamic - kinematic collision detection.
pub const DYNAMIC_KINEMATIC: ActiveCollisionTypes = ActiveCollisionTypes { bits: 0xc };

/// Enables dynamic - fixed collision detection.
pub const DYNAMIC_FIXED: ActiveCollisionTypes = ActiveCollisionTypes { bits: 0x2 };

/// Enables kinematic - kinematic collision detection (rarely needed).
pub const KINEMATIC_KINEMATIC: ActiveCollisionTypes = ActiveCollisionTypes { bits: 0xcc00 };

/// Enables kinematic - fixed collision detection (rarely needed).
pub const KINEMATIC_FIXED: ActiveCollisionTypes = ActiveCollisionTypes { bits: 0x2200 };

/// Enables fixed - fixed collision detection (rarely needed).
pub const FIXED_FIXED: ActiveCollisionTypes = ActiveCollisionTypes { bits: 0x20 };

/// Which combinations of body types can collide with each other. Not `Default`: see
/// [`ActiveCollisionTypesDefault`].
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ActiveCollisionTypes {
    /// Raw mask, bit `i` is the flag `1 << i`.
    pub bits: u16,
}

/// Wraps a raw mask (upstream `from_bits_retain`).
pub impl U16IntoActiveCollisionTypes of Into<u16, ActiveCollisionTypes> {
    #[inline(always)]
    fn into(self: u16) -> ActiveCollisionTypes {
        ActiveCollisionTypes { bits: self }
    }
}

/// Unwraps the raw mask.
pub impl ActiveCollisionTypesIntoU16 of Into<ActiveCollisionTypes, u16> {
    #[inline(always)]
    fn into(self: ActiveCollisionTypes) -> u16 {
        self.bits
    }
}

/// Set union.
pub impl ActiveCollisionTypesBitOr of BitOr<ActiveCollisionTypes> {
    #[inline(always)]
    fn bitor(lhs: ActiveCollisionTypes, rhs: ActiveCollisionTypes) -> ActiveCollisionTypes {
        ActiveCollisionTypes { bits: lhs.bits | rhs.bits }
    }
}

/// Set intersection.
pub impl ActiveCollisionTypesBitAnd of BitAnd<ActiveCollisionTypes> {
    #[inline(always)]
    fn bitand(lhs: ActiveCollisionTypes, rhs: ActiveCollisionTypes) -> ActiveCollisionTypes {
        ActiveCollisionTypes { bits: lhs.bits & rhs.bits }
    }
}

/// Set operations on [`ActiveCollisionTypes`].
#[generate_trait]
pub impl ActiveCollisionTypesImpl of ActiveCollisionTypesTrait {
    /// No flag set.
    #[inline(always)]
    fn empty() -> ActiveCollisionTypes {
        ActiveCollisionTypes { bits: 0 }
    }

    /// Every defined flag set.
    #[inline(always)]
    fn all() -> ActiveCollisionTypes {
        ActiveCollisionTypes { bits: 0xee2f }
    }

    /// Returns `true` when no flag is set.
    #[inline(always)]
    fn is_empty(self: ActiveCollisionTypes) -> bool {
        self.bits == 0
    }

    /// Returns `true` when every flag of `other` is in `self`.
    #[inline(always)]
    fn contains(self: ActiveCollisionTypes, other: ActiveCollisionTypes) -> bool {
        self.bits & other.bits == other.bits
    }

    /// Returns `true` when `self` and `other` share at least one flag.
    #[inline(always)]
    fn intersects(self: ActiveCollisionTypes, other: ActiveCollisionTypes) -> bool {
        self.bits & other.bits != 0
    }

    /// Sets every flag of `other`.
    #[inline(always)]
    fn insert(ref self: ActiveCollisionTypes, other: ActiveCollisionTypes) {
        self.bits = self.bits | other.bits;
    }

    /// Clears every flag of `other`.
    #[inline(always)]
    fn remove(ref self: ActiveCollisionTypes, other: ActiveCollisionTypes) {
        self.bits = self.bits & (0xffff - other.bits);
    }

    /// Tests whether contact should be computed between two rigid-bodies of the given types:
    /// `true` when `self` enables `type1 -> type2` or `type2 -> type1`. Symmetric.
    #[inline(always)]
    fn test(self: ActiveCollisionTypes, type1: RigidBodyType, type2: RigidBodyType) -> bool {
        self.bits & pair_mask(type1, type2) != 0
    }
}

/// Upstream default: `DYNAMIC_DYNAMIC | DYNAMIC_KINEMATIC | DYNAMIC_FIXED` (`0xf`).
pub impl ActiveCollisionTypesDefault of Default<ActiveCollisionTypes> {
    #[inline(always)]
    fn default() -> ActiveCollisionTypes {
        ActiveCollisionTypes { bits: DEFAULT_BITS }
    }
}

#[cfg(test)]
mod alternatives {
    use crate::rigid_body::body_type::{RigidBodyType, RigidBodyTypeTrait};
    use super::ActiveCollisionTypes;

    const NZ_16: NonZero<u16> = 16;
    const NZ_256: NonZero<u16> = 256;
    const NZ_4096: NonZero<u16> = 4096;
    const NZ_TWO: NonZero<u16> = 2;

    /// The nibble of `bits` selected by `type1`: `(bits >> 4 * type1) & 0xf`, shifted with
    /// `DivRem` by a constant.
    fn nibble(bits: u16, type1: RigidBodyType) -> u16 {
        match type1 {
            RigidBodyType::Dynamic => {
                let (_, low) = DivRem::div_rem(bits, NZ_16);
                low
            },
            RigidBodyType::Fixed => {
                let (shifted, _) = DivRem::div_rem(bits, NZ_16);
                let (_, low) = DivRem::div_rem(shifted, NZ_16);
                low
            },
            RigidBodyType::KinematicPositionBased => {
                let (shifted, _) = DivRem::div_rem(bits, NZ_256);
                let (_, low) = DivRem::div_rem(shifted, NZ_16);
                low
            },
            RigidBodyType::KinematicVelocityBased => {
                let (shifted, _) = DivRem::div_rem(bits, NZ_4096);
                shifted
            },
        }
    }

    /// `1 << type as u32`.
    fn type_bit(type1: RigidBodyType) -> u16 {
        match type1 {
            RigidBodyType::Dynamic => 1,
            RigidBodyType::Fixed => 2,
            RigidBodyType::KinematicPositionBased => 4,
            RigidBodyType::KinematicVelocityBased => 8,
        }
    }

    /// Upstream's expression, both directions.
    pub fn test_shift(
        self: ActiveCollisionTypes, type1: RigidBodyType, type2: RigidBodyType,
    ) -> bool {
        nibble(self.bits, type1) & type_bit(type2) != 0
            || nibble(self.bits, type2) & type_bit(type1) != 0
    }

    /// The symmetric mask selected by `match` on `4 * type1 + type2` (dense integer arms).
    pub fn test_index_match(
        self: ActiveCollisionTypes, type1: RigidBodyType, type2: RigidBodyType,
    ) -> bool {
        let mask: u16 = match type1.index() * 4 + type2.index() {
            0 => 0x1,
            1 | 4 => 0x12,
            2 | 8 => 0x104,
            3 | 12 => 0x1008,
            5 => 0x20,
            6 | 9 => 0x240,
            7 | 13 => 0x2080,
            10 => 0x400,
            11 | 14 => 0x4800,
            _ => 0x8000,
        };
        self.bits & mask != 0
    }

    /// `2^k` as a `NonZero<u16>`, for `k < 16`.
    fn pow2(k: u32) -> NonZero<u16> {
        match k {
            0 => 1,
            1 => 2,
            2 => 4,
            3 => 8,
            4 => 16,
            5 => 32,
            6 => 64,
            7 => 128,
            8 => 256,
            9 => 512,
            10 => 1024,
            11 => 2048,
            12 => 4096,
            13 => 8192,
            14 => 16384,
            _ => 32768,
        }
    }

    /// `(bits / 2^k) % 2 == 1`, no bitwise builtin.
    fn bit_set(bits: u16, k: u32) -> bool {
        let (shifted, _) = DivRem::div_rem(bits, pow2(k));
        let (_, bit) = DivRem::div_rem(shifted, NZ_TWO);
        bit == 1
    }

    /// Two arithmetic bit tests, `4 * type1 + type2` and `4 * type2 + type1`.
    pub fn test_arith(
        self: ActiveCollisionTypes, type1: RigidBodyType, type2: RigidBodyType,
    ) -> bool {
        let (i1, i2) = (type1.index(), type2.index());
        bit_set(self.bits, i1 * 4 + i2) || bit_set(self.bits, i2 * 4 + i1)
    }
}

#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use crate::rigid_body::body_type::RigidBodyType;
    use super::alternatives::{test_arith, test_index_match, test_shift};
    use super::{
        ActiveCollisionTypes, ActiveCollisionTypesTrait, DYNAMIC_DYNAMIC, DYNAMIC_FIXED,
        DYNAMIC_KINEMATIC, FIXED_FIXED, KINEMATIC_FIXED, KINEMATIC_KINEMATIC,
    };

    const D: RigidBodyType = RigidBodyType::Dynamic;
    const F: RigidBodyType = RigidBodyType::Fixed;
    const KP: RigidBodyType = RigidBodyType::KinematicPositionBased;
    const KV: RigidBodyType = RigidBodyType::KinematicVelocityBased;

    /// The four body types, in `RigidBodyType::index` order.
    fn types() -> Array<RigidBodyType> {
        array![D, F, KP, KV]
    }

    /// Coarse class of a body type: 0 dynamic, 1 fixed, 2 kinematic.
    fn class(t: RigidBodyType) -> u32 {
        match t {
            RigidBodyType::Dynamic => 0,
            RigidBodyType::Fixed => 1,
            _ => 2,
        }
    }

    /// The named flag that enables the pair of types (one flag per unordered class pair).
    fn expected_flag(t1: RigidBodyType, t2: RigidBodyType) -> ActiveCollisionTypes {
        let (a, b) = (class(t1), class(t2));
        let (low, high) = if a <= b {
            (a, b)
        } else {
            (b, a)
        };
        match (low, high) {
            (0, 0) => DYNAMIC_DYNAMIC,
            (0, 1) => DYNAMIC_FIXED,
            (0, _) => DYNAMIC_KINEMATIC,
            (1, 1) => FIXED_FIXED,
            (1, _) => KINEMATIC_FIXED,
            _ => KINEMATIC_KINEMATIC,
        }
    }

    /// The shipped implementation and every candidate, in both argument orders.
    fn assert_test(
        flags: ActiveCollisionTypes, t1: RigidBodyType, t2: RigidBodyType, expected: bool,
    ) {
        assert_eq!(flags.test(t1, t2), expected);
        assert_eq!(flags.test(t2, t1), expected);
        assert_eq!(test_shift(flags, t1, t2), expected);
        assert_eq!(test_shift(flags, t2, t1), expected);
        assert_eq!(test_index_match(flags, t1, t2), expected);
        assert_eq!(test_index_match(flags, t2, t1), expected);
        assert_eq!(test_arith(flags, t1, t2), expected);
        assert_eq!(test_arith(flags, t2, t1), expected);
    }

    #[test]
    fn test_flag_values() {
        assert_eq!(DYNAMIC_DYNAMIC.bits, 0b0000_0000_0000_0001);
        assert_eq!(DYNAMIC_KINEMATIC.bits, 0b0000_0000_0000_1100);
        assert_eq!(DYNAMIC_FIXED.bits, 0b0000_0000_0000_0010);
        assert_eq!(KINEMATIC_KINEMATIC.bits, 0b1100_1100_0000_0000);
        assert_eq!(KINEMATIC_FIXED.bits, 0b0010_0010_0000_0000);
        assert_eq!(FIXED_FIXED.bits, 0b0000_0000_0010_0000);
        let union = DYNAMIC_DYNAMIC
            | DYNAMIC_KINEMATIC
            | DYNAMIC_FIXED
            | KINEMATIC_KINEMATIC
            | KINEMATIC_FIXED
            | FIXED_FIXED;
        assert_eq!(union, ActiveCollisionTypesTrait::all());
        assert_eq!(ActiveCollisionTypesTrait::all().bits, 0xee2f);
    }

    #[test]
    fn test_default() {
        let default: ActiveCollisionTypes = Default::default();
        assert_eq!(default, DYNAMIC_DYNAMIC | DYNAMIC_KINEMATIC | DYNAMIC_FIXED);
        assert_eq!(default.bits, 0xf);
    }

    /// Each named flag alone enables exactly its class pair, for every ordered pair of types.
    #[test]
    fn test_truth_table_one_flag_at_a_time() {
        let flags = array![
            DYNAMIC_DYNAMIC, DYNAMIC_KINEMATIC, DYNAMIC_FIXED, KINEMATIC_KINEMATIC, KINEMATIC_FIXED,
            FIXED_FIXED,
        ];
        for flag in flags {
            for t1 in types() {
                for t2 in types() {
                    assert_test(flag, t1, t2, expected_flag(t1, t2) == flag);
                }
            }
        }
    }

    /// The default collides dynamic with everything and nothing else.
    #[test]
    fn test_truth_table_default() {
        let default: ActiveCollisionTypes = Default::default();
        assert_test(default, D, D, true);
        assert_test(default, D, F, true);
        assert_test(default, D, KP, true);
        assert_test(default, D, KV, true);
        assert_test(default, F, F, false);
        assert_test(default, F, KP, false);
        assert_test(default, F, KV, false);
        assert_test(default, KP, KP, false);
        assert_test(default, KP, KV, false);
        assert_test(default, KV, KV, false);
    }

    #[test]
    fn test_truth_table_empty_and_all() {
        for t1 in types() {
            for t2 in types() {
                assert_test(ActiveCollisionTypesTrait::empty(), t1, t2, false);
                assert_test(ActiveCollisionTypesTrait::all(), t1, t2, true);
            }
        }
    }

    /// Enabling the kinematic pairs makes both kinematic types collide, in every combination.
    #[test]
    fn test_kinematic_kinematic_covers_both_kinematic_types() {
        let flags = (ActiveCollisionTypes { bits: 0xf }) | KINEMATIC_KINEMATIC;
        assert_test(flags, KP, KP, true);
        assert_test(flags, KP, KV, true);
        assert_test(flags, KV, KV, true);
        assert_test(flags, KV, F, false);
    }

    #[test]
    fn test_set_operations() {
        let mut flags: ActiveCollisionTypes = Default::default();
        flags.insert(FIXED_FIXED);
        assert!(flags.contains(FIXED_FIXED));
        assert!(flags.contains(DYNAMIC_KINEMATIC));
        assert!(!flags.contains(KINEMATIC_FIXED));
        assert!(flags.intersects(KINEMATIC_FIXED | FIXED_FIXED));
        flags.remove(DYNAMIC_KINEMATIC);
        assert_eq!(flags, DYNAMIC_DYNAMIC | DYNAMIC_FIXED | FIXED_FIXED);
        assert_eq!(flags & FIXED_FIXED, FIXED_FIXED);
        assert!(ActiveCollisionTypesTrait::empty().is_empty());
        let bits: u16 = flags.into();
        assert_eq!(bits, 0x23);
        let back: ActiveCollisionTypes = 0x23_u16.into();
        assert_eq!(back, flags);
    }

    #[test]
    #[fuzzer(runs: 256, seed: 20260920)]
    fn fuzz_candidates_agree(bits: u16) {
        let flags: ActiveCollisionTypes = bits.into();
        for t1 in types() {
            for t2 in types() {
                let expected = flags.test(t1, t2);
                assert_eq!(test_shift(flags, t1, t2), expected);
                assert_eq!(test_index_match(flags, t1, t2), expected);
                assert_eq!(test_arith(flags, t1, t2), expected);
                assert_eq!(flags.test(t2, t1), expected);
            }
        }
    }

    #[test]
    fn gas_baseline() {}

    /// Cost of building the probe inputs alone: subtract it from the `test*` probes.
    #[test]
    fn gas_inputs() {
        let flags: ActiveCollisionTypes = Default::default();
        assert!(opaque(flags).bits != 0 && opaque(D) != opaque(KV));
    }

    #[test]
    fn gas_test_mask() {
        let flags: ActiveCollisionTypes = Default::default();
        assert!(opaque(flags).test(opaque(D), opaque(KV)));
    }

    #[test]
    fn gas_test_mask_miss() {
        let flags: ActiveCollisionTypes = Default::default();
        assert!(!opaque(flags).test(opaque(KP), opaque(F)));
    }

    #[test]
    fn gas_test_index_match() {
        let flags: ActiveCollisionTypes = Default::default();
        assert!(test_index_match(opaque(flags), opaque(D), opaque(KV)));
    }

    #[test]
    fn gas_test_shift() {
        let flags: ActiveCollisionTypes = Default::default();
        assert!(test_shift(opaque(flags), opaque(D), opaque(KV)));
    }

    #[test]
    fn gas_test_shift_miss() {
        let flags: ActiveCollisionTypes = Default::default();
        assert!(!test_shift(opaque(flags), opaque(KP), opaque(F)));
    }

    #[test]
    fn gas_test_arith() {
        let flags: ActiveCollisionTypes = Default::default();
        assert!(test_arith(opaque(flags), opaque(D), opaque(KV)));
    }

    #[test]
    fn gas_test_arith_miss() {
        let flags: ActiveCollisionTypes = Default::default();
        assert!(!test_arith(opaque(flags), opaque(KP), opaque(F)));
    }

    #[test]
    fn gas_contains_u16() {
        let flags: ActiveCollisionTypes = Default::default();
        assert!(opaque(flags).contains(DYNAMIC_FIXED));
    }

    #[test]
    fn gas_insert_u16() {
        let mut flags: ActiveCollisionTypes = Default::default();
        flags = opaque(flags);
        flags.insert(FIXED_FIXED);
        assert!(flags.bits == 0x2f);
    }
}
