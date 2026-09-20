//! Pairwise collision / solver filtering with bit masks (upstream
//! `rapier::geometry::{Group, InteractionGroups, InteractionTestMode}`).
//!
//! Two objects interact when their groups pass [`InteractionGroupsTrait::test`]:
//!
//! * `And` mode: `(a.memberships & b.filter) != 0 && (b.memberships & a.filter) != 0`;
//! * `Or` mode: the same with `||`, used only when **both** sides ask for `Or`.
//!
//! Candidates for the mask test (ranked by the `gas_*` probes of this module):
//!
//! 1. two `u32` bitwise ANDs with short-circuit — **winner**;
//! 2. `alternatives::*_packed_u64` — both masks of each side packed in one `u64` so that a
//!    single bitwise builtin call yields both intersections. The packing (one bounded-int
//!    mul + add per side) and the final split cost more than the builtin call they save, except
//!    for `test_or` when the first direction misses (both ANDs evaluated by the winner);
//! 3. `alternatives::*_packed_u128` — same packing through `felt252` into a `u128`;
//! 4. `alternatives::*_arith` — no bitwise builtin at all, bit-by-bit `DivRem` loop.

use core::traits::{BitAnd, BitNot, BitOr};

/// A set of up to 32 collision groups, one per bit (upstream `Group`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Group {
    /// Raw mask: bit `i` set means membership of group `i + 1`.
    pub bits: u32,
}

/// Group 1 (bit 0).
pub const GROUP_1: Group = Group { bits: 0x1 };
/// Group 2 (bit 1).
pub const GROUP_2: Group = Group { bits: 0x2 };
/// Group 3 (bit 2).
pub const GROUP_3: Group = Group { bits: 0x4 };
/// Group 4 (bit 3).
pub const GROUP_4: Group = Group { bits: 0x8 };
/// Group 5 (bit 4).
pub const GROUP_5: Group = Group { bits: 0x10 };
/// Group 6 (bit 5).
pub const GROUP_6: Group = Group { bits: 0x20 };
/// Group 7 (bit 6).
pub const GROUP_7: Group = Group { bits: 0x40 };
/// Group 8 (bit 7).
pub const GROUP_8: Group = Group { bits: 0x80 };
/// Group 9 (bit 8).
pub const GROUP_9: Group = Group { bits: 0x100 };
/// Group 10 (bit 9).
pub const GROUP_10: Group = Group { bits: 0x200 };
/// Group 11 (bit 10).
pub const GROUP_11: Group = Group { bits: 0x400 };
/// Group 12 (bit 11).
pub const GROUP_12: Group = Group { bits: 0x800 };
/// Group 13 (bit 12).
pub const GROUP_13: Group = Group { bits: 0x1000 };
/// Group 14 (bit 13).
pub const GROUP_14: Group = Group { bits: 0x2000 };
/// Group 15 (bit 14).
pub const GROUP_15: Group = Group { bits: 0x4000 };
/// Group 16 (bit 15).
pub const GROUP_16: Group = Group { bits: 0x8000 };
/// Group 17 (bit 16).
pub const GROUP_17: Group = Group { bits: 0x10000 };
/// Group 18 (bit 17).
pub const GROUP_18: Group = Group { bits: 0x20000 };
/// Group 19 (bit 18).
pub const GROUP_19: Group = Group { bits: 0x40000 };
/// Group 20 (bit 19).
pub const GROUP_20: Group = Group { bits: 0x80000 };
/// Group 21 (bit 20).
pub const GROUP_21: Group = Group { bits: 0x100000 };
/// Group 22 (bit 21).
pub const GROUP_22: Group = Group { bits: 0x200000 };
/// Group 23 (bit 22).
pub const GROUP_23: Group = Group { bits: 0x400000 };
/// Group 24 (bit 23).
pub const GROUP_24: Group = Group { bits: 0x800000 };
/// Group 25 (bit 24).
pub const GROUP_25: Group = Group { bits: 0x1000000 };
/// Group 26 (bit 25).
pub const GROUP_26: Group = Group { bits: 0x2000000 };
/// Group 27 (bit 26).
pub const GROUP_27: Group = Group { bits: 0x4000000 };
/// Group 28 (bit 27).
pub const GROUP_28: Group = Group { bits: 0x8000000 };
/// Group 29 (bit 28).
pub const GROUP_29: Group = Group { bits: 0x10000000 };
/// Group 30 (bit 29).
pub const GROUP_30: Group = Group { bits: 0x20000000 };
/// Group 31 (bit 30).
pub const GROUP_31: Group = Group { bits: 0x40000000 };
/// Group 32 (bit 31).
pub const GROUP_32: Group = Group { bits: 0x80000000 };

/// Every group.
pub const ALL: Group = Group { bits: 0xffffffff };

/// No group.
pub const NONE: Group = Group { bits: 0 };

/// Wraps a raw mask; every `u32` is a valid group set (upstream `from_bits_retain`).
pub impl U32IntoGroup of Into<u32, Group> {
    #[inline(always)]
    fn into(self: u32) -> Group {
        Group { bits: self }
    }
}

/// Unwraps the raw mask.
pub impl GroupIntoU32 of Into<Group, u32> {
    #[inline(always)]
    fn into(self: Group) -> u32 {
        self.bits
    }
}

/// Set union.
pub impl GroupBitOr of BitOr<Group> {
    #[inline(always)]
    fn bitor(lhs: Group, rhs: Group) -> Group {
        Group { bits: lhs.bits | rhs.bits }
    }
}

/// Set intersection.
pub impl GroupBitAnd of BitAnd<Group> {
    #[inline(always)]
    fn bitand(lhs: Group, rhs: Group) -> Group {
        Group { bits: lhs.bits & rhs.bits }
    }
}

/// Set complement within the 32 groups; computed as `0xffffffff - bits`, no bitwise builtin.
pub impl GroupBitNot of BitNot<Group> {
    #[inline(always)]
    fn bitnot(a: Group) -> Group {
        Group { bits: ALL.bits - a.bits }
    }
}

/// Set predicates on [`Group`].
#[generate_trait]
pub impl GroupImpl of GroupTrait {
    /// Returns `true` when the two sets share at least one group.
    #[inline(always)]
    fn intersects(self: Group, other: Group) -> bool {
        self.bits & other.bits != 0
    }

    /// Returns `true` when every group of `other` is in `self`.
    #[inline(always)]
    fn contains(self: Group, other: Group) -> bool {
        self.bits & other.bits == other.bits
    }
}

/// How the two directed mask tests are combined (upstream `InteractionTestMode`).
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub enum InteractionTestMode {
    /// Both directions must match. Wins whenever at least one side uses it.
    #[default]
    And,
    /// One direction is enough; applies only when both sides use it.
    Or,
}

/// Pairwise filter: the groups an object belongs to and the groups it accepts.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct InteractionGroups {
    /// Groups this object is part of.
    pub memberships: Group,
    /// Groups this object can interact with.
    pub filter: Group,
    /// Combination rule, see [`InteractionTestMode`].
    pub test_mode: InteractionTestMode,
}

/// Upstream default: member of `GROUP_1`, interacts with everything, `And` mode.
pub impl InteractionGroupsDefault of Default<InteractionGroups> {
    #[inline(always)]
    fn default() -> InteractionGroups {
        InteractionGroups { memberships: GROUP_1, filter: ALL, test_mode: InteractionTestMode::And }
    }
}

/// Constructors and pairwise tests of [`InteractionGroups`].
#[generate_trait]
pub impl InteractionGroupsImpl of InteractionGroupsTrait {
    /// Builds interaction groups from their three parts.
    #[inline(always)]
    fn new(memberships: Group, filter: Group, test_mode: InteractionTestMode) -> InteractionGroups {
        InteractionGroups { memberships, filter, test_mode }
    }

    /// Member of every group, interacts with every group (`And` mode).
    #[inline(always)]
    fn all() -> InteractionGroups {
        InteractionGroups { memberships: ALL, filter: ALL, test_mode: InteractionTestMode::And }
    }

    /// Member of no group, interacts with nothing (`And` mode).
    #[inline(always)]
    fn none() -> InteractionGroups {
        InteractionGroups { memberships: NONE, filter: NONE, test_mode: InteractionTestMode::And }
    }

    /// Returns a copy with `memberships` replaced.
    #[inline(always)]
    fn with_memberships(self: InteractionGroups, memberships: Group) -> InteractionGroups {
        InteractionGroups { memberships, ..self }
    }

    /// Returns a copy with `filter` replaced.
    #[inline(always)]
    fn with_filter(self: InteractionGroups, filter: Group) -> InteractionGroups {
        InteractionGroups { filter, ..self }
    }

    /// `(self.memberships & rhs.filter) != 0 && (rhs.memberships & self.filter) != 0`,
    /// ignoring the test modes. The second AND is skipped when the first one is empty.
    #[inline(always)]
    fn test_and(self: InteractionGroups, rhs: InteractionGroups) -> bool {
        self.memberships.bits & rhs.filter.bits != 0 && rhs.memberships.bits & self.filter.bits != 0
    }

    /// `(self.memberships & rhs.filter) != 0 || (rhs.memberships & self.filter) != 0`,
    /// ignoring the test modes. The second AND is skipped when the first one matches.
    #[inline(always)]
    fn test_or(self: InteractionGroups, rhs: InteractionGroups) -> bool {
        self.memberships.bits & rhs.filter.bits != 0 || rhs.memberships.bits & self.filter.bits != 0
    }

    /// Returns `true` when the two objects may interact. Symmetric. Uses
    /// [`test_or`](InteractionGroupsTrait::test_or) when both sides are in `Or` mode and
    /// [`test_and`](InteractionGroupsTrait::test_and) otherwise, as upstream.
    fn test(self: InteractionGroups, rhs: InteractionGroups) -> bool {
        match (self.test_mode, rhs.test_mode) {
            (InteractionTestMode::Or, InteractionTestMode::Or) => self.test_or(rhs),
            _ => self.test_and(rhs),
        }
    }
}

#[cfg(test)]
mod alternatives {
    #[feature("bounded-int-utils")]
    use core::internal::bounded_int::{
        self, AddHelper, BoundedInt, DivRemHelper, MulHelper, UnitInt,
    };
    use super::InteractionGroups;

    const TWO_POW_32: felt252 = 0x100000000;
    const TWO_POW_32_UNIT: UnitInt<0x100000000> = 0x100000000;
    const NZ_TWO_POW_32_UNIT: NonZero<UnitInt<0x100000000>> = 0x100000000;
    const NZ_TWO_POW_32_U128: NonZero<u128> = 0x100000000;
    const NZ_TWO: NonZero<u32> = 2;

    pub mod errors {
        pub const PACK_OVERFLOW: felt252 = 'Groups: pack overflow';
    }

    impl MulU32ByTwoPow32 of MulHelper<u32, UnitInt<0x100000000>> {
        type Result = BoundedInt<0, 0xffffffff00000000>;
    }

    impl AddShiftedU32AndU32 of AddHelper<BoundedInt<0, 0xffffffff00000000>, u32> {
        type Result = BoundedInt<0, 0xffffffffffffffff>;
    }

    impl DivRemU64ByTwoPow32 of DivRemHelper<u64, UnitInt<0x100000000>> {
        type DivT = BoundedInt<0, 0xffffffff>;
        type RemT = BoundedInt<0, 0xffffffff>;
    }

    /// `high * 2^32 + low` without overflow checks.
    #[inline(always)]
    fn pack_u64(high: u32, low: u32) -> u64 {
        bounded_int::upcast(bounded_int::add(bounded_int::mul(high, TWO_POW_32_UNIT), low))
    }

    /// One `u64` holding both directed intersections: `a.m & b.f` (low), `b.m & a.f` (high).
    #[inline(always)]
    fn intersections_u64(a: InteractionGroups, b: InteractionGroups) -> u64 {
        pack_u64(b.memberships.bits, a.memberships.bits) & pack_u64(a.filter.bits, b.filter.bits)
    }

    /// One `u128` holding both directed intersections, packed with felt arithmetic.
    #[inline(always)]
    fn intersections_u128(a: InteractionGroups, b: InteractionGroups) -> u128 {
        let members: felt252 = b.memberships.bits.into() * TWO_POW_32 + a.memberships.bits.into();
        let filters: felt252 = a.filter.bits.into() * TWO_POW_32 + b.filter.bits.into();
        let members: u128 = members.try_into().expect(errors::PACK_OVERFLOW);
        let filters: u128 = filters.try_into().expect(errors::PACK_OVERFLOW);
        members & filters
    }

    pub fn test_and_packed_u64(a: InteractionGroups, b: InteractionGroups) -> bool {
        let (high, low) = bounded_int::div_rem(intersections_u64(a, b), NZ_TWO_POW_32_UNIT);
        let high: u32 = bounded_int::upcast(high);
        let low: u32 = bounded_int::upcast(low);
        low != 0 && high != 0
    }

    pub fn test_or_packed_u64(a: InteractionGroups, b: InteractionGroups) -> bool {
        intersections_u64(a, b) != 0
    }

    pub fn test_and_packed_u128(a: InteractionGroups, b: InteractionGroups) -> bool {
        let (high, low) = DivRem::div_rem(intersections_u128(a, b), NZ_TWO_POW_32_U128);
        low != 0 && high != 0
    }

    pub fn test_or_packed_u128(a: InteractionGroups, b: InteractionGroups) -> bool {
        intersections_u128(a, b) != 0
    }

    /// `(x & y) != 0` without the bitwise builtin: peel the low bits until both are set.
    fn intersects_arith(mut x: u32, mut y: u32) -> bool {
        loop {
            if x == 0 || y == 0 {
                break false;
            }
            let (x_high, x_bit) = DivRem::div_rem(x, NZ_TWO);
            let (y_high, y_bit) = DivRem::div_rem(y, NZ_TWO);
            if x_bit + y_bit == 2 {
                break true;
            }
            x = x_high;
            y = y_high;
        }
    }

    pub fn test_and_arith(a: InteractionGroups, b: InteractionGroups) -> bool {
        intersects_arith(a.memberships.bits, b.filter.bits)
            && intersects_arith(b.memberships.bits, a.filter.bits)
    }

    pub fn test_or_arith(a: InteractionGroups, b: InteractionGroups) -> bool {
        intersects_arith(a.memberships.bits, b.filter.bits)
            || intersects_arith(b.memberships.bits, a.filter.bits)
    }
}

#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use super::alternatives::{
        test_and_arith, test_and_packed_u128, test_and_packed_u64, test_or_arith,
        test_or_packed_u128, test_or_packed_u64,
    };
    use super::{
        ALL, GROUP_1, GROUP_17, GROUP_2, GROUP_3, GROUP_32, Group, GroupTrait, InteractionGroups,
        InteractionGroupsTrait, InteractionTestMode, NONE,
    };

    const AND: InteractionTestMode = InteractionTestMode::And;
    const OR: InteractionTestMode = InteractionTestMode::Or;

    fn groups(memberships: u32, filter: u32, test_mode: InteractionTestMode) -> InteractionGroups {
        InteractionGroupsTrait::new(memberships.into(), filter.into(), test_mode)
    }

    /// Asserts `test_and` / `test_or` on the shipped implementation and every candidate, in
    /// both argument orders.
    fn assert_tests(a: InteractionGroups, b: InteractionGroups, and: bool, or: bool) {
        assert_eq!(a.test_and(b), and);
        assert_eq!(b.test_and(a), and);
        assert_eq!(a.test_or(b), or);
        assert_eq!(b.test_or(a), or);
        assert_eq!(test_and_packed_u64(a, b), and);
        assert_eq!(test_and_packed_u64(b, a), and);
        assert_eq!(test_or_packed_u64(a, b), or);
        assert_eq!(test_or_packed_u64(b, a), or);
        assert_eq!(test_and_packed_u128(a, b), and);
        assert_eq!(test_and_packed_u128(b, a), and);
        assert_eq!(test_or_packed_u128(a, b), or);
        assert_eq!(test_or_packed_u128(b, a), or);
        assert_eq!(test_and_arith(a, b), and);
        assert_eq!(test_and_arith(b, a), and);
        assert_eq!(test_or_arith(a, b), or);
        assert_eq!(test_or_arith(b, a), or);
    }

    #[test]
    fn test_group_constants() {
        assert_eq!(GROUP_1.bits, 1);
        assert_eq!(GROUP_2.bits, 2);
        assert_eq!(GROUP_17.bits, 0x10000);
        assert_eq!(GROUP_32.bits, 0x80000000);
        assert_eq!(ALL.bits, 0xffffffff);
        assert_eq!(NONE.bits, 0);
    }

    #[test]
    fn test_group_conversions() {
        let group: Group = 0x80000001_u32.into();
        assert_eq!(group, GROUP_1 | GROUP_32);
        let bits: u32 = group.into();
        assert_eq!(bits, 0x80000001);
    }

    #[test]
    fn test_group_operators() {
        assert_eq!(GROUP_1 | GROUP_2, 3_u32.into());
        assert_eq!((GROUP_1 | GROUP_2) & GROUP_2, GROUP_2);
        assert_eq!(GROUP_1 & GROUP_2, NONE);
        assert_eq!(~NONE, ALL);
        assert_eq!(~ALL, NONE);
        assert_eq!(~GROUP_1, 0xfffffffe_u32.into());
        assert_eq!(~GROUP_32, 0x7fffffff_u32.into());
    }

    #[test]
    fn test_group_intersects_contains() {
        let both = GROUP_1 | GROUP_2;
        assert!(both.intersects(GROUP_2));
        assert!(!both.intersects(GROUP_3));
        assert!(!both.intersects(NONE));
        assert!(!NONE.intersects(NONE));
        assert!(ALL.intersects(GROUP_32));
        assert!(both.contains(GROUP_2));
        assert!(both.contains(both));
        assert!(both.contains(NONE));
        assert!(!both.contains(GROUP_2 | GROUP_3));
        assert!(!NONE.contains(GROUP_1));
        assert!(ALL.contains(both));
    }

    #[test]
    fn test_constructors() {
        let default: InteractionGroups = Default::default();
        assert_eq!(default, InteractionGroupsTrait::new(GROUP_1, ALL, AND));
        assert_eq!(InteractionGroupsTrait::all(), InteractionGroupsTrait::new(ALL, ALL, AND));
        assert_eq!(InteractionGroupsTrait::none(), InteractionGroupsTrait::new(NONE, NONE, AND));
        let default_mode: InteractionTestMode = Default::default();
        assert_eq!(default_mode, AND);
        let edited = default.with_memberships(GROUP_3).with_filter(GROUP_2);
        assert_eq!(edited, InteractionGroupsTrait::new(GROUP_3, GROUP_2, AND));
        let edited = InteractionGroupsTrait::new(GROUP_1, GROUP_1, OR).with_filter(ALL);
        assert_eq!(edited.test_mode, OR);
    }

    /// Truth table of the two directed intersections `a.m & b.f` and `b.m & a.f`.
    #[test]
    fn test_truth_table() {
        // both directions match
        assert_tests(groups(0b01, 0b10, AND), groups(0b10, 0b01, AND), true, true);
        // only a.m & b.f
        assert_tests(groups(0b01, 0b100, AND), groups(0b10, 0b01, AND), false, true);
        // only b.m & a.f
        assert_tests(groups(0b01, 0b10, AND), groups(0b10, 0b100, AND), false, true);
        // none
        assert_tests(groups(0b01, 0b100, AND), groups(0b10, 0b100, AND), false, false);
    }

    #[test]
    fn test_edge_masks() {
        let all = InteractionGroupsTrait::all();
        let none = InteractionGroupsTrait::none();
        assert_tests(all, all, true, true);
        assert_tests(all, none, false, false);
        assert_tests(none, none, false, false);
        // Default groups collide with each other and with `all`.
        assert_tests(Default::default(), Default::default(), true, true);
        assert_tests(Default::default(), all, true, true);
        // Highest bit on both sides (the packed candidates must not mix the halves).
        let top = groups(0x80000000, 0x80000000, AND);
        assert_tests(top, top, true, true);
        assert_tests(top, groups(0x80000000, 0x7fffffff, AND), false, true);
        assert_tests(top, groups(0x7fffffff, 0x80000000, AND), false, true);
        assert_tests(top, groups(0x7fffffff, 0x7fffffff, AND), false, false);
        // Memberships without filter and conversely.
        assert_tests(groups(0xffffffff, 0, AND), groups(0xffffffff, 0, AND), false, false);
        assert_tests(groups(0, 0xffffffff, AND), groups(0, 0xffffffff, AND), false, false);
        assert_tests(groups(0xffffffff, 0, AND), groups(0, 0xffffffff, AND), false, true);
        // Low half of one product must not leak into the high half of the other.
        assert_tests(groups(1, 0, AND), groups(0, 1, AND), false, true);
        assert_tests(groups(0x80000000, 0, AND), groups(0, 1, AND), false, false);
    }

    /// Mode resolution as upstream: `Or` only when both sides are `Or`.
    #[test]
    fn test_modes() {
        // One direction only: passes `or`, fails `and`.
        let (m, f) = (0b01, 0b100);
        let one_way_and = groups(m, f, AND);
        let one_way_or = groups(m, f, OR);
        let other_and = groups(0b10, 0b01, AND);
        let other_or = groups(0b10, 0b01, OR);
        assert!(!one_way_and.test(other_and));
        assert!(!one_way_and.test(other_or));
        assert!(!one_way_or.test(other_and));
        assert!(one_way_or.test(other_or));
        // Symmetry.
        assert!(!other_and.test(one_way_and));
        assert!(!other_or.test(one_way_and));
        assert!(!other_and.test(one_way_or));
        assert!(other_or.test(one_way_or));
        // Both directions: every mode combination passes.
        let mutual_and = groups(0b01, 0b10, AND);
        let mutual_or = groups(0b01, 0b10, OR);
        assert!(mutual_and.test(other_and));
        assert!(mutual_and.test(other_or));
        assert!(mutual_or.test(other_and));
        assert!(mutual_or.test(other_or));
        // No direction: none passes.
        let never_or = groups(0b1000, 0b1000, OR);
        assert!(!never_or.test(other_or));
        assert!(!never_or.test(other_and));
    }

    #[test]
    #[fuzzer(runs: 256, seed: 20260920)]
    fn fuzz_candidates_equivalent(am: u32, af: u32, bm: u32, bf: u32) {
        let a = groups(am, af, AND);
        let b = groups(bm, bf, AND);
        let and = a.test_and(b);
        let or = a.test_or(b);
        assert_eq!(test_and_packed_u64(a, b), and);
        assert_eq!(test_and_packed_u128(a, b), and);
        assert_eq!(test_and_arith(a, b), and);
        assert_eq!(test_or_packed_u64(a, b), or);
        assert_eq!(test_or_packed_u128(a, b), or);
        assert_eq!(test_or_arith(a, b), or);
        assert_eq!(b.test_and(a), and);
        assert_eq!(b.test_or(a), or);
    }

    // Gas probes. `hit` pairs match in both directions (both ANDs evaluated by `test_and`, one
    // by `test_or`); `miss` pairs fail on the first direction (the reverse).

    fn hit() -> (InteractionGroups, InteractionGroups) {
        (opaque(groups(0x00010001, 0x80000002, AND)), opaque(groups(0x80000002, 0x00010001, AND)))
    }

    fn miss() -> (InteractionGroups, InteractionGroups) {
        (opaque(groups(0x00010001, 0x80000002, AND)), opaque(groups(0x80000002, 0x00100010, AND)))
    }

    #[test]
    fn gas_baseline() {}

    /// Cost of building the probe inputs alone: subtract it from the `test_*` probes.
    #[test]
    fn gas_inputs() {
        let (a, b) = hit();
        assert!(a.test_mode == b.test_mode);
    }

    #[test]
    fn gas_test_and_u32_hit() {
        let (a, b) = hit();
        assert!(a.test_and(b));
    }

    #[test]
    fn gas_test_and_u32_miss() {
        let (a, b) = miss();
        assert!(!a.test_and(b));
    }

    #[test]
    fn gas_test_and_packed_u64_hit() {
        let (a, b) = hit();
        assert!(test_and_packed_u64(a, b));
    }

    #[test]
    fn gas_test_and_packed_u64_miss() {
        let (a, b) = miss();
        assert!(!test_and_packed_u64(a, b));
    }

    #[test]
    fn gas_test_and_packed_u128_hit() {
        let (a, b) = hit();
        assert!(test_and_packed_u128(a, b));
    }

    #[test]
    fn gas_test_and_packed_u128_miss() {
        let (a, b) = miss();
        assert!(!test_and_packed_u128(a, b));
    }

    #[test]
    fn gas_test_and_arith_hit() {
        let (a, b) = hit();
        assert!(test_and_arith(a, b));
    }

    #[test]
    fn gas_test_and_arith_miss() {
        let (a, b) = miss();
        assert!(!test_and_arith(a, b));
    }

    #[test]
    fn gas_test_or_u32_hit() {
        let (a, b) = hit();
        assert!(a.test_or(b));
    }

    #[test]
    fn gas_test_or_u32_miss() {
        let (a, b) = miss();
        assert!(a.test_or(b));
    }

    #[test]
    fn gas_test_or_packed_u64_hit() {
        let (a, b) = hit();
        assert!(test_or_packed_u64(a, b));
    }

    #[test]
    fn gas_test_or_packed_u64_miss() {
        let (a, b) = miss();
        assert!(test_or_packed_u64(a, b));
    }

    #[test]
    fn gas_test_or_packed_u128_hit() {
        let (a, b) = hit();
        assert!(test_or_packed_u128(a, b));
    }

    #[test]
    fn gas_test_or_packed_u128_miss() {
        let (a, b) = miss();
        assert!(test_or_packed_u128(a, b));
    }

    #[test]
    fn gas_test_or_arith_hit() {
        let (a, b) = hit();
        assert!(test_or_arith(a, b));
    }

    #[test]
    fn gas_test_or_arith_miss() {
        let (a, b) = miss();
        assert!(test_or_arith(a, b));
    }

    #[test]
    fn gas_test_modes_and() {
        let (a, b) = hit();
        assert!(a.test(b));
    }

    #[test]
    fn gas_test_modes_or() {
        let (a, b) = hit();
        let a = InteractionGroups { test_mode: opaque(OR), ..a };
        let b = InteractionGroups { test_mode: opaque(OR), ..b };
        assert!(a.test(b));
    }

    #[test]
    fn gas_new() {
        let built = InteractionGroupsTrait::new(opaque(GROUP_1), opaque(ALL), opaque(AND));
        assert!(opaque(built).filter == ALL);
    }

    #[test]
    fn gas_all_none_default() {
        assert!(opaque(InteractionGroupsTrait::all()).filter == ALL);
        assert!(opaque(InteractionGroupsTrait::none()).filter == NONE);
        let default: InteractionGroups = Default::default();
        assert!(opaque(default).filter == ALL);
    }

    #[test]
    fn gas_with_memberships_with_filter() {
        let (a, _) = hit();
        let edited = a.with_memberships(opaque(GROUP_3)).with_filter(opaque(GROUP_2));
        assert!(opaque(edited).filter == GROUP_2);
    }

    #[test]
    fn gas_group_conversions() {
        let group: Group = opaque(5_u32).into();
        let bits: u32 = opaque(group).into();
        assert!(bits == 5);
    }

    #[test]
    fn gas_group_bitor() {
        assert!((opaque(GROUP_1) | opaque(GROUP_2)).bits == 3);
    }

    #[test]
    fn gas_group_bitand() {
        assert!((opaque(GROUP_1) & opaque(GROUP_2)).bits == 0);
    }

    #[test]
    fn gas_group_bitnot() {
        assert!((~opaque(GROUP_1)).bits == 0xfffffffe);
    }

    #[test]
    fn gas_group_intersects() {
        assert!(!opaque(GROUP_1).intersects(opaque(GROUP_2)));
    }

    #[test]
    fn gas_group_contains() {
        assert!(opaque(ALL).contains(opaque(GROUP_2)));
    }
}
