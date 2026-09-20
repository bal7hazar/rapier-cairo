//! Event flags of a collider (upstream `pipeline::ActiveEvents` and
//! `geometry::CollisionEventFlags`), and the reference benchmark of the bit-set operations shared
//! by every flag type of `rapier_core` (`ActiveHooks`, `ActiveCollisionTypes`, `ColliderChanges`,
//! `RigidBodyChanges`, ...).
//!
//! A flag set is a newtype over `u32` (`u16` for `ActiveCollisionTypes`) with public `bits`.
//! Candidates for the set operations, ranked by the `gas_*` probes of this module (the other
//! flag modules copy the winners):
//!
//! * `contains(flag)`: `bits & flag == flag` **(shipped)**. Alternatives usable only for a
//!   *constant single-bit* flag: a single-`DivRem` bit test (`bits % 2^(k+1) >= 2^k`,
//!   `alternatives::contains_mod`, about 300 gas cheaper than the shipped form in that case) and a
//!   double-`DivRem` test (`alternatives::contains_divrem2`, dearer). The shipped form is the
//!   only one valid for any flag set, hence the API;
//! * `insert(flag)`: `bits | flag` **(winner)** against `bits + (flag - (bits & flag))`
//!   (`alternatives::insert_add_and`) and the arithmetic-only `bits + flag * (1 - already_set)`
//!   (`alternatives::insert_arith`);
//! * `remove(flag)`: `bits & (0xffffffff - flag)` **(winner)** against `bits - (bits & flag)`
//!   (`alternatives::remove_sub_and`) and the arithmetic-only `bits - flag * already_set`
//!   (`alternatives::remove_arith`).
//!
//! Every probe includes the cost of its (opaque) inputs, see `gas_inputs`.

use core::traits::{BitAnd, BitOr};

/// Enables `Started` / `Stopped` collision events for this collider.
pub const COLLISION_EVENTS: ActiveEvents = ActiveEvents { bits: 0x1 };

/// Enables contact force events when the force exceeds the collider's threshold.
pub const CONTACT_FORCE_EVENTS: ActiveEvents = ActiveEvents { bits: 0x2 };

/// The events enabled for a collider. Default: none.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct ActiveEvents {
    /// Raw mask, bit `i` is the flag `1 << i`.
    pub bits: u32,
}

/// Wraps a raw mask (upstream `from_bits_retain`).
pub impl U32IntoActiveEvents of Into<u32, ActiveEvents> {
    #[inline(always)]
    fn into(self: u32) -> ActiveEvents {
        ActiveEvents { bits: self }
    }
}

/// Unwraps the raw mask.
pub impl ActiveEventsIntoU32 of Into<ActiveEvents, u32> {
    #[inline(always)]
    fn into(self: ActiveEvents) -> u32 {
        self.bits
    }
}

/// Set union.
pub impl ActiveEventsBitOr of BitOr<ActiveEvents> {
    #[inline(always)]
    fn bitor(lhs: ActiveEvents, rhs: ActiveEvents) -> ActiveEvents {
        ActiveEvents { bits: lhs.bits | rhs.bits }
    }
}

/// Set intersection.
pub impl ActiveEventsBitAnd of BitAnd<ActiveEvents> {
    #[inline(always)]
    fn bitand(lhs: ActiveEvents, rhs: ActiveEvents) -> ActiveEvents {
        ActiveEvents { bits: lhs.bits & rhs.bits }
    }
}

/// Set operations on [`ActiveEvents`].
#[generate_trait]
pub impl ActiveEventsImpl of ActiveEventsTrait {
    /// No flag set.
    #[inline(always)]
    fn empty() -> ActiveEvents {
        ActiveEvents { bits: 0 }
    }

    /// Every defined flag set.
    #[inline(always)]
    fn all() -> ActiveEvents {
        ActiveEvents { bits: 0x3 }
    }

    /// Returns `true` when no flag is set.
    #[inline(always)]
    fn is_empty(self: ActiveEvents) -> bool {
        self.bits == 0
    }

    /// Returns `true` when every flag of `other` is in `self`.
    #[inline(always)]
    fn contains(self: ActiveEvents, other: ActiveEvents) -> bool {
        self.bits & other.bits == other.bits
    }

    /// Returns `true` when `self` and `other` share at least one flag.
    #[inline(always)]
    fn intersects(self: ActiveEvents, other: ActiveEvents) -> bool {
        self.bits & other.bits != 0
    }

    /// Sets every flag of `other`.
    #[inline(always)]
    fn insert(ref self: ActiveEvents, other: ActiveEvents) {
        self.bits = self.bits | other.bits;
    }

    /// Clears every flag of `other`.
    #[inline(always)]
    fn remove(ref self: ActiveEvents, other: ActiveEvents) {
        self.bits = self.bits & (0xffffffff - other.bits);
    }
}

/// At least one of the colliders involved was a sensor when the event was fired.
pub const SENSOR: CollisionEventFlags = CollisionEventFlags { bits: 0x1 };

/// A `Stopped` event was fired because at least one collider was removed.
pub const REMOVED: CollisionEventFlags = CollisionEventFlags { bits: 0x2 };

/// Flags giving more information about a collision event. Default: none.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct CollisionEventFlags {
    /// Raw mask, bit `i` is the flag `1 << i`.
    pub bits: u32,
}

/// Wraps a raw mask (upstream `from_bits_retain`).
pub impl U32IntoCollisionEventFlags of Into<u32, CollisionEventFlags> {
    #[inline(always)]
    fn into(self: u32) -> CollisionEventFlags {
        CollisionEventFlags { bits: self }
    }
}

/// Unwraps the raw mask.
pub impl CollisionEventFlagsIntoU32 of Into<CollisionEventFlags, u32> {
    #[inline(always)]
    fn into(self: CollisionEventFlags) -> u32 {
        self.bits
    }
}

/// Set union.
pub impl CollisionEventFlagsBitOr of BitOr<CollisionEventFlags> {
    #[inline(always)]
    fn bitor(lhs: CollisionEventFlags, rhs: CollisionEventFlags) -> CollisionEventFlags {
        CollisionEventFlags { bits: lhs.bits | rhs.bits }
    }
}

/// Set intersection.
pub impl CollisionEventFlagsBitAnd of BitAnd<CollisionEventFlags> {
    #[inline(always)]
    fn bitand(lhs: CollisionEventFlags, rhs: CollisionEventFlags) -> CollisionEventFlags {
        CollisionEventFlags { bits: lhs.bits & rhs.bits }
    }
}

/// Set operations on [`CollisionEventFlags`].
#[generate_trait]
pub impl CollisionEventFlagsImpl of CollisionEventFlagsTrait {
    /// No flag set.
    #[inline(always)]
    fn empty() -> CollisionEventFlags {
        CollisionEventFlags { bits: 0 }
    }

    /// Every defined flag set.
    #[inline(always)]
    fn all() -> CollisionEventFlags {
        CollisionEventFlags { bits: 0x3 }
    }

    /// Returns `true` when no flag is set.
    #[inline(always)]
    fn is_empty(self: CollisionEventFlags) -> bool {
        self.bits == 0
    }

    /// Returns `true` when every flag of `other` is in `self`.
    #[inline(always)]
    fn contains(self: CollisionEventFlags, other: CollisionEventFlags) -> bool {
        self.bits & other.bits == other.bits
    }

    /// Returns `true` when `self` and `other` share at least one flag.
    #[inline(always)]
    fn intersects(self: CollisionEventFlags, other: CollisionEventFlags) -> bool {
        self.bits & other.bits != 0
    }

    /// Sets every flag of `other`.
    #[inline(always)]
    fn insert(ref self: CollisionEventFlags, other: CollisionEventFlags) {
        self.bits = self.bits | other.bits;
    }

    /// Clears every flag of `other`.
    #[inline(always)]
    fn remove(ref self: CollisionEventFlags, other: CollisionEventFlags) {
        self.bits = self.bits & (0xffffffff - other.bits);
    }
}

#[cfg(test)]
mod alternatives {
    use super::{ActiveEvents, CONTACT_FORCE_EVENTS};

    /// `CONTACT_FORCE_EVENTS` is bit 1: the divisors below are `2^1`, `2^2` and `2`.
    const NZ_POW: NonZero<u32> = 2;
    const NZ_POW_TIMES_TWO: NonZero<u32> = 4;
    const NZ_TWO: NonZero<u32> = 2;
    const BIT: u32 = 2;

    /// One `DivRem`: the flag is set iff `bits % 4 >= 2`.
    pub fn contains_mod(self: ActiveEvents) -> bool {
        let (_, remainder) = DivRem::div_rem(self.bits, NZ_POW_TIMES_TWO);
        remainder >= BIT
    }

    /// Two `DivRem`s: the flag is set iff `(bits / 2) % 2 == 1`.
    pub fn contains_divrem2(self: ActiveEvents) -> bool {
        let (quotient, _) = DivRem::div_rem(self.bits, NZ_POW);
        let (_, bit) = DivRem::div_rem(quotient, NZ_TWO);
        bit == 1
    }

    /// `bits + (flag - (bits & flag))`: one bitwise builtin call and two checked additions.
    pub fn insert_add_and(self: ActiveEvents, other: ActiveEvents) -> ActiveEvents {
        ActiveEvents { bits: self.bits + (other.bits - (self.bits & other.bits)) }
    }

    /// Arithmetic only: `bits + flag * (1 - already_set)`.
    pub fn insert_arith(self: ActiveEvents) -> ActiveEvents {
        let (quotient, _) = DivRem::div_rem(self.bits, NZ_POW);
        let (_, already_set) = DivRem::div_rem(quotient, NZ_TWO);
        ActiveEvents { bits: self.bits + BIT * (1 - already_set) }
    }

    /// `bits - (bits & flag)`: one bitwise builtin call and a checked subtraction.
    pub fn remove_sub_and(self: ActiveEvents, other: ActiveEvents) -> ActiveEvents {
        ActiveEvents { bits: self.bits - (self.bits & other.bits) }
    }

    /// Arithmetic only: `bits - flag * already_set`.
    pub fn remove_arith(self: ActiveEvents) -> ActiveEvents {
        let (quotient, _) = DivRem::div_rem(self.bits, NZ_POW);
        let (_, already_set) = DivRem::div_rem(quotient, NZ_TWO);
        ActiveEvents { bits: self.bits - BIT * already_set }
    }

    /// The winner of each operation on the constant flag, for the probes.
    pub fn flag() -> ActiveEvents {
        CONTACT_FORCE_EVENTS
    }
}

#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use super::alternatives::{
        contains_divrem2, contains_mod, flag, insert_add_and, insert_arith, remove_arith,
        remove_sub_and,
    };
    use super::{
        ActiveEvents, ActiveEventsTrait, COLLISION_EVENTS, CONTACT_FORCE_EVENTS,
        CollisionEventFlags, CollisionEventFlagsTrait, REMOVED, SENSOR,
    };

    #[test]
    fn test_defaults_are_empty() {
        let events: ActiveEvents = Default::default();
        assert_eq!(events, ActiveEventsTrait::empty());
        assert!(events.is_empty());
        let flags: CollisionEventFlags = Default::default();
        assert_eq!(flags, CollisionEventFlagsTrait::empty());
        assert!(flags.is_empty());
    }

    #[test]
    fn test_flag_values() {
        assert_eq!(COLLISION_EVENTS.bits, 0b01);
        assert_eq!(CONTACT_FORCE_EVENTS.bits, 0b10);
        assert_eq!(COLLISION_EVENTS | CONTACT_FORCE_EVENTS, ActiveEventsTrait::all());
        assert_eq!(SENSOR.bits, 0b01);
        assert_eq!(REMOVED.bits, 0b10);
        assert_eq!(SENSOR | REMOVED, CollisionEventFlagsTrait::all());
    }

    #[test]
    fn test_active_events_set_operations() {
        let mut events: ActiveEvents = Default::default();
        events.insert(CONTACT_FORCE_EVENTS);
        events.insert(CONTACT_FORCE_EVENTS);
        assert_eq!(events, CONTACT_FORCE_EVENTS);
        assert!(events.contains(CONTACT_FORCE_EVENTS));
        assert!(!events.contains(COLLISION_EVENTS));
        assert!(!events.contains(COLLISION_EVENTS | CONTACT_FORCE_EVENTS));
        assert!(events.contains(ActiveEventsTrait::empty()));
        assert!(events.intersects(COLLISION_EVENTS | CONTACT_FORCE_EVENTS));
        assert!(!events.intersects(COLLISION_EVENTS));
        assert!(!events.intersects(ActiveEventsTrait::empty()));
        events.insert(COLLISION_EVENTS);
        assert_eq!(events & COLLISION_EVENTS, COLLISION_EVENTS);
        events.remove(CONTACT_FORCE_EVENTS);
        assert_eq!(events, COLLISION_EVENTS);
        events.remove(CONTACT_FORCE_EVENTS);
        assert_eq!(events, COLLISION_EVENTS);
        events.remove(ActiveEventsTrait::all());
        assert!(events.is_empty());
        let bits: u32 = ActiveEventsTrait::all().into();
        assert_eq!(bits, 3);
        let back: ActiveEvents = 2_u32.into();
        assert_eq!(back, CONTACT_FORCE_EVENTS);
    }

    #[test]
    fn test_collision_event_flags_set_operations() {
        let mut flags: CollisionEventFlags = Default::default();
        flags.insert(SENSOR);
        assert!(flags.contains(SENSOR));
        assert!(!flags.contains(REMOVED));
        assert!(flags.intersects(SENSOR | REMOVED));
        flags.insert(REMOVED);
        assert_eq!(flags, SENSOR | REMOVED);
        flags.remove(SENSOR);
        assert_eq!(flags, REMOVED);
        assert_eq!((SENSOR | REMOVED) & REMOVED, REMOVED);
        let bits: u32 = flags.into();
        assert_eq!(bits, 2);
        let back: CollisionEventFlags = 1_u32.into();
        assert_eq!(back, SENSOR);
    }

    /// The candidates agree with the winners for every combination of the low bits and the
    /// extremes of the other bits, on the single-bit flag `CONTACT_FORCE_EVENTS`.
    #[test]
    fn test_candidates_agree_on_edges() {
        let samples = array![
            0_u32, 1, 2, 3, 4, 5, 6, 7, 0xfffffffc, 0xfffffffd, 0xfffffffe, 0xffffffff, 0x80000000,
            0x7ffffffe, 0x7fffffff, 0x55555555, 0xaaaaaaaa,
        ];
        for bits in samples {
            assert_agree(ActiveEvents { bits });
        }
    }

    #[test]
    #[fuzzer(runs: 256, seed: 20260920)]
    fn fuzz_candidates_agree(bits: u32) {
        assert_agree(ActiveEvents { bits });
    }

    fn assert_agree(events: ActiveEvents) {
        let f = flag();
        let contains = events.contains(f);
        assert_eq!(contains_mod(events), contains);
        assert_eq!(contains_divrem2(events), contains);
        let mut inserted = events;
        inserted.insert(f);
        assert_eq!(insert_add_and(events, f), inserted);
        assert_eq!(insert_arith(events), inserted);
        let mut removed = events;
        removed.remove(f);
        assert_eq!(remove_sub_and(events, f), removed);
        assert_eq!(remove_arith(events), removed);
        assert_eq!(removed.bits, events.bits & (0xffffffff - f.bits));
        assert_eq!(inserted.bits, events.bits | f.bits);
    }

    // Gas probes: `bits` is opaque, the single-bit flag is a constant (the usual call shape).

    #[test]
    fn gas_baseline() {}

    /// Cost of building the probe inputs alone: subtract it from the probes below.
    #[test]
    fn gas_inputs() {
        assert!(opaque(ActiveEvents { bits: 0x80000001 }).bits != 0);
    }

    #[test]
    fn gas_contains_and() {
        assert!(!opaque(ActiveEvents { bits: 0x80000001 }).contains(CONTACT_FORCE_EVENTS));
    }

    #[test]
    fn gas_contains_mod() {
        assert!(!contains_mod(opaque(ActiveEvents { bits: 0x80000001 })));
    }

    #[test]
    fn gas_contains_divrem2() {
        assert!(!contains_divrem2(opaque(ActiveEvents { bits: 0x80000001 })));
    }

    /// The winner with an opaque flag too (the flag is not known at compile time).
    #[test]
    fn gas_contains_and_dynamic_flag() {
        let events = opaque(ActiveEvents { bits: 0x80000001 });
        assert!(!events.contains(opaque(CONTACT_FORCE_EVENTS)));
    }

    #[test]
    fn gas_intersects() {
        assert!(!opaque(ActiveEvents { bits: 0x80000001 }).intersects(CONTACT_FORCE_EVENTS));
    }

    #[test]
    fn gas_insert_or() {
        let mut events = opaque(ActiveEvents { bits: 0x80000001 });
        events.insert(CONTACT_FORCE_EVENTS);
        assert!(events.bits == 0x80000003);
    }

    #[test]
    fn gas_insert_add_and() {
        let events = insert_add_and(
            opaque(ActiveEvents { bits: 0x80000001 }), CONTACT_FORCE_EVENTS,
        );
        assert!(events.bits == 0x80000003);
    }

    #[test]
    fn gas_insert_arith() {
        let events = insert_arith(opaque(ActiveEvents { bits: 0x80000001 }));
        assert!(events.bits == 0x80000003);
    }

    #[test]
    fn gas_remove_and_not() {
        let mut events = opaque(ActiveEvents { bits: 0x80000003 });
        events.remove(CONTACT_FORCE_EVENTS);
        assert!(events.bits == 0x80000001);
    }

    #[test]
    fn gas_remove_sub_and() {
        let events = remove_sub_and(
            opaque(ActiveEvents { bits: 0x80000003 }), CONTACT_FORCE_EVENTS,
        );
        assert!(events.bits == 0x80000001);
    }

    #[test]
    fn gas_remove_arith() {
        let events = remove_arith(opaque(ActiveEvents { bits: 0x80000003 }));
        assert!(events.bits == 0x80000001);
    }

    /// Winners and losers with a flag unknown at compile time.
    #[test]
    fn gas_insert_or_dynamic_flag() {
        let mut events = opaque(ActiveEvents { bits: 0x80000001 });
        events.insert(opaque(CONTACT_FORCE_EVENTS));
        assert!(events.bits == 0x80000003);
    }

    #[test]
    fn gas_insert_add_and_dynamic_flag() {
        let events = insert_add_and(
            opaque(ActiveEvents { bits: 0x80000001 }), opaque(CONTACT_FORCE_EVENTS),
        );
        assert!(events.bits == 0x80000003);
    }

    #[test]
    fn gas_remove_and_not_dynamic_flag() {
        let mut events = opaque(ActiveEvents { bits: 0x80000003 });
        events.remove(opaque(CONTACT_FORCE_EVENTS));
        assert!(events.bits == 0x80000001);
    }

    #[test]
    fn gas_remove_sub_and_dynamic_flag() {
        let events = remove_sub_and(
            opaque(ActiveEvents { bits: 0x80000003 }), opaque(CONTACT_FORCE_EVENTS),
        );
        assert!(events.bits == 0x80000001);
    }

    #[test]
    fn gas_bitor_bitand() {
        let union = opaque(COLLISION_EVENTS) | opaque(CONTACT_FORCE_EVENTS);
        assert!((union & opaque(COLLISION_EVENTS)).bits == 1);
    }
}
