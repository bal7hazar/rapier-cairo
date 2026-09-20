//! Generational handles (upstream `rapier::data::Index`).
//!
//! A [`Handle`] names a slot of an [`Arena`](super::arena::Arena) together with the generation
//! the slot had when the handle was issued. A handle whose generation no longer matches the
//! slot is *stale* and resolves to nothing, which makes slot reuse safe.
//!
//! Representation: the two-field struct won the benchmark against single-word packings
//! (`u64`, `felt252`): the arena reads `index` and `generation` separately on every access and
//! a struct field read is free, whereas a packed word pays a `DivRem` per access. The packed
//! forms remain available through [`HandleTrait::pack`] / [`HandleTrait::unpack`] (storage,
//! events) and [`HandleTrait::key`] (dictionary key); the rejected representations and
//! pack / unpack variants live in `alternatives`.

#[feature("bounded-int-utils")]
use core::internal::bounded_int::{self, AddHelper, BoundedInt, DivRemHelper, MulHelper, UnitInt};

/// Value of `index` and `generation` in the invalid handle (upstream `INVALID_U32`).
pub const INVALID_U32: u32 = 0xffffffff;

/// The handle that never resolves to anything (upstream `Index::default()`).
pub const INVALID_HANDLE: Handle = Handle { index: INVALID_U32, generation: INVALID_U32 };

/// `2^32` as a felt, the weight of `index` in the packed forms.
const TWO_POW_32: felt252 = 0x100000000;

/// `2^32` as a singleton bounded integer, for overflow-check-free packing.
const TWO_POW_32_UNIT: UnitInt<0x100000000> = 0x100000000;

/// `2^32` as a non-zero singleton bounded integer, for typed unpacking.
const NZ_TWO_POW_32_UNIT: NonZero<UnitInt<0x100000000>> = 0x100000000;

/// A `u32` seen as a bounded integer.
type U32Bounded = BoundedInt<0, 0xffffffff>;

impl MulU32ByTwoPow32 of MulHelper<u32, UnitInt<0x100000000>> {
    type Result = BoundedInt<0, 0xffffffff00000000>;
}

impl AddShiftedU32AndU32 of AddHelper<BoundedInt<0, 0xffffffff00000000>, u32> {
    type Result = BoundedInt<0, 0xffffffffffffffff>;
}

impl DivRemU64ByTwoPow32 of DivRemHelper<u64, UnitInt<0x100000000>> {
    type DivT = U32Bounded;
    type RemT = U32Bounded;
}

/// Generational index of an arena slot.
///
/// `index` is the slot number, `generation` the value of the arena generation counter when the
/// slot was filled. Both span the full `u32` range; `(0xffffffff, 0xffffffff)` is reserved for
/// [`INVALID_HANDLE`].
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Handle {
    /// Slot number inside the arena.
    pub index: u32,
    /// Generation of the slot when the handle was issued.
    pub generation: u32,
}

/// The default handle is the invalid one, as upstream.
pub impl HandleDefault of Default<Handle> {
    #[inline(always)]
    fn default() -> Handle {
        INVALID_HANDLE
    }
}

/// Handles convert to the felt252 dictionary key of [`HandleTrait::key`].
pub impl HandleIntoFelt252 of Into<Handle, felt252> {
    #[inline(always)]
    fn into(self: Handle) -> felt252 {
        self.key()
    }
}

/// Constructors, accessors and scalar encodings of [`Handle`].
#[generate_trait]
pub impl HandleImpl of HandleTrait {
    /// Builds a handle from its raw parts (upstream `Index::from_raw_parts`).
    ///
    /// # Arguments
    /// * `index` - slot number.
    /// * `generation` - generation of the slot.
    #[inline(always)]
    fn new(index: u32, generation: u32) -> Handle {
        Handle { index, generation }
    }

    /// Returns [`INVALID_HANDLE`].
    #[inline(always)]
    fn invalid() -> Handle {
        INVALID_HANDLE
    }

    /// Returns `true` when the handle is [`INVALID_HANDLE`].
    #[inline(always)]
    fn is_invalid(self: Handle) -> bool {
        self == INVALID_HANDLE
    }

    /// Returns `(index, generation)` (upstream `Index::into_raw_parts`).
    #[inline(always)]
    fn into_raw_parts(self: Handle) -> (u32, u32) {
        (self.index, self.generation)
    }

    /// Packs the handle into one word: `index * 2^32 + generation`.
    ///
    /// The integer order of packed handles is upstream's `Ord` on `Index` (index first, then
    /// generation). Never panics: the result always fits a `u64`.
    #[inline(always)]
    fn pack(self: Handle) -> u64 {
        let high = bounded_int::mul(self.index, TWO_POW_32_UNIT);
        bounded_int::upcast(bounded_int::add(high, self.generation))
    }

    /// Inverse of [`pack`](HandleTrait::pack). Total: every `u64` is a packed handle.
    #[inline(always)]
    fn unpack(packed: u64) -> Handle {
        let (index, generation) = bounded_int::div_rem(packed, NZ_TWO_POW_32_UNIT);
        Handle { index: bounded_int::upcast(index), generation: bounded_int::upcast(generation) }
    }

    /// Returns `index * 2^32 + generation` as a felt252, the cheapest injective scalar form of
    /// a handle (two felt operations, no range check). Meant for `Felt252Dict` keys.
    #[inline(always)]
    fn key(self: Handle) -> felt252 {
        self.index.into() * TWO_POW_32 + self.generation.into()
    }
}

#[cfg(test)]
mod alternatives {
    use super::{Handle, TWO_POW_32};

    const TWO_POW_32_U64: u64 = 0x100000000;
    const NZ_TWO_POW_32_U64: NonZero<u64> = 0x100000000;
    const NZ_TWO_POW_32_U128: NonZero<u128> = 0x100000000;
    const MASK_32: u64 = 0xffffffff;

    pub mod errors {
        pub const PART_OVERFLOW: felt252 = 'Handle: part overflow';
        pub const PACK_OVERFLOW: felt252 = 'Handle: pack overflow';
    }

    /// Pack with checked `u64` arithmetic.
    pub fn pack_math(handle: Handle) -> u64 {
        handle.index.into() * TWO_POW_32_U64 + handle.generation.into()
    }

    /// Pack with felt arithmetic, then one range check back to `u64`.
    pub fn pack_felt(handle: Handle) -> u64 {
        let packed: felt252 = handle.index.into() * TWO_POW_32 + handle.generation.into();
        packed.try_into().expect(errors::PACK_OVERFLOW)
    }

    /// Unpack with one `u64` `DivRem` by a constant, then two checked downcasts.
    pub fn unpack_divrem(packed: u64) -> Handle {
        let (index, generation) = DivRem::div_rem(packed, NZ_TWO_POW_32_U64);
        Handle {
            index: index.try_into().expect(errors::PART_OVERFLOW),
            generation: generation.try_into().expect(errors::PART_OVERFLOW),
        }
    }

    /// Unpack with a bitwise mask for the low part and a division for the high part.
    pub fn unpack_bitwise(packed: u64) -> Handle {
        let generation = packed & MASK_32;
        let index = packed / TWO_POW_32_U64;
        Handle {
            index: index.try_into().expect(errors::PART_OVERFLOW),
            generation: generation.try_into().expect(errors::PART_OVERFLOW),
        }
    }

    /// Candidate representation: one packed `u64`.
    #[derive(Copy, Drop, Serde, PartialEq, Debug)]
    pub struct PackedHandle {
        pub raw: u64,
    }

    #[generate_trait]
    pub impl PackedHandleImpl of PackedHandleTrait {
        fn new(index: u32, generation: u32) -> PackedHandle {
            PackedHandle { raw: super::HandleTrait::pack(Handle { index, generation }) }
        }

        fn parts(self: PackedHandle) -> (u32, u32) {
            let handle = super::HandleTrait::unpack(self.raw);
            (handle.index, handle.generation)
        }

        fn key(self: PackedHandle) -> felt252 {
            self.raw.into()
        }
    }

    /// Candidate representation: one packed `felt252`.
    #[derive(Copy, Drop, Serde, PartialEq, Debug)]
    pub struct FeltHandle {
        pub raw: felt252,
    }

    #[generate_trait]
    pub impl FeltHandleImpl of FeltHandleTrait {
        fn new(index: u32, generation: u32) -> FeltHandle {
            FeltHandle { raw: index.into() * TWO_POW_32 + generation.into() }
        }

        fn parts(self: FeltHandle) -> (u32, u32) {
            let raw: u128 = self.raw.try_into().expect(errors::PACK_OVERFLOW);
            let (index, generation) = DivRem::div_rem(raw, NZ_TWO_POW_32_U128);
            (
                index.try_into().expect(errors::PART_OVERFLOW),
                generation.try_into().expect(errors::PART_OVERFLOW),
            )
        }

        fn key(self: FeltHandle) -> felt252 {
            self.raw
        }
    }
}

#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use super::alternatives::{
        FeltHandleTrait, PackedHandleTrait, pack_felt, pack_math, unpack_bitwise, unpack_divrem,
    };
    use super::{Handle, HandleTrait, INVALID_HANDLE, INVALID_U32};

    const INDEX: u32 = 0x12345678;
    const GENERATION: u32 = 0x9abcdef0;
    const PACKED: u64 = 0x123456789abcdef0;

    fn samples() -> Span<Handle> {
        [
            Handle { index: 0, generation: 0 }, Handle { index: 1, generation: 0 },
            Handle { index: 0, generation: 1 }, Handle { index: INDEX, generation: GENERATION },
            Handle { index: INVALID_U32, generation: 0 },
            Handle { index: 0, generation: INVALID_U32 }, INVALID_HANDLE,
        ]
            .span()
    }

    #[test]
    fn test_new_and_raw_parts() {
        let handle = HandleTrait::new(INDEX, GENERATION);
        assert_eq!(handle.index, INDEX);
        assert_eq!(handle.generation, GENERATION);
        assert_eq!(handle.into_raw_parts(), (INDEX, GENERATION));
    }

    #[test]
    fn test_invalid() {
        assert_eq!(HandleTrait::invalid(), INVALID_HANDLE);
        assert_eq!(Default::<Handle>::default(), INVALID_HANDLE);
        assert!(INVALID_HANDLE.is_invalid());
        assert!(!HandleTrait::new(INVALID_U32, 0).is_invalid());
        assert!(!HandleTrait::new(0, INVALID_U32).is_invalid());
        assert!(!HandleTrait::new(0, 0).is_invalid());
    }

    #[test]
    fn test_equality() {
        assert!(HandleTrait::new(1, 2) == HandleTrait::new(1, 2));
        assert!(HandleTrait::new(1, 2) != HandleTrait::new(1, 3));
        assert!(HandleTrait::new(1, 2) != HandleTrait::new(2, 2));
    }

    #[test]
    fn test_pack_layout() {
        assert_eq!(HandleTrait::new(INDEX, GENERATION).pack(), PACKED);
        assert_eq!(HandleTrait::new(0, 0).pack(), 0);
        assert_eq!(HandleTrait::new(1, 0).pack(), 0x100000000);
        assert_eq!(INVALID_HANDLE.pack(), 0xffffffffffffffff);
    }

    #[test]
    fn test_pack_preserves_upstream_order() {
        // Index first, then generation.
        assert!(HandleTrait::new(1, 0).pack() > HandleTrait::new(0, INVALID_U32).pack());
        assert!(HandleTrait::new(1, 1).pack() > HandleTrait::new(1, 0).pack());
    }

    #[test]
    fn test_unpack_roundtrip() {
        let mut handles = samples();
        while let Option::Some(handle) = handles.pop_front() {
            assert_eq!(HandleTrait::unpack((*handle).pack()), *handle);
        }
    }

    #[test]
    fn test_key_matches_pack() {
        let mut handles = samples();
        while let Option::Some(handle) = handles.pop_front() {
            let key: felt252 = (*handle).into();
            assert_eq!(key, (*handle).pack().into());
            assert_eq!((*handle).key(), key);
        }
    }

    #[test]
    fn test_alternatives_match() {
        let mut handles = samples();
        while let Option::Some(handle) = handles.pop_front() {
            let handle = *handle;
            let packed = handle.pack();
            assert_eq!(pack_math(handle), packed);
            assert_eq!(pack_felt(handle), packed);
            assert_eq!(unpack_divrem(packed), handle);
            assert_eq!(unpack_bitwise(packed), handle);
            let packed_handle = PackedHandleTrait::new(handle.index, handle.generation);
            assert_eq!(packed_handle.parts(), (handle.index, handle.generation));
            assert_eq!(packed_handle.key(), handle.key());
            let felt_handle = FeltHandleTrait::new(handle.index, handle.generation);
            assert_eq!(felt_handle.parts(), (handle.index, handle.generation));
            assert_eq!(felt_handle.key(), handle.key());
        }
    }

    #[test]
    #[fuzzer(runs: 256, seed: 1)]
    fn fuzz_pack_unpack_candidates(index: u32, generation: u32) {
        let handle = HandleTrait::new(index, generation);
        let packed = handle.pack();
        assert_eq!(pack_math(handle), packed);
        assert_eq!(pack_felt(handle), packed);
        assert_eq!(HandleTrait::unpack(packed), handle);
        assert_eq!(unpack_divrem(packed), handle);
        assert_eq!(unpack_bitwise(packed), handle);
        assert_eq!(FeltHandleTrait::new(index, generation).parts(), (index, generation));
    }

    #[test]
    fn gas_baseline() {}

    // Representation candidates: build from parts, read both parts back.

    #[test]
    fn gas_repr_struct() {
        let handle = HandleTrait::new(opaque(INDEX), opaque(GENERATION));
        let (index, generation) = opaque(handle).into_raw_parts();
        assert!(index == INDEX && generation == GENERATION);
    }

    #[test]
    fn gas_repr_packed_u64() {
        let handle = PackedHandleTrait::new(opaque(INDEX), opaque(GENERATION));
        let (index, generation) = opaque(handle).parts();
        assert!(index == INDEX && generation == GENERATION);
    }

    #[test]
    fn gas_repr_packed_felt() {
        let handle = FeltHandleTrait::new(opaque(INDEX), opaque(GENERATION));
        let (index, generation) = opaque(handle).parts();
        assert!(index == INDEX && generation == GENERATION);
    }

    // Public functions and their candidates.

    #[test]
    fn gas_new() {
        let handle = HandleTrait::new(opaque(INDEX), opaque(GENERATION));
        assert!(opaque(handle).index == INDEX);
    }

    #[test]
    fn gas_is_invalid() {
        assert!(!opaque(HandleTrait::new(INDEX, GENERATION)).is_invalid());
    }

    #[test]
    fn gas_eq() {
        assert!(
            opaque(
                HandleTrait::new(INDEX, GENERATION),
            ) == opaque(HandleTrait::new(INDEX, GENERATION)),
        );
    }

    #[test]
    fn gas_key() {
        assert!(opaque(HandleTrait::new(INDEX, GENERATION)).key() == PACKED.into());
    }

    #[test]
    fn gas_pack_bounded() {
        assert!(opaque(HandleTrait::new(INDEX, GENERATION)).pack() == PACKED);
    }

    #[test]
    fn gas_pack_math() {
        assert!(pack_math(opaque(HandleTrait::new(INDEX, GENERATION))) == PACKED);
    }

    #[test]
    fn gas_pack_felt() {
        assert!(pack_felt(opaque(HandleTrait::new(INDEX, GENERATION))) == PACKED);
    }

    #[test]
    fn gas_unpack_bounded() {
        assert!(HandleTrait::unpack(opaque(PACKED)).index == INDEX);
    }

    #[test]
    fn gas_unpack_divrem() {
        assert!(unpack_divrem(opaque(PACKED)).index == INDEX);
    }

    #[test]
    fn gas_unpack_bitwise() {
        assert!(unpack_bitwise(opaque(PACKED)).index == INDEX);
    }
}
