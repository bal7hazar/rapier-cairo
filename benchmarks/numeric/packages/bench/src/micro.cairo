//! Micro-benchmarks: mul-then-shift variants, DivRem by constant vs runtime divisor,
//! and the "math < bitwise < loops" heuristic check.
use core::internal::bounded_int::{self, AddHelper, BoundedInt, DivRemHelper, MulHelper, UnitInt, downcast, upcast};
use core::num::traits::WideMul;

const NZ_2_32_U128: NonZero<u128> = 0x100000000;
const NZ_2_32_U64: NonZero<u64> = 0x100000000;
const NZ_2_U64: NonZero<u64> = 2;

// ---------------------------------------------------------------- mul then >> 32 (u64 x u64 -> u64)
/// cubit style: generic `/` operator with a literal divisor.
pub fn mulshift_div_operator(a: u64, b: u64) -> u64 {
    (a.wide_mul(b) / 0x100000000_u128).try_into().unwrap()
}
/// DivRem with a `const NonZero` divisor.
pub fn mulshift_divrem_const(a: u64, b: u64) -> u64 {
    let (q, _) = DivRem::div_rem(a.wide_mul(b), NZ_2_32_U128);
    q.try_into().unwrap()
}
/// Divisor only known at run time (includes the zero check).
pub fn mulshift_div_runtime(a: u64, b: u64, d: u128) -> u64 {
    (a.wide_mul(b) / d).try_into().unwrap()
}
type U64Prod = BoundedInt<0, 0xfffffffffffffffe0000000000000001>;
type U64ProdQ = BoundedInt<0, 0xfffffffffffffffe00000000>;
impl MulU64 of MulHelper<u64, u64> {
    type Result = U64Prod;
}
impl DivU64Prod of DivRemHelper<U64Prod, UnitInt<0x100000000>> {
    type DivT = U64ProdQ;
    type RemT = BoundedInt<0, 0xffffffff>;
}
const NZ_SCALE_BI: NonZero<UnitInt<0x100000000>> = 0x100000000;
/// BoundedInt: mul (no check) -> div_rem by constant -> one downcast.
pub fn mulshift_bounded(a: u64, b: u64) -> u64 {
    let p: U64Prod = bounded_int::mul(a, b);
    let (q, _r) = bounded_int::div_rem(p, NZ_SCALE_BI);
    downcast(q).unwrap()
}
/// felt252 product, one felt->u128 conversion, DivRem const, u128->u64.
pub fn mulshift_felt(a: u64, b: u64) -> u64 {
    let p: felt252 = a.into() * b.into();
    let u: u128 = p.try_into().unwrap();
    let (q, _) = DivRem::div_rem(u, NZ_2_32_U128);
    q.try_into().unwrap()
}

// ---------------------------------------------------------------- DivRem: const vs runtime divisor
pub fn u128_div_const(a: u128) -> u128 {
    let (q, _) = DivRem::div_rem(a, NZ_2_32_U128);
    q
}
pub fn u128_div_runtime(a: u128, d: u128) -> u128 {
    a / d
}
pub fn u128_div_runtime_nz(a: u128, d: NonZero<u128>) -> u128 {
    let (q, _) = DivRem::div_rem(a, d);
    q
}
pub fn u64_div_const(a: u64) -> u64 {
    let (q, _) = DivRem::div_rem(a, NZ_2_32_U64);
    q
}
pub fn u64_div_runtime(a: u64, d: u64) -> u64 {
    a / d
}
pub fn u256_div_runtime(a: u256, d: u256) -> u256 {
    a / d
}
pub fn u128_wide_mul_only(a: u128, b: u128) -> u256 {
    a.wide_mul(b)
}
pub fn u64_wide_mul_only(a: u64, b: u64) -> u128 {
    a.wide_mul(b)
}
pub fn u128_checked_mul(a: u128, b: u128) -> u128 {
    a * b
}
pub fn u64_checked_mul(a: u64, b: u64) -> u64 {
    a * b
}

// ================================================================ F. math vs bitwise vs loop
// F1. split a u64 at bit 32 -> (hi, lo)
pub fn split_math(x: u64) -> (u64, u64) {
    DivRem::div_rem(x, NZ_2_32_U64)
}
type Hi32 = BoundedInt<0, 0xffffffff>;
impl DivU64BI of DivRemHelper<u64, UnitInt<0x100000000>> {
    type DivT = Hi32;
    type RemT = Hi32;
}
pub fn split_math_bounded(x: u64) -> (u32, u32) {
    let (q, r) = bounded_int::div_rem(x, NZ_SCALE_BI);
    (upcast(q), upcast(r))
}
pub fn split_bitwise(x: u64) -> (u64, u64) {
    let lo = x & 0xffffffff;
    let hi = (x & 0xffffffff00000000) / 0x100000000;
    (hi, lo)
}
pub fn split_bitwise_lo_only(x: u64) -> u64 {
    x & 0xffffffff
}
pub fn split_math_lo_only(x: u64) -> u64 {
    let (_, r) = DivRem::div_rem(x, NZ_2_32_U64);
    r
}
pub fn split_loop(x: u64) -> (u64, u64) {
    // 32 halvings; low part rebuilt bit by bit
    let mut hi = x;
    let mut lo: u64 = 0;
    let mut w: u64 = 1;
    let mut i: u32 = 0;
    while i != 32 {
        let (q, r) = DivRem::div_rem(hi, NZ_2_U64);
        lo += r * w;
        w *= 2;
        hi = q;
        i += 1;
    }
    (hi, lo)
}

// F2. parity
pub fn parity_math(x: u64) -> bool {
    let (_, r) = DivRem::div_rem(x, NZ_2_U64);
    r == 1
}
pub fn parity_bitwise(x: u64) -> bool {
    x & 1 == 1
}

// F3. multiply by 2^k, k known only at run time (k < 32)
const POW2: [u64; 32] = [
    0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80, 0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000,
    0x8000, 0x10000, 0x20000, 0x40000, 0x80000, 0x100000, 0x200000, 0x400000, 0x800000, 0x1000000,
    0x2000000, 0x4000000, 0x8000000, 0x10000000, 0x20000000, 0x40000000, 0x80000000,
];
pub fn shl_math_table(x: u64, k: u32) -> u64 {
    x * *POW2.span()[k]
}
pub fn shl_math_pow(x: u64, k: u32) -> u64 {
    x * core::num::traits::Pow::pow(2_u64, k)
}
pub fn shl_loop(x: u64, k: u32) -> u64 {
    let mut r = x;
    let mut i = k;
    while i != 0 {
        r *= 2;
        i -= 1;
    }
    r
}

// F4. pack two u32 into a u64
pub fn pack_math(hi: u32, lo: u32) -> u64 {
    hi.into() * 0x100000000_u64 + lo.into()
}
impl MulHi32 of MulHelper<u32, UnitInt<0x100000000>> {
    type Result = BoundedInt<0, 0xffffffff00000000>;
}
impl AddHiLo32 of AddHelper<BoundedInt<0, 0xffffffff00000000>, u32> {
    type Result = BoundedInt<0, 0xffffffffffffffff>;
}
pub fn pack_math_bounded(hi: u32, lo: u32) -> u64 {
    upcast(bounded_int::add(bounded_int::mul::<u32, UnitInt<0x100000000>>(hi, 0x100000000), lo))
}
pub fn pack_math_felt(hi: u32, lo: u32) -> u64 {
    (hi.into() * 0x100000000 + lo.into()).try_into().unwrap()
}
pub fn pack_bitwise(hi: u32, lo: u32) -> u64 {
    (hi.into() * 0x100000000_u64) | lo.into()
}
