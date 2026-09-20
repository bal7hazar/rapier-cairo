//! Sign-magnitude 64.64 {mag: u128, sign: bool}.
use core::internal::bounded_int::{self, AddHelper, BoundedInt, MulHelper, UnitInt, downcast, upcast};
use core::num::traits::{Sqrt, WideMul};
use super::Fx;

pub const ONE: u128 = 0x10000000000000000;
const NZ_ONE: NonZero<u128> = 0x10000000000000000;

type U64B = BoundedInt<0, 0xffffffffffffffff>;
type HiSh = BoundedInt<0, 0xffffffffffffffff0000000000000000>;
impl MulHi of MulHelper<U64B, UnitInt<0x10000000000000000>> {
    type Result = HiSh;
}
impl AddHiLo of AddHelper<HiSh, U64B> {
    type Result = BoundedInt<0, 0xffffffffffffffffffffffffffffffff>;
}

#[derive(Copy, Drop, Debug, PartialEq)]
pub struct Sm128 {
    pub mag: u128,
    pub sign: bool,
}

pub impl Sm128Add of Add<Sm128> {
    fn add(lhs: Sm128, rhs: Sm128) -> Sm128 {
        if lhs.sign == rhs.sign {
            return Sm128 { mag: lhs.mag + rhs.mag, sign: lhs.sign };
        }
        if lhs.mag == rhs.mag {
            return Sm128 { mag: 0, sign: false };
        }
        if lhs.mag > rhs.mag {
            Sm128 { mag: lhs.mag - rhs.mag, sign: lhs.sign }
        } else {
            Sm128 { mag: rhs.mag - lhs.mag, sign: rhs.sign }
        }
    }
}
pub impl Sm128Sub of Sub<Sm128> {
    fn sub(lhs: Sm128, rhs: Sm128) -> Sm128 {
        Sm128Add::add(lhs, Sm128 { mag: rhs.mag, sign: !rhs.sign })
    }
}
pub impl Sm128Mul of Mul<Sm128> {
    fn mul(lhs: Sm128, rhs: Sm128) -> Sm128 {
        // (hi, lo) = a*b ; result = (hi << 64) | (lo >> 64), requires hi < 2^64
        let u256 { low: lo, high: hi } = lhs.mag.wide_mul(rhs.mag);
        let hi64: U64B = downcast(hi).expect('sm128 mul ovf');
        let (lo_hi, _) = DivRem::div_rem(lo, NZ_ONE);
        let lo_hi: U64B = downcast(lo_hi).unwrap();
        let mag: u128 = upcast(
            bounded_int::add(
                bounded_int::mul::<U64B, UnitInt<0x10000000000000000>>(hi64, 0x10000000000000000),
                lo_hi,
            ),
        );
        Sm128 { mag, sign: lhs.sign ^ rhs.sign }
    }
}
pub impl Sm128Div of Div<Sm128> {
    fn div(lhs: Sm128, rhs: Sm128) -> Sm128 {
        let n: u256 = lhs.mag.wide_mul(ONE);
        let d: NonZero<u256> = u256 { low: rhs.mag, high: 0 }.try_into().expect('sm128 div by 0');
        let (q, _) = DivRem::div_rem(n, d);
        assert(q.high == 0, 'sm128 div ovf');
        Sm128 { mag: q.low, sign: lhs.sign ^ rhs.sign }
    }
}
pub impl Sm128Neg of Neg<Sm128> {
    fn neg(a: Sm128) -> Sm128 {
        Sm128 { mag: a.mag, sign: !a.sign }
    }
}
pub impl Sm128PartialOrd of PartialOrd<Sm128> {
    fn lt(lhs: Sm128, rhs: Sm128) -> bool {
        if lhs.sign != rhs.sign {
            lhs.sign
        } else {
            (lhs.mag != rhs.mag) && ((lhs.mag < rhs.mag) ^ lhs.sign)
        }
    }
}
pub impl Sm128Fx of Fx<Sm128> {
    fn sqrt(self: Sm128) -> Sm128 {
        assert(!self.sign, 'sqrt of negative');
        // sqrt(mag * 2^64) exactly, via u256 sqrt (full 64 fractional bits, unlike cubit f128)
        let n: u256 = self.mag.wide_mul(ONE);
        Sm128 { mag: n.sqrt(), sign: false }
    }
}
