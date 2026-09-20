//! Sign-magnitude 32.32 (cubit style), re-implemented with NonZero constant divisors.
use core::num::traits::{Sqrt, WideMul};
use super::Fx;

pub const ONE: u64 = 0x100000000;
const NZ_ONE_U128: NonZero<u128> = 0x100000000;

#[derive(Copy, Drop, Debug, PartialEq)]
pub struct Sm64 {
    pub mag: u64,
    pub sign: bool,
}

pub impl Sm64Add of Add<Sm64> {
    fn add(lhs: Sm64, rhs: Sm64) -> Sm64 {
        if lhs.sign == rhs.sign {
            return Sm64 { mag: lhs.mag + rhs.mag, sign: lhs.sign };
        }
        if lhs.mag == rhs.mag {
            return Sm64 { mag: 0, sign: false };
        }
        if lhs.mag > rhs.mag {
            Sm64 { mag: lhs.mag - rhs.mag, sign: lhs.sign }
        } else {
            Sm64 { mag: rhs.mag - lhs.mag, sign: rhs.sign }
        }
    }
}

pub impl Sm64Sub of Sub<Sm64> {
    fn sub(lhs: Sm64, rhs: Sm64) -> Sm64 {
        // Negation inlined (no negative-zero normalisation needed: add() handles equal magnitudes).
        Sm64Add::add(lhs, Sm64 { mag: rhs.mag, sign: !rhs.sign })
    }
}

pub impl Sm64Mul of Mul<Sm64> {
    fn mul(lhs: Sm64, rhs: Sm64) -> Sm64 {
        let p: u128 = lhs.mag.wide_mul(rhs.mag);
        let (q, _) = DivRem::div_rem(p, NZ_ONE_U128);
        Sm64 { mag: q.try_into().expect('sm64 mul ovf'), sign: lhs.sign ^ rhs.sign }
    }
}

pub impl Sm64Div of Div<Sm64> {
    fn div(lhs: Sm64, rhs: Sm64) -> Sm64 {
        let n: u128 = lhs.mag.wide_mul(ONE);
        let d: NonZero<u128> = Into::<u64, u128>::into(rhs.mag).try_into().expect('sm64 div by 0');
        let (q, _) = DivRem::div_rem(n, d);
        Sm64 { mag: q.try_into().expect('sm64 div ovf'), sign: lhs.sign ^ rhs.sign }
    }
}

pub impl Sm64Neg of Neg<Sm64> {
    fn neg(a: Sm64) -> Sm64 {
        Sm64 { mag: a.mag, sign: !a.sign }
    }
}

pub impl Sm64PartialOrd of PartialOrd<Sm64> {
    fn lt(lhs: Sm64, rhs: Sm64) -> bool {
        if lhs.sign != rhs.sign {
            lhs.sign
        } else {
            (lhs.mag != rhs.mag) && ((lhs.mag < rhs.mag) ^ lhs.sign)
        }
    }
}

pub impl Sm64Fx of Fx<Sm64> {
    fn sqrt(self: Sm64) -> Sm64 {
        assert(!self.sign, 'sqrt of negative');
        let scaled: u128 = self.mag.wide_mul(ONE);
        Sm64 { mag: scaled.sqrt(), sign: false }
    }
}
