//! felt252-backed signed 32.32 with *lazy* range checks.
//!
//! Representation: the signed integer x (raw fixed-point value) is stored as `x mod P`.
//! add / sub / neg are raw field ops: 0 range checks.
//! Soundness invariant (see report): every value is a sum of at most 2^30 "leaves", each leaf
//! being an input checked to |x| < 2^63 or the output of mul/div/sqrt (|x| < 2^96). Then
//! |x| < 2^126 always, products |a.b| < 2^252 < P/2 never wrap, and the single
//! `felt252 -> u128` conversion performed by mul / lt / div / sqrt is the overflow check
//! (it panics instead of silently wrapping).
use core::num::traits::Sqrt;
use super::Fx;

const OFF: felt252 = 0x80000000000000000000000000000000; // 2^127
const OFF_U128: u128 = 0x80000000000000000000000000000000;
const OFF_Q: felt252 = 0x800000000000000000000000; // 2^95
const NZ_ONE: NonZero<u128> = 0x100000000;
const ONE_U128: u128 = 0x100000000;

#[derive(Copy, Drop, Debug, PartialEq)]
pub struct Fe {
    pub v: felt252,
}

/// floor(p / 2^32) for a lazily accumulated product sum p, |p| < 2^127 (panics otherwise).
#[inline(always)]
pub fn rescale(p: felt252) -> Fe {
    let u: u128 = (p + OFF).try_into().expect('fe ovf');
    let (q, _) = DivRem::div_rem(u, NZ_ONE);
    Fe { v: q.into() - OFF_Q }
}

/// (|x|, x < 0); panics if |x| >= 2^127.
#[inline(always)]
fn abs_sign(x: felt252) -> (u128, bool) {
    let u: u128 = (x + OFF).try_into().expect('fe ovf');
    if u >= OFF_U128 {
        (u - OFF_U128, false)
    } else {
        (OFF_U128 - u, true)
    }
}

/// Explicit normalisation: checks the value fits a real i64 (to be used at storage boundaries).
pub fn check(a: Fe) -> Fe {
    let _u: u64 = (a.v + 0x8000000000000000).try_into().expect('fe range');
    a
}

pub impl FeAdd of Add<Fe> {
    #[inline(always)]
    fn add(lhs: Fe, rhs: Fe) -> Fe {
        Fe { v: lhs.v + rhs.v }
    }
}
pub impl FeSub of Sub<Fe> {
    #[inline(always)]
    fn sub(lhs: Fe, rhs: Fe) -> Fe {
        Fe { v: lhs.v - rhs.v }
    }
}
pub impl FeNeg of Neg<Fe> {
    #[inline(always)]
    fn neg(a: Fe) -> Fe {
        Fe { v: -a.v }
    }
}
pub impl FeMul of Mul<Fe> {
    fn mul(lhs: Fe, rhs: Fe) -> Fe {
        rescale(lhs.v * rhs.v)
    }
}
pub impl FeDiv of Div<Fe> {
    fn div(lhs: Fe, rhs: Fe) -> Fe {
        let (_, sa) = abs_sign(lhs.v);
        let (nb, sb) = abs_sign(rhs.v);
        // |a| * 2^32 through the field: a single conversion instead of a checked u128 mul
        let na_scaled: felt252 = if sa { lhs.v * -0x100000000 } else { lhs.v * 0x100000000 };
        let num: u128 = na_scaled.try_into().expect('fe div ovf'); // panics if |a| >= 2^96
        let d: NonZero<u128> = nb.try_into().expect('fe div by 0');
        let (q, _) = DivRem::div_rem(num, d);
        if sa ^ sb {
            Fe { v: -q.into() }
        } else {
            Fe { v: q.into() }
        }
    }
}
pub impl FePartialOrd of PartialOrd<Fe> {
    fn lt(lhs: Fe, rhs: Fe) -> bool {
        // lhs - rhs + 2^127 in [0, 2^128); negative difference <=> below 2^127
        let d: u128 = (lhs.v - rhs.v + OFF).try_into().expect('fe ovf');
        d < OFF_U128
    }
}
pub impl FeFx of Fx<Fe> {
    fn sqrt(self: Fe) -> Fe {
        // one conversion: negative values map to huge felts and fail it; |x| >= 2^96 fails too
        let u: u128 = (self.v * 0x100000000).try_into().expect('sqrt neg or ovf');
        let r: u64 = u.sqrt();
        Fe { v: r.into() }
    }
}
