//! Native i64 32.32, "naive" version: i128 widening and the core signed Div.
use core::num::traits::{Sqrt, WideMul};
use super::Fx;

const ONE_I128: i128 = 0x100000000;

#[derive(Copy, Drop, Debug, PartialEq)]
pub struct I64n {
    pub v: i64,
}

pub impl I64nAdd of Add<I64n> {
    fn add(lhs: I64n, rhs: I64n) -> I64n {
        I64n { v: lhs.v + rhs.v }
    }
}
pub impl I64nSub of Sub<I64n> {
    fn sub(lhs: I64n, rhs: I64n) -> I64n {
        I64n { v: lhs.v - rhs.v }
    }
}
pub impl I64nMul of Mul<I64n> {
    fn mul(lhs: I64n, rhs: I64n) -> I64n {
        let p: i128 = lhs.v.wide_mul(rhs.v);
        I64n { v: (p / ONE_I128).try_into().expect('i64n mul ovf') }
    }
}
pub impl I64nDiv of Div<I64n> {
    fn div(lhs: I64n, rhs: I64n) -> I64n {
        let n: i128 = lhs.v.into() * ONE_I128;
        I64n { v: (n / rhs.v.into()).try_into().expect('i64n div ovf') }
    }
}
pub impl I64nNeg of Neg<I64n> {
    fn neg(a: I64n) -> I64n {
        I64n { v: -a.v }
    }
}
pub impl I64nPartialOrd of PartialOrd<I64n> {
    fn lt(lhs: I64n, rhs: I64n) -> bool {
        lhs.v < rhs.v
    }
}
pub impl I64nFx of Fx<I64n> {
    fn sqrt(self: I64n) -> I64n {
        let m: u64 = self.v.try_into().expect('sqrt of negative');
        let scaled: u128 = m.wide_mul(0x100000000_u64);
        let r: u64 = scaled.sqrt();
        I64n { v: r.try_into().unwrap() }
    }
}
