//! Native i64 32.32 storage; mul/div rescaling done with `core::internal::bounded_int`:
//! the product of two i64 is a BoundedInt whose range is known statically, so no range check is
//! needed until the single final downcast to i64 (which doubles as the overflow check).
//! Rounding: floor (toward -inf) for mul (offset trick), truncation toward zero for div.
use core::internal::bounded_int::{
    self, AddHelper, BoundedInt, DivRemHelper, MulHelper, SubHelper, UnitInt, downcast, upcast,
};
use core::num::traits::Sqrt;
use super::Fx;

// ---- type-level ranges -------------------------------------------------------------------
pub type Prod = BoundedInt<-0x3fffffffffffffff8000000000000000, 0x40000000000000000000000000000000>; // i64 * i64
type Off1 = UnitInt<0x40000000000000000000000000000000>; // 2^126, multiple of 2^32
type Sh1 = BoundedInt<0x8000000000000000, 0x80000000000000000000000000000000>; // Prod + 2^126 >= 0
type Q1 = BoundedInt<0x80000000, 0x800000000000000000000000>; // Sh1 / 2^32
type Off1Q = UnitInt<0x400000000000000000000000>; // 2^94
type R1 = BoundedInt<-0x3fffffffffffffff80000000, 0x400000000000000000000000>; // floor(Prod / 2^32)
type Scale = UnitInt<0x100000000>;
type Rem32 = BoundedInt<0, 0xffffffff>;

pub type Prod2 = BoundedInt<-0x7fffffffffffffff0000000000000000, 0x80000000000000000000000000000000>; // Prod + Prod
type Off2 = UnitInt<0x80000000000000000000000000000000>;
type Sh2 = BoundedInt<0x10000000000000000, 0x100000000000000000000000000000000>;
type Q2 = BoundedInt<0x100000000, 0x1000000000000000000000000>;
type Off2Q = UnitInt<0x800000000000000000000000>;
type R2 = BoundedInt<-0x7fffffffffffffff00000000, 0x800000000000000000000000>;

pub type Prod3 = BoundedInt<-0xbffffffffffffffe8000000000000000, 0xc0000000000000000000000000000000>; // Prod2 + Prod
type Off3 = UnitInt<0x100000000000000000000000000000000>;
type Sh3 = BoundedInt<0x40000000000000018000000000000000, 0x1c0000000000000000000000000000000>;
type Q3 = BoundedInt<0x400000000000000180000000, 0x1c00000000000000000000000>;
type Off3Q = UnitInt<0x1000000000000000000000000>;
type R3 = BoundedInt<-0xbffffffffffffffe80000000, 0xc00000000000000000000000>;

impl MulI64 of MulHelper<i64, i64> {
    type Result = Prod;
}
impl AddOff1 of AddHelper<Prod, Off1> {
    type Result = Sh1;
}
impl DivSh1 of DivRemHelper<Sh1, Scale> {
    type DivT = Q1;
    type RemT = Rem32;
}
impl SubOff1 of SubHelper<Q1, Off1Q> {
    type Result = R1;
}
impl AddProdProd of AddHelper<Prod, Prod> {
    type Result = Prod2;
}
impl AddOff2 of AddHelper<Prod2, Off2> {
    type Result = Sh2;
}
impl DivSh2 of DivRemHelper<Sh2, Scale> {
    type DivT = Q2;
    type RemT = Rem32;
}
impl SubOff2 of SubHelper<Q2, Off2Q> {
    type Result = R2;
}
impl AddProd2Prod of AddHelper<Prod2, Prod> {
    type Result = Prod3;
}
impl AddOff3 of AddHelper<Prod3, Off3> {
    type Result = Sh3;
}
impl DivSh3 of DivRemHelper<Sh3, Scale> {
    type DivT = Q3;
    type RemT = Rem32;
}
impl SubOff3 of SubHelper<Q3, Off3Q> {
    type Result = R3;
}

// division: |a| * 2^32 / |b|
type AbsNeg = BoundedInt<1, 0x8000000000000000>; // -x for x in [MIN, -1]
type AbsPos = BoundedInt<0, 0x7fffffffffffffff>;
type Abs = BoundedInt<0, 0x8000000000000000>;
type AbsNz = BoundedInt<1, 0x8000000000000000>;
type Num = BoundedInt<0, 0x800000000000000000000000>;
type QuoNeg = BoundedInt<-0x800000000000000000000000, 0>;
type QuoAll = BoundedInt<-0x800000000000000000000000, 0x800000000000000000000000>;
impl MulAbsScale of MulHelper<Abs, Scale> {
    type Result = Num;
}
impl DivNumNeg of DivRemHelper<Num, AbsNeg> {
    type DivT = Num;
    type RemT = BoundedInt<0, 0x7fffffffffffffff>;
}
impl DivNumPos of DivRemHelper<Num, AbsPos> {
    type DivT = Num;
    type RemT = BoundedInt<0, 0x7ffffffffffffffe>;
}
impl NegNum of MulHelper<Num, UnitInt<-1>> {
    type Result = QuoNeg;
}
// sqrt
type SqIn = BoundedInt<0, 0x7fffffffffffffff00000000>;
impl MulPosScale of MulHelper<AbsPos, Scale> {
    type Result = SqIn;
}

#[derive(Copy, Drop, Debug, PartialEq)]
pub struct I64b {
    pub v: i64,
}

const NZ_SCALE: NonZero<Scale> = 0x100000000;

/// Widening product, no range check, no rescale (for fused multiply-accumulate).
#[inline(always)]
pub fn wide(a: I64b, b: I64b) -> Prod {
    bounded_int::mul(a.v, b.v)
}
#[inline(always)]
pub fn rescale1(p: Prod) -> I64b {
    let sh: Sh1 = bounded_int::add::<Prod, Off1>(p, 0x40000000000000000000000000000000);
    let (q, _r) = bounded_int::div_rem(sh, NZ_SCALE);
    let r: R1 = bounded_int::sub::<Q1, Off1Q>(q, 0x400000000000000000000000);
    I64b { v: downcast(r).expect('i64b mul ovf') }
}
#[inline(always)]
pub fn rescale2(p: Prod2) -> I64b {
    let sh: Sh2 = bounded_int::add::<Prod2, Off2>(p, 0x80000000000000000000000000000000);
    let (q, _r) = bounded_int::div_rem(sh, NZ_SCALE);
    let r: R2 = bounded_int::sub::<Q2, Off2Q>(q, 0x800000000000000000000000);
    I64b { v: downcast(r).expect('i64b dot ovf') }
}
#[inline(always)]
pub fn rescale3(p: Prod3) -> I64b {
    let sh: Sh3 = bounded_int::add::<Prod3, Off3>(p, 0x100000000000000000000000000000000);
    let (q, _r) = bounded_int::div_rem(sh, NZ_SCALE);
    let r: R3 = bounded_int::sub::<Q3, Off3Q>(q, 0x1000000000000000000000000);
    I64b { v: downcast(r).expect('i64b dot ovf') }
}
/// a.b + c.d with one rescale and one range check.
#[inline(always)]
pub fn fused2(a: I64b, b: I64b, c: I64b, d: I64b) -> I64b {
    rescale2(bounded_int::add(wide(a, b), wide(c, d)))
}
/// a.b - c.d (2D cross) with one rescale: negate d's product by swapping sign of one factor is
/// not free for i64 (MIN), so use a second Prod with add of negated bounded product.
#[inline(always)]
pub fn fused3(a: I64b, b: I64b, c: I64b, d: I64b, e: I64b, f: I64b) -> I64b {
    let s2: Prod2 = bounded_int::add(wide(a, b), wide(c, d));
    rescale3(bounded_int::add(s2, wide(e, f)))
}

pub impl I64bAdd of Add<I64b> {
    fn add(lhs: I64b, rhs: I64b) -> I64b {
        I64b { v: lhs.v + rhs.v }
    }
}
pub impl I64bSub of Sub<I64b> {
    fn sub(lhs: I64b, rhs: I64b) -> I64b {
        I64b { v: lhs.v - rhs.v }
    }
}
pub impl I64bMul of Mul<I64b> {
    fn mul(lhs: I64b, rhs: I64b) -> I64b {
        rescale1(wide(lhs, rhs))
    }
}
#[inline(always)]
fn abs_sign(x: i64) -> (Abs, bool) {
    match bounded_int::constrain::<i64, 0>(x) {
        Ok(lt0) => (upcast::<AbsNeg, Abs>(bounded_int::NegateHelper::negate(lt0)), true),
        Err(ge0) => (upcast::<AbsPos, Abs>(ge0), false),
    }
}
pub impl I64bDiv of Div<I64b> {
    fn div(lhs: I64b, rhs: I64b) -> I64b {
        let (na, sa) = abs_sign(lhs.v);
        let nzb: NonZero<i64> = rhs.v.try_into().expect('i64b div by 0');
        let num: Num = bounded_int::mul::<Abs, Scale>(na, 0x100000000);
        let (q, sb) = match bounded_int::constrain::<NonZero<i64>, 0>(nzb) {
            Ok(lt0) => {
                let d: NonZero<AbsNeg> = bounded_int::NegateHelper::negate(lt0);
                let (q, _r) = bounded_int::div_rem(num, d);
                (q, true)
            },
            Err(ge0) => {
                let (q, _r) = bounded_int::div_rem(num, ge0);
                (q, false)
            },
        };
        let signed: QuoAll = if sa ^ sb {
            upcast::<QuoNeg, QuoAll>(bounded_int::mul::<Num, UnitInt<-1>>(q, -1))
        } else {
            upcast::<Num, QuoAll>(q)
        };
        I64b { v: downcast(signed).expect('i64b div ovf') }
    }
}
pub impl I64bNeg of Neg<I64b> {
    fn neg(a: I64b) -> I64b {
        I64b { v: -a.v }
    }
}
pub impl I64bPartialOrd of PartialOrd<I64b> {
    fn lt(lhs: I64b, rhs: I64b) -> bool {
        lhs.v < rhs.v
    }
}
pub impl I64bFx of Fx<I64b> {
    fn sqrt(self: I64b) -> I64b {
        let pos: AbsPos = match bounded_int::constrain::<i64, 0>(self.v) {
            Ok(_) => core::panic_with_felt252('sqrt of negative'),
            Err(ge0) => ge0,
        };
        let scaled: u128 = upcast(bounded_int::mul::<AbsPos, Scale>(pos, 0x100000000));
        let r: u64 = scaled.sqrt();
        I64b { v: downcast(r).unwrap() }
    }
}

// 2D cross: Prod - Prod
pub type ProdD = BoundedInt<-0x7fffffffffffffff8000000000000000, 0x7fffffffffffffff8000000000000000>;
type ShD = BoundedInt<0x8000000000000000, 0xffffffffffffffff8000000000000000>;
type QD = BoundedInt<0x80000000, 0xffffffffffffffff80000000>;
type RD = BoundedInt<-0x7fffffffffffffff80000000, 0x7fffffffffffffff80000000>;
impl SubProdProd of SubHelper<Prod, Prod> {
    type Result = ProdD;
}
impl AddOffD of AddHelper<ProdD, Off2> {
    type Result = ShD;
}
impl DivShD of DivRemHelper<ShD, Scale> {
    type DivT = QD;
    type RemT = Rem32;
}
impl SubOffD of SubHelper<QD, Off2Q> {
    type Result = RD;
}
/// a.b - c.d with one rescale and one range check.
#[inline(always)]
pub fn fused_diff(a: I64b, b: I64b, c: I64b, d: I64b) -> I64b {
    let p: ProdD = bounded_int::sub(wide(a, b), wide(c, d));
    let sh: ShD = bounded_int::add::<ProdD, Off2>(p, 0x80000000000000000000000000000000);
    let (q, _r) = bounded_int::div_rem(sh, NZ_SCALE);
    let r: RD = bounded_int::sub::<QD, Off2Q>(q, 0x800000000000000000000000);
    I64b { v: downcast(r).expect('i64b cross ovf') }
}

// 6-term accumulation (e.g. a 2D constraint row J.v for two bodies): Prod3 + Prod3
pub type Prod6 = BoundedInt<-0x17ffffffffffffffd0000000000000000, 0x180000000000000000000000000000000>;
type Off6 = UnitInt<0x400000000000000000000000000000000>;
type Sh6 = BoundedInt<0x280000000000000030000000000000000, 0x580000000000000000000000000000000>;
type Q6 = BoundedInt<0x2800000000000000300000000, 0x5800000000000000000000000>;
type Off6Q = UnitInt<0x4000000000000000000000000>;
type R6 = BoundedInt<-0x17ffffffffffffffd00000000, 0x1800000000000000000000000>;
impl AddProd3Prod3 of AddHelper<Prod3, Prod3> {
    type Result = Prod6;
}
impl AddOff6 of AddHelper<Prod6, Off6> {
    type Result = Sh6;
}
impl DivSh6 of DivRemHelper<Sh6, Scale> {
    type DivT = Q6;
    type RemT = Rem32;
}
impl SubOff6 of SubHelper<Q6, Off6Q> {
    type Result = R6;
}
/// sum of 6 products with one rescale and one range check.
#[inline(always)]
pub fn fused6(a: [I64b; 6], b: [I64b; 6]) -> I64b {
    let [a0, a1, a2, a3, a4, a5] = a;
    let [b0, b1, b2, b3, b4, b5] = b;
    let s: Prod3 = bounded_int::add(bounded_int::add(wide(a0, b0), wide(a1, b1)), wide(a2, b2));
    let t: Prod3 = bounded_int::add(bounded_int::add(wide(a3, b3), wide(a4, b4)), wide(a5, b5));
    let p: Prod6 = bounded_int::add(s, t);
    let sh: Sh6 = bounded_int::add::<Prod6, Off6>(p, 0x400000000000000000000000000000000);
    let (q, _r) = bounded_int::div_rem(sh, NZ_SCALE);
    let r: R6 = bounded_int::sub::<Q6, Off6Q>(q, 0x4000000000000000000000000);
    I64b { v: downcast(r).expect('i64b dot6 ovf') }
}

/// Euclidean length straight from the wide sum of squares: sqrt(x^2 + y^2) where the squares keep
/// their 2^64 scale, so the u128 sqrt directly yields a 32.32 result. No rescale, and the squared
/// length never has to fit 32.32 (no overflow for |v| > 46340).
#[inline(always)]
pub fn length_wide2(x: I64b, y: I64b) -> I64b {
    let s: Prod2 = bounded_int::add(wide(x, x), wide(y, y));
    let u: u128 = downcast(s).expect('len ovf'); // sum of squares is >= 0; fails only if >= 2^128 (never: max 2^127)
    let r: u64 = u.sqrt();
    I64b { v: downcast(r).expect('len ovf') }
}
