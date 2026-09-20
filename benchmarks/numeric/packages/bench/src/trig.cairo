//! Section D: cheap trigonometry on the i64 32.32 scalar.
//! Range reduction is pure arithmetic (DivRem by pi, parity by DivRem 2); the polynomial core
//! runs on raw felt252 with one rescale (= one range check block) per multiplication. All
//! intermediate magnitudes are statically tiny (< 2^35), so the lazy-felt arithmetic is sound here.
use core::internal::bounded_int::{self, BoundedInt, upcast};
use crate::reprs::felt::rescale;
use crate::reprs::i64b::I64b;

pub const PI: u64 = 13493037705;
pub const HALF_PI: u64 = 6746518852;
const NZ_PI: NonZero<u64> = 13493037705;
const NZ_2: NonZero<u64> = 2;
const FIVE_PI2_64: felt252 = 910310332478264180736; // 5*pi^2 scaled by 2^64

#[inline(always)]
fn abs_sign(x: i64) -> (u64, bool) {
    match bounded_int::constrain::<i64, 0>(x) {
        Ok(lt0) => (upcast::<BoundedInt<1, 0x8000000000000000>, u64>(bounded_int::NegateHelper::negate(lt0)), true),
        Err(ge0) => (upcast(ge0), false),
    }
}

/// Reduces an angle to r in [0, pi/2] and the sign of sin.
#[inline(always)]
fn reduce(x: i64) -> (u64, bool) {
    let (mag, neg) = abs_sign(x);
    let (k, r) = DivRem::div_rem(mag, NZ_PI);
    let (_, odd) = DivRem::div_rem(k, NZ_2);
    let neg = neg ^ (odd == 1);
    let r = if r > HALF_PI { PI - r } else { r };
    (r, neg)
}

#[inline(always)]
fn finish(res: felt252, neg: bool) -> I64b {
    let v: i64 = res.try_into().unwrap();
    if neg { I64b { v: -v } } else { I64b { v } }
}

/// Odd minimax polynomial of degree 5 on [0, pi/2] (max error ~6.8e-5).
pub fn sin_poly5(x: I64b) -> I64b {
    let (r, neg) = reduce(x.v);
    let rf: felt252 = r.into();
    let z = rescale(rf * rf).v;
    let mut p: felt252 = 32274275;
    p = rescale(p * z).v + (-711561176);
    p = rescale(p * z).v + (4293665310);
    finish(rescale(p * rf).v, neg)
}
/// Odd minimax polynomial of degree 7 on [0, pi/2] (max error ~5.9e-7).
pub fn sin_poly7(x: I64b) -> I64b {
    let (r, neg) = reduce(x.v);
    let rf: felt252 = r.into();
    let z = rescale(rf * rf).v;
    let mut p: felt252 = -788713;
    p = rescale(p * z).v + (35675395);
    p = rescale(p * z).v + (-715748929);
    p = rescale(p * z).v + (4294952762);
    finish(rescale(p * rf).v, neg)
}
/// Odd minimax polynomial of degree 9 on [0, pi/2] (max error ~3.3e-9 before rounding).
pub fn sin_poly9(x: I64b) -> I64b {
    let (r, neg) = reduce(x.v);
    let rf: felt252 = r.into();
    let z = rescale(rf * rf).v;
    let mut p: felt252 = 11126;
    p = rescale(p * z).v + (-850442);
    p = rescale(p * z).v + (35789532);
    p = rescale(p * z).v + (-715827065);
    p = rescale(p * z).v + (4294967195);
    finish(rescale(p * rf).v, neg)
}
pub fn cos_poly7(x: I64b) -> I64b {
    sin_poly7(I64b { v: x.v + 6746518852 })
}
/// Bhaskara I: sin(r) ~ 16 r (pi - r) / (5 pi^2 - 4 r (pi - r)), r in [0, pi] (max error ~1.6e-3).
pub fn sin_bhaskara(x: I64b) -> I64b {
    let (mag, neg) = abs_sign(x.v);
    let (k, r) = DivRem::div_rem(mag, NZ_PI);
    let (_, odd) = DivRem::div_rem(k, NZ_2);
    let neg = neg ^ (odd == 1);
    let rf: felt252 = r.into();
    let t: felt252 = rf * (13493037705 - rf); // scale 2^64, < 2^68
    let num: u128 = (t * 0x1000000000).try_into().unwrap(); // 16 * t * 2^32
    let den: u128 = (FIVE_PI2_64 - 4 * t).try_into().unwrap();
    let (q, _) = DivRem::div_rem(num, den.try_into().unwrap());
    finish(q.into(), neg)
}

/// atan2(y, x): one division + odd minimax polynomial of degree 11 on [0, 1] (max error ~1.7e-6).
pub fn atan2_poly11(y: I64b, x: I64b) -> I64b {
    let (ay, sy) = abs_sign(y.v);
    let (ax, sx) = abs_sign(x.v);
    let (lo, hi, swap) = if ay > ax { (ax, ay, true) } else { (ay, ax, false) };
    if hi == 0 {
        return I64b { v: 0 };
    }
    let num: u128 = core::num::traits::WideMul::wide_mul(lo, 0x100000000_u64);
    let den: NonZero<u128> = Into::<u64, u128>::into(hi).try_into().unwrap();
    let (q, _) = DivRem::div_rem(num, den); // q in [0, 2^32]
    let qf: felt252 = q.into();
    let z = rescale(qf * qf).v;
    let mut p: felt252 = -50329876;
    p = rescale(p * z).v + (226110354);
    p = rescale(p * z).v + (-500040732);
    p = rescale(p * z).v + (831246900);
    p = rescale(p * z).v + (-1428603768);
    p = rescale(p * z).v + (4294869437);
    let mut a = rescale(p * qf).v;
    if swap { a = 6746518852 - a; }
    if sx { a = 13493037705 - a; }
    finish(a, sy)
}
