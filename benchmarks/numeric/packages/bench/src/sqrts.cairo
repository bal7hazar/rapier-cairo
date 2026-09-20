//! Section C: square roots of an unsigned 32.32 magnitude (u64 -> u64).
use core::num::traits::{Sqrt, WideMul};

const NZ_2_U128: NonZero<u128> = 2;

/// cubit f64 algorithm: checked u128 mul then core sqrt.
pub fn sqrt_cubit_style(mag: u64) -> u64 {
    let scaled: u128 = mag.into() * 0x100000000_u128;
    scaled.sqrt()
}
/// core u128_sqrt on the widened magnitude (exact floor(sqrt(mag << 32))).
pub fn sqrt_core_u128(mag: u64) -> u64 {
    mag.wide_mul(0x100000000_u64).sqrt()
}
/// core u64_sqrt then scale by 2^16: cheap but only 16 fractional bits of precision.
pub fn sqrt_core_u64_lowprec(mag: u64) -> u64 {
    let r: u32 = mag.sqrt();
    r.into() * 0x10000_u64
}
#[inline(always)]
fn newton_step(n: u128, x: u128) -> u128 {
    let (q, _) = DivRem::div_rem(n, x.try_into().unwrap());
    let (h, _) = DivRem::div_rem(x + q, NZ_2_U128);
    h
}
/// Newton (Heron) on N = mag << 32, fixed 8 iterations, loop. x0 = (N/2^32 + 2^32)/2 >= sqrt(N).
pub fn sqrt_newton_loop(mag: u64) -> u64 {
    if mag == 0 {
        return 0;
    }
    let n: u128 = mag.wide_mul(0x100000000_u64);
    let mut x: u128 = (mag.into() + 0x100000000_u128) / 2;
    let mut i: u32 = 0;
    while i != 8 {
        x = newton_step(n, x);
        i += 1;
    }
    x.try_into().unwrap()
}
/// Same, 8 iterations unrolled.
pub fn sqrt_newton_unrolled(mag: u64) -> u64 {
    if mag == 0 {
        return 0;
    }
    let n: u128 = mag.wide_mul(0x100000000_u64);
    let x: u128 = (mag.into() + 0x100000000_u128) / 2;
    let x = newton_step(n, x);
    let x = newton_step(n, x);
    let x = newton_step(n, x);
    let x = newton_step(n, x);
    let x = newton_step(n, x);
    let x = newton_step(n, x);
    let x = newton_step(n, x);
    let x = newton_step(n, x);
    x.try_into().unwrap()
}
/// Inverse sqrt: 2^64 / floor(sqrt(mag << 32))  (one core sqrt + one u128 division).
pub fn inv_sqrt_core(mag: u64) -> u64 {
    let r: u64 = mag.wide_mul(0x100000000_u64).sqrt();
    let d: NonZero<u128> = Into::<u64, u128>::into(r).try_into().expect('inv_sqrt of 0');
    let (q, _) = DivRem::div_rem(0x10000000000000000_u128, d);
    q.try_into().expect('inv_sqrt ovf')
}
/// Inverse sqrt with full precision: sqrt(2^96 / mag) computed as u128_sqrt(2^96 / mag)... the
/// quotient 2^96/mag keeps 32 extra bits, so precision does not degrade for large inputs.
pub fn inv_sqrt_div_first(mag: u64) -> u64 {
    let d: NonZero<u128> = Into::<u64, u128>::into(mag).try_into().expect('inv_sqrt of 0');
    let (q, _) = DivRem::div_rem(0x1000000000000000000000000_u128, d); // 2^96 / mag
    q.sqrt()
}
