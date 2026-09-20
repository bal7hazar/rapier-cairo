use core::integer;

use orion::numbers::fixed_point::implementations::fp8x23::math::lut;
use orion::numbers::fixed_point::implementations::fp8x23::core::{
    HALF, ONE, TWO, FP8x23, FP8x23Impl, FP8x23Add, FP8x23Sub, FP8x23Mul, FP8x23Div,
    FP8x23IntoFelt252, FixedTrait
};

// CONSTANTS
const TWO_PI: u32 = 52707178;
const PI: u32 = 26353589;
const HALF_PI: u32 = 13176795;

// PUBLIC

// Calculates arccos(a) for -1 <= a <= 1 (fixed point)
// arccos(a) = arcsin(sqrt(1 - a^2)) - arctan identity has discontinuity at zero
fn acos(a: FP8x23) -> FP8x23 {
    let asin_arg = (FixedTrait::ONE() - a * a).sqrt(); // will fail if a > 1
    let asin_res = asin(asin_arg);

    if (a.sign) {
        FixedTrait::new(PI, false) - asin_res
    } else {
        asin_res
    }
}

fn acos_fast(a: FP8x23) -> FP8x23 {
    let asin_arg = (FixedTrait::ONE() - a * a).sqrt(); // will fail if a > 1
    let asin_res = asin_fast(asin_arg);

    if (a.sign) {
        FixedTrait::new(PI, false) - asin_res
    } else {
        asin_res
    }
}

// Calculates arcsin(a) for -1 <= a <= 1 (fixed point)
// arcsin(a) = arctan(a / sqrt(1 - a^2))
fn asin(a: FP8x23) -> FP8x23 {
    if (a.mag == ONE) {
        return FixedTrait::new(HALF_PI, a.sign);
    }

    let div = (FixedTrait::ONE() - a * a).sqrt(); // will fail if a > 1

    atan(a / div)
}

fn asin_fast(a: FP8x23) -> FP8x23 {
    if (a.mag == ONE) {
        return FixedTrait::new(HALF_PI, a.sign);
    }

    let div = (FixedTrait::ONE() - a * a).sqrt(); // will fail if a > 1

    atan_fast(a / div)
}

// Calculates arctan(a) (fixed point)
// See https://stackoverflow.com/a/50894477 for range adjustments
fn atan(a: FP8x23) -> FP8x23 {
    let mut at = a.abs();
    let mut shift = false;
    let mut invert = false;

    // Invert value when a > 1
    if (at.mag > ONE) {
        at = FixedTrait::ONE() / at;
        invert = true;
    }

    // Account for lack of precision in polynomaial when a > 0.7
    if (at.mag > 5872026) {
        let sqrt3_3 = FixedTrait::new(4843165, false); // sqrt(3) / 3
        at = (at - sqrt3_3) / (FixedTrait::ONE() + at * sqrt3_3);
        shift = true;
    }

    let r10 = FixedTrait::new(15363, true) * at;
    let r9 = (r10 + FixedTrait::new(392482, true)) * at;
    let r8 = (r9 + FixedTrait::new(1629064, false)) * at;
    let r7 = (r8 + FixedTrait::new(2197820, true)) * at;
    let r6 = (r7 + FixedTrait::new(366693, false)) * at;
    let r5 = (r6 + FixedTrait::new(1594324, false)) * at;
    let r4 = (r5 + FixedTrait::new(11519, false)) * at;
    let r3 = (r4 + FixedTrait::new(2797104, true)) * at;
    let r2 = (r3 + FixedTrait::new(34, false)) * at;
    let mut res = (r2 + FixedTrait::new(8388608, false)) * at;

    // Adjust for sign change, inversion, and shift
    if (shift) {
        res = res + FixedTrait::new(4392265, false); // pi / 6
    }

    if (invert) {
        res = res - FixedTrait::new(HALF_PI, false);
    }

    FixedTrait::new(res.mag, a.sign)
}

fn atan_fast(a: FP8x23) -> FP8x23 {
    let mut at = a.abs();
    let mut shift = false;
    let mut invert = false;

    // Invert value when a > 1
    if (at.mag > ONE) {
        at = FixedTrait::ONE() / at;
        invert = true;
    }

    // Account for lack of precision in polynomaial when a > 0.7
    if (at.mag > 5872026) {
        let sqrt3_3 = FixedTrait::new(4843165, false); // sqrt(3) / 3
        at = (at - sqrt3_3) / (FixedTrait::ONE() + at * sqrt3_3);
        shift = true;
    }

    let (start, low, high) = lut::atan(at.mag);
    let partial_step = FixedTrait::new(at.mag - start, false) / FixedTrait::new(58720, false);
    let mut res = partial_step * FixedTrait::new(high - low, false) + FixedTrait::new(low, false);

    // Adjust for sign change, inversion, and shift
    if (shift) {
        res = res + FixedTrait::new(4392265, false); // pi / 6
    }

    if (invert) {
        res = res - FixedTrait::<FP8x23>::new(HALF_PI, false);
    }

    FixedTrait::new(res.mag, a.sign)
}

// Calculates cos(a) with a in radians (fixed point)
fn cos(a: FP8x23) -> FP8x23 {
    sin(FixedTrait::new(HALF_PI, false) - a)
}

fn cos_fast(a: FP8x23) -> FP8x23 {
    sin_fast(FixedTrait::new(HALF_PI, false) - a)
}

fn sin(a: FP8x23) -> FP8x23 {
    let a1 = a.mag % TWO_PI;
    let (whole_rem, partial_rem) = integer::u32_safe_divmod(a1, integer::u32_as_non_zero(PI));
    let a2 = FixedTrait::new(partial_rem, false);
    let partial_sign = whole_rem == 1;

    let loop_res = a2 * _sin_loop(a2, 7, FixedTrait::ONE());

    FixedTrait::new(loop_res.mag, a.sign ^ partial_sign && loop_res.mag != 0)
}

fn sin_fast(a: FP8x23) -> FP8x23 {
    let a1 = a.mag % TWO_PI;
    let (whole_rem, mut partial_rem) = integer::u32_safe_divmod(a1, integer::u32_as_non_zero(PI));
    let partial_sign = whole_rem == 1;

    if partial_rem >= HALF_PI {
        partial_rem = PI - partial_rem;
    }

    let (start, low, high) = lut::sin(partial_rem);
    let partial_step = FixedTrait::new(partial_rem - start, false) / FixedTrait::new(51472, false);
    let res = partial_step * (FixedTrait::new(high, false) - FixedTrait::new(low, false))
        + FixedTrait::<FP8x23>::new(low, false);

    FixedTrait::new(res.mag, a.sign ^ partial_sign && res.mag != 0)
}

// Calculates tan(a) with a in radians (fixed point)
fn tan(a: FP8x23) -> FP8x23 {
    let sinx = sin(a);
    let cosx = cos(a);
    assert(cosx.mag != 0, 'tan undefined');

    sinx / cosx
}

fn tan_fast(a: FP8x23) -> FP8x23 {
    let sinx = sin_fast(a);
    let cosx = cos_fast(a);
    assert(cosx.mag != 0, 'tan undefined');

    sinx / cosx
}

// Helper function to calculate Taylor series for sin
fn _sin_loop(a: FP8x23, i: u32, acc: FP8x23) -> FP8x23 {
    let div = (2 * i + 2) * (2 * i + 3);
    let term = a * a * acc / FixedTrait::new_unscaled(div, false);
    let new_acc = FixedTrait::ONE() - term;

    if (i == 0) {
        return new_acc;
    }

    _sin_loop(a, i - 1, new_acc)
}

