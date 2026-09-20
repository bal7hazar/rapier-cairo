use core::debug::PrintTrait;
use core::integer::{u128_safe_divmod, u128_as_non_zero};
use core::option::OptionTrait;

use cubit::f128::math::lut;
use cubit::f128::types::fixed::{
    Fixed, FixedTrait, FixedAdd, FixedSub, FixedMul, FixedDiv, ONE_u128
};

// CONSTANTS

const PI_u128: u128 = 57952155664616982739;
const HALF_PI_u128: u128 = 28976077832308491370;

// PUBLIC

// Calculates arccos(a) for -1 <= a <= 1 (fixed point)
// arccos(a) = arcsin(sqrt(1 - a^2)) - arctan identity has discontinuity at zero
fn acos(a: Fixed) -> Fixed {
    let asin_arg = (FixedTrait::ONE() - a * a).sqrt(); // will fail if a > 1
    let asin_res = asin(asin_arg);

    if (a.sign) {
        return FixedTrait::new(PI_u128, false) - asin_res;
    } else {
        return asin_res;
    }
}

fn acos_fast(a: Fixed) -> Fixed {
    let asin_arg = (FixedTrait::ONE() - a * a).sqrt(); // will fail if a > 1
    let asin_res = asin_fast(asin_arg);

    if (a.sign) {
        return FixedTrait::new(PI_u128, false) - asin_res;
    } else {
        return asin_res;
    }
}

// Calculates arcsin(a) for -1 <= a <= 1 (fixed point)
// arcsin(a) = arctan(a / sqrt(1 - a^2))
fn asin(a: Fixed) -> Fixed {
    assert(a.mag <= ONE_u128, 'out of range');

    if (a.mag == ONE_u128) {
        return FixedTrait::new(HALF_PI_u128, a.sign);
    }

    let div = (FixedTrait::ONE() - a * a).sqrt();
    return atan(a / div);
}

fn asin_fast(a: Fixed) -> Fixed {
    assert(a.mag <= ONE_u128, 'out of range');

    if (a.mag == ONE_u128) {
        return FixedTrait::new(HALF_PI_u128, a.sign);
    }

    let div = (FixedTrait::ONE() - a * a).sqrt();
    return atan_fast(a / div);
}

// Calculates arctan(a) (fixed point)
// See https://stackoverflow.com/a/50894477 for range adjustments
fn atan(a: Fixed) -> Fixed {
    let mut at = a.abs();
    let mut shift = false;
    let mut invert = false;

    // Invert value when a > 1
    if (at.mag > ONE_u128) {
        at = FixedTrait::ONE() / at;
        invert = true;
    }

    // Account for lack of precision in polynomaial when a > 0.7
    if (at.mag > 12912720851596686131) {
        let sqrt3_3 = FixedTrait::new(10650232656328343401, false); // sqrt(3) / 3
        at = (at - sqrt3_3) / (FixedTrait::ONE() + at * sqrt3_3);
        shift = true;
    }

    let r10 = FixedTrait::new(33784601907694228, true) * at;
    let r9 = (r10 + FixedTrait::new(863077567022907619, true)) * at;
    let r8 = (r9 + FixedTrait::new(3582351446937658863, false)) * at;
    let r7 = (r8 + FixedTrait::new(4833057334070945981, true)) * at;
    let r6 = (r7 + FixedTrait::new(806366139934153963, false)) * at;
    let r5 = (r6 + FixedTrait::new(3505955710573417812, false)) * at;
    let r4 = (r5 + FixedTrait::new(25330242983263508, false)) * at;
    let r3 = (r4 + FixedTrait::new(6150896368532115927, true)) * at;
    let r2 = (r3 + FixedTrait::new(75835542453775, false)) * at;
    let mut res = (r2 + FixedTrait::new(18446743057812048409, false)) * at;

    // Adjust for sign change, inversion, and shift
    if (shift) {
        res = res + FixedTrait::new(9658692610769497123, false); // pi / 6
    }

    if (invert) {
        res = res - FixedTrait::new(HALF_PI_u128, false);
    }

    return FixedTrait::new(res.mag, a.sign);
}

fn atan_fast(a: Fixed) -> Fixed {
    let mut at = a.abs();
    let mut shift = false;
    let mut invert = false;

    // Invert value when a > 1
    if (at.mag > ONE_u128) {
        at = FixedTrait::ONE() / at;
        invert = true;
    }

    // Account for lack of precision in polynomaial when a > 0.7
    if (at.mag > 12912720851596686131) {
        let sqrt3_3 = FixedTrait::new(10650232656328343401, false); // sqrt(3) / 3
        at = (at - sqrt3_3) / (FixedTrait::ONE() + at * sqrt3_3);
        shift = true;
    }

    let (start, low, high) = lut::atan(at.mag);
    let partial_step = FixedTrait::new(at.mag - start, false)
        / FixedTrait::new(129127208515966848, false);
    let mut res = partial_step * FixedTrait::new(high - low, false) + FixedTrait::new(low, false);

    // Adjust for sign change, inversion, and shift
    if (shift) {
        res = res + FixedTrait::new(9658692610769497123, false); // pi / 6
    }

    if (invert) {
        res = res - FixedTrait::new(HALF_PI_u128, false);
    }

    return FixedTrait::new(res.mag, a.sign);
}

// Calculates cos(a) with a in radians (fixed point)
fn cos(a: Fixed) -> Fixed {
    return sin(FixedTrait::new(HALF_PI_u128, false) - a);
}

fn cos_fast(a: Fixed) -> Fixed {
    return sin_fast(FixedTrait::new(HALF_PI_u128, false) - a);
}

fn sin(a: Fixed) -> Fixed {
    let a1_u128 = a.mag % (2 * PI_u128);
    let (whole_rem, partial_rem) = u128_safe_divmod(a1_u128, u128_as_non_zero(PI_u128));
    let a2 = FixedTrait::new(partial_rem, false);
    let partial_sign = whole_rem == 1;

    let loop_res = a2 * _sin_loop(a2, 7, FixedTrait::ONE());
    return FixedTrait::new(loop_res.mag, a.sign ^ partial_sign && loop_res.mag != 0);
}

fn sin_fast(a: Fixed) -> Fixed {
    let a1_u128 = a.mag % (2 * PI_u128);
    let (whole_rem, mut partial_rem) = u128_safe_divmod(a1_u128, u128_as_non_zero(PI_u128));
    let partial_sign = whole_rem == 1;

    if partial_rem >= HALF_PI_u128 {
        partial_rem = PI_u128 - partial_rem;
    }

    let (start, low, high) = lut::sin(partial_rem);
    let partial_step = (FixedTrait::new(partial_rem, false) - FixedTrait::new(start, false))
        / FixedTrait::new(113187804032455040, false);
    let res = partial_step * (FixedTrait::new(high, false) - FixedTrait::new(low, false))
        + FixedTrait::new(low, false);

    return FixedTrait::new(res.mag, a.sign ^ partial_sign && res.mag != 0);
}

// Calculates tan(a) with a in radians (fixed point)
fn tan(a: Fixed) -> Fixed {
    let sinx = sin(a);
    let cosx = cos(a);
    assert(cosx.mag != 0, 'tan undefined');
    return sinx / cosx;
}

fn tan_fast(a: Fixed) -> Fixed {
    let sinx = sin_fast(a);
    let cosx = cos_fast(a);
    assert(cosx.mag != 0, 'tan undefined');
    return sinx / cosx;
}

// Helper function to calculate Taylor series for sin
fn _sin_loop(a: Fixed, i: u128, acc: Fixed) -> Fixed {
    let div_u128 = (2 * i + 2) * (2 * i + 3);
    let term = a * a * acc / FixedTrait::new_unscaled(div_u128, false);
    let new_acc = FixedTrait::ONE() - term;

    if (i == 0) {
        return new_acc;
    }

    return _sin_loop(a, i - 1, new_acc);
}

