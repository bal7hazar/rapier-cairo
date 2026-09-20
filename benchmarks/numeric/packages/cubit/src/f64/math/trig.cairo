use core::debug::PrintTrait;
use core::integer::{u64_safe_divmod, u64_as_non_zero};
use core::option::OptionTrait;

use cubit::f64::math::lut;
use cubit::f64::types::fixed::{Fixed, FixedTrait, FixedAdd, FixedSub, FixedMul, FixedDiv, ONE};

// CONSTANTS

const TWO_PI: u64 = 26986075409;
const PI: u64 = 13493037705;
const HALF_PI: u64 = 6746518852;

// PUBLIC

// Calculates arccos(a) for -1 <= a <= 1 (fixed point)
// arccos(a) = arcsin(sqrt(1 - a^2)) - arctan identity has discontinuity at zero
fn acos(a: Fixed) -> Fixed {
    let asin_arg = (FixedTrait::ONE() - a * a).sqrt(); // will fail if a > 1
    let asin_res = asin(asin_arg);

    if (a.sign) {
        return FixedTrait::new(PI, false) - asin_res;
    } else {
        return asin_res;
    }
}

fn acos_fast(a: Fixed) -> Fixed {
    let asin_arg = (FixedTrait::ONE() - a * a).sqrt(); // will fail if a > 1
    let asin_res = asin_fast(asin_arg);

    if (a.sign) {
        return FixedTrait::new(PI, false) - asin_res;
    } else {
        return asin_res;
    }
}

// Calculates arcsin(a) for -1 <= a <= 1 (fixed point)
// arcsin(a) = arctan(a / sqrt(1 - a^2))
fn asin(a: Fixed) -> Fixed {
    if (a.mag == ONE) {
        return FixedTrait::new(HALF_PI, a.sign);
    }

    let div = (FixedTrait::ONE() - a * a).sqrt(); // will fail if a > 1
    return atan(a / div);
}

fn asin_fast(a: Fixed) -> Fixed {
    if (a.mag == ONE) {
        return FixedTrait::new(HALF_PI, a.sign);
    }

    let div = (FixedTrait::ONE() - a * a).sqrt(); // will fail if a > 1
    return atan_fast(a / div);
}

// Calculates arctan(a) (fixed point)
// See https://stackoverflow.com/a/50894477 for range adjustments
fn atan(a: Fixed) -> Fixed {
    let mut at = a.abs();
    let mut shift = false;
    let mut invert = false;

    // Invert value when a > 1
    if (at.mag > ONE) {
        at = FixedTrait::ONE() / at;
        invert = true;
    }

    // Account for lack of precision in polynomaial when a > 0.7
    if (at.mag > 3006477107) {
        let sqrt3_3 = FixedTrait::new(2479700525, false); // sqrt(3) / 3
        at = (at - sqrt3_3) / (FixedTrait::ONE() + at * sqrt3_3);
        shift = true;
    }

    let r10 = FixedTrait::new(7866091, true) * at;
    let r9 = (r10 + FixedTrait::new(200950905, true)) * at;
    let r8 = (r9 + FixedTrait::new(834081193, false)) * at;
    let r7 = (r8 + FixedTrait::new(1125283850, true)) * at;
    let r6 = (r7 + FixedTrait::new(187746747, false)) * at;
    let r5 = (r6 + FixedTrait::new(816293925, false)) * at;
    let r4 = (r5 + FixedTrait::new(5897657, false)) * at;
    let r3 = (r4 + FixedTrait::new(1432117161, true)) * at;
    let r2 = (r3 + FixedTrait::new(17657, false)) * at;
    let mut res = (r2 + FixedTrait::new(4294967059, false)) * at;

    // Adjust for sign change, inversion, and shift
    if (shift) {
        res = res + FixedTrait::new(2248839617, false); // pi / 6
    }

    if (invert) {
        res = res - FixedTrait::new(HALF_PI, false);
    }

    return FixedTrait::new(res.mag, a.sign);
}

fn atan_fast(a: Fixed) -> Fixed {
    let mut at = a.abs();
    let mut shift = false;
    let mut invert = false;

    // Invert value when a > 1
    if (at.mag > ONE) {
        at = FixedTrait::ONE() / at;
        invert = true;
    }

    // Account for lack of precision in polynomaial when a > 0.7
    if (at.mag > 3006477107) {
        let sqrt3_3 = FixedTrait::new(2479700525, false); // sqrt(3) / 3
        at = (at - sqrt3_3) / (FixedTrait::ONE() + at * sqrt3_3);
        shift = true;
    }

    let (start, low, high) = lut::atan(at.mag);
    let partial_step = FixedTrait::new(at.mag - start, false) / FixedTrait::new(30064771, false);
    let mut res = partial_step * FixedTrait::new(high - low, false) + FixedTrait::new(low, false);

    // Adjust for sign change, inversion, and shift
    if (shift) {
        res = res + FixedTrait::new(2248839617, false); // pi / 6
    }

    if (invert) {
        res = res - FixedTrait::new(HALF_PI, false);
    }

    return FixedTrait::new(res.mag, a.sign);
}

// Calculates cos(a) with a in radians (fixed point)
fn cos(a: Fixed) -> Fixed {
    return sin(FixedTrait::new(HALF_PI, false) - a);
}

fn cos_fast(a: Fixed) -> Fixed {
    return sin_fast(FixedTrait::new(HALF_PI, false) - a);
}

fn sin(a: Fixed) -> Fixed {
    let a1 = a.mag % TWO_PI;
    let (whole_rem, partial_rem) = u64_safe_divmod(a1, u64_as_non_zero(PI));
    let a2 = FixedTrait::new(partial_rem, false);
    let partial_sign = whole_rem == 1;

    let loop_res = a2 * _sin_loop(a2, 7, FixedTrait::ONE());
    return FixedTrait::new(loop_res.mag, a.sign ^ partial_sign && loop_res.mag != 0);
}

fn sin_fast(a: Fixed) -> Fixed {
    let a1 = a.mag % TWO_PI;
    let (whole_rem, mut partial_rem) = u64_safe_divmod(a1, u64_as_non_zero(PI));
    let partial_sign = whole_rem == 1;

    if partial_rem >= HALF_PI {
        partial_rem = PI - partial_rem;
    }

    let (start, low, high) = lut::sin(partial_rem);
    let partial_step = (FixedTrait::new(partial_rem, false) - FixedTrait::new(start, false))
        / FixedTrait::new(26353589, false);
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
fn _sin_loop(a: Fixed, i: u64, acc: Fixed) -> Fixed {
    let div = (2 * i + 2) * (2 * i + 3);
    let term = a * a * acc / FixedTrait::new_unscaled(div, false);
    let new_acc = FixedTrait::ONE() - term;

    if (i == 0) {
        return new_acc;
    }

    return _sin_loop(a, i - 1, new_acc);
}

