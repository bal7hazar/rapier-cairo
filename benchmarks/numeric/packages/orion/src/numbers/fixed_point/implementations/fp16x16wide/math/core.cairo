use core::integer;

use orion::numbers::fixed_point::implementations::fp16x16wide::core::{
    HALF, ONE, MAX, FP16x16W, FP16x16WImpl, FP16x16WAdd, FP16x16WAddEq, FP16x16WSub, FP16x16WMul,
    FP16x16WMulEq, FP16x16WTryIntoU128, FP16x16WPartialEq, FP16x16WPartialOrd, FP16x16WSubEq,
    FP16x16WNeg, FP16x16WDiv, FP16x16WIntoFelt252, FixedTrait
};
use orion::numbers::fixed_point::implementations::fp16x16wide::math::lut;

// PUBLIC
fn abs(a: FP16x16W) -> FP16x16W {
    FixedTrait::new(a.mag, false)
}

fn add(a: FP16x16W, b: FP16x16W) -> FP16x16W {
    if a.sign == b.sign {
        return FixedTrait::new(a.mag + b.mag, a.sign);
    }

    if a.mag == b.mag {
        return FixedTrait::ZERO();
    }

    if (a.mag > b.mag) {
        FixedTrait::new(a.mag - b.mag, a.sign)
    } else {
        FixedTrait::new(b.mag - a.mag, b.sign)
    }
}

fn ceil(a: FP16x16W) -> FP16x16W {
    let (div, rem) = integer::u64_safe_divmod(a.mag, integer::u64_as_non_zero(ONE));

    if rem == 0 {
        a
    } else if !a.sign {
        FixedTrait::new_unscaled(div + 1, false)
    } else if div == 0 {
        FixedTrait::new_unscaled(0, false)
    } else {
        FixedTrait::new_unscaled(div, true)
    }
}

fn div(a: FP16x16W, b: FP16x16W) -> FP16x16W {
    let a_u64 = integer::u64_wide_mul(a.mag, ONE);
    let res_u64 = a_u64 / b.mag.into();

    // Re-apply sign
    FixedTrait::new(res_u64.try_into().unwrap(), a.sign ^ b.sign)
}

fn eq(a: @FP16x16W, b: @FP16x16W) -> bool {
    (*a.mag == *b.mag) && (*a.sign == *b.sign)
}

// Calculates the natural exponent of x: e^x
fn exp(a: FP16x16W) -> FP16x16W {
    exp2(FixedTrait::new(94548, false) * a) // log2(e) * 2^23 ≈ 12102203
}

// Calculates the binary exponent of x: 2^x
fn exp2(a: FP16x16W) -> FP16x16W {
    if (a.mag == 0) {
        return FixedTrait::ONE();
    }

    let (int_part, frac_part) = integer::u64_safe_divmod(a.mag, integer::u64_as_non_zero(ONE));
    let int_res = FixedTrait::new_unscaled(lut::exp2(int_part), false);
    let mut res_u = int_res;

    if frac_part != 0 {
        let frac = FixedTrait::new(frac_part, false);
        let r7 = FixedTrait::new(1, false) * frac;
        let r6 = (r7 + FixedTrait::new(10, false)) * frac;
        let r5 = (r6 + FixedTrait::new(87, false)) * frac;
        let r4 = (r5 + FixedTrait::new(630, false)) * frac;
        let r3 = (r4 + FixedTrait::new(3638, false)) * frac;
        let r2 = (r3 + FixedTrait::new(15743, false)) * frac;
        let r1 = (r2 + FixedTrait::new(45426, false)) * frac;
        res_u = res_u * (r1 + FixedTrait::ONE());
    }

    if a.sign {
        FixedTrait::ONE() / res_u
    } else {
        res_u
    }
}

fn exp2_int(exp: u64) -> FP16x16W {
    FixedTrait::new_unscaled(lut::exp2(exp), false)
}

fn floor(a: FP16x16W) -> FP16x16W {
    let (div, rem) = integer::u64_safe_divmod(a.mag, integer::u64_as_non_zero(ONE));

    if rem == 0 {
        a
    } else if !a.sign {
        FixedTrait::new_unscaled(div, false)
    } else {
        FixedTrait::new_unscaled(div + 1, true)
    }
}

fn ge(a: FP16x16W, b: FP16x16W) -> bool {
    if a.sign != b.sign {
        !a.sign
    } else {
        (a.mag == b.mag) || ((a.mag > b.mag) ^ a.sign)
    }
}

fn gt(a: FP16x16W, b: FP16x16W) -> bool {
    if a.sign != b.sign {
        !a.sign
    } else {
        (a.mag != b.mag) && ((a.mag > b.mag) ^ a.sign)
    }
}

fn le(a: FP16x16W, b: FP16x16W) -> bool {
    if a.sign != b.sign {
        a.sign
    } else {
        (a.mag == b.mag) || ((a.mag < b.mag) ^ a.sign)
    }
}

// Calculates the natural logarithm of x: ln(x)
// self must be greater than zero
fn ln(a: FP16x16W) -> FP16x16W {
    FixedTrait::new(45426, false) * log2(a) // ln(2) = 0.693...
}

// Calculates the binary logarithm of x: log2(x)
// self must be greather than zero
fn log2(a: FP16x16W) -> FP16x16W {
    assert(a.sign == false, 'must be positive');

    if (a.mag == ONE) {
        return FixedTrait::ZERO();
    } else if (a.mag < ONE) {
        // Compute true inverse binary log if 0 < x < 1
        let div = FixedTrait::ONE() / a;
        return -log2(div);
    }

    let whole = a.mag / ONE;
    let (msb, div) = lut::msb(whole);

    if a.mag == div * ONE {
        FixedTrait::new_unscaled(msb, false)
    } else {
        let norm = a / FixedTrait::new_unscaled(div, false);
        let r8 = FixedTrait::new(596, true) * norm;
        let r7 = (r8 + FixedTrait::new(8116, false)) * norm;
        let r6 = (r7 + FixedTrait::new(49044, true)) * norm;
        let r5 = (r6 + FixedTrait::new(172935, false)) * norm;
        let r4 = (r5 + FixedTrait::new(394096, true)) * norm;
        let r3 = (r4 + FixedTrait::new(608566, false)) * norm;
        let r2 = (r3 + FixedTrait::new(655828, true)) * norm;
        let r1 = (r2 + FixedTrait::new(534433, false)) * norm;

        r1 + FixedTrait::new(224487, true) + FixedTrait::new_unscaled(msb, false)
    }
}

// Calculates the base 10 log of x: log10(x)
// self must be greater than zero
fn log10(a: FP16x16W) -> FP16x16W {
    FixedTrait::new(19728, false) * log2(a) // log10(2) = 0.301...
}

fn lt(a: FP16x16W, b: FP16x16W) -> bool {
    if a.sign != b.sign {
        a.sign
    } else {
        (a.mag != b.mag) && ((a.mag < b.mag) ^ a.sign)
    }
}

fn mul(a: FP16x16W, b: FP16x16W) -> FP16x16W {
    let prod_u128 = integer::u64_wide_mul(a.mag, b.mag);

    // Re-apply sign
    FixedTrait::new((prod_u128 / ONE.into()).try_into().unwrap(), a.sign ^ b.sign)
}

fn ne(a: @FP16x16W, b: @FP16x16W) -> bool {
    (*a.mag != *b.mag) || (*a.sign != *b.sign)
}

fn neg(a: FP16x16W) -> FP16x16W {
    if a.mag == 0 {
        a
    } else if !a.sign {
        FixedTrait::new(a.mag, !a.sign)
    } else {
        FixedTrait::new(a.mag, false)
    }
}

// Calclates the value of x^y and checks for overflow before returning
// self is a FP16x16W point value
// b is a FP16x16W point value
fn pow(a: FP16x16W, b: FP16x16W) -> FP16x16W {
    let (_, rem) = integer::u64_safe_divmod(b.mag, integer::u64_as_non_zero(ONE));

    // use the more performant integer pow when y is an int
    if (rem == 0) {
        return pow_int(a, b.mag / ONE, b.sign);
    }

    // x^y = exp(y*ln(x)) for x > 0 will error for x < 0
    exp(b * ln(a))
}

// Calclates the value of a^b and checks for overflow before returning
fn pow_int(a: FP16x16W, b: u64, sign: bool) -> FP16x16W {
    let mut x = a;
    let mut n = b;

    if sign {
        x = FixedTrait::ONE() / x;
    }

    if n == 0 {
        return FixedTrait::ONE();
    }

    let mut y = FixedTrait::ONE();
    let two = integer::u64_as_non_zero(2);

    while n > 1 {
        let (div, rem) = integer::u64_safe_divmod(n, two);

        if rem == 1 {
            y = x * y;
        }

        x = x * x;
        n = div;
    };

    x * y
}

fn rem(a: FP16x16W, b: FP16x16W) -> FP16x16W {
    a - floor(a / b) * b
}

fn round(a: FP16x16W) -> FP16x16W {
    let (div, rem) = integer::u64_safe_divmod(a.mag, integer::u64_as_non_zero(ONE));

    if (HALF <= rem) {
        FixedTrait::new_unscaled(div + 1, a.sign)
    } else {
        FixedTrait::new_unscaled(div, a.sign)
    }
}

// Calculates the square root of a FP16x16W point value
// x must be positive
fn sqrt(a: FP16x16W) -> FP16x16W {
    assert(a.sign == false, 'must be positive');

    let root = integer::u64_sqrt(a.mag.into() * ONE.into());

    FixedTrait::new(root.into(), false)
}

fn sub(a: FP16x16W, b: FP16x16W) -> FP16x16W {
    add(a, -b)
}

fn sign(a: FP16x16W) -> FP16x16W {
    if a.mag == 0 {
        FixedTrait::new(0, false)
    } else {
        FixedTrait::new(ONE, a.sign)
    }
}

