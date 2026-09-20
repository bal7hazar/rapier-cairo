use orion::numbers::fixed_point::implementations::fp8x23wide::core::{
    HALF, ONE, TWO, FP8x23W, FP8x23WImpl, FP8x23WAdd, FP8x23WAddEq, FP8x23WSub, FP8x23WMul,
    FP8x23WMulEq, FP8x23WTryIntoU128, FP8x23WPartialEq, FP8x23WPartialOrd, FP8x23WSubEq, FP8x23WNeg,
    FP8x23WDiv, FP8x23WIntoFelt252, FixedTrait
};

// Calculates hyperbolic cosine of a (fixed point)
fn cosh(a: FP8x23W) -> FP8x23W {
    let ea = a.exp();

    (ea + (FixedTrait::ONE() / ea)) / FixedTrait::new(TWO, false)
}

// Calculates hyperbolic sine of a (fixed point)
fn sinh(a: FP8x23W) -> FP8x23W {
    let ea = a.exp();

    (ea - (FixedTrait::ONE() / ea)) / FixedTrait::new(TWO, false)
}

// Calculates hyperbolic tangent of a (fixed point)
fn tanh(a: FP8x23W) -> FP8x23W {
    let ea = a.exp();
    let ea_i = FixedTrait::ONE() / ea;

    (ea - ea_i) / (ea + ea_i)
}

// Calculates inverse hyperbolic cosine of a (fixed point)
fn acosh(a: FP8x23W) -> FP8x23W {
    let root = (a * a - FixedTrait::ONE()).sqrt();

    (a + root).ln()
}

// Calculates inverse hyperbolic sine of a (fixed point)
fn asinh(a: FP8x23W) -> FP8x23W {
    let root = (a * a + FixedTrait::ONE()).sqrt();

    (a + root).ln()
}

// Calculates inverse hyperbolic tangent of a (fixed point)
fn atanh(a: FP8x23W) -> FP8x23W {
    let one = FixedTrait::ONE();
    let ln_arg = (one + a) / (one - a);

    ln_arg.ln() / FixedTrait::new(TWO, false)
}

