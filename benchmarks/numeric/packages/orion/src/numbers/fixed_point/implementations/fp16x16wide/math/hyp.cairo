use orion::numbers::fixed_point::implementations::fp16x16wide::core::{
    HALF, ONE, TWO, FP16x16W, FP16x16WImpl, FP16x16WAdd, FP16x16WAddEq, FP16x16WSub, FP16x16WMul,
    FP16x16WMulEq, FP16x16WTryIntoU128, FP16x16WPartialEq, FP16x16WPartialOrd, FP16x16WSubEq,
    FP16x16WNeg, FP16x16WDiv, FP16x16WIntoFelt252, FixedTrait
};

// Calculates hyperbolic cosine of a (fixed point)
fn cosh(a: FP16x16W) -> FP16x16W {
    let ea = a.exp();

    (ea + (FixedTrait::ONE() / ea)) / FixedTrait::new(TWO, false)
}

// Calculates hyperbolic sine of a (fixed point)
fn sinh(a: FP16x16W) -> FP16x16W {
    let ea = a.exp();

    (ea - (FixedTrait::ONE() / ea)) / FixedTrait::new(TWO, false)
}

// Calculates hyperbolic tangent of a (fixed point)
fn tanh(a: FP16x16W) -> FP16x16W {
    let ea = a.exp();
    let ea_i = FixedTrait::ONE() / ea;

    (ea - ea_i) / (ea + ea_i)
}

// Calculates inverse hyperbolic cosine of a (fixed point)
fn acosh(a: FP16x16W) -> FP16x16W {
    let root = (a * a - FixedTrait::ONE()).sqrt();

    (a + root).ln()
}

// Calculates inverse hyperbolic sine of a (fixed point)
fn asinh(a: FP16x16W) -> FP16x16W {
    let root = (a * a + FixedTrait::ONE()).sqrt();

    (a + root).ln()
}

// Calculates inverse hyperbolic tangent of a (fixed point)
fn atanh(a: FP16x16W) -> FP16x16W {
    let one = FixedTrait::ONE();
    let ln_arg = (one + a) / (one - a);

    ln_arg.ln() / FixedTrait::new(TWO, false)
}

