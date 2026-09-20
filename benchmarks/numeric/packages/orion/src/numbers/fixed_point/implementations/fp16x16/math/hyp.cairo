use orion::numbers::fixed_point::implementations::fp16x16::core::{
    HALF, ONE, TWO, FP16x16, FP16x16Impl, FP16x16Add, FP16x16AddEq, FP16x16Sub, FP16x16Mul,
    FP16x16MulEq, FP16x16TryIntoU128, FP16x16PartialEq, FP16x16PartialOrd, FP16x16SubEq, FP16x16Neg,
    FP16x16Div, FP16x16IntoFelt252, FixedTrait
};

// Calculates hyperbolic cosine of a (fixed point)
fn cosh(a: FP16x16) -> FP16x16 {
    let ea = a.exp();

    (ea + (FixedTrait::ONE() / ea)) / FixedTrait::new(TWO, false)
}

// Calculates hyperbolic sine of a (fixed point)
fn sinh(a: FP16x16) -> FP16x16 {
    let ea = a.exp();

    (ea - (FixedTrait::ONE() / ea)) / FixedTrait::new(TWO, false)
}

// Calculates hyperbolic tangent of a (fixed point)
fn tanh(a: FP16x16) -> FP16x16 {
    let ea = a.exp();
    let ea_i = FixedTrait::ONE() / ea;

    (ea - ea_i) / (ea + ea_i)
}

// Calculates inverse hyperbolic cosine of a (fixed point)
fn acosh(a: FP16x16) -> FP16x16 {
    let root = (a * a - FixedTrait::ONE()).sqrt();

    (a + root).ln()
}

// Calculates inverse hyperbolic sine of a (fixed point)
fn asinh(a: FP16x16) -> FP16x16 {
    let root = (a * a + FixedTrait::ONE()).sqrt();

    (a + root).ln()
}

// Calculates inverse hyperbolic tangent of a (fixed point)
fn atanh(a: FP16x16) -> FP16x16 {
    let one = FixedTrait::ONE();
    let ln_arg = (one + a) / (one - a);

    ln_arg.ln() / FixedTrait::new(TWO, false)
}

