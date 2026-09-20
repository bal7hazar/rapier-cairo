use core::debug::PrintTrait;

use orion::numbers::fixed_point::implementations::fp8x23::core::{
    HALF, ONE, TWO, FP8x23, FP8x23Impl, FP8x23Add, FP8x23AddEq, FP8x23Sub, FP8x23Mul, FP8x23MulEq,
    FP8x23TryIntoU128, FP8x23PartialEq, FP8x23PartialOrd, FP8x23SubEq, FP8x23Neg, FP8x23Div,
    FP8x23IntoFelt252, FixedTrait
};

// Calculates hyperbolic cosine of a (fixed point)
fn cosh(a: FP8x23) -> FP8x23 {
    let ea = a.exp();

    (ea + (FixedTrait::ONE() / ea)) / FixedTrait::new(TWO, false)
}

// Calculates hyperbolic sine of a (fixed point)
fn sinh(a: FP8x23) -> FP8x23 {
    let ea = a.exp();

    (ea - (FixedTrait::ONE() / ea)) / FixedTrait::new(TWO, false)
}

// Calculates hyperbolic tangent of a (fixed point)
fn tanh(a: FP8x23) -> FP8x23 {
    let ea = a.exp();
    let ea_i = FixedTrait::ONE() / ea;

    (ea - ea_i) / (ea + ea_i)
}

// Calculates inverse hyperbolic cosine of a (fixed point)
fn acosh(a: FP8x23) -> FP8x23 {
    let root = (a * a - FixedTrait::ONE()).sqrt();

    (a + root).ln()
}

// Calculates inverse hyperbolic sine of a (fixed point)
fn asinh(a: FP8x23) -> FP8x23 {
    let root = (a * a + FixedTrait::ONE()).sqrt();

    (a + root).ln()
}

// Calculates inverse hyperbolic tangent of a (fixed point)
fn atanh(a: FP8x23) -> FP8x23 {
    let one = FixedTrait::ONE();
    let ln_arg = (one + a) / (one - a);

    ln_arg.ln() / FixedTrait::new(TWO, false)
}

