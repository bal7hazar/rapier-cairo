use cubit::f64::types::fixed::{Fixed, FixedTrait, HALF, ONE, TWO};
use core::debug::PrintTrait;

// Calculates hyperbolic cosine of a (fixed point)
fn cosh(a: Fixed) -> Fixed {
    let ea = a.exp();
    return (ea + (FixedTrait::ONE() / ea)) / FixedTrait::new(TWO, false);
}

// Calculates hyperbolic sine of a (fixed point)
fn sinh(a: Fixed) -> Fixed {
    let ea = a.exp();
    return (ea - (FixedTrait::ONE() / ea)) / FixedTrait::new(TWO, false);
}

// Calculates hyperbolic tangent of a (fixed point)
fn tanh(a: Fixed) -> Fixed {
    let ea = a.exp();
    let ea_i = FixedTrait::ONE() / ea;
    return (ea - ea_i) / (ea + ea_i);
}

// Calculates inverse hyperbolic cosine of a (fixed point)
fn acosh(a: Fixed) -> Fixed {
    let root = (a * a - FixedTrait::ONE()).sqrt();
    return (a + root).ln();
}

// Calculates inverse hyperbolic sine of a (fixed point)
fn asinh(a: Fixed) -> Fixed {
    let root = (a * a + FixedTrait::ONE()).sqrt();
    return (a + root).ln();
}

// Calculates inverse hyperbolic tangent of a (fixed point)
fn atanh(a: Fixed) -> Fixed {
    let one = FixedTrait::ONE();
    let ln_arg = (one + a) / (one - a);
    return ln_arg.ln() / FixedTrait::new(TWO, false);
}

