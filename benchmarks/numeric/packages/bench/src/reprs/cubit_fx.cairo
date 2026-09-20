//! Adapter so the generic vector kernels can run on the unmodified cubit types.
use cubit::f128::{Fixed as F128, FixedTrait as F128Trait};
use cubit::f64::{Fixed as F64, FixedTrait as F64Trait};
use super::Fx;

pub impl CubitF64Fx of Fx<F64> {
    fn sqrt(self: F64) -> F64 {
        F64Trait::sqrt(self)
    }
}
pub impl CubitF128Fx of Fx<F128> {
    fn sqrt(self: F128) -> F128 {
        F128Trait::sqrt(self)
    }
}
