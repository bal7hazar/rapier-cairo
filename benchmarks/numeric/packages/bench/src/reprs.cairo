//! Section B: hand-written 32.32 / 64.64 signed fixed-point representations.
pub mod sm64; // 1. sign-magnitude {mag: u64, sign: bool}
pub mod i64n; // 2a. native i64, i128 widening, core signed Div
pub mod i64b; // 2b. native i64 storage, BoundedInt arithmetic for mul/div rescale
pub mod felt; // 3. felt252-backed, lazy range checks
pub mod sm128; // 4. sign-magnitude {mag: u128, sign: bool}, 64.64
pub mod cubit_fx; // adapter: cubit f64 / f128 in the generic kernels

/// Non-operator functions every representation provides.
pub trait Fx<T> {
    fn sqrt(self: T) -> T;
}
