//! Black-box helpers: keep inputs opaque to the optimizer and force results to be materialised.

/// Opaque identity. `#[inline(never)]` prevents the compiler from seeing the constant behind it,
/// so the benchmarked operation cannot be constant-folded.
#[inline(never)]
pub fn bb<T>(x: T) -> T {
    x
}

/// Opaque consumer: forces the computed value to exist.
#[inline(never)]
pub fn sink<T, +Drop<T>>(x: T) {}
