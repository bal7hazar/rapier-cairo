//! Shared helpers for tests and gas probes across the rapier-cairo workspace.

/// Returns its argument through a call the compiler cannot inline.
///
/// Gas probes must route their inputs through this function, otherwise constant
/// folding can erase the very computation the probe is meant to measure.
#[inline(never)]
pub fn opaque<T>(value: T) -> T {
    value
}

#[cfg(test)]
mod tests {
    use super::opaque;

    /// Empty probe: the fixed overhead snforge charges to any test. Subtract it from
    /// other entries of `.gas-snapshot` to obtain the net cost of an operation.
    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_opaque_u64() {
        assert_eq!(opaque(42_u64), 42);
    }
}
