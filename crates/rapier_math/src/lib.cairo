//! Physics-side maths for rapier.cairo, on top of the shared `fixed` scalar from glam.cairo.
//!
//! This crate plays the role of upstream's `glamx`: `Rot2`, `Pose2`, the fused kernels the engine
//! needs and the scalar helpers Rapier/Parry rely on (`inv(0) = 0`, wide comparisons of squared
//! quantities, angular constants). See `docs/PLAN.md` (D12) for the split with glam.cairo.

#[cfg(test)]
mod tests {
    use fixed::wide::dot2;
    use fixed::{Fixed, FixedTrait, ONE};
    use rapier_testing::opaque;

    /// Empty probe: harness overhead to subtract from other entries of `.gas-snapshot`.
    #[test]
    fn gas_baseline() {}

    /// Confirms the shared scalar links and behaves as documented: `dot2` rescales once and
    /// `sqrt` is exact on a perfect square.
    #[test]
    fn gas_fixed_dot2_sqrt() {
        let a: Fixed = FixedTrait::from_int(opaque(3));
        let b: Fixed = FixedTrait::from_int(opaque(4));
        assert_eq!(dot2(a, a, b, b).sqrt(), FixedTrait::from_int(5));
        assert_eq!(ONE * ONE, ONE);
    }
}
