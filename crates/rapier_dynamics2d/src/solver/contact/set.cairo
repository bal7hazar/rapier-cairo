//! Ordered manifold sweeps; no island discovery or body integration.
use fixed::Fixed;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_geometry2d::contact::ContactManifold;
use super::super::body::SolverBody;
use super::{ContactConstraint, ContactConstraintTrait, errors, writeback};

/// Constraints in the original span order, including inert entries for empty/disabled
/// manifolds so ids remain stable. Arrays are rebuilt on each mutating sweep in Cairo.
#[derive(Drop, Default)]
pub struct ContactConstraintsSet {
    pub constraints: Array<ContactConstraint>,
}

#[generate_trait]
pub impl ContactConstraintsSetImpl of ContactConstraintsSetTrait {
    /// Generate in span order (caller supplies ascending pair-slot order). Full-generation
    /// handle checks, numeric requirements, rounding and panics match `ContactConstraint`.
    fn generate(
        mut manifolds: Span<ContactManifold>,
        bodies: Span<SolverBody>,
        params: IntegrationParameters,
        dt: Fixed,
    ) -> ContactConstraintsSet {
        let mut constraints = array![];
        while let Some(m) = manifolds.pop_front() {
            let mut c = ContactConstraintTrait::generate(*m, bodies, params, dt);
            c.manifold_id = constraints.len();
            constraints.append(c);
        }
        ContactConstraintsSet { constraints }
    }
    /// Refresh every constraint before warm starting. `manifolds` must retain generation's
    /// order/count. Numeric requirements and panics match the per-manifold `update`.
    fn update(
        ref self: ContactConstraintsSet,
        params: IntegrationParameters,
        bodies: Span<SolverBody>,
        manifolds: Span<ContactManifold>,
    ) {
        assert(self.constraints.len() == manifolds.len(), errors::COUNT);
        let mut out = array![];
        while let Some(mut c) = self.constraints.pop_front() {
            c.update(params, bodies, *manifolds.at(c.manifold_id));
            out.append(c);
        }
        self.constraints = out;
    }
    /// Apply warm starts in manifold order; rounds/panics as the per-manifold operation.
    fn warmstart(self: @ContactConstraintsSet, ref bodies: Array<SolverBody>) {
        let mut cs = self.constraints.span();
        while let Some(c) = cs.pop_front() {
            (*c).warmstart(ref bodies);
        }
    }
    /// Gauss-Seidel sweep, observing earlier manifolds' velocity writes. Flags match upstream
    /// normal/friction switches; division-free row solve, checked fixed arithmetic.
    fn solve(
        ref self: ContactConstraintsSet,
        ref bodies: Array<SolverBody>,
        solve_restitution: bool,
        solve_friction: bool,
    ) {
        let mut out = array![];
        while let Some(mut c) = self.constraints.pop_front() {
            c.solve(ref bodies, solve_restitution, solve_friction);
            out.append(c);
        }
        self.constraints = out;
    }
    /// Strip cached biases/softness without re-evaluating moved poses; exact copies.
    fn remove_bias(ref self: ContactConstraintsSet) {
        let mut out = array![];
        while let Some(mut c) = self.constraints.pop_front() {
            c.remove_bias();
            out.append(c);
        }
        self.constraints = out;
    }
    /// Refresh all speculative rhs values after integration; same rounding/range/panics as
    /// the per-manifold operation. Does not bank or rescale impulses.
    fn update_rhs_wo_bias(ref self: ContactConstraintsSet, bodies: Span<SolverBody>) {
        let mut out = array![];
        while let Some(mut c) = self.constraints.pop_front() {
            c.update_rhs_wo_bias(bodies);
            out.append(c);
        }
        self.constraints = out;
    }
    /// End-of-step bounce pass in manifold order; call once, after all substeps.
    /// Eligibility, rounding and panics match per-manifold `apply_restitution`.
    fn apply_restitution(ref self: ContactConstraintsSet, ref bodies: Array<SolverBody>) {
        let mut out = array![];
        while let Some(mut c) = self.constraints.pop_front() {
            c.apply_restitution(ref bodies);
            out.append(c);
        }
        self.constraints = out;
    }
    /// Persist totals and warm starts with one ordered array rebuild. Manifolds retain their
    /// original order/count. Count mismatch panics with `Contact: invalid count`.
    fn writeback_impulses(self: @ContactConstraintsSet, ref manifolds: Array<ContactManifold>) {
        assert(self.constraints.len() == manifolds.len(), errors::COUNT);
        let mut out = array![];
        let mut cs = self.constraints.span();
        while let Some(mut m) = manifolds.pop_front() {
            let c = *cs.pop_front().unwrap();
            if c.num_elements != 0 {
                writeback(c, ref m);
            }
            out.append(m);
        }
        manifolds = out;
    }
}

#[cfg(test)]
mod tests {
    use fixed::{ONE, ZERO};
    use rapier_core::integration_parameters::IntegrationParametersTrait;
    use rapier_testing::opaque;
    use super::super::ContactConstraintTrait;
    use super::super::fixtures::{fixture, prepared};
    use super::{ContactConstraintsSet, ContactConstraintsSetTrait};

    #[test]
    fn test_set_keeps_empty_slots_and_matches_manual_sweeps() {
        let (m, bs, p) = fixture(2);
        let empty = Default::default();
        let mut ms = array![empty, m, empty];
        let mut cs = ContactConstraintsSetTrait::generate(ms.span(), bs.span(), p, p.substep_dt());
        assert_eq!(cs.constraints.len(), 3);
        assert_eq!(*cs.constraints.at(1).manifold_id, 1);
        cs.update(p, bs.span(), ms.span());
        let mut direct = *cs.constraints.at(1);
        let mut ds = array![*bs.at(0)];
        let mut bs = bs;
        cs.warmstart(ref bs);
        direct.warmstart(ref ds);
        cs.solve(ref bs, true, true);
        direct.solve(ref ds, true, true);
        cs.update_rhs_wo_bias(bs.span());
        direct.update_rhs_wo_bias(ds.span());
        cs.solve(ref bs, true, true);
        direct.solve(ref ds, true, true);
        cs.apply_restitution(ref bs);
        direct.apply_restitution(ref ds);
        assert_eq!(*bs.at(0), *ds.at(0));
        assert_eq!(*cs.constraints.at(1), direct);
        cs.writeback_impulses(ref ms);
        assert_eq!(*ms.at(0), empty);
        assert_eq!(*ms.at(2), empty);
        cs.remove_bias();
        assert_eq!(*cs.constraints.at(1).cfm_factor, ONE);
    }
    #[test]
    fn test_empty_set() {
        let mut cs: ContactConstraintsSet = Default::default();
        let mut bs = array![];
        let mut ms = array![];
        cs.update(Default::default(), bs.span(), ms.span());
        cs.warmstart(ref bs);
        cs.solve(ref bs, true, true);
        cs.remove_bias();
        cs.update_rhs_wo_bias(bs.span());
        cs.apply_restitution(ref bs);
        cs.writeback_impulses(ref ms);
        assert_eq!(cs.constraints.len(), 0);
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }
    #[test]
    fn gas_generate() {
        let (m, bs, p) = fixture(1);
        let cs = ContactConstraintsSetTrait::generate(
            array![opaque(m)].span(), bs.span(), p, opaque(p.substep_dt()),
        );
        assert_eq!(cs.constraints.len(), 1);
    }
    // One operation selector keeps setup and output identical across these public API probes.
    fn probe(operation: u8) {
        let (c, mut bs) = prepared(1);
        let (m, _, p) = fixture(1);
        let mut cs = ContactConstraintsSet { constraints: array![opaque(c)] };
        let mut ms = array![opaque(m)];
        match operation {
            0 => cs.update(opaque(p), bs.span(), ms.span()),
            1 => cs.warmstart(ref bs),
            2 => cs.solve(ref bs, true, true),
            3 => cs.remove_bias(),
            4 => cs.update_rhs_wo_bias(bs.span()),
            5 => cs.apply_restitution(ref bs),
            _ => cs.writeback_impulses(ref ms),
        }
        let _ = opaque((*cs.constraints.at(0), *bs.at(0), *ms.at(0), ZERO));
    }
    #[test]
    fn gas_update() {
        probe(0);
    }
    #[test]
    fn gas_warmstart() {
        probe(1);
    }
    #[test]
    fn gas_solve() {
        probe(2);
    }
    #[test]
    fn gas_remove_bias() {
        probe(3);
    }
    #[test]
    fn gas_update_rhs_wo_bias() {
        probe(4);
    }
    #[test]
    fn gas_apply_restitution() {
        probe(5);
    }
    #[test]
    fn gas_writeback_impulses() {
        probe(6);
    }
}
