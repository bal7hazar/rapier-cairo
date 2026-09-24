//! The same substep order when all generated contacts are inert; no contact sweeps.
use super::*;
use super::sweeps::JointBuilder;

/// Whether every generated constraint has zero elements. Generation validates manifolds first;
/// these entries neither update bodies nor write impulses back into their manifolds.
pub(crate) fn all_inert(mut cs: Span<super::super::contact::ContactConstraint>) -> bool {
    while let Some(c) = cs.pop_front() {
        if *c.num_elements != 0 {
            return false;
        }
    }
    true
}

/// Execute the force/joint/integration/damping stages for an empty contact set.
/// Arguments are validated/prepared by the parent driver; arithmetic and panics are unchanged.
pub(crate) fn run<B, +DenseBodiesTrait<B>, +Destruct<B>>(
    p: IntegrationParameters,
    ref bodies: B,
    steps: Span<BodyStep>,
    builders: Span<JointBuilder>,
    ref joint_set: Array<ImpulseJoint>,
    dt: Fixed,
    max_lin: Fixed,
    max_ang: Fixed,
) {
    let mut rows = array![];
    let mut substep = 0;
    while substep != p.num_solver_iterations {
        add_forces(ref bodies, steps);
        rows = rebuild_joints(ref bodies, builders, rows.span(), p, substep != 0);
        let mut i = 0;
        while i != p.num_internal_pgs_iterations {
            joints(ref rows, ref bodies, true, p.warmstart_joints && i == 0);
            i += 1;
        }
        integrate(ref bodies, steps, dt, max_lin, max_ang);
        let mut i = 0;
        while i != p.num_internal_stabilization_iterations {
            joints(ref rows, ref bodies, false, false);
            i += 1;
        }
        substep += 1;
    }
    sweeps::write_joints(rows.span(), ref joint_set);
    damp(ref bodies, steps, p.dt);
}

#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use crate::solver::contact::ContactConstraint;
    use super::*;

    #[test]
    fn gas_baseline() {
        let _ = opaque(fixed::ONE);
    }

    #[test]
    fn gas_all_inert() {
        let c: ContactConstraint = Default::default();
        assert!(all_inert([opaque(c), opaque(c)].span()));
    }

    #[test]
    fn gas_first_active() {
        let mut c: ContactConstraint = Default::default();
        c.num_elements = opaque(1);
        assert!(!all_inert([opaque(c), opaque(Default::default())].span()));
    }
}
