//! Identical prepared two-slot input for both gather/scatter candidates. The baseline includes
//! preparation and output consumption; subtract it to isolate each one- or two-point sweep.
#[cfg(test)]
mod tests {
    use fixed::{ONE, ZERO};
    use rapier_core::integration_parameters::IntegrationParametersTrait;
    use rapier_testing::opaque;
    use super::super::fixtures::fixture;
    use super::super::super::body::SolverBody;
    use super::super::{ContactConstraint, ContactConstraintTrait, alternatives};

    fn input(count: u8) -> (ContactConstraint, Array<SolverBody>) {
        let (mut m, mut bs, p) = fixture(2);
        let mut other = *bs.at(0);
        other.handle.index = 19;
        other.linvel = Default::default();
        m.data.rigid_body1 = Some(other.handle);
        bs.append(other);
        let mut c = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
        c.update(p, bs.span(), m);
        c.num_elements = count;
        let b1 = opaque(bs.pop_front().unwrap());
        let b2 = opaque(bs.pop_front().unwrap());
        (opaque(c), array![b1, b2])
    }
    fn consume(c: ContactConstraint, bs: @Array<SolverBody>) {
        let _ = opaque((c, *bs.at(0)));
        if bs.len() == 2 {
            let _ = opaque(*bs.at(1));
        }
    }
    #[test]
    fn gas_baseline() {
        let (c, bs) = input(2);
        consume(c, @bs);
    }
    #[test]
    fn gas_solve_gathered_one() {
        let (mut c, mut bs) = input(1);
        c.solve(ref bs, true, true);
        consume(c, @bs);
    }
    #[test]
    fn gas_solve_gathered_two() {
        let (mut c, mut bs) = input(2);
        c.solve(ref bs, true, true);
        consume(c, @bs);
    }
    #[test]
    fn gas_solve_direct_one() {
        let (mut c, mut bs) = input(1);
        alternatives::solve(ref c, ref bs, true, true);
        consume(c, @bs);
    }
    #[test]
    fn gas_solve_direct_two() {
        let (mut c, mut bs) = input(2);
        alternatives::solve(ref c, ref bs, true, true);
        consume(c, @bs);
    }
    // One normal-only biased pass, bias removal, then one normal+friction relax pass.
    // Excludes force/position integration; both candidates use exactly the same stage order.
    fn substep(count: u8, direct: bool) {
        let (mut c, mut bs) = input(count);
        if direct {
            alternatives::solve(ref c, ref bs, true, false);
        } else {
            c.solve(ref bs, true, false);
        }
        c.remove_bias();
        if direct {
            alternatives::solve(ref c, ref bs, true, true);
        } else {
            c.solve(ref bs, true, true);
        }
        consume(c, @bs);
    }
    #[test]
    fn gas_solve_substep_gathered_one() {
        substep(1, false);
    }
    #[test]
    fn gas_solve_substep_gathered_two() {
        substep(2, false);
    }
    #[test]
    fn gas_solve_substep_direct_one() {
        substep(1, true);
    }
    #[test]
    fn gas_solve_substep_direct_two() {
        substep(2, true);
    }

    #[test]
    fn gas_generate() {
        let (m, bs, p) = fixture(2);
        let bs = array![opaque(*bs.at(0))];
        let c = ContactConstraintTrait::generate(
            opaque(m), bs.span(), opaque(p), opaque(p.substep_dt()),
        );
        consume(c, @bs);
    }
    #[test]
    fn gas_update() {
        let (mut c, bs) = input(2);
        let (m, _, p) = fixture(2);
        c.update(opaque(p), bs.span(), opaque(m));
        consume(c, @bs);
    }
    #[test]
    fn gas_warmstart() {
        let (c, mut bs) = input(2);
        c.warmstart(ref bs);
        consume(c, @bs);
    }
    #[test]
    fn gas_remove_bias() {
        let (mut c, bs) = input(2);
        c.remove_bias();
        consume(c, @bs);
    }
    #[test]
    fn gas_update_rhs_wo_bias() {
        let (mut c, bs) = input(2);
        c.update_rhs_wo_bias(bs.span());
        consume(c, @bs);
    }
    #[test]
    fn gas_apply_restitution() {
        let (mut c, mut bs) = input(2);
        let [mut a, b] = c.elements;
        a.restitution_seed = opaque(-ONE);
        a.normal_part.impulse_accumulator = opaque(ONE);
        a.normal_part.impulse = opaque(ZERO);
        c.elements = [a, b];
        c.apply_restitution(ref bs);
        consume(c, @bs);
    }
    #[test]
    fn gas_writeback_impulses() {
        let (c, bs) = input(2);
        let (m, _, _) = fixture(2);
        let mut ms = array![opaque(m)];
        c.writeback_impulses(ref ms);
        let _ = opaque(*ms.at(0));
        consume(c, @bs);
    }
}
