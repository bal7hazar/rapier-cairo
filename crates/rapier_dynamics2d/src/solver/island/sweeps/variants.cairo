//! Fixed-seed equivalence for every measured contact sweep formulation.
use fixed::{Fixed, FixedTrait};
use rapier_core::integration_parameters::IntegrationParametersTrait;
use super::*;
use super::super::super::body_store::DenseBodies;
use super::super::super::contact::cached as coefficients;

#[test]
fn gas_baseline() {
    let _ = rapier_testing::opaque(fixed::ONE);
}

#[test]
#[fuzzer(runs: 8, seed: 925)]
fn fuzz_all_contact_candidates(x: i16, y: i16) {
    let (input, _, ms) = super::super::fixtures::stack(2);
    let first = SolverBody { angvel: Fixed { raw: x.into() * 32768 }, ..*input.at(0) };
    let second = SolverBody { angvel: Fixed { raw: y.into() * 65536 }, ..*input.at(1) };
    let bs = [first, second];
    let mut m = *ms.at(if x < 0 {
        0
    } else {
        1
    });
    m.data.num_solver_contacts = if y < 0 {
        1
    } else {
        2
    };
    let p = IntegrationParameters { friction_in_bias_pass: x < 0, ..Default::default() };
    let mut c = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
    let [mut a, b] = c.elements;
    a.normal_part.impulse = Fixed { raw: x.into() * 32768 }.abs();
    a.tangent_part.impulse = Fixed { raw: y.into() * 16384 };
    c.elements = [a, b];
    let directions = coefficients::prepare([c].span());
    for variant in [0_u8, 1, 2, 3, 4, 5, 6, 7].span() {
        let mut old = ContactConstraintsSet { constraints: array![c] };
        let mut new = ContactConstraintsSet { constraints: array![c] };
        let mut old_bodies: DenseBodies = DenseBodiesTrait::new(bs.span());
        let mut new_bodies: DenseBodies = DenseBodiesTrait::new(bs.span());
        let mut dict = direct::alternatives::new_contacts([c].span());
        for stage in [0_u8, 1, 2, 3, 4].span() {
            alternatives::contacts_original(ref old, ref old_bodies, [m].span(), p, *stage);
            match *variant {
                0 => contacts(ref new, ref new_bodies, [m].span(), p, *stage),
                1 => alternatives::contacts_metered(ref new, ref new_bodies, [m].span(), p, *stage),
                2 => direct::contacts(ref new, ref new_bodies, [m].span(), p, *stage),
                3 => metered_pair::contacts(ref new, ref new_bodies, [m].span(), p, *stage),
                4 => specialized::contacts(ref new, ref new_bodies, [m].span(), p, *stage),
                5 => cached::contacts(
                    ref new, ref new_bodies, [m].span(), p, *stage, directions.span(),
                ),
                6 => sparse::contacts(
                    ref new, ref new_bodies, [m].span(), p, *stage, directions.span(),
                ),
                _ => {
                    direct::alternatives::contacts(ref dict, ref new_bodies, [m].span(), p, *stage);
                    new.constraints = direct::alternatives::finish_contacts(ref dict);
                },
            }
            assert_eq!(old.constraints.span(), new.constraints.span());
            assert_eq!(old_bodies.get(0), new_bodies.get(0));
            assert_eq!(old_bodies.get(1), new_bodies.get(1));
        }
    }
}
