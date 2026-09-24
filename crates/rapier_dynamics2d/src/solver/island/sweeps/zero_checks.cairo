//! Exercise each field of the no-op guard, including unused second-point fields.
use fixed::{ONE, ZERO};
use rapier_core::integration_parameters::IntegrationParametersTrait;
use super::*;
use super::super::super::body::BodyPair;
use super::super::super::contact::cached::{prepare, zero};

#[test]
fn test_zero_guard_and_each_nonzero_field() {
    let (bs, _, ms) = super::super::fixtures::stack(2);
    let p: IntegrationParameters = Default::default();
    let mut initial = ContactConstraintTrait::generate(*ms.at(1), bs.span(), p, p.substep_dt());
    initial.update(p, bs.span(), *ms.at(1));
    let d = *prepare([initial].span()).at(0);
    for count in [1_u8, 2].span() {
        for friction in [false, true].span() {
            let mut field = 0;
            while field != 15 {
                let mut old = initial;
                old.num_elements = *count;
                let [mut a, mut b] = old.elements;
                let mut pair = BodyPair { first: *bs.at(0), second: *bs.at(1) };
                match field {
                    1 => pair.first.linvel.x = ONE,
                    2 => pair.first.linvel.y = ONE,
                    3 => pair.second.linvel.x = ONE,
                    4 => pair.second.linvel.y = ONE,
                    5 => pair.first.angvel = ONE,
                    6 => pair.second.angvel = ONE,
                    7 => a.normal_part.impulse = ONE,
                    8 => a.normal_part.rhs = -ONE,
                    9 => a.tangent_part.impulse = ONE,
                    10 => a.tangent_part.rhs = -ONE,
                    11 => b.normal_part.impulse = ONE,
                    12 => b.normal_part.rhs = -ONE,
                    13 => b.tangent_part.impulse = ONE,
                    14 => b.tangent_part.rhs = -ONE,
                    _ => {},
                }
                old.elements = [a, b];
                let mut new = old;
                let mut bodies = array![pair.first, pair.second];
                old.solve(ref bodies, true, *friction);
                if *friction {
                    zero::solve_both(ref new, d, ref pair);
                } else {
                    zero::solve_normal_only(ref new, d, ref pair);
                }
                assert_eq!(old, new);
                assert_eq!(*bodies.at(0), pair.first);
                assert_eq!(*bodies.at(1), pair.second);
                if field == 0 {
                    assert_eq!(pair.first.angvel, ZERO);
                }
                field += 1;
            }
        }
    }
}

#[test]
fn gas_baseline() {
    let _ = rapier_testing::opaque(fixed::ONE);
}
