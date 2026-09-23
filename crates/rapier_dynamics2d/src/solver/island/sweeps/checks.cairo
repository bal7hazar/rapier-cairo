//! Raw equivalence, including velocities, impulses, accumulated impulses and unchanged fields.
use fixed::{Fixed, HALF, ONE, ZERO};
use rapier_core::integration_parameters::IntegrationParametersTrait;
use super::*;
use super::super::super::body_store::DenseBodies;
use super::super::super::contact::ContactConstraintsSetTrait;

#[test]
#[fuzzer(runs: 32, seed: 923)]
fn fuzz_old_new_stack(x: i16, y: i16, size: u8, flags: u8) {
    let (_, size) = core::num::traits::DivRem::div_rem(size, 4);
    let (input, steps, mut ms) = super::super::fixtures::stack(size.into() + 1);
    let mut bs = array![];
    for body in input.span() {
        let mut b = *body;
        b.linvel.x = Fixed { raw: x.into() * 32768 };
        b.linvel.y = Fixed { raw: y.into() * 65536 };
        b.angvel = Fixed { raw: x.into() * 16384 };
        bs.append(b);
    }
    let mut manifolds = array![];
    let mut id = 0_u32;
    while let Some(mut m) = ms.pop_front() {
        m.data.num_solver_contacts = if flags < 128 {
            1
        } else {
            2
        };
        m.data.restitution = HALF;
        if flags < 64 {
            // Old contacts exercise nonzero cached warm starts and impulse banking.
            let [mut a, mut b] = m.data.solver_contacts;
            a.contact_id = 0;
            b.contact_id = 1;
            m.data.solver_contacts = [a, b];
            let [mut a, mut b] = m.points;
            a.data.warmstart_impulse = HALF;
            b.data.warmstart_impulse = ONE;
            a.data.warmstart_tangent_impulse = -HALF;
            b.data.warmstart_tangent_impulse = HALF;
            m.points = [a, b];
        }
        if flags == 0 && id == 0 {
            m.data.num_solver_contacts = 0;
        }
        manifolds.append(m);
        id += 1;
    }
    let p = IntegrationParameters {
        friction_in_bias_pass: flags < 128,
        warmstart_coefficient: if flags == 255 {
            ZERO
        } else {
            HALF
        },
        ..Default::default(),
    };
    let mut old = ContactConstraintsSetTrait::generate(
        manifolds.span(), bs.span(), p, p.substep_dt(),
    );
    let mut new = ContactConstraintsSetTrait::generate(
        manifolds.span(), bs.span(), p, p.substep_dt(),
    );
    let directions = super::super::super::contact::cached::prepare(new.constraints.span());
    let mut old_bs: DenseBodies = DenseBodiesTrait::new(bs.span());
    let mut new_bs: DenseBodies = DenseBodiesTrait::new(bs.span());
    for stage in [0_u8, 1, 2, 3, 0, 1, 2, 4].span() {
        if *stage == 2 {
            super::super::integrate(ref old_bs, steps.span(), p.substep_dt(), fixed::MAX, ONE);
            super::super::integrate(ref new_bs, steps.span(), p.substep_dt(), fixed::MAX, ONE);
        }
        alternatives::contacts_original(ref old, ref old_bs, manifolds.span(), p, *stage);
        zero::contacts(ref new, ref new_bs, manifolds.span(), p, *stage, directions.span());
        assert_eq!(old.constraints.span(), new.constraints.span());
        let mut i = 0;
        while i != bs.len() {
            assert_eq!(old_bs.get(i), new_bs.get(i));
            i += 1;
        }
    }
}

#[test]
fn gas_baseline() {
    let _ = rapier_testing::opaque(fixed::ONE);
}
