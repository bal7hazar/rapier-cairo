//! The split sweeps against the constraint-set sweeps they replace (`sweeps::contact` on a
//! `DenseBodies` store), stage by stage, on a three-body stack with a world endpoint, a
//! one-point manifold, warm starts and approaching NEW contacts (restitution bounces). Gas and
//! step probes per stage on the same scene, one `gas_baseline`.
use fixed::{Fixed, FixedTrait, HALF, ONE};
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_geometry2d::contact::{ContactManifold, NEW_CONTACT_BIT};
use rapier_testing::opaque;
use super::generation::generate;
use super::super::contact::contacts as reference;
use super::super::super::fixtures::stack;
use super::super::super::super::body::SolverBody;
use super::super::super::super::body_store::{BodyStep, DenseBodies, DenseBodiesTrait};
use super::super::super::super::contact::{ContactConstraintsSetTrait, cached};
use super::super::super::{add_forces, damp, integrate};
use super::{SweepBodies, SweepBodiesTrait, alternatives, contacts, writeback};

/// The stack of three with velocities from `(vx, vy, spin)`, restitution on the world manifold
/// (whose contacts are NEW), warm starts `warm` on the second manifold (tracked point ids), and
/// one solver contact on the third.
fn scene(
    vx: i16, vy: i16, spin: i16, warm: u8,
) -> (Array<SolverBody>, Array<BodyStep>, Array<ContactManifold>, IntegrationParameters) {
    let (bs, steps, ms) = stack(3);
    let mut bodies = array![];
    let mut i: i64 = 0;
    for b in bs {
        let mut b = b;
        b.linvel.x = FixedTrait::from_raw(vx.into() * 1048573 + i * 77);
        b.linvel.y = FixedTrait::from_raw(vy.into() * 1048577 - i * 131);
        b.angvel = FixedTrait::from_raw(spin.into() * 524287 + i);
        bodies.append(b);
        i += 1;
    }
    let mut manifolds = array![];
    let mut k = 0;
    for m in ms {
        let mut m = m;
        if k == 0 {
            m.data.restitution = HALF;
        } else if k == 1 {
            let [mut a, mut b] = m.data.solver_contacts;
            a.contact_id = 0;
            b.contact_id = 1;
            m.data.solver_contacts = [a, b];
            let [mut p0, mut p1] = m.points;
            p0.data.warmstart_impulse = FixedTrait::from_raw(warm.into() * 16777213);
            p1.data.warmstart_impulse = FixedTrait::from_raw(warm.into() * 8388617);
            p0.data.warmstart_tangent_impulse = FixedTrait::from_raw(-(warm.into() * 4194301));
            m.points = [p0, p1];
        } else {
            m.data.num_solver_contacts = 1;
        }
        manifolds.append(m);
        k += 1;
    }
    (bodies, steps, manifolds, Default::default())
}

/// Every stage of one substep and the final bounce, plus forces, integration, damping and
/// writeback, through both implementations; bodies and manifolds must match exactly.
fn check(vx: i16, vy: i16, spin: i16, warm: u8) {
    let (bs, steps, ms, p) = scene(vx, vy, spin, warm);
    let dt = p.substep_dt();
    let mut cs = ContactConstraintsSetTrait::generate(ms.span(), bs.span(), p, dt);
    let directions = cached::prepare(cs.constraints.span());
    let mut dense: DenseBodies = DenseBodiesTrait::new(bs.span());
    let (frozen, mut state) = generate(ms.span(), bs.span(), p, dt);
    let mut sb: SweepBodies = DenseBodiesTrait::new(bs.span());
    let (max_lin, max_ang) = (p.max_linear_velocity(), Fixed { raw: 3373259426 } * p.inv_dt());
    add_forces(ref dense, steps.span());
    sb.add_forces(steps.span());
    // Stage 5 (the next substep's update from the separations cached by stage 2) against the
    // reference's recomputing update.
    for stage in array![0_u8, 1, 2, 3, 5, 1, 4] {
        if stage == 2 {
            integrate(ref dense, steps.span(), dt, max_lin, max_ang);
            sb.integrate(steps.span(), dt, max_lin, max_ang);
        }
        let reference_stage = if stage == 5 {
            0
        } else {
            stage
        };
        reference(ref cs, ref dense, ms.span(), p, reference_stage, directions.span());
        contacts(ref state, frozen.span(), ref sb, p, stage);
        let mut i = 0;
        while i != 3 {
            assert_eq!(sb.get(i), dense.get(i));
            i += 1;
        }
    }
    damp(ref dense, steps.span(), p.dt);
    sb.damp(steps.span(), p.dt);
    let mut expected = ms.clone();
    cs.writeback_impulses(ref expected);
    let mut actual = ms;
    writeback(frozen.span(), @state, ref actual);
    assert_eq!(actual, expected);
    let mut out: DenseBodies = DenseBodiesTrait::new(bs.span());
    sb.finish(ref out);
    let mut i = 0;
    while i != 3 {
        assert_eq!(out.get(i), dense.get(i));
        i += 1;
    }
}

#[test]
fn test_split_matches_constraint_set_sweeps() {
    // (vx, vy, spin, warm): resting, falling onto the stack (bounces), sliding and spinning,
    // rising (no bounce), and a cold start.
    let cases = array![
        (0_i16, 0_i16, 0_i16, 0_u8), (0, -2000, 0, 3), (1500, -700, 900, 9), (-800, 1200, -300, 1),
        (300, -3000, 20, 0),
    ];
    for (vx, vy, spin, warm) in cases {
        check(vx, vy, spin, warm);
    }
}

#[test]
#[fuzzer(runs: 24, seed: 925)]
fn fuzz_split_matches_constraint_set_sweeps(vx: i16, vy: i16, spin: i16, warm: u8) {
    check(vx / 8, vy / 8, spin / 8, warm);
}

/// Probe of the split path: generation only (`stage == 5`; `8`: BT1's generation through the
/// constraints, `alternatives::generate_via_constraints`), or generation then one stage.
fn probe(stage: u8) {
    let (bs, _, ms, p) = scene(opaque(0), opaque(-2000), opaque(900), opaque(3));
    let (frozen, mut state) = if stage == 8 {
        alternatives::generate_via_constraints(ms.span(), bs.span(), p, p.substep_dt())
    } else {
        generate(ms.span(), bs.span(), p, p.substep_dt())
    };
    let mut sb: SweepBodies = DenseBodiesTrait::new(bs.span());
    if stage < 5 {
        contacts(ref state, frozen.span(), ref sb, p, stage);
    } else if stage == 7 {
        alternatives::biased_metered(ref state, frozen.span(), ref sb);
    } else if stage == 6 {
        let mut ms = ms;
        writeback(frozen.span(), @state, ref ms);
        let _ = opaque(ms.span());
    }
    let _ = opaque((state.hot.span(), state.bank.span(), sb.get(opaque(1))));
}

#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}
#[test]
fn gas_generate_stack3() {
    probe(5);
}
#[test]
fn gas_generate_via_constraints_stack3() {
    probe(8);
}
#[test]
fn gas_update_stack3() {
    probe(0);
}
#[test]
fn gas_biased_stack3() {
    probe(1);
}
#[test]
fn gas_biased_metered_stack3() {
    probe(7);
}
#[test]
fn gas_refresh_stack3() {
    probe(2);
}
#[test]
fn gas_relax_stack3() {
    probe(3);
}
#[test]
fn gas_restitution_stack3() {
    probe(4);
}
#[test]
fn gas_writeback_stack3() {
    probe(6);
}
