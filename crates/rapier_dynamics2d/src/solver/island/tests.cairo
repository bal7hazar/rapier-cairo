//! Driver regressions, including the immutable-array candidate as an independent store oracle.
use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use rapier_core::rigid_body::RigidBodyType;
use rapier_geometry2d::contact::ContactManifoldTrait;
use crate::joint::RevoluteJointBuilderTrait;
use crate::rigid_body_set::{RigidBodySetTrait, RigidBodyTrait};
use super::*;
use super::fixtures::{h, stack, v};
use super::super::body_store::alternatives::ArrayBodies;
use super::super::body_store::{DenseBodies, SolverBodyStoreTrait};
use super::super::joint::JointConstraintTrait;

#[test]
fn test_inert_manifolds_match_absent_contacts_and_preserve_cached_data() {
    for disabled in [false, true].span() {
        let (bs, steps, ms) = stack(2);
        let mut m = *ms.at(0);
        if *disabled {
            m.data.solver_flags.bits = 0;
        } else {
            m.data.num_solver_contacts = 0;
        }
        let [mut point, second] = m.points;
        point.data.impulse = ONE;
        point.data.warmstart_impulse = HALF;
        point.data.warmstart_tangent_impulse = -HALF;
        m.points = [point, second];
        let mut inert = array![m, m];
        let mut absent = array![];
        let mut a: DenseBodies = DenseBodiesTrait::new(bs.span());
        let mut b: DenseBodies = DenseBodiesTrait::new(bs.span());
        let joint = ImpulseJoint {
            body1: h(0),
            body2: h(1),
            data: RevoluteJointBuilderTrait::new().build(),
            impulses: [HALF, -HALF, ZERO],
        };
        let mut ja = array![joint];
        let mut jb = array![joint];
        run(Default::default(), ref a, steps.span(), ref inert, ref ja);
        run(Default::default(), ref b, steps.span(), ref absent, ref jb);
        assert_eq!(a.get(0), b.get(0));
        assert_eq!(a.get(1), b.get(1));
        assert_eq!(ja.span(), jb.span());
        assert_eq!(inert.span(), [m, m].span());
    }
}

#[test]
fn test_empty_zero_dt_and_disabled_fixed_kinematic() {
    let mut set = RigidBodySetTrait::new();
    let mut p: IntegrationParameters = Default::default();
    p.dt = ZERO;
    let (mut store, _) = SolverBodyStoreTrait::from_bodies(ref set, v(ZERO, -ONE), p);
    let mut ms = array![];
    let mut js = array![];
    solve_island(p, ref store, ref ms, ref js);
    store.to_bodies(ref set);
    assert_eq!(store.len(), 0);
    for kind in [
        RigidBodyType::Fixed, RigidBodyType::Dynamic, RigidBodyType::KinematicPositionBased,
        RigidBodyType::KinematicVelocityBased,
    ]
        .span() {
        let mut rb = RigidBodyTrait::new(*kind, Default::default());
        rb.enabled = *kind != RigidBodyType::Dynamic;
        rb.vels.linvel.x = ONE;
        rb.pos.next_position.translation.x = HALF;
        let _ = set.insert(rb);
    }
    p = Default::default();
    let (mut store, _) = SolverBodyStoreTrait::from_bodies(ref set, v(ZERO, -ONE), p);
    solve_island(p, ref store, ref ms, ref js);
    store.to_bodies(ref set);
    assert_eq!(set.get(h(0)).unwrap().pos.next_position.translation.x, HALF);
    assert_eq!(set.get(h(1)).unwrap().pos.next_position.translation.x, HALF);
    assert_eq!(set.get(h(2)).unwrap().pos.next_position.translation.x, HALF);
    assert_eq!(set.get(h(3)).unwrap().pos.next_position.translation.x, p.dt);
}
#[test]
fn test_forces_caps_damping_and_next_position() {
    let mut set = RigidBodySetTrait::new();
    let mut rb = RigidBodyTrait::dynamic(Default::default());
    rb.mprops.local_mprops.inv_mass = ONE;
    rb.mprops.local_mprops.inv_principal_inertia = ONE;
    rb.forces.user_force = v(ONE, ZERO);
    rb.forces.user_torque = ONE;
    rb.vels.linvel = v(FixedTrait::from_int(3), FixedTrait::from_int(4));
    rb.vels.angvel = FixedTrait::from_int(100);
    rb.damping.linear_damping = ONE;
    let h = set.insert(rb);
    let p = IntegrationParameters { normalized_max_linear_velocity: ONE, ..Default::default() };
    let (mut store, _) = SolverBodyStoreTrait::from_bodies(ref set, v(ZERO, -ONE), p);
    let mut ms = array![];
    let mut js = array![];
    solve_island(p, ref store, ref ms, ref js);
    store.to_bodies(ref set);
    let out = set.get(h).unwrap();
    assert!(out.vels.linvel.length() < ONE);
    assert!(out.vels.angvel <= Fixed { raw: 3373259426 } * p.inv_dt());
    assert_eq!(out.pos.position, rb.pos.position);
    assert!(out.pos.next_position.translation.length() <= p.dt);
    // Infinite sentinel is guarded before length scaling, even with a large length unit.
    let p = IntegrationParameters {
        normalized_max_linear_velocity: fixed::MAX,
        normalized_max_corrective_velocity: fixed::MAX,
        length_unit: fixed::MAX,
        ..p,
    };
    let (mut store, _) = SolverBodyStoreTrait::from_bodies(ref set, Default::default(), p);
    solve_island(p, ref store, ref ms, ref js);
    assert!(store.get(0).linvel.x > ZERO);
}
#[test]
fn test_force_substeps_closed_form_and_zero_dt_no_damping() {
    for n in [1_u32, 2, 4, 8].span() {
        let mut set = RigidBodySetTrait::new();
        let mut rb = RigidBodyTrait::dynamic(Default::default());
        rb.mprops.local_mprops.inv_mass = ONE;
        rb.forces.user_force.x = ONE;
        let h = set.insert(rb);
        let p = IntegrationParameters {
            dt: ONE,
            num_solver_iterations: *n,
            num_internal_pgs_iterations: 0,
            num_internal_stabilization_iterations: 0,
            ..Default::default(),
        };
        let (mut store, _) = SolverBodyStoreTrait::from_bodies(ref set, Default::default(), p);
        let mut ms = array![];
        let mut js = array![];
        solve_island(p, ref store, ref ms, ref js);
        store.to_bodies(ref set);
        let out = set.get(h).unwrap();
        assert_eq!(out.vels.linvel.x, ONE);
        assert_eq!(out.pos.next_position.translation.x, (ONE + p.substep_dt()) * HALF);
        let zero = IntegrationParameters { dt: ZERO, ..p };
        let (mut store, _) = SolverBodyStoreTrait::from_bodies(ref set, Default::default(), zero);
        let before = store.get(0);
        solve_island(zero, ref store, ref ms, ref js);
        assert_eq!(store.get(0), before);
    }
}
#[test]
fn test_iteration_counts_candidates_and_impulse_writeback() {
    for (pgs, relax) in [(0_u32, 0_u32), (1, 0), (0, 2), (1, 1), (3, 2)].span() {
        let (bs, steps, mut ms) = stack(3);
        let mut other_ms = array![];
        other_ms.append_span(ms.span());
        let mut a: DenseBodies = DenseBodiesTrait::new(bs.span());
        let mut b: ArrayBodies = DenseBodiesTrait::new(bs.span());
        let p = IntegrationParameters {
            num_internal_pgs_iterations: *pgs,
            num_internal_stabilization_iterations: *relax,
            ..Default::default(),
        };
        let mut js = array![];
        let mut other_js = array![];
        run(p, ref a, steps.span(), ref ms, ref js);
        run(p, ref b, steps.span(), ref other_ms, ref other_js);
        let mut i = 0;
        while i != 3 {
            assert_eq!(a.get(i), b.get(i));
            assert_eq!(*ms.at(i), *other_ms.at(i));
            i += 1;
        }
        if *pgs == 0 && *relax == 0 {
            assert_eq!(ms.at(0).point(0).data.impulse, ZERO);
        } else {
            assert!(ms.at(0).point(0).data.impulse > ZERO);
        }
    }
}
#[test]
#[fuzzer(runs: 24, seed: 19)]
fn fuzz_nonrestitutive_contact_does_not_add_energy(x: i8, y: u8) {
    let (bs, steps, mut ms) = stack(1);
    let b = SolverBody {
        linvel: v(Fixed { raw: x.into() * 16777216 }, -Fixed { raw: y.into() * 16777216 }),
        ..*bs.at(0),
    };
    let mut bodies: DenseBodies = DenseBodiesTrait::new([b].span());
    let step = BodyStep { increment: Default::default(), ..*steps.at(0) };
    let mut js = array![];
    run(Default::default(), ref bodies, [step].span(), ref ms, ref js);
    let after = bodies.get(0);
    let initial = b.linvel.length_squared();
    let final_energy = after.linvel.length_squared() + after.angvel * after.angvel / b.ii;
    assert!(final_energy <= initial + Fixed { raw: 64 });
}
#[test]
#[should_panic(expected: 'IntegrationParams: zero iters')]
fn test_zero_substeps_rejected() {
    let mut set = RigidBodySetTrait::new();
    let p = IntegrationParameters { num_solver_iterations: 0, ..Default::default() };
    let _ = SolverBodyStoreTrait::from_bodies(ref set, Default::default(), p);
}
#[test]
#[should_panic(expected: 'Island: negative parameter')]
fn test_negative_dt_rejected() {
    let (bs, steps, mut ms) = stack(1);
    let mut bodies: DenseBodies = DenseBodiesTrait::new(bs.span());
    let mut js = array![];
    run(
        IntegrationParameters { dt: -ONE, ..Default::default() },
        ref bodies,
        steps.span(),
        ref ms,
        ref js,
    );
}

// Whole-array DC/DE calls provide an oracle for pair remapping, stage order, repeated sweeps,
// and joint warmstart carry across substeps, including simultaneous contact/joint constraints.
#[test]
fn test_mixed_constraints_match_frozen_whole_array_driver() {
    for warm in [false, true].span() {
        let (bs, steps, mut ms) = stack(2);
        let p = IntegrationParameters {
            warmstart_joints: *warm,
            num_internal_pgs_iterations: 2,
            num_internal_stabilization_iterations: 3,
            ..Default::default(),
        };
        let mut j = ImpulseJoint {
            body1: h(0),
            body2: h(1),
            data: RevoluteJointBuilderTrait::new()
                .local_anchor1(v(ZERO, HALF))
                .local_anchor2(v(ZERO, -HALF))
                .build(),
            impulses: [HALF, -HALF, ZERO],
        };
        let mut js = array![j];
        let mut dict: DenseBodies = DenseBodiesTrait::new(bs.span());
        let mut reference = bs;
        let mut cs = ContactConstraintsSetTrait::generate(
            ms.span(), reference.span(), p, p.substep_dt(),
        );
        let mut sub = 0;
        while sub != p.num_solver_iterations {
            let mut next = array![];
            let mut i = 0;
            while let Some(mut b) = reference.pop_front() {
                b.linvel = b.linvel + *steps.at(i).increment.linvel;
                b.angvel += *steps.at(i).increment.angvel;
                next.append(b);
                i += 1;
            }
            reference = next;
            let mut jc = JointConstraintTrait::generate(j, reference.span(), p);
            cs.update(p, reference.span(), ms.span());
            cs.warmstart(ref reference);
            i = 0;
            while i != p.num_internal_pgs_iterations {
                if *warm && i == 0 {
                    jc.warmstart(ref reference);
                }
                jc.solve(ref reference, true);
                cs.solve(ref reference, true, p.friction_in_bias_pass);
                i += 1;
            }
            let mut next = array![];
            while let Some(mut b) = reference.pop_front() {
                b
                    .position = RigidBodyVelocity { linvel: b.linvel, angvel: b.angvel }
                    .integrate(p.substep_dt(), b.position, Default::default());
                next.append(b);
            }
            reference = next;
            i = 0;
            while i != p.num_internal_stabilization_iterations {
                jc.solve(ref reference, false);
                if i == 0 {
                    cs.update_rhs_wo_bias(reference.span());
                }
                cs.solve(ref reference, true, true);
                i += 1;
            }
            jc.writeback_impulses(ref j);
            sub += 1;
        }
        cs.apply_restitution(ref reference);
        let mut expected_ms = array![];
        expected_ms.append_span(ms.span());
        cs.writeback_impulses(ref expected_ms);
        run(p, ref dict, steps.span(), ref ms, ref js);
        assert_eq!(*js.at(0), j);
        for i in [0_u32, 1].span() {
            assert_eq!(dict.get(*i), *reference.at(*i));
            assert_eq!(*ms.at(*i), *expected_ms.at(*i));
        }
    }
}
#[test]
fn test_joint_body_frames_shift_to_com_and_disabled_joint_is_inert() {
    let mut set = RigidBodySetTrait::new();
    let _ = set.insert(RigidBodyTrait::fixed(Default::default()));
    let mut bob = RigidBodyTrait::dynamic(
        rapier_math::pose2::Pose2 { translation: v(ONE, ZERO), ..Default::default() },
    );
    bob.mprops.local_mprops.local_com = v(HALF, ZERO);
    bob.mprops.local_mprops.inv_mass = ONE;
    bob.mprops.local_mprops.inv_principal_inertia = ONE;
    let _ = set.insert(bob);
    let p: IntegrationParameters = Default::default();
    let (mut store, _) = SolverBodyStoreTrait::from_bodies(ref set, Default::default(), p);
    let joint = ImpulseJoint {
        body1: h(0),
        body2: h(1),
        impulses: [ZERO, ZERO, ZERO],
        data: RevoluteJointBuilderTrait::new().local_anchor2(v(-ONE, ZERO)).build(),
    };
    let mut disabled = joint;
    disabled.body1 = h(99);
    disabled.data.enabled = crate::joint::JointEnabled::Disabled;
    disabled.impulses = [ONE, HALF, -ONE];
    let mut joints = array![joint, disabled];
    let mut ms = array![];
    let before = store.get(1);
    solve_island(p, ref store, ref ms, ref joints);
    assert_eq!(store.get(1), before);
    assert_eq!(*joints.at(1), disabled);
}

/// BT4: `solve_island_input` against the store driver `run` on the same input: the same bodies
/// and joints, and the returned impulses written into the input manifolds give the manifolds
/// `run` rebuilt. Rows `(stack size, joint, parameters)`: 0 default, 1 zero dt, 2 one substep
/// with two stabilization sweeps, 3 inert contacts (joint-only stages).
#[test]
fn test_input_driver_matches_store_driver() {
    let rows = array![
        (1_u32, false, 0_u8), (2, true, 0), (3, false, 2), (2, true, 1), (2, true, 3),
    ];
    for (n, joint, k) in rows {
        let (bs, steps, ms) = stack(n);
        let mut p: IntegrationParameters = Default::default();
        if k == 1 {
            p.dt = ZERO;
        } else if k == 2 {
            p.num_solver_iterations = 1;
            p.num_internal_stabilization_iterations = 2;
        }
        let mut input_ms = array![];
        for m in ms.span() {
            let mut m = *m;
            if k == 3 {
                m.data.solver_flags.bits = 0;
            }
            input_ms.append(m);
        }
        let revolute = ImpulseJoint {
            body1: h(0),
            body2: h(1),
            data: RevoluteJointBuilderTrait::new()
                .local_anchor1(v(ZERO, HALF))
                .local_anchor2(v(ZERO, -HALF))
                .build(),
            impulses: [HALF, -HALF, ZERO],
        };
        let mut js = if joint {
            array![revolute]
        } else {
            array![]
        };
        let mut js_input = if joint {
            array![revolute]
        } else {
            array![]
        };
        let mut dict: DenseBodies = DenseBodiesTrait::new(bs.span());
        let mut expected_ms = input_ms.clone();
        run(p, ref dict, steps.span(), ref expected_ms, ref js);
        let solved = solve_island_input(
            p, SolverInput { bodies: bs, steps }, input_ms.span(), ref js_input,
        );
        assert!(js_input == js, "joints, row {}", n);
        let mut i = 0;
        while i != n {
            assert!(*solved.bodies.at(i) == dict.get(i), "body {} of {}", i, n);
            i += 1;
        }
        let mut id = 0;
        for m in input_ms.span() {
            let mut m = *m;
            solved.write_impulses(id, ref m);
            assert!(m == *expected_ms.at(id), "manifold {} of {}", id, n);
            id += 1;
        }
    }
}
