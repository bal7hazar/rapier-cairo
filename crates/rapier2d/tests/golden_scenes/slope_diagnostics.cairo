//! SD: isolate geometry, solver arms and integration without changing the engine.
use glam::Vec2Trait;
use rapier2d::pipeline;
use rapier2d::prelude::{RigidBodyTrait, Vec2, World, WorldTrait};
use rapier_core::integration_parameters::IntegrationParametersTrait;
use rapier_dynamics2d::rigid_body::{RigidBodyVelocity, RigidBodyVelocityTrait};
use rapier_dynamics2d::solver::body::SolverBody;
use rapier_dynamics2d::solver::contact::ContactConstraintTrait;
use rapier_geometry2d::contact::{ContactManifold, NEW_CONTACT_BIT};
use rapier_geometry2d::manifold::ManifoldTrait;
use rapier_golden::compare::abs_diff;
use rapier_golden::scenes;
use rapier_math::pose2::Pose2Trait;
use super::builder::{body_handle, build_world, f};
use super::{Stats, compare};

fn geometry(m: ContactManifold) {
    println!("normal {:?} {:?}; world {:?}", m.local_n1, m.local_n2, m.data.normal);
    println!(
        "material {} {}; count {} {}",
        m.data.friction.raw,
        m.data.restitution.raw,
        m.num_points,
        m.data.num_solver_contacts,
    );
    let [p0, p1] = m.points;
    for p in array![p0, p1] {
        println!(
            "point {} {} / {} {} dist {} fid {} {} impulse {} {}",
            p.local_p1.x.raw,
            p.local_p1.y.raw,
            p.local_p2.x.raw,
            p.local_p2.y.raw,
            p.dist.raw,
            p.fid1.packed,
            p.fid2.packed,
            p.data.impulse.raw,
            p.data.tangent_impulse.raw,
        );
    }
    for sc in m.data.solver_contacts.span() {
        println!(
            "solver {} arms {} {} / {} {} dist {}",
            sc.contact_id,
            sc.anchor1.x.raw,
            sc.anchor1.y.raw,
            sc.anchor2.x.raw,
            sc.anchor2.y.raw,
            sc.dist.raw,
        );
    }
}

/// Public DC calls in DF order, specific to this single pair, with no joints/damping/caps.
/// Cross-checked against the actual pipeline solve below so a diagnostic drift fails loudly.
fn trace(ref world: World, m: ContactManifold) -> SolverBody {
    let rb = world.body(body_handle(1)).unwrap();
    let b = SolverBody {
        handle: body_handle(1),
        position: rb.position(),
        linvel: rb.linvel(),
        angvel: rb.vels.angvel,
        im: rb.mprops.effective_inv_mass,
        ii: rb.mprops.effective_world_inv_inertia,
    };
    let fixed_body = SolverBody { handle: body_handle(0), ..Default::default() };
    let mut bodies = array![fixed_body, b];
    let p = world.integration_parameters;
    let dt = p.substep_dt();
    let mut c = ContactConstraintTrait::generate(m, bodies.span(), p, dt);
    println!(
        "coeff invdt {} erp_invdt {} cfm {}", c.inv_dt.raw, c.erp_inv_dt.raw, c.soft_cfm_factor.raw,
    );
    for e in c.elements.span() {
        println!(
            "row ng {} {} nr {} tg {} {} tr {} base {}",
            e.normal_part.gcross1.raw,
            e.normal_part.gcross2.raw,
            e.normal_part.r.raw,
            e.tangent_part.gcross1.raw,
            e.tangent_part.gcross2.raw,
            e.tangent_part.r.raw,
            e.dist.raw,
        );
    }
    let mut sub = 0;
    while sub != p.num_solver_iterations {
        let mut b = *bodies.at(1);
        b.linvel = b.linvel + world.gravity.mul_scalar(dt);
        bodies = array![fixed_body, b];
        c.update(p, bodies.span(), m);
        println!(
            "rhs {} {}",
            c.elements.span().at(0).normal_part.rhs.raw,
            c.elements.span().at(1).normal_part.rhs.raw,
        );
        c.warmstart(ref bodies);
        c.solve(ref bodies, true, p.friction_in_bias_pass);
        let mut b = *bodies.at(1);
        println!(
            "sub {} integrate v {} {} w {}", sub, b.linvel.x.raw, b.linvel.y.raw, b.angvel.raw,
        );
        b
            .position = RigidBodyVelocity { linvel: b.linvel, angvel: b.angvel }
            .integrate(dt, b.position, Default::default());
        bodies = array![fixed_body, b];
        c.update_rhs_wo_bias(bodies.span());
        c.solve(ref bodies, true, true);
        let b = *bodies.at(1);
        println!("sub {} relaxed v {} {} w {}", sub, b.linvel.x.raw, b.linvel.y.raw, b.angvel.raw);
        sub += 1;
    }
    c.apply_restitution(ref bodies);
    *bodies.at(1)
}

// Modes: 0 current engine (midpoint arms since DM); 1 same; 2 plus f64 last-match aliasing.
fn diagnostic_step(ref world: World, mode: u8, verbose: bool) {
    pipeline::handle_user_changes(ref world.bodies, ref world.colliders);
    let old = if world.narrow_phase.pairs.is_empty() {
        None
    } else {
        Some(*world.narrow_phase.pairs.at(0).manifold)
    };
    let pos12 = world
        .body(body_handle(0))
        .unwrap()
        .position()
        .inv_mul(world.body(body_handle(1)).unwrap().position());
    let old_last = match old {
        Some(mut m) => {
            let last = *m.points.span().at(1);
            if mode == 2 && !m.try_update_contacts(pos12) {
                Some(last.data)
            } else {
                None
            }
        },
        None => None,
    };
    let _ = pipeline::detect_collisions(
        world.integration_parameters, ref world.bodies, ref world.colliders, ref world.narrow_phase,
    );
    // DM moved the shared-midpoint lever arms into the engine: mode 1 is now mode 0.
    if mode != 0 {
        let mut pairs = array![];
        while let Some(mut pair) = world.narrow_phase.pairs.pop_front() {
            if let Some(data) = old_last {
                println!(
                    "f64 alias warm {} {}",
                    data.warmstart_impulse.raw,
                    data.warmstart_tangent_impulse.raw,
                );
                let [mut p0, mut p1] = pair.manifold.points;
                p0.data = data;
                p1.data = data;
                pair.manifold.points = [p0, p1];
                let [mut c0, mut c1] = pair.manifold.data.solver_contacts;
                c0.contact_id = if data.impulse.raw == 0 {
                    NEW_CONTACT_BIT
                } else {
                    0
                };
                c1.contact_id = c0.contact_id + 1;
                pair.manifold.data.solver_contacts = [c0, c1];
            }
            pairs.append(pair);
        }
        world.narrow_phase.pairs = pairs;
    }
    let expected = if verbose {
        let m = *world.narrow_phase.pairs.at(0).manifold;
        geometry(m);
        Some(trace(ref world, m))
    } else {
        None
    };
    pipeline::solve(
        world.gravity,
        world.integration_parameters,
        ref world.bodies,
        ref world.narrow_phase,
        ref world.impulse_joints,
    );
    pipeline::advance_to_final_positions(ref world.bodies, ref world.colliders);
    if let Some(b) = expected {
        let rb = world.body(body_handle(1)).unwrap();
        println!(
            "position {} {} rotation {} {}",
            b.position.translation.x.raw,
            b.position.translation.y.raw,
            b.position.rotation.re.raw,
            b.position.rotation.im.raw,
        );
        assert_eq!(rb.position(), b.position);
        assert_eq!(rb.linvel(), b.linvel);
        assert_eq!(rb.vels.angvel, b.angvel);
    }
}

#[test]
fn test_slope_first_contact_and_f64_matching() {
    for scene in array![scenes::BOX_SLOPE_STICK, scenes::BOX_SLOPE_SLIDE] {
        for mode in array![0_u8, 1, 2] {
            println!("scene {} mode {}", scene.id, mode);
            let mut world = build_world(scene);
            let mut stats: Stats = Default::default();
            let mut step = 1_u32;
            while step != 11 {
                diagnostic_step(ref world, mode, step == 3);
                stats = compare(ref world, scene, *scene.samples.span().at(step), stats);
                if step == 3 {
                    assert_eq!(stats.violations, 0, "midpoint recovers first contact");
                    let m = *world.narrow_phase.pairs.at(0).manifold;
                    assert_first_geometry(m);
                }
                step += 1;
            }
            println!("violations {}", stats.violations);
            assert_eq!(stats.violations, if mode != 2 {
                7
            } else {
                0
            });
        }
    }
}

// Upstream step 3 in scenes.json, identical for both materials. IDs intentionally retain
// the port's correct f32 sign-bit convention instead of the f64 duplicate-ID bug.
fn assert_first_geometry(m: ContactManifold) {
    assert_eq!(m.num_points, 2);
    assert_eq!(m.data.num_solver_contacts, 2);
    assert_eq!(m.local_n1, Vec2 { x: f(0), y: f(4294967296) });
    assert_eq!(m.local_n2, Vec2 { x: f(0), y: f(-4294967296) });
    assert_eq!(m.data.normal, Vec2 { x: f(-2147483648), y: f(3719550787) });
    for (i, x1, x2, fid2) in array![
        (0_u32, 2134316890_i64, 2147483648_i64, 1073741826_u32),
        (1, -2160650407, -2147483648, 1073741827),
    ] {
        let p = *m.points.span().at(i);
        assert!(abs_diff(p.local_p1.x.raw, x1) <= 16);
        assert_eq!(p.local_p1.y.raw, 2147483648);
        assert_eq!(p.local_p2, Vec2 { x: f(x2), y: f(-2147483648) });
        assert!(abs_diff(p.dist.raw, 20144178) <= 8);
        assert_eq!((p.fid1.packed, p.fid2.packed), (3221225524, fid2));
        assert_eq!(*m.data.solver_contacts.span().at(i).contact_id, NEW_CONTACT_BIT + i);
    }
}

#[test]
fn test_slide_late_integration() {
    late(scenes::BOX_SLOPE_SLIDE, 0);
}

#[test]
fn test_slide_f64_counterfactual() {
    late(scenes::BOX_SLOPE_SLIDE, 2);
}

#[test]
fn test_stick_f64_counterfactual() {
    late(scenes::BOX_SLOPE_STICK, 2);
}

fn late(scene: rapier_golden::types::SceneCase, mode: u8) {
    let mut world = build_world(scene);
    let mut step = 1_u32;
    let mut next = 1;
    let mut stats: Stats = Default::default();
    while step != 121 {
        diagnostic_step(ref world, mode, step == 120);
        if step == 119 {
            let b = world.body(body_handle(1)).unwrap();
            println!(
                "position119 {} {}", b.position().translation.x.raw, b.position().translation.y.raw,
            );
        }
        if *scene.samples.span().at(next).step == step {
            stats = compare(ref world, scene, *scene.samples.span().at(next), stats);
            next += 1;
        }
        step += 1;
    }
    if mode == 2 {
        assert_eq!(stats.violations, 0, "f64 counterfactual recovers all samples");
    } else {
        let last = compare(ref world, scene, *scene.samples.span().at(21), Default::default());
        assert!(last.vx.ulps <= 2000 && last.vy.ulps <= 2000);
        assert!(last.tx.ulps > 4096 * 120 && last.ty.ulps > 4096 * 120);
    }
}
