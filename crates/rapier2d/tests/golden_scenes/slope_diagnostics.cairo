//! SD / GS: isolate geometry, solver arms and integration of the two slope scenes without
//! changing the engine. Since GS the references carry correct cuboid feature ids, so the f64
//! duplicate-id emulation is gone; what remains is a public-API tracer of the single slope pair,
//! checked bit for bit against `pipeline::solve`, and one counterfactual: upstream switches a
//! normal row to rigid when its refreshed gap is `> 0`, and a speculative contact that the
//! previous substep closed exactly sits at a gap of `0` in Q32.32 but at an f64 rounding residue
//! of either sign upstream.
use glam::Vec2Trait;
use rapier2d::pipeline;
use rapier2d::prelude::{RigidBodyTrait, Vec2, World, WorldTrait};
use rapier_core::integration_parameters::IntegrationParametersTrait;
use rapier_dynamics2d::rigid_body::{RigidBodyVelocity, RigidBodyVelocityTrait};
use rapier_dynamics2d::solver::body::SolverBody;
use rapier_dynamics2d::solver::contact::{ContactConstraint, ContactConstraintTrait};
use rapier_geometry2d::contact::{ContactManifold, NEW_CONTACT_BIT};
use rapier_golden::compare::abs_diff;
use rapier_golden::scenes;
use super::builder::{body_handle, build_world, f};
use super::{Stats, compare};

/// `1` in Q32.32.
const ONE: i64 = 0x100000000;

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
/// Cross-checked against the actual pipeline solve so a diagnostic drift fails loudly.
/// With `zero_rigid`, a normal row whose refreshed gap is exactly zero (`rhs == 0`: a gap of
/// -1 raw already gives `rhs <= -17`) is solved rigidly, as upstream does when its f64 gap
/// rounds to `+0`. Returns the solved body, the constraint and the number of such rows.
fn trace(
    ref world: World, m: ContactManifold, zero_rigid: bool, verbose: bool,
) -> (SolverBody, ContactConstraint, u32) {
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
    if verbose {
        println!(
            "coeff invdt {} erp_invdt {} cfm {}",
            c.inv_dt.raw,
            c.erp_inv_dt.raw,
            c.soft_cfm_factor.raw,
        );
    }
    let mut flips = 0;
    let mut sub = 0;
    while sub != p.num_solver_iterations {
        let mut b = *bodies.at(1);
        b.linvel = b.linvel + world.gravity.mul_scalar(dt);
        bodies = array![fixed_body, b];
        c.update(p, bodies.span(), m);
        let [mut e0, mut e1] = c.elements;
        if zero_rigid && e0.normal_part.rhs == f(0) && e0.normal_part.cfm_factor != f(ONE) {
            e0.normal_part.cfm_factor = f(ONE);
            flips += 1;
        }
        if zero_rigid
            && c.num_elements == 2
            && e1.normal_part.rhs == f(0)
            && e1.normal_part.cfm_factor != f(ONE) {
            e1.normal_part.cfm_factor = f(ONE);
            flips += 1;
        }
        c.elements = [e0, e1];
        c.warmstart(ref bodies);
        c.solve(ref bodies, true, p.friction_in_bias_pass);
        let mut b = *bodies.at(1);
        if verbose {
            println!(
                "sub {} rhs {} {} integrate v {} {} w {}",
                sub,
                e0.normal_part.rhs.raw,
                e1.normal_part.rhs.raw,
                b.linvel.x.raw,
                b.linvel.y.raw,
                b.angvel.raw,
            );
        }
        b
            .position = RigidBodyVelocity { linvel: b.linvel, angvel: b.angvel }
            .integrate(dt, b.position, Default::default());
        bodies = array![fixed_body, b];
        c.update_rhs_wo_bias(bodies.span());
        c.solve(ref bodies, true, true);
        sub += 1;
    }
    c.apply_restitution(ref bodies);
    (*bodies.at(1), c, flips)
}

/// One `World::step` split into its pipeline stages. Engine mode solves with the pipeline and,
/// when `verbose`, also traces the single slope pair with the public constraint API and asserts
/// that the pipeline reproduces the trace bit for bit. The `zero_rigid` counterfactual always
/// traces and writes the traced state and impulses back instead of solving (one solve per step
/// keeps 120 steps within the VM budget). Returns the number of counterfactual rows.
fn diagnostic_step(ref world: World, zero_rigid: bool, verbose: bool) -> u32 {
    pipeline::handle_user_changes(
        ref world.bodies, ref world.colliders, world.narrow_phase.pairs.span(),
    );
    let _ = pipeline::detect_collisions(
        world.integration_parameters, ref world.bodies, ref world.colliders, ref world.narrow_phase,
    );
    let traced = if world.narrow_phase.pairs.is_empty() {
        None
    } else {
        let m = *world.narrow_phase.pairs.at(0).manifold;
        if verbose {
            geometry(m);
        }
        if m.data.num_solver_contacts == 0 || !(zero_rigid || verbose) {
            None
        } else {
            Some((m, trace(ref world, m, zero_rigid, verbose)))
        }
    };
    let flips = match traced {
        Some((_, (_, _, n))) => n,
        None => 0,
    };
    let adopt = zero_rigid && traced.is_some();
    if !adopt {
        pipeline::solve(
            world.gravity,
            world.integration_parameters,
            ref world.bodies,
            ref world.narrow_phase,
            ref world.impulse_joints,
        );
    } else if let Some((m, (b, c, _))) = traced {
        let mut solved = array![m];
        c.writeback_impulses(ref solved);
        let mut pair = world.narrow_phase.pairs.pop_front().unwrap();
        pair.manifold = *solved.at(0);
        world.narrow_phase.pairs = array![pair];
        let mut rb = world.body(body_handle(1)).unwrap();
        rb.pos.next_position = b.position;
        rb.vels = RigidBodyVelocity { linvel: b.linvel, angvel: b.angvel };
        assert!(world.set_body(body_handle(1), rb));
    }
    pipeline::advance_to_final_positions(
        ref world.bodies, ref world.colliders, world.integration_parameters,
    );
    if let Some((_, (b, _, _))) = traced {
        // Engine mode: the cross-check; counterfactual: the write-back reached the body.
        let rb = world.body(body_handle(1)).unwrap();
        assert_eq!(rb.position(), b.position);
        assert_eq!(rb.linvel(), b.linvel);
        assert_eq!(rb.vels.angvel, b.angvel);
    }
    flips
}

/// Upstream samples beyond tolerance over steps 1–10 per scene and mode (engine, zero-gap
/// rigid counterfactual), and the counterfactual rows it needed.
#[test]
fn test_slope_first_steps() {
    for (scene, want_engine, want_cf) in array![
        (scenes::BOX_SLOPE_STICK, 0_u32, 0_u32), (scenes::BOX_SLOPE_SLIDE, 5, 0),
    ] {
        for zero_rigid in array![false, true] {
            println!("scene {} zero_rigid {}", scene.id, zero_rigid);
            let mut world = build_world(scene);
            let mut stats: Stats = Default::default();
            let mut flips = 0;
            let mut step = 1_u32;
            while step != 11 {
                let n = diagnostic_step(ref world, zero_rigid, step == 3 || step == 4);
                if n != 0 {
                    println!("step {}: {} zero-gap rows solved rigidly", step, n);
                }
                flips += n;
                stats = compare(ref world, scene, *scene.samples.span().at(step), stats, true);
                if step == 3 {
                    assert_eq!(stats.violations, 0, "first contact within tolerance");
                    assert_first_geometry(*world.narrow_phase.pairs.at(0).manifold);
                }
                step += 1;
            }
            println!("violations {} zero-gap rows {}", stats.violations, flips);
            assert_eq!(stats.violations, if zero_rigid {
                want_cf
            } else {
                want_engine
            });
        }
    }
}

/// Upstream step 3 in scenes.json, identical for both materials. Since GS the references carry
/// the corrected f64 feature ids, equal to the port's.
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

/// The whole slide trace under the zero-gap counterfactual: every sample within tolerance.
#[test]
fn test_slide_zero_gap_counterfactual() {
    let scene = scenes::BOX_SLOPE_SLIDE;
    let mut world = build_world(scene);
    let mut stats: Stats = Default::default();
    let mut flips = 0;
    let mut next = 1;
    let mut step = 1_u32;
    while step != 121 {
        flips += diagnostic_step(ref world, true, false);
        if *scene.samples.span().at(next).step == step {
            stats = compare(ref world, scene, *scene.samples.span().at(next), stats, true);
            next += 1;
        }
        step += 1;
    }
    println!(
        "zero-gap rows {}; max ulps tx {} ty {} re {} im {} vx {} vy {} w {}",
        flips,
        stats.tx.ulps,
        stats.ty.ulps,
        stats.re.ulps,
        stats.im.ulps,
        stats.vx.ulps,
        stats.vy.ulps,
        stats.w.ulps,
    );
    assert!(flips != 0);
    assert_eq!(stats.violations, 0, "zero-gap counterfactual recovers all samples");
}
