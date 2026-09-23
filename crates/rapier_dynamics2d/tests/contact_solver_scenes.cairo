//! Mock-manifold cross-module scenes, with the upstream four-substep stage order.
//! Geometry is analytic: unit balls and bottom corners of a unit box against an infinite plane.
use fixed::wide::dot2;
use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam::Vec2;
use rapier_core::data::handle::Handle;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_dynamics2d::solver::body::SolverBody;
use rapier_dynamics2d::solver::contact::ContactConstraintsSetTrait;
use rapier_geometry2d::contact::{
    ContactManifold, ContactManifoldTrait, NEW_CONTACT_BIT, SolverContact, SolverFlags,
};
use rapier_golden::compare::{abs_diff, within};
use rapier_golden::scenes;
use rapier_golden::types::SceneCase;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::{Rot2, Rot2Trait};
use rapier_testing::opaque;

const DT: Fixed = Fixed { raw: 71582788 };
const GRAVITY: Fixed = Fixed { raw: -42133629174 };
const TOL: Fixed = Fixed { raw: 4294967 }; // 0.001 for invariant checks, not golden samples.

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}
fn scale(a: Vec2, s: Fixed) -> Vec2 {
    a * v(s, s)
}
fn dot(a: Vec2, b: Vec2) -> Fixed {
    dot2(a.x, b.x, a.y, b.y)
}
fn h(index: u32) -> Handle {
    Handle { index, generation: 0 }
}
fn body(index: u32, y: Fixed) -> SolverBody {
    SolverBody {
        handle: h(index),
        position: Pose2 { translation: v(ZERO, y), ..Default::default() },
        im: v(ONE, ONE),
        ..Default::default(),
    }
}
fn params() -> IntegrationParameters {
    IntegrationParameters { dt: DT, ..Default::default() }
}

// The driver is deliberately test-only: forces, pose integration and velocity caps belong to DF.
fn step(ref bodies: Array<SolverBody>, ref ms: Array<ContactManifold>, gravity: Fixed) {
    let p = params();
    let dt = p.substep_dt();
    let mut cs = ContactConstraintsSetTrait::generate(ms.span(), bodies.span(), p, dt);
    let mut substep = 0;
    while substep != p.num_solver_iterations {
        let mut out = array![];
        while let Some(mut b) = bodies.pop_front() {
            b.linvel.y += gravity * dt;
            out.append(b);
        }
        bodies = out;
        cs.update(p, bodies.span(), ms.span());
        cs.warmstart(ref bodies);
        let mut i = 0;
        while i != p.num_internal_pgs_iterations {
            cs.solve(ref bodies, true, p.friction_in_bias_pass);
            i += 1;
        }
        let mut out = array![];
        while let Some(mut b) = bodies.pop_front() {
            b.position.translation = b.position.translation + scale(b.linvel, dt);
            b.position.rotation = b.position.rotation.integrate(b.angvel, dt);
            out.append(b);
        }
        bodies = out;
        let mut i = 0;
        while i != p.num_internal_stabilization_iterations {
            cs.update_rhs_wo_bias(bodies.span());
            cs.solve(ref bodies, true, true);
            i += 1;
        }
        substep += 1;
    }
    cs.apply_restitution(ref bodies);
    cs.writeback_impulses(ref ms);
}

fn ball_manifold(
    previous: ContactManifold, lower: Option<SolverBody>, upper: SolverBody, restitution: Fixed,
) -> ContactManifold {
    let mut m = previous;
    let (anchor1, d, rb1) = match lower {
        Some(b) => (
            v(ZERO, HALF),
            upper.position.translation.y - b.position.translation.y - ONE,
            Some(b.handle),
        ),
        None => (v(upper.position.translation.x, ZERO), upper.position.translation.y - HALF, None),
    };
    m.num_points = 1;
    m.data.num_solver_contacts = 1;
    m.data.solver_flags = SolverFlags { bits: 1 };
    m.data.rigid_body1 = rb1;
    m.data.rigid_body2 = Some(upper.handle);
    m.data.normal = v(ZERO, ONE);
    m.data.restitution = restitution;
    let cid = if m.point(0).data.impulse == ZERO {
        NEW_CONTACT_BIT
    } else {
        0
    };
    let sc = SolverContact {
        anchor1, anchor2: v(ZERO, -HALF), dist: d, contact_id: cid, ..Default::default(),
    };
    m.data.solver_contacts = [sc, Default::default()];
    m
}

#[test]
fn test_ball_rests_for_sixty_four_substep_frames() {
    let mut bs = array![body(0, HALF)];
    let mut m = Default::default();
    let mut frame = 0;
    while frame != 60 {
        m = ball_manifold(m, None, *bs.at(0), ZERO);
        let mut ms = array![m];
        step(ref bs, ref ms, GRAVITY);
        m = *ms.at(0);
        frame += 1;
    }
    assert!((*bs.at(0).linvel.y).abs() < TOL);
    assert!((m.point(0).data.impulse + GRAVITY * DT).abs() < TOL);
    // The static soft contact settles just below the plane, without a slop dead-zone.
    assert!((*bs.at(0).position.translation.y - HALF).abs() < TOL);
}

#[test]
fn test_unit_restitution_reverses_velocity_after_substeps() {
    let mut b = body(0, HALF);
    b.linvel.y = -ONE;
    let mut bs = array![b];
    let m = ball_manifold(Default::default(), None, b, ONE);
    let mut ms = array![m];
    step(ref bs, ref ms, ZERO);
    assert!((*bs.at(0).linvel.y - ONE).abs() < FixedTrait::from_raw(32));
    assert!(
        (ms.at(0).point(0).data.impulse - FixedTrait::from_int(2)).abs() < FixedTrait::from_raw(32),
    );
}

#[test]
fn test_two_ball_stack_loads_sum() {
    let mut bs = array![body(0, HALF), body(1, ONE + HALF)];
    let mut ground = Default::default();
    let mut pair = Default::default();
    let mut frame = 0;
    while frame != 60 {
        ground = ball_manifold(ground, None, *bs.at(0), ZERO);
        pair = ball_manifold(pair, Some(*bs.at(0)), *bs.at(1), ZERO);
        let mut ms = array![ground, pair];
        step(ref bs, ref ms, GRAVITY);
        ground = *ms.at(0);
        pair = *ms.at(1);
        frame += 1;
    }
    let expected = -GRAVITY * DT;
    assert!((pair.point(0).data.impulse - expected).abs() < TOL);
    assert!((ground.point(0).data.impulse - expected - expected).abs() < TOL);
    assert!((*bs.at(0).linvel.y).abs() < TOL);
    assert!((*bs.at(1).linvel.y).abs() < TOL);
}

// The golden harness retains Parry's temporal-coherence fast path and its documented f64
// cuboid-id collision. Keep that fixture behavior in this mock, never in the solver.
fn box_manifold(
    previous: ContactManifold,
    b: SolverBody,
    normal: Vec2,
    height: Fixed,
    friction: Fixed,
    golden_matching: bool,
) -> ContactManifold {
    let mut coherent = previous.num_points == 2;
    if coherent {
        coherent =
            -dot(
                normal, b.position.rotation.rotate(previous.local_n2),
            ) >= FixedTrait::from_raw(4294313152); // cos(1 degree)
    }
    let mut i: u8 = 0;
    while i != 2 {
        let corner = v(if i == 0 {
            HALF
        } else {
            -HALF
        }, -HALF);
        let world = b.position.translation + b.position.rotation.rotate(corner);
        let pt = previous.point(i);
        let dist = dot(world - pt.local_p1, normal);
        let drift = world - scale(normal, dist) - pt.local_p1;
        if (dist < ZERO && pt.dist > ZERO)
            || (dist > ZERO && pt.dist < ZERO)
            || dot(drift, drift) > FixedTrait::from_raw(4295) {
            coherent = false;
        }
        i += 1;
    }
    let mut m = previous;
    m.num_points = 2;
    m.data.num_solver_contacts = 0;
    m.data.normal = normal;
    m.data.friction = friction;
    m.data.rigid_body2 = Some(b.handle);
    m.data.solver_flags = SolverFlags { bits: 1 };
    if !coherent {
        m.local_n2 = b.position.rotation.inverse_rotate(-normal);
    }
    let mut contacts: [SolverContact; 2] = [Default::default(), Default::default()];
    let mut i: u8 = 0;
    while i != 2 {
        let corner = v(if i == 0 {
            HALF
        } else {
            -HALF
        }, -HALF);
        let anchor = b.position.rotation.rotate(corner);
        let world = b.position.translation + anchor;
        let surface = if coherent {
            previous.point(i).local_p1
        } else {
            world - scale(normal, dot(world, normal) - height)
        };
        let dist = dot(world - surface, normal);
        let mut pt = previous.point(i);
        if golden_matching && !coherent && previous.num_points == 2 {
            pt.data = previous.point(1).data;
        }
        pt.local_p1 = surface;
        pt.local_p2 = corner;
        pt.dist = dist;
        let [a, bpt] = m.points;
        m.points = if i == 0 {
            [pt, bpt]
        } else {
            [a, pt]
        };
        if dist <= params().prediction_distance() {
            let id: u32 = i.into();
            let contact_id = if pt.data.impulse == ZERO {
                id + NEW_CONTACT_BIT
            } else {
                id
            };
            // Per-surface witnesses, as the narrow phase emits them; the solver builds the
            // common midpoint lever arms itself.
            let sc = SolverContact {
                anchor1: surface, anchor2: anchor, dist, contact_id, ..Default::default(),
            };
            let [a, b] = contacts;
            contacts = if m.data.num_solver_contacts == 0 {
                [sc, b]
            } else {
                [a, sc]
            };
            m.data.num_solver_contacts += 1;
        }
        i += 1;
    }
    m.data.solver_contacts = contacts;
    m
}

fn box_body(rotation: Rot2, normal: Vec2, distance: Fixed) -> SolverBody {
    SolverBody {
        position: Pose2 { translation: scale(normal, distance), rotation },
        ii: FixedTrait::from_int(6),
        ..body(0, ZERO),
    }
}

#[test]
fn test_half_friction_sticks_at_twenty_degrees_slides_at_forty_five() {
    // Trigonometric constants rounded from cos/sin of 20 and 45 degrees.
    let mut cases = array![
        (4035949075_i64, 1468965330_i64, true), (3037000500_i64, 3037000500_i64, false),
    ]
        .span();
    while let Some(case) = cases.pop_front() {
        let (cos, sin, sticks) = *case;
        let rotation = Rot2Trait::from_cos_sin(
            FixedTrait::from_raw(cos), FixedTrait::from_raw(sin),
        );
        let normal = rotation.rotate(v(ZERO, ONE));
        let mut bs = array![box_body(rotation, normal, HALF)];
        let initial = *bs.at(0).position.translation;
        let mut m = Default::default();
        let mut frame = 0;
        while frame != 60 {
            m = box_manifold(m, *bs.at(0), normal, ZERO, HALF, false);
            let mut ms = array![m];
            step(ref bs, ref ms, GRAVITY);
            m = *ms.at(0);
            frame += 1;
        }
        let tangent = v(normal.y, -normal.x);
        let travel = dot(*bs.at(0).position.translation - initial, tangent);
        let speed = dot(*bs.at(0).linvel, tangent);
        if sticks {
            assert!(travel.abs() < FixedTrait::from_raw(42949673)); // 1 cm transient slip.
            assert!(speed.abs() < TOL);
        } else {
            assert!(travel < -HALF);
            assert!(speed < -ONE);
        }
    }
}

fn compare_sample(b: SolverBody, scene: SceneCase, sample_id: u32) {
    let sample = *scene.samples.span().at(sample_id);
    let s = *sample.states.span().at(0);
    // Recommended tolerance in tools/golden/README.md; velocities have twice the position budget.
    let tol: u64 = sample.step.into() * 4096;
    assert!(
        within(b.position.translation.x.raw, s.translation.x, tol),
        "{} step {} x: {} vs {}",
        scene.id,
        sample.step,
        b.position.translation.x.raw,
        s.translation.x,
    );
    assert!(
        within(b.position.translation.y.raw, s.translation.y, tol),
        "{} step {} y: {} vs {}",
        scene.id,
        sample.step,
        b.position.translation.y.raw,
        s.translation.y,
    );
    assert!(
        within(b.linvel.x.raw, s.linvel.x, 2 * tol),
        "{} step {} vx: {} vs {}",
        scene.id,
        sample.step,
        b.linvel.x.raw,
        s.linvel.x,
    );
    assert!(
        within(b.linvel.y.raw, s.linvel.y, 2 * tol),
        "{} step {} vy: {} vs {}",
        scene.id,
        sample.step,
        b.linvel.y.raw,
        s.linvel.y,
    );
    assert!(within(b.position.rotation.re.raw, s.rotation.re, tol));
    assert!(within(b.position.rotation.im.raw, s.rotation.im, tol));
    assert!(within(b.angvel.raw, s.angvel, 2 * tol));
}

#[test]
fn test_box_slope_golden_samples() {
    let mut cases = array![scenes::BOX_SLOPE_STICK, scenes::BOX_SLOPE_SLIDE].span();
    while let Some(scene) = cases.pop_front() {
        let s = *scene;
        let description = *s.bodies.span().at(1);
        let rotation = Rot2 {
            re: FixedTrait::from_raw(description.pose.rotation.re),
            im: FixedTrait::from_raw(description.pose.rotation.im),
        };
        let normal = rotation.rotate(v(ZERO, ONE));
        let mut b = box_body(rotation, normal, ONE);
        b
            .position
            .translation =
                v(
                    FixedTrait::from_raw(description.pose.translation.x),
                    FixedTrait::from_raw(description.pose.translation.y),
                );
        let friction = FixedTrait::from_raw(*description.colliders.span().at(0).friction);
        let mut bs = array![b];
        let mut m = Default::default();
        let mut frame = 0;
        let mut sample = 0;
        compare_sample(b, s, sample);
        sample += 1;
        while frame != s.num_steps {
            m = box_manifold(m, *bs.at(0), normal, HALF, friction, true);
            let mut ms = array![m];
            step(ref bs, ref ms, FixedTrait::from_raw(s.gravity.y));
            m = *ms.at(0);
            frame += 1;
            if frame == *s.samples.span().at(sample).step {
                compare_sample(*bs.at(0), s, sample);
                sample += 1;
            }
        }
        assert_eq!(sample, 22);
    }
}

// SD's first-impulse step (tools/golden/README.md): upstream's common-midpoint lever arms
// recover the step-3 velocities; separate witness arms missed by 0.5M to 4.5M ulp.
#[test]
fn test_box_slope_first_contact_velocities() {
    let mut cases = array![(scenes::BOX_SLOPE_STICK, 1000_u64), (scenes::BOX_SLOPE_SLIDE, 2000)]
        .span();
    while let Some((scene, bound)) = cases.pop_front() {
        let s = *scene;
        let description = *s.bodies.span().at(1);
        let rotation = Rot2 {
            re: FixedTrait::from_raw(description.pose.rotation.re),
            im: FixedTrait::from_raw(description.pose.rotation.im),
        };
        let normal = rotation.rotate(v(ZERO, ONE));
        let mut b = box_body(rotation, normal, ONE);
        b
            .position
            .translation =
                v(
                    FixedTrait::from_raw(description.pose.translation.x),
                    FixedTrait::from_raw(description.pose.translation.y),
                );
        let friction = FixedTrait::from_raw(*description.colliders.span().at(0).friction);
        let mut bs = array![b];
        let mut m = Default::default();
        let mut frame = 0;
        while frame != 3 {
            m = box_manifold(m, *bs.at(0), normal, HALF, friction, true);
            let mut ms = array![m];
            step(ref bs, ref ms, FixedTrait::from_raw(s.gravity.y));
            m = *ms.at(0);
            frame += 1;
        }
        let sample = *s.samples.span().at(3);
        assert_eq!(sample.step, 3);
        let expected = *sample.states.span().at(0);
        let got = *bs.at(0);
        let errors = array![
            abs_diff(got.linvel.x.raw, expected.linvel.x),
            abs_diff(got.linvel.y.raw, expected.linvel.y),
            abs_diff(got.angvel.raw, expected.angvel),
        ];
        println!("{} step 3 velocity ulps {:?}", s.id, errors);
        for e in errors.span() {
            assert!(*e <= *bound, "{} step 3 velocity error {} > {}", s.id, *e, *bound);
        }
    }
}

#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}
#[test]
fn gas_one_manifold_four_substeps() {
    let b = opaque(body(0, HALF));
    let mut bs = array![b];
    let mut ms = array![ball_manifold(Default::default(), None, b, ZERO)];
    step(ref bs, ref ms, opaque(GRAVITY));
    let _ = opaque((*bs.at(0), *ms.at(0)));
}
