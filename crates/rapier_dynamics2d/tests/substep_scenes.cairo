//! Public DF driver golden replays with analytic manifolds (GG is not required).
use fixed::wide::dot2;
use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_core::Handle;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_dynamics2d::joint::{ImpulseJoint, RevoluteJointBuilderTrait};
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet, RigidBodySetTrait, RigidBodyTrait};
use rapier_dynamics2d::solver::body::SolverBody;
use rapier_dynamics2d::solver::body_store::SolverBodyStoreTrait;
use rapier_dynamics2d::solver::island::solve_island;
use rapier_geometry2d::contact::{
    ContactManifold, ContactManifoldTrait, NEW_CONTACT_BIT, SolverContact, SolverFlags,
};
use rapier_geometry2d::mass::MassPropertiesTrait;
use rapier_golden::compare::within;
use rapier_golden::scenes;
use rapier_golden::types::{BodyKindRaw, SceneCase, ShapeRaw};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::{Rot2, Rot2Trait};
use rapier_testing::opaque;

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}
fn scale(a: Vec2, s: Fixed) -> Vec2 {
    a.mul_scalar(s)
}
fn dot(a: Vec2, b: Vec2) -> Fixed {
    dot2(a.x, b.x, a.y, b.y)
}
fn h(index: u32) -> Handle {
    Handle { index, generation: 0 }
}
fn f(raw: i64) -> Fixed {
    Fixed { raw }
}
fn params() -> IntegrationParameters {
    Default::default()
}
fn solver_body(rb: RigidBody, handle: Handle) -> SolverBody {
    SolverBody {
        handle,
        position: rb.pos.position,
        linvel: rb.vels.linvel,
        angvel: rb.vels.angvel,
        im: rb.mprops.effective_inv_mass,
        ii: rb.mprops.effective_world_inv_inertia,
    }
}
fn setup(scene: SceneCase) -> (RigidBodySet, Array<ImpulseJoint>) {
    let mut bodies = RigidBodySetTrait::new();
    let mut i = 0;
    while i != scene.num_bodies {
        let desc = *scene.bodies.span().at(i);
        let pose = Pose2 {
            translation: v(f(desc.pose.translation.x), f(desc.pose.translation.y)),
            rotation: Rot2 { re: f(desc.pose.rotation.re), im: f(desc.pose.rotation.im) },
        };
        let mut rb = if desc.kind == BodyKindRaw::Fixed {
            RigidBodyTrait::fixed(pose)
        } else {
            RigidBodyTrait::dynamic(pose)
        };
        if desc.num_colliders != 0 {
            let co = *desc.colliders.span().at(0);
            rb.mprops.local_mprops = match co.shape {
                ShapeRaw::Ball(radius) => MassPropertiesTrait::from_ball(f(co.density), f(radius)),
                ShapeRaw::Cuboid(half) => MassPropertiesTrait::from_cuboid(
                    f(co.density), v(f(half.x), f(half.y)),
                ),
                _ => Default::default(),
            };
        }
        rb.damping.linear_damping = f(desc.linear_damping);
        rb.damping.angular_damping = f(desc.angular_damping);
        rb.forces.gravity_scale = f(desc.gravity_scale);
        let _ = bodies.insert(rb);
        i += 1;
    }
    let mut js = array![];
    i = 0;
    while i != scene.num_joints {
        let j = *scene.joints.span().at(i);
        js
            .append(
                ImpulseJoint {
                    body1: h(j.body1),
                    body2: h(j.body2),
                    data: RevoluteJointBuilderTrait::new()
                        .local_anchor1(v(f(j.local_anchor1.x), f(j.local_anchor1.y)))
                        .local_anchor2(v(f(j.local_anchor2.x), f(j.local_anchor2.y)))
                        .build(),
                    impulses: [ZERO, ZERO, ZERO],
                },
            );
        i += 1;
    }
    (bodies, js)
}
fn step(
    ref bodies: RigidBodySet,
    ref ms: Array<ContactManifold>,
    ref js: Array<ImpulseJoint>,
    p: IntegrationParameters,
    gravity: Vec2,
) {
    let (mut store, _) = SolverBodyStoreTrait::from_bodies(ref bodies, gravity, p);
    solve_island(p, ref store, ref ms, ref js);
    store.to_bodies(ref bodies);
    // The future pipeline owns this stage, after constraints and CCD.
    for (handle, mut rb) in bodies.iter() {
        rb.set_position(rb.pos.next_position);
        assert!(bodies.set(handle, rb));
    }
}
fn ball_manifold(previous: ContactManifold, b: SolverBody, restitution: Fixed) -> ContactManifold {
    let dist = b.position.translation.y - HALF;
    let mut prediction = params().prediction_distance();
    let velocity_prediction = -b.linvel.y * params().dt;
    if velocity_prediction > prediction {
        prediction = velocity_prediction;
    }
    if dist > prediction {
        return Default::default();
    }
    let mut m = previous;
    m.num_points = 1;
    m.data.num_solver_contacts = 1;
    m.data.solver_flags = SolverFlags { bits: 1 };
    m.data.rigid_body2 = Some(b.handle);
    m.data.normal = v(ZERO, ONE);
    m.data.restitution = restitution;
    // Per-surface witnesses; the solver builds the common midpoint lever arms itself.
    m
        .data
        .solver_contacts =
            [
                SolverContact {
                    anchor1: v(b.position.translation.x, ZERO),
                    anchor2: v(ZERO, -HALF),
                    dist,
                    contact_id: if previous.num_points == 0 {
                        NEW_CONTACT_BIT
                    } else {
                        0
                    },
                    ..Default::default(),
                },
                Default::default(),
            ];
    m
}
// Match the golden harness's temporal coherence and documented f64 feature-id collision.
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
            // Per-surface witnesses; the solver builds the common midpoint lever arms itself.
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

fn sample(scene: SceneCase, sample_id: u32, ref bodies: RigidBodySet) {
    let sample = *scene.samples.span().at(sample_id);
    let mut i = 0;
    while i != scene.num_dynamic {
        let expected = *sample.states.span().at(i);
        let rb = bodies.get(h(expected.body)).unwrap();
        let tol: u64 = 4096 * sample.step.into();
        for (got, want, budget) in [
            (rb.pos.position.translation.x.raw, expected.translation.x, tol),
            (rb.pos.position.translation.y.raw, expected.translation.y, tol),
            (rb.pos.position.rotation.re.raw, expected.rotation.re, tol),
            (rb.pos.position.rotation.im.raw, expected.rotation.im, tol),
            (rb.vels.linvel.x.raw, expected.linvel.x, 2 * tol),
            (rb.vels.linvel.y.raw, expected.linvel.y, 2 * tol),
            (rb.vels.angvel.raw, expected.angvel, 2 * tol),
        ]
            .span() {
            assert!(
                within(*got, *want, *budget),
                "{} step {} got {} want {}",
                scene.id,
                sample.step,
                *got,
                *want,
            );
        }
        i += 1;
    }
}
fn replay(scene: SceneCase) {
    let (mut bodies, mut js) = setup(scene);
    let mut m = Default::default();
    let desc = *scene.bodies.span().at(1);
    let co = *desc.colliders.span().at(0);
    let normal = Rot2 { re: f(desc.pose.rotation.re), im: f(desc.pose.rotation.im) }
        .rotate(v(ZERO, ONE));
    let p = IntegrationParameters { dt: f(scene.dt), ..Default::default() };
    let gravity = v(f(scene.gravity.x), f(scene.gravity.y));
    let mut frame = 0;
    let mut sample_id = 0;
    let mut bounced = false;
    let mut first_impact = 0;
    let mut apex = ZERO;
    let mut last_vy = ZERO;
    sample(scene, sample_id, ref bodies);
    sample_id += 1;
    while frame != scene.num_steps {
        let b = solver_body(bodies.get(h(1)).unwrap(), h(1));
        m =
            if scene.num_joints != 0 {
                Default::default()
            } else if scene.id == 'ball_drop' || scene.id == 'ball_bounce' {
                ball_manifold(m, b, f(co.restitution))
            } else {
                box_manifold(m, b, normal, HALF, f(co.friction), false)
            };
        let mut ms = array![m];
        step(ref bodies, ref ms, ref js, p, gravity);
        m = *ms.at(0);
        frame += 1;
        let rb = bodies.get(h(1)).unwrap();
        if scene.id == 'ball_bounce' && !bounced && rb.vels.linvel.y > ZERO && last_vy < ZERO {
            bounced = true;
            first_impact = frame;
        }
        if bounced && rb.pos.position.translation.y > apex {
            apex = rb.pos.position.translation.y;
        }
        last_vy = rb.vels.linvel.y;
        if frame == *scene.samples.span().at(sample_id).step {
            if !bounced {
                sample(scene, sample_id, ref bodies);
            }
            sample_id += 1;
        }
    }
    assert_eq!(sample_id, 22);
    if scene.id == 'ball_bounce' {
        // The first upward golden sample brackets impact to its preceding sample interval.
        let mut previous_step = 0;
        let mut expected_apex = ZERO;
        let mut first_upward = 0;
        for s in scene.samples.span() {
            let e = *s.states.span().at(0);
            if e.linvel.y > 0 && first_upward == 0 {
                assert!(first_impact > previous_step && first_impact <= *s.step);
                first_upward = *s.step;
            }
            if first_upward != 0 && f(e.translation.y) > expected_apex {
                expected_apex = f(e.translation.y);
            }
            previous_step = *s.step;
        }
        assert!(bounced);
        // Samples are ten steps apart: maximum apex undersampling is |g| (10 dt)^2 / 8.
        let interval = p.dt * FixedTrait::from_int(10);
        let sample_error = -gravity.y * interval * interval / FixedTrait::from_int(8);
        let rounding = f(4096 * scene.num_steps.into());
        assert!(apex + rounding >= expected_apex);
        assert!(apex - expected_apex <= sample_error + rounding);
    }
}
#[test]
fn test_ball_drop_contact_phase_and_all_golden_samples() {
    replay(scenes::BALL_DROP);
}
#[test]
fn test_ball_bounce_samples_impact_and_apex() {
    replay(scenes::BALL_BOUNCE);
}
#[test]
fn test_slope_stick_all_golden_samples() {
    replay(scenes::BOX_SLOPE_STICK);
}
#[test]
fn test_slope_slide_all_golden_samples() {
    replay(scenes::BOX_SLOPE_SLIDE);
}
#[test]
fn test_pendulum_all_golden_samples() {
    replay(scenes::PENDULUM);
}

fn stack_manifold(
    previous: ContactManifold, lower: Option<SolverBody>, b: SolverBody,
) -> ContactManifold {
    let mut m = previous;
    let (normal, lower_pos, surface, lower_handle) = match lower {
        Some(a) => (
            a.position.rotation.rotate(v(ZERO, ONE)),
            a.position.translation,
            a.position.translation + a.position.rotation.rotate(v(ZERO, HALF)),
            Some(a.handle),
        ),
        None => (v(ZERO, ONE), v(ZERO, ZERO), v(ZERO, ZERO), None),
    };
    m.num_points = 2;
    m.data.num_solver_contacts = 2;
    m.data.normal = normal;
    m.data.rigid_body1 = lower_handle;
    m.data.rigid_body2 = Some(b.handle);
    m.data.friction = HALF;
    m.data.solver_flags = SolverFlags { bits: 1 };
    let mut contacts = array![];
    let mut i: u8 = 0;
    while i != 2 {
        let corner = v(if i == 0 {
            HALF
        } else {
            -HALF
        }, -HALF);
        let world = b.position.translation + b.position.rotation.rotate(corner);
        let dist = dot(world - surface, normal);
        // Per-surface witnesses; the solver builds the common midpoint lever arms itself.
        let witness1 = world - scale(normal, dist);
        let id: u32 = i.into();
        contacts
            .append(
                SolverContact {
                    anchor1: witness1 - lower_pos,
                    anchor2: world - b.position.translation,
                    dist,
                    contact_id: if previous.num_points == 0 {
                        id + NEW_CONTACT_BIT
                    } else {
                        id
                    },
                    ..Default::default(),
                },
            );
        i += 1;
    }
    m.data.solver_contacts = [*contacts.at(0), *contacts.at(1)];
    m
}
// Two independent 60-step windows fit snforge's default VM budget. The second starts from
// the upstream step-60 sample (cold contact cache); only invariants are compared for stacks.
// The continuous 120-step replay was also validated with --max-n-steps 50000000.
fn stack_replay(start: u32) {
    let scene = scenes::BOX_STACK3;
    let (mut bodies, mut js) = setup(scene);
    if start != 0 {
        for sample in scene.samples.span() {
            if *sample.step == start {
                for state in sample.states.span() {
                    let mut rb = bodies.get(h(*state.body)).unwrap();
                    rb
                        .pos
                        .position =
                            Pose2 {
                                translation: v(f(*state.translation.x), f(*state.translation.y)),
                                rotation: Rot2 {
                                    re: f(*state.rotation.re), im: f(*state.rotation.im),
                                },
                            };
                    rb.pos.next_position = rb.pos.position;
                    rb.vels.linvel = v(f(*state.linvel.x), f(*state.linvel.y));
                    rb.vels.angvel = f(*state.angvel);
                    assert!(bodies.set(h(*state.body), rb));
                }
            }
        }
    }
    let mut ms: Array<ContactManifold> = array![
        Default::default(), Default::default(), Default::default(),
    ];
    let mut frame = 0;
    while frame != 60 {
        let mut next = array![];
        let mut i = 1;
        while i != 4 {
            let lower = if i == 1 {
                None
            } else {
                Some(solver_body(bodies.get(h(i - 1)).unwrap(), h(i - 1)))
            };
            next
                .append(
                    stack_manifold(
                        *ms.at(i - 1), lower, solver_body(bodies.get(h(i)).unwrap(), h(i)),
                    ),
                );
            i += 1;
        }
        ms = next;
        step(ref bodies, ref ms, ref js, params(), v(ZERO, f(scene.gravity.y)));
        frame += 1;
    }
    let mut i = 1;
    while i != 4 {
        let rb = bodies.get(h(i)).unwrap();
        let height = FixedTrait::from_int((i - 1).try_into().unwrap()) + HALF;
        assert!((rb.pos.position.translation.y - height).abs() < params().allowed_linear_error());
        assert!(rb.pos.position.translation.x.abs() < f(42949673));
        assert!(rb.vels.linvel.length() < f(4294967));
        assert!(rb.vels.angvel.abs() < f(4294967));
        i += 1;
    }
}
#[test]
fn test_box_stack3_golden_rest_first_window() {
    stack_replay(0);
}
#[test]
fn test_box_stack3_golden_rest_second_window() {
    stack_replay(60);
}
#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}
fn bench(scene: SceneCase) {
    let (mut bodies, mut js) = setup(scene);
    let mut b = bodies.get(h(1)).unwrap();
    if scene.id == 'ball_drop' || scene.id == 'ball_bounce' {
        b.pos.position.translation.y = HALF;
        b.vels.linvel.y = -ONE;
        assert!(bodies.set(h(1), b));
    }
    let co = *scene.bodies.span().at(1).colliders.span().at(0);
    let b = solver_body(b, h(1));
    let m = if scene.num_joints != 0 {
        Default::default()
    } else if scene.id == 'ball_drop' || scene.id == 'ball_bounce' {
        ball_manifold(Default::default(), b, f(co.restitution))
    } else {
        box_manifold(
            Default::default(),
            b,
            b.position.rotation.rotate(v(ZERO, ONE)),
            HALF,
            f(co.friction),
            true,
        )
    };
    let mut ms = array![opaque(m)];
    step(ref bodies, ref ms, ref js, opaque(params()), opaque(v(ZERO, f(scene.gravity.y))));
    let _ = opaque(bodies.get(h(1)).unwrap());
}
#[test]
fn gas_ball_drop_contact_step() {
    bench(scenes::BALL_DROP);
}
#[test]
fn gas_ball_bounce_contact_step() {
    bench(scenes::BALL_BOUNCE);
}
#[test]
fn gas_box_slope_stick_step() {
    bench(scenes::BOX_SLOPE_STICK);
}
#[test]
fn gas_box_slope_slide_step() {
    bench(scenes::BOX_SLOPE_SLIDE);
}
#[test]
fn gas_pendulum_step() {
    bench(scenes::PENDULUM);
}
