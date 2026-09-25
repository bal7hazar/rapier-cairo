//! KD: controlled kinematic targets, upstream replay, wake-ups, and paired step probes.
use fixed::{HALF, ONE, ZERO};
use rapier_core::rigid_body::RigidBodyType;
use rapier_dynamics2d::collider::ColliderBuilderTrait;
use rapier_dynamics2d::joint::RevoluteJointBuilderTrait;
use rapier_dynamics2d::rigid_body_set::RigidBodyBuilderTrait;
use rapier_golden::types::KinematicSceneCase;
use rapier_testing::opaque;
use super::*;

fn world(case: KinematicSceneCase) -> World {
    let mut w = build_world(case.scene);
    if case.kinematic_body < case.scene.num_bodies {
        let h = body_handle(case.kinematic_body);
        let mut body = w.body(h).unwrap();
        body
            .body_type =
                if case.position_based {
                    RigidBodyType::KinematicPositionBased
                } else {
                    RigidBodyType::KinematicVelocityBased
                };
        body.mprops = body.mprops.update_world_mass_properties(body.body_type, body.position());
        body.set_linvel(vr(case.velocity));
        w.set_body(h, body);
    }
    if case.dominance_body < case.scene.num_bodies {
        let h = body_handle(case.dominance_body);
        let mut body = w.body(h).unwrap();
        body.set_dominance_group(case.dominance_group);
        w.set_body(h, body);
    }
    w
}

fn target(ref w: World, case: KinematicSceneCase, step: u32) {
    if case.position_based {
        let h = body_handle(case.kinematic_body);
        let mut body = w.body(h).unwrap();
        let initial = pose(*case.scene.bodies.span().at(case.kinematic_body).pose);
        let n: i64 = step.into();
        body
            .set_next_kinematic_translation(
                Vec2 {
                    x: initial.translation.x + f(case.target_delta.x * n),
                    y: initial.translation.y + f(case.target_delta.y * n),
                },
            );
        w.set_body(h, body);
    }
}

fn replay(case: KinematicSceneCase, start: u32, end: u32) {
    let mut w = world(case);
    let mut step = start;
    if start != 0 {
        for sample in case.scene.samples.span() {
            if *sample.step == start {
                reseed(ref w, case.scene, *sample);
            }
        }
    }
    if start == 60 && case.scene.id == 'dominance_stack' {
        seed_dominance(ref w);
    }
    let mut stats = Default::default();
    for sample in case.scene.samples.span() {
        if *sample.step < start || (*sample.step == start && start != 0) {
            continue;
        }
        if *sample.step > end {
            break;
        }
        while step != *sample.step {
            step += 1;
            target(ref w, case, step);
            let expected = if case.position_based {
                w.body(body_handle(case.kinematic_body)).unwrap().next_position()
            } else {
                Default::default()
            };
            w.step();
            if case.position_based {
                assert_eq!(w.body(body_handle(case.kinematic_body)).unwrap().position(), expected);
            }
            assert_fixed_bodies_still(ref w, case.scene, step);
        }
        stats = compare(ref w, case.scene, *sample, stats, true);
    }
    assert_eq!(stats.violations, 0);
}
#[test]
fn test_kinematic_platform() {
    replay(scenes::kinematic_platform::KINEMATIC_PLATFORM, 0, 120);
}
#[test]
fn test_kinematic_pusher() {
    replay(scenes::kinematic_pusher::KINEMATIC_PUSHER, 0, 60);
}
#[test]
fn test_dominance_stack() {
    replay(scenes::dominance_stack::DOMINANCE_STACK, 0, 60);
}

// Measure the first step on identical opaque input worlds; setup excludes target updates.
fn probe(case: KinematicSceneCase, step: bool) {
    let case = opaque(case);
    let mut w = world(case);
    target(ref w, case, 1);
    let mut pending = step;
    while pending {
        w.step();
        pending = false;
    }
    let _ = opaque(w.body(body_handle(1)).unwrap());
}
#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}
#[test]
fn gas_setup_platform() {
    probe(scenes::kinematic_platform::KINEMATIC_PLATFORM, false);
}
#[test]
fn gas_step_platform() {
    probe(scenes::kinematic_platform::KINEMATIC_PLATFORM, true);
}
#[test]
fn gas_setup_pusher() {
    probe(scenes::kinematic_pusher::KINEMATIC_PUSHER, false);
}
#[test]
fn gas_step_pusher() {
    probe(scenes::kinematic_pusher::KINEMATIC_PUSHER, true);
}
#[test]
fn gas_setup_dominance() {
    probe(scenes::dominance_stack::DOMINANCE_STACK, false);
}
#[test]
fn gas_step_dominance() {
    probe(scenes::dominance_stack::DOMINANCE_STACK, true);
}

/// Settled touching pairs sleep together; either kinematic motion mode wakes its partner.
#[test]
fn test_moving_kinematic_wakes_sleeping_partner() {
    for position_based in array![false, true] {
        let mut w = WorldTrait::new(Vec2 { x: ZERO, y: ZERO }, Default::default());
        let initial = Pose2 { translation: Vec2 { x: -ONE, y: ZERO }, ..Default::default() };
        let driver = if position_based {
            RigidBodyTrait::kinematic_position_based(initial)
        } else {
            RigidBodyTrait::kinematic_velocity_based(initial)
        };
        let collider = ColliderBuilderTrait::cuboid(HALF, HALF).build();
        let (a, _) = w.insert(driver, collider);
        let (b, _) = w.insert(RigidBodyTrait::dynamic(Default::default()), collider);
        w.step();
        for h in array![a, b] {
            let mut rb = w.body(h).unwrap();
            rb.sleep();
            w.set_body(h, rb);
        }
        w.step();
        assert!(w.body(b).unwrap().is_sleeping());
        let mut rb = w.body(a).unwrap();
        if position_based {
            rb
                .set_next_kinematic_translation(
                    Vec2 { x: -ONE + w.integration_parameters.dt, y: ZERO },
                );
        } else {
            rb.set_linvel(Vec2 { x: ONE, y: ZERO });
        }
        w.set_body(a, rb);
        w.step();
        assert!(!w.body(b).unwrap().is_sleeping());
        assert!(
            abs_diff(w.body(b).unwrap().linvel().x.raw, 4315799862) <= 8192,
            "upstream wake-step velocity",
        );
    }
}

#[test]
fn test_position_target_rotation_offset_com_and_zero_dt() {
    let mut w = WorldTrait::new(Vec2 { x: ZERO, y: -ONE }, Default::default());
    let (h, _) = w
        .insert(
            RigidBodyTrait::kinematic_position_based(Default::default()),
            ColliderBuilderTrait::ball(HALF)
                .position(Pose2 { translation: Vec2 { x: ONE, y: ZERO }, ..Default::default() })
                .build(),
        );
    let goal = Pose2 {
        translation: Vec2 { x: ONE, y: -ONE }, rotation: Rot2 { re: ZERO, im: ONE },
    };
    let mut b = w.body(h).unwrap();
    b.set_next_kinematic_position(goal);
    b.add_force(Vec2 { x: ONE, y: ONE }, true);
    w.set_body(h, b);
    w.step();
    assert_eq!(w.body(h).unwrap().position(), goal);
    w.step();
    assert_eq!(w.body(h).unwrap().position(), goal);
    assert_eq!(w.body(h).unwrap().vels, Default::default());
    w.integration_parameters.dt = ZERO;
    b = w.body(h).unwrap();
    b.set_next_kinematic_translation(Vec2 { x: ONE, y: ONE });
    w.set_body(h, b);
    w.step();
    assert_eq!(w.body(h).unwrap().vels, Default::default());
}

#[test]
fn test_dominance_builder_and_setter_direction() {
    for group in array![-1_i8, 0_i8, 1_i8] {
        let mut w = WorldTrait::new(Vec2 { x: ZERO, y: ZERO }, Default::default());
        let a = RigidBodyBuilderTrait::dynamic().dominance_group(group).build();
        let mut b = RigidBodyTrait::dynamic(
            Pose2 { translation: Vec2 { x: ONE, y: ZERO }, ..Default::default() },
        );
        b.set_dominance_group(0);
        let (_, ca) = w.insert(a, ColliderBuilderTrait::cuboid(HALF, HALF).build());
        let (_, cb) = w.insert(b, ColliderBuilderTrait::cuboid(HALF, HALF).build());
        w.step();
        assert_eq!(w.contact_pair(ca, cb).unwrap().manifold.data.relative_dominance, group.into());
    }
}

/// Joints see the same interpolated velocity as contact rows (no joint implementation changes).
#[test]
fn test_position_kinematic_drives_joint_partner() {
    let mut w = WorldTrait::new(Vec2 { x: ZERO, y: ZERO }, Default::default());
    let a = w.insert_body(RigidBodyTrait::kinematic_position_based(Default::default()));
    let (b, _) = w
        .insert(
            RigidBodyTrait::dynamic(
                Pose2 { translation: Vec2 { x: ONE, y: ZERO }, ..Default::default() },
            ),
            ColliderBuilderTrait::ball(HALF).build(),
        );
    w
        .insert_impulse_joint(
            a, b, RevoluteJointBuilderTrait::new().local_anchor2(Vec2 { x: -ONE, y: ZERO }).build(),
        );
    let mut rb = w.body(a).unwrap();
    let goal = Pose2 {
        translation: Vec2 { x: w.integration_parameters.dt, y: ZERO }, ..Default::default(),
    };
    rb.set_next_kinematic_position(goal);
    w.set_body(a, rb);
    w.step();
    assert_eq!(w.body(a).unwrap().position(), goal);
    assert!(w.body(b).unwrap().linvel().x > ZERO);
}

// Two windows respect snforge's default VM step budget; the second starts at the
// upstream step-60 pose/velocity, as the existing stack replay does.
#[test]
fn test_kinematic_pusher_second_window() {
    replay(scenes::kinematic_pusher::KINEMATIC_PUSHER, 60, 120);
}

#[test]
fn test_dominance_stack_second_window() {
    replay(scenes::dominance_stack::DOMINANCE_STACK, 60, 120);
}

fn seed_dominance(ref w: World) {
    rapier2d::pipeline::handle_user_changes(
        ref w.bodies, ref w.colliders, w.narrow_phase.pairs.span(),
    );
    rapier2d::pipeline::detect_collisions(
        w.integration_parameters, ref w.bodies, ref w.colliders, ref w.narrow_phase,
    );
    let seed = scenes::dominance_stack::warmstart::WARMSTART_60.span();
    let mut matched = 0;
    let mut pairs = array![];
    for pair in w.narrow_phase.pairs.span() {
        let mut pair = *pair;
        let [mut p0, mut p1] = pair.manifold.points;
        for up in seed {
            if *up.collider1 == pair.collider1.index && *up.collider2 == pair.collider2.index {
                let point = rapier_geometry2d::contact::TrackedContact {
                    local_p1: vr(*up.local_p1),
                    local_p2: vr(*up.local_p2),
                    dist: f(*up.dist),
                    fid1: rapier_geometry2d::feature_id::FeatureId { packed: *up.fid1 },
                    fid2: rapier_geometry2d::feature_id::FeatureId { packed: *up.fid2 },
                    data: rapier_geometry2d::contact::ContactData {
                        impulse: f(*up.impulse),
                        tangent_impulse: f(*up.tangent_impulse),
                        warmstart_impulse: f(*up.warmstart_impulse),
                        warmstart_tangent_impulse: f(*up.warmstart_tangent_impulse),
                    },
                };
                if p0.fid1 == point.fid1 && p0.fid2 == point.fid2 {
                    p0 = point;
                    matched += 1;
                } else if p1.fid1 == point.fid1 && p1.fid2 == point.fid2 {
                    p1 = point;
                    matched += 1;
                }
                pair.manifold.local_n1 = vr(*up.local_n1);
                pair.manifold.local_n2 = vr(*up.local_n2);
            }
        }
        pair.manifold.points = [p0, p1];
        pairs.append(pair);
    }
    w.narrow_phase.pairs = pairs;
    assert_eq!(matched, seed.len());
}
