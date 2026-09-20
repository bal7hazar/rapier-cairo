//! Sanity checks of the scene fixtures: shape of the tables, body order, and the physics that can
//! be verified in closed form (free fall, rest heights). They guard the harness (wrong index,
//! wrong unit, wrong sign), not the port.

use rapier_golden::compare::within;
use rapier_golden::generated::{integration_parameters, scenes};
use rapier_golden::types::{BodyKindRaw, BodyStateRaw, SceneBodyRaw, SceneCase, SceneSampleRaw};

const ONE: i128 = 0x100000000;
const HALF: i64 = 0x80000000;
/// Number of samples of every scene: step 0, steps 1..=10, then every 10th step up to 120.
const NUM_SAMPLES: usize = 22;

fn scene(id: felt252) -> SceneCase {
    for case in scenes::cases() {
        if *case.id == id {
            return *case;
        }
    }
    panic!("no scene {}", id)
}

fn body(scene: @SceneCase, index: usize) -> SceneBodyRaw {
    *scene.bodies.span().at(index)
}

fn sample(scene: @SceneCase, index: usize) -> SceneSampleRaw {
    *scene.samples.span().at(index)
}

/// State of the `k`-th dynamic body in the `index`-th sample.
fn state(scene: @SceneCase, index: usize, k: usize) -> BodyStateRaw {
    *sample(scene, index).states.span().at(k)
}

fn is_dynamic(kind: BodyKindRaw) -> bool {
    match kind {
        BodyKindRaw::Dynamic => true,
        BodyKindRaw::Fixed => false,
    }
}

/// `|a| <= tolerance`.
fn small(a: i64, tolerance: u64) -> bool {
    within(a, 0, tolerance)
}

#[test]
fn test_scene_table_has_the_expected_shape() {
    let cases = scenes::cases();
    assert_eq!(cases.len(), 6);
    let ids = array![
        'ball_drop', 'ball_bounce', 'box_slope_stick', 'box_slope_slide', 'box_stack3', 'pendulum',
    ];
    let bodies = array![2_u32, 2, 2, 2, 4, 2];
    let dynamic = array![1_u32, 1, 1, 1, 3, 1];
    let joints = array![0_u32, 0, 0, 0, 0, 1];
    let mut i = 0;
    while i != 6 {
        let case = cases.at(i);
        assert_eq!(*case.id, *ids.at(i));
        assert_eq!(*case.num_bodies, *bodies.at(i), "bodies of {}", *case.id);
        assert_eq!(*case.num_dynamic, *dynamic.at(i), "dynamic bodies of {}", *case.id);
        assert_eq!(*case.num_joints, *joints.at(i), "joints of {}", *case.id);
        assert_eq!(*case.num_steps, 120);
        assert_eq!(*case.dt, 71582788);
        assert_eq!(*case.gravity.x, 0);
        assert!(*case.gravity.y < 0, "gravity points down");
        i += 1;
    }
}

#[test]
fn test_samples_follow_the_sampling_schedule() {
    for case in scenes::cases() {
        let mut i = 0;
        while i != NUM_SAMPLES {
            let expected: u32 = if i <= 10 {
                i.try_into().unwrap()
            } else {
                (i - 9).try_into().unwrap() * 10
            };
            assert_eq!(sample(case, i).step, expected, "step of sample {} of {}", i, *case.id);
            i += 1;
        }
        assert_eq!(sample(case, NUM_SAMPLES - 1).step, *case.num_steps);
    }
}

#[test]
fn test_dynamic_bodies_are_sampled_in_body_order_and_fixed_ones_never() {
    for case in scenes::cases() {
        // Indices of the dynamic bodies, in insertion order.
        let mut dynamic = array![];
        let mut i = 0;
        while i != (*case.num_bodies).try_into().unwrap() {
            if is_dynamic(body(case, i).kind) {
                dynamic.append(i);
            }
            i += 1;
        }
        assert_eq!(dynamic.len(), (*case.num_dynamic).try_into().unwrap(), "{}", *case.id);

        let mut s = 0;
        while s != NUM_SAMPLES {
            let mut k = 0;
            while k != dynamic.len() {
                let expected: u32 = (*dynamic.at(k)).try_into().unwrap();
                assert_eq!(state(case, s, k).body, expected, "{}", *case.id);
                k += 1;
            }
            s += 1;
        }
    }
}

#[test]
fn test_first_body_of_every_scene_is_fixed_and_carries_the_static_geometry() {
    for case in scenes::cases() {
        let first = body(case, 0);
        assert!(!is_dynamic(first.kind), "first body of {} is fixed", *case.id);
        // Velocity-free and unsampled: the pose recorded in the description is all there is.
        assert_eq!(first.linear_damping, 0);
        assert_eq!(first.angular_damping, 0);
    }
    // The flat ground of the three flat scenes: a 20 x 1 slab whose top face is the line y = 0.
    let flat = array!['ball_drop', 'ball_bounce', 'box_stack3'];
    for id in flat.span() {
        let case = scene(*id);
        let ground = body(@case, 0);
        assert_eq!(ground.pose.translation.x, 0);
        assert_eq!(ground.pose.translation.y, -HALF);
        assert_eq!(ground.pose.rotation.re, ONE.try_into().unwrap());
        assert_eq!(ground.pose.rotation.im, 0);
        assert_eq!(ground.num_colliders, 1);
    }
}

#[test]
fn test_unused_slots_are_zeroed() {
    for case in scenes::cases() {
        let mut i: usize = (*case.num_bodies).try_into().unwrap();
        while i != 4 {
            let unused = body(case, i);
            assert_eq!(unused.name, 0);
            assert_eq!(unused.num_colliders, 0);
            assert_eq!(unused.pose.rotation.re, 0);
            i += 1;
        }
        let mut s = 0;
        while s != NUM_SAMPLES {
            let mut k: usize = (*case.num_dynamic).try_into().unwrap();
            while k != 3 {
                let unused = state(case, s, k);
                assert_eq!(unused.translation.y, 0);
                assert_eq!(unused.rotation.re, 0);
                k += 1;
            }
            s += 1;
        }
    }
}

#[test]
fn test_ball_drop_falls_strictly_then_rests_above_the_slab() {
    let case = scene('ball_drop');
    // Free fall lasts up to the sample of step 30 (the ball is still 0.26 above the prediction
    // distance of the slab there): each sample is strictly lower and faster than the previous one.
    let mut i = 1;
    while i != 21 {
        let before = state(@case, i - 1, 0);
        let after = state(@case, i, 0);
        if sample(@case, i).step > 30 {
            break;
        }
        assert!(after.translation.y < before.translation.y, "y decreases at sample {}", i);
        assert!(after.linvel.y < before.linvel.y, "vy decreases at sample {}", i);
        assert_eq!(after.translation.x, 0);
        assert_eq!(after.angvel, 0);
        i += 1;
    }

    // Closed form of the free fall over 4 semi-implicit Euler substeps of dt / 4 per step: after
    // n steps v = n g dt and y = y_0 + g (dt / 4)² m(m+1)/2 with m = 4n.
    // Error budget of the README: 2^12 raw per step on positions, twice that on velocities.
    let g: i128 = case.gravity.y.into();
    let dt: i128 = case.dt.into();
    let y0: i128 = state(@case, 0, 0).translation.y.into();
    let mut i = 0;
    while i != 12 {
        let step: i128 = sample(@case, i).step.into();
        let got = state(@case, i, 0);
        let vy = g * dt / ONE * step;
        let m = 4 * step;
        let y = y0 + g * dt * dt * (m * (m + 1)) / 32 / (ONE * ONE);
        let budget: u64 = 4096 * (sample(@case, i).step).into();
        assert!(within(got.linvel.y, vy.try_into().unwrap(), 2 * budget), "vy at sample {}", i);
        assert!(within(got.translation.y, y.try_into().unwrap(), budget), "y at sample {}", i);
        i += 1;
    }

    // Rest: the ball (radius 0.5) sits on the slab (top at y = 0), sunk by less than the allowed
    // linear error, and does not move any more.
    let last = state(@case, NUM_SAMPLES - 1, 0);
    let allowed = integration_parameters::DEFAULTS.normalized_allowed_linear_error;
    assert!(within(last.translation.y, HALF, allowed.try_into().unwrap()), "rest height");
    assert!(last.translation.y > 0, "the ball is above the slab");
    assert!(small(last.linvel.x, 4096) && small(last.linvel.y, 4096), "at rest");
    assert!(small(last.angvel, 4096), "not spinning");
}

#[test]
fn test_ball_bounce_leaves_the_slab_again() {
    let case = scene('ball_bounce');
    let mut rebounded = false;
    let mut i = 1;
    while i != NUM_SAMPLES {
        if state(@case, i, 0).linvel.y > 0 {
            rebounded = true;
        }
        i += 1;
    }
    assert!(rebounded, "restitution 0.7 sends the ball back up");
}

#[test]
fn test_box_stack3_settles_on_unit_spacing() {
    let case = scene('box_stack3');
    let allowed: u64 = integration_parameters::DEFAULTS
        .normalized_allowed_linear_error
        .try_into()
        .unwrap();
    let last = NUM_SAMPLES - 1;
    let mut k = 0;
    while k != 3 {
        let final_state = state(@case, last, k);
        // Rest height 0.5 + k, `allowed_linear_error` of slack.
        let rest: i64 = HALF + (k.try_into().unwrap() * ONE).try_into().unwrap();
        assert!(within(final_state.translation.y, rest, allowed), "rest height of box {}", k);
        assert!(final_state.translation.x < ONE.try_into().unwrap() / 100, "no drift");
        k += 1;
    }
}

#[test]
fn test_box_slope_friction_decides_between_sticking_and_sliding() {
    // 30° slope: the unit box starts at rest, then the friction of 0.7 (> tan 30°) holds it while
    // the friction of 0.25 (< tan 30°) lets it slide down-slope (towards -x).
    let stick = scene('box_slope_stick');
    let slide = scene('box_slope_slide');
    let last = NUM_SAMPLES - 1;
    let held = state(@stick, last, 0);
    let slid = state(@slide, last, 0);
    let start = state(@stick, 0, 0);
    assert!(small(held.linvel.x, 4096) && small(held.linvel.y, 4096), "stuck box is at rest");
    assert!(within(held.translation.x, start.translation.x, ONE.try_into().unwrap() / 100));
    assert!(slid.translation.x < start.translation.x - ONE.try_into().unwrap() * 4, "slid box");
}

#[test]
fn test_pendulum_joint_links_the_pivot_to_the_bob() {
    let case = scene('pendulum');
    assert_eq!(*case.joints.span().at(0).body1, 0);
    assert_eq!(*case.joints.span().at(0).body2, 1);
    assert_eq!(*case.joints.span().at(0).local_anchor1.x, 0);
    assert_eq!(*case.joints.span().at(0).local_anchor2.x, -ONE.try_into().unwrap());
    // The arm has length 1: the bob starts a full length to the right of the pivot and never
    // ends up further than that plus the joint's softness.
    assert_eq!(state(@case, 0, 0).translation.x, ONE.try_into().unwrap());
    let mut i = 0;
    while i != NUM_SAMPLES {
        let s = state(@case, i, 0);
        let x: i128 = s.translation.x.into();
        let y: i128 = s.translation.y.into();
        let norm_squared = (x * x + y * y) / ONE;
        // |arm|² within 1 ± 2^-8 (≈ 0.4 %), far above the drift of a healthy joint.
        assert!(
            within(norm_squared.try_into().unwrap(), ONE.try_into().unwrap(), 0x1000000),
            "arm length at sample {}",
            i,
        );
        i += 1;
    }
}
