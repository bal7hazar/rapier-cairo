//! The sleep timer of lot SC: `update_sleep_timer` against SL's reference
//! (`alternatives::update_sleep_timer_v1`) on random bodies, and the per-body gas probes.

use fixed::{FRAC_PI_2, Fixed, HALF, MAX, ONE, ZERO};
use glam::Vec2;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_core::rigid_body::activation::DEFAULT_NORMALIZED_LINEAR_THRESHOLD;
use rapier_core::rigid_body::{RigidBodyActivation, RigidBodyActivationTrait, RigidBodyType};
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodyTrait};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use super::alternatives::{
    update_sleep_timer_captured, update_sleep_timer_rest_static, update_sleep_timer_v1,
    update_sleep_timer_wide,
};
use super::super::fixtures::{at, f, v};
use super::{
    DEFAULT_DT, DEFAULT_LIMIT, NEAR_SQ_MINUS_ONE, translation_below, translation_near,
    update_sleep_timer,
};

/// `(|delta| + chord) / 2 < limit` through the square root, as `dynamic_gate` on the drift.
fn reference_below_with(delta: Vec2, limit: Fixed, chord: Fixed) -> bool {
    (glam::Vec2Trait::length(delta) + chord) * HALF < limit
}

fn reference_below(delta: Vec2, limit: Fixed) -> bool {
    reference_below_with(delta, limit, ZERO)
}

/// `translation_below` on a `Vec2` delta.
fn below_with(delta: Vec2, limit: Fixed, chord: Fixed) -> bool {
    translation_below(delta.x.raw.into(), delta.y.raw.into(), limit, chord)
}

fn below(delta: Vec2, limit: Fixed) -> bool {
    below_with(delta, limit, ZERO)
}

/// A body of `kind` at `previous + delta`, the previous pose being `previous`.
fn body_of(
    kind: u32, delta: Vec2, turn: i64, angvel: i64, extent: i64, ang_thr: i64, thr: i64,
) -> (RigidBody, Pose2) {
    let previous = at(ONE, -ONE);
    let mut current = at(ONE + delta.x, -ONE + delta.y);
    current.rotation = Rot2 { re: ONE, im: f(turn) };
    let mut body = RigidBodyTrait::new(
        if kind == 0 {
            RigidBodyType::Fixed
        } else if kind == 1 {
            RigidBodyType::KinematicPositionBased
        } else if kind == 2 {
            RigidBodyType::KinematicVelocityBased
        } else {
            RigidBodyType::Dynamic
        },
        current,
    );
    body.vels.angvel = f(angvel);
    body.vels.linvel = if kind == 1 && turn != 0 {
        v(ONE, ZERO)
    } else {
        v(ZERO, ZERO)
    };
    body.mprops.max_extent = f(extent);
    body.activation.angular_threshold = f(ang_thr);
    body.activation.normalized_linear_threshold = f(thr);
    body.activation.time_since_can_sleep = HALF;
    (body, previous)
}

/// Both timers agree on the whole body.
fn check(body: RigidBody, previous: Pose2, params: IntegrationParameters) {
    let mut new = body;
    let mut old = body;
    let mut rest_static = body;
    update_sleep_timer(ref new, previous, params);
    update_sleep_timer_v1(ref old, previous, params);
    update_sleep_timer_rest_static(ref rest_static, previous, params);
    assert_eq!(new, old);
    assert_eq!(rest_static, old);
}

fn params_of(unit_sel: u8, dt_sel: u8) -> IntegrationParameters {
    let mut params: IntegrationParameters = Default::default();
    params
        .length_unit =
            if unit_sel % 3 == 0 {
                ONE
            } else if unit_sel % 3 == 1 {
                HALF
            } else {
                ONE + ONE
            };
    if dt_sel % 4 == 1 {
        params.dt = f(143165577);
    } else if dt_sel % 4 == 2 {
        params.dt = ZERO;
    }
    params
}

fn extent_of(sel: u16) -> i64 {
    if sel % 8 == 0 {
        0
    } else if sel % 8 == 1 {
        MAX.raw
    } else {
        sel.into() * 65536
    }
}

/// The configuration the fast test assumes.
fn defaults() -> IntegrationParameters {
    Default::default()
}

/// Any pose delta up to about 0.002 units, any turn (none, small or large), velocity, threshold
/// (negative too, and the default half of the time), extent (none, `MAX`, some), length unit, `dt`
/// and body type.
#[test]
#[fuzzer(runs: 128, seed: 20260924)]
fn fuzz_timer_matches_reference(
    dx: i32, dy: i32, turn: i16, angvel: i16, thr: i32, ang_thr: i16, extent: u16, mix: u32,
) {
    let default_configuration = mix % 2 == 0;
    let (body, previous) = body_of(
        (mix / 2 % 4).into(),
        v(f(dx.into() / 256), f(dy.into() / 256)),
        if mix % 3 == 0 {
            0
        } else if mix % 5 == 0 {
            turn.into() * 4096
        } else {
            // A turn whose chord is of the order of the limit.
            turn.into() * 16
        },
        angvel.into() * 262144,
        extent_of(extent),
        ang_thr.into() * 262144,
        if default_configuration {
            DEFAULT_NORMALIZED_LINEAR_THRESHOLD.raw
        } else {
            thr.into()
        },
    );
    let params = if default_configuration {
        defaults()
    } else {
        params_of((mix / 8 % 255).try_into().unwrap(), (mix / 2048 % 255).try_into().unwrap())
    };
    check(body, previous, params);
}

/// Around the boundary `|delta| + chord = 2 · limit`, on an axis and on a 3-4-5 diagonal, with the
/// default configuration (which has a constant limit) or a random threshold and length unit; the
/// timer sees a chord-free delta, `translation_below` any chord.
#[test]
#[fuzzer(runs: 128, seed: 20260925)]
fn fuzz_timer_boundary(
    thr: u32, offset: u8, diagonal: bool, negative: bool, unit_sel: u8, chord_raw: u16,
) {
    let default_configuration = unit_sel % 2 == 0;
    let params = if default_configuration {
        defaults()
    } else {
        params_of(unit_sel, 0)
    };
    let mut activation: RigidBodyActivation = RigidBodyActivationTrait::active();
    if !default_configuration {
        activation.normalized_linear_threshold = f(thr.into());
    }
    let limit = activation.linear_limit(params.length_unit, params.dt);
    let sign: i64 = if negative {
        -1
    } else {
        1
    };
    let mut cases = array![(ZERO, limit.raw * 2 + offset.into() % 5 - 2)];
    cases
        .append(
            (
                f(chord_raw.into() * 64),
                limit.raw * 2 - chord_raw.into() * 64 + offset.into() % 5 - 2,
            ),
        );
    for (chord, target) in cases {
        let target = if target < 0 {
            0
        } else {
            target
        };
        let delta = if diagonal {
            v(f(sign * (target / 5) * 3), f((target / 5) * 4))
        } else {
            v(f(sign * target), ZERO)
        };
        assert_eq!(below_with(delta, limit, chord), reference_below_with(delta, limit, chord));
        if chord == ZERO {
            let (body, previous) = body_of(
                3, delta, 0, 0, 32768, 0, activation.normalized_linear_threshold.raw,
            );
            check(body, previous, params);
        }
    }
}

/// `translation_below` and `translation_near` at the exact edge of a few limits: `length < 2 ·
/// limit` holds one raw unit below and fails at it, whatever the axis or the sign.
#[test]
fn test_translation_below_edges() {
    for limit in array![1_i64, 2, 3, 3579139, 3579140, 1000000007, 0x3fffffffffffffff].span() {
        let edge = *limit * 2;
        for (dx, dy) in array![(edge, 0_i64), (0, -edge), (-edge, 0), (0, edge)].span() {
            assert!(!below(v(f(*dx), f(*dy)), f(*limit)));
            assert!(!reference_below(v(f(*dx), f(*dy)), f(*limit)));
        }
        for (dx, dy) in array![(edge - 1, 0_i64), (0, -(edge - 1)), (0, 0)].span() {
            assert!(below(v(f(*dx), f(*dy)), f(*limit)));
            assert!(reference_below(v(f(*dx), f(*dy)), f(*limit)));
        }
    }
    // A zero or negative limit is never above a length: no motion is still not below it.
    assert!(!below(v(ZERO, ZERO), ZERO));
    assert!(!below(v(ZERO, ZERO), f(-5)));
    assert!(!reference_below(v(ZERO, ZERO), ZERO));
    assert!(!reference_below(v(ZERO, ZERO), f(-5)));
    // The constant of the fast test.
    let edge = DEFAULT_LIMIT.raw * 2;
    assert!(translation_near(0, (edge - 1).into()));
    assert!(translation_near((-(edge - 1)).into(), 0));
    assert!(!translation_near(edge.into(), 0));
    assert!(!translation_near(0, (-edge).into()));
    assert!(translation_near(0, 0));
    assert!(!translation_near((edge * 71 / 100).into(), (edge * 71 / 100).into()));
    assert!(translation_near((edge * 7 / 10).into(), (edge * 7 / 10).into()));
}

/// The constants of the fast test are what the default configuration gives.
#[test]
fn test_default_configuration_constants() {
    let params = defaults();
    let activation: RigidBodyActivation = RigidBodyActivationTrait::active();
    assert_eq!(params.dt, DEFAULT_DT);
    assert_eq!(params.length_unit, ONE);
    assert_eq!(activation.linear_limit(params.length_unit, params.dt), DEFAULT_LIMIT);
    let edge: felt252 = (DEFAULT_LIMIT.raw * 2).into();
    assert_eq!(NEAR_SQ_MINUS_ONE, edge * edge - 1);
}

/// The wide angular gate at the edge of `(pi / 2)^2`, on both signs, against the rescaled one.
#[test]
fn test_angular_gate_wide_matches_the_rescaled_gate() {
    let mut activation: RigidBodyActivation = RigidBodyActivationTrait::active();
    for negative_threshold in array![false, true].span() {
        if *negative_threshold {
            activation.angular_threshold = f(-1);
        }
        for offset in array![-4_i64, -2, -1, 0, 1, 2, 3, 10, 1000].span() {
            for sign in array![1_i64, -1].span() {
                let angvel = f(*sign * (FRAC_PI_2.raw + *offset));
                assert_eq!(
                    activation.angular_gate_wide(angvel),
                    activation.angular_gate(angvel * angvel, HALF),
                );
            }
        }
    }
}

// The scenarios of the gas probes: a falling ball (the translation of the step is far above the
// limit), a ball at rest (a few ulp, no turn) and a turning one; default configuration.
const FALL: u8 = 0;
const REST: u8 = 1;
const TURN: u8 = 2;

#[inline(never)]
fn scenario(kind: u8) -> (RigidBody, Pose2) {
    let previous = at(ZERO, ONE + HALF);
    let mut body = RigidBodyTrait::dynamic(opaque(at(ZERO, ONE)));
    body.mprops.max_extent = HALF;
    if kind == REST {
        body.pos.position = at(f(1000), ONE + HALF);
    } else if kind == TURN {
        body.pos.position = at(f(1000), ONE + HALF);
        body.pos.position.rotation = Rot2 { re: ONE, im: f(1000) };
    }
    (body, previous)
}

#[inline(never)]
fn opaque_params() -> IntegrationParameters {
    opaque(defaults())
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

/// Every `gas_timer_*` minus its `gas_setup_*` is one timer update.
#[test]
fn gas_setup_fall() {
    let (body, previous) = scenario(opaque(FALL));
    let _ = opaque(opaque_params().dt);
    let _ = opaque((body.activation.time_since_can_sleep, previous));
}

#[test]
fn gas_setup_rest() {
    let (body, previous) = scenario(opaque(REST));
    let _ = opaque(opaque_params().dt);
    let _ = opaque((body.activation.time_since_can_sleep, previous));
}

#[test]
fn gas_setup_turn() {
    let (body, previous) = scenario(opaque(TURN));
    let _ = opaque(opaque_params().dt);
    let _ = opaque((body.activation.time_since_can_sleep, previous));
}

#[test]
fn gas_timer_v1_fall() {
    let (mut body, previous) = scenario(opaque(FALL));
    update_sleep_timer_v1(ref body, previous, opaque_params());
    let _ = opaque((body.activation.time_since_can_sleep, previous));
}

#[test]
fn gas_timer_v1_rest() {
    let (mut body, previous) = scenario(opaque(REST));
    update_sleep_timer_v1(ref body, previous, opaque_params());
    let _ = opaque((body.activation.time_since_can_sleep, previous));
}

#[test]
fn gas_timer_v1_turn() {
    let (mut body, previous) = scenario(opaque(TURN));
    update_sleep_timer_v1(ref body, previous, opaque_params());
    let _ = opaque((body.activation.time_since_can_sleep, previous));
}

#[test]
fn gas_timer_wide_fall() {
    let (mut body, previous) = scenario(opaque(FALL));
    update_sleep_timer_wide(ref body, previous, opaque_params());
    let _ = opaque((body.activation.time_since_can_sleep, previous));
}

#[test]
fn gas_timer_wide_rest() {
    let (mut body, previous) = scenario(opaque(REST));
    update_sleep_timer_wide(ref body, previous, opaque_params());
    let _ = opaque((body.activation.time_since_can_sleep, previous));
}

#[test]
fn gas_timer_captured_fall() {
    let (mut body, previous) = scenario(opaque(FALL));
    update_sleep_timer_captured(ref body, previous, opaque_params());
    let _ = opaque((body.activation.time_since_can_sleep, previous));
}

#[test]
fn gas_timer_fall() {
    let (mut body, previous) = scenario(opaque(FALL));
    update_sleep_timer(ref body, previous, opaque_params());
    let _ = opaque((body.activation.time_since_can_sleep, previous));
}

#[test]
fn gas_timer_rest() {
    let (mut body, previous) = scenario(opaque(REST));
    update_sleep_timer(ref body, previous, opaque_params());
    let _ = opaque((body.activation.time_since_can_sleep, previous));
}

#[test]
fn gas_timer_turn() {
    let (mut body, previous) = scenario(opaque(TURN));
    update_sleep_timer(ref body, previous, opaque_params());
    let _ = opaque((body.activation.time_since_can_sleep, previous));
}

/// A body that cannot sleep (negative threshold): the default-configuration test is skipped.
#[test]
fn gas_timer_cannot_sleep() {
    let (mut body, previous) = scenario(opaque(FALL));
    body.activation = RigidBodyActivationTrait::cannot_sleep();
    update_sleep_timer(ref body, previous, opaque_params());
    let _ = opaque((body.activation.time_since_can_sleep, previous));
}

#[test]
fn gas_timer_v1_cannot_sleep() {
    let (mut body, previous) = scenario(opaque(FALL));
    body.activation = RigidBodyActivationTrait::cannot_sleep();
    update_sleep_timer_v1(ref body, previous, opaque_params());
    let _ = opaque((body.activation.time_since_can_sleep, previous));
}

#[test]
fn gas_timer_rest_static_fall() {
    let (mut body, previous) = scenario(opaque(FALL));
    update_sleep_timer_rest_static(ref body, previous, opaque_params());
    let _ = opaque((body.activation.time_since_can_sleep, previous));
}

#[test]
fn gas_timer_rest_static_rest() {
    let (mut body, previous) = scenario(opaque(REST));
    update_sleep_timer_rest_static(ref body, previous, opaque_params());
    let _ = opaque((body.activation.time_since_can_sleep, previous));
}
