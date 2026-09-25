use fixed::{Fixed, HALF, MIN, ONE, PI, TAU, ZERO};
use rapier_core::data::handle::Handle;
use rapier_testing::opaque;
use crate::joint::GenericJointTrait;
use super::alternatives::{
    angular_cos, controls_looped, generate_looped, interior_cos, plain_metered, plain_static,
};
use super::{*, StepJoint};

fn rot(angle: Fixed) -> Rot2 {
    let (im, re) = angle.sin_cos();
    Rot2 { re, im }
}
/// Pivot at the origin, a unit body at (2, 0) rotated by `angle`, joined at (1, 0).
fn fixture(data: GenericJoint, angle: Fixed) -> (ImpulseJoint, SolverBody, SolverBody) {
    let b1 = SolverBody { handle: Handle { index: 0, generation: 1 }, ..Default::default() };
    let b2 = SolverBody {
        handle: Handle { index: 1, generation: 1 },
        position: Pose2 { translation: Vec2 { x: ONE + ONE, y: ZERO }, rotation: rot(angle) },
        linvel: Vec2 { x: ZERO, y: -HALF },
        angvel: HALF,
        im: Vec2 { x: ONE, y: ONE },
        ii: ONE + ONE,
    };
    let joint = ImpulseJoint {
        body1: b1.handle, body2: b2.handle, data, impulses: [ZERO, ZERO, ZERO],
    };
    (joint, b1, b2)
}
fn revolute() -> GenericJoint {
    GenericJoint {
        locked_axes: JointAxesMask { bits: 3 },
        local_frame1: Pose2 { translation: Vec2 { x: ONE, y: ZERO }, ..Default::default() },
        local_frame2: Pose2 { translation: Vec2 { x: -ONE, y: ZERO }, ..Default::default() },
        ..Default::default(),
    }
}
fn step_of(data: GenericJoint) -> StepJoint {
    StepJoint {
        frame1: data.local_frame1,
        frame2: data.local_frame2,
        locks: data.locked_axes,
        softness: data.softness,
    }
}
/// JL's atan2 distance from the range centre and the interior shortcut for one angle.
fn classify(min: Fixed, max: Fixed, angle: Fixed) -> (bool, bool, bool) {
    let l = angular(JointLimits { min, max, impulse: ZERO }, true);
    let r = rot(angle);
    let dist = atan2_centered(l, r);
    (
        dist > -l.half && dist < l.half,
        interior(l, r),
        interior_cos(angular_cos(JointLimits { min, max, impulse: ZERO }), r),
    )
}

#[test]
fn test_interior_shortcut_never_claims_a_boundary_or_outside_angle() {
    let d = Fixed { raw: 65536 };
    for range in [
        (-HALF, HALF), (HALF, ONE), (-PI + HALF, PI - HALF), (-ONE - ONE, ONE),
        (ZERO, Fixed { raw: 1 }), (ONE, -ONE), (-PI, PI - Fixed { raw: 1 }),
    ]
        .span() {
        let (min, max) = *range;
        for base in [min, max, (min + max) / (ONE + ONE), max + ONE, min - ONE].span() {
            for k in [-4_i64, -2, -1, 0, 1, 2, 4, 64].span() {
                let angle = *base + Fixed { raw: *k * d.raw };
                let (exact, fast, cosine) = classify(min, max, angle);
                // The shortcuts are conservative: interior only if JL's distance is interior.
                assert!(!fast || exact);
                assert!(!cosine || exact);
            }
        }
    }
    // Well inside a range, the shortcut does fire (it is not vacuous).
    let (exact, fast, cosine) = classify(-HALF, HALF, Fixed { raw: 268435456 });
    assert!(exact && fast && cosine);
    let (_, fast, _) = classify(-PI + HALF, PI - HALF, ONE + ONE);
    assert!(!fast);
    // A full turn never becomes active; an empty range never becomes interior.
    assert_eq!(angular(JointLimits { min: MIN, max: fixed::MAX, impulse: ZERO }, true).mode, 2);
    assert_eq!(angular(JointLimits { min: ONE, max: -ONE, impulse: ZERO }, true).inside, ZERO);
}
#[test]
#[fuzzer(runs: 64, seed: 2211)]
fn fuzz_interior_shortcut_sound(center: i32, half: u32, offset: i32) {
    let center = Fixed { raw: center.into() * 4 };
    let half = Fixed { raw: half.into() % TAU.raw };
    let angle = center + Fixed { raw: offset.into() };
    let (exact, fast, cosine) = classify(center - half, center + half, angle);
    assert!(!fast || exact);
    assert!(!cosine || exact);
}
#[test]
fn test_controls_masks_order_and_initial_impulses() {
    let mut data = revolute();
    data.set_limits(2, [-HALF, HALF]);
    data.set_limits(0, [-HALF, HALF]);
    data.set_motor_velocity(2, ONE, ONE);
    let [mut lx, ly, lw] = data.limits;
    lx.impulse = HALF;
    data.limits = [lx, ly, lw];
    let c = controls(data, true);
    // Locked X carries no control; the angular axis carries both motor and limit.
    assert_eq!(c.axes.len(), 1);
    assert_eq!(*c.axes.at(0).axis, 2);
    assert!(c.axes.at(0).motor.is_some() && c.axes.at(0).limit.is_some());
    assert_eq!(c.limits, [HALF, ZERO, ZERO]);
    assert_eq!(c, controls_looped(data, true));
    // Free joint: angular first, then X, then Y; coupled axes are skipped.
    data.locked_axes.bits = 0;
    data.set_motor_velocity(1, ONE, ONE);
    let c = controls(data, false);
    assert_eq!(c.axes.len(), 3);
    assert_eq!((*c.axes.at(0).axis, *c.axes.at(1).axis, *c.axes.at(2).axis), (2, 0, 1));
    data.coupled_axes.bits = 2;
    assert_eq!(controls(data, false).axes.len(), 2);
    // Kinds: plain without limit/motor axes, legacy on an out-of-range mask.
    let (j, _, _) = fixture(revolute(), ZERO);
    assert!(match specialise(j) {
        StepKind::Plain => true,
        _ => false,
    });
    let mut j = j;
    j.data.set_limits(2, [-HALF, HALF]);
    j.data.coupled_axes.bits = 9;
    assert!(match specialise(j) {
        StepKind::Legacy(_) => true,
        _ => false,
    });
}
#[test]
fn test_generate_matches_public_and_carried_matches_writeback() {
    let p = IntegrationParameters {
        warmstart_joints: true, warmstart_coefficient: HALF, ..Default::default(),
    };
    for (min, angle) in [(HALF, ZERO), (-HALF, ZERO), (-ONE, ONE + HALF)].span() {
        let mut data = revolute();
        data.set_limits(2, [*min, ONE]);
        data.set_motor_position(2, HALF, ONE, HALF);
        let (mut j, b1, b2) = fixture(data, *angle);
        j.impulses = [HALF, -HALF, ZERO];
        let public = super::super::bounded::alternatives::generate_public_jl(j, [b1, b2].span(), p);
        let ctl = controls(j.data, true);
        let mut c = generate(
            step_of(j.data), ctl, j.impulses, ctl.motors, ctl.limits, b1, b2, p, false,
        );
        c.solver_vel2 = 1;
        assert_eq!(c, public);
        let looped = generate_looped(
            step_of(j.data), ctl, j.impulses, ctl.motors, ctl.limits, b1, b2, p, false,
        );
        assert_eq!(looped.bounded, c.bounded);
        let mut written = j;
        c.writeback_impulses(ref written);
        let (locks, motors, limits) = carried(c, j.impulses);
        assert_eq!(locks, written.impulses);
        let [_, _, m] = written.data.motors;
        let [_, _, mw] = motors;
        assert_eq!(mw, m.impulse);
        let [_, _, w] = written.data.limits;
        let [_, _, lw] = limits;
        // A limit row reads its impulse; an absent one reads zero.
        assert_eq!(lw, if c.num_rows == 4 {
            w.impulse
        } else {
            ZERO
        });
    }
}

#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}
fn plain_probe(op: u8) {
    let (j, b1, b2) = fixture(opaque(revolute()), opaque(HALF));
    let p: IntegrationParameters = opaque(Default::default());
    let step = step_of(j.data);
    match op {
        0 => { let _ = opaque((j, step, b1, b2, p)); },
        1 => { let _ = opaque(JointConstraintTrait::generate(j, [b1, b2].span(), p)); },
        2 => { let _ = opaque(plain(step, j.impulses, b1, b2, p)); },
        3 => { let _ = opaque(plain_static(step, j.impulses, b1, b2, p)); },
        4 => { let _ = opaque(plain_metered(step, j.impulses, b1, b2, p)); },
        _ => {
            let mut n = opaque(1_u32);
            let mut c = Default::default();
            while n != 0 {
                c = super::alternatives::plain_inline(step, j.impulses, b1, b2, p);
                n -= 1;
            }
            let _ = opaque(c);
        },
    }
}
#[test]
fn gas_plain_setup() {
    plain_probe(0);
}
#[test]
fn gas_plain_public() {
    plain_probe(1);
}
#[test]
fn gas_plain_wallet() {
    plain_probe(2);
}
#[test]
fn gas_plain_static() {
    plain_probe(3);
}
#[test]
fn gas_plain_metered() {
    plain_probe(4);
}
#[test]
fn gas_plain_loop_body() {
    plain_probe(5);
}
/// Controlled generation: 0 active limit, 1 velocity motor; `looped` selects the loser.
fn controlled_probe(kind: u8, run: bool, looped: bool) {
    let mut data = revolute();
    if kind == 0 {
        data.set_limits(2, [HALF, ONE]);
    } else {
        data.set_motor_velocity(2, ONE, ONE);
    }
    let (j, b1, b2) = fixture(opaque(data), opaque(ZERO));
    let p: IntegrationParameters = opaque(Default::default());
    let ctl = controls(j.data, true);
    let step = step_of(j.data);
    let mut c = Default::default();
    if run {
        c =
            if looped {
                generate_looped(step, ctl, j.impulses, ctl.motors, ctl.limits, b1, b2, p, true)
            } else {
                generate(step, ctl, j.impulses, ctl.motors, ctl.limits, b1, b2, p, true)
            };
    }
    let _ = opaque((c, ctl, step, b1, b2));
}
#[test]
fn gas_controlled_limit_setup() {
    controlled_probe(0, false, false);
}
#[test]
fn gas_controlled_limit_unrolled() {
    controlled_probe(0, true, false);
}
#[test]
fn gas_controlled_limit_looped() {
    controlled_probe(0, true, true);
}
#[test]
fn gas_controlled_motor_setup() {
    controlled_probe(1, false, false);
}
#[test]
fn gas_controlled_motor_unrolled() {
    controlled_probe(1, true, false);
}
#[test]
fn gas_controlled_motor_looped() {
    controlled_probe(1, true, true);
}
fn specialise_probe(op: u8) {
    let mut data = revolute();
    data.set_limits(2, [-HALF, HALF]);
    data.set_motor_velocity(2, ONE, ONE);
    let data = opaque(data);
    let [_, _, l] = data.limits;
    match op {
        0 => { let _ = opaque(data); },
        1 => { let _ = opaque(controls(data, true)); },
        2 => { let _ = opaque(controls_looped(data, true)); },
        3 => { let _ = opaque(angular(l, true)); },
        _ => { let _ = opaque(angular_cos(l)); },
    }
}
#[test]
fn gas_specialise_setup() {
    specialise_probe(0);
}
#[test]
fn gas_specialise_unrolled() {
    specialise_probe(1);
}
#[test]
fn gas_specialise_looped() {
    specialise_probe(2);
}
#[test]
fn gas_angular_tan() {
    specialise_probe(3);
}
#[test]
fn gas_angular_cos() {
    specialise_probe(4);
}
fn interior_probe(op: u8) {
    let l = opaque(angular(JointLimits { min: -HALF, max: HALF, impulse: ZERO }, true));
    let lc = opaque(angular_cos(JointLimits { min: -HALF, max: HALF, impulse: ZERO }));
    let r = opaque(rot(Fixed { raw: 268435456 }));
    match op {
        0 => { let _ = opaque((l, lc, r)); },
        1 => { let _ = opaque(interior(l, r)); },
        _ => { let _ = opaque(interior_cos(lc, r)); },
    }
}
#[test]
fn gas_interior_setup() {
    interior_probe(0);
}
#[test]
fn gas_interior_tan() {
    interior_probe(1);
}
#[test]
fn gas_interior_cos() {
    interior_probe(2);
}
