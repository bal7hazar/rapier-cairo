use fixed::{Fixed, HALF, ONE, ZERO};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::joint::{GenericJointTrait, JointAxesMask};
use super::{*, alternatives};

fn fixture(axis: u8, position: Fixed) -> (ImpulseJoint, Array<SolverBody>, IntegrationParameters) {
    let h1 = Handle { index: 0, generation: 0 };
    let h2 = Handle { index: 1, generation: 0 };
    let b1 = SolverBody { handle: h1, ..Default::default() };
    let b2 = SolverBody {
        handle: h2,
        im: Vec2 { x: ONE, y: ONE },
        ii: ONE,
        position: if axis == 2 {
            let (im, re) = position.sin_cos();
            Pose2 { rotation: Rot2 { re, im }, ..Default::default() }
        } else {
            Pose2 { translation: Vec2 { x: position, y: ZERO }, ..Default::default() }
        },
        ..Default::default(),
    };
    let data = GenericJoint {
        locked_axes: JointAxesMask { bits: if axis == 2 {
            3
        } else {
            6
        } }, ..Default::default(),
    };
    (
        ImpulseJoint { body1: h1, body2: h2, data, impulses: [ZERO, ZERO, ZERO] },
        array![b1, b2],
        IntegrationParameters { dt: ONE, num_solver_iterations: 1, ..Default::default() },
    )
}
fn rows(c: JointConstraint) -> BoundedRows {
    c.bounded.unwrap().data.unbox()
}
fn near(a: Fixed, b: Fixed) {
    assert!((a - b).abs().raw <= 64);
}
#[test]
fn test_limits_interior_boundary_lower_upper_both_axes() {
    for axis in [0_u8, 2].span() {
        for pos in [-ONE, -HALF, ZERO, HALF, ONE].span() {
            let (mut j, mut bodies, p) = fixture(*axis, *pos);
            j.data.set_limits(*axis, [-HALF, HALF]);
            let mut c = JointConstraintTrait::generate(j, bodies.span(), p);
            let r = *rows(c).limits.at(0);
            // atan2 roundoff can move a boundary a few ulps; exact boundary checks use linear.
            if *axis == 0 || pos.abs() != HALF {
                assert_eq!(r.min, if *pos <= -HALF {
                    MIN
                } else {
                    ZERO
                });
                assert_eq!(r.max, if *pos >= HALF {
                    MAX
                } else {
                    ZERO
                });
            }
            c.solve(ref bodies, true);
            let r = *rows(c).limits.at(0);
            assert!(r.row.impulse >= r.min && r.row.impulse <= r.max);
            if *pos == ZERO {
                assert_eq!(r.row.impulse, ZERO);
            }
            c.solve(ref bodies, false);
            assert_eq!(*rows(c).limits.at(0).row.rhs, ZERO);
            c.writeback_impulses(ref j);
            assert_eq!(get_limit(j.data, *axis).impulse, *rows(c).limits.at(0).row.impulse);
        }
    }
}
#[test]
fn test_motor_velocity_position_force_cap_and_models() {
    for axis in [0_u8, 2].span() {
        for model in [MotorModel::AccelerationBased, MotorModel::ForceBased].span() {
            let (mut j, bodies, p) = fixture(*axis, ZERO);
            j.data.set_motor_velocity(*axis, ONE, ONE);
            j.data.set_motor_model(*axis, *model);
            // Change mass/inertia to distinguish the models.
            let b2 = SolverBody { im: Vec2 { x: HALF, y: HALF }, ii: HALF, ..*bodies.at(1) };
            let mut bodies = array![*bodies.at(0), b2];
            let mut c = JointConstraintTrait::generate(j, bodies.span(), p);
            let r = *rows(c).motors.at(0);
            assert_eq!(r.row.rhs, -ONE);
            assert_eq!(
                r.row.cfm_gain, if *model == MotorModel::AccelerationBased {
                    HALF
                } else {
                    ONE
                },
            );
            c.solve(ref bodies, true);
            let expected = if *model == MotorModel::AccelerationBased {
                -ONE
            } else {
                -inv(ONE + HALF)
            };
            near(*rows(c).motors.at(0).row.impulse, expected);
            c.writeback_impulses(ref j);
            near(get_motor(j.data, *axis).impulse, expected);
            j.data.set_motor_position(*axis, ONE, ONE, ONE);
            j.data.set_motor_max_force(*axis, Fixed { raw: 429496730 });
            let mut c = JointConstraintTrait::generate(j, bodies.span(), p);
            let rhs = *rows(c).motors.at(0).row.rhs;
            near(rhs, -HALF);
            c.remove_bias();
            assert_eq!(*rows(c).motors.at(0).row.rhs, rhs);
            c.solve(ref bodies, false);
            let r = *rows(c).motors.at(0);
            assert!(r.row.impulse.abs() <= Fixed { raw: 429496730 });
        }
    }
}
#[test]
fn test_warmstart_carry_and_separate_limit_motor_writeback() {
    let (mut j, mut bs, mut p) = fixture(0, ONE);
    j.data.set_limits(0, [-HALF, HALF]);
    j.data.set_motor_velocity(0, ONE, ONE);
    j.data.set_motor_max_force(0, HALF);
    let [mut l, y, w] = j.data.limits;
    l.impulse = ONE;
    j.data.limits = [l, y, w];
    let [mut m, y, w] = j.data.motors;
    m.impulse = -ONE;
    j.data.motors = [m, y, w];
    p.warmstart_joints = true;
    let mut c = JointConstraintTrait::generate(j, bs.span(), p);
    assert_eq!(*rows(c).limits.at(0).row.impulse, ONE);
    assert_eq!(*rows(c).motors.at(0).row.impulse, -ONE);
    c.warmstart(ref bs);
    assert_eq!(*bs.at(1).linvel.x, ZERO);
    c.solve(ref bs, true);
    c.solve(ref bs, false);
    c.writeback_impulses(ref j);
    assert_eq!(get_limit(j.data, 0).impulse, *rows(c).limits.at(0).row.impulse);
    assert_eq!(get_motor(j.data, 0).impulse, *rows(c).motors.at(0).row.impulse);
    // Regeneration carries the impulse; the next clamped solve releases the inactive limit.
    let b2 = SolverBody { position: Default::default(), ..*bs.at(1) };
    let mut bs = array![*bs.at(0), b2];
    let mut c = JointConstraintTrait::generate(j, bs.span(), p);
    assert_eq!(*rows(c).limits.at(0).row.impulse, get_limit(j.data, 0).impulse);
    c.warmstart(ref bs);
    c.solve(ref bs, true);
    assert_eq!(*rows(c).limits.at(0).row.impulse, ZERO);
}
#[test]
fn test_angular_wrap_full_turn_empty_range_and_zero_step() {
    // The angle comes from both local frames, including when the bodies have no rotation.
    for frame_angle in [HALF, -HALF].span() {
        let (mut framed, bodies, params) = fixture(2, ZERO);
        let (s, co) = frame_angle.sin_cos();
        framed.data.local_frame1.rotation = Rot2 { re: co, im: s };
        framed.data.local_frame2.rotation = Rot2 { re: co, im: -s };
        framed.data.set_limits(2, [-HALF, HALF]);
        let c = JointConstraintTrait::generate(framed, bodies.span(), params);
        let r = *rows(c).limits.at(0);
        assert_eq!(r.min == MIN, *frame_angle > ZERO);
        assert_eq!(r.max == MAX, *frame_angle < ZERO);
    }
    let (mut j, bs, mut p) = fixture(2, -PI + HALF);
    j.data.set_limits(2, [PI - ONE, PI + ONE]);
    let c = JointConstraintTrait::generate(j, bs.span(), p);
    let r = *rows(c).limits.at(0);
    assert_eq!(r.min, ZERO);
    assert_eq!(r.max, ZERO);
    j.data.set_limits(2, [MIN, MAX]);
    let c = JointConstraintTrait::generate(j, bs.span(), p);
    assert_eq!(*rows(c).limits.at(0).row.rhs, ZERO);
    j.data.set_limits(2, [ONE, -ONE]);
    let _ = JointConstraintTrait::generate(j, bs.span(), p);
    j.data.set_motor_velocity(2, ONE, ONE);
    j.data.set_motor_max_force(2, ONE);
    p.dt = ZERO;
    let c = JointConstraintTrait::generate(j, bs.span(), p);
    assert_eq!(*rows(c).motors.at(0).max, ZERO);
    assert_eq!(*rows(c).motors.at(0).min, ZERO);
}
#[test]
fn test_locked_coupled_disabled_and_generic_free_axes() {
    let (mut j, mut bs, p) = fixture(2, ZERO);
    j.data.locked_axes.bits = 0;
    j.data.set_motor_velocity(0, ONE, ONE);
    j.data.set_motor_velocity(1, ONE, ONE);
    j.data.set_motor_velocity(2, ONE, ONE);
    j.data.set_limits(0, [-ONE, ONE]);
    j.data.set_limits(1, [-ONE, ONE]);
    j.data.set_limits(2, [-ONE, ONE]);
    let mut c = JointConstraintTrait::generate(j, bs.span(), p);
    assert_eq!(rows(c).motors.len(), 3);
    assert_eq!(rows(c).limits.len(), 3);
    c.solve(ref bs, true);
    c.writeback_impulses(ref j);
    j.data.locked_axes.bits = 7;
    let c = JointConstraintTrait::generate(j, bs.span(), p);
    assert!(c.bounded.is_none());
    j.data.locked_axes.bits = 0;
    j.data.coupled_axes.bits = 7;
    let c = JointConstraintTrait::generate(j, bs.span(), p);
    // RJ: coupled linear axes form one distance DOF (one coupled motor, one coupled limit);
    // a coupled angular axis gets no row in 2D, as upstream.
    assert_eq!(rows(c).motors.len(), 1);
    assert_eq!(rows(c).limits.len(), 1);
    j.data.enabled = JointEnabled::Disabled;
    let c = JointConstraintTrait::generate(j, [].span(), p);
    assert_eq!(c.num_rows, 0);
}
#[test]
#[should_panic(expected: 'Joint: negative input')]
fn test_negative_motor_input() {
    let (mut j, bs, p) = fixture(2, ZERO);
    j.data.set_motor_velocity(2, ONE, -ONE);
    let _ = JointConstraintTrait::generate(j, bs.span(), p);
}
#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}

#[test]
#[fuzzer(runs: 32, seed: 13)]
fn fuzz_dispatch_bit_identical(velocity_raw: i16) {
    for controlled in [false, true].span() {
        let (mut j, bs, p) = fixture(2, HALF);
        if *controlled {
            j.data.set_motor_velocity(2, ONE, ONE);
            j.data.set_limits(2, [-HALF, HALF]);
        }
        let b2 = SolverBody { angvel: Fixed { raw: velocity_raw.into() * 65536 }, ..*bs.at(1) };
        let mut a = array![*bs.at(0), b2];
        let mut b = array![*bs.at(0), b2];
        let mut ca = JointConstraintTrait::generate(j, a.span(), p);
        let mut cb = ca;
        let mut cc = ca;
        let mut third = array![*bs.at(0), b2];
        for biased in [true, false].span() {
            ca.solve(ref a, *biased);
            alternatives::solve_early_return(ref cb, ref b, *biased);
            alternatives::solve_metered(ref cc, ref third, *biased);
            assert_eq!(ca, cc);
            assert_eq!(*a.at(1), *third.at(1));
            assert_eq!(ca, cb);
            assert_eq!(*a.at(0), *b.at(0));
            assert_eq!(*a.at(1), *b.at(1));
        }
    }
}
fn probe_input(controlled: bool) -> (JointConstraint, Array<SolverBody>) {
    let (mut j, bs, p) = fixture(2, opaque(HALF));
    if controlled {
        j.data.set_motor_velocity(2, ONE, ONE);
    }
    (opaque(JointConstraintTrait::generate(opaque(j), bs.span(), opaque(p))), bs)
}
#[test]
fn gas_dispatch_inline_plain() {
    let (mut c, mut bs) = probe_input(false);
    c.solve(ref bs, true);
    let _ = opaque((c, *bs.at(1)));
}
#[test]
fn gas_dispatch_early_plain() {
    let (mut c, mut bs) = probe_input(false);
    alternatives::solve_early_return(ref c, ref bs, true);
    let _ = opaque((c, *bs.at(1)));
}
#[test]
fn gas_dispatch_inline_motor() {
    let (mut c, mut bs) = probe_input(true);
    c.solve(ref bs, true);
    let _ = opaque((c, *bs.at(1)));
}
#[test]
fn gas_dispatch_early_motor() {
    let (mut c, mut bs) = probe_input(true);
    alternatives::solve_early_return(ref c, ref bs, true);
    let _ = opaque((c, *bs.at(1)));
}

#[test]
fn test_motor_limit_velocity_clip_angular_shortest_path_and_zero_force() {
    let (mut j, bs, p) = fixture(0, ONE);
    j.data.set_limits(0, [-HALF, HALF]);
    j.data.set_motor_velocity(0, ONE, ONE);
    let c = JointConstraintTrait::generate(j, bs.span(), p);
    assert_eq!(*rows(c).motors.at(0).row.rhs, HALF);
    let (mut j, mut bs, p) = fixture(2, -PI + HALF);
    j.data.set_motor_position(2, PI - HALF, ONE, ONE);
    let c = JointConstraintTrait::generate(j, bs.span(), p);
    near(*rows(c).motors.at(0).row.rhs, HALF);
    j.data.set_motor_max_force(2, ZERO);
    let before = *bs.at(1);
    let mut c = JointConstraintTrait::generate(j, bs.span(), p);
    c.solve(ref bs, true);
    assert_eq!(*rows(c).motors.at(0).row.impulse, ZERO);
    assert_eq!(*bs.at(1), before);
}

#[test]
fn gas_dispatch_loop_plain() {
    let (mut c, mut bs) = probe_input(false);
    alternatives::solve_metered(ref c, ref bs, true);
    let _ = opaque((c, *bs.at(1)));
}
#[test]
fn gas_dispatch_loop_motor() {
    let (mut c, mut bs) = probe_input(true);
    alternatives::solve_metered(ref c, ref bs, true);
    let _ = opaque((c, *bs.at(1)));
}
