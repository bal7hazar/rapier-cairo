use fixed::wide::dot2;
use fixed::{Fixed, HALF, MAX, MIN, ONE, TWO, ZERO};
use rapier_core::data::handle::Handle;
use rapier_math::pose2::Pose2;
use rapier_testing::opaque;
use crate::joint::{
    GenericJointTrait, JointAxesMask, LIN_AXES, RopeJointBuilderTrait, SpringJointBuilderTrait,
};
use super::super::bounded::BoundedRows;
use super::super::step::{StepJoint, StepKind, specialise};
use super::{*, CoupledControl, alternatives, base, coupled};

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}
/// Static anchor body at the origin; body 2 at `at` with inverse mass `im` (both axes), unit
/// inverse inertia, moving at `vel`.
fn fixture(
    data: GenericJoint, at: Vec2, im: Fixed, vel: Vec2,
) -> (ImpulseJoint, Array<SolverBody>, IntegrationParameters) {
    let b1 = SolverBody { handle: Handle { index: 0, generation: 1 }, ..Default::default() };
    let b2 = SolverBody {
        handle: Handle { index: 1, generation: 1 },
        position: Pose2 { translation: at, ..Default::default() },
        im: v(im, im),
        ii: ONE,
        linvel: vel,
        ..Default::default(),
    };
    let j = ImpulseJoint { body1: b1.handle, body2: b2.handle, data, impulses: [ZERO, ZERO, ZERO] };
    (j, array![b1, b2], Default::default())
}
fn extra(c: JointConstraint) -> BoundedRows {
    c.bounded.unwrap().data.unbox()
}
fn step_of(data: GenericJoint) -> StepJoint {
    StepJoint {
        frame1: data.local_frame1,
        frame2: data.local_frame2,
        locks: data.locked_axes,
        softness: data.softness,
    }
}

#[test]
fn test_selection_table() {
    let rope = RopeJointBuilderTrait::new(ONE).build();
    let spring = SpringJointBuilderTrait::new(HALF, ONE, HALF).build();
    assert_eq!(
        coupled(rope),
        Some(CoupledControl { axis: 0, bits: 3, motor: None, limit: Some((ZERO, ONE)) }),
    );
    let [m, _, _] = spring.motors;
    assert_eq!(m.model, MotorModel::ForceBased);
    assert_eq!(m.target_pos, HALF);
    assert_eq!(
        coupled(spring), Some(CoupledControl { axis: 0, bits: 3, motor: Some(m), limit: None }),
    );
    // (coupled, locked, limit, motor bits) -> (axis, has motor, has limit) or no coupled rows.
    let cases: Array<(u8, u8, u8, u8, (bool, u8, bool, bool))> = array![
        (3, 0, 0, 0, (false, 0, false, false)), // coupling alone emits nothing
        (4, 0, 7, 7, (false, 0, false, false)), // coupled angular axis: no 2D row upstream
        (2, 0, 2, 0, (true, 1, false, true)), // Y only: first coupled axis is Y
        (
            2, 0, 1, 0, (false, 0, false, false),
        ), // a limit on the uncoupled X is not the coupled limit
        (3, 0, 0, 2, (true, 0, true, false)), // any coupled motor axis enables the motor
        (3, 1, 1, 1, (false, 0, false, false)), // first coupled axis locked: neither
        (3, 1, 1, 2, (true, 0, true, false)), // ... but the Y motor still counts
        (7, 4, 1, 4, (true, 0, false, true)),
    ];
    for case in cases.span() {
        let (c, l, lim, mot, expected) = *case;
        let data = GenericJoint {
            coupled_axes: JointAxesMask { bits: c },
            locked_axes: JointAxesMask { bits: l },
            limit_axes: JointAxesMask { bits: lim },
            motor_axes: JointAxesMask { bits: mot },
            ..Default::default(),
        };
        let got = match coupled(data) {
            Some(cc) => (true, cc.axis, cc.motor.is_some(), cc.limit.is_some()),
            None => (false, 0, false, false),
        };
        assert_eq!(got, expected);
    }
    // Specialisation: rope and spring are their own kind; free-axis controls are kept.
    let j = ImpulseJoint {
        body1: Handle { index: 0, generation: 1 },
        body2: Handle { index: 1, generation: 1 },
        data: rope,
        impulses: [ZERO, ZERO, ZERO],
    };
    assert!(
        match specialise(j) {
            StepKind::Legacy(k) => super::super::step::coupled_kind(k.unbox().data, true)
                .controls
                .axes
                .is_empty(),
            _ => false,
        },
    );
    let mut j = j;
    j.data.set_motor_velocity(2, ONE, ONE);
    assert!(
        match specialise(j) {
            StepKind::Legacy(k) => super::super::step::coupled_kind(k.unbox().data, true)
                .controls
                .axes
                .len() == 1,
            _ => false,
        },
    );
}

/// Rope of length 1 from the origin, body at distance `d` along a diagonal-ish direction.
#[test]
fn test_rope_rows_slack_taut() {
    let dir = v(Fixed { raw: 2576980378 }, Fixed { raw: 3435973837 }); // (0.6, 0.8)
    // (distance, separating speed, expect a clamping impulse)
    let cases = array![
        (HALF, HALF, false), // slack, slow: the speculative rhs keeps the impulse at zero
        (HALF, -ONE, false), // slack, approaching
        (HALF, Fixed { raw: 240 * ONE.raw }, true), // slack but overshooting within the substep
        (ONE, HALF, true), // at the bound, separating: clamped
        (ONE + HALF, ZERO, true), // taut past the bound: ERP bias pulls back
        (
            ONE + HALF, Fixed { raw: -100 * ONE.raw }, false,
        ) // past the bound but approaching fast enough
    ];
    for case in cases.span() {
        let (d, speed, active) = *case;
        let data = RopeJointBuilderTrait::new(ONE).build();
        let at = v(dir.x * d, dir.y * d);
        let (j, mut bs, p) = fixture(data, at, ONE, v(dir.x * speed, dir.y * speed));
        let mut c = JointConstraintTrait::generate(j, bs.span(), p);
        // Upstream always emits the coupled limit row, one row along the separation.
        assert_eq!(c.num_rows, 4);
        let rows = extra(c);
        assert_eq!(rows.locks, 0);
        assert_eq!(rows.motors.len(), 0);
        assert_eq!(rows.limits.len(), 1);
        let r = *rows.limits.at(0);
        assert_eq!((r.min, r.max), (ZERO, MAX));
        assert!(
            (r.row.lin_jac.x - dir.x).abs().raw <= 4 && (r.row.lin_jac.y - dir.y).abs().raw <= 4,
        );
        assert_eq!(r.row.ang_jac2, ZERO);
        if d < ONE {
            assert!(r.row.rhs_wo_bias < ZERO && r.row.rhs == r.row.rhs_wo_bias);
        } else {
            assert_eq!(r.row.rhs_wo_bias, ZERO);
            assert!((d == ONE) == (r.row.rhs == ZERO));
        }
        let before = *bs.at(1);
        c.solve(ref bs, true);
        let after = *bs.at(1);
        let impulse = (*extra(c).limits.at(0)).row.impulse;
        assert!(impulse >= ZERO);
        assert_eq!(impulse > ZERO, active);
        if active {
            // The separation velocity is clamped to the speculative/bias target.
            let sep = dot2(after.linvel.x, dir.x, after.linvel.y, dir.y);
            assert!((sep + r.row.rhs).abs().raw <= 1024);
        } else {
            assert_eq!(after, before);
        }
    }
}

#[test]
fn test_spring_force_sign_and_magnitude() {
    let dir = v(ONE, ZERO);
    // (distance, inverse mass, force based): rest length 1, stiffness 1, damping 0.
    let cases = array![
        (TWO, ONE, true), (HALF, ONE, true), (ONE, ONE, true), (TWO, HALF, true),
        (TWO, HALF, false), (HALF, HALF, false),
    ];
    for case in cases.span() {
        let (d, im, force_based) = *case;
        let model = if force_based {
            MotorModel::ForceBased
        } else {
            MotorModel::AccelerationBased
        };
        let data = SpringJointBuilderTrait::new(ONE, ONE, ZERO).spring_model(model).build();
        let (j, mut bs, p) = fixture(data, v(d, ZERO), im, v(ZERO, ZERO));
        let mut c = JointConstraintTrait::generate(j, bs.span(), p);
        let rows = extra(c);
        assert_eq!((rows.motors.len(), rows.limits.len()), (1, 0));
        c.solve(ref bs, true);
        let impulse = (*extra(c).motors.at(0)).row.impulse;
        // Positive impulse pulls body 2 toward body 1 (upstream sign convention).
        let x = d - ONE;
        assert_eq!(impulse > ZERO, x > ZERO);
        assert_eq!(impulse < ZERO, x < ZERO);
        // Implicit spring: impulse = k x dt / (1 + m_eff / (k dt^2)) for force-based springs,
        // times the effective mass for acceleration-based ones (small-dt limit, 1 % slack).
        let dt = p.substep_dt();
        let expected = if force_based {
            x * dt
        } else {
            x * dt / im
        };
        assert!((impulse - expected).abs() <= expected.abs() / Fixed { raw: 100 * ONE.raw });
        assert!(dot2((*bs.at(1)).linvel.x, dir.x, (*bs.at(1)).linvel.y, dir.y) * x <= ZERO);
    }
}

#[test]
fn test_zero_separation_and_unset_bound() {
    // Coincident anchors: zero Jacobian (upstream's simd_inv(0) = 0), no impulse, no panic.
    let rope = RopeJointBuilderTrait::new(ONE).build();
    let (j, mut bs, p) = fixture(rope, v(ZERO, ZERO), ONE, v(ONE, HALF));
    let mut c = JointConstraintTrait::generate(j, bs.span(), p);
    let r = *extra(c).limits.at(0);
    assert_eq!(r.row.lin_jac, v(ZERO, ZERO));
    assert_eq!(r.row.inv_lhs, ZERO);
    let before = *bs.at(1);
    c.solve(ref bs, true);
    assert_eq!(*bs.at(1), before);
    // A one-ulp separation keeps a unit direction (the literal Fixed `inv` would overflow).
    let (h, _) = base(
        JointConstraintHelper {
            x: v(ONE, ZERO),
            y: v(ZERO, ONE),
            r1: v(ZERO, ZERO),
            r2: v(ZERO, ZERO),
            lin_err: v(Fixed { raw: 1 }, ZERO),
            ang_err: ZERO,
        },
        3,
        0,
        *bs.at(0),
        *bs.at(1),
    );
    assert_eq!(h.lin_jac, v(ONE, ZERO));
    // The MAX sentinel of an unset upper bound never acts: no row.
    let mut data = GenericJoint { coupled_axes: LIN_AXES, ..Default::default() };
    data.set_limits(0, [MIN, MAX]);
    let (j, bs, p) = fixture(data, v(TWO, ZERO), ONE, v(ZERO, ZERO));
    let c = JointConstraintTrait::generate(j, bs.span(), p);
    assert_eq!(c.num_rows, 0);
}

#[test]
fn test_island_path_matches_public_and_writeback() {
    let rope = RopeJointBuilderTrait::new(ONE).local_anchor2(v(ZERO, HALF)).build();
    let mut spring = SpringJointBuilderTrait::new(HALF, ONE, HALF).build();
    spring.set_limits(0, [ZERO, TWO]); // clamps the target velocity, adds the coupled limit
    spring.set_motor_velocity(2, ONE, ONE); // a free-axis motor precedes the coupled one
    for data in [spring, rope].span() {
        let (mut j, mut bs, mut p) = fixture(*data, v(ONE + HALF, HALF), ONE, v(ONE, ZERO));
        p.warmstart_joints = true;
        let [mx, my, mw] = j.data.motors;
        j.data.motors = [JointMotor { impulse: HALF, ..mx }, my, mw];
        let public = JointConstraintTrait::generate(j, bs.span(), p);
        let kind = match specialise(j) {
            StepKind::Legacy(k) => super::super::step::coupled_kind(k.unbox().data, true),
            _ => core::panic_with_felt252('not coupled'),
        };
        let island = super::super::step::generate_coupled(
            step_of(j.data),
            kind,
            j.impulses,
            kind.controls.motors,
            kind.controls.limits,
            *bs.at(0),
            *bs.at(1),
            p,
            true,
        );
        let mut island = island;
        island.solver_vel1 = 0;
        island.solver_vel2 = 1;
        assert_eq!(public, island);
        let mut c = public;
        c.warmstart(ref bs);
        c.solve(ref bs, true);
        c.remove_bias();
        c.solve(ref bs, false);
        let before = j;
        c.writeback_impulses(ref j);
        let rows = extra(c);
        let [mx, _, mw] = j.data.motors;
        let [lx, _, _] = j.data.limits;
        let has_motor = rows.motors.len() != 0;
        if has_motor {
            let motor = *rows.motors.at(rows.motors.len() - 1);
            assert_eq!(motor.row.axis, 0);
            assert_eq!(mx.impulse, motor.row.impulse);
        }
        if rows.limits.len() != 0 {
            assert_eq!(lx.impulse, (*rows.limits.at(0)).row.impulse);
        }
        if rows.motors.len() == 2 {
            assert_eq!(mw.impulse, (*rows.motors.at(0)).row.impulse);
        } else {
            assert_eq!(mw, *before.data.motors.span()[2]);
        }
        // carried() reads back exactly what writeback persisted.
        let (_, motors, limits) = super::super::step::carried(c, j.impulses);
        let [cx, _, _] = motors;
        let [clx, _, _] = limits;
        assert_eq!(cx, if has_motor {
            mx.impulse
        } else {
            ZERO
        });
        assert_eq!(clx, if rows.limits.len() != 0 {
            lx.impulse
        } else {
            ZERO
        });
    }
}

#[test]
#[fuzzer(runs: 32, seed: 11)]
fn fuzz_base_candidates(x: i32, y: i32, rx: i16, ry: i16) {
    let e = v(Fixed { raw: x.into() * 64 }, Fixed { raw: y.into() * 64 });
    if e.x == ZERO && e.y == ZERO {
        return;
    }
    let h = JointConstraintHelper {
        x: v(ONE, ZERO),
        y: v(ZERO, ONE),
        r1: v(Fixed { raw: rx.into() * 65536 }, Fixed { raw: ry.into() * 65536 }),
        r2: v(Fixed { raw: ry.into() * 65536 }, ZERO),
        lin_err: e,
        ang_err: ZERO,
    };
    let b = SolverBody { ii: HALF, ..Default::default() };
    let (a, da) = base(h, 3, 0, b, b);
    let (l, dl) = alternatives::literal(h, 3, 0, b, b);
    let (f, df) = alternatives::fixed_inv(h, 3, 0, b, b);
    assert_eq!(da, dl);
    assert_eq!(da, df);
    // The candidates agree up to rounding (relative to the lever arm for angular terms).
    let tol = Fixed { raw: 64 };
    assert!((a.lin_jac.x - l.lin_jac.x).abs() <= tol && (a.lin_jac.y - f.lin_jac.y).abs() <= tol);
    let lever = h.r1.x.abs() + h.r1.y.abs() + ONE;
    assert!((a.ang_jac1 - l.ang_jac1).abs() <= tol * lever);
    assert!((a.ang_jac1 - f.ang_jac1).abs() <= tol * lever);
}

#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}
fn helper() -> JointConstraintHelper {
    JointConstraintHelper {
        x: v(ONE, ZERO),
        y: v(ZERO, ONE),
        r1: v(ONE, HALF),
        r2: v(HALF, ZERO),
        lin_err: v(ONE, HALF),
        ang_err: ZERO,
    }
}
#[test]
fn gas_base_wide() {
    let b = SolverBody { ii: HALF, ..Default::default() };
    let _ = opaque(base(opaque(helper()), opaque(3), 0, opaque(b), opaque(b)));
}
#[test]
fn gas_base_literal() {
    let b = SolverBody { ii: HALF, ..Default::default() };
    let _ = opaque(alternatives::literal(opaque(helper()), opaque(3), 0, opaque(b), opaque(b)));
}
#[test]
fn gas_base_fixed_inv() {
    let b = SolverBody { ii: HALF, ..Default::default() };
    let _ = opaque(alternatives::fixed_inv(opaque(helper()), opaque(3), 0, opaque(b), opaque(b)));
}
fn bench(data: GenericJoint, solve: bool) {
    let (j, mut bs, p) = fixture(data, v(ONE + HALF, HALF), ONE, v(ONE, ZERO));
    let mut c = JointConstraintTrait::generate(opaque(j), bs.span(), opaque(p));
    if solve {
        c.solve(ref bs, true);
    }
    let _ = opaque((c, *bs.at(1)));
}
#[test]
fn gas_generate_rope() {
    bench(RopeJointBuilderTrait::new(ONE).build(), false);
}
#[test]
fn gas_generate_solve_rope() {
    bench(RopeJointBuilderTrait::new(ONE).build(), true);
}
#[test]
fn gas_generate_spring() {
    bench(SpringJointBuilderTrait::new(HALF, ONE, HALF).build(), false);
}
#[test]
fn gas_generate_solve_spring() {
    bench(SpringJointBuilderTrait::new(HALF, ONE, HALF).build(), true);
}
