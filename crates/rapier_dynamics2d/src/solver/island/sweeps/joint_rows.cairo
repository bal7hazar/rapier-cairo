//! JM: JL's island joint path (joint copies in the builders, every limit/motor row rebuilt with a
//! second `prepare`, impulses written into a joint copy between substeps), kept for the
//! bit-identity fuzz and the per-kind frame probes (`gas_<kind>_{jl,jm}` minus `_setup`).
use fixed::{Fixed, HALF, MAX, MIN, ONE, ZERO};
use glam::Vec2;
use rapier_core::integration_parameters::IntegrationParametersTrait;
use rapier_testing::opaque;
use crate::joint::{GenericJoint, GenericJointTrait, JointAxesMask};
use super::*;
use super::super::super::body_store::DenseBodies;
use super::super::super::joint::bounded::alternatives::{
    generate_public_jl, solve_public_jl, warmstart_public_jl,
};
use super::super::super::joint::probes::{chain, joint_data};
use super::super::{add_forces, integrate};

#[derive(Copy, Drop)]
pub(crate) struct JlBuilder {
    pub joint: ImpulseJoint,
    pub i: u32,
    pub j: u32,
}
pub(crate) fn prepare_jl(
    mut js: Span<ImpulseJoint>, bs: Span<SolverBody>, steps: Span<BodyStep>,
) -> Array<JlBuilder> {
    let mut out = array![];
    while let Some(joint) = js.pop_front() {
        let mut joint = *joint;
        if joint.data.enabled == JointEnabled::Enabled {
            let i = resolve(bs, joint.body1);
            let j = resolve(bs, joint.body2);
            joint.data.local_frame1.translation = joint.data.local_frame1.translation
                - *steps.at(i).local_com;
            joint.data.local_frame2.translation = joint.data.local_frame2.translation
                - *steps.at(j).local_com;
            out.append(JlBuilder { joint, i, j });
        } else {
            out.append(JlBuilder { joint, i: WORLD, j: WORLD });
        }
    }
    out
}
fn rebuild_jl(
    ref bodies: DenseBodies,
    mut builders: Span<JlBuilder>,
    old: Span<JointConstraint>,
    p: IntegrationParameters,
    reuse: bool,
    public: bool,
) -> Array<JointConstraint> {
    let mut out = array![];
    while let Some(builder) = builders.pop_front() {
        let JlBuilder { mut joint, i, j } = *builder;
        if reuse {
            (*old.at(out.len())).writeback_impulses(ref joint);
        }
        let pair = [bodies.get(i), bodies.get(j)];
        let mut row = if public {
            JointConstraintTrait::generate(joint, pair.span(), p)
        } else {
            generate_public_jl(joint, pair.span(), p)
        };
        row.solver_vel1 = i;
        row.solver_vel2 = j;
        out.append(row);
    }
    out
}
/// `array_joint::joints` over JL's bounded solve.
fn joints_jl(ref rows: Array<JointConstraint>, ref bodies: DenseBodies, biased: bool, warm: bool) {
    let mut out = array![];
    while let Some(mut c) = rows.pop_front() {
        if c.num_rows != 0 {
            let i = c.solver_vel1;
            let j = c.solver_vel2;
            let mut pair = array![bodies.get(i), bodies.get(j)];
            c.solver_vel1 = 0;
            c.solver_vel2 = 1;
            if warm {
                warmstart_public_jl(c, ref pair);
            }
            solve_public_jl(ref c, ref pair, biased);
            bodies.set_pair(i, *pair.at(0), j, *pair.at(1));
            c.solver_vel1 = i;
            c.solver_vel2 = j;
        }
        out.append(c);
    }
    rows = out;
}
/// One contact-free island frame, as `empty::run` without damping.
fn frame(
    jm: u8,
    ref bodies: DenseBodies,
    ref js: Array<ImpulseJoint>,
    steps: Span<BodyStep>,
    p: IntegrationParameters,
) {
    let mut initial = array![];
    let mut k = 0;
    while k != bodies.len() {
        initial.append(bodies.get(k));
        k += 1;
    }
    let rows = if jm != 0 && jm != 5 {
        let builders = prepare_joints(js.span(), initial.span(), steps);
        substeps(ref bodies, builders.span(), [].span(), steps, p, jm)
    } else {
        let builders = prepare_jl(js.span(), initial.span(), steps);
        substeps(ref bodies, [].span(), builders.span(), steps, p, jm)
    };
    write_joints(rows.span(), ref js);
}
fn substeps(
    ref bodies: DenseBodies,
    jm: Span<JointBuilder>,
    jl: Span<JlBuilder>,
    steps: Span<BodyStep>,
    p: IntegrationParameters,
    variant: u8,
) -> Array<JointConstraint> {
    let dt = p.substep_dt();
    let max_ang = Fixed { raw: 3373259426 } * p.inv_dt();
    let mut rows = array![];
    let mut substep = 0;
    while substep != p.num_solver_iterations {
        add_forces(ref bodies, steps);
        rows =
            if variant == 1 {
                rebuild_joints(ref bodies, jm, rows.span(), p, substep != 0)
            } else if variant == 2 {
                rebuild_plain_bound(ref bodies, jm, rows.span(), p, substep != 0)
            } else if variant == 3 {
                rebuild_two_arms(ref bodies, jm, rows.span(), p, substep != 0)
            } else if variant == 4 {
                rebuild_metered(ref bodies, jm, rows.span(), p, substep != 0)
            } else {
                rebuild_jl(ref bodies, jl, rows.span(), p, substep != 0, variant == 5)
            };
        if variant == 0 {
            joints_jl(ref rows, ref bodies, true, p.warmstart_joints);
        } else {
            array_joint::joints(ref rows, ref bodies, true, p.warmstart_joints);
        }
        integrate(ref bodies, steps, dt, p.max_linear_velocity(), max_ang);
        if variant == 0 {
            joints_jl(ref rows, ref bodies, false, false);
        } else {
            array_joint::joints(ref rows, ref bodies, false, false);
        }
        substep += 1;
    }
    rows
}

fn probe(kind: u8, run: bool, jm: u8) {
    let (bs, mut js, steps) = chain(opaque(1), opaque(kind));
    let p: IntegrationParameters = opaque(Default::default());
    let mut bodies: DenseBodies = DenseBodiesTrait::new(bs.span());
    // Metered so the setup probe does not pay the frame's Sierra gas.
    let mut pending = run;
    while pending {
        frame(jm, ref bodies, ref js, steps.span(), p);
        pending = false;
    }
    let _ = opaque((js.span(), bodies.get(1)));
}
#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}
#[test]
fn gas_revolute_setup() {
    probe(0, false, 1);
}
#[test]
fn gas_revolute_jl() {
    probe(0, true, 0);
}
#[test]
fn gas_revolute_jm() {
    probe(0, true, 1);
}
#[test]
fn gas_prismatic_setup() {
    probe(1, false, 1);
}
#[test]
fn gas_prismatic_jl() {
    probe(1, true, 0);
}
#[test]
fn gas_prismatic_jm() {
    probe(1, true, 1);
}
#[test]
fn gas_fixed_setup() {
    probe(2, false, 1);
}
#[test]
fn gas_fixed_jl() {
    probe(2, true, 0);
}
#[test]
fn gas_fixed_jm() {
    probe(2, true, 1);
}
#[test]
fn gas_inactive_limit_setup() {
    probe(3, false, 1);
}
#[test]
fn gas_inactive_limit_jl() {
    probe(3, true, 0);
}
#[test]
fn gas_inactive_limit_jm() {
    probe(3, true, 1);
}
#[test]
fn gas_active_limit_setup() {
    probe(4, false, 1);
}
#[test]
fn gas_active_limit_jl() {
    probe(4, true, 0);
}
#[test]
fn gas_active_limit_jm() {
    probe(4, true, 1);
}
#[test]
fn gas_velocity_motor_setup() {
    probe(5, false, 1);
}
#[test]
fn gas_velocity_motor_jl() {
    probe(5, true, 0);
}
#[test]
fn gas_velocity_motor_jm() {
    probe(5, true, 1);
}
#[test]
fn gas_position_motor_setup() {
    probe(6, false, 1);
}
#[test]
fn gas_position_motor_jl() {
    probe(6, true, 0);
}
#[test]
fn gas_position_motor_jm() {
    probe(6, true, 1);
}
#[test]
fn gas_prismatic_limit_setup() {
    probe(7, false, 1);
}
#[test]
fn gas_prismatic_limit_jl() {
    probe(7, true, 0);
}
#[test]
fn gas_prismatic_limit_jm() {
    probe(7, true, 1);
}

fn fx(raw: i16, shift: i64) -> Fixed {
    let raw: i64 = raw.into();
    Fixed { raw: raw * shift }
}
/// Joint `k` of a random chain: a kind 0..=7 (see `joint_data`) or a free joint (8, 9) with
/// limits and motors on several axes, ranges centred near the current angle/offset.
fn random_joint(code: u32, range: i16, target: i16) -> GenericJoint {
    let a = Vec2 { x: ONE, y: ZERO };
    let b = Vec2 { x: -ONE, y: ZERO };
    let (rest, kind) = DivRem::div_rem(code, 10);
    let kind: u8 = kind.try_into().unwrap();
    let width: i64 = range.into();
    let width = if width < 0 {
        -width
    } else {
        width
    };
    let mut lo = fx(range, 8192) - fx(target, 4096);
    let mut hi = lo + Fixed { raw: width * 16384 + (rest % 3).into() };
    match rest % 5 {
        0 => {
            lo = MIN;
            hi = MAX;
        },
        1 => {
            let t = lo;
            lo = hi;
            hi = t;
        },
        _ => {},
    }
    let mut data = joint_data(if kind < 8 {
        kind
    } else {
        0
    }, a, b);
    if kind == 3 || kind == 4 {
        data.set_limits(2, [lo, hi]);
    }
    if kind >= 8 {
        data.locked_axes = JointAxesMask { bits: if kind == 8 {
            0
        } else {
            1
        } };
        data.set_limits(0, [lo, hi]);
        data.set_limits(2, [lo, hi]);
        data.set_motor_velocity(1, fx(target, 65536), HALF);
        data.set_motor_position(2, fx(target, 32768), ONE, HALF);
        if rest % 2 == 0 {
            data.coupled_axes = JointAxesMask { bits: 1 };
        }
    }
    if kind == 5 && rest % 2 == 0 {
        data.set_limits(2, [lo, hi]);
    }
    if kind == 2 && rest % 7 == 0 {
        // Out-of-range but unreachable mask (every axis locked): the legacy kind, no panic.
        data.limit_axes = JointAxesMask { bits: 8 };
    }
    data
}

#[test]
#[fuzzer(runs: 24, seed: 3104)]
fn fuzz_island_rows_bit_identical(code: u32, v: i16, range: i16, target: i16, warm: bool) {
    let (bs, mut js, steps) = chain(3, 0);
    let mut out = array![];
    let mut k: u32 = 0;
    for j in js.span() {
        let code = (code / (k * 97 + 1)) % 1000;
        let mut j = *j;
        j.data = random_joint(code, range, target);
        out.append(j);
        k += 1;
    }
    let mut js = out;
    let mut moved = array![];
    for body in bs.span() {
        let mut body = *body;
        body.angvel = fx(v, 65536);
        body.linvel.x = fx(-v, 32768);
        moved.append(body);
    }
    let p = IntegrationParameters {
        warmstart_joints: warm, warmstart_coefficient: HALF, ..Default::default(),
    };
    let mut a: DenseBodies = DenseBodiesTrait::new(moved.span());
    let mut b: DenseBodies = DenseBodiesTrait::new(moved.span());
    let mut ja = js.clone();
    let mut jc = js.clone();
    let mut c: DenseBodies = DenseBodiesTrait::new(moved.span());
    let mut frames = 0;
    while frames != 3 {
        frame(0, ref a, ref ja, steps.span(), p);
        frame(1, ref b, ref js, steps.span(), p);
        // The public per-substep API (every row, joint copies) over JM's single-frame rows.
        frame(5, ref c, ref jc, steps.span(), p);
        assert_eq!(ja.span(), js.span());
        assert_eq!(ja.span(), jc.span());
        let mut k = 0;
        while k != 4 {
            assert_eq!(a.get(k), b.get(k));
            assert_eq!(a.get(k), c.get(k));
            k += 1;
        }
        frames += 1;
    }
}

/// Limits entering and leaving their range across frames (impulses persisted between frames,
/// step-initial impulses nonzero), motors and a free joint, against JL for eight frames.
/// Returns how often joint 0's limit impulse turned on and off between frames.
fn transitions(cases: Span<(u32, i16)>) -> (u32, u32) {
    let mut on = 0;
    let mut off = 0;
    for (code, v) in cases {
        let (bs, js, steps) = chain(2, 0);
        let mut ja = array![];
        for j in js.span() {
            let mut j = *j;
            j.data = random_joint(*code, 2000, -1000);
            ja.append(j);
        }
        let mut jb = ja.clone();
        let mut moved = array![];
        for body in bs.span() {
            let mut body = *body;
            body.angvel = fx(*v, 65536 * 8);
            body.linvel.y = fx(-*v, 65536);
            moved.append(body);
        }
        let p = IntegrationParameters {
            warmstart_joints: true, warmstart_coefficient: HALF, ..Default::default(),
        };
        let mut a: DenseBodies = DenseBodiesTrait::new(moved.span());
        let mut b: DenseBodies = DenseBodiesTrait::new(moved.span());
        let mut frames = 0;
        let mut active = false;
        while frames != 8 {
            frame(0, ref a, ref ja, steps.span(), p);
            frame(1, ref b, ref jb, steps.span(), p);
            assert_eq!(ja.span(), jb.span());
            assert_eq!(a.get(1), b.get(1));
            assert_eq!(a.get(2), b.get(2));
            let [lx, _, lw] = (*jb.at(0)).data.limits;
            let now = lx.impulse != ZERO || lw.impulse != ZERO;
            if now && !active {
                on += 1;
            }
            if active && !now {
                off += 1;
            }
            active = now;
            frames += 1;
        }
    }
    (on, off)
}
#[test]
fn test_island_rows_bit_identical_across_limit_transitions() {
    let (on, off) = transitions(
        [(23_u32, 30000_i16), (24, -30000), (13, -25000), (7, 30000), (33, 20000)].span(),
    );
    assert!(on >= 4 && off >= 4);
}
#[test]
fn test_island_rows_bit_identical_motors_free_and_legacy() {
    let _ = transitions(
        [(5_u32, 20000_i16), (6, -20000), (8, 25000), (19, -30000), (70, 5000)].span(),
    );
}

// Rebuild dispatch candidates (the shipped one matches the kind in the loop body): the plain-only
// lower bound, the plain arm inline with the other kinds outlined, and the other kinds metered.
fn rebuild_plain_bound(
    ref bodies: DenseBodies,
    mut builders: Span<JointBuilder>,
    old: Span<JointConstraint>,
    p: IntegrationParameters,
    reuse: bool,
) -> Array<JointConstraint> {
    let mut out = array![];
    while let Some(builder) = builders.pop_front() {
        let JointBuilder { joint, kind: _, impulses, i, j } = *builder;
        let mut row = if i == WORLD {
            Default::default()
        } else {
            let mut seeds = impulses;
            if reuse {
                let c = *old.at(out.len());
                write_rows(c.rows, c.num_rows, ref seeds);
            }
            assert(i != j, super::super::super::joint::errors::SAME_BODY);
            step::plain(joint, seeds, bodies.get(i), bodies.get(j), p)
        };
        row.solver_vel1 = i;
        row.solver_vel2 = j;
        out.append(row);
    }
    out
}
#[inline(never)]
fn other_kinds(
    ref bodies: DenseBodies,
    joint: StepJoint,
    kind: StepKind,
    impulses: [Fixed; 3],
    i: u32,
    j: u32,
    old: Span<JointConstraint>,
    index: u32,
    p: IntegrationParameters,
    reuse: bool,
) -> JointConstraint {
    match kind {
        StepKind::Controlled(controls) => {
            let controls = controls.unbox();
            let (seeds, motors, limits) = if reuse {
                step::carried(*old.at(index), impulses)
            } else {
                (impulses, controls.motors, controls.limits)
            };
            assert(i != j, super::super::super::joint::errors::SAME_BODY);
            step::generate(
                joint, controls, seeds, motors, limits, bodies.get(i), bodies.get(j), p, true,
            )
        },
        StepKind::Legacy(legacy) => {
            let mut joint = legacy.unbox();
            if reuse {
                (*old.at(index)).writeback_impulses(ref joint);
            }
            let pair = [bodies.get(i), bodies.get(j)];
            JointConstraintTrait::generate(joint, pair.span(), p)
        },
        StepKind::Plain => Default::default(),
    }
}
fn rebuild_two_arms(
    ref bodies: DenseBodies,
    mut builders: Span<JointBuilder>,
    old: Span<JointConstraint>,
    p: IntegrationParameters,
    reuse: bool,
) -> Array<JointConstraint> {
    let mut out = array![];
    while let Some(builder) = builders.pop_front() {
        let JointBuilder { joint, kind, impulses, i, j } = *builder;
        let mut row = if i == WORLD {
            Default::default()
        } else if let StepKind::Plain = kind {
            let mut seeds = impulses;
            if reuse {
                let c = *old.at(out.len());
                write_rows(c.rows, c.num_rows, ref seeds);
            }
            assert(i != j, super::super::super::joint::errors::SAME_BODY);
            step::plain(joint, seeds, bodies.get(i), bodies.get(j), p)
        } else {
            other_kinds(ref bodies, joint, kind, impulses, i, j, old, out.len(), p, reuse)
        };
        row.solver_vel1 = i;
        row.solver_vel2 = j;
        out.append(row);
    }
    out
}
fn rebuild_metered(
    ref bodies: DenseBodies,
    mut builders: Span<JointBuilder>,
    old: Span<JointConstraint>,
    p: IntegrationParameters,
    reuse: bool,
) -> Array<JointConstraint> {
    let mut out = array![];
    while let Some(builder) = builders.pop_front() {
        let JointBuilder { joint, kind, impulses, i, j } = *builder;
        let mut row: JointConstraint = Default::default();
        let mut pending = false;
        if i != WORLD {
            if let StepKind::Plain = kind {
                let mut seeds = impulses;
                if reuse {
                    let c = *old.at(out.len());
                    write_rows(c.rows, c.num_rows, ref seeds);
                }
                assert(i != j, super::super::super::joint::errors::SAME_BODY);
                row = step::plain(joint, seeds, bodies.get(i), bodies.get(j), p);
            } else {
                pending = true;
            }
        }
        while pending {
            row = other_kinds(ref bodies, joint, kind, impulses, i, j, old, out.len(), p, reuse);
            pending = false;
        }
        row.solver_vel1 = i;
        row.solver_vel2 = j;
        out.append(row);
    }
    out
}
#[test]
fn gas_dispatch_revolute_bound() {
    probe(0, true, 2);
}
#[test]
fn gas_dispatch_revolute_two_arms() {
    probe(0, true, 3);
}
#[test]
fn gas_dispatch_revolute_metered() {
    probe(0, true, 4);
}
#[test]
fn gas_dispatch_active_limit_two_arms() {
    probe(4, true, 3);
}
#[test]
fn gas_dispatch_active_limit_metered() {
    probe(4, true, 4);
}
/// Island stages of one joint, each metered: 0 generate, 1 warmstart + biased solve, 2 relaxed
/// solve, 3 regenerate from the previous rows, 4 final write, 5 prepare (specialisation).
fn stage_probe(kind: u8, stage: u8, run: bool) {
    let (bs, mut js, steps) = chain(opaque(1), opaque(kind));
    let p: IntegrationParameters = opaque(Default::default());
    let mut bodies: DenseBodies = DenseBodiesTrait::new(bs.span());
    let builders = prepare_joints(js.span(), bs.span(), steps.span());
    let mut rows = rebuild_joints(ref bodies, builders.span(), [].span(), p, false);
    let mut pending = run;
    while pending {
        match stage {
            0 => { rows = rebuild_joints(ref bodies, builders.span(), [].span(), p, false); },
            1 => array_joint::joints(ref rows, ref bodies, true, true),
            2 => array_joint::joints(ref rows, ref bodies, false, false),
            3 => { rows = rebuild_joints(ref bodies, builders.span(), rows.span(), p, true); },
            5 => { let _ = opaque(prepare_joints(js.span(), bs.span(), steps.span())); },
            4 => write_joints(rows.span(), ref js),
            _ => {},
        }
        pending = false;
    }
    let _ = opaque((js.span(), rows.span(), bodies.get(1)));
}
#[test]
fn gas_stage_revolute_setup() {
    stage_probe(0, 0, false);
}
#[test]
fn gas_stage_revolute_gen() {
    stage_probe(0, 0, true);
}
#[test]
fn gas_stage_revolute_warm_biased() {
    stage_probe(0, 1, true);
}
#[test]
fn gas_stage_revolute_relaxed() {
    stage_probe(0, 2, true);
}
#[test]
fn gas_stage_revolute_regen() {
    stage_probe(0, 3, true);
}
#[test]
fn gas_stage_revolute_write() {
    stage_probe(0, 4, true);
}
#[test]
fn gas_stage_revolute_prepare() {
    stage_probe(0, 5, true);
}
#[test]
fn gas_stage_inactive_limit_setup() {
    stage_probe(3, 0, false);
}
#[test]
fn gas_stage_inactive_limit_gen() {
    stage_probe(3, 0, true);
}
#[test]
fn gas_stage_inactive_limit_warm_biased() {
    stage_probe(3, 1, true);
}
#[test]
fn gas_stage_inactive_limit_relaxed() {
    stage_probe(3, 2, true);
}
#[test]
fn gas_stage_inactive_limit_regen() {
    stage_probe(3, 3, true);
}
#[test]
fn gas_stage_inactive_limit_write() {
    stage_probe(3, 4, true);
}
#[test]
fn gas_stage_inactive_limit_prepare() {
    stage_probe(3, 5, true);
}
#[test]
fn gas_stage_active_limit_setup() {
    stage_probe(4, 0, false);
}
#[test]
fn gas_stage_active_limit_gen() {
    stage_probe(4, 0, true);
}
#[test]
fn gas_stage_active_limit_warm_biased() {
    stage_probe(4, 1, true);
}
#[test]
fn gas_stage_active_limit_relaxed() {
    stage_probe(4, 2, true);
}
#[test]
fn gas_stage_active_limit_regen() {
    stage_probe(4, 3, true);
}
#[test]
fn gas_stage_active_limit_write() {
    stage_probe(4, 4, true);
}
#[test]
fn gas_stage_active_limit_prepare() {
    stage_probe(4, 5, true);
}
#[test]
fn gas_stage_velocity_motor_setup() {
    stage_probe(5, 0, false);
}
#[test]
fn gas_stage_velocity_motor_gen() {
    stage_probe(5, 0, true);
}
#[test]
fn gas_stage_velocity_motor_warm_biased() {
    stage_probe(5, 1, true);
}
#[test]
fn gas_stage_velocity_motor_relaxed() {
    stage_probe(5, 2, true);
}
#[test]
fn gas_stage_velocity_motor_regen() {
    stage_probe(5, 3, true);
}
#[test]
fn gas_stage_velocity_motor_write() {
    stage_probe(5, 4, true);
}
#[test]
fn gas_stage_velocity_motor_prepare() {
    stage_probe(5, 5, true);
}
#[test]
fn gas_stage_position_motor_setup() {
    stage_probe(6, 0, false);
}
#[test]
fn gas_stage_position_motor_gen() {
    stage_probe(6, 0, true);
}
#[test]
fn gas_stage_position_motor_warm_biased() {
    stage_probe(6, 1, true);
}
#[test]
fn gas_stage_position_motor_relaxed() {
    stage_probe(6, 2, true);
}
#[test]
fn gas_stage_position_motor_regen() {
    stage_probe(6, 3, true);
}
#[test]
fn gas_stage_position_motor_write() {
    stage_probe(6, 4, true);
}
#[test]
fn gas_stage_position_motor_prepare() {
    stage_probe(6, 5, true);
}
