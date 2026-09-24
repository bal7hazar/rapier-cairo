//! Bit-for-bit multi-substep replay through the public joint API.
use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam::Vec2;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_math::rot2::Rot2Trait;
use rapier_testing::opaque;
use crate::joint::ImpulseJoint;
use crate::rigid_body::{RigidBodyVelocity, RigidBodyVelocityTrait};
use crate::solver::body::SolverBody;
use crate::solver::joint::{JointConstraint, JointConstraintTrait};
use super::chain;
use super::super::alternatives::original;

trait Kernel {
    fn generate(j: ImpulseJoint, bs: Span<SolverBody>, p: IntegrationParameters) -> JointConstraint;
    fn warmstart(c: JointConstraint, ref bs: Array<SolverBody>);
    fn solve(ref c: JointConstraint, ref bs: Array<SolverBody>, biased: bool);
    fn write(c: JointConstraint, ref j: ImpulseJoint);
}
impl Old of Kernel {
    #[inline(always)]
    fn generate(
        j: ImpulseJoint, bs: Span<SolverBody>, p: IntegrationParameters,
    ) -> JointConstraint {
        original::generate(j, bs, p)
    }
    #[inline(always)]
    fn warmstart(c: JointConstraint, ref bs: Array<SolverBody>) {
        original::warmstart(c, ref bs);
    }
    #[inline(always)]
    fn solve(ref c: JointConstraint, ref bs: Array<SolverBody>, biased: bool) {
        original::solve(ref c, ref bs, biased);
    }
    #[inline(always)]
    fn write(c: JointConstraint, ref j: ImpulseJoint) {
        original::writeback_impulses(c, ref j);
    }
}
impl Selected of Kernel {
    #[inline(always)]
    fn generate(
        j: ImpulseJoint, bs: Span<SolverBody>, p: IntegrationParameters,
    ) -> JointConstraint {
        JointConstraintTrait::generate(j, bs, p)
    }
    #[inline(always)]
    fn warmstart(c: JointConstraint, ref bs: Array<SolverBody>) {
        c.warmstart(ref bs);
    }
    #[inline(always)]
    fn solve(ref c: JointConstraint, ref bs: Array<SolverBody>, biased: bool) {
        c.solve(ref bs, biased);
    }
    #[inline(always)]
    fn write(c: JointConstraint, ref j: ImpulseJoint) {
        c.writeback_impulses(ref j);
    }
}

// Identical driver, preserving angular/X/Y row order and warm-before-biased ordering.
// The public Array adapter is intentionally exercised with all chain bodies.
fn replay<impl K: Kernel>(
    ref bs: Array<SolverBody>, ref js: Array<ImpulseJoint>, p: IntegrationParameters,
) {
    let dt = p.substep_dt();
    let gravity = Fixed { raw: -42133629174 } * dt;
    let mut sub = 0;
    while sub != p.num_solver_iterations {
        let mut out = array![];
        while let Some(mut b) = bs.pop_front() {
            if b.im.y != ZERO {
                b.linvel.y += gravity;
            }
            out.append(b);
        }
        bs = out;
        let mut rows = array![];
        for j in js.span() {
            rows.append(K::generate(*j, bs.span(), p));
        }
        let mut out = array![];
        while let Some(mut c) = rows.pop_front() {
            if p.warmstart_joints {
                K::warmstart(c, ref bs);
            }
            K::solve(ref c, ref bs, true);
            out.append(c);
        }
        rows = out;
        let mut out = array![];
        while let Some(mut b) = bs.pop_front() {
            if b.im.y != ZERO {
                b
                    .position = RigidBodyVelocity { linvel: b.linvel, angvel: b.angvel }
                    .integrate(dt, b.position, Default::default());
            }
            out.append(b);
        }
        bs = out;
        let mut out = array![];
        while let Some(mut c) = rows.pop_front() {
            K::solve(ref c, ref bs, false);
            let mut j = js.pop_front().unwrap();
            K::write(c, ref j);
            out.append(j);
        }
        js = out;
        sub += 1;
    }
}

fn input(
    n: u32, kind: u8, x: Fixed, y: Fixed, warm: bool, soft: bool,
) -> (Array<SolverBody>, Array<ImpulseJoint>, IntegrationParameters) {
    let (mut bs, mut js, _) = chain(n, kind);
    let mut out = array![];
    while let Some(mut b) = bs.pop_front() {
        if b.im.y != ZERO {
            let scale = FixedTrait::from_int(b.handle.index.try_into().unwrap());
            b.position.rotation = Rot2Trait::from_cos_sin(ONE, x * scale);
            b.position.translation.y += y * scale;
            b.linvel = Vec2 { x: x * scale, y: y * scale };
            b.angvel = -x * scale;
        }
        out.append(b);
    }
    bs = out;
    let mut out = array![];
    while let Some(mut j) = js.pop_front() {
        j.data.local_frame1.rotation = Rot2Trait::from_cos_sin(ONE, y);
        j.data.local_frame2.rotation = Rot2Trait::from_cos_sin(ONE, -y);
        j.impulses = [x, -y, HALF];
        if soft {
            j.data.softness.natural_frequency = FixedTrait::from_int(7);
        }
        out.append(j);
    }
    (
        bs,
        out,
        IntegrationParameters {
            warmstart_joints: warm, warmstart_coefficient: HALF, ..Default::default(),
        },
    )
}

fn compare(n: u32, kind: u8, x: i16, y: i16, warm: bool, soft: bool) {
    let x = Fixed { raw: x.into() * 32768 };
    let y = Fixed { raw: y.into() * 32768 };
    let (mut a, mut ja, p) = input(n, kind, x, y, warm, soft);
    let mut b = array![];
    b.append_span(a.span());
    let mut jb = array![];
    jb.append_span(ja.span());
    let mut frame = 0;
    while frame != 2 {
        replay::<Old>(ref a, ref ja, p);
        replay::<Selected>(ref b, ref jb, p);
        assert_eq!(a.span(), b.span());
        assert_eq!(ja.span(), jb.span());
        frame += 1;
    }
}

#[test]
#[fuzzer(runs: 32, seed: 9242026)]
fn fuzz_random_joint_chains(x: i16, y: i16, choice: u8, warm: bool, soft: bool) {
    let (q, kind) = core::num::traits::DivRem::div_rem(choice, 3);
    let (_, size) = core::num::traits::DivRem::div_rem(q, 3);
    let n = match size {
        0 => 1,
        1 => 3,
        _ => 8,
    };
    compare(n, kind, x, y, warm, soft);
}
#[test]
fn test_chain_sizes_types_and_warmstart() {
    for n in [1_u32, 3, 8].span() {
        for kind in [0_u8, 1, 2].span() {
            for warm in [false, true].span() {
                compare(*n, *kind, 1234, -2345, *warm, *warm);
            }
        }
    }
}

fn probe<impl K: Kernel>(n: u32, kind: u8, run: bool) {
    let (mut bs, mut js, p) = input(
        opaque(n),
        opaque(kind),
        opaque(Fixed { raw: 12345678 }),
        opaque(Fixed { raw: -23456789 }),
        opaque(true),
        opaque(false),
    );
    if run {
        replay::<K>(ref bs, ref js, opaque(p));
    }
    let _ = opaque((bs.span(), js.span()));
}
#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}
#[test]
fn gas_revolute1_setup() {
    probe::<Selected>(1, 0, false);
}
#[test]
fn gas_revolute1_old() {
    probe::<Old>(1, 0, true);
}
#[test]
fn gas_revolute1_selected() {
    probe::<Selected>(1, 0, true);
}
#[test]
fn gas_revolute3_setup() {
    probe::<Selected>(3, 0, false);
}
#[test]
fn gas_revolute3_old() {
    probe::<Old>(3, 0, true);
}
#[test]
fn gas_revolute3_selected() {
    probe::<Selected>(3, 0, true);
}
#[test]
fn gas_revolute8_setup() {
    probe::<Selected>(8, 0, false);
}
#[test]
fn gas_revolute8_old() {
    probe::<Old>(8, 0, true);
}
#[test]
fn gas_revolute8_selected() {
    probe::<Selected>(8, 0, true);
}
#[test]
fn gas_prismatic1_setup() {
    probe::<Selected>(1, 1, false);
}
#[test]
fn gas_prismatic1_old() {
    probe::<Old>(1, 1, true);
}
#[test]
fn gas_prismatic1_selected() {
    probe::<Selected>(1, 1, true);
}
#[test]
fn gas_prismatic3_setup() {
    probe::<Selected>(3, 1, false);
}
#[test]
fn gas_prismatic3_old() {
    probe::<Old>(3, 1, true);
}
#[test]
fn gas_prismatic3_selected() {
    probe::<Selected>(3, 1, true);
}
#[test]
fn gas_prismatic8_setup() {
    probe::<Selected>(8, 1, false);
}
#[test]
fn gas_prismatic8_old() {
    probe::<Old>(8, 1, true);
}
#[test]
fn gas_prismatic8_selected() {
    probe::<Selected>(8, 1, true);
}
#[test]
fn gas_fixed1_setup() {
    probe::<Selected>(1, 2, false);
}
#[test]
fn gas_fixed1_old() {
    probe::<Old>(1, 2, true);
}
#[test]
fn gas_fixed1_selected() {
    probe::<Selected>(1, 2, true);
}
#[test]
fn gas_fixed3_setup() {
    probe::<Selected>(3, 2, false);
}
#[test]
fn gas_fixed3_old() {
    probe::<Old>(3, 2, true);
}
#[test]
fn gas_fixed3_selected() {
    probe::<Selected>(3, 2, true);
}
#[test]
fn gas_fixed8_setup() {
    probe::<Selected>(8, 2, false);
}
#[test]
fn gas_fixed8_old() {
    probe::<Old>(8, 2, true);
}
#[test]
fn gas_fixed8_selected() {
    probe::<Selected>(8, 2, true);
}
