//! Joint dispatch equivalence across zero to three rows, static endpoints and warm starts.
use fixed::{Fixed, HALF, ZERO};
use crate::joint::{GenericJoint, JointAxesMask};
use super::*;
use super::super::super::body_store::DenseBodies;

#[test]
#[fuzzer(runs: 16, seed: 924)]
fn fuzz_joint_stages(x: i16, y: i16, fixed: bool) {
    let (bs, _, _) = super::super::fixtures::stack(2);
    let mut first = *bs.at(0);
    let mut second = *bs.at(1);
    first.linvel.x = Fixed { raw: x.into() * 32768 };
    second.linvel.y = Fixed { raw: y.into() * 65536 };
    second.angvel = Fixed { raw: x.into() * 16384 };
    if fixed {
        first.im = Default::default();
        first.ii = ZERO;
    }
    let input = [first, second];
    let p = IntegrationParameters {
        warmstart_joints: true, warmstart_coefficient: HALF, ..Default::default(),
    };
    for bits in [0_u8, 1, 3, 6, 7].span() {
        let j = ImpulseJoint {
            body1: first.handle,
            body2: second.handle,
            data: GenericJoint { locked_axes: JointAxesMask { bits: *bits }, ..Default::default() },
            impulses: [HALF, -HALF, HALF],
        };
        let c = JointConstraintTrait::generate(j, input.span(), p);
        for variant in [0_u8, 1, 2, 3, 4, 5].span() {
            let mut old = array![c];
            let mut new = array![c];
            let mut a: DenseBodies = DenseBodiesTrait::new(input.span());
            let mut b: DenseBodies = DenseBodiesTrait::new(input.span());
            let mut dict = direct::alternatives::new_joints([c].span());
            for (biased, warm) in [(true, true), (true, false), (false, false)].span() {
                alternatives::joints_original(ref old, ref a, *biased, *warm);
                match *variant {
                    0 => array_joint::joints(ref new, ref b, *biased, *warm),
                    1 => direct::joints(ref new, ref b, *biased, *warm),
                    2 => metered_pair::joints(ref new, ref b, *biased, *warm),
                    3 => specialized::joints(ref new, ref b, *biased, *warm),
                    4 => alternatives::joints_skip_static(ref new, ref b, *biased, *warm),
                    _ => {
                        direct::alternatives::joints(ref dict, ref b, *biased, *warm);
                        new = direct::alternatives::finish_joints(ref dict);
                    },
                }
                assert_eq!(old.span(), new.span());
                assert_eq!(a.get(0), b.get(0));
                assert_eq!(a.get(1), b.get(1));
            }
        }
    }
}

#[test]
fn gas_baseline() {
    let _ = rapier_testing::opaque(fixed::ONE);
}
