//! Separate anchors/frames and Gram-Schmidt costs, net of matched setup probes.
use fixed::ONE;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_math::pose2::Pose2Trait;
use rapier_testing::opaque;
use crate::solver::joint::alternatives::original;
use crate::solver::joint::{JointConstraintHelperTrait, prepare};
use super::chain;

#[inline(always)]
fn probe(kind: u8, stage: u8) {
    let (bs, js, _) = chain(opaque(1), opaque(kind));
    let joint = *js.at(0);
    let p: IntegrationParameters = opaque(Default::default());
    if stage == 1 {
        let b1 = *bs.at(0);
        let b2 = *bs.at(1);
        let f1 = b1.position.mul(joint.data.local_frame1);
        let f2 = b2.position.mul(joint.data.local_frame2);
        let h = JointConstraintHelperTrait::new(
            f1, f2, b1.position.translation, b2.position.translation, joint.data.locked_axes,
        );
        let _ = opaque(h);
    } else if stage != 0 {
        let (mut c, h, b1, b2, erp, cfm) = prepare(joint, bs.span(), p);
        match kind {
            0 => {
                c
                    .rows =
                        [
                            h.lock_linear(0, b1, b2, erp, cfm), h.lock_linear(1, b1, b2, erp, cfm),
                            Default::default(),
                        ];
                c.num_rows = 2;
            },
            1 => {
                c
                    .rows =
                        [
                            h.lock_angular(b1, b2, erp, cfm), h.lock_linear(1, b1, b2, erp, cfm),
                            Default::default(),
                        ];
                c.num_rows = 2;
            },
            _ => {
                c
                    .rows =
                        [
                            h.lock_angular(b1, b2, erp, cfm), h.lock_linear(0, b1, b2, erp, cfm),
                            h.lock_linear(1, b1, b2, erp, cfm),
                        ];
                c.num_rows = 3;
            },
        }
        let mut c = opaque(c);
        if stage == 3 {
            original::finalize(ref c);
        }
        if stage == 4 {
            JointConstraintHelperTrait::finalize(ref c);
        }
        let _ = opaque(c);
    }
    let _ = opaque((bs.span(), js.span(), p));
}
#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}
#[test]
fn gas_revolute_setup() {
    probe(0, 0);
}
#[test]
fn gas_revolute_frames() {
    probe(0, 1);
}
#[test]
fn gas_revolute_rows() {
    probe(0, 2);
}
#[test]
fn gas_revolute_gram_old() {
    probe(0, 3);
}
#[test]
fn gas_revolute_gram_selected() {
    probe(0, 4);
}
#[test]
fn gas_prismatic_setup() {
    probe(1, 0);
}
#[test]
fn gas_prismatic_frames() {
    probe(1, 1);
}
#[test]
fn gas_prismatic_rows() {
    probe(1, 2);
}
#[test]
fn gas_prismatic_gram_old() {
    probe(1, 3);
}
#[test]
fn gas_prismatic_gram_selected() {
    probe(1, 4);
}
#[test]
fn gas_fixed_setup() {
    probe(2, 0);
}
#[test]
fn gas_fixed_frames() {
    probe(2, 1);
}
#[test]
fn gas_fixed_rows() {
    probe(2, 2);
}
#[test]
fn gas_fixed_gram_old() {
    probe(2, 3);
}
#[test]
fn gas_fixed_gram_selected() {
    probe(2, 4);
}
