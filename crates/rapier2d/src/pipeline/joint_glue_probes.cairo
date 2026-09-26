//! Probes of the joint glue of the step (`joint_values`, `write_joints`).
use rapier_dynamics2d::joint::RevoluteJointBuilderTrait;
use rapier_testing::opaque;
use super::*;
#[inline(always)]
fn probe(stage: u8) {
    let mut set = ImpulseJointSetTrait::new();
    let mut i = 0;
    while i != 3 {
        let _ = set
            .insert(
                opaque(Handle { index: i, generation: 1 }),
                opaque(Handle { index: i + 1, generation: 1 }),
                opaque(RevoluteJointBuilderTrait::new().build()),
            );
        i += 1;
    }
    let entries = set.to_array();
    if stage != 0 {
        let values = joint_values(opaque(entries.span()));
        if stage == 2 {
            write_joints(opaque(entries.span()), opaque(values.span()), ref set);
        }
        let _ = opaque(values.span());
    }
    let _ = opaque(set.len());
}
#[test]
fn gas_baseline() {
    probe(0);
}
#[test]
fn gas_joint_values3() {
    probe(1);
}
#[test]
fn gas_write_joints3() {
    probe(2);
}
