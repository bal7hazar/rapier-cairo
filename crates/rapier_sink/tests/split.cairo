//! The split layouts against the in-process step on pile10 (`pile10`): bit-identical digests, and
//! the exact Cairo steps of each (`steps_*`, run with `--tracked-resource cairo-steps`; snforge
//! counts the steps of the library-called classes in the test's).

use rapier2d::prelude::BasicStepConfig;
use rapier_testing::opaque;
use rapier_sink::split::{CONTACT_CLASS_SLOT, SplitNarrowStepConfig};
use snforge_std::{DeclareResultTrait, declare};
use starknet::SyscallResultTrait;
use starknet::syscalls::storage_write_syscall;
use crate::pile10::{digest, run};

/// Declares `name` and stores its class hash at `slot` of the test contract's storage, where the
/// split dispatchers read it.
fn install(name: ByteArray, slot: felt252) {
    let class_hash = *declare(name).unwrap_syscall().contract_class().class_hash;
    storage_write_syscall(0, slot.try_into().unwrap(), class_hash.into()).unwrap_syscall();
}

fn basic(ticks: u32) {
    let (mut pile, _) = run::<BasicStepConfig>(ticks);
    opaque(digest(ref pile));
}

fn split_narrow(ticks: u32) {
    install("ContactClass", CONTACT_CLASS_SLOT);
    let (mut pile, _) = run::<SplitNarrowStepConfig>(ticks);
    opaque(digest(ref pile));
}

#[test]
fn test_split_narrow_bit_identical() {
    let ticks = 60;
    let (mut pile, events) = run::<BasicStepConfig>(ticks);
    let expected = digest(ref pile);
    install("ContactClass", CONTACT_CLASS_SLOT);
    let (mut pile, split_events) = run::<SplitNarrowStepConfig>(ticks);
    assert_eq!(split_events, events);
    assert_eq!(digest(ref pile), expected);
}

#[test]
fn test_trace() {
    let mut pile = crate::pile10::build::<BasicStepConfig>();
    crate::pile10::launch(ref pile, crate::pile10::PULL);
    let mut i = 1;
    while i != 152 {
        let n = crate::pile10::tick::<BasicStepConfig>(ref pile);
        println!("tick {} events {} pairs {}", i, n, pile.world.narrow_phase.pairs.len());
        i += 1;
    }
}

#[test]
fn steps_basic_0() {
    basic(0);
}

#[test]
fn steps_basic_30() {
    basic(30);
}

#[test]
fn steps_split_narrow_0() {
    split_narrow(0);
}

#[test]
fn steps_split_narrow_30() {
    split_narrow(30);
}
