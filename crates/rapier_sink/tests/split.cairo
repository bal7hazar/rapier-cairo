//! The split layouts against the in-process step on pile10 (`pile10`): bit-identical digests, and
//! the exact Cairo steps of each (`steps_*`, run with `--tracked-resource cairo-steps`; snforge
//! counts the steps of the library-called classes in the test's).

use rapier2d::prelude::BasicStepConfig;
use rapier_testing::opaque;
use rapier_sink::split::{CONTACT_CLASS_SLOT, SplitNarrowStepConfig};
use snforge_std::{DeclareResultTrait, declare};
use starknet::SyscallResultTrait;
use starknet::syscalls::{library_call_syscall, storage_write_syscall};
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

/// `n` library calls of `Echo::echo` with `len` felts each way.
fn echo(n: u32, len: u32) {
    let class_hash = *declare("Echo").unwrap_syscall().contract_class().class_hash;
    let mut data: Array<felt252> = array![];
    let mut i = 0;
    while i != len {
        data.append(i.into());
        i += 1;
    }
    let mut calldata = array![];
    data.span().serialize(ref calldata);
    let calldata = opaque(calldata);
    let mut k = 0;
    while k != n {
        let ret = library_call_syscall(class_hash, selector!("echo"), calldata.span())
            .unwrap_syscall();
        assert(ret.len() == len + 1, 'echo');
        k += 1;
    }
}

#[test]
fn steps_echo_0_0() {
    echo(0, 0);
}

#[test]
fn steps_echo_10_0() {
    echo(10, 0);
}

#[test]
fn steps_echo_10_100() {
    echo(10, 100);
}

#[test]
fn steps_echo_10_1000() {
    echo(10, 1000);
}

#[test]
fn steps_basic_0() {
    basic(0);
}

#[test]
fn steps_basic_10() {
    basic(10);
}

#[test]
fn steps_basic_20() {
    basic(20);
}

#[test]
fn steps_basic_30() {
    basic(30);
}

#[test]
fn steps_basic_40() {
    basic(40);
}

#[test]
fn steps_basic_42() {
    basic(42);
}

#[test]
fn steps_basic_43() {
    basic(43);
}

#[test]
fn steps_basic_44() {
    basic(44);
}

#[test]
fn steps_basic_50() {
    basic(50);
}

#[test]
fn steps_basic_60() {
    basic(60);
}

#[test]
fn steps_basic_70() {
    basic(70);
}

#[test]
fn steps_basic_80() {
    basic(80);
}

#[test]
fn steps_basic_90() {
    basic(90);
}

#[test]
fn steps_basic_100() {
    basic(100);
}

#[test]
fn steps_basic_110() {
    basic(110);
}

#[test]
fn steps_basic_120() {
    basic(120);
}

#[test]
fn steps_basic_130() {
    basic(130);
}

#[test]
fn steps_basic_140() {
    basic(140);
}

#[test]
fn steps_basic_150() {
    basic(150);
}

#[test]
fn steps_basic_151() {
    basic(151);
}

#[test]
fn steps_split_narrow_0() {
    split_narrow(0);
}

#[test]
fn steps_split_narrow_10() {
    split_narrow(10);
}

#[test]
fn steps_split_narrow_20() {
    split_narrow(20);
}

#[test]
fn steps_split_narrow_30() {
    split_narrow(30);
}

#[test]
fn steps_split_narrow_40() {
    split_narrow(40);
}

#[test]
fn steps_split_narrow_42() {
    split_narrow(42);
}

#[test]
fn steps_split_narrow_43() {
    split_narrow(43);
}

#[test]
fn steps_split_narrow_44() {
    split_narrow(44);
}

#[test]
fn steps_split_narrow_50() {
    split_narrow(50);
}

#[test]
fn steps_split_narrow_60() {
    split_narrow(60);
}

#[test]
fn steps_split_narrow_70() {
    split_narrow(70);
}

#[test]
fn steps_split_narrow_80() {
    split_narrow(80);
}

#[test]
fn steps_split_narrow_90() {
    split_narrow(90);
}

#[test]
fn steps_split_narrow_100() {
    split_narrow(100);
}

#[test]
fn steps_split_narrow_110() {
    split_narrow(110);
}

#[test]
fn steps_split_narrow_120() {
    split_narrow(120);
}

#[test]
fn steps_split_narrow_130() {
    split_narrow(130);
}

#[test]
fn steps_split_narrow_140() {
    split_narrow(140);
}

#[test]
fn steps_split_narrow_150() {
    split_narrow(150);
}

#[test]
fn steps_split_narrow_151() {
    split_narrow(151);
}
