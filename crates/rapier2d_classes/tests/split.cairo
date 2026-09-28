//! The split step against the in-process step on pile10 (`pile10`): identical worlds and events
//! after every tick, the library calls made, and the exact Cairo steps of each (`steps_*`, run
//! with `--tracked-resource cairo-steps`; snforge counts the steps of the library-called classes
//! in the test's).
//!
//! Two `ClassHashes` impls are measured: [`PinnedHashes`], the constants a game compiles (the
//! hashes snforge gives the classes of this package, checked by `test_pinned_class_hashes`), and
//! [`StoredHashes`], each hash read from a storage slot of the calling contract (CS3's
//! dispatchers), the loser.

use rapier2d::pipeline::config::{NoComposites, NoJoints, NoSensors};
use rapier2d::prelude::{BasicStepConfig, StepConfig};
use rapier2d_classes::{ClassHashes, FamilyDispatcher, SplitStepConfig};
use rapier_testing::opaque;
use snforge_std::{DeclareResultTrait, declare};
use starknet::syscalls::{storage_read_syscall, storage_write_syscall};
use starknet::{ClassHash, SyscallResultTrait};
use crate::pile10::{PULL, build, digest, launch, run, tick, tick_digest};

/// The class hashes snforge declares for this package's classes (`test_pinned_class_hashes`
/// prints the new ones when the classes change).
pub const CONTACT_BALL_HASH: felt252 =
    0x417a92c3eb60047608b0af1cf607b554935753f48ed8532d05e70546549ed88;
pub const CONTACT_POLYGON_HASH: felt252 =
    0x21f7f731102a52193774039ae57bd483bd8e090fb901fb55d5b786a2e281c45;
pub const SOLVER_HASH: felt252 = 0x447d2dddc0d41fd2ac6d384e96f468d300f0509756eeb49435f004eac8a33ef;

/// The classes of this package at their pinned hashes, as constants.
impl PinnedHashes of ClassHashes {
    fn contact_ball() -> ClassHash {
        const H: ClassHash = CONTACT_BALL_HASH.try_into().unwrap();
        H
    }

    fn contact_polygon() -> ClassHash {
        const H: ClassHash = CONTACT_POLYGON_HASH.try_into().unwrap();
        H
    }

    fn solver() -> ClassHash {
        const H: ClassHash = SOLVER_HASH.try_into().unwrap();
        H
    }
}

const BALL_SLOT: felt252 = selector!("contact_ball_class");
const POLYGON_SLOT: felt252 = selector!("contact_polygon_class");
const SOLVER_SLOT: felt252 = selector!("solver_class");
/// Call counters of [`CountingHashes`].
const BALL_CALLS: felt252 = selector!("contact_ball_calls");
const POLYGON_CALLS: felt252 = selector!("contact_polygon_calls");
const SOLVER_CALLS: felt252 = selector!("solver_calls");

fn read(slot: felt252) -> felt252 {
    storage_read_syscall(0, slot.try_into().unwrap()).unwrap_syscall()
}

fn write(slot: felt252, value: felt252) {
    storage_write_syscall(0, slot.try_into().unwrap(), value).unwrap_syscall();
}

fn class_at(slot: felt252) -> ClassHash {
    read(slot).try_into().unwrap()
}

/// Measured and rejected: each hash read from a storage slot of the test contract (filled by
/// [`install`]), as CS3's dispatchers read theirs.
impl StoredHashes of ClassHashes {
    fn contact_ball() -> ClassHash {
        class_at(BALL_SLOT)
    }

    fn contact_polygon() -> ClassHash {
        class_at(POLYGON_SLOT)
    }

    fn solver() -> ClassHash {
        class_at(SOLVER_SLOT)
    }
}

fn count(slot: felt252) {
    write(slot, read(slot) + 1);
}

/// [`PinnedHashes`], counting the calls of each class in storage.
impl CountingHashes of ClassHashes {
    fn contact_ball() -> ClassHash {
        count(BALL_CALLS);
        PinnedHashes::contact_ball()
    }

    fn contact_polygon() -> ClassHash {
        count(POLYGON_CALLS);
        PinnedHashes::contact_polygon()
    }

    fn solver() -> ClassHash {
        count(SOLVER_CALLS);
        PinnedHashes::solver()
    }
}

fn declared(name: ByteArray) -> ClassHash {
    *declare(name).unwrap_syscall().contract_class().class_hash
}

/// Declares the three classes; with `store`, writes their hashes where [`StoredHashes`] reads
/// them.
fn install(store: bool) {
    let classes = [
        ("ContactBallClass", BALL_SLOT), ("ContactPolygonClass", POLYGON_SLOT),
        ("SolverClass", SOLVER_SLOT),
    ];
    for (name, slot) in classes.span() {
        let class_hash = declared(name.clone());
        if store {
            write(*slot, class_hash.into());
        }
    }
}

#[test]
fn test_pinned_class_hashes() {
    let classes = [
        ("ContactBallClass", CONTACT_BALL_HASH), ("ContactPolygonClass", CONTACT_POLYGON_HASH),
        ("SolverClass", SOLVER_HASH),
    ];
    let mut stale = false;
    for (name, pinned) in classes.span() {
        let pinned = *pinned;
        let class_hash: felt252 = declared(name.clone()).into();
        if class_hash != pinned {
            println!("{name}: pinned {pinned:x}, declared {class_hash:x}: update the constant");
            stale = true;
        }
    }
    assert(!stale, 'class hashes: stale pins');
}

/// The tick-by-tick digests (`tick_digest`: whole world, events, entities) of the shot with `C`,
/// the settled world first.
fn trace<impl C: StepConfig>(ticks: u32) -> Array<felt252> {
    let mut pile = build::<C>();
    let mut out = array![tick_digest(ref pile, array![].span())];
    launch(ref pile, PULL);
    let mut i = 0;
    while i != ticks {
        let events = tick::<C>(ref pile);
        out.append(tick_digest(ref pile, events.span()));
        i += 1;
    }
    out
}

#[test]
fn test_split_bit_identical() {
    let expected = trace::<BasicStepConfig>(151);
    install(false);
    let split = trace::<SplitStepConfig<PinnedHashes>>(151);
    assert_eq!(split.len(), expected.len());
    let mut tick = 0;
    for (a, b) in split.span().into_iter().zip(expected.span()) {
        assert!(a == b, "tick {tick} differs");
        tick += 1;
    }
}

#[test]
fn test_split_stored_bit_identical() {
    let (mut pile, events) = run::<BasicStepConfig>(151);
    let expected = digest(ref pile);
    install(true);
    let (mut pile, split_events) = run::<SplitStepConfig<StoredHashes>>(151);
    assert_eq!(split_events, events);
    assert_eq!(digest(ref pile), expected);
}

/// The library calls of the shot (settle step included), per class.
#[test]
fn test_call_counts() {
    install(false);
    let (mut pile, _) = run::<SplitStepConfig<CountingHashes>>(151);
    opaque(digest(ref pile));
    let (ball, polygon, solver) = (read(BALL_CALLS), read(POLYGON_CALLS), read(SOLVER_CALLS));
    println!("calls: contact_ball {ball}, contact_polygon {polygon}, solver {solver}");
    assert(ball != 0 && polygon != 0 && solver != 0, 'calls');
}

/// The contact generation library-called, the solve in process: splits the shot's extra steps
/// between the contact calls and the solver calls.
impl ContactsOnlyConfig of StepConfig {
    impl Dispatcher = FamilyDispatcher<PinnedHashes>;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
}

fn contacts_only(ticks: u32) {
    install(false);
    let (mut pile, _) = run::<ContactsOnlyConfig>(ticks);
    opaque(digest(ref pile));
}

fn basic(ticks: u32) {
    let (mut pile, _) = run::<BasicStepConfig>(ticks);
    opaque(digest(ref pile));
}

fn split(ticks: u32) {
    install(false);
    let (mut pile, _) = run::<SplitStepConfig<PinnedHashes>>(ticks);
    opaque(digest(ref pile));
}

fn split_stored(ticks: u32) {
    install(true);
    let (mut pile, _) = run::<SplitStepConfig<StoredHashes>>(ticks);
    opaque(digest(ref pile));
}

/// The declarations alone (subtracted from the `steps_split*` probes).
#[test]
fn steps_install() {
    install(false);
}

#[test]
fn steps_install_stored() {
    install(true);
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
fn steps_basic_43() {
    basic(43);
}

#[test]
fn steps_basic_44() {
    basic(44);
}

#[test]
fn steps_basic_100() {
    basic(100);
}

#[test]
fn steps_basic_151() {
    basic(151);
}

#[test]
fn steps_split_0() {
    split(0);
}

#[test]
fn steps_split_30() {
    split(30);
}

#[test]
fn steps_split_43() {
    split(43);
}

#[test]
fn steps_split_44() {
    split(44);
}

#[test]
fn steps_split_100() {
    split(100);
}

#[test]
fn steps_split_151() {
    split(151);
}

#[test]
fn steps_split_stored_0() {
    split_stored(0);
}

#[test]
fn steps_split_stored_44() {
    split_stored(44);
}

#[test]
fn steps_split_stored_151() {
    split_stored(151);
}

#[test]
fn steps_contacts_only_151() {
    contacts_only(151);
}
