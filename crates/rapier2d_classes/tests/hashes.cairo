//! The `ClassHashes` impls of the tests: [`PinnedHashes`], the constants a game compiles (the
//! hashes snforge gives the classes of this package, checked by `test_pinned_class_hashes`),
//! [`StoredHashes`], each hash read from a storage slot of the calling contract (CS3's
//! dispatchers, the loser), and [`CountingHashes`], which also counts the calls of each class.
//!
//! ```text
//! snforge test -p rapier2d_classes test_pinned_class_hashes --include-ignored
//!     # prints the declared hashes; copy them into the `*_HASH` constants below
//! ```

use rapier2d_classes::ClassHashes;
use snforge_std::{DeclareResultTrait, declare};
use starknet::syscalls::{storage_read_syscall, storage_write_syscall};
use starknet::{ClassHash, SyscallResultTrait};

/// The class hashes snforge declares for this package's classes (`test_pinned_class_hashes`
/// prints the new ones when the classes change). Pinned from CI (Scarb 2.20.1, TC1): root path
/// `/home/runner/work/rapier-cairo/rapier-cairo`, run 37017663782, artifact `class-hashes`.
pub const CONTACT_BALL_HASH: felt252 =
    0x5f9b275f6554d67248a2d06302bc48eb7fc08854ab213a36cd558dc9ef7d08b;
pub const CONTACT_POLYGON_HASH: felt252 =
    0xfb2be1deb70c09f61be9f7869281ae68ae8a89d3d67a05bdb47ef4ed2bd1e5;
/// WS3: pinned from CI (Scarb 2.20.1): root path `/home/runner/work/rapier-cairo/rapier-cairo`, run
/// 37136259120, artifact `class-hashes`.
pub const SOLVER_HASH: felt252 = 0x4b4d4c626dcc096cf17b481225d6f02221b2bd8eff0b11f28e2a63d2d1c2202;
/// WS3: pinned from CI (Scarb 2.20.1): root path `/home/runner/work/rapier-cairo/rapier-cairo`, run
/// 37136259120, artifact `class-hashes`.
pub const SOLVE_ADVANCE_HASH: felt252 =
    0x10ef4a9329f35950be92676726891af707c79be4fbf440924185a318b8ddbdf;
/// WS3: pinned from CI (Scarb 2.20.1): root path `/home/runner/work/rapier-cairo/rapier-cairo`, run
/// 37136259120, artifact `class-hashes`.
pub const ISLANDS_HASH: felt252 = 0x361cc6d2517b1a7cebc32c9a4407a351d67a687f46c1c4b1b757f36aff0cb85;
pub const BROAD_PHASE_HASH: felt252 =
    0x53423c89826872bf0609676dde383e1e7fd4716d8c76c4964785c1e73a51014;
/// WS3: pinned from CI (Scarb 2.20.1): root path `/home/runner/work/rapier-cairo/rapier-cairo`, run
/// 37136259120, artifact `class-hashes`.
pub const MASS_HASH: felt252 = 0x284415fb7c88ec80e7b588b4d75bd5ae569260b7bbaca77b0668c0c62a8d18c;
/// WS3: pinned from CI (Scarb 2.20.1): root path `/home/runner/work/rapier-cairo/rapier-cairo`, run
/// 37136259120, artifact `class-hashes`.
pub const NARROW_PHASE_HASH: felt252 =
    0x661eba445acbe307090aecc37e13b5dcd531d5144442afa94bf168f5961b51d;
/// CX3: pinned from CI (Scarb 2.20.1): root path `/home/runner/work/rapier-cairo/rapier-cairo`, run
/// 37033470531, artifact `class-hashes`.
pub const ACTIVE_SET_HASH: felt252 =
    0x5ea0e678c30220f403888fa50bf0b304040ee2628f14edfa4158cf082e90e9b;
/// WS3: pinned from CI (Scarb 2.20.1): root path `/home/runner/work/rapier-cairo/rapier-cairo`, run
/// 37136259120, artifact `class-hashes`.
pub const FORCE_EVENTS_HASH: felt252 =
    0x96d356a58081f0fde110f5caa1dd91c99bd8e1caef3462c927427242fdbdc;

/// The declared classes: name, pinned hash, storage slot of [`StoredHashes`], call counter of
/// [`CountingHashes`].
fn classes() -> Array<(ByteArray, felt252, felt252, felt252)> {
    array![
        ("ContactBallClass", CONTACT_BALL_HASH, BALL_SLOT, BALL_CALLS),
        ("ContactPolygonClass", CONTACT_POLYGON_HASH, POLYGON_SLOT, POLYGON_CALLS),
        ("SolverClass", SOLVER_HASH, SOLVER_SLOT, SOLVER_CALLS),
        ("SolveAdvanceClass", SOLVE_ADVANCE_HASH, SOLVE_ADVANCE_SLOT, SOLVE_ADVANCE_CALLS),
        ("IslandsClass", ISLANDS_HASH, ISLANDS_SLOT, ISLANDS_CALLS),
        ("BroadPhaseClass", BROAD_PHASE_HASH, BROAD_PHASE_SLOT, BROAD_PHASE_CALLS),
        ("MassClass", MASS_HASH, MASS_SLOT, MASS_CALLS),
        ("NarrowPhaseClass", NARROW_PHASE_HASH, NARROW_PHASE_SLOT, NARROW_PHASE_CALLS),
        ("ActiveSetClass", ACTIVE_SET_HASH, ACTIVE_SET_SLOT, ACTIVE_SET_CALLS),
        ("ForceEventsClass", FORCE_EVENTS_HASH, FORCE_EVENTS_SLOT, FORCE_EVENTS_CALLS),
    ]
}

/// The classes of this package at their pinned hashes, as constants.
pub impl PinnedHashes of ClassHashes {
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

    fn solve_advance() -> ClassHash {
        const H: ClassHash = SOLVE_ADVANCE_HASH.try_into().unwrap();
        H
    }

    fn islands() -> ClassHash {
        const H: ClassHash = ISLANDS_HASH.try_into().unwrap();
        H
    }

    fn broad_phase() -> ClassHash {
        const H: ClassHash = BROAD_PHASE_HASH.try_into().unwrap();
        H
    }

    fn mass() -> ClassHash {
        const H: ClassHash = MASS_HASH.try_into().unwrap();
        H
    }

    fn narrow_phase() -> ClassHash {
        const H: ClassHash = NARROW_PHASE_HASH.try_into().unwrap();
        H
    }

    fn active_set() -> ClassHash {
        const H: ClassHash = ACTIVE_SET_HASH.try_into().unwrap();
        H
    }

    fn force_events() -> ClassHash {
        const H: ClassHash = FORCE_EVENTS_HASH.try_into().unwrap();
        H
    }
}

const BALL_SLOT: felt252 = selector!("contact_ball_class");
const POLYGON_SLOT: felt252 = selector!("contact_polygon_class");
const SOLVER_SLOT: felt252 = selector!("solver_class");
const SOLVE_ADVANCE_SLOT: felt252 = selector!("solve_advance_class");
const ISLANDS_SLOT: felt252 = selector!("islands_class");
const BROAD_PHASE_SLOT: felt252 = selector!("broad_phase_class");
const MASS_SLOT: felt252 = selector!("mass_class");
const NARROW_PHASE_SLOT: felt252 = selector!("narrow_phase_class");
const ACTIVE_SET_SLOT: felt252 = selector!("active_set_class");
const FORCE_EVENTS_SLOT: felt252 = selector!("force_events_class");
/// Call counters of [`CountingHashes`].
pub const BALL_CALLS: felt252 = selector!("contact_ball_calls");
pub const POLYGON_CALLS: felt252 = selector!("contact_polygon_calls");
pub const SOLVER_CALLS: felt252 = selector!("solver_calls");
pub const SOLVE_ADVANCE_CALLS: felt252 = selector!("solve_advance_calls");
pub const ISLANDS_CALLS: felt252 = selector!("islands_calls");
pub const BROAD_PHASE_CALLS: felt252 = selector!("broad_phase_calls");
pub const MASS_CALLS: felt252 = selector!("mass_calls");
pub const NARROW_PHASE_CALLS: felt252 = selector!("narrow_phase_calls");
pub const ACTIVE_SET_CALLS: felt252 = selector!("active_set_calls");
pub const FORCE_EVENTS_CALLS: felt252 = selector!("force_events_calls");

pub fn read(slot: felt252) -> felt252 {
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
pub impl StoredHashes of ClassHashes {
    fn contact_ball() -> ClassHash {
        class_at(BALL_SLOT)
    }

    fn contact_polygon() -> ClassHash {
        class_at(POLYGON_SLOT)
    }

    fn solver() -> ClassHash {
        class_at(SOLVER_SLOT)
    }

    fn solve_advance() -> ClassHash {
        class_at(SOLVE_ADVANCE_SLOT)
    }

    fn islands() -> ClassHash {
        class_at(ISLANDS_SLOT)
    }

    fn broad_phase() -> ClassHash {
        class_at(BROAD_PHASE_SLOT)
    }

    fn mass() -> ClassHash {
        class_at(MASS_SLOT)
    }

    fn narrow_phase() -> ClassHash {
        class_at(NARROW_PHASE_SLOT)
    }

    fn active_set() -> ClassHash {
        class_at(ACTIVE_SET_SLOT)
    }

    fn force_events() -> ClassHash {
        class_at(FORCE_EVENTS_SLOT)
    }
}

fn count(slot: felt252) {
    write(slot, read(slot) + 1);
}

/// [`StoredHashes`], counting the calls of each class in storage.
pub impl CountingHashes of ClassHashes {
    fn contact_ball() -> ClassHash {
        count(BALL_CALLS);
        StoredHashes::contact_ball()
    }

    fn contact_polygon() -> ClassHash {
        count(POLYGON_CALLS);
        StoredHashes::contact_polygon()
    }

    fn solver() -> ClassHash {
        count(SOLVER_CALLS);
        StoredHashes::solver()
    }

    fn solve_advance() -> ClassHash {
        count(SOLVE_ADVANCE_CALLS);
        StoredHashes::solve_advance()
    }

    fn islands() -> ClassHash {
        count(ISLANDS_CALLS);
        StoredHashes::islands()
    }

    fn broad_phase() -> ClassHash {
        count(BROAD_PHASE_CALLS);
        StoredHashes::broad_phase()
    }

    fn mass() -> ClassHash {
        count(MASS_CALLS);
        StoredHashes::mass()
    }

    fn narrow_phase() -> ClassHash {
        count(NARROW_PHASE_CALLS);
        StoredHashes::narrow_phase()
    }

    fn active_set() -> ClassHash {
        count(ACTIVE_SET_CALLS);
        StoredHashes::active_set()
    }

    fn force_events() -> ClassHash {
        count(FORCE_EVENTS_CALLS);
        StoredHashes::force_events()
    }
}

fn declared(name: ByteArray) -> ClassHash {
    *declare(name).unwrap_syscall().contract_class().class_hash
}

/// Declares every class; with `store`, writes their hashes where [`StoredHashes`] reads them.
pub fn install(store: bool) {
    for (name, _, slot, _) in classes() {
        let class_hash = declared(name);
        if store {
            write(slot, class_hash.into());
        }
    }
}

/// The calls counted by [`CountingHashes`], per class, in the order of `classes`.
pub fn calls() -> Array<(ByteArray, felt252)> {
    let mut out = array![];
    for (name, _, _, counter) in classes() {
        out.append((name, read(counter)));
    }
    out
}

#[test]
#[ignore]
fn test_pinned_class_hashes() {
    let mut stale = false;
    for (name, hash, _, _) in classes() {
        let class_hash: felt252 = declared(name.clone()).into();
        if class_hash != hash {
            println!("{name}: pinned {hash:x}, declared {class_hash:x}: update the constant");
            stale = true;
        }
    }
    assert(!stale, 'class hashes: stale pins');
}
