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
/// prints the new ones when the classes change).
pub const CONTACT_BALL_HASH: felt252 =
    0x5bf9108e4308b295865dde2a05ce51cbcdbff5c94e81124fa271020360c1f65;
pub const CONTACT_POLYGON_HASH: felt252 =
    0x53543e3ef58b4a20a9e9c1f1110f64b4ac4f08f2d6de5c9763f623d23ba2604;
pub const SOLVER_HASH: felt252 = 0x447d2dddc0d41fd2ac6d384e96f468d300f0509756eeb49435f004eac8a33ef;
pub const SOLVE_ADVANCE_HASH: felt252 =
    0x484f69de20d79c8ec04effc9557b3ce9a63cc0940d9643f9a954c4a436c4239;
pub const ISLANDS_HASH: felt252 = 0x6124b2b6093c09826da9b57e8008363ad730eae18a95ad5d6f5208342830f54;
pub const BROAD_PHASE_HASH: felt252 =
    0x7d3eed01dc6daa7f8ccdc93ffbed0f86c938302bbc3f8b1fcc24694b448c4d7;
pub const MASS_HASH: felt252 = 0x5415bd22e6c965a3c006b598c748686672ef13cecde4d60cadb674184e451a2;
pub const NARROW_PHASE_HASH: felt252 =
    0x32d058844ef7cc421dfac8ffd7c95380dabfcfc47d467046b293f95cdf8d431;
pub const ACTIVE_SET_HASH: felt252 =
    0x4a6e55fafd8273f75152b3509aa40223f5f50e882ab1978a91beeb895201d02;
pub const FORCE_EVENTS_HASH: felt252 =
    0x322bab3a8c6a846ade7d18d85db51f1ad5cd92c1680dfc79e7049510928fe5a;

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
