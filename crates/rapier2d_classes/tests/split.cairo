//! The split steps against the in-process step on pile10 (`pile10`): identical worlds and events
//! after every tick (`tick_digest`: the whole `WorldState`, the force events, the entities), and
//! the library calls each layout makes. The layouts:
//!
//! * `SplitStages` (CS5): every stage out, one contact call per pair;
//! * `SplitBatchedStages`: the same with the contact generation batched per family;
//! * `SplitHybridStages`: the same as `SplitStages` with no solve-and-advance call on a
//!   step without a touching pair;
//! * `ContactSolveStepConfig`: CS4's layout (contact families per pair, solve in `SolverClass`);
//! * [`BatchedInProcess`]: `BasicStepConfig` with the batched narrow phase in process.
//!
//! The tests that run by default use `StoredHashes` only, so that a change to the engine code the
//! classes compile (which changes their hashes) never breaks them; the exact Cairo steps are the
//! `#[ignore]`d probes of `steps`.

use rapier2d::pipeline::config::{
    BasicShapesDispatcher, InProcessBroadPhase, InProcessIslands, InProcessMass,
    InProcessSolveAdvance, NoComposites, NoJoints, NoSensors,
};
use rapier2d::pipeline::stages::narrow::{BatchedNarrowPhase, InProcessBatch};
use rapier2d::pipeline::stages::{
    InProcessActiveSet, InProcessForceEvents, InProcessFreePath, InProcessShapes, StageConfig,
};
use rapier2d::prelude::{BasicStepConfig, RigidBodyTrait, WorldTrait};
use rapier2d_classes::{ContactSolveStepConfig, SplitBatchedStages, SplitHybridStages, SplitStages};
use rapier_testing::opaque;
use crate::hashes::{CountingHashes, StoredHashes, calls, install};
use crate::pile10::{
    InProcess, Layout, PULL, Pile, Staged, build, digest, launch, run, tick, tick_digest,
};

/// `BasicStepConfig` with the batched narrow phase in process (the batch measured against the
/// per-pair loop without a library call).
pub impl BatchedInProcess of StageConfig {
    impl Narrow = BatchedNarrowPhase<InProcessBatch<BasicShapesDispatcher>, NoSensors>;
    impl Broad = InProcessBroadPhase;
    impl Islands = InProcessIslands;
    impl Advance = InProcessSolveAdvance<NoJoints>;
    impl Mass = InProcessMass;
    impl Shapes = InProcessShapes;
    impl Forces = InProcessForceEvents;
    impl Active = InProcessActiveSet<InProcessShapes>;
    impl Free = InProcessFreePath;
    const KINEMATIC: bool = true;
}

/// The tick-by-tick digests (`tick_digest`) of the shot with `C`, the settled world first.
pub fn trace<impl L: Layout>(ticks: u32) -> Array<felt252> {
    let mut pile = build::<L>();
    let mut out = array![tick_digest(ref pile, array![].span())];
    launch(ref pile, PULL);
    let mut i = 0;
    while i != ticks {
        let events = tick::<L>(ref pile);
        out.append(tick_digest(ref pile, events.span()));
        i += 1;
    }
    out
}

pub fn assert_same(layout: ByteArray, got: Span<felt252>, expected: Span<felt252>) {
    assert_eq!(got.len(), expected.len());
    let mut tick = 0;
    for (a, b) in got.into_iter().zip(expected) {
        assert!(a == b, "{layout}: tick {tick} differs");
        tick += 1;
    }
}

#[test]
fn test_split_bit_identical() {
    let expected = trace::<InProcess<BasicStepConfig>>(151);
    install(true);
    assert_same(
        "split",
        trace::<Staged<BasicStepConfig, SplitStages<StoredHashes>>>(151).span(),
        expected.span(),
    );
    assert_same(
        "batched in process",
        trace::<Staged<BasicStepConfig, BatchedInProcess>>(151).span(),
        expected.span(),
    );
}

/// CS5's measured variants and CS4's layout over `ticks` ticks.
fn variants_bit_identical(ticks: u32) {
    let expected = trace::<InProcess<BasicStepConfig>>(ticks);
    install(true);
    assert_same(
        "batched",
        trace::<Staged<BasicStepConfig, SplitBatchedStages<StoredHashes>>>(ticks).span(),
        expected.span(),
    );
    assert_same(
        "hybrid",
        trace::<Staged<BasicStepConfig, SplitHybridStages<StoredHashes>>>(ticks).span(),
        expected.span(),
    );
    assert_same(
        "CS4 layout",
        trace::<InProcess<ContactSolveStepConfig<StoredHashes>>>(ticks).span(),
        expected.span(),
    );
}

/// The first 50 ticks (flight, impact at tick 43, the first destructions): CI's runners cannot hold
/// every layout's whole shot at once (memory); the whole shot is `#[ignore]`d.
#[test]
fn test_split_variants_bit_identical() {
    variants_bit_identical(50);
}

#[test]
#[ignore]
fn test_split_variants_bit_identical_151() {
    variants_bit_identical(151);
}

/// The user changes of [`trace_with_changes`] at tick `t`: a second pebble launched into the
/// settled pile (colliders inserted next to sleeping bodies), a block woken up by hand, a block
/// moved.
fn change(ref pile: Pile, t: u32) {
    if t == 70 {
        launch(ref pile, (-800, -200));
    } else if t == 75 {
        let block = *pile.entities.at(5).handle;
        if pile.world.body(block).is_some() {
            pile.world.wake_up(block);
        }
    } else if t == 80 {
        let block = *pile.entities.at(8).handle;
        if let Some(mut body) = pile.world.body(block) {
            let mut position = body.position();
            position.translation.x = position.translation.x + fixed::HALF;
            body.set_position(position);
            let _ = pile.world.set_body(block, body);
        }
    }
}

/// [`trace`] with the user changes of [`change`].
pub fn trace_with_changes<impl L: Layout>(ticks: u32) -> Array<felt252> {
    let mut pile = build::<L>();
    let mut out = array![tick_digest(ref pile, array![].span())];
    launch(ref pile, PULL);
    let mut i = 0;
    while i != ticks {
        change(ref pile, i);
        let events = tick::<L>(ref pile);
        out.append(tick_digest(ref pile, events.span()));
        i += 1;
    }
    out
}

/// User changes in the middle of the shot (insertions next to sleeping bodies, a wake-up, a
/// move): the split step still agrees with the in-process one after every tick.
#[test]
fn test_split_user_changes_bit_identical() {
    let expected = trace_with_changes::<InProcess<BasicStepConfig>>(100);
    install(true);
    assert_same(
        "split, user changes",
        trace_with_changes::<Staged<BasicStepConfig, SplitStages<StoredHashes>>>(100).span(),
        expected.span(),
    );
}

fn print_calls<impl L: Layout>(layout: ByteArray) {
    let (mut pile, _) = run::<L>(151);
    opaque(digest(ref pile));
    let mut line: ByteArray = format!("calls, {layout}:");
    for (name, count) in calls() {
        line += format!(" {name} {count}");
    }
    println!("{line}");
}

/// The library calls of the shot (settle step included), per class.
#[test]
fn test_call_counts() {
    install(true);
    print_calls::<Staged<BasicStepConfig, SplitStages<CountingHashes>>>("split");
    let counts = calls();
    // CS4's solve and CS6's classes are not part of `SplitStages`.
    let unused: Array<ByteArray> = array![
        "SolverClass", "NarrowPhaseClass", "ActiveSetClass", "ForceEventsClass",
    ];
    for (name, count) in counts.span() {
        let mut skipped = false;
        for other in unused.span() {
            if other == name {
                skipped = true;
            }
        }
        assert!(*count != 0 || skipped, "no call of {name}");
    }
}
