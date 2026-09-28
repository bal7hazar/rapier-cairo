//! The split steps against the in-process step on pile10 (`pile10`): identical worlds and events
//! after every tick (`tick_digest`: the whole `WorldState`, the force events, the entities), and
//! the library calls each layout makes. The layouts:
//!
//! * `SplitStepConfig` (CS5): every stage out, one contact call per pair;
//! * `SplitBatchedStepConfig`: the same with the contact generation batched per family;
//! * `SplitHybridStepConfig`: the same as `SplitStepConfig` with no solve-and-advance call on a
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
use rapier2d::prelude::{BasicStepConfig, RigidBodyTrait, StepConfig, WorldTrait};
use rapier2d_classes::{
    ContactSolveStepConfig, SplitBatchedStepConfig, SplitHybridStepConfig, SplitStepConfig,
};
use rapier_testing::opaque;
use crate::hashes::{CountingHashes, StoredHashes, calls, install};
use crate::pile10::{PULL, Pile, build, digest, launch, run, tick, tick_digest};

/// `BasicStepConfig` with the batched narrow phase in process (the batch measured against the
/// per-pair loop without a library call).
pub impl BatchedInProcess of StepConfig {
    impl Dispatcher = BasicShapesDispatcher;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
    impl Narrow = BatchedNarrowPhase<InProcessBatch<BasicShapesDispatcher>, NoSensors>;
    impl Broad = InProcessBroadPhase;
    impl Islands = InProcessIslands;
    impl Advance = InProcessSolveAdvance<NoJoints>;
    impl Mass = InProcessMass;
}

/// The tick-by-tick digests (`tick_digest`) of the shot with `C`, the settled world first.
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

fn assert_same(layout: ByteArray, got: Span<felt252>, expected: Span<felt252>) {
    assert_eq!(got.len(), expected.len());
    let mut tick = 0;
    for (a, b) in got.into_iter().zip(expected) {
        assert!(a == b, "{layout}: tick {tick} differs");
        tick += 1;
    }
}

#[test]
fn test_split_bit_identical() {
    let expected = trace::<BasicStepConfig>(151);
    install(true);
    assert_same("split", trace::<SplitStepConfig<StoredHashes>>(151).span(), expected.span());
    assert_same("batched in process", trace::<BatchedInProcess>(151).span(), expected.span());
}

#[test]
fn test_split_variants_bit_identical() {
    let expected = trace::<BasicStepConfig>(151);
    install(true);
    assert_same(
        "batched", trace::<SplitBatchedStepConfig<StoredHashes>>(151).span(), expected.span(),
    );
    assert_same(
        "hybrid", trace::<SplitHybridStepConfig<StoredHashes>>(151).span(), expected.span(),
    );
    assert_same(
        "CS4 layout", trace::<ContactSolveStepConfig<StoredHashes>>(151).span(), expected.span(),
    );
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
fn trace_with_changes<impl C: StepConfig>(ticks: u32) -> Array<felt252> {
    let mut pile = build::<C>();
    let mut out = array![tick_digest(ref pile, array![].span())];
    launch(ref pile, PULL);
    let mut i = 0;
    while i != ticks {
        change(ref pile, i);
        let events = tick::<C>(ref pile);
        out.append(tick_digest(ref pile, events.span()));
        i += 1;
    }
    out
}

/// User changes in the middle of the shot (insertions next to sleeping bodies, a wake-up, a
/// move): the split step still agrees with the in-process one after every tick.
#[test]
fn test_split_user_changes_bit_identical() {
    let expected = trace_with_changes::<BasicStepConfig>(100);
    install(true);
    assert_same(
        "split, user changes",
        trace_with_changes::<SplitStepConfig<StoredHashes>>(100).span(),
        expected.span(),
    );
}

fn print_calls<impl C: StepConfig>(layout: ByteArray) {
    let (mut pile, _) = run::<C>(151);
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
    print_calls::<SplitStepConfig<CountingHashes>>("split");
    let counts = calls();
    for (name, count) in counts.span() {
        let solver: ByteArray = "SolverClass";
        assert!(*count != 0 || *name == solver, "no call of {name}");
    }
}
