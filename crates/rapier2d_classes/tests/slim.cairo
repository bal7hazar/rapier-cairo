//! CS6: the slim caller's stages (`SlimSplitStages`, route (a)) against `BasicStepConfig` on
//! pile10, tick by tick, and the exact Cairo steps of the shot for each lever of route (a)
//! (`#[ignore]`d measurements with `PinnedHashes`, as `steps`):
//!
//! * [`Levers12`]: levers 1 and 2 (no pair-free path, basic shape kernels, no kinematic
//!   preparation), the pair loop in the caller (one contact call per pair, as `SplitStages`);
//! * [`NarrowOut`]: the pair loop in `NarrowPhaseClass` (lever 5);
//! * `SlimSplitStages` (shipped): and the rebuild of the active set in `ActiveSetClass` (lever 4);
//! * [`ForcesOut`]: and the force events in `ForceEventsClass` (lever 3, measured, not shipped).

use rapier2d::pipeline::config::{NoComposites, NoSensors, PairLoopNarrowPhase};
use rapier2d::pipeline::stages::{
    BasicShapeKernels, InProcessActiveSet, InProcessForceEvents, NoFreePath, StageConfig,
};
use rapier2d::prelude::{BasicStepConfig, WorldTrait};
use rapier2d::world::basic_state::into_basic_state;
use rapier2d_classes::{
    ClassHashes, FamilyDispatcher, LibraryCallActiveSet, LibraryCallBroadPhase,
    LibraryCallForceEvents, LibraryCallIslands, LibraryCallMass, LibraryCallNarrowPhase,
    LibraryCallSolveAdvance, SlimSplitStages,
};
use rapier_testing::opaque;
use crate::hashes::{CountingHashes, PinnedHashes, StoredHashes, calls, install};
use crate::pile10::{InProcess, Layout, PULL, Pile, Staged, build, digest, launch, run, tick};
use crate::split::{assert_same, trace, trace_with_changes};

/// Levers 1 and 2 alone.
impl Levers12<impl H: ClassHashes> of StageConfig {
    impl Narrow = PairLoopNarrowPhase<FamilyDispatcher<H>, NoSensors, NoComposites>;
    impl Broad = LibraryCallBroadPhase<H>;
    impl Islands = LibraryCallIslands<H>;
    impl Advance = LibraryCallSolveAdvance<H>;
    impl Mass = LibraryCallMass<H>;
    impl Shapes = BasicShapeKernels;
    impl Forces = InProcessForceEvents;
    impl Active = InProcessActiveSet<BasicShapeKernels>;
    impl Free = NoFreePath;
    const KINEMATIC: bool = false;
}

/// [`Levers12`] with the pair loop out.
impl NarrowOut<impl H: ClassHashes> of StageConfig {
    impl Narrow = LibraryCallNarrowPhase<H>;
    impl Broad = LibraryCallBroadPhase<H>;
    impl Islands = LibraryCallIslands<H>;
    impl Advance = LibraryCallSolveAdvance<H>;
    impl Mass = LibraryCallMass<H>;
    impl Shapes = BasicShapeKernels;
    impl Forces = InProcessForceEvents;
    impl Active = InProcessActiveSet<BasicShapeKernels>;
    impl Free = NoFreePath;
    const KINEMATIC: bool = false;
}

/// `SlimSplitStages` with the force events out.
impl ForcesOut<impl H: ClassHashes> of StageConfig {
    impl Narrow = LibraryCallNarrowPhase<H>;
    impl Broad = LibraryCallBroadPhase<H>;
    impl Islands = LibraryCallIslands<H>;
    impl Advance = LibraryCallSolveAdvance<H>;
    impl Mass = LibraryCallMass<H>;
    impl Shapes = BasicShapeKernels;
    impl Forces = LibraryCallForceEvents<H>;
    impl Active = LibraryCallActiveSet<H>;
    impl Free = NoFreePath;
    const KINEMATIC: bool = false;
}

#[test]
fn test_slim_bit_identical() {
    let expected = trace::<InProcess<BasicStepConfig>>(151);
    install(true);
    assert_same(
        "slim",
        trace::<Staged<BasicStepConfig, SlimSplitStages<StoredHashes>>>(151).span(),
        expected.span(),
    );
}

/// The measured layouts over `ticks` ticks.
fn levers_bit_identical(ticks: u32) {
    let expected = trace::<InProcess<BasicStepConfig>>(ticks);
    install(true);
    assert_same(
        "levers 1-2",
        trace::<Staged<BasicStepConfig, Levers12<StoredHashes>>>(ticks).span(),
        expected.span(),
    );
    assert_same(
        "force events out",
        trace::<Staged<BasicStepConfig, ForcesOut<StoredHashes>>>(ticks).span(),
        expected.span(),
    );
}

/// The first 50 ticks (flight, impact at tick 43, the first destructions): CI's runners run the
/// whole shot of the shipped layout only (memory).
#[test]
fn test_slim_levers_bit_identical() {
    levers_bit_identical(50);
}

#[test]
#[ignore]
fn test_slim_levers_bit_identical_151() {
    levers_bit_identical(151);
}

/// User changes in the middle of the shot (`split::change`).
#[test]
fn test_slim_user_changes_bit_identical() {
    let expected = trace_with_changes::<InProcess<BasicStepConfig>>(100);
    install(true);
    assert_same(
        "slim, user changes",
        trace_with_changes::<Staged<BasicStepConfig, SlimSplitStages<StoredHashes>>>(100).span(),
        expected.span(),
    );
}

/// The library calls of the shot per class (settle step included).
#[test]
#[ignore]
fn test_call_counts_slim() {
    install(true);
    let (mut pile, _) = run::<Staged<BasicStepConfig, SlimSplitStages<CountingHashes>>>(151);
    opaque(digest(ref pile));
    let mut line: ByteArray = "calls, slim:";
    for (name, count) in calls() {
        line += format!(" {name} {count}");
    }
    println!("{line}");
}

fn shot<impl L: Layout>(ticks: u32) {
    install(false);
    let (mut pile, _) = run::<L>(ticks);
    opaque(digest(ref pile));
}

#[test]
#[ignore]
fn steps_levers12_151() {
    shot::<Staged<BasicStepConfig, Levers12<PinnedHashes>>>(151);
}

#[test]
#[ignore]
fn steps_narrow_out_151() {
    shot::<Staged<BasicStepConfig, NarrowOut<PinnedHashes>>>(151);
}

#[test]
#[ignore]
fn steps_forces_out_151() {
    shot::<Staged<BasicStepConfig, ForcesOut<PinnedHashes>>>(151);
}

/// The shipped layout at every tenth tick (the transactions of the shot).
fn slim(ticks: u32) {
    shot::<Staged<BasicStepConfig, SlimSplitStages<PinnedHashes>>>(ticks);
}

#[test]
#[ignore]
fn steps_slim_0() {
    slim(0);
}

#[test]
#[ignore]
fn steps_slim_10() {
    slim(10);
}

#[test]
#[ignore]
fn steps_slim_20() {
    slim(20);
}

#[test]
#[ignore]
fn steps_slim_30() {
    slim(30);
}

#[test]
#[ignore]
fn steps_slim_40() {
    slim(40);
}

#[test]
#[ignore]
fn steps_slim_50() {
    slim(50);
}

#[test]
#[ignore]
fn steps_slim_60() {
    slim(60);
}

#[test]
#[ignore]
fn steps_slim_70() {
    slim(70);
}

#[test]
#[ignore]
fn steps_slim_80() {
    slim(80);
}

#[test]
#[ignore]
fn steps_slim_90() {
    slim(90);
}

#[test]
#[ignore]
fn steps_slim_100() {
    slim(100);
}

#[test]
#[ignore]
fn steps_slim_110() {
    slim(110);
}

#[test]
#[ignore]
fn steps_slim_120() {
    slim(120);
}

#[test]
#[ignore]
fn steps_slim_130() {
    slim(130);
}

#[test]
#[ignore]
fn steps_slim_140() {
    slim(140);
}

#[test]
#[ignore]
fn steps_slim_150() {
    slim(150);
}

#[test]
#[ignore]
fn steps_slim_151() {
    slim(151);
}

/// The felts of the world (`WorldState`'s `Serde`, the basic codec's) after the settle step and
/// after every tenth tick of the shot: what crosses a transaction boundary.
#[test]
#[ignore]
fn test_state_felts() {
    let mut pile = build::<InProcess<BasicStepConfig>>();
    launch(ref pile, PULL);
    let mut line: ByteArray = "state felts:";
    let mut i: u32 = 0;
    while i != 152 {
        if i % 10 == 0 || i == 151 {
            let mut felts = array![];
            pile.world.to_state().serialize(ref felts);
            line += format!(" {i}:{}", felts.len());
        }
        if i != 151 {
            let _ = tick::<InProcess<BasicStepConfig>>(ref pile);
        }
        i += 1;
    }
    println!("{line}");
    let Pile { world, entities: _, pebble: _ } = pile;
    let mut basic = array![];
    into_basic_state(world).serialize(ref basic);
    println!("basic codec at 151: {}", basic.len());
}
