//! Exact Cairo steps of the shot per layout (`--tracked-resource cairo-steps`; snforge counts the
//! steps of the library-called classes in the test's). Every probe is an `#[ignore]`d measurement
//! with `PinnedHashes` (constant class hashes, as a game compiles them):
//!
//! ```text
//! snforge test -p rapier2d_classes steps_ --include-ignored --tracked-resource cairo-steps \
//!     --detailed-resources
//! ```
//!
//! `steps_<layout>_<ticks>`: the settled world, the launch and `ticks` ticks, minus nothing
//! (`steps_install`: the declarations, to subtract). The one-stage layouts (`*Only`) put one stage
//! out and keep the others in process: their difference with `steps_basic_151` over the calls of
//! that stage (`test_call_counts_*`) is the stage's cost per call.

use rapier2d::pipeline::config::{
    BasicShapesDispatcher, InProcessBroadPhase, InProcessIslands, InProcessMass,
    InProcessSolveAdvance, NoComposites, NoJoints, NoSensors, PairLoopNarrowPhase,
};
use rapier2d::pipeline::stages::narrow::BatchedNarrowPhase;
use rapier2d::prelude::{BasicStepConfig, StepConfig};
use rapier2d_classes::{
    ContactSolveStepConfig, FamilyBatch, FamilyDispatcher, LibraryCallBroadPhase,
    LibraryCallIslands, LibraryCallMass, LibraryCallSolveAdvance, SplitBatchedStepConfig,
    SplitHybridStepConfig, SplitStepConfig,
};
use rapier_testing::opaque;
use crate::hashes::{CountingHashes, PinnedHashes, StoredHashes, calls, install};
use crate::pile10::{digest, run};
use crate::split::BatchedInProcess;

/// One stage out: the contact generation, one call per pair.
impl PerPairOnly of StepConfig {
    impl Dispatcher = FamilyDispatcher<PinnedHashes>;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
    impl Narrow = PairLoopNarrowPhase<FamilyDispatcher<PinnedHashes>, NoSensors, NoComposites>;
    impl Broad = InProcessBroadPhase;
    impl Islands = InProcessIslands;
    impl Advance = InProcessSolveAdvance<NoJoints>;
    impl Mass = InProcessMass;
}

/// One stage out: the contact generation batched per family.
impl BatchOnly of StepConfig {
    impl Dispatcher = BasicShapesDispatcher;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
    impl Narrow = BatchedNarrowPhase<FamilyBatch<PinnedHashes>, NoSensors>;
    impl Broad = InProcessBroadPhase;
    impl Islands = InProcessIslands;
    impl Advance = InProcessSolveAdvance<NoJoints>;
    impl Mass = InProcessMass;
}

/// One stage out: the broad phase.
impl BroadOnly of StepConfig {
    impl Dispatcher = BasicShapesDispatcher;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
    impl Narrow = PairLoopNarrowPhase<BasicShapesDispatcher, NoSensors, NoComposites>;
    impl Broad = LibraryCallBroadPhase<PinnedHashes>;
    impl Islands = InProcessIslands;
    impl Advance = InProcessSolveAdvance<NoJoints>;
    impl Mass = InProcessMass;
}

/// One stage out: the island stage.
impl IslandsOnly of StepConfig {
    impl Dispatcher = BasicShapesDispatcher;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
    impl Narrow = PairLoopNarrowPhase<BasicShapesDispatcher, NoSensors, NoComposites>;
    impl Broad = InProcessBroadPhase;
    impl Islands = LibraryCallIslands<PinnedHashes>;
    impl Advance = InProcessSolveAdvance<NoJoints>;
    impl Mass = InProcessMass;
}

/// One stage out: the fused solve and position update.
impl AdvanceOnly of StepConfig {
    impl Dispatcher = BasicShapesDispatcher;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
    impl Narrow = PairLoopNarrowPhase<BasicShapesDispatcher, NoSensors, NoComposites>;
    impl Broad = InProcessBroadPhase;
    impl Islands = InProcessIslands;
    impl Advance = LibraryCallSolveAdvance<PinnedHashes>;
    impl Mass = InProcessMass;
}

/// The advance crossing in process (measurement: `ValuesSolveAdvance`).
impl AdvanceValues of StepConfig {
    impl Dispatcher = BasicShapesDispatcher;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
    impl Narrow = PairLoopNarrowPhase<BasicShapesDispatcher, NoSensors, NoComposites>;
    impl Broad = InProcessBroadPhase;
    impl Islands = InProcessIslands;
    impl Advance = rapier2d_classes::advance::ValuesSolveAdvance;
    impl Mass = InProcessMass;
}

/// One stage out: the mass properties.
impl MassOnly of StepConfig {
    impl Dispatcher = BasicShapesDispatcher;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
    impl Narrow = PairLoopNarrowPhase<BasicShapesDispatcher, NoSensors, NoComposites>;
    impl Broad = InProcessBroadPhase;
    impl Islands = InProcessIslands;
    impl Advance = InProcessSolveAdvance<NoJoints>;
    impl Mass = LibraryCallMass<PinnedHashes>;
}

fn shot<impl C: StepConfig>(ticks: u32) {
    install(false);
    let (mut pile, _) = run::<C>(ticks);
    opaque(digest(ref pile));
}

/// [`shot`] with the hashes read from storage (CS3's dispatchers, the loser; and the probes that
/// stay valid when the classes' code changes).
fn shot_stored<impl C: StepConfig>(ticks: u32) {
    install(true);
    let (mut pile, _) = run::<C>(ticks);
    opaque(digest(ref pile));
}

/// The declarations alone (subtracted from the probes of the split layouts).
#[test]
fn steps_install() {
    install(false);
}

#[test]
fn steps_install_stored() {
    install(true);
}

fn print_calls<impl C: StepConfig>(layout: ByteArray) {
    install(true);
    let (mut pile, _) = run::<C>(151);
    opaque(digest(ref pile));
    let mut line: ByteArray = format!("calls, {layout}:");
    for (name, count) in calls() {
        line += format!(" {name} {count}");
    }
    println!("{line}");
}

#[test]
#[ignore]
fn test_call_counts_batched() {
    print_calls::<SplitBatchedStepConfig<CountingHashes>>("batched");
}

#[test]
#[ignore]
fn test_call_counts_hybrid() {
    print_calls::<SplitHybridStepConfig<CountingHashes>>("hybrid");
}

#[test]
#[ignore]
fn test_call_counts_cs4() {
    print_calls::<ContactSolveStepConfig<CountingHashes>>("CS4 layout");
}

fn run_only<impl C: StepConfig>(ticks: u32) {
    let (mut pile, _) = run::<C>(ticks);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_0() {
    let (mut pile, _) = run::<BasicStepConfig>(0);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_30() {
    let (mut pile, _) = run::<BasicStepConfig>(30);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_43() {
    let (mut pile, _) = run::<BasicStepConfig>(43);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_44() {
    let (mut pile, _) = run::<BasicStepConfig>(44);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_100() {
    let (mut pile, _) = run::<BasicStepConfig>(100);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_151() {
    let (mut pile, _) = run::<BasicStepConfig>(151);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_10() {
    let (mut pile, _) = run::<BasicStepConfig>(10);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_20() {
    let (mut pile, _) = run::<BasicStepConfig>(20);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_40() {
    let (mut pile, _) = run::<BasicStepConfig>(40);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_50() {
    let (mut pile, _) = run::<BasicStepConfig>(50);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_60() {
    let (mut pile, _) = run::<BasicStepConfig>(60);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_70() {
    let (mut pile, _) = run::<BasicStepConfig>(70);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_80() {
    let (mut pile, _) = run::<BasicStepConfig>(80);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_90() {
    let (mut pile, _) = run::<BasicStepConfig>(90);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_110() {
    let (mut pile, _) = run::<BasicStepConfig>(110);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_120() {
    let (mut pile, _) = run::<BasicStepConfig>(120);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_130() {
    let (mut pile, _) = run::<BasicStepConfig>(130);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_140() {
    let (mut pile, _) = run::<BasicStepConfig>(140);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_150() {
    let (mut pile, _) = run::<BasicStepConfig>(150);
    opaque(digest(ref pile));
}

#[test]
fn steps_batched_in_process_0() {
    run_only::<BatchedInProcess>(0);
}

#[test]
fn steps_batched_in_process_151() {
    run_only::<BatchedInProcess>(151);
}

#[test]
#[ignore]
fn steps_split_0() {
    shot::<SplitStepConfig<PinnedHashes>>(0);
}

#[test]
#[ignore]
fn steps_split_30() {
    shot::<SplitStepConfig<PinnedHashes>>(30);
}

#[test]
#[ignore]
fn steps_split_43() {
    shot::<SplitStepConfig<PinnedHashes>>(43);
}

#[test]
#[ignore]
fn steps_split_44() {
    shot::<SplitStepConfig<PinnedHashes>>(44);
}

#[test]
#[ignore]
fn steps_split_100() {
    shot::<SplitStepConfig<PinnedHashes>>(100);
}

#[test]
#[ignore]
fn steps_split_151() {
    shot::<SplitStepConfig<PinnedHashes>>(151);
}

#[test]
#[ignore]
fn steps_split_10() {
    shot::<SplitStepConfig<PinnedHashes>>(10);
}

#[test]
#[ignore]
fn steps_split_20() {
    shot::<SplitStepConfig<PinnedHashes>>(20);
}

#[test]
#[ignore]
fn steps_split_40() {
    shot::<SplitStepConfig<PinnedHashes>>(40);
}

#[test]
#[ignore]
fn steps_split_50() {
    shot::<SplitStepConfig<PinnedHashes>>(50);
}

#[test]
#[ignore]
fn steps_split_60() {
    shot::<SplitStepConfig<PinnedHashes>>(60);
}

#[test]
#[ignore]
fn steps_split_70() {
    shot::<SplitStepConfig<PinnedHashes>>(70);
}

#[test]
#[ignore]
fn steps_split_80() {
    shot::<SplitStepConfig<PinnedHashes>>(80);
}

#[test]
#[ignore]
fn steps_split_90() {
    shot::<SplitStepConfig<PinnedHashes>>(90);
}

#[test]
#[ignore]
fn steps_split_110() {
    shot::<SplitStepConfig<PinnedHashes>>(110);
}

#[test]
#[ignore]
fn steps_split_120() {
    shot::<SplitStepConfig<PinnedHashes>>(120);
}

#[test]
#[ignore]
fn steps_split_130() {
    shot::<SplitStepConfig<PinnedHashes>>(130);
}

#[test]
#[ignore]
fn steps_split_140() {
    shot::<SplitStepConfig<PinnedHashes>>(140);
}

#[test]
#[ignore]
fn steps_split_150() {
    shot::<SplitStepConfig<PinnedHashes>>(150);
}

#[test]
#[ignore]
fn steps_batched_0() {
    shot::<SplitBatchedStepConfig<PinnedHashes>>(0);
}

#[test]
#[ignore]
fn steps_batched_30() {
    shot::<SplitBatchedStepConfig<PinnedHashes>>(30);
}

#[test]
#[ignore]
fn steps_batched_44() {
    shot::<SplitBatchedStepConfig<PinnedHashes>>(44);
}

#[test]
#[ignore]
fn steps_batched_151() {
    shot::<SplitBatchedStepConfig<PinnedHashes>>(151);
}

#[test]
#[ignore]
fn steps_hybrid_0() {
    shot::<SplitHybridStepConfig<PinnedHashes>>(0);
}

#[test]
#[ignore]
fn steps_hybrid_30() {
    shot::<SplitHybridStepConfig<PinnedHashes>>(30);
}

#[test]
#[ignore]
fn steps_hybrid_44() {
    shot::<SplitHybridStepConfig<PinnedHashes>>(44);
}

#[test]
#[ignore]
fn steps_hybrid_151() {
    shot::<SplitHybridStepConfig<PinnedHashes>>(151);
}

#[test]
#[ignore]
fn steps_cs4_0() {
    shot::<ContactSolveStepConfig<PinnedHashes>>(0);
}

#[test]
#[ignore]
fn steps_cs4_30() {
    shot::<ContactSolveStepConfig<PinnedHashes>>(30);
}

#[test]
#[ignore]
fn steps_cs4_44() {
    shot::<ContactSolveStepConfig<PinnedHashes>>(44);
}

#[test]
#[ignore]
fn steps_cs4_151() {
    shot::<ContactSolveStepConfig<PinnedHashes>>(151);
}

#[test]
#[ignore]
fn steps_per_pair_only_151() {
    shot::<PerPairOnly>(151);
}

#[test]
#[ignore]
fn steps_batch_only_151() {
    shot::<BatchOnly>(151);
}

#[test]
#[ignore]
fn steps_broad_only_151() {
    shot::<BroadOnly>(151);
}

#[test]
#[ignore]
fn steps_islands_only_151() {
    shot::<IslandsOnly>(151);
}

#[test]
#[ignore]
fn steps_advance_only_151() {
    shot::<AdvanceOnly>(151);
}

#[test]
#[ignore]
fn steps_mass_only_151() {
    shot::<MassOnly>(151);
}

#[test]
#[ignore]
fn steps_mass_only_0() {
    shot::<MassOnly>(0);
}

#[test]
#[ignore]
fn steps_advance_values_151() {
    shot::<AdvanceValues>(151);
}

#[test]
#[ignore]
fn steps_advance_only_30() {
    shot::<AdvanceOnly>(30);
}

#[test]
#[ignore]
fn steps_advance_values_30() {
    shot::<AdvanceValues>(30);
}

#[test]
fn steps_split_stored_0() {
    shot_stored::<SplitStepConfig<StoredHashes>>(0);
}

#[test]
fn steps_split_stored_44() {
    shot_stored::<SplitStepConfig<StoredHashes>>(44);
}

#[test]
fn steps_split_stored_151() {
    shot_stored::<SplitStepConfig<StoredHashes>>(151);
}
