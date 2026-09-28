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
use rapier2d::pipeline::stages::{
    InProcessActiveSet, InProcessForceEvents, InProcessFreePath, InProcessShapes, StageConfig,
};
use rapier2d::prelude::BasicStepConfig;
use rapier2d_classes::{
    ContactSolveStepConfig, FamilyBatch, FamilyDispatcher, LibraryCallBroadPhase,
    LibraryCallIslands, LibraryCallMass, LibraryCallSolveAdvance, SplitBatchedStages,
    SplitHybridStages, SplitStages,
};
use rapier_testing::opaque;
use crate::hashes::{CountingHashes, PinnedHashes, StoredHashes, calls, install};
use crate::pile10::{InProcess, Layout, Staged, digest, run};
use crate::split::BatchedInProcess;

/// One stage out: the contact generation, one call per pair.
impl PerPairOnly of StageConfig {
    impl Narrow = PairLoopNarrowPhase<FamilyDispatcher<PinnedHashes>, NoSensors, NoComposites>;
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

/// One stage out: the contact generation batched per family.
impl BatchOnly of StageConfig {
    impl Narrow = BatchedNarrowPhase<FamilyBatch<PinnedHashes>, NoSensors>;
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

/// One stage out: the broad phase.
impl BroadOnly of StageConfig {
    impl Narrow = PairLoopNarrowPhase<BasicShapesDispatcher, NoSensors, NoComposites>;
    impl Broad = LibraryCallBroadPhase<PinnedHashes>;
    impl Islands = InProcessIslands;
    impl Advance = InProcessSolveAdvance<NoJoints>;
    impl Mass = InProcessMass;
    impl Shapes = InProcessShapes;
    impl Forces = InProcessForceEvents;
    impl Active = InProcessActiveSet<InProcessShapes>;
    impl Free = InProcessFreePath;
    const KINEMATIC: bool = true;
}

/// One stage out: the island stage.
impl IslandsOnly of StageConfig {
    impl Narrow = PairLoopNarrowPhase<BasicShapesDispatcher, NoSensors, NoComposites>;
    impl Broad = InProcessBroadPhase;
    impl Islands = LibraryCallIslands<PinnedHashes>;
    impl Advance = InProcessSolveAdvance<NoJoints>;
    impl Mass = InProcessMass;
    impl Shapes = InProcessShapes;
    impl Forces = InProcessForceEvents;
    impl Active = InProcessActiveSet<InProcessShapes>;
    impl Free = InProcessFreePath;
    const KINEMATIC: bool = true;
}

/// One stage out: the fused solve and position update.
impl AdvanceOnly of StageConfig {
    impl Narrow = PairLoopNarrowPhase<BasicShapesDispatcher, NoSensors, NoComposites>;
    impl Broad = InProcessBroadPhase;
    impl Islands = InProcessIslands;
    impl Advance = LibraryCallSolveAdvance<PinnedHashes>;
    impl Mass = InProcessMass;
    impl Shapes = InProcessShapes;
    impl Forces = InProcessForceEvents;
    impl Active = InProcessActiveSet<InProcessShapes>;
    impl Free = InProcessFreePath;
    const KINEMATIC: bool = true;
}

/// The advance crossing in process (measurement: `ValuesSolveAdvance`).
pub impl AdvanceValues of StageConfig {
    impl Narrow = PairLoopNarrowPhase<BasicShapesDispatcher, NoSensors, NoComposites>;
    impl Broad = InProcessBroadPhase;
    impl Islands = InProcessIslands;
    impl Advance = rapier2d_classes::advance::ValuesSolveAdvance;
    impl Mass = InProcessMass;
    impl Shapes = InProcessShapes;
    impl Forces = InProcessForceEvents;
    impl Active = InProcessActiveSet<InProcessShapes>;
    impl Free = InProcessFreePath;
    const KINEMATIC: bool = true;
}

/// CS5's advance crossing in process (measurement: `alternatives::Cs5ValuesSolveAdvance`).
pub impl AdvanceCs5Values of StageConfig {
    impl Narrow = PairLoopNarrowPhase<BasicShapesDispatcher, NoSensors, NoComposites>;
    impl Broad = InProcessBroadPhase;
    impl Islands = InProcessIslands;
    impl Advance = rapier2d_classes::advance::alternatives::Cs5ValuesSolveAdvance;
    impl Mass = InProcessMass;
    impl Shapes = InProcessShapes;
    impl Forces = InProcessForceEvents;
    impl Active = InProcessActiveSet<InProcessShapes>;
    impl Free = InProcessFreePath;
    const KINEMATIC: bool = true;
}

/// The islands crossing in process (measurement: `ValuesIslands`).
pub impl IslandsValues of StageConfig {
    impl Narrow = PairLoopNarrowPhase<BasicShapesDispatcher, NoSensors, NoComposites>;
    impl Broad = InProcessBroadPhase;
    impl Islands = rapier2d_classes::islands::ValuesIslands;
    impl Advance = InProcessSolveAdvance<NoJoints>;
    impl Mass = InProcessMass;
    impl Shapes = InProcessShapes;
    impl Forces = InProcessForceEvents;
    impl Active = InProcessActiveSet<InProcessShapes>;
    impl Free = InProcessFreePath;
    const KINEMATIC: bool = true;
}

/// CX1's islands crossing in process with `update_islands_slow` in the class (measurement:
/// `alternatives::SlowValuesIslands`).
pub impl IslandsSlowValues of StageConfig {
    impl Narrow = PairLoopNarrowPhase<BasicShapesDispatcher, NoSensors, NoComposites>;
    impl Broad = InProcessBroadPhase;
    impl Islands = rapier2d_classes::islands::alternatives::SlowValuesIslands;
    impl Advance = InProcessSolveAdvance<NoJoints>;
    impl Mass = InProcessMass;
    impl Shapes = InProcessShapes;
    impl Forces = InProcessForceEvents;
    impl Active = InProcessActiveSet<InProcessShapes>;
    impl Free = InProcessFreePath;
    const KINEMATIC: bool = true;
}

/// CS5's islands crossing in process (measurement: `alternatives::Cs5ValuesIslands`).
pub impl IslandsCs5Values of StageConfig {
    impl Narrow = PairLoopNarrowPhase<BasicShapesDispatcher, NoSensors, NoComposites>;
    impl Broad = InProcessBroadPhase;
    impl Islands = rapier2d_classes::islands::alternatives::Cs5ValuesIslands;
    impl Advance = InProcessSolveAdvance<NoJoints>;
    impl Mass = InProcessMass;
    impl Shapes = InProcessShapes;
    impl Forces = InProcessForceEvents;
    impl Active = InProcessActiveSet<InProcessShapes>;
    impl Free = InProcessFreePath;
    const KINEMATIC: bool = true;
}

/// One stage out: the mass properties.
impl MassOnly of StageConfig {
    impl Narrow = PairLoopNarrowPhase<BasicShapesDispatcher, NoSensors, NoComposites>;
    impl Broad = InProcessBroadPhase;
    impl Islands = InProcessIslands;
    impl Advance = InProcessSolveAdvance<NoJoints>;
    impl Mass = LibraryCallMass<PinnedHashes>;
    impl Shapes = InProcessShapes;
    impl Forces = InProcessForceEvents;
    impl Active = InProcessActiveSet<InProcessShapes>;
    impl Free = InProcessFreePath;
    const KINEMATIC: bool = true;
}

fn shot<impl L: Layout>(ticks: u32) {
    install(false);
    let (mut pile, _) = run::<L>(ticks);
    opaque(digest(ref pile));
}

/// [`shot`] with the hashes read from storage (CS3's dispatchers, the loser; and the probes that
/// stay valid when the classes' code changes).
fn shot_stored<impl L: Layout>(ticks: u32) {
    install(true);
    let (mut pile, _) = run::<L>(ticks);
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

fn print_calls<impl L: Layout>(layout: ByteArray) {
    install(true);
    let (mut pile, _) = run::<L>(151);
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
    print_calls::<Staged<BasicStepConfig, SplitBatchedStages<CountingHashes>>>("batched");
}

#[test]
#[ignore]
fn test_call_counts_hybrid() {
    print_calls::<Staged<BasicStepConfig, SplitHybridStages<CountingHashes>>>("hybrid");
}

#[test]
#[ignore]
fn test_call_counts_cs4() {
    print_calls::<InProcess<ContactSolveStepConfig<CountingHashes>>>("CS4 layout");
}

fn run_only<impl L: Layout>(ticks: u32) {
    let (mut pile, _) = run::<L>(ticks);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_0() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(0);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_30() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(30);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_43() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(43);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_44() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(44);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_100() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(100);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_151() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(151);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_10() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(10);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_20() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(20);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_40() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(40);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_50() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(50);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_60() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(60);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_70() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(70);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_80() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(80);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_90() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(90);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_110() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(110);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_120() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(120);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_130() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(130);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_140() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(140);
    opaque(digest(ref pile));
}

#[test]
fn steps_basic_150() {
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(150);
    opaque(digest(ref pile));
}

#[test]
fn steps_batched_in_process_0() {
    run_only::<Staged<BasicStepConfig, BatchedInProcess>>(0);
}

#[test]
fn steps_batched_in_process_151() {
    run_only::<Staged<BasicStepConfig, BatchedInProcess>>(151);
}

#[test]
#[ignore]
fn steps_split_0() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(0);
}

#[test]
#[ignore]
fn steps_split_30() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(30);
}

#[test]
#[ignore]
fn steps_split_43() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(43);
}

#[test]
#[ignore]
fn steps_split_44() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(44);
}

#[test]
#[ignore]
fn steps_split_100() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(100);
}

#[test]
#[ignore]
fn steps_split_151() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(151);
}

#[test]
#[ignore]
fn steps_split_10() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(10);
}

#[test]
#[ignore]
fn steps_split_20() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(20);
}

#[test]
#[ignore]
fn steps_split_40() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(40);
}

#[test]
#[ignore]
fn steps_split_50() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(50);
}

#[test]
#[ignore]
fn steps_split_60() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(60);
}

#[test]
#[ignore]
fn steps_split_70() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(70);
}

#[test]
#[ignore]
fn steps_split_80() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(80);
}

#[test]
#[ignore]
fn steps_split_90() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(90);
}

#[test]
#[ignore]
fn steps_split_110() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(110);
}

#[test]
#[ignore]
fn steps_split_120() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(120);
}

#[test]
#[ignore]
fn steps_split_130() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(130);
}

#[test]
#[ignore]
fn steps_split_140() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(140);
}

#[test]
#[ignore]
fn steps_split_150() {
    shot::<Staged<BasicStepConfig, SplitStages<PinnedHashes>>>(150);
}

#[test]
#[ignore]
fn steps_batched_0() {
    shot::<Staged<BasicStepConfig, SplitBatchedStages<PinnedHashes>>>(0);
}

#[test]
#[ignore]
fn steps_batched_30() {
    shot::<Staged<BasicStepConfig, SplitBatchedStages<PinnedHashes>>>(30);
}

#[test]
#[ignore]
fn steps_batched_44() {
    shot::<Staged<BasicStepConfig, SplitBatchedStages<PinnedHashes>>>(44);
}

#[test]
#[ignore]
fn steps_batched_151() {
    shot::<Staged<BasicStepConfig, SplitBatchedStages<PinnedHashes>>>(151);
}

#[test]
#[ignore]
fn steps_hybrid_0() {
    shot::<Staged<BasicStepConfig, SplitHybridStages<PinnedHashes>>>(0);
}

#[test]
#[ignore]
fn steps_hybrid_30() {
    shot::<Staged<BasicStepConfig, SplitHybridStages<PinnedHashes>>>(30);
}

#[test]
#[ignore]
fn steps_hybrid_44() {
    shot::<Staged<BasicStepConfig, SplitHybridStages<PinnedHashes>>>(44);
}

#[test]
#[ignore]
fn steps_hybrid_151() {
    shot::<Staged<BasicStepConfig, SplitHybridStages<PinnedHashes>>>(151);
}

#[test]
#[ignore]
fn steps_cs4_0() {
    shot::<InProcess<ContactSolveStepConfig<PinnedHashes>>>(0);
}

#[test]
#[ignore]
fn steps_cs4_30() {
    shot::<InProcess<ContactSolveStepConfig<PinnedHashes>>>(30);
}

#[test]
#[ignore]
fn steps_cs4_44() {
    shot::<InProcess<ContactSolveStepConfig<PinnedHashes>>>(44);
}

#[test]
#[ignore]
fn steps_cs4_10() {
    shot::<InProcess<ContactSolveStepConfig<PinnedHashes>>>(10);
}

#[test]
#[ignore]
fn steps_cs4_20() {
    shot::<InProcess<ContactSolveStepConfig<PinnedHashes>>>(20);
}

#[test]
#[ignore]
fn steps_cs4_40() {
    shot::<InProcess<ContactSolveStepConfig<PinnedHashes>>>(40);
}

#[test]
#[ignore]
fn steps_cs4_50() {
    shot::<InProcess<ContactSolveStepConfig<PinnedHashes>>>(50);
}

#[test]
#[ignore]
fn steps_cs4_60() {
    shot::<InProcess<ContactSolveStepConfig<PinnedHashes>>>(60);
}

#[test]
#[ignore]
fn steps_cs4_70() {
    shot::<InProcess<ContactSolveStepConfig<PinnedHashes>>>(70);
}

#[test]
#[ignore]
fn steps_cs4_80() {
    shot::<InProcess<ContactSolveStepConfig<PinnedHashes>>>(80);
}

#[test]
#[ignore]
fn steps_cs4_90() {
    shot::<InProcess<ContactSolveStepConfig<PinnedHashes>>>(90);
}

#[test]
#[ignore]
fn steps_cs4_100() {
    shot::<InProcess<ContactSolveStepConfig<PinnedHashes>>>(100);
}

#[test]
#[ignore]
fn steps_cs4_110() {
    shot::<InProcess<ContactSolveStepConfig<PinnedHashes>>>(110);
}

#[test]
#[ignore]
fn steps_cs4_120() {
    shot::<InProcess<ContactSolveStepConfig<PinnedHashes>>>(120);
}

#[test]
#[ignore]
fn steps_cs4_130() {
    shot::<InProcess<ContactSolveStepConfig<PinnedHashes>>>(130);
}

#[test]
#[ignore]
fn steps_cs4_140() {
    shot::<InProcess<ContactSolveStepConfig<PinnedHashes>>>(140);
}

#[test]
#[ignore]
fn steps_cs4_150() {
    shot::<InProcess<ContactSolveStepConfig<PinnedHashes>>>(150);
}

#[test]
#[ignore]
fn steps_cs4_151() {
    shot::<InProcess<ContactSolveStepConfig<PinnedHashes>>>(151);
}

#[test]
#[ignore]
fn steps_per_pair_only_151() {
    shot::<Staged<BasicStepConfig, PerPairOnly>>(151);
}

#[test]
#[ignore]
fn steps_batch_only_151() {
    shot::<Staged<BasicStepConfig, BatchOnly>>(151);
}

#[test]
#[ignore]
fn steps_broad_only_151() {
    shot::<Staged<BasicStepConfig, BroadOnly>>(151);
}

#[test]
#[ignore]
fn steps_islands_only_151() {
    shot::<Staged<BasicStepConfig, IslandsOnly>>(151);
}

#[test]
#[ignore]
fn steps_advance_only_151() {
    shot::<Staged<BasicStepConfig, AdvanceOnly>>(151);
}

#[test]
#[ignore]
fn steps_mass_only_151() {
    shot::<Staged<BasicStepConfig, MassOnly>>(151);
}

#[test]
#[ignore]
fn steps_mass_only_0() {
    shot::<Staged<BasicStepConfig, MassOnly>>(0);
}

#[test]
#[ignore]
fn steps_advance_values_151() {
    shot::<Staged<BasicStepConfig, AdvanceValues>>(151);
}

#[test]
#[ignore]
fn steps_advance_cs5_values_151() {
    shot::<Staged<BasicStepConfig, AdvanceCs5Values>>(151);
}

#[test]
#[ignore]
fn steps_islands_values_151() {
    shot::<Staged<BasicStepConfig, IslandsValues>>(151);
}

#[test]
#[ignore]
fn steps_islands_slow_values_151() {
    shot::<Staged<BasicStepConfig, IslandsSlowValues>>(151);
}

#[test]
#[ignore]
fn steps_islands_cs5_values_151() {
    shot::<Staged<BasicStepConfig, IslandsCs5Values>>(151);
}

#[test]
#[ignore]
fn steps_advance_only_30() {
    shot::<Staged<BasicStepConfig, AdvanceOnly>>(30);
}

#[test]
#[ignore]
fn steps_advance_values_30() {
    shot::<Staged<BasicStepConfig, AdvanceValues>>(30);
}

#[test]
fn steps_split_stored_0() {
    shot_stored::<Staged<BasicStepConfig, SplitStages<StoredHashes>>>(0);
}

#[test]
fn steps_split_stored_44() {
    shot_stored::<Staged<BasicStepConfig, SplitStages<StoredHashes>>>(44);
}

#[test]
fn steps_split_stored_151() {
    shot_stored::<Staged<BasicStepConfig, SplitStages<StoredHashes>>>(151);
}
