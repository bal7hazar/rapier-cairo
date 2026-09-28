//! CS6 route (b), measured and not shipped: the world crosses to `OrchestratorClass` and back at
//! every tick (`rapier2d_classes::orchestrator`), the game's damage rule runs in the caller between
//! the ticks. Bit identity with the in-process step on pile10, and the exact Cairo steps of the
//! shot (`#[ignore]`d measurements): with the basic codec (`steps_route_b_*`), and the transfer of
//! the same felts alone, without any codec (`steps_route_b_echo_151`: the floor of any in-memory
//! layout that carries them).

use rapier2d::prelude::{BasicStepConfig, ContactForceEvent, WorldTrait};
use rapier2d::world::basic_state::{from_basic_state, into_basic_state};
use rapier2d_classes::SlimSplitStages;
use rapier2d_classes::orchestrator::orchestrated_step;
use rapier_testing::opaque;
use snforge_std::{DeclareResultTrait, declare};
use starknet::syscalls::library_call_syscall;
use starknet::{ClassHash, SyscallResultTrait};
use crate::hashes::{StoredHashes, install};
use crate::pile10::{
    InProcess, PULL, Pile, Staged, apply_damage, build, digest, launch, run, tick_digest,
};
use crate::split::{assert_same, trace};

fn orchestrator() -> ClassHash {
    *declare("OrchestratorClass").unwrap_syscall().contract_class().class_hash
}

/// One tick of route (b): the step in the orchestrator, the damage rule here.
fn tick_b(class_hash: ClassHash, pile: Pile) -> (Pile, Array<ContactForceEvent>) {
    let Pile { world, entities, pebble } = pile;
    let (state, events) = orchestrated_step(class_hash, into_basic_state(world));
    let mut pile = Pile { world: from_basic_state(state), entities, pebble };
    let _ = apply_damage(ref pile, events.span());
    (pile, events)
}

/// The world of `pile` through the basic codec (with `echo`, also to `class_hash` and back
/// without decoding).
fn trip(class_hash: ClassHash, pile: Pile, echo: bool) -> Pile {
    let Pile { world, entities, pebble } = pile;
    let state = into_basic_state(world);
    let mut felts = array![];
    state.serialize(ref felts);
    if echo {
        let mut calldata = array![];
        felts.span().serialize(ref calldata);
        let ret = library_call_syscall(class_hash, selector!("echo"), calldata.span())
            .unwrap_syscall();
        opaque(ret.len());
    }
    let mut span = opaque(felts).span();
    Pile { world: from_basic_state(Serde::deserialize(ref span).unwrap()), entities, pebble }
}

/// pile10 settled with the orchestrator's stages in the caller, launched, `ticks` ticks of route
/// (b).
fn run_b(ticks: u32, ref digests: Array<felt252>) -> Pile {
    install(true);
    let class_hash = orchestrator();
    let mut pile = build::<Staged<BasicStepConfig, SlimSplitStages<StoredHashes>>>();
    digests.append(tick_digest(ref pile, array![].span()));
    launch(ref pile, PULL);
    let mut i: u32 = 0;
    while i != ticks {
        let (next, events) = tick_b(class_hash, pile);
        pile = next;
        digests.append(tick_digest(ref pile, events.span()));
        i += 1;
    }
    pile
}

fn route_b_bit_identical(ticks: u32) {
    let expected = trace::<InProcess<BasicStepConfig>>(ticks);
    let mut got = array![];
    let _ = run_b(ticks, ref got);
    assert_same("route (b)", got.span(), expected.span());
}

/// The first 50 ticks (flight, impact, the first destructions); the whole shot is `#[ignore]`d
/// (CI's runners, memory).
#[test]
fn test_route_b_bit_identical() {
    route_b_bit_identical(50);
}

#[test]
#[ignore]
fn test_route_b_bit_identical_151() {
    route_b_bit_identical(151);
}

/// The shot without the per-tick digests (the probes).
fn shot_b(ticks: u32) {
    install(true);
    let class_hash = orchestrator();
    let mut pile = build::<Staged<BasicStepConfig, SlimSplitStages<StoredHashes>>>();
    launch(ref pile, PULL);
    let mut i: u32 = 0;
    while i != ticks {
        let (next, _) = tick_b(class_hash, pile);
        pile = next;
        i += 1;
    }
    opaque(digest(ref pile));
}

/// The in-process shot with, at every tick, the world through the basic codec and its felts sent
/// to the orchestrator and back without decoding (`echo`): minus [`steps_route_b_codec_151`], what
/// the transfer costs.
fn shot_trip(echo: bool) {
    install(false);
    let class_hash = orchestrator();
    let (mut pile, _) = run::<InProcess<BasicStepConfig>>(0);
    let mut i: u32 = 0;
    while i != 151 {
        let (_, events) = pile.world.step_with_force_events_with::<BasicStepConfig>();
        let _ = apply_damage(ref pile, events.span());
        pile = trip(class_hash, pile, echo);
        i += 1;
    }
    opaque(digest(ref pile));
}

#[test]
#[ignore]
fn steps_route_b_echo_151() {
    shot_trip(true);
}

/// The codec's round trip at every tick, without the call.
#[test]
#[ignore]
fn steps_route_b_codec_151() {
    shot_trip(false);
}

#[test]
#[ignore]
fn steps_route_b_0() {
    shot_b(0);
}

#[test]
#[ignore]
fn steps_route_b_10() {
    shot_b(10);
}

#[test]
#[ignore]
fn steps_route_b_20() {
    shot_b(20);
}

#[test]
#[ignore]
fn steps_route_b_30() {
    shot_b(30);
}

#[test]
#[ignore]
fn steps_route_b_40() {
    shot_b(40);
}

#[test]
#[ignore]
fn steps_route_b_50() {
    shot_b(50);
}

#[test]
#[ignore]
fn steps_route_b_60() {
    shot_b(60);
}

#[test]
#[ignore]
fn steps_route_b_70() {
    shot_b(70);
}

#[test]
#[ignore]
fn steps_route_b_80() {
    shot_b(80);
}

#[test]
#[ignore]
fn steps_route_b_90() {
    shot_b(90);
}

#[test]
#[ignore]
fn steps_route_b_100() {
    shot_b(100);
}

#[test]
#[ignore]
fn steps_route_b_110() {
    shot_b(110);
}

#[test]
#[ignore]
fn steps_route_b_120() {
    shot_b(120);
}

#[test]
#[ignore]
fn steps_route_b_130() {
    shot_b(130);
}

#[test]
#[ignore]
fn steps_route_b_140() {
    shot_b(140);
}

#[test]
#[ignore]
fn steps_route_b_150() {
    shot_b(150);
}

#[test]
#[ignore]
fn steps_route_b_151() {
    shot_b(151);
}
