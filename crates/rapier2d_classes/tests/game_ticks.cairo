//! The game's use of the step on every layout it can use, the slim layout first (CX1, programme
//! conditions): pile10 as slingfall plays it, with what the pile10 shot alone does not exercise.
//!
//! * force events read every step (`step_with_force_events_with_stages`), the collision and
//!   force events of each tick compared;
//! * the world saved and restored at chunk boundaries in the middle of the collapse (the basic
//!   codec, `into_basic_state` / `from_basic_state`, for the slim layout; the full `WorldState`
//!   codec for the others), through felts, the run continuing from the restored world;
//! * bodies removed during the run in ascending handle order: the destroyed blocks (the damage
//!   rule), two blocks at once, the spent pebble;
//! * a body inserted between steps with an initial velocity (a second pebble).
//!
//! Each layout runs with its codec against the in-process step without any round trip, so the
//! round trips are also checked to be transparent: the whole `WorldState`, the events and the
//! entities after every tick (`digest`).

use rapier2d::prelude::{
    BasicStepConfig, CollisionEvent, ContactForceEvent, World, WorldState, WorldTrait,
};
use rapier2d::world::basic_state::{BasicWorldState, from_basic_state, into_basic_state};
use rapier2d_classes::{
    ContactSolveStepConfig, SlimSplitStages, SplitBatchedStages, SplitHybridStages, SplitStages,
};
use crate::hashes::{StoredHashes, install};
use crate::pile10::{InProcess, Layout, PULL, Pile, Staged, apply_damage, build, launch};
use crate::split::assert_same;

/// Ticks of the run: the flight, the impact (tick 43), the collapse, the second pebble.
const TICKS: u32 = 100;
/// The tick before which the spent pebble is removed, and the one before which the second
/// pebble is launched.
const SPENT: u32 = 70;
const SECOND: u32 = 72;
/// The tick before which two blocks are removed at once (ascending).
const CLEARED: u32 = 64;

/// How the world is saved and restored at a chunk boundary.
trait Codec {
    fn trip(world: World) -> World;
}

/// No round trip (the reference).
impl NoTrip of Codec {
    fn trip(world: World) -> World {
        world
    }
}

/// The full codec: `into_state`, its felts, `from_state`.
impl FullCodec of Codec {
    fn trip(world: World) -> World {
        let mut felts = array![];
        world.into_state().serialize(ref felts);
        let mut span = felts.span();
        let state: WorldState = Serde::deserialize(ref span).unwrap();
        assert!(span.is_empty(), "full codec: trailing felts");
        WorldTrait::from_state(state)
    }
}

/// The basic codec of the slim caller: `into_basic_state`, its felts, `from_basic_state`.
impl BasicCodec of Codec {
    fn trip(world: World) -> World {
        let mut felts = array![];
        into_basic_state(world).serialize(ref felts);
        let mut span = felts.span();
        let state: BasicWorldState = Serde::deserialize(ref span).unwrap();
        assert!(span.is_empty(), "basic codec: trailing felts");
        from_basic_state(state)
    }
}

/// A chunk boundary: before the first tick of a chunk (flight, impact, collapse).
fn boundary(t: u32) -> bool {
    t == 30 || t == 45 || t == 52 || t == 60 || t == 68 || t == 80 || t == 90
}

/// Poseidon digest of the whole world, the tick's collision and force events and the entities.
fn digest(
    ref pile: Pile, collisions: Span<CollisionEvent>, forces: Span<ContactForceEvent>,
) -> felt252 {
    let mut felts: Array<felt252> = array![];
    pile.world.to_state().serialize(ref felts);
    collisions.serialize(ref felts);
    forces.serialize(ref felts);
    for entity in pile.entities.span() {
        felts.append((*entity.hp).into());
        felts.append((*entity.alive).into());
    }
    core::poseidon::poseidon_hash_span(felts.span())
}

/// Removes the live blocks of entities `first` and `second` (`first < second`), in that order.
fn clear(ref pile: Pile, first: u32, second: u32) {
    let mut updated = array![];
    let mut index = 0;
    for entity in pile.entities.span() {
        let mut entity = *entity;
        if (index == first || index == second) && entity.alive {
            let _ = pile.world.remove_body(entity.handle);
            entity.alive = false;
        }
        updated.append(entity);
        index += 1;
    }
    pile.entities = updated;
}

/// pile10 played with `L`, the world through `K` at every chunk boundary: the digests after the
/// settle and after every tick. Checks along the way that the run exercises what it claims.
fn game<impl L: Layout, impl K: Codec>() -> Array<felt252> {
    let mut pile = build::<L>();
    let mut out = array![digest(ref pile, array![].span(), array![].span())];
    launch(ref pile, PULL);
    let (mut force_events, mut destroyed, mut trips) = (0, 0, 0);
    let mut t = 0;
    while t != TICKS {
        if boundary(t) {
            let Pile { world, entities, pebble } = pile;
            pile = Pile { world: K::trip(world), entities, pebble };
            trips += 1;
        }
        if t == CLEARED {
            clear(ref pile, 8, 9);
        } else if t == SPENT {
            let _ = pile.world.remove_body(pile.pebble.unwrap());
            pile.pebble = None;
        } else if t == SECOND {
            launch(ref pile, (-900, -150));
        }
        let (collisions, forces) = pile
            .world
            .step_with_force_events_with_stages::<L::Step, L::Stages>();
        force_events += forces.len();
        destroyed += apply_damage(ref pile, forces.span());
        out.append(digest(ref pile, collisions.span(), forces.span()));
        t += 1;
    }
    assert!(trips == 7, "seven round trips");
    assert!(force_events != 0, "force events every step of the collapse");
    assert!(destroyed != 0, "blocks destroyed by the damage rule");
    out
}

/// The in-process game without round trips.
fn expected() -> Array<felt252> {
    game::<InProcess<BasicStepConfig>, NoTrip>()
}

#[test]
fn test_game_slim_bit_identical() {
    let expected = expected();
    install(true);
    assert_same(
        "slim, basic codec",
        game::<Staged<BasicStepConfig, SlimSplitStages<StoredHashes>>, BasicCodec>().span(),
        expected.span(),
    );
}

#[test]
fn test_game_in_process_codecs_bit_identical() {
    let expected = expected();
    assert_same(
        "in process, full codec",
        game::<InProcess<BasicStepConfig>, FullCodec>().span(),
        expected.span(),
    );
    assert_same(
        "in process, basic codec",
        game::<InProcess<BasicStepConfig>, BasicCodec>().span(),
        expected.span(),
    );
}

#[test]
fn test_game_split_bit_identical() {
    let expected = expected();
    install(true);
    assert_same(
        "split, full codec",
        game::<Staged<BasicStepConfig, SplitStages<StoredHashes>>, FullCodec>().span(),
        expected.span(),
    );
}

#[test]
fn test_game_hybrid_bit_identical() {
    let expected = expected();
    install(true);
    assert_same(
        "hybrid, full codec",
        game::<Staged<BasicStepConfig, SplitHybridStages<StoredHashes>>, FullCodec>().span(),
        expected.span(),
    );
}

#[test]
fn test_game_batched_bit_identical() {
    let expected = expected();
    install(true);
    assert_same(
        "batched, full codec",
        game::<Staged<BasicStepConfig, SplitBatchedStages<StoredHashes>>, FullCodec>().span(),
        expected.span(),
    );
}

#[test]
fn test_game_cs4_bit_identical() {
    let expected = expected();
    install(true);
    assert_same(
        "CS4 layout, full codec",
        game::<InProcess<ContactSolveStepConfig<StoredHashes>>, FullCodec>().span(),
        expected.span(),
    );
}
