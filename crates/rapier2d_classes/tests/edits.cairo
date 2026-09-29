//! CS7: `WorldEditClass` (`rapier2d_classes::edits`) against the same edits in process on pile10:
//! the settle's sleeps, the launch, the destructions of the damage rule and the shot's end (every
//! live body put to sleep, the pebble removed), each through a library call, the steps in process.
//! The `#[ignore]`d `steps_edit_*` probes measure one call of each kind (run with
//! `--tracked-resource cairo-steps`; a `_class` probe minus its `_in_process` twin is the call).

use rapier2d::pipeline::stages::InProcessStages;
use rapier2d::prelude::{
    BasicStepConfig, ColliderBuilderTrait, ContactForceEvent, Handle, RigidBodyBuilderTrait, World,
    WorldTrait,
};
use rapier2d::world::state::WorldState;
use rapier2d_classes::{WorldEdit, apply_edits, edit_world};
use rapier_testing::opaque;
use snforge_std::{DeclareResultTrait, declare};
use starknet::{ClassHash, SyscallResultTrait};
use crate::pile10::{
    InProcess, PULL, Pile, damage, digest, dynamic_alive, launch_edit, settled, tick, tick_digest,
};
use crate::split::{assert_same, trace};

fn edit_class() -> ClassHash {
    *declare("WorldEditClass").unwrap_syscall().contract_class().class_hash
}

/// `edits` applied to the pile's world, in `class` when there is one, in process otherwise; the
/// inserted handles.
fn edit(class: Option<ClassHash>, pile: Pile, edits: Span<WorldEdit>) -> (Pile, Array<Handle>) {
    let Pile { world, entities, pebble } = pile;
    let (world, inserted) = match class {
        Some(class) => {
            let mut felts = array![];
            edits.serialize(ref felts);
            edit_world(class, world, felts.span())
        },
        None => {
            let mut world = world;
            let inserted = apply_edits(ref world, edits);
            (world, inserted)
        },
    };
    (Pile { world, entities, pebble }, inserted)
}

fn sleeps(handles: Span<Handle>) -> Array<WorldEdit> {
    let mut out = array![];
    for handle in handles {
        out.append(WorldEdit::Sleep(*handle));
    }
    out
}

/// pile10 settled, its dynamic bodies put to sleep, the pebble launched: the edits by `class`.
fn launched(class: Option<ClassHash>) -> Pile {
    let mut pile = settled::<InProcess<BasicStepConfig>>();
    let alive = dynamic_alive(ref pile);
    let (pile, _) = edit(class, pile, sleeps(alive.span()).span());
    let (mut pile, inserted) = edit(class, pile, array![launch_edit(PULL)].span());
    pile.pebble = Some(*inserted[0]);
    pile
}

/// One tick: the step in process, the destroyed bodies removed by `class`.
fn edited_tick(class: Option<ClassHash>, pile: Pile) -> (Pile, Array<ContactForceEvent>) {
    let mut pile = pile;
    let (_, events) = pile
        .world
        .step_with_force_events_with_stages::<BasicStepConfig, InProcessStages<BasicStepConfig>>();
    let destroyed = damage(ref pile, events.span());
    if destroyed.is_empty() {
        return (pile, events);
    }
    let mut removals = array![];
    for handle in destroyed {
        removals.append(WorldEdit::Remove(handle));
    }
    let (pile, _) = edit(class, pile, removals.span());
    (pile, events)
}

/// `split::trace`'s digests with the edits by `class`, then the shot's end: every live body put
/// to sleep and the pebble removed (one more digest).
fn edited_trace(class: Option<ClassHash>, ticks: u32) -> Array<felt252> {
    let mut pile = launched(class);
    let mut out = array![];
    let mut i = 0;
    while i != ticks {
        let (next, events) = edited_tick(class, pile);
        pile = next;
        out.append(tick_digest(ref pile, events.span()));
        i += 1;
    }
    let mut end = sleeps(dynamic_alive(ref pile).span());
    end.append(WorldEdit::Remove(pile.pebble.unwrap()));
    let (mut pile, _) = edit(class, pile, end.span());
    out.append(tick_digest(ref pile, array![].span()));
    out
}

/// Every tick of the shot, and its end: the class's edits give the in-process world, and both give
/// the reference shot (`split::trace`, the edits by the game's own calls).
#[test]
fn test_edits_bit_identical() {
    let class = edit_class();
    let got = edited_trace(Some(class), 151);
    let expected = edited_trace(None, 151);
    assert_same("edits", got.span(), expected.span());
    // `split::trace` starts with the settled world and has no end digest.
    let reference = trace::<InProcess<BasicStepConfig>>(151);
    assert_same("edits in process", expected.span().slice(0, 151), reference.span().slice(1, 151));
}

/// The inserted handles come back in edit order, a handle that does not resolve is skipped, and a
/// sleeping body is left as it is.
#[test]
fn test_edits_skip_and_order() {
    let class = edit_class();
    let pile = launched(None);
    let pebble = pile.pebble.unwrap();
    let gone = Handle { index: 40, generation: 0 };
    let edits = array![
        WorldEdit::Remove(gone), WorldEdit::Sleep(gone), launch_edit(PULL),
        WorldEdit::Sleep(pebble), launch_edit((-10, -10)),
    ];
    let (mut a, got) = edit(Some(class), pile, edits.span());
    let (mut b, expected) = edit(None, launched(None), edits.span());
    assert!(got == expected && got.len() == 2);
    assert!(a.world.to_state() == b.world.to_state());
    assert!(a.world.is_sleeping(pebble) == Some(true));
}

/// A world with another shape is rejected by the codec on its way in.
#[test]
#[should_panic(expected: 'State: not a basic shape')]
fn test_edits_reject_other_shapes() {
    let mut world: World = WorldTrait::new(Default::default(), Default::default());
    let body = RigidBodyBuilderTrait::dynamic().build();
    let _ = world.insert(body, ColliderBuilderTrait::capsule_y(fixed::ONE, fixed::ONE).build());
    let _ = edit_world(edit_class(), world, array![0].span());
}

/// The shot's world after `ticks` in-process ticks, the class declared.
fn at_tick(ticks: u32) -> (Pile, ClassHash) {
    let class = edit_class();
    let mut pile = launched(None);
    let mut i = 0;
    while i != opaque(ticks) {
        let _ = tick::<InProcess<BasicStepConfig>>(ref pile);
        i += 1;
    }
    (pile, class)
}

/// The probes: at tick 60 (after the first destructions), one call of each `kind` (1: a launch,
/// 2: a removal, 3: the end's sleeps and the pebble's removal), in the class or in process.
fn probe(kind: u32, in_class: bool) {
    let (mut pile, class) = at_tick(60);
    let target = *dynamic_alive(ref pile)[0];
    let edits = match kind {
        1 => array![launch_edit(PULL)],
        2 => array![WorldEdit::Remove(target)],
        _ => {
            let mut end = sleeps(dynamic_alive(ref pile).span());
            end.append(WorldEdit::Remove(pile.pebble.unwrap()));
            end
        },
    };
    let class = if in_class {
        Some(class)
    } else {
        None
    };
    let (mut pile, _) = edit(class, pile, edits.span());
    opaque(digest(ref pile));
}

#[test]
#[ignore]
fn steps_edit_launch_in_process() {
    probe(1, false);
}

#[test]
#[ignore]
fn steps_edit_launch_class() {
    probe(1, true);
}

#[test]
#[ignore]
fn steps_edit_remove_in_process() {
    probe(2, false);
}

#[test]
#[ignore]
fn steps_edit_remove_class() {
    probe(2, true);
}

#[test]
#[ignore]
fn steps_edit_end_in_process() {
    probe(3, false);
}

#[test]
#[ignore]
fn steps_edit_end_class() {
    probe(3, true);
}

/// The felts of the world at tick 60 (what crosses each way).
#[test]
#[ignore]
fn test_edit_crossing_felts() {
    let (mut pile, _) = at_tick(60);
    let mut felts = array![];
    let state: WorldState = pile.world.to_state();
    state.serialize(ref felts);
    println!("world felts at tick 60: {}", felts.len());
}
