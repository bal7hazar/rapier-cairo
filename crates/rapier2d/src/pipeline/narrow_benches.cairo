//! Gas probes of the narrow-phase stage (work package ON) on the P3 scenes of
//! `tests/gas_scenes.cairo`, rebuilt here with the same builders.
//!
//! Stage probes: `gas_np_setup_<scene>` builds the scene, runs its warm-up step (P3
//! `WARMUP_CONTACTS`) and the fused stages of the next step up to `find_pairs`;
//! `gas_np_<scene>_<variant>` adds the narrow phase: the difference is the narrow-phase stage
//! of the step P3 measures.
//!
//! Per-pair probes: `gas_pair_prep_<kind>` builds the inputs of one real pair of a warmed-up
//! scene (its two `PairCollider`s, its previous `ContactPair`, the step's pairs); every
//! `gas_pair_<piece>_<kind>` runs one piece of the pair loop on those inputs (routed through
//! `opaque`), so `piece − prep` is that piece. Kinds: `bb` ball–ball (`fixtures::row`), `bh`
//! ball–half-space, `cc` cuboid–cuboid (resting, cached), `ch` cuboid–half-space.
//!
//! Per-kind stage probes: `gas_np_<kind>_<variant>` on a scene with one broad-phase pair of that
//! kind (`bb` a row of two balls, `bh` one ball on the half-space, `cc` a row of two cuboids,
//! `ch` one cuboid on the half-space), net of `gas_np_setup_<kind>`: the whole cost of that pair
//! in the pair loop, before (`outlined`) and after (`shipped`) ON.

use fixed::Fixed;
use rapier_core::collider::CoefficientCombineRuleTrait;
use rapier_core::integration_parameters::IntegrationParametersTrait;
use rapier_dynamics2d::narrow_phase::{
    CarryOver, ContactDispatcher, ContactPair, NarrowPhaseTrait, PairCollider, SortedMerge,
    compute_contacts_from_scratch, pair_filtered, pair_transition, process_pair, solver_contact,
    update_manifold,
};
use rapier_geometry2d::broad_phase::find_pairs;
use rapier_geometry2d::contact::ContactManifold;
use rapier_geometry2d::manifold::ManifoldTrait;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_testing::opaque;
use crate::dispatcher::DefaultDispatcher;
use crate::world::{World, WorldTrait};
use super::fixtures::p3_scene as scene;
use super::narrow_alternatives::{
    PersistentDispatcher, compute_contacts_inlined, compute_contacts_outlined,
    process_pair_outlined, update_manifold_outlined,
};
use super::step_dispatcher::StepDispatcher;
use super::{collision_inputs, user_changes_bodies};

/// Shipped: `compute_contacts_from_scratch::<StepDispatcher>`.
const SHIPPED: u8 = 0;
/// `narrow_alternatives::compute_contacts_outlined::<DefaultDispatcher>` (before ON).
const OUTLINED: u8 = 1;
/// `compute_contacts_from_scratch::<DefaultDispatcher>` (metered arms, no fast path).
const METERED: u8 = 2;
/// `compute_contacts_from_scratch::<PersistentDispatcher>`.
const PERSISTENT: u8 = 3;
/// `narrow_alternatives::compute_contacts_inlined::<DefaultDispatcher>`.
const INLINED: u8 = 4;

/// The scene after its warm-up step, with the scratch and the broad-phase pairs of the next
/// step.
fn warm(id: felt252, n: u32) -> (World, Span<PairCollider>, Array<(u32, u32)>, Fixed) {
    let mut world = scene(id, n);
    let _ = world.step();
    let (snapshot, infos, _) = user_changes_bodies(ref world.bodies, ref world.colliders);
    let prediction = world.integration_parameters.prediction_distance();
    let (proxies, scratch) = collision_inputs(snapshot, infos, ref world.bodies, prediction);
    let pairs = find_pairs(proxies.span());
    (world, scratch, pairs, prediction)
}

/// The narrow-phase stage of the step after the warm-up; nothing when `!narrow`.
#[inline(never)]
fn stage(id: felt252, n: u32, narrow: bool, variant: u8) {
    let (mut world, scratch, pairs, prediction) = warm(id, n);
    if !narrow {
        return;
    }
    let _ = if variant == SHIPPED {
        compute_contacts_from_scratch::<
            StepDispatcher,
        >(ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders)
    } else if variant == METERED {
        compute_contacts_from_scratch::<
            DefaultDispatcher,
        >(ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders)
    } else if variant == PERSISTENT {
        compute_contacts_from_scratch::<
            PersistentDispatcher,
        >(ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders)
    } else if variant == INLINED {
        compute_contacts_inlined::<
            DefaultDispatcher,
        >(ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders)
    } else {
        compute_contacts_outlined::<
            DefaultDispatcher, SortedMerge,
        >(ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders)
    };
}

/// Inputs of one real pair: `(co1, co2, previous pair, previous pairs, prediction)`.
#[derive(Drop)]
struct PairInputs {
    co1: PairCollider,
    co2: PairCollider,
    previous: ContactPair,
    pairs: Array<ContactPair>,
    prediction: Fixed,
}

/// `kind`: `'bb'` pair 0 of a row of 2 balls, `'bh'` the pair of one ball on the half-space,
/// `'cc'` the upper pair of a stack of 2 cuboids, `'ch'` the pair of one cuboid on the
/// half-space.
#[inline(never)]
fn prep(kind: felt252) -> PairInputs {
    let (id, n, index) = if kind == 'bb' {
        ('row', 2, 0)
    } else if kind == 'bh' {
        ('balls', 1, 0)
    } else if kind == 'cc' {
        ('stack', 2, 2)
    } else {
        ('stack', 1, 0)
    };
    let (world, scratch, pairs, prediction) = warm(id, n);
    let (a, b) = *pairs.at(index);
    let co1 = *scratch.at(a);
    let co2 = *scratch.at(b);
    let previous = world.narrow_phase.contact_pair(co1.handle, co2.handle).unwrap();
    assert!(previous.manifold.data.num_solver_contacts != 0);
    let World { narrow_phase, .. } = world;
    PairInputs { co1, co2, previous, pairs: narrow_phase.pairs, prediction }
}

#[inline(never)]
fn piece_take(pairs: Span<ContactPair>, co1: PairCollider, co2: PairCollider) {
    let mut carry: SortedMerge = CarryOver::begin(pairs);
    let _ = opaque(carry.take(co1.handle, co2.handle));
}

#[inline(never)]
fn piece_pos12(co1: PairCollider, co2: PairCollider) {
    let _ = opaque(co1.pose.inv_mul(co2.pose));
}

#[inline(never)]
fn piece_dispatch(
    pos12: Pose2,
    co1: PairCollider,
    co2: PairCollider,
    prediction: Fixed,
    manifold: ContactManifold,
) {
    let mut manifold = manifold;
    let _ = DefaultDispatcher::contact_manifold(
        pos12, co1.shape, co2.shape, prediction, ref manifold,
    );
    let _ = opaque(manifold);
}

#[inline(never)]
fn piece_combine(co1: PairCollider, co2: PairCollider) {
    let _ = opaque(
        CoefficientCombineRuleTrait::combine(
            co1.friction, co2.friction, co1.friction_combine_rule, co2.friction_combine_rule,
        ),
    );
    let _ = opaque(
        CoefficientCombineRuleTrait::combine(
            co1.restitution,
            co2.restitution,
            co1.restitution_combine_rule,
            co2.restitution_combine_rule,
        ),
    );
}

#[inline(never)]
fn piece_append(pair: ContactPair) {
    let mut out = array![];
    out.append(pair);
    let _ = opaque(out);
}

/// Runs `piece` on the pair `kind` (`'prep'` = inputs only).
#[inline(always)]
fn pair_probe(kind: felt252, piece: felt252) {
    let PairInputs { co1, co2, previous, pairs, prediction } = prep(opaque(kind));
    let (co1, co2, previous) = (opaque(co1), opaque(co2), opaque(previous));
    if piece == 'take' {
        piece_take(pairs.span(), co1, co2);
    } else if piece == 'filtered' {
        let _ = opaque(pair_filtered(co1, co2));
    } else if piece == 'pos12' {
        piece_pos12(co1, co2);
    } else if piece == 'dispatch' {
        let pos12 = opaque(co1.pose.inv_mul(co2.pose));
        piece_dispatch(pos12, co1, co2, prediction, previous.manifold);
    } else if piece == 'update' {
        let _ = opaque(
            update_manifold::<DefaultDispatcher>(prediction, co1, co2, previous.manifold),
        );
    } else if piece == 'solver_contact' {
        let [p0, _] = previous.manifold.points;
        let _ = opaque(solver_contact(p0, 0, co1, co2));
    } else if piece == 'combine' {
        piece_combine(co1, co2);
    } else if piece == 'process' {
        let _ = opaque(process_pair::<DefaultDispatcher>(prediction, co1, co2, Some(previous)));
    } else if piece == 'update_old' {
        let _ = opaque(
            update_manifold_outlined::<DefaultDispatcher>(prediction, co1, co2, previous.manifold),
        );
    } else if piece == 'process_old' {
        let _ = opaque(
            process_pair_outlined::<DefaultDispatcher>(prediction, co1, co2, Some(previous)),
        );
    } else if piece == 'match' {
        let mut manifold = previous.manifold;
        manifold.match_contacts(@previous.manifold);
        let _ = opaque(manifold);
    } else if piece == 'transition' {
        let _ = opaque(pair_transition(co1, co2, previous.manifold, previous.event_status, true));
    } else if piece == 'append' {
        piece_append(previous);
    }
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
fn gas_np_setup_stack3() {
    stage(opaque('stack'), opaque(3), false, SHIPPED);
}

#[test]
fn gas_np_stack3_shipped() {
    stage(opaque('stack'), opaque(3), true, SHIPPED);
}

#[test]
fn gas_np_stack3_outlined() {
    stage(opaque('stack'), opaque(3), true, OUTLINED);
}

#[test]
fn gas_np_stack3_metered() {
    stage(opaque('stack'), opaque(3), true, METERED);
}

#[test]
fn gas_np_stack3_persistent() {
    stage(opaque('stack'), opaque(3), true, PERSISTENT);
}

#[test]
fn gas_np_stack3_inlined() {
    stage(opaque('stack'), opaque(3), true, INLINED);
}

#[test]
fn gas_np_setup_balls8() {
    stage(opaque('balls'), opaque(8), false, SHIPPED);
}

#[test]
fn gas_np_balls8_shipped() {
    stage(opaque('balls'), opaque(8), true, SHIPPED);
}

#[test]
fn gas_np_balls8_outlined() {
    stage(opaque('balls'), opaque(8), true, OUTLINED);
}

#[test]
fn gas_np_balls8_metered() {
    stage(opaque('balls'), opaque(8), true, METERED);
}

#[test]
fn gas_np_balls8_persistent() {
    stage(opaque('balls'), opaque(8), true, PERSISTENT);
}

#[test]
fn gas_np_balls8_inlined() {
    stage(opaque('balls'), opaque(8), true, INLINED);
}

#[test]
fn gas_np_setup_mixed8() {
    stage(opaque('mixed'), opaque(8), false, SHIPPED);
}

#[test]
fn gas_np_mixed8_shipped() {
    stage(opaque('mixed'), opaque(8), true, SHIPPED);
}

#[test]
fn gas_np_mixed8_outlined() {
    stage(opaque('mixed'), opaque(8), true, OUTLINED);
}

#[test]
fn gas_np_mixed8_metered() {
    stage(opaque('mixed'), opaque(8), true, METERED);
}

#[test]
fn gas_np_mixed8_persistent() {
    stage(opaque('mixed'), opaque(8), true, PERSISTENT);
}

#[test]
fn gas_np_mixed8_inlined() {
    stage(opaque('mixed'), opaque(8), true, INLINED);
}

#[test]
fn gas_np_setup_bb() {
    stage(opaque('row'), opaque(2), false, SHIPPED);
}

#[test]
fn gas_np_bb_shipped() {
    stage(opaque('row'), opaque(2), true, SHIPPED);
}

#[test]
fn gas_np_bb_outlined() {
    stage(opaque('row'), opaque(2), true, OUTLINED);
}

#[test]
fn gas_np_setup_bh() {
    stage(opaque('balls'), opaque(1), false, SHIPPED);
}

#[test]
fn gas_np_bh_shipped() {
    stage(opaque('balls'), opaque(1), true, SHIPPED);
}

#[test]
fn gas_np_bh_outlined() {
    stage(opaque('balls'), opaque(1), true, OUTLINED);
}

#[test]
fn gas_np_setup_cc() {
    stage(opaque('cubes'), opaque(2), false, SHIPPED);
}

#[test]
fn gas_np_cc_shipped() {
    stage(opaque('cubes'), opaque(2), true, SHIPPED);
}

#[test]
fn gas_np_cc_outlined() {
    stage(opaque('cubes'), opaque(2), true, OUTLINED);
}

#[test]
fn gas_np_setup_ch() {
    stage(opaque('stack'), opaque(1), false, SHIPPED);
}

#[test]
fn gas_np_ch_shipped() {
    stage(opaque('stack'), opaque(1), true, SHIPPED);
}

#[test]
fn gas_np_ch_outlined() {
    stage(opaque('stack'), opaque(1), true, OUTLINED);
}

#[test]
fn gas_pair_prep_bb() {
    pair_probe('bb', 'prep');
}

#[test]
fn gas_pair_take_bb() {
    pair_probe('bb', 'take');
}

#[test]
fn gas_pair_filtered_bb() {
    pair_probe('bb', 'filtered');
}

#[test]
fn gas_pair_pos12_bb() {
    pair_probe('bb', 'pos12');
}

#[test]
fn gas_pair_dispatch_bb() {
    pair_probe('bb', 'dispatch');
}

#[test]
fn gas_pair_update_bb() {
    pair_probe('bb', 'update');
}

#[test]
fn gas_pair_solver_contact_bb() {
    pair_probe('bb', 'solver_contact');
}

#[test]
fn gas_pair_combine_bb() {
    pair_probe('bb', 'combine');
}

#[test]
fn gas_pair_process_bb() {
    pair_probe('bb', 'process');
}

#[test]
fn gas_pair_process_old_bb() {
    pair_probe('bb', 'process_old');
}

#[test]
fn gas_pair_update_old_bb() {
    pair_probe('bb', 'update_old');
}

#[test]
fn gas_pair_append_bb() {
    pair_probe('bb', 'append');
}

#[test]
fn gas_pair_prep_bh() {
    pair_probe('bh', 'prep');
}

#[test]
fn gas_pair_dispatch_bh() {
    pair_probe('bh', 'dispatch');
}

#[test]
fn gas_pair_update_bh() {
    pair_probe('bh', 'update');
}

#[test]
fn gas_pair_process_bh() {
    pair_probe('bh', 'process');
}

#[test]
fn gas_pair_process_old_bh() {
    pair_probe('bh', 'process_old');
}

#[test]
fn gas_pair_update_old_bh() {
    pair_probe('bh', 'update_old');
}

#[test]
fn gas_pair_prep_cc() {
    pair_probe('cc', 'prep');
}

#[test]
fn gas_pair_dispatch_cc() {
    pair_probe('cc', 'dispatch');
}

#[test]
fn gas_pair_update_cc() {
    pair_probe('cc', 'update');
}

#[test]
fn gas_pair_process_cc() {
    pair_probe('cc', 'process');
}

#[test]
fn gas_pair_process_old_cc() {
    pair_probe('cc', 'process_old');
}

#[test]
fn gas_pair_update_old_cc() {
    pair_probe('cc', 'update_old');
}

#[test]
fn gas_pair_prep_ch() {
    pair_probe('ch', 'prep');
}

#[test]
fn gas_pair_dispatch_ch() {
    pair_probe('ch', 'dispatch');
}

#[test]
fn gas_pair_update_ch() {
    pair_probe('ch', 'update');
}

#[test]
fn gas_pair_process_ch() {
    pair_probe('ch', 'process');
}

#[test]
fn gas_pair_process_old_ch() {
    pair_probe('ch', 'process_old');
}

#[test]
fn gas_pair_update_old_ch() {
    pair_probe('ch', 'update_old');
}

#[test]
fn gas_pair_match_bb() {
    pair_probe('bb', 'match');
}

#[test]
fn gas_pair_transition_bb() {
    pair_probe('bb', 'transition');
}

#[test]
fn gas_pair_match_cc() {
    pair_probe('cc', 'match');
}

#[test]
fn gas_pair_match_ch() {
    pair_probe('ch', 'match');
}

#[test]
fn gas_pair_match_bh() {
    pair_probe('bh', 'match');
}
