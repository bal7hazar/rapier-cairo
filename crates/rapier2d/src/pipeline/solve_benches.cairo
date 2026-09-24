//! Gas probes of the fused solve + position update (work package OI). `run` builds a world,
//! steps it `warmup` times, runs stages 1–3 of one more step, then the first `stage` parts of
//! `solve_and_advance` (a copy, cut after each part): 1 constrained membership and store
//! entries, 2 `from_entries`, 3 `solve_island`, 4 impulse scatter and joint writeback,
//! 5 `FreeBodySolverTrait::new`, 6 the body walk (= the whole stage). `gas_<scene>_solve_<k>`
//! − `gas_<scene>_solve_<k-1>` is the cost of part `k` (`_solve_0` = stages 1–3 only).
//! `gas_<scene>_<variant>` − `gas_<scene>_solve_0` is the whole stage of a candidate: `staged`
//! (`solve` then `advance_with_snapshot`, before OI), `shipped`, and the `solve_alternatives`.

use core::dict::{Felt252Dict, Felt252DictTrait};
use rapier_core::integration_parameters::IntegrationParametersTrait;
use rapier_core::rigid_body::RigidBodyType;
use rapier_dynamics2d::joint::{ImpulseJointSetTrait, JointEnabled};
use rapier_dynamics2d::solver::body_store::SolverBodyStoreTrait;
use rapier_dynamics2d::solver::island::{FreeBodySolverTrait, solve_island};
use rapier_geometry2d::broad_phase::find_pairs;
use rapier_testing::opaque;
use crate::world::{World, WorldTrait};
use super::fixtures::free_fall;
use super::solve_alternatives::{
    solve_and_advance_island_always, solve_and_advance_lazy, solve_and_advance_member_handles,
    solve_and_advance_metered, solve_and_advance_separate_marking,
};
use super::{
    advance_body_with_snapshot, collision_inputs, contacts_from_scratch, joint_values,
    scatter_touching, solve_order, user_changes_bodies, write_joints,
};

fn world_of(id: felt252) -> World {
    if id == 'free32' {
        free_fall(32)
    } else if id == 'free8' {
        free_fall(8)
    } else if id == 'free1' {
        free_fall(1)
    } else if id == 'pendulum' {
        let (world, _) = super::fixtures::scene_world(rapier_golden::scenes::PENDULUM);
        world
    } else if id == 'resting' {
        let (world, _, _) = super::fixtures::ball_on_ground(fixed::HALF, false);
        world
    } else {
        let (world, _) = super::fixtures::scene_world(rapier_golden::scenes::BOX_STACK3);
        world
    }
}

/// The staged `solve` then `advance_with_snapshot` (before OI).
const STAGED: u8 = 7;
/// `solve_and_advance` as shipped.
const SHIPPED: u8 = 8;
/// `solve_alternatives::solve_and_advance_metered`.
const METERED: u8 = 9;
/// `solve_alternatives::solve_and_advance_separate_marking`.
const SEPARATE_MARKING: u8 = 10;
/// `solve_alternatives::solve_and_advance_member_handles`.
const MEMBER_HANDLES: u8 = 11;
/// `solve_alternatives::solve_and_advance_lazy`.
const LAZY: u8 = 12;
/// `solve_alternatives::solve_and_advance_island_always`.
const ISLAND_ALWAYS: u8 = 13;

#[inline(never)]
fn run(id: felt252, warmup: u32, stage: u8) {
    let mut world = world_of(id);
    let mut i = 0;
    while i != warmup {
        let _ = world.step();
        i += 1;
    }
    let (snapshot, infos, entries) = user_changes_bodies(ref world.bodies, ref world.colliders);
    let prediction = world.integration_parameters.prediction_distance();
    let (proxies, scratch) = collision_inputs(snapshot, infos, ref world.bodies, prediction);
    let pairs = find_pairs(proxies.span());
    let _ = contacts_from_scratch(
        ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders,
    );
    if stage == 0 {
        return;
    }
    if stage == STAGED {
        super::solve(
            world.gravity,
            world.integration_parameters,
            ref world.bodies,
            ref world.narrow_phase,
            ref world.impulse_joints,
        );
        super::advance_with_snapshot(ref world.bodies, ref world.colliders, snapshot);
        return;
    }
    if stage >= 8 {
        let g = world.gravity;
        let p = world.integration_parameters;
        if stage == SHIPPED {
            super::solve_and_advance(
                g,
                p,
                ref world.bodies,
                ref world.colliders,
                ref world.narrow_phase,
                ref world.impulse_joints,
                entries,
                snapshot,
            );
        } else if stage == METERED {
            solve_and_advance_metered(
                g,
                p,
                ref world.bodies,
                ref world.colliders,
                ref world.narrow_phase,
                ref world.impulse_joints,
                entries,
                snapshot,
            );
        } else if stage == SEPARATE_MARKING {
            solve_and_advance_separate_marking(
                g,
                p,
                ref world.bodies,
                ref world.colliders,
                ref world.narrow_phase,
                ref world.impulse_joints,
                entries,
                snapshot,
            );
        } else if stage == MEMBER_HANDLES {
            solve_and_advance_member_handles(
                g,
                p,
                ref world.bodies,
                ref world.colliders,
                ref world.narrow_phase,
                ref world.impulse_joints,
                entries,
                snapshot,
            );
        } else if stage == LAZY {
            solve_and_advance_lazy(
                g,
                p,
                ref world.bodies,
                ref world.colliders,
                ref world.narrow_phase,
                ref world.impulse_joints,
                entries,
                snapshot,
            );
        } else {
            solve_and_advance_island_always(
                g,
                p,
                ref world.bodies,
                ref world.colliders,
                ref world.narrow_phase,
                ref world.impulse_joints,
                entries,
                snapshot,
            );
        }
        return;
    }
    let gravity = world.gravity;
    let params = world.integration_parameters;
    let mut constrained: Felt252Dict<bool> = Default::default();
    let mut manifolds = array![];
    for pair in world.narrow_phase.pairs.span() {
        if *pair.manifold.data.num_solver_contacts != 0 {
            let manifold = *pair.manifold;
            if let Some(h) = manifold.data.rigid_body1 {
                constrained.insert(h.into(), true);
            }
            if let Some(h) = manifold.data.rigid_body2 {
                constrained.insert(h.into(), true);
            }
            manifolds.append(manifold);
        }
    }
    let joint_entries = world.impulse_joints.to_array();
    let mut joints = joint_values(joint_entries.span());
    for joint in joints.span() {
        if *joint.data.enabled == JointEnabled::Enabled {
            constrained.insert((*joint.body1).into(), true);
            constrained.insert((*joint.body2).into(), true);
        }
    }
    let any = !manifolds.is_empty() || !joints.is_empty();
    let (mut manifolds, last) = if manifolds.is_empty() {
        (manifolds, array![])
    } else {
        solve_order(world.narrow_phase.pairs.span(), entries)
    };
    let mut members = array![];
    let mut has_free = false;
    if any {
        for entry in entries {
            let (handle, body) = entry;
            if constrained.get((*handle).into()) {
                members.append(*entry);
            } else if *body.enabled && *body.body_type != RigidBodyType::Fixed {
                has_free = true;
            }
        }
    }
    if stage == 1 {
        return;
    }
    let mut store = SolverBodyStoreTrait::from_entries(members.span(), gravity, params);
    if stage == 2 {
        return;
    }
    if any {
        solve_island(params, ref store, ref manifolds, ref joints);
    }
    if stage == 3 {
        return;
    }
    if any {
        if !manifolds.is_empty() {
            world
                .narrow_phase
                .pairs =
                    scatter_touching(
                        world.narrow_phase.pairs.span(), manifolds.span(), last.span(),
                    );
        }
        write_joints(joint_entries.span(), joints.span(), ref world.impulse_joints);
    }
    if stage == 4 {
        return;
    }
    let free = if !any || has_free {
        FreeBodySolverTrait::new(params, gravity)
    } else {
        Default::default()
    };
    if stage == 5 {
        return;
    }
    let mut dense: u32 = 0;
    for (handle, body) in entries {
        let member = any && constrained.get((*handle).into());
        if *body.enabled && *body.body_type != RigidBodyType::Fixed {
            let body = if member {
                let mut body = *body;
                store.write_body(dense, ref body);
                body
            } else {
                free.solve(*handle, *body)
            };
            advance_body_with_snapshot(
                *handle, body, ref world.bodies, ref world.colliders, snapshot,
            );
        }
        if member {
            dense += 1;
        }
    }
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
fn gas_free32_solve_0() {
    run(opaque('free32'), 2, 0);
}

#[test]
fn gas_free32_solve_1() {
    run(opaque('free32'), 2, 1);
}

#[test]
fn gas_free32_solve_2() {
    run(opaque('free32'), 2, 2);
}

#[test]
fn gas_free32_solve_3() {
    run(opaque('free32'), 2, 3);
}

#[test]
fn gas_free32_solve_4() {
    run(opaque('free32'), 2, 4);
}

#[test]
fn gas_free32_solve_5() {
    run(opaque('free32'), 2, 5);
}

#[test]
fn gas_free32_solve_6() {
    run(opaque('free32'), 2, 6);
}

#[test]
fn gas_resting_solve_0() {
    run(opaque('resting'), 3, 0);
}

#[test]
fn gas_resting_solve_1() {
    run(opaque('resting'), 3, 1);
}

#[test]
fn gas_resting_solve_2() {
    run(opaque('resting'), 3, 2);
}

#[test]
fn gas_resting_solve_3() {
    run(opaque('resting'), 3, 3);
}

#[test]
fn gas_resting_solve_4() {
    run(opaque('resting'), 3, 4);
}

#[test]
fn gas_resting_solve_5() {
    run(opaque('resting'), 3, 5);
}

#[test]
fn gas_resting_solve_6() {
    run(opaque('resting'), 3, 6);
}

#[test]
fn gas_stack3_solve_0() {
    run(opaque('stack3'), 60, 0);
}
#[test]
fn gas_pendulum_solve_0() {
    run(opaque('pendulum'), 3, 0);
}

#[test]
fn gas_free32_staged() {
    run(opaque('free32'), 2, STAGED);
}

#[test]
fn gas_free32_shipped() {
    run(opaque('free32'), 2, SHIPPED);
}

#[test]
fn gas_free32_metered() {
    run(opaque('free32'), 2, METERED);
}

#[test]
fn gas_free32_separate_marking() {
    run(opaque('free32'), 2, SEPARATE_MARKING);
}

#[test]
fn gas_free32_member_handles() {
    run(opaque('free32'), 2, MEMBER_HANDLES);
}

#[test]
fn gas_free32_lazy() {
    run(opaque('free32'), 2, LAZY);
}

#[test]
fn gas_free32_island_always() {
    run(opaque('free32'), 2, ISLAND_ALWAYS);
}

#[test]
fn gas_stack3_staged() {
    run(opaque('stack3'), 60, STAGED);
}

#[test]
fn gas_stack3_shipped() {
    run(opaque('stack3'), 60, SHIPPED);
}

#[test]
fn gas_stack3_metered() {
    run(opaque('stack3'), 60, METERED);
}

#[test]
fn gas_stack3_separate_marking() {
    run(opaque('stack3'), 60, SEPARATE_MARKING);
}

#[test]
fn gas_stack3_member_handles() {
    run(opaque('stack3'), 60, MEMBER_HANDLES);
}

#[test]
fn gas_stack3_lazy() {
    run(opaque('stack3'), 60, LAZY);
}

#[test]
fn gas_stack3_island_always() {
    run(opaque('stack3'), 60, ISLAND_ALWAYS);
}

#[test]
fn gas_resting_staged() {
    run(opaque('resting'), 3, STAGED);
}

#[test]
fn gas_resting_shipped() {
    run(opaque('resting'), 3, SHIPPED);
}

#[test]
fn gas_resting_metered() {
    run(opaque('resting'), 3, METERED);
}

#[test]
fn gas_resting_separate_marking() {
    run(opaque('resting'), 3, SEPARATE_MARKING);
}

#[test]
fn gas_resting_member_handles() {
    run(opaque('resting'), 3, MEMBER_HANDLES);
}

#[test]
fn gas_resting_lazy() {
    run(opaque('resting'), 3, LAZY);
}

#[test]
fn gas_resting_island_always() {
    run(opaque('resting'), 3, ISLAND_ALWAYS);
}

#[test]
fn gas_pendulum_staged() {
    run(opaque('pendulum'), 3, STAGED);
}

#[test]
fn gas_pendulum_shipped() {
    run(opaque('pendulum'), 3, SHIPPED);
}

#[test]
fn gas_pendulum_metered() {
    run(opaque('pendulum'), 3, METERED);
}

#[test]
fn gas_pendulum_separate_marking() {
    run(opaque('pendulum'), 3, SEPARATE_MARKING);
}

#[test]
fn gas_pendulum_member_handles() {
    run(opaque('pendulum'), 3, MEMBER_HANDLES);
}

#[test]
fn gas_pendulum_lazy() {
    run(opaque('pendulum'), 3, LAZY);
}

#[test]
fn gas_pendulum_island_always() {
    run(opaque('pendulum'), 3, ISLAND_ALWAYS);
}
