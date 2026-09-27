//! SF1: the `tilted_landing` family against rapier2d-f64 0.35.3: `ell_twin`, an L made of two
//! plain cuboid colliders on one dynamic body (a tall bar and an arm, the centre of mass off the
//! bar's base), dropped flat on the upward half-space, 90 steps. It lands at step 6 on the bar's
//! two corners, one of them at a near-zero gap, and pivots on the other one (the arm reaches the
//! ground at step 94, beyond the trace: there upstream solves the new ground–arm pair first, the
//! port in pair order).
//! The ground is a standalone collider (slot 0, collision events on), the parts slots 1 and 2; no
//! contact recycling nor clustering, as the other scene families.
//!
//! Every step compares the body with upstream's sample (`2^12 · step` ulp on the pose, twice that
//! on the velocities), the number of manifolds with solver contacts of the ground's pairs and the
//! collision events, exactly. Before SF1 the port left upstream's trajectory at step 7 (1.1e6 ulp
//! on the velocities): the corner's first-substep gap came out `0` (soft row) instead of upstream's
//! `+1.26` raw (rigid row), the floored round trip of the anchors; the frozen separation is now
//! rebased on it (`rapier_dynamics2d::solver::contact::rebase`).
use rapier2d::prelude::{ColliderBuilderTrait, RigidBodyTrait, World, WorldTrait};
use rapier_core::collider::events::COLLISION_EVENTS;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_dynamics2d::events::CollisionEventTrait;
use rapier_dynamics2d::narrow_phase::composite::contact_pair_manifolds;
use rapier_golden::generated::tilted_landing;
use rapier_math::pose2::Pose2;
use super::builder::{body_handle, f, vr};
use super::compound::{judge_sample, part, pose};

fn run(index: u32) {
    let (id, start) = *tilted_landing::scenes().at(index);
    let params = IntegrationParameters {
        dt: f(tilted_landing::DT),
        contact_recycling: false,
        contact_clustering: false,
        max_ccd_substeps: 0,
        ..Default::default(),
    };
    let mut world: World = WorldTrait::new(vr(tilted_landing::GRAVITY), params);
    let up = rapier2d::prelude::Vec2 { x: f(0), y: f(0x100000000) };
    let g = world
        .insert_collider(
            ColliderBuilderTrait::halfspace(up).active_events(COLLISION_EVENTS).build(), None,
        );
    let rb = RigidBodyTrait::dynamic(Pose2 { translation: vr(start), ..Default::default() });
    let mut parts = tilted_landing::parts(index).parts;
    let (p0, s0) = *parts.pop_front().unwrap();
    let (body, c0) = world
        .insert(rb, ColliderBuilderTrait::new(part(s0)).position(pose(p0)).build());
    let mut colliders = array![c0];
    for (p, s) in parts {
        colliders
            .append(
                world
                    .insert_collider(
                        ColliderBuilderTrait::new(part(*s)).position(pose(*p)).build(), Some(body),
                    ),
            );
    }
    assert!(g.index == 0 && c0.index == 1 && body == body_handle(0), "{} handles", id);
    let mut events = tilted_landing::events(index);
    let (mut worst, mut worst_v): (u64, u64) = (0, 0);
    for sample in tilted_landing::samples(index) {
        let (step, _, _, _, _, _, _, _, manifolds) = *sample;
        for e in world.step() {
            let (e1, e2) = (e.collider1(), e.collider2());
            let (s, started, c1, c2) = *events.pop_front().expect('unexpected event');
            assert_eq!((step, e.started(), e1.index, e2.index), (s, started, c1, c2), "{}", id);
        }
        let (d, d_v) = judge_sample(id, ref world, body, *sample, false);
        if d > worst {
            worst = d;
        }
        if d_v > worst_v {
            worst_v = d_v;
        }
        let mut with_contacts = 0;
        for c in colliders.span() {
            for m in contact_pair_manifolds(world.narrow_phase.pairs.span(), g, *c) {
                if m.data.num_solver_contacts != 0 {
                    with_contacts += 1;
                }
            }
        }
        assert_eq!(with_contacts, manifolds, "{} step {} manifolds", id, step);
    }
    assert!(events.is_empty(), "{} missing events", id);
    println!("{}: worst pose deviation {} ulp, velocity {} ulp", id, worst, worst_v);
}

#[test]
fn test_ell_twin() {
    run(0);
}
