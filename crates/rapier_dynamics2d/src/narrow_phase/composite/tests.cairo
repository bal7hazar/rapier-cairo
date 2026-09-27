use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::collider::events::COLLISION_EVENTS;
use rapier_core::interaction_groups::{InteractionGroups, InteractionGroupsTrait};
use rapier_geometry2d::contact::ContactManifold;
use rapier_geometry2d::dispatch::contact_manifold_step;
use rapier_geometry2d::shape::Shape;
use rapier_math::pose2::Pose2;
use rapier_testing::opaque;
use crate::collider::ColliderBuilderTrait;
use crate::collider_set::{ColliderSet, ColliderSetTrait};
use crate::events::{CollisionEvent, CollisionEventTrait, PairEventStatusTrait};
use crate::rigid_body_set::{RigidBodySet, RigidBodySetTrait, RigidBodyTrait};
use super::super::mock::{at, broad_phase};
use super::super::{ContactDispatcher, NarrowPhase, NarrowPhaseTrait};
use super::{contact_pair_manifolds, group_len};

/// The geometry crate's step dispatcher (composite pairs: unsupported, handled by the group).
impl GeoDispatcher of ContactDispatcher {
    fn contact_manifold(
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) -> bool {
        contact_manifold_step(pos12, shape1, shape2, prediction, ref manifold)
    }
}

const PREDICTION: Fixed = Fixed { raw: 85899346 };

fn int(x: i32) -> Fixed {
    FixedTrait::from_int(x)
}

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

/// A fixed flat heightfield over `x in [-4, 4]` (four cells) with collision events, and a box of
/// half extents `(1.25, 0.5)` on a dynamic body at `(x, y)`: `(bodies, colliders, ground, box)`.
fn scene(x: Fixed, y: Fixed) -> (RigidBodySet, ColliderSet, Handle, Handle) {
    let mut bodies = RigidBodySetTrait::new();
    let mut colliders = ColliderSetTrait::new();
    let ground_body = bodies.insert(RigidBodyTrait::fixed(at(ZERO, ZERO)));
    let ground = colliders
        .insert_with_parent(
            ColliderBuilderTrait::heightfield(
                array![ZERO, ZERO, ZERO, ZERO, ZERO].span(), v(int(8), ONE),
            )
                .active_events(COLLISION_EVENTS)
                .build(),
            ground_body,
            ref bodies,
        );
    let body = bodies.insert(RigidBodyTrait::dynamic(at(x, y)));
    let cuboid = colliders
        .insert_with_parent(
            ColliderBuilderTrait::cuboid(ONE + FixedTrait::from_ratio(1, 4), HALF).build(),
            body,
            ref bodies,
        );
    (bodies, colliders, ground, cuboid)
}

fn step(
    ref np: NarrowPhase, ref bodies: RigidBodySet, ref colliders: ColliderSet,
) -> Array<CollisionEvent> {
    let pairs = broad_phase(ref bodies, ref colliders, PREDICTION);
    np.compute_contacts::<GeoDispatcher>(PREDICTION, ref bodies, ref colliders, pairs.span())
}

/// Sets the collision groups of the collider `handle`.
fn set_groups(ref colliders: ColliderSet, handle: Handle, groups: InteractionGroups) {
    let mut c = colliders.get(handle).unwrap();
    c.flags.collision_groups = groups;
    let _ = colliders.set(handle, c);
}

/// Moves the collider `handle` to `pose` (the narrow phase reads the colliders' poses).
fn move_body(ref bodies: RigidBodySet, ref colliders: ColliderSet, handle: Handle, pose: Pose2) {
    let mut c = colliders.get(handle).unwrap();
    c.pos.pose = pose;
    let _ = colliders.set(handle, c);
}

#[test]
fn test_group_entries_and_events_per_collider_pair() {
    // Resting across the seam `x = 0`, 1/64 into the ground: cells 1 and 2 touch.
    let sink = FixedTrait::from_ratio(1, 64);
    let (mut bodies, mut colliders, ground, body) = scene(
        FixedTrait::from_ratio(1, 4), HALF - sink,
    );
    let mut np = NarrowPhaseTrait::new();
    let events = step(ref np, ref bodies, ref colliders);
    // One `Started` for the collider pair, whatever the number of manifolds.
    assert_eq!(events.len(), 1);
    assert!((*events.at(0)).started());
    assert_eq!(np.len(), 2);
    assert_eq!(group_len(np.pairs.span(), 0), 2);
    let pair = np.pairs.at(0);
    assert!(*pair.collider1 == ground || *pair.collider2 == ground);
    let manifolds = contact_pair_manifolds(np.pairs.span(), *pair.collider1, *pair.collider2);
    assert_eq!(manifolds.len(), 2);
    // Every entry has solver contacts; the lead (first) carries the event status.
    assert!(*manifolds.at(0).data.num_solver_contacts == 2);
    assert!(*manifolds.at(1).data.num_solver_contacts == 2);
    assert!((*np.pairs.at(0).event_status).start_event_emitted());
    assert!(!(*np.pairs.at(1).event_status).start_event_emitted());
    // Same poses: no new event, same group.
    let events = step(ref np, ref bodies, ref colliders);
    assert_eq!(events.len(), 0);
    assert_eq!(np.len(), 2);
    // Filtered by collision groups: the lead had a contact, one `Stopped`, one plain entry.
    set_groups(ref colliders, body, InteractionGroupsTrait::none());
    let events = step(ref np, ref bodies, ref colliders);
    assert_eq!(events.len(), 1, "filtered");
    assert!((*events.at(0)).stopped());
    assert_eq!(np.len(), 1);
    // Groups restored: `Started` again, the group rebuilt.
    set_groups(ref colliders, body, InteractionGroupsTrait::all());
    let events = step(ref np, ref bodies, ref colliders);
    assert_eq!(events.len(), 1, "back");
    assert!((*events.at(0)).started());
    assert_eq!(np.len(), 2);
    // Out of the broad phase: the dropped group emits one `Stopped` (from its lead).
    move_body(ref bodies, ref colliders, body, at(ZERO, int(10)));
    let events = step(ref np, ref bodies, ref colliders);
    assert_eq!(events.len(), 1, "away");
    assert!((*events.at(0)).stopped());
    assert_eq!(np.len(), 0);
}

#[test]
fn test_group_lead_is_a_touching_manifold() {
    // Resting on cell 2 only, the box's AABB (loosened) reaching into cell 1 without touching it
    // is impossible on a flat ground: shift the box so that cell 3 is in the box and touches.
    let sink = FixedTrait::from_ratio(1, 64);
    let (mut bodies, mut colliders, _, _) = scene(int(2), HALF - sink);
    let mut np = NarrowPhaseTrait::new();
    let _ = step(ref np, ref bodies, ref colliders);
    // Cells 2 and 3 each touch: the lead has solver contacts.
    assert!(np.len() >= 2);
    assert!(*np.pairs.at(0).manifold.data.num_solver_contacts != 0);
    // Warm start: the second step keeps the parts' contact data (impulses zero, ids matched).
    let first = np.pairs.span();
    let _ = step(ref np, ref bodies, ref colliders);
    assert_eq!(np.pairs.span().len(), first.len());
}

#[test]
fn gas_baseline() {}

#[test]
fn gas_group_step_box_on_heightfield() {
    let (mut bodies, mut colliders, _, _) = scene(
        FixedTrait::from_ratio(1, 4), HALF - FixedTrait::from_ratio(1, 64),
    );
    let mut np = NarrowPhaseTrait::new();
    let pairs = broad_phase(ref bodies, ref colliders, PREDICTION);
    let _ = np
        .compute_contacts::<
            GeoDispatcher,
        >(opaque(PREDICTION), ref bodies, ref colliders, opaque(pairs.span()));
}
