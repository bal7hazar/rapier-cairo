//! The set of colliders (upstream `geometry/collider_set.rs`, `ColliderSet`).
//!
//! A generational [`Arena`] of [`Collider`] values. Attaching a collider to a body goes through
//! [`ColliderSetTrait::insert_with_parent`], which keeps the body's collider list, mass
//! properties and the collider's world pose consistent, as upstream.
//!
//! [`ColliderSetTrait::broad_phase_proxies`] builds the input of
//! `rapier_geometry2d::broad_phase::find_pairs` with one proxy per collider in ascending slot
//! index, so that the proxy indices of the returned pairs are indices into
//! [`ColliderSetTrait::iter`] — the contract `NarrowPhaseTrait::compute_contacts` relies on.
//!
//! Deviations from upstream: no modified / removed collider lists (the step is stateless apart
//! from the sets, `docs/PLAN.md` D7/D9); `remove` takes no island manager (no wake-up); `get_mut`
//! is `set`.

use fixed::{Fixed, HALF};
use rapier_core::Handle;
use rapier_core::data::arena::{Arena, ArenaTrait};
use rapier_geometry2d::aabb::AabbTrait;
use rapier_geometry2d::broad_phase::BroadPhaseProxy;
use crate::collider::{Collider, ColliderParent, ColliderTrait};
use crate::rigid_body_set::{
    RigidBodySet, RigidBodySetTrait, RigidBodyTrait, attach_collider, detach_collider,
};

/// The set of colliders. Holds a dict: pass it by `ref`.
#[derive(Destruct, Default)]
pub struct ColliderSet {
    colliders: Arena<Collider>,
}

/// Operations of [`ColliderSet`]. Reads take `ref self` because arena reads mutate the dict log.
#[generate_trait]
pub impl ColliderSetImpl of ColliderSetTrait {
    /// An empty set.
    #[inline(always)]
    fn new() -> ColliderSet {
        ColliderSet { colliders: ArenaTrait::new() }
    }

    /// Stores a standalone collider (its parent is cleared, as upstream) and returns its handle.
    /// Its world pose is its current `pos`.
    fn insert(ref self: ColliderSet, collider: Collider) -> Handle {
        let mut collider = collider;
        collider.parent = None;
        self.colliders.insert(collider)
    }

    /// Stores `collider` attached to the body `parent` and returns its handle (upstream
    /// `insert_with_parent`). The pose relative to the parent is the collider's existing
    /// `pos_wrt_parent` if it already had a parent, its current world pose `pos` otherwise; the
    /// world pose becomes `body.position * pos_wrt_parent`. The body gets the handle appended to
    /// its collider list, `COLLIDERS` raised, and the collider's mass added to its own.
    ///
    /// # Panics
    /// `RigidBodySet: body not found` when `parent` does not resolve (upstream: "Parent rigid
    /// body not found."). Nothing is inserted in that case.
    fn insert_with_parent(
        ref self: ColliderSet, collider: Collider, parent: Handle, ref bodies: RigidBodySet,
    ) -> Handle {
        let mut collider = collider;
        let pos_wrt_parent = match collider.parent {
            Some(previous) => previous.pos_wrt_parent,
            None => collider.pos.pose,
        };
        collider.parent = Some(ColliderParent { handle: parent, pos_wrt_parent });
        assert(bodies.contains(parent), crate::rigid_body_set::errors::BODY_NOT_FOUND);
        let handle = self.colliders.insert(collider);
        let body_pose = attach_collider(ref bodies, parent, handle, collider, pos_wrt_parent);
        collider.pos.pose = body_pose * pos_wrt_parent;
        let _ = self.colliders.set(handle, collider);
        handle
    }

    /// Removes the collider behind `handle`, `None` when it does not resolve. It is removed
    /// from its parent's collider list (swap-remove, `COLLIDERS` raised) when the parent still
    /// exists.
    fn remove(ref self: ColliderSet, handle: Handle, ref bodies: RigidBodySet) -> Option<Collider> {
        let collider = self.colliders.remove(handle)?;
        if let Some(parent) = collider.parent {
            detach_collider(ref bodies, parent.handle, handle);
        }
        Some(collider)
    }

    /// The collider behind `handle`, `None` when the handle is stale or unknown.
    #[inline(always)]
    fn get(ref self: ColliderSet, handle: Handle) -> Option<Collider> {
        self.colliders.get(handle)
    }

    /// Overwrites the collider behind `handle` (upstream `get_mut`). Returns `false` and
    /// changes nothing when the handle does not resolve. The parent link is the caller's
    /// responsibility: change it through `insert_with_parent` / `remove` only.
    #[inline(always)]
    fn set(ref self: ColliderSet, handle: Handle, collider: Collider) -> bool {
        self.colliders.set(handle, collider)
    }

    /// `true` when `handle` resolves to a collider.
    #[inline(always)]
    fn contains(ref self: ColliderSet, handle: Handle) -> bool {
        self.colliders.contains(handle)
    }

    /// Number of colliders.
    #[inline(always)]
    fn len(self: @ColliderSet) -> u32 {
        self.colliders.len()
    }

    /// `true` when the set holds no collider.
    #[inline(always)]
    fn is_empty(self: @ColliderSet) -> bool {
        self.colliders.is_empty()
    }

    /// Every `(handle, collider)` in ascending slot index.
    #[inline(always)]
    fn iter(ref self: ColliderSet) -> Array<(Handle, Collider)> {
        self.colliders.to_array()
    }

    /// One broad-phase proxy per collider, in the order of [`iter`](ColliderSetTrait::iter):
    /// the world AABB loosened by `prediction / 2` on each side (upstream
    /// `compute_collision_aabb(prediction / 2)`, so two shapes closer than `prediction` overlap),
    /// `is_static` when the collider has no parent or a fixed parent. Disabled colliders keep
    /// their proxy (the narrow phase skips them) so that indices stay aligned with `iter`.
    ///
    /// Cost: one dict read per collider and one per parented collider.
    fn broad_phase_proxies(
        ref self: ColliderSet, ref bodies: RigidBodySet, prediction: Fixed,
    ) -> Array<BroadPhaseProxy> {
        let margin = prediction * HALF;
        let mut proxies = array![];
        for (handle, collider) in self.colliders.to_array() {
            let is_static = match collider.parent {
                Some(parent) => match bodies.get(parent.handle) {
                    Some(body) => body.is_fixed(),
                    None => true,
                },
                None => true,
            };
            proxies
                .append(
                    BroadPhaseProxy {
                        collider: handle, aabb: collider.compute_aabb().loosened(margin), is_static,
                    },
                );
        }
        proxies
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_core::Handle;
    use rapier_core::collider::ColliderChangesTrait;
    use rapier_core::collider::changes::{PARENT, POSITION as CO_POSITION};
    use rapier_core::rigid_body::RigidBodyChangesTrait;
    use rapier_core::rigid_body::changes::{COLLIDERS, POSITION};
    use rapier_math::pose2::Pose2;
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::collider::{Collider, ColliderBuilderTrait, ColliderTrait};
    use crate::rigid_body_set::{RigidBody, RigidBodySetTrait, RigidBodyTrait};
    use super::ColliderSetTrait;

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn at(x: Fixed, y: Fixed) -> Pose2 {
        Pose2 { translation: v(x, y), rotation: Rot2 { re: ONE, im: ZERO } }
    }

    fn cuboid_at(x: Fixed, y: Fixed) -> Collider {
        ColliderBuilderTrait::cuboid(HALF, HALF).translation(v(x, y)).build()
    }

    #[test]
    fn test_insert_standalone_clears_parent() {
        let mut colliders = ColliderSetTrait::new();
        let mut bodies = RigidBodySetTrait::new();
        let b = bodies.insert(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
        let h = colliders.insert_with_parent(cuboid_at(ZERO, ZERO), b, ref bodies);
        let attached = colliders.get(h).unwrap();
        let h2 = colliders.insert(attached);
        assert_eq!(colliders.get(h2).unwrap().parent(), None);
        assert_eq!(colliders.len(), 2);
        assert!(!colliders.is_empty());
    }

    #[test]
    fn test_insert_with_parent_links_and_poses() {
        let mut colliders = ColliderSetTrait::new();
        let mut bodies = RigidBodySetTrait::new();
        let b = bodies.insert(RigidBodyTrait::dynamic(at(ONE, TWO)));
        // Local offsets (pos_wrt_parent) of the two colliders.
        let h1 = colliders.insert_with_parent(cuboid_at(ONE, ZERO), b, ref bodies);
        let h2 = colliders.insert_with_parent(cuboid_at(-ONE, ZERO), b, ref bodies);
        let body = bodies.get(b).unwrap();
        assert_eq!(body.colliders, array![h1, h2].span());
        assert!(body.changes.contains(COLLIDERS));
        // Two unit-area boxes of density 1 at x = ±1: mass 2, centre of mass at the body origin.
        assert_eq!(body.mprops.local_mprops.inv_mass, HALF);
        assert_eq!(body.mprops.world_com, v(ONE, TWO));
        let c1 = colliders.get(h1).unwrap();
        assert_eq!(c1.parent(), Some(b));
        assert_eq!(c1.position_wrt_parent(), Some(at(ONE, ZERO)));
        assert_eq!(c1.position(), at(TWO, TWO));
        assert_eq!(colliders.get(h2).unwrap().position(), at(ZERO, TWO));
    }

    #[test]
    #[should_panic(expected: 'RigidBodySet: body not found')]
    fn test_insert_with_missing_parent_panics() {
        let mut colliders = ColliderSetTrait::new();
        let mut bodies = RigidBodySetTrait::new();
        let _ = colliders
            .insert_with_parent(
                cuboid_at(ZERO, ZERO), Handle { index: 3, generation: 0 }, ref bodies,
            );
    }

    #[test]
    fn test_remove_swap_removes_from_parent() {
        let mut colliders = ColliderSetTrait::new();
        let mut bodies = RigidBodySetTrait::new();
        let b = bodies.insert(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
        let h0 = colliders.insert_with_parent(cuboid_at(ZERO, ZERO), b, ref bodies);
        let h1 = colliders.insert_with_parent(cuboid_at(ONE, ZERO), b, ref bodies);
        let h2 = colliders.insert_with_parent(cuboid_at(TWO, ZERO), b, ref bodies);
        let removed = colliders.remove(h0, ref bodies).unwrap();
        assert_eq!(removed.parent(), Some(b));
        // Upstream `swap_remove`: the last handle takes the removed slot.
        assert_eq!(bodies.get(b).unwrap().colliders, array![h2, h1].span());
        assert_eq!(colliders.remove(h0, ref bodies), None);
        assert!(!colliders.contains(h0));
        let _ = colliders.remove(h1, ref bodies).unwrap();
        assert_eq!(bodies.get(b).unwrap().colliders, array![h2].span());
        let _ = colliders.remove(h2, ref bodies).unwrap();
        assert_eq!(bodies.get(b).unwrap().colliders, array![].span());
        assert!(colliders.is_empty());
    }

    #[test]
    fn test_body_remove_detaches_or_removes_colliders() {
        // (remove_attached_colliders, colliders left)
        let cases: Array<(bool, u32)> = array![(true, 0), (false, 2)];
        for (remove_attached, left) in cases {
            let mut colliders = ColliderSetTrait::new();
            let mut bodies = RigidBodySetTrait::new();
            let b = bodies.insert(RigidBodyTrait::dynamic(at(ONE, ZERO)));
            let h1 = colliders.insert_with_parent(cuboid_at(ZERO, ZERO), b, ref bodies);
            let _ = colliders.insert_with_parent(cuboid_at(ONE, ZERO), b, ref bodies);
            let body = bodies.remove(b, ref colliders, remove_attached).unwrap();
            assert_eq!(body.colliders.len(), 2);
            assert_eq!(colliders.len(), left);
            assert!(bodies.is_empty());
            if !remove_attached {
                let c1 = colliders.get(h1).unwrap();
                assert_eq!(c1.parent(), None);
                assert!(c1.changes.contains(PARENT));
                assert_eq!(c1.position(), at(ONE, ZERO));
            }
            assert!(bodies.remove(b, ref colliders, remove_attached).is_none());
        }
    }

    #[test]
    fn test_propagate_positions() {
        let mut colliders = ColliderSetTrait::new();
        let mut bodies = RigidBodySetTrait::new();
        let moved = bodies.insert(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
        let still = bodies.insert(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
        let h1 = colliders.insert_with_parent(cuboid_at(ONE, ZERO), moved, ref bodies);
        let h2 = colliders.insert_with_parent(cuboid_at(ONE, ZERO), still, ref bodies);
        // Clear the flags a pipeline would clear, then move one body.
        let mut body = bodies.get(still).unwrap();
        body.changes = RigidBodyChangesTrait::empty();
        let _ = bodies.set(still, body);
        let mut body: RigidBody = bodies.get(moved).unwrap();
        body.changes = RigidBodyChangesTrait::empty();
        // Quarter turn, then translation by (0, 2): local (1, 0) goes to (0, 3).
        body
            .set_position(
                Pose2 { translation: v(ZERO, TWO), rotation: Rot2 { re: ZERO, im: ONE } },
            );
        assert!(body.changes.contains(POSITION));
        let _ = bodies.set(moved, body);
        let mut c2 = colliders.get(h2).unwrap();
        c2.changes = ColliderChangesTrait::empty();
        let _ = colliders.set(h2, c2);
        bodies.propagate_modified_body_positions_to_colliders(ref colliders);
        let c1 = colliders.get(h1).unwrap();
        assert_eq!(c1.position().translation, v(ZERO, FixedTrait::from_int(3)));
        assert!(c1.changes.contains(CO_POSITION));
        let c2 = colliders.get(h2).unwrap();
        assert_eq!(c2.position(), at(ONE, ZERO));
        assert!(c2.changes.is_empty());
    }

    #[test]
    fn test_broad_phase_proxies() {
        let mut colliders = ColliderSetTrait::new();
        let mut bodies = RigidBodySetTrait::new();
        let ground = bodies.insert(RigidBodyTrait::fixed(at(ZERO, ZERO)));
        let dynamic = bodies.insert(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
        let h0 = colliders.insert_with_parent(cuboid_at(ZERO, ZERO), ground, ref bodies);
        let h1 = colliders.insert_with_parent(cuboid_at(ZERO, ONE), dynamic, ref bodies);
        let h2 = colliders.insert(cuboid_at(TWO, ZERO));
        let proxies = colliders.broad_phase_proxies(ref bodies, HALF);
        assert_eq!(proxies.len(), 3);
        let expected = array![(h0, true), (h1, false), (h2, true)];
        let mut i = 0;
        for (handle, is_static) in expected {
            let p = *proxies.at(i);
            assert_eq!(p.collider, handle);
            assert_eq!(p.is_static, is_static);
            i += 1;
        }
        // Half-extent 0.5 loosened by 0.25.
        let quarter = FixedTrait::from_raw(HALF.raw / 2);
        assert_eq!((*proxies.at(0)).aabb.maxs, v(HALF + quarter, HALF + quarter));
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }

    #[test]
    fn gas_insert() {
        let mut colliders = ColliderSetTrait::new();
        let _ = colliders.insert(opaque(cuboid_at(ZERO, ZERO)));
    }

    #[test]
    fn gas_insert_with_parent() {
        let mut colliders = ColliderSetTrait::new();
        let mut bodies = RigidBodySetTrait::new();
        let b = bodies.insert(opaque(RigidBodyTrait::dynamic(at(ZERO, ZERO))));
        let _ = colliders.insert_with_parent(opaque(cuboid_at(ZERO, ZERO)), b, ref bodies);
    }

    #[test]
    fn gas_get_set() {
        let mut colliders = ColliderSetTrait::new();
        let h = colliders.insert(opaque(cuboid_at(ZERO, ZERO)));
        let c = colliders.get(opaque(h)).unwrap();
        let _ = colliders.set(h, c);
    }

    #[test]
    fn gas_remove_with_parent() {
        let mut colliders = ColliderSetTrait::new();
        let mut bodies = RigidBodySetTrait::new();
        let b = bodies.insert(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
        let h = colliders.insert_with_parent(opaque(cuboid_at(ZERO, ZERO)), b, ref bodies);
        let _ = colliders.remove(opaque(h), ref bodies);
    }

    #[test]
    fn gas_iter_8() {
        let mut colliders = ColliderSetTrait::new();
        let c = opaque(cuboid_at(ZERO, ZERO));
        let mut i: u32 = 0;
        while i != 8 {
            let _ = colliders.insert(c);
            i += 1;
        }
        let _ = colliders.iter();
    }

    #[test]
    fn gas_broad_phase_proxies_8() {
        let mut colliders = ColliderSetTrait::new();
        let mut bodies = RigidBodySetTrait::new();
        let b = bodies.insert(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
        let c = opaque(cuboid_at(ZERO, ZERO));
        let mut i: u32 = 0;
        while i != 8 {
            let _ = colliders.insert_with_parent(c, b, ref bodies);
            i += 1;
        }
        let _ = colliders.broad_phase_proxies(ref bodies, opaque(HALF));
    }
}
