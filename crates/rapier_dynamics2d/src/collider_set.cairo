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
//! from the sets, `docs/PLAN.md` D7/D9: the pipeline reads the change flags instead), hence no
//! `take_modified` / `take_removed`; `remove` takes no island manager (no wake-up: the world
//! wakes the contact partners, `rapier2d::world`); colliders are values, so the `*_mut`
//! accessors are their copy-out forms (`get`, `iter`, `iter_enabled`, `get_unknown_gen`,
//! `get_pair_mut`) and a change is written back with `set`; `Index` / `IndexMut` are not
//! implemented (an arena read needs `ref self`, Cairo's `IndexView` takes a snapshot).

use fixed::{Fixed, HALF};
use rapier_core::Handle;
use rapier_core::collider::ColliderChangesTrait;
use rapier_core::collider::changes::PARENT;
use rapier_core::data::arena::{Arena, ArenaState, ArenaStateTrait, ArenaTrait};
use rapier_core::data::handle::INVALID_HANDLE;
use rapier_geometry2d::aabb::AabbTrait;
use rapier_geometry2d::broad_phase::BroadPhaseProxy;
use rapier_math::pose2::IDENTITY;
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

    /// An empty set. Cairo arenas reserve no memory, so `capacity` is intentionally ignored.
    #[inline(always)]
    fn with_capacity(_capacity: u32) -> ColliderSet {
        Self::new()
    }

    /// The handle that never resolves (upstream `invalid_handle`: both raw parts `u32::MAX`).
    #[inline(always)]
    fn invalid_handle() -> Handle {
        INVALID_HANDLE
    }

    /// Stores a standalone collider (its parent is cleared, as upstream) and returns its handle.
    /// Its world pose is its current `pos`.
    fn insert(ref self: ColliderSet, collider: Collider) -> Handle {
        let mut collider = collider;
        collider.parent = None;
        collider.changes = ColliderChangesTrait::all();
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
        collider.changes = ColliderChangesTrait::all();
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

    /// Whether a collider was inserted, written, removed or re-parented since the last
    /// [`clear_modified`](Self::clear_modified), or since the set was created. A set rebuilt by
    /// [`from_state`](Self::from_state) starts unmodified.
    #[inline(always)]
    fn is_modified(self: @ColliderSet) -> bool {
        self.colliders.is_modified()
    }

    /// Raises the [`is_modified`](Self::is_modified) flag (a caller that wrote through
    /// [`set_internal`](Self::set_internal)).
    #[inline(always)]
    fn mark_modified(ref self: ColliderSet) {
        self.colliders.mark_modified();
    }

    /// [`set`](Self::set) without raising the [`is_modified`](Self::is_modified) flag (upstream
    /// `get_mut_internal`): the step's own write-backs, which it tracks itself (BT4).
    #[inline(always)]
    fn set_internal(ref self: ColliderSet, handle: Handle, collider: Collider) -> bool {
        self.colliders.set_untracked(handle, collider)
    }

    /// Clears the flag [`is_modified`](Self::is_modified) reads (the step does).
    #[inline(always)]
    fn clear_modified(ref self: ColliderSet) {
        self.colliders.clear_modified();
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

    /// Every enabled `(handle, collider)` in ascending slot index (upstream `iter_enabled`).
    fn iter_enabled(ref self: ColliderSet) -> Array<(Handle, Collider)> {
        let mut out = array![];
        for (handle, collider) in self.colliders.to_array() {
            if collider.is_enabled() {
                out.append((handle, collider));
            }
        }
        out
    }

    /// The collider in slot `index` whatever its generation, with its live handle (upstream
    /// `get_unknown_gen`); `None` for an empty slot. O(len), ascending slot order.
    fn get_unknown_gen(ref self: ColliderSet, index: u32) -> Option<(Collider, Handle)> {
        for (handle, collider) in self.colliders.to_array() {
            if handle.index == index {
                return Some((collider, handle));
            }
        }
        None
    }

    /// Copies of the colliders behind two handles (upstream `get_pair_mut`); equal handles give
    /// `(first, None)`, as upstream. Write changes back with `set`.
    fn get_pair_mut(
        ref self: ColliderSet, handle1: Handle, handle2: Handle,
    ) -> (Option<Collider>, Option<Collider>) {
        if handle1 == handle2 {
            (self.get(handle1), None)
        } else {
            (self.get(handle1), self.get(handle2))
        }
    }

    /// Re-parents the collider behind `handle` (upstream `set_parent`); nothing when it does not
    /// resolve or when `new_parent` is its current parent. Otherwise `PARENT` is raised, the
    /// collider leaves its current parent's collider list (swap-remove, `COLLIDERS` raised, the
    /// mass recomputed by the next step), and with `Some(body)`:
    /// * the pose relative to the parent is kept if it had a parent, the identity otherwise (not
    ///   its world pose, unlike `insert_with_parent`);
    /// * when `body` exists, the collider is appended to its list, its mass added to the body's
    ///   and its world pose set to `body.position * pos_wrt_parent`; when it does not, only the
    ///   link is stored, as upstream.
    /// With `None` the collider becomes standalone and keeps its world pose.
    ///
    /// Cost: one collider read and write, plus the body reads and writes of the detach / attach.
    fn set_parent(
        ref self: ColliderSet, handle: Handle, new_parent: Option<Handle>, ref bodies: RigidBodySet,
    ) {
        if let Some(mut collider) = self.colliders.get(handle) {
            let current = collider.parent();
            if new_parent == current {
                return;
            }
            collider.changes = collider.changes | PARENT;
            if let Some(parent_handle) = current {
                detach_collider(ref bodies, parent_handle, handle);
            }
            match new_parent {
                Some(body) => {
                    let pos_wrt_parent = match collider.parent {
                        Some(parent) => parent.pos_wrt_parent,
                        None => IDENTITY,
                    };
                    collider.parent = Some(ColliderParent { handle: body, pos_wrt_parent });
                    if bodies.contains(body) {
                        let body_pose = attach_collider(
                            ref bodies, body, handle, collider, pos_wrt_parent,
                        );
                        collider.pos.pose = body_pose * pos_wrt_parent;
                    }
                },
                None => { collider.parent = None; },
            }
            let _ = self.colliders.set(handle, collider);
        }
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

    /// Flat image of the set (generation counter, capacity, free list, every `(handle, collider)`
    /// in ascending slot index), for save / restore. [`from_state`](Self::from_state) rebuilds a
    /// set that issues the same handles as this one for the same future calls, removals included.
    /// Cost: one dict read per allocated slot and per free slot.
    fn to_state(ref self: ColliderSet) -> ArenaState<Collider> {
        self.colliders.to_state()
    }

    /// Rebuilds a set from its [`to_state`](Self::to_state) image. Cost: one dict write per
    /// allocated slot.
    ///
    /// # Panics
    /// `Arena: state ...` (`rapier_core::data::arena::errors`) when `state` is not a valid image.
    fn from_state(state: ArenaState<Collider>) -> ColliderSet {
        ColliderSet { colliders: ArenaStateTrait::from_state(state) }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_core::Handle;
    use rapier_core::collider::ColliderChangesTrait;
    use rapier_core::collider::changes::{PARENT, POSITION as CO_POSITION};
    use rapier_core::data::handle::HandleTrait;
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

    /// BT2: every write raises the `modified` flag of the set it touches (both sets for an
    /// attach, a detach or a body removal); reads do not; `clear_modified` and `from_state` clear.
    #[test]
    fn test_modified_flags() {
        let mut colliders = ColliderSetTrait::new();
        let mut bodies = RigidBodySetTrait::new();
        assert!(!colliders.is_modified() && !bodies.is_modified());
        let b = bodies.insert(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
        assert!(bodies.is_modified() && !colliders.is_modified());
        bodies.clear_modified();
        let h = colliders.insert_with_parent(cuboid_at(ONE, ZERO), b, ref bodies);
        assert!(bodies.is_modified() && colliders.is_modified());
        // (step, collider write?, body write?): 0 reads, 1 set, 2 body set, 3 collider remove,
        // 4 body remove, 5 standalone insert, 6 set_parent.
        let s = colliders.insert(cuboid_at(TWO, ZERO));
        let mut step: u8 = 0;
        while step != 7 {
            bodies.clear_modified();
            colliders.clear_modified();
            let (co, body) = if step == 0 {
                let _ = colliders.get(h);
                let _ = bodies.get(b);
                let _ = colliders.iter();
                let _ = bodies.iter();
                (false, false)
            } else if step == 1 {
                let _ = colliders.set(h, colliders.get(h).unwrap());
                (true, false)
            } else if step == 2 {
                let _ = bodies.set(b, bodies.get(b).unwrap());
                (false, true)
            } else if step == 3 {
                let _ = colliders.remove(h, ref bodies);
                (true, true)
            } else if step == 4 {
                let _ = bodies.remove(b, ref colliders, true);
                (false, true)
            } else if step == 5 {
                let _ = colliders.insert(cuboid_at(ZERO, ONE));
                (true, false)
            } else {
                let b2 = bodies.insert(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
                bodies.clear_modified();
                colliders.set_parent(s, Some(b2), ref bodies);
                (true, true)
            };
            assert_eq!(
                (colliders.is_modified(), bodies.is_modified()), (co, body), "step {}", step,
            );
            step += 1;
        }
        let restored = ColliderSetTrait::from_state(colliders.to_state());
        assert!(!restored.is_modified());
        let restored = RigidBodySetTrait::from_state(bodies.to_state());
        assert!(!restored.is_modified());
    }

    /// `with_capacity`, `invalid_handle`, `iter_enabled`, `get_unknown_gen`, `get_pair_mut`.
    #[test]
    fn test_set_and_handle_helpers() {
        let mut colliders = ColliderSetTrait::with_capacity(16);
        let mut bodies = RigidBodySetTrait::new();
        assert!(colliders.is_empty());
        let invalid = ColliderSetTrait::invalid_handle();
        assert_eq!(invalid.into_raw_parts(), (0xffffffff, 0xffffffff));
        assert!(!colliders.contains(invalid));
        let h0 = colliders.insert(cuboid_at(ZERO, ZERO));
        let h1 = colliders.insert(cuboid_at(ONE, ZERO));
        let h2 = colliders.insert(cuboid_at(TWO, ZERO));
        let mut off = colliders.get(h1).unwrap();
        off.set_enabled(false);
        let _ = colliders.set(h1, off);
        let enabled = colliders.iter_enabled();
        assert_eq!(enabled.len(), 2);
        let (first, _) = *enabled.at(0);
        let (last, _) = *enabled.at(1);
        assert_eq!((first, last), (h0, h2));
        // Slot 0 reused with a new generation: found by index, with its live handle.
        let _ = colliders.remove(h0, ref bodies);
        let h3 = colliders.insert(cuboid_at(ZERO, ONE));
        assert_eq!(h3, Handle { index: 0, generation: 1 });
        // (index, expected handle)
        let cases = array![(0, Some(h3)), (1, Some(h1)), (2, Some(h2)), (3, None)];
        for (index, expected) in cases {
            match colliders.get_unknown_gen(index) {
                Some((
                    collider, handle,
                )) => {
                    assert_eq!(Some(handle), expected);
                    assert_eq!(Some(collider), colliders.get(handle));
                },
                None => assert!(expected.is_none()),
            }
        }
        let (a, b) = colliders.get_pair_mut(h1, h2);
        assert_eq!((a, b), (colliders.get(h1), colliders.get(h2)));
        let (a, b) = colliders.get_pair_mut(h2, h2);
        assert_eq!((a, b), (colliders.get(h2), None));
        let (a, b) = colliders.get_pair_mut(h0, h2);
        assert!(a.is_none() && b.is_some());
    }

    /// Re-parenting (upstream `set_parent`): (from, to) over standalone, body `a`, body `b` and
    /// a missing body; the lists, the pose relative to the parent, the world pose and the
    /// masses follow upstream.
    #[test]
    fn test_set_parent() {
        let three = FixedTrait::from_int(3);
        // (initial parent: 0 none / 1 a / 2 b, new parent: same codes, 3 = missing body)
        let cases: Array<(u8, u8)> = array![(0, 1), (1, 2), (1, 0), (1, 1), (0, 0), (2, 3)];
        for (from, to) in cases {
            let mut colliders = ColliderSetTrait::new();
            let mut bodies = RigidBodySetTrait::new();
            let a = bodies.insert(RigidBodyTrait::dynamic(at(ONE, ZERO)));
            let b = bodies.insert(RigidBodyTrait::dynamic(at(ZERO, three)));
            let body_of = array![None, Some(a), Some(b), Some(Handle { index: 9, generation: 0 })];
            // A second collider on `a` checks the swap-remove.
            let other = colliders.insert_with_parent(cuboid_at(ZERO, ZERO), a, ref bodies);
            let h = match *body_of.at(from.into()) {
                Some(parent) => colliders
                    .insert_with_parent(cuboid_at(ZERO, ONE), parent, ref bodies),
                None => colliders.insert(cuboid_at(ZERO, ONE)),
            };
            let mut quiet = colliders.get(h).unwrap();
            quiet.changes = ColliderChangesTrait::empty();
            let _ = colliders.set(h, quiet);
            let before = colliders.get(h).unwrap();
            let target = *body_of.at(to.into());
            colliders.set_parent(h, target, ref bodies);
            let after = colliders.get(h).unwrap();
            if from == to {
                assert_eq!(after, before);
                continue;
            }
            assert!(after.changes.contains(PARENT));
            assert_eq!(after.parent(), target);
            if from == 1 {
                assert_eq!(bodies.get(a).unwrap().colliders, array![other].span());
            }
            match target {
                Some(body) => {
                    // A former parent's offset is kept, a standalone collider gets the identity.
                    let offset = if from == 0 {
                        at(ZERO, ZERO)
                    } else {
                        at(ZERO, ONE)
                    };
                    assert_eq!(after.position_wrt_parent(), Some(offset));
                    if let Some(parent) = bodies.get(body) {
                        assert_eq!(*parent.colliders.at(parent.colliders.len() - 1), h);
                        assert_eq!(after.position(), parent.position() * offset);
                    } else {
                        assert_eq!(after.position(), before.position());
                    }
                },
                None => { assert_eq!(after.position(), before.position()); },
            }
        }
        // The new parent's mass grows by the collider's (two unit boxes on `b`).
        let mut colliders = ColliderSetTrait::new();
        let mut bodies = RigidBodySetTrait::new();
        let b = bodies.insert(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
        let _ = colliders.insert_with_parent(cuboid_at(ZERO, ZERO), b, ref bodies);
        let h = colliders.insert(cuboid_at(ZERO, ZERO));
        colliders.set_parent(h, Some(b), ref bodies);
        assert_eq!(bodies.get(b).unwrap().mprops.local_mprops.inv_mass, HALF);
        // A stale handle does nothing.
        colliders.set_parent(Handle { index: 7, generation: 0 }, Some(b), ref bodies);
        assert_eq!(bodies.get(b).unwrap().colliders.len(), 2);
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

    /// `gas_set_parent_*` − `gas_set_parent_setup`: one body `a` holding the collider, one
    /// body `b`.
    fn set_parent_setup() -> (
        super::ColliderSet, crate::rigid_body_set::RigidBodySet, Handle, Handle,
    ) {
        let mut colliders = ColliderSetTrait::new();
        let mut bodies = RigidBodySetTrait::new();
        let a = bodies.insert(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
        let b = bodies.insert(RigidBodyTrait::dynamic(at(ONE, ZERO)));
        let h = colliders.insert_with_parent(opaque(cuboid_at(ZERO, ZERO)), a, ref bodies);
        (colliders, bodies, h, b)
    }

    #[test]
    fn gas_set_parent_setup() {
        let _ = set_parent_setup();
    }

    #[test]
    fn gas_set_parent_move() {
        let (mut colliders, mut bodies, h, b) = set_parent_setup();
        colliders.set_parent(opaque(h), Some(b), ref bodies);
    }

    #[test]
    fn gas_set_parent_detach() {
        let (mut colliders, mut bodies, h, _) = set_parent_setup();
        colliders.set_parent(opaque(h), None, ref bodies);
    }

    #[test]
    fn gas_iter_enabled_8() {
        let mut colliders = ColliderSetTrait::new();
        let c = opaque(cuboid_at(ZERO, ZERO));
        let mut i: u32 = 0;
        while i != 8 {
            let _ = colliders.insert(c);
            i += 1;
        }
        let _ = colliders.iter_enabled();
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

    /// Save / restore of eight standalone colliders, one of them removed: `gas_to_state` −
    /// `gas_state_setup`, `gas_from_state` − `gas_to_state`.
    fn state_setup() -> super::ColliderSet {
        let mut colliders = ColliderSetTrait::new();
        let mut bodies = RigidBodySetTrait::new();
        let collider = opaque(cuboid_at(ZERO, ZERO));
        let mut i: u32 = 0;
        while i != 8 {
            let _ = colliders.insert(collider);
            i += 1;
        }
        let _ = colliders.remove(Handle { index: 3, generation: 0 }, ref bodies);
        colliders
    }

    #[test]
    fn gas_state_setup() {
        let _ = state_setup();
    }

    #[test]
    fn gas_to_state() {
        let mut colliders = state_setup();
        let _ = colliders.to_state();
    }

    #[test]
    fn gas_from_state() {
        let mut colliders = state_setup();
        let mut restored = ColliderSetTrait::from_state(colliders.to_state());
        assert_eq!(restored.insert(cuboid_at(ONE, ZERO)), Handle { index: 3, generation: 1 });
    }
}
