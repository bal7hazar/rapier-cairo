//! Generational storage with ascending-slot iteration, never dictionary order.
use fixed::{Fixed, ZERO};
use rapier_core::data::arena::{Arena, ArenaState, ArenaStateTrait, ArenaTrait};
use rapier_core::data::handle::Handle;
use super::{GenericJoint, GenericJointTrait};
/// Joint plus signed impulses indexed by LinX, LinY, AngX.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ImpulseJoint {
    pub body1: Handle,
    pub body2: Handle,
    pub data: GenericJoint,
    pub impulses: [Fixed; 3],
}
#[generate_trait]
pub impl ImpulseJointImpl of ImpulseJointTrait {
    /// The first attached body (upstream `body1`).
    #[inline(always)]
    fn body1(self: ImpulseJoint) -> Handle {
        self.body1
    }
    /// The second attached body (upstream `body2`).
    #[inline(always)]
    fn body2(self: ImpulseJoint) -> Handle {
        self.body2
    }
}
/// Arena-backed joints. Removal invalidates old handles even when slots are reused.
#[derive(Destruct, Default)]
pub struct ImpulseJointSet {
    joints: Arena<ImpulseJoint>,
}
#[generate_trait]
pub impl ImpulseJointSetImpl of ImpulseJointSetTrait {
    /// Empty set, exact; no allocation until insert.
    fn new() -> ImpulseJointSet {
        Default::default()
    }
    /// Insert with zero impulses; panics only on arena capacity overflow. Body validity is checked
    /// by generation.
    fn insert(
        ref self: ImpulseJointSet, body1: Handle, body2: Handle, data: GenericJoint,
    ) -> Handle {
        self.joints.insert(ImpulseJoint { body1, body2, data, impulses: [ZERO, ZERO, ZERO] })
    }
    /// Copy joint or None for absent/stale handle; exact. Also stands for upstream's `get_mut`
    /// (joints are values: write the change back with [`set`](Self::set)).
    fn get(ref self: ImpulseJointSet, handle: Handle) -> Option<ImpulseJoint> {
        self.joints.get(handle)
    }
    /// Whether `handle` is a live joint (upstream `contains`); false for an absent or stale handle.
    #[inline(always)]
    fn contains(ref self: ImpulseJointSet, handle: Handle) -> bool {
        self.joints.contains(handle)
    }
    /// Whether the set holds no joint (upstream `is_empty`).
    #[inline(always)]
    fn is_empty(self: @ImpulseJointSet) -> bool {
        self.joints.is_empty()
    }
    /// Every `(handle, joint)` in ascending slot index (upstream `iter`, and `iter_mut`: write a
    /// change back with [`set`](Self::set)). Upstream walks its joint graph in insertion order
    /// (swap-removal reorders it): the order here is the arena's, never dictionary order.
    #[inline(always)]
    fn iter(ref self: ImpulseJointSet) -> Array<(Handle, ImpulseJoint)> {
        self.joints.to_array()
    }
    /// The joint in slot `index` whatever its generation, with its live handle (upstream
    /// `get_unknown_gen`, and `get_unknown_gen_mut`); `None` for an empty slot. O(len).
    fn get_unknown_gen(ref self: ImpulseJointSet, index: u32) -> Option<(ImpulseJoint, Handle)> {
        for (handle, joint) in self.joints.to_array() {
            if handle.index == index {
                return Some((joint, handle));
            }
        }
        None
    }
    /// The joints attached to `body` as `(body1, body2, joint handle, joint)` (upstream
    /// `attached_joints`), in ascending slot index. O(len).
    fn attached_joints(
        ref self: ImpulseJointSet, body: Handle,
    ) -> Array<(Handle, Handle, Handle, ImpulseJoint)> {
        let mut out = array![];
        for (handle, joint) in self.joints.to_array() {
            if joint.body1 == body || joint.body2 == body {
                out.append((joint.body1, joint.body2, handle, joint));
            }
        }
        out
    }
    /// The enabled joints attached to `body` (upstream `attached_enabled_joints`), as
    /// [`attached_joints`](Self::attached_joints).
    fn attached_enabled_joints(
        ref self: ImpulseJointSet, body: Handle,
    ) -> Array<(Handle, Handle, Handle, ImpulseJoint)> {
        let mut out = array![];
        for (handle, joint) in self.joints.to_array() {
            if (joint.body1 == body || joint.body2 == body) && joint.data.is_enabled() {
                out.append((joint.body1, joint.body2, handle, joint));
            }
        }
        out
    }
    /// The joints between `body1` and `body2`, in either order, as `(handle, joint)` (upstream
    /// `joints_between`), in ascending slot index. O(len).
    fn joints_between(
        ref self: ImpulseJointSet, body1: Handle, body2: Handle,
    ) -> Array<(Handle, ImpulseJoint)> {
        let mut out = array![];
        for (handle, joint) in self.joints.to_array() {
            if (joint.body1 == body1 && joint.body2 == body2)
                || (joint.body1 == body2 && joint.body2 == body1) {
                out.append((handle, joint));
            }
        }
        out
    }
    /// Removes every joint attached to `body` and returns their handles, in ascending slot index
    /// (upstream `remove_joints_attached_to_rigid_body`). No body is woken up: the set does not
    /// own the bodies (`World::remove_body` does it). O(len).
    fn remove_joints_attached_to_rigid_body(
        ref self: ImpulseJointSet, body: Handle,
    ) -> Array<Handle> {
        let mut removed = array![];
        for (handle, joint) in self.joints.to_array() {
            if joint.body1 == body || joint.body2 == body {
                let _ = self.joints.remove(handle);
                removed.append(handle);
            }
        }
        removed
    }
    /// Attaches the joint behind `handle` to two other bodies (upstream `set_bodies`); returns the
    /// updated joint, `None` for an absent or stale handle. The handle stays valid. No body is
    /// woken up (`World::set_impulse_joint_bodies` does).
    fn set_bodies(
        ref self: ImpulseJointSet, handle: Handle, body1: Handle, body2: Handle,
    ) -> Option<ImpulseJoint> {
        let joint = ImpulseJoint { body1, body2, ..self.joints.get(handle)? };
        let _ = self.joints.set(handle, joint);
        Some(joint)
    }
    /// Replace joint, returning false for absent/stale handles; exact.
    fn set(ref self: ImpulseJointSet, handle: Handle, joint: ImpulseJoint) -> bool {
        self.joints.set(handle, joint)
    }
    /// Remove joint, returning None for absent/stale handles. Arena generation overflow panics.
    fn remove(ref self: ImpulseJointSet, handle: Handle) -> Option<ImpulseJoint> {
        self.joints.remove(handle)
    }
    /// Live count, exact.
    fn len(self: @ImpulseJointSet) -> u32 {
        self.joints.len()
    }
    /// Copy all live entries in ascending slot order; exact, O(capacity).
    fn to_array(ref self: ImpulseJointSet) -> Array<(Handle, ImpulseJoint)> {
        self.joints.to_array()
    }

    /// Flat image of the set (generation counter, capacity, free list, every `(handle, joint)` in
    /// ascending slot index), for save / restore. [`from_state`](Self::from_state) rebuilds a set
    /// that issues the same handles as this one for the same future calls, removals included.
    /// Cost: one dict read per allocated slot and per free slot.
    fn to_state(ref self: ImpulseJointSet) -> ArenaState<ImpulseJoint> {
        self.joints.to_state()
    }

    /// Rebuilds a set from its [`to_state`](Self::to_state) image. Cost: one dict write per
    /// allocated slot.
    ///
    /// # Panics
    /// `Arena: state ...` (`rapier_core::data::arena::errors`) when `state` is not a valid image.
    fn from_state(state: ArenaState<ImpulseJoint>) -> ImpulseJointSet {
        ImpulseJointSet { joints: ArenaStateTrait::from_state(state) }
    }
}
#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use super::*;
    #[test]
    fn test_reuse_and_order() {
        let mut s = ImpulseJointSetTrait::new();
        let b = Handle { index: 0, generation: 0 };
        let a = s.insert(b, b, Default::default());
        let c = s.insert(b, b, Default::default());
        let j = s.remove(a).unwrap();
        assert!(s.get(a).is_none());
        let d = s.insert(b, b, j.data);
        assert_eq!(d.index, a.index);
        assert!(d.generation != a.generation);
        assert!(!s.set(a, j));
        assert!(s.set(d, j));
        assert_eq!(s.len(), 2);
        let entries = s.to_array();
        let (h0, _) = *entries.at(0);
        let (h1, _) = *entries.at(1);
        assert_eq!(h0, d);
        assert_eq!(h1, c);
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque(ZERO);
    }
    #[test]
    fn gas_new() {
        let s = ImpulseJointSetTrait::new();
        let _ = opaque(s.len());
    }
    fn probe(op: u8) {
        let mut s = ImpulseJointSetTrait::new();
        let b = opaque(Handle { index: 0, generation: 0 });
        let h = s.insert(b, b, opaque(Default::default()));
        match op {
            0 => { let _ = s.get(h); },
            1 => {
                let j = s.get(h).unwrap();
                let _ = s.set(h, opaque(j));
            },
            2 => { let _ = s.remove(h); },
            3 => { let _ = s.to_array(); },
            _ => { let _ = opaque(s.len()); },
        }
    }
    /// Save / restore of eight joints, one of them removed: `gas_to_state` − `gas_state_setup`,
    /// `gas_from_state` − `gas_to_state`.
    fn state_setup() -> ImpulseJointSet {
        let mut s = ImpulseJointSetTrait::new();
        let b = opaque(Handle { index: 0, generation: 0 });
        let mut i: u32 = 0;
        while i != 8 {
            let _ = s.insert(b, b, Default::default());
            i += 1;
        }
        let _ = s.remove(Handle { index: 3, generation: 0 });
        s
    }
    #[test]
    fn gas_state_setup() {
        let _ = state_setup();
    }
    #[test]
    fn gas_to_state() {
        let mut s = state_setup();
        let _ = s.to_state();
    }
    #[test]
    fn gas_from_state() {
        let mut s = state_setup();
        let state = s.to_state();
        let mut restored = ImpulseJointSetTrait::from_state(state);
        assert_eq!(restored.len(), 7);
        let issued = restored
            .insert(
                Handle { index: 0, generation: 0 },
                Handle { index: 0, generation: 0 },
                Default::default(),
            );
        assert_eq!(issued, Handle { index: 3, generation: 1 });
    }
    #[test]
    fn gas_insert_len() {
        probe(4);
    }
    #[test]
    fn gas_get() {
        probe(0);
    }
    #[test]
    fn gas_set() {
        probe(1);
    }
    #[test]
    fn gas_remove() {
        probe(2);
    }
    #[test]
    fn gas_to_array() {
        probe(3);
    }
}
