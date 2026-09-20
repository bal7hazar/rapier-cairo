//! Generational storage with ascending-slot iteration, never dictionary order.
use fixed::{Fixed, ZERO};
use rapier_core::data::arena::{Arena, ArenaTrait};
use rapier_core::data::handle::Handle;
use super::GenericJoint;
/// Joint plus signed impulses indexed by LinX, LinY, AngX.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ImpulseJoint {
    pub body1: Handle,
    pub body2: Handle,
    pub data: GenericJoint,
    pub impulses: [Fixed; 3],
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
    /// Copy joint or None for absent/stale handle; exact.
    fn get(ref self: ImpulseJointSet, handle: Handle) -> Option<ImpulseJoint> {
        self.joints.get(handle)
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
