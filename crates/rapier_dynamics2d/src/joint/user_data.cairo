//! User data of joints (upstream `GenericJoint::user_data`, set by
//! `GenericJointBuilder::user_data`), PX9.
//!
//! Upstream stores the `u128` in the stepped `GenericJoint`. A field there rides along every copy
//! of the joint, so the data lives beside the joint set instead: the builder carries it, and
//! [`JointUserData`] keeps it by joint handle. Nothing the step reads changes.
use core::dict::{Felt252Dict, Felt252DictTrait};
use rapier_core::data::handle::Handle;
use super::{GenericJointBuilder, ImpulseJointSet, ImpulseJointSetTrait};

/// The user data of joints by handle; a joint without an entry has `0`, upstream's default.
/// Holds a dict: pass it by `ref`. Entries of removed joints are the caller's to
/// [`remove`](JointUserDataTrait::remove); a handle is never reused (its generation changes).
#[derive(Destruct, Default)]
pub struct JointUserData {
    data: Felt252Dict<u128>,
}

#[generate_trait]
pub impl JointUserDataImpl of JointUserDataTrait {
    /// An empty table: every joint reads `0`.
    fn new() -> JointUserData {
        Default::default()
    }

    /// Inserts the joint of `builder` and keeps the user data set on the builder under its
    /// handle (upstream: `joints.insert(body1, body2, builder)` with `user_data` on the joint).
    fn insert(
        ref self: JointUserData,
        ref joints: ImpulseJointSet,
        body1: Handle,
        body2: Handle,
        builder: GenericJointBuilder,
    ) -> Handle {
        let user_data = builder.user_data;
        let handle = joints.insert(body1, body2, builder.data);
        if user_data != 0 {
            self.set(handle, user_data);
        }
        handle
    }

    /// The user data of the joint behind `handle` (upstream `joint.user_data`), `0` when none was
    /// set.
    fn get(ref self: JointUserData, handle: Handle) -> u128 {
        self.data.get(key(handle))
    }

    /// Sets the user data of the joint behind `handle` (upstream `joint.user_data = data`).
    fn set(ref self: JointUserData, handle: Handle, data: u128) {
        self.data.insert(key(handle), data);
    }

    /// Forgets the user data of `handle`, returning it (`0` when none was set).
    fn remove(ref self: JointUserData, handle: Handle) -> u128 {
        let data = self.get(handle);
        self.data.insert(key(handle), 0);
        data
    }
}

/// The dict key of a handle: its generation above its index, injective on both `u32`s.
fn key(handle: Handle) -> felt252 {
    handle.index.into() + handle.generation.into() * 0x100000000
}

#[cfg(test)]
mod tests {
    use rapier_core::data::handle::Handle;
    use super::JointUserData;
    use super::super::{
        GenericJointBuilderTrait, ImpulseJointSet, ImpulseJointSetTrait, JointAxesMask,
        JointUserDataTrait,
    };

    /// The builder's user data reaches the table under the joint's handle; `0` by default, and
    /// the joint itself is the one built without user data.
    #[test]
    fn test_user_data_by_handle() {
        let mut joints: ImpulseJointSet = ImpulseJointSetTrait::new();
        let mut table: JointUserData = JointUserDataTrait::new();
        let locks = JointAxesMask { bits: 3 };
        let (b1, b2) = (Handle { index: 0, generation: 0 }, Handle { index: 1, generation: 0 });
        let plain = table.insert(ref joints, b1, b2, GenericJointBuilderTrait::new(locks));
        let big = 0xffffffffffffffffffffffffffffffff;
        let tagged = table
            .insert(ref joints, b1, b2, GenericJointBuilderTrait::new(locks).user_data(big));
        assert_eq!(table.get(plain), 0);
        assert_eq!(table.get(tagged), big);
        assert_eq!(joints.get(tagged).unwrap().data, joints.get(plain).unwrap().data);
        table.set(plain, 7);
        assert_eq!(table.get(plain), 7);
        assert_eq!(table.remove(tagged), big);
        assert_eq!(table.get(tagged), 0);
    }

    /// The same slot with another generation is another joint.
    #[test]
    fn test_user_data_generation() {
        let mut table: JointUserData = JointUserDataTrait::new();
        let old = Handle { index: 3, generation: 1 };
        let new = Handle { index: 3, generation: 2 };
        table.set(old, 5);
        assert_eq!(table.get(old), 5);
        assert_eq!(table.get(new), 0);
    }

    /// The builder reads back what was set, `0` by default.
    #[test]
    fn test_builder_user_data_value() {
        let locks = JointAxesMask { bits: 3 };
        assert_eq!(GenericJointBuilderTrait::new(locks).user_data_value(), 0);
        assert_eq!(GenericJointBuilderTrait::new(locks).user_data(9).user_data_value(), 9);
    }
}
