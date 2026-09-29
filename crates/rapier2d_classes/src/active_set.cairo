//! The rebuild of the active set (`StageConfig::Active`) in a declared class (CS6, route (a)).
//!
//! `rapier2d::pipeline::active_set::rebuild` reads four fields of a body (its change flags, type,
//! enabled flag and sleep flag), four of a collider (its change flags, parent, shape and pose) and
//! the two colliders of a pair: only those cross ([`RebuildBody`], [`RebuildCollider`], the pairs'
//! handles). `ActiveSetClass` puts them back into placeholder bodies, colliders and pairs and runs
//! the same `rebuild`, so that the active set that comes back is the caller's.

use rapier2d::pipeline::active_set::ActiveSet;
use rapier2d::pipeline::stages::ActiveSetStage;
use rapier2d::prelude::{Fixed, Handle, Pose2, Shape};
use rapier2d::world::basic_state::decode::read_active_set;
use rapier2d::world::basic_state::{deserialize_basic_shape, serialize_basic_shape};
use rapier_core::collider::ColliderChanges;
use rapier_core::rigid_body::{RigidBodyChanges, RigidBodyType};
use rapier_dynamics2d::collider::{Collider, ColliderTrait};
use rapier_dynamics2d::narrow_phase::ContactPair;
use rapier_dynamics2d::rigid_body_set::RigidBody;
use starknet::SyscallResultTrait;
use starknet::syscalls::library_call_syscall;
use crate::hashes::{ClassHashes, errors};

/// What `rebuild` reads of a body.
#[derive(Copy, Drop, Serde)]
pub struct RebuildBody {
    pub handle: Handle,
    pub changes: RigidBodyChanges,
    pub body_type: RigidBodyType,
    pub enabled: bool,
    pub sleeping: bool,
}

/// What `rebuild` reads of a collider.
#[derive(Copy, Drop)]
pub struct RebuildCollider {
    pub handle: Handle,
    pub changes: ColliderChanges,
    pub parent: Option<Handle>,
    pub shape: Shape,
    pub pose: Pose2,
}

/// The crossing of a [`RebuildCollider`], the shape by the basic codec.
///
/// # Panics
/// `'State: not a basic shape'` on another shape.
pub impl RebuildColliderSerde of Serde<RebuildCollider> {
    fn serialize(self: @RebuildCollider, ref output: Array<felt252>) {
        self.handle.serialize(ref output);
        self.changes.serialize(ref output);
        self.parent.serialize(ref output);
        serialize_basic_shape(self.shape, ref output);
        self.pose.serialize(ref output);
    }

    fn deserialize(ref serialized: Span<felt252>) -> Option<RebuildCollider> {
        Some(
            RebuildCollider {
                handle: Serde::deserialize(ref serialized)?,
                changes: Serde::deserialize(ref serialized)?,
                parent: Serde::deserialize(ref serialized)?,
                shape: deserialize_basic_shape(ref serialized)?,
                pose: Serde::deserialize(ref serialized)?,
            },
        )
    }
}

/// The rebuild library-called in `ActiveSetClass` (at `H::active_set()`), once per step that
/// fills the active set. Same results as `InProcessActiveSet<BasicShapeKernels>`.
///
/// # Panics
/// `errors::DECODE` when the class returns something else than its result; as the class (a shape
/// that is not basic).
pub impl LibraryCallActiveSet<impl H: ClassHashes> of ActiveSetStage {
    fn rebuild(
        snapshot: Span<(Handle, Collider)>,
        entries: Span<(Handle, RigidBody)>,
        pairs: Span<ContactPair>,
        force_events: bool,
        prediction: Fixed,
    ) -> ActiveSet {
        let mut calldata = array![];
        calldata.append(entries.len().into());
        for (handle, body) in entries {
            RebuildBody {
                handle: *handle,
                changes: *body.changes,
                body_type: *body.body_type,
                enabled: *body.enabled,
                sleeping: *body.activation.sleeping,
            }
                .serialize(ref calldata);
        }
        calldata.append(snapshot.len().into());
        for (handle, collider) in snapshot {
            RebuildCollider {
                handle: *handle,
                changes: *collider.changes,
                parent: collider.parent(),
                shape: *collider.shape,
                pose: *collider.pos.pose,
            }
                .serialize(ref calldata);
        }
        calldata.append(pairs.len().into());
        for pair in pairs {
            pair.collider1.serialize(ref calldata);
            pair.collider2.serialize(ref calldata);
        }
        force_events.serialize(ref calldata);
        prediction.serialize(ref calldata);
        let mut ret = library_call_syscall(H::active_set(), selector!("rebuild"), calldata.span())
            .unwrap_syscall();
        read_active_set(ref ret).expect(errors::DECODE)
    }
}

/// The rebuild of the active set of the basic shapes (`active_set::rebuild`).
#[starknet::contract]
pub mod ActiveSetClass {
    use rapier2d::pipeline::active_set::ActiveSet;
    use rapier2d::pipeline::stages::BasicShapeKernels;
    use rapier2d::prelude::{Fixed, Handle, RigidBodyTrait};
    use rapier_dynamics2d::collider::components::ColliderParent;
    use rapier_dynamics2d::collider::{Collider, ColliderBuilderTrait};
    use rapier_dynamics2d::narrow_phase::{ContactPair, ContactPairTrait};
    use rapier_dynamics2d::rigid_body_set::RigidBody;
    use super::{RebuildBody, RebuildCollider, RebuildColliderSerde};

    #[storage]
    struct Storage {}

    /// `rebuild::<BasicShapeKernels>` on placeholders holding what crossed.
    #[external(v0)]
    fn rebuild(
        self: @ContractState,
        bodies: Span<RebuildBody>,
        colliders: Span<RebuildCollider>,
        pairs: Span<(Handle, Handle)>,
        force_events: bool,
        prediction: Fixed,
    ) -> ActiveSet {
        let base = RigidBodyTrait::dynamic(Default::default());
        let mut entries: Array<(Handle, RigidBody)> = array![];
        for body in bodies {
            let mut value = base;
            value.changes = *body.changes;
            value.body_type = *body.body_type;
            value.enabled = *body.enabled;
            value.activation.sleeping = *body.sleeping;
            entries.append((*body.handle, value));
        }
        let placeholder = ColliderBuilderTrait::ball(fixed::ONE).build();
        let mut snapshot: Array<(Handle, Collider)> = array![];
        for collider in colliders {
            let mut value = placeholder;
            value.changes = *collider.changes;
            value.parent = match *collider.parent {
                Some(handle) => Some(ColliderParent { handle, pos_wrt_parent: Default::default() }),
                None => None,
            };
            value.shape = *collider.shape;
            value.pos.pose = *collider.pose;
            snapshot.append((*collider.handle, value));
        }
        let mut list: Array<ContactPair> = array![];
        for (collider1, collider2) in pairs {
            list.append(ContactPairTrait::new(*collider1, *collider2));
        }
        rapier2d::pipeline::active_set::rebuild::<
            BasicShapeKernels,
        >(snapshot.span(), entries.span(), list.span(), force_events, prediction)
    }
}
