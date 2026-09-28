//! The mass properties of the user changes (`StepConfig::Mass`) in a declared class (CS5): the
//! body and its colliders cross (every collider of the body the caller's set resolves), its mass
//! properties come back. Called once per body whose collider list or local mass changed (a
//! level's bodies at their first step), never on a step without user changes.

use rapier2d::pipeline::stages::MassStage;
use rapier2d::prelude::{Collider, ColliderSet, ColliderSetTrait, Handle, RigidBody};
use rapier_dynamics2d::rigid_body::mass_props::RigidBodyMassProps;
use starknet::SyscallResultTrait;
use starknet::syscalls::library_call_syscall;
use crate::hashes::{ClassHashes, errors};

/// `InProcessMass` library-called in `MassClass` (at `H::mass()`).
///
/// # Panics
/// `errors::DECODE` when the class returns something else than its result; as the stage.
pub impl LibraryCallMass<impl H: ClassHashes> of MassStage {
    fn recompute_mass_properties(ref body: RigidBody, ref colliders: ColliderSet) {
        let mut found = array![];
        for co_handle in body.colliders {
            if let Some(collider) = colliders.get(*co_handle) {
                found.append((*co_handle, collider));
            }
        }
        let mut calldata = array![];
        body.serialize(ref calldata);
        found.span().serialize(ref calldata);
        let mut ret = library_call_syscall(H::mass(), selector!("mass_properties"), calldata.span())
            .unwrap_syscall();
        body.mprops = Serde::deserialize(ref ret).expect(errors::DECODE);
    }
}

/// `recompute_mass_properties_from_colliders` on `body` and its `colliders` (the ones the
/// caller's set resolves): the body's new mass properties.
pub fn mass_properties_values(
    body: RigidBody, colliders: Span<(Handle, Collider)>,
) -> RigidBodyMassProps {
    let mut body = body;
    let mut set = ColliderSetTrait::from_state(
        crate::arena::partial_state(crate::arena::ascending(colliders)),
    );
    rapier2d::pipeline::recompute_mass_properties_from_colliders(ref body, ref set);
    body.mprops
}

/// The mass properties of a body from its colliders.
#[starknet::contract]
pub mod MassClass {
    use rapier2d::prelude::{Collider, Handle, RigidBody};
    use rapier_dynamics2d::rigid_body::mass_props::RigidBodyMassProps;

    #[storage]
    struct Storage {}

    /// [`super::mass_properties_values`].
    #[external(v0)]
    fn mass_properties(
        self: @ContractState, body: RigidBody, colliders: Span<(Handle, Collider)>,
    ) -> RigidBodyMassProps {
        super::mass_properties_values(body, colliders)
    }
}
