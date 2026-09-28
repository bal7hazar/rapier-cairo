//! The broad phase (`StepConfig::Broad`) in a declared class (CS5): the proxies cross, the
//! candidate pairs come back. The active-set step sends its dynamic proxies and the static ones
//! near them (`near_statics` stays in the caller: the pairs index into the list it keeps).

use rapier2d::pipeline::stages::BroadPhaseStage;
use rapier_geometry2d::broad_phase::BroadPhaseProxy;
use starknet::SyscallResultTrait;
use starknet::syscalls::library_call_syscall;
use crate::hashes::{ClassHashes, errors};

fn decode(mut ret: Span<felt252>) -> Array<(u32, u32)> {
    Serde::deserialize(ref ret).expect(errors::DECODE)
}

/// `InProcessBroadPhase` library-called in `BroadPhaseClass` (at `H::broad_phase()`), one call
/// per step.
///
/// # Panics
/// `errors::DECODE` when the class returns something else than its result.
pub impl LibraryCallBroadPhase<impl H: ClassHashes> of BroadPhaseStage {
    fn find_pairs(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
        let mut calldata = array![];
        proxies.serialize(ref calldata);
        decode(
            library_call_syscall(H::broad_phase(), selector!("find_pairs"), calldata.span())
                .unwrap_syscall(),
        )
    }

    fn find_pairs_sparse(
        statics: Span<BroadPhaseProxy>, dynamic: Span<BroadPhaseProxy>,
    ) -> Array<(u32, u32)> {
        let mut calldata = array![];
        statics.serialize(ref calldata);
        dynamic.serialize(ref calldata);
        decode(
            library_call_syscall(H::broad_phase(), selector!("find_pairs_sparse"), calldata.span())
                .unwrap_syscall(),
        )
    }
}

/// The broad phase (`rapier_geometry2d::broad_phase`).
#[starknet::contract]
pub mod BroadPhaseClass {
    use rapier_geometry2d::broad_phase::BroadPhaseProxy;

    #[storage]
    struct Storage {}

    /// `broad_phase::find_pairs`.
    #[external(v0)]
    fn find_pairs(self: @ContractState, proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
        rapier_geometry2d::broad_phase::find_pairs(proxies)
    }

    /// `broad_phase::find_pairs_sparse`.
    #[external(v0)]
    fn find_pairs_sparse(
        self: @ContractState, statics: Span<BroadPhaseProxy>, dynamic: Span<BroadPhaseProxy>,
    ) -> Array<(u32, u32)> {
        rapier_geometry2d::broad_phase::find_pairs_sparse(statics, dynamic)
    }
}
