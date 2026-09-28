//! Multi-class layouts of the game's step (work package CS3, `docs/research/class-split.md`):
//! the step compiled with `BasicStepConfig` (the game's), and the same step with its contact
//! generation moved into another declared class, called through `library_call_syscall`.
//!
//! `StepConfig` already selects the contact dispatcher statically: [`LibraryCallDispatcher`] is a
//! `ContactDispatcher` whose `contact_manifold` serialises its arguments and library-calls
//! [`ContactClass`] (the game's `BasicShapesDispatcher`), so that the caller class compiles no
//! contact generator and no geometry kernel. The narrow phase calls it once per pair whose AABBs
//! overlap: a tick with no pair makes no call. Results are bit-identical to the in-process
//! dispatcher (`tests/split.cairo`).
//!
//! * `BasicGameStep`: `WorldState` in, `steps` × `step_with_force_events_with::<BasicStepConfig>`,
//!   `WorldState` out: the class the game's step compiles to, the reference of the cuts.
//! * `ContactClass`: the contact generators of the game's shapes behind one entry point
//!   (`contact_geometry`: a manifold's geometry in, its update out).
//! * `SplitNarrowStep`: `BasicGameStep` with [`SplitNarrowStepConfig`]: the caller class of the
//!   two-class layout. It reads the class hash of the generators from its storage slot
//!   [`CONTACT_CLASS_SLOT`] (under `library_call` the storage is the calling contract's).

use rapier2d::pipeline::config::{ContactDispatcher, NoComposites, NoJoints, NoSensors, StepConfig};
use rapier2d::prelude::{Fixed, Pose2, Shape, Vec2};
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldData, TrackedContact};
use starknet::syscalls::{library_call_syscall, storage_read_syscall};
use starknet::{ClassHash, SyscallResultTrait};

/// Storage slot of the contact class hash (`selector!("contact_class")`, the address of a
/// `contact_class` storage variable).
pub const CONTACT_CLASS_SLOT: felt252 = selector!("contact_class");

/// Errors of the split layout.
pub mod errors {
    /// A class returned felts that do not decode as its result.
    pub const DECODE: felt252 = 'Split: decode';
    /// The class-hash slot is empty or not a class hash.
    pub const CLASS: felt252 = 'Split: class hash';
}

/// The class hash stored at `slot` of the executing contract's storage.
pub fn class_at(slot: felt252) -> ClassHash {
    let address = slot.try_into().expect(errors::CLASS);
    let raw = storage_read_syscall(0, address).unwrap_syscall();
    raw.try_into().expect(errors::CLASS)
}

/// What a contact generator reads and writes of a manifold: [`ContactManifold`] without its
/// solver data (`data`, which only the narrow phase writes, after the generator), so that the
/// crossing carries about half the felts.
#[derive(Copy, Drop, Serde)]
pub struct ManifoldGeometry {
    pub points: [TrackedContact; 2],
    pub num_points: u8,
    pub local_n1: Vec2,
    pub local_n2: Vec2,
    pub subshape1: u32,
    pub subshape2: u32,
}

/// `manifold`'s geometry.
pub fn geometry(manifold: @ContactManifold) -> ManifoldGeometry {
    ManifoldGeometry {
        points: *manifold.points,
        num_points: *manifold.num_points,
        local_n1: *manifold.local_n1,
        local_n2: *manifold.local_n2,
        subshape1: *manifold.subshape1,
        subshape2: *manifold.subshape2,
    }
}

/// The manifold of `geometry` with the solver data `data`.
pub fn with_geometry(geometry: ManifoldGeometry, data: ContactManifoldData) -> ContactManifold {
    let ManifoldGeometry {
        points, num_points, local_n1, local_n2, subshape1, subshape2,
    } = geometry;
    ContactManifold { points, num_points, local_n1, local_n2, subshape1, subshape2, data }
}

/// The contact dispatcher of the caller class: every pair's manifold geometry computed by the
/// class at [`CONTACT_CLASS_SLOT`] (`ContactClass::contact_geometry`); the solver data stays in
/// the caller. Measured against the whole-manifold crossing (`alternatives`).
pub impl LibraryCallDispatcher of ContactDispatcher {
    fn contact_manifold(
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) -> bool {
        let mut calldata = array![];
        pos12.serialize(ref calldata);
        shape1.serialize(ref calldata);
        shape2.serialize(ref calldata);
        prediction.serialize(ref calldata);
        geometry(@manifold).serialize(ref calldata);
        let mut ret = library_call_syscall(
            class_at(CONTACT_CLASS_SLOT), selector!("contact_geometry"), calldata.span(),
        )
            .unwrap_syscall();
        let (supported, out): (bool, ManifoldGeometry) = Serde::deserialize(ref ret)
            .expect(errors::DECODE);
        manifold = with_geometry(out, manifold.data);
        supported
    }
}

/// `BasicStepConfig` with the contact generation in another class.
pub impl SplitNarrowStepConfig of StepConfig {
    impl Dispatcher = LibraryCallDispatcher;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
}

#[starknet::contract]
pub mod BasicGameStep {
    use rapier2d::prelude::{BasicStepConfig, WorldState, WorldTrait};

    #[storage]
    struct Storage {}

    /// `state` after `steps` calls of `step_with_force_events_with::<BasicStepConfig>` (the
    /// events are dropped).
    #[external(v0)]
    fn step_state(self: @ContractState, state: WorldState, steps: u32) -> WorldState {
        let mut world = WorldTrait::from_state(state);
        let mut i = 0;
        while i != steps {
            let _ = world.step_with_force_events_with::<BasicStepConfig>();
            i += 1;
        }
        world.into_state()
    }
}

#[starknet::contract]
pub mod ContactClass {
    use rapier2d::dispatcher::BasicShapesDispatcher;
    use rapier2d::prelude::{Fixed, Pose2, Shape};
    use super::{ManifoldGeometry, with_geometry};

    #[storage]
    struct Storage {}

    /// `BasicShapesDispatcher::contact_manifold` on the manifold of `geometry` (default solver
    /// data, which no generator reads): whether the pair is supported, and the updated geometry.
    #[external(v0)]
    fn contact_geometry(
        self: @ContractState,
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        geometry: ManifoldGeometry,
    ) -> (bool, ManifoldGeometry) {
        let mut manifold = with_geometry(geometry, Default::default());
        let supported = BasicShapesDispatcher::contact_manifold(
            pos12, shape1, shape2, prediction, ref manifold,
        );
        (supported, super::geometry(@manifold))
    }
}

#[starknet::contract]
pub mod SplitNarrowStep {
    use rapier2d::prelude::{WorldState, WorldTrait};
    use starknet::ClassHash;
    use starknet::storage::StoragePointerWriteAccess;
    use super::SplitNarrowStepConfig;

    #[storage]
    struct Storage {
        /// Read raw at `super::CONTACT_CLASS_SLOT` by the dispatcher.
        contact_class: ClassHash,
    }

    #[constructor]
    fn constructor(ref self: ContractState, contact_class: ClassHash) {
        self.contact_class.write(contact_class);
    }

    /// `BasicGameStep::step_state`, the contact manifolds computed by the contact class.
    #[external(v0)]
    fn step_state(self: @ContractState, state: WorldState, steps: u32) -> WorldState {
        let mut world = WorldTrait::from_state(state);
        let mut i = 0;
        while i != steps {
            let _ = world.step_with_force_events_with::<SplitNarrowStepConfig>();
            i += 1;
        }
        world.into_state()
    }
}

#[starknet::contract]
pub mod Echo {
    #[storage]
    struct Storage {}

    /// `data` back: the fixed cost of a `library_call` and its cost per crossing felt.
    #[external(v0)]
    fn echo(self: @ContractState, data: Span<felt252>) -> Span<felt252> {
        data
    }
}

/// Measured and rejected (`tests/split.cairo`, `steps_split_full_*`): the whole manifold, solver
/// data included, crosses both ways.
pub mod alternatives {
    use rapier2d::pipeline::config::{
        ContactDispatcher, NoComposites, NoJoints, NoSensors, StepConfig,
    };
    use rapier2d::prelude::{Fixed, Pose2, Shape};
    use rapier_geometry2d::contact::ContactManifold;
    use starknet::SyscallResultTrait;
    use starknet::syscalls::library_call_syscall;
    use super::{class_at, errors};

    /// Storage slot of `ContactClassFull`'s class hash.
    pub const CONTACT_FULL_CLASS_SLOT: felt252 = selector!("contact_full_class");

    /// `super::LibraryCallDispatcher` with the whole manifold crossing.
    pub impl FullManifoldDispatcher of ContactDispatcher {
        fn contact_manifold(
            pos12: Pose2,
            shape1: Shape,
            shape2: Shape,
            prediction: Fixed,
            ref manifold: ContactManifold,
        ) -> bool {
            let mut calldata = array![];
            pos12.serialize(ref calldata);
            shape1.serialize(ref calldata);
            shape2.serialize(ref calldata);
            prediction.serialize(ref calldata);
            manifold.serialize(ref calldata);
            let mut ret = library_call_syscall(
                class_at(CONTACT_FULL_CLASS_SLOT), selector!("contact_manifold"), calldata.span(),
            )
                .unwrap_syscall();
            let (supported, out): (bool, ContactManifold) = Serde::deserialize(ref ret)
                .expect(errors::DECODE);
            manifold = out;
            supported
        }
    }

    /// `super::SplitNarrowStepConfig` with [`FullManifoldDispatcher`].
    pub impl FullManifoldStepConfig of StepConfig {
        impl Dispatcher = FullManifoldDispatcher;
        impl Sensors = NoSensors;
        impl Composites = NoComposites;
        impl Joints = NoJoints;
    }

    #[starknet::contract]
    pub mod ContactClassFull {
        use rapier2d::dispatcher::BasicShapesDispatcher;
        use rapier2d::prelude::{Fixed, Pose2, Shape};
        use rapier_geometry2d::contact::ContactManifold;

        #[storage]
        struct Storage {}

        /// `BasicShapesDispatcher::contact_manifold`: whether the pair is supported, and the
        /// updated manifold.
        #[external(v0)]
        fn contact_manifold(
            self: @ContractState,
            pos12: Pose2,
            shape1: Shape,
            shape2: Shape,
            prediction: Fixed,
            manifold: ContactManifold,
        ) -> (bool, ContactManifold) {
            let mut manifold = manifold;
            let supported = BasicShapesDispatcher::contact_manifold(
                pos12, shape1, shape2, prediction, ref manifold,
            );
            (supported, manifold)
        }
    }
}
