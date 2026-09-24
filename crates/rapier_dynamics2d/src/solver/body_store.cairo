//! Dense, deterministic per-step storage. The dictionary is never iterated: dense indices
//! follow `RigidBodySet::iter`. Frozen DC/DE rows gather two bodies and scatter once per row set.
use core::dict::{Felt252Dict, Felt252DictTrait};
use core::nullable::{FromNullableResult, NullableTrait, match_nullable};
use fixed::{Fixed, ZERO};
use glam::Vec2;
use rapier_core::data::handle::Handle;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_core::rigid_body::{RigidBodyDamping, RigidBodyType};
use rapier_math::pose2::Pose2;
use crate::rigid_body::{RigidBodyForcesTrait, RigidBodyMassPropsTrait, RigidBodyVelocity};
use crate::rigid_body_set::{RigidBody, RigidBodySet, RigidBodySetTrait};
use super::body::{SolverBody, WORLD};

/// Invalid dense indices or mutation of the persistent body set during a solve.
pub mod errors {
    /// Dense index is out of bounds.
    pub const INDEX: felt252 = 'SolverStore: invalid index';
    /// A body was removed or replaced while solver scratch was alive.
    pub const BODY: felt252 = 'SolverStore: missing body';
}

/// Common interface of dictionary and rebuilding-array candidates. No arithmetic/rounding.
/// `WORLD` reads as an identity, immovable body and ignores writes; other invalid ids panic.
pub trait DenseBodiesTrait<T> {
    /// Copy a dense span in its exact order.
    fn new(bodies: Span<SolverBody>) -> T;
    /// Number of allocated dense indices, excluding the world sentinel.
    fn len(self: @T) -> u32;
    /// Gather a body; panics with `SolverStore: invalid index` for a non-world invalid id.
    fn get(ref self: T, index: u32) -> SolverBody;
    /// Scatter two bodies, retaining every other entry. If ids coincide the second wins.
    /// Invalid non-world ids panic with `SolverStore: invalid index`.
    fn set_pair(ref self: T, i: u32, a: SolverBody, j: u32, b: SolverBody);
}

/// Dictionary candidate: one nullable body per dense index, O(1) gather/scatter.
#[derive(Destruct)]
pub struct DenseBodies {
    values: Felt252Dict<Nullable<SolverBody>>,
    count: u32,
}
pub impl DenseBodiesImpl of DenseBodiesTrait<DenseBodies> {
    fn new(mut bodies: Span<SolverBody>) -> DenseBodies {
        let mut values: Felt252Dict<Nullable<SolverBody>> = Default::default();
        let mut count: u32 = 0;
        while let Some(b) = bodies.pop_front() {
            values.insert(count.into(), NullableTrait::new(*b));
            count += 1;
        }
        DenseBodies { values, count }
    }
    fn len(self: @DenseBodies) -> u32 {
        *self.count
    }
    fn get(ref self: DenseBodies, index: u32) -> SolverBody {
        if index == WORLD {
            return Default::default();
        }
        assert(index < self.count, errors::INDEX);
        match match_nullable(self.values.get(index.into())) {
            FromNullableResult::NotNull(b) => b.unbox(),
            FromNullableResult::Null => core::panic_with_felt252(errors::INDEX),
        }
    }
    fn set_pair(ref self: DenseBodies, i: u32, a: SolverBody, j: u32, b: SolverBody) {
        if i != WORLD {
            assert(i < self.count, errors::INDEX);
            self.values.insert(i.into(), NullableTrait::new(a));
        }
        if j != WORLD {
            assert(j < self.count, errors::INDEX);
            self.values.insert(j.into(), NullableTrait::new(b));
        }
    }
}

/// Full generational handle → dense index map. Missing and stale handles return `None`.
#[derive(Destruct, Default)]
pub struct SolverBodyIndexMap {
    values: Felt252Dict<u64>,
}
#[generate_trait]
pub impl SolverBodyIndexMapImpl of SolverBodyIndexMapTrait {
    /// Exact lookup, no rounding; does not accept a stale generation.
    fn get(ref self: SolverBodyIndexMap, handle: Handle) -> Option<u32> {
        let value = self.values.get(handle.into());
        if value == 0 {
            None
        } else {
            Some((value - 1).try_into().unwrap())
        }
    }
}

#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub(crate) struct BodyStep {
    pub increment: RigidBodyVelocity,
    pub local_com: Vec2,
    pub damping: RigidBodyDamping,
    pub moving: bool,
}

/// Whole-world scratch, valid for one solve only. Force increments, mass properties and body
/// membership are frozen at construction. Sleeping is deliberately ignored (DF deferral).
#[derive(Destruct)]
pub struct SolverBodyStore {
    pub(crate) bodies: DenseBodies,
    pub(crate) steps: Span<BodyStep>,
}

#[generate_trait]
pub impl SolverBodyStoreImpl of SolverBodyStoreTrait {
    /// Gather all bodies in ascending arena slot order, plus a full-generation index map.
    /// Requires `ref bodies` because the frozen set API logs dictionary reads. Refreshes mass
    /// properties and computes gravity/user-force increments at `params.substep_dt()` once.
    /// Kinematic velocities must already be prepared by the caller. Products floor, divisions
    /// round to nearest; fixed overflow and zero solver-iteration panics propagate.
    fn from_bodies(
        ref bodies: RigidBodySet, gravity: Vec2, params: IntegrationParameters,
    ) -> (SolverBodyStore, SolverBodyIndexMap) {
        let dt = params.substep_dt();
        let mut dense = array![];
        let mut steps = array![];
        let mut map: SolverBodyIndexMap = Default::default();
        for (handle, rb) in bodies.iter() {
            let (body, step) = gather(handle, rb, gravity, dt);
            map.values.insert(handle.into(), dense.len().into() + 1_u64);
            dense.append(body);
            steps.append(step);
        }
        (SolverBodyStore { bodies: DenseBodiesTrait::new(dense.span()), steps: steps.span() }, map)
    }
    /// [`Self::from_bodies`] over given `(handle, body)` entries, in their order, without the
    /// index map and without reading a set: the pipeline passes only the bodies a constraint
    /// references. Same arithmetic and panics as `from_bodies`.
    fn from_entries(
        entries: Span<(Handle, RigidBody)>, gravity: Vec2, params: IntegrationParameters,
    ) -> SolverBodyStore {
        let dt = params.substep_dt();
        let mut dense = array![];
        let mut steps = array![];
        for (handle, rb) in entries {
            let (body, step) = gather(*handle, *rb, gravity, dt);
            dense.append(body);
            steps.append(step);
        }
        SolverBodyStore { bodies: DenseBodiesTrait::new(dense.span()), steps: steps.span() }
    }
    /// Read a dense solver body at its centre of mass. Invalid non-world ids panic.
    fn get(ref self: SolverBodyStore, index: u32) -> SolverBody {
        self.bodies.get(index)
    }
    /// Number of bodies, including fixed and disabled bodies.
    fn len(self: @SolverBodyStore) -> u32 {
        self.bodies.len()
    }
    /// Persist damped velocities and `next_position`; current poses stay at the beginning of
    /// the step, as upstream. The pipeline advances them after solving. Position-based
    /// kinematics retain their exact user target. Fixed/disabled bodies are untouched.
    /// Products floor; fixed overflow or `SolverStore: missing body` can panic. Do not change
    /// the body set between construction and writeback.
    fn to_bodies(ref self: SolverBodyStore, ref bodies: RigidBodySet) {
        let mut i = 0;
        let n = self.bodies.len();
        while i != n {
            let sb = self.bodies.get(i);
            let mut rb = bodies.get(sb.handle).expect(errors::BODY);
            let step = *self.steps.at(i);
            if step.moving {
                writeback(sb, step, ref rb);
                assert(bodies.set(sb.handle, rb), errors::BODY);
            }
            i += 1;
        }
    }
    /// [`Self::to_bodies`] for the dense body `index` applied to the value `rb` (the body of
    /// that handle) instead of the set; a non-moving body is left unchanged. Invalid ids panic.
    fn write_body(ref self: SolverBodyStore, index: u32, ref rb: RigidBody) {
        let step = *self.steps.at(index);
        if step.moving {
            writeback(self.bodies.get(index), step, ref rb);
        }
    }
}

/// The solver body and step data of one body, as `from_bodies` gathers it: world mass
/// properties at the current pose, force increment over one substep `dt`.
#[inline(always)]
pub(crate) fn gather(
    handle: Handle, rb: RigidBody, gravity: Vec2, dt: Fixed,
) -> (SolverBody, BodyStep) {
    let moving = rb.enabled && rb.body_type != RigidBodyType::Fixed;
    let mp = rb.mprops.update_world_mass_properties(rb.body_type, rb.pos.position);
    let forces = rb.forces.compute_effective_force_and_torque(gravity, mp.effective_mass());
    let increment = if moving && rb.body_type == RigidBodyType::Dynamic {
        forces.integrate(dt, Default::default(), mp)
    } else {
        Default::default()
    };
    let body = SolverBody {
        handle,
        position: Pose2 { translation: mp.world_com, rotation: rb.pos.position.rotation },
        linvel: if moving {
            rb.vels.linvel
        } else {
            Default::default()
        },
        angvel: if moving {
            rb.vels.angvel
        } else {
            ZERO
        },
        im: if moving {
            mp.effective_inv_mass
        } else {
            Default::default()
        },
        ii: if moving {
            mp.effective_world_inv_inertia
        } else {
            ZERO
        },
    };
    (
        body,
        BodyStep { increment, local_com: mp.local_mprops.local_com, damping: rb.damping, moving },
    )
}

/// The writeback of one moving body (see `to_bodies`): velocities, and `next_position` unless
/// the body is position-based kinematic. Products floor.
#[inline(always)]
pub(crate) fn writeback(sb: SolverBody, step: BodyStep, ref rb: RigidBody) {
    rb.vels = RigidBodyVelocity { linvel: sb.linvel, angvel: sb.angvel };
    if rb.body_type != RigidBodyType::KinematicPositionBased {
        rb
            .pos
            .next_position =
                Pose2 {
                    translation: sb.position.translation
                        - sb.position.rotation.rotate(step.local_com),
                    rotation: sb.position.rotation,
                };
    }
}
use rapier_math::rot2::Rot2Trait;

#[cfg(test)]
pub(crate) mod alternatives {
    use super::*;
    /// Rejected candidate retained for gas reranking: O(B) rebuilding scatter.
    #[derive(Drop)]
    pub struct ArrayBodies {
        values: Array<SolverBody>,
    }
    pub impl ArrayBodiesImpl of DenseBodiesTrait<ArrayBodies> {
        fn new(bodies: Span<SolverBody>) -> ArrayBodies {
            let mut values = array![];
            values.append_span(bodies);
            ArrayBodies { values }
        }
        fn len(self: @ArrayBodies) -> u32 {
            self.values.len()
        }
        fn get(ref self: ArrayBodies, index: u32) -> SolverBody {
            if index == WORLD {
                Default::default()
            } else {
                assert(index < self.values.len(), errors::INDEX);
                *self.values.at(index)
            }
        }
        fn set_pair(ref self: ArrayBodies, i: u32, a: SolverBody, j: u32, b: SolverBody) {
            let n = self.values.len();
            assert((i == WORLD || i < n) && (j == WORLD || j < n), errors::INDEX);
            let mut out = array![];
            let mut k = 0;
            while let Some(mut value) = self.values.pop_front() {
                if k == i {
                    value = a;
                }
                if k == j {
                    value = b;
                }
                out.append(value);
                k += 1;
            }
            self.values = out;
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, ONE};
    use rapier_testing::opaque;
    use crate::collider_set::ColliderSetTrait;
    use crate::rigid_body_set::RigidBodyTrait;
    use super::*;
    use super::alternatives::ArrayBodies;

    fn fixture(n: u32) -> Array<SolverBody> {
        let mut out = array![];
        let mut i = 0;
        while i != n {
            out
                .append(
                    SolverBody { handle: Handle { index: i, generation: 7 }, ..Default::default() },
                );
            i += 1;
        }
        out
    }
    #[test]
    fn test_dictionary_array_order_world_and_alias() {
        let input = fixture(5);
        let mut a: DenseBodies = DenseBodiesTrait::new(input.span());
        let mut b: ArrayBodies = DenseBodiesTrait::new(input.span());
        let value = SolverBody { angvel: ONE, ..*input.at(2) };
        let mut i = 0;
        for (j, k) in [(0, 4), (2, 2), (WORLD, 3), (1, WORLD), (WORLD, WORLD)].span() {
            a.set_pair(*j, value, *k, *input.at(0));
            b.set_pair(*j, value, *k, *input.at(0));
            while i != 5 {
                assert_eq!(a.get(i), b.get(i));
                i += 1;
            }
            i = 0;
        }
        assert_eq!(a.get(WORLD), Default::default());
        assert_eq!(a.len(), b.len());
    }
    #[test]
    fn test_gather_map_generation_and_writeback() {
        let mut set = RigidBodySetTrait::new();
        let mut colliders = ColliderSetTrait::new();
        let old = set.insert(RigidBodyTrait::dynamic(Default::default()));
        let fixed = set.insert(RigidBodyTrait::fixed(Default::default()));
        let _ = set.remove(old, ref colliders, false);
        let mut rb = RigidBodyTrait::dynamic(Default::default());
        rb.mprops.local_mprops.local_com = Vec2 { x: ONE, y: ZERO };
        rb.mprops.local_mprops.inv_mass = ONE;
        let h = set.insert(rb);
        let (mut store, mut map) = SolverBodyStoreTrait::from_bodies(
            ref set, Default::default(), Default::default(),
        );
        assert_eq!(store.len(), 2);
        assert_eq!(map.get(old), None);
        assert_eq!(map.get(h), Some(0));
        assert_eq!(map.get(fixed), Some(1));
        let mut b = store.get(0);
        assert_eq!(b.position.translation.x, ONE);
        b.position.translation.y = ONE;
        b.linvel.x = ONE;
        store.bodies.set_pair(0, b, WORLD, Default::default());
        store.to_bodies(ref set);
        let written = set.get(h).unwrap();
        assert_eq!(written.pos.position, rb.pos.position);
        assert_eq!(written.pos.next_position.translation, Vec2 { x: ZERO, y: ONE });
        assert_eq!(written.vels.linvel.x, ONE);
    }
    #[test]
    #[fuzzer(runs: 32, seed: 91)]
    fn fuzz_store_equivalence(x: i16, index: u8) {
        let input = fixture(8);
        let mut a: DenseBodies = DenseBodiesTrait::new(input.span());
        let mut b: ArrayBodies = DenseBodiesTrait::new(input.span());
        let (_, i) = core::num::traits::DivRem::div_rem(index, 8);
        let i: u32 = i.into();
        let value = SolverBody { angvel: Fixed { raw: x.into() }, ..a.get(i) };
        a.set_pair(i, value, WORLD, Default::default());
        b.set_pair(i, value, WORLD, Default::default());
        let mut j = 0;
        while j != 8 {
            assert_eq!(a.get(j), b.get(j));
            j += 1;
        }
    }
    #[test]
    #[should_panic(expected: 'SolverStore: invalid index')]
    fn test_invalid_read() {
        let mut a: DenseBodies = DenseBodiesTrait::new([].span());
        a.get(0);
    }
    #[test]
    #[should_panic(expected: 'SolverStore: invalid index')]
    fn test_invalid_write() {
        let mut a: DenseBodies = DenseBodiesTrait::new([].span());
        a.set_pair(WORLD, Default::default(), 0, Default::default());
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }
    fn probe<B, +DenseBodiesTrait<B>, +Destruct<B>>(n: u32, rounds: u32) {
        let input = fixture(opaque(n));
        let mut store: B = DenseBodiesTrait::new(input.span());
        let mut k = 0;
        while k != rounds {
            let mut i = 0;
            while i != n {
                let mut b = store.get(i);
                b.angvel = opaque(ONE);
                store.set_pair(i, b, WORLD, Default::default());
                i += 1;
            }
            k += 1;
        }
        assert_eq!(store.len(), n);
        let _ = opaque(store.get(n - 1));
    }
    #[test]
    fn gas_dict_new_len_get() {
        probe::<DenseBodies>(5, 0);
    }
    #[test]
    fn gas_array_new_len_get() {
        probe::<ArrayBodies>(5, 0);
    }
    #[test]
    fn gas_dict_scatter_1() {
        probe::<DenseBodies>(1, 4);
    }
    #[test]
    fn gas_array_scatter_1() {
        probe::<ArrayBodies>(1, 4);
    }
    #[test]
    fn gas_dict_scatter_2() {
        probe::<DenseBodies>(2, 4);
    }
    #[test]
    fn gas_array_scatter_2() {
        probe::<ArrayBodies>(2, 4);
    }
    #[test]
    fn gas_dict_scatter_5() {
        probe::<DenseBodies>(5, 4);
    }
    #[test]
    fn gas_array_scatter_5() {
        probe::<ArrayBodies>(5, 4);
    }
    #[test]
    fn gas_dict_scatter_16() {
        probe::<DenseBodies>(16, 4);
    }
    #[test]
    fn gas_array_scatter_16() {
        probe::<ArrayBodies>(16, 4);
    }
    #[test]
    fn gas_dict_scatter_32() {
        probe::<DenseBodies>(32, 4);
    }
    #[test]
    fn gas_array_scatter_32() {
        probe::<ArrayBodies>(32, 4);
    }
    #[test]
    fn gas_from_bodies_map_get_store_get_len_to_bodies() {
        let mut set = RigidBodySetTrait::new();
        let h = set.insert(opaque(RigidBodyTrait::dynamic(Default::default())));
        let (mut store, mut map) = SolverBodyStoreTrait::from_bodies(
            ref set, opaque(Default::default()), opaque(Default::default()),
        );
        assert_eq!(map.get(h), Some(0));
        assert_eq!(store.len(), 1);
        let _ = opaque(store.get(0));
        store.to_bodies(ref set);
    }
}
