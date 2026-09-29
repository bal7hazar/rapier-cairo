//! CS7's rejected reader: the contact pairs by the outlined readers of `decode`, for the codec and
//! for the answer of `rapier2d_classes`' `NarrowPhaseClass` (the derived `Serde` is shared by both
//! in a caller class). Measured on `SlimSplitStep`: −2,098 CASM felts, but +927,684 Cairo steps
//! on the pile10 shot (≈ 650 per pair decoded, 1,428 pairs): the outlined leaf readers cost a
//! call per field where the derived code converts inline. `tests`: same values as the derived
//! `Serde`, and the `gas_*` probes of both readers.

use rapier_core::Handle;
use rapier_dynamics2d::events::PairEventStatus;
use rapier_dynamics2d::narrow_phase::ContactPair;
use rapier_geometry2d::contact::{
    ContactData, ContactManifold, ContactManifoldData, SolverContact, SolverFlags, TrackedContact,
};
use rapier_geometry2d::feature_id::FeatureId;
use super::{read_fixed, read_handle, read_u32, read_vec2};

fn read_option_handle(ref s: Span<felt252>) -> Option<Option<Handle>> {
    let tag: felt252 = Serde::deserialize(ref s)?;
    match tag {
        0 => Some(Some(read_handle(ref s)?)),
        1 => Some(None),
        _ => None,
    }
}

#[inline(never)]
fn read_tracked(ref s: Span<felt252>) -> Option<TrackedContact> {
    Some(
        TrackedContact {
            local_p1: read_vec2(ref s)?,
            local_p2: read_vec2(ref s)?,
            dist: read_fixed(ref s)?,
            fid1: FeatureId { packed: read_u32(ref s)? },
            fid2: FeatureId { packed: read_u32(ref s)? },
            data: ContactData {
                impulse: read_fixed(ref s)?,
                tangent_impulse: read_fixed(ref s)?,
                warmstart_impulse: read_fixed(ref s)?,
                warmstart_tangent_impulse: read_fixed(ref s)?,
            },
        },
    )
}

#[inline(never)]
fn read_solver_contact(ref s: Span<felt252>) -> Option<SolverContact> {
    Some(
        SolverContact {
            anchor1: read_vec2(ref s)?,
            anchor2: read_vec2(ref s)?,
            dist: read_fixed(ref s)?,
            tangent_velocity: read_vec2(ref s)?,
            contact_id: read_u32(ref s)?,
        },
    )
}

fn read_pair(ref s: Span<felt252>) -> Option<ContactPair> {
    let collider1 = read_handle(ref s)?;
    let collider2 = read_handle(ref s)?;
    let points = [read_tracked(ref s)?, read_tracked(ref s)?];
    let num_points = Serde::deserialize(ref s)?;
    let local_n1 = read_vec2(ref s)?;
    let local_n2 = read_vec2(ref s)?;
    let subshape1 = read_u32(ref s)?;
    let subshape2 = read_u32(ref s)?;
    let data = ContactManifoldData {
        rigid_body1: read_option_handle(ref s)?,
        rigid_body2: read_option_handle(ref s)?,
        solver_flags: SolverFlags { bits: read_u32(ref s)? },
        normal: read_vec2(ref s)?,
        solver_contacts: [read_solver_contact(ref s)?, read_solver_contact(ref s)?],
        num_solver_contacts: Serde::deserialize(ref s)?,
        relative_dominance: Serde::deserialize(ref s)?,
        user_data: read_u32(ref s)?,
        friction: read_fixed(ref s)?,
        restitution: read_fixed(ref s)?,
    };
    Some(
        ContactPair {
            collider1,
            collider2,
            manifold: ContactManifold {
                points, num_points, local_n1, local_n2, subshape1, subshape2, data,
            },
            event_status: PairEventStatus { bits: Serde::deserialize(ref s)? },
        },
    )
}

/// `Array<ContactPair>`'s felts.
pub fn read_pairs(ref s: Span<felt252>) -> Option<Array<ContactPair>> {
    let len = read_u32(ref s)?;
    let mut out = array![];
    let mut i = 0;
    while i != len {
        out.append(read_pair(ref s)?);
        i += 1;
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use rapier_dynamics2d::narrow_phase::ContactPair;
    use rapier_testing::opaque;
    use crate::pipeline::config::BasicStepConfig;
    use crate::pipeline::config::tests::basic_level;
    use crate::world::WorldTrait;
    use super::read_pairs;

    /// The pairs of `basic_level(seed)` after `steps` basic steps, as felts.
    fn pair_felts(seed: u32, steps: u32) -> Array<felt252> {
        let mut world = basic_level(seed);
        let mut k = 0;
        while k != steps {
            let _ = world.step_with_force_events_with::<BasicStepConfig>();
            k += 1;
        }
        let mut out = array![];
        world.narrow_phase.pairs.serialize(ref out);
        out
    }

    #[test]
    fn test_read_pairs_as_derived() {
        for (seed, steps) in array![(1_u32, 3_u32), (2, 12), (5, 40)] {
            let felts = pair_felts(seed, steps);
            let mut a = felts.span();
            let mut b = felts.span();
            let derived: Array<ContactPair> = Serde::deserialize(ref a).unwrap();
            assert!(read_pairs(ref b).unwrap() == derived, "level {} step {}", seed, steps);
            assert!(a.is_empty() && b.is_empty());
        }
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(1_u32);
    }

    #[test]
    fn gas_read_pairs_derived() {
        let felts = pair_felts(opaque(2), 12);
        let mut span = felts.span();
        let pairs: Array<ContactPair> = Serde::deserialize(ref span).unwrap();
        let _ = opaque(pairs.len());
    }

    #[test]
    fn gas_read_pairs_outlined() {
        let felts = pair_felts(opaque(2), 12);
        let mut span = felts.span();
        let _ = opaque(read_pairs(ref span).unwrap().len());
    }
}
