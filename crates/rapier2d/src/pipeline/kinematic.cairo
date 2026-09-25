//! Kinematic velocity preparation, before collision detection and island wake-ups.
use fixed::Fixed;
use rapier_core::Handle;

#[cfg(test)]
mod alternatives;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_core::rigid_body::RigidBodyType;
use rapier_dynamics2d::rigid_body::RigidBodyPositionTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet, RigidBodySetTrait};
use rapier_math::math_ext::inv;

/// Updates one awake enabled position-based body; preserves the target exactly.
/// Products floor, inv(dt) rounds nearest-even (zero maps to zero); overflow panics.
#[inline(never)]
pub(crate) fn prepare(body: RigidBody, dt: Fixed) -> RigidBody {
    let mut body = body;
    if body.enabled && !body.activation.sleeping {
        body.vels = body.pos.interpolate_velocity(inv(dt), body.mprops.local_mprops.local_com);
    }
    body
}

// The normal body's path never copies a whole RigidBody into a merge variable.
#[inline(never)]
pub(crate) fn prepare_in_set(ref bodies: RigidBodySet, handle: Handle, dt: Fixed) {
    if let Some(body) = bodies.get(handle) {
        let _ = bodies.set(handle, prepare(body, dt));
    }
}

/// Scans the already collected snapshots; reads only kinematic bodies from the updated set.
pub(crate) fn prepare_existing(
    ref bodies: RigidBodySet, entries: Span<(Handle, RigidBody)>, dt: Fixed,
) {
    for (h, body) in entries {
        if *body.body_type == RigidBodyType::KinematicPositionBased {
            prepare_in_set(ref bodies, *h, dt);
        }
    }
}

/// Upstream `interpolate_kinematic_velocities`, for callers using separate stages.
/// Call after user changes and before collision detection/island computation. Ascending slots;
/// current poses stay unchanged. Rounding/overflow as `RigidBodyPosition::interpolate_velocity`.
pub fn interpolate_kinematic_velocities(ref bodies: RigidBodySet, params: IntegrationParameters) {
    prepare_all(ref bodies, params.dt);
}

pub(crate) fn prepare_all(ref bodies: RigidBodySet, dt: Fixed) {
    for (h, body) in bodies.iter() {
        if body.body_type == RigidBodyType::KinematicPositionBased {
            let _ = bodies.set(h, prepare(body, dt));
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{ONE, ZERO};
    use glam::Vec2;
    use rapier_dynamics2d::rigid_body_set::RigidBodyTrait;
    use rapier_testing::opaque;
    use crate::world::WorldTrait;
    use super::*;
    fn probe(variant: u8, position_based: bool) {
        let mut world = crate::pipeline::fixtures::free_fall(opaque(8));
        if position_based {
            world.insert_body(RigidBodyTrait::kinematic_position_based(Default::default()));
        }
        if variant == 0 {
            let _ = alternatives::user_changes_copied_body(
                ref world.bodies,
                ref world.colliders,
                array![].span(),
                Some(world.integration_parameters.dt),
            );
        } else if variant == 1 {
            let _ = alternatives::user_changes_per_body(
                ref world.bodies,
                ref world.colliders,
                array![].span(),
                Some(world.integration_parameters.dt),
            );
        } else if variant == 2 {
            let _ = alternatives::user_changes_second_walk(
                ref world.bodies,
                ref world.colliders,
                array![].span(),
                Some(world.integration_parameters.dt),
            );
        } else {
            let _ = super::super::user_changes_bodies_for_step(
                ref world.bodies,
                ref world.colliders,
                array![].span(),
                Some(world.integration_parameters.dt),
                true,
            );
        }
    }
    #[test]
    fn gas_prepare_copied_dynamic8() {
        probe(0, false);
    }
    #[test]
    fn gas_prepare_per_body_dynamic8() {
        probe(1, false);
    }
    #[test]
    fn gas_prepare_copied_kinematic() {
        probe(0, true);
    }
    #[test]
    fn gas_prepare_per_body_kinematic() {
        probe(1, true);
    }
    #[test]
    fn gas_prepare_conditional_dynamic8() {
        probe(2, false);
    }
    #[test]
    fn gas_prepare_conditional_kinematic() {
        probe(2, true);
    }
    #[test]
    fn gas_prepare_reused_dynamic8() {
        probe(3, false);
    }
    #[test]
    fn gas_prepare_reused_kinematic() {
        probe(3, true);
    }
    fn prepared(variant: u8, delta: Fixed, enabled: bool) -> Array<(Handle, RigidBody)> {
        let mut world = crate::pipeline::fixtures::free_fall(3);
        let mut body = RigidBodyTrait::kinematic_position_based(Default::default());
        body.set_next_kinematic_translation(Vec2 { x: delta, y: ONE });
        body.enabled = enabled;
        world.insert_body(body);
        let dt = Some(world.integration_parameters.dt);
        if variant == 0 {
            let _ = alternatives::user_changes_copied_body(
                ref world.bodies, ref world.colliders, array![].span(), dt,
            );
        } else if variant == 1 {
            let _ = alternatives::user_changes_per_body(
                ref world.bodies, ref world.colliders, array![].span(), dt,
            );
        } else if variant == 2 {
            let _ = alternatives::user_changes_second_walk(
                ref world.bodies, ref world.colliders, array![].span(), dt,
            );
        } else {
            let _ = super::super::user_changes_bodies_for_step(
                ref world.bodies, ref world.colliders, array![].span(), dt, true,
            );
        }
        world.bodies.iter()
    }
    #[test]
    #[fuzzer(runs: 8, seed: 20260925)]
    fn fuzz_preparation_variants(delta: i16, enabled: bool) {
        let delta = Fixed { raw: delta.into() * 65536 };
        let expected = prepared(3, delta, enabled);
        for variant in array![0, 1, 2] {
            assert_eq!(prepared(variant, delta, enabled), expected);
        }
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }
    #[test]
    fn gas_interpolate_kinematic_velocities() {
        let mut set = RigidBodySetTrait::new();
        let mut b = RigidBodyTrait::kinematic_position_based(Default::default());
        b.set_next_kinematic_translation(opaque(Vec2 { x: ONE, y: ZERO }));
        set.insert(b);
        interpolate_kinematic_velocities(ref set, opaque(Default::default()));
    }
}
