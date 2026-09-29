//! The basic codec's reader (CS7): `WorldState`'s felts read field by field.
//!
//! The derived `Serde` of a struct inlines the conversion of every field (a `Fixed` is a range
//! check and a branch per field, a `[Vec2; 8]` a chain of seven tuple splits): the reader of a
//! world state was 12k of a caller class's 73k CASM felts. Here each leaf (`Fixed`, `Vec2`, a pose,
//! a handle, the small integers) has one outlined reader that every field calls, and the arenas'
//! entries and the polygons' vertices are read in loops. Same felts, same values as the derived
//! `Serde` (`tests`: `read.state == world.to_state()` on stepped levels); a malformed input is
//! `None`, as with the derived code. The contact pairs keep their derived `Serde`, which the
//! narrow phase's crossing (`rapier2d_classes`) compiles in the same class.

use fixed::Fixed;
use glam_core::Vec2;
use rapier_core::Handle;
use rapier_core::collider::{
    ActiveCollisionTypes, ActiveEvents, ActiveHooks, ColliderChanges, ColliderFlags,
    ColliderMaterial,
};
use rapier_core::data::arena::ArenaState;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_core::integration_parameters::spring::SpringCoefficients;
use rapier_core::interaction_groups::{Group, InteractionGroups};
use rapier_core::rigid_body::{RigidBodyActivation, RigidBodyChanges, RigidBodyDamping};
use rapier_dynamics2d::collider::components::OneWayPlatform;
use rapier_dynamics2d::collider::{Collider, ColliderMassProps, ColliderParent, ColliderPosition};
use rapier_dynamics2d::rigid_body::{
    LockedAxes, RigidBodyCcd, RigidBodyForces, RigidBodyMassProps, RigidBodyPosition,
    RigidBodyVelocity,
};
use rapier_dynamics2d::rigid_body_set::{
    ColdExtraSlotTrait, RigidBody, RigidBodyCold, RigidBodyColdExtra,
};
use rapier_geometry2d::aabb::Aabb;
use rapier_geometry2d::broad_phase::BroadPhaseProxy;
use rapier_geometry2d::mass::MassProperties;
use rapier_geometry2d::shape::ball::Ball;
use rapier_geometry2d::shape::cuboid::Cuboid;
use rapier_geometry2d::shape::halfspace::HalfSpace;
use rapier_geometry2d::shape::{ConvexPolygon, Shape};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use crate::pipeline::active_set::ActiveSet;
use super::errors;

#[inline(never)]
pub(crate) fn read_fixed(ref s: Span<felt252>) -> Option<Fixed> {
    Serde::deserialize(ref s)
}

#[inline(never)]
pub(crate) fn read_u32(ref s: Span<felt252>) -> Option<u32> {
    Serde::deserialize(ref s)
}

#[inline(never)]
pub(crate) fn read_bool(ref s: Span<felt252>) -> Option<bool> {
    Serde::deserialize(ref s)
}

#[inline(never)]
pub(crate) fn read_vec2(ref s: Span<felt252>) -> Option<Vec2> {
    Some(Vec2 { x: read_fixed(ref s)?, y: read_fixed(ref s)? })
}

#[inline(never)]
pub(crate) fn read_handle(ref s: Span<felt252>) -> Option<Handle> {
    Some(Handle { index: read_u32(ref s)?, generation: read_u32(ref s)? })
}

#[inline(never)]
fn read_pose(ref s: Span<felt252>) -> Option<Pose2> {
    Some(
        Pose2 {
            translation: read_vec2(ref s)?,
            rotation: Rot2 { re: read_fixed(ref s)?, im: read_fixed(ref s)? },
        },
    )
}

#[inline(never)]
fn read_mass_properties(ref s: Span<felt252>) -> Option<MassProperties> {
    Some(
        MassProperties {
            local_com: read_vec2(ref s)?,
            inv_mass: read_fixed(ref s)?,
            inv_principal_inertia: read_fixed(ref s)?,
        },
    )
}

/// A length, then its handles.
fn read_handles(ref s: Span<felt252>) -> Option<Array<Handle>> {
    let len = read_u32(ref s)?;
    let mut out = array![];
    let mut i = 0;
    while i != len {
        out.append(read_handle(ref s)?);
        i += 1;
    }
    Some(out)
}

/// A length, then its `u32`s.
fn read_u32s(ref s: Span<felt252>) -> Option<Array<u32>> {
    let len = read_u32(ref s)?;
    let mut out = array![];
    let mut i = 0;
    while i != len {
        out.append(read_u32(ref s)?);
        i += 1;
    }
    Some(out)
}

fn read_spring(ref s: Span<felt252>) -> Option<SpringCoefficients> {
    Some(
        SpringCoefficients {
            natural_frequency: read_fixed(ref s)?, damping_ratio: read_fixed(ref s)?,
        },
    )
}

/// `IntegrationParameters`' felts.
pub(crate) fn read_parameters(ref s: Span<felt252>) -> Option<IntegrationParameters> {
    Some(
        IntegrationParameters {
            dt: read_fixed(ref s)?,
            min_ccd_dt: read_fixed(ref s)?,
            contact_softness: read_spring(ref s)?,
            static_contact_softness: read_spring(ref s)?,
            warmstart_coefficient: read_fixed(ref s)?,
            length_unit: read_fixed(ref s)?,
            normalized_allowed_linear_error: read_fixed(ref s)?,
            normalized_max_corrective_velocity: read_fixed(ref s)?,
            normalized_prediction_distance: read_fixed(ref s)?,
            normalized_max_linear_velocity: read_fixed(ref s)?,
            num_solver_iterations: read_u32(ref s)?,
            num_internal_pgs_iterations: read_u32(ref s)?,
            num_internal_stabilization_iterations: read_u32(ref s)?,
            max_ccd_substeps: read_u32(ref s)?,
            contact_clustering: read_bool(ref s)?,
            contact_recycling: read_bool(ref s)?,
            normalized_contact_recycle_distance: read_fixed(ref s)?,
            friction_in_bias_pass: read_bool(ref s)?,
            warmstart_joints: read_bool(ref s)?,
        },
    )
}

/// `RigidBodyMassProps`' felts (also the answer of `rapier2d_classes`' `MassClass`).
pub fn read_body_mass_props(ref s: Span<felt252>) -> Option<RigidBodyMassProps> {
    Some(
        RigidBodyMassProps {
            flags: LockedAxes { bits: Serde::deserialize(ref s)? },
            local_mprops: read_mass_properties(ref s)?,
            world_com: read_vec2(ref s)?,
            effective_inv_mass: read_vec2(ref s)?,
            effective_world_inv_inertia: read_fixed(ref s)?,
            max_extent: read_fixed(ref s)?,
        },
    )
}

/// `Box<Option<RigidBodyCold>>`'s felts.
fn read_cold(ref s: Span<felt252>) -> Option<Box<Option<RigidBodyCold>>> {
    let tag: felt252 = Serde::deserialize(ref s)?;
    if tag == 1 {
        return Some(BoxTrait::new(None));
    }
    if tag != 0 {
        return None;
    }
    let additional_local_mprops = read_mass_properties(ref s)?;
    let solver_flags: u128 = Serde::deserialize(ref s)?;
    let extra_tag: felt252 = Serde::deserialize(ref s)?;
    let extra = if extra_tag == 1 {
        Default::default()
    } else if extra_tag == 0 {
        let user_data: u128 = Serde::deserialize(ref s)?;
        let ccd = RigidBodyCcd {
            ccd_thickness: read_fixed(ref s)?,
            ccd_active: read_bool(ref s)?,
            ccd_enabled: read_bool(ref s)?,
            soft_ccd_prediction: read_fixed(ref s)?,
        };
        ColdExtraSlotTrait::new(RigidBodyColdExtra { user_data, ccd })
    } else {
        return None;
    };
    Some(BoxTrait::new(Some(RigidBodyCold { additional_local_mprops, solver_flags, extra })))
}

fn read_body(ref s: Span<felt252>) -> Option<RigidBody> {
    let pos = RigidBodyPosition { position: read_pose(ref s)?, next_position: read_pose(ref s)? };
    let mprops = read_body_mass_props(ref s)?;
    let vels = RigidBodyVelocity { linvel: read_vec2(ref s)?, angvel: read_fixed(ref s)? };
    let damping = RigidBodyDamping {
        linear_damping: read_fixed(ref s)?, angular_damping: read_fixed(ref s)?,
    };
    let forces = RigidBodyForces {
        force: read_vec2(ref s)?,
        torque: read_fixed(ref s)?,
        gravity_scale: read_fixed(ref s)?,
        user_force: read_vec2(ref s)?,
        user_torque: read_fixed(ref s)?,
    };
    let colliders = read_handles(ref s)?.span();
    let activation = RigidBodyActivation {
        normalized_linear_threshold: read_fixed(ref s)?,
        angular_threshold: read_fixed(ref s)?,
        time_until_sleep: read_fixed(ref s)?,
        time_since_can_sleep: read_fixed(ref s)?,
        sleeping: read_bool(ref s)?,
    };
    Some(
        RigidBody {
            pos,
            mprops,
            vels,
            damping,
            forces,
            colliders,
            activation,
            changes: RigidBodyChanges { bits: read_u32(ref s)? },
            body_type: Serde::deserialize(ref s)?,
            dominance: Serde::deserialize(ref s)?,
            enabled: read_bool(ref s)?,
            cold: read_cold(ref s)?,
        },
    )
}

/// `ArenaState<RigidBody>`'s felts.
pub(crate) fn read_bodies(ref s: Span<felt252>) -> Option<ArenaState<RigidBody>> {
    let generation = read_u32(ref s)?;
    let capacity = read_u32(ref s)?;
    let free_list = read_u32s(ref s)?.span();
    let len = read_u32(ref s)?;
    let mut entries = array![];
    let mut i = 0;
    while i != len {
        let handle = read_handle(ref s)?;
        entries.append((handle, read_body(ref s)?));
        i += 1;
    }
    Some(ArenaState { generation, capacity, free_list, entries: entries.span() })
}

fn read_vertices(ref s: Span<felt252>) -> Option<[Vec2; 8]> {
    Some(
        [
            read_vec2(ref s)?, read_vec2(ref s)?, read_vec2(ref s)?, read_vec2(ref s)?,
            read_vec2(ref s)?, read_vec2(ref s)?, read_vec2(ref s)?, read_vec2(ref s)?,
        ],
    )
}

/// `deserialize_basic_shape`'s shapes.
///
/// # Panics
/// [`errors::NOT_BASIC`] on another tag.
fn read_shape(ref s: Span<felt252>) -> Option<Shape> {
    let tag: felt252 = Serde::deserialize(ref s)?;
    Some(
        match tag {
            0 => Shape::Ball(Ball { radius: read_fixed(ref s)? }),
            1 => Shape::Cuboid(Cuboid { half_extents: read_vec2(ref s)? }),
            4 => Shape::HalfSpace(HalfSpace { normal: read_vec2(ref s)? }),
            5 => {
                let polygon = ConvexPolygon {
                    vertices: read_vertices(ref s)?,
                    normals: read_vertices(ref s)?,
                    count: Serde::deserialize(ref s)?,
                };
                Shape::ConvexPolygon(BoxTrait::new(polygon))
            },
            _ => core::panic_with_felt252(errors::NOT_BASIC),
        },
    )
}

fn read_collider_mass(ref s: Span<felt252>) -> Option<ColliderMassProps> {
    let tag: felt252 = Serde::deserialize(ref s)?;
    match tag {
        0 => Some(ColliderMassProps::Density(read_fixed(ref s)?)),
        1 => Some(ColliderMassProps::Mass(read_fixed(ref s)?)),
        2 => Some(ColliderMassProps::MassProperties(read_mass_properties(ref s)?)),
        _ => None,
    }
}

fn read_parent(ref s: Span<felt252>) -> Option<Option<ColliderParent>> {
    let tag: felt252 = Serde::deserialize(ref s)?;
    match tag {
        0 => Some(
            Some(ColliderParent { handle: read_handle(ref s)?, pos_wrt_parent: read_pose(ref s)? }),
        ),
        1 => Some(None),
        _ => None,
    }
}

fn read_groups(ref s: Span<felt252>) -> Option<InteractionGroups> {
    Some(
        InteractionGroups {
            memberships: Group { bits: read_u32(ref s)? },
            filter: Group { bits: read_u32(ref s)? },
            test_mode: Serde::deserialize(ref s)?,
        },
    )
}

fn read_flags(ref s: Span<felt252>) -> Option<ColliderFlags> {
    Some(
        ColliderFlags {
            active_collision_types: ActiveCollisionTypes { bits: Serde::deserialize(ref s)? },
            collision_groups: read_groups(ref s)?,
            solver_groups: read_groups(ref s)?,
            active_hooks: ActiveHooks { bits: read_u32(ref s)? },
            active_events: ActiveEvents { bits: read_u32(ref s)? },
            enabled: Serde::deserialize(ref s)?,
        },
    )
}

fn read_one_way(ref s: Span<felt252>) -> Option<Box<Option<OneWayPlatform>>> {
    let tag: felt252 = Serde::deserialize(ref s)?;
    if tag == 1 {
        return Some(BoxTrait::new(None));
    }
    if tag != 0 {
        return None;
    }
    let platform = OneWayPlatform {
        local_up: read_vec2(ref s)?, cos_allowed_angle: read_fixed(ref s)?,
    };
    Some(BoxTrait::new(Some(platform)))
}

fn read_collider(ref s: Span<felt252>) -> Option<Collider> {
    Some(
        Collider {
            co_type: Serde::deserialize(ref s)?,
            shape: read_shape(ref s)?,
            mprops: read_collider_mass(ref s)?,
            changes: ColliderChanges { bits: read_u32(ref s)? },
            parent: read_parent(ref s)?,
            pos: ColliderPosition { pose: read_pose(ref s)? },
            material: ColliderMaterial {
                friction: read_fixed(ref s)?,
                restitution: read_fixed(ref s)?,
                friction_combine_rule: Serde::deserialize(ref s)?,
                restitution_combine_rule: Serde::deserialize(ref s)?,
            },
            flags: read_flags(ref s)?,
            contact_force_event_threshold: read_fixed(ref s)?,
            one_way: read_one_way(ref s)?,
            user_data: Serde::deserialize(ref s)?,
        },
    )
}

/// `ArenaState<Collider>`'s felts, the shapes by the basic codec.
pub(crate) fn read_colliders(ref s: Span<felt252>) -> Option<ArenaState<Collider>> {
    let generation = read_u32(ref s)?;
    let capacity = read_u32(ref s)?;
    let free_list = read_u32s(ref s)?.span();
    let len = read_u32(ref s)?;
    let mut entries = array![];
    let mut i = 0;
    while i != len {
        let handle = read_handle(ref s)?;
        entries.append((handle, read_collider(ref s)?));
        i += 1;
    }
    Some(ArenaState { generation, capacity, free_list, entries: entries.span() })
}

/// `ActiveSet`'s felts (also the answer of `rapier2d_classes`' `ActiveSetClass`).
pub fn read_active_set(ref s: Span<felt252>) -> Option<ActiveSet> {
    let valid = read_bool(ref s)?;
    let bodies = read_handles(ref s)?;
    let len = read_u32(ref s)?;
    let mut colliders = array![];
    let mut i = 0;
    while i != len {
        let handle = read_handle(ref s)?;
        colliders.append((handle, read_u32(ref s)?));
        i += 1;
    }
    let len = read_u32(ref s)?;
    let mut statics = array![];
    let mut i = 0;
    while i != len {
        let collider = read_handle(ref s)?;
        let aabb = Aabb { mins: read_vec2(ref s)?, maxs: read_vec2(ref s)? };
        statics.append(BroadPhaseProxy { collider, aabb, is_static: read_bool(ref s)? });
        i += 1;
    }
    Some(
        ActiveSet {
            valid,
            bodies,
            colliders,
            statics,
            pairs: read_u32s(ref s)?,
            sleeping: read_u32(ref s)?,
            force_events: read_bool(ref s)?,
            prediction: read_fixed(ref s)?,
        },
    )
}

/// `Vec2`'s felts (the gravity).
pub(crate) fn read_gravity(ref s: Span<felt252>) -> Option<Vec2> {
    read_vec2(ref s)
}

/// A `u32` (the version).
pub(crate) fn read_version(ref s: Span<felt252>) -> Option<u32> {
    read_u32(ref s)
}

/// The joint arena's bookkeeping: generation, capacity, free list, and the entry count.
pub(crate) fn read_joint_bookkeeping(ref s: Span<felt252>) -> Option<(u32, u32, Span<u32>, u32)> {
    let generation = read_u32(ref s)?;
    let capacity = read_u32(ref s)?;
    let free_list = read_u32s(ref s)?.span();
    Some((generation, capacity, free_list, read_u32(ref s)?))
}

/// Measured and rejected (CS7): the contact pairs by the same readers.
#[cfg(test)]
mod alternatives;
