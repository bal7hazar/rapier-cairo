//! The sweep machinery of the CCD pass (upstream `dynamics/ccd/sweeps.rs`): the fast colliders of
//! a body, the targets they sweep against, the per-pair swept time of impact
//! (`rapier_geometry2d::query::sweep::sweep_time_of_impact`, CC1) and the per-body sweep.
//!
//! The closed shape set has no composite shape and no shape that is never swept: every shape but
//! the half-space has a point-cloud proxy (`ToiProxyTrait::from_shape`). Upstream sends a pair
//! without proxy (a half-space on either side) to the nonlinear shape cast, which does not support
//! the half-space (`cast_shapes_nonlinear` answers `Unsupported`): such a pair never hits, here as
//! upstream, and is skipped before any cast.

use fixed::{Fixed, ONE, ZERO};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::collider::{ActiveEvents, ColliderEnabled, ColliderType};
use rapier_core::interaction_groups::{InteractionGroups, InteractionGroupsTrait};
use rapier_core::rigid_body::RigidBodyType;
use rapier_dynamics2d::collider::Collider;
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::rigid_body_set::{BodyType, RigidBodySet, RigidBodySetTrait};
use rapier_geometry2d::aabb::{Aabb, AabbTrait};
use rapier_geometry2d::query::nonlinear_shape_cast::ccd_thickness;
use rapier_geometry2d::query::sweep::{
    Sweep, SweepToiStatus, SweepTrait, ToiProxy, ToiProxyTrait, sweep_time_of_impact,
};
use rapier_geometry2d::shape::{Shape, ShapeTrait};
use rapier_math::pose2::{Pose2, Pose2Trait};

/// Upstream `parry::query::sweep_toi::CORE_FRACTION` (`1/4`): the radius of the core ball of the
/// initial-overlap retry, as a fraction of the fast shape's thickness.
pub const CORE_FRACTION: Fixed = Fixed { raw: 0x40000000 };

/// One convex collider of a fast body, described once per step (upstream `FastColliderInfo` with
/// its `FastSubShape`).
#[derive(Copy, Drop, Debug)]
pub struct FastCollider {
    pub handle: Handle,
    pub proxy: ToiProxy,
    /// The collider's sweep, rotating about the body's centre of mass.
    pub sweep: Sweep,
    /// Union of the shape's boxes at the start and end poses.
    pub swept_aabb: Aabb,
    pub sensor: bool,
    pub collision_groups: InteractionGroups,
    pub solver_groups: InteractionGroups,
    pub active_events: ActiveEvents,
    pub shape: Shape,
    /// The collider's pose at the start of the sweep.
    pub start: Pose2,
    /// The collider's local pose in its body.
    pub pos_wrt_parent: Pose2,
}

/// A collider a fast body may hit, at its end-of-step pose (upstream reads the target's
/// `next_position`; stationary during the sweep).
#[derive(Copy, Drop, Debug, PartialEq)]
pub struct Target {
    pub handle: Handle,
    pub shape: Shape,
    pub pose: Pose2,
    /// Candidate filter: the shape's box loosened by the pass's margin.
    pub aabb: Aabb,
    pub body: Option<Handle>,
    /// Parentless, or attached to a fixed or missing body (upstream `is_fixed_target`).
    pub fixed: bool,
    /// Attached to a bullet (a dynamic `ccd_enabled` body).
    pub bullet: bool,
    pub sensor: bool,
    pub collision_groups: InteractionGroups,
    pub solver_groups: InteractionGroups,
    pub active_events: ActiveEvents,
}

/// A hit against a sensor, or across mismatched solver groups (upstream `PseudoHit`): recorded
/// for the sensor events, never clamps.
#[derive(Copy, Drop, Debug, PartialEq)]
pub struct PseudoHit {
    pub ch1: Handle,
    pub ch2: Handle,
    pub fraction: Fixed,
    /// Start pose of the fast collider.
    pub start1: Pose2,
    /// Local pose of the fast collider in its body.
    pub pos_wrt_parent1: Pose2,
}

/// `handle` is one of `handles`.
pub fn contains(handles: Span<Handle>, handle: Handle) -> bool {
    for h in handles {
        if *h == handle {
            return true;
        }
    }
    false
}

/// The targets among the enabled colliders of `colliders` (ascending slot): all of them, or only
/// the fixed ones with `fixed_only`; boxes loosened by `margin`. `bullets` are the handles of
/// the bullet bodies.
pub fn collect_targets(
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    bullets: Span<Handle>,
    margin: Fixed,
    fixed_only: bool,
) -> Array<Target> {
    let mut out = array![];
    for (handle, collider) in colliders.iter() {
        if collider.flags.enabled != ColliderEnabled::Enabled {
            continue;
        }
        let target = target_with_parent(ref bodies, handle, collider, bullets, margin);
        if fixed_only && !target.fixed {
            continue;
        }
        out.append(target);
    }
    out
}

/// The [`Target`] of one collider, its parent's type read from `bodies` (fixed when absent or
/// missing), bullet when its parent is one of `bullets`.
pub fn target_with_parent(
    ref bodies: RigidBodySet,
    handle: Handle,
    collider: Collider,
    bullets: Span<Handle>,
    margin: Fixed,
) -> Target {
    let body = match collider.parent {
        Some(parent) => Some(parent.handle),
        None => None,
    };
    let (fixed, bullet) = match body {
        Some(h) => (
            match bodies.get_field::<RigidBodyType, BodyType>(h) {
                Some(body_type) => body_type == RigidBodyType::Fixed,
                None => true,
            },
            contains(bullets, h),
        ),
        None => (true, false),
    };
    target_of(handle, collider, body, fixed, bullet, margin)
}

/// The [`Target`] of one collider.
fn target_of(
    handle: Handle,
    collider: Collider,
    body: Option<Handle>,
    fixed: bool,
    bullet: bool,
    margin: Fixed,
) -> Target {
    let pose = collider.pos.pose;
    Target {
        handle,
        shape: collider.shape,
        pose,
        aabb: collider.shape.compute_aabb(pose).loosened(margin),
        body,
        fixed,
        bullet,
        sensor: collider.co_type == ColliderType::Sensor,
        collision_groups: collider.flags.collision_groups,
        solver_groups: collider.flags.solver_groups,
        active_events: collider.flags.active_events,
    }
}

/// The smallest `ccd_thickness` of the colliders attached to a body ([`NO_THICKNESS`] without
/// any; upstream keeps the running minimum of the attached shapes).
///
/// [`NO_THICKNESS`]: rapier_dynamics2d::rigid_body::ccd::NO_THICKNESS
pub fn body_thickness(attached: Span<Handle>, ref colliders: ColliderSet) -> Fixed {
    let mut thickness = rapier_dynamics2d::rigid_body::ccd::NO_THICKNESS;
    for handle in attached {
        if let Some(collider) = colliders.get(*handle) {
            let t = ccd_thickness(collider.shape);
            if t < thickness {
                thickness = t;
            }
        }
    }
    thickness
}

/// The fast colliders of a body moving from `start` to `end` (body poses) about `local_com`
/// (upstream `FastColliderInfo::new` for each collider of `rb1.colliders`, disabled ones
/// included as upstream); sensors skipped with `skip_sensors` (the substep splitter), colliders
/// without proxy (half-spaces) skipped (see the module documentation).
pub fn fast_colliders(
    attached: Span<Handle>,
    ref colliders: ColliderSet,
    start: Pose2,
    end: Pose2,
    local_com: Vec2,
    skip_sensors: bool,
) -> Array<FastCollider> {
    let mut out = array![];
    for handle in attached {
        let Some(collider) = colliders.get(*handle) else {
            continue;
        };
        let Some(parent) = collider.parent else {
            continue;
        };
        let sensor = collider.co_type == ColliderType::Sensor;
        if skip_sensors && sensor {
            continue;
        }
        let Some(proxy) = ToiProxyTrait::from_shape(collider.shape) else {
            continue;
        };
        let s = start * parent.pos_wrt_parent;
        let e = end * parent.pos_wrt_parent;
        let lc = parent.pos_wrt_parent.inverse_transform_point(local_com);
        out
            .append(
                FastCollider {
                    handle: *handle,
                    proxy,
                    sweep: SweepTrait::from_poses(s, e, lc),
                    swept_aabb: collider
                        .shape
                        .compute_aabb(s)
                        .merged(collider.shape.compute_aabb(e)),
                    sensor,
                    collision_groups: collider.flags.collision_groups,
                    solver_groups: collider.flags.solver_groups,
                    active_events: collider.flags.active_events,
                    shape: collider.shape,
                    start: s,
                    pos_wrt_parent: parent.pos_wrt_parent,
                },
            );
    }
    out
}

/// The accepted impact fraction of `fast` against a target (upstream `cast_sub_shape` for a
/// proxy target): a solid pair stops only at `0 < fraction < max_fraction`, retrying an initial
/// overlap (fraction 0) with the core ball of radius `CORE_FRACTION · min_extent` about the
/// centroid; a pseudo pair reports any `Hit`, `Failed` or `Overlapped` fraction up to
/// `max_fraction`. `None` for a target without proxy (a half-space).
pub fn cast_pair(
    fast: @FastCollider,
    shape2: Shape,
    pose2: Pose2,
    max_fraction: Fixed,
    linear_slop: Fixed,
    is_pseudo: bool,
) -> Option<Fixed> {
    let target_proxy = ToiProxyTrait::from_shape(shape2)?;
    let target_sweep = SweepTrait::constant(pose2, Vec2 { x: ZERO, y: ZERO });
    let output = sweep_time_of_impact(
        target_proxy, target_sweep, *fast.proxy, *fast.sweep, max_fraction, linear_slop,
    );
    if is_pseudo {
        return match output.status {
            SweepToiStatus::Separated => None,
            _ => if output.fraction <= max_fraction {
                Some(output.fraction)
            } else {
                None
            },
        };
    }
    if ZERO < output.fraction && output.fraction < max_fraction {
        return Some(output.fraction);
    }
    if output.fraction == ZERO {
        // The core ball's centre and radius (upstream precomputes them per fast collider).
        let shape = *fast.shape;
        let core = ToiProxyTrait::point(
            shape.mass_properties(ONE).local_com, CORE_FRACTION * ccd_thickness(shape),
        );
        let output = sweep_time_of_impact(
            target_proxy, target_sweep, core, *fast.sweep, max_fraction, linear_slop,
        );
        if ZERO < output.fraction && output.fraction < max_fraction {
            return Some(output.fraction);
        }
    }
    None
}

/// The earliest solid impact fraction (`1` when free) of a body whose fast colliders are `fast`
/// against `targets` (upstream `sweep_fast_body`): candidates are the targets whose box meets a
/// fast collider's swept box, minus the collider itself and the body's own colliders; a bullet
/// sweeps every target but the bullets' colliders, another body the fixed targets only; the
/// collision groups must match. Pseudo hits (a sensor on either side, mismatched solver groups)
/// are appended to `pseudo` when `record_pseudo`, skipped otherwise.
pub fn sweep_body(
    fast: Span<FastCollider>,
    body: Handle,
    bullet: bool,
    targets: Span<Target>,
    linear_slop: Fixed,
    record_pseudo: bool,
    ref pseudo: Array<PseudoHit>,
) -> Fixed {
    let mut fraction = ONE;
    for fc in fast {
        for target in targets {
            if !target.aabb.intersects(*fc.swept_aabb) {
                continue;
            }
            if *target.handle == *fc.handle || *target.body == Some(body) {
                continue;
            }
            let allowed = if bullet {
                !*target.bullet
            } else {
                *target.fixed
            };
            if !allowed || !fc.collision_groups.test(*target.collision_groups) {
                continue;
            }
            let is_pseudo = *fc.sensor
                || *target.sensor
                || !fc.solver_groups.test(*target.solver_groups);
            if is_pseudo && !record_pseudo {
                continue;
            }
            if let Some(hit) =
                cast_pair(fc, *target.shape, *target.pose, fraction, linear_slop, is_pseudo) {
                if is_pseudo {
                    pseudo
                        .append(
                            PseudoHit {
                                ch1: *fc.handle,
                                ch2: *target.handle,
                                fraction: hit,
                                start1: *fc.start,
                                pos_wrt_parent1: *fc.pos_wrt_parent,
                            },
                        );
                } else {
                    fraction = hit;
                }
            }
        }
    }
    fraction
}
