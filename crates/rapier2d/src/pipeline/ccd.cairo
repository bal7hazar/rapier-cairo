//! Continuous collision detection (upstream `dynamics/ccd/ccd_solver.rs` and the CCD part of
//! `pipeline/physics_pipeline/substep.rs`), work package CC2.
//!
//! [`step_with_ccd`] is a step with upstream's CCD: the fast bodies sweep their colliders from
//! their pose at the start of the step to their solved pose, and a body that would pass through
//! a collider is stopped at its first time of impact (pose only, velocities untouched: the
//! speculative contacts of the next step resolve the approach). With `max_ccd_substeps > 1`, a
//! first impact found before the solve splits the step (upstream's substep splitter).
//!
//! A body is *fast* when it can move more than half its thinnest extent in one (sub)step
//! (`RigidBodyCcdTrait::is_moving_fast_with_next_position`). Who is examined:
//! * a dynamic body with `ccd_enabled` (a *bullet*) sweeps every collider but the bullets' ones;
//! * with [`CCDSolverTrait::set_automatic`] (upstream's behaviour, off by default, see below),
//!   every other fast dynamic body also sweeps the fixed colliders (parentless, or attached to a
//!   fixed body).
//!
//! Sensors (and pairs with mismatched solver groups) never stop a body: a sensor crossed before
//! the body's impact and seen neither at the start nor at the end pose emits a `Started` and a
//! `Stopped` event flagged `SENSOR`, `(fast collider, sensor)`, after the step's other events.
//!
//! # Where the pass runs
//!
//! `World::step` does not run CCD: its Cairo steps stay those of a world without CCD, to the
//! step. Measured alternatives: a one-branch gate in the step costs 7 steps per whole-path step
//! and moves the sparse path's code (+8 steps per tick); the CCD state as a `World` field costs
//! about 65 steps per tick (every `ref world` call copies it). Upstream passes its
//! `CCDSolver` to `PhysicsPipeline::step`; here [`step_with_ccd`] (and
//! [`step_with_ccd_and_force_events`], `WorldTrait::step_with_ccd`) take it. The pass runs after
//! the regular fused step, on the start poses it recorded before: the solved pose upstream clamps
//! before `advance_to_final_positions` is the advanced pose here, and a clamp rewrites the body's
//! pose, world centre of mass and colliders (untracked writes, as the step's own). Same results,
//! but for the sleep timer of a clamped body, which reads the unclamped displacement (it differs
//! only for a body slower than its sleep threshold).
//!
//! # Default: `ccd_enabled` bodies only (owner's rule, ADR 0001 entry requested)
//!
//! The pinned upstream activates CCD for every fast dynamic body. Measured on the G0 levels
//! (`tests/ccd_budget.cairo`), that sweeps the pebble against the fixed colliders on every flight
//! tick and examines every awake dynamic body of the P3 scenes; the default of
//! [`CCDSolverTrait::new`] is therefore `automatic = false`: only `ccd_enabled` bodies are
//! examined, and a world without one pays the cache check and one call (a few hundred steps).
//! `set_automatic(true)` restores upstream's behaviour.
//!
//! # Caches
//!
//! The solver keeps the handles of the bodies it examines and the fixed targets (upstream caches
//! the latter). Both are rebuilt when the body or collider set was written since the last
//! [`step_with_ccd`] (the sets' modified flags, which the CCD step leaves cleared) or changed
//! size; after writing a set and stepping it with `World::step` before the next CCD step, call
//! [`CCDSolverTrait::invalidate`].
//!
//! # Deviations from upstream
//!
//! * The pre-solve activation of the splitter reads the body's velocities (upstream: the last
//!   post-solve interpolated velocity, `ccd_vels`, which the port does not persist).
//! * Bullet targets are filtered by their end-of-step box (upstream: the broad-phase tree's box,
//!   from the start of the step); a sensor attached to a moving body that is not examined is
//!   taken at its end pose for the start-of-sweep intersection test.
//! * `ccd_thickness` is derived from the attached colliders at each pass (upstream caches the
//!   minimum at attachment and never raises it).
//! * No fixed-target list limit (upstream falls back to its tree past 512 targets; same results)
//!   and no hooks. Soft CCD (`soft_ccd_prediction`) is stored on the body only: its pipeline
//!   effect (enlarged broad-phase box, per-pair prediction distance in the narrow phase) is
//!   deferred.

use fixed::{Fixed, FixedTrait, HALF, ONE, TrigTrait, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_core::Handle;
use rapier_core::collider::events::{COLLISION_EVENTS, SENSOR};
use rapier_core::collider::{ActiveEventsTrait, ColliderType};
use rapier_core::integration_parameters::IntegrationParametersTrait;
use rapier_core::rigid_body::RigidBodyType;
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::events::{CollisionEvent, ContactForceEvent};
use rapier_dynamics2d::rigid_body::ccd::FAST_BODY_SAFETY_FACTOR;
use rapier_dynamics2d::rigid_body::{
    RigidBodyCcd, RigidBodyCcdTrait, RigidBodyMassPropsTrait, RigidBodyPosition,
    RigidBodyPositionTrait, RigidBodyVelocity,
};
use rapier_dynamics2d::rigid_body_set::ccd_api::{RigidBodyCcdApiTrait, body_ccd, wants_ccd};
use rapier_dynamics2d::rigid_body_set::{
    BodyPose, BodySleeping, RigidBody, RigidBodySet, RigidBodySetTrait,
};
use rapier_geometry2d::aabb::Aabb;
use rapier_geometry2d::dispatch::intersection::intersection_test;
use rapier_geometry2d::query::sweep::SweepTrait;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::world::World;
use super::active_set::ActiveSet;
use super::config::{DefaultStepConfig, StepConfig};
use super::force_events::{CollisionOnly, StepOutput, WithForces};
use super::stages::InProcessStages;
use super::{active_set, moving};
mod configured;
pub use configured::{step_with_ccd_and_force_events_with, step_with_ccd_with};
pub mod sweeps;
pub mod targets;
use sweeps::{
    FastCollider, PseudoHit, Target, body_thickness, collect_targets, fast_colliders, sweep_body,
};
use targets::{ActiveView, swept_region, targets_near};

#[cfg(test)]
mod benches;
#[cfg(test)]
mod tests;

/// The world's continuous-collision solver (upstream `CCDSolver`): the automatic-mode switch and
/// the caches of the pass. Owned by the caller of [`step_with_ccd`], as upstream's by the caller
/// of `PhysicsPipeline::step`.
#[derive(Copy, Drop, PartialEq, Debug)]
pub struct CCDSolver {
    /// Upstream's automatic tier: every fast dynamic body sweeps the fixed colliders.
    automatic: bool,
    /// The caches below describe the sets.
    built: bool,
    /// Body and collider counts the caches were built with.
    bodies_len: u32,
    colliders_len: u32,
    /// The bodies the pass examines, ascending slot: the dynamic `ccd_enabled` ones, and every
    /// dynamic one in the automatic mode.
    candidates: Span<Handle>,
    /// The bullets among them (dynamic, `ccd_enabled`).
    bullets: Span<Handle>,
    /// The fixed targets, boxes loosened by the prediction distance they carry.
    fixed_targets: Option<(Fixed, Span<Target>)>,
}

/// Serialized as its switch only (upstream serializes no cache either): a restored solver
/// rebuilds its caches at its first step.
pub impl CCDSolverSerde of Serde<CCDSolver> {
    fn serialize(self: @CCDSolver, ref output: Array<felt252>) {
        self.automatic.serialize(ref output);
    }

    fn deserialize(ref serialized: Span<felt252>) -> Option<CCDSolver> {
        let automatic: bool = Serde::deserialize(ref serialized)?;
        let mut solver = CCDSolverTrait::new();
        solver.automatic = automatic;
        Some(solver)
    }
}

/// Upstream `CCDSolver::default`: [`CCDSolverTrait::new`].
pub impl CCDSolverDefault of Default<CCDSolver> {
    #[inline(always)]
    fn default() -> CCDSolver {
        CCDSolverTrait::new()
    }
}

/// A body examined by the pass, with its pose at the start of the (sub)step.
#[derive(Copy, Drop, Debug, PartialEq)]
pub struct CcdStart {
    pub handle: Handle,
    pub pose: Pose2,
}

/// A fast body: its value when found fast, and its sweep's end poses.
#[derive(Copy, Drop, Debug)]
pub struct FastBody {
    pub handle: Handle,
    pub body: RigidBody,
    pub start: Pose2,
    pub end: Pose2,
}

#[generate_trait]
pub impl CCDSolverImpl of CCDSolverTrait {
    /// A solver with empty caches, `automatic` off (upstream `CCDSolver::new`; see the module
    /// documentation for the default).
    fn new() -> CCDSolver {
        CCDSolver {
            automatic: false,
            built: false,
            bodies_len: 0,
            colliders_len: 0,
            candidates: array![].span(),
            bullets: array![].span(),
            fixed_targets: None,
        }
    }

    /// Whether every fast dynamic body sweeps the fixed colliders (upstream's behaviour).
    #[inline(always)]
    fn automatic(self: @CCDSolver) -> bool {
        *self.automatic
    }

    /// Switches upstream's automatic tier on or off; drops the caches.
    fn set_automatic(ref self: CCDSolver, automatic: bool) {
        self.automatic = automatic;
        self.invalidate();
    }

    /// Discards the cached fixed-target list (upstream `invalidate_fixed_targets_cache`).
    fn invalidate_fixed_targets_cache(ref self: CCDSolver) {
        self.fixed_targets = None;
    }

    /// Discards every cache: the next step rescans the sets.
    fn invalidate(ref self: CCDSolver) {
        self.built = false;
        self.fixed_targets = None;
    }

    /// The bodies the pass examines (valid after a step).
    #[inline(always)]
    fn candidates(self: @CCDSolver) -> Span<Handle> {
        *self.candidates
    }

    /// Updates `ccd_active` of the examined awake dynamic bodies over `dt` and tells whether one
    /// is fast (upstream `update_ccd_active_flags`). With `include_forces`, the pre-solve test
    /// (`is_moving_fast` with the forces) on the current poses; otherwise the post-solve test on
    /// the motion from `starts` to the current poses.
    fn update_ccd_active_flags(
        ref self: CCDSolver,
        ref world: World,
        starts: Span<CcdStart>,
        dt: Fixed,
        include_forces: bool,
    ) -> bool {
        self.refresh(ref world);
        let (non_bullets, bullets) = if include_forces {
            activate_pre(ref world.bodies, ref world.colliders, self.candidates, dt)
        } else {
            activate_post(ref world.bodies, ref world.colliders, starts, dt)
        };
        !non_bullets.is_empty() || !bullets.is_empty()
    }

    /// The first time of impact in `[0, dt)` of a fast body predicted by its velocities and
    /// forces (upstream `find_first_impact`, the substep splitter): sensors and pseudo pairs are
    /// ignored; `None` without impact. Updates the `ccd_active` flags (pre-solve test).
    fn find_first_impact(ref self: CCDSolver, ref world: World, dt: Fixed) -> Option<Fixed> {
        self.refresh(ref world);
        first_impact(ref self, ref world, dt)
    }

    /// The poses at which the examined bodies start the coming (sub)step, from the caches as
    /// they are (call [`CCDSolverTrait::refresh`] first when the sets may have changed).
    #[inline(always)]
    fn starts(ref self: CCDSolver, ref world: World) -> Array<CcdStart> {
        let mut out = array![];
        for handle in self.candidates {
            if let Some(pose) = world.bodies.get_field::<Pose2, BodyPose>(*handle) {
                out.append(CcdStart { handle: *handle, pose });
            }
        }
        out
    }

    /// The continuous pass after a (sub)step of `world.integration_parameters.dt` that started
    /// at `starts` (upstream `update_ccd_active_flags` post-solve, then `solve_continuous`): the
    /// fast non-bullets sweep the fixed targets and are clamped, then the bullets sweep every
    /// non-bullet collider and are clamped; returns the sensor events.
    #[inline(always)]
    fn solve_continuous(
        ref self: CCDSolver, ref world: World, starts: Span<CcdStart>,
    ) -> Array<CollisionEvent> {
        let params = world.integration_parameters;
        let (non_bullets, bullets) = activate_post(
            ref world.bodies, ref world.colliders, starts, params.dt,
        );
        if non_bullets.is_empty() && bullets.is_empty() {
            return array![];
        }
        let slop = params.allowed_linear_error();
        let prediction = params.prediction_distance();
        let mut kept: Array<(PseudoHit, Pose2)> = array![];
        if !non_bullets.is_empty() {
            let targets = self.fixed_targets(ref world, prediction);
            let (fast, _) = prepare(ref world.colliders, non_bullets.span(), false);
            let mut i = 0;
            for body in non_bullets.span() {
                sweep_and_clamp(
                    ref world.bodies,
                    ref world.colliders,
                    body,
                    *fast.at(i),
                    false,
                    targets,
                    slop,
                    ref kept,
                );
                i += 1;
            }
        }
        if !bullets.is_empty() {
            let (fast, region) = prepare(ref world.colliders, bullets.span(), false);
            if let Some(region) = region {
                let targets = targets_near(
                    ref world.bodies,
                    ref world.colliders,
                    active_view(@world),
                    self.bullets,
                    prediction * HALF,
                    region,
                );
                let mut i = 0;
                for body in bullets.span() {
                    sweep_and_clamp(
                        ref world.bodies,
                        ref world.colliders,
                        body,
                        *fast.at(i),
                        true,
                        targets.span(),
                        slop,
                        ref kept,
                    );
                    i += 1;
                }
            }
        }
        sensor_events(ref world.colliders, kept.span(), starts)
    }

    /// The fixed targets, from the cache when it was built with `prediction`.
    fn fixed_targets(ref self: CCDSolver, ref world: World, prediction: Fixed) -> Span<Target> {
        if let Some((p, list)) = self.fixed_targets {
            if p == prediction {
                return list;
            }
        }
        let list = collect_targets(
            ref world.bodies, ref world.colliders, array![].span(), prediction, true,
        )
            .span();
        self.fixed_targets = Some((prediction, list));
        list
    }

    /// Rebuilds the stale caches (see the module documentation). The check is inlined; the walk
    /// of the bodies is out of line (`rebuild_candidates`).
    #[inline(always)]
    fn refresh(ref self: CCDSolver, ref world: World) {
        if !self.built || world.bodies.is_modified() || world.bodies.len() != self.bodies_len {
            rebuild_candidates(ref self, ref world);
        }
        if world.colliders.is_modified() || world.colliders.len() != self.colliders_len {
            self.colliders_len = world.colliders.len();
            self.fixed_targets = None;
        }
    }

    /// The end of a CCD step: the sets' modified flags cleared when the active set does not rely
    /// on them (it is invalid), so that the next step sees the user's writes only.
    #[inline(always)]
    fn settle(ref self: CCDSolver, ref world: World) {
        if !active_set::is_valid(@world) {
            world.bodies.clear_modified();
            world.colliders.clear_modified();
        }
        self.bodies_len = world.bodies.len();
        self.colliders_len = world.colliders.len();
    }
}

/// Upstream's post-solve test (`is_moving_fast_with_next_position` on the velocity interpolated
/// from `pos`, as `ccd_vels`), with the same result: the actual motion alone decides when it is
/// above the threshold (the test takes the larger of the two), and a motion without rotation
/// has no angular velocity, so the interpolation's `atan2` runs only when both are needed.
pub fn moving_fast(
    ccd: RigidBodyCcd,
    dt: Fixed,
    inv_dt: Fixed,
    pos: RigidBodyPosition,
    local_com: Vec2,
    max_extent: Fixed,
) -> bool {
    let linear = pos.next_position.transform_point(local_com)
        - pos.position.transform_point(local_com);
    let delta_rot = pos.next_position.rotation * pos.position.rotation.inverse();
    let threshold = FAST_BODY_SAFETY_FACTOR * ccd.ccd_thickness;
    if linear.length() + delta_rot.im.abs() * max_extent > threshold {
        return true;
    }
    let angle = if delta_rot.im == ZERO && delta_rot.re > ZERO {
        ZERO
    } else {
        delta_rot.im.atan2(delta_rot.re)
    };
    let vels = RigidBodyVelocity { linvel: linear.mul_scalar(inv_dt), angvel: angle * inv_dt };
    ccd.max_point_velocity(vels, max_extent) * dt > threshold
}

/// [`CCDSolverTrait::find_first_impact`] on the caches as they are (the step built them).
fn first_impact(ref solver: CCDSolver, ref world: World, dt: Fixed) -> Option<Fixed> {
    let (non_bullets, bullets) = activate_pre(
        ref world.bodies, ref world.colliders, solver.candidates, dt,
    );
    if non_bullets.is_empty() && bullets.is_empty() {
        return None;
    }
    let params = world.integration_parameters;
    let slop = params.allowed_linear_error();
    let (fast_nb, _) = prepare(ref world.colliders, non_bullets.span(), true);
    let (fast_b, _) = prepare(ref world.colliders, bullets.span(), true);
    let mut all = array![];
    all.append_span(fast_nb.span());
    all.append_span(fast_b.span());
    let Some(region) = swept_region(all.span()) else {
        return None;
    };
    let targets = targets_near(
        ref world.bodies,
        ref world.colliders,
        active_view(@world),
        solver.bullets,
        params.prediction_distance() * HALF,
        region,
    );
    let mut unused = array![];
    let mut min = ONE;
    let mut i = 0;
    for fast in non_bullets.span() {
        let f = sweep_body(
            *fast_nb.at(i), *fast.handle, false, targets.span(), slop, false, ref unused,
        );
        if f < min {
            min = f;
        }
        i += 1;
    }
    let mut i = 0;
    for fast in bullets.span() {
        let f = sweep_body(
            *fast_b.at(i), *fast.handle, true, targets.span(), slop, false, ref unused,
        );
        if f < min {
            min = f;
        }
        i += 1;
    }
    if min < ONE {
        Some(min * dt)
    } else {
        None
    }
}

/// What the targets read from the world's active set (`targets::ActiveView`).
#[inline(always)]
fn active_view(world: @World) -> ActiveView {
    let set: @ActiveSet = world.active_set.as_snapshot().unbox();
    ActiveView {
        valid: *set.valid,
        prediction: *set.prediction,
        statics: set.statics.span(),
        awake: set.colliders.span(),
        bodies: set.bodies.span(),
    }
}

/// The candidate and bullet lists, rebuilt from every body (see [`CCDSolverTrait::refresh`]).
#[inline(never)]
fn rebuild_candidates(ref solver: CCDSolver, ref world: World) {
    let mut candidates = array![];
    let mut bullets = array![];
    for (handle, body) in world.bodies.iter() {
        if body.body_type == RigidBodyType::Dynamic {
            if wants_ccd(@body) {
                candidates.append(handle);
                bullets.append(handle);
            } else if solver.automatic {
                candidates.append(handle);
            }
        }
    }
    solver.candidates = candidates.span();
    solver.bullets = bullets.span();
    solver.bodies_len = world.bodies.len();
    solver.built = true;
    solver.fixed_targets = None;
}

/// The fast bodies among the awake dynamic `candidates` by the pre-solve test over `dt`
/// (velocities advanced by the forces), `(non-bullets, bullets)`, each with its predicted pose
/// (upstream `integrate_forces_and_velocities`); `ccd_active` and `ccd_thickness` written back.
fn activate_pre(
    ref bodies: RigidBodySet, ref colliders: ColliderSet, candidates: Span<Handle>, dt: Fixed,
) -> (Array<FastBody>, Array<FastBody>) {
    let mut non_bullets = array![];
    let mut bullets = array![];
    for handle in candidates {
        let Some(mut body) = bodies.get(*handle) else {
            continue;
        };
        if !moving(@body) || body.body_type != RigidBodyType::Dynamic {
            continue;
        }
        let stored = body_ccd(@body);
        let mut ccd = stored;
        ccd.ccd_thickness = body_thickness(body.colliders, ref colliders);
        let fast = ccd.is_moving_fast(dt, body.vels, Some(body.forces), body.mprops.max_extent);
        ccd.ccd_active = fast;
        if ccd != stored {
            body.set_ccd(ccd);
            let _ = bodies.set_internal(*handle, body);
        }
        if fast {
            let end = body
                .pos
                .predict_position_using_velocity_and_forces(
                    dt, body.forces, body.vels, body.mprops,
                );
            let entry = FastBody { handle: *handle, body, start: body.pos.position, end };
            if ccd.ccd_enabled {
                bullets.append(entry);
            } else {
                non_bullets.append(entry);
            }
        }
    }
    (non_bullets, bullets)
}

/// The fast bodies among `starts` by the post-solve test over `dt` (the motion from the start
/// pose to the current one, velocity interpolated from it as upstream's `ccd_vels`),
/// `(non-bullets, bullets)`; `ccd_active` and `ccd_thickness` written back. Bodies asleep,
/// disabled, removed or no longer dynamic are left as they are (upstream walks the active bodies).
fn activate_post(
    ref bodies: RigidBodySet, ref colliders: ColliderSet, starts: Span<CcdStart>, dt: Fixed,
) -> (Array<FastBody>, Array<FastBody>) {
    let inv_dt = rapier_math::math_ext::scalar::inv(dt);
    let mut non_bullets = array![];
    let mut bullets = array![];
    for start in starts {
        // A body still asleep is skipped on one field read.
        if bodies.get_field::<bool, BodySleeping>(*start.handle) != Some(false) {
            continue;
        }
        let Some(mut body) = bodies.get(*start.handle) else {
            continue;
        };
        if !moving(@body) || body.body_type != RigidBodyType::Dynamic {
            continue;
        }
        let stored = body_ccd(@body);
        let mut ccd = stored;
        ccd.ccd_thickness = body_thickness(body.colliders, ref colliders);
        let pos = RigidBodyPosition { position: *start.pose, next_position: body.pos.position };
        let fast = moving_fast(
            ccd, dt, inv_dt, pos, body.mprops.local_mprops.local_com, body.mprops.max_extent,
        );
        ccd.ccd_active = fast;
        if ccd != stored {
            body.set_ccd(ccd);
            let _ = bodies.set_internal(*start.handle, body);
        }
        if fast {
            let entry = FastBody {
                handle: *start.handle, body, start: *start.pose, end: body.pos.position,
            };
            if ccd.ccd_enabled {
                bullets.append(entry);
            } else {
                non_bullets.append(entry);
            }
        }
    }
    (non_bullets, bullets)
}

/// The fast colliders of each of `fast` (sensors skipped with `skip_sensors`) and the union of
/// their swept boxes.
fn prepare(
    ref colliders: ColliderSet, fast: Span<FastBody>, skip_sensors: bool,
) -> (Array<Span<FastCollider>>, Option<Aabb>) {
    let mut out = array![];
    for body in fast {
        let local_com = *body.body.mprops.local_mprops.local_com;
        out
            .append(
                fast_colliders(
                    *body.body.colliders,
                    ref colliders,
                    *body.start,
                    *body.end,
                    local_com,
                    skip_sensors,
                )
                    .span(),
            );
    }
    let region = swept_region(out.span());
    (out, region)
}

/// Sweeps one fast body, clamps it to its impact (upstream `apply_clamps`) and keeps its pseudo
/// hits that happen strictly before the impact, with the body's final pose.
fn sweep_and_clamp(
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    fast: @FastBody,
    swept: Span<FastCollider>,
    bullet: bool,
    targets: Span<Target>,
    slop: Fixed,
    ref kept: Array<(PseudoHit, Pose2)>,
) {
    let mut pseudo = array![];
    let fraction = sweep_body(swept, *fast.handle, bullet, targets, slop, true, ref pseudo);
    let end = if fraction < ONE {
        clamp(ref bodies, ref colliders, fast, fraction)
    } else {
        *fast.end
    };
    for hit in pseudo.span() {
        if *hit.fraction < fraction {
            kept.append((*hit, end));
        }
    }
}

/// Moves the body to `Sweep::from_poses(start, end, local_com).transform_at(fraction)`: pose,
/// world mass properties and collider poses (untracked writes). Returns the clamped pose.
fn clamp(
    ref bodies: RigidBodySet, ref colliders: ColliderSet, fast: @FastBody, fraction: Fixed,
) -> Pose2 {
    let Some(mut body) = bodies.get(*fast.handle) else {
        return *fast.end;
    };
    let sweep = SweepTrait::from_poses(*fast.start, *fast.end, body.mprops.local_mprops.local_com);
    let pose = sweep.transform_at(fraction);
    body.pos.position = pose;
    body.pos.next_position = pose;
    body.mprops = body.mprops.update_world_mass_properties(body.body_type, pose);
    let _ = bodies.set_internal(*fast.handle, body);
    for handle in body.colliders {
        if let Some(mut collider) = colliders.get(*handle) {
            if let Some(parent) = collider.parent {
                collider.pos.pose = pose * parent.pos_wrt_parent;
                let _ = colliders.set_internal(*handle, collider);
            }
        }
    }
    pose
}

/// The sensor crossings the narrow phase never sees (upstream `solve_continuous`, last loop):
/// for each kept pseudo hit with a sensor on either side and `COLLISION_EVENTS` on either
/// collider, a `Started` then a `Stopped` event flagged `SENSOR` when the shapes intersect
/// neither at the start nor at the end of the sweep.
fn sensor_events(
    ref colliders: ColliderSet, kept: Span<(PseudoHit, Pose2)>, starts: Span<CcdStart>,
) -> Array<CollisionEvent> {
    let mut events = array![];
    for entry in kept {
        let (hit, end1) = *entry;
        let Some(co1) = colliders.get(hit.ch1) else {
            continue;
        };
        let Some(co2) = colliders.get(hit.ch2) else {
            continue;
        };
        if co1.co_type != ColliderType::Sensor && co2.co_type != ColliderType::Sensor {
            continue;
        }
        if !(co1.flags.active_events | co2.flags.active_events).contains(COLLISION_EVENTS) {
            continue;
        }
        let start2 = match co2.parent {
            Some(parent) => {
                let mut pose = co2.pos.pose;
                for start in starts {
                    if *start.handle == parent.handle {
                        pose = *start.pose * parent.pos_wrt_parent;
                    }
                }
                pose
            },
            None => co2.pos.pose,
        };
        let before = intersection_test(hit.start1.inv_mul(start2), co1.shape, co2.shape);
        let next1 = end1 * hit.pos_wrt_parent1;
        let after = intersection_test(next1.inv_mul(co2.pos.pose), co1.shape, co2.shape);
        if before != Some(true) && after != Some(true) {
            events.append(CollisionEvent::Started((hit.ch1, hit.ch2, SENSOR)));
            events.append(CollisionEvent::Stopped((hit.ch1, hit.ch2, SENSOR)));
        }
    }
    events
}

/// One step of `world` with continuous collision detection (see the module documentation);
/// returns the collision events (the step's, then the sensor events of the pass, per substep).
/// `max_ccd_substeps == 0` is `World::step`.
///
/// # Panics
/// As `World::step`, and the overflow panics of the sweeps.
pub fn step_with_ccd(ref world: World, ref ccd_solver: CCDSolver) -> Array<CollisionEvent> {
    if world.integration_parameters.max_ccd_substeps == 0 {
        return super::step(ref world);
    }
    ccd_solver.refresh(ref world);
    if ccd_solver.candidates.is_empty() {
        let output = super::step(ref world);
        ccd_solver.settle(ref world);
        return output;
    }
    step_ccd::<Array<CollisionEvent>, CollisionOnly, DefaultStepConfig>(ref world, ref ccd_solver)
}

/// [`step_with_ccd`] that also returns the post-solver contact-force events of every substep.
pub fn step_with_ccd_and_force_events(
    ref world: World, ref ccd_solver: CCDSolver,
) -> (Array<CollisionEvent>, Array<ContactForceEvent>) {
    if world.integration_parameters.max_ccd_substeps == 0 {
        return super::step_with_force_events(ref world);
    }
    ccd_solver.refresh(ref world);
    if ccd_solver.candidates.is_empty() {
        let output = super::step_with_force_events(ref world);
        ccd_solver.settle(ref world);
        return output;
    }
    step_ccd::<
        (Array<CollisionEvent>, Array<ContactForceEvent>), WithForces, DefaultStepConfig,
    >(ref world, ref ccd_solver)
}

/// Upstream's substep loop (`PhysicsPipeline::step`, `substep.rs`) around the regular step, for
/// a world with bodies to examine (the entry points take the regular step directly otherwise).
fn step_ccd<T, impl Output: StepOutput<T>, impl C: StepConfig, +Drop<T>>(
    ref world: World, ref solver: CCDSolver,
) -> T {
    let params = world.integration_parameters;
    let mut remaining_time = params.dt;
    let mut remaining = params.max_ccd_substeps;
    let dt = next_dt(ref solver, ref world, ref remaining_time, ref remaining, params.min_ccd_dt);
    let mut output = substep::<T, Output, C>(ref world, ref solver, dt);
    while remaining != 0 {
        let dt = next_dt(
            ref solver, ref world, ref remaining_time, ref remaining, params.min_ccd_dt,
        );
        let step = substep::<T, Output, C>(ref world, ref solver, dt);
        output = Output::merge(output, step);
    }
    world.integration_parameters = params;
    solver.settle(ref world);
    output
}

/// The length of the next substep (upstream's splitter): with more than one substep left, the
/// first impact of a fast body predicted over the remaining time splits it (`remaining` counts
/// down), or the rest of the step is taken at once when there is none; a remainder at or under
/// `min_ccd_dt` joins the substep.
fn next_dt(
    ref solver: CCDSolver,
    ref world: World,
    ref remaining_time: Fixed,
    ref remaining: u32,
    min_ccd_dt: Fixed,
) -> Fixed {
    let mut dt = remaining_time;
    if remaining > 1 {
        match first_impact(ref solver, ref world, remaining_time) {
            Some(toi) => {
                let n = FixedTrait::from_int(remaining.try_into().unwrap());
                let interval = remaining_time / n;
                dt = if toi < interval {
                    interval
                } else {
                    toi + (remaining_time - toi) / n
                };
                remaining -= 1;
            },
            None => { remaining = 0; },
        }
        remaining_time = remaining_time - dt;
        if remaining_time <= min_ccd_dt {
            dt = dt + remaining_time;
            remaining = 0;
        }
    } else {
        remaining_time = ZERO;
        remaining = 0;
    }
    dt
}

/// One substep of `dt`: the start poses, the regular step, then the continuous pass; its sensor
/// events follow the step's.
fn substep<T, impl Output: StepOutput<T>, impl C: StepConfig, +Drop<T>>(
    ref world: World, ref solver: CCDSolver, dt: Fixed,
) -> T {
    world.integration_parameters.dt = dt;
    let starts = solver.starts(ref world);
    let step = super::step_internal::<T, Output, C, InProcessStages<C>>(ref world);
    let events = solver.solve_continuous(ref world, starts.span());
    Output::with_events(step, events)
}
