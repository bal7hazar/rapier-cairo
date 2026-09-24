//! Rejected candidates and reference implementations (AGENTS.md §5), kept for re-ranking.
//!
//! `update_islands_metered`: the slow path behind a one-iteration `while` (metered call, AGENTS.md
//! §7) instead of a plain `if`. The loop machinery costs more than what it saves: the dict
//! creations of the slow path are runtime costs, not statically charged to the fast path
//! (`tests::gas_islands_*`).
//!
//! The sleep timer (lot SC), per moving body, Sierra gas | Cairo steps (`timer_tests::gas_timer_*`
//! minus `gas_setup_*`, default configuration; fall = translation far above the limit, rest = a
//! few ulp without turn, turn = a few ulp with a turn):
//!
//! * `update_sleep_timer_v1` (SL as shipped, the reference): fall 18.2k | 146, rest 18.6k | 150,
//!   turn 35.3k | 301;
//! * `update_sleep_timer_wide` (wide squares, no shortcut): fall 18.3k | 143, rest 18.8k | 147;
//! * `update_sleep_timer_captured` (shortcut, the loop captures the body): fall 14.3k | 111;
//! * `update_sleep_timer_rest_static` (shortcut, rest decided statically): fall 11.0k | 73, rest
//!   15.6k | 122;
//! * `update_sleep_timer` (**shipped**): fall 10.4k | 71, rest 18.9k | 160, turn 38.6k | 343.
//!
//! On the P3 scenes (`gas_step_*` net of `gas_setup_*`) `rest_static` costs 1.9k more per body on
//! free fall (+0.4 %) and saves 1.7k per body on the contact scenes (-0.02 %): rejected, the
//! target of the lot being the awake free-fall step.

use fixed::{Fixed, HALF, MAX, ONE, ZERO};
use glam::Vec2Trait;
use rapier_core::Handle;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_core::rigid_body::activation::DEFAULT_NORMALIZED_LINEAR_THRESHOLD;
use rapier_core::rigid_body::{RigidBodyActivationTrait, RigidBodyType};
use rapier_dynamics2d::joint::ImpulseJoint;
use rapier_dynamics2d::narrow_phase::ContactPair;
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet};
use rapier_math::is_norm2_lt;
use rapier_math::pose2::Pose2;
use super::{
    DEFAULT_DT, SleepCensus, dynamic_gate_slow, is_default_configuration, relative_pose_drift,
    translation_near, update_islands_slow,
};

/// `update_islands` with the slow path behind a one-iteration `while`.
pub fn update_islands_metered(
    ref bodies: RigidBodySet,
    pairs: Span<ContactPair>,
    dormant: Span<ContactPair>,
    joints: Span<(Handle, ImpulseJoint)>,
    entries: Span<(Handle, RigidBody)>,
    census: SleepCensus,
) -> (Span<(Handle, RigidBody)>, bool, bool) {
    let mut result = (entries, census.sleeping != 0, false);
    let mut pending = census.awake != 0 && (census.sleeping != 0 || census.eligible);
    while pending {
        result = update_islands_slow(ref bodies, pairs, dormant, joints, entries);
        pending = false;
    }
    result
}

/// `update_sleep_timer` as SL shipped it (the reference of `timer_tests`): the drift (square
/// root, chord) computed for every dynamic body that may sleep, then `dynamic_gate`'s three
/// products.
#[inline(always)]
pub fn update_sleep_timer_v1(ref body: RigidBody, previous: Pose2, params: IntegrationParameters) {
    let can_sleep = match body.body_type {
        RigidBodyType::Dynamic => {
            if body.activation.normalized_linear_threshold < ZERO {
                false
            } else {
                let angvel = body.vels.angvel;
                let sq_angvel = if angvel == ZERO {
                    ZERO
                } else {
                    angvel * angvel
                };
                let max_extent = body.mprops.max_extent;
                let drift = relative_pose_drift(previous, body.pos.position, max_extent);
                let linear_threshold = body.activation.normalized_linear_threshold
                    * params.length_unit;
                body.activation.angular_gate(sq_angvel, max_extent) && drift
                    * HALF < linear_threshold
                    * params.dt
            }
        },
        RigidBodyType::KinematicPositionBased |
        RigidBodyType::KinematicVelocityBased => body.vels.linvel == Vec2Trait::ZERO
            && body.vels.angvel == ZERO,
        RigidBodyType::Fixed => true,
    };
    body.activation.update_timer(can_sleep, params.dt);
}

/// Candidate "wide": no default-configuration shortcut. Without turn and with colliders the
/// gates are decided by wide squares (`is_norm2_lt`, `angular_gate_wide`), the general gate is
/// behind a one-iteration `while`. Every dynamic body pays the two products of the limit and the
/// wide tests: no gain over the reference on the fall, 2k less than it at rest
/// (`gas_timer_wide_*`).
#[inline(always)]
pub fn update_sleep_timer_wide(
    ref body: RigidBody, previous: Pose2, params: IntegrationParameters,
) {
    let can_sleep = match body.body_type {
        RigidBodyType::Dynamic => {
            if body.activation.normalized_linear_threshold < ZERO {
                false
            } else {
                let max_extent = body.mprops.max_extent;
                if max_extent > ZERO
                    && max_extent != MAX
                    && body.pos.position.rotation == previous.rotation {
                    let limit = body.activation.linear_limit(params.length_unit, params.dt);
                    body.activation.angular_gate_wide(body.vels.angvel)
                        && limit.raw > 0
                        && limit.raw < 0x4000000000000000
                        && is_norm2_lt(
                            body.pos.position.translation.x - previous.translation.x,
                            body.pos.position.translation.y - previous.translation.y,
                            Fixed { raw: limit.raw * 2 },
                        )
                } else {
                    let mut gate = false;
                    let mut pending = true;
                    while pending {
                        let angvel = body.vels.angvel;
                        let sq_angvel = if angvel == ZERO {
                            ZERO
                        } else {
                            angvel * angvel
                        };
                        let drift = relative_pose_drift(previous, body.pos.position, max_extent);
                        gate = body
                            .activation
                            .dynamic_gate(
                                params.length_unit, sq_angvel, max_extent, drift, params.dt,
                            );
                        pending = false;
                    }
                    gate
                }
            }
        },
        RigidBodyType::KinematicPositionBased |
        RigidBodyType::KinematicVelocityBased => body.vels.linvel == Vec2Trait::ZERO
            && body.vels.angvel == ZERO,
        RigidBodyType::Fixed => true,
    };
    body.activation.update_timer(can_sleep, params.dt);
}

/// Candidate "captured": the shipped shortcut (default configuration, far translation decided in
/// `felt252`), but the metered `while` reads `body` itself: the loop captures the whole
/// `RigidBody` (about 60 felts) and every body pays for passing it, entered or not
/// (`gas_timer_captured_fall`). Shipped: the loop gets only the eleven values it needs.
#[inline(always)]
pub fn update_sleep_timer_captured(
    ref body: RigidBody, previous: Pose2, params: IntegrationParameters,
) {
    let can_sleep = match body.body_type {
        RigidBodyType::Dynamic => {
            let mut can_sleep = false;
            let mut pending = true;
            if body.activation.normalized_linear_threshold == DEFAULT_NORMALIZED_LINEAR_THRESHOLD
                && params.length_unit == ONE
                && params.dt == DEFAULT_DT {
                let dx: felt252 = body.pos.position.translation.x.raw.into()
                    - previous.translation.x.raw.into();
                let dy: felt252 = body.pos.position.translation.y.raw.into()
                    - previous.translation.y.raw.into();
                pending = translation_near(dx, dy);
            }
            while pending {
                let angvel = body.vels.angvel;
                let sq_angvel = if angvel == ZERO {
                    ZERO
                } else {
                    angvel * angvel
                };
                let max_extent = body.mprops.max_extent;
                let drift = relative_pose_drift(previous, body.pos.position, max_extent);
                can_sleep = body
                    .activation
                    .dynamic_gate(params.length_unit, sq_angvel, max_extent, drift, params.dt);
                pending = false;
            }
            can_sleep
        },
        RigidBodyType::KinematicPositionBased |
        RigidBodyType::KinematicVelocityBased => body.vels.linvel == Vec2Trait::ZERO
            && body.vels.angvel == ZERO,
        RigidBodyType::Fixed => true,
    };
    body.activation.update_timer(can_sleep, params.dt);
}

/// Candidate "rest static": the shipped timer with the no-turn, at-rest case (default
/// configuration, translation below `2 · DEFAULT_LIMIT`, colliders) decided statically (rotation,
/// extent, `angular_gate_wide`) instead of in the metered `while`. A body at rest saves the loop
/// (1.7k less per body in the contact scenes), but every body pays for the extra tests: +1.9k on
/// free fall (`gas_step_free_fall32` +1.2 %), the target of the lot. Rejected.
#[inline(always)]
pub fn update_sleep_timer_rest_static(
    ref body: RigidBody, previous: Pose2, params: IntegrationParameters,
) {
    // `RigidBodyActivationTrait::update_energy` arm by arm, inlined: an outlined call is
    // charged its dearest path (the drift's square roots and division) for every body.
    let can_sleep = match body.body_type {
        RigidBodyType::Dynamic => {
            let position = body.pos.position;
            let dx: felt252 = position.translation.x.raw.into() - previous.translation.x.raw.into();
            let dy: felt252 = position.translation.y.raw.into() - previous.translation.y.raw.into();
            let mut can_sleep = false;
            let mut pending = true;
            let mut near = false;
            if is_default_configuration(
                body.activation.normalized_linear_threshold, params.length_unit, params.dt,
            ) {
                if translation_near(dx, dy) {
                    near = true;
                    if position.rotation == previous.rotation
                        && body.mprops.max_extent > ZERO
                        && body.mprops.max_extent != MAX {
                        // At rest without turn, with colliders: only the angular gate is left.
                        can_sleep = body.activation.angular_gate_wide(body.vels.angvel);
                        pending = false;
                    }
                } else {
                    pending = false;
                }
            } else if body.activation.normalized_linear_threshold < ZERO {
                // Cannot sleep.
                pending = false;
            }
            let threshold = body.activation.normalized_linear_threshold;
            let angular_threshold = body.activation.angular_threshold;
            let angvel = body.vels.angvel;
            let max_extent = body.mprops.max_extent;
            if pending {
                while pending {
                    can_sleep =
                        dynamic_gate_slow(
                            threshold,
                            angular_threshold,
                            angvel,
                            max_extent,
                            dx,
                            dy,
                            position.rotation,
                            previous.rotation,
                            params.length_unit,
                            params.dt,
                            near,
                        );
                    pending = false;
                }
            }
            can_sleep
        },
        RigidBodyType::KinematicPositionBased |
        RigidBodyType::KinematicVelocityBased => body.vels.linvel == Vec2Trait::ZERO
            && body.vels.angvel == ZERO,
        RigidBodyType::Fixed => true,
    };
    body.activation.update_timer(can_sleep, params.dt);
}
