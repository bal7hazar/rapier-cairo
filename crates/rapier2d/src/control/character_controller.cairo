//! The kinematic character controller (upstream `control/character_controller.rs`), 2D: walking,
//! sliding along walls and slopes, climbing steps and snapping to the ground by casting the
//! character's shape through the scene (CC1's `QueryPipeline::cast_shape`).
//!
//! Upstream borrows a `QueryPipeline` (bodies, colliders, BVH, filter). The port's
//! [`QueryPipeline`] is the filter bundle, so every method also takes the world by `ref`
//! (`controller.move_shape(dt, ref world, queries, shape, pos, desired, ref events)`); the world is
//! only read, except by [`KinematicCharacterControllerTrait::solve_character_collision_impulses`],
//! which writes the impulses to the bodies it pushes.
//!
//! # Semantics kept from upstream (rapier2d-f64 0.35.3)
//!
//! * The same passes, in the same order, with the same options: depenetration when the desired
//!   translation is (almost) zero, grounded status at the start, up to 20 cast-and-slide
//!   iterations (each: cast, collision event, stairs or slopes, grounded status with the
//!   kinematic platforms' friction), the grounded status when not moving, the snap to the ground.
//! * The 2D slope classification is upstream's: `angle_with_floor = up.angle_to(normal)`, glam's
//!   **signed** angle, so a surface whose normal leans to the right of `up` has a negative angle
//!   and is never a wall (never auto-stepped, never blocked from climbing). Ported as is.
//! * `decompose_hit` has no horizontal tangent in 2D (upstream's `Vector::ZERO` direction): the
//!   whole tangent is vertical.
//! * The internal constants (`1e-5` normalisation threshold, `1e-3` grounded cosine, `0.05` ground
//!   prediction, `0.25` depenetration budget, `1 + 1e-5` surface correction) are their nearest
//!   Q32.32 values.
//!
//! # Deviations
//!
//! * The collision callback is an array: `move_shape` appends every [`CharacterCollision`] to
//!   `events`, in upstream's callback order.
//! * The scene scan is the port's brute force in ascending collider handle order (upstream walks
//! its
//!   BVH): with several overlapping colliders, the depenetration pushes and the platforms' friction
//!   are accumulated in that order.
//! * Contact manifolds come from `rapier_geometry2d::dispatch` (convex pairs) and
//!   `dispatch::composite` (composite pairs): segment–segment and segment–capsule have no
//!   generator there (upstream: GJK-based PFM–PFM), so a segment character never counts as
//!   grounded on a segment or a capsule.
//! * Golden (`tests/control_golden.cairo`, 20 moves): one known divergence, `wall_slide`. After a
//!   wall hit the move left is exactly parallel to the ground the character touches; upstream's
//!   contact normal is exactly `(0, 1)` there and `normal · velocity >= 0` drops the hit, CC1's
//!   ball–cuboid start normal is tilted by 10–20 raw, so the port counts one more ground hit
//!   (and its `1e-4` nudge up).
//!
//! # Candidates
//!
//! Measured with the `gas_*` probes of `tests` and `tests_scenes` (Sierra gas net of the module's
//! `gas_baseline` or of the scene's `gas_setup_*`; Cairo steps in parentheses); the losers live in
//! `alternatives`.
//!
//! * The slope classification: glam's `angle_to` (one `atan2` per hit, **shipped**): 43,010
//!   (335) against a cosine comparison (`alternatives::hit_info_cosine`, a `cos` per threshold
//!   and a `sqrt`): 68,930 (543).
//! * The contact manifolds of the grounded status: the step's plain table
//!   (`dispatch::contact_manifold_step`, **shipped**) against the metered one
//!   (`alternatives::manifolds_metered`): alone 124,860 (1,025) against 147,870 (1,252); inside the
//!   moves (flat / slope / step / wall) 3.15M / 6.48M / 17.28M / 8.31M (26,000 / 50,642 /
//!   131,463 / 63,940) against 3.22M / 6.60M / 17.78M / 8.47M (26,681 / 51,777 / 133,548 /
//!   65,529).

use fixed::{FRAC_PI_4, Fixed, ONE, ZERO};
use glam_core::vec2::{Vec2, Vec2Trait};
use rapier_core::Handle;
use rapier_geometry2d::aabb::AabbTrait;
use rapier_geometry2d::query::ShapeCastHit;
use rapier_geometry2d::shape::{Shape, ShapeTrait};
use rapier_math::pose2::Pose2;
use crate::queries::pipeline::QueryPipeline;
use crate::world::World;

#[cfg(test)]
mod alternatives;
mod contacts;
mod motion;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_scenes;

/// `1e-5`, the length below which a translation counts as zero
/// (`utils::try_normalize_and_get_length`).
pub const EPS_LENGTH: Fixed = Fixed { raw: 42950 };
/// `1e-3`, the smallest `normal · up` of a contact that grounds the character.
pub const GROUNDED_COS: Fixed = Fixed { raw: 4294967 };
/// `0.05`, added to the offset for the ground prediction distance.
pub const GROUND_PREDICTION: Fixed = Fixed { raw: 214748365 };
/// `1e-5`, the penetration depth the depenetration pass ignores.
pub const PENETRATION_EPS: Fixed = Fixed { raw: 42950 };
/// `0.25`, the depenetration budget per call, relative to the character's height.
pub const MAX_CORRECTION: Fixed = Fixed { raw: 1073741824 };
/// `1 + 1e-5`, the surface correction factor of `subtract_hit`.
pub const SURFACE_CORRECTION: Fixed = Fixed { raw: 4295010246 };
/// The iteration bound of `move_shape`'s cast-and-slide loop.
pub const MAX_ITERATIONS: u32 = 20;

/// `0.01`, the default relative offset.
pub const DEFAULT_OFFSET: Fixed = Fixed { raw: 42949673 };
/// `0.2`, the default relative snap distance.
pub const DEFAULT_SNAP: Fixed = Fixed { raw: 858993459 };
/// `1e-4`, the default normal nudge factor.
pub const DEFAULT_NUDGE: Fixed = Fixed { raw: 429497 };

/// A length option of the controller (upstream `CharacterLength`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum CharacterLength {
    /// Relative to the character's size (its height, or its width for `min_width`).
    Relative: Fixed,
    /// An absolute length.
    Absolute: Fixed,
}

/// The auto-stepping configuration (upstream `CharacterAutostep`). Default: steps up to 0.25 of
/// the height, landings of at least 0.5 of the width, dynamic bodies included.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct CharacterAutostep {
    /// The highest step climbed automatically.
    pub max_height: CharacterLength,
    /// The narrowest landing accepted after a step.
    pub min_width: CharacterLength,
    /// Whether dynamic bodies count as steps too.
    pub include_dynamic_bodies: bool,
}

/// A collision met by the character during a move (upstream `CharacterCollision`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct CharacterCollision {
    /// The collider hit.
    pub handle: Handle,
    /// The character's pose when the collider was hit.
    pub character_pos: Pose2,
    /// The translation already applied when the hit happened.
    pub translation_applied: Vec2,
    /// The translation still to apply when the hit happened.
    pub translation_remaining: Vec2,
    /// The hit, `witness1` / `normal1` on the collider in world space.
    pub hit: ShapeCastHit,
}

/// The kinematic character controller's options (upstream `KinematicCharacterController`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct KinematicCharacterController {
    /// The up direction (floor and slope angles are measured against it).
    pub up: Vec2,
    /// The gap kept between the character and its surroundings (must not be zero).
    pub offset: CharacterLength,
    /// Whether the character slides along what it hits.
    pub slide: bool,
    /// Auto-stepping over small obstacles (off by default).
    pub autostep: Option<CharacterAutostep>,
    /// The steepest slope climbed, radians from `up`.
    pub max_slope_climb_angle: Fixed,
    /// The gentlest slope the character slides down, radians from `up`.
    pub min_slope_slide_angle: Fixed,
    /// Snap to the ground within this distance when grounded at the start of a move.
    pub snap_to_ground: Option<CharacterLength>,
    /// The distance pushed along the hit normals while sliding.
    pub normal_nudge_factor: Fixed,
}

/// The movement computed by the controller (upstream `EffectiveCharacterMovement`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct EffectiveCharacterMovement {
    /// The translation to apply.
    pub translation: Vec2,
    /// Whether the character touches the ground after `translation`.
    pub grounded: bool,
    /// Whether the character slides down a slope steeper than `min_slope_slide_angle`.
    pub is_sliding_down_slope: bool,
}

/// A hit split along its normal (upstream's private `HitDecomposition`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct HitDecomposition {
    /// The part along the normal, away from the surface (zero when moving into it).
    pub normal_part: Vec2,
    /// The horizontal tangent part: always zero in 2D.
    pub horizontal_tangent: Vec2,
    /// The tangent part.
    pub vertical_tangent: Vec2,
}

/// A hit and its classification (upstream's private `HitInfo`).
#[derive(Copy, Drop, Debug)]
pub(crate) struct HitInfo {
    pub toi: ShapeCastHit,
    pub is_wall: bool,
    pub is_nonslip_slope: bool,
}

pub impl CharacterAutostepDefault of Default<CharacterAutostep> {
    #[inline(always)]
    fn default() -> CharacterAutostep {
        CharacterAutostep {
            max_height: CharacterLength::Relative(Fixed { raw: 1073741824 }),
            min_width: CharacterLength::Relative(Fixed { raw: 2147483648 }),
            include_dynamic_bodies: true,
        }
    }
}

pub impl KinematicCharacterControllerDefault of Default<KinematicCharacterController> {
    #[inline(always)]
    fn default() -> KinematicCharacterController {
        KinematicCharacterController {
            up: Vec2 { x: ZERO, y: ONE },
            offset: CharacterLength::Relative(DEFAULT_OFFSET),
            slide: true,
            autostep: None,
            max_slope_climb_angle: FRAC_PI_4,
            min_slope_slide_angle: FRAC_PI_4,
            snap_to_ground: Some(CharacterLength::Relative(DEFAULT_SNAP)),
            normal_nudge_factor: DEFAULT_NUDGE,
        }
    }
}

#[generate_trait]
pub impl CharacterLengthImpl of CharacterLengthTrait {
    /// `Absolute(f(value))` for an absolute length, `self` otherwise.
    fn map_absolute<
        F, +Drop<F>, impl Func: core::ops::FnOnce<F, (Fixed,)>, +Into<Func::Output, Fixed>,
    >(
        self: CharacterLength, f: F,
    ) -> CharacterLength {
        match self {
            CharacterLength::Absolute(value) => CharacterLength::Absolute(f(value).into()),
            CharacterLength::Relative(_) => self,
        }
    }

    /// `Relative(f(value))` for a relative length, `self` otherwise.
    fn map_relative<
        F, +Drop<F>, impl Func: core::ops::FnOnce<F, (Fixed,)>, +Into<Func::Output, Fixed>,
    >(
        self: CharacterLength, f: F,
    ) -> CharacterLength {
        match self {
            CharacterLength::Relative(value) => CharacterLength::Relative(f(value).into()),
            CharacterLength::Absolute(_) => self,
        }
    }

    /// The length for a character of size `value` (upstream's private `eval`): `value * x` for
    /// `Relative(x)` (floored), `x` for `Absolute(x)`.
    #[inline(always)]
    fn eval(self: CharacterLength, value: Fixed) -> Fixed {
        match self {
            CharacterLength::Relative(x) => value * x,
            CharacterLength::Absolute(x) => x,
        }
    }
}

#[generate_trait]
pub impl HitDecompositionImpl of HitDecompositionTrait {
    /// The movement once the penetration part is removed: normal part plus both tangents.
    #[inline(always)]
    fn unconstrained_slide_part(self: HitDecomposition) -> Vec2 {
        self.normal_part + self.horizontal_tangent + self.vertical_tangent
    }
}

#[generate_trait]
pub impl KinematicCharacterControllerImpl of KinematicCharacterControllerTrait {
    /// The translation `character_shape` at `character_pos` can make towards
    /// `desired_translation` (upstream `move_shape`), with its grounded / sliding status. Every
    /// collider hit on the way is appended to `events`. `dt` is the timestep (used for the
    /// kinematic platforms' motion); `queries` filters the scene (exclude the character's own
    /// collider there).
    ///
    /// # Panics
    /// * `character_pos.rotation` and the colliders' rotations must be unit; the panics of the
    ///   shape casts and of the contact generators.
    fn move_shape(
        self: @KinematicCharacterController,
        dt: Fixed,
        ref world: World,
        queries: QueryPipeline,
        character_shape: Shape,
        character_pos: Pose2,
        desired_translation: Vec2,
        ref events: Array<CharacterCollision>,
    ) -> EffectiveCharacterMovement {
        motion::move_shape(
            *self,
            dt,
            ref world,
            queries,
            character_shape,
            character_pos,
            desired_translation,
            ref events,
        )
    }

    /// Applies to the dynamic bodies around the character the impulses of each collision in
    /// `collisions` (upstream `solve_character_collision_impulses`): the part of the remaining
    /// translation along the hit normal, as a velocity over `dt`, transferred at every contact
    /// within the ground prediction with the reduced mass of `character_mass` and the body's. An
    /// approximation, not a constraint solve (as upstream). Writes the bodies (and wakes them).
    fn solve_character_collision_impulses(
        self: @KinematicCharacterController,
        dt: Fixed,
        ref world: World,
        queries: QueryPipeline,
        character_shape: Shape,
        character_mass: Fixed,
        collisions: Span<CharacterCollision>,
    ) {
        for collision in collisions {
            contacts::solve_single_character_collision_impulse(
                *self, dt, ref world, queries, character_shape, character_mass, *collision,
            );
        }
    }
}

/// `(side extent, up extent)` of `shape`'s local box (upstream's private `compute_dims`).
pub(crate) fn compute_dims(up: Vec2, shape: Shape) -> Vec2 {
    let extents = shape.compute_local_aabb().extents();
    let up_abs = up.abs();
    let up_extent = extents.dot(up_abs);
    let side_extent = (extents - up_abs.mul_scalar(up_extent)).length();
    Vec2 { x: side_extent, y: up_extent }
}

/// The ground prediction distance for a character of height `up_extent`.
#[inline(always)]
pub(crate) fn predict_ground(controller: KinematicCharacterController, up_extent: Fixed) -> Fixed {
    controller.offset.eval(up_extent) + GROUND_PREDICTION
}

/// `pos` translated by `t` (upstream `Pose::from_translation(t) * pos`, exact).
#[inline(always)]
pub(crate) fn shifted(pos: Pose2, t: Vec2) -> Pose2 {
    Pose2 { translation: pos.translation + t, rotation: pos.rotation }
}

/// `(v / |v|, |v|)` when `|v| > threshold` (upstream `utils::try_normalize_and_get_length`).
#[inline(always)]
pub(crate) fn try_normalize_and_get_length(v: Vec2, threshold: Fixed) -> Option<(Vec2, Fixed)> {
    let (dir, len) = v.normalize_and_length();
    if len > threshold {
        Some((dir, len))
    } else {
        None
    }
}
