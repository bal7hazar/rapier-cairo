//! Nonlinear shape casts (Parry `query/nonlinear_shape_cast/`, work package CC1): the first time
//! two shapes moving with constant linear **and angular** velocities touch.
//!
//! * [`NonlinearRigidMotion`] is upstream's motion: a start pose, a local rotation centre, a
//!   linear and an angular velocity; [`NonlinearRigidMotionTrait::position_at_time`] rotates
//!   about the centre by `angvel * t` (one `sin_cos` of `fixed`, about 1 ulp) and translates by
//!   `linvel * t`.
//! * [`cast_shapes_nonlinear`] is upstream's `query::cast_shapes_nonlinear` and
//!   `DefaultQueryDispatcher::cast_shapes_nonlinear`: every pair of support maps goes to the
//!   conservative advancement of [`support_map`]; a half-space on either side is unsupported
//!   (`None`), as upstream (its half-space kernel is commented out). Composite shapes are lot SH2.
//! * [`NonlinearShapeCastMode`] selects upstream's two termination rules: stop at a start in
//!   contact, or ([`NonlinearShapeCastModeTrait::directional_toi`]) look past it for the first
//!   contact that the motion would tunnel through.
//!
//! The witnesses and normals of a hit are in the local frames of their shapes at the time of
//! impact, as upstream.
//!
//! # Candidates
//!
//! `position_at_time` meters its rotation behind a one-iteration `while` (AGENTS §7), so that a
//! motion without angular velocity does not pay for the `sin_cos`: 31.7k gas / 190 steps without
//! rotation, 84.8k / 652 with it. `alternatives::position_at_time_branch` (a plain `if`): 73.9k
//! gas on both paths, 146 / 548 steps. Sierra gas is the criterion (AGENTS §2): the metered form
//! makes the rotating bar of `tests` 30.8M gas instead of 36.9M, the translating cuboid 509k
//! instead of 804k, for +44 / +104 steps per call.

pub mod support_map;
use fixed::trig::TrigTrait;
use fixed::{FRAC_PI_2, FRAC_PI_4, Fixed, PI, ZERO};
use glam_core::Vec2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::{Rot2, Rot2Trait};
pub use support_map::cast_shapes_nonlinear_support_map_support_map;
use crate::aabb::AabbTrait;
use crate::shape::{ConvexPolygonTrait, Shape, ShapeTrait};
use super::shape_cast::ShapeCastHit;

#[cfg(test)]
mod alternatives;
#[cfg(test)]
mod tests;

/// The motion of a rigid body with constant velocities (Parry `NonlinearRigidMotion`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct NonlinearRigidMotion {
    /// The pose at time 0.
    pub start: Pose2,
    /// The rotation centre, in the body's local frame (its centre of mass for a rigid body).
    pub local_center: Vec2,
    /// Linear velocity of the centre.
    pub linvel: Vec2,
    /// Angular velocity, in radians per unit of time.
    pub angvel: Fixed,
}

#[generate_trait]
pub impl NonlinearRigidMotionImpl of NonlinearRigidMotionTrait {
    /// A motion (upstream `new`).
    #[inline(always)]
    fn new(start: Pose2, local_center: Vec2, linvel: Vec2, angvel: Fixed) -> NonlinearRigidMotion {
        NonlinearRigidMotion { start, local_center, linvel, angvel }
    }

    /// The motion standing still at the identity (upstream `identity`).
    #[inline(always)]
    fn identity() -> NonlinearRigidMotion {
        Self::constant_position(Default::default())
    }

    /// The motion standing still at `pos` (upstream `constant_position`).
    #[inline(always)]
    fn constant_position(pos: Pose2) -> NonlinearRigidMotion {
        NonlinearRigidMotion {
            start: pos,
            local_center: Vec2 { x: ZERO, y: ZERO },
            linvel: Vec2 { x: ZERO, y: ZERO },
            angvel: ZERO,
        }
    }

    /// Stops the motion at its pose at time `t` (upstream `freeze`).
    /// #### Panics
    /// * See [`NonlinearRigidMotionTrait::position_at_time`].
    fn freeze(ref self: NonlinearRigidMotion, t: Fixed) {
        self.start = Self::position_at_time(@self, t);
        self.linvel = Vec2 { x: ZERO, y: ZERO };
        self.angvel = ZERO;
    }

    /// The motion with `tra` applied after its start pose (upstream `append_translation`); the
    /// rotation centre keeps its world position.
    fn append_translation(self: @NonlinearRigidMotion, tra: Vec2) -> NonlinearRigidMotion {
        let s = *self.start;
        set_start(*self, Pose2 { translation: s.translation + tra, rotation: s.rotation })
    }

    /// The motion with `tra` applied before its start pose (upstream `prepend_translation`).
    fn prepend_translation(self: @NonlinearRigidMotion, tra: Vec2) -> NonlinearRigidMotion {
        let s = *self.start;
        set_start(*self, Pose2 { translation: s.transform_point(tra), rotation: s.rotation })
    }

    /// The motion with `iso` applied after its start pose (upstream `append`).
    fn append(self: @NonlinearRigidMotion, iso: Pose2) -> NonlinearRigidMotion {
        set_start(*self, iso.mul(*self.start))
    }

    /// The motion with `iso` applied before its start pose (upstream `prepend`).
    fn prepend(self: @NonlinearRigidMotion, iso: Pose2) -> NonlinearRigidMotion {
        set_start(*self, (*self.start).mul(iso))
    }

    /// The pose at time `t` (upstream `position_at_time`): the start pose rotated by
    /// `angvel * t` about the world rotation centre, then translated by `linvel * t`. A zero
    /// `angvel` skips the rotation (same answer: `sin_cos(0)` is exactly the identity).
    /// #### Panics
    /// * `'Fixed: overflow'` if `linvel * t`, `angvel * t` or the pose leaves the scalar range.
    fn position_at_time(self: @NonlinearRigidMotion, t: Fixed) -> Pose2 {
        let start = *self.start;
        let lin = *self.linvel;
        let shift = Vec2 { x: lin.x * t, y: lin.y * t };
        let angvel = *self.angvel;
        let mut pose = Pose2 { translation: start.translation + shift, rotation: start.rotation };
        // The rotation is metered: a `while` body is only charged when it runs, so a motion
        // without angular velocity does not pay for the `sin_cos` (`tests::gas_position_*`).
        let mut pending = angvel != ZERO;
        while pending {
            pose = rotated(start, *self.local_center, shift, angvel * t);
            pending = false;
        }
        pose
    }
}

/// `start` rotated by `angle` about its local point `local_center`, then moved by `shift`.
fn rotated(start: Pose2, local_center: Vec2, shift: Vec2, angle: Fixed) -> Pose2 {
    let center = start.transform_point(local_center);
    let (sin, cos) = angle.sin_cos();
    let rot = Rot2 { re: cos, im: sin };
    Pose2 {
        translation: center + shift + rot.rotate(start.translation - center),
        rotation: rot.mul(start.rotation),
    }
}

/// `motion` with the start pose `new_start`, its rotation centre moved so that it stays at the
/// same world position (upstream `set_start`).
fn set_start(motion: NonlinearRigidMotion, new_start: Pose2) -> NonlinearRigidMotion {
    let world_center = motion.start.transform_point(motion.local_center);
    NonlinearRigidMotion {
        start: new_start,
        local_center: new_start.inverse_transform_point(world_center),
        linvel: motion.linvel,
        angvel: motion.angvel,
    }
}

/// How a nonlinear cast treats a start in contact (Parry `NonlinearShapeCastMode`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum NonlinearShapeCastMode {
    /// A start in contact is the impact (`t = start_time`).
    StopAtPenetration,
    /// A start in contact is only an impact if the motion would tunnel through one of the
    /// contacts met while rotating: `(sum_linear_thickness, max_angular_thickness)`.
    Directional: (Fixed, Fixed),
}

#[generate_trait]
pub impl NonlinearShapeCastModeImpl of NonlinearShapeCastModeTrait {
    /// The directional mode of two shapes (upstream `directional_toi`): the sum of their
    /// [`ccd_thickness`] and the larger [`ccd_angular_thickness`].
    fn directional_toi(shape1: Shape, shape2: Shape) -> NonlinearShapeCastMode {
        let (a1, a2) = (ccd_angular_thickness(shape1), ccd_angular_thickness(shape2));
        NonlinearShapeCastMode::Directional(
            (ccd_thickness(shape1) + ccd_thickness(shape2), if a1 > a2 {
                a1
            } else {
                a2
            }),
        )
    }
}

/// The thickness under which a shape may tunnel (upstream `Shape::ccd_thickness`): the radius
/// of a ball or capsule, the smallest half extent of a cuboid or of a polygon's local box, zero
/// for a segment, a triangle, a polyline or a heightfield, the inner value plus the border for a
/// round shape, the smallest of its parts' for a compound, and upstream's `f32::MAX` (saturated to
/// `fixed::MAX`) for a half-space.
pub fn ccd_thickness(shape: Shape) -> Fixed {
    match shape {
        Shape::Ball(s) => s.radius,
        Shape::Cuboid(s) => min_element(s.half_extents),
        Shape::Capsule(s) => s.radius,
        Shape::Segment(_) => ZERO,
        Shape::HalfSpace(_) => fixed::MAX,
        Shape::ConvexPolygon(s) => min_element(s.unbox().compute_local_aabb().half_extents()),
        Shape::Triangle(_) => ZERO,
        Shape::RoundCuboid(s) => min_element(s.inner_shape.half_extents) + s.border_radius,
        Shape::RoundTriangle(s) => s.unbox().border_radius,
        Shape::RoundConvexPolygon(s) => {
            let s = s.unbox();
            min_element(s.local_aabb.half_extents()) + s.border_radius
        },
        Shape::Polyline(_) => ZERO,
        Shape::HeightField(_) => ZERO,
        Shape::Compound(s) => compound_ccd_thickness(@s.unbox()),
    }
}

/// The smallest rotation after which a shape may touch with a new contact (upstream
/// `Shape::ccd_angular_thickness`): `pi` for a ball or a half-space, `pi / 4` for a convex
/// polygon (round or not), a polyline or a heightfield, `fixed::MAX` for a compound (upstream folds
/// its parts' with `max` from `Real::MAX`), `pi / 2` otherwise.
pub fn ccd_angular_thickness(shape: Shape) -> Fixed {
    match shape {
        Shape::Ball(_) => PI,
        Shape::HalfSpace(_) => PI,
        Shape::ConvexPolygon(_) => FRAC_PI_4,
        Shape::RoundConvexPolygon(_) => FRAC_PI_4,
        Shape::Polyline(_) => FRAC_PI_4,
        Shape::HeightField(_) => FRAC_PI_4,
        // Upstream folds `max` from `Real::MAX`: always the maximum.
        Shape::Compound(_) => fixed::MAX,
        _ => FRAC_PI_2,
    }
}

/// The smallest `ccd_thickness` of the parts (upstream: folded with `min` from `Real::MAX`).
fn compound_ccd_thickness(compound: @crate::shape::Compound) -> Fixed {
    let mut out = fixed::MAX;
    for part in crate::shape::CompoundTrait::shapes(compound) {
        let (_, shape) = *part;
        let t = ccd_thickness(shape);
        if t < out {
            out = t;
        }
    }
    out
}

#[inline(always)]
fn min_element(v: Vec2) -> Fixed {
    if v.x < v.y {
        v.x
    } else {
        v.y
    }
}

/// The first impact of `g1` following `motion1` and `g2` following `motion2` within
/// `[start_time, end_time]` (upstream `query::cast_shapes_nonlinear` /
/// `DefaultQueryDispatcher::cast_shapes_nonlinear`). With `stop_at_penetration`, a start in
/// contact answers `t = start_time`; otherwise the directional mode of the two shapes applies.
/// The outer `None` is an unsupported pair (a half-space on either side, a heightfield); a
/// polyline answers through its segments (`super::composite`).
/// #### Panics
/// * The panics of [`cast_shapes_nonlinear_support_map_support_map`].
pub fn cast_shapes_nonlinear(
    motion1: NonlinearRigidMotion,
    g1: Shape,
    motion2: NonlinearRigidMotion,
    g2: Shape,
    start_time: Fixed,
    end_time: Fixed,
    stop_at_penetration: bool,
) -> Option<Option<ShapeCastHit>> {
    if g1.as_support_map().is_none() || g2.as_support_map().is_none() {
        // SH2a: a polyline answers through its segments (a heightfield is unsupported).
        return super::composite::cast_shapes_nonlinear_composite(
            motion1, g1, motion2, g2, start_time, end_time, stop_at_penetration,
        );
    }
    let mode = if stop_at_penetration {
        NonlinearShapeCastMode::StopAtPenetration
    } else {
        NonlinearShapeCastModeTrait::directional_toi(g1, g2)
    };
    Some(
        cast_shapes_nonlinear_support_map_support_map(
            motion1, g1, motion2, g2, start_time, end_time, mode,
        ),
    )
}
