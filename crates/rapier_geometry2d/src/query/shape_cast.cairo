//! Linear shape casts (Parry `query/shape_cast/`, work package CC1): the first time two shapes
//! moving at constant linear velocities touch.
//!
//! # Entry points
//!
//! * [`cast_shapes`] takes both world poses and velocities, as upstream's free function:
//!   `pos12 = pos1.inv_mul(pos2)`, `vel12 = pos1.rotation^-1 (vel2 - vel1)`, then the local table
//!   [`cast_shapes_local`] (upstream `DefaultQueryDispatcher::cast_shapes`).
//! * The table sends ball–ball to [`ball_ball`], a half-space against a support map to
//!   [`halfspace`], every other supported pair to [`support_map`]; half-space–half-space is
//!   unsupported (`None`, upstream `Err(Unsupported)`). Composite shapes are lot SH2.
//!
//! # Semantics (upstream's)
//!
//! * A time of impact `t` is in units of the relative velocity: shape 2 has moved by
//!   `vel12 * t` relative to shape 1. `t <= options.max_time_of_impact` (inclusive).
//! * `witness1` / `normal1` are in the local frame of shape 1, `witness2` / `normal2` in the local
//!   frame of shape 2; `normal1` points from shape 1 towards shape 2.
//! * `target_distance` inflates shape 1: the cast stops when the shapes are that far apart, and
//!   `witness1` stays on the real surface of shape 1.
//! * A pair starting within `target_distance` (touching included) answers `t = 0`; its status is
//!   `PenetratingOrWithinTargetDist` when the start is strictly closer than `target_distance`,
//!   `Converged` otherwise. With `stop_at_penetration = false`, a pair whose `t` is below `1e-4`
//!   and whose normal velocity separates it is not a hit (`None`).
//! * A zero relative velocity never hits the support-map pairs (upstream's GJK ray cast has no
//!   direction), even when they overlap; the ball–ball and half-space kernels answer `t = 0`
//!   for an overlapping start, as upstream's ray casts do.
//!
//! # Deviations
//!
//! * The support-map pairs are answered exactly ([`support_map`]: a ray cast on the Minkowski
//!   difference of the polygonal cores dilated by the radii) where upstream runs GJK's
//!   conservative-advancement ray cast: the time of impact agrees within GJK's tolerance, the
//!   witnesses of a face–face impact are one member of the same family.

pub mod ball_ball;
pub mod halfspace;
pub mod support_map;
pub use ball_ball::cast_shapes_ball_ball;
use fixed::{Fixed, MAX, ZERO};
use glam::Vec2;
pub use halfspace::{cast_shapes_halfspace_support_map, cast_shapes_support_map_halfspace};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
pub use support_map::cast_shapes_support_map_support_map;
use crate::shape::Shape;

#[cfg(test)]
mod alternatives;
#[cfg(test)]
mod tests;

/// `1e-4`, upstream's literal below which a time of impact is "at the start" (nearest raw).
pub const TOI_AT_START: Fixed = Fixed { raw: 429497 };

/// How a shape cast ended (Parry `ShapeCastStatus`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum ShapeCastStatus {
    /// An iterative method ran out of iterations before converging.
    OutOfIterations,
    /// The time of impact was found.
    Converged,
    /// An iterative method failed to converge (the answer is a best effort).
    Failed,
    /// The shapes started penetrating, or closer than the target distance.
    PenetratingOrWithinTargetDist,
}

/// The first impact of a shape cast (Parry `ShapeCastHit`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ShapeCastHit {
    /// The time of impact, in units of the relative velocity (see the module documentation).
    pub time_of_impact: Fixed,
    /// The impact point on shape 1, in its local frame.
    pub witness1: Vec2,
    /// The impact point on shape 2, in its local frame.
    pub witness2: Vec2,
    /// Unit outward normal of shape 1 at `witness1`, in its local frame.
    pub normal1: Vec2,
    /// Unit outward normal of shape 2 at `witness2`, in its local frame.
    pub normal2: Vec2,
    pub status: ShapeCastStatus,
}

#[generate_trait]
pub impl ShapeCastHitImpl of ShapeCastHitTrait {
    /// The hit with its two sides swapped (upstream `swapped`).
    #[inline(always)]
    fn swapped(self: ShapeCastHit) -> ShapeCastHit {
        ShapeCastHit {
            time_of_impact: self.time_of_impact,
            witness1: self.witness2,
            witness2: self.witness1,
            normal1: self.normal2,
            normal2: self.normal1,
            status: self.status,
        }
    }

    /// The hit with side 1 moved by `pos` (upstream `transform1_by`).
    #[inline(always)]
    fn transform1_by(self: ShapeCastHit, pos: Pose2) -> ShapeCastHit {
        ShapeCastHit {
            time_of_impact: self.time_of_impact,
            witness1: pos.transform_point(self.witness1),
            witness2: self.witness2,
            normal1: pos.rotation.rotate(self.normal1),
            normal2: self.normal2,
            status: self.status,
        }
    }
}

/// The parameters of a shape cast (Parry `ShapeCastOptions`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ShapeCastOptions {
    /// The largest time of impact reported (inclusive); `fixed::MAX` by default.
    pub max_time_of_impact: Fixed,
    /// The distance at which the shapes count as touching; 0 by default.
    pub target_distance: Fixed,
    /// Whether a pair starting within the target distance answers `t = 0` whatever its motion
    /// (`true` by default); `false` ignores such a start when the motion separates the shapes.
    pub stop_at_penetration: bool,
    /// Whether an impact at the start reports the contact geometry (witnesses and normals of the
    /// contact query) rather than the cast's own (`true` by default).
    pub compute_impact_geometry_on_penetration: bool,
}

#[generate_trait]
pub impl ShapeCastOptionsImpl of ShapeCastOptionsTrait {
    /// The default options with `max_time_of_impact` (upstream `with_max_time_of_impact`).
    #[inline(always)]
    fn with_max_time_of_impact(max_time_of_impact: Fixed) -> ShapeCastOptions {
        ShapeCastOptions { max_time_of_impact, ..Default::default() }
    }
}

/// Upstream's defaults: no bound, no target distance, stop at penetration, contact geometry.
pub impl ShapeCastOptionsDefault of Default<ShapeCastOptions> {
    #[inline(always)]
    fn default() -> ShapeCastOptions {
        ShapeCastOptions {
            max_time_of_impact: MAX,
            target_distance: ZERO,
            stop_at_penetration: true,
            compute_impact_geometry_on_penetration: true,
        }
    }
}

/// The first impact of `shape2`, placed at `pos12` and moving at `vel12` relative to `shape1`
/// (both in the frame of `shape1`), with `shape1` (upstream `DefaultQueryDispatcher::cast_shapes`).
/// The outer `None` is upstream's `Err(Unsupported)` (half-space–half-space), the inner one "no
/// impact".
/// #### Panics
/// * The panics of the selected kernel.
#[inline(always)]
pub fn cast_shapes_local(
    pos12: Pose2, vel12: Vec2, shape1: Shape, shape2: Shape, options: ShapeCastOptions,
) -> Option<Option<ShapeCastHit>> {
    match (shape1, shape2) {
        (
            Shape::Ball(b1), Shape::Ball(b2),
        ) => Some(cast_shapes_ball_ball(pos12, vel12, b1, b2, options)),
        (Shape::HalfSpace(_), Shape::HalfSpace(_)) => None,
        (
            Shape::HalfSpace(h), _,
        ) => Some(cast_shapes_halfspace_support_map(pos12, vel12, h, shape2, options)),
        (
            _, Shape::HalfSpace(h),
        ) => Some(cast_shapes_support_map_halfspace(pos12, vel12, shape1, h, options)),
        _ => Some(cast_shapes_support_map_support_map(pos12, vel12, shape1, shape2, options)),
    }
}

/// The first impact of `g1` at `pos1` moving at `vel1` and `g2` at `pos2` moving at `vel2`
/// (world space; upstream `query::cast_shapes`). The witnesses and normals of the hit are in
/// the local frame of their shape. The outer `None` is an unsupported pair.
/// #### Panics
/// * `pos1.rotation` must be unit (`Pose2::inv_mul`); the panics of [`cast_shapes_local`].
pub fn cast_shapes(
    pos1: Pose2,
    vel1: Vec2,
    g1: Shape,
    pos2: Pose2,
    vel2: Vec2,
    g2: Shape,
    options: ShapeCastOptions,
) -> Option<Option<ShapeCastHit>> {
    let pos12 = pos1.inv_mul(pos2);
    let vel12 = pos1.rotation.inverse_rotate(vel2 - vel1);
    cast_shapes_local(pos12, vel12, g1, g2, options)
}
