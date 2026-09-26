//! Contact manifold between a convex shape and a ball (Parry `contact_manifolds_convex_ball.rs`).
//!
//! The ball centre is projected on shape 1 (`project_local_point_and_get_feature_*`); the signed
//! distance is `|centre - projection|`, negative when the centre is inside, and the point is kept
//! when `dist <= r2 + prediction`. The normal is the direction from the surface to the centre
//! (flipped when inside); `fid1` is the projected feature, `fid2` is `Face(0)` (the ball's only
//! face, as in the f32 golden vectors).
//!
//! # Entry points
//!
//! Upstream has one generic function with a `flipped` flag, called by a `_shapes` wrapper that
//! swaps the arguments when the ball comes first. The port keeps the same split, with the flag
//! hidden behind two names:
//!
//! * [`contact_manifold_convex_ball`] — convex shape 1, ball 2;
//! * [`contact_manifold_ball_convex`] — ball 1, convex shape 2 (inverts `pos12`, writes the
//!   manifold with the two sides swapped);
//! * [`contact_manifold_convex_ball_shapes`] — the `Shape` wrapper: `true` when one of the two is
//!   a ball, ball–ball being the business of `ball_ball` (upstream's dispatcher tries it first).
//!
//! # Fixed-point choices
//!
//! * **One square root.** `norm2_wide(centre - projection)` is shared by the distance
//!   (`to_fixed`) and the normal (`recip`), so there is no `Fixed` division and one square root.
//!   Upstream divides `dpos / dist` on purpose (to keep vertical normals exactly vertical);
//!   `Recip::mul` rounds to nearest, which is exact for axis-aligned `dpos` as well.
//! * **Per-shape projection behind `#[inline(never)]`.** A computing arm of an enum `match` is
//!   charged to every arm, so each projection sits in its own helper.
//! * **Warm start.** Like upstream, a manifold that already holds exactly one point keeps its
//!   `ContactData` and only gets new geometry; anything else is cleared first.
//!
//! # Candidates
//!
//! Ranked by the `gas_*` probes of this module; the losers live in `#[cfg(test)] mod
//! alternatives`.
//!
//! 1. **shared norm** (this module): projection, then one `norm2_wide` feeding `dist` and the
//!    normal.
//! 2. `alternatives::contact_manifold_convex_ball_separate_norm`: projection, then `norm2` for the
//!    distance and `try_normalize2` for the normal — two square roots.
//! 3. `alternatives::contact_manifold_convex_ball_early_reject`: wide `|dpos|^2 <= (r2 +
//!    prediction)^2` test before the square root (skipped when the centre is inside).
//! 4. `alternatives::contact_manifold_convex_ball_inlined`: the projection `match` with the
//!    projections inlined in its arms.
//!
//! Three formulations of the `_shapes` wrapper (`match` on the pair, nested `match`es, upstream's
//! `as_ball` downcasts) compile to the same code (148 880 gas in the same probe), so
//! only the pair `match` is kept.

use fixed::wide::{NormTrait, RecipTrait, norm2_wide};
use fixed::{Fixed, ONE, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::math_ext::vec2::try_normalize2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::contact::{ContactManifold, ContactManifoldTrait, TrackedContact};
use crate::feature_id::{FeatureId, FeatureIdTrait};
use crate::point::{
    PointProjection, project_local_point_and_get_feature_ball,
    project_local_point_and_get_feature_capsule, project_local_point_and_get_feature_cuboid,
    project_local_point_and_get_feature_halfspace, project_local_point_and_get_feature_segment,
};
use crate::shape::{Ball, Capsule, Cuboid, HalfSpace, Segment, Shape};

/// Computes the contact manifold between a convex shape and a ball, either of them first.
///
/// Mirrors `contact_manifold_convex_ball_shapes`: when `shape1` is a ball the pair is generated
/// with the arguments swapped (`flipped = true`), otherwise when `shape2` is a ball it is
/// generated directly. Returns `false` and leaves `manifold` untouched when neither is a ball.
/// Two balls take the first branch and are handled as a convex–ball pair (upstream's dispatcher
/// sends them to `ball_ball` first).
/// #### Panics
/// * See [`contact_manifold_convex_ball`].
/// #### Deviations
/// * Upstream returns `()`; the `bool` tells the dispatcher whether the pair was handled. The
///   normal-constraint arguments (`NormalConstraints`, for triangle meshes) are dropped.
pub fn contact_manifold_convex_ball_shapes(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    match (shape1, shape2) {
        (
            Shape::Ball(ball1), _,
        ) => {
            contact_manifold_ball_convex(pos12, ball1, shape2, prediction, ref manifold);
            true
        },
        (
            _, Shape::Ball(ball2),
        ) => {
            contact_manifold_convex_ball(pos12, shape1, ball2, prediction, ref manifold);
            true
        },
        _ => false,
    }
}

/// Computes the contact manifold between a convex `shape1` and `ball2`.
///
/// `pos12` is the pose of the ball in the frame of `shape1`. The manifold gets one point when
/// `dist <= radius + prediction` (`dist` is the signed distance from the ball centre to `shape1`)
/// and is cleared otherwise; the normals are only written together with a point.
/// #### Panics
/// * `'Fixed: overflow'` if `|centre - projection| >= 2^31`, or `radius + prediction` leaves the
///   scalar range.
/// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if `centre - projection` leaves the scalar
///   range.
/// * The panics of the projection on `shape1` (see `crate::point`).
/// #### Deviations
/// * `dist` is the floored wide length; the normal is `dpos * recip(length)`, so it is exact for
///   axis-aligned `dpos` and within `~1 ulp / |dpos|` otherwise.
/// * When the centre is on the surface (`dpos == 0`) the normal is the normalised
///   `pos12.translation`, or `+Y` when that is zero too (upstream's `normalize_or(Y)`), negated
///   because a point on the boundary counts as inside.
/// * A manifold holding exactly one point keeps its `ContactData` (`copy_geometry_from`).
/// * Normal constraints and the ray-cast correction that goes with them are deferred.
pub fn contact_manifold_convex_ball(
    pos12: Pose2, shape1: Shape, ball2: Ball, prediction: Fixed, ref manifold: ContactManifold,
) {
    let (proj, fid1) = project(shape1, pos12.translation);
    finish(pos12, proj, fid1, ball2, prediction, false, ref manifold);
}

/// Computes the contact manifold between `ball1` and a convex `shape2`.
///
/// `pos12` is the pose of `shape2` in the frame of the ball. Upstream's `flipped = true` branch:
/// the pair is generated as `shape2` against the ball with `pos12.inverse()`, and the two sides
/// of the manifold are swapped (`local_p1`/`local_p2`, `fid1`/`fid2`, `local_n1`/`local_n2`).
/// #### Panics
/// * See [`contact_manifold_convex_ball`], with `pos12.inverse()` in place of `pos12`, and the
///   panics of `Pose2::inverse` (`pos12.rotation` must be unit).
/// #### Deviations
/// * See [`contact_manifold_convex_ball`]; `pos12.inverse()` floors each component once.
pub fn contact_manifold_ball_convex(
    pos12: Pose2, ball1: Ball, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) {
    let pos21 = pos12.inverse();
    let (proj, fid2) = project(shape2, pos21.translation);
    finish(pos21, proj, fid2, ball1, prediction, true, ref manifold);
}

#[inline(never)]
fn project_ball(shape: Ball, pt: Vec2) -> (PointProjection, FeatureId) {
    project_local_point_and_get_feature_ball(shape, pt)
}

#[inline(never)]
fn project_cuboid(shape: Cuboid, pt: Vec2) -> (PointProjection, FeatureId) {
    project_local_point_and_get_feature_cuboid(shape, pt)
}

#[inline(never)]
fn project_capsule(shape: Capsule, pt: Vec2) -> (PointProjection, FeatureId) {
    project_local_point_and_get_feature_capsule(shape, pt)
}

#[inline(never)]
fn project_segment(shape: Segment, pt: Vec2) -> (PointProjection, FeatureId) {
    project_local_point_and_get_feature_segment(shape, pt)
}

#[inline(never)]
fn project_halfspace(shape: HalfSpace, pt: Vec2) -> (PointProjection, FeatureId) {
    project_local_point_and_get_feature_halfspace(shape, pt)
}

/// `Shape::project_local_point_and_get_feature`: one `#[inline(never)]` helper per arm.
fn project(shape: Shape, pt: Vec2) -> (PointProjection, FeatureId) {
    match shape {
        Shape::Ball(s) => project_ball(s, pt),
        Shape::Cuboid(s) => project_cuboid(s, pt),
        Shape::Capsule(s) => project_capsule(s, pt),
        Shape::Segment(s) => project_segment(s, pt),
        Shape::HalfSpace(s) => project_halfspace(s, pt),
        Shape::ConvexPolygon(s) => crate::point::convex_polygon::project_local_point_and_get_feature_convex_polygon(
            s.unbox(), pt,
        ),
        _ => project_sh1(shape, pt),
    }
}

/// The SH1 arms (triangle, round shapes): one out-of-line helper, as the other arms.
#[inline(never)]
fn project_sh1(shape: Shape, pt: Vec2) -> (PointProjection, FeatureId) {
    crate::point::query::ShapePointQuery::project_local_point_and_get_feature(shape, pt)
}

/// The normal of a centre exactly on the surface: `normalize_or(translation, +Y)`. Cold path.
#[inline(never)]
fn fallback_normal(translation: Vec2) -> Vec2 {
    match try_normalize2(translation.x, translation.y) {
        Some((x, y)) => Vec2 { x, y },
        None => Vec2 { x: ZERO, y: ONE },
    }
}

/// Everything after the projection: distance, normals, the point, and its storage.
///
/// `pos12` is the pose of the ball in the frame of the convex shape, `flipped` tells that the
/// manifold's shape 1 is the ball.
fn finish(
    pos12: Pose2,
    proj: PointProjection,
    fid_convex: FeatureId,
    ball: Ball,
    prediction: Fixed,
    flipped: bool,
    ref manifold: ContactManifold,
) {
    let t = pos12.translation;
    let d = Vec2 { x: t.x - proj.point.x, y: t.y - proj.point.y };
    let n = norm2_wide(d.x, d.y);
    let len = n.to_fixed();
    let dist = if proj.is_inside {
        -len
    } else {
        len
    };
    if dist <= ball.radius + prediction {
        let dir = match n.try_recip() {
            Some(r) => Vec2 { x: r.mul(d.x), y: r.mul(d.y) },
            None => fallback_normal(t),
        };
        let local_n1 = if proj.is_inside {
            -dir
        } else {
            dir
        };
        write_contact(pos12, proj.point, fid_convex, ball, local_n1, dist, flipped, ref manifold);
    } else {
        manifold.clear();
    }
}

/// Stores the point and the normals. `local_n1` is the normal in the convex shape's frame.
#[inline(always)]
fn write_contact(
    pos12: Pose2,
    local_p_convex: Vec2,
    fid_convex: FeatureId,
    ball: Ball,
    local_n1: Vec2,
    dist: Fixed,
    flipped: bool,
    ref manifold: ContactManifold,
) {
    let local_n2 = -pos12.rotation.inverse_rotate(local_n1);
    let local_p_ball = local_n2.mul_scalar(ball.radius);
    let face = FeatureIdTrait::face(0);
    let [old, second] = manifold.points;
    // Upstream keeps the warm-start data of a manifold that already holds exactly one point.
    let data = if manifold.num_points == 1 {
        old.data
    } else {
        Default::default()
    };
    let (p1, p2, fid1, fid2, n1, n2) = if flipped {
        (local_p_ball, local_p_convex, face, fid_convex, local_n2, local_n1)
    } else {
        (local_p_convex, local_p_ball, fid_convex, face, local_n1, local_n2)
    };
    manifold
        .points =
            [
                TrackedContact {
                    local_p1: p1, local_p2: p2, dist: dist - ball.radius, fid1, fid2, data,
                },
                second,
            ];
    manifold.num_points = 1;
    manifold.local_n1 = n1;
    manifold.local_n2 = n2;
}

/// Rejected candidates, kept for the `gas_*` ranking.
#[cfg(test)]
pub mod alternatives {
    use fixed::Fixed;
    use fixed::wide::{NormTrait, RecipTrait, norm2, norm2_wide};
    use glam::Vec2;
    use rapier_math::math_ext::norm2::is_norm2_lt;
    use rapier_math::math_ext::vec2::try_normalize2;
    use rapier_math::pose2::Pose2;
    use crate::contact::{ContactManifold, ContactManifoldTrait};
    use crate::feature_id::FeatureId;
    use crate::point::{
        PointProjection, project_local_point_and_get_feature_ball,
        project_local_point_and_get_feature_capsule, project_local_point_and_get_feature_cuboid,
        project_local_point_and_get_feature_halfspace, project_local_point_and_get_feature_segment,
    };
    use crate::shape::{Ball, Shape};
    use super::{fallback_normal, project, write_contact};

    /// Distance from `norm2`, normal from `try_normalize2`: two integer square roots, the shape
    /// of upstream's `dpos.length()` followed by `dpos / dist` without the shared norm.
    pub fn contact_manifold_convex_ball_separate_norm(
        pos12: Pose2,
        shape1: Shape,
        ball2: Ball,
        prediction: Fixed,
        flipped: bool,
        ref manifold: ContactManifold,
    ) {
        let (proj, fid1) = project(shape1, pos12.translation);
        let t = pos12.translation;
        let d = Vec2 { x: t.x - proj.point.x, y: t.y - proj.point.y };
        let len = norm2(d.x, d.y);
        let dist = if proj.is_inside {
            -len
        } else {
            len
        };
        if dist <= ball2.radius + prediction {
            let dir = match try_normalize2(d.x, d.y) {
                Some((x, y)) => Vec2 { x, y },
                None => fallback_normal(t),
            };
            let local_n1 = if proj.is_inside {
                -dir
            } else {
                dir
            };
            write_contact(pos12, proj.point, fid1, ball2, local_n1, dist, flipped, ref manifold);
        } else {
            manifold.clear();
        }
    }

    /// Wide `|dpos|^2 <= (r2 + prediction)^2` before the square root, when the centre is outside
    /// (`floor(L) <= t` iff `L < t + 1 ulp`, so this test is exact for a representable `t`).
    pub fn contact_manifold_convex_ball_early_reject(
        pos12: Pose2,
        shape1: Shape,
        ball2: Ball,
        prediction: Fixed,
        flipped: bool,
        ref manifold: ContactManifold,
    ) {
        let (proj, fid1) = project(shape1, pos12.translation);
        let t = pos12.translation;
        let d = Vec2 { x: t.x - proj.point.x, y: t.y - proj.point.y };
        let reach = ball2.radius + prediction;
        // `floor(L) <= reach` iff `L < reach + 1 ulp`: compare against `reach + 1 ulp`, strictly.
        if !proj.is_inside && !is_norm2_lt(d.x, d.y, reach + Fixed { raw: 1 }) {
            manifold.clear();
            return;
        }
        let n = norm2_wide(d.x, d.y);
        let len = n.to_fixed();
        let dist = if proj.is_inside {
            -len
        } else {
            len
        };
        let dir = match n.try_recip() {
            Some(r) => Vec2 { x: r.mul(d.x), y: r.mul(d.y) },
            None => fallback_normal(t),
        };
        let local_n1 = if proj.is_inside {
            -dir
        } else {
            dir
        };
        write_contact(pos12, proj.point, fid1, ball2, local_n1, dist, flipped, ref manifold);
    }

    /// The projection `match` with the projections inlined in its arms: every arm is charged the
    /// cost of the dearest one.
    fn project_inlined(shape: Shape, pt: Vec2) -> (PointProjection, FeatureId) {
        match shape {
            Shape::Ball(s) => project_local_point_and_get_feature_ball(s, pt),
            Shape::Cuboid(s) => project_local_point_and_get_feature_cuboid(s, pt),
            Shape::Capsule(s) => project_local_point_and_get_feature_capsule(s, pt),
            Shape::Segment(s) => project_local_point_and_get_feature_segment(s, pt),
            Shape::HalfSpace(s) => project_local_point_and_get_feature_halfspace(s, pt),
            Shape::ConvexPolygon(s) => crate::point::convex_polygon::project_local_point_and_get_feature_convex_polygon(
                s.unbox(), pt,
            ),
            _ => super::project_sh1(shape, pt),
        }
    }

    /// Same as [`super::contact_manifold_convex_ball`] with [`project_inlined`].
    pub fn contact_manifold_convex_ball_inlined(
        pos12: Pose2, shape1: Shape, ball2: Ball, prediction: Fixed, ref manifold: ContactManifold,
    ) {
        let (proj, fid1) = project_inlined(shape1, pos12.translation);
        super::finish(pos12, proj, fid1, ball2, prediction, false, ref manifold);
    }
}

#[cfg(test)]
mod tests;
