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
    }
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
mod tests {
    use fixed::{Fixed, HALF, ONE};
    use glam::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::contact::{ContactData, ContactManifold, ContactManifoldTrait, TrackedContact};
    use crate::feature_id::{FeatureId, FeatureIdTrait};
    use crate::shape::{Ball, Capsule, Cuboid, HalfSpace, Segment, Shape};
    use super::alternatives::{
        contact_manifold_convex_ball_early_reject, contact_manifold_convex_ball_inlined,
        contact_manifold_convex_ball_separate_norm,
    };
    use super::{
        contact_manifold_ball_convex, contact_manifold_convex_ball,
        contact_manifold_convex_ball_shapes,
    };

    const UNIT: i64 = 0x1_0000_0000;
    const PRED_RAW: i64 = 0x1000_0000;
    /// `2^-4`, exactly representable: the prediction of the boundary cases.
    const PREDICTION: Fixed = Fixed { raw: PRED_RAW };
    /// Tolerance of the non-dyadic normals: `1 ulp / |dpos|` plus the rounding of two products.
    const TOL: i64 = 3;

    #[derive(Copy, Drop)]
    struct Case {
        shape: Shape,
        centre: Vec2,
        radius: Fixed,
        num_points: u8,
        /// Manifold distance (already net of the ball radius).
        dist: i64,
        /// Normal in the convex frame, from the shape towards the ball.
        n: Vec2,
        /// Contact point on the convex shape.
        p: Vec2,
        fid: FeatureId,
    }

    fn f(raw: i64) -> Fixed {
        Fixed { raw }
    }

    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: f(x), y: f(y) }
    }

    fn ball(radius: Fixed) -> Ball {
        Ball { radius }
    }

    fn translation(x: i64, y: i64) -> Pose2 {
        Pose2Trait::new(v(x, y), Rot2 { re: f(UNIT), im: f(0) })
    }

    fn fresh() -> ContactManifold {
        Default::default()
    }

    fn cuboid() -> Shape {
        Shape::Cuboid(Cuboid { half_extents: v(UNIT, UNIT / 2) })
    }

    fn segment() -> Shape {
        Shape::Segment(Segment { a: v(-UNIT, 0), b: v(UNIT, 0) })
    }

    fn halfspace() -> Shape {
        Shape::HalfSpace(HalfSpace { normal: v(0, UNIT) })
    }

    fn capsule() -> Shape {
        Shape::Capsule(Capsule { segment: Segment { a: v(0, -UNIT), b: v(0, UNIT) }, radius: HALF })
    }

    fn unit_ball() -> Shape {
        Shape::Ball(ball(ONE))
    }

    /// `x / 4` and `y / 4` of the 3-4-5 direction, i.e. `(0.6, 0.8)` in raw units.
    const N35: Vec2 = Vec2 { x: Fixed { raw: 2576980378 }, y: Fixed { raw: 3435973837 } };
    const NONE: Vec2 = Vec2 { x: Fixed { raw: 0 }, y: Fixed { raw: 0 } };

    fn case(
        shape: Shape,
        cx: i64,
        cy: i64,
        radius: i64,
        dist: i64,
        n: Vec2,
        px: i64,
        py: i64,
        fid: FeatureId,
    ) -> Case {
        let num_points = if n == NONE {
            0
        } else {
            1
        };
        Case { shape, centre: v(cx, cy), radius: f(radius), num_points, dist, n, p: v(px, py), fid }
    }

    fn close(actual: Fixed, expected: i64, tol: i64) -> bool {
        let d = actual.raw - expected;
        -tol <= d && d <= tol
    }

    fn close_vec(actual: Vec2, expected: Vec2, tol: i64) -> bool {
        close(actual.x, expected.x.raw, tol) && close(actual.y, expected.y.raw, tol)
    }

    fn cases() -> Span<Case> {
        let up = v(0, UNIT);
        let right = v(UNIT, 0);
        let (u1, u2, u4, u8) = (UNIT, UNIT / 2, UNIT / 4, UNIT / 8);
        let f0 = FeatureIdTrait::face(0);
        array![
            // Cuboid, half extents (1, 0.5): beyond the reach, exactly at it, one ulp past it.
            case(cuboid(), 3 * u1, 0, u2, 0, NONE, 0, 0, f0),
            case(cuboid(), 6710886400, u4, u2, PRED_RAW, right, u1, u4, f0),
            case(cuboid(), 6710886401, u4, u2, 0, NONE, 0, 0, f0),
            // Touching a face, deep inside (through the -y face), touching a vertex.
            case(cuboid(), 3 * u2, u4, u2, 0, right, u1, u4, f0),
            case(cuboid(), u4, -u8, u2, -7 * u8, v(0, -u1), u4, -u2, FeatureIdTrait::face(3)),
            case(cuboid(), 7 * u4, 3 * u2, 5 * u4, 0, N35, u1, u2, FeatureIdTrait::vertex(0)),
            // Segment: above (side 1), vertex region, centre exactly on it (fallback normal).
            case(segment(), u2, u1, u1, 0, up, u2, 0, FeatureIdTrait::face(1)),
            case(segment(), 7 * u4, u1, 5 * u4, 0, N35, u1, 0, FeatureIdTrait::vertex(1)),
            case(segment(), u2, 0, u1, -u1, v(-u1, 0), u2, 0, f0),
            // Half-space: touching, far inside, centre on the origin (fallback normal, inside).
            case(halfspace(), u4, u1, u1, 0, up, u4, 0, f0),
            case(halfspace(), 2 * u1, -3 * u1, u1, -4 * u1, up, 2 * u1, 0, f0),
            case(halfspace(), 0, 0, u1, -u1, v(0, -u1), 0, 0, f0),
            // Capsule: touching the side, centre inside.
            case(capsule(), 3 * u2, u4, u1, 0, right, u2, u4, f0),
            case(capsule(), u4, 0, u2, -3 * u4, right, u2, 0, f0),
            // Ball as shape 1: touching (3-4-5), centre at the centre (projects on +x).
            case(unit_ball(), 3 * u2, 2 * u1, 3 * u2, 0, N35, 2576980378, 3435973837, f0),
            case(unit_ball(), 0, 0, u2, -3 * u2, right, u1, 0, f0),
        ]
            .span()
    }

    /// Checks one manifold of `case`; `flipped` tells that the ball is shape 1 of `m` and that
    /// the pose was the identity-rotation translation of the ball in the convex frame.
    fn check(case: Case, m: ContactManifold, flipped: bool) {
        assert_eq!(m.num_points, case.num_points);
        if case.num_points == 0 {
            return;
        }
        let c = m.point(0);
        let face = FeatureIdTrait::face(0);
        assert!(close(c.dist, case.dist, TOL), "dist {} vs {}", c.dist.raw, case.dist);
        let (n_convex, n_ball) = if flipped {
            (m.local_n2, m.local_n1)
        } else {
            (m.local_n1, m.local_n2)
        };
        let (p_convex, p_ball, fid_convex, fid_ball) = if flipped {
            (c.local_p2, c.local_p1, c.fid2, c.fid1)
        } else {
            (c.local_p1, c.local_p2, c.fid1, c.fid2)
        };
        assert!(close_vec(n_convex, case.n, TOL), "n {:?} vs {:?}", n_convex, case.n);
        assert!(close_vec(p_convex, case.p, TOL), "p {:?} vs {:?}", p_convex, case.p);
        // Identity rotation: the ball normal is the opposite one, and its point sits on the circle.
        assert!(close_vec(n_ball, Vec2 { x: -case.n.x, y: -case.n.y }, TOL));
        assert!(
            close_vec(p_ball, Vec2 { x: n_ball.x * case.radius, y: n_ball.y * case.radius }, 1),
        );
        assert_eq!(fid_convex, case.fid);
        assert_eq!(fid_ball, face);
        assert_eq!(c.data, Default::default());
    }

    #[test]
    fn test_regimes_convex_first() {
        for case in cases() {
            let mut m = fresh();
            contact_manifold_convex_ball(
                translation((*case).centre.x.raw, (*case).centre.y.raw),
                (*case).shape,
                ball((*case).radius),
                PREDICTION,
                ref m,
            );
            check(*case, m, false);
        }
    }

    #[test]
    fn test_regimes_ball_first() {
        // The same scenes seen from the ball: `pos12` is the convex shape in the ball frame.
        for case in cases() {
            let mut m = fresh();
            contact_manifold_ball_convex(
                translation(-(*case).centre.x.raw, -(*case).centre.y.raw),
                ball((*case).radius),
                (*case).shape,
                PREDICTION,
                ref m,
            );
            check(*case, m, true);
        }
    }

    #[test]
    fn test_normals_follow_the_rotation() {
        // Ball touching the +x face of the cuboid, turned a quarter turn: n2 = -R^-1 n1 = +y.
        let pos12 = Pose2Trait::new(v(3 * UNIT / 2, UNIT / 4), Rot2 { re: f(0), im: f(UNIT) });
        let mut m = fresh();
        contact_manifold_convex_ball(pos12, cuboid(), ball(HALF), PREDICTION, ref m);
        assert_eq!(m.local_n1, v(UNIT, 0));
        assert_eq!(m.local_n2, v(0, UNIT));
        assert_eq!(m.point(0).local_p2, v(0, UNIT / 2));
        // Flipped, the same scene: the ball is shape 1, `pos12` becomes the inverse pose.
        let pos21 = pos12.inverse();
        let mut m = fresh();
        contact_manifold_ball_convex(pos21, ball(HALF), cuboid(), PREDICTION, ref m);
        assert_eq!(m.local_n2, v(UNIT, 0));
        assert_eq!(m.local_n1, v(0, UNIT));
        assert_eq!(m.point(0).local_p1, v(0, UNIT / 2));
        assert_eq!(m.point(0).fid1, FeatureIdTrait::face(0));
        assert_eq!(m.point(0).fid2, FeatureIdTrait::face(0));
    }

    #[test]
    fn test_warm_start_data_and_stale_state() {
        let data = ContactData { impulse: ONE, ..Default::default() };
        let old = TrackedContact { data, ..Default::default() };
        let pose = translation(3 * UNIT / 2, UNIT / 4);
        // Exactly one point: the data survives, the geometry is refreshed.
        let mut m = fresh();
        m.points = [old, old];
        m.num_points = 1;
        contact_manifold_convex_ball(pose, cuboid(), ball(HALF), PREDICTION, ref m);
        assert_eq!((m.num_points, m.point(0).data), (1, data));
        // Zero or two points: cleared first, so the data restarts from default.
        for stale in array![0_u8, 2].span() {
            let mut m = fresh();
            m.points = [old, old];
            m.num_points = *stale;
            contact_manifold_convex_ball(pose, cuboid(), ball(HALF), PREDICTION, ref m);
            assert_eq!((m.num_points, m.point(0).data), (1, Default::default()));
        }
        // Beyond the reach: cleared, normals kept.
        let mut m = fresh();
        contact_manifold_convex_ball(pose, cuboid(), ball(HALF), PREDICTION, ref m);
        let n1 = m.local_n1;
        contact_manifold_convex_ball(
            translation(9 * UNIT, 0), cuboid(), ball(HALF), PREDICTION, ref m,
        );
        assert_eq!((m.num_points, m.local_n1), (0, n1));
    }

    #[test]
    fn test_shapes_wrapper_dispatch() {
        let pose = translation(3 * UNIT / 2, UNIT / 4);
        let ball_shape = Shape::Ball(ball(HALF));
        let mut expected = fresh();
        contact_manifold_convex_ball(pose, cuboid(), ball(HALF), PREDICTION, ref expected);
        let mut m = fresh();
        assert!(contact_manifold_convex_ball_shapes(pose, cuboid(), ball_shape, PREDICTION, ref m));
        assert_eq!(m, expected);
        let mut expected = fresh();
        contact_manifold_ball_convex(pose, ball(HALF), cuboid(), PREDICTION, ref expected);
        let mut m = fresh();
        assert!(contact_manifold_convex_ball_shapes(pose, ball_shape, cuboid(), PREDICTION, ref m));
        assert_eq!(m, expected);
        // Neither is a ball: unhandled, untouched.
        let mut m = fresh();
        for other in array![segment(), halfspace(), capsule(), cuboid()].span() {
            assert!(
                !contact_manifold_convex_ball_shapes(pose, cuboid(), *other, PREDICTION, ref m),
            );
        }
        assert_eq!(m, fresh());
    }

    #[test]
    #[fuzzer(runs: 64, seed: 20260921)]
    fn fuzz_alternatives_match(x: i16, y: i16, r: u8) {
        let radius = ball(f(r.into() * 0x0100_0000));
        let pose = Pose2Trait::new(
            v(x.into() * 0x2_0000 + 3, y.into() * 0x1_8000 - 5),
            Rot2 { re: f(2576980378), im: f(3435973837) },
        );
        for shape in array![cuboid(), segment(), halfspace(), capsule(), unit_ball()].span() {
            let mut a = fresh();
            let mut b = fresh();
            let mut c = fresh();
            let mut d = fresh();
            contact_manifold_convex_ball(pose, *shape, radius, PREDICTION, ref a);
            contact_manifold_convex_ball_separate_norm(
                pose, *shape, radius, PREDICTION, false, ref b,
            );
            contact_manifold_convex_ball_early_reject(
                pose, *shape, radius, PREDICTION, false, ref c,
            );
            contact_manifold_convex_ball_inlined(pose, *shape, radius, PREDICTION, ref d);
            assert_eq!(a, b);
            assert_eq!(a, c);
            assert_eq!(a, d);
        }
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }

    /// A ball of radius 0.5 whose centre is `0.03` outside the reference shape, so that every
    /// shape produces a point and none is in a degenerate branch.
    fn probe(shape: Shape, x: i64, y: i64) {
        let mut m = fresh();
        contact_manifold_convex_ball(
            opaque(translation(x, y)), opaque(shape), opaque(ball(HALF)), opaque(PREDICTION), ref m,
        );
    }

    #[test]
    fn gas_convex_ball_cuboid() {
        probe(cuboid(), 3 * UNIT / 2, UNIT / 4);
    }

    #[test]
    fn gas_convex_ball_capsule() {
        probe(capsule(), UNIT, UNIT / 4);
    }

    #[test]
    fn gas_convex_ball_segment() {
        probe(segment(), UNIT / 2, UNIT / 2);
    }

    #[test]
    fn gas_convex_ball_halfspace() {
        probe(halfspace(), UNIT / 2, UNIT / 2);
    }

    #[test]
    fn gas_convex_ball_ball() {
        probe(unit_ball(), 3 * UNIT / 2, UNIT / 4);
    }

    #[test]
    fn gas_convex_ball_cuboid_separated() {
        probe(cuboid(), 5 * UNIT, UNIT / 4);
    }

    /// Opaque `(pose, ball, prediction)` of the alternative and wrapper probes: the ball touches
    /// the +x face of the cuboid, `x` is its centre abscissa.
    fn opaque_args(x: i64) -> (Pose2, Ball, Fixed) {
        (opaque(translation(x, UNIT / 4)), opaque(ball(HALF)), opaque(PREDICTION))
    }

    #[test]
    fn gas_ball_convex_cuboid() {
        let (pose, b, pred) = opaque_args(-3 * UNIT / 2);
        let mut m = fresh();
        contact_manifold_ball_convex(pose, b, opaque(cuboid()), pred, ref m);
    }

    #[test]
    fn gas_convex_ball_shapes_flipped() {
        let (pose, b, pred) = opaque_args(-3 * UNIT / 2);
        let mut m = fresh();
        let _ = contact_manifold_convex_ball_shapes(
            pose, opaque(Shape::Ball(b)), opaque(cuboid()), pred, ref m,
        );
    }

    #[test]
    fn gas_convex_ball_shapes_direct() {
        let (pose, b, pred) = opaque_args(3 * UNIT / 2);
        let mut m = fresh();
        let _ = contact_manifold_convex_ball_shapes(
            pose, opaque(cuboid()), opaque(Shape::Ball(b)), pred, ref m,
        );
    }

    #[test]
    fn gas_convex_ball_cuboid_separate_norm() {
        let (pose, b, pred) = opaque_args(3 * UNIT / 2);
        let mut m = fresh();
        contact_manifold_convex_ball_separate_norm(pose, opaque(cuboid()), b, pred, false, ref m);
    }

    #[test]
    fn gas_convex_ball_cuboid_early_reject() {
        let (pose, b, pred) = opaque_args(3 * UNIT / 2);
        let mut m = fresh();
        contact_manifold_convex_ball_early_reject(pose, opaque(cuboid()), b, pred, false, ref m);
    }

    #[test]
    fn gas_convex_ball_cuboid_early_reject_separated() {
        let (pose, b, pred) = opaque_args(5 * UNIT);
        let mut m = fresh();
        contact_manifold_convex_ball_early_reject(pose, opaque(cuboid()), b, pred, false, ref m);
    }

    #[test]
    fn gas_convex_ball_cuboid_inlined() {
        let (pose, b, pred) = opaque_args(3 * UNIT / 2);
        let mut m = fresh();
        contact_manifold_convex_ball_inlined(pose, opaque(cuboid()), b, pred, ref m);
    }

    #[test]
    fn gas_convex_ball_halfspace_inlined() {
        let (pose, b, pred) = opaque_args(3 * UNIT / 2);
        let mut m = fresh();
        contact_manifold_convex_ball_inlined(pose, opaque(halfspace()), b, pred, ref m);
    }
}
