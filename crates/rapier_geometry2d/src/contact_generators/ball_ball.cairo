//! Contact manifold between two balls (Parry `contact_manifolds_ball_ball.rs`).
//!
//! One contact point at most: on the line through the two centres, on each circle, with
//! `fid1 = fid2 = Face(0)` (the f32 ids of the golden vectors; a ball has a single face).
//!
//! # Fixed-point choices
//!
//! * **One square root.** The wide squared length of `pos12.translation` (`norm2_wide`) is shared
//!   by the distance (`to_fixed`, floored to 1 ulp) and by the normal (`recip`, then two
//!   products), so the whole function costs a single integer square root and no `Fixed` division.
//! * **`<`, not `<=`.** The point is kept when `dist < prediction`, upstream's only strict test.
//! * **Warm start.** Like upstream's `copy_geometry_from`, an existing point keeps its
//!   `ContactData`; only its geometry is refreshed. `match_contacts` is the caller's job.
//!
//! # Candidates
//!
//! Ranked by the `gas_*` probes of this module; the loser lives in `#[cfg(test)] mod
//! alternatives`.
//!
//! 1. **shared norm** (this module): `norm2_wide` once, `dist` and the normal read from it.
//! 2. `alternatives::contact_manifold_ball_ball_early_reject`: tests `|t|^2 < (r1 + r2 +
//!    prediction)^2` on the wide squared length first, so a separated pair never pays the square
//!    root; a pair in contact pays the wide comparison on top.

use fixed::wide::{NormTrait, RecipTrait, norm2_wide};
use fixed::{Fixed, ONE, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2Trait;
use crate::contact::{ContactManifold, ContactManifoldTrait, TrackedContact};
use crate::feature_id::FeatureIdTrait;
use crate::shape::{Ball, Shape};

/// Computes the contact manifold between two balls given as [`Shape`]s.
///
/// Mirrors `contact_manifold_ball_ball_shapes`: returns `true` and updates `manifold` when both
/// shapes are balls, returns `false` and leaves `manifold` untouched otherwise.
/// #### Panics
/// * See [`contact_manifold_ball_ball`].
/// #### Deviations
/// * Upstream returns `()`; the `bool` tells the dispatcher whether the pair was handled.
pub fn contact_manifold_ball_ball_shapes(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    match (shape1, shape2) {
        (
            Shape::Ball(ball1), Shape::Ball(ball2),
        ) => {
            contact_manifold_ball_ball(pos12, ball1, ball2, prediction, ref manifold);
            true
        },
        _ => false,
    }
}

/// Computes the contact manifold between two balls.
///
/// `pos12` is the pose of ball 2 in the frame of ball 1. The manifold gets one point when
/// `|pos12.translation| - r1 - r2 < prediction` (strict) and is cleared otherwise; in both cases
/// the previous normals are kept when nothing is written (upstream).
/// #### Panics
/// * `'Fixed: overflow'` if `|pos12.translation| >= 2^31`, or if a radius sum leaves the scalar
///   range.
/// #### Deviations
/// * The centre distance is the floored wide length (`fixed::wide::norm2_wide`), at most 1 ulp
///   below the exact one; the normal is `t * recip(length)`, so it is exact for axis-aligned
///   translations and within `~1 ulp / |t|` otherwise. Below `|t| = 1` the unit-normal tolerance
///   of `rapier_math::consts` degrades as documented there.
/// * Coincident centres (`t == 0`) give `local_n1 = +Y`, as upstream.
/// * A manifold that already holds a point keeps its `ContactData` (`copy_geometry_from`); the
///   second slot is never written and the count becomes 1.
pub fn contact_manifold_ball_ball(
    pos12: Pose2, ball1: Ball, ball2: Ball, prediction: Fixed, ref manifold: ContactManifold,
) {
    let t = pos12.translation;
    let n = norm2_wide(t.x, t.y);
    let dist = n.to_fixed() - ball1.radius - ball2.radius;
    if dist < prediction {
        let local_n1 = match n.try_recip() {
            Some(r) => Vec2 { x: r.mul(t.x), y: r.mul(t.y) },
            None => Vec2 { x: ZERO, y: ONE },
        };
        write_contact(pos12, ball1, ball2, local_n1, dist, ref manifold);
    } else {
        manifold.clear();
    }
}

/// Writes the single ball-ball point and the normals.
#[inline(always)]
fn write_contact(
    pos12: Pose2,
    ball1: Ball,
    ball2: Ball,
    local_n1: Vec2,
    dist: Fixed,
    ref manifold: ContactManifold,
) {
    let local_n2 = -pos12.rotation.inverse_rotate(local_n1);
    let [old, second] = manifold.points;
    let data = if manifold.num_points != 0 {
        old.data
    } else {
        Default::default()
    };
    let face = FeatureIdTrait::face(0);
    let contact = TrackedContact {
        local_p1: local_n1.mul_scalar(ball1.radius),
        local_p2: local_n2.mul_scalar(ball2.radius),
        dist,
        fid1: face,
        fid2: face,
        data,
    };
    manifold.points = [contact, second];
    manifold.num_points = 1;
    manifold.local_n1 = local_n1;
    manifold.local_n2 = local_n2;
}

/// Rejected candidates, kept for the `gas_*` ranking.
#[cfg(test)]
pub mod alternatives {
    use fixed::wide::{NormTrait, RecipTrait, norm2_wide};
    use fixed::{Fixed, ONE, ZERO};
    use glam::Vec2;
    use rapier_math::math_ext::norm2::is_norm2_lt;
    use rapier_math::pose2::Pose2;
    use crate::contact::{ContactManifold, ContactManifoldTrait};
    use crate::shape::{Ball, Shape, ShapeTrait};
    use super::{contact_manifold_ball_ball, write_contact};

    /// Same answers as [`super::contact_manifold_ball_ball_shapes`], but downcasting with two
    /// `as_ball` calls and matching the resulting `Option` pair, as upstream's
    /// `if let (Some(..), Some(..))` does.
    pub fn contact_manifold_ball_ball_shapes_as_ball(
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) -> bool {
        match (shape1.as_ball(), shape2.as_ball()) {
            (
                Some(ball1), Some(ball2),
            ) => {
                contact_manifold_ball_ball(pos12, ball1, ball2, prediction, ref manifold);
                true
            },
            _ => false,
        }
    }

    /// Same answers as [`super::contact_manifold_ball_ball`], but the strict test is done on the
    /// wide squared length (`floor(L) < t` iff `L < t` for a representable `t`), so a separated
    /// pair skips the square root. A pair in contact pays the extra wide comparison.
    pub fn contact_manifold_ball_ball_early_reject(
        pos12: Pose2, ball1: Ball, ball2: Ball, prediction: Fixed, ref manifold: ContactManifold,
    ) {
        let t = pos12.translation;
        if is_norm2_lt(t.x, t.y, ball1.radius + ball2.radius + prediction) {
            let n = norm2_wide(t.x, t.y);
            let dist = n.to_fixed() - ball1.radius - ball2.radius;
            let local_n1 = match n.try_recip() {
                Some(r) => Vec2 { x: r.mul(t.x), y: r.mul(t.y) },
                None => Vec2 { x: ZERO, y: ONE },
            };
            write_contact(pos12, ball1, ball2, local_n1, dist, ref manifold);
        } else {
            manifold.clear();
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, HALF, ONE, TWO};
    use glam::Vec2;
    use rapier_math::pose2::Pose2;
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::contact::{ContactData, ContactManifold, ContactManifoldTrait, TrackedContact};
    use crate::feature_id::FeatureIdTrait;
    use crate::shape::{Ball, Cuboid, Shape};
    use super::alternatives::{
        contact_manifold_ball_ball_early_reject, contact_manifold_ball_ball_shapes_as_ball,
    };
    use super::{contact_manifold_ball_ball, contact_manifold_ball_ball_shapes};

    const UNIT: i64 = 0x1_0000_0000;
    /// `2^-4`, exactly representable: the prediction of the boundary cases.
    const PREDICTION: Fixed = Fixed { raw: 0x1000_0000 };

    fn f(raw: i64) -> Fixed {
        Fixed { raw }
    }

    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: f(x), y: f(y) }
    }

    fn ball(radius: Fixed) -> Ball {
        Ball { radius }
    }

    fn pose(x: i64, y: i64, re: i64, im: i64) -> Pose2 {
        Pose2 { translation: v(x, y), rotation: Rot2 { re: f(re), im: f(im) } }
    }

    fn translation(x: i64, y: i64) -> Pose2 {
        pose(x, y, UNIT, 0)
    }

    fn fresh() -> ContactManifold {
        Default::default()
    }

    /// (translation, r1, r2, expected num_points, expected dist raw, expected local_n1)
    fn cases() -> Span<(Pose2, Fixed, Fixed, u8, i64, Vec2)> {
        array![
            // Beyond the prediction distance: no point.
            (translation(3 * UNIT, 0), ONE, ONE, 0, 0, v(0, 0)),
            // dist == prediction is *not* kept (`<`)...
            (translation(2 * UNIT + 0x1000_0000, 0), ONE, ONE, 0, 0, v(0, 0)),
            // ...one ulp below it is.
            (translation(2 * UNIT + 0x0fff_ffff, 0), ONE, ONE, 1, 0x0fff_ffff, v(UNIT, 0)),
            // Touching: the 3-4-5 triangle, distance exactly 1.25 = 0.75 + 0.5.
            (
                translation(3 * UNIT / 4, UNIT),
                f(3 * UNIT / 4),
                HALF,
                1,
                0,
                v(2576980378, 3435973837),
            ),
            // Penetrating along -y.
            (translation(0, -UNIT / 2), ONE, HALF, 1, -UNIT, v(0, -UNIT)),
            // Centre of ball 2 inside ball 1, off axis: |t| = 5 / 2 = 2.5 with r1 + r2 = 4.
            (
                translation(3 * UNIT / 2, 2 * UNIT),
                TWO,
                TWO,
                1,
                -3 * UNIT / 2,
                v(2576980378, 3435973837),
            ),
            // Coincident centres: upstream's fallback normal is +Y.
            (translation(0, 0), ONE, HALF, 1, -3 * UNIT / 2, v(0, UNIT)),
            // One ulp apart: still a real direction, however noisy.
            (translation(1, 0), ONE, HALF, 1, 1 - 3 * UNIT / 2, v(UNIT, 0)),
        ]
            .span()
    }

    #[test]
    fn test_regimes() {
        for case in cases() {
            let (pos12, r1, r2, num_points, dist, n1) = *case;
            let mut m = fresh();
            contact_manifold_ball_ball(pos12, ball(r1), ball(r2), PREDICTION, ref m);
            assert_eq!(m.num_points, num_points);
            if num_points == 0 {
                continue;
            }
            let c = m.point(0);
            assert_eq!(c.dist.raw, dist);
            assert_eq!(m.local_n1, n1);
            assert_eq!(c.local_p1, Vec2 { x: n1.x * r1, y: n1.y * r1 });
            assert_eq!(c.fid1, FeatureIdTrait::face(0));
            assert_eq!(c.fid2, FeatureIdTrait::face(0));
            assert_eq!(c.data, Default::default());
        }
    }

    #[test]
    fn test_normals_follow_the_rotation() {
        // Ball 2 sits at +x of ball 1 and is turned a quarter turn: n2 = -R^-1 * n1.
        let mut m = fresh();
        contact_manifold_ball_ball(
            pose(UNIT, 0, 0, UNIT), ball(HALF), ball(HALF), PREDICTION, ref m,
        );
        assert_eq!(m.local_n1, v(UNIT, 0));
        assert_eq!(m.local_n2, v(0, UNIT));
        assert_eq!(m.point(0).local_p2, v(0, UNIT / 2));
        // Coincident centres, half turn: n1 = +Y, n2 = -R^-1 * +Y = +Y.
        let mut m = fresh();
        contact_manifold_ball_ball(pose(0, 0, -UNIT, 0), ball(HALF), ball(HALF), PREDICTION, ref m);
        assert_eq!(m.local_n1, v(0, UNIT));
        assert_eq!(m.local_n2, v(0, UNIT));
    }

    #[test]
    fn test_warm_start_data_and_stale_state() {
        let data = ContactData { impulse: ONE, ..Default::default() };
        let old = TrackedContact { data, ..Default::default() };
        let stale = TrackedContact { dist: TWO, ..old };
        // A live point keeps its ContactData, its geometry is refreshed.
        let mut m = fresh();
        m.points = [old, stale];
        m.num_points = 1;
        contact_manifold_ball_ball(translation(UNIT, 0), ball(ONE), ball(ONE), PREDICTION, ref m);
        assert_eq!(m.num_points, 1);
        assert_eq!(m.point(0).data, data);
        assert_eq!(m.point(0).dist, -ONE);
        // A cleared manifold with stale slots starts from default data.
        let mut m = fresh();
        m.points = [old, stale];
        contact_manifold_ball_ball(translation(UNIT, 0), ball(ONE), ball(ONE), PREDICTION, ref m);
        assert_eq!(m.num_points, 1);
        assert_eq!(m.point(0).data, Default::default());
        // Separation clears the points and keeps the normals (upstream).
        contact_manifold_ball_ball(
            translation(9 * UNIT, 0), ball(ONE), ball(ONE), PREDICTION, ref m,
        );
        assert_eq!(m.num_points, 0);
        assert_eq!(m.local_n1, v(UNIT, 0));
    }

    #[test]
    fn test_shapes_wrapper_handles_only_ball_pairs() {
        let b = Shape::Ball(ball(ONE));
        let c = Shape::Cuboid(Cuboid { half_extents: v(UNIT, UNIT) });
        let mut m = fresh();
        assert!(contact_manifold_ball_ball_shapes(translation(UNIT, 0), b, b, PREDICTION, ref m));
        assert_eq!(m.num_points, 1);
        let before = m;
        assert!(
            !contact_manifold_ball_ball_shapes(translation(9 * UNIT, 0), b, c, PREDICTION, ref m),
        );
        assert!(
            !contact_manifold_ball_ball_shapes(translation(9 * UNIT, 0), c, b, PREDICTION, ref m),
        );
        assert!(
            !contact_manifold_ball_ball_shapes(translation(9 * UNIT, 0), c, c, PREDICTION, ref m),
        );
        assert_eq!(m, before);
    }

    #[test]
    #[fuzzer(runs: 64, seed: 20260921)]
    fn fuzz_early_reject_matches(x: i16, y: i16, r: u8) {
        let t = pose(x.into() * 0x2_0000 + 3, y.into() * 0x1_8000 - 5, 2576980378, 3435973837);
        let r1 = f(r.into() * 0x0100_0000);
        let mut a = fresh();
        let mut b = fresh();
        contact_manifold_ball_ball(t, ball(r1), ball(HALF), PREDICTION, ref a);
        contact_manifold_ball_ball_early_reject(t, ball(r1), ball(HALF), PREDICTION, ref b);
        assert_eq!(a, b);
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }

    #[test]
    fn gas_ball_ball_contact() {
        let mut m = fresh();
        contact_manifold_ball_ball(
            opaque(pose(UNIT, UNIT / 2, 2576980378, 3435973837)),
            opaque(ball(ONE)),
            opaque(ball(HALF)),
            opaque(PREDICTION),
            ref m,
        );
    }

    #[test]
    fn gas_ball_ball_separated() {
        let mut m = fresh();
        contact_manifold_ball_ball(
            opaque(pose(5 * UNIT, UNIT / 2, 2576980378, 3435973837)),
            opaque(ball(ONE)),
            opaque(ball(HALF)),
            opaque(PREDICTION),
            ref m,
        );
    }

    #[test]
    fn gas_ball_ball_coincident() {
        let mut m = fresh();
        contact_manifold_ball_ball(
            opaque(pose(0, 0, 2576980378, 3435973837)),
            opaque(ball(ONE)),
            opaque(ball(HALF)),
            opaque(PREDICTION),
            ref m,
        );
    }

    #[test]
    fn gas_ball_ball_shapes() {
        let mut m = fresh();
        let _ = contact_manifold_ball_ball_shapes(
            opaque(pose(UNIT, UNIT / 2, 2576980378, 3435973837)),
            opaque(Shape::Ball(ball(ONE))),
            opaque(Shape::Ball(ball(HALF))),
            opaque(PREDICTION),
            ref m,
        );
    }

    #[test]
    fn gas_ball_ball_early_reject_contact() {
        let mut m = fresh();
        contact_manifold_ball_ball_early_reject(
            opaque(pose(UNIT, UNIT / 2, 2576980378, 3435973837)),
            opaque(ball(ONE)),
            opaque(ball(HALF)),
            opaque(PREDICTION),
            ref m,
        );
    }

    #[test]
    fn gas_ball_ball_early_reject_separated() {
        let mut m = fresh();
        contact_manifold_ball_ball_early_reject(
            opaque(pose(5 * UNIT, UNIT / 2, 2576980378, 3435973837)),
            opaque(ball(ONE)),
            opaque(ball(HALF)),
            opaque(PREDICTION),
            ref m,
        );
    }

    #[test]
    fn gas_ball_ball_shapes_as_ball() {
        let mut m = fresh();
        let _ = contact_manifold_ball_ball_shapes_as_ball(
            opaque(pose(UNIT, UNIT / 2, 2576980378, 3435973837)),
            opaque(Shape::Ball(ball(ONE))),
            opaque(Shape::Ball(ball(HALF))),
            opaque(PREDICTION),
            ref m,
        );
    }
}
