//! Non-uniform scaling of balls, capsules and convex polygons (Parry `Ball::scaled`,
//! `Capsule::scaled`, `ConvexPolygon::scaled`; PX3).
//!
//! A ball or a capsule under a non-uniform scale is no longer a ball or a capsule: upstream
//! samples its outline (`to_polyline(nsubdivs)`), scales the points and builds a
//! [`ConvexPolygon`] of them. The port's polygon holds at most 8 vertices, so the outline
//! functions answer `None` beyond that (see [`ball_outline`] and [`capsule_outline`]); a
//! polygon that cannot be built from the scaled points (collinear or reversed) is `None` too,
//! as upstream's `from_convex_polyline(..)?`.

use fixed::trig::TrigTrait;
use fixed::{Fixed, PI_RAW, TAU_RAW, ZERO};
use glam_core::Vec2;
use rapier_math::pose2::Pose2Trait;
use super::capsule::{Capsule, CapsuleTrait};
use super::convex_polygon::{ConvexPolygon, ConvexPolygonTrait};

/// The port of the `either::Either` upstream's `scaled` returns: the shape itself (`Left`) for a
/// uniform scale, its polygon approximation (`Right`) otherwise.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum Either<L, R> {
    Left: L,
    Right: R,
}

/// The `n` points `(cos(theta) * radius, sin(theta) * radius)`, `theta = k * arc / n` for point
/// `k` (upstream `push_xy_arc`): the step `arc / n` is rounded to nearest once and accumulated,
/// as upstream does, so the angle of point `k` is off by at most `k / 2` ulp (`k < 8`).
/// Ranked against one rounded division per point in `alternatives` (41k Sierra gas cheaper for 8
/// points).
fn arc_points(radius: Fixed, arc_raw: i64, n: u32) -> Array<Vec2> {
    let mut points = array![];
    let n_raw: i64 = n.into();
    let step = Fixed { raw: (arc_raw + n_raw / 2) / n_raw };
    let mut theta = ZERO;
    let mut k: i64 = 0;
    while k != n_raw {
        let (sin, cos) = theta.sin_cos();
        points.append(Vec2 { x: cos * radius, y: sin * radius });
        theta = theta + step;
        k += 1;
    }
    points
}

/// The outline of a ball (upstream `Ball::to_polyline(nsubdivs)`): `nsubdivs` points on the
/// circle, counter-clockwise from `(radius, 0)`.
/// #### Panics
/// * `'Fixed: overflow'` if a coordinate leaves the scalar range.
/// #### Deviations
/// * `None` for fewer than 3 or more than 8 subdivisions (the `ConvexPolygon` capacity).
pub fn ball_outline(radius: Fixed, nsubdivs: u32) -> Option<Array<Vec2>> {
    if nsubdivs < 3 || nsubdivs > 8 {
        return None;
    }
    Some(arc_points(radius, TAU_RAW, nsubdivs))
}

/// The outline of a capsule (upstream `Capsule::to_polyline(nsubdiv)`): `2 * nsubdiv` points, a
/// half circle of `nsubdiv` points about each end of the core, in the frame of the capsule.
/// #### Panics
/// * `'Fixed: overflow'` if a coordinate leaves the scalar range.
/// #### Deviations
/// * `None` for fewer than 2 or more than 4 subdivisions (the `ConvexPolygon` capacity).
pub fn capsule_outline(capsule: Capsule, nsubdiv: u32) -> Option<Array<Vec2>> {
    if nsubdiv < 2 || nsubdiv > 4 {
        return None;
    }
    let half_height = capsule.half_height();
    let arc = arc_points(capsule.radius, PI_RAW, nsubdiv);
    let pose = capsule.canonical_transform();
    let mut top = array![];
    let mut bottom = array![];
    for p in arc.span() {
        let up = Vec2 { x: *p.x, y: *p.y + half_height };
        top.append(pose.transform_point(up));
        bottom.append(pose.transform_point(-up));
    }
    let mut points = top;
    for p in bottom.span() {
        points.append(*p);
    }
    Some(points)
}

/// The polygon of `points` scaled component-wise by `scale`.
pub fn scaled_polygon(points: Span<Vec2>, scale: Vec2) -> Option<ConvexPolygon> {
    let mut scaled = array![];
    for p in points {
        scaled.append(*p * scale);
    }
    ConvexPolygonTrait::from_convex_polyline(scaled.span())
}

/// Rejected candidates, kept for the `gas_*` ranking.
#[cfg(test)]
pub mod alternatives {
    use fixed::Fixed;
    use fixed::trig::TrigTrait;
    use glam_core::Vec2;

    /// [`super::arc_points`] with the angle of every point rounded once,
    /// `theta = arc * k / n`: exact to the ulp, but one `i64` division per point.
    pub fn arc_points_divided(radius: Fixed, arc_raw: i64, n: u32) -> Array<Vec2> {
        let mut points = array![];
        let n_raw: i64 = n.into();
        let mut k: i64 = 0;
        while k != n_raw {
            let (sin, cos) = Fixed { raw: arc_raw * k / n_raw }.sin_cos();
            points.append(Vec2 { x: cos * radius, y: sin * radius });
            k += 1;
        }
        points
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, ONE, ZERO};
    use glam_core::Vec2;
    use rapier_testing::opaque;
    use crate::shape::ball::BallTrait;
    use crate::shape::capsule::{Capsule, CapsuleTrait};
    use crate::shape::convex_polygon::{ConvexPolygon, ConvexPolygonTrait};
    use super::{Either, ball_outline, capsule_outline, scaled_polygon};

    fn f(n: i64) -> Fixed {
        Fixed { raw: n * 0x1_0000_0000 }
    }

    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: f(x), y: f(y) }
    }

    fn triangle() -> ConvexPolygon {
        ConvexPolygonTrait::from_convex_polyline(array![v(0, 0), v(2, 0), v(0, 2)].span()).unwrap()
    }

    #[test]
    fn test_outlines_fit_the_polygon_capacity() {
        assert!(ball_outline(ONE, 2).is_none());
        assert!(ball_outline(ONE, 9).is_none());
        assert_eq!(ball_outline(ONE, 8).unwrap().len(), 8);
        assert!(capsule_outline(CapsuleTrait::new_y(ONE, ONE), 1).is_none());
        assert!(capsule_outline(CapsuleTrait::new_y(ONE, ONE), 5).is_none());
        assert_eq!(capsule_outline(CapsuleTrait::new_y(ONE, ONE), 4).unwrap().len(), 8);
    }

    #[test]
    fn test_square_outline_starts_on_the_x_axis() {
        // Four subdivisions: `(r, 0)`, `(0, r)`, `(-r, 0)`, `(0, -r)`, to a few ulps.
        let pts = ball_outline(f(2), 4).unwrap();
        let near = |p: Vec2, x: i64, y: i64| {
            let dx = p.x.raw - x * 0x1_0000_0000;
            let dy = p.y.raw - y * 0x1_0000_0000;
            assert!(dx >= -4 && dx <= 4 && dy >= -4 && dy <= 4);
        };
        near(*pts.at(0), 2, 0);
        near(*pts.at(1), 0, 2);
        near(*pts.at(2), -2, 0);
        near(*pts.at(3), 0, -2);
    }

    #[test]
    fn test_uniform_scale_keeps_the_shape() {
        match BallTrait::scaled(BallTrait::new(f(2)), Vec2 { x: f(-3), y: f(-3) }, 8) {
            Some(Either::Left(b)) => assert_eq!(b, BallTrait::new(f(6))),
            _ => panic!("expected a ball"),
        }
        let capsule = CapsuleTrait::new(v(-1, 0), v(1, 2), ONE);
        match CapsuleTrait::scaled(capsule, Vec2 { x: f(2), y: f(2) }, 4) {
            Some(Either::Left(c)) => assert_eq!(c, CapsuleTrait::new(v(-2, 0), v(2, 4), f(2))),
            _ => panic!("expected a capsule"),
        }
    }

    #[test]
    fn test_non_uniform_scale_gives_a_polygon() {
        match BallTrait::scaled(BallTrait::new(ONE), v(2, 1), 6) {
            Some(Either::Right(p)) => assert_eq!(p.count, 6),
            _ => panic!("expected a polygon"),
        }
        match CapsuleTrait::scaled(CapsuleTrait::new_x(ONE, ONE), v(1, 2), 3) {
            Some(Either::Right(p)) => assert_eq!(p.count, 6),
            _ => panic!("expected a polygon"),
        }
        // No room for the polygon, a collapsed axis, a reversed outline: none.
        assert!(BallTrait::scaled(BallTrait::new(ONE), v(2, 1), 12).is_none());
        assert!(BallTrait::scaled(BallTrait::new(ONE), v(0, 1), 6).is_none());
        assert!(BallTrait::scaled(BallTrait::new(ONE), v(-1, 2), 6).is_none());
        assert!(CapsuleTrait::scaled(CapsuleTrait::new_x(ONE, ONE), v(1, 2), 5).is_none());
    }

    #[test]
    fn test_polygon_scaled() {
        let p = ConvexPolygonTrait::scaled(triangle(), v(2, 2)).unwrap();
        assert_eq!((p.vertex(1), p.vertex(2)), (v(4, 0), v(0, 4)));
        // Uniform scale keeps the unit normals; padding stays zero.
        assert_eq!(p.normal(0), triangle().normal(0));
        let [_, _, _, _, _, _, _, n7] = p.normals();
        assert_eq!(n7, Vec2 { x: ZERO, y: ZERO });
        // Upstream's formula: `normalize(n * scale)`.
        let q = ConvexPolygonTrait::scaled(triangle(), v(3, 1)).unwrap();
        assert_eq!(q.vertex(1), v(6, 0));
        assert_eq!(q.normal(0), Vec2 { x: ZERO, y: -ONE });
        // A normal collapsing to zero: none.
        assert!(ConvexPolygonTrait::scaled(triangle(), v(0, 1)).is_none());
    }

    #[test]
    fn test_scaled_polygon_rejects_a_reversed_outline() {
        let pts = ball_outline(ONE, 4).unwrap();
        assert!(scaled_polygon(pts.span(), v(1, 2)).is_some());
        assert!(scaled_polygon(pts.span(), v(-1, 2)).is_none());
    }

    #[test]
    fn test_accumulated_angles_stay_within_a_few_ulps_of_the_divided_ones() {
        let accumulated = super::arc_points(f(2), fixed::TAU_RAW, 8);
        let exact = super::alternatives::arc_points_divided(f(2), fixed::TAU_RAW, 8);
        let mut k = 0;
        while k != 8 {
            let (a, b) = (*exact.at(k), *accumulated.at(k));
            let (dx, dy) = (a.x.raw - b.x.raw, a.y.raw - b.y.raw);
            assert!(dx >= -8 && dx <= 8 && dy >= -8 && dy <= 8);
            k += 1;
        }
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_arc_points() {
        let _ = super::arc_points(opaque(f(2)), opaque(fixed::TAU_RAW), opaque(8));
    }

    #[test]
    fn gas_arc_points_divided() {
        let _ = super::alternatives::arc_points_divided(
            opaque(f(2)), opaque(fixed::TAU_RAW), opaque(8),
        );
    }

    #[test]
    fn gas_ball_outline() {
        let _ = ball_outline(opaque(ONE), opaque(8));
    }

    #[test]
    fn gas_capsule_outline() {
        let _ = capsule_outline(opaque(CapsuleTrait::new(v(-1, 0), v(1, 2), ONE)), opaque(4));
    }

    #[test]
    fn gas_ball_scaled_uniform() {
        let _ = BallTrait::scaled(opaque(BallTrait::new(f(2))), opaque(v(2, 2)), opaque(8));
    }

    #[test]
    fn gas_ball_scaled_polygon() {
        let _ = BallTrait::scaled(opaque(BallTrait::new(f(2))), opaque(v(2, 1)), opaque(8));
    }

    #[test]
    fn gas_capsule_scaled_uniform() {
        let c: Capsule = CapsuleTrait::new(v(-1, 0), v(1, 2), ONE);
        let _ = CapsuleTrait::scaled(opaque(c), opaque(v(2, 2)), opaque(4));
    }

    #[test]
    fn gas_capsule_scaled_polygon() {
        let c: Capsule = CapsuleTrait::new(v(-1, 0), v(1, 2), ONE);
        let _ = CapsuleTrait::scaled(opaque(c), opaque(v(2, 1)), opaque(4));
    }

    #[test]
    fn gas_convex_polygon_scaled() {
        let _ = ConvexPolygonTrait::scaled(opaque(triangle()), opaque(v(3, 1)));
    }

    #[test]
    fn gas_scaled_polygon() {
        let pts = ball_outline(ONE, 8).unwrap();
        let _ = scaled_polygon(opaque(pts).span(), opaque(v(2, 1)));
    }
}
