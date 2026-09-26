//! Cuboid–cuboid kernels of the shape-pair queries (Parry `distance_cuboid_cuboid.rs`,
//! `closest_points_cuboid_cuboid.rs`, `contact_cuboid_cuboid.rs`), a literal port on top of
//! `crate::sat::cuboid_cuboid_find_local_separating_normal_oneway`.
//!
//! Upstream's dispatcher uses [`distance_cuboid_cuboid`] for `distance` only; `closest_points`
//! and `contact` of two cuboids go to GJK / EPA there and to [`super::support_map`] here. The
//! three functions are public upstream and ported with their own semantics: the witness is the
//! support point of one cuboid along the best SAT axis projected on the other and back, which is
//! the exact closest pair in 2D.

use fixed::{Fixed, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::consts::DEFAULT_EPSILON;
use rapier_math::math_ext::norm2::is_norm2_gt;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::point::project_local_point_cuboid;
use crate::sat::cuboid_cuboid_find_local_separating_normal_oneway as oneway;
use crate::shape::{Cuboid, CuboidTrait};
use super::{ClosestPoints, Contact, ContactTrait, normalize_and_length};

/// `support_point(pos, dir)`: the support point of `cuboid` placed at `pos` along the
/// frame-of-`pos` direction `dir`, in that frame.
#[inline(always)]
fn support_point(cuboid: Cuboid, pos: Pose2, dir: Vec2) -> Vec2 {
    pos.transform_point(cuboid.local_support_point(pos.rotation.inverse_rotate(dir)))
}

/// Closest points of two cuboids within `margin` (upstream `closest_points_cuboid_cuboid`).
///
/// `Disjoint` when either one-way SAT separation exceeds `margin` or the witness pair is further
/// than `margin` apart (wide comparison); `Intersecting` when both separations are `<= 0`.
/// #### Panics
/// * The overflow panics of the SAT and of the pose transforms; `Pose2::inverse` needs a unit
///   rotation.
pub fn closest_points_cuboid_cuboid(
    pos12: Pose2, cuboid1: Cuboid, cuboid2: Cuboid, margin: Fixed,
) -> ClosestPoints {
    let pos21 = pos12.inverse();
    let (sep1, n1) = oneway(cuboid1, cuboid2, pos12);
    if sep1 > margin {
        return ClosestPoints::Disjoint;
    }
    let (sep2, n2) = oneway(cuboid2, cuboid1, pos21);
    if sep2 > margin {
        return ClosestPoints::Disjoint;
    }
    if sep1 <= ZERO && sep2 <= ZERO {
        return ClosestPoints::Intersecting;
    }
    let (p1, p2) = if sep1 >= sep2 {
        let pt2_1 = support_point(cuboid2, pos12, -n1);
        let proj1 = project_local_point_cuboid(cuboid1, pt2_1, true);
        let proj2 = project_local_point_cuboid(cuboid2, pos21.transform_point(proj1.point), true);
        (proj1.point, proj2.point)
    } else {
        let pt1_2 = support_point(cuboid1, pos21, -n2);
        let proj2 = project_local_point_cuboid(cuboid2, pt1_2, true);
        let proj1 = project_local_point_cuboid(cuboid1, pos12.transform_point(proj2.point), true);
        (proj1.point, proj2.point)
    };
    let d = p1 - pos12.transform_point(p2);
    if is_norm2_gt(d.x, d.y, margin) {
        ClosestPoints::Disjoint
    } else {
        ClosestPoints::WithinMargin((p1, p2))
    }
}

/// Distance between two cuboids: the length between the witness pair of
/// [`closest_points_cuboid_cuboid`] with an unbounded margin, zero when they intersect
/// (upstream `distance_cuboid_cuboid`).
/// #### Panics
/// * See [`closest_points_cuboid_cuboid`].
pub fn distance_cuboid_cuboid(pos12: Pose2, cuboid1: Cuboid, cuboid2: Cuboid) -> Fixed {
    match closest_points_cuboid_cuboid(pos12, cuboid1, cuboid2, fixed::MAX) {
        ClosestPoints::WithinMargin((
            p1, p2,
        )) => {
            let (_, len) = normalize_and_length(p1 - pos12.transform_point(p2));
            len
        },
        _ => ZERO,
    }
}

/// Contact between two cuboids when their SAT separation is at most `prediction` (upstream
/// `contact_cuboid_cuboid`, ported literally).
///
/// Along the larger one-way separation, the support point of the other cuboid is projected on
/// the boundary of this one; the normal is the direction between them, or the SAT axis when the
/// point is inside or within `DEFAULT_EPSILON`. As upstream, a penetrating pair whose support
/// corner lands outside the other box (a tie of the support map, broken towards `+` here and by
/// the sign of `-0.0` upstream) reads a positive distance: `query::contact` uses the exact kernel
/// of [`super::support_map`] for cuboid pairs instead, as upstream uses EPA.
/// #### Panics
/// * See [`closest_points_cuboid_cuboid`].
pub fn contact_cuboid_cuboid(
    pos12: Pose2, cuboid1: Cuboid, cuboid2: Cuboid, prediction: Fixed,
) -> Option<Contact> {
    let pos21 = pos12.inverse();
    let (sep1, n1) = oneway(cuboid1, cuboid2, pos12);
    if sep1 > prediction {
        return None;
    }
    let (sep2, n2) = oneway(cuboid2, cuboid1, pos21);
    if sep2 > prediction {
        return None;
    }
    if sep1 >= sep2 {
        let pt2_1 = support_point(cuboid2, pos12, -n1);
        let proj1 = project_local_point_cuboid(cuboid1, pt2_1, false);
        let (normal1, dist) = witness_normal(pt2_1 - proj1.point, n1);
        if dist > prediction {
            return None;
        }
        Some(
            ContactTrait::new(
                proj1.point,
                pos21.transform_point(pt2_1),
                normal1,
                pos12.rotation.inverse_rotate(-normal1),
                dist,
            ),
        )
    } else {
        let pt1_2 = support_point(cuboid1, pos21, -n2);
        let proj2 = project_local_point_cuboid(cuboid2, pt1_2, false);
        let (normal2, dist) = witness_normal(pt1_2 - proj2.point, n2);
        if dist > prediction {
            return None;
        }
        Some(
            ContactTrait::new(
                pos12.transform_point(pt1_2),
                proj2.point,
                pos12.rotation.rotate(-normal2),
                normal2,
                dist,
            ),
        )
    }
}

/// `(normal, signed distance)` of a witness vector `d` against the SAT axis `axis`: the unit `d`
/// and its length, or `(axis, d . axis)` when `d` points against the axis or is shorter than
/// `DEFAULT_EPSILON`.
#[inline(always)]
fn witness_normal(d: Vec2, axis: Vec2) -> (Vec2, Fixed) {
    let separation = d.dot(axis);
    let (dir, len) = normalize_and_length(d);
    match dir {
        Some(n) => if separation < ZERO || len <= DEFAULT_EPSILON {
            (axis, separation)
        } else {
            (n, len)
        },
        None => (axis, separation),
    }
}

/// Closest points of a cuboid and a triangle placed at `pos12` within `margin` (Parry
/// `closest_points_cuboid_triangle`): the exact support-map kernel of
/// [`super::support_map`], each point in its shape's frame.
/// #### Panics
/// * See [`super::support_map::closest_points_support_map_support_map`].
pub fn closest_points_cuboid_triangle(
    pos12: Pose2, cuboid1: Cuboid, triangle2: crate::shape::Triangle, margin: Fixed,
) -> ClosestPoints {
    super::support_map::closest_points_support_map_support_map(
        pos12, crate::shape::Shape::Cuboid(cuboid1), triangle2.into(), margin,
    )
}

/// [`closest_points_cuboid_triangle`] with the shapes in the other order (Parry
/// `closest_points_triangle_cuboid`).
/// #### Panics
/// * See [`closest_points_cuboid_triangle`].
pub fn closest_points_triangle_cuboid(
    pos12: Pose2, triangle1: crate::shape::Triangle, cuboid2: Cuboid, margin: Fixed,
) -> ClosestPoints {
    super::support_map::closest_points_support_map_support_map(
        pos12, triangle1.into(), crate::shape::Shape::Cuboid(cuboid2), margin,
    )
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::query::ClosestPoints;
    use crate::query::support_map::{distance_support_map_support_map, witness};
    use crate::shape::{Cuboid, CuboidTrait, Shape};
    use super::{closest_points_cuboid_cuboid, contact_cuboid_cuboid, distance_cuboid_cuboid};

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn int(x: i32) -> Fixed {
        FixedTrait::from_int(x)
    }

    fn at(x: Fixed, y: Fixed) -> Pose2 {
        Pose2Trait::new(v(x, y), Rot2 { re: ONE, im: ZERO })
    }

    /// 30 degrees (`im = 0.5` exact, `re` rounded) around `(x, y)`.
    fn turned(x: Fixed, y: Fixed) -> Pose2 {
        Pose2Trait::new(v(x, y), Rot2 { re: Fixed { raw: 3719550787 }, im: HALF })
    }

    fn box2() -> Cuboid {
        CuboidTrait::new(v(ONE, HALF))
    }

    /// `(pos12, distance)`, and the analytic kernel of `support_map` agrees.
    #[test]
    fn test_distance_table() {
        let cases: Span<(Pose2, Fixed)> = array![
            (at(int(3), ZERO), ONE), (at(ZERO, TWO_), ONE), (at(int(2), ZERO), ZERO),
            (at(HALF, HALF), ZERO),
        ]
            .span();
        for (pos12, expected) in cases {
            assert_eq!(distance_cuboid_cuboid(*pos12, box2(), box2()), *expected);
            assert_eq!(
                distance_support_map_support_map(
                    *pos12, Shape::Cuboid(box2()), Shape::Cuboid(box2()),
                ),
                *expected,
            );
        }
        // Rotated: both within a few ulps of each other.
        let pos12 = turned(int(3), ONE);
        let literal = distance_cuboid_cuboid(pos12, box2(), box2());
        let generic = distance_support_map_support_map(
            pos12, Shape::Cuboid(box2()), Shape::Cuboid(box2()),
        );
        assert!(literal.abs_diff_eq(generic, Fixed { raw: 8 }));
    }

    const TWO_: Fixed = Fixed { raw: 0x2_0000_0000 };

    #[test]
    fn test_closest_points_and_contact() {
        let far = at(int(3), ZERO);
        assert_eq!(
            closest_points_cuboid_cuboid(far, box2(), box2(), HALF), ClosestPoints::Disjoint,
        );
        assert_eq!(
            closest_points_cuboid_cuboid(far, box2(), box2(), ONE),
            ClosestPoints::WithinMargin((v(ONE, HALF), v(-ONE, HALF))),
        );
        assert_eq!(
            closest_points_cuboid_cuboid(at(ONE, ZERO), box2(), box2(), ONE),
            ClosestPoints::Intersecting,
        );
        assert!(contact_cuboid_cuboid(far, box2(), box2(), HALF).is_none());
        let c = contact_cuboid_cuboid(far, box2(), box2(), ONE).unwrap();
        assert_eq!((c.normal1, c.normal2, c.dist), (v(ONE, ZERO), v(-ONE, ZERO), ONE));
        // Overlapping by 0.25 along y: the support corner of the second box lands outside the
        // first one, so the kernel reads a positive distance and answers nothing, where the
        // generic kernel finds the penetration. Upstream's function has the same blind spot
        // (the dispatchers do not use it for `contact`); pinned here as documentation.
        let deep = at(HALF, int(3) / int(4));
        assert!(contact_cuboid_cuboid(deep, box2(), box2(), ZERO).is_none());
        let w = witness(deep, Shape::Cuboid(box2()), Shape::Cuboid(box2()));
        assert_eq!((w.normal1, w.dist), (v(ZERO, ONE), -HALF / int(2)));
    }

    /// The SAT witness is the exact closest pair: same distance as the generic kernel.
    #[test]
    #[fuzzer(runs: 64, seed: 20260926)]
    fn fuzz_sat_witness_is_exact(x: i8, y: i8, turn: bool) {
        let (tx, ty) = (int(x.into()) / int(16), int(y.into()) / int(16));
        let pos12 = if turn {
            turned(tx, ty)
        } else {
            at(tx, ty)
        };
        let literal = distance_cuboid_cuboid(pos12, box2(), box2());
        let generic = distance_support_map_support_map(
            pos12, Shape::Cuboid(box2()), Shape::Cuboid(box2()),
        );
        assert!(literal.abs_diff_eq(generic, Fixed { raw: 8 }));
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_distance_cuboid_cuboid() {
        let _ = distance_cuboid_cuboid(opaque(at(int(3), HALF)), opaque(box2()), opaque(box2()));
    }

    #[test]
    fn gas_distance_cuboid_cuboid_turned() {
        let _ = distance_cuboid_cuboid(opaque(turned(int(3), ONE)), opaque(box2()), opaque(box2()));
    }

    #[test]
    fn gas_distance_support_map_cuboid_cuboid_turned() {
        let _ = distance_support_map_support_map(
            opaque(turned(int(3), ONE)),
            opaque(Shape::Cuboid(box2())),
            opaque(Shape::Cuboid(box2())),
        );
    }

    #[test]
    fn gas_closest_points_cuboid_cuboid() {
        let _ = closest_points_cuboid_cuboid(
            opaque(at(int(3), HALF)), opaque(box2()), opaque(box2()), opaque(ONE),
        );
    }

    #[test]
    fn gas_contact_cuboid_cuboid_overlapping() {
        let _ = contact_cuboid_cuboid(
            opaque(at(ONE, HALF)), opaque(box2()), opaque(box2()), opaque(ZERO),
        );
    }
}
