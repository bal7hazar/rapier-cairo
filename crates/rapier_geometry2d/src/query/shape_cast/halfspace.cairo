//! Half-space shape casts (Parry `shape_cast_halfspace_support_map.rs`): the deepest point of the
//! other shape along `-normal` (inflated by `target_distance`) is cast as a solid ray on the
//! half-space, with the exact kernel of `crate::ray::halfspace`.

use fixed::ZERO;
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::point::wide2::dot_wide;
use crate::ray::halfspace::cast_local_ray_halfspace;
use crate::ray::{Ray, RayTrait};
use crate::shape::{HalfSpace, Shape};
use super::super::support_map::local_support_point_toward;
use super::{ShapeCastHit, ShapeCastHitTrait, ShapeCastOptions, ShapeCastStatus};

/// The first impact of the support-map shape `other`, placed at `pos12` and moving at `vel12`,
/// with `halfspace` (upstream `cast_shapes_halfspace_support_map`).
///
/// With `stop_at_penetration = false`, a motion leaving the plane (`vel12 . n > 0`, exact) never
/// hits. The support point starting on or below the plane answers `t = 0`; the status is
/// `PenetratingOrWithinTargetDist` when it starts strictly below it. `witness1` is the ray's hit
/// projected on the plane, `witness2` the support point of `other` (its real surface).
/// #### Panics
/// * `'Query: not a support map'` when `other` is a half-space; the transforms' overflow panics.
pub fn cast_shapes_halfspace_support_map(
    pos12: Pose2, vel12: Vec2, halfspace: HalfSpace, other: Shape, options: ShapeCastOptions,
) -> Option<ShapeCastHit> {
    let n = halfspace.normal;
    if !options.stop_at_penetration && dot_wide(vel12.x, vel12.y, n.x, n.y) > 0 {
        return None;
    }
    let dir = -n;
    let local = local_support_point_toward(other, pos12.rotation.inverse_rotate(dir));
    let mut support_point = pos12.transform_point(local);
    if options.target_distance > ZERO {
        support_point = support_point + dir.mul_scalar(options.target_distance);
    }
    let ray = Ray { origin: support_point, dir: vel12 };
    let time_of_impact = cast_local_ray_halfspace(
        halfspace, ray, options.max_time_of_impact, true,
    )?;
    let witness2 = support_point + n.mul_scalar(options.target_distance);
    let hit = ray.point_at(time_of_impact);
    let witness1 = hit - n.mul_scalar(hit.dot(n));
    let status = if dot_wide(support_point.x, support_point.y, n.x, n.y) < 0 {
        ShapeCastStatus::PenetratingOrWithinTargetDist
    } else {
        ShapeCastStatus::Converged
    };
    Some(
        ShapeCastHit {
            time_of_impact,
            normal1: n,
            normal2: pos12.rotation.inverse_rotate(dir),
            witness1,
            witness2: pos12.inverse_transform_point(witness2),
            status,
        },
    )
}

/// [`cast_shapes_halfspace_support_map`] with the shapes in the other order (upstream
/// `cast_shapes_support_map_halfspace`): the pose is inverted, the velocity moved into the
/// half-space's frame and negated, and the hit swapped back.
/// #### Panics
/// * See [`cast_shapes_halfspace_support_map`] and `Pose2::inverse`.
pub fn cast_shapes_support_map_halfspace(
    pos12: Pose2, vel12: Vec2, other: Shape, halfspace: HalfSpace, options: ShapeCastOptions,
) -> Option<ShapeCastHit> {
    let hit = cast_shapes_halfspace_support_map(
        pos12.inverse(), -pos12.rotation.inverse_rotate(vel12), halfspace, other, options,
    )?;
    Some(hit.swapped())
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, MAX, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::Pose2Trait;
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::shape::{CuboidTrait, HalfSpaceTrait, Shape};
    use super::super::{ShapeCastOptions, ShapeCastStatus};
    use super::{cast_shapes_halfspace_support_map, cast_shapes_support_map_halfspace};

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn int(x: i32) -> Fixed {
        FixedTrait::from_int(x)
    }

    fn options(target: Fixed, stop: bool) -> ShapeCastOptions {
        ShapeCastOptions {
            max_time_of_impact: MAX,
            target_distance: target,
            stop_at_penetration: stop,
            compute_impact_geometry_on_penetration: true,
        }
    }

    /// A unit box (half extents 0.5) at height `y` falling at `vy` on the plane `y = 0`:
    /// `(y, vy, target, stop) -> (hit, toi, status)`.
    #[test]
    fn test_halfspace_cuboid_table() {
        let cases = array![
            (int(3), -ONE, ZERO, true, true, TWO + HALF, ShapeCastStatus::Converged),
            (int(3), ONE, ZERO, true, false, ZERO, ShapeCastStatus::Converged),
            (int(3), -ONE, HALF, true, true, TWO, ShapeCastStatus::Converged),
            (HALF, -ONE, ZERO, true, true, ZERO, ShapeCastStatus::Converged),
            (ZERO, ONE, ZERO, true, true, ZERO, ShapeCastStatus::PenetratingOrWithinTargetDist),
            (ZERO, ONE, ZERO, false, false, ZERO, ShapeCastStatus::Converged),
            (ZERO, ZERO, ZERO, true, true, ZERO, ShapeCastStatus::PenetratingOrWithinTargetDist),
        ];
        let hs = HalfSpaceTrait::new(v(ZERO, ONE));
        let cuboid = Shape::Cuboid(CuboidTrait::new(v(HALF, HALF)));
        for (y, vy, target, stop, hit, toi, status) in cases {
            let pos12 = Pose2Trait::new(v(ONE, y), Rot2 { re: ONE, im: ZERO });
            let answer = cast_shapes_halfspace_support_map(
                pos12, v(ZERO, vy), hs, cuboid, options(target, stop),
            );
            assert_eq!(answer.is_some(), hit, "{:?} {:?}", y, vy);
            if let Some(h) = answer {
                assert_eq!((h.time_of_impact, h.status), (toi, status));
                assert_eq!((h.normal1, h.normal2), (v(ZERO, ONE), v(ZERO, -ONE)));
                assert_eq!(h.witness2, v(HALF, -HALF));
                assert_eq!(h.witness1.y, ZERO);
            }
            // The swapped order answers the swapped hit.
            let swapped = cast_shapes_support_map_halfspace(
                pos12.inverse(), v(ZERO, -vy), cuboid, hs, options(target, stop),
            );
            assert_eq!(swapped.is_some(), hit);
            if let (Some(a), Some(b)) = (answer, swapped) {
                assert_eq!(b.time_of_impact, a.time_of_impact);
                assert_eq!((b.witness1, b.normal1), (a.witness2, a.normal2));
            }
        }
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_cast_shapes_halfspace_support_map_cuboid() {
        let _ = cast_shapes_halfspace_support_map(
            opaque(Pose2Trait::new(v(ONE, int(3)), Rot2 { re: ZERO, im: ONE })),
            opaque(v(ZERO, -ONE)),
            opaque(HalfSpaceTrait::new(v(ZERO, ONE))),
            opaque(Shape::Cuboid(CuboidTrait::new(v(HALF, HALF)))),
            opaque(Default::default()),
        );
    }

    #[test]
    fn gas_cast_shapes_support_map_halfspace_cuboid() {
        let _ = cast_shapes_support_map_halfspace(
            opaque(Pose2Trait::new(v(ONE, -int(3)), Rot2 { re: ZERO, im: ONE })),
            opaque(v(ZERO, ONE)),
            opaque(Shape::Cuboid(CuboidTrait::new(v(HALF, HALF)))),
            opaque(HalfSpaceTrait::new(v(ZERO, ONE))),
            opaque(Default::default()),
        );
    }
}
