//! Axis-aligned bounding boxes (Parry `Aabb`) for 2D broad-phase and shape bounds.
//!
//! Intersections and containment use closed intervals, so touching edges and corners overlap.
//! `transform_by` uses Parry's absolute-rotation formula; the four-corner implementation is kept
//! in the test-only alternatives for equivalence checks and gas ranking.

pub mod bounding_volume;
use bounding_volume::{BoundingSphere, local_point_cloud_aabb};
use fixed::wide::{dot2, norm2};
use fixed::{Fixed, FixedTrait, HALF, MAX, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::consts::DEFAULT_EPSILON;
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::feature_id::{FeatureId, FeatureIdTrait};
use crate::point::PointProjection;
use crate::ray::cuboid::{cast_local_ray_and_get_normal_cuboid, cast_local_ray_cuboid};
use crate::ray::{Ray, RayIntersection};
use crate::shape::Cuboid;

/// An axis-aligned bounding box, represented by minimum and maximum corners.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct Aabb {
    /// Minimum x/y corner.
    pub mins: Vec2,
    /// Maximum x/y corner.
    pub maxs: Vec2,
}

/// `|R| * h`: the half extents of the box of half extents `h` rotated by `pose` (upstream
/// `PoseOps::absolute_transform_vector`). One rescale per component, so an exact quarter turn
/// gives an exact box. Shared by [`AabbTrait::transform_by`] and the shape AABBs.
/// #### Panics
/// * `'i64_neg Underflow'` for a rotation component equal to `fixed::MIN` (not unit).
/// * `'Fixed: overflow'` if a component of the result leaves the scalar range.
#[inline(always)]
pub fn absolute_transform_vector(pose: Pose2, h: Vec2) -> Vec2 {
    let c = pose.rotation.re.abs();
    let s = pose.rotation.im.abs();
    Vec2 { x: dot2(c, h.x, s, h.y), y: dot2(s, h.x, c, h.y) }
}

#[generate_trait]
pub impl AabbImpl of AabbTrait {
    /// Creates a new AABB from its minimum and maximum corners.
    ///
    /// The bounds are stored as-is; callers are responsible for `mins <= maxs` on each axis.
    #[inline(always)]
    fn new(mins: Vec2, maxs: Vec2) -> Aabb {
        Aabb { mins, maxs }
    }

    /// Creates an AABB from its center and half-extents.
    ///
    /// Negative half-extents invert that axis, matching direct vector arithmetic.
    #[inline(always)]
    fn from_half_extents(center: Vec2, half_extents: Vec2) -> Aabb {
        Aabb { mins: center - half_extents, maxs: center + half_extents }
    }

    /// Returns the midpoint between `mins` and `maxs`, floored component-wise.
    fn center(self: Aabb) -> Vec2 {
        (self.mins + self.maxs).mul_scalar(HALF)
    }

    /// Returns half the full extents, floored component-wise.
    fn half_extents(self: Aabb) -> Vec2 {
        self.extents().mul_scalar(HALF)
    }

    /// Returns the full extents `maxs - mins`.
    fn extents(self: Aabb) -> Vec2 {
        self.maxs - self.mins
    }

    /// Returns `true` when `self` and `other` overlap on both axes, including edge touches.
    fn intersects(self: Aabb, other: Aabb) -> bool {
        self.mins.x.raw <= other.maxs.x.raw
            && other.mins.x.raw <= self.maxs.x.raw
            && self.mins.y.raw <= other.maxs.y.raw
            && other.mins.y.raw <= self.maxs.y.raw
    }

    /// Returns `true` when `self` fully contains `other`, including equal bounds.
    fn contains(self: Aabb, other: Aabb) -> bool {
        self.mins.x.raw <= other.mins.x.raw
            && other.maxs.x.raw <= self.maxs.x.raw
            && self.mins.y.raw <= other.mins.y.raw
            && other.maxs.y.raw <= self.maxs.y.raw
    }

    /// Returns `true` when `p` lies inside this AABB or on its boundary.
    fn contains_local_point(self: Aabb, p: Vec2) -> bool {
        self.mins.x.raw <= p.x.raw
            && p.x.raw <= self.maxs.x.raw
            && self.mins.y.raw <= p.y.raw
            && p.y.raw <= self.maxs.y.raw
    }

    /// Returns the smallest AABB containing both inputs.
    fn merged(self: Aabb, other: Aabb) -> Aabb {
        Aabb { mins: self.mins.min(other.mins), maxs: self.maxs.max(other.maxs) }
    }

    /// Expands every side by `margin`.
    ///
    /// A negative margin tightens the box and may invert empty axes, matching upstream arithmetic.
    #[inline(always)]
    fn loosened(self: Aabb, margin: Fixed) -> Aabb {
        let v = Vec2 { x: margin, y: margin };
        Aabb { mins: self.mins - v, maxs: self.maxs + v }
    }

    /// Shrinks every side by `margin`.
    ///
    /// A negative margin loosens the box and may invert empty axes, matching upstream arithmetic.
    fn tightened(self: Aabb, margin: Fixed) -> Aabb {
        let v = Vec2 { x: margin, y: margin };
        Aabb { mins: self.mins + v, maxs: self.maxs - v }
    }

    /// Bounds this AABB after applying `pose`.
    ///
    /// Uses Parry's `abs(rotation) * half_extents` formulation instead of transforming four
    /// corners. Each product pair is accumulated wide and floored once per output component.
    fn transform_by(self: Aabb, pose: Pose2) -> Aabb {
        let center = pose.transform_point(self.center());
        let half = self.half_extents();
        let half = absolute_transform_vector(pose, half);
        Aabb { mins: center - half, maxs: center + half }
    }

    /// Returns the 2D area of this AABB (`extent.x * extent.y`).
    fn volume(self: Aabb) -> Fixed {
        let e = self.extents();
        e.x * e.y
    }

    /// Scales `mins` and `maxs` component-wise, reordering axes for negative scale components.
    fn scaled(self: Aabb, scale: Vec2) -> Aabb {
        let a = self.mins * scale;
        let b = self.maxs * scale;
        Aabb { mins: a.min(b), maxs: a.max(b) }
    }

    /// Vertex indices (into [`AabbTrait::vertices`]) of the four faces: `+x`, `-x`, `+y`, `-y`.
    const FACES_VERTEX_IDS: [(u32, u32); 4] = [(1, 2), (3, 0), (2, 3), (0, 1)];

    /// The empty box `[MAX, -MAX]`: merging or `take_point` into it gives the other operand.
    #[inline(always)]
    fn new_invalid() -> Aabb {
        let max = Vec2 { x: MAX, y: MAX };
        Aabb { mins: max, maxs: -max }
    }

    /// Exact bounds of `pts` (upstream `from_points`; `Span` also stands for `from_points_ref`).
    /// #### Panics
    /// * `'Bounding: empty point cloud'` for no point.
    fn from_points(pts: Span<Vec2>) -> Aabb {
        local_point_cloud_aabb(pts)
    }

    /// `extent.x + extent.y`: the 2D surface-area-heuristic cost (upstream `half_perimeter`).
    #[inline(always)]
    fn half_perimeter(self: Aabb) -> Fixed {
        let e = self.extents();
        e.x + e.y
    }

    /// [`AabbTrait::half_perimeter`] in 2D.
    #[inline(always)]
    fn half_area_or_perimeter(self: Aabb) -> Fixed {
        Self::half_perimeter(self)
    }

    /// Grows the box to contain `pt` (exact).
    #[inline(always)]
    fn take_point(ref self: Aabb, pt: Vec2) {
        self.mins = self.mins.min(pt);
        self.maxs = self.maxs.max(pt);
    }

    /// The box moved by `translation` (exact).
    #[inline(always)]
    fn translated(self: Aabb, translation: Vec2) -> Aabb {
        Aabb { mins: self.mins + translation, maxs: self.maxs + translation }
    }

    /// The box grown by `half_extents` on each side of its axis (exact; negative components
    /// shrink it).
    #[inline(always)]
    fn add_half_extents(self: Aabb, half_extents: Vec2) -> Aabb {
        Aabb { mins: self.mins - half_extents, maxs: self.maxs + half_extents }
    }

    /// The sphere through the corners: centre [`AabbTrait::center`], radius `|maxs - mins| / 2`
    /// (the floored diagonal, halved and floored).
    fn bounding_sphere(self: Aabb) -> BoundingSphere {
        let e = self.extents();
        BoundingSphere { center: self.center(), radius: norm2(e.x, e.y) * HALF }
    }

    /// The overlap of both boxes, `None` when they are disjoint on an axis (touching boxes give
    /// a flat box).
    fn intersection(self: Aabb, other: Aabb) -> Option<Aabb> {
        let result = Aabb { mins: self.mins.max(other.mins), maxs: self.maxs.min(other.maxs) };
        if result.mins.x > result.maxs.x || result.mins.y > result.maxs.y {
            None
        } else {
            Some(result)
        }
    }

    /// The corners, counter-clockwise from `mins`: `(mins.x, mins.y)`, `(maxs.x, mins.y)`,
    /// `maxs`, `(mins.x, maxs.y)`.
    #[inline(always)]
    fn vertices(self: Aabb) -> [Vec2; 4] {
        [
            self.mins, Vec2 { x: self.maxs.x, y: self.mins.y }, self.maxs,
            Vec2 { x: self.mins.x, y: self.maxs.y },
        ]
    }

    /// The four quadrants around [`AabbTrait::center`], counter-clockwise from the one at
    /// `mins`.
    fn split_at_center(self: Aabb) -> [Aabb; 4] {
        let c = self.center();
        [
            Aabb { mins: self.mins, maxs: c },
            Aabb { mins: Vec2 { x: c.x, y: self.mins.y }, maxs: Vec2 { x: self.maxs.x, y: c.y } },
            Aabb { mins: c, maxs: self.maxs },
            Aabb { mins: Vec2 { x: self.mins.x, y: c.y }, maxs: Vec2 { x: c.x, y: self.maxs.y } },
        ]
    }

    /// `self` minus `rhs` as at most four disjoint boxes (upstream `difference`).
    fn difference(self: Aabb, rhs: Aabb) -> Array<Aabb> {
        let (pieces, _) = Self::difference_with_cut_sequence(self, rhs);
        pieces
    }

    /// `self` minus `rhs` as at most four boxes, and the cuts that produced them: `(axis + 1,
    /// rhs.mins[axis])` for a cut below, `(-(axis + 1), -rhs.maxs[axis])` above. `self` alone
    /// (no cut) when the interiors do not overlap.
    fn difference_with_cut_sequence(self: Aabb, rhs: Aabb) -> (Array<Aabb>, Array<(i8, Fixed)>) {
        let mut pieces = array![];
        let mut cuts = array![];
        if self.mins.x >= rhs.maxs.x
            || self.maxs.x <= rhs.mins.x
            || self.mins.y >= rhs.maxs.y
            || self.maxs.y <= rhs.mins.y {
            pieces.append(self);
            return (pieces, cuts);
        }
        let mut rest = self;
        if rhs.mins.x > rest.mins.x {
            pieces.append(Aabb { mins: rest.mins, maxs: Vec2 { x: rhs.mins.x, y: rest.maxs.y } });
            rest.mins.x = rhs.mins.x;
            cuts.append((1, rhs.mins.x));
        }
        if rhs.maxs.x < rest.maxs.x {
            pieces.append(Aabb { mins: Vec2 { x: rhs.maxs.x, y: rest.mins.y }, maxs: rest.maxs });
            rest.maxs.x = rhs.maxs.x;
            cuts.append((-1, -rhs.maxs.x));
        }
        if rhs.mins.y > rest.mins.y {
            pieces.append(Aabb { mins: rest.mins, maxs: Vec2 { x: rest.maxs.x, y: rhs.mins.y } });
            rest.mins.y = rhs.mins.y;
            cuts.append((2, rhs.mins.y));
        }
        if rhs.maxs.y < rest.maxs.y {
            pieces.append(Aabb { mins: Vec2 { x: rest.mins.x, y: rhs.maxs.y }, maxs: rest.maxs });
            cuts.append((-2, -rhs.maxs.y));
        }
        (pieces, cuts)
    }
}

/// Exact decomposition of `pt` against `aabb` (upstream `Aabb::do_project_local_point`):
/// `(inside, projected point, shift)` with `projected = pt + shift`.
///
/// Outside, the point is clamped. Inside and hollow, it is pushed to the nearest face; the side
/// of each axis is chosen by the sign of `pt - center` (ties to the positive side, as
/// upstream's `copy_sign_to(1)`), decided exactly as `pt - mins >= maxs - pt`, so the centre is
/// never rounded.
fn do_project_local_point_aabb(aabb: Aabb, pt: Vec2, solid: bool) -> (bool, Vec2, Vec2) {
    let shift = Vec2 {
        x: (aabb.mins.x - pt.x).max(ZERO) - (pt.x - aabb.maxs.x).max(ZERO),
        y: (aabb.mins.y - pt.y).max(ZERO) - (pt.y - aabb.maxs.y).max(ZERO),
    };
    if shift.x != ZERO || shift.y != ZERO {
        return (false, pt + shift, shift);
    }
    if solid {
        return (true, pt, shift);
    }
    let (diff_x, shift_x) = nearest_face(aabb.mins.x, aabb.maxs.x, pt.x);
    let (diff_y, shift_y) = nearest_face(aabb.mins.y, aabb.maxs.y, pt.y);
    let shift = if diff_x <= diff_y {
        Vec2 { x: shift_x, y: ZERO }
    } else {
        Vec2 { x: ZERO, y: shift_y }
    };
    (true, pt + shift, shift)
}

/// `(distance to the face of the side of p, signed shift to it)` for `min <= p <= max`.
#[inline(always)]
fn nearest_face(min: Fixed, max: Fixed, p: Fixed) -> (Fixed, Fixed) {
    let to_max = max - p;
    let to_min = p - min;
    if to_min >= to_max {
        (to_max, to_max)
    } else {
        (to_min, -to_min)
    }
}

/// `2 p < min + max`, i.e. `p` below the centre of `[min, max]`, exactly.
#[inline(always)]
fn below_center(min: Fixed, max: Fixed, p: Fixed) -> bool {
    let twice: i128 = p.raw.into() * 2;
    twice < min.raw.into() + max.raw.into()
}

/// Projects `pt` on `aabb`: onto the filled box if `solid`, onto its boundary otherwise.
///
/// Mirrors `PointQuery::project_local_point` for `Aabb`. Exact (only subtractions and
/// comparisons).
/// #### Panics
/// * `'i64_sub Overflow'` / `'i64_add Overflow'` when a coordinate difference leaves the scalar
///   range.
pub fn project_local_point_aabb(aabb: Aabb, pt: Vec2, solid: bool) -> PointProjection {
    let (inside, point, _) = do_project_local_point_aabb(aabb, pt, solid);
    PointProjection { is_inside: inside, point }
}

/// Projects `pt` on the boundary of `aabb` and returns the feature: `Face(0)` / `Face(1)` for
/// the `+x` / `+y` faces, `Face(2)` / `Face(3)` for `-x` / `-y`, `Vertex(bits)` for a corner
/// (bit `i` set when the corner is on the negative side of axis `i`), `Unknown` for a
/// degenerate box.
///
/// Mirrors `PointQuery::project_local_point_and_get_feature` for `Aabb`, including its
/// `DEFAULT_EPSILON` face snapping for a point on the boundary.
/// #### Panics
/// * See [`project_local_point_aabb`].
pub fn project_local_point_and_get_feature_aabb(
    aabb: Aabb, pt: Vec2,
) -> (PointProjection, FeatureId) {
    let (inside, point, shift) = do_project_local_point_aabb(aabb, pt, false);
    let proj = PointProjection { is_inside: inside, point };
    let zero_x = shift.x == ZERO;
    let zero_y = shift.y == ZERO;
    let feature = if zero_x && zero_y {
        if point.x > aabb.maxs.x - DEFAULT_EPSILON {
            FeatureIdTrait::face(0)
        } else if point.x <= aabb.mins.x + DEFAULT_EPSILON {
            FeatureIdTrait::face(2)
        } else if point.y > aabb.maxs.y - DEFAULT_EPSILON {
            FeatureIdTrait::face(1)
        } else if point.y <= aabb.mins.y + DEFAULT_EPSILON {
            FeatureIdTrait::face(3)
        } else {
            Default::default()
        }
    } else if zero_x {
        if below_center(aabb.mins.y, aabb.maxs.y, point.y) {
            FeatureIdTrait::face(3)
        } else {
            FeatureIdTrait::face(1)
        }
    } else if zero_y {
        if below_center(aabb.mins.x, aabb.maxs.x, point.x) {
            FeatureIdTrait::face(2)
        } else {
            FeatureIdTrait::face(0)
        }
    } else {
        let bit_x = if below_center(aabb.mins.x, aabb.maxs.x, point.x) {
            1
        } else {
            0
        };
        let bit_y = if below_center(aabb.mins.y, aabb.maxs.y, point.y) {
            2
        } else {
            0
        };
        FeatureIdTrait::vertex(bit_x + bit_y)
    };
    (proj, feature)
}

/// Signed distance from `pt` to `aabb`: negative inside when `solid = false`, zero inside when
/// `solid = true`.
///
/// Mirrors `PointQuery::distance_to_local_point` for `Aabb` (`|max(mins - pt, pt - maxs, 0)|`
/// outside). The length is the floored wide norm.
/// #### Panics
/// * See [`project_local_point_aabb`].
pub fn distance_to_local_point_aabb(aabb: Aabb, pt: Vec2, solid: bool) -> Fixed {
    let sx = (aabb.mins.x - pt.x).max(pt.x - aabb.maxs.x).max(ZERO);
    let sy = (aabb.mins.y - pt.y).max(pt.y - aabb.maxs.y).max(ZERO);
    if solid || sx != ZERO || sy != ZERO {
        norm2(sx, sy)
    } else {
        let proj = project_local_point_aabb(aabb, pt, false);
        let d = pt - proj.point;
        -norm2(d.x, d.y)
    }
}

/// Whether `pt` is inside `aabb`, boundary included (`PointQuery::contains_local_point`).
#[inline(always)]
pub fn contains_local_point_aabb(aabb: Aabb, pt: Vec2) -> bool {
    pt.x >= aabb.mins.x && pt.x <= aabb.maxs.x && pt.y >= aabb.mins.y && pt.y <= aabb.maxs.y
}

/// `aabb` as a cuboid and the ray moved to its centre: upstream's cuboid casts are its AABB
/// casts on `[-half_extents, half_extents]`.
#[inline(always)]
fn centered(aabb: Aabb, ray: Ray) -> (Cuboid, Ray) {
    let center = aabb.center();
    (
        Cuboid { half_extents: aabb.half_extents() },
        Ray { origin: ray.origin - center, dir: ray.dir },
    )
}

/// Time of impact of `ray` on `aabb` (`RayCast::cast_local_ray` for `Aabb`, the slab loop).
/// #### Panics
/// * See `crate::ray::cuboid`.
/// #### Deviations
/// * Computed as the cuboid cast of `crate::ray::cuboid` on the box re-centred at the origin:
///   exact when `mins` and `maxs` have raws of equal parity on each axis (centre and half extents
///   representable), within one ulp of the box otherwise.
pub fn cast_local_ray_aabb(
    aabb: Aabb, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<Fixed> {
    let (cuboid, ray) = centered(aabb, ray);
    cast_local_ray_cuboid(cuboid, ray, max_time_of_impact, solid)
}

/// Time of impact, normal and feature of `ray` on `aabb` (`RayCast::cast_local_ray_and_get_normal`
/// for `Aabb`, through `clip_aabb_line`). Feature ids are upstream's for a cuboid.
/// #### Panics
/// * See `crate::ray::cuboid`.
/// #### Deviations
/// * See [`cast_local_ray_aabb`].
pub fn cast_local_ray_and_get_normal_aabb(
    aabb: Aabb, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<RayIntersection> {
    let (cuboid, ray) = centered(aabb, ray);
    cast_local_ray_and_get_normal_cuboid(cuboid, ray, max_time_of_impact, solid)
}

#[cfg(test)]
pub mod alternatives {
    use fixed::wide::norm2;
    use glam::{Vec2, Vec2Trait};
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use super::bounding_volume::BoundingSphere;
    use super::{Aabb, AabbTrait};

    /// `bounding_sphere` as the length of the floored half extents (one fewer rescale, but the
    /// halving happens before the square root).
    pub fn bounding_sphere_half_extents(aabb: Aabb) -> BoundingSphere {
        let h = aabb.half_extents();
        BoundingSphere { center: aabb.center(), radius: norm2(h.x, h.y) }
    }

    /// Transforms all four corners and merges them. Same semantics as `transform_by`, but costlier.
    pub fn transform_by_corners(aabb: Aabb, pose: Pose2) -> Aabb {
        let p0 = pose.transform_point(aabb.mins);
        let p1 = pose.transform_point(Vec2 { x: aabb.mins.x, y: aabb.maxs.y });
        let p2 = pose.transform_point(Vec2 { x: aabb.maxs.x, y: aabb.mins.y });
        let p3 = pose.transform_point(aabb.maxs);
        Aabb { mins: p0.min(p1).min(p2).min(p3), maxs: p0.max(p1).max(p2).max(p3) }
    }
}

#[cfg(test)]
mod helper_tests;

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::{Vec2, Vec2Trait};
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::{IDENTITY, Rot2};
    use rapier_testing::opaque;
    use super::{Aabb, AabbTrait, absolute_transform_vector, alternatives};

    const QUARTER: Fixed = Fixed { raw: 1073741824 };
    const NEG_ONE: Fixed = Fixed { raw: -4294967296 };
    const THREE: Fixed = Fixed { raw: 12884901888 };
    const FOUR: Fixed = Fixed { raw: 17179869184 };
    const R: Rot2 = Rot2 { re: Fixed { raw: 3037000500 }, im: Fixed { raw: 3037000500 } };
    const P: Pose2 = Pose2 {
        translation: Vec2 { x: Fixed { raw: 8589934592 }, y: NEG_ONE }, rotation: R,
    };
    const A: Aabb = Aabb {
        mins: Vec2 { x: NEG_ONE, y: Fixed { raw: -8589934592 } }, maxs: Vec2 { x: THREE, y: TWO },
    };
    const B: Aabb = Aabb {
        mins: Vec2 { x: ONE, y: TWO }, maxs: Vec2 { x: FOUR, y: Fixed { raw: 21474836480 } },
    };

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn close(a: Vec2, b: Vec2, ulps: i64) {
        assert!(a.x.abs_diff_eq(b.x, Fixed { raw: ulps }));
        assert!(a.y.abs_diff_eq(b.y, Fixed { raw: ulps }));
    }

    #[test]
    fn test_constructors_center_and_extents() {
        let a = AabbTrait::new(v(NEG_ONE, -TWO), v(THREE, TWO));
        assert_eq!(a.center(), v(ONE, ZERO));
        assert_eq!(a.extents(), v(FOUR, FOUR));
        assert_eq!(a.half_extents(), v(TWO, TWO));
        assert_eq!(AabbTrait::from_half_extents(v(ONE, ZERO), v(TWO, TWO)), a);

        let empty = AabbTrait::new(v(ONE, TWO), v(ONE, TWO));
        assert_eq!(empty.center(), v(ONE, TWO));
        assert_eq!(empty.volume(), ZERO);
    }

    #[test]
    fn test_intersections_are_closed() {
        for (lhs, rhs, hit) in array![
            (A, B, true),
            (A, AabbTrait::new(v(FOUR + FixedTrait::from_raw(1), ZERO), v(FOUR, ONE)), false),
            (A, AabbTrait::new(v(THREE, TWO), v(FOUR, THREE)), true),
            (
                AabbTrait::new(v(FOUR, FOUR), v(FixedTrait::from_int(5), FixedTrait::from_int(5))),
                A,
                false,
            ),
        ]
            .span() {
            assert_eq!((*lhs).intersects(*rhs), *hit);
            assert_eq!((*rhs).intersects(*lhs), *hit);
        }
    }

    #[test]
    fn test_containment_point_merge_margin_and_scale() {
        assert!(A.contains_local_point(v(NEG_ONE, -TWO)));
        assert!(A.contains_local_point(v(THREE, TWO)));
        assert!(!A.contains_local_point(v(THREE + FixedTrait::from_raw(1), ZERO)));
        assert!(A.contains(AabbTrait::new(v(ZERO, NEG_ONE), v(ONE, ONE))));
        assert!(!A.contains(B));
        assert_eq!(A.merged(B), AabbTrait::new(v(NEG_ONE, -TWO), v(FOUR, FixedTrait::from_int(5))));
        assert_eq!(
            A.loosened(HALF),
            AabbTrait::new(v(NEG_ONE - HALF, -TWO - HALF), v(THREE + HALF, TWO + HALF)),
        );
        assert_eq!(
            A.tightened(QUARTER),
            AabbTrait::new(v(NEG_ONE + QUARTER, -TWO + QUARTER), v(THREE - QUARTER, TWO - QUARTER)),
        );
        assert_eq!(
            A.scaled(v(-ONE, TWO)),
            AabbTrait::new(v(-THREE, Fixed { raw: -17179869184 }), v(ONE, FOUR)),
        );
    }

    #[test]
    fn test_transform_by_abs_rotation_matches_corners() {
        for a in array![A, B, AabbTrait::new(v(ZERO, ZERO), v(ZERO, ZERO))].span() {
            let fast = (*a).transform_by(P);
            let corners = alternatives::transform_by_corners(*a, P);
            close(fast.mins, corners.mins, 8);
            close(fast.maxs, corners.maxs, 8);
            assert_eq!(
                (*a).transform_by(Pose2Trait::new(v(ONE, TWO), IDENTITY)),
                AabbTrait::new((*a).mins + v(ONE, TWO), (*a).maxs + v(ONE, TWO)),
            );
        }
    }

    #[test]
    #[fuzzer(runs: 128, seed: 20260920)]
    fn fuzz_transform_candidates(x0: i16, y0: i16, x1: i16, y1: i16) {
        let lo = v(Fixed { raw: x0.into() }, Fixed { raw: y0.into() });
        let hi = v(Fixed { raw: x1.into() }, Fixed { raw: y1.into() });
        let a = AabbTrait::new(lo.min(hi), lo.max(hi));
        let fast = a.transform_by(P);
        let corners = alternatives::transform_by_corners(a, P);
        close(fast.mins, corners.mins, 8);
        close(fast.maxs, corners.maxs, 8);
    }

    #[test]
    fn test_absolute_transform_vector_quarter_turns_are_exact() {
        let h = v(TWO, ONE);
        let turn = |
            re: Fixed, im: Fixed,
        | Pose2 { translation: v(ZERO, ZERO), rotation: Rot2 { re, im } };
        assert_eq!(absolute_transform_vector(turn(ONE, ZERO), h), h);
        assert_eq!(absolute_transform_vector(turn(ZERO, ONE), h), v(ONE, TWO));
        assert_eq!(absolute_transform_vector(turn(NEG_ONE, ZERO), h), h);
        assert_eq!(absolute_transform_vector(turn(ZERO, NEG_ONE), h), v(ONE, TWO));
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_absolute_transform_vector() {
        let _ = absolute_transform_vector(opaque(P), opaque(v(ONE, HALF)));
    }
    #[test]
    fn gas_new() {
        assert_eq!(AabbTrait::new(opaque(A.mins), opaque(A.maxs)), A);
    }
    #[test]
    fn gas_from_half_extents() {
        assert_eq!(
            AabbTrait::from_half_extents(opaque(v(ONE, ZERO)), opaque(v(TWO, TWO))),
            AabbTrait::new(v(NEG_ONE, -TWO), v(THREE, TWO)),
        );
    }
    #[test]
    fn gas_center() {
        assert_eq!(opaque(A).center(), v(ONE, ZERO));
    }
    #[test]
    fn gas_half_extents() {
        assert_eq!(opaque(A).half_extents(), v(TWO, TWO));
    }
    #[test]
    fn gas_extents() {
        assert_eq!(opaque(A).extents(), v(FOUR, FOUR));
    }
    #[test]
    fn gas_intersects() {
        assert!(opaque(A).intersects(opaque(B)));
    }
    #[test]
    fn gas_contains() {
        assert!(opaque(A).contains(opaque(A)));
    }
    #[test]
    fn gas_contains_local_point() {
        assert!(opaque(A).contains_local_point(opaque(v(ONE, ZERO))));
    }
    #[test]
    fn gas_merged() {
        assert_eq!(opaque(A).merged(opaque(B)).maxs.y, FixedTrait::from_int(5));
    }
    #[test]
    fn gas_loosened() {
        assert_eq!(opaque(A).loosened(opaque(HALF)).mins.x, NEG_ONE - HALF);
    }
    #[test]
    fn gas_tightened() {
        assert_eq!(opaque(A).tightened(opaque(HALF)).mins.x, NEG_ONE + HALF);
    }
    #[test]
    fn gas_transform_by_abs_rotation() {
        assert!(
            opaque(A).transform_by(opaque(P)).contains_local_point(P.transform_point(A.center())),
        );
    }
    #[test]
    fn gas_transform_by_corners() {
        assert!(
            alternatives::transform_by_corners(opaque(A), opaque(P))
                .contains_local_point(P.transform_point(A.center())),
        );
    }
    #[test]
    fn gas_volume() {
        assert_eq!(opaque(A).volume(), FixedTrait::from_int(16));
    }
    #[test]
    fn gas_scaled() {
        assert_eq!(opaque(A).scaled(opaque(v(-ONE, TWO))).mins.x, -THREE);
    }
}
