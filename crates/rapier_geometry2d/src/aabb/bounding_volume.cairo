//! Bounding volumes (Parry `bounding_volume/{bounding_volume,bounding_sphere,aabb_utils,
//! aabb_ball,bounding_sphere_utils}.rs`): the [`BoundingVolume`] trait on [`Aabb`] and
//! [`BoundingSphere`], and the point-cloud / ball / support-map box and sphere builders.
//!
//! None of this is reached by the step: the broad phase keeps using `AabbTrait` directly.
//! Sphere tests (`intersects`, `contains`) compare exact wide squares instead of floored lengths.

use core::num::traits::WideMul;
use fixed::wide::norm2;
use fixed::{Fixed, MAX};
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::aabb::{Aabb, AabbTrait};
use crate::point::dot_wide;
use crate::shape::support_map::SupportMap;

pub mod errors {
    /// A loosening / tightening margin is negative (upstream asserts `amount >= 0`).
    pub const NEGATIVE_MARGIN: felt252 = 'Bounding: negative margin';
    /// Tightening would invert the volume.
    pub const MARGIN_TOO_LARGE: felt252 = 'Bounding: margin too large';
    /// A point cloud needs at least one point.
    pub const EMPTY_POINT_CLOUD: felt252 = 'Bounding: empty point cloud';
}

/// A ball bounding a shape: `center` and `radius` (upstream `BoundingSphere`).
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct BoundingSphere {
    pub center: Vec2,
    pub radius: Fixed,
}

#[generate_trait]
pub impl BoundingSphereImpl of BoundingSphereTrait {
    /// Sphere of `center` and `radius` (`radius >= 0`, not checked, as upstream).
    #[inline(always)]
    fn new(center: Vec2, radius: Fixed) -> BoundingSphere {
        BoundingSphere { center, radius }
    }

    #[inline(always)]
    fn center(self: BoundingSphere) -> Vec2 {
        self.center
    }

    #[inline(always)]
    fn radius(self: BoundingSphere) -> Fixed {
        self.radius
    }

    /// The sphere moved by `pose`: the centre is transformed (one floor per component), the
    /// radius is kept.
    #[inline(always)]
    fn transform_by(self: BoundingSphere, pose: Pose2) -> BoundingSphere {
        BoundingSphere { center: pose.transform_point(self.center), radius: self.radius }
    }

    /// The sphere moved by `translation` (exact).
    #[inline(always)]
    fn translated(self: BoundingSphere, translation: Vec2) -> BoundingSphere {
        BoundingSphere { center: self.center + translation, radius: self.radius }
    }
}

/// Operations shared by bounding volumes (upstream `BoundingVolume`), implemented for [`Aabb`]
/// and [`BoundingSphere`].
///
/// `AabbTrait` has inherent `center` / `intersects` / `contains` / `merged` / `loosened` /
/// `tightened` with the same results except for the margin checks below; bring only one of the
/// two traits into scope, or call through the trait path.
pub trait BoundingVolume<T> {
    /// The centre of the volume.
    fn center(self: T) -> Vec2;
    /// Whether the volumes overlap, boundary included.
    fn intersects(self: T, other: T) -> bool;
    /// Whether `self` contains `other`, boundary included.
    fn contains(self: T, other: T) -> bool;
    /// Grows `self` to contain `other`.
    fn merge(ref self: T, other: T);
    /// The smallest volume of this kind containing both (smallest for spheres up to rounding).
    fn merged(self: T, other: T) -> T;
    /// Grows every side by `amount`.
    /// #### Panics
    /// * `'Bounding: negative margin'` for `amount < 0`.
    fn loosen(ref self: T, amount: Fixed);
    /// `self` grown by `amount`.
    /// #### Panics
    /// * `'Bounding: negative margin'` for `amount < 0`.
    fn loosened(self: T, amount: Fixed) -> T;
    /// Shrinks every side by `amount`.
    /// #### Panics
    /// * `'Bounding: negative margin'` for `amount < 0`.
    /// * `'Bounding: margin too large'` when the volume would invert.
    fn tighten(ref self: T, amount: Fixed);
    /// `self` shrunk by `amount` (for an `Aabb`, the result may be inverted, as upstream).
    /// #### Panics
    /// * `'Bounding: negative margin'` for `amount < 0`.
    /// * `'Bounding: margin too large'` for a sphere whose radius is below `amount`.
    fn tightened(self: T, amount: Fixed) -> T;
}

pub impl AabbBoundingVolume of BoundingVolume<Aabb> {
    #[inline(always)]
    fn center(self: Aabb) -> Vec2 {
        AabbTrait::center(self)
    }
    #[inline(always)]
    fn intersects(self: Aabb, other: Aabb) -> bool {
        AabbTrait::intersects(self, other)
    }
    #[inline(always)]
    fn contains(self: Aabb, other: Aabb) -> bool {
        AabbTrait::contains(self, other)
    }
    #[inline(always)]
    fn merge(ref self: Aabb, other: Aabb) {
        self = AabbTrait::merged(self, other);
    }
    #[inline(always)]
    fn merged(self: Aabb, other: Aabb) -> Aabb {
        AabbTrait::merged(self, other)
    }
    fn loosen(ref self: Aabb, amount: Fixed) {
        self = Self::loosened(self, amount);
    }
    fn loosened(self: Aabb, amount: Fixed) -> Aabb {
        assert(amount.raw >= 0, errors::NEGATIVE_MARGIN);
        AabbTrait::loosened(self, amount)
    }
    fn tighten(ref self: Aabb, amount: Fixed) {
        let tight = Self::tightened(self, amount);
        assert(
            tight.mins.x <= tight.maxs.x && tight.mins.y <= tight.maxs.y, errors::MARGIN_TOO_LARGE,
        );
        self = tight;
    }
    fn tightened(self: Aabb, amount: Fixed) -> Aabb {
        assert(amount.raw >= 0, errors::NEGATIVE_MARGIN);
        AabbTrait::tightened(self, amount)
    }
}

/// `|d| <= |r|` exactly (any sign of `r`, as upstream compares squares): `|d|^2` and `r^2` as
/// exact Q64.64 raws, widening `i64` products; `r` beyond the `i64` range squares in `u128`.
/// #### Panics
/// * `'i128_add Overflow'` only when both components of `d` sit at the ends of the range.
#[inline(always)]
fn within(d: Vec2, r: i128) -> bool {
    let d_sq = dot_wide(d.x, d.y, d.x, d.y);
    let r64: Option<i64> = r.try_into();
    match r64 {
        Option::Some(r) => d_sq <= r.wide_mul(r),
        Option::None => {
            let r_abs = if r < 0 {
                -r
            } else {
                r
            };
            let r_u: u128 = r_abs.try_into().unwrap();
            let d_u: u128 = d_sq.try_into().unwrap();
            d_u <= r_u * r_u
        },
    }
}

/// `|d| + r2 <= r1`, exactly: `r1 - r2 >= 0` and `|d|^2 <= (r1 - r2)^2`.
#[inline(always)]
fn sphere_contains(d: Vec2, r1: Fixed, r2: Fixed) -> bool {
    let slack: i128 = r1.raw.into() - r2.raw.into();
    slack >= 0 && within(d, slack)
}

pub impl BoundingSphereBoundingVolume of BoundingVolume<BoundingSphere> {
    #[inline(always)]
    fn center(self: BoundingSphere) -> Vec2 {
        self.center
    }
    /// `|c2 - c1|^2 <= (r1 + r2)^2`, exact.
    fn intersects(self: BoundingSphere, other: BoundingSphere) -> bool {
        within(other.center - self.center, self.radius.raw.into() + other.radius.raw.into())
    }
    /// `|c2 - c1| + r2 <= r1`, exact (no square root).
    fn contains(self: BoundingSphere, other: BoundingSphere) -> bool {
        sphere_contains(other.center - self.center, self.radius, other.radius)
    }
    fn merge(ref self: BoundingSphere, other: BoundingSphere) {
        self = Self::merged(self, other);
    }
    /// Closed form of upstream's two-extreme-points merge: the containing sphere when one
    /// contains the other (decided on the floored `|d|`, as upstream decides on its rounded one),
    /// else radius `(|d| + r1 + r2) / 2` centred on `c1 + d (R - r1) / |d|`. One floored length,
    /// one division.
    fn merged(self: BoundingSphere, other: BoundingSphere) -> BoundingSphere {
        let d = other.center - self.center;
        let length = norm2(d.x, d.y);
        let l: i128 = length.raw.into();
        let slack: i128 = self.radius.raw.into() - other.radius.raw.into();
        if l <= slack {
            return self;
        }
        if l <= -slack {
            return other;
        }
        let radius = Fixed { raw: (length.raw + self.radius.raw + other.radius.raw) / 2 };
        let t = (radius - self.radius) / length;
        BoundingSphere { center: self.center + d.mul_scalar(t), radius }
    }
    fn loosen(ref self: BoundingSphere, amount: Fixed) {
        self = Self::loosened(self, amount);
    }
    fn loosened(self: BoundingSphere, amount: Fixed) -> BoundingSphere {
        assert(amount.raw >= 0, errors::NEGATIVE_MARGIN);
        BoundingSphere { center: self.center, radius: self.radius + amount }
    }
    fn tighten(ref self: BoundingSphere, amount: Fixed) {
        self = Self::tightened(self, amount);
    }
    fn tightened(self: BoundingSphere, amount: Fixed) -> BoundingSphere {
        assert(amount.raw >= 0, errors::NEGATIVE_MARGIN);
        assert(amount <= self.radius, errors::MARGIN_TOO_LARGE);
        BoundingSphere { center: self.center, radius: self.radius - amount }
    }
}

/// Box of the ball of `radius` centred on `center` (upstream `ball_aabb`). Exact.
#[inline(always)]
pub fn ball_aabb(center: Vec2, radius: Fixed) -> Aabb {
    AabbTrait::from_half_extents(center, Vec2 { x: radius, y: radius })
}

/// Box of the ball of `radius` centred on the origin (upstream `local_ball_aabb`). Exact.
#[inline(always)]
pub fn local_ball_aabb(radius: Fixed) -> Aabb {
    let h = Vec2 { x: radius, y: radius };
    AabbTrait::new(-h, h)
}

/// Exact component-wise bounds of `pts` (upstream `local_point_cloud_aabb`; `Span` is the
/// by-reference form, so it also stands for `local_point_cloud_aabb_ref`).
/// #### Panics
/// * `'Bounding: empty point cloud'` for no point.
pub fn local_point_cloud_aabb(pts: Span<Vec2>) -> Aabb {
    let mut pts = pts;
    let p0 = *pts.pop_front().expect(errors::EMPTY_POINT_CLOUD);
    let mut aabb = Aabb { mins: p0, maxs: p0 };
    for p in pts {
        aabb.take_point(*p);
    }
    aabb
}

/// Bounds of `pts` placed at `pose` (upstream `point_cloud_aabb` / `point_cloud_aabb_ref`):
/// every point is transformed (one floor per component), then bounded exactly.
/// #### Panics
/// * `'Bounding: empty point cloud'` for no point.
pub fn point_cloud_aabb(pose: Pose2, pts: Span<Vec2>) -> Aabb {
    let mut pts = pts;
    let p0 = pose.transform_point(*pts.pop_front().expect(errors::EMPTY_POINT_CLOUD));
    let mut aabb = Aabb { mins: p0, maxs: p0 };
    for p in pts {
        aabb.take_point(pose.transform_point(*p));
    }
    aabb
}

/// Local box of a support map from its four axis support points (upstream
/// `local_support_map_aabb`): exact for the shapes of the closed set.
pub fn local_support_map_aabb<T, +Drop<T>, +Copy<T>, +SupportMap<T>>(shape: T) -> Aabb {
    let x = Vec2Trait::X;
    let y = Vec2Trait::Y;
    Aabb {
        mins: Vec2 {
            x: SupportMap::local_support_point(shape, -x).x,
            y: SupportMap::local_support_point(shape, -y).y,
        },
        maxs: Vec2 {
            x: SupportMap::local_support_point(shape, x).x,
            y: SupportMap::local_support_point(shape, y).y,
        },
    }
}

/// Sphere of `center` through the farthest point of `pts` (upstream
/// `point_cloud_bounding_sphere_with_center`): the farthest point is chosen on exact wide
/// squares, its distance is floored. An empty cloud gives a zero radius, as upstream.
pub fn point_cloud_bounding_sphere_with_center(pts: Span<Vec2>, center: Vec2) -> BoundingSphere {
    let mut farthest = Vec2Trait::ZERO;
    let mut max_sq: i128 = 0;
    for p in pts {
        let delta = *p - center;
        let sq = dot_wide(delta.x, delta.y, delta.x, delta.y);
        if sq > max_sq {
            max_sq = sq;
            farthest = delta;
        }
    }
    BoundingSphere { center, radius: norm2(farthest.x, farthest.y) }
}

/// Sphere centred on the mean of `pts` (upstream `point_cloud_bounding_sphere`). The mean sums
/// the raws wide and truncates toward zero, as `ConvexPolygonTrait::compute_local_bounding_sphere`.
/// #### Panics
/// * `'Bounding: empty point cloud'` for no point.
pub fn point_cloud_bounding_sphere(pts: Span<Vec2>) -> BoundingSphere {
    let n: i128 = pts.len().into();
    assert(n != 0, errors::EMPTY_POINT_CLOUD);
    let mut sx: i128 = 0;
    let mut sy: i128 = 0;
    for p in pts {
        sx += (*p.x.raw).into();
        sy += (*p.y.raw).into();
    }
    let center = Vec2 {
        x: Fixed { raw: (sx / n).try_into().unwrap() },
        y: Fixed { raw: (sy / n).try_into().unwrap() },
    };
    point_cloud_bounding_sphere_with_center(pts, center)
}

/// Radius of an unbounded shape's sphere (upstream `Real::max_value()`).
pub(crate) const UNBOUNDED_RADIUS: Fixed = MAX;

/// Sphere of the origin-centred `radius` placed at `pose`: the centre is the translation.
#[inline(always)]
pub(crate) fn centered_bounding_sphere(pose: Pose2, radius: Fixed) -> BoundingSphere {
    BoundingSphere { center: pose.translation, radius }
}

/// Rejected candidates, kept for the `gas_*` ranking and as oracles.
#[cfg(test)]
pub mod alternatives {
    use fixed::wide::norm2;
    use fixed::{Fixed, ZERO};
    use glam::{Vec2, Vec2Trait};
    use super::{BoundingSphere, BoundingVolume};

    /// Upstream's literal merge: normalise `d`, take the two extreme points along it, and centre
    /// on their midpoint. Two divisions (normalisation) and a second square root.
    pub fn merged_extremes(a: BoundingSphere, b: BoundingSphere) -> BoundingSphere {
        let d = b.center - a.center;
        let (dir, length) = d.normalize_and_length();
        if length == ZERO {
            return if b.radius > a.radius {
                BoundingSphere { center: a.center, radius: b.radius }
            } else {
                a
            };
        }
        let s = a.center.dot(dir);
        let o = b.center.dot(dir);
        let right = if s + a.radius > o + b.radius {
            a.center + dir.mul_scalar(a.radius)
        } else {
            b.center + dir.mul_scalar(b.radius)
        };
        let left = if -s + a.radius > -o + b.radius {
            a.center - dir.mul_scalar(a.radius)
        } else {
            b.center - dir.mul_scalar(b.radius)
        };
        let center = left.midpoint(right);
        let r = right - center;
        BoundingSphere { center, radius: norm2(r.x, r.y) }
    }

    /// `|v|^2` through `i128` squares converted to `u128` (no overflow anywhere).
    fn norm_squared_u128(v: Vec2) -> u128 {
        let x: i128 = v.x.raw.into();
        let y: i128 = v.y.raw.into();
        let xx: u128 = (x * x).try_into().unwrap();
        let yy: u128 = (y * y).try_into().unwrap();
        xx + yy
    }

    fn within_u128(d_sq: u128, r: i128) -> bool {
        let r: u128 = if r < 0 {
            (-r).try_into().unwrap()
        } else {
            r.try_into().unwrap()
        };
        if r >= 0x10000000000000000 {
            true
        } else {
            d_sq <= r * r
        }
    }

    /// Exact `intersects` on `i128` squares widened to `u128` (the first formulation).
    pub fn intersects_u128(a: BoundingSphere, b: BoundingSphere) -> bool {
        within_u128(
            norm_squared_u128(b.center - a.center), a.radius.raw.into() + b.radius.raw.into(),
        )
    }

    /// Exact `contains` on `i128` squares widened to `u128` (the first formulation).
    pub fn contains_u128(a: BoundingSphere, b: BoundingSphere) -> bool {
        let slack: i128 = a.radius.raw.into() - b.radius.raw.into();
        slack >= 0 && within_u128(norm_squared_u128(b.center - a.center), slack)
    }

    /// The closed-form merge with the two exact containment tests before the square root.
    pub fn merged_exact_containment(a: BoundingSphere, b: BoundingSphere) -> BoundingSphere {
        if BoundingVolume::contains(a, b) {
            return a;
        }
        if BoundingVolume::contains(b, a) {
            return b;
        }
        let d = b.center - a.center;
        let length = norm2(d.x, d.y);
        let radius = Fixed { raw: (length.raw + a.radius.raw + b.radius.raw) / 2 };
        let t = (radius - a.radius) / length;
        BoundingSphere { center: a.center + d.mul_scalar(t), radius }
    }

    /// Upstream's literal `contains`: floored length, then compare (not exact).
    pub fn contains_sqrt(a: BoundingSphere, b: BoundingSphere) -> bool {
        let d = b.center - a.center;
        norm2(d.x, d.y) + b.radius <= a.radius
    }

    /// `intersects` on floored `Fixed` squares (loses the sub-ulp part, overflows early).
    pub fn intersects_fixed(a: BoundingSphere, b: BoundingSphere) -> bool {
        let d = b.center - a.center;
        let s: Fixed = a.radius + b.radius;
        d.length_squared() <= s * s
    }
}

#[cfg(test)]
mod tests;
