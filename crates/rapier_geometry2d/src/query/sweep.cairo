//! Swept time of impact of two convex proxies (Parry `query/sweep_toi/`, from Box2D's
//! `b2TimeOfImpact`), work package CC1: the part of upstream's API that Rapier's CCD solver calls
//! (`Sweep`, `ToiProxy`, `sweep_time_of_impact`, `SweepToiStatus`), for CC2.
//!
//! * [`Sweep`] interpolates a body's pose between two poses: its centre linearly, its rotation by
//!   normalised linear interpolation (`nlerp`).
//! * [`ToiProxy`] is a convex point cloud dilated by a radius: every shape of the closed set but
//!   the half-space ([`ToiProxyTrait::from_shape`]).
//! * [`sweep_time_of_impact`] advances `t1` from 0 by conservative root finding on the
//!   separation functions of `separation` until the proxies are within `target ± tolerance` of
//!   each other, `target = max(linear_slop, r_a + r_b - linear_slop)`, `tolerance =
//!   linear_slop / 4`; each distance comes from the exact kernel of [`proxy_distance`] (upstream:
//!   GJK).
//!
//! Iteration bounds are upstream's 2D ones: 20 distance iterations, 8 push-back iterations, 50
//! root-finder iterations; deterministic (no data-dependent exit other than upstream's).
//!
//! # Deviations
//!
//! * [`proxy_distance`] is exact (closest features, see its module) where upstream runs GJK; the
//!   simplex cache it returns (the features) drives the same separation functions.
//! * A half-turn `nlerp` (opposite rotations at `t = 1/2`) has no direction: upstream normalises
//!   a zero complex number (`NaN`); the port keeps the start rotation there.
//! * Composite targets: [`composite::sweep_time_of_impact_composite`] (SH2a).

pub mod composite;
pub mod proxy;
pub mod separation;
use fixed::{Fixed, HALF, ZERO};
use glam::{Vec2, Vec2Trait};
pub use proxy::{ProxyDistanceOutput, SimplexCache, proxy_distance};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::{Rot2, Rot2Trait};
use separation::SeparationFunctionTrait;
use crate::aabb::Aabb;
use crate::point::wide2::dot_wide;
use crate::shape::Shape;

#[cfg(test)]
mod tests;

/// Maximum inline point count of a proxy in 2D (upstream `TOI_PROXY_INLINE_POINTS`). The port
/// stores every proxy as a `Span`, so this is informative only.
pub const TOI_PROXY_INLINE_POINTS: u32 = 4;
/// Upstream's 2D bound of the distance iterations.
const MAX_DISTANCE_ITERATIONS: u32 = 20;
/// Upstream's 2D bound of the push-back iterations (the largest polygon vertex count).
const MAX_PUSH_BACK_ITERATIONS: u32 = 8;
/// Upstream's bound of the root-finder iterations.
const MAX_ROOT_ITERATIONS: u32 = 50;

/// The motion of a body over a step (Parry `Sweep`): its centre `c1 -> c2`, its rotation
/// `q1 -> q2`, about `local_center` (the centre of mass in the body frame).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Sweep {
    pub local_center: Vec2,
    pub c1: Vec2,
    pub c2: Vec2,
    pub q1: Rot2,
    pub q2: Rot2,
}

/// `normalize(q1 + (q2 - q1) t)` (upstream `nlerp`, 2D); the start rotation when the
/// interpolation vanishes (a half turn at `t = 1/2`).
fn nlerp(q1: Rot2, q2: Rot2, t: Fixed) -> Rot2 {
    let q = Rot2 { re: q1.re + (q2.re - q1.re) * t, im: q1.im + (q2.im - q1.im) * t };
    if q.re == ZERO && q.im == ZERO {
        return q1;
    }
    q.renormalize()
}

#[generate_trait]
pub impl SweepImpl of SweepTrait {
    /// The sweep from `start` to `end` about `local_center` (upstream `from_poses`).
    fn from_poses(start: Pose2, end: Pose2, local_center: Vec2) -> Sweep {
        Sweep {
            local_center,
            c1: start.transform_point(local_center),
            c2: end.transform_point(local_center),
            q1: start.rotation,
            q2: end.rotation,
        }
    }

    /// The sweep standing still at `pose` (upstream `constant`).
    #[inline(always)]
    fn constant(pose: Pose2, local_center: Vec2) -> Sweep {
        Self::from_poses(pose, pose, local_center)
    }

    /// The pose at fraction `t` (upstream `transform_at`): `nlerp` of the rotations, linear
    /// interpolation of the centre.
    /// #### Panics
    /// * `'Fixed: overflow'` for coordinates leaving the scalar range.
    fn transform_at(self: Sweep, t: Fixed) -> Pose2 {
        let q = nlerp(self.q1, self.q2, t);
        let c = self.c1 + (self.c2 - self.c1).mul_scalar(t);
        Pose2 { translation: c - q.rotate(self.local_center), rotation: q }
    }

    /// The pose at the end of the sweep (upstream `final_transform`).
    fn final_transform(self: Sweep) -> Pose2 {
        Pose2 { translation: self.c2 - self.q2.rotate(self.local_center), rotation: self.q2 }
    }

    /// The sweep with its centres moved by `-origin` (upstream `shifted`).
    #[inline(always)]
    fn shifted(self: Sweep, origin: Vec2) -> Sweep {
        Sweep { c1: self.c1 - origin, c2: self.c2 - origin, ..self }
    }
}

/// A convex point cloud dilated by `radius` (Parry `ToiProxy`), in its body's frame.
#[derive(Copy, Drop, Debug, PartialEq)]
pub struct ToiProxy {
    /// One point, a segment's two, or a convex polygon's vertices in order.
    pub points: Span<Vec2>,
    pub radius: Fixed,
}

#[generate_trait]
pub impl ToiProxyImpl of ToiProxyTrait {
    /// A single point (upstream `point`).
    #[inline(always)]
    fn point(point: Vec2, radius: Fixed) -> ToiProxy {
        ToiProxy { points: array![point].span(), radius }
    }

    /// A point cloud (upstream `from_points`; `from_array` is the same here).
    /// #### Panics
    /// * `'ToiProxy: no point'` for an empty span (upstream asserts it).
    fn from_points(points: Span<Vec2>, radius: Fixed) -> ToiProxy {
        assert(points.len() != 0, 'ToiProxy: no point');
        ToiProxy { points, radius }
    }

    /// Upstream `from_array` (a fixed-size array there): see [`ToiProxyTrait::from_points`].
    #[inline(always)]
    fn from_array(points: Span<Vec2>, radius: Fixed) -> ToiProxy {
        Self::from_points(points, radius)
    }

    /// The proxy of a shape (upstream `from_shape`): a ball is its centre, a cuboid its four
    /// corners, a capsule or a segment its two end points, a triangle or a convex polygon its
    /// vertices, a round shape its inner shape's with the border as radius; `None` for a
    /// half-space and the composite shapes (their parts are proxied one by one).
    fn from_shape(shape: Shape) -> Option<ToiProxy> {
        match shape {
            Shape::Ball(b) => Some(Self::point(Vec2 { x: ZERO, y: ZERO }, b.radius)),
            Shape::Cuboid(c) => Some(cuboid_proxy(c.half_extents, ZERO)),
            Shape::RoundCuboid(r) => Some(
                cuboid_proxy(r.inner_shape.half_extents, r.border_radius),
            ),
            Shape::Capsule(c) => Some(
                ToiProxy { points: array![c.segment.a, c.segment.b].span(), radius: c.radius },
            ),
            Shape::Segment(s) => Some(ToiProxy { points: array![s.a, s.b].span(), radius: ZERO }),
            Shape::Triangle(t) => {
                let t = t.unbox();
                Some(ToiProxy { points: array![t.a, t.b, t.c].span(), radius: ZERO })
            },
            Shape::RoundTriangle(r) => {
                let r = r.unbox();
                let t = r.inner_shape;
                Some(ToiProxy { points: array![t.a, t.b, t.c].span(), radius: r.border_radius })
            },
            Shape::ConvexPolygon(p) => Some(polygon_proxy(p.unbox(), ZERO)),
            Shape::RoundConvexPolygon(r) => {
                let r = r.unbox();
                Some(polygon_proxy(r.inner_shape, r.border_radius))
            },
            Shape::HalfSpace(_) => None,
            Shape::Polyline(_) => None,
            Shape::HeightField(_) => None,
            Shape::Compound(_) => None,
        }
    }

    /// The points (upstream `points`).
    #[inline(always)]
    fn points(self: ToiProxy) -> Span<Vec2> {
        self.points
    }

    /// The index of the point farthest along `direction`, measured from the first point (upstream
    /// `support`: the first maximum wins, exact wide dot products).
    fn support(self: ToiProxy, direction: Vec2) -> u32 {
        let origin = *self.points[0];
        let mut best_index = 0;
        let mut best_value: i128 = 0;
        let n = self.points.len();
        let mut i = 1;
        while i != n {
            let d = *self.points[i] - origin;
            let value = dot_wide(direction.x, direction.y, d.x, d.y);
            if value > best_value {
                best_index = i;
                best_value = value;
            }
            i += 1;
        }
        best_index
    }

    /// The box of the proxy placed at `pose` (upstream `compute_aabb`).
    fn compute_aabb(self: ToiProxy, pose: Pose2) -> Aabb {
        let first = pose.transform_point(*self.points[0]);
        let (mut mins, mut maxs) = (first, first);
        for p in self.points {
            let q = pose.transform_point(*p);
            mins = mins.min(q);
            maxs = maxs.max(q);
        }
        let r = Vec2 { x: self.radius, y: self.radius };
        Aabb { mins: mins - r, maxs: maxs + r }
    }
}

fn cuboid_proxy(h: Vec2, radius: Fixed) -> ToiProxy {
    ToiProxy {
        points: array![
            Vec2 { x: -h.x, y: -h.y }, Vec2 { x: h.x, y: -h.y }, h, Vec2 { x: -h.x, y: h.y },
        ]
            .span(),
        radius,
    }
}

fn polygon_proxy(p: crate::shape::ConvexPolygon, radius: Fixed) -> ToiProxy {
    let mut points = array![];
    let mut i = 0;
    while i != p.count {
        points.append(crate::shape::ConvexPolygonTrait::vertex(p, i));
        i += 1;
    }
    ToiProxy { points: points.span(), radius }
}

/// How a swept time of impact ended (Parry `SweepToiStatus`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum SweepToiStatus {
    /// The cores overlap at the start (fraction 0).
    Overlapped,
    /// The proxies come within the target distance at `fraction`.
    Hit,
    /// No impact before `max_fraction`.
    Separated,
    /// The root finder did not converge; `fraction` is the last safe time.
    Failed,
}

/// The answer of [`sweep_time_of_impact`] (Parry `SweepToiOutput`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SweepToiOutput {
    pub status: SweepToiStatus,
    /// The sweep fraction of the impact (`max_fraction` when separated, 0 when overlapped).
    pub fraction: Fixed,
    /// World point halfway between the two dilated surfaces at the impact.
    pub point: Vec2,
    /// World unit normal from A towards B at the impact.
    pub normal: Vec2,
}

/// The first fraction of the sweeps at which `proxy_a` and `proxy_b` come within the target
/// distance (upstream `sweep_time_of_impact`); see the module documentation.
/// #### Panics
/// * The overflow panics of the pose interpolation and of the wide products.
pub fn sweep_time_of_impact(
    proxy_a: ToiProxy,
    sweep_a: Sweep,
    proxy_b: ToiProxy,
    sweep_b: Sweep,
    max_fraction: Fixed,
    linear_slop: Fixed,
) -> SweepToiOutput {
    let mut output = SweepToiOutput {
        status: SweepToiStatus::Separated,
        fraction: max_fraction,
        point: Vec2 { x: ZERO, y: ZERO },
        normal: Vec2 { x: ZERO, y: ZERO },
    };
    let origin = sweep_a.c1;
    let sweep_a = sweep_a.shifted(origin);
    let sweep_b = sweep_b.shifted(origin);
    let t_max = max_fraction;
    let total_radius = proxy_a.radius + proxy_b.radius;
    let target = if linear_slop > total_radius - linear_slop {
        linear_slop
    } else {
        total_radius - linear_slop
    };
    let tolerance = Fixed { raw: linear_slop.raw / 4 };
    let mut t1 = ZERO;
    let mut distance_iterations = 0;
    let mut cache: SimplexCache = Default::default();
    loop {
        let xf_a = sweep_a.transform_at(t1);
        let xf_b = sweep_b.transform_at(t1);
        let pos12 = xf_a.inv_mul(xf_b);
        let distance_output = proxy_distance(pos12, proxy_a, proxy_b, false, ref cache);
        let world_normal = xf_a.rotation.rotate(distance_output.normal);
        let world_point_a = xf_a.transform_point(distance_output.point_a);
        let world_point_b = xf_a.transform_point(distance_output.point_b);
        distance_iterations += 1;
        let pa = world_point_a + world_normal.mul_scalar(proxy_a.radius);
        let pb = world_point_b - world_normal.mul_scalar(proxy_b.radius);
        let hit_point = (pa + pb).mul_scalar(HALF) + origin;
        if distance_output.distance <= ZERO {
            output.status = SweepToiStatus::Overlapped;
            output.fraction = ZERO;
            break;
        }
        if distance_output.distance <= target + tolerance {
            output.status = SweepToiStatus::Hit;
            output.point = hit_point;
            output.normal = world_normal;
            output.fraction = t1;
            break;
        }
        let fcn = SeparationFunctionTrait::new(cache, proxy_a, sweep_a, proxy_b, sweep_b, t1);
        let mut done = false;
        let mut t2 = t_max;
        let mut push_back_iterations = 0;
        loop {
            let (s2_first, index_a, index_b) = fcn.find_min_separation(t2);
            let mut s2 = s2_first;
            if s2 - target > tolerance {
                output.status = SweepToiStatus::Separated;
                output.fraction = t_max;
                done = true;
                break;
            }
            if s2 >= target - tolerance {
                t1 = t2;
                break;
            }
            let mut s1 = fcn.evaluate(index_a, index_b, t1);
            if s1 < target - tolerance {
                output.status = SweepToiStatus::Failed;
                output.fraction = t1;
                done = true;
                break;
            }
            if s1 <= target + tolerance {
                output.status = SweepToiStatus::Hit;
                output.point = hit_point;
                output.normal = world_normal;
                output.fraction = t1;
                done = true;
                break;
            }
            // Mixed secant / bisection root finder on `[t1, t2]`.
            let mut root_iteration_count: u32 = 0;
            let (mut a1, mut a2) = (t1, t2);
            loop {
                let t = if root_iteration_count % 2 == 1 {
                    a1 + (target - s1) * (a2 - a1) / (s2 - s1)
                } else {
                    (a1 + a2).mul_half()
                };
                root_iteration_count += 1;
                let s = fcn.evaluate(index_a, index_b, t);
                if (s - target).abs_fixed() <= tolerance {
                    t2 = t;
                    break;
                }
                if s > target {
                    a1 = t;
                    s1 = s;
                } else {
                    a2 = t;
                    s2 = s;
                }
                if root_iteration_count == MAX_ROOT_ITERATIONS {
                    break;
                }
            }
            push_back_iterations += 1;
            if push_back_iterations == MAX_PUSH_BACK_ITERATIONS {
                break;
            }
        }
        if done {
            break;
        }
        if distance_iterations == MAX_DISTANCE_ITERATIONS {
            output.status = SweepToiStatus::Failed;
            output.point = hit_point;
            output.normal = world_normal;
            output.fraction = t1;
            break;
        }
    }
    output
}

#[generate_trait]
impl FixedHelpers of FixedHelpersTrait {
    /// `x / 2`, rounded down.
    #[inline(always)]
    fn mul_half(self: Fixed) -> Fixed {
        Fixed { raw: self.raw / 2 }
    }

    #[inline(always)]
    fn abs_fixed(self: Fixed) -> Fixed {
        if self < ZERO {
            -self
        } else {
            self
        }
    }
}
