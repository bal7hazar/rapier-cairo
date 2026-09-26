//! Nonlinear cast of two support maps (Parry `nonlinear_shape_cast_support_map_support_map.rs`):
//! conservative advancement driven by the exact closest points of `super::super::dispatcher`
//! (upstream: GJK), with upstream's bisection along a frozen separating axis and its handling of
//! a start in contact.
//!
//! # The iteration (upstream's)
//!
//! From the current time, the closest points give a separating direction `n` and a distance;
//! [`bisect`] then searches along `n` (frozen in world space) for the time where the support
//! distance along `n` reaches `[0, tol]`, halving `[min_t, max_t]`. The cast converges when a
//! bisection no longer moves `min_t` by `tol`; before accepting, the configuration at `end_time`
//! is checked along `n` (a separating axis there means no impact at all).
//!
//! # Fixed-point tolerances and bounds
//!
//! Upstream's tolerances are f64 epsilons; here they are `rapier_math::consts`' f32-sized values:
//! `GJK_EPS_TOL = 5120 ulp` (~1.2e-6) for the time and distance tolerances of the bisection and
//! `DEFAULT_EPSILON = 512 ulp` for the degenerate-distance test. The bisection halves its range
//! until it is below `GJK_EPS_TOL`, so it runs at most `log2((end_time - start_time) / tol)`
//! steps (20 for a unit range). Upstream's outer loop has no bound; here it stops after
//! [`MAX_ADVANCEMENTS`] advancements with status `OutOfIterations`. The directional scan of a
//! start in contact runs at most 11 steps, as upstream's `(end - start) / 10` increment does.
//!
//! # Candidates
//!
//! The tolerance is a parameter of [`compute_toi_with_tolerance`]; `tests::gas_compute_toi_*`
//! rank two values. **`GJK_EPS_TOL` (5120 ulp)**: the rotating bar of `tests` costs 30.8M gas /
//! 249k steps; 64 ulp: 40.8M / 331k for a time closer by at most `5056 ulp / approach speed`.
//! The cast is dominated by `NonlinearRigidMotionTrait::position_at_time` (two per bisection
//! step): see `super` for its metered rotation.

use fixed::{Fixed, FixedTrait, MAX, PI, ZERO};
use glam::Vec2;
use rapier_math::consts::{DEFAULT_EPSILON, GJK_EPS_TOL};
use rapier_math::math_ext::vec2::gcross_sv;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::ray::quotient::div_wide;
use crate::shape::{Shape, ShapeTrait};
use super::super::shape_cast::{ShapeCastHit, ShapeCastHitTrait, ShapeCastStatus};
use super::super::support_map::local_support_point_toward;
use super::super::{ClosestPoints, X, dispatcher, normalize_and_length};
use super::{NonlinearRigidMotion, NonlinearRigidMotionTrait, NonlinearShapeCastMode};

/// The time and distance tolerance of the advancement and of the bisection (upstream: f64's
/// `gjk::eps_tol()`); see the module documentation.
pub const TOLERANCE: Fixed = GJK_EPS_TOL;
/// Bound of the conservative-advancement loop (upstream: none).
pub const MAX_ADVANCEMENTS: u32 = 32;
/// Bound of the directional scan: `(end - start) / 10` increments reach the end in 11 steps.
const MAX_SCAN_STEPS: u32 = 11;
/// `1e-5`, upstream's "start time" tolerance of the directional mode (nearest raw).
const AT_START: Fixed = Fixed { raw: 42950 };
/// `2^30`: the margin / prediction standing for upstream's `Real::MAX` in the closest-point and
/// contact queries (`fixed::MAX` would overflow the kernels' `radius sum + margin`).
pub const FAR: Fixed = Fixed { raw: 0x4000000000000000 };
/// `2^32`: a `Fixed` raw at the Q64.64 scale of a wide product.
const SCALE: i128 = 0x100000000;

/// A bisection range (upstream `BisectionRange`).
#[derive(Copy, Drop, Debug, PartialEq)]
pub struct BisectionRange {
    pub min_t: Fixed,
    pub curr_t: Fixed,
    pub max_t: Fixed,
}

/// What a bisection reads the support points of: a shape, or a point fixed in the body.
#[derive(Copy, Drop, Debug)]
pub enum SupportSource {
    Shape: Shape,
    /// Upstream's `ConstantPoint`.
    Point: Vec2,
}

/// The support point of `source` toward the unit `dir`, in its local frame.
#[inline(always)]
fn support(source: SupportSource, dir: Vec2) -> Vec2 {
    match source {
        SupportSource::Shape(s) => local_support_point_toward(s, dir),
        SupportSource::Point(p) => p,
    }
}

#[inline(always)]
fn dot(a: Vec2, b: Vec2) -> Fixed {
    fixed::wide::dot2(a.x, b.x, a.y, b.y)
}

/// `a + (b - a) / 2`, rounded down.
#[inline(always)]
fn midpoint(a: Fixed, b: Fixed) -> Fixed {
    a + Fixed { raw: (b.raw - a.raw) / 2 }
}

/// The relative pose `pos1^-1 pos2` of the two motions at `t`, and `pos1`.
#[inline(always)]
fn poses_at(m1: NonlinearRigidMotion, m2: NonlinearRigidMotion, t: Fixed) -> (Pose2, Pose2) {
    let pos1 = m1.position_at_time(t);
    let pos2 = m2.position_at_time(t);
    (pos1, pos1.inv_mul(pos2))
}

/// The support distance of the two sources along `normal1` (frame of body 1) at `pos12`.
#[inline(always)]
fn support_gap(pos12: Pose2, s1: SupportSource, s2: SupportSource, normal1: Vec2) -> Fixed {
    let pt1 = support(s1, normal1);
    let pt2 = pos12.transform_point(support(s2, pos12.rotation.inverse_rotate(-normal1)));
    dot(pt2, normal1) - dot(pt1, normal1)
}

/// Upstream's `bisect`: from the support distance `dist` along `normal1` (frame of body 1 at
/// `range.curr_t`, frozen in world space), halves the range towards the time where the distance
/// is in `[0, tol]`; `curr_t` jumps to `max_t` when the range falls below `tol`.
/// `(range, number of evaluations)`.
/// #### Panics
/// * See [`NonlinearRigidMotionTrait::position_at_time`].
pub fn bisect(
    dist: Fixed,
    m1: NonlinearRigidMotion,
    s1: SupportSource,
    m2: NonlinearRigidMotion,
    s2: SupportSource,
    normal1: Vec2,
    range: BisectionRange,
    tol: Fixed,
) -> (BisectionRange, u32) {
    let world_normal1 = m1.position_at_time(range.curr_t).rotation.rotate(normal1);
    let mut range = range;
    let mut dist = dist;
    let mut niter = 0;
    loop {
        if dist < ZERO {
            range.max_t = range.curr_t;
            range.curr_t = midpoint(range.min_t, range.curr_t);
        } else if dist > tol {
            range.min_t = range.curr_t;
            range.curr_t = midpoint(range.curr_t, range.max_t);
        } else {
            break;
        }
        if range.max_t - range.min_t < tol {
            range.curr_t = range.max_t;
            break;
        }
        let (pos1, pos12) = poses_at(m1, m2, range.curr_t);
        dist = support_gap(pos12, s1, s2, pos1.rotation.inverse_rotate(world_normal1));
        niter += 1;
    }
    (range, niter)
}

/// The first impact of the support maps `g1` following `motion1` and `g2` following `motion2`
/// within `[start_time, end_time]` (upstream `cast_shapes_nonlinear_support_map_support_map`):
/// the shape of larger bounding sphere goes first (better convergence, as upstream), the hit is
/// swapped back.
/// #### Panics
/// * `'Query: not a support map'` for a half-space; see [`compute_toi`].
pub fn cast_shapes_nonlinear_support_map_support_map(
    motion1: NonlinearRigidMotion,
    g1: Shape,
    motion2: NonlinearRigidMotion,
    g2: Shape,
    start_time: Fixed,
    end_time: Fixed,
    mode: NonlinearShapeCastMode,
) -> Option<ShapeCastHit> {
    let r1 = g1.compute_local_bounding_sphere().radius;
    let r2 = g2.compute_local_bounding_sphere().radius;
    if r1 >= r2 {
        compute_toi(motion1, g1, motion2, g2, start_time, end_time, mode)
    } else {
        let hit = compute_toi(motion2, g2, motion1, g1, start_time, end_time, mode)?;
        Some(hit.swapped())
    }
}

/// Upstream's `compute_toi`: conservative advancement from `start_time` (see the module
/// documentation), then the directional handling of a start in contact.
///
/// Status: `PenetratingOrWithinTargetDist` for a start in contact, `Converged` when the
/// advancement stalls or stops, `Failed` when the shapes are found in contact after the start
/// or the closest points degenerate, `OutOfIterations` past [`MAX_ADVANCEMENTS`].
/// #### Panics
/// * See [`NonlinearRigidMotionTrait::position_at_time`] and the closest-point kernels.
pub fn compute_toi(
    motion1: NonlinearRigidMotion,
    g1: Shape,
    motion2: NonlinearRigidMotion,
    g2: Shape,
    start_time: Fixed,
    end_time: Fixed,
    mode: NonlinearShapeCastMode,
) -> Option<ShapeCastHit> {
    compute_toi_with_tolerance(motion1, g1, motion2, g2, start_time, end_time, mode, TOLERANCE)
}

/// [`compute_toi`] with the time and distance tolerance `tol` of the advancement and bisection.
pub fn compute_toi_with_tolerance(
    motion1: NonlinearRigidMotion,
    g1: Shape,
    motion2: NonlinearRigidMotion,
    g2: Shape,
    start_time: Fixed,
    end_time: Fixed,
    mode: NonlinearShapeCastMode,
    tol: Fixed,
) -> Option<ShapeCastHit> {
    let mut prev_min_t = start_time;
    let mut result = ShapeCastHit {
        time_of_impact: start_time,
        witness1: Vec2 { x: ZERO, y: ZERO },
        witness2: Vec2 { x: ZERO, y: ZERO },
        normal1: X,
        normal2: X,
        status: ShapeCastStatus::OutOfIterations,
    };
    let mut advancements = 0;
    while advancements != MAX_ADVANCEMENTS {
        advancements += 1;
        let (_, pos12) = poses_at(motion1, motion2, result.time_of_impact);
        match dispatcher::closest_points(pos12, g1, g2, FAR)? {
            ClosestPoints::Intersecting => {
                result
                    .status =
                        if result.time_of_impact == start_time {
                            ShapeCastStatus::PenetratingOrWithinTargetDist
                        } else {
                            ShapeCastStatus::Failed
                        };
                break;
            },
            ClosestPoints::WithinMargin((
                p1, p2,
            )) => {
                result.witness1 = p1;
                result.witness2 = p2;
                let (normal1, dist) = normalize_and_length(pos12.transform_point(p2) - p1);
                let Some(normal1) = normal1 else {
                    result.status = ShapeCastStatus::Failed;
                    break;
                };
                if dist < DEFAULT_EPSILON {
                    result.status = ShapeCastStatus::Failed;
                    break;
                }
                result.normal1 = normal1;
                result.normal2 = pos12.rotation.inverse_rotate(-normal1);
                let range = BisectionRange {
                    min_t: result.time_of_impact, max_t: end_time, curr_t: result.time_of_impact,
                };
                let (new_range, niter) = bisect(
                    dist,
                    motion1,
                    SupportSource::Shape(g1),
                    motion2,
                    SupportSource::Shape(g2),
                    normal1,
                    range,
                    tol,
                );
                result.time_of_impact = new_range.curr_t;
                if new_range.min_t - prev_min_t < tol {
                    if new_range.max_t == end_time {
                        // A separating axis at the end configuration: no impact.
                        let (_, pos12) = poses_at(motion1, motion2, new_range.max_t);
                        let gap = support_gap(
                            pos12, SupportSource::Shape(g1), SupportSource::Shape(g2), normal1,
                        );
                        if gap > ZERO {
                            return None;
                        }
                    }
                    result.status = ShapeCastStatus::Converged;
                    break;
                }
                prev_min_t = new_range.min_t;
                if niter == 0 {
                    result.status = ShapeCastStatus::Converged;
                    break;
                }
            },
            ClosestPoints::Disjoint => {
                result.status = ShapeCastStatus::Failed;
                break;
            },
        }
    }
    match mode {
        NonlinearShapeCastMode::Directional((
            sum_linear_thickness, max_angular_thickness,
        )) => {
            let d = result.time_of_impact - start_time;
            if d.abs() < AT_START {
                handle_penetration_at_start_time(
                    motion1,
                    g1,
                    motion2,
                    g2,
                    start_time,
                    end_time,
                    sum_linear_thickness,
                    max_angular_thickness,
                    tol,
                )
            } else {
                Some(result)
            }
        },
        NonlinearShapeCastMode::StopAtPenetration => Some(result),
    }
}

/// `num / den` for `den > 0`, `fixed::MAX` when it overflows; 0 for a zero `den` (upstream's
/// `utils::inv(0) = 0`).
#[inline(always)]
fn ratio(num: Fixed, den: Fixed) -> Fixed {
    if den == ZERO {
        return ZERO;
    }
    div_wide(num.raw.into(), den.raw.into()).unwrap_or(MAX)
}

#[inline(always)]
fn min(a: Fixed, b: Fixed) -> Fixed {
    if a < b {
        a
    } else {
        b
    }
}

/// The velocity of the point `local` (frame of the body at `pos`) of `motion`.
#[inline(always)]
fn point_velocity(motion: NonlinearRigidMotion, pos: Pose2, local: Vec2) -> Vec2 {
    let r = pos.rotation.rotate(local - motion.local_center);
    let (x, y) = gcross_sv(motion.angvel, r.x, r.y);
    motion.linvel + Vec2 { x, y }
}

/// Upstream's `handle_penetration_at_start_time`: scans `[start_time, end_time]` by increments
/// of the time to rotate by `pi - max_angular_thickness` or to move by `sum_linear_thickness`
/// (at most a tenth of the interval), and reports the first contact whose normal velocity would
/// cover more than the thickness before `end_time`, bisected to its time of impact. Without a
/// relative rotation only the start is examined.
/// #### Panics
/// * See [`compute_toi`].
fn handle_penetration_at_start_time(
    motion1: NonlinearRigidMotion,
    g1: Shape,
    motion2: NonlinearRigidMotion,
    g2: Shape,
    start_time: Fixed,
    end_time: Fixed,
    sum_linear_thickness: Fixed,
    max_angular_thickness: Fixed,
    tol: Fixed,
) -> Option<ShapeCastHit> {
    let dangvel = (motion2.angvel - motion1.angvel).abs();
    let dlin = motion2.linvel - motion1.linvel;
    let (_, dlin_len) = normalize_and_length(dlin);
    let linear_time_increment = ratio(sum_linear_thickness, dlin_len);
    let angular_time_increment = ratio(PI - max_angular_thickness, dangvel);
    let tenth = ratio(end_time - start_time, Fixed { raw: 10 * 0x100000000 });
    let mut time_increment = min(min(angular_time_increment, linear_time_increment), tenth);
    if time_increment == ZERO {
        time_increment = end_time;
    }
    let mut next_time = start_time;
    let mut steps = 0;
    while next_time < end_time && steps != MAX_SCAN_STEPS {
        steps += 1;
        let (pos1, pos12) = poses_at(motion1, motion2, next_time);
        let pos2 = pos1.mul(pos12);
        let contact = dispatcher::contact(pos12, g1, g2, FAR)??;
        let vel1 = point_velocity(motion1, pos1, contact.point1);
        let vel2 = point_velocity(motion2, pos2, contact.point2);
        let vel12 = vel2 - vel1;
        let n = pos1.rotation.rotate(contact.normal1);
        let normal_vel = -dot(vel12, n);
        let ccd_threshold = if contact.dist <= ZERO {
            sum_linear_thickness
        } else {
            contact.dist + sum_linear_thickness
        };
        // `normal_vel * (end_time - next_time) > ccd_threshold`, compared wide.
        let travel: i128 = normal_vel.raw.into() * (end_time - next_time).raw.into();
        let threshold: i128 = ccd_threshold.raw.into() * SCALE;
        if travel > threshold {
            let mut result = ShapeCastHit {
                time_of_impact: next_time,
                witness1: contact.point1,
                witness2: contact.point2,
                normal1: contact.normal1,
                normal2: contact.normal2,
                status: ShapeCastStatus::Converged,
            };
            let (range, s1, s2) = if contact.dist > ZERO {
                (
                    BisectionRange { min_t: next_time, max_t: end_time, curr_t: next_time },
                    SupportSource::Shape(g1),
                    SupportSource::Shape(g2),
                )
            } else {
                (
                    BisectionRange { min_t: start_time, max_t: next_time, curr_t: next_time },
                    SupportSource::Point(contact.point1),
                    SupportSource::Point(contact.point2),
                )
            };
            let (new_range, _) = bisect(
                contact.dist, motion1, s1, motion2, s2, contact.normal1, range, tol,
            );
            result.time_of_impact = new_range.curr_t;
            return Some(result);
        }
        if dangvel == ZERO {
            return None;
        }
        next_time = next_time + time_increment;
    }
    None
}
