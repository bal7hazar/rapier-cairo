//! Half-plane clipping in ray time units (no GJK and no unit-direction assumption).
//! Exact edge cross products decide signs; quotients round to nearest (ties away from zero). At a
//! corner the lowest edge id wins a tied rounded time. Normals are outward on entry and inward on
//! exit.
use fixed::{Fixed, ZERO};
use glam::Vec2Trait;
use crate::feature_id::{FEATURE_UNKNOWN, FeatureIdTrait};
use crate::point::cross_wide;
use crate::shape::{ConvexPolygon, ConvexPolygonTrait};
use super::quotient::div_wide;
use super::{Ray, RayIntersection};

/// Clips `ray` against the polygon. `max_time_of_impact` is inclusive and must be >= 0.
/// A solid interior origin returns zero with zero normal/unknown feature; a hollow origin
/// returns the exit with inward normal. A stationary hollow ray has no hit.
/// Panics if coordinate differences or exact i128 cross products overflow. Unrepresentable
/// positive entry times are misses; unrepresentable exits leave the finite time interval open.
pub fn cast_local_ray_and_get_normal_convex_polygon(
    polygon: ConvexPolygon, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<RayIntersection> {
    if max_time_of_impact < ZERO {
        return None;
    }
    let mut enter = ZERO;
    let mut exit = max_time_of_impact;
    let mut enter_id = 0;
    let mut has_enter = false;
    let mut exit_id = 0;
    let mut has_exit = false;
    let mut inside = true;
    let mut i = 0;
    while i != polygon.count {
        let a = polygon.vertex(i);
        let edge = polygon.vertex(polygon.next(i)) - a;
        let d = ray.origin - a;
        let side = cross_wide(edge.x, edge.y, d.x, d.y);
        let speed = cross_wide(edge.x, edge.y, ray.dir.x, ray.dir.y);
        if side < 0 {
            inside = false;
        }
        if speed == 0 {
            if side < 0 {
                return None;
            }
        } else if speed > 0 {
            if side < 0 {
                let t = div_wide(-side, speed)?;
                if t > enter || !has_enter {
                    enter = t;
                    enter_id = i;
                    has_enter = true;
                }
            }
        } else {
            if side < 0 {
                return None;
            }
            if let Some(t) = div_wide(-side, speed) {
                if t < exit || (t == exit && !has_exit) {
                    exit = t;
                    exit_id = i;
                    has_exit = true;
                }
            }
        }
        if enter > exit {
            return None;
        }
        i += 1;
    }
    if inside && solid {
        return Some(
            RayIntersection {
                time_of_impact: ZERO, normal: Vec2Trait::ZERO, feature: FEATURE_UNKNOWN,
            },
        );
    }
    let (time_of_impact, normal, id) = if inside {
        if !has_exit {
            return None;
        }
        (exit, -polygon.normal(exit_id), exit_id)
    } else {
        (enter, polygon.normal(enter_id), enter_id)
    };
    Some(RayIntersection { time_of_impact, normal, feature: FeatureIdTrait::face(id.into()) })
}

/// Time-only wrapper. Semantics, rounding and panics as the normal-returning cast.
pub fn cast_local_ray_convex_polygon(
    polygon: ConvexPolygon, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<Fixed> {
    Some(
        cast_local_ray_and_get_normal_convex_polygon(polygon, ray, max_time_of_impact, solid)?
            .time_of_impact,
    )
}
