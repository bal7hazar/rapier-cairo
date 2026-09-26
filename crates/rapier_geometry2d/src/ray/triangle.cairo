//! Ray casts on a [`Triangle`] (Parry `query/ray/ray_triangle.rs`, 2D branch).
//!
//! Upstream's algorithm: a solid ray whose origin is strictly on the same side of the three
//! edges answers `t = 0` with the normal `+Y` and `Face(0)`; otherwise the earliest hit among the
//! three edge segments (`ab`, `bc`, `ca`, strictly earlier wins), with the segment's own normal
//! and feature. The side tests are exact wide cross products and the edge casts the exact
//! segment kernel of [`super::segment`].

use fixed::{Fixed, MAX, ONE, ZERO};
use glam::vec2::Vec2;
use crate::feature_id::FeatureIdTrait;
use crate::point::wide2::cross_wide;
use crate::shape::{Segment, Triangle};
use super::segment::cast_local_ray_and_get_normal_segment;
use super::{Ray, RayIntersection};

/// `cross(q - p, o - p) > 0`, exact.
#[inline(always)]
fn left_of(p: Vec2, q: Vec2, o: Vec2) -> bool {
    let e = q - p;
    let d = o - p;
    cross_wide(e.x, e.y, d.x, d.y) > 0
}

/// Time of impact, normal and feature of `ray` on `triangle` (local frame; upstream 2D
/// `cast_local_ray_and_get_normal`). See the module documentation.
/// #### Panics
/// * The panics of the segment kernel (coordinate differences leaving the scalar range).
/// #### Deviations
/// * The side tests are exact; upstream reads the signs of rounded cross products.
pub fn cast_local_ray_and_get_normal_triangle(
    triangle: Triangle, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<RayIntersection> {
    let (a, b, c) = (triangle.a, triangle.b, triangle.c);
    if solid {
        let s1 = left_of(a, b, ray.origin);
        let s2 = left_of(b, c, ray.origin);
        let s3 = left_of(c, a, ray.origin);
        if s1 == s2 && s1 == s3 {
            return Some(
                RayIntersection {
                    time_of_impact: ZERO,
                    normal: Vec2 { x: ZERO, y: ONE },
                    feature: FeatureIdTrait::face(0),
                },
            );
        }
    }
    let mut best: Option<RayIntersection> = None;
    let mut smallest = MAX;
    for edge in [Segment { a, b }, Segment { a: b, b: c }, Segment { a: c, b: a }].span() {
        if let Some(hit) =
            cast_local_ray_and_get_normal_segment(*edge, ray, max_time_of_impact, solid) {
            if hit.time_of_impact < smallest {
                smallest = hit.time_of_impact;
                best = Some(hit);
            }
        }
    }
    best
}

/// Time of impact of `ray` on `triangle` (upstream default `cast_local_ray`).
pub fn cast_local_ray_triangle(
    triangle: Triangle, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<Fixed> {
    Some(
        cast_local_ray_and_get_normal_triangle(triangle, ray, max_time_of_impact, solid)?
            .time_of_impact,
    )
}
