//! Ray casts on the [`Compound`] (work package SH2b): Parry `impl RayCast for Compound`
//! (`query/ray/ray_composite_shape.rs`, through `CompositeShapeRef`).
//!
//! The earliest hit among the parts, each cast in its own frame (upstream
//! `Shape::cast_ray_and_get_normal(part_pose, ray, best_so_far, solid)`: the ray moved into the
//! part's frame, the normal rotated back), strictly below `max_time_of_impact`, ascending part
//! index on exact ties (upstream: BVH order). The part's feature is kept, as upstream's part
//! answer. The `*_part` function also returns the part's index (upstream
//! `CompositeShapeRef::cast_local_ray_and_get_normal`).

use fixed::Fixed;
use crate::shape::{Compound, CompoundTrait, Shape};
use super::{
    Ray, RayIntersection, RayIntersectionTrait, RayTrait, cast_local_ray_and_get_normal_ball,
    cast_local_ray_and_get_normal_capsule, cast_local_ray_and_get_normal_cuboid,
    cast_local_ray_and_get_normal_halfspace, cast_local_ray_and_get_normal_rounded,
    cast_local_ray_and_get_normal_segment, convex_polygon, triangle,
};

/// `cast_local_ray_and_get_normal` on a convex part, out of line (the inlined dispatcher cannot
/// sit on the recursion compound → part).
#[inline(never)]
fn cast_part(
    shape: Shape, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<RayIntersection> {
    match shape {
        Shape::Ball(s) => cast_local_ray_and_get_normal_ball(s, ray, max_time_of_impact, solid),
        Shape::Cuboid(s) => cast_local_ray_and_get_normal_cuboid(s, ray, max_time_of_impact, solid),
        Shape::Capsule(s) => cast_local_ray_and_get_normal_capsule(
            s, ray, max_time_of_impact, solid,
        ),
        Shape::Segment(s) => cast_local_ray_and_get_normal_segment(
            s, ray, max_time_of_impact, solid,
        ),
        Shape::ConvexPolygon(s) => convex_polygon::cast_local_ray_and_get_normal_convex_polygon(
            s.unbox(), ray, max_time_of_impact, solid,
        ),
        Shape::HalfSpace(s) => cast_local_ray_and_get_normal_halfspace(
            s, ray, max_time_of_impact, solid,
        ),
        Shape::Triangle(s) => triangle::cast_local_ray_and_get_normal_triangle(
            s.unbox(), ray, max_time_of_impact, solid,
        ),
        _ => cast_local_ray_and_get_normal_rounded(shape, ray, max_time_of_impact, solid),
    }
}

/// The earliest hit of `ray` on the parts of `compound`, strictly below `max_time_of_impact`:
/// `(part, hit)`, the normal in the compound's frame.
pub fn cast_local_ray_and_get_normal_compound_part(
    compound: @Compound, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<(u32, RayIntersection)> {
    let mut best: Option<(u32, RayIntersection)> = None;
    let mut bound = max_time_of_impact;
    let mut i: u32 = 0;
    for part in compound.shapes() {
        let (pose, shape) = *part;
        if let Some(hit) = cast_part(shape, ray.inverse_transform_by(pose), bound, solid) {
            let better = match best {
                Some((_, b)) => hit.time_of_impact < b.time_of_impact,
                None => hit.time_of_impact < max_time_of_impact,
            };
            if better {
                best = Some((i, hit.transform_by(pose)));
                bound = hit.time_of_impact;
            }
        }
        i += 1;
    }
    best
}

/// Upstream `RayCast::cast_local_ray_and_get_normal` for `Compound`.
pub fn cast_local_ray_and_get_normal_compound(
    compound: @Compound, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<RayIntersection> {
    let (_, hit) = cast_local_ray_and_get_normal_compound_part(
        compound, ray, max_time_of_impact, solid,
    )?;
    Some(hit)
}

/// Upstream `RayCast::cast_local_ray` for `Compound`.
pub fn cast_local_ray_compound(
    compound: @Compound, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<Fixed> {
    let (_, hit) = cast_local_ray_and_get_normal_compound_part(
        compound, ray, max_time_of_impact, solid,
    )?;
    Some(hit.time_of_impact)
}

pub impl CompoundRayCast of super::RayCast<Compound> {
    fn cast_local_ray(
        self: Compound, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<Fixed> {
        cast_local_ray_compound(@self, ray, max_time_of_impact, solid)
    }
    fn cast_local_ray_and_get_normal(
        self: Compound, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<RayIntersection> {
        cast_local_ray_and_get_normal_compound(@self, ray, max_time_of_impact, solid)
    }
}
