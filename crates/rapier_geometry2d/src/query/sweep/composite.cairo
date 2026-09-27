//! Swept time of impact against a composite shape (Parry 0.31 `query/sweep_toi/composite.rs`,
//! work package SH2a): the composite is stationary; its parts met by the moving shape's swept
//! box are each a segment proxy, swept against the moving proxy by [`sweep_time_of_impact`], the
//! earliest impact strictly inside `(0, max_fraction)` winning; a part reporting an initial
//! overlap is retried with a ball of `CORE_FRACTION * min_extent` about the moving shape's
//! centroid.
//!
//! 2D branches, as upstream: a compound (SH2b: each part's proxy swept at its world pose; parts
//! without a proxy, half-spaces, skipped), a polyline (its `ORIENTED` flag enables the one-sided
//! early-out:
//! a segment is skipped when the centroid starts behind it, or ends in front of it by more than
//! the core distance after moving less than that towards it) and a heightfield (two-sided unless
//! the caller asks for `one_sided`).
//!
//! Deviations: the polyline's parts come from the implicit tree (ascending), upstream's from its
//! BVH; ties between parts keep the first.

use fixed::{Fixed, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::aabb::{Aabb, AabbTrait};
use crate::shape::{CompoundTrait, HeightFieldTrait, PolylineTrait, Segment, Shape};
use super::{
    Sweep, SweepToiOutput, SweepToiStatus, SweepTrait, ToiProxy, ToiProxyTrait,
    sweep_time_of_impact,
};

/// Upstream `CORE_FRACTION` (`1/4`): the fallback ball's radius as a fraction of the moving
/// shape's smallest extent.
pub const CORE_FRACTION: Fixed = Fixed { raw: 0x40000000 };

/// The moving shape of a composite sweep (upstream `SweepCompositeFastShape`).
#[derive(Copy, Drop, Debug)]
pub struct SweepCompositeFastShape {
    pub proxy: ToiProxy,
    pub sweep: Sweep,
    /// Centroid in the moving body's frame (the fallback ball's centre).
    pub local_centroid: Vec2,
    /// The moving shape's thickness (its `ccd_thickness`).
    pub min_extent: Fixed,
}

/// The search state (upstream `CompositeToiContext`).
#[derive(Copy, Drop)]
struct Context {
    fast: SweepCompositeFastShape,
    local_centroid1: Vec2,
    local_centroid2: Vec2,
    fallback_radius: Fixed,
    one_sided: bool,
    linear_slop: Fixed,
    max_fraction: Fixed,
    best: Option<SweepToiOutput>,
}

/// Upstream `toi_against_element`: the segment proxy against the moving proxy, then the fallback
/// ball on an initial overlap; keeps an impact strictly inside `(0, max_fraction)`.
fn toi_against_element(ref ctx: Context, segment: Segment, element_sweep: Sweep) {
    let element = ToiProxyTrait::from_array(array![segment.a, segment.b].span(), ZERO);
    let output = sweep_time_of_impact(
        element, element_sweep, ctx.fast.proxy, ctx.fast.sweep, ctx.max_fraction, ctx.linear_slop,
    );
    if ZERO < output.fraction && output.fraction < ctx.max_fraction {
        ctx.max_fraction = output.fraction;
        ctx.best = Some(output);
    } else if output.fraction == ZERO {
        let fallback = ToiProxyTrait::point(ctx.fast.local_centroid, ctx.fallback_radius);
        let output = sweep_time_of_impact(
            element, element_sweep, fallback, ctx.fast.sweep, ctx.max_fraction, ctx.linear_slop,
        );
        if ZERO < output.fraction && output.fraction < ctx.max_fraction {
            ctx.max_fraction = output.fraction;
            ctx.best = Some(output);
        }
    }
}

/// Upstream `toi_against_element` on a compound part's proxy: [`toi_against_element`] with any
/// proxy (a copy, so that the segment path keeps its code).
fn toi_against_proxy(ref ctx: Context, element: ToiProxy, element_sweep: Sweep) {
    let output = sweep_time_of_impact(
        element, element_sweep, ctx.fast.proxy, ctx.fast.sweep, ctx.max_fraction, ctx.linear_slop,
    );
    if ZERO < output.fraction && output.fraction < ctx.max_fraction {
        ctx.max_fraction = output.fraction;
        ctx.best = Some(output);
    } else if output.fraction == ZERO {
        let fallback = ToiProxyTrait::point(ctx.fast.local_centroid, ctx.fallback_radius);
        let output = sweep_time_of_impact(
            element, element_sweep, fallback, ctx.fast.sweep, ctx.max_fraction, ctx.linear_slop,
        );
        if ZERO < output.fraction && output.fraction < ctx.max_fraction {
            ctx.max_fraction = output.fraction;
            ctx.best = Some(output);
        }
    }
}

/// Upstream `one_sided_early_out` (2D): `true` when the one-sided `segment` cannot stop the
/// motion (see the module documentation).
fn one_sided_early_out(ctx: @Context, segment: Segment) -> bool {
    if !*ctx.one_sided {
        return false;
    }
    let e = segment.b - segment.a;
    let length = e.length();
    if length <= *ctx.linear_slop {
        return false;
    }
    let e = e.div_scalar(length);
    let separation1 = (*ctx.local_centroid1 - segment.a).perp_dot(e);
    let separation2 = (*ctx.local_centroid2 - segment.a).perp_dot(e);
    let core_distance = CORE_FRACTION * *ctx.fast.min_extent;
    separation1 < ZERO || (separation1 - separation2 < core_distance && separation2 > core_distance)
}

/// Upstream `sweep_time_of_impact_composite` for the 2D polyline and heightfield: the earliest
/// accepted impact of the moving shape `fast` against the stationary `composite` at
/// `composite_pose` (`Separated` at `max_fraction` when none), `None` for a shape that is not a
/// polyline, a heightfield or a compound. `target_is_sensor` only matters in 3D upstream and is
/// ignored.
/// #### Panics
/// * The panics of [`sweep_time_of_impact`].
pub fn sweep_time_of_impact_composite(
    composite: Shape,
    composite_pose: Pose2,
    fast: SweepCompositeFastShape,
    one_sided: bool,
    target_is_sensor: bool,
    max_fraction: Fixed,
    linear_slop: Fixed,
) -> Option<SweepToiOutput> {
    let _ = target_is_sensor;
    let start_aabb = fast.proxy.compute_aabb(fast.sweep.transform_at(ZERO));
    let end_aabb = fast.proxy.compute_aabb(fast.sweep.transform_at(max_fraction));
    let local_aabb: Aabb = start_aabb.merged(end_aabb).transform_by(composite_pose.inverse());
    let centroid_world1 = fast.sweep.transform_at(ZERO).transform_point(fast.local_centroid);
    let centroid_world2 = fast.sweep.final_transform().transform_point(fast.local_centroid);
    let mut ctx = Context {
        fast,
        local_centroid1: composite_pose.inverse_transform_point(centroid_world1),
        local_centroid2: composite_pose.inverse_transform_point(centroid_world2),
        fallback_radius: CORE_FRACTION * fast.min_extent,
        one_sided,
        linear_slop,
        max_fraction,
        best: None,
    };
    let composite_sweep = SweepTrait::constant(composite_pose, Vec2 { x: ZERO, y: ZERO });
    match composite {
        Shape::Polyline(p) => {
            let p = p.unbox();
            ctx.one_sided = one_sided || p.is_oriented();
            for seg_id in p.segments_in_aabb(local_aabb) {
                let segment = p.segment(seg_id);
                if one_sided_early_out(@ctx, segment) {
                    continue;
                }
                toi_against_element(ref ctx, segment, composite_sweep);
            }
        },
        Shape::HeightField(h) => {
            for (_, segment) in h.unbox().elements_in_local_aabb(local_aabb) {
                if !one_sided_early_out(@ctx, segment) {
                    toi_against_element(ref ctx, segment, composite_sweep);
                }
            }
        },
        Shape::Compound(c) => {
            let c = c.unbox();
            let shapes = c.shapes();
            for id in c.parts_in_aabb(local_aabb) {
                let (part_pose, part) = *shapes.at(id);
                // Parts without a proxy (half-spaces) are skipped, as upstream; a compound has no
                // composite part.
                if let Some(proxy) = ToiProxyTrait::from_shape(part) {
                    let sweep = SweepTrait::constant(
                        composite_pose.mul(part_pose), Vec2 { x: ZERO, y: ZERO },
                    );
                    toi_against_proxy(ref ctx, proxy, sweep);
                }
            }
        },
        _ => { return None; },
    }
    Some(
        ctx
            .best
            .unwrap_or(
                SweepToiOutput {
                    status: SweepToiStatus::Separated,
                    fraction: max_fraction,
                    point: Vec2 { x: ZERO, y: ZERO },
                    normal: Vec2 { x: ZERO, y: ZERO },
                },
            ),
    )
}
