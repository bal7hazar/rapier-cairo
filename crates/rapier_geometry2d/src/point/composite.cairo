//! Point queries on the composite shapes (work package SH2a): the [`Polyline`] (Parry
//! `query/point/point_composite_shape.rs`) and the 2D [`HeightField`]
//! (`query/point/point_heightfield.rs`).
//!
//! Both answer the projection on their closest part: every segment of a polyline, every enabled
//! cell of a heightfield, in ascending index, the first strictly closer part winning (the closest
//! point compared on the exact wide squared distance). Upstream walks its BVH best first
//! (polyline) or a growing window of cells (heightfield), which finds the same minimum; only
//! exact ties may pick another part. The part's index is returned out of band by the `*_part`
//! functions (upstream's `CompositeShapeRef` answers and `PointProjection::subshape`, a field the
//! port leaves out of `PointProjection` so that the convex shapes' projections keep their width).
//!
//! An `ORIENTED` polyline decides `is_inside` with the pseudo-normal of the feature it projected
//! on, as upstream: `(pt - proj) . n <= 0`, where `n` is the vertex pseudo-normal on a vertex and
//! the face normal on an edge; a solid query inside answers the point itself.

use fixed::wide::distance2;
use fixed::{Fixed, FixedTrait, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::math_ext::norm2::norm2_sq_wide;
use crate::feature_id::{FEATURE_UNKNOWN, FeatureId, FeatureIdTrait};
use crate::shape::{
    HeightField, HeightFieldTrait, Polyline, PolylineTrait, Segment, SegmentTrait, Shape,
};
use super::query::{PointQuery, PointQueryWithLocation};
use super::segment::{
    contains_local_point_segment, project_local_point_and_get_feature_segment,
    project_local_point_and_get_location_segment,
};
use super::{PointProjection, PointProjectionTrait, SegmentPointLocation};

/// `|pt - q|^2`, exact.
#[inline(always)]
fn sq_dist(pt: Vec2, q: Vec2) -> i128 {
    norm2_sq_wide(q.x - pt.x, q.y - pt.y)
}

/// The closest part of `parts` to `pt` with its location: `(part index, projection, location)`,
/// `None` for no part. `ids` are the indices of the parts (same length).
fn closest_segment(
    parts: Span<Segment>, ids: Span<u32>, pt: Vec2, solid: bool,
) -> Option<(u32, PointProjection, SegmentPointLocation)> {
    let mut best: Option<(u32, PointProjection, SegmentPointLocation)> = None;
    let mut best_sq: i128 = 0;
    let mut k: u32 = 0;
    for seg in parts {
        let (proj, loc) = project_local_point_and_get_location_segment(*seg, pt, solid);
        let d = sq_dist(pt, proj.point);
        if best.is_none() || d < best_sq {
            best = Some((*ids.at(k), proj, loc));
            best_sq = d;
        }
        k += 1;
    }
    best
}

/// `0, 1, …, n - 1`.
fn range(n: u32) -> Span<u32> {
    let mut out = array![];
    let mut i: u32 = 0;
    while i != n {
        out.append(i);
        i += 1;
    }
    out.span()
}

/// The pseudo-normal of `location` on segment `seg_id` of an `ORIENTED` polyline, `None`
/// otherwise (upstream: `segment_normal_constraints`, the vertex's on a vertex).
#[inline(always)]
fn oriented_normal(
    polyline: @Polyline, seg_id: u32, location: SegmentPointLocation,
) -> Option<Vec2> {
    let constraints = polyline.segment_normal_constraints(seg_id)?;
    let [e0, e1] = constraints.edges;
    Some(
        match location {
            SegmentPointLocation::OnVertex(i) => if i == 0 {
                e0
            } else {
                e1
            },
            SegmentPointLocation::OnEdge(_) => constraints.face,
        },
    )
}

/// Upstream `Polyline::project_local_point_and_get_location_with_max_dist` without the
/// distance bound: `(segment, projection, location)` of the closest segment, `None` for an
/// empty polyline.
pub fn project_local_point_and_get_location_polyline_part(
    polyline: @Polyline, pt: Vec2, solid: bool,
) -> Option<(u32, PointProjection, SegmentPointLocation)> {
    let segments = polyline.segments();
    let (seg_id, proj, loc) = closest_segment(segments.span(), range(segments.len()), pt, solid)?;
    let mut proj = proj;
    if let Some(n) = oriented_normal(polyline, seg_id, loc) {
        proj.is_inside = (pt - proj.point).dot(n) <= ZERO;
        if proj.is_inside && solid {
            proj.point = pt;
        }
    }
    Some((seg_id, proj, loc))
}

/// Upstream `Polyline::project_local_point_and_get_location`: the projection on the closest
/// segment and `(segment, location)`; an empty polyline answers `pt` outside and
/// `(0, OnVertex(0))`.
pub fn project_local_point_and_get_location_polyline(
    polyline: @Polyline, pt: Vec2, solid: bool,
) -> (PointProjection, (u32, SegmentPointLocation)) {
    match project_local_point_and_get_location_polyline_part(polyline, pt, solid) {
        Some((id, proj, loc)) => (proj, (id, loc)),
        None => (PointProjectionTrait::new(false, pt), (0, SegmentPointLocation::OnVertex(0))),
    }
}

/// Upstream `PointQuery::project_local_point` for `Polyline`.
pub fn project_local_point_polyline(polyline: @Polyline, pt: Vec2, solid: bool) -> PointProjection {
    let (proj, _) = project_local_point_and_get_location_polyline(polyline, pt, solid);
    proj
}

/// Upstream `PointQuery::project_local_point_and_get_feature` for `Polyline`: the closest
/// segment's projection (its feature decides the pseudo-normal of an `ORIENTED` polyline) and the
/// polyline feature `Face(segment)`; `pt` outside and `Unknown` for an empty polyline.
pub fn project_local_point_and_get_feature_polyline(
    polyline: @Polyline, pt: Vec2,
) -> (PointProjection, FeatureId) {
    let mut best: Option<(u32, PointProjection, FeatureId)> = None;
    let mut best_sq: i128 = 0;
    let mut i: u32 = 0;
    for seg in polyline.segments() {
        let (proj, feature) = project_local_point_and_get_feature_segment(seg, pt);
        let d = sq_dist(pt, proj.point);
        if best.is_none() || d < best_sq {
            best = Some((i, proj, feature));
            best_sq = d;
        }
        i += 1;
    }
    let Some((seg_id, proj, feature)) = best else {
        return (PointProjectionTrait::new(false, pt), FEATURE_UNKNOWN);
    };
    let mut proj = proj;
    if let Some(constraints) = polyline.segment_normal_constraints(seg_id) {
        let [e0, e1] = constraints.edges;
        let n = if feature.is_vertex() {
            if feature.unwrap_vertex() == 0 {
                e0
            } else {
                e1
            }
        } else {
            constraints.face
        };
        proj.is_inside = (pt - proj.point).dot(n) <= ZERO;
    }
    (proj, polyline.segment_feature_to_polyline_feature(seg_id, feature))
}

/// Upstream's default `distance_to_local_point` on the polyline projection: the distance, negated
/// for a non-solid query from inside (an `ORIENTED` polyline's interior).
pub fn distance_to_local_point_polyline(polyline: @Polyline, pt: Vec2, solid: bool) -> Fixed {
    let proj = project_local_point_polyline(polyline, pt, solid);
    let dist = distance2(pt.x, pt.y, proj.point.x, proj.point.y);
    if solid || !proj.is_inside {
        dist
    } else {
        ZERO - dist
    }
}

/// Upstream `PointQuery::contains_local_point` for `Polyline`: an `ORIENTED` polyline contains
/// its interior (solid projection inside); otherwise `pt` must lie on a segment.
pub fn contains_local_point_polyline(polyline: @Polyline, pt: Vec2) -> bool {
    if polyline.is_oriented() {
        let (proj, _) = project_local_point_and_get_location_polyline(polyline, pt, true);
        return proj.is_inside;
    }
    for seg in polyline.segments() {
        if contains_local_point_segment(seg, pt) {
            return true;
        }
    }
    false
}

/// Upstream `Polyline::project_local_point_assuming_solid_interior_ccw`: the non-solid
/// projection, with `is_inside` decided as if the polyline bounded a counter-clockwise solid (a
/// reentrant vertex is inside; collinear neighbours fall back to the segment normal).
/// #### Panics
/// * `'Polyline: open at vertex'` when a vertex projection's neighbouring segment does not share
///   it (upstream `assert_eq!`), as upstream for an open or unordered polyline.
pub fn project_local_point_assuming_solid_interior_ccw(
    polyline: @Polyline, pt: Vec2,
) -> (PointProjection, (u32, SegmentPointLocation)) {
    let (mut proj, (seg_id, loc)) = project_local_point_and_get_location_polyline(
        polyline, pt, false,
    );
    if polyline.num_segments() == 0 {
        return (proj, (seg_id, loc));
    }
    let segment1 = polyline.segment(seg_id);
    let Some(normal1) = SegmentTrait::normal(segment1) else {
        return (proj, (seg_id, loc));
    };
    let n = polyline.num_segments();
    proj.is_inside = match loc {
        SegmentPointLocation::OnVertex(i) => {
            let dir2 = if i == 0 {
                let adj = if seg_id == 0 {
                    n - 1
                } else {
                    seg_id - 1
                };
                let adj_seg = polyline.segment(adj);
                assert(segment1.a == adj_seg.b, 'Polyline: open at vertex');
                -adj_seg.scaled_direction()
            } else {
                let adj = if seg_id + 1 == n {
                    0
                } else {
                    seg_id + 1
                };
                let adj_seg = polyline.segment(adj);
                assert(segment1.b == adj_seg.a, 'Polyline: open at vertex');
                adj_seg.scaled_direction()
            };
            let dot = normal1.dot(dir2);
            // `1e-3 * |dir2|`, as upstream.
            let threshold = dir2.length() * THOUSANDTH;
            if FixedTrait::abs(dot) > threshold {
                dot >= ZERO
            } else {
                (pt - proj.point).dot(normal1) <= ZERO
            }
        },
        SegmentPointLocation::OnEdge(_) => (pt - proj.point).dot(normal1) <= ZERO,
    };
    (proj, (seg_id, loc))
}

/// `1e-3` rounded to the nearest Q32.32.
const THOUSANDTH: Fixed = Fixed { raw: 4294967 };

/// The closest enabled cell of `heightfield` to `pt`: `(cell, projection)`, `None` when every
/// cell is removed.
pub fn project_local_point_heightfield_part(
    heightfield: @HeightField, pt: Vec2, solid: bool,
) -> Option<(u32, PointProjection)> {
    let mut parts = array![];
    let mut ids = array![];
    let mut i: u32 = 0;
    while i != heightfield.num_cells() {
        if let Some(seg) = heightfield.segment_at(i) {
            parts.append(seg);
            ids.append(i);
        }
        i += 1;
    }
    let (id, proj, _) = closest_segment(parts.span(), ids.span(), pt, solid)?;
    Some((id, proj))
}

/// Upstream `PointQuery::project_local_point` for `HeightField`: the closest enabled cell's
/// projection; `pt` outside when every cell is removed.
pub fn project_local_point_heightfield(
    heightfield: @HeightField, pt: Vec2, solid: bool,
) -> PointProjection {
    match project_local_point_heightfield_part(heightfield, pt, solid) {
        Some((_, proj)) => proj,
        None => PointProjectionTrait::new(false, pt),
    }
}

/// Upstream `project_local_point_with_max_dist` for `HeightField`: the projection when it lies
/// within `max_dist` of `pt`.
pub fn project_local_point_with_max_dist_heightfield(
    heightfield: @HeightField, pt: Vec2, solid: bool, max_dist: Fixed,
) -> Option<PointProjection> {
    let (_, proj) = project_local_point_heightfield_part(heightfield, pt, solid)?;
    let d = proj.point - pt;
    if max_dist < ZERO || rapier_math::math_ext::norm2::is_norm2_gt(d.x, d.y, max_dist) {
        None
    } else {
        Some(proj)
    }
}

/// Upstream `project_local_point_and_get_feature` for `HeightField`: the non-solid projection
/// and `Unknown`.
pub fn project_local_point_and_get_feature_heightfield(
    heightfield: @HeightField, pt: Vec2,
) -> (PointProjection, FeatureId) {
    (project_local_point_heightfield(heightfield, pt, false), FEATURE_UNKNOWN)
}

/// Upstream's default `distance_to_local_point` on the heightfield projection (a heightfield
/// has no interior: never negative).
pub fn distance_to_local_point_heightfield(
    heightfield: @HeightField, pt: Vec2, solid: bool,
) -> Fixed {
    let proj = project_local_point_heightfield(heightfield, pt, solid);
    distance2(pt.x, pt.y, proj.point.x, proj.point.y)
}

/// `false`: upstream's heightfield contains no point.
#[inline(always)]
pub fn contains_local_point_heightfield(heightfield: @HeightField, pt: Vec2) -> bool {
    false
}

pub impl PolylinePointQuery of PointQuery<Polyline> {
    fn project_local_point(self: Polyline, pt: Vec2, solid: bool) -> PointProjection {
        project_local_point_polyline(@self, pt, solid)
    }
    fn project_local_point_and_get_feature(
        self: Polyline, pt: Vec2,
    ) -> (PointProjection, FeatureId) {
        project_local_point_and_get_feature_polyline(@self, pt)
    }
    fn distance_to_local_point(self: Polyline, pt: Vec2, solid: bool) -> Fixed {
        distance_to_local_point_polyline(@self, pt, solid)
    }
    fn contains_local_point(self: Polyline, pt: Vec2) -> bool {
        contains_local_point_polyline(@self, pt)
    }
}

pub impl PolylinePointQueryWithLocation of PointQueryWithLocation<
    Polyline, (u32, SegmentPointLocation),
> {
    fn project_local_point_and_get_location(
        self: Polyline, pt: Vec2, solid: bool,
    ) -> (PointProjection, (u32, SegmentPointLocation)) {
        project_local_point_and_get_location_polyline(@self, pt, solid)
    }
}

pub impl HeightFieldPointQuery of PointQuery<HeightField> {
    fn project_local_point(self: HeightField, pt: Vec2, solid: bool) -> PointProjection {
        project_local_point_heightfield(@self, pt, solid)
    }
    fn project_local_point_and_get_feature(
        self: HeightField, pt: Vec2,
    ) -> (PointProjection, FeatureId) {
        project_local_point_and_get_feature_heightfield(@self, pt)
    }
    fn distance_to_local_point(self: HeightField, pt: Vec2, solid: bool) -> Fixed {
        distance_to_local_point_heightfield(@self, pt, solid)
    }
    fn contains_local_point(self: HeightField, pt: Vec2) -> bool {
        contains_local_point_heightfield(@self, pt)
    }
    fn project_local_point_with_max_dist(
        self: HeightField, pt: Vec2, solid: bool, max_dist: Fixed,
    ) -> Option<PointProjection> {
        project_local_point_with_max_dist_heightfield(@self, pt, solid, max_dist)
    }
}

/// The composite arms of `ShapePointQuery::project_local_point`, out of line.
#[inline(never)]
pub fn project_local_point_composite(shape: Shape, pt: Vec2, solid: bool) -> PointProjection {
    match shape {
        Shape::Polyline(s) => project_local_point_polyline(@s.unbox(), pt, solid),
        Shape::HeightField(s) => project_local_point_heightfield(@s.unbox(), pt, solid),
        _ => PointProjectionTrait::new(false, pt),
    }
}

/// The composite arms of `ShapePointQuery::project_local_point_and_get_feature`, out of line.
#[inline(never)]
pub fn project_local_point_and_get_feature_composite(
    shape: Shape, pt: Vec2,
) -> (PointProjection, FeatureId) {
    match shape {
        Shape::Polyline(s) => project_local_point_and_get_feature_polyline(@s.unbox(), pt),
        Shape::HeightField(s) => project_local_point_and_get_feature_heightfield(@s.unbox(), pt),
        _ => (PointProjectionTrait::new(false, pt), FEATURE_UNKNOWN),
    }
}

/// The composite arms of `ShapePointQuery::distance_to_local_point`, out of line.
#[inline(never)]
pub fn distance_to_local_point_composite(shape: Shape, pt: Vec2, solid: bool) -> Fixed {
    match shape {
        Shape::Polyline(s) => distance_to_local_point_polyline(@s.unbox(), pt, solid),
        Shape::HeightField(s) => distance_to_local_point_heightfield(@s.unbox(), pt, solid),
        _ => ZERO,
    }
}

/// The composite arms of `ShapePointQuery::contains_local_point`, out of line.
#[inline(never)]
pub fn contains_local_point_composite(shape: Shape, pt: Vec2) -> bool {
    match shape {
        Shape::Polyline(s) => contains_local_point_polyline(@s.unbox(), pt),
        _ => false,
    }
}
