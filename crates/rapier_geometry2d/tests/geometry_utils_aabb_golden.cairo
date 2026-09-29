//! `rapier_golden::generated::geometry_utils` against the `Aabb` utilities of PX3: distance to
//! the origin, projection on an axis, scaling about the centre, the canonical split, line / ray
//! / segment clipping, `clip_aabb_line` and the polygon clips.
//!
//! Every case carries its own band in ulps (`tol`, `0` = exact); integers (flags, sides, tags,
//! counts) are always exact.

use fixed::Fixed;
use glam_core::Vec2;
use rapier_geometry2d::aabb::clip::{AabbClipTrait, clip_aabb_line, clip_halfspace_polygon};
use rapier_geometry2d::aabb::utils::AabbUtilsTrait;
use rapier_geometry2d::aabb::{Aabb, AabbTrait};
use rapier_geometry2d::query::split::SplitResult;
use rapier_geometry2d::ray::Ray;
use rapier_geometry2d::shape::Segment;
use rapier_golden::generated::geometry_utils as g;
use rapier_golden::generated::geometry_utils::UtilCase;

fn next(ref s: Span<i64>) -> i64 {
    *s.pop_front().unwrap()
}

fn q(ref s: Span<i64>) -> Fixed {
    Fixed { raw: next(ref s) }
}

fn vec(ref s: Span<i64>) -> Vec2 {
    let x = q(ref s);
    let y = q(ref s);
    Vec2 { x, y }
}

fn bx(ref s: Span<i64>) -> Aabb {
    let mins = vec(ref s);
    let maxs = vec(ref s);
    AabbTrait::new(mins, maxs)
}

fn points(ref s: Span<i64>) -> Array<Vec2> {
    let n = next(ref s);
    let mut out = array![];
    let mut i = 0;
    while i != n {
        out.append(vec(ref s));
        i += 1;
    }
    out
}

fn near(got: Fixed, ref exp: Span<i64>, tol: i64, id: felt252) {
    let d = got.raw - next(ref exp);
    assert(d <= tol && -d <= tol, id);
}

fn near_vec(got: Vec2, ref exp: Span<i64>, tol: i64, id: felt252) {
    near(got.x, ref exp, tol, id);
    near(got.y, ref exp, tol, id);
}

fn near_aabb(got: Aabb, ref exp: Span<i64>, tol: i64, id: felt252) {
    near_vec(got.mins, ref exp, tol, id);
    near_vec(got.maxs, ref exp, tol, id);
}

fn near_points(got: Span<Vec2>, ref exp: Span<i64>, tol: i64, id: felt252) {
    assert(got.len().into() == next(ref exp), id);
    for p in got {
        near_vec(*p, ref exp, tol, id);
    }
}

fn near_segment(got: Option<Segment>, ref exp: Span<i64>, tol: i64, id: felt252) {
    let some = next(ref exp);
    match got {
        None => assert(some == 0, id),
        Some(s) => {
            assert(some == 1, id);
            near_vec(s.a, ref exp, tol, id);
            near_vec(s.b, ref exp, tol, id);
        },
    }
}

fn near_params(got: Option<(Fixed, Fixed)>, ref exp: Span<i64>, tol: i64, id: felt252) {
    let some = next(ref exp);
    match got {
        None => assert(some == 0, id),
        Some((
            t0, t1,
        )) => {
            assert(some == 1, id);
            near(t0, ref exp, tol, id);
            near(t1, ref exp, tol, id);
        },
    }
}

fn near_hit(got: (Fixed, Vec2, i32), ref exp: Span<i64>, tol: i64, id: felt252) {
    let (t, n, side) = got;
    near(t, ref exp, tol, id);
    near_vec(n, ref exp, tol, id);
    assert(side.into() == next(ref exp), id);
}

#[test]
fn test_aabb_utilities() {
    for c in g::aabb_distance_to_origin().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        near(bx(ref i).distance_to_origin(), ref o, tol, id);
    }
    for c in g::aabb_project_on_axis().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let b = bx(ref i);
        let (lo, hi) = b.project_on_axis(vec(ref i));
        near(lo, ref o, tol, id);
        near(hi, ref o, tol, id);
    }
    for c in g::aabb_scaled_wrt_center().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let b = bx(ref i);
        near_aabb(b.scaled_wrt_center(vec(ref i)), ref o, tol, id);
    }
    for c in g::aabb_canonical_split().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let b = bx(ref i);
        let axis: u32 = next(ref i).try_into().unwrap();
        let bias = q(ref i);
        let eps = q(ref i);
        let tag = next(ref o);
        match b.canonical_split(axis, bias, eps) {
            SplitResult::Pair((
                l, r,
            )) => {
                assert(tag == 0, id);
                near_aabb(l, ref o, tol, id);
                near_aabb(r, ref o, tol, id);
            },
            SplitResult::Negative => assert(tag == 1, id),
            SplitResult::Positive => assert(tag == 2, id),
        }
    }
}

#[test]
fn test_clip_aabb_line() {
    for c in g::clip_aabb_line().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let b = bx(ref i);
        let origin = vec(ref i);
        let dir = vec(ref i);
        let some = next(ref o);
        match clip_aabb_line(b, origin, dir) {
            None => assert(some == 0, id),
            Some((
                near_hit_, far_hit,
            )) => {
                assert(some == 1, id);
                near_hit(near_hit_, ref o, tol, id);
                near_hit(far_hit, ref o, tol, id);
            },
        }
    }
}

#[test]
fn test_aabb_clip_lines_rays_segments() {
    for c in g::aabb_clip_line().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let b = bx(ref i);
        let origin = vec(ref i);
        near_segment(b.clip_line(origin, vec(ref i)), ref o, tol, id);
    }
    for c in g::aabb_clip_line_parameters().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let b = bx(ref i);
        let origin = vec(ref i);
        near_params(b.clip_line_parameters(origin, vec(ref i)), ref o, tol, id);
    }
    for c in g::aabb_clip_ray().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let b = bx(ref i);
        let origin = vec(ref i);
        near_segment(b.clip_ray(Ray { origin, dir: vec(ref i) }), ref o, tol, id);
    }
    for c in g::aabb_clip_ray_parameters().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let b = bx(ref i);
        let origin = vec(ref i);
        near_params(b.clip_ray_parameters(Ray { origin, dir: vec(ref i) }), ref o, tol, id);
    }
    for c in g::aabb_clip_segment().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let b = bx(ref i);
        let origin = vec(ref i);
        let dir = vec(ref i);
        near_segment(b.clip_segment(origin, origin + dir), ref o, tol, id);
    }
}

#[test]
fn test_clip_polygons() {
    for c in g::aabb_clip_polygon().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o, mut o_workspace) = (input, output, output);
        let b = bx(ref i);
        let poly = points(ref i);
        near_points(b.clip_polygon(poly.span()).span(), ref o, tol, id);
        let clipped = b.clip_polygon_with_workspace(poly.span(), array![]);
        near_points(clipped.span(), ref o_workspace, tol, id);
    }
    for c in g::clip_halfspace_polygon().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let center = vec(ref i);
        let normal = vec(ref i);
        let poly = points(ref i);
        near_points(clip_halfspace_polygon(center, normal, poly.span()).span(), ref o, tol, id);
    }
}
