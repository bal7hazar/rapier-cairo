//! Second half of the `geometry_utils` family: support-map projection, polygon mass, `Segment`
//! splits, scaled shapes, bounding sphere and polygonal-feature contacts (see the parent
//! module for the case layout).

use super::{add, Case, Io};
use crate::leaf::qv;
use crate::q::{jq, QPose, QRot};
use rapier2d_f64::math::Vector;
use rapier2d_f64::parry::bounding_volume::BoundingSphere;
use rapier2d_f64::parry::mass_properties::details::convex_polygon_area_and_center_of_mass;
use rapier2d_f64::parry::query::details::local_point_projection_on_support_map;
use rapier2d_f64::parry::query::gjk::VoronoiSimplex;
use rapier2d_f64::parry::query::{
    ContactManifold, PointProjection, PointQuery, Ray, RayCast, SplitResult,
};
use rapier2d_f64::parry::shape::{
    Ball, Capsule, ConvexPolygon, Cuboid, PackedFeatureId, PolygonalFeature, Segment,
};

// --- support map, mass, segment split, from_array --------------------------------------------

pub(super) fn misc_cases(cases: &mut Vec<Case>) {
    // `local_point_projection_on_support_map`: in `[kind, shape params, point, solid]`, out
    // `[is_inside, point]`. Kinds: 0 ball `[r]`, 1 cuboid `[hx, hy]`, 2 capsule `[a, b, r]`,
    // 3 segment `[a, b]`, 4 polygon `[n, points]`.
    let f = "support_map_projection";
    let poly = vec![(-1.0, -1.0), (1.0, -1.0), (1.5, 0.5), (0.0, 1.5), (-1.0, 1.0)];
    type P = (&'static str, i64, (f64, f64), bool, &'static str);
    let table: Vec<P> = vec![
        ("ball_out", 0, (3.0, 4.0), true, "ball radius 2, point outside"),
        ("ball_solid_in", 0, (0.5, 0.25), true, "solid point inside: itself"),
        ("ball_boundary", 0, (2.0, 0.0), true, "point on the boundary: inside"),
        ("cuboid_out", 1, (3.0, 2.0), true, "cuboid, point beyond a corner"),
        ("cuboid_side", 1, (0.5, 4.0), false, "cuboid, point above a side (hollow)"),
        ("cuboid_solid_in", 1, (0.5, -0.5), true, "solid point inside"),
        ("cuboid_hollow_in", 1, (0.75, 0.25), false, "hollow point inside: pushed to the nearest side"),
        ("capsule_out", 2, (0.0, 4.0), true, "capsule, point beyond a cap"),
        ("capsule_side", 2, (0.5, 3.0), false, "capsule, point over the flat side"),
        ("capsule_solid_in", 2, (0.5, 0.25), true, "solid point inside"),
        ("segment_out", 3, (0.5, 3.0), true, "segment, point above the middle"),
        ("segment_end", 3, (4.0, 3.0), true, "segment, closest point is an end point"),
        ("segment_on", 3, (0.5, 0.0), true, "point on the segment: inside"),
        ("polygon_out", 4, (3.0, 0.0), true, "polygon, point beyond a vertex region"),
        ("polygon_face", 4, (0.0, -3.0), false, "polygon, point below a face"),
        ("polygon_solid_in", 4, (0.0, 0.0), true, "solid point inside"),
        ("polygon_hollow_in", 4, (0.25, -0.5), false, "hollow point inside: pushed to the nearest face"),
    ];
    for (name, kind, pt, solid, note) in table {
        let tol = 4;
        add(cases, f, name, tol, note, |i, o| {
            i.i(kind);
            let mut simplex = VoronoiSimplex::new();
            let proj: PointProjection;
            match kind {
                0 => {
                    let r = i.q(2.0);
                    let p = i.v(pt.0, pt.1);
                    i.i(solid as i64);
                    proj = local_point_projection_on_support_map(&Ball::new(r), &mut simplex, p, solid);
                }
                1 => {
                    let h = i.v(1.0, 0.5);
                    let p = i.v(pt.0, pt.1);
                    i.i(solid as i64);
                    proj = local_point_projection_on_support_map(&Cuboid::new(h), &mut simplex, p, solid);
                }
                2 => {
                    let (a, b) = (i.v(-1.0, 0.0), i.v(1.0, 0.0));
                    let r = i.q(0.5);
                    let p = i.v(pt.0, pt.1);
                    i.i(solid as i64);
                    proj = local_point_projection_on_support_map(&Capsule::new(a, b, r), &mut simplex, p, solid);
                }
                3 => {
                    let (a, b) = (i.v(-1.0, 0.0), i.v(1.0, 0.0));
                    let p = i.v(pt.0, pt.1);
                    i.i(solid as i64);
                    proj = local_point_projection_on_support_map(&Segment::new(a, b), &mut simplex, p, solid);
                }
                _ => {
                    let points = i.pts_in(&poly);
                    let p = i.v(pt.0, pt.1);
                    i.i(solid as i64);
                    let polygon = ConvexPolygon::from_convex_polyline(points).unwrap();
                    proj = local_point_projection_on_support_map(&polygon, &mut simplex, p, solid);
                }
            }
            o.flag(proj.is_inside);
            o.vec(proj.point);
        });
    }

    // `convex_polygon_area_and_center_of_mass`: in `[n, points]`, out `[area, com]`.
    for (name, pts, note) in [
        ("square", vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)], "unit square: 1, (0.5, 0.5)"),
        ("triangle", vec![(0.0, 0.0), (3.0, 0.0), (0.0, 3.0)], "right triangle: 4.5, (1, 1)"),
        ("clockwise", vec![(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0)], "clockwise square: the area is positive"),
        ("pentagon", vec![(-1.0, -1.0), (1.0, -1.0), (1.5, 0.5), (0.0, 1.5), (-1.0, 1.0)], "off-centre pentagon"),
        ("collinear", vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)], "zero area: the geometric centre"),
        ("far", vec![(100.0, 100.0), (102.0, 100.0), (102.0, 101.0), (100.0, 101.0)], "far from the origin: 2, (101, 100.5)"),
    ] {
        add(cases, "convex_polygon_area_and_center_of_mass", name, 2, note, |i, o| {
            let p = i.pts_in(&pts);
            let (area, com) = convex_polygon_area_and_center_of_mass(&p);
            o.f(area);
            o.vec(com);
        });
    }

    // `Segment`: `from_array` in `[a, b]`, out `[a, b]`; the splits in `[a, b, axis, bias, eps]`
    // (`local_split*`: `axis` is a vector), out `[tag, first, second]` plus, for
    // `local_split_and_get_intersection`, `[has, point, bcoord]`.
    for (name, a, b, note) in [
        ("plain", (0.0, 0.0), (1.0, 2.0), "an oblique segment"),
        ("point", (0.5, 0.5), (0.5, 0.5), "a zero-length segment"),
        ("reversed", (3.0, -1.0), (-2.0, 4.0), "far end points"),
    ] {
        add(cases, "segment_from_array", name, 0, note, |i, o| {
            let (a, b) = (i.v(a.0, a.1), i.v(b.0, b.1));
            o.seg(Segment::from_array(&[a, b]).clone());
        });
    }
    type S = (&'static str, [f64; 4], (f64, f64), f64, f64, &'static str);
    let splits: Vec<S> = vec![
        ("pair_x", [0.0, 0.0, 4.0, 2.0], (1.0, 0.0), 1.0, 0.0, "plane x = 1 through the segment"),
        ("pair_y", [0.0, -2.0, 1.0, 2.0], (0.0, 1.0), 0.5, 0.0, "plane y = 0.5, the positive end second"),
        ("pair_rev", [4.0, 2.0, 0.0, 0.0], (1.0, 0.0), 1.0, 0.0, "the same segment reversed: sides swap"),
        ("negative", [0.0, 0.0, 4.0, 2.0], (1.0, 0.0), 5.0, 0.0, "plane beyond the segment: negative"),
        ("positive", [0.0, 0.0, 4.0, 2.0], (1.0, 0.0), -1.0, 0.0, "plane before the segment: positive"),
        ("parallel", [0.0, 1.0, 4.0, 1.0], (0.0, 1.0), 0.5, 0.0, "segment parallel to the plane: positive"),
        ("parallel_neg", [0.0, 1.0, 4.0, 1.0], (0.0, 1.0), 2.0, 0.0, "parallel, plane above: negative"),
        ("near_end", [0.0, 0.0, 4.0, 0.0], (1.0, 0.0), 0.125, 0.25, "intersection within epsilon of the start: no split"),
        ("near_far_end", [0.0, 0.0, 4.0, 0.0], (1.0, 0.0), 3.875, 0.25, "intersection within epsilon of the end: no split"),
        ("diagonal_axis", [0.0, 0.0, 4.0, 4.0], (1.0, 1.0), 4.0, 0.0, "non-unit axis (1, 1), bias 4: the midpoint"),
        ("zero_len", [1.0, 1.0, 1.0, 1.0], (1.0, 0.0), 1.0, 0.0, "a point on the plane"),
    ];
    for (name, s, ax, bias, eps, note) in &splits {
        let segment = |i: &mut Io| {
            let (a, b) = (i.v(s[0], s[1]), i.v(s[2], s[3]));
            Segment::new(a, b)
        };
        let out = |o: &mut Io, r: &SplitResult<Segment>| match r {
            SplitResult::Pair(a, b) => {
                o.i(0);
                o.seg(*a);
                o.seg(*b);
            }
            SplitResult::Negative => {
                o.i(1);
                (0..8).for_each(|_| o.f(0.0));
            }
            SplitResult::Positive => {
                o.i(2);
                (0..8).for_each(|_| o.f(0.0));
            }
        };
        if ax.0 == 1.0 && ax.1 == 0.0 || ax.0 == 0.0 && ax.1 == 1.0 {
            add(cases, "segment_canonical_split", name, 2, note, |i, o| {
                let sg = segment(i);
                let axis = if ax.0 == 1.0 { 0 } else { 1 };
                i.i(axis);
                let (bias, eps) = (i.q(*bias), i.q(*eps));
                out(o, &sg.canonical_split(axis as usize, bias, eps));
            });
        }
        add(cases, "segment_local_split", name, 2, note, |i, o| {
            let sg = segment(i);
            let axis = i.v(ax.0, ax.1);
            let (bias, eps) = (i.q(*bias), i.q(*eps));
            out(o, &sg.local_split(axis, bias, eps));
        });
        add(cases, "segment_local_split_and_get_intersection", name, 2, note, |i, o| {
            let sg = segment(i);
            let axis = i.v(ax.0, ax.1);
            let (bias, eps) = (i.q(*bias), i.q(*eps));
            let (r, hit) = sg.local_split_and_get_intersection(axis, bias, eps);
            out(o, &r);
            match hit {
                None => {
                    o.flag(false);
                    (0..3).for_each(|_| o.f(0.0));
                }
                Some((p, t)) => {
                    o.flag(true);
                    o.vec(p);
                    o.f(t);
                }
            }
        });
    }
}

// --- bounding sphere, scaled shapes ----------------------------------------------------------

/// `Ball::scaled`: in `[radius, scale, nsubdivs]`; `Capsule::scaled`: in `[a, b, radius, scale,
/// nsubdiv]`; `ConvexPolygon::scaled`: in `[n, points, scale]`. Out `[kind, radius, n, points,
/// normals]` with kind 0 none, 1 same shape (radius set), 2 polygon; `ConvexPolygon::scaled`:
/// `[some, n, points, normals]`. `deviation` cases are answered by the upstream polygon, that
/// does not fit the port's 8 vertices: the Cairo tests expect `None` there.
pub(super) fn shape_cases(cases: &mut Vec<Case>) {
    fn poly_out(o: &mut Io, p: &ConvexPolygon) {
        o.i(p.points().len() as i64);
        p.points().iter().for_each(|v| o.vec(*v));
        p.normals().iter().for_each(|v| o.vec(*v));
    }
    let tol = 16;
    for (name, r, s, n, note) in [
        ("uniform", 2.0, (1.5, 1.5), 8, "uniform scale: a ball of radius 3"),
        ("uniform_negative", 2.0, (-0.5, -0.5), 8, "negative uniform scale: radius 1"),
        ("square", 1.0, (2.0, 1.0), 4, "4 subdivisions, stretched in x"),
        ("hexagon", 1.0, (0.5, 2.0), 6, "6 subdivisions, stretched in y"),
        ("octagon", 2.0, (3.0, 1.0), 8, "8 subdivisions"),
        ("zero_scale", 1.0, (0.0, 1.0), 6, "a collapsed axis: no polygon"),
        ("deviation_mirrored", 1.0, (-1.0, 2.0), 6, "negative determinant: the outline is reversed, the port answers none"),
        ("deviation_12", 1.0, (2.0, 1.0), 12, "12 subdivisions: more vertices than the port holds"),
        ("deviation_2", 1.0, (2.0, 1.0), 2, "2 subdivisions: no polygon upstream either"),
    ] {
        add(cases, "ball_scaled", name, tol, note, |i, o| {
            let r = i.q(r);
            let sc = i.v(s.0, s.1);
            i.i(n);
            let deviation = name.starts_with("deviation");
            match Ball::new(r).scaled(sc, n as u32) {
                Some(rapier2d_f64::parry::either::Either::Left(b)) => {
                    o.i(1);
                    o.f(b.radius);
                }
                Some(rapier2d_f64::parry::either::Either::Right(p)) => {
                    o.i(if deviation { 3 } else { 2 });
                    o.f(0.0);
                    poly_out(o, &p);
                }
                None => {
                    o.i(0);
                    o.f(0.0);
                }
            }
        });
    }
    for (name, a, b, r, s, n, note) in [
        ("uniform", (-1.0, 0.0), (1.0, 0.0), 0.5, (2.0, 2.0), 4, "uniform scale: a capsule of twice the size"),
        ("stretch_x", (-1.0, 0.0), (1.0, 0.0), 0.5, (2.0, 1.0), 4, "8 outline points, stretched in x"),
        ("stretch_y", (0.0, -1.0), (0.0, 1.0), 0.5, (1.0, 2.0), 3, "6 outline points, vertical capsule"),
        ("oblique", (-1.0, -0.5), (1.0, 0.5), 0.5, (2.0, 1.0), 4, "oblique capsule: the outline is rotated"),
        ("small", (-1.0, 0.0), (1.0, 0.0), 0.5, (1.0, 3.0), 2, "4 outline points"),
        ("deviation_5", (-1.0, 0.0), (1.0, 0.0), 0.5, (2.0, 1.0), 5, "10 outline points: more vertices than the port holds"),
    ] {
        add(cases, "capsule_scaled", name, tol, note, |i, o| {
            let (a, b) = (i.v(a.0, a.1), i.v(b.0, b.1));
            let r = i.q(r);
            let sc = i.v(s.0, s.1);
            i.i(n);
            let deviation = name.starts_with("deviation");
            match Capsule::new(a, b, r).scaled(sc, n as u32) {
                Some(rapier2d_f64::parry::either::Either::Left(c)) => {
                    o.i(1);
                    o.f(c.radius);
                    o.i(0);
                    o.seg(c.segment);
                }
                Some(rapier2d_f64::parry::either::Either::Right(p)) => {
                    o.i(if deviation { 3 } else { 2 });
                    o.f(0.0);
                    poly_out(o, &p);
                }
                None => {
                    o.i(0);
                    o.f(0.0);
                }
            }
        });
    }
    for (name, pts, s, note) in [
        ("stretch", vec![(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)], (2.0, 0.5), "a square stretched in x"),
        ("uniform", vec![(0.0, 0.0), (2.0, 0.0), (0.0, 2.0)], (2.0, 2.0), "uniform scale"),
        ("shear_normals", vec![(0.0, 0.0), (2.0, 0.0), (0.0, 2.0)], (3.0, 1.0), "upstream normalises normal * scale, not the inverse transpose"),
        ("zero", vec![(0.0, 0.0), (2.0, 0.0), (0.0, 2.0)], (0.0, 1.0), "a normal collapses: none"),
    ] {
        add(cases, "convex_polygon_scaled", name, tol, note, |i, o| {
            let p = i.pts_in(&pts);
            let sc = i.v(s.0, s.1);
            let polygon = ConvexPolygon::from_convex_polyline(p).unwrap();
            match polygon.scaled(sc) {
                Some(p) => {
                    o.flag(true);
                    poly_out(o, &p);
                }
                None => {
                    o.flag(false);
                    o.i(0);
                }
            }
        });
    }

    // Bounding sphere: point queries, in `[center, radius, point, solid]`, out `[is_inside,
    // point, distance, contains]`; ray casts, in `[center, radius, origin, dir, max, solid]`,
    // out `[some_t, t, some_hit, t, normal, intersects]`.
    for (name, c, r, p, solid, note) in [
        ("outside", (1.0, 1.0), 2.0, (4.0, 5.0), true, "point outside, 5 from the centre"),
        ("inside_solid", (1.0, 1.0), 2.0, (1.5, 1.0), true, "solid point inside"),
        ("inside_hollow", (1.0, 1.0), 2.0, (1.5, 1.0), false, "hollow point inside: pushed to the surface"),
        ("boundary", (0.0, 0.0), 2.0, (0.0, -2.0), false, "point on the sphere"),
    ] {
        add(cases, "bounding_sphere_point", name, 2, note, |i, o| {
            let ctr = i.v(c.0, c.1);
            let rad = i.q(r);
            let pt = i.v(p.0, p.1);
            i.i(solid as i64);
            let s = BoundingSphere::new(ctr, rad);
            let proj = s.project_local_point(pt, solid);
            o.flag(proj.is_inside);
            o.vec(proj.point);
            o.f(s.distance_to_local_point(pt, solid));
            o.flag(s.contains_local_point(pt));
        });
    }
    for (name, c, r, org, dir, max, solid, note) in [
        ("hit", (1.0, 1.0), 1.0, (-3.0, 1.0), (1.0, 0.0), 8.0, true, "ray from the left along the diameter"),
        ("miss", (1.0, 1.0), 1.0, (-3.0, 3.0), (1.0, 0.0), 8.0, true, "ray passing above"),
        ("tangent", (0.0, 0.0), 1.0, (-3.0, 1.0), (1.0, 0.0), 8.0, true, "grazing ray"),
        ("inside_solid", (0.0, 0.0), 2.0, (0.5, 0.0), (1.0, 0.0), 8.0, true, "solid ray from inside: time 0"),
        ("inside_hollow", (0.0, 0.0), 2.0, (0.5, 0.0), (1.0, 0.0), 8.0, false, "hollow ray from inside: the exit"),
        ("too_far", (5.0, 0.0), 1.0, (0.0, 0.0), (1.0, 0.0), 2.0, true, "beyond the maximum time"),
    ] {
        add(cases, "bounding_sphere_ray", name, 4, note, |i, o| {
            let ctr = i.v(c.0, c.1);
            let rad = i.q(r);
            let (origin, d) = (i.v(org.0, org.1), i.v(dir.0, dir.1));
            let m = i.q(max);
            i.i(solid as i64);
            let s = BoundingSphere::new(ctr, rad);
            let ray = Ray::new(origin, d);
            match s.cast_local_ray(&ray, m, solid) {
                Some(t) => {
                    o.flag(true);
                    o.f(t);
                }
                None => {
                    o.flag(false);
                    o.f(0.0);
                }
            }
            match s.cast_local_ray_and_get_normal(&ray, m, solid) {
                Some(h) => {
                    o.flag(true);
                    o.f(h.time_of_impact);
                    o.vec(h.normal);
                }
                None => {
                    o.flag(false);
                    (0..3).for_each(|_| o.f(0.0));
                }
            }
            o.flag(s.intersects_local_ray(&ray, m));
        });
    }
}

// --- polygonal feature contacts --------------------------------------------------------------

/// In `[pos12 (translation, rotation), face1 (a, b), sep_axis1, face2 (a, b) or vertex2 (p, id),
/// flipped]`, out `[n, (p1, p2, fid1, fid2, dist) * n]` (two slots, zero padded). The faces are
/// `PolygonalFeature::from(Segment)`: vertex ids `0` and `2`, face id `1`.
pub(super) fn feature_cases(cases: &mut Vec<Case>) {
    fn pose(i: &mut Io, tr: (f64, f64), deg: f64) -> rapier2d_f64::math::Pose {
        let t = qv(tr.0, tr.1);
        let rot = QRot::from_degrees(deg);
        for q in [t.x, t.y, rot.re, rot.im] {
            i.vals.push(jq(q));
        }
        QPose::new(t, rot).p()
    }
    fn contacts_out(o: &mut Io, m: &ContactManifold<(), ()>) {
        o.i(m.points.len() as i64);
        for k in 0..2 {
            match m.points.get(k) {
                Some(c) => {
                    o.vec(c.local_p1);
                    o.vec(c.local_p2);
                    o.i(c.fid1.0 as i64);
                    o.i(c.fid2.0 as i64);
                    o.f(c.dist);
                }
                None => {
                    (0..4).for_each(|_| o.f(0.0));
                    o.i(0);
                    o.i(0);
                    o.f(0.0);
                }
            }
        }
    }
    type FF = (&'static str, ((f64, f64), f64), [f64; 4], (f64, f64), [f64; 4], bool, &'static str);
    let face_face: Vec<FF> = vec![
        ("stacked", ((0.0, 0.25), 0.0), [-1.0, 0.0, 1.0, 0.0], (0.0, 1.0), [-1.0, 0.0, 1.0, 0.0], false, "two parallel faces of equal extent, 0.25 apart"),
        ("offset", ((0.5, 0.25), 0.0), [-1.0, 0.0, 1.0, 0.0], (0.0, 1.0), [-1.0, 0.0, 1.0, 0.0], false, "the second face shifted by half a unit: clipped"),
        ("flipped", ((0.5, 0.25), 0.0), [-1.0, 0.0, 1.0, 0.0], (0.0, 1.0), [-1.0, 0.0, 1.0, 0.0], true, "the same with the points and ids swapped"),
        ("rotated", ((0.0, 0.5), 30.0), [-1.0, 0.0, 1.0, 0.0], (0.0, 1.0), [-0.5, 0.0, 0.5, 0.0], false, "the second face rotated by 30 degrees"),
        ("apart", ((3.0, 0.25), 0.0), [-1.0, 0.0, 1.0, 0.0], (0.0, 1.0), [-1.0, 0.0, 1.0, 0.0], false, "faces not overlapping along the axis: no contact"),
    ];
    for (name, (tr, deg), f1, n1, f2, flipped, note) in &face_face {
        add(cases, "polygonal_feature_face_face_contacts", name, 4, note, |i, o| {
            let p = pose(i, *tr, *deg);
            let (a1, b1) = (i.v(f1[0], f1[1]), i.v(f1[2], f1[3]));
            let n = i.v(n1.0, n1.1);
            let (a2, b2) = (i.v(f2[0], f2[1]), i.v(f2[2], f2[3]));
            i.i(*flipped as i64);
            let (s1, s2) = (Segment::new(a1, b1), Segment::new(a2, b2));
            let (face1, face2) = (PolygonalFeature::from(s1), PolygonalFeature::from(s2));
            let mut m = ContactManifold::<(), ()>::new();
            PolygonalFeature::face_face_contacts(&p, &face1, n, &face2, &mut m, *flipped);
            contacts_out(o, &m);
        });
    }
    type FV = (&'static str, ((f64, f64), f64), [f64; 4], (f64, f64), (f64, f64), i64, bool, &'static str);
    let face_vertex: Vec<FV> = vec![
        ("above", ((0.0, 0.0), 0.0), [-1.0, 0.0, 1.0, 0.0], (0.0, 1.0), (0.25, 0.5), 1, false, "a vertex 0.5 above a face"),
        ("flipped", ((0.0, 0.0), 0.0), [-1.0, 0.0, 1.0, 0.0], (0.0, 1.0), (0.25, 0.5), 1, true, "the same, flipped"),
        ("penetrating", ((0.0, 0.0), 0.0), [-1.0, 0.0, 1.0, 0.0], (0.0, 1.0), (-0.5, -0.25), 3, false, "a vertex below the face: negative distance"),
        ("posed", ((1.0, 1.0), 90.0), [0.0, -1.0, 0.0, 1.0], (1.0, 0.0), (0.5, 0.0), 0, false, "a posed vertex against a vertical face"),
    ];
    for (name, (tr, deg), f1, n1, v2, id, flipped, note) in &face_vertex {
        add(cases, "polygonal_feature_face_vertex_contacts", name, 4, note, |i, o| {
            let p = pose(i, *tr, *deg);
            let (a1, b1) = (i.v(f1[0], f1[1]), i.v(f1[2], f1[3]));
            let n = i.v(n1.0, n1.1);
            let v = i.v(v2.0, v2.1);
            i.i(*id);
            i.i(*flipped as i64);
            let face1 = PolygonalFeature::from(Segment::new(a1, b1));
            let vertex2 = PolygonalFeature {
                vertices: [v, Vector::ZERO],
                vids: [PackedFeatureId::vertex(*id as u32), PackedFeatureId::UNKNOWN],
                fid: PackedFeatureId::UNKNOWN,
                num_vertices: 1,
            };
            let mut m = ContactManifold::<(), ()>::new();
            PolygonalFeature::face_vertex_contacts(&p, &face1, n, &vertex2, &mut m, *flipped);
            contacts_out(o, &m);
        });
    }
}
