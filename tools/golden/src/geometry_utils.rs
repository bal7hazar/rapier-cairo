//! Family `geometry_utils` (PX3): the Parry geometry utilities that sit off the step path —
//! `Aabb` clipping / splitting / projection, `clip_halfspace_polygon`, line-line closest points,
//! the support-map point projection, the polygon area and centre of mass, `Segment` splits,
//! `Ball` / `Capsule` / `ConvexPolygon::scaled`, the bounding-sphere queries and the polygonal
//! feature contacts.
//!
//! A case is a flat record: `in` and `out` are lists of scalars, `{f64, raw}` for a real number
//! and `{int}` for a flag, a count, a side or a tag. Each function documents its layout below;
//! the Cairo test module of the same name reads them back in the same order. `tol` is the
//! comparison band in ulps of Q32.32 (`0` = the kernel is exact); integers are always compared
//! exactly.

use crate::q::{jf, jq, Q};
use rapier2d_f64::math::Vector;
use rapier2d_f64::parry::bounding_volume::Aabb;
use rapier2d_f64::parry::query::details::{
    clip_aabb_line, clip_halfspace_polygon, closest_points_line_line,
    closest_points_line_line_parameters, closest_points_line_line_parameters_eps,
};
use rapier2d_f64::parry::query::{Ray, SplitResult};
use rapier2d_f64::parry::shape::Segment;
use serde_json::{json, Value};

mod rest;
use rest::{feature_cases, misc_cases, shape_cases};

/// Builds the `in` and `out` lists of one case.
struct Io {
    vals: Vec<Value>,
}

impl Io {
    fn new() -> Io {
        Io { vals: Vec::new() }
    }
    /// Input real, snapped to Q32.32 (the exact `f64` is returned).
    fn q(&mut self, x: f64) -> f64 {
        let q = Q::snap(x);
        self.vals.push(jq(q));
        q.f()
    }
    fn v(&mut self, x: f64, y: f64) -> Vector {
        let (x, y) = (self.q(x), self.q(y));
        Vector::new(x, y)
    }
    fn i(&mut self, n: i64) -> i64 {
        self.vals.push(json!({ "int": n }));
        n
    }
    /// Output real.
    fn f(&mut self, x: f64) {
        self.vals.push(jf(x));
    }
    fn vec(&mut self, p: Vector) {
        self.f(p.x);
        self.f(p.y);
    }
    fn flag(&mut self, b: bool) {
        self.vals.push(json!({ "int": b as i64 }));
    }
    fn aabb(&mut self, a: Aabb) {
        self.vec(a.mins);
        self.vec(a.maxs);
    }
    fn seg(&mut self, s: Segment) {
        self.vec(s.a);
        self.vec(s.b);
    }
    /// Input box `[min, max]`.
    fn bx(&mut self, b: [f64; 4]) -> Aabb {
        let mins = self.v(b[0], b[1]);
        let maxs = self.v(b[2], b[3]);
        Aabb::new(mins, maxs)
    }
    fn pts_in(&mut self, pts: &[(f64, f64)]) -> Vec<Vector> {
        self.i(pts.len() as i64);
        pts.iter().map(|p| self.v(p.0, p.1)).collect()
    }
    fn pts_out(&mut self, pts: &[Vector]) {
        self.i(pts.len() as i64);
        for p in pts {
            self.vec(*p);
        }
    }
}

struct Case {
    id: String,
    func: &'static str,
    note: &'static str,
    tol: i64,
    inp: Vec<Value>,
    out: Vec<Value>,
}

fn add(
    cases: &mut Vec<Case>,
    func: &'static str,
    name: &str,
    tol: i64,
    note: &'static str,
    run: impl FnOnce(&mut Io, &mut Io),
) {
    if std::env::var_os("GOLDEN_TRACE").is_some() {
        eprintln!("{func}/{name}");
    }
    let (mut i, mut o) = (Io::new(), Io::new());
    run(&mut i, &mut o);
    cases.push(Case {
        id: format!("{func}/{name}"),
        func,
        note,
        tol,
        inp: i.vals,
        out: o.vals,
    });
}

const UNIT: [f64; 4] = [-1.0, -1.0, 1.0, 1.0];

/// `(name, box, origin, dir, note)` of the line / ray / segment clipping tables.
type Line = (&'static str, [f64; 4], (f64, f64), (f64, f64), &'static str);

fn lines() -> Vec<Line> {
    vec![
        ("cross_x", UNIT, (-3.0, 0.0), (1.0, 0.0), "horizontal line through the middle, from the left"),
        ("cross_x_rev", UNIT, (3.0, 0.0), (-2.0, 0.0), "the same from the right, dir of length 2"),
        ("cross_y", UNIT, (0.25, 4.0), (0.0, -1.0), "vertical line from above"),
        ("oblique", UNIT, (-3.0, -2.0), (2.0, 1.0), "oblique line through the box, entering by the left side"),
        ("inside", UNIT, (0.5, 0.25), (1.0, 0.5), "origin inside: entry before 0, exit through the right side"),
        ("corner_tie", UNIT, (-2.0, -2.0), (1.0, 1.0), "diagonal through two opposite corners: both times tie on both slabs"),
        ("miss", UNIT, (-3.0, 2.0), (1.0, 0.0), "parallel to an axis and outside its slab"),
        ("miss_oblique", UNIT, (-3.0, 3.0), (1.0, 0.0625), "oblique, passing above the box"),
        ("on_boundary", UNIT, (-3.0, 1.0), (1.0, 0.0), "parallel to an axis and exactly on the boundary: hit"),
        ("touch_corner", UNIT, (0.0, 2.0), (1.0, -1.0), "grazes the corner (1, 1): entry and exit tie"),
        ("zero_dir_inside", UNIT, (0.5, 0.5), (0.0, 0.0), "zero direction, origin inside: zero clip"),
        ("zero_dir_outside", UNIT, (2.0, 0.5), (0.0, 0.0), "zero direction, origin outside: miss"),
        ("zero_dir_on_edge", UNIT, (1.0, 0.0), (0.0, 0.0), "zero direction, origin on the boundary: hit"),
        ("behind", UNIT, (3.0, 0.25), (1.0, 0.0), "the box is behind the origin (a ray misses, a line hits)"),
        ("point_box", [0.5, 0.5, 0.5, 0.5], (0.0, 0.0), (1.0, 1.0), "degenerate box (a point) on the line"),
    ]
}

fn polygons() -> Vec<(&'static str, Vec<(f64, f64)>, &'static str)> {
    vec![
        ("inside", vec![(-0.5, -0.5), (0.5, -0.5), (0.0, 0.5)], "triangle inside the box: unchanged"),
        ("outside", vec![(2.0, 2.0), (3.0, 2.0), (2.5, 3.0)], "triangle outside: empty"),
        ("contains_box", vec![(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0)], "square around the box: the box"),
        ("cut_right", vec![(0.0, -0.5), (2.0, -0.5), (2.0, 0.5), (0.0, 0.5)], "rectangle cut by the right side"),
        ("cut_corner", vec![(0.5, 0.5), (2.5, 0.5), (0.5, 2.5)], "triangle cut by two sides at the corner"),
        ("on_edge", vec![(1.0, -0.5), (2.0, -0.5), (2.0, 0.5), (1.0, 0.5)], "rectangle touching the right side along an edge"),
        ("at_corner", vec![(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0)], "square touching the box at one corner"),
        ("empty", vec![], "no vertex"),
    ]
}

pub fn generate() -> Value {
    let mut cases: Vec<Case> = Vec::new();
    aabb_cases(&mut cases);
    clip_cases(&mut cases);
    line_line_cases(&mut cases);
    misc_cases(&mut cases);
    shape_cases(&mut cases);
    feature_cases(&mut cases);
    json!({
        "family": "geometry_utils",
        "cases": cases.iter().map(|c| json!({
            "id": c.id,
            "fn": c.func,
            "note": c.note,
            "tol": c.tol,
            "in": c.inp,
            "out": c.out,
        })).collect::<Vec<_>>(),
    })
}

// --- Aabb ------------------------------------------------------------------------------------

/// `distance_to_origin`: in `[box]`, out `[d]`. `project_on_axis`: in `[box, axis]`, out `[lo, hi]`.
/// `scaled_wrt_center`: in `[box, scale]`, out `[box]`. `canonical_split`: in `[box, axis, bias,
/// eps]`, out `[tag (0 pair, 1 negative, 2 positive), left, right]` (zeros unless a pair).
fn aabb_cases(cases: &mut Vec<Case>) {
    let f = "aabb_distance_to_origin";
    for (name, b, note) in [
        ("contains", UNIT, "box around the origin: 0"),
        ("pythagoras", [3.0, 4.0, 6.0, 8.0], "closest corner (3, 4): 5, exact"),
        ("lower_left", [-4.0, -3.0, -1.0, -2.0], "closest corner (-1, -2): sqrt 5"),
        ("touching", [0.0, 0.0, 1.0, 1.0], "corner on the origin: 0"),
        ("side", [2.0, -1.0, 5.0, 1.0], "closest point on a side: 2"),
        ("point", [0.0, 3.0, 0.0, 3.0], "degenerate box: 3"),
    ] {
        add(cases, f, name, 2, note, |i, o| {
            let a = i.bx(b);
            o.f(a.distance_to_origin());
        });
    }
    let f = "aabb_project_on_axis";
    for (name, b, ax, note) in [
        ("x", UNIT, (1.0, 0.0), "axis x: the x extent"),
        ("y_shifted", [0.0, 1.0, 2.0, 5.0], (0.0, 1.0), "axis y of an off-centre box"),
        ("diagonal", UNIT, (1.0, 1.0), "non-unit diagonal axis: centre 0, shift 2"),
        ("negative", [1.0, 1.0, 3.0, 2.0], (-1.0, -0.5), "axis with negative components"),
        ("zero_axis", UNIT, (0.0, 0.0), "zero axis: an empty interval at 0"),
    ] {
        add(cases, f, name, 2, note, |i, o| {
            let a = i.bx(b);
            let axis = i.v(ax.0, ax.1);
            let (lo, hi) = a.project_on_axis(axis);
            o.f(lo);
            o.f(hi);
        });
    }
    let f = "aabb_scaled_wrt_center";
    for (name, b, s, note) in [
        ("stretch", UNIT, (2.0, 0.5), "stretched in x, squeezed in y"),
        ("negative", [0.0, 0.0, 2.0, 4.0], (-1.0, 3.0), "a negative component scales the half extent by its absolute value"),
        ("zero", [0.0, 0.0, 2.0, 4.0], (0.0, 0.0), "zero scale: the centre point"),
        ("identity", [-1.0, 0.0, 3.0, 2.0], (1.0, 1.0), "unit scale"),
    ] {
        add(cases, f, name, 2, note, |i, o| {
            let a = i.bx(b);
            let scale = i.v(s.0, s.1);
            o.aabb(a.scaled_wrt_center(scale));
        });
    }
    let f = "aabb_canonical_split";
    for (name, axis, bias, eps, note) in [
        ("pair_x", 0, 0.25, 0.0, "plane through the box"),
        ("pair_y", 1, -0.5, 0.125, "plane through the box, along y"),
        ("above", 0, 2.0, 0.0, "plane above the box: negative"),
        ("below", 0, -2.0, 0.0, "plane below the box: positive"),
        ("touch_max", 1, 1.0, 0.0, "plane on the max side: negative"),
        ("touch_min", 1, -1.0, 0.0, "plane on the min side: positive"),
        ("eps_absorbs", 0, 0.9375, 0.0625, "plane within epsilon of the max side: negative"),
        ("eps_edge", 0, -0.9375, 0.0625, "plane exactly epsilon from the min side: positive"),
    ] {
        add(cases, f, name, 0, note, |i, o| {
            let a = i.bx(UNIT);
            i.i(axis);
            let (bias, eps) = (i.q(bias), i.q(eps));
            match a.canonical_split(axis as usize, bias, eps) {
                SplitResult::Pair(l, r) => {
                    o.i(0);
                    o.aabb(l);
                    o.aabb(r);
                }
                SplitResult::Negative => {
                    o.i(1);
                    (0..8).for_each(|_| o.f(0.0));
                }
                SplitResult::Positive => {
                    o.i(2);
                    (0..8).for_each(|_| o.f(0.0));
                }
            }
        });
    }
}

// --- clipping --------------------------------------------------------------------------------

/// `clip_aabb_line`: in `[box, origin, dir]`, out `[some, t0, n0, side0, t1, n1, side1]`.
/// `clip_line`, `clip_ray`, `clip_segment`: in `[box, origin, dir]` (the segment is `origin` to
/// `origin + dir`), out `[some, a, b]`. `clip_line_parameters`, `clip_ray_parameters`: out
/// `[some, t0, t1]`. `clip_polygon`: in `[box, n, points]`, out `[n, points]`.
/// `clip_halfspace_polygon`: in `[center, normal, n, points]`, out `[n, points]`.
fn clip_cases(cases: &mut Vec<Case>) {
    for (name, b, o_, d, note) in lines() {
        let tol = 2;
        add(cases, "clip_aabb_line", name, tol, note, |i, o| {
            let a = i.bx(b);
            let (origin, dir) = (i.v(o_.0, o_.1), i.v(d.0, d.1));
            match clip_aabb_line(&a, origin, dir) {
                None => {
                    o.flag(false);
                    o.f(0.0);
                    o.vec(Vector::ZERO);
                    o.i(0);
                    o.f(0.0);
                    o.vec(Vector::ZERO);
                    o.i(0);
                }
                Some((n, f)) => {
                    o.flag(true);
                    o.f(n.0);
                    o.vec(n.1);
                    o.i(n.2 as i64);
                    o.f(f.0);
                    o.vec(f.1);
                    o.i(f.2 as i64);
                }
            }
        });
        add(cases, "aabb_clip_line", name, tol, note, |i, o| {
            let a = i.bx(b);
            let (origin, dir) = (i.v(o_.0, o_.1), i.v(d.0, d.1));
            seg_out(o, a.clip_line(origin, dir));
        });
        add(cases, "aabb_clip_line_parameters", name, tol, note, |i, o| {
            let a = i.bx(b);
            let (origin, dir) = (i.v(o_.0, o_.1), i.v(d.0, d.1));
            params_out(o, a.clip_line_parameters(origin, dir));
        });
        add(cases, "aabb_clip_ray", name, tol, note, |i, o| {
            let a = i.bx(b);
            let (origin, dir) = (i.v(o_.0, o_.1), i.v(d.0, d.1));
            seg_out(o, a.clip_ray(&Ray::new(origin, dir)));
        });
        add(cases, "aabb_clip_ray_parameters", name, tol, note, |i, o| {
            let a = i.bx(b);
            let (origin, dir) = (i.v(o_.0, o_.1), i.v(d.0, d.1));
            params_out(o, a.clip_ray_parameters(&Ray::new(origin, dir)));
        });
        add(cases, "aabb_clip_segment", name, tol, note, |i, o| {
            let a = i.bx(b);
            let (origin, dir) = (i.v(o_.0, o_.1), i.v(d.0, d.1));
            seg_out(o, a.clip_segment(origin, origin + dir));
        });
    }
    for (name, pts, note) in polygons() {
        add(cases, "aabb_clip_polygon", name, 2, note, |i, o| {
            let a = i.bx(UNIT);
            let mut poly = i.pts_in(&pts);
            let mut workspace = Vec::new();
            let mut same = poly.clone();
            a.clip_polygon_with_workspace(&mut same, &mut workspace);
            a.clip_polygon(&mut poly);
            assert_eq!(poly, same, "the workspace variant must agree");
            o.pts_out(&poly);
        });
    }
    let cut = |name: &'static str, center: (f64, f64), normal: (f64, f64), pts: Vec<(f64, f64)>, note: &'static str, cases: &mut Vec<Case>| {
        add(cases, "clip_halfspace_polygon", name, 2, note, |i, o| {
            let c = i.v(center.0, center.1);
            let n = i.v(normal.0, normal.1);
            let poly = i.pts_in(&pts);
            let mut result = Vec::new();
            clip_halfspace_polygon(c, n, &poly, &mut result);
            o.pts_out(&result);
        });
    };
    let square = vec![(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)];
    cut("half", (1.0, 0.0), (1.0, 0.0), square.clone(), "square cut through the middle: left half", cases);
    cut("keep_all", (3.0, 0.0), (1.0, 0.0), square.clone(), "plane beyond the polygon: unchanged", cases);
    cut("drop_all", (-1.0, 0.0), (1.0, 0.0), square.clone(), "plane before the polygon: empty", cases);
    cut("on_edge", (2.0, 0.0), (1.0, 0.0), square.clone(), "plane on an edge: unchanged (boundary is kept)", cases);
    cut("diagonal", (1.0, 1.0), (1.0, 1.0), square.clone(), "plane through two corners: a triangle", cases);
    cut("triangle_corner", (0.5, 0.5), (-0.5, -0.5), vec![(0.0, 0.0), (2.0, 0.0), (0.0, 2.0)], "non-unit normal, cutting a corner off a triangle", cases);
    cut("empty", (0.0, 0.0), (1.0, 0.0), vec![], "empty polygon", cases);
}

fn seg_out(o: &mut Io, s: Option<Segment>) {
    match s {
        None => {
            o.flag(false);
            (0..4).for_each(|_| o.f(0.0));
        }
        Some(s) => {
            o.flag(true);
            o.seg(s);
        }
    }
}

fn params_out(o: &mut Io, p: Option<(f64, f64)>) {
    match p {
        None => {
            o.flag(false);
            o.f(0.0);
            o.f(0.0);
        }
        Some((a, b)) => {
            o.flag(true);
            o.f(a);
            o.f(b);
        }
    }
}

// --- line-line -------------------------------------------------------------------------------

/// In `[origin1, dir1, origin2, dir2]` (plus `eps` for `_eps`); out `[s, t]` (plus `parallel`),
/// `[p1, p2]` for `closest_points_line_line`.
fn line_line_cases(cases: &mut Vec<Case>) {
    type L = (&'static str, [f64; 8], &'static str);
    let table: Vec<L> = vec![
        ("crossing", [0.0, 0.0, 1.0, 0.0, 0.5, -1.0, 0.0, 1.0], "perpendicular lines crossing at (0.5, 0)"),
        ("oblique", [0.0, 0.0, 2.0, 1.0, 1.0, 3.0, -1.0, 1.0], "oblique lines"),
        ("parallel", [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 2.0, 0.0], "parallel lines, offset: s = 0, t the projection"),
        ("antiparallel", [0.0, 0.0, 1.0, 0.0, 3.0, 2.0, -1.0, 0.0], "opposite directions"),
        ("collinear", [0.0, 0.0, 1.0, 0.0, 2.0, 0.0, 1.0, 0.0], "the same line"),
        ("first_point", [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0], "first direction zero"),
        ("second_point", [0.0, 0.0, 2.0, 0.0, 1.0, 1.0, 0.0, 0.0], "second direction zero"),
        ("both_points", [1.0, 1.0, 0.0, 0.0, 2.0, 3.0, 0.0, 0.0], "both directions zero"),
        ("nearly_parallel", [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0009765625], "slope 2^-10: above the parallel threshold"),
        ("far", [-8.0, 0.5, 4.0, 0.0, 3.0, -6.0, 0.0, 2.0], "origins far from the crossing point"),
    ];
    for (name, v, note) in &table {
        let read = |i: &mut Io| {
            (i.v(v[0], v[1]), i.v(v[2], v[3]), i.v(v[4], v[5]), i.v(v[6], v[7]))
        };
        add(cases, "closest_points_line_line_parameters", name, 4, note, |i, o| {
            let (o1, d1, o2, d2) = read(i);
            let (s, t) = closest_points_line_line_parameters(o1, d1, o2, d2);
            o.f(s);
            o.f(t);
        });
        add(cases, "closest_points_line_line", name, 4, note, |i, o| {
            let (o1, d1, o2, d2) = read(i);
            let (p1, p2) = closest_points_line_line(o1, d1, o2, d2);
            o.vec(p1);
            o.vec(p2);
        });
        add(cases, "closest_points_line_line_parameters_eps", name, 4, note, |i, o| {
            let (o1, d1, o2, d2) = read(i);
            // 2^-23, the tolerance the port defaults to.
            let eps = i.q(1.0 / 8_388_608.0);
            let (s, t, parallel) = closest_points_line_line_parameters_eps(o1, d1, o2, d2, eps);
            o.f(s);
            o.f(t);
            o.flag(parallel);
        });
    }
}
