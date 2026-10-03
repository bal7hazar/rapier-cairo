//! Work package SH2b: the compound shape. Three families, in files of their own so that no
//! existing table grows:
//!
//! * `compound_contacts`: `DefaultQueryDispatcher::contact_manifolds` of a compound against the
//!   convex shapes and the half-space (both orders), and against the SH2a polylines and
//!   heightfields and another compound (both orders): one manifold per part pair; every manifold
//!   with points is recorded with both sub-shape ids (upstream's order is its BVH's: compare by
//!   ids). Each case also records `query::intersection_test` and `query::distance` (`None` when
//!   unsupported).
//! * `compound_queries`: point projections, world-space ray casts, the shape-pair queries
//!   (`intersection_test`, `distance`, `contact`, `closest_points`, `cast_shapes`), and the AABBs,
//!   bounding spheres and mass properties (sum of the parts') of the compounds.
//! * `compound_scenes`: an L-shaped compound toppling on a half-space, lying across a polyline
//!   vertex, and a three-part compound sliding over a heightfield (rapier2d-f64, no CCD, no contact
//!   recycling or clustering, as the other scene families), with every collision event.
//!
//! The compounds are given by Q32.32 inputs (part poses and shapes); both engines build them from
//! the same raws.

use crate::manifolds::fid_json;
use crate::point_projection::{feature_json, proj_json};
use crate::q::{jf, jq, jqpose, jqvec, jvec, QPose, QRot, QVec, Q};
use crate::ray_casts::hit_json;
use crate::sh1::Sh1Shape;
use crate::sh2a::{b, closest_json, composite, index_of, inverse, json_opt, qv, status_index, Collector, Composite};
use crate::shapes::ShapeSpec;
use rapier2d_f64::math::{Pose, Vector};
use rapier2d_f64::parry::query::{
    self, ContactManifold, DefaultQueryDispatcher, PersistentQueryDispatcher, Ray, ShapeCastOptions,
};
use rapier2d_f64::parry::shape::SharedShape;
use rapier2d_f64::prelude::*;
use serde_json::{json, Value};

/// A compound of the SH2b families: parts placed by their poses.
#[derive(Clone, Debug)]
pub struct CompoundSpec {
    pub parts: Vec<(QPose, Sh1Shape)>,
}

impl CompoundSpec {
    pub fn shared(&self) -> SharedShape {
        SharedShape::compound(self.parts.iter().map(|(p, s)| (p.p(), s.shared())).collect())
    }

    pub fn json(&self) -> Value {
        json!({
            "type": "compound",
            "parts": self.parts.iter().map(|(p, s)| json!({ "pose": jqpose(*p), "shape": s.json() })).collect::<Vec<_>>(),
        })
    }
}

fn at(x: f64, y: f64) -> QPose {
    QPose::new(qv(x, y), QRot::IDENTITY)
}

fn at_deg(x: f64, y: f64, deg: f64) -> QPose {
    QPose::new(qv(x, y), QRot::from_degrees(deg))
}

/// The compounds, by name.
pub fn compounds() -> Vec<(&'static str, CompoundSpec)> {
    vec![
        (
            "ell",
            CompoundSpec {
                parts: vec![
                    (at(0.0, 0.0), b(ShapeSpec::cuboid(1.0, 0.25))),
                    (at(-0.75, 1.0), b(ShapeSpec::cuboid(0.25, 0.75))),
                ],
            },
        ),
        (
            "trio",
            CompoundSpec {
                parts: vec![
                    (at(-0.8, 0.0), b(ShapeSpec::ball(0.3))),
                    (at_deg(0.0, 0.0, 20.0), b(ShapeSpec::cuboid(0.4, 0.3))),
                    (at(0.8, 0.1), b(ShapeSpec::capsule_y(0.3, 0.2))),
                ],
            },
        ),
        (
            "mixed",
            CompoundSpec {
                parts: vec![
                    (at(-0.6, 0.0), Sh1Shape::triangle((-0.4, -0.3), (0.4, -0.3), (0.0, 0.4))),
                    (at_deg(0.6, 0.1, 15.0), b(ShapeSpec::cuboid(0.4, 0.25)).rounded(0.1)),
                    (at(0.0, -0.5), b(ShapeSpec::polygon(&[(-0.5, -0.2), (0.5, -0.2), (0.6, 0.1), (-0.6, 0.1)]))),
                    (at(0.0, 0.6), b(ShapeSpec::segment((-0.3, 0.0), (0.3, 0.0)))),
                ],
            },
        ),
    ]
}

fn compound(name: &str) -> CompoundSpec {
    compounds().into_iter().find(|(n, _)| *n == name).unwrap().1
}

fn compound_index(name: &str) -> usize {
    compounds().iter().position(|(n, _)| *n == name).unwrap()
}

/// The convex shapes of the families.
fn convexes() -> Vec<(&'static str, Sh1Shape)> {
    vec![
        ("ball", b(ShapeSpec::ball(0.4))),
        ("cuboid", b(ShapeSpec::cuboid(0.5, 0.3))),
        ("capsule", b(ShapeSpec::capsule_y(0.3, 0.2))),
        ("segment", b(ShapeSpec::segment((-0.4, 0.0), (0.4, 0.0)))),
        ("poly", b(ShapeSpec::polygon(&[(-0.5, -0.4), (0.5, -0.4), (0.6, 0.1), (0.0, 0.5), (-0.6, 0.1)]))),
        ("tri", Sh1Shape::triangle((-0.4, -0.3), (0.4, -0.3), (0.0, 0.4))),
        ("rcub", b(ShapeSpec::cuboid(0.4, 0.25)).rounded(0.1)),
    ]
}

/// The height `y` (snapped) at which `s2` at `(x, y)` turned by `rot` sits `gap` above `s1` at the
/// identity: the touching height found by bisection on upstream's `distance`, plus `gap`.
fn height_above(s1: &SharedShape, s2: &SharedShape, x: f64, rot: QRot, gap: f64) -> f64 {
    let (mut lo, mut hi) = (-20.0f64, 20.0f64);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        let p2 = Pose::from_parts(Vector::new(x, mid), rot.r());
        let d = query::distance(&Pose::IDENTITY, &*s1.0, &p2, &*s2.0).map(|d| d.distance).unwrap();
        if d > 0.0 {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    Q::snap(hi + gap).f()
}

// ---------------------------------------------------------------------------------------------
// Manifolds
// ---------------------------------------------------------------------------------------------

/// `(regime, gap, degrees)`: shape 2 above shape 1.
const REGIMES: [(&str, f64, f64); 3] =
    [("separated", 0.5, 0.0), ("within_pred", 0.01, 0.0), ("overlapping", -0.05, 10.0)];

/// `x` over which the convex shapes are placed on each compound (over a part seam when possible).
fn placement(name: &str) -> f64 {
    match name {
        "ell" => -0.45,
        "trio" => -0.45,
        _ => 0.3,
    }
}

/// The other shape of a case: convex (its shape), an SH2a composite or a compound (their index).
#[derive(Clone)]
enum Other {
    Convex(Sh1Shape),
    Composite(&'static str),
    Compound(&'static str),
}

impl Other {
    fn shared(&self) -> SharedShape {
        match self {
            Other::Convex(s) => s.shared(),
            Other::Composite(n) => composite(n).shared(),
            Other::Compound(n) => compound(n).shared(),
        }
    }

    fn json(&self) -> Value {
        match self {
            Other::Convex(s) => json!({ "kind": "convex", "shape": s.json() }),
            Other::Composite(n) => json!({ "kind": "composite", "index": index_of(n) }),
            Other::Compound(n) => json!({ "kind": "compound", "index": compound_index(n) }),
        }
    }
}

fn manifolds_json(manifolds: &[ContactManifold<(), ()>]) -> Vec<Value> {
    let mut parts: Vec<Value> = manifolds
        .iter()
        .filter(|m| !m.points.is_empty())
        .map(|m| {
            json!({
                "subshape1": m.subshape1,
                "subshape2": m.subshape2,
                "local_n1": jvec(m.local_n1),
                "local_n2": jvec(m.local_n2),
                "num_points": m.points.len(),
                "points": m.points.iter().map(|pt| json!({
                    "local_p1": jvec(pt.local_p1),
                    "local_p2": jvec(pt.local_p2),
                    "dist": jf(pt.dist),
                    "fid1": fid_json(pt.fid1.0),
                    "fid2": fid_json(pt.fid2.0),
                })).collect::<Vec<_>>(),
            })
        })
        .collect();
    parts.sort_by_key(|m| (m["subshape1"].as_u64().unwrap(), m["subshape2"].as_u64().unwrap()));
    assert!(parts.len() <= 4, "at most four touching part pairs per case");
    parts
}

/// One case: `pos` places the other shape in the compound's frame; `first` puts the compound
/// first (`pos12 = pos`), otherwise second (`pos12 = pos⁻¹`).
fn manifold_case(id: String, comp_name: &str, other: &Other, pos: QPose, first: bool, prediction: Q) -> Value {
    let pos12 = if first { pos } else { inverse(pos) };
    let (s_comp, s_other) = (compound(comp_name).shared(), other.shared());
    let (s1, s2) = if first { (&s_comp, &s_other) } else { (&s_other, &s_comp) };
    let mut manifolds: Vec<ContactManifold<(), ()>> = Vec::new();
    let mut workspace = None;
    let p = pos12.p();
    DefaultQueryDispatcher
        .contact_manifolds(&p, &*s1.0, &*s2.0, prediction.f(), &mut manifolds, &mut workspace)
        .expect("unsupported compound pair");
    let identity = Pose::IDENTITY;
    json!({
        "id": id,
        "compound": compound_index(comp_name),
        "compound_first": first,
        "other": other.json(),
        "pos12": jqpose(pos12),
        "expected": {
            "manifolds": manifolds_json(&manifolds),
            "intersects": json_opt(query::intersection_test(&identity, &*s1.0, &p, &*s2.0).map(|i| i.intersecting), |v| json!(v)),
            "distance": distance_json(query::distance(&identity, &*s1.0, &p, &*s2.0).map(|d| d.distance)),
        },
    })
}

/// `json_opt` of a distance; upstream's `Real::MAX` (no supported part) is `"infinite": true`.
fn distance_json(r: Result<f64, query::Unsupported>) -> Value {
    match r {
        Ok(v) if v == f64::MAX => json!({ "supported": true, "infinite": true, "value": Value::Null }),
        other => json_opt(other, jf),
    }
}

pub fn compound_contacts() -> Value {
    let prediction = Q::snap(IntegrationParameters::default().prediction_distance());
    let mut cases = Vec::new();
    for (comp_name, comp) in compounds() {
        let sc = comp.shared();
        let x = placement(comp_name);
        for (shape_name, shape) in convexes() {
            for (regime, gap, deg) in REGIMES {
                let rot = if deg == 0.0 { QRot::IDENTITY } else { QRot::from_degrees(deg) };
                let y = height_above(&sc, &shape.shared(), x, rot, gap);
                let pos = QPose::new(qv(x, y), rot);
                let other = Other::Convex(shape);
                let id = format!("{comp_name}_{shape_name}/{regime}");
                cases.push(manifold_case(id, comp_name, &other, pos, true, prediction));
                if regime == "overlapping" && (shape_name == "ball" || shape_name == "cuboid") {
                    let id = format!("{shape_name}_{comp_name}/{regime}");
                    cases.push(manifold_case(id, comp_name, &other, pos, false, prediction));
                }
            }
        }
        // The half-space below the compound (upward normal): resting, within the prediction.
        for (regime, gap) in [("within_pred", 0.01), ("overlapping", -0.05)] {
            let aabb = sc.compute_local_aabb();
            let pos = at(0.0, Q::snap(aabb.mins.y - gap).f());
            let other = Other::Convex(b(ShapeSpec::halfspace_up()));
            let id = format!("{comp_name}_halfspace/{regime}");
            cases.push(manifold_case(id, comp_name, &other, pos, true, prediction));
            let id = format!("halfspace_{comp_name}/{regime}");
            cases.push(manifold_case(id, comp_name, &other, pos, false, prediction));
        }
    }
    // Compound against the composites: the compound resting over (or into) the composite's
    // surface (`gap`), both orders; `pos` places the composite in the compound's frame.
    let composites: [(&str, &str, f64, f64); 4] =
        [("ell", "bumps", -2.0, -0.03), ("trio", "vee", 0.3, -0.12), ("ell", "hills", 1.4, -0.03), ("mixed", "flat", 0.0, 0.01)];
    for (comp_name, ground, x, gap) in composites {
        let sc = compound(comp_name).shared();
        let local = sc.compute_local_aabb();
        let g = composite(ground);
        let y = Q::snap(g.top_over(x + local.mins.x, x + local.maxs.x) - local.mins.y + gap).f();
        let comp_in_ground = at(x, y);
        let pos = inverse(comp_in_ground);
        let other = Other::Composite(ground);
        cases.push(manifold_case(format!("{comp_name}_{ground}/rest"), comp_name, &other, pos, true, prediction));
        cases.push(manifold_case(format!("{ground}_{comp_name}/rest"), comp_name, &other, pos, false, prediction));
    }
    // Compound against compound: `trio` over `ell`, `ell` over `mixed`.
    for (comp_name, other_name, x, deg) in [("ell", "trio", 0.2, 0.0), ("mixed", "ell", -0.3, 15.0)] {
        let rot = if deg == 0.0 { QRot::IDENTITY } else { QRot::from_degrees(deg) };
        let y = height_above(&compound(comp_name).shared(), &compound(other_name).shared(), x, rot, -0.04);
        let pos = QPose::new(qv(x, y), rot);
        let other = Other::Compound(other_name);
        cases.push(manifold_case(format!("{comp_name}_{other_name}/overlapping"), comp_name, &other, pos, true, prediction));
        cases.push(manifold_case(format!("{other_name}_{comp_name}/overlapping"), comp_name, &other, pos, false, prediction));
    }
    json!({
        "family": "compound_contacts",
        "prediction": jq(prediction),
        "compounds": compounds().iter().map(|(n, c)| json!({ "name": n, "shape": c.json() })).collect::<Vec<_>>(),
        "cases": cases,
    })
}

// ---------------------------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------------------------

fn points() -> Vec<Value> {
    let pts = [(0.3, 2.0), (-0.75, 1.2), (0.0, 0.0), (2.5, 0.2), (-0.45, 0.2), (0.25, -1.5)];
    let mut out = Vec::new();
    for (name, comp) in compounds() {
        let s = comp.shared();
        for (k, (x, y)) in pts.iter().enumerate() {
            let p = qv(*x, *y);
            let pv: Vector = p.v();
            let (proj, feature) = s.project_local_point_and_get_feature(pv);
            out.push(json!({
                "id": format!("{name}/p{k}"),
                "composite": compound_index(name),
                "point": jqvec(p),
                "expected": {
                    "projection": proj_json(&s.project_local_point(pv, false)),
                    "projection_solid": proj_json(&s.project_local_point(pv, true)),
                    "feature_projection": proj_json(&proj),
                    "feature": feature_json(feature),
                    "distance": jf(s.distance_to_local_point(pv, false)),
                    "contains": s.contains_local_point(pv),
                },
            }));
        }
    }
    out
}

fn rays() -> Vec<Value> {
    let id = QPose::new(QVec::ZERO, QRot::IDENTITY);
    let posed = QPose::new(qv(0.5, -0.25), QRot::from_degrees(30.0));
    let rs = [
        ("down", id, (-0.7, 3.0), (0.0, -1.0)),
        ("slant", id, (-3.0, 2.0), (1.0, -0.6)),
        ("inside", id, (0.1, 0.05), (1.0, 0.2)),
        ("miss", id, (-6.0, 5.0), (1.0, 0.0)),
        ("posed", posed, (1.0, 3.0), (-0.2, -1.0)),
    ];
    let max = Q::snap(100.0);
    let mut out = Vec::new();
    for (name, comp) in compounds() {
        let s = comp.shared();
        for (ray_name, pose, origin, dir) in rs {
            let (o, d) = (qv(origin.0, origin.1), qv(dir.0, dir.1));
            let ray = Ray::new(o.v(), d.v());
            let p = pose.p();
            let answer = |solid: bool| json!({
                "toi": s.cast_ray(&p, &ray, max.f(), solid).map(jf).unwrap_or(Value::Null),
                "hit": hit_json(s.cast_ray_and_get_normal(&p, &ray, max.f(), solid)),
            });
            out.push(json!({
                "id": format!("{name}/{ray_name}"),
                "composite": compound_index(name),
                "pose": jqpose(pose),
                "origin": jqvec(o),
                "dir": jqvec(d),
                "max_toi": jq(max),
                "expected": { "solid": answer(true), "hollow": answer(false) },
            }));
        }
    }
    out
}

fn pairs() -> Vec<Value> {
    let prediction = Q::snap(0.1);
    let margin = Q::snap(1.0);
    let others = [
        ("ball", b(ShapeSpec::ball(0.4))),
        ("cuboid", b(ShapeSpec::cuboid(0.5, 0.3))),
        ("segment", b(ShapeSpec::segment((-0.4, 0.0), (0.4, 0.0)))),
    ];
    let mut out = Vec::new();
    for (comp_name, comp) in compounds() {
        let sc = comp.shared();
        for (other_name, other) in others {
            for (gap_name, gap) in [("hover", 0.3), ("deep", -0.05)] {
                for first in [true, false] {
                    let x = placement(comp_name);
                    let y = height_above(&sc, &other.shared(), x, QRot::IDENTITY, gap);
                    // Compound at `pos_comp`, the other shape at `pos_comp * (x, y)`: pure
                    // translations, exact in Q32.32.
                    let pos_comp = at(1.0, -2.0);
                    let world_other = at(1.0 + x, -2.0 + y);
                    let so = other.shared();
                    let (pos1, s1, pos2, s2) = if first {
                        (pos_comp, &sc, world_other, &so)
                    } else {
                        (world_other, &so, pos_comp, &sc)
                    };
                    let (p1, p2) = (pos1.p(), pos2.p());
                    let vel = Vector::new(0.3, -1.0);
                    let (v1, v2) = if first { (Vector::ZERO, vel) } else { (vel, Vector::ZERO) };
                    let options = ShapeCastOptions::with_max_time_of_impact(10.0);
                    let cast = query::cast_shapes(&p1, v1, &*s1.0, &p2, v2, &*s2.0, options);
                    let order = if first { format!("{comp_name}_{other_name}") } else { format!("{other_name}_{comp_name}") };
                    out.push(json!({
                        "id": format!("{order}/{gap_name}"),
                        "composite": compound_index(comp_name),
                        "composite_first": first,
                        "other": other.json(),
                        "pos1": jqpose(pos1),
                        "pos2": jqpose(pos2),
                        "vel": jvec(vel),
                        "prediction": jq(prediction),
                        "margin": jq(margin),
                        "expected": {
                            "intersects": json_opt(query::intersection_test(&p1, &*s1.0, &p2, &*s2.0).map(|i| i.intersecting), |v| json!(v)),
                            "distance": distance_json(query::distance(&p1, &*s1.0, &p2, &*s2.0).map(|d| d.distance)),
                            "contact": json_opt(query::contact(&p1, &*s1.0, &p2, &*s2.0, prediction.f()), |c| match c {
                                Some(c) => json!({ "some": true, "point1": jvec(c.point1), "point2": jvec(c.point2),
                                    "normal1": jvec(c.normal1), "normal2": jvec(c.normal2), "dist": jf(c.dist) }),
                                None => json!({ "some": false }),
                            }),
                            "closest_points": json_opt(query::closest_points(&p1, &*s1.0, &p2, &*s2.0, margin.f()), closest_json),
                            "cast": json_opt(cast, |h| match h {
                                Some(h) => json!({ "some": true, "toi": jf(h.time_of_impact), "witness1": jvec(h.witness1),
                                    "witness2": jvec(h.witness2), "normal1": jvec(h.normal1), "normal2": jvec(h.normal2),
                                    "status": status_index(h.status) }),
                                None => json!({ "some": false }),
                            }),
                        },
                    }));
                }
            }
        }
    }
    out
}

fn mass() -> Vec<Value> {
    let poses = [("identity", at(0.0, 0.0)), ("rot30", at_deg(1.0, 2.0, 30.0))];
    let mut out = Vec::new();
    for (name, comp) in compounds() {
        for (pose_name, pose) in poses {
            for density in [Q::ONE, Q::snap(2.5)] {
                let s = comp.shared();
                let aabb = s.compute_aabb(&pose.p());
                let sphere = s.compute_bounding_sphere(&pose.p());
                let props = s.mass_properties(density.f());
                out.push(json!({
                    "id": format!("{name}/{pose_name}/d{}", if density == Q::ONE { "1" } else { "2_5" }),
                    "compound": compound_index(name),
                    "pose": jqpose(pose),
                    "density": jq(density),
                    "expected": {
                        "mins": jvec(aabb.mins), "maxs": jvec(aabb.maxs),
                        "center": jvec(sphere.center), "radius": jf(sphere.radius),
                        "mass": jf(props.mass()), "local_com": jvec(props.local_com),
                        "inertia": jf(props.principal_inertia()),
                    },
                }));
            }
        }
    }
    out
}

pub fn compound_queries() -> Value {
    json!({
        "family": "compound_queries",
        "compounds": compounds().iter().map(|(n, c)| json!({ "name": n, "shape": c.json() })).collect::<Vec<_>>(),
        "points": points(),
        "rays": rays(),
        "pairs": pairs(),
        "mass": mass(),
    })
}

// ---------------------------------------------------------------------------------------------
// Scenes
// ---------------------------------------------------------------------------------------------

/// The ground of a scene: the upward half-space or an SH2a-style composite.
fn scene_ground(name: &str) -> (Value, SharedShape) {
    match name {
        "halfspace" => {
            let h = b(ShapeSpec::halfspace_up());
            (json!({ "kind": "convex", "shape": h.json() }), h.shared())
        }
        "seam" => {
            let c = Composite::Polyline {
                vertices: vec![qv(-3.0, 0.0), qv(0.0, 0.0), qv(3.0, 0.0)],
                indices: None,
                oriented: false,
            };
            (json!({ "kind": "composite", "shape": c.json() }), c.shared())
        }
        "hills2" => {
            let c = Composite::HeightField {
                heights: [0.0, 0.3, 0.1, 0.5, 0.2, 0.0, 0.4, 0.1].iter().map(|h| Q::snap(*h)).collect(),
                scale: qv(14.0, 1.0),
                removed: vec![],
            };
            (json!({ "kind": "composite", "shape": c.json() }), c.shared())
        }
        _ => unreachable!(),
    }
}

/// `(id, ground, compound, start, linvel, steps)`: the ground is a standalone collider (index 0,
/// `COLLISION_EVENTS`), the dynamic body carries the compound (index 1).
fn scene_configs() -> Vec<(&'static str, &'static str, CompoundSpec, (f64, f64), (f64, f64), usize)> {
    let tall_ell = CompoundSpec {
        parts: vec![
            (at(0.0, 0.0), b(ShapeSpec::cuboid(0.15, 0.8))),
            (at(0.55, 0.65), b(ShapeSpec::cuboid(0.4, 0.15))),
        ],
    };
    let flat_ell = CompoundSpec {
        parts: vec![
            (at(0.0, 0.0), b(ShapeSpec::cuboid(1.0, 0.2))),
            (at(-0.8, 0.4), b(ShapeSpec::cuboid(0.2, 0.2))),
        ],
    };
    vec![
        ("ell_topple", "halfspace", tall_ell, (0.0, 0.85), (0.0, 0.0), 150),
        ("ell_on_seam", "seam", flat_ell, (0.1, 0.25), (0.0, 0.0), 90),
        ("trio_on_hills", "hills2", compound("trio"), (-5.0, 1.0), (2.0, 0.0), 90),
    ]
}

fn scene(config: (&'static str, &'static str, CompoundSpec, (f64, f64), (f64, f64), usize)) -> Value {
    let (id, ground_name, comp, start, linvel, num_steps) = config;
    let dt = Q::snap(1.0 / 60.0);
    let gravity = QVec::snap(0.0, -9.81);
    let start = qv(start.0, start.1);
    let linvel = qv(linvel.0, linvel.1);
    let params = IntegrationParameters {
        dt: dt.f(),
        contact_recycling: false,
        contact_clustering: false,
        max_ccd_substeps: 0,
        ..Default::default()
    };
    let mut pipeline = PhysicsPipeline::new();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let (ground_json, ground) = scene_ground(ground_name);
    let g = colliders.insert(ColliderBuilder::new(ground).active_events(ActiveEvents::COLLISION_EVENTS));
    let body = bodies.insert(RigidBodyBuilder::dynamic().translation(start.v()).linvel(linvel.v()));
    let c = colliders.insert_with_parent(ColliderBuilder::new(comp.shared()), body, &mut bodies);
    let collector = Collector::default();
    let mut samples = Vec::new();
    let mut events = Vec::new();
    for step in 1..=num_steps {
        pipeline.step(
            gravity.v(), &params, &mut islands, &mut broad_phase, &mut narrow_phase, &mut bodies,
            &mut colliders, &mut impulse_joints, &mut multibody_joints, &mut ccd_solver, &(), &collector,
        );
        for e in collector.0.lock().unwrap().drain(..) {
            let (h1, h2, started) = match e {
                CollisionEvent::Started(a, b, _) => (a, b, true),
                CollisionEvent::Stopped(a, b, _) => (a, b, false),
            };
            let idx = |h: ColliderHandle| if h == g { 0 } else if h == c { 1 } else { 9 };
            events.push(json!({ "step": step, "started": started, "collider1": idx(h1), "collider2": idx(h2) }));
        }
        let rb = &bodies[body];
        samples.push(json!({
            "step": step,
            "x": jf(rb.translation().x), "y": jf(rb.translation().y),
            "re": jf(rb.rotation().re), "im": jf(rb.rotation().im),
            "vx": jf(rb.linvel().x), "vy": jf(rb.linvel().y), "w": jf(rb.angvel()),
            "manifolds": narrow_phase.contact_pair(g, c).map(|p| p.manifolds.iter().filter(|m| !m.data.solver_contacts.is_empty()).count()).unwrap_or(0),
        }));
    }
    json!({
        "id": id,
        "ground": ground_json,
        "ground_name": ground_name,
        "compound": comp.json(),
        "start": jqvec(start),
        "linvel": jqvec(linvel),
        "samples": samples,
        "events": events,
    })
}

pub fn compound_scenes() -> Value {
    json!({
        "family": "compound_scenes",
        "note": "SH2b scenes: an L-shaped compound toppling on a half-space, an L lying across a polyline vertex, a three-part compound sliding over a heightfield; see tools/golden/src/sh2b.rs",
        "dt": jq(Q::snap(1.0 / 60.0)),
        "gravity": jqvec(QVec::snap(0.0, -9.81)),
        "scenes": scene_configs().into_iter().map(scene).collect::<Vec<_>>(),
    })
}
