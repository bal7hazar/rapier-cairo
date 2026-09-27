//! Work package SH2a: the polyline and the 2D heightfield. Three families, in files of their own
//! so that no existing table grows:
//!
//! * `composite_contacts`: `DefaultQueryDispatcher::contact_manifolds` of a composite shape
//!   against the convex shapes (both orders), one manifold per part; every manifold with points
//!   is recorded with its sub-shape ids (upstream's order is its BVH's: compare by id). Each case
//!   also records `query::intersection_test` and `query::distance` (`None` when unsupported).
//! * `composite_queries`: point projections (feature, `contains`), world-space ray casts, the
//!   shape-pair queries (`intersection_test`, `distance`, `contact`, `closest_points`,
//!   `cast_shapes`) and the AABBs and bounding spheres of the composite shapes.
//! * `composite_scenes`: a box sliding over a heightfield, a ball rolling along a polyline and a
//!   box resting across a polyline vertex (rapier2d-f64, no CCD, no contact recycling or
//!   clustering, as the other scene families), with every collision event.
//!
//! The composite shapes are given by Q32.32 inputs: the vertices of the polylines and the
//! heights and scales of the heightfields; both engines build them from the same raws.

use crate::manifolds::fid_json;
use crate::point_projection::{feature_json, proj_json};
use crate::q::{jf, jq, jqpose, jqvec, jvec, QPose, QRot, QVec, Q};
use crate::ray_casts::hit_json;
use crate::sh1::Sh1Shape;
use crate::shapes::ShapeSpec;
use rapier2d_f64::math::{Pose, Vector};
use rapier2d_f64::parry::query::{
    self, ClosestPoints, ContactManifold, DefaultQueryDispatcher, PersistentQueryDispatcher, Ray,
    ShapeCastOptions,
};
use rapier2d_f64::parry::shape::{HeightField, Polyline, PolylineFlags, SharedShape};
use rapier2d_f64::prelude::*;
use serde_json::{json, Value};
use std::sync::Mutex;

/// A composite shape of the SH2a families.
#[derive(Clone, Debug)]
pub enum Composite {
    Polyline { vertices: Vec<QVec>, indices: Option<Vec<[u32; 2]>>, oriented: bool },
    HeightField { heights: Vec<Q>, scale: QVec, removed: Vec<usize> },
}

pub(crate) fn qv(x: f64, y: f64) -> QVec {
    QVec::snap(x, y)
}

impl Composite {
    fn polyline(points: &[(f64, f64)]) -> Self {
        Composite::Polyline {
            vertices: points.iter().map(|p| qv(p.0, p.1)).collect(),
            indices: None,
            oriented: false,
        }
    }

    fn heightfield(heights: &[f64], sx: f64, sy: f64, removed: &[usize]) -> Self {
        Composite::HeightField {
            heights: heights.iter().map(|h| Q::snap(*h)).collect(),
            scale: qv(sx, sy),
            removed: removed.to_vec(),
        }
    }

    pub fn shared(&self) -> SharedShape {
        match self {
            Composite::Polyline { vertices, indices, oriented } => {
                let flags = if *oriented { PolylineFlags::ORIENTED } else { PolylineFlags::empty() };
                SharedShape::new(Polyline::with_flags(
                    vertices.iter().map(|v| v.v()).collect(),
                    indices.clone(),
                    flags,
                ))
            }
            Composite::HeightField { heights, scale, removed } => {
                let mut h = HeightField::new(heights.iter().map(|q| q.f()).collect(), scale.v());
                for i in removed {
                    h.set_segment_removed(*i, true);
                }
                SharedShape::new(h)
            }
        }
    }

    pub fn json(&self) -> Value {
        match self {
            Composite::Polyline { vertices, indices, oriented } => json!({
                "type": "polyline",
                "vertices": vertices.iter().map(|v| jqvec(*v)).collect::<Vec<_>>(),
                "indices": indices.clone(),
                "oriented": oriented,
            }),
            Composite::HeightField { heights, scale, removed } => json!({
                "type": "heightfield",
                "heights": heights.iter().map(|h| jq(*h)).collect::<Vec<_>>(),
                "scale": jqvec(*scale),
                "removed": removed,
            }),
        }
    }

    /// The surface points `(x, y)` in order (polyline vertices; heightfield samples).
    fn surface(&self) -> Vec<(f64, f64)> {
        match self {
            Composite::Polyline { vertices, .. } => vertices.iter().map(|v| (v.x.f(), v.y.f())).collect(),
            Composite::HeightField { heights, scale, .. } => {
                let n = heights.len() as f64 - 1.0;
                heights
                    .iter()
                    .enumerate()
                    .map(|(i, h)| ((-0.5 + i as f64 / n) * scale.x.f(), h.f() * scale.y.f()))
                    .collect()
            }
        }
    }

    /// The largest surface height over `[x0, x1]` (placement of a shape resting above).
    pub(crate) fn top_over(&self, x0: f64, x1: f64) -> f64 {
        let s = self.surface();
        let mut top = f64::MIN;
        for w in s.windows(2) {
            let ((ax, ay), (bx, by)) = (w[0], w[1]);
            let (lo, hi) = (ax.min(bx), ax.max(bx));
            if hi < x0 || lo > x1 {
                continue;
            }
            let at = |x: f64| if bx == ax { ay.max(by) } else { ay + (by - ay) * (x - ax) / (bx - ax) };
            top = top.max(at(x0.clamp(lo, hi))).max(at(x1.clamp(lo, hi)));
        }
        top
    }
}

/// The composite shapes, by name.
pub fn composites() -> Vec<(&'static str, Composite)> {
    let square = Composite::Polyline {
        vertices: vec![qv(-1.0, -1.0), qv(1.0, -1.0), qv(1.0, 1.0), qv(-1.0, 1.0)],
        indices: Some(vec![[0, 1], [1, 2], [2, 3], [3, 0]]),
        oriented: true,
    };
    let mut plain_square = square.clone();
    if let Composite::Polyline { oriented, .. } = &mut plain_square {
        *oriented = false;
    }
    vec![
        ("vee", Composite::polyline(&[(-2.0, 1.0), (0.0, 0.0), (2.0, 1.0)])),
        (
            "bumps",
            Composite::polyline(&[
                (-5.0, 0.0), (-4.0, 0.2), (-3.0, 0.0), (-2.0, 0.3), (-1.0, 0.1), (0.0, 0.0),
                (1.0, 0.25), (2.0, 0.0), (3.0, 0.15), (4.0, 0.0), (5.0, 0.0),
            ]),
        ),
        ("flat", Composite::heightfield(&[0.0, 0.0, 0.0, 0.0, 0.0], 8.0, 1.0, &[])),
        ("hills", Composite::heightfield(&[0.0, 0.5, 0.2, 0.8, 0.3, 0.0], 10.0, 1.0, &[])),
        ("holed", Composite::heightfield(&[0.0, 0.5, 0.2, 0.8, 0.3, 0.0], 10.0, 1.0, &[2])),
        ("square", square),
        ("plain_square", plain_square),
    ]
}

pub(crate) fn composite(name: &str) -> Composite {
    composites().into_iter().find(|(n, _)| *n == name).unwrap().1
}

pub(crate) fn index_of(name: &str) -> usize {
    composites().iter().position(|(n, _)| *n == name).unwrap()
}

pub(crate) fn b(s: ShapeSpec) -> Sh1Shape {
    Sh1Shape::Base(s)
}

/// The convex shapes of the contact family.
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

pub(crate) fn inverse(p: QPose) -> QPose {
    let inv = p.p().inverse();
    QPose::new(QVec::snap(inv.translation.x, inv.translation.y), QRot { re: Q::snap(inv.rotation.re), im: Q::snap(inv.rotation.im) })
}

pub(crate) fn json_opt<T>(r: Result<T, query::Unsupported>, f: impl Fn(T) -> Value) -> Value {
    match r {
        Ok(v) => json!({ "supported": true, "value": f(v) }),
        Err(_) => json!({ "supported": false, "value": Value::Null }),
    }
}

// ---------------------------------------------------------------------------------------------
// Manifolds
// ---------------------------------------------------------------------------------------------

/// `(regime, gap, degrees)`: shape 2 above the composite's surface.
const REGIMES: [(&str, f64, f64); 3] =
    [("separated", 0.5, 0.0), ("within_pred", 0.01, 0.0), ("overlapping", -0.05, 10.0)];

/// `(composite, x)`: over a vertex or a cell boundary, so that several parts touch.
const PLACEMENTS: [(&str, f64); 5] =
    [("vee", 0.0), ("bumps", -2.0), ("flat", 0.0), ("hills", 1.4), ("holed", -1.0)];

fn manifold_case(comp_name: &str, comp: &Composite, shape_name: &str, shape: Sh1Shape, x: f64, regime: (&str, f64, f64), first: bool, prediction: Q) -> Value {
    let (regime_name, gap, deg) = regime;
    let rot = if deg == 0.0 { QRot::IDENTITY } else { QRot::from_degrees(deg) };
    // The shape's box turned by `rot`: its bottom rests `gap` above the highest surface point
    // under it; a ball in the "V" is placed tangent to both arms (slope 1/2).
    let local = shape.shared().compute_aabb(&QPose::new(QVec::ZERO, rot).p());
    let y = if comp_name == "vee" && shape_name == "ball" {
        (0.4 + gap) * 1.25f64.sqrt()
    } else {
        comp.top_over(x + local.mins.x, x + local.maxs.x) - local.mins.y + gap
    };
    let above = QPose::new(QVec::snap(x, y), rot);
    let pos12 = if first { above } else { inverse(above) };
    let (s_comp, s_conv) = (comp.shared(), shape.shared());
    let (s1, s2) = if first { (&s_comp, &s_conv) } else { (&s_conv, &s_comp) };
    let mut manifolds: Vec<ContactManifold<(), ()>> = Vec::new();
    let mut workspace = None;
    let p = pos12.p();
    DefaultQueryDispatcher
        .contact_manifolds(&p, &*s1.0, &*s2.0, prediction.f(), &mut manifolds, &mut workspace)
        .expect("unsupported composite pair");
    let mut parts: Vec<Value> = manifolds
        .iter()
        .filter(|m| !m.points.is_empty())
        .map(|m| {
            json!({
                "part": if first { m.subshape1 } else { m.subshape2 },
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
    parts.sort_by_key(|m| m["part"].as_u64().unwrap());
    assert!(parts.len() <= 4, "at most four touching parts per case");
    let identity = Pose::IDENTITY;
    let order = if first { format!("{comp_name}_{shape_name}") } else { format!("{shape_name}_{comp_name}") };
    json!({
        "id": format!("{order}/{regime_name}"),
        "composite": index_of(comp_name),
        "composite_first": first,
        "other": shape.json(),
        "pos12": jqpose(pos12),
        "expected": {
            "manifolds": parts,
            "intersects": json_opt(query::intersection_test(&identity, &*s1.0, &p, &*s2.0), |v| json!(v)),
            "distance": json_opt(query::distance(&identity, &*s1.0, &p, &*s2.0), jf),
        },
    })
}

pub fn composite_contacts() -> Value {
    let prediction = Q::snap(IntegrationParameters::default().prediction_distance());
    let mut cases = Vec::new();
    for (comp_name, x) in PLACEMENTS {
        let comp = composite(comp_name);
        for (shape_name, shape) in convexes() {
            for regime in REGIMES {
                cases.push(manifold_case(comp_name, &comp, shape_name, shape, x, regime, true, prediction));
            }
        }
        // Reversed order (composite second): two shapes, touching regime.
        for (shape_name, shape) in convexes().into_iter().take(2) {
            cases.push(manifold_case(comp_name, &comp, shape_name, shape, x, REGIMES[2], false, prediction));
        }
    }
    json!({
        "family": "composite_contacts",
        "prediction": jq(prediction),
        "composites": composites().iter().map(|(n, c)| json!({ "name": n, "shape": c.json() })).collect::<Vec<_>>(),
        "cases": cases,
    })
}

// ---------------------------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------------------------

fn points() -> Vec<Value> {
    let pts = [(0.3, 2.0), (-1.7, 0.4), (0.0, -0.5), (2.5, 0.2), (0.1, 0.05), (0.25, -0.5)];
    let mut out = Vec::new();
    for (name, comp) in composites() {
        let s = comp.shared();
        for (k, (x, y)) in pts.iter().enumerate() {
            let p = qv(*x, *y);
            let pv: Vector = p.v();
            let (proj, feature) = s.project_local_point_and_get_feature(pv);
            out.push(json!({
                "id": format!("{name}/p{k}"),
                "composite": index_of(name),
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
        ("down", id, (0.3, 3.0), (0.0, -1.0)),
        ("slant", id, (-3.0, 2.0), (1.0, -0.6)),
        ("up", id, (0.2, -2.0), (0.0, 1.0)),
        ("miss", id, (-6.0, 5.0), (1.0, 0.0)),
        ("posed", posed, (1.0, 3.0), (-0.2, -1.0)),
    ];
    let max = Q::snap(100.0);
    let mut out = Vec::new();
    for (name, comp) in composites() {
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
                "composite": index_of(name),
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

/// `ShapeCastStatus` as its declaration index in the port.
pub(crate) fn status_index(s: query::ShapeCastStatus) -> u8 {
    match s {
        query::ShapeCastStatus::OutOfIterations => 0,
        query::ShapeCastStatus::Converged => 1,
        query::ShapeCastStatus::Failed => 2,
        query::ShapeCastStatus::PenetratingOrWithinTargetDist => 3,
    }
}

pub(crate) fn closest_json(c: ClosestPoints) -> Value {
    match c {
        ClosestPoints::Intersecting => json!({ "kind": 0 }),
        ClosestPoints::WithinMargin(a, b) => json!({ "kind": 1, "p1": jvec(a), "p2": jvec(b) }),
        ClosestPoints::Disjoint => json!({ "kind": 2 }),
    }
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
    for comp_name in ["vee", "hills", "flat", "bumps"] {
        let comp = composite(comp_name);
        for (other_name, other) in others {
            for (gap_name, gap) in [("hover", 0.3), ("deep", -0.05)] {
                for first in [true, false] {
                    let x = 0.9;
                    let y = comp.top_over(x - 0.6, x + 0.6) - other.y_range().0 + gap;
                    let pos_other = QPose::new(qv(x, y), QRot::IDENTITY);
                    // Composite at `pos_comp`, the other shape at `pos_comp * pos_other`: pure
                    // translations, exact in Q32.32.
                    let pos_comp = QPose::new(qv(1.0, -2.0), QRot::IDENTITY);
                    let world_other = QPose::new(qv(1.0 + x, -2.0 + y), QRot::IDENTITY);
                    let _ = pos_other;
                    let (sc, so) = (comp.shared(), other.shared());
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
                        "composite": index_of(comp_name),
                        "composite_first": first,
                        "other": other.json(),
                        "pos1": jqpose(pos1),
                        "pos2": jqpose(pos2),
                        "vel": jvec(vel),
                        "prediction": jq(prediction),
                        "margin": jq(margin),
                        "expected": {
                            "intersects": json_opt(query::intersection_test(&p1, &*s1.0, &p2, &*s2.0), |v| json!(v)),
                            "distance": json_opt(query::distance(&p1, &*s1.0, &p2, &*s2.0), jf),
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

fn aabbs() -> Vec<Value> {
    let poses = [
        ("identity", QPose::new(QVec::ZERO, QRot::IDENTITY)),
        ("rot30", QPose::new(qv(1.0, 2.0), QRot::from_degrees(30.0))),
    ];
    let mut out = Vec::new();
    for (name, comp) in composites() {
        for (pose_name, pose) in poses {
            let s = comp.shared();
            let aabb = s.compute_aabb(&pose.p());
            let sphere = s.compute_bounding_sphere(&pose.p());
            out.push(json!({
                "id": format!("{name}/{pose_name}"),
                "composite": index_of(name),
                "pose": jqpose(pose),
                "expected": {
                    "mins": jvec(aabb.mins), "maxs": jvec(aabb.maxs),
                    "center": jvec(sphere.center), "radius": jf(sphere.radius),
                    "mass": jf(s.mass_properties(1.0).mass()),
                },
            }));
        }
    }
    out
}

pub fn composite_queries() -> Value {
    json!({
        "family": "composite_queries",
        "composites": composites().iter().map(|(n, c)| json!({ "name": n, "shape": c.json() })).collect::<Vec<_>>(),
        "points": points(),
        "rays": rays(),
        "pairs": pairs(),
        "aabbs": aabbs(),
    })
}

// ---------------------------------------------------------------------------------------------
// Scenes
// ---------------------------------------------------------------------------------------------

#[derive(Default)]
pub(crate) struct Collector(pub(crate) Mutex<Vec<CollisionEvent>>);

impl EventHandler for Collector {
    fn handle_collision_event(&self, _: &RigidBodySet, _: &ColliderSet, event: CollisionEvent, _: Option<&ContactPair>) {
        self.0.lock().unwrap().push(event);
    }
    fn handle_contact_force_event(&self, _: Real, _: &RigidBodySet, _: &ColliderSet, _: &ContactPair, _: Real) {}
}

const NUM_STEPS: usize = 90;

/// `(id, ground, dynamic shape, start, linvel)`: the ground is a standalone collider (index 0,
/// `COLLISION_EVENTS`), the dynamic body carries one collider (index 1).
fn scene_configs() -> Vec<(&'static str, &'static str, Sh1Shape, (f64, f64), (f64, f64))> {
    vec![
        ("box_on_hills", "hills2", b(ShapeSpec::cuboid(0.4, 0.2)), (-5.0, 0.45), (3.0, 0.0)),
        ("ball_on_bumps", "bumps", b(ShapeSpec::ball(0.3)), (-4.0, 0.9), (2.0, 0.0)),
        ("box_on_seam", "seam", b(ShapeSpec::cuboid(1.0, 0.25)), (0.25, 0.26), (0.0, 0.0)),
    ]
}

fn scene_ground(name: &str) -> Composite {
    match name {
        "hills2" => Composite::heightfield(&[0.0, 0.3, 0.1, 0.5, 0.2, 0.0, 0.4, 0.1], 14.0, 1.0, &[]),
        "seam" => Composite::polyline(&[(-3.0, 0.0), (0.0, 0.0), (3.0, 0.0)]),
        other => composite(other),
    }
}

fn scene(config: (&'static str, &'static str, Sh1Shape, (f64, f64), (f64, f64))) -> Value {
    let (id, ground_name, shape, start, linvel) = config;
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
    let ground = scene_ground(ground_name);
    let g = colliders.insert(ColliderBuilder::new(ground.shared()).active_events(ActiveEvents::COLLISION_EVENTS));
    let body = bodies.insert(RigidBodyBuilder::dynamic().translation(start.v()).linvel(linvel.v()));
    let c = colliders.insert_with_parent(ColliderBuilder::new(shape.shared()), body, &mut bodies);
    let collector = Collector::default();
    let mut samples = Vec::new();
    let mut events = Vec::new();
    for step in 1..=NUM_STEPS {
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
        "ground": ground.json(),
        "ground_name": ground_name,
        "shape": shape.json(),
        "start": jqvec(start),
        "linvel": jqvec(linvel),
        "samples": samples,
        "events": events,
    })
}

pub fn composite_scenes() -> Value {
    json!({
        "family": "composite_scenes",
        "note": "SH2a scenes: a box over a heightfield, a ball along a polyline, a box across a polyline vertex; see tools/golden/src/sh2a.rs",
        "dt": jq(Q::snap(1.0 / 60.0)),
        "gravity": jqvec(QVec::snap(0.0, -9.81)),
        "num_steps": NUM_STEPS,
        "scenes": scene_configs().into_iter().map(scene).collect::<Vec<_>>(),
    })
}
