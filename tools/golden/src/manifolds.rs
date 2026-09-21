//! Family `contact_manifolds`: `DefaultQueryDispatcher::contact_manifolds` for every pair of the
//! MVP matrix over six regimes (plus a few extra degenerate and flipped-order cases).

use crate::q::{jf, jq, jqpose, jvec, QPose, QRot, QVec, Q};
use crate::shapes::ShapeSpec;
use rapier2d_f64::dynamics::IntegrationParameters;
use rapier2d_f64::parry::query::{
    ContactManifold, DefaultQueryDispatcher, PersistentQueryDispatcher,
};
use rapier2d_f64::parry::shape::{FeatureId, PackedFeatureId};
use serde_json::{json, Value};

struct Case {
    pair: &'static str,
    regime: &'static str,
    /// Id suffix; equal to `regime` for the six canonical cases of a pair.
    name: &'static str,
    shape1: ShapeSpec,
    shape2: ShapeSpec,
    pos12: QPose,
    /// Discrete outputs (point count, feature ids, normal sign) hinge on an exact tie or on a
    /// fallback branch upstream: a port may legitimately differ, compare with care.
    ambiguous: bool,
    note: &'static str,
}

fn pose(x: f64, y: f64, rot: QRot) -> QPose {
    QPose::new(QVec::snap(x, y), rot)
}

fn deg(d: f64) -> QRot {
    QRot::from_degrees(d)
}

const ID: QRot = QRot::IDENTITY;

#[rustfmt::skip]
fn cases() -> Vec<Case> {
    let mut v = Vec::new();
    let mut add = |pair, regime, name, shape1, shape2, pos12, ambiguous, note| {
        v.push(Case { pair, regime, name, shape1, shape2, pos12, ambiguous, note });
    };
    let ball = ShapeSpec::ball;
    let cuboid = ShapeSpec::cuboid;
    let cap_x = ShapeSpec::capsule_x;
    let cap_y = ShapeSpec::capsule_y;
    let segment = ShapeSpec::segment;

    // --- ball / ball -------------------------------------------------------------------------
    let (s1, s2) = (ball(0.5), ball(0.25));
    add("ball_ball", "separated", "separated", s1, s2, pose(1.5, 1.0, deg(30.0)), false, "gap ~1.05");
    add("ball_ball", "within_pred", "within_pred", s1, s2, pose(0.76, 0.0, ID), false, "gap 0.01 < prediction");
    add("ball_ball", "touching", "touching", ball(0.75), ball(0.5), pose(0.75, 1.0, QRot::QUARTER), false,
        "3-4-5 triangle: centre distance is exactly r1 + r2 = 1.25");
    add("ball_ball", "shallow", "shallow", s1, s2, pose(0.0, -0.7, deg(45.0)), false, "penetration 0.05");
    add("ball_ball", "deep", "deep", s1, s2, pose(0.1, 0.05, deg(-135.0)), false, "centre of ball 2 inside ball 1");
    add("ball_ball", "degenerate", "degenerate", s1, s2, pose(0.0, 0.0, deg(30.0)), true,
        "coincident centres: upstream falls back to local_n1 = +Y");

    // --- ball / cuboid (ball first: upstream runs convex-ball with `flipped = true`) ---------
    let (s1, s2) = (ball(0.5), cuboid(1.0, 0.5));
    add("ball_cuboid", "separated", "separated", s1, s2, pose(2.5, 0.0, ID), false, "gap 1.0");
    add("ball_cuboid", "within_pred", "within_pred", s1, s2, pose(1.51, 0.2, ID), false, "gap 0.01, face region");
    add("ball_cuboid", "touching", "touching", s1, s2, pose(0.0, 1.0, ID), false, "bottom face of the cuboid tangent to the ball");
    add("ball_cuboid", "shallow", "shallow", s1, s2, pose(1.3, 0.8, ID), false, "vertex region of the cuboid");
    add("ball_cuboid", "deep", "deep", s1, s2, pose(0.2, 0.1, deg(30.0)), false, "ball centre inside the rotated cuboid");
    add("ball_cuboid", "degenerate", "degenerate", s1, s2, pose(0.0, 0.0, ID), true,
        "ball centre at the cuboid centre: +Y / -Y faces are equidistant");
    add("ball_cuboid", "degenerate", "degen_on_face", s1, s2, pose(0.0, 0.5, ID), true,
        "ball centre exactly on the cuboid boundary: projection distance is 0");

    // --- ball / capsule ----------------------------------------------------------------------
    let (s1, s2) = (ball(0.5), cap_y(0.5, 0.25));
    add("ball_capsule", "separated", "separated", s1, s2, pose(2.0, 0.0, ID), false, "gap 1.25");
    add("ball_capsule", "within_pred", "within_pred", s1, s2, pose(0.76, 0.1, ID), false, "gap 0.01, segment interior");
    add("ball_capsule", "touching", "touching", s1, s2, pose(0.75, 0.0, ID), false, "exactly tangent");
    add("ball_capsule", "shallow", "shallow", s1, s2, pose(0.8, 0.9, deg(-30.0)), false, "end cap region, rotated capsule");
    add("ball_capsule", "deep", "deep", s1, s2, pose(0.1, 0.0, QRot::QUARTER), false, "ball centre inside the capsule");
    add("ball_capsule", "degenerate", "degenerate", s1, s2, pose(0.0, 0.0, ID), true,
        "ball centre on the capsule segment: zero projection distance, normal fallback");

    // --- cuboid / cuboid ---------------------------------------------------------------------
    let (s1, s2) = (cuboid(1.0, 0.5), cuboid(0.5, 0.5));
    add("cuboid_cuboid", "separated", "separated", s1, s2, pose(3.0, 0.0, ID), false, "gap 1.5");
    add("cuboid_cuboid", "within_pred", "within_pred", s1, s2, pose(1.51, 0.25, ID), false, "parallel faces, gap 0.01, two points");
    add("cuboid_cuboid", "touching", "touching", s1, s2, pose(1.5, 0.0, ID), false, "parallel faces in exact contact");
    add("cuboid_cuboid", "shallow", "shallow", s1, s2, pose(1.6, 0.3, deg(30.0)), false, "vertex of cuboid 2 inside a face of cuboid 1");
    add("cuboid_cuboid", "deep", "deep", s1, s2, pose(0.3, 0.2, deg(45.0)), false, "centre of cuboid 2 inside cuboid 1");
    add("cuboid_cuboid", "degenerate", "degenerate", s1, s2, pose(0.0, 0.0, ID), true,
        "coincident centres, parallel edges: +Y / -Y separating axes tie");
    add("cuboid_cuboid", "degenerate", "degen_corner", s1, s2, pose(1.5, 1.0, ID), true,
        "corner against corner, exactly touching: X and Y axes tie at separation 0");
    add("cuboid_cuboid", "degenerate", "degen_rot90", s1, cuboid(1.0, 0.5), pose(0.0, 1.49, QRot::QUARTER), false,
        "exact quarter turn: edges parallel through a rotation, penetration 0.01");

    // --- cuboid / capsule (upstream: generic PFM-PFM, i.e. GJK/EPA + feature clipping) --------
    let (s1, s2) = (cuboid(1.0, 0.5), cap_y(0.5, 0.25));
    add("cuboid_capsule", "separated", "separated", s1, s2, pose(3.0, 0.0, ID), false, "gap 1.75");
    add("cuboid_capsule", "within_pred", "within_pred", s1, s2, pose(1.26, 0.0, ID), false, "capsule side parallel to the cuboid face, gap 0.01");
    add("cuboid_capsule", "touching", "touching", s1, s2, pose(1.25, 0.0, ID), true, "exact contact: GJK sits on its intersection threshold");
    add("cuboid_capsule", "shallow", "shallow", s1, s2, pose(1.3, 0.2, deg(30.0)), false, "rotated capsule, end cap against the +X face");
    add("cuboid_capsule", "deep", "deep", s1, s2, pose(0.2, 0.1, QRot::QUARTER), false, "capsule segment crosses the cuboid: EPA");
    add("cuboid_capsule", "degenerate", "degenerate", s1, s2, pose(0.0, 0.0, ID), true,
        "coincident centres: EPA on a symmetric configuration");

    // --- capsule / capsule -------------------------------------------------------------------
    let (s1, s2) = (cap_y(0.5, 0.25), cap_y(0.5, 0.25));
    add("capsule_capsule", "separated", "separated", s1, s2, pose(2.0, 0.0, ID), false, "gap 1.5");
    add("capsule_capsule", "within_pred", "within_pred", s1, s2, pose(0.51, 0.2, ID), false, "parallel segments, gap 0.01, two points");
    add("capsule_capsule", "touching", "touching", s1, s2, pose(0.5, 0.0, ID), true, "parallel segments in exact contact (closest-point pair is not unique)");
    add("capsule_capsule", "shallow", "shallow", s1, s2, pose(0.9, 0.0, QRot::QUARTER), false, "T configuration: end cap of 2 against the side of 1");
    add("capsule_capsule", "deep", "deep", s1, s2, pose(0.2, 0.0, deg(10.0)), false, "heavily overlapping, nearly parallel segments that do not cross");
    add("capsule_capsule", "degenerate", "degenerate", s1, s2, pose(0.1, 0.1, QRot::QUARTER), true,
        "segments cross: zero distance, upstream falls back to local_n1 = +Y");
    add("capsule_capsule", "degenerate", "degen_cross30", s1, s2, pose(0.1, 0.0, deg(30.0)), true,
        "segments cross at 30 degrees: the closest points coincide up to float noise, which upstream normalises into a normal");
    add("capsule_capsule", "degenerate", "degen_zero_len", s1, ShapeSpec::capsule((0.0, 0.0), (0.0, 0.0), 0.25), pose(0.45, 0.1, ID), false,
        "capsule 2 has a zero-length segment (a ball in disguise)");

    // --- halfspace / ball --------------------------------------------------------------------
    let (s1, s2) = (ShapeSpec::halfspace_up(), ball(0.5));
    add("halfspace_ball", "separated", "separated", s1, s2, pose(0.3, 1.5, ID), false, "gap 1.0");
    add("halfspace_ball", "within_pred", "within_pred", s1, s2, pose(-2.0, 0.51, deg(30.0)), false, "gap 0.01");
    add("halfspace_ball", "touching", "touching", s1, s2, pose(1.0, 0.5, ID), false, "exactly tangent");
    add("halfspace_ball", "shallow", "shallow", s1, s2, pose(0.0, 0.45, deg(45.0)), false, "penetration 0.05");
    add("halfspace_ball", "deep", "deep", s1, s2, pose(5.0, -3.0, ID), false, "ball entirely below the plane");
    add("halfspace_ball", "degenerate", "degenerate", s1, s2, pose(0.0, 0.0, ID), true,
        "ball centre at the half-space origin: zero distance, normal fallback");
    add("halfspace_ball", "degenerate", "degen_on_plane", s1, s2, pose(1.0, 0.0, ID), true,
        "ball centre on the plane away from the origin: upstream normal = normalised translation");

    // --- halfspace / cuboid ------------------------------------------------------------------
    let (s1, s2) = (ShapeSpec::halfspace_up(), cuboid(1.0, 0.5));
    add("halfspace_cuboid", "separated", "separated", s1, s2, pose(0.0, 2.0, ID), false, "gap 1.5");
    add("halfspace_cuboid", "within_pred", "within_pred", s1, s2, pose(0.0, 0.51, ID), false, "face parallel to the plane, gap 0.01");
    add("halfspace_cuboid", "touching", "touching", s1, s2, pose(3.0, 0.5, ID), false, "face resting exactly on the plane");
    add("halfspace_cuboid", "shallow", "shallow", s1, s2, pose(0.0, 0.9, deg(30.0)), false, "one vertex below the plane");
    add("halfspace_cuboid", "deep", "deep", s1, s2, pose(0.0, -2.0, deg(45.0)), false, "cuboid entirely below the plane");
    add("halfspace_cuboid", "degenerate", "degenerate", s1, s2, pose(0.0, 0.5, deg(1.0)), true,
        "edge almost parallel to the plane: second vertex sits next to the prediction threshold");

    // --- segment / ball ----------------------------------------------------------------------
    let (s1, s2) = (ShapeSpec::segment((-1.0, 0.0), (1.0, 0.0)), ball(0.5));
    add("segment_ball", "separated", "separated", s1, s2, pose(0.0, 2.0, ID), false, "gap 1.5");
    add("segment_ball", "within_pred", "within_pred", s1, s2, pose(0.3, 0.51, deg(30.0)), false, "gap 0.01, segment interior");
    add("segment_ball", "touching", "touching", s1, s2, pose(-0.5, 0.5, ID), false, "exactly tangent");
    add("segment_ball", "shallow", "shallow", s1, s2, pose(1.3, 0.3, ID), false, "vertex region of the segment");
    add("segment_ball", "deep", "deep", s1, s2, pose(0.2, -0.1, deg(45.0)), false, "ball centre 0.1 below the segment");
    add("segment_ball", "degenerate", "degenerate", s1, s2, pose(0.0, 0.0, ID), true,
        "ball centre on the segment at the origin: zero distance, normal fallback +Y");
    add("segment_ball", "degenerate", "degen_on_segment", s1, s2, pose(0.5, 0.0, ID), true,
        "ball centre on the segment away from the origin: upstream normal = normalised translation (along the segment)");

    // --- flipped argument order --------------------------------------------------------------
    add("cuboid_ball", "shallow", "shallow", cuboid(1.0, 0.5), ball(0.5), pose(-1.3, -0.8, ID), false,
        "mirror of ball_cuboid/shallow with the shapes swapped");
    add("capsule_ball", "shallow", "shallow", cap_y(0.5, 0.25), ball(0.5), pose(0.3, 0.9, deg(30.0)), false,
        "capsule first: convex-ball without flipping");
    add("ball_halfspace", "shallow", "shallow", ball(0.5), ShapeSpec::halfspace_up(), pose(0.0, -0.45, ID), false,
        "mirror of halfspace_ball/shallow with the shapes swapped");
    add("ball_segment", "shallow", "shallow", ball(0.5), ShapeSpec::segment((-1.0, 0.0), (1.0, 0.0)), pose(-1.3, -0.3, ID), false,
        "mirror of segment_ball/shallow with the shapes swapped");
    add("capsule_cuboid", "shallow", "shallow", cap_y(0.5, 0.25), cuboid(1.0, 0.5), pose(-1.2, 0.1, ID), false,
        "capsule first in the PFM-PFM path");

    // --- G3: halfspace / capsule (upstream: halfspace-PFM) -----------------------------------
    let (s1, s2) = (ShapeSpec::halfspace_up(), cap_y(0.5, 0.25));
    add("halfspace_capsule", "separated", "separated", s1, s2, pose(0.0, 1.5, ID), false,
        "upright capsule above the plane, gap 0.75");
    add("halfspace_capsule", "within_pred", "within_pred", s1, s2, pose(0.0, 0.76, ID), false,
        "upright capsule, bottom cap gap 0.01 < prediction");
    add("halfspace_capsule", "touching", "touching", s1, cap_x(0.5, 0.25), pose(0.0, 0.25, ID), false,
        "capsule axis parallel to the plane, both rounded endpoints tangent");
    add("halfspace_capsule", "shallow", "shallow", s1, cap_x(0.5, 0.25), pose(0.0, 0.2, deg(30.0)), false,
        "tilted 30 degrees with one rounded endpoint below the plane");
    add("halfspace_capsule", "deep", "deep", s1, cap_x(0.5, 0.25), pose(0.0, -0.2, deg(30.0)), false,
        "tilted 30 degrees with the capsule deeply crossing the plane");
    add("halfspace_capsule", "degenerate", "degenerate", s1, cap_x(0.5, 0.25), pose(0.0, 0.27000000001862645, ID), true,
        "axis parallel to the plane and both endpoints exactly at the prediction distance");

    // --- G3: halfspace / segment (upstream: halfspace-PFM) -----------------------------------
    let (s1, s2) = (ShapeSpec::halfspace_up(), segment((0.0, -0.5), (0.0, 0.5)));
    add("halfspace_segment", "separated", "separated", s1, s2, pose(0.0, 1.0, ID), false,
        "upright segment above the plane, gap 0.5");
    add("halfspace_segment", "within_pred", "within_pred", s1, s2, pose(0.0, 0.51, ID), false,
        "upright segment, lower endpoint gap 0.01 < prediction");
    add("halfspace_segment", "touching", "touching", s1, segment((-0.5, 0.0), (0.5, 0.0)), pose(0.0, 0.0, ID), false,
        "segment lying on the plane");
    add("halfspace_segment", "shallow", "shallow", s1, segment((-0.5, 0.0), (0.5, 0.0)), pose(0.0, -0.05, deg(30.0)), false,
        "tilted 30 degrees with one endpoint below the plane");
    add("halfspace_segment", "deep", "deep", s1, segment((-0.5, 0.0), (0.5, 0.0)), pose(0.0, -0.5, deg(30.0)), false,
        "tilted 30 degrees and crossing the plane deeply");
    add("halfspace_segment", "degenerate", "degenerate", s1, segment((-0.5, 0.0), (0.5, 0.0)), pose(0.0, 0.0, ID), true,
        "segment exactly collinear with the halfspace boundary");

    // --- G3: cuboid / segment (upstream: generic PFM-PFM, i.e. GJK/EPA + clipping) -----------
    let (s1, s2) = (cuboid(1.0, 0.5), segment((0.0, -0.5), (0.0, 0.5)));
    add("cuboid_segment", "separated", "separated", s1, s2, pose(1.5, 0.0, ID), false,
        "segment parallel to the +X face with a gap 0.5");
    add("cuboid_segment", "within_pred", "within_pred", s1, s2, pose(1.01, 0.0, ID), false,
        "segment parallel to the +X face, gap 0.01 < prediction");
    add("cuboid_segment", "touching", "touching", s1, s2, pose(1.0, 0.0, ID), false,
        "segment exactly on the +X face, yielding two contact points");
    add("cuboid_segment", "shallow", "shallow", s1, segment((0.0, 0.0), (0.25, 0.0)), pose(1.01, 0.51, deg(30.0)), true,
        "segment endpoint tilted 30 degrees towards the top-right corner; upstream emits duplicate tied features and f32/f64 pick different ids");
    add("cuboid_segment", "deep", "deep", s1, segment((-1.5, 0.0), (1.5, 0.0)), pose(0.0, 0.0, ID), false,
        "segment crosses the cuboid through its centre");
    add("cuboid_segment", "degenerate", "degenerate", s1, segment((-1.0, 0.0), (1.0, 0.0)), pose(0.0, 0.5, ID), true,
        "segment collinear with the cuboid top edge; feature tie in the PFM-PFM path");

    // --- G3: flipped argument order ----------------------------------------------------------
    add("capsule_halfspace", "shallow", "shallow", cap_y(0.5, 0.25), ShapeSpec::halfspace_up(), pose(0.0, -0.7, ID), false,
        "capsule first, mirror of a shallow halfspace-capsule contact");
    add("segment_halfspace", "shallow", "shallow", segment((-0.5, 0.0), (0.5, 0.0)), ShapeSpec::halfspace_up(), pose(0.0, 0.05, ID), false,
        "segment first, mirror of a shallow halfspace-segment contact");
    add("segment_cuboid", "shallow", "shallow", segment((0.0, -0.5), (0.0, 0.5)), cuboid(1.0, 0.5), pose(-0.95, 0.0, ID), false,
        "segment first, mirror of a shallow cuboid-segment face contact");

    v
}

fn fid_json(packed: u32) -> Value {
    let (kind, code) = match PackedFeatureId(packed).unpack() {
        FeatureId::Vertex(c) => ("vertex", c),
        FeatureId::Face(c) => ("face", c),
        FeatureId::Unknown => ("unknown", 0),
    };
    json!({ "packed": packed, "kind": kind, "code": code })
}

/// One contact point of the f32 cross-check run: `(local_p1, fid1, fid2)`.
type PointF32 = ([f32; 2], u32, u32);

/// Runs the same query with the f32 build of parry and returns the points of each manifold.
fn run_f32(case: &Case, prediction: Q) -> Vec<Vec<PointF32>> {
    use parry2d::math::{Pose, Rotation, Vector};
    use parry2d::query::{ContactManifold, DefaultQueryDispatcher, PersistentQueryDispatcher};

    let t = case.pos12.translation;
    let r = case.pos12.rotation;
    let pos12 = Pose::from_parts(
        Vector::new(t.x.f() as f32, t.y.f() as f32),
        Rotation {
            re: r.re.f() as f32,
            im: r.im.f() as f32,
        },
    );
    let mut manifolds: Vec<ContactManifold<(), ()>> = Vec::new();
    let mut workspace = None;
    let (shape1, shape2) = (case.shape1.shared_f32(), case.shape2.shared_f32());
    DefaultQueryDispatcher
        .contact_manifolds(
            &pos12,
            &*shape1.0,
            &*shape2.0,
            prediction.f() as f32,
            &mut manifolds,
            &mut workspace,
        )
        .expect("unsupported shape pair");
    manifolds
        .iter()
        .map(|m| {
            m.points
                .iter()
                .map(|p| ([p.local_p1.x, p.local_p1.y], p.fid1.0, p.fid2.0))
                .collect()
        })
        .collect()
}

fn run(case: &Case, prediction: Q) -> Value {
    let mut manifolds: Vec<ContactManifold<(), ()>> = Vec::new();
    let mut workspace = None;
    let (shape1, shape2) = (case.shape1.shared(), case.shape2.shared());
    DefaultQueryDispatcher
        .contact_manifolds(
            &case.pos12.p(),
            &*shape1.0,
            &*shape2.0,
            prediction.f(),
            &mut manifolds,
            &mut workspace,
        )
        .expect("unsupported shape pair");
    let manifolds_f32 = run_f32(case, prediction);
    let id = format!("{}/{}", case.pair, case.name);

    let manifolds_json: Vec<Value> = manifolds
        .iter()
        .enumerate()
        .map(|(i, m)| {
            // The f32 run only contributes feature ids, and only when it found the same points
            // in the same order. Ambiguous cases may legitimately disagree.
            let points_f32 = manifolds_f32.get(i).filter(|pts| {
                pts.len() == m.points.len()
                    && pts.iter().zip(&m.points).all(|(a, b)| {
                        (a.0[0] as f64 - b.local_p1.x).abs() < 1e-4
                            && (a.0[1] as f64 - b.local_p1.y).abs() < 1e-4
                    })
            });
            assert!(
                points_f32.is_some() || case.ambiguous,
                "{id}: the f32 and f64 builds disagree on a case not tagged ambiguous"
            );
            let points: Vec<Value> = m
                .points
                .iter()
                .enumerate()
                .map(|(k, p)| {
                    let (fid1, fid2) = match points_f32 {
                        Some(pts) => (fid_json(pts[k].1), fid_json(pts[k].2)),
                        None => (Value::Null, Value::Null),
                    };
                    json!({
                        "local_p1": jvec(p.local_p1),
                        "local_p2": jvec(p.local_p2),
                        "dist": jf(p.dist),
                        "fid1": fid1,
                        "fid2": fid2,
                        "fid1_f64_build": fid_json(p.fid1.0),
                        "fid2_f64_build": fid_json(p.fid2.0),
                    })
                })
                .collect();
            json!({
                "local_n1": jvec(m.local_n1),
                "local_n2": jvec(m.local_n2),
                "num_points": points.len(),
                "points": points,
            })
        })
        .collect();

    json!({
        "id": id,
        "pair": case.pair,
        "regime": case.regime,
        "ambiguous": case.ambiguous,
        "note": case.note,
        "shape1": case.shape1.json(),
        "shape2": case.shape2.json(),
        "pos12": jqpose(case.pos12),
        "expected": {
            "num_manifolds": manifolds_json.len(),
            "manifolds": manifolds_json,
        },
    })
}

pub fn generate() -> Value {
    let default_prediction = IntegrationParameters::default().prediction_distance();
    let prediction = Q::snap(default_prediction);
    let cases: Vec<Value> = cases().iter().map(|c| run(c, prediction)).collect();

    json!({
        "family": "contact_manifolds",
        "prediction_upstream_default": jf(default_prediction),
        "prediction": jq(prediction),
        "cases": cases,
    })
}
