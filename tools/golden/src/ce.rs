//! Lot CE: contact manifolds of compounds built with parry 0.31's
//! `CompoundFlags::FIX_INTERNAL_EDGES` against a body sliding across the cut between two parts,
//! and the same placements on the unflagged compound (`plain`), which must equal today's
//! `compound_contacts` behaviour. In a file of its own so that no existing table grows.
//!
//! * `row2` (two unit cuboids side by side) and `row3` (three): a ball and a cuboid sliding across
//!   the seams, slightly sunk into the top (cuboid–cuboid ignores the cones upstream; the ball is
//!   the constrained pair);
//! * `poly2` (two unit squares as convex polygons): a square polygon and a ball across the seam
//!   (the PFM–PFM pair, which SAT answers here).
//!
//! Every case is generated with the compound first and, for the ball, also second.

use crate::q::{jq, jqpose, QPose, QRot, Q};
use crate::sh1::Sh1Shape;
use crate::sh2a::{b, inverse, json_opt, qv};
use crate::sh2b::{at, distance_json, manifolds_json, CompoundSpec};
use crate::shapes::ShapeSpec;
use rapier2d_f64::math::Pose;
use rapier2d_f64::parry::query::{self, ContactManifold, DefaultQueryDispatcher, PersistentQueryDispatcher};
use rapier2d_f64::parry::shape::{Compound, CompoundFlags, SharedShape};
use rapier2d_f64::prelude::*;
use serde_json::{json, Value};

fn compounds() -> Vec<(&'static str, CompoundSpec)> {
    let unit = || b(ShapeSpec::cuboid(0.5, 0.5));
    let square = || b(ShapeSpec::polygon(&[(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)]));
    vec![
        ("row2", CompoundSpec { parts: vec![(at(0.0, 0.0), unit()), (at(1.0, 0.0), unit())] }),
        (
            "row3",
            CompoundSpec { parts: vec![(at(0.0, 0.0), unit()), (at(1.0, 0.0), unit()), (at(2.0, 0.0), unit())] },
        ),
        ("poly2", CompoundSpec { parts: vec![(at(0.0, 0.0), square()), (at(1.0, 0.0), square())] }),
    ]
}

fn compound_index(name: &str) -> usize {
    compounds().iter().position(|(n, _)| *n == name).unwrap()
}

fn shared(name: &str, flagged: bool) -> SharedShape {
    let spec = compounds().into_iter().find(|(n, _)| *n == name).unwrap().1;
    let parts = spec.parts.iter().map(|(p, s)| (p.p(), s.shared())).collect();
    if flagged {
        SharedShape::new(Compound::with_flags(parts, CompoundFlags::FIX_INTERNAL_EDGES, None))
    } else {
        SharedShape::new(Compound::new(parts))
    }
}

/// One case, as `sh2b`'s `manifold_case`: `pos` places `other` in the compound's frame; `first`
/// puts the compound first.
fn case(id: String, comp: &str, other: &Sh1Shape, pos: QPose, first: bool, flagged: bool, prediction: Q) -> Value {
    let pos12 = if first { pos } else { inverse(pos) };
    let (s_comp, s_other) = (shared(comp, flagged), other.shared());
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
        "compound": compound_index(comp),
        "compound_first": first,
        "other": json!({ "kind": "convex", "shape": other.json() }),
        "pos12": jqpose(pos12),
        "expected": {
            "manifolds": manifolds_json(&manifolds),
            "intersects": json_opt(query::intersection_test(&identity, &*s1.0, &p, &*s2.0).map(|i| i.intersecting), |v| json!(v)),
            "distance": distance_json(query::distance(&identity, &*s1.0, &p, &*s2.0).map(|d| d.distance)),
        },
    })
}

pub fn compound_internal_edges() -> Value {
    let prediction = Q::snap(IntegrationParameters::default().prediction_distance());
    let ball = b(ShapeSpec::ball(0.5));
    let cuboid = b(ShapeSpec::cuboid(0.25, 0.25));
    let polygon = b(ShapeSpec::polygon(&[(-0.25, -0.25), (0.25, -0.25), (0.25, 0.25), (-0.25, 0.25)]));
    // `(compound, other name, other, centre heights, x positions across the seam at x = 0.5)`:
    // the ball sinks 0.02 into the top, the boxes 0.01.
    let slides: Vec<(&str, &str, &Sh1Shape, f64, Vec<f64>, bool)> = vec![
        ("row2", "ball", &ball, 0.98, vec![0.3, 0.45, 0.5, 0.55, 0.7], true),
        ("row2", "cuboid", &cuboid, 0.74, vec![0.255, 0.5, 0.745], false),
        ("row3", "ball", &ball, 0.98, vec![0.45, 1.45, 1.55], false),
        ("poly2", "polygon", &polygon, 0.74, vec![0.255, 0.5, 0.745], false),
        ("poly2", "ball", &ball, 0.98, vec![0.45, 0.55], false),
    ];
    let (mut flagged, mut plain) = (Vec::new(), Vec::new());
    for (comp, name, other, y, xs, both_orders) in slides {
        for x in xs {
            let pos = QPose::new(qv(x, y), QRot::IDENTITY);
            let tag = format!("{comp}_{name}/x{}", (x * 1000.0).round() as i64);
            let orders: &[bool] = if both_orders { &[true, false] } else { &[true] };
            for first in orders {
                let id = if *first { tag.clone() } else { format!("{tag}/flip") };
                flagged.push(case(id.clone(), comp, other, pos, *first, true, prediction));
                plain.push(case(id, comp, other, pos, *first, false, prediction));
            }
        }
    }
    json!({
        "family": "compound_internal_edges",
        "prediction": jq(prediction),
        "compounds": compounds().iter().map(|(n, c)| json!({ "name": n, "shape": c.json() })).collect::<Vec<_>>(),
        "flagged": flagged,
        "plain": plain,
    })
}
