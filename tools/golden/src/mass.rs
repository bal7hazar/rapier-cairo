//! Family `mass_properties`: per-shape mass properties (parry) and a two-collider body (rapier).

use crate::q::{jf, jq, jqpose, jvec, QPose, QRot, QVec, Q};
use crate::shapes::ShapeSpec;
use rapier2d_f64::prelude::*;
use serde_json::{json, Value};

pub(crate) fn mprops_json(mp: &MassProperties) -> Value {
    json!({
        "mass": jf(mp.mass()),
        "inv_mass": jf(mp.inv_mass),
        "local_com": jvec(mp.local_com),
        "principal_inertia": jf(mp.principal_inertia()),
        "inv_principal_inertia": jf(mp.inv_principal_inertia),
    })
}

fn shape_case(id: &str, shape: ShapeSpec, density: f64) -> Value {
    let density = Q::snap(density);
    let mp = shape.shared().mass_properties(density.f());
    json!({
        "id": id,
        "shape": shape.json(),
        "density": jq(density),
        "expected": mprops_json(&mp),
    })
}

fn compound_case() -> Value {
    let body_pose = QPose::new(QVec::snap(1.0, 2.0), QRot::from_degrees(30.0));
    let parts = [
        (
            ShapeSpec::cuboid(0.5, 0.25),
            QPose::translation(-0.75, 0.0),
            Q::snap(1.0),
        ),
        (
            ShapeSpec::ball(0.5),
            QPose::new(QVec::snap(0.5, 0.25), QRot::from_degrees(45.0)),
            Q::snap(2.0),
        ),
    ];

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let handle = bodies.insert(RigidBodyBuilder::dynamic().pose(body_pose.p()));
    let mut expected_sum = MassProperties::new(Vector::ZERO, 0.0, 0.0);
    for (shape, local_pose, density) in parts {
        let collider = ColliderBuilder::new(shape.shared())
            .density(density.f())
            .position(local_pose.p());
        colliders.insert_with_parent(collider, handle, &mut bodies);
        expected_sum += shape
            .shared()
            .mass_properties(density.f())
            .transform_by(&local_pose.p());
    }
    let body = &mut bodies[handle];
    body.recompute_mass_properties_from_colliders(&colliders);
    let mp = body.mass_properties();

    // Rapier must agree with the plain parry sum of the transformed parts.
    assert!((mp.local_mprops.mass() - expected_sum.mass()).abs() < 1e-12);
    assert!((mp.local_mprops.local_com - expected_sum.local_com).length() < 1e-12);
    assert!((mp.local_mprops.principal_inertia() - expected_sum.principal_inertia()).abs() < 1e-12);

    let colliders_json: Vec<Value> = parts
        .iter()
        .map(|(shape, pose, density)| {
            json!({
                "shape": shape.json(),
                "pose_wrt_parent": jqpose(*pose),
                "density": jq(*density),
            })
        })
        .collect();

    json!({
        "id": "compound/cuboid_ball",
        "body_pose": jqpose(body_pose),
        "colliders": colliders_json,
        "expected": {
            "local": mprops_json(&mp.local_mprops),
            "world_com": jvec(mp.world_com),
            "effective_inv_mass": jvec(mp.effective_inv_mass),
            "effective_world_inv_inertia": jf(mp.effective_world_inv_inertia),
        },
    })
}

pub fn generate() -> Value {
    let shapes = vec![
        shape_case("ball/r0.5_d1", ShapeSpec::ball(0.5), 1.0),
        shape_case("ball/r1_d1", ShapeSpec::ball(1.0), 1.0),
        shape_case("ball/r2.5_d0.75", ShapeSpec::ball(2.5), 0.75),
        shape_case("ball/r0.05_d1", ShapeSpec::ball(0.05), 1.0),
        shape_case("cuboid/0.5x0.5_d1", ShapeSpec::cuboid(0.5, 0.5), 1.0),
        shape_case("cuboid/2x0.25_d3", ShapeSpec::cuboid(2.0, 0.25), 3.0),
        shape_case("cuboid/0.1x3_d0.3", ShapeSpec::cuboid(0.1, 3.0), 0.3),
        shape_case(
            "capsule/y0.5_r0.25_d1",
            ShapeSpec::capsule_y(0.5, 0.25),
            1.0,
        ),
        shape_case("capsule/x1_r0.5_d2", ShapeSpec::capsule_x(1.0, 0.5), 2.0),
        shape_case(
            "capsule/oblique_r0.3_d1.5",
            ShapeSpec::capsule((-0.5, -0.25), (1.0, 0.75), 0.3),
            1.5,
        ),
        shape_case(
            "capsule/zero_length_r0.5_d1",
            ShapeSpec::capsule((0.25, 0.25), (0.25, 0.25), 0.5),
            1.0,
        ),
    ];

    json!({
        "family": "mass_properties",
        "shapes": shapes,
        "polygons": ShapeSpec::polygons().into_iter().map(|(id,s)| shape_case(id,s,1.0)).collect::<Vec<_>>(),
        "bodies": [compound_case()],
    })
}
