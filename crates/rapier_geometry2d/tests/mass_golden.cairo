//! Golden-vector comparison of `MassProperties` against Parry / Rapier f64
//! (`rapier_golden::mass_properties`, `parry2d-f64 0.30.2`).
//!
//! Tolerances (raw Q32.32 units, `tools/golden/README.md`): mass, inertia and centre of mass
//! 16 ulp; the inverses `4 + 2 * inv^2` ulp with `inv` in real units (a 1-ulp error on a small
//! mass is amplified by `inv^2`, so this is a 2-ulp budget on the mass / inertia itself).
//! `mass()` and `principal_inertia()` read the *stored inverse* back (`1 / inv`), which carries the
//! same amplification the other way round (`x^2` ulp for a value `x`, no such error in f64): their
//! tolerance is `16 + x^2`.

use fixed::Fixed;
use glam::Vec2;
use rapier_geometry2d::mass::{MassProperties, MassPropertiesTrait};
use rapier_geometry2d::shape::{Ball, Capsule, Cuboid, HalfSpace, Segment, Shape, ShapeTrait};
use rapier_golden::compare::abs_diff;
use rapier_golden::mass_properties::{body_cases, cases};
use rapier_golden::types::{MassPropertiesRaw, PoseRaw, ShapeRaw, Vec2Raw};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;

fn vector(v: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: v.x }, y: Fixed { raw: v.y } }
}

fn shape(s: ShapeRaw) -> Shape {
    match s {
        ShapeRaw::Ball(r) => Shape::Ball(Ball { radius: Fixed { raw: r } }),
        ShapeRaw::Cuboid(h) => Shape::Cuboid(Cuboid { half_extents: vector(h) }),
        ShapeRaw::Capsule(c) => Shape::Capsule(
            Capsule {
                segment: Segment { a: vector(c.a), b: vector(c.b) },
                radius: Fixed { raw: c.radius },
            },
        ),
        ShapeRaw::HalfSpace(n) => Shape::HalfSpace(HalfSpace { normal: vector(n) }),
        ShapeRaw::Segment(s) => Shape::Segment(Segment { a: vector(s.a), b: vector(s.b) }),
    }
}

fn pose(p: PoseRaw) -> Pose2 {
    Pose2Trait::new(
        vector(p.translation),
        Rot2 { re: Fixed { raw: p.rotation.re }, im: Fixed { raw: p.rotation.im } },
    )
}

/// Tolerance of an inverse whose raw is `inv`: `4 + 2 * (inv / 2^32)^2` ulp.
fn inverse_tolerance(inv: i64) -> u64 {
    let raw: u128 = inv.try_into().unwrap();
    4 + (2 * raw * raw / 0x10000000000000000).try_into().unwrap()
}

/// Tolerance of a value whose raw is `x`, read back through its stored inverse: `16 + x^2` ulp.
fn roundtrip_tolerance(x: i64) -> u64 {
    let raw: u128 = x.try_into().unwrap();
    16 + (raw * raw / 0x10000000000000000).try_into().unwrap()
}

/// Per-field errors in ulp: mass, inv_mass, com.x, com.y, inertia, inv_inertia.
fn errors(actual: MassProperties, expected: MassPropertiesRaw) -> [u64; 6] {
    [
        abs_diff(actual.mass().raw, expected.mass),
        abs_diff(actual.inv_mass.raw, expected.inv_mass),
        abs_diff(actual.local_com.x.raw, expected.local_com.x),
        abs_diff(actual.local_com.y.raw, expected.local_com.y),
        abs_diff(actual.principal_inertia().raw, expected.principal_inertia),
        abs_diff(actual.inv_principal_inertia.raw, expected.inv_principal_inertia),
    ]
}

fn assert_within(index: u32, e: [u64; 6], expected: MassPropertiesRaw) {
    let [m, im, cx, cy, i, ii] = e;
    println!(
        "case {}: mass {} inv_mass {} com ({}, {}) inertia {} inv_inertia {}",
        index,
        m,
        im,
        cx,
        cy,
        i,
        ii,
    );
    assert!(m <= roundtrip_tolerance(expected.mass), "mass");
    assert!(im <= inverse_tolerance(expected.inv_mass), "inv_mass");
    assert!(cx <= 16 && cy <= 16, "local_com");
    assert!(i <= roundtrip_tolerance(expected.principal_inertia), "principal_inertia");
    assert!(ii <= inverse_tolerance(expected.inv_principal_inertia), "inv_principal_inertia");
}

#[test]
fn test_shape_mass_properties() {
    let mut index = 0;
    for c in cases() {
        let actual = shape(*c.shape).mass_properties(Fixed { raw: *c.density });
        assert_within(index, errors(actual, *c.expected), *c.expected);
        index += 1;
    }
}

#[test]
fn test_two_collider_body() {
    for c in body_cases() {
        let [c1, c2] = *c.colliders;
        let p1 = shape(c1.shape).mass_properties(Fixed { raw: c1.density });
        let p2 = shape(c2.shape).mass_properties(Fixed { raw: c2.density });
        let body = p1.transform_by(pose(c1.pose_wrt_parent))
            + p2.transform_by(pose(c2.pose_wrt_parent));
        assert_within(100, errors(body, *c.expected), *c.expected);
        // World quantities: 2D inverse inertia is rotation invariant, inverse mass is isotropic.
        let world_com = body.world_com(pose(*c.body_pose));
        assert!(abs_diff(world_com.x.raw, *c.world_com.x) <= 16, "world_com.x");
        assert!(abs_diff(world_com.y.raw, *c.world_com.y) <= 16, "world_com.y");
        assert!(
            abs_diff(
                body.inv_mass.raw, *c.effective_inv_mass.x,
            ) <= inverse_tolerance(*c.effective_inv_mass.x),
        );
        assert!(
            abs_diff(
                body.inv_principal_inertia.raw, *c.effective_world_inv_inertia,
            ) <= inverse_tolerance(*c.effective_world_inv_inertia),
        );
    }
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
fn gas_shape_mass_cases() {
    let c = opaque(*cases().at(9));
    let _ = shape(c.shape).mass_properties(Fixed { raw: c.density });
}

#[test]
fn gas_two_collider_body() {
    let c = opaque(*body_cases().at(0));
    let [c1, c2] = c.colliders;
    let p1 = shape(c1.shape).mass_properties(Fixed { raw: c1.density });
    let p2 = shape(c2.shape).mass_properties(Fixed { raw: c2.density });
    let _ = p1.transform_by(pose(c1.pose_wrt_parent)) + p2.transform_by(pose(c2.pose_wrt_parent));
}
