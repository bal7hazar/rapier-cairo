//! Golden replay of the collider through the builder, against the `rapier2d-f64` / `parry2d-f64`
//! vectors of `rapier_golden`.
//!
//! * every AABB case of `aabb.json` is rebuilt as a collider (`ColliderBuilder` + `position`) and
//!   compared with `Collider::compute_aabb`: exact for a quarter-turn pose, 4 ulp otherwise;
//! * every shape of `mass_properties.json` goes through `ColliderBuilder::density` and
//!   `Collider::mass_properties`, and the two-collider body is summed from the colliders with
//!   their poses wrt the parent;
//! * every collider of every scene of `scenes.json` is rebuilt from its description (shape, pose
//!   wrt the parent, density, friction, restitution) and compared with the golden case of the same
//!   shape when there is one, with the analytic value (`pi r^2`, `4 hx hy`) otherwise.
//!
//! Mass tolerances follow `tools/golden/README.md` as applied in `rapier_geometry2d`'s
//! `mass_golden`: `4 + 2 inv^2` ulp on a stored inverse, 16 ulp on a centre of mass.

use fixed::{Fixed, FixedTrait, PI};
use glam::Vec2;
use rapier_core::collider::ColliderChangesTrait;
use rapier_dynamics2d::collider::{Collider, ColliderBuilder, ColliderBuilderTrait, ColliderTrait};
use rapier_geometry2d::mass::{MassProperties, MassPropertiesTrait};
use rapier_golden::compare::{abs_diff, vec2_within};
use rapier_golden::types::{
    AabbCase, MassPropertiesRaw, PoseRaw, SceneBodyRaw, SceneColliderRaw, ShapeRaw, Vec2Raw,
};
use rapier_golden::{aabb, mass_properties, scenes};
use rapier_math::inv;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use rapier_testing::opaque;

fn vector(v: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: v.x }, y: Fixed { raw: v.y } }
}

fn raw(v: Vec2) -> Vec2Raw {
    Vec2Raw { x: v.x.raw, y: v.y.raw }
}

/// The pose as the engine holds it: the rotation of a golden vector is renormalised.
fn pose(p: PoseRaw) -> Pose2 {
    let rotation = Rot2Trait::from_cos_sin(
        Fixed { raw: p.rotation.re }, Fixed { raw: p.rotation.im },
    );
    Pose2Trait::new(vector(p.translation), rotation)
}

/// The builder of the collider described by `shape`, through the named constructors.
fn builder(shape: ShapeRaw) -> ColliderBuilder {
    match shape {
        ShapeRaw::Ball(r) => ColliderBuilderTrait::ball(Fixed { raw: r }),
        ShapeRaw::Cuboid(h) => ColliderBuilderTrait::cuboid(Fixed { raw: h.x }, Fixed { raw: h.y }),
        ShapeRaw::Capsule(c) => ColliderBuilderTrait::capsule_from_endpoints(
            vector(c.a), vector(c.b), Fixed { raw: c.radius },
        ),
        ShapeRaw::HalfSpace(n) => ColliderBuilderTrait::halfspace(vector(n)),
        ShapeRaw::Segment(s) => ColliderBuilderTrait::segment(vector(s.a), vector(s.b)),
    }
}

/// Tolerance of an inverse whose raw is `inv`: `4 + 2 * (inv / 2^32)^2` ulp.
fn inverse_tolerance(inv: i64) -> u64 {
    let raw: u128 = inv.try_into().unwrap();
    4 + (2 * raw * raw / 0x10000000000000000).try_into().unwrap()
}

fn assert_mass_properties(actual: MassProperties, expected: MassPropertiesRaw, id: felt252) {
    assert!(
        abs_diff(actual.inv_mass.raw, expected.inv_mass) <= inverse_tolerance(expected.inv_mass),
        "inv_mass {}",
        id,
    );
    assert!(
        abs_diff(
            actual.inv_principal_inertia.raw, expected.inv_principal_inertia,
        ) <= inverse_tolerance(expected.inv_principal_inertia),
        "inv_principal_inertia {}",
        id,
    );
    assert!(vec2_within(raw(actual.local_com), expected.local_com, 16), "local_com {}", id);
}

/// `0` for a quarter-turn pose (the box must be exact), `4` ulp otherwise.
fn aabb_tolerance(p: PoseRaw) -> u64 {
    if p.rotation.re == 0 || p.rotation.im == 0 {
        0
    } else {
        4
    }
}

fn assert_aabb(collider: Collider, mins: Vec2Raw, maxs: Vec2Raw, tolerance: u64, id: felt252) {
    let aabb = collider.compute_aabb();
    assert!(vec2_within(raw(aabb.mins), mins, tolerance), "mins {}", id);
    assert!(vec2_within(raw(aabb.maxs), maxs, tolerance), "maxs {}", id);
}

#[test]
fn test_aabb_cases_through_the_collider() {
    for case in aabb::cases() {
        let case: AabbCase = *case;
        let collider = builder(case.shape).position(pose(case.pose)).build();
        assert_eq!(collider.position(), pose(case.pose));
        assert_aabb(collider, case.mins, case.maxs, aabb_tolerance(case.pose), case.id);
        // Moving the collider moves the box with it (`set_position`, as the collider set does).
        let mut moved = builder(case.shape).build();
        moved.set_position(pose(case.pose));
        assert_eq!(moved.compute_aabb(), collider.compute_aabb());
    }
}

#[test]
fn test_mass_cases_through_the_collider() {
    for case in mass_properties::cases() {
        let case = *case;
        let collider = builder(case.shape).density(Fixed { raw: case.density }).build();
        assert_mass_properties(collider.mass_properties(), case.expected, case.id);
        // The stored specification is the density, and `density()` returns it as is.
        assert_eq!(collider.density().raw, case.density);
    }
}

/// The two-collider body of `mass_properties.json`: the local mass properties of the body are
/// the sum of the collider properties moved by their poses wrt the parent.
#[test]
fn test_compound_body_from_its_colliders() {
    for case in mass_properties::body_cases() {
        let case = *case;
        let [first, second] = case.colliders;
        let mut total: MassProperties = Default::default();
        for weighted in array![first, second].span() {
            let weighted = *weighted;
            let collider = builder(weighted.shape)
                .density(Fixed { raw: weighted.density })
                .position(pose(weighted.pose_wrt_parent))
                .build();
            total = total + collider.mass_properties().transform_by(collider.position());
        }
        assert_mass_properties(total, case.expected, case.id);
    }
}

/// A collider of a scene, rebuilt from its description.
fn scene_collider(description: SceneColliderRaw) -> Collider {
    builder(description.shape)
        .density(Fixed { raw: description.density })
        .friction(Fixed { raw: description.friction })
        .restitution(Fixed { raw: description.restitution })
        .position(pose(description.pose_wrt_parent))
        .build()
}

/// The golden mass properties of `shape` at `density`, if `mass_properties.json` has the case.
fn golden_mass(shape: ShapeRaw, density: i64) -> Option<MassPropertiesRaw> {
    let mut found = None;
    for case in mass_properties::cases() {
        if (*case.shape) == shape && *case.density == density {
            found = Some(*case.expected);
        }
    }
    found
}

/// The golden box of `shape` at `pose`, if `aabb.json` has the case.
fn golden_aabb(shape: ShapeRaw, pose: PoseRaw) -> Option<AabbCase> {
    let mut found = None;
    for case in aabb::cases() {
        if (*case.shape) == shape && (*case.pose) == pose {
            found = Some(*case);
        }
    }
    found
}

/// Analytic mass properties of a disc or a box of unit-free density `d`, at the origin:
/// the fallback when no golden case exists.
fn analytic_mass(shape: ShapeRaw, density: i64) -> MassPropertiesRaw {
    let d = Fixed { raw: density };
    let (mass, inertia) = match shape {
        ShapeRaw::Ball(r) => {
            let r = Fixed { raw: r };
            let mass = PI * r * r * d;
            (mass, mass * r * r / Fixed { raw: 2 * 4294967296 })
        },
        ShapeRaw::Cuboid(h) => {
            let (hx, hy) = (Fixed { raw: h.x }, Fixed { raw: h.y });
            let mass = Fixed { raw: 4 * 4294967296 } * hx * hy * d;
            (mass, mass * (hx * hx + hy * hy) / Fixed { raw: 3 * 4294967296 })
        },
        _ => panic!("no analytic mass for this shape"),
    };
    MassPropertiesRaw {
        mass: mass.raw,
        inv_mass: inv(mass).raw,
        local_com: Vec2Raw { x: 0, y: 0 },
        principal_inertia: inertia.raw,
        inv_principal_inertia: inv(inertia).raw,
    }
}

/// Analytic box of a disc or a box at `pose`: `translation +- |R| h`.
fn analytic_aabb(shape: ShapeRaw, pose: Pose2) -> (Vec2Raw, Vec2Raw) {
    let half = match shape {
        ShapeRaw::Ball(r) => Vec2 { x: Fixed { raw: r }, y: Fixed { raw: r } },
        ShapeRaw::Cuboid(h) => {
            let (c, s) = (pose.rotation.re.abs(), pose.rotation.im.abs());
            let (hx, hy) = (Fixed { raw: h.x }, Fixed { raw: h.y });
            Vec2 { x: c * hx + s * hy, y: s * hx + c * hy }
        },
        _ => panic!("no analytic box for this shape"),
    };
    let t = pose.translation;
    (
        Vec2Raw { x: t.x.raw - half.x.raw, y: t.y.raw - half.y.raw },
        Vec2Raw { x: t.x.raw + half.x.raw, y: t.y.raw + half.y.raw },
    )
}

/// Every collider of every scene, attached at its parent's pose: the builder keeps the
/// description, the mass properties match the golden case (or the analytic value) and the world
/// box is the golden one (or the analytic one).
#[test]
fn test_scene_colliders_through_the_builder() {
    let mut checked: u32 = 0;
    for scene in scenes::ALL.span() {
        let scene = *scene;
        let mut index: u32 = 0;
        for body in scene.bodies.span() {
            let body: SceneBodyRaw = *body;
            index += 1;
            if index > scene.num_bodies || body.num_colliders == 0 {
                continue;
            }
            let description: SceneColliderRaw = *body.colliders.span()[0];
            let mut collider = scene_collider(description);
            // The description survives the builder.
            assert_eq!(collider.friction().raw, description.friction);
            assert_eq!(collider.restitution().raw, description.restitution);
            assert_eq!(collider.density().raw, description.density);
            assert_eq!(collider.position(), pose(description.pose_wrt_parent));
            assert!(collider.parent().is_none() && collider.is_enabled() && !collider.is_sensor());
            assert_eq!(collider.changes, ColliderChangesTrait::all());
            // Local mass properties.
            let expected = match golden_mass(description.shape, description.density) {
                Some(golden) => golden,
                None => analytic_mass(description.shape, description.density),
            };
            let mprops = collider.mass_properties();
            // The analytic values carry a few more ulp of rounding than the golden ones: allow
            // the tolerance of an inverse a factor 4 more.
            let tolerance = 4 * inverse_tolerance(expected.inv_mass);
            assert!(abs_diff(mprops.inv_mass.raw, expected.inv_mass) <= tolerance, "inv_mass");
            let tolerance = 4 * inverse_tolerance(expected.inv_principal_inertia);
            assert!(
                abs_diff(
                    mprops.inv_principal_inertia.raw, expected.inv_principal_inertia,
                ) <= tolerance,
                "inv_principal_inertia",
            );
            assert_eq!(mprops.local_com, Vec2 { x: Fixed { raw: 0 }, y: Fixed { raw: 0 } });
            // World box: the collider sits at `body pose * pose wrt parent`.
            let world = pose(body.pose) * pose(description.pose_wrt_parent);
            collider.set_position(world);
            let aabb = collider.compute_aabb();
            let (mins, maxs) = match golden_aabb(description.shape, body.pose) {
                Some(golden) => (golden.mins, golden.maxs),
                None => analytic_aabb(description.shape, world),
            };
            assert!(vec2_within(raw(aabb.mins), mins, 4), "mins");
            assert!(vec2_within(raw(aabb.maxs), maxs, 4), "maxs");
            checked += 1;
        }
    }
    // ball_drop, ball_bounce: ground + ball; slope stick and slide: slope + box; stack: ground
    // + three boxes; pendulum: the bob (the pivot has no collider).
    // Six original scenes, then the two SL sleep scenes (4 and 3 colliders).
    assert_eq!(checked, 2 + 2 + 2 + 2 + 4 + 1 + 4 + 3);
}

/// The mass of a collider built by `mass` keeps the golden inertia-to-mass ratio of its shape.
#[test]
fn test_mass_specification_through_the_collider() {
    // The unit-density 0.5-ball is a golden case: giving its own mass back is a no-op.
    let case = mass_properties::BALL_R0_5_D1;
    let by_density = builder(case.shape).density(Fixed { raw: case.density }).build();
    let by_mass = builder(case.shape).mass(Fixed { raw: case.expected.mass }).build();
    assert_eq!(by_mass.mass().raw, case.expected.mass);
    let (a, b) = (by_density.mass_properties(), by_mass.mass_properties());
    assert!(abs_diff(a.inv_mass.raw, b.inv_mass.raw) <= 2 * inverse_tolerance(a.inv_mass.raw));
    assert!(
        abs_diff(a.inv_principal_inertia.raw, b.inv_principal_inertia.raw) <= 2
            * inverse_tolerance(a.inv_principal_inertia.raw),
    );
    // The explicit properties are returned untouched.
    let explicit = builder(case.shape).mass_properties(a).build();
    assert_eq!(explicit.mass_properties(), a);
}

/// The box of a rotated cuboid collider contains its four corners and touches its four sides
/// (up to a few ulp of rounding in the pose product and in `|R| h`).
#[test]
#[fuzzer(runs: 64, seed: 20260920)]
fn fuzz_rotated_cuboid_box_is_tight(hx: u16, hy: u16, re: i16, im: i16, tx: i16, ty: i16) {
    let hx = Fixed { raw: (hx.into() % 4096 + 1) * 1048576 };
    let hy = Fixed { raw: (hy.into() % 4096 + 1) * 1048576 };
    if re == 0 && im == 0 {
        return;
    }
    let rotation = Rot2Trait::from_cos_sin(
        Fixed { raw: re.into() * 65536 }, Fixed { raw: im.into() * 65536 },
    );
    let translation = Vec2 {
        x: Fixed { raw: tx.into() * 65536 }, y: Fixed { raw: ty.into() * 65536 },
    };
    let position = Pose2Trait::new(translation, rotation);
    let collider = ColliderBuilderTrait::cuboid(hx, hy).position(position).build();
    let aabb = collider.compute_aabb();
    let corners = array![
        Vec2 { x: hx, y: hy }, Vec2 { x: -hx, y: hy }, Vec2 { x: hx, y: -hy },
        Vec2 { x: -hx, y: -hy },
    ];
    let slack: i64 = 64;
    let mut touches_min_x = false;
    let mut touches_max_x = false;
    let mut touches_min_y = false;
    let mut touches_max_y = false;
    for corner in corners.span() {
        let p = position.transform_point(*corner);
        assert!(p.x.raw >= aabb.mins.x.raw - slack && p.x.raw <= aabb.maxs.x.raw + slack);
        assert!(p.y.raw >= aabb.mins.y.raw - slack && p.y.raw <= aabb.maxs.y.raw + slack);
        touches_min_x = touches_min_x || p.x.raw - aabb.mins.x.raw <= slack;
        touches_max_x = touches_max_x || aabb.maxs.x.raw - p.x.raw <= slack;
        touches_min_y = touches_min_y || p.y.raw - aabb.mins.y.raw <= slack;
        touches_max_y = touches_max_y || aabb.maxs.y.raw - p.y.raw <= slack;
    }
    assert!(touches_min_x && touches_max_x && touches_min_y && touches_max_y);
}

#[test]
fn gas_baseline() {}

#[test]
fn gas_opaque_collider() {
    let _ = opaque(builder(mass_properties::BALL_R0_5_D1.shape).build());
}

#[test]
fn gas_build_from_golden() {
    let case = mass_properties::CUBOID_2X0_25_D3;
    let _ = builder(opaque(case.shape)).density(opaque(Fixed { raw: case.density })).build();
}

#[test]
fn gas_mass_properties_from_golden() {
    let case = mass_properties::CAPSULE_OBLIQUE_R0_3_D1_5;
    let collider = builder(case.shape).density(Fixed { raw: case.density }).build();
    assert!(opaque(collider).mass_properties().inv_mass.raw != 0);
}

#[test]
fn gas_compute_aabb_from_golden() {
    let case = aabb::CUBOID_ROT30;
    let collider = builder(case.shape).position(pose(case.pose)).build();
    assert!(opaque(collider).compute_aabb().maxs.x.raw != 0);
}
