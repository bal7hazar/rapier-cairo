//! Golden-vector comparison of `Shape::compute_aabb` against Parry f64
//! (`rapier_golden::aabb`: 4 shapes x 8 poses).
//!
//! The pose is rebuilt from `PoseRaw` through `Rot2::from_cos_sin` (renormalised, as the engine
//! does). Tolerance (`tools/golden/README.md`): exact for quarter-turn poses, 4 ulp otherwise
//! (`|R| * h` is two products and a sum per axis).

use fixed::Fixed;
use glam::Vec2;
use rapier_geometry2d::aabb::Aabb;
use rapier_geometry2d::shape::{Ball, Capsule, Cuboid, HalfSpace, Segment, Shape, ShapeTrait};
use rapier_golden::aabb::cases;
use rapier_golden::compare::{abs_diff, vec2_within};
use rapier_golden::types::{PoseRaw, ShapeRaw, Vec2Raw};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use rapier_testing::opaque;

fn vector(v: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: v.x }, y: Fixed { raw: v.y } }
}

fn raw(v: Vec2) -> Vec2Raw {
    Vec2Raw { x: v.x.raw, y: v.y.raw }
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
    let rotation = Rot2Trait::from_cos_sin(
        Fixed { raw: p.rotation.re }, Fixed { raw: p.rotation.im },
    );
    Pose2Trait::new(vector(p.translation), rotation)
}

/// `0` for a quarter-turn pose (the box must be exact), `4` ulp otherwise.
fn tolerance(p: PoseRaw) -> u64 {
    if p.rotation.re == 0 || p.rotation.im == 0 {
        0
    } else {
        4
    }
}

fn max4(a: u64, b: u64, c: u64, d: u64) -> u64 {
    let ab = if a > b {
        a
    } else {
        b
    };
    let cd = if c > d {
        c
    } else {
        d
    };
    if ab > cd {
        ab
    } else {
        cd
    }
}

#[test]
fn test_all_shape_aabbs() {
    let mut index = 0;
    for c in cases() {
        let aabb: Aabb = shape(*c.shape).compute_aabb(pose(*c.pose));
        let (mins, maxs) = (raw(aabb.mins), raw(aabb.maxs));
        let t = tolerance(*c.pose);
        assert!(vec2_within(mins, *c.mins, t), "mins {}", *c.id);
        assert!(vec2_within(maxs, *c.maxs, t), "maxs {}", *c.id);
        // Worst error of the case in ulp, for the report.
        let worst = max4(
            abs_diff(mins.x, *c.mins.x),
            abs_diff(mins.y, *c.mins.y),
            abs_diff(maxs.x, *c.maxs.x),
            abs_diff(maxs.y, *c.maxs.y),
        );
        println!("aabb case {}: {} ulp", index, worst);
        index += 1;
    }
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
fn gas_all_shape_aabbs() {
    for c in cases() {
        let _ = shape(opaque(*c.shape)).compute_aabb(pose(opaque(*c.pose)));
    }
}
