//! Rejected broad-phase-assisted variants of `super::cast_ray`, kept for the `gas_*` ranking
//! (`super::benches`), and of `super::intersect_shape` (`super::benches_qy2`). Every variant
//! returns exactly what the brute-force scan returns.

use fixed::Fixed;
use glam::vec2::Vec2Trait;
use rapier_core::Handle;
use rapier_dynamics2d::collider::ColliderTrait;
use rapier_geometry2d::aabb::{Aabb, AabbTrait};
use rapier_geometry2d::broad_phase::{BroadPhaseProxy, find_pairs};
use rapier_geometry2d::query::intersection_test;
use rapier_geometry2d::ray::{Ray, RayTrait, cast_local_ray_cuboid, cast_ray};
use rapier_geometry2d::shape::{Cuboid, Shape};
use rapier_math::pose2::Pose2;
use crate::world::World;
use super::{QueryFilter, candidates};

/// `true` when `ray` meets `aabb` at a time `<= bound` (solid slab test on the box).
#[inline(always)]
fn ray_meets_aabb(aabb: Aabb, ray: Ray, bound: Fixed) -> bool {
    let local = Ray { origin: ray.origin - aabb.center(), dir: ray.dir };
    cast_local_ray_cuboid(Cuboid { half_extents: aabb.half_extents() }, local, bound, true)
        .is_some()
}

/// Brute force with an AABB pre-test: the exact cast only runs on colliders whose world AABB the
/// ray reaches before the best time so far.
pub fn cast_ray_aabb_pretest(
    ref world: World, ray: Ray, max_toi: Fixed, solid: bool, filter: QueryFilter,
) -> Option<(Handle, Fixed)> {
    let mut best: Option<(Handle, Fixed)> = None;
    let mut bound = max_toi;
    for (handle, collider) in candidates(ref world, filter) {
        if !ray_meets_aabb(collider.compute_aabb(), ray, bound) {
            continue;
        }
        if let Some(t) = cast_ray(collider.shape, collider.position(), ray, bound, solid) {
            if t < bound {
                bound = t;
                best = Some((handle, t));
            }
        }
    }
    best
}

/// BG's grid: every collider is a static proxy and the ray's AABB (up to `max_toi`) the only
/// dynamic one, so `find_pairs` emits exactly the colliders whose AABB overlaps the ray's.
pub fn cast_ray_grid(
    ref world: World, ray: Ray, max_toi: Fixed, solid: bool, filter: QueryFilter,
) -> Option<(Handle, Fixed)> {
    let colliders = candidates(ref world, filter);
    let mut proxies = array![];
    for (handle, collider) in colliders.span() {
        proxies
            .append(
                BroadPhaseProxy {
                    collider: *handle, aabb: collider.compute_aabb(), is_static: true,
                },
            );
    }
    let end = ray.point_at(max_toi);
    let ray_box = AabbTrait::new(ray.origin.min(end), ray.origin.max(end));
    let ray_index = colliders.len();
    proxies
        .append(BroadPhaseProxy { collider: Default::default(), aabb: ray_box, is_static: false });
    let mut best: Option<(Handle, Fixed)> = None;
    let mut bound = max_toi;
    // Pairs come sorted by `(i, j)`, so candidates are visited in ascending handle order.
    for (i, j) in find_pairs(proxies.span()) {
        if j != ray_index {
            continue;
        }
        let (handle, collider) = *colliders.at(i);
        if let Some(t) = cast_ray(collider.shape, collider.position(), ray, bound, solid) {
            if t < bound {
                bound = t;
                best = Some((handle, t));
            }
        }
    }
    best
}

/// `intersect_shape` without the broad-phase test: the exact kernel runs on every candidate.
pub fn intersect_shape_direct(
    ref world: World, shape_pos: Pose2, shape: Shape, filter: QueryFilter,
) -> Array<Handle> {
    let mut out = array![];
    for (handle, collider) in candidates(ref world, filter) {
        if intersection_test(shape_pos, shape, collider.position(), collider.shape) == Some(true) {
            out.append(handle);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait};
    use glam::vec2::Vec2;
    use rapier_dynamics2d::collider::ColliderBuilderTrait;
    use rapier_geometry2d::ray::Ray;
    use rapier_geometry2d::shape::{Ball, Shape};
    use rapier_math::pose2::Pose2Trait;
    use rapier_math::rot2::Rot2;
    use crate::world::{World, WorldTrait};
    use super::super::{QueryFilterTrait, cast_ray, intersect_shape};
    use super::{cast_ray_aabb_pretest, cast_ray_grid, intersect_shape_direct};

    /// 70 mixed shapes (balls and boxes) on a jittered grid: enough for BG's grid path.
    fn scene() -> World {
        let mut world = WorldTrait::new(
            Vec2 { x: FixedTrait::from_int(0), y: FixedTrait::from_int(-1) }, Default::default(),
        );
        let mut i: u32 = 0;
        while i != 70 {
            let x: i64 = (i % 10).into() * 0x1_4000_0000 + (i % 3).into() * 0x1000_0000;
            let y: i64 = (i / 10).into() * 0x1_2000_0000;
            let builder = if i % 2 == 0 {
                ColliderBuilderTrait::ball(Fixed { raw: 0x6000_0000 })
            } else {
                ColliderBuilderTrait::cuboid(Fixed { raw: 0x5000_0000 }, Fixed { raw: 0x3000_0000 })
            };
            let _ = world
                .insert_collider(
                    builder.translation(Vec2 { x: Fixed { raw: x }, y: Fixed { raw: y } }).build(),
                    None,
                );
            i += 1;
        }
        world
    }

    #[test]
    #[fuzzer(runs: 16, seed: 9)]
    fn fuzz_assisted_variants_match_brute_force(ox: i16, oy: i16, dx: i8, dy: i8) {
        let mut world = scene();
        let ray = Ray {
            origin: Vec2 {
                x: Fixed { raw: ox.into() * 0x8_0000 }, y: Fixed { raw: oy.into() * 0x8_0000 },
            },
            dir: Vec2 {
                x: Fixed { raw: dx.into() * 0x100_0000 }, y: Fixed { raw: dy.into() * 0x100_0000 },
            },
        };
        let max = FixedTrait::from_int(40);
        let filter = QueryFilterTrait::new();
        let brute = cast_ray(ref world, ray, max, true, filter);
        assert_eq!(cast_ray_aabb_pretest(ref world, ray, max, true, filter), brute);
        assert_eq!(cast_ray_grid(ref world, ray, max, true, filter), brute);
    }

    /// The AABB pre-test of `intersect_shape` never drops an intersection.
    #[test]
    fn test_intersect_shape_direct_matches_the_winner() {
        let mut world = scene();
        let unit = Rot2 { re: FixedTrait::from_int(1), im: FixedTrait::from_int(0) };
        let filter = QueryFilterTrait::new();
        // (x, y, radius) in 1/16 units: on a collider, between two, touching one, far away.
        let cases = array![
            (0_i64, 0_i64, 0x8_i64), (0x14, 0x12, 0x10), (0x28, 0x20, 0x4), (0xA0, 0x90, 0x1),
            (0x3E, 0x24, 0x30), (0x1000, 0x1000, 0x10),
        ];
        for (x, y, r) in cases {
            let pos = Pose2Trait::new(
                Vec2 { x: Fixed { raw: x * 0x1000_0000 }, y: Fixed { raw: y * 0x1000_0000 } }, unit,
            );
            let shape = Shape::Ball(Ball { radius: Fixed { raw: r * 0x1000_0000 } });
            assert_eq!(
                intersect_shape_direct(ref world, pos, shape, filter),
                intersect_shape(ref world, pos, shape, filter),
            );
        }
    }
}
