//! Test-only [`ContactDispatcher`]: exact ball–ball, halfspace–ball and axis-aligned
//! halfspace–cuboid manifolds, with upstream's warm-start matching (`match_contacts`). Stands in
//! for work package GG until it lands. Points are generated only within `prediction`, as Parry's
//! generators do.

use fixed::{Fixed, FixedTrait, ONE, ZERO};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::collider::events::COLLISION_EVENTS;
use rapier_geometry2d::broad_phase::find_pairs;
use rapier_geometry2d::contact::{ContactManifold, TrackedContact};
use rapier_geometry2d::feature_id::FeatureIdTrait;
use rapier_geometry2d::manifold::ManifoldTrait;
use rapier_geometry2d::shape::Shape;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::{IDENTITY, Rot2Trait};
use crate::collider::ColliderBuilderTrait;
use crate::collider_set::{ColliderSet, ColliderSetTrait};
use crate::rigid_body_set::{RigidBodySet, RigidBodySetTrait, RigidBodyTrait};
use super::ContactDispatcher;

pub impl MockDispatcher of ContactDispatcher {
    fn contact_manifold(
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) -> bool {
        let old = manifold;
        let mut new: ContactManifold = Default::default();
        new.data = old.data;
        let supported = match shape1 {
            Shape::Ball(b1) => match shape2 {
                Shape::Ball(b2) => {
                    ball_ball(ref new, pos12, b1.radius, b2.radius, prediction);
                    true
                },
                _ => false,
            },
            Shape::HalfSpace(h) => match shape2 {
                Shape::Ball(b) => {
                    halfspace_ball(ref new, pos12, h.normal, b.radius, prediction);
                    true
                },
                Shape::Cuboid(c) => {
                    halfspace_cuboid(ref new, pos12, h.normal, c.half_extents, prediction);
                    true
                },
                _ => false,
            },
            _ => false,
        };
        if supported {
            new.match_contacts(@old);
            manifold = new;
        }
        supported
    }
}

fn scale(v: Vec2, s: Fixed) -> Vec2 {
    Vec2 { x: v.x * s, y: v.y * s }
}

fn dot(a: Vec2, b: Vec2) -> Fixed {
    a.x * b.x + a.y * b.y
}

fn point(local_p1: Vec2, local_p2: Vec2, dist: Fixed, fid2: u32) -> TrackedContact {
    TrackedContact {
        local_p1,
        local_p2,
        dist,
        fid1: FeatureIdTrait::face(0),
        fid2: FeatureIdTrait::vertex(fid2),
        data: Default::default(),
    }
}

fn ball_ball(ref m: ContactManifold, pos12: Pose2, r1: Fixed, r2: Fixed, prediction: Fixed) {
    let t = pos12.translation;
    let d = dot(t, t).sqrt();
    let n1 = scale(t, ONE / d);
    let dist = d - r1 - r2;
    let n2 = -pos12.rotation.inverse_rotate(n1);
    m.local_n1 = n1;
    m.local_n2 = n2;
    if dist <= prediction {
        m.points = [point(scale(n1, r1), scale(n2, r2), dist, 0), Default::default()];
        m.num_points = 1;
    }
}

fn halfspace_ball(ref m: ContactManifold, pos12: Pose2, n: Vec2, r: Fixed, prediction: Fixed) {
    let c = pos12.translation;
    let depth = dot(c, n);
    let n2 = -pos12.rotation.inverse_rotate(n);
    m.local_n1 = n;
    m.local_n2 = n2;
    let dist = depth - r;
    if dist <= prediction {
        m.points = [point(c - scale(n, depth), scale(n2, r), dist, 0), Default::default()];
        m.num_points = 1;
    }
}

/// Assumes the cuboid is axis-aligned with the halfspace normal (bottom face towards it).
fn halfspace_cuboid(ref m: ContactManifold, pos12: Pose2, n: Vec2, h: Vec2, prediction: Fixed) {
    let n2 = -pos12.rotation.inverse_rotate(n);
    m.local_n1 = n;
    m.local_n2 = n2;
    // The two vertices of the face of normal `n2` (the face whose normal is -n in frame 1).
    let tangent = Vec2 { x: -n2.y, y: n2.x };
    let base = Vec2 { x: n2.x * h.x, y: n2.y * h.y };
    let side = Vec2 { x: tangent.x * h.x, y: tangent.y * h.y };
    let vertices = array![(base - side, 0_u32), (base + side, 1_u32)];
    let mut points: Array<TrackedContact> = array![];
    for (v, id) in vertices {
        let p = pos12.transform_point(v);
        let dist = dot(p, n);
        if dist <= prediction {
            points.append(point(p - scale(n, dist), v, dist, id));
        }
    }
    let n_points = points.len();
    if n_points == 2 {
        m.points = [*points.at(0), *points.at(1)];
    } else if n_points == 1 {
        m.points = [*points.at(0), Default::default()];
    }
    m.num_points = n_points.try_into().unwrap();
}

/// Upstream's default prediction distance, `0.002` (raw Q32.32).
pub const PREDICTION: Fixed = Fixed { raw: 8589935 };

/// Ball radius of the fixture scenes.
pub const RADIUS: Fixed = Fixed { raw: 0x80000000 };

/// Penetration of the resting contacts of the fixture scenes, `1/256`.
pub const OVERLAP: Fixed = Fixed { raw: 0x1000000 };

/// A translation-only pose.
pub fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2 { translation: Vec2 { x, y }, rotation: IDENTITY }
}

/// A fixed ground body carrying an upward halfspace at `y = 0`.
pub fn ground(ref bodies: RigidBodySet, ref colliders: ColliderSet) -> (Handle, Handle) {
    let body = bodies.insert(RigidBodyTrait::fixed(at(ZERO, ZERO)));
    let collider = colliders
        .insert_with_parent(
            ColliderBuilderTrait::halfspace(Vec2 { x: ZERO, y: ONE })
                .active_events(COLLISION_EVENTS)
                .build(),
            body,
            ref bodies,
        );
    (body, collider)
}

/// A dynamic body at `(x, y)` carrying a ball of radius [`RADIUS`].
pub fn ball(ref bodies: RigidBodySet, ref colliders: ColliderSet, x: Fixed, y: Fixed) -> Handle {
    let body = bodies.insert(RigidBodyTrait::dynamic(at(x, y)));
    colliders.insert_with_parent(ColliderBuilderTrait::ball(RADIUS).build(), body, ref bodies)
}

/// `n` balls on the ground: stacked on each other (`stack`) each penetrating its support by
/// [`OVERLAP`], or side by side two units apart, each penetrating the ground by [`OVERLAP`].
pub fn scene(n: u32, stack: bool) -> (RigidBodySet, ColliderSet) {
    let mut bodies = RigidBodySetTrait::new();
    let mut colliders = ColliderSetTrait::new();
    let _ = ground(ref bodies, ref colliders);
    let step = ONE - OVERLAP;
    let mut i: u32 = 0;
    let mut offset = RADIUS - OVERLAP;
    while i != n {
        let k = FixedTrait::from_int(i.try_into().unwrap());
        if stack {
            let _ = ball(ref bodies, ref colliders, ZERO, offset);
            offset = offset + step;
        } else {
            let _ = ball(ref bodies, ref colliders, k + k, offset);
        }
        i += 1;
    }
    (bodies, colliders)
}

/// Broad phase of the current poses: proxies loosened by `prediction / 2`, then `find_pairs`.
pub fn broad_phase(
    ref bodies: RigidBodySet, ref colliders: ColliderSet, prediction: Fixed,
) -> Array<(u32, u32)> {
    let proxies = colliders.broad_phase_proxies(ref bodies, prediction);
    find_pairs(proxies.span())
}
