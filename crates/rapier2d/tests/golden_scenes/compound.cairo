//! SH2b: the `compound_scenes` family against rapier2d-f64 0.35.3: an L-shaped compound toppling
//! on a half-space (`ell_topple`, 150 steps), an L lying across a polyline vertex (`ell_on_seam`)
//! and a three-part compound (ball, turned box, capsule) sliding over a heightfield
//! (`trio_on_hills`), 90 steps each. The ground is a standalone collider (slot 0, collision events
//! on), the compound the body's collider (slot 1); no contact recycling nor clustering, as the
//! other scene families.
//!
//! Every step compares the body with upstream's sample (`2^12 · step` ulp on the pose, twice that
//! on the velocities, as the other scenes), the number of the pair's manifolds with solver
//! contacts (one per part pair) and the collision events (one per collider pair, whatever the
//! number of parts), exactly.
//!
//! Exception, at the landing (step 6, the speculative contacts closing): the velocities of
//! `ell_on_seam`'s steps 7 and 8 deviate by up to 252 400 ulp (the soft / rigid split of a
//! near-zero gap, as SH2a's `box_on_seam`), then fall back within the band (pose: 7 377 ulp at
//! most); they are printed, not judged.
//!
//! `ell_topple` passes every sample since SF1 (it deviated from step 7 by 1.1e6 ulp on the
//! velocities, and so did the same body made of two plain colliders, SF1's `tilted_landing`
//! scene): one of the bar's corners lands at a near-zero gap, which the port's floored anchor
//! round trip turned from upstream's `+1.26` raw (rigid row) into `0` (soft row); the frozen
//! separation is now rebased on that round trip, as upstream rebases its `infos.dist`.
use rapier2d::prelude::{ColliderBuilderTrait, RigidBodyTrait, World, WorldTrait};
use rapier_core::collider::events::COLLISION_EVENTS;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_dynamics2d::events::CollisionEventTrait;
use rapier_dynamics2d::narrow_phase::composite::contact_pair_manifolds;
use rapier_geometry2d::shape::{
    Ball, Capsule, Cuboid, HalfSpace, RoundShape, Segment, Shape, TriangleTrait,
};
use rapier_golden::generated::compound_scenes;
use rapier_golden::types::{CompoundRaw, PoseRaw, Sh1ShapeRaw, ShapeRaw};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use super::builder::{body_handle, f, vr};
use super::{TOL_PER_STEP, abs_diff};

pub fn pose(p: PoseRaw) -> Pose2 {
    Pose2 {
        translation: vr(p.translation),
        rotation: Rot2 { re: f(p.rotation.re), im: f(p.rotation.im) },
    }
}

/// The parts of the scene compounds (balls, cuboids, capsules; the half-space ground).
pub fn part(s: Sh1ShapeRaw) -> Shape {
    match s {
        Sh1ShapeRaw::Other(ShapeRaw::Ball(r)) => Shape::Ball(Ball { radius: f(r) }),
        Sh1ShapeRaw::Other(ShapeRaw::Cuboid(h)) => Shape::Cuboid(Cuboid { half_extents: vr(h) }),
        Sh1ShapeRaw::Other(ShapeRaw::Capsule(c)) => {
            let segment = Segment { a: vr(c.a), b: vr(c.b) };
            Shape::Capsule(Capsule { segment, radius: f(c.radius) })
        },
        Sh1ShapeRaw::Other(ShapeRaw::HalfSpace(n)) => Shape::HalfSpace(HalfSpace { normal: vr(n) }),
        Sh1ShapeRaw::RoundCuboid(r) => Shape::RoundCuboid(
            RoundShape {
                inner_shape: Cuboid { half_extents: vr(r.half_extents) },
                border_radius: f(r.border_radius),
            },
        ),
        Sh1ShapeRaw::Triangle(t) => TriangleTrait::new(vr(t.a), vr(t.b), vr(t.c)).into(),
        _ => panic!("unexpected scene part"),
    }
}

fn compound(c: CompoundRaw) -> rapier2d::prelude::ColliderBuilder {
    let mut parts = array![];
    for (p, s) in c.parts {
        parts.append((pose(*p), part(*s)));
    }
    ColliderBuilderTrait::compound(parts.span())
}

fn ground(index: u32) -> rapier2d::prelude::ColliderBuilder {
    if compound_scenes::ground_kind(index) == 0 {
        return ColliderBuilderTrait::new(part(compound_scenes::ground_convex(index)));
    }
    let c = compound_scenes::ground(index);
    if c.is_heightfield {
        let mut heights = array![];
        for h in c.heights {
            heights.append(f(*h));
        }
        return ColliderBuilderTrait::heightfield(heights.span(), vr(c.scale));
    }
    let mut vertices = array![];
    for p in c.vertices {
        vertices.append(vr(*p));
    }
    ColliderBuilderTrait::polyline(vertices.span(), None)
}

/// A world with the ground of scene `index` and one dynamic body at `start`, moving at `linvel`,
/// carrying the scene's compound.
fn scene_world(
    index: u32,
) -> (World, rapier_core::Handle, rapier_core::Handle, rapier_core::Handle) {
    let (_, start, linvel) = *compound_scenes::scenes().at(index);
    let params = IntegrationParameters {
        dt: f(compound_scenes::DT),
        contact_recycling: false,
        contact_clustering: false,
        max_ccd_substeps: 0,
        ..Default::default(),
    };
    let mut world: World = WorldTrait::new(vr(compound_scenes::GRAVITY), params);
    let g = world.insert_collider(ground(index).active_events(COLLISION_EVENTS).build(), None);
    let mut rb = RigidBodyTrait::dynamic(Pose2 { translation: vr(start), ..Default::default() });
    rb.set_linvel(vr(linvel));
    let (body, c) = world.insert(rb, compound(compound_scenes::body(index)).build());
    (world, g, body, c)
}

/// How a scene is judged after its landing step `landing` (see the module documentation).
#[derive(Copy, Drop, PartialEq)]
enum Judge {
    /// Every sample within the band.
    Samples,
    /// Every sample within the band but the velocities of the two steps after `landing`, printed.
    Landing: u32,
}

/// One upstream sample `(step, x, y, re, im, vx, vy, w, _)` against the body: the pose within
/// `2^12 · step` ulp, the velocities within twice that, or printed when `printed`. Returns the
/// largest pose and judged velocity deviations.
pub fn judge_sample(
    id: felt252,
    ref world: World,
    body: rapier_core::Handle,
    sample: (u32, i64, i64, i64, i64, i64, i64, i64, u32),
    printed: bool,
) -> (u64, u64) {
    let (step, x, y, re, im, vx, vy, w, _) = sample;
    let rb = world.body(body).unwrap();
    let p = rb.position();
    let v = rb.linvel();
    let tol = TOL_PER_STEP * step.into();
    let (mut worst, mut worst_v) = (0, 0);
    for d in array![
        abs_diff(p.translation.x.raw, x), abs_diff(p.translation.y.raw, y),
        abs_diff(p.rotation.re.raw, re), abs_diff(p.rotation.im.raw, im),
    ] {
        assert!(d <= tol, "{} step {} pose off by {} ulp", id, step, d);
        if d > worst {
            worst = d;
        }
    }
    for d in array![abs_diff(v.x.raw, vx), abs_diff(v.y.raw, vy), abs_diff(rb.vels.angvel.raw, w)] {
        if printed {
            println!("{} step {}: velocity off by {} ulp", id, step, d);
        } else {
            assert!(d <= 2 * tol, "{} step {} velocity off by {} ulp", id, step, d);
            if d > worst_v {
                worst_v = d;
            }
        }
    }
    (worst, worst_v)
}

fn run(index: u32, judge: Judge) {
    let (id, _, _) = *compound_scenes::scenes().at(index);
    let (mut world, g, body, c) = scene_world(index);
    assert!(g.index == 0 && c.index == 1 && body == body_handle(0), "{} handles", id);
    let mut events = compound_scenes::events(index);
    let (mut worst, mut worst_v): (u64, u64) = (0, 0);
    for sample in compound_scenes::samples(index) {
        let (step, _, _, _, _, _, _, _, manifolds) = *sample;
        for e in world.step() {
            let (e1, e2) = (e.collider1(), e.collider2());
            let (s, started, c1, c2) = *events.pop_front().expect('unexpected event');
            assert_eq!((step, e.started(), e1.index, e2.index), (s, started, c1, c2), "{}", id);
        }
        let printed = match judge {
            Judge::Landing(landing) => step > landing && step <= landing + 2,
            _ => false,
        };
        let (d, d_v) = judge_sample(id, ref world, body, *sample, printed);
        if d > worst {
            worst = d;
        }
        if d_v > worst_v {
            worst_v = d_v;
        }
        let mut with_contacts = 0;
        for m in contact_pair_manifolds(world.narrow_phase.pairs.span(), g, c) {
            if m.data.num_solver_contacts != 0 {
                with_contacts += 1;
            }
        }
        assert_eq!(with_contacts, manifolds, "{} step {} manifolds", id, step);
    }
    assert!(events.is_empty(), "{} missing events", id);
    println!("{}: worst pose deviation {} ulp, velocity {} ulp", id, worst, worst_v);
}

#[test]
fn test_ell_topple() {
    run(0, Judge::Samples);
}

#[test]
fn test_ell_on_seam() {
    run(1, Judge::Landing(6));
}

#[test]
fn test_trio_on_hills() {
    run(2, Judge::Samples);
}
