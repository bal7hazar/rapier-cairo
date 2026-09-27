//! RG1: the game's call pattern on level 10, in exact Cairo steps. The level is loaded as
//! `golden_scenes::levels::load_level` loads it, but every collider has contact-force events on
//! (`CONTACT_FORCE_EVENTS`, threshold `FORCE_THRESHOLD`), as the game's blocks do. One tick is
//! the game's: `World::step_with_force_events`, then (by `mode`) the activation reads of every
//! dynamic body (`is_sleeping`, and `body` when awake), the despawn rule (`levels::despawn`) and
//! one forced removal (`DESPAWNED` at tick `DESPAWN_TICK`); with `CHUNKED`, the world goes
//! through a `WorldState` round trip (`into_state`, `Serde::serialize`, `Serde::deserialize`,
//! `from_state`) every `CHUNK` ticks, as the game's chunked replay restores and saves it.
//!
//! The `steps_game_*` probes add one component at a time (flight plus the first impact ticks,
//! `IMPACT`); their differences give the cost of the force-event step, the reads, the despawn
//! and the state round trips. Run with `--tracked-resource cairo-steps`.

use core::poseidon::poseidon_hash_span;
use rapier2d::pipeline::config::BasicStepConfig;
use rapier2d::prelude::{
    CONTACT_FORCE_EVENTS, ColliderBuilderTrait, Fixed, IntegrationParameters, RigidBodyTrait, Vec2,
    World, WorldTrait,
};
use rapier2d::world::state::WorldState;
use rapier_dynamics2d::collider::Collider;
use rapier_golden::generated::level_scenes;
use rapier_golden::types::{LevelBodyRaw, PolygonContactShapeRaw, PoseRaw, ShapeRaw, Vec2Raw};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::golden_scenes::levels::{despawn, handle, level};

/// Flight plus the first five impact ticks (first contact at tick 26).
const IMPACT: u32 = 30;
/// Ticks per chunk of the chunked run.
const CHUNK: u32 = 10;
/// The body removed at `DESPAWN_TICK` (a block of the pile, as a destroyed block is).
const DESPAWNED: u32 = 5;
const DESPAWN_TICK: u32 = 27;
/// Contact-force event threshold of every collider: 4 N.
const FORCE_THRESHOLD: i64 = 0x400000000;

/// `digest` of the level after `IMPACT` ticks with reads and despawns, measured on
/// `0.1.0-alpha.4` (1213fbb) and every commit up to RG1; re-pinned by SF1 (rebased frozen contact
/// separations; before: 3326…5775).
const GAME_DIGEST: felt252 =
    2022157671005519863083205651691126028746519623184051613454076122783391620987;

/// `mode` bits.
const READS: u32 = 1;
const DESPAWN: u32 = 2;
const CHUNKED: u32 = 4;
/// Plain `World::step` instead of `step_with_force_events`.
const PLAIN: u32 = 8;

fn f(raw: i64) -> Fixed {
    Fixed { raw }
}

fn vr(raw: Vec2Raw) -> Vec2 {
    Vec2 { x: f(raw.x), y: f(raw.y) }
}

fn pose(raw: PoseRaw) -> Pose2 {
    Pose2 {
        translation: vr(raw.translation),
        rotation: Rot2 { re: f(raw.rotation.re), im: f(raw.rotation.im) },
    }
}

fn collider(desc: LevelBodyRaw) -> Collider {
    let builder = match desc.shape {
        PolygonContactShapeRaw::Polygon(polygon) => {
            let mut points = array![];
            let mut k: u8 = 0;
            for v in polygon.vertices.span() {
                if k == polygon.count {
                    break;
                }
                points.append(vr(*v));
                k += 1;
            }
            ColliderBuilderTrait::convex_polygon(points.span()).expect('level polygon')
        },
        PolygonContactShapeRaw::Other(shape) => match shape {
            ShapeRaw::Ball(radius) => ColliderBuilderTrait::ball(f(radius)),
            ShapeRaw::Cuboid(half) => ColliderBuilderTrait::cuboid(f(half.x), f(half.y)),
            ShapeRaw::HalfSpace(normal) => ColliderBuilderTrait::halfspace(vr(normal)),
            _ => panic!("unexpected level shape"),
        },
    };
    builder
        .density(f(desc.density))
        .friction(f(desc.friction))
        .restitution(f(desc.restitution))
        .active_events(CONTACT_FORCE_EVENTS)
        .contact_force_event_threshold(f(FORCE_THRESHOLD))
        .build()
}

/// Level 10 at 60 Hz, 4 iterations, loaded as `levels::load_level` does.
fn load() -> (World, u32, (i64, i64)) {
    let (bodies, pebble_linvel, bounds) = level(opaque(10));
    let params = IntegrationParameters {
        dt: f(level_scenes::level10_hz60_sub4::DT), num_solver_iterations: 4, ..Default::default(),
    };
    let mut world = WorldTrait::new(vr(level_scenes::GRAVITY), params);
    let n = bodies.len();
    let mut i = 0;
    while i != n - 1 {
        let desc = *bodies.at(i);
        let body = if desc.role == 'ground' {
            RigidBodyTrait::fixed(pose(desc.pose))
        } else {
            RigidBodyTrait::dynamic(pose(desc.pose))
        };
        let _ = world.insert(body, collider(desc));
        i += 1;
    }
    world.gravity = Vec2 { x: f(0), y: f(0) };
    let _ = world.step();
    world.gravity = vr(level_scenes::GRAVITY);
    let mut i = 1;
    while i != n - 1 {
        let mut rb = world.body(handle(i)).unwrap();
        rb.sleep();
        assert!(world.set_body(handle(i), rb));
        i += 1;
    }
    let desc = *bodies.at(n - 1);
    let mut pebble = RigidBodyTrait::dynamic(pose(desc.pose));
    pebble.set_linvel(vr(pebble_linvel));
    let _ = world.insert(pebble, collider(desc));
    (world, n, bounds)
}

/// The game's activation reads: every live dynamic body's flag, its velocity when awake.
fn reads(ref world: World, n: u32) -> u32 {
    let mut awake = 0;
    let mut i = 1;
    while i != n {
        if let Some(sleeping) = world.is_sleeping(handle(i)) {
            if !sleeping {
                let rb = world.body(handle(i)).unwrap();
                if rb.linvel().x.raw != 0 {
                    awake += 1;
                }
            }
        }
        i += 1;
    }
    awake
}

/// One chunk boundary: the world saved, serialised, deserialised and restored.
fn round_trip(world: World) -> World {
    let state = world.into_state();
    let mut out = array![];
    state.serialize(ref out);
    let mut span = out.span();
    let state: WorldState = Serde::deserialize(ref span).unwrap();
    WorldTrait::from_state(state)
}

/// Runs `ticks` game ticks in `mode`; returns the world and the force events seen.
fn game(ticks: u32, mode: u32) -> (World, u32) {
    let (mut world, n, bounds) = load();
    let mut events = 0;
    let mut t = 1;
    while t != ticks + 1 {
        if mode & PLAIN != 0 {
            let _ = world.step();
        } else {
            let (_, forces) = world.step_with_force_events();
            events += forces.len();
        }
        if mode & READS != 0 {
            let _ = reads(ref world, n);
        }
        if mode & DESPAWN != 0 {
            let _ = despawn(ref world, n, bounds);
            if t == DESPAWN_TICK {
                let _ = world.remove_body(handle(DESPAWNED));
            }
        }
        if mode & CHUNKED != 0 && t % CHUNK == 0 {
            world = round_trip(world);
        }
        t += 1;
    }
    (world, events)
}

/// [`game`] with the game-shaped step (CS2): `step_with_force_events_with::<BasicStepConfig>`
/// (`step_with::<BasicStepConfig>` with `PLAIN`). A copy, so that the probes of [`game`] keep
/// their exact steps.
fn game_basic(ticks: u32, mode: u32) -> (World, u32) {
    let (mut world, n, bounds) = load();
    let mut events = 0;
    let mut t = 1;
    while t != ticks + 1 {
        if mode & PLAIN != 0 {
            let _ = world.step_with::<BasicStepConfig>();
        } else {
            let (_, forces) = world.step_with_force_events_with::<BasicStepConfig>();
            events += forces.len();
        }
        if mode & READS != 0 {
            let _ = reads(ref world, n);
        }
        if mode & DESPAWN != 0 {
            let _ = despawn(ref world, n, bounds);
            if t == DESPAWN_TICK {
                let _ = world.remove_body(handle(DESPAWNED));
            }
        }
        if mode & CHUNKED != 0 && t % CHUNK == 0 {
            world = round_trip(world);
        }
        t += 1;
    }
    (world, events)
}

/// Loads the level (after `ticks` plain ticks) and runs `trips` state round trips, through
/// `Serde` or not.
fn trips(ticks: u32, trips: u32, serde: bool) {
    let (mut world, _) = game(ticks, PLAIN);
    let mut k = 0;
    while k != trips {
        world = if serde {
            round_trip(world)
        } else {
            WorldTrait::from_state(world.into_state())
        };
        k += 1;
    }
    let _ = opaque(world.gravity);
}

fn probe(ticks: u32, mode: u32) {
    let (world, events) = game(opaque(ticks), opaque(mode));
    let _ = opaque(world.gravity);
    let _ = opaque(events);
}

fn probe_basic(ticks: u32, mode: u32) {
    let (world, events) = game_basic(opaque(ticks), opaque(mode));
    let _ = opaque(world.gravity);
    let _ = opaque(events);
}

/// Poseidon over the raw poses and velocities of the level's live bodies.
fn digest(ref world: World, n: u32) -> felt252 {
    let mut felts: Array<felt252> = array![];
    let mut i = 0;
    while i != n {
        if let Some(rb) = world.body(handle(i)) {
            let p = rb.position();
            let v = rb.linvel();
            felts.append(p.translation.x.raw.into());
            felts.append(p.translation.y.raw.into());
            felts.append(p.rotation.re.raw.into());
            felts.append(p.rotation.im.raw.into());
            felts.append(v.x.raw.into());
            felts.append(v.y.raw.into());
            felts.append(rb.vels.angvel.raw.into());
        }
        i += 1;
    }
    poseidon_hash_span(felts.span())
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
fn steps_game_load() {
    probe(0, 0);
}

#[test]
fn steps_game_step() {
    probe(IMPACT, PLAIN);
}

#[test]
fn steps_game_force() {
    probe(IMPACT, 0);
}

#[test]
fn steps_game_reads() {
    probe(IMPACT, READS);
}

#[test]
fn steps_game_despawn() {
    probe(IMPACT, READS | DESPAWN);
}

#[test]
fn steps_game_chunked() {
    probe(IMPACT, READS | DESPAWN | CHUNKED);
}

#[test]
fn steps_game_state_trips() {
    trips(opaque(IMPACT), opaque(3), opaque(false));
}

#[test]
fn steps_game_serde_trips() {
    trips(opaque(IMPACT), opaque(3), opaque(true));
}

/// The chunked run ends in the same state as the straight one, with the same force events, and
/// both are pinned bit for bit (the digest and event count measured on `0.1.0-alpha.4`, the digest
/// re-pinned by SF1).
#[test]
fn test_game_digest() {
    let (mut straight, events) = game(IMPACT, READS | DESPAWN);
    let (mut chunked, chunked_events) = game(IMPACT, READS | DESPAWN | CHUNKED);
    let (bodies, _, _) = level(10);
    let n = bodies.len();
    let d = digest(ref straight, n);
    assert!(d == GAME_DIGEST, "game digest {d}");
    assert!(events == 51, "force events {events}");
    assert!(digest(ref chunked, n) == d, "chunked digest");
    assert!(chunked_events == events, "chunked events");
}

#[test]
fn steps_game_basic_step() {
    probe_basic(IMPACT, PLAIN);
}

#[test]
fn steps_game_basic_force() {
    probe_basic(IMPACT, 0);
}

#[test]
fn steps_game_basic_despawn() {
    probe_basic(IMPACT, READS | DESPAWN);
}

#[test]
fn steps_game_basic_chunked() {
    probe_basic(IMPACT, READS | DESPAWN | CHUNKED);
}

/// CS2: the game-shaped step ends in the pinned state, with the same force events, straight and
/// chunked.
#[test]
fn test_game_basic_digest() {
    let (mut straight, events) = game_basic(IMPACT, READS | DESPAWN);
    let (mut chunked, chunked_events) = game_basic(IMPACT, READS | DESPAWN | CHUNKED);
    let (bodies, _, _) = level(10);
    let n = bodies.len();
    let d = digest(ref straight, n);
    assert!(d == GAME_DIGEST, "game digest {d}");
    assert!(events == 51, "force events {events}");
    assert!(digest(ref chunked, n) == d, "chunked digest");
    assert!(chunked_events == events, "chunked events");
}
