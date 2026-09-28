//! A local reproduction of slingfall's `pile10` level (`fixtures/levels/pile10.json`) and of the
//! owner's reference shot, pull `(-1022, -63)` (slingfall `docs/PLAN.md` B4: 151 ticks, 22.0M
//! steps). The world is built as slingfall's `GameTrait::new` builds it (level order, one body and
//! one collider per entity, dynamic bodies inserted asleep, contact-force events on blocks and
//! cores, `settle`), the pebble as `sling::launch` spawns it, and each tick is the game's step
//! followed by its damage rule (D6: damage and removal of the destroyed bodies). The rest of the
//! game's tick (calm rule, out-of-bounds, scoring) is not reproduced: the run is a fixed number of
//! ticks.
//!
//! Every tick is generic over the `StepConfig` (`tick::<C>`), so that the in-process step and the
//! split layouts (`rapier_sink::split`) run the same world.

use fixed::FixedTrait;
use rapier2d::prelude::{
    CONTACT_FORCE_EVENTS, ColliderBuilder, ColliderBuilderTrait, ContactForceEvent, Fixed, Handle,
    IntegrationParameters, Pose2, RigidBodyBuilderTrait, RigidBodyTrait, Rot2, StepConfig, Vec2,
    World, WorldTrait,
};

/// `-9.81` in raw Q32.32.
const GRAVITY_Y: i64 = -42133629174;
/// `floor(2^32 / 60)`: slingfall's `TICK_DT_RAW`.
const DT: i64 = 71582788;
const ONE: i64 = 0x100000000;
const HALF: i64 = 0x80000000;
/// `launch_scale` 0.02 (raw, rounded to nearest as slingfall's `levelc`).
const LAUNCH_SCALE: i64 = 85899346;
/// The owner's pull.
pub const PULL: (i32, i32) = (-1022, -63);
/// Ticks of the owner's shot in slingfall (B4).
pub const SHOT_TICKS: u32 = 151;

/// A pile10 material: density, friction, restitution, force threshold, damage per impulse·dt,
/// hit points (raw Q32.32 but `hp`).
#[derive(Copy, Drop)]
struct Material {
    density: i64,
    friction: i64,
    restitution: i64,
    threshold: i64,
    damage: i64,
    hp: u32,
}

fn material(index: u32) -> Material {
    match index {
        0 => Material {
            density: ONE,
            friction: 2576980378,
            restitution: 429496730,
            threshold: 150 * ONE,
            damage: 644245094,
            hp: 100,
        },
        1 => Material {
            density: 10737418240,
            friction: 3435973837,
            restitution: 214748365,
            threshold: 350 * ONE,
            damage: 644245094,
            hp: 300,
        },
        2 => Material {
            density: 3865470566,
            friction: 214748365,
            restitution: 858993459,
            threshold: 40 * ONE,
            damage: 2 * ONE,
            hp: 40,
        },
        _ => Material {
            density: ONE,
            friction: 2576980378,
            restitution: 429496730,
            threshold: 10 * ONE,
            damage: ONE,
            hp: 30,
        },
    }
}

/// An entity of the level: its handles (slot `i`), material, hit points and liveness.
#[derive(Copy, Drop)]
pub struct Entity {
    pub handle: Handle,
    pub is_static: bool,
    pub material: u32,
    pub hp: u32,
    pub alive: bool,
}

/// The game's state as far as the physics sees it.
#[derive(Destruct)]
pub struct Pile {
    pub world: World,
    pub entities: Array<Entity>,
    pub pebble: Option<Handle>,
}

fn f(raw: i64) -> Fixed {
    Fixed { raw }
}

fn at(x: i64, y: i64) -> Pose2 {
    Pose2 { translation: Vec2 { x: f(x), y: f(y) }, rotation: Rot2 { re: f(ONE), im: f(0) } }
}

/// `(shape, pose, material)` of pile10's bodies, in level order; the ground first.
fn bodies() -> Array<(ColliderBuilder, Pose2, u32)> {
    let cube = ColliderBuilderTrait::cuboid(f(HALF), f(HALF));
    array![
        (ColliderBuilderTrait::halfspace(Vec2 { x: f(0), y: f(ONE) }), at(0, 0), 1),
        (cube, at(18 * ONE, HALF), 0), (cube, at(19 * ONE, HALF), 0), (cube, at(20 * ONE, HALF), 0),
        (cube, at(21 * ONE, HALF), 0), (cube, at(18 * ONE + HALF, 6438155977), 1),
        (cube, at(19 * ONE + HALF, 6438155977), 1), (cube, at(20 * ONE + HALF, 6438155977), 1),
        (cube, at(19 * ONE, 10733123273), 2), (cube, at(20 * ONE, 10733123273), 2),
        (ColliderBuilderTrait::ball(f(1717986918)), at(19 * ONE + HALF, 14598593839), 3),
    ]
}

/// pile10 as slingfall's `GameTrait::new` builds it, `settle` included (one `dt = 0` step with
/// `C`, then every dynamic body back to sleep).
pub fn build<impl C: StepConfig>() -> Pile {
    let mut params: IntegrationParameters = Default::default();
    params.dt = f(DT);
    params.num_solver_iterations = 4;
    let mut world = WorldTrait::new(Vec2 { x: f(0), y: f(GRAVITY_Y) }, params);
    let mut entities = array![];
    let mut index: u32 = 0;
    for (shape, pose, m) in bodies() {
        let mat = material(m);
        let mut collider = shape
            .density(f(mat.density))
            .friction(f(mat.friction))
            .restitution(f(mat.restitution))
            .contact_force_event_threshold(f(mat.threshold))
            .user_data(index.into());
        let is_static = index == 0;
        let body = if is_static {
            RigidBodyBuilderTrait::fixed().position(pose).build()
        } else {
            collider = collider.active_events(CONTACT_FORCE_EVENTS);
            RigidBodyBuilderTrait::dynamic().position(pose).sleeping(true).build()
        };
        let (handle, _) = world.insert(body, collider.build());
        entities.append(Entity { handle, is_static, material: m, hp: mat.hp, alive: true });
        index += 1;
    }
    world.integration_parameters.dt = f(0);
    let _ = world.step_with::<C>();
    world.integration_parameters.dt = f(DT);
    let mut pile = Pile { world, entities, pebble: None };
    sleep_all(ref pile);
    pile
}

fn sleep_all(ref pile: Pile) {
    for entity in pile.entities.span() {
        if *entity.alive && !*entity.is_static {
            let handle = *entity.handle;
            if !pile.world.is_sleeping(handle).unwrap() {
                let mut body = pile.world.body(handle).unwrap();
                body.sleep();
                let _ = pile.world.set_body(handle, body);
            }
        }
    }
}

/// slingfall's `sling::launch` of a pull `(px, py)` from the sling anchor `(3, 2.5)`.
pub fn launch(ref pile: Pile, pull: (i32, i32)) {
    let (px, py) = pull;
    let scale = f(LAUNCH_SCALE);
    let linvel = Vec2 {
        x: FixedTrait::from_int(-px) * scale, y: FixedTrait::from_int(-py) * scale,
    };
    let body = RigidBodyBuilderTrait::dynamic()
        .position(at(3 * ONE, 5 * HALF))
        .linvel(linvel)
        .build();
    let collider = ColliderBuilderTrait::ball(f(0x40000000))
        .density(f(0x400000000))
        .friction(f(0x80000000))
        .restitution(f(858993459))
        .user_data(0x100000000)
        .contact_force_event_threshold(f(0))
        .active_events(CONTACT_FORCE_EVENTS)
        .build();
    let (handle, _) = pile.world.insert(body, collider);
    pile.pebble = Some(handle);
}

/// `floor((force - threshold) · damage)` when positive (slingfall `damage::damage_of`).
fn damage_of(force: Fixed, mat: Material) -> u32 {
    let excess = force - f(mat.threshold);
    if excess.raw <= 0 {
        return 0;
    }
    let scaled = excess * f(mat.damage);
    if scaled.raw <= 0 {
        return 0;
    }
    let raw: u64 = scaled.raw.try_into().unwrap();
    let (whole, _) = DivRem::div_rem(raw, 0x100000000_u64.try_into().unwrap());
    whole.try_into().unwrap()
}

fn hit(ref totals: Array<u32>, entities: Span<Entity>, collider: Handle, force: Fixed) {
    let index = collider.index;
    if index >= entities.len() {
        return;
    }
    let entity = *entities[index];
    if entity.handle != collider || entity.is_static || !entity.alive {
        return;
    }
    let amount = damage_of(force, material(entity.material));
    if amount != 0 {
        let mut out = array![];
        let mut i = 0;
        for t in totals.span() {
            out.append(if i == index {
                *t + amount
            } else {
                *t
            });
            i += 1;
        }
        totals = out;
    }
}

/// slingfall's D6: damage of this tick's force events, then removal of the destroyed bodies in
/// ascending entity order. Returns the number destroyed.
fn apply_damage(ref pile: Pile, events: Span<ContactForceEvent>) -> u32 {
    if events.is_empty() {
        return 0;
    }
    let entities = pile.entities.span();
    let mut totals = array![];
    for _ in entities {
        totals.append(0_u32);
    }
    for event in events {
        hit(ref totals, entities, *event.collider1, *event.total_force_magnitude);
        hit(ref totals, entities, *event.collider2, *event.total_force_magnitude);
    }
    let mut updated = array![];
    let mut destroyed = 0;
    let mut totals = totals.span();
    for entity in entities {
        let mut entity = *entity;
        let total = *totals.pop_front().unwrap();
        if total != 0 {
            entity.hp = if total >= entity.hp {
                0
            } else {
                entity.hp - total
            };
            if entity.hp == 0 {
                let _ = pile.world.remove_body(entity.handle);
                entity.alive = false;
                destroyed += 1;
            }
        }
        updated.append(entity);
    }
    pile.entities = updated;
    destroyed
}

/// One game tick with `C`: the step with force events, then the damage rule. Returns the number
/// of force events.
pub fn tick<impl C: StepConfig>(ref pile: Pile) -> u32 {
    let (_, events) = pile.world.step_with_force_events_with::<C>();
    let _ = apply_damage(ref pile, events.span());
    events.len()
}

/// pile10, the owner's shot, `ticks` ticks with `C`; returns the force events counted.
pub fn run<impl C: StepConfig>(ticks: u32) -> (Pile, u32) {
    let mut pile = build::<C>();
    launch(ref pile, PULL);
    let mut events = 0;
    let mut i = 0;
    while i != ticks {
        events += tick::<C>(ref pile);
        i += 1;
    }
    (pile, events)
}

/// Poseidon digest of every live body's pose and velocities (raw), and of the entity states.
pub fn digest(ref pile: Pile) -> felt252 {
    let mut felts: Array<felt252> = array![];
    let mut handles = array![];
    for entity in pile.entities.span() {
        felts.append((*entity.hp).into());
        if *entity.alive {
            handles.append(*entity.handle);
        }
    }
    if let Some(pebble) = pile.pebble {
        handles.append(pebble);
    }
    for handle in handles {
        if let Some(body) = pile.world.body(handle) {
            body.position().serialize(ref felts);
            body.linvel().serialize(ref felts);
            body.angvel().serialize(ref felts);
        }
    }
    core::poseidon::poseidon_hash_span(felts.span())
}
