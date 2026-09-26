//! BT2: the cost of the sleeping bodies of a level (`level_budget`'s levels, loaded and ticked as
//! the game does), in Sierra gas (`gas_*`, ceilings at measured + 10 %) and Cairo steps (their
//! uncapped `steps_*` twins, `--tracked-resource cairo-steps`):
//! * the one-tick flight (`flight1`: the tick that takes the pebble's insertion), to subtract
//!   from `level_budget`'s 25-tick flight: 24 flight ticks with the structure asleep;
//! * the all-asleep window: the level loaded, the pebble removed (the end of a shot, every block
//!   asleep), one tick (`asleep1`, it takes the removal) then six (`asleep`): five steady ticks.

use rapier2d::prelude::WorldTrait;
use rapier_golden::generated::level_scenes;
use rapier_testing::opaque;
use crate::golden_scenes::levels::{handle, level, load_level, tick};

/// Level `blocks` loaded at 60 Hz with 4 iterations, the pebble removed when `asleep`, then
/// `ticks` ticks.
fn probe(blocks: u32, asleep: bool, ticks: u32) {
    let (blocks, asleep, ticks) = (opaque(blocks), opaque(asleep), opaque(ticks));
    let (bodies, linvel, bounds) = level(blocks);
    let n = bodies.len();
    let mut world = load_level(bodies, linvel, level_scenes::level10_hz60_sub4::DT, 4);
    if asleep {
        let _ = world.remove_body(handle(n - 1));
    }
    let mut t = 0;
    while t != ticks {
        tick(ref world, n, bounds);
        t += 1;
    }
    let _ = opaque(world.gravity);
}

/// Ticks of the all-asleep window.
const ASLEEP: u32 = 6;

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
#[available_gas(l2_gas: 80500206)]
fn gas_flight1_level10() {
    probe(10, false, 1);
}

#[test]
#[available_gas(l2_gas: 145462366)]
fn gas_flight1_level20() {
    probe(20, false, 1);
}

#[test]
#[available_gas(l2_gas: 80685223)]
fn gas_asleep1_level10() {
    probe(10, true, 1);
}

#[test]
#[available_gas(l2_gas: 83442868)]
fn gas_asleep_level10() {
    probe(10, true, ASLEEP);
}

#[test]
#[available_gas(l2_gas: 146795464)]
fn gas_asleep1_level20() {
    probe(20, true, 1);
}

#[test]
#[available_gas(l2_gas: 150957809)]
fn gas_asleep_level20() {
    probe(20, true, ASLEEP);
}

#[test]
fn steps_flight1_level10() {
    probe(10, false, 1);
}

#[test]
fn steps_flight1_level20() {
    probe(20, false, 1);
}

#[test]
fn steps_asleep1_level10() {
    probe(10, true, 1);
}

#[test]
fn steps_asleep_level10() {
    probe(10, true, ASLEEP);
}

#[test]
fn steps_asleep1_level20() {
    probe(20, true, 1);
}

#[test]
fn steps_asleep_level20() {
    probe(20, true, ASLEEP);
}
