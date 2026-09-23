//! Nonzero warm starts and velocities keep both no-op guards on their computing paths.
use fixed::{HALF, ONE};
use super::*;

fn active_input(
    count: u8,
) -> (ContactConstraintsSet, DenseBodies, ContactManifold, IntegrationParameters) {
    let (mut cs, mut bodies, m, p) = input(count);
    let mut c = cs.constraints.pop_front().unwrap();
    let [mut a, b] = c.elements;
    a.normal_part.impulse = opaque(HALF);
    a.tangent_part.impulse = opaque(HALF);
    c.elements = [a, b];
    cs.constraints.append(c);
    let mut first = bodies.get(0);
    first.linvel.x = opaque(HALF);
    first.linvel.y = opaque(-ONE);
    bodies.set_pair(0, first, WORLD, Default::default());
    (cs, bodies, m, p)
}
fn original(count: u8, solve: bool) {
    let (mut cs, mut bodies, m, p) = active_input(count);
    if solve {
        Original::run(ref cs, ref bodies, [m].span(), p, 0);
        Original::run(ref cs, ref bodies, [m].span(), p, 1);
        Original::run(ref cs, ref bodies, [m].span(), p, 2);
        Original::run(ref cs, ref bodies, [m].span(), p, 4);
    }
    consume(cs, ref bodies);
}
fn selected(count: u8) {
    let (mut cs, mut bodies, m, p) = active_input(count);
    let directions = crate::solver::contact::cached::prepare(cs.constraints.span());
    zero::contacts(ref cs, ref bodies, [m].span(), p, 0, directions.span());
    zero::contacts(ref cs, ref bodies, [m].span(), p, 1, directions.span());
    zero::contacts(ref cs, ref bodies, [m].span(), p, 2, directions.span());
    zero::contacts(ref cs, ref bodies, [m].span(), p, 4, directions.span());
    consume(cs, ref bodies);
}
#[test]
fn gas_baseline() {
    original(2, false);
}
#[test]
fn gas_original_one() {
    original(1, true);
}
#[test]
fn gas_original_two() {
    original(2, true);
}
#[test]
fn gas_selected_one() {
    selected(1);
}
#[test]
fn gas_selected_two() {
    selected(2);
}
