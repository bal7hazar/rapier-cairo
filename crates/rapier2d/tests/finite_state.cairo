//! Q32.32 state cannot become NaN or infinite: the parity reason for upstream's `Quarantine`
//! (`pipeline/physics_pipeline/quarantine.rs`), which detects and contains non-finite state. Here
//! the operations most likely to leave the representable range — a large impulse on a body of
//! tiny mass, and the step that integrates it — panic with `fixed`'s documented messages
//! (`'Fixed: overflow'`, `'Fixed: division by zero'`); they never produce an invalid value, so
//! there is nothing to contain.

use rapier2d::prelude::{ColliderBuilderTrait, Fixed, RigidBodyTrait, Vec2, World, WorldTrait};
use rapier_math::pose2::Pose2;

fn raw(raw: i64) -> Fixed {
    Fixed { raw }
}

fn v(x: i64, y: i64) -> Vec2 {
    Vec2 { x: raw(x), y: raw(y) }
}

/// A world without gravity holding one dynamic ball of radius 1 and density `density_raw` (raw
/// Q32.32), at the origin.
fn tiny_ball(density_raw: i64) -> (World, rapier2d::prelude::Handle) {
    let mut world = WorldTrait::new(v(0, 0), Default::default());
    let body = RigidBodyTrait::dynamic(Default::default());
    let collider = ColliderBuilderTrait::ball(raw(1_i64 * 4294967296))
        .density(raw(density_raw))
        .build();
    let (handle, _) = world.insert(body, collider);
    (world, handle)
}

/// A small mass (density `2^-16`, mass ≈ π·2^-16): its inverses fit, and a unit impulse gives a
/// large (≈ 2·10^4 m/s) but finite velocity that the step integrates.
#[test]
fn test_small_mass_stays_finite() {
    let (mut world, handle) = tiny_ball(65536);
    let mut rb = world.body(handle).unwrap();
    rb.apply_impulse(v(4294967296, 0), true);
    assert!(world.set_body(handle, rb));
    let _ = world.step();
    let rb = world.body(handle).unwrap();
    assert!(rb.linvel().x.raw > 0, "a finite, positive velocity");
    let _: Pose2 = rb.position();
}

/// The smallest representable density (1 raw unit) gives a mass whose inverse inertia is beyond
/// the Q32.32 range: the first impulse and step refuse it with a panic.
#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_smallest_mass_panics() {
    let (mut world, handle) = tiny_ball(1);
    let mut rb = world.body(handle).unwrap();
    rb.apply_impulse(v(4294967296, 0), true);
    assert!(world.set_body(handle, rb));
    let _ = world.step();
}

/// A velocity beyond the Q32.32 range is refused where it would be produced: a huge impulse on a
/// small mass panics instead of wrapping or saturating silently.
#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_huge_impulse_panics() {
    let (mut world, handle) = tiny_ball(65536);
    let mut rb = world.body(handle).unwrap();
    rb.apply_impulse(v(0x7fffffffffffffff / 4, 0), true);
    assert!(world.set_body(handle, rb));
    let _ = world.step();
}
