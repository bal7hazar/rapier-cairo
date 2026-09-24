//! `FreeBodySolverTrait::solve` against the store path (`from_bodies`, `solve_island` with no
//! manifold and no joint, `to_bodies`), raw compare of the whole body, plus its candidates.
use fixed::{Fixed, FixedTrait, ONE, ZERO};
use rapier_core::rigid_body::RigidBodyType;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::{Rot2, Rot2Trait};
use crate::rigid_body::RigidBodyMassPropsTrait;
use crate::rigid_body_set::{RigidBodySetTrait, RigidBodyTrait};
use super::*;
use super::free_alternatives::{solve_carried, solve_looped};
use super::super::body_store::SolverBodyStoreTrait;

fn f(raw: i64) -> Fixed {
    Fixed { raw }
}

/// `rb` stepped through the store with a fixed neighbour before it, as the pipeline did.
fn store_path(rb: RigidBody, params: IntegrationParameters, gravity: Vec2) -> RigidBody {
    let mut set = RigidBodySetTrait::new();
    let _ = set.insert(RigidBodyTrait::fixed(Default::default()));
    let handle = set.insert(rb);
    // `insert` resets the collider list and raises the change flags; keep them in the input.
    let rb = set.get(handle).unwrap();
    let (mut store, _) = SolverBodyStoreTrait::from_bodies(ref set, gravity, params);
    let mut manifolds = array![];
    let mut joints = array![];
    solve_island(params, ref store, ref manifolds, ref joints);
    store.to_bodies(ref set);
    let solved = set.get(handle).unwrap();
    // Compare against the free path on the same input.
    let free = FreeBodySolverTrait::new(params, gravity);
    assert_eq!(free.solve(handle, rb), solved);
    assert_eq!(solve_looped(@free, handle, rb), solved);
    let mut fresh = rb;
    fresh.mprops = rb.mprops.update_world_mass_properties(rb.body_type, rb.pos.position);
    let mut expected = solved;
    expected.mprops = fresh.mprops;
    assert_eq!(solve_carried(@free, handle, fresh), expected);
    solved
}

/// A body of `kind` (0 dynamic, 1 fixed, 2 kinematic position, 3 kinematic velocity, 4
/// disabled dynamic) with every field the solve reads drawn from `seed`.
fn body(kind: u8, seed: u64) -> RigidBody {
    let (s, a) = DivRem::div_rem(seed, 65536);
    let (s, b) = DivRem::div_rem(s, 65536);
    let (s, c) = DivRem::div_rem(s, 65536);
    let (_, d) = DivRem::div_rem(s, 65536);
    let (_, lock) = DivRem::div_rem(d, 4);
    let lock: u8 = lock.try_into().unwrap();
    let a: i64 = a.try_into().unwrap() - 32768;
    let b: i64 = b.try_into().unwrap() - 32768;
    let c: i64 = c.try_into().unwrap() - 32768;
    let d: i64 = d.try_into().unwrap() - 32768;
    let body_type = if kind == 1 {
        RigidBodyType::Fixed
    } else if kind == 2 {
        RigidBodyType::KinematicPositionBased
    } else if kind == 3 {
        RigidBodyType::KinematicVelocityBased
    } else {
        RigidBodyType::Dynamic
    };
    let rotation = Rot2 { re: f(a * 65536), im: f(b * 65536 + 1) }.renormalize();
    let mut rb = RigidBodyTrait::new(
        body_type, Pose2 { translation: Vec2 { x: f(c * 131072), y: f(d * 65536) }, rotation },
    );
    rb.enabled = kind != 4;
    rb.vels.linvel = Vec2 { x: f(b * 4194304), y: f(c * 2097152) };
    rb.vels.angvel = f(d * 1048576);
    rb.mprops.local_mprops.local_com = Vec2 { x: f(a * 8192), y: f(-b * 4096) };
    rb.mprops.local_mprops.inv_mass = f(4294967296 + a * 32768);
    rb.mprops.local_mprops.inv_principal_inertia = f(8589934592 + c * 65536);
    // Translation locks only (bits 0–1): a rotation lock would zero the inertia.
    rb.mprops.flags.bits = lock;
    rb.damping.linear_damping = if a > 0 {
        f(a * 16384)
    } else {
        ZERO
    };
    rb.damping.angular_damping = if b > 0 {
        f(b * 16384)
    } else {
        ZERO
    };
    rb.forces.gravity_scale = f(4294967296 + d * 32768);
    rb.forces.user_force = Vec2 { x: f(c * 65536), y: f(a * 65536) };
    rb.forces.user_torque = f(b * 65536);
    rb
}

fn gravity() -> Vec2 {
    Vec2 { x: ZERO, y: f(-42133629174) }
}

/// Every body kind, default and edge parameters: zero dt, 1/3/4 substeps, a velocity cap that
/// clamps, a rotation cap that clamps.
#[test]
fn test_free_body_matches_the_store_path() {
    let base: IntegrationParameters = Default::default();
    let params = [
        base, IntegrationParameters { dt: ZERO, ..base },
        IntegrationParameters { num_solver_iterations: 1, ..base },
        IntegrationParameters { num_solver_iterations: 3, ..base },
        IntegrationParameters { normalized_max_linear_velocity: ONE, ..base },
        IntegrationParameters { dt: FixedTrait::from_int(2), ..base },
    ];
    let mut moved: u32 = 0;
    for p in params.span() {
        let mut kind = 0;
        while kind != 5 {
            let rb = body(kind, 0x7a31c4e219bf8d05);
            let solved = store_path(rb, *p, gravity());
            if solved != rb {
                moved += 1;
            }
            kind += 1;
        }
    }
    // Moving kinds (3 of 5) change under every non-zero dt (5 of 6 parameter sets).
    assert!(moved >= 15);
}

#[test]
#[fuzzer(runs: 24, seed: 20260924)]
fn fuzz_free_body_matches_the_store_path(seed: u64, kind: u8, iterations: u8) {
    let (_, kind) = DivRem::div_rem(kind, 5);
    let (_, iterations) = DivRem::div_rem(iterations, 5);
    let params = IntegrationParameters {
        num_solver_iterations: iterations.into() + 1, ..Default::default(),
    };
    let _ = store_path(body(kind, seed), params, gravity());
}
