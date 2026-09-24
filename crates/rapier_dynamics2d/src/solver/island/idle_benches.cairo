//! Per-idle-body breakdown of one solve (work package OI). Every probe builds `n` bodies with
//! no constraint, then runs the first `stage` stages: `gas_idle<n>_<stage>` − the previous stage
//! is that stage's cost. Stages: 1 `from_bodies`, 2 `snapshot`, 3 four `add_forces`, 4 four
//! `integrate`, 5 `damp`, 6 `to_bodies`.
use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use rapier_testing::opaque;
use crate::rigid_body_set::{RigidBodySet, RigidBodySetTrait, RigidBodyTrait};
use super::*;
use super::super::body_store::SolverBodyStoreTrait;

pub(crate) fn idle_set(n: u32) -> RigidBodySet {
    let mut set = RigidBodySetTrait::new();
    let mut k = 0;
    while k != n {
        let x = FixedTrait::from_int(k.try_into().unwrap()) * FixedTrait::from_int(4);
        let mut rb = RigidBodyTrait::dynamic(
            rapier_math::pose2::Pose2 {
                translation: glam::Vec2 { x, y: FixedTrait::from_int(100) }, ..Default::default(),
            },
        );
        rb.mprops.local_mprops.inv_mass = ONE;
        rb.mprops.local_mprops.inv_principal_inertia = FixedTrait::from_int(2);
        rb.mprops.local_mprops.local_com = glam::Vec2 { x: HALF, y: ZERO };
        rb.vels.linvel = glam::Vec2 { x: HALF, y: -ONE };
        rb.vels.angvel = HALF;
        let _ = set.insert(rb);
        k += 1;
    }
    set
}

fn gravity() -> glam::Vec2 {
    glam::Vec2 { x: ZERO, y: Fixed { raw: -42133629174 } }
}

#[inline(never)]
fn idle_probe(n: u32, stage: u8) {
    let mut set = idle_set(opaque(n));
    if stage == 0 {
        return;
    }
    let p: IntegrationParameters = opaque(Default::default());
    let (mut store, _) = SolverBodyStoreTrait::from_bodies(ref set, opaque(gravity()), p);
    if stage == 1 {
        return;
    }
    let dt = p.substep_dt();
    let max_lin = p.max_linear_velocity();
    let max_ang = Fixed { raw: 3373259426 } * p.inv_dt();
    let initial = snapshot(ref store.bodies);
    let _ = opaque(initial.len());
    if stage == 2 {
        return;
    }
    let mut s = 0;
    while s != p.num_solver_iterations {
        add_forces(ref store.bodies, store.steps);
        s += 1;
    }
    if stage == 3 {
        return;
    }
    let mut s = 0;
    while s != p.num_solver_iterations {
        integrate(ref store.bodies, store.steps, dt, max_lin, max_ang);
        s += 1;
    }
    if stage == 4 {
        return;
    }
    damp(ref store.bodies, store.steps, p.dt);
    if stage == 5 {
        return;
    }
    store.to_bodies(ref set);
}

/// `FreeBodySolverTrait::solve` on every body of `idle_set(n)`: stage 1 the entries only, stage
/// 2 `new`, stage 3 the gathers only, stage 4 the full solves, stage 5 `solve_looped`, stage 6
/// `solve_carried` (`free_alternatives`).
#[inline(never)]
fn free_probe(n: u32, stage: u8) {
    let mut set = idle_set(opaque(n));
    let entries = set.iter();
    if stage == 1 {
        return;
    }
    let free = FreeBodySolverTrait::new(opaque(Default::default()), opaque(gravity()));
    if stage == 2 {
        return;
    }
    for (handle, rb) in entries.span() {
        if stage == 3 {
            let _ = opaque(super::super::body_store::gather(*handle, *rb, gravity(), free.dt));
        } else if stage == 5 {
            let _ = opaque(super::free_alternatives::solve_looped(@free, *handle, *rb));
        } else if stage == 6 {
            let _ = opaque(super::free_alternatives::solve_carried(@free, *handle, *rb));
        } else {
            let _ = opaque(free.solve(*handle, *rb));
        }
    }
}

#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}

#[test]
fn gas_idle1_setup() {
    idle_probe(1, 0);
}
#[test]
fn gas_idle1_from_bodies() {
    idle_probe(1, 1);
}
#[test]
fn gas_idle1_snapshot() {
    idle_probe(1, 2);
}
#[test]
fn gas_idle1_add_forces() {
    idle_probe(1, 3);
}
#[test]
fn gas_idle1_integrate() {
    idle_probe(1, 4);
}
#[test]
fn gas_idle1_damp() {
    idle_probe(1, 5);
}
#[test]
fn gas_idle1_to_bodies() {
    idle_probe(1, 6);
}
#[test]
fn gas_idle9_setup() {
    idle_probe(9, 0);
}
#[test]
fn gas_idle9_from_bodies() {
    idle_probe(9, 1);
}
#[test]
fn gas_idle9_snapshot() {
    idle_probe(9, 2);
}
#[test]
fn gas_idle9_add_forces() {
    idle_probe(9, 3);
}
#[test]
fn gas_idle9_integrate() {
    idle_probe(9, 4);
}
#[test]
fn gas_idle9_damp() {
    idle_probe(9, 5);
}
#[test]
fn gas_idle9_to_bodies() {
    idle_probe(9, 6);
}
#[test]
fn gas_free1_entries() {
    free_probe(1, 1);
}
#[test]
fn gas_free1_new() {
    free_probe(1, 2);
}
#[test]
fn gas_free1_gather() {
    free_probe(1, 3);
}
#[test]
fn gas_free1_solve() {
    free_probe(1, 4);
}
#[test]
fn gas_free9_entries() {
    free_probe(9, 1);
}
#[test]
fn gas_free9_gather() {
    free_probe(9, 3);
}
#[test]
fn gas_free9_solve() {
    free_probe(9, 4);
}
#[test]
fn gas_free1_solve_looped() {
    free_probe(1, 5);
}
#[test]
fn gas_free9_solve_looped() {
    free_probe(9, 5);
}
#[test]
fn gas_free1_solve_carried() {
    free_probe(1, 6);
}
#[test]
fn gas_free9_solve_carried() {
    free_probe(9, 6);
}
