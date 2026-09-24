//! Rejected candidates of `FreeBodySolverTrait::solve` (work package OI), kept for re-ranking
//! (AGENTS.md §5). Probes in `idle_benches`, ranking in REPORT/PR of OI:
//! * `solve_looped`: the substep `while` loop for every iteration count, not unrolled for 4;
//! * `solve_carried` (candidate B): the world mass properties the body carries (refreshed by the
//!   previous position update or user change) instead of recomputing them at the current pose.
//!   Not bit-identical when a body is edited through `World::set_body` without a change flag
//!   (e.g. a new `body_type` or local mass), so it is not shipped.
use rapier_core::rigid_body::RigidBodyType;
use rapier_math::pose2::Pose2;
use crate::rigid_body::{RigidBodyForcesTrait, RigidBodyMassPropsTrait};
use super::*;
use super::super::body_store::BodyStep;

/// `FreeBodySolverTrait::solve` with the substep loop for every iteration count.
pub(crate) fn solve_looped(s: @FreeBodySolver, handle: Handle, rb: RigidBody) -> RigidBody {
    let (mut b, step) = gather(handle, rb, *s.gravity, *s.dt);
    let mut rb = rb;
    if !step.moving {
        return rb;
    }
    let full_dt = *s.full_dt;
    if full_dt != ZERO {
        let (dt, max_lin, max_ang) = (*s.dt, *s.max_lin, *s.max_ang);
        let iterations = *s.iterations;
        let mut substep = 0;
        while substep != iterations {
            add_force(ref b, step.increment);
            integrate_body(ref b, dt, max_lin, max_ang);
            substep += 1;
        }
        damp_body(ref b, step.damping, full_dt);
    }
    writeback(b, step, ref rb);
    rb
}

/// `FreeBodySolverTrait::solve` reading the carried `rb.mprops` (candidate B).
pub(crate) fn solve_carried(s: @FreeBodySolver, handle: Handle, rb: RigidBody) -> RigidBody {
    let mut rb = rb;
    let moving = rb.enabled && rb.body_type != RigidBodyType::Fixed;
    if !moving {
        return rb;
    }
    let mp = rb.mprops;
    let forces = rb.forces.compute_effective_force_and_torque(*s.gravity, mp.effective_mass());
    let increment = if rb.body_type == RigidBodyType::Dynamic {
        forces.integrate(*s.dt, Default::default(), mp)
    } else {
        Default::default()
    };
    let mut b = SolverBody {
        handle,
        position: Pose2 { translation: mp.world_com, rotation: rb.pos.position.rotation },
        linvel: rb.vels.linvel,
        angvel: rb.vels.angvel,
        im: mp.effective_inv_mass,
        ii: mp.effective_world_inv_inertia,
    };
    let step = BodyStep {
        increment, local_com: mp.local_mprops.local_com, damping: rb.damping, moving,
    };
    let full_dt = *s.full_dt;
    if full_dt != ZERO {
        let (dt, max_lin, max_ang) = (*s.dt, *s.max_lin, *s.max_ang);
        if *s.iterations == 4 {
            add_force(ref b, step.increment);
            integrate_body(ref b, dt, max_lin, max_ang);
            add_force(ref b, step.increment);
            integrate_body(ref b, dt, max_lin, max_ang);
            add_force(ref b, step.increment);
            integrate_body(ref b, dt, max_lin, max_ang);
            add_force(ref b, step.increment);
            integrate_body(ref b, dt, max_lin, max_ang);
        } else {
            let mut substep = 0;
            while substep != *s.iterations {
                add_force(ref b, step.increment);
                integrate_body(ref b, dt, max_lin, max_ang);
                substep += 1;
            }
        }
        damp_body(ref b, step.damping, full_dt);
    }
    writeback(b, step, ref rb);
    rb
}
