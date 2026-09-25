//! Sequential whole-world soft-step driver (D8/D9). Manifolds and joints retain caller order;
//! each sweep solves joints before contacts. No sleeping, island discovery, CCD or colouring.
//! Contact sweeps use cached frame coefficients and two body values; joint sweeps retain
//! their measured array adapter. Global scattering remains O(1).
mod empty;
mod sweeps;
use fixed::{Fixed, MAX, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_core::Handle;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_core::rigid_body::RigidBodyDamping;
use rapier_geometry2d::contact::ContactManifold;
use sweeps::array_joint::joints;
#[cfg(test)]
use sweeps::contact::contacts;
use sweeps::split::{SweepBodies, SweepBodiesTrait};
use sweeps::{prepare_joints, rebuild_joints, split};
use crate::joint::ImpulseJoint;
use crate::rigid_body::{RigidBodyVelocity, RigidBodyVelocityTrait};
use crate::rigid_body_set::RigidBody;
use super::body::{SolverBody, WORLD};
use super::body_store::{BodyStep, DenseBodiesTrait, SolverBodyStore, gather, writeback};
#[cfg(test)]
use super::contact::ContactConstraintsSetTrait;

/// Invalid timestep/velocity cap. Parameter and fixed-point panics otherwise propagate.
pub mod errors {
    /// The timestep or a scaled velocity limit is negative.
    pub const NEGATIVE: felt252 = 'Island: negative parameter';
}

/// Advance one step on a store built with these parameters, persisting impulses into the
/// ordered manifold/joint arrays. `contact_set` contains frozen manifold membership for this
/// frame (flatten DD pairs in pair-slot order); `joint_set` contains body-local joint frames.
/// Applies force increments, joint rebuilding, contact warmstarts, biased PGS, velocity caps,
/// linearized integration, relaxed PGS, restitution, impulse writeback, then full-step damping.
/// Call `store.to_bodies` afterwards; do not reuse this per-step scratch for another step.
/// Products floor, divisions round to nearest, rotations renormalize. All intermediates must fit
/// Q32.32. Zero dt is a no-op; zero solver iterations and negative dt/caps panic.
pub fn solve_island(
    params: IntegrationParameters,
    ref store: SolverBodyStore,
    ref contact_set: Array<ContactManifold>,
    ref joint_set: Array<ImpulseJoint>,
) {
    run(params, ref store.bodies, store.steps, ref contact_set, ref joint_set);
}

fn snapshot<B, +DenseBodiesTrait<B>, +Destruct<B>>(ref bodies: B) -> Array<SolverBody> {
    let n = bodies.len();
    let mut i = 0;
    let mut out = array![];
    while i != n {
        out.append(bodies.get(i));
        i += 1;
    }
    out
}

fn run<B, +DenseBodiesTrait<B>, +Destruct<B>>(
    params: IntegrationParameters,
    ref bodies: B,
    steps: Span<BodyStep>,
    ref manifolds: Array<ContactManifold>,
    ref joint_set: Array<ImpulseJoint>,
) {
    let dt = params.substep_dt();
    let max_lin = params.max_linear_velocity();
    let max_corrective = params.max_corrective_velocity();
    assert(params.dt >= ZERO && max_lin >= ZERO && max_corrective >= ZERO, errors::NEGATIVE);
    if params.dt == ZERO {
        return;
    }
    // π/4 per FULL frame, exactly upstream's MAX_ROTATION policy.
    let max_ang = Fixed { raw: 3373259426 } * params.inv_dt();
    let initial = snapshot(ref bodies);
    if manifolds.is_empty() {
        let builders = prepare_joints(joint_set.span(), initial.span(), steps);
        empty::run(params, ref bodies, steps, builders.span(), ref joint_set, dt, max_lin, max_ang);
        return;
    }
    let (frozen, mut hot) = split::generate(manifolds.span(), initial.span(), params, dt);
    let builders = prepare_joints(joint_set.span(), initial.span(), steps);
    if frozen.is_empty() {
        empty::run(params, ref bodies, steps, builders.span(), ref joint_set, dt, max_lin, max_ang);
        return;
    }
    let frozen = frozen.span();
    let mut sb: SweepBodies = DenseBodiesTrait::new(initial.span());
    let mut rows = array![];
    let mut substep = 0;
    while substep != params.num_solver_iterations {
        sb.add_forces(steps);
        rows = rebuild_joints(ref sb, builders.span(), rows.span(), params, substep != 0);
        split::contacts(ref hot, frozen, ref sb, params, 0);
        let mut i = 0;
        while i != params.num_internal_pgs_iterations {
            joints(ref rows, ref sb, true, params.warmstart_joints && i == 0);
            split::contacts(ref hot, frozen, ref sb, params, 1);
            i += 1;
        }
        sb.integrate(steps, dt, max_lin, max_ang);
        let mut i = 0;
        while i != params.num_internal_stabilization_iterations {
            joints(ref rows, ref sb, false, false);
            split::contacts(ref hot, frozen, ref sb, params, if i == 0 {
                2
            } else {
                3
            });
            i += 1;
        }
        substep += 1;
    }
    split::contacts(ref hot, frozen, ref sb, params, 4);
    split::writeback(frozen, hot.span(), ref manifolds);
    sweeps::write_joints(rows.span(), ref joint_set);
    sb.damp(steps, params.dt);
    sb.finish(ref bodies);
}

fn add_forces<B, +DenseBodiesTrait<B>, +Destruct<B>>(ref bodies: B, steps: Span<BodyStep>) {
    let mut i = 0;
    let n = bodies.len();
    while i != n {
        if *steps.at(i).moving {
            let mut b = bodies.get(i);
            add_force(ref b, *steps.at(i).increment);
            bodies.set_pair(i, b, WORLD, Default::default());
        }
        i += 1;
    }
}
fn integrate<B, +DenseBodiesTrait<B>, +Destruct<B>>(
    ref bodies: B, steps: Span<BodyStep>, dt: Fixed, max_lin: Fixed, max_ang: Fixed,
) {
    let mut i = 0;
    let n = bodies.len();
    while i != n {
        if *steps.at(i).moving {
            let mut b = bodies.get(i);
            integrate_body(ref b, dt, max_lin, max_ang);
            bodies.set_pair(i, b, WORLD, Default::default());
        }
        i += 1;
    }
}
fn damp<B, +DenseBodiesTrait<B>, +Destruct<B>>(ref bodies: B, steps: Span<BodyStep>, dt: Fixed) {
    let mut i = 0;
    let n = bodies.len();
    while i != n {
        if *steps.at(i).moving {
            let mut b = bodies.get(i);
            damp_body(ref b, *steps.at(i).damping, dt);
            bodies.set_pair(i, b, WORLD, Default::default());
        }
        i += 1;
    }
}
/// One body's share of `add_forces`.
#[inline(always)]
fn add_force(ref b: SolverBody, dv: RigidBodyVelocity) {
    b.linvel = b.linvel + dv.linvel;
    b.angvel += dv.angvel;
}
/// One body's share of `integrate`: velocity caps, then the pose update.
#[inline(always)]
fn integrate_body(ref b: SolverBody, dt: Fixed, max_lin: Fixed, max_ang: Fixed) {
    // Sentinel guard is before length computation: disabled caps need no sqrt.
    if max_lin != MAX {
        let length = b.linvel.length();
        if length > max_lin {
            b.linvel = b.linvel.mul_scalar(max_lin / length);
        }
    }
    if b.angvel > max_ang {
        b.angvel = max_ang;
    }
    if b.angvel < -max_ang {
        b.angvel = -max_ang;
    }
    let v = RigidBodyVelocity { linvel: b.linvel, angvel: b.angvel };
    b.position = v.integrate(dt, b.position, Default::default());
}
/// One body's share of `damp`.
#[inline(always)]
fn damp_body(ref b: SolverBody, damping: RigidBodyDamping, dt: Fixed) {
    let v = RigidBodyVelocity { linvel: b.linvel, angvel: b.angvel }.apply_damping(dt, damping);
    b.linvel = v.linvel;
    b.angvel = v.angvel;
}

/// Per-step constants of [`FreeBodySolverTrait::solve`], computed once per step. `Default` is
/// a placeholder for a step without free body: zero dt, it integrates nothing.
#[derive(Copy, Drop, Debug, PartialEq, Default)]
pub struct FreeBodySolver {
    gravity: Vec2,
    full_dt: Fixed,
    dt: Fixed,
    max_lin: Fixed,
    max_ang: Fixed,
    iterations: u32,
}

#[generate_trait]
pub impl FreeBodySolverImpl of FreeBodySolverTrait {
    /// The constants `solve_island` derives from `params`. Panics as `solve_island` does on
    /// zero solver iterations (`IntegrationParameters`) and negative parameters
    /// (`Island: negative parameter`).
    fn new(params: IntegrationParameters, gravity: Vec2) -> FreeBodySolver {
        let dt = params.substep_dt();
        let max_lin = params.max_linear_velocity();
        let max_corrective = params.max_corrective_velocity();
        assert(params.dt >= ZERO && max_lin >= ZERO && max_corrective >= ZERO, errors::NEGATIVE);
        // π/4 per FULL frame, exactly upstream's MAX_ROTATION policy.
        let max_ang = Fixed { raw: 3373259426 } * params.inv_dt();
        FreeBodySolver {
            gravity,
            full_dt: params.dt,
            dt,
            max_lin,
            max_ang,
            iterations: params.num_solver_iterations,
        }
    }

    /// `rb` after `SolverBodyStoreTrait::from_bodies`, `solve_island` and `to_bodies`, when no
    /// manifold and no enabled joint of the step references `handle`: per substep the force
    /// increment then the capped integration, then full-step damping, then velocities and
    /// `next_position` written back (none for a fixed or disabled body). Bit-identical to the
    /// store path: same expressions in the same order (products floor, divisions round to
    /// nearest, rotations renormalize). Zero dt leaves the velocities and pose unintegrated.
    /// The default 4 substeps are unrolled (−28k gas per body against the loop,
    /// `free_alternatives::solve_looped`); other counts loop. Inlined: the body is not copied
    /// into a call.
    #[inline(always)]
    fn solve(self: @FreeBodySolver, handle: Handle, rb: RigidBody) -> RigidBody {
        let (mut b, step) = gather(handle, rb, *self.gravity, *self.dt);
        let mut rb = rb;
        if !step.moving {
            return rb;
        }
        let full_dt = *self.full_dt;
        if full_dt != ZERO {
            let (dt, max_lin, max_ang) = (*self.dt, *self.max_lin, *self.max_ang);
            let iterations = *self.iterations;
            if iterations == 4 {
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
                while substep != iterations {
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
}

#[cfg(test)]
mod benches;

#[cfg(test)]
mod fixtures;

#[cfg(test)]
mod free_alternatives;

#[cfg(test)]
mod free_checks;

#[cfg(test)]
mod idle_benches;

#[cfg(test)]
mod tests;
