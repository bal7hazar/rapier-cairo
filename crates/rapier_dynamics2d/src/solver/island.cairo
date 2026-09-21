//! Sequential whole-world soft-step driver (D8/D9). Manifolds and joints retain caller order;
//! each sweep solves joints before contacts. No sleeping, island discovery, CCD or colouring.
//! DC/DE's frozen array APIs operate on two-body scratch arrays; global scattering is O(1).
mod sweeps;
use fixed::{Fixed, MAX, ZERO};
use glam::Vec2Trait;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_geometry2d::contact::ContactManifold;
use sweeps::{contacts, joints, prepare_joints, rebuild_joints};
use crate::joint::ImpulseJoint;
use crate::rigid_body::{RigidBodyVelocity, RigidBodyVelocityTrait};
use super::body::{SolverBody, WORLD};
use super::body_store::{BodyStep, DenseBodiesTrait, SolverBodyStore};
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
/// Products floor, divisions truncate, rotations renormalize. All intermediates must fit
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
    let mut cs = ContactConstraintsSetTrait::generate(manifolds.span(), initial.span(), params, dt);
    let builders = prepare_joints(joint_set.span(), initial.span(), steps);
    let mut rows = array![];
    let mut substep = 0;
    while substep != params.num_solver_iterations {
        add_forces(ref bodies, steps);
        rows = rebuild_joints(ref bodies, builders.span(), rows.span(), params, substep != 0);
        contacts(ref cs, ref bodies, manifolds.span(), params, 0);
        let mut i = 0;
        while i != params.num_internal_pgs_iterations {
            joints(ref rows, ref bodies, true, params.warmstart_joints && i == 0);
            contacts(ref cs, ref bodies, manifolds.span(), params, 1);
            i += 1;
        }
        integrate(ref bodies, steps, dt, max_lin, max_ang);
        let mut i = 0;
        while i != params.num_internal_stabilization_iterations {
            joints(ref rows, ref bodies, false, false);
            contacts(ref cs, ref bodies, manifolds.span(), params, if i == 0 {
                2
            } else {
                3
            });
            i += 1;
        }
        substep += 1;
    }
    contacts(ref cs, ref bodies, manifolds.span(), params, 4);
    cs.writeback_impulses(ref manifolds);
    sweeps::write_joints(rows.span(), ref joint_set);
    damp(ref bodies, steps, params.dt);
}

fn add_forces<B, +DenseBodiesTrait<B>, +Destruct<B>>(ref bodies: B, steps: Span<BodyStep>) {
    let mut i = 0;
    let n = bodies.len();
    while i != n {
        if *steps.at(i).moving {
            let mut b = bodies.get(i);
            let dv = *steps.at(i).increment;
            b.linvel = b.linvel + dv.linvel;
            b.angvel += dv.angvel;
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
            let v = RigidBodyVelocity { linvel: b.linvel, angvel: b.angvel }
                .apply_damping(dt, *steps.at(i).damping);
            b.linvel = v.linvel;
            b.angvel = v.angvel;
            bodies.set_pair(i, b, WORLD, Default::default());
        }
        i += 1;
    }
}
#[cfg(test)]
mod benches;

#[cfg(test)]
mod fixtures;

#[cfg(test)]
mod tests;
