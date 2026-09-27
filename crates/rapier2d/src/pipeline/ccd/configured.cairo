//! The CCD step compiled with a [`StepConfig`] (CS2, `crate::pipeline::config`): the parent's
//! entry points with `step_with::<C>` as the regular step, so that a game with CCD also leaves
//! out the strategies it does not use. With `DefaultStepConfig` they are
//! `super::step_with_ccd` / `super::step_with_ccd_and_force_events`.

use rapier_dynamics2d::events::{CollisionEvent, ContactForceEvent};
use crate::pipeline::config::StepConfig;
use crate::pipeline::force_events::{CollisionOnly, WithForces};
use crate::pipeline::{step_with, step_with_force_events_with};
use crate::world::World;
use super::{CCDSolver, CCDSolverTrait, step_ccd};

/// `super::step_with_ccd` compiled with `C`.
///
/// # Panics
/// As `super::step_with_ccd`, and as `crate::pipeline::step_with`.
pub fn step_with_ccd_with<impl C: StepConfig>(
    ref world: World, ref ccd_solver: CCDSolver,
) -> Array<CollisionEvent> {
    if world.integration_parameters.max_ccd_substeps == 0 {
        return step_with::<C>(ref world);
    }
    ccd_solver.refresh(ref world);
    if ccd_solver.candidates.is_empty() {
        let output = step_with::<C>(ref world);
        ccd_solver.settle(ref world);
        return output;
    }
    step_ccd::<Array<CollisionEvent>, CollisionOnly, C>(ref world, ref ccd_solver)
}

/// `super::step_with_ccd_and_force_events` compiled with `C`. Panics as
/// [`step_with_ccd_with`].
pub fn step_with_ccd_and_force_events_with<impl C: StepConfig>(
    ref world: World, ref ccd_solver: CCDSolver,
) -> (Array<CollisionEvent>, Array<ContactForceEvent>) {
    if world.integration_parameters.max_ccd_substeps == 0 {
        return step_with_force_events_with::<C>(ref world);
    }
    ccd_solver.refresh(ref world);
    if ccd_solver.candidates.is_empty() {
        let output = step_with_force_events_with::<C>(ref world);
        ccd_solver.settle(ref world);
        return output;
    }
    step_ccd::<
        (Array<CollisionEvent>, Array<ContactForceEvent>), WithForces, C,
    >(ref world, ref ccd_solver)
}
