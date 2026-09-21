//! Public 2D facade of rapier.cairo (wave 5, `docs/PLAN.md`): the `World` bundle, the step
//! pipeline and the dispatcher glue between `rapier_geometry2d` and `rapier_dynamics2d`.
//! Modules are pre-declared per work package (see `docs/briefs/`).

pub mod dispatcher;
pub mod pipeline;
pub mod world;
