//! 2D rigid-body dynamics for rapier.cairo (port of the Rapier subset described in
//! `docs/PLAN.md`): body and collider sets, narrow-phase bookkeeping, joints, the soft-contact
//! substep solver. Modules are pre-declared per work package (see `docs/briefs/`).

pub mod rigid_body;
pub mod solver;
