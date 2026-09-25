//! Upstream's pipeline objects (`pipeline/collision_pipeline.rs`, `CollisionPipeline`, and
//! `pipeline/physics_pipeline/mod.rs`, `PhysicsPipeline`), as stateless handles over the stages
//! of `crate::pipeline`.
//!
//! Upstream keeps per-pipeline scratch (and the quarantine, a thread pool); the port rebuilds
//! every per-step buffer inside the step (`docs/PLAN.md` D7/D9), so both are empty structs and
//! their `step` only forwards. [`CollisionPipelineTrait::step`] is collision detection without
//! dynamics: the user changes (stage 1) then the broad and narrow phases with their collision
//! events (stage 2, `staged::detect_collisions_with_prediction`); no island, solver or position
//! update. As upstream's, it wakes bodies through the user changes only.

use fixed::Fixed;
use rapier_dynamics2d::collider_set::ColliderSet;
use rapier_dynamics2d::events::CollisionEvent;
use rapier_dynamics2d::narrow_phase::NarrowPhase;
use rapier_dynamics2d::rigid_body_set::RigidBodySet;
use crate::world::World;
use super::{detect_collisions_with_prediction, handle_user_changes};

/// Collision detection without dynamics (upstream `CollisionPipeline`). Stateless.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct CollisionPipeline {}

/// Upstream `Default`: [`CollisionPipelineTrait::new`].
pub impl CollisionPipelineDefault of Default<CollisionPipeline> {
    #[inline(always)]
    fn default() -> CollisionPipeline {
        CollisionPipeline {}
    }
}

#[generate_trait]
pub impl CollisionPipelineImpl of CollisionPipelineTrait {
    /// A collision pipeline.
    #[inline(always)]
    fn new() -> CollisionPipeline {
        CollisionPipeline {}
    }

    /// Upstream `CollisionPipeline::step`: applies the user changes of `bodies` and `colliders`
    /// (poses propagated to the colliders, masses recomputed, change flags cleared, parents and
    /// contact partners of touched colliders woken up), then runs the broad phase over AABBs
    /// loosened by `prediction_distance / 2` and the narrow phase on its pairs, updating
    /// `narrow_phase` (contact manifolds, sensor pairs; dormant pairs of sleeping bodies kept)
    /// and returning the collision events, in the order of `NarrowPhaseTrait::compute_contacts`.
    /// Bodies do not move. Equals the first two stages of `World::step` on the same state.
    ///
    /// # Panics
    /// As the stages: fixed-point overflow, a negative prediction distance.
    fn step(
        self: @CollisionPipeline,
        prediction_distance: Fixed,
        ref narrow_phase: NarrowPhase,
        ref bodies: RigidBodySet,
        ref colliders: ColliderSet,
    ) -> Array<CollisionEvent> {
        handle_user_changes(ref bodies, ref colliders, narrow_phase.pairs.span());
        detect_collisions_with_prediction(
            prediction_distance, ref bodies, ref colliders, ref narrow_phase,
        )
    }
}

/// The full step (upstream `PhysicsPipeline`). Stateless: [`PhysicsPipelineTrait::step`] is
/// `crate::pipeline::step`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct PhysicsPipeline {}

/// Upstream `Default`: [`PhysicsPipelineTrait::new`].
pub impl PhysicsPipelineDefault of Default<PhysicsPipeline> {
    #[inline(always)]
    fn default() -> PhysicsPipeline {
        PhysicsPipeline {}
    }
}

#[generate_trait]
pub impl PhysicsPipelineImpl of PhysicsPipelineTrait {
    /// A physics pipeline.
    #[inline(always)]
    fn new() -> PhysicsPipeline {
        PhysicsPipeline {}
    }

    /// One step of `world` (upstream `PhysicsPipeline::step` on the world's sets); returns the
    /// collision events. Same as `WorldTrait::step`.
    ///
    /// # Panics
    /// As `crate::pipeline::step`.
    #[inline(always)]
    fn step(self: @PhysicsPipeline, ref world: World) -> Array<CollisionEvent> {
        super::step(ref world)
    }
}

#[cfg(test)]
mod tests;
