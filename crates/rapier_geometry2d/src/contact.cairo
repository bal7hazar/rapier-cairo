//! Contact points, manifolds and the per-manifold solver data shared with the dynamics crate.
//!
//! Frozen in `docs/interfaces/geometry-dynamics.md` §2–3 (Parry's `TrackedContact` /
//! `ContactManifold`, Rapier's `ContactData` / `ContactManifoldData` / `SolverContact`).
//! Invariants every manifold generator must respect: `local_n1` and `local_n2` are unit within
//! `rapier_math::consts::UNIT_TOL_SQ_RAW`; `local_n2 == -pos12.rotation.inverse_rotate(local_n1)`;
//! points beyond the prediction distance may be present (upstream keeps them); a manifold with
//! `num_points == 0` is legal and is ignored by the solver, never treated as an error.

use fixed::Fixed;
use glam::vec2::Vec2;
use rapier_core::data::handle::Handle;
use crate::feature_id::FeatureId;

/// Maximum number of points of a 2D manifold.
pub const MAX_MANIFOLD_POINTS: u8 = 2;

/// Set on `SolverContact::contact_id` when the point had no match in the previous manifold
/// (no warm start, restitution allowed). Upstream `NEW_CONTACT_BIT`.
pub const NEW_CONTACT_BIT: u32 = 0x8000_0000;

/// Solver state persisted on a contact point between steps (Rapier `ContactData`).
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct ContactData {
    pub impulse: Fixed,
    /// One tangent direction in 2D.
    pub tangent_impulse: Fixed,
    pub warmstart_impulse: Fixed,
    pub warmstart_tangent_impulse: Fixed,
}

/// One tracked contact point, in the local frames of the two shapes (Parry `TrackedContact`).
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct TrackedContact {
    pub local_p1: Vec2,
    pub local_p2: Vec2,
    /// Signed distance along the manifold normal: `< 0` penetration, `> 0` separation
    /// (speculative contact), `0` touching.
    pub dist: Fixed,
    pub fid1: FeatureId,
    pub fid2: FeatureId,
    pub data: ContactData,
}

/// Rapier `SolverFlags`: bit 0 = `COMPUTE_RIGID_IMPULSES`.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct SolverFlags {
    pub bits: u32,
}

pub const SOLVER_COMPUTE_RIGID_IMPULSES: u32 = 1;

/// What the constraint builder reads for one point (Rapier `SolverContact`, one lane).
/// `anchor1`/`anchor2` are the contact point relative to each body's centre of mass, in world
/// space.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct SolverContact {
    pub anchor1: Vec2,
    pub anchor2: Vec2,
    pub dist: Fixed,
    pub tangent_velocity: Vec2,
    /// Index of the `TrackedContact` in the manifold, plus [`NEW_CONTACT_BIT`] for new points.
    pub contact_id: u32,
}

/// Rapier `ContactManifoldData` minus the parallel-solver fields (colour, graph position,
/// solver body ids), which the sequential solver does not need.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct ContactManifoldData {
    pub rigid_body1: Option<Handle>,
    pub rigid_body2: Option<Handle>,
    pub solver_flags: SolverFlags,
    /// World-space normal from body 1 to body 2, refreshed by the narrow phase each step.
    pub normal: Vec2,
    pub solver_contacts: [SolverContact; 2],
    pub num_solver_contacts: u8,
    pub relative_dominance: i16,
    pub user_data: u32,
    pub friction: Fixed,
    pub restitution: Fixed,
}

/// A contact manifold between two (sub)shapes (Parry `ContactManifold`, 2D: at most 2 points).
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct ContactManifold {
    /// Points in generation order; only the first `num_points` are meaningful.
    pub points: [TrackedContact; 2],
    pub num_points: u8,
    /// Manifold normal in the local frame of shape 1 / shape 2 (unit, pointing from 1 to 2).
    pub local_n1: Vec2,
    pub local_n2: Vec2,
    /// Subshape indices (compound shapes; 0 for simple shapes).
    pub subshape1: u32,
    pub subshape2: u32,
    pub data: ContactManifoldData,
}

#[generate_trait]
pub impl ContactManifoldImpl of ContactManifoldTrait {
    /// The `i`-th point (`i < num_points`).
    fn point(self: @ContactManifold, i: u8) -> TrackedContact {
        let points = *self.points;
        if i == 0 {
            let [p0, _] = points;
            p0
        } else {
            let [_, p1] = points;
            p1
        }
    }

    /// Forgets every point; normals and data are kept (upstream `clear`).
    fn clear(ref self: ContactManifold) {
        self.num_points = 0;
    }
}

#[cfg(test)]
mod tests {
    use fixed::{FixedTrait, ONE};
    use glam::vec2::vec2;
    use rapier_testing::opaque;
    use super::{ContactManifold, ContactManifoldTrait, TrackedContact};

    #[test]
    fn test_default_manifold_is_empty_and_points_are_addressable() {
        let mut m: ContactManifold = Default::default();
        assert_eq!(m.num_points, 0);
        let p = TrackedContact { dist: ONE, ..Default::default() };
        m.points = [p, Default::default()];
        m.num_points = 1;
        assert_eq!(m.point(0).dist, ONE);
        assert_eq!(m.point(1).dist, FixedTrait::from_raw(0));
        m.clear();
        assert_eq!(m.num_points, 0);
        assert_eq!(m.point(0).dist, ONE);
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }

    #[test]
    fn gas_manifold_default_and_point() {
        let mut m: ContactManifold = Default::default();
        m.local_n1 = vec2(opaque(ONE), FixedTrait::from_raw(0));
        let _ = m.point(opaque(1_u8));
    }
}
