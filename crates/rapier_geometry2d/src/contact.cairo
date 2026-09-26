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

    /// An empty manifold (upstream `ContactManifold::new`).
    #[inline(always)]
    fn new() -> ContactManifold {
        Default::default()
    }

    /// An empty manifold between two subshapes, carrying `data` (upstream `with_data`).
    #[inline(always)]
    fn with_data(subshape1: u32, subshape2: u32, data: ContactManifoldData) -> ContactManifold {
        ContactManifold { subshape1, subshape2, data, ..Default::default() }
    }

    /// A copy of the manifold, which is cleared (upstream `take`: the points move out, the
    /// normals, subshapes and data are copied).
    fn take(ref self: ContactManifold) -> ContactManifold {
        let taken = self;
        self.num_points = 0;
        taken
    }

    /// The `num_points` meaningful points, in generation order (upstream `contacts`).
    fn contacts(self: @ContactManifold) -> Span<TrackedContact> {
        let [p0, p1] = *self.points;
        let n = *self.num_points;
        if n == 0 {
            array![].span()
        } else if n == 1 {
            array![p0].span()
        } else {
            array![p0, p1].span()
        }
    }

    /// Zeroes `local_n2`, so that the next `try_update_contacts` regenerates the manifold
    /// (upstream `mark_shapes_deformed`).
    #[inline(always)]
    fn mark_shapes_deformed(ref self: ContactManifold) {
        self.local_n2 = Default::default();
    }
}

#[generate_trait]
pub impl TrackedContactImpl of TrackedContactTrait {
    /// A contact point with default solver data (upstream `TrackedContact::new`).
    #[inline(always)]
    fn new(
        local_p1: Vec2, local_p2: Vec2, fid1: FeatureId, fid2: FeatureId, dist: Fixed,
    ) -> TrackedContact {
        TrackedContact { local_p1, local_p2, dist, fid1, fid2, data: Default::default() }
    }

    /// [`TrackedContactTrait::new`] with the two sides swapped when `flipped` (upstream
    /// `TrackedContact::flipped`).
    #[inline(always)]
    fn flipped(
        local_p1: Vec2,
        local_p2: Vec2,
        fid1: FeatureId,
        fid2: FeatureId,
        dist: Fixed,
        flipped: bool,
    ) -> TrackedContact {
        if flipped {
            Self::new(local_p2, local_p1, fid2, fid1, dist)
        } else {
            Self::new(local_p1, local_p2, fid1, fid2, dist)
        }
    }

    /// Copies the geometry of `contact` (points, feature ids, distance) and keeps the solver
    /// data (upstream `copy_geometry_from`).
    #[inline(always)]
    fn copy_geometry_from(ref self: TrackedContact, contact: TrackedContact) {
        self.local_p1 = contact.local_p1;
        self.local_p2 = contact.local_p2;
        self.fid1 = contact.fid1;
        self.fid2 = contact.fid2;
        self.dist = contact.dist;
    }
}

#[generate_trait]
pub impl ContactManifoldDataImpl of ContactManifoldDataTrait {
    /// The number of solver contacts of the manifold (Rapier `num_active_contacts`).
    #[inline(always)]
    fn num_active_contacts(self: @ContactManifoldData) -> u32 {
        (*self.num_solver_contacts).into()
    }
}

/// Rapier's `ContactManifoldExt`.
pub trait ContactManifoldExt<T> {
    /// The sum of the normal impulses of the manifold's points.
    fn total_impulse(self: @T) -> Fixed;
}

pub impl ContactManifoldExtImpl of ContactManifoldExt<ContactManifold> {
    fn total_impulse(self: @ContactManifold) -> Fixed {
        let mut sum: Fixed = Default::default();
        for pt in self.contacts() {
            sum = sum + *pt.data.impulse;
        }
        sum
    }
}

#[cfg(test)]
mod tests {
    use fixed::{FixedTrait, ONE};
    use glam::vec2::vec2;
    use rapier_testing::opaque;
    use crate::feature_id::FeatureIdTrait;
    use super::{
        ContactData, ContactManifold, ContactManifoldData, ContactManifoldDataTrait,
        ContactManifoldExt, ContactManifoldTrait, TrackedContact, TrackedContactTrait,
    };

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
    fn test_manifold_utilities() {
        let fid1 = FeatureIdTrait::face(1);
        let fid2 = FeatureIdTrait::vertex(2);
        let (a, b) = (vec2(ONE, ONE), vec2(ONE + ONE, ONE));
        let p = TrackedContactTrait::new(a, b, fid1, fid2, ONE);
        assert_eq!(TrackedContactTrait::flipped(a, b, fid1, fid2, ONE, false), p);
        let q = TrackedContactTrait::flipped(a, b, fid1, fid2, ONE, true);
        assert_eq!((q.local_p1, q.local_p2, q.fid1, q.fid2), (b, a, fid2, fid1));
        let mut kept = TrackedContact {
            data: ContactData { impulse: ONE, ..Default::default() }, ..Default::default(),
        };
        kept.copy_geometry_from(q);
        assert_eq!((kept.local_p1, kept.fid2, kept.dist, kept.data.impulse), (b, fid1, ONE, ONE));
        let data = ContactManifoldData { num_solver_contacts: 2, ..Default::default() };
        assert_eq!(data.num_active_contacts(), 2);
        let mut m = ContactManifoldTrait::with_data(3, 4, data);
        assert_eq!((m.subshape1, m.subshape2, m.num_points, m.data), (3, 4, 0, data));
        assert_eq!(m.contacts().len(), 0);
        m.points = [p, kept];
        m.num_points = 2;
        m.local_n2 = vec2(ONE, ONE);
        assert_eq!(m.contacts(), array![p, kept].span());
        assert_eq!(m.total_impulse(), ONE);
        m.mark_shapes_deformed();
        assert_eq!(m.local_n2, Default::default());
        let taken = m.take();
        assert_eq!((taken.num_points, m.num_points, taken.subshape1), (2, 0, 3));
        assert_eq!(ContactManifoldTrait::new(), Default::default());
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }

    #[test]
    fn gas_manifold_contacts() {
        let mut m: ContactManifold = Default::default();
        m.num_points = opaque(2_u8);
        let _ = m.contacts();
    }

    #[test]
    fn gas_manifold_total_impulse() {
        let mut m: ContactManifold = Default::default();
        m.num_points = opaque(2_u8);
        let _ = m.total_impulse();
    }

    #[test]
    fn gas_manifold_default_and_point() {
        let mut m: ContactManifold = Default::default();
        m.local_n1 = vec2(opaque(ONE), FixedTrait::from_raw(0));
        let _ = m.point(opaque(1_u8));
    }
}
