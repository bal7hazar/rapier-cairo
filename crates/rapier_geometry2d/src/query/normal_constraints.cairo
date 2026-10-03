//! Normal constraints of contact manifolds (Parry `query/contact_manifolds/normals_constraint.rs`;
//! PX6).
//!
//! A shape part that restricts the directions a contact normal may take (an oriented polyline's
//! segment, a triangle with pseudo-normals) implements [`LocalNormalProjector`], the one required
//! method of upstream's `NormalConstraints`; the trait [`NormalConstraints`] and its provided
//! methods (`project_local_normal`, `project_local_normal1`, `project_local_normal2`) are then
//! available for it through [`NormalConstraintsFromProjector`].
//!
//! # Deviations
//!
//! * Cairo has no `&mut`: `project_local_normal_mut(&self, &mut normal) -> bool` answers
//!   `(bool, normal)` (the projected normal, `normal` itself when the projection keeps it);
//!   `project_local_normal1` / `2` answer `(bool, normal1, normal2)` the same way.
//! * Upstream's `Option<&dyn NormalConstraints>` pair is `(Option<A>, Option<B>)` of values.

use glam_core::Vec2;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2Trait;

/// The required method of `NormalConstraints`: projects the unit local `normal` onto the cone of
/// allowed directions. Answers `(accepted, projected normal)`.
pub trait LocalNormalProjector<T> {
    fn project_local_normal_mut(self: @T, normal: Vec2) -> (bool, Vec2);
}

/// Upstream `NormalConstraints`: a restriction of the directions a contact normal may take.
pub trait NormalConstraints<T> {
    /// Upstream `project_local_normal_mut`: `(accepted, projected normal)`.
    fn project_local_normal_mut(self: @T, normal: Vec2) -> (bool, Vec2);
    /// The projected normal, or `None` when the projection rejects it (upstream
    /// `project_local_normal`).
    fn project_local_normal(self: @T, normal: Vec2) -> Option<Vec2>;
    /// Projects `normal1` (the normal of shape 1 in its local frame) and recomputes `normal2`
    /// from it with `pos12` (upstream `project_local_normal1`; `normal2` is overwritten, the
    /// normal is assumed to be unit-sized). Answers `(accepted, normal1, normal2)`.
    fn project_local_normal1(
        self: @T, pos12: Pose2, normal1: Vec2, normal2: Vec2,
    ) -> (bool, Vec2, Vec2);
    /// Projects `normal2` and recomputes `normal1` from it with `pos12` (upstream
    /// `project_local_normal2`). Answers `(accepted, normal1, normal2)`.
    fn project_local_normal2(
        self: @T, pos12: Pose2, normal1: Vec2, normal2: Vec2,
    ) -> (bool, Vec2, Vec2);
}

/// Every projector provides the `NormalConstraints` methods, as upstream's trait does.
pub impl NormalConstraintsFromProjector<T, +LocalNormalProjector<T>> of NormalConstraints<T> {
    #[inline(always)]
    fn project_local_normal_mut(self: @T, normal: Vec2) -> (bool, Vec2) {
        LocalNormalProjector::project_local_normal_mut(self, normal)
    }

    fn project_local_normal(self: @T, normal: Vec2) -> Option<Vec2> {
        let (accepted, projected) = LocalNormalProjector::project_local_normal_mut(self, normal);
        if accepted {
            Some(projected)
        } else {
            None
        }
    }

    fn project_local_normal1(
        self: @T, pos12: Pose2, normal1: Vec2, normal2: Vec2,
    ) -> (bool, Vec2, Vec2) {
        let (accepted, projected) = LocalNormalProjector::project_local_normal_mut(self, normal1);
        if !accepted {
            return (false, projected, normal2);
        }
        (true, projected, pos12.rotation.inverse_rotate(-projected))
    }

    fn project_local_normal2(
        self: @T, pos12: Pose2, normal1: Vec2, normal2: Vec2,
    ) -> (bool, Vec2, Vec2) {
        let (accepted, projected) = LocalNormalProjector::project_local_normal_mut(self, normal2);
        if !accepted {
            return (false, normal1, projected);
        }
        (true, pos12.rotation.rotate(-projected), projected)
    }
}

/// The unit constraint (upstream `impl NormalConstraints for ()`): every normal is kept.
pub impl UnitLocalNormalProjector of LocalNormalProjector<()> {
    #[inline(always)]
    fn project_local_normal_mut(self: @(), normal: Vec2) -> (bool, Vec2) {
        (true, normal)
    }
}

/// Upstream `NormalConstraintsPair`: the constraints of the two shapes of a pair.
pub trait NormalConstraintsPair<T> {
    /// Projects the two normals through the constraints of shape 1 then shape 2 (upstream
    /// `project_local_normals`). Answers `(accepted, normal1, normal2)`; a rejection by the
    /// first constraint stops before the second is applied.
    fn project_local_normals(
        self: @T, pos12: Pose2, normal1: Vec2, normal2: Vec2,
    ) -> (bool, Vec2, Vec2);
}

/// Upstream's `impl NormalConstraintsPair for (Option<&dyn NormalConstraints>, Option<&dyn
/// NormalConstraints>)`, over values.
pub impl OptionPairNormalConstraintsPair<
    A, B, +NormalConstraints<A>, +NormalConstraints<B>, +Drop<A>, +Drop<B>,
> of NormalConstraintsPair<(Option<A>, Option<B>)> {
    fn project_local_normals(
        self: @(Option<A>, Option<B>), pos12: Pose2, normal1: Vec2, normal2: Vec2,
    ) -> (bool, Vec2, Vec2) {
        let (first, second) = self;
        let mut normal1 = normal1;
        let mut normal2 = normal2;
        if let Some(proj) = first {
            let (accepted, n1, n2) = NormalConstraints::project_local_normal1(
                proj, pos12, normal1, normal2,
            );
            if !accepted {
                return (false, n1, n2);
            }
            normal1 = n1;
            normal2 = n2;
        }
        if let Some(proj) = second {
            NormalConstraints::project_local_normal2(proj, pos12, normal1, normal2)
        } else {
            (true, normal1, normal2)
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, ONE, ZERO};
    use glam_core::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::shape::segment::SegmentPseudoNormals;
    use super::{NormalConstraints, NormalConstraintsPair};

    fn v(x: i32, y: i32) -> Vec2 {
        Vec2 { x: FixedTrait::from_int(x), y: FixedTrait::from_int(y) }
    }

    fn vr(x: i64, y: i64) -> Vec2 {
        Vec2 { x: Fixed { raw: x }, y: Fixed { raw: y } }
    }

    fn close(a: Vec2, b: Vec2) -> bool {
        let tol = 64;
        (a.x - b.x).abs().raw <= tol && (a.y - b.y).abs().raw <= tol
    }

    /// The outward cone of +Y bounded at +-45 degrees (end-point pseudo-normals at +-22.5).
    fn cone() -> SegmentPseudoNormals {
        SegmentPseudoNormals {
            face: v(0, 1), edges: [vr(1643612827, 3968032378), vr(-1643612827, 3968032378)],
        }
    }

    /// A quarter turn: `rotate(x, y) = (-y, x)`.
    fn quarter_turn() -> Pose2 {
        Pose2Trait::new(v(10, 0), Rot2 { re: ZERO, im: ONE })
    }

    /// `(1, 0.2) / |.|`, outward but outside the cone; its projection is `(1, 1) / sqrt(2)`.
    fn sideways() -> Vec2 {
        vr(4211561933, 842312387)
    }

    fn boundary() -> Vec2 {
        vr(3037000500, 3037000500)
    }

    /// `(0.3, -1) / |.|`: into the solid, rejected.
    fn solid() -> Vec2 {
        vr(1234149771, -4113832570)
    }

    #[test]
    fn test_unit_constraint_keeps_every_normal() {
        let n = v(3, -4);
        assert_eq!(NormalConstraints::project_local_normal_mut(@(), n), (true, n));
        assert_eq!(NormalConstraints::project_local_normal(@(), n), Some(n));
        let pos12 = quarter_turn();
        assert_eq!(
            NormalConstraints::project_local_normal1(@(), pos12, v(0, 1), v(5, 5)),
            (true, v(0, 1), v(-1, 0)),
        );
    }

    #[test]
    fn test_project_local_normal() {
        let pn = cone();
        assert_eq!(NormalConstraints::project_local_normal(@pn, v(0, 1)), Some(v(0, 1)));
        let projected = NormalConstraints::project_local_normal(@pn, sideways()).unwrap();
        assert!(close(projected, boundary()), "{:?}", projected);
        assert!(NormalConstraints::project_local_normal(@pn, solid()).is_none());
        // `_mut` is the same projection, with the projected normal of a rejection as well.
        let (accepted, out) = NormalConstraints::project_local_normal_mut(@pn, solid());
        assert!(!accepted);
        assert!(close(out, boundary()), "{:?}", out);
    }

    #[test]
    fn test_project_local_normal1_and_2() {
        let pn = cone();
        let pos12 = quarter_turn();
        // Shape 1 constrained: `normal1` is projected, `normal2 = rotation^-1 * -normal1`
        // (`inverse_rotate(x, y) = (y, -x)`), whatever `normal2` was.
        let (ok, n1, n2) = NormalConstraints::project_local_normal1(
            @pn, pos12, sideways(), v(5, 5),
        );
        assert!(ok);
        assert!(close(n1, boundary()), "n1 {:?}", n1);
        assert!(close(n2, vr(-3037000500, 3037000500)), "n2 {:?}", n2);
        // Shape 2 constrained: `normal2` is projected, `normal1 = rotation * -normal2`.
        let (ok, n1, n2) = NormalConstraints::project_local_normal2(
            @pn, pos12, v(5, 5), sideways(),
        );
        assert!(ok);
        assert!(close(n2, boundary()), "n2 {:?}", n2);
        assert!(close(n1, vr(3037000500, -3037000500)), "n1 {:?}", n1);
        // A rejection leaves the other normal as it was.
        let (ok, _, n2) = NormalConstraints::project_local_normal1(@pn, pos12, solid(), v(5, 5));
        assert!(!ok);
        assert_eq!(n2, v(5, 5));
        let (ok, n1, _) = NormalConstraints::project_local_normal2(@pn, pos12, v(5, 5), solid());
        assert!(!ok);
        assert_eq!(n1, v(5, 5));
    }

    #[test]
    fn test_pair_applies_each_present_constraint() {
        let pn = cone();
        let pos12 = quarter_turn();
        let none: (Option<SegmentPseudoNormals>, Option<SegmentPseudoNormals>) = (None, None);
        assert_eq!(
            NormalConstraintsPair::project_local_normals(@none, pos12, v(3, 4), v(5, 6)),
            (true, v(3, 4), v(5, 6)),
        );
        // Only shape 1: as `project_local_normal1`.
        let first = (Some(pn), None::<SegmentPseudoNormals>);
        let (ok, n1, n2) = NormalConstraintsPair::project_local_normals(
            @first, pos12, sideways(), v(5, 5),
        );
        assert!(ok);
        assert!(close(n1, boundary()) && close(n2, vr(-3037000500, 3037000500)), "{:?}", n2);
        // Only shape 2: as `project_local_normal2`.
        let second = (None::<SegmentPseudoNormals>, Some(pn));
        let (ok, n1, n2) = NormalConstraintsPair::project_local_normals(
            @second, pos12, v(5, 5), sideways(),
        );
        assert!(ok);
        assert!(close(n2, boundary()) && close(n1, vr(3037000500, -3037000500)), "{:?}", n1);
        // Both: the second constraint sees the normal the first one produced; the second cone
        // (rotated by the quarter turn) accepts `rotation^-1 * -boundary = (-1, 1) / sqrt(2)`,
        // which is on the boundary of the cone of +Y.
        let both = (Some(pn), Some(pn));
        let (ok, n1, n2) = NormalConstraintsPair::project_local_normals(
            @both, pos12, sideways(), v(5, 5),
        );
        assert!(ok);
        assert!(close(n1, boundary()), "n1 {:?}", n1);
        assert!(close(n2, vr(-3037000500, 3037000500)), "n2 {:?}", n2);
        // A rejection by the first constraint stops there.
        let (ok, _, n2) = NormalConstraintsPair::project_local_normals(
            @both, pos12, solid(), v(5, 5),
        );
        assert!(!ok);
        assert_eq!(n2, v(5, 5));
    }

    #[test]
    fn gas_baseline() {}
    #[test]
    fn gas_project_local_normal() {
        let _ = NormalConstraints::project_local_normal(@opaque(cone()), opaque(sideways()));
    }
    #[test]
    fn gas_project_local_normal1() {
        let _ = NormalConstraints::project_local_normal1(
            @opaque(cone()), opaque(quarter_turn()), opaque(sideways()), opaque(v(5, 5)),
        );
    }
    #[test]
    fn gas_project_local_normal2() {
        let _ = NormalConstraints::project_local_normal2(
            @opaque(cone()), opaque(quarter_turn()), opaque(v(5, 5)), opaque(sideways()),
        );
    }
    #[test]
    fn gas_project_local_normals() {
        let pair = opaque((Some(cone()), Some(cone())));
        let _ = NormalConstraintsPair::project_local_normals(
            @pair, opaque(quarter_turn()), opaque(sideways()), opaque(v(5, 5)),
        );
    }
}
