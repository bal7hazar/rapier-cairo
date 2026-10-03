//! `TrianglePseudoNormals` (Parry `shape/triangle_pseudo_normals.rs`; PX6): the pseudo-normals of
//! a triangle of a mesh, approximating the outward normal cones of its three edges and vertices.
//!
//! Upstream builds them for the triangles of a 3D triangle mesh; the type itself is dimension
//! free (`Vector` is `Vec2` in 2D, and `edges` still holds three entries), so it is ported as is.
//! The projection is [`project_into_cone`] of the closest of the three pseudo-normals (the first
//! one on a tie, as nalgebra's `max_position`).
//!
//! # Deviations
//!
//! * None; `NormalConstraints` for it is [`LocalNormalProjector`], see
//!   [`crate::query::normal_constraints`].

use fixed::ZERO;
use glam_core::{Vec2, Vec2Trait};
use crate::query::normal_constraints::LocalNormalProjector;
use crate::shape::pseudo_normals::project_into_cone;

/// The pseudo-normals of a triangle (upstream `TrianglePseudoNormals`): `face` is the triangle's
/// outward normal and `edges` the pseudo-normals at its three edges.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct TrianglePseudoNormals {
    /// The triangle's outward normal.
    pub face: Vec2,
    /// The pseudo-normals of the triangle's three edges.
    pub edges: [Vec2; 3],
    /// Whether the back of the triangle is constrained like its front (mirrored cone).
    pub two_sided: bool,
}

/// The projection onto the front cone: the closest pseudo-normal bounds it.
fn project_front(pn: @TrianglePseudoNormals, dir: Vec2) -> (bool, Vec2) {
    let [e0, e1, e2] = *pn.edges;
    let d0 = dir.dot(e0);
    let d1 = dir.dot(e1);
    let d2 = dir.dot(e2);
    // The first maximum wins.
    let closest_edge = if d1 > d0 {
        if d2 > d1 {
            e2
        } else {
            e1
        }
    } else if d2 > d0 {
        e2
    } else {
        e0
    };
    project_into_cone(*pn.face, closest_edge, dir)
}

/// Upstream `impl NormalConstraints for TrianglePseudoNormals`. A two-sided triangle projects a
/// direction of its back (`dir . face < 0`) through the front cone mirrored through its plane and
/// always accepts it.
pub impl TrianglePseudoNormalsProjector of LocalNormalProjector<TrianglePseudoNormals> {
    fn project_local_normal_mut(self: @TrianglePseudoNormals, normal: Vec2) -> (bool, Vec2) {
        if *self.two_sided && normal.dot(*self.face) < ZERO {
            // The back cone is the front cone mirrored through the triangle's plane.
            let (_, mirrored) = project_front(self, -normal);
            return (true, -mirrored);
        }
        project_front(self, normal)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait};
    use glam_core::Vec2;
    use rapier_testing::opaque;
    use crate::query::normal_constraints::LocalNormalProjector;
    use super::TrianglePseudoNormals;

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

    /// Three identical edges: the cone of +Y bounded at +-45 degrees (see the segment test).
    fn cone() -> [Vec2; 3] {
        let edge = vr(1643612827, 3968032378);
        [edge, edge, edge]
    }

    #[test]
    fn test_trivial_pseudo_normals_projection() {
        // Upstream's `trivial_pseudo_normals_projection`: every edge is the face, the cone is +Y.
        let pn = TrianglePseudoNormals {
            face: v(0, 1), edges: [v(0, 1), v(0, 1), v(0, 1)], two_sided: false,
        };
        assert_eq!(pn.project_local_normal_mut(v(1, 1)), (true, v(0, 1)));
        let (accepted, _) = pn.project_local_normal_mut(v(0, -1));
        assert!(!accepted);
    }

    #[test]
    fn test_edge_pseudo_normals_projection() {
        // The expected values come from an f64 evaluation of upstream's `project_into_cone`.
        let pn = TrianglePseudoNormals { face: v(0, 1), edges: cone(), two_sided: false };
        // (normal, accepted, projected): inside (kept), sideways (pulled onto the boundary),
        // into the solid (rejected, still projected).
        let cases: Span<(Vec2, bool, Vec2)> = array![
            (vr(842312387, 4211561933), true, vr(842312387, 4211561933)),
            (vr(4211561933, 842312387), true, vr(3037000500, 3037000500)),
            (vr(1234149771, -4113832570), false, vr(3037000500, 3037000500)),
        ]
            .span();
        for (normal, accepted, projected) in cases {
            let (ok, out) = pn.project_local_normal_mut(*normal);
            assert_eq!(ok, *accepted);
            assert!(close(out, *projected), "projected {:?}", out);
        }
    }

    #[test]
    fn test_closest_edge_wins_and_the_first_on_a_tie() {
        // The edges are the degenerate one (+Y), the 22.5 degree one and its mirror: a direction
        // near +X is bounded by the second, one near -X by the third.
        let right = vr(1643612827, 3968032378);
        let left = vr(-1643612827, 3968032378);
        let pn = TrianglePseudoNormals {
            face: v(0, 1), edges: [v(0, 1), right, left], two_sided: false,
        };
        let (_, out) = pn.project_local_normal_mut(vr(4211561933, 842312387));
        assert!(close(out, vr(3037000500, 3037000500)), "right {:?}", out);
        let (_, out) = pn.project_local_normal_mut(vr(-4211561933, 842312387));
        assert!(close(out, vr(-3037000500, 3037000500)), "left {:?}", out);
        // `(1, 0)` has the dots 0, 0, -0.38 with the edges [+Y, +Y, left]: the first maximum wins
        // (the degenerate cone collapses to the face, and `(1, 0) . face = 0` is accepted).
        let tied = TrianglePseudoNormals {
            face: v(0, 1), edges: [v(0, 1), v(0, 1), left], two_sided: false,
        };
        assert_eq!(tied.project_local_normal_mut(v(1, 0)), (true, v(0, 1)));
    }

    #[test]
    fn test_two_sided_pseudo_normals_mirror_the_cone() {
        let one_sided = TrianglePseudoNormals {
            face: v(0, 1), edges: [v(0, 1), v(0, 1), v(0, 1)], two_sided: false,
        };
        let two_sided = TrianglePseudoNormals { two_sided: true, ..one_sided };
        // Front directions behave the same.
        assert_eq!(two_sided.project_local_normal_mut(v(1, 1)), (true, v(0, 1)));
        // Back directions are kept and constrained to the mirrored cone.
        let (accepted, _) = one_sided.project_local_normal_mut(v(1, -1));
        assert!(!accepted);
        assert_eq!(two_sided.project_local_normal_mut(v(1, -1)), (true, v(0, -1)));
        // The mirrored projection is the negation of the front projection.
        let pn = TrianglePseudoNormals { face: v(0, 1), edges: cone(), two_sided: true };
        let dir = vr(4211561933, 842312387);
        let (_, front) = pn.project_local_normal_mut(dir);
        let (_, back) = pn.project_local_normal_mut(-dir);
        assert!(close(back, -front), "back {:?}", back);
    }

    #[test]
    fn gas_baseline() {}
    #[test]
    fn gas_project_local_normal_mut() {
        let pn = TrianglePseudoNormals { face: v(0, 1), edges: cone(), two_sided: false };
        let _ = opaque(pn).project_local_normal_mut(opaque(vr(4211561933, 842312387)));
    }
    #[test]
    fn gas_project_local_normal_mut_two_sided() {
        let pn = TrianglePseudoNormals { face: v(0, 1), edges: cone(), two_sided: true };
        let _ = opaque(pn).project_local_normal_mut(opaque(vr(4211561933, -842312387)));
    }
}
