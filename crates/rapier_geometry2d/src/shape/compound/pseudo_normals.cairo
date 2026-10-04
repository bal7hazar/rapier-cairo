//! The outward normal cones of a [`Compound`](super::Compound)'s parts (Parry 0.31
//! `shape/compound_pseudo_normals.rs`, 2D; lot CE): one [`CompoundEdgeCone`] per boundary edge of a
//! polygonal part, cut edges dropped, so that a body sliding across the join of two parts does not
//! catch on a face that is not a surface.
//!
//! # Fixed point
//!
//! Every decision (a cone's containment, the turn between two directions, the best alignment) is
//! read off an exact wide dot or cross product of the raw components (`crate::point::wide2`), never
//! off a rounded `Fixed` product, so two directions within a few raw of each other compare the same
//! way on every run. The projected direction is one of the inputs (`dir` or a cone limit), never a
//! computed one.

use glam_core::Vec2;
use crate::point::wide2::{cross_wide, dot_wide};
use crate::query::normal_constraints::LocalNormalProjector;

/// Whether the turn from `from` to `to` is clockwise (upstream `turns_clockwise`, exact).
#[inline(always)]
pub fn turns_clockwise(from: Vec2, to: Vec2) -> bool {
    cross_wide(from.x, from.y, to.x, to.y) < 0
}

/// `a . b`, exact.
#[inline(always)]
fn dot(a: Vec2, b: Vec2) -> i128 {
    dot_wide(a.x, a.y, b.x, b.y)
}

/// The directions one boundary edge of a compound part may push along (upstream
/// `CompoundEdgeCone`).
///
/// Each limit reaches halfway towards the boundary edge next to it along the union's outline, which
/// may belong to another part where a cut splits the outline at that corner, so the two edges
/// meeting there split its range between them. A limit stops at `face` where the corner is concave,
/// which leaves no range to share, or where the surface simply ends against a cut. All three are
/// unit vectors in the part's frame.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct CompoundEdgeCone {
    /// The edge's outward normal.
    pub face: Vec2,
    /// How far the cone reaches at the edge's clockwise end.
    pub clockwise_limit: Vec2,
    /// How far the cone reaches at the edge's counter-clockwise end.
    pub counter_clockwise_limit: Vec2,
}

#[generate_trait]
pub impl CompoundEdgeConeImpl of CompoundEdgeConeTrait {
    /// Whether `dir` lies in the cone: in front of the face and between the two limits (upstream
    /// `contains`).
    fn contains(self: @CompoundEdgeCone, dir: Vec2) -> bool {
        dot(dir, *self.face) > 0
            && !turns_clockwise(*self.clockwise_limit, dir)
            && !turns_clockwise(dir, *self.counter_clockwise_limit)
    }

    /// The direction in the cone closest to `dir` (upstream `clamped`): `dir` itself when the cone
    /// contains it, else the limit it is better aligned with (the clockwise one on ties).
    fn clamped(self: @CompoundEdgeCone, dir: Vec2) -> Vec2 {
        if self.contains(dir) {
            dir
        } else if dot(dir, *self.clockwise_limit) >= dot(dir, *self.counter_clockwise_limit) {
            *self.clockwise_limit
        } else {
            *self.counter_clockwise_limit
        }
    }
}

/// The outward normal cones of one compound part (upstream `CompoundPseudoNormals`): one cone per
/// boundary edge, in the part's local frame; cut edges are absent. An empty span is a part buried
/// inside the union, which rejects every direction.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct CompoundPseudoNormals {
    pub boundary_edges: Span<CompoundEdgeCone>,
}

/// Upstream `impl NormalConstraints for CompoundPseudoNormals` (2D): projects `dir` onto the
/// closest direction the part's outline can face, the first best-aligned cone on ties. Rejected
/// (`false`, `dir` unchanged) when nothing faces that way (best alignment `<= 0`), so a contact
/// reached from inside the union is dropped rather than turned around.
pub impl CompoundPseudoNormalsLocalNormalProjector of LocalNormalProjector<CompoundPseudoNormals> {
    fn project_local_normal_mut(self: @CompoundPseudoNormals, normal: Vec2) -> (bool, Vec2) {
        let mut nearest: Option<(Vec2, i128)> = None;
        for edge in *self.boundary_edges {
            let candidate = edge.clamped(normal);
            let alignment = dot(normal, candidate);
            let better = match nearest {
                Some((_, best)) => alignment > best,
                None => true,
            };
            if better {
                nearest = Some((candidate, alignment));
            }
        }
        match nearest {
            Some((direction, alignment)) => if alignment > 0 {
                (true, direction)
            } else {
                (false, normal)
            },
            None => (false, normal),
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, ONE, ZERO};
    use glam_core::Vec2;
    use crate::query::normal_constraints::NormalConstraints;
    use super::{CompoundEdgeCone, CompoundPseudoNormals};

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn vr(x: i64, y: i64) -> Vec2 {
        Vec2 { x: Fixed { raw: x }, y: Fixed { raw: y } }
    }

    /// `(1, 1) / sqrt(2)` and `(-1, 1) / sqrt(2)`, rounded to nearest.
    fn diag() -> Vec2 {
        vr(3037000500, 3037000500)
    }

    fn anti_diag() -> Vec2 {
        vr(-3037000500, 3037000500)
    }

    fn cone(face: Vec2, cw: Vec2, ccw: Vec2) -> CompoundEdgeCone {
        CompoundEdgeCone { face, clockwise_limit: cw, counter_clockwise_limit: ccw }
    }

    /// A cone with nowhere to open, as where a surface runs into a cut at both ends.
    fn pinned(face: Vec2) -> CompoundEdgeCone {
        cone(face, face, face)
    }

    fn normals(edges: Array<CompoundEdgeCone>) -> CompoundPseudoNormals {
        CompoundPseudoNormals { boundary_edges: edges.span() }
    }

    /// Upstream's tests (`compound_pseudo_normals.rs`), as `(cones, dir, expected)`.
    #[test]
    fn test_projection_table() {
        let x = v(ONE, ZERO);
        let y = v(ZERO, ONE);
        // `(-0.6, 0.8)`, exact enough to be a unit ramp.
        let ramp = vr(-2576980378, 3435973837);
        let inside = vr(1233284375, 4110947916);
        let cases: Array<(CompoundPseudoNormals, Vec2, Option<Vec2>)> = array![
            // A buried part rejects everything.
            (normals(array![]), y, None), (normals(array![]), -y, None),
            // A direction already on the outline is left alone.
            (normals(array![cone(y, diag(), anti_diag())]), y, Some(y)),
            (normals(array![cone(y, diag(), anti_diag())]), inside, Some(inside)),
            // A direction past the corner is pulled back to it.
            (normals(array![cone(y, diag(), anti_diag())]), x, Some(diag())),
            // Picks the edge the normal points out of.
            (normals(array![pinned(y), pinned(x)]), y, Some(y)),
            (normals(array![pinned(y), pinned(x)]), x, Some(x)),
            // A direction reaching the part from behind is dropped.
            (
                normals(array![pinned(y)]), -y, None,
            ), // A wedge tip's normal is clamped onto its one boundary edge.
            (normals(array![pinned(ramp)]), -x, Some(ramp)),
        ];
        for (n, dir, expected) in cases {
            assert_eq!(NormalConstraints::project_local_normal(@n, dir), expected);
        }
    }

    /// A tie between two cones keeps the first, as upstream (`alignment <= best` skips).
    #[test]
    fn test_first_cone_wins_ties() {
        let n = normals(array![pinned(diag()), pinned(anti_diag())]);
        let up = v(ZERO, FixedTrait::from_int(1));
        assert_eq!(NormalConstraints::project_local_normal(@n, up), Some(diag()));
    }
}
