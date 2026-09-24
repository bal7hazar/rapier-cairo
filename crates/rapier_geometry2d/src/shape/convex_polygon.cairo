//! Bounded, strictly convex CCW polygons. Construction rejects collinear vertices instead of
//! removing them as Parry does, and verifies every vertex against every edge (at most 64 tests).
//! Coordinates and differences must fit Q32.32; wide cross/dot sums must fit i128. Arithmetic
//! overflow panics. Padded slots are zero and do not participate in any query.

use fixed::Fixed;
use glam::{Vec2, Vec2Trait};
use rapier_math::math_ext::vec2::try_normalize2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::aabb::Aabb;
use crate::feature_id::FeatureIdTrait;
use crate::mass::{MassProperties, MassPropertiesTrait};
use crate::point::{cross_wide, dot_wide};
use crate::polygonal_feature::PolygonalFeature;

/// A strictly convex counter-clockwise polygon with 3 through 8 vertices. Use the constructor
/// to establish the count, convexity and outward unit-normal invariants.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ConvexPolygon {
    pub vertices: [Vec2; 8],
    pub normals: [Vec2; 8],
    pub count: u8,
}

/// Box serialization is the polygon value, with no address or allocation identity.
pub impl BoxedConvexPolygonSerde of Serde<Box<ConvexPolygon>> {
    fn serialize(self: @Box<ConvexPolygon>, ref output: Array<felt252>) {
        let value = (*self).unbox();
        value.serialize(ref output);
    }
    fn deserialize(ref serialized: Span<felt252>) -> Option<Box<ConvexPolygon>> {
        Some(BoxTrait::new(Serde::<ConvexPolygon>::deserialize(ref serialized)?))
    }
}
/// Structural equality of boxed polygons, independent of allocation identity.
pub impl BoxedConvexPolygonPartialEq of PartialEq<Box<ConvexPolygon>> {
    fn eq(lhs: @Box<ConvexPolygon>, rhs: @Box<ConvexPolygon>) -> bool {
        (*lhs).unbox() == (*rhs).unbox()
    }
    fn ne(lhs: @Box<ConvexPolygon>, rhs: @Box<ConvexPolygon>) -> bool {
        !Self::eq(lhs, rhs)
    }
}

/// Polygon invariant and arithmetic failures.
pub mod errors {
    /// Vertex index outside the live range.
    pub const VERTEX_INDEX: felt252 = 'Polygon: vertex index';
    /// Normal index outside the live range.
    pub const NORMAL_INDEX: felt252 = 'Polygon: normal index';
    /// Triangle area cannot be represented as Q32.32.
    pub const AREA_OVERFLOW: felt252 = 'Polygon: area overflow';
}

/// Fixed storage indexing without converting to an allocated span. Internal indices are < 8.
fn get(values: [Vec2; 8], i: u8) -> Vec2 {
    let [a, b, c, d, e, f, g, h] = values;
    match i {
        0 => a,
        1 => b,
        2 => c,
        3 => d,
        4 => e,
        5 => f,
        6 => g,
        _ => h,
    }
}

fn padded(points: Span<Vec2>, i: u32) -> Vec2 {
    if i < points.len() {
        *points.at(i)
    } else {
        Vec2Trait::ZERO
    }
}

#[generate_trait]
pub impl ConvexPolygonImpl of ConvexPolygonTrait {
    /// Constructs from 3..=8 CCW vertices. Returns `None` for duplicate, collinear, clockwise,
    /// concave or self-intersecting input; unlike upstream, redundant vertices are not removed.
    /// Normalisation rounds to nearest. Panics on coordinate differences / wide sums overflow.
    fn from_convex_polyline(points: Span<Vec2>) -> Option<ConvexPolygon> {
        let n = points.len();
        if n < 3 || n > 8 {
            return None;
        }
        let mut normals = array![];
        let mut i = 0;
        while i != n {
            let a = *points.at(i);
            let next = if i + 1 == n {
                0
            } else {
                i + 1
            };
            let edge = *points.at(next) - a;
            let mut j = 0;
            while j != n {
                if j != i && j != next {
                    let d = *points.at(j) - a;
                    if cross_wide(edge.x, edge.y, d.x, d.y) <= 0 {
                        return None;
                    }
                }
                j += 1;
            }
            let (x, y) = try_normalize2(edge.y, -edge.x)?;
            normals.append(Vec2 { x, y });
            i += 1;
        }
        let ns = normals.span();
        Some(
            ConvexPolygon {
                vertices: [
                    padded(points, 0), padded(points, 1), padded(points, 2), padded(points, 3),
                    padded(points, 4), padded(points, 5), padded(points, 6), padded(points, 7),
                ],
                normals: [
                    padded(ns, 0), padded(ns, 1), padded(ns, 2), padded(ns, 3), padded(ns, 4),
                    padded(ns, 5), padded(ns, 6), padded(ns, 7),
                ],
                count: n.try_into().unwrap(),
            },
        )
    }

    /// Padded vertex storage, with `count` live entries in CCW order.
    fn vertices(self: ConvexPolygon) -> [Vec2; 8] {
        self.vertices
    }
    /// Upstream `points`: padded vertex storage, with `count` live entries.
    fn points(self: ConvexPolygon) -> [Vec2; 8] {
        self.vertices
    }
    /// Padded outward unit normals; face i joins vertex i to the next vertex.
    fn normals(self: ConvexPolygon) -> [Vec2; 8] {
        self.normals
    }
    /// Number of live vertices (3..=8).
    fn count(self: ConvexPolygon) -> u8 {
        self.count
    }
    /// Vertex at `i < count`; panics for an invalid index.
    fn vertex(self: ConvexPolygon, i: u8) -> Vec2 {
        assert(i < self.count, errors::VERTEX_INDEX);
        get(self.vertices, i)
    }
    /// Outward unit face normal at `i < count`; panics for an invalid index.
    fn normal(self: ConvexPolygon, i: u8) -> Vec2 {
        assert(i < self.count, errors::NORMAL_INDEX);
        get(self.normals, i)
    }
    /// Successor of `i < count` with wraparound.
    fn next(self: ConvexPolygon, i: u8) -> u8 {
        if i + 1 == self.count {
            0
        } else {
            i + 1
        }
    }
    /// Maximum-dot vertex. Exact wide comparisons; lowest index wins ties, including zero dir.
    /// Panics on wide dot overflow at the scalar extremes.
    fn local_support_point(self: ConvexPolygon, dir: Vec2) -> Vec2 {
        let mut best = self.vertex(0);
        let mut score = dot_wide(best.x, best.y, dir.x, dir.y);
        let mut i = 1;
        while i != self.count {
            let p = self.vertex(i);
            let d = dot_wide(p.x, p.y, dir.x, dir.y);
            if d > score {
                best = p;
                score = d;
            }
            i += 1;
        }
        best
    }
    /// Support point alias; direction need not be normalised.
    fn support_point(self: ConvexPolygon, dir: Vec2) -> Vec2 {
        self.local_support_point(dir)
    }
    /// Most aligned face, with upstream PFM ids: vertex `2*i`, face `2*i+1`.
    /// Lowest face index wins equal wide dots. Panics as `local_support_point`.
    fn support_feature(self: ConvexPolygon, dir: Vec2) -> PolygonalFeature {
        let mut best = 0;
        let n = self.normal(0);
        let mut score = dot_wide(n.x, n.y, dir.x, dir.y);
        let mut i = 1;
        while i != self.count {
            let n = self.normal(i);
            let d = dot_wide(n.x, n.y, dir.x, dir.y);
            if d > score {
                best = i;
                score = d;
            }
            i += 1;
        }
        let next = self.next(best);
        PolygonalFeature {
            vertices: [self.vertex(best), self.vertex(next)],
            vids: [
                FeatureIdTrait::vertex(best.into() * 2), FeatureIdTrait::vertex(next.into() * 2),
            ],
            fid: FeatureIdTrait::face(best.into() * 2 + 1),
            num_vertices: 2,
        }
    }
    /// Local bounds, exact min/max of the live vertices.
    fn compute_local_aabb(self: ConvexPolygon) -> Aabb {
        let p = self.vertex(0);
        let mut bounds = Aabb { mins: p, maxs: p };
        let mut i = 1;
        while i != self.count {
            let p = self.vertex(i);
            bounds.mins = bounds.mins.min(p);
            bounds.maxs = bounds.maxs.max(p);
            i += 1;
        }
        bounds
    }
    /// Tight world bounds of all transformed vertices (one floor per transformed component).
    /// Panics if a transformed point exceeds Q32.32.
    fn compute_aabb(self: ConvexPolygon, pose: Pose2) -> Aabb {
        let p = pose.transform_point(self.vertex(0));
        let mut bounds = Aabb { mins: p, maxs: p };
        let mut i = 1;
        while i != self.count {
            let p = pose.transform_point(self.vertex(i));
            bounds.mins = bounds.mins.min(p);
            bounds.maxs = bounds.maxs.max(p);
            i += 1;
        }
        bounds
    }
    /// Upstream point-cloud sphere: arithmetic mean of the vertices, then the greatest
    /// distance to that center. Mean components truncate toward zero; radius floors.
    /// Panics if sums, differences, wide squared norms or radius exceed their numeric ranges.
    fn compute_local_bounding_sphere(self: ConvexPolygon) -> (Vec2, Fixed) {
        let mut center = Vec2Trait::ZERO;
        let mut i = 0;
        while i != self.count { center = center + self.vertex(i); i += 1; }
        let n: i64 = self.count.into();
        center = Vec2 { x: Fixed { raw: center.x.raw / n }, y: Fixed { raw: center.y.raw / n } };
        let mut farthest = Vec2Trait::ZERO;
        let mut max_sq = 0;
        i = 0;
        while i != self.count {
            let delta = self.vertex(i) - center;
            let sq = dot_wide(delta.x,delta.y,delta.x,delta.y);
            if sq > max_sq { max_sq = sq; farthest = delta; }
            i += 1;
        }
        (center, fixed::wide::norm2(farthest.x,farthest.y))
    }

    /// Uniform density mass properties; panics on unrepresentable intermediate values/inverses.
    fn mass_properties(self: ConvexPolygon, density: Fixed) -> MassProperties {
        MassPropertiesTrait::from_convex_polygon(density, self)
    }
}

#[cfg(test)]
mod tests;

/// Four support queries instead of transforming every vertex; benchmarked against the scan.
#[cfg(test)]
mod alternatives {
    use fixed::{ONE, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2Trait;
    use crate::aabb::Aabb;
    use super::{ConvexPolygon, ConvexPolygonTrait};

    pub fn compute_aabb_support(p: ConvexPolygon, pose: Pose2) -> Aabb {
        let x = Vec2 { x: ONE, y: ZERO };
        let y = Vec2 { x: ZERO, y: ONE };
        let dx = pose.rotation.inverse_rotate(x);
        let dy = pose.rotation.inverse_rotate(y);
        let xmin = pose.transform_point(p.local_support_point(-dx));
        let xmax = pose.transform_point(p.local_support_point(dx));
        let ymin = pose.transform_point(p.local_support_point(-dy));
        let ymax = pose.transform_point(p.local_support_point(dy));
        Aabb { mins: Vec2 { x: xmin.x, y: ymin.y }, maxs: Vec2 { x: xmax.x, y: ymax.y } }
    }
}
