//! Bounded, strictly convex CCW polygons. Construction rejects collinear vertices instead of
//! removing them as Parry does, and verifies every vertex against every edge (at most 64 tests).
//! Coordinates and differences must fit Q32.32; wide cross/dot sums must fit i128. Arithmetic
//! overflow panics. Padded slots are zero and do not participate in any query.

use fixed::{Fixed, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::math_ext::vec2::try_normalize2;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2Trait;
use crate::aabb::Aabb;
use crate::aabb::bounding_volume::{BoundingSphere, BoundingSphereTrait};
use crate::feature_id::{FeatureId, FeatureIdTrait};
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

/// Raw of `cos(1 degree)` (rounded): a face is the support feature when its normal is within one
/// degree of the direction (upstream `support_feature_id_toward`).
const COS_ONE_DEGREE_RAW: i128 = 4294313152;
/// `2^32`: lifts a raw `Fixed` to the Q64.64 scale of `dot_wide`.
const RAW_ONE: i128 = 0x100000000;

/// Polygon invariant and arithmetic failures.
pub mod errors {
    /// Vertex index outside the live range.
    pub const VERTEX_INDEX: felt252 = 'Polygon: vertex index';
    /// Normal index outside the live range.
    pub const NORMAL_INDEX: felt252 = 'Polygon: normal index';
    /// Triangle area cannot be represented as Q32.32.
    pub const AREA_OVERFLOW: felt252 = 'Polygon: area overflow';
    /// `offsetted` was given a negative amount.
    pub const NEGATIVE_OFFSET: felt252 = 'Polygon: negative offset';
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

    /// The convex hull of `points`, as a polygon (upstream `from_convex_hull`).
    ///
    /// Gift wrapping with exact wide predicates, so the result does not depend on rounding:
    /// the hull starts at the lowest `x` (lowest `y` on ties) and runs counter-clockwise, points
    /// on a hull edge and duplicates are dropped. Returns `None` when the hull is degenerate
    /// (fewer than 3 points, all equal or all collinear) or has more than 8 vertices. Unlike
    /// upstream (quickhull, unspecified start vertex) the cost is `O(hull * points)`, which is
    /// what a hull of at most 8 vertices needs. Panics on coordinate differences / wide sums
    /// overflow, as [`Self::from_convex_polyline`].
    fn from_convex_hull(points: Span<Vec2>) -> Option<ConvexPolygon> {
        let n = points.len();
        if n < 3 {
            return None;
        }
        let mut start = *points.at(0);
        let mut i = 1;
        while i != n {
            let p = *points.at(i);
            if p.x < start.x || (p.x == start.x && p.y < start.y) {
                start = p;
            }
            i += 1;
        }
        let mut hull = array![start];
        let mut current = start;
        loop {
            // The most clockwise point around `current` (the farthest one among collinear
            // points): every other point is on its left, so the hull turns counter-clockwise.
            let mut next = current;
            let mut j = 0;
            while j != n {
                let p = *points.at(j);
                if p != current {
                    if next == current {
                        next = p;
                    } else {
                        let e = next - current;
                        let d = p - current;
                        let turn = cross_wide(e.x, e.y, d.x, d.y);
                        if turn < 0
                            || (turn == 0
                                && dot_wide(d.x, d.y, d.x, d.y) > dot_wide(e.x, e.y, e.x, e.y)) {
                            next = p;
                        }
                    }
                }
                j += 1;
            }
            if next == current {
                // Every point equals `current`.
                return None;
            }
            if next == start {
                break;
            }
            if hull.len() == 8 {
                return None;
            }
            hull.append(next);
            current = next;
        }
        Self::from_convex_polyline(hull.span())
    }

    /// The polygon grown by `amount` along its edge normals (upstream `offsetted`): every vertex
    /// moves along the sum of its two adjacent normals so that each edge moves outward by exactly
    /// `amount`. The normals are unchanged; `amount / (1 + n_prev . n)` is one `Fixed` division
    /// per vertex.
    ///
    /// # Panics
    /// * `Polygon: negative offset` if `amount` is negative.
    /// * `Fixed: overflow` if a corner is so sharp that the vertex leaves the Q32.32 range.
    fn offsetted(self: ConvexPolygon, amount: Fixed) -> ConvexPolygon {
        assert(amount >= ZERO, errors::NEGATIVE_OFFSET);
        let mut moved = array![];
        let mut i = 0;
        while i != self.count {
            let normal = get(self.normals, i);
            let previous = get(self.normals, if i == 0 {
                self.count - 1
            } else {
                i - 1
            });
            let direction = previous + normal;
            let scale = amount / direction.dot(previous);
            let vertex = get(self.vertices, i);
            moved
                .append(
                    Vec2 { x: vertex.x + direction.x * scale, y: vertex.y + direction.y * scale },
                );
            i += 1;
        }
        let moved = moved.span();
        ConvexPolygon {
            vertices: [
                padded(moved, 0), padded(moved, 1), padded(moved, 2), padded(moved, 3),
                padded(moved, 4), padded(moved, 5), padded(moved, 6), padded(moved, 7),
            ],
            normals: self.normals,
            count: self.count,
        }
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
    /// Unrolled to keep the closed Shape AABB dispatch free of loop-dependent allocation-pointer
    /// changes. This preserves existing scene step costs; the cheaper isolated gas loop remains
    /// benchmarked under alternatives. Rotate first, then translate the extrema once. Panics
    /// if a rotated or translated point exceeds Q32.32.
    fn compute_aabb(self: ConvexPolygon, pose: Pose2) -> Aabb {
        let [a, b, c, d, e, f, g, h] = self.vertices;
        let p = pose.rotation.rotate(a);
        let mut bounds = Aabb { mins: p, maxs: p };
        let p = pose.rotation.rotate(b);
        bounds.mins = bounds.mins.min(p);
        bounds.maxs = bounds.maxs.max(p);
        let p = pose.rotation.rotate(c);
        bounds.mins = bounds.mins.min(p);
        bounds.maxs = bounds.maxs.max(p);
        if self.count > 3 {
            let p = pose.rotation.rotate(d);
            bounds.mins = bounds.mins.min(p);
            bounds.maxs = bounds.maxs.max(p);
        }
        if self.count > 4 {
            let p = pose.rotation.rotate(e);
            bounds.mins = bounds.mins.min(p);
            bounds.maxs = bounds.maxs.max(p);
        }
        if self.count > 5 {
            let p = pose.rotation.rotate(f);
            bounds.mins = bounds.mins.min(p);
            bounds.maxs = bounds.maxs.max(p);
        }
        if self.count > 6 {
            let p = pose.rotation.rotate(g);
            bounds.mins = bounds.mins.min(p);
            bounds.maxs = bounds.maxs.max(p);
        }
        if self.count > 7 {
            let p = pose.rotation.rotate(h);
            bounds.mins = bounds.mins.min(p);
            bounds.maxs = bounds.maxs.max(p);
        }
        Aabb { mins: bounds.mins + pose.translation, maxs: bounds.maxs + pose.translation }
    }
    /// Upstream point-cloud sphere: arithmetic mean of the vertices, then the greatest
    /// distance to that center. Mean components truncate toward zero; radius floors.
    /// Panics if sums, differences, wide squared norms or radius exceed their numeric ranges.
    fn compute_local_bounding_sphere(self: ConvexPolygon) -> (Vec2, Fixed) {
        let mut center = Vec2Trait::ZERO;
        let mut i = 0;
        while i != self.count {
            center = center + self.vertex(i);
            i += 1;
        }
        let n: i64 = self.count.into();
        center = Vec2 { x: Fixed { raw: center.x.raw / n }, y: Fixed { raw: center.y.raw / n } };
        let mut farthest = Vec2Trait::ZERO;
        let mut max_sq = 0;
        i = 0;
        while i != self.count {
            let delta = self.vertex(i) - center;
            let sq = dot_wide(delta.x, delta.y, delta.x, delta.y);
            if sq > max_sq {
                max_sq = sq;
                farthest = delta;
            }
            i += 1;
        }
        (center, fixed::wide::norm2(farthest.x, farthest.y))
    }

    /// Upstream name of [`ConvexPolygonTrait::compute_local_aabb`].
    #[inline(always)]
    fn local_aabb(self: ConvexPolygon) -> Aabb {
        Self::compute_local_aabb(self)
    }
    /// Upstream name of [`ConvexPolygonTrait::compute_aabb`] (the point-cloud box of the placed
    /// vertices).
    #[inline(always)]
    fn aabb(self: ConvexPolygon, pose: Pose2) -> Aabb {
        Self::compute_aabb(self, pose)
    }
    /// [`ConvexPolygonTrait::compute_local_bounding_sphere`] as a [`BoundingSphere`] (upstream
    /// `local_bounding_sphere`, the point-cloud sphere of the vertices).
    fn local_bounding_sphere(self: ConvexPolygon) -> BoundingSphere {
        let (center, radius) = Self::compute_local_bounding_sphere(self);
        BoundingSphere { center, radius }
    }
    /// [`ConvexPolygonTrait::local_bounding_sphere`] placed at `pose`.
    fn bounding_sphere(self: ConvexPolygon, pose: Pose2) -> BoundingSphere {
        Self::local_bounding_sphere(self).transform_by(pose)
    }
    /// Normal of a feature with upstream's `FeatureId` indices (not the PFM ids): `Face(i)` is
    /// normal `i`, `Vertex(i)` the normalised sum of the normals of the faces meeting at vertex
    /// `i` (rounded to nearest). `None` for the unknown feature and, unlike upstream (which
    /// panics), for an index `>= count`.
    fn feature_normal(self: ConvexPolygon, feature: FeatureId) -> Option<Vec2> {
        let code = feature.code();
        if code >= self.count.into() {
            return None;
        }
        let i: u8 = code.try_into().unwrap();
        if feature.is_face() {
            Some(get(self.normals, i))
        } else if feature.is_vertex() {
            let prev = if i == 0 {
                self.count - 1
            } else {
                i - 1
            };
            let sum = get(self.normals, prev) + get(self.normals, i);
            match try_normalize2(sum.x, sum.y) {
                Some((x, y)) => Some(Vec2 { x, y }),
                None => None,
            }
        } else {
            None
        }
    }
    /// The feature supporting the unit `local_dir` (upstream `support_feature_id_toward`): the
    /// first face whose normal is within one degree of it (exact wide test against the rounded
    /// `cos(1 degree)`), else the vertex of largest dot (first on ties), with upstream's plain
    /// indices.
    fn support_feature_id_toward(self: ConvexPolygon, local_dir: Vec2) -> FeatureId {
        let threshold = COS_ONE_DEGREE_RAW * RAW_ONE;
        let mut i = 0;
        while i != self.count {
            let n = get(self.normals, i);
            if dot_wide(n.x, n.y, local_dir.x, local_dir.y) >= threshold {
                return FeatureIdTrait::face(i.into());
            }
            i += 1;
        }
        let p = get(self.vertices, 0);
        let mut best: u8 = 0;
        let mut score = dot_wide(p.x, p.y, local_dir.x, local_dir.y);
        i = 1;
        while i != self.count {
            let p = get(self.vertices, i);
            let d = dot_wide(p.x, p.y, local_dir.x, local_dir.y);
            if d > score {
                best = i;
                score = d;
            }
            i += 1;
        }
        FeatureIdTrait::vertex(best.into())
    }

    /// Uniform density mass properties; panics on unrepresentable intermediate values/inverses.
    fn mass_properties(self: ConvexPolygon, density: Fixed) -> MassProperties {
        MassPropertiesTrait::from_convex_polygon(density, self)
    }
}

#[cfg(test)]
mod hull_tests;
#[cfg(test)]
mod tests;

/// Four support queries instead of transforming every vertex; benchmarked against the scan.
#[cfg(test)]
mod alternatives {
    use fixed::{ONE, ZERO};
    use glam::{Vec2, Vec2Trait};
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2Trait;
    use crate::aabb::Aabb;
    use super::{ConvexPolygon, ConvexPolygonTrait};

    /// Unrolled candidate translating every vertex before taking min/max.
    pub fn compute_aabb_shift_each(polygon: ConvexPolygon, pose: Pose2) -> Aabb {
        let [a, b, c, d, e, f, g, h] = polygon.vertices;
        let p = pose.transform_point(a);
        let mut bounds = Aabb { mins: p, maxs: p };
        let p = pose.transform_point(b);
        bounds.mins = bounds.mins.min(p);
        bounds.maxs = bounds.maxs.max(p);
        let p = pose.transform_point(c);
        bounds.mins = bounds.mins.min(p);
        bounds.maxs = bounds.maxs.max(p);
        if polygon.count > 3 {
            let p = pose.transform_point(d);
            bounds.mins = bounds.mins.min(p);
            bounds.maxs = bounds.maxs.max(p);
        }
        if polygon.count > 4 {
            let p = pose.transform_point(e);
            bounds.mins = bounds.mins.min(p);
            bounds.maxs = bounds.maxs.max(p);
        }
        if polygon.count > 5 {
            let p = pose.transform_point(f);
            bounds.mins = bounds.mins.min(p);
            bounds.maxs = bounds.maxs.max(p);
        }
        if polygon.count > 6 {
            let p = pose.transform_point(g);
            bounds.mins = bounds.mins.min(p);
            bounds.maxs = bounds.maxs.max(p);
        }
        if polygon.count > 7 {
            let p = pose.transform_point(h);
            bounds.mins = bounds.mins.min(p);
            bounds.maxs = bounds.maxs.max(p);
        }
        bounds
    }

    /// Bounded loop: lower isolated Sierra gas, but makes every Shape AABB call AP-unknown.
    pub fn compute_aabb_loop(polygon: ConvexPolygon, pose: Pose2) -> Aabb {
        let p = pose.transform_point(polygon.vertex(0));
        let mut bounds = Aabb { mins: p, maxs: p };
        let mut i = 1;
        while i != polygon.count {
            let p = pose.transform_point(polygon.vertex(i));
            bounds.mins = bounds.mins.min(p);
            bounds.maxs = bounds.maxs.max(p);
            i += 1;
        }
        bounds
    }

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
