//! `Triangle` (Parry `shape/triangle.rs`, 2D branch; `bounding_volume/aabb_triangle.rs`,
//! `bounding_volume/bounding_sphere_triangle.rs`; `mass_properties_triangle.rs`).
//!
//! The triangle is stored as upstream: three free vertices, any orientation, possibly
//! degenerate. Every discrete decision (orientation, containment, support vertex, Voronoi
//! region) is read off exact wide products (`cross_wide`, `dot_wide`), never off a rounded value.
//!
//! The contact generators and the shape-pair queries use its *core*: the same three vertices as a
//! counter-clockwise [`ConvexPolygon`] (`b` and `c` swapped for a clockwise triangle), built
//! without the constructor's validation, like `cuboid_core`. Features of the core are mapped back
//! to upstream's triangle ids (`Vertex(i)`, `Face(i)` for the edge `i -> i + 1`, see
//! [`TriangleTrait::support_face`]), so manifolds carry upstream's feature ids whatever the
//! orientation.

use fixed::wide::{WideAdd, WideNarrow, distance2, dot2, norm2, wide_mul};
use fixed::{Fixed, FixedTrait, MAX, ONE, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::math_ext::norm2::norm2_sq_wide;
use rapier_math::math_ext::vec2::try_normalize2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::aabb::Aabb;
use crate::aabb::bounding_volume::{BoundingSphere, BoundingSphereTrait};
use crate::feature_id::{FeatureId, FeatureIdTrait};
use crate::mass::{MassProperties, MassPropertiesTrait};
use crate::point::{cross_wide, dot_wide};
use crate::polygonal_feature::PolygonalFeature;
use crate::shape::convex_polygon::ConvexPolygon;
use crate::shape::segment::Segment;

/// A triangle with vertices `a`, `b`, `c` (upstream `Triangle`).
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct Triangle {
    pub a: Vec2,
    pub b: Vec2,
    pub c: Vec2,
}

/// The orientation of a triangle (upstream `TriangleOrientation`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum TriangleOrientation {
    Clockwise,
    CounterClockwise,
    Degenerate,
}

/// Where a point projects on a triangle (upstream `TrianglePointLocation`).
///
/// `OnVertex(i)`: vertex `i` (`0 = a`, `1 = b`, `2 = c`). `OnEdge(i, (u, v))`: edge `0 = ab`,
/// `1 = bc`, `2 = ac` (upstream's numbering: `ac`, not `ca`), the point being `u * first + v *
/// second` of that pair. `OnFace(i, (u, v, w))`: 3D only upstream, never produced in 2D.
/// `OnSolid`: a solid projection of an interior point.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum TrianglePointLocation {
    OnVertex: u32,
    OnEdge: (u32, (Fixed, Fixed)),
    OnFace: (u32, (Fixed, Fixed, Fixed)),
    OnSolid,
}

#[generate_trait]
pub impl TrianglePointLocationImpl of TrianglePointLocationTrait {
    /// The barycentric coordinates `(ca, cb, cc)` of the location, `None` for `OnSolid`
    /// (upstream `barycentric_coordinates`, a `[Real; 3]` there).
    fn barycentric_coordinates(self: TrianglePointLocation) -> Option<(Fixed, Fixed, Fixed)> {
        match self {
            TrianglePointLocation::OnVertex(i) => if i == 0 {
                Some((ONE, ZERO, ZERO))
            } else if i == 1 {
                Some((ZERO, ONE, ZERO))
            } else {
                Some((ZERO, ZERO, ONE))
            },
            TrianglePointLocation::OnEdge((i, (u, v))) => if i == 0 {
                Some((u, v, ZERO))
            } else if i == 1 {
                Some((ZERO, u, v))
            } else {
                Some((u, ZERO, v))
            },
            TrianglePointLocation::OnFace((_, uvw)) => Some(uvw),
            TrianglePointLocation::OnSolid => None,
        }
    }

    /// Whether the location is `OnFace` (upstream `is_on_face`).
    fn is_on_face(self: TrianglePointLocation) -> bool {
        match self {
            TrianglePointLocation::OnFace(_) => true,
            _ => false,
        }
    }
}

/// Upstream `impl From<[Vector; 3]> for Triangle`.
pub impl ArrayIntoTriangle of Into<[Vec2; 3], Triangle> {
    #[inline(always)]
    fn into(self: [Vec2; 3]) -> Triangle {
        let [a, b, c] = self;
        Triangle { a, b, c }
    }
}

/// Box serialization is the triangle value.
pub impl BoxedTriangleSerde of Serde<Box<Triangle>> {
    fn serialize(self: @Box<Triangle>, ref output: Array<felt252>) {
        (*self).unbox().serialize(ref output);
    }
    fn deserialize(ref serialized: Span<felt252>) -> Option<Box<Triangle>> {
        Some(BoxTrait::new(Serde::<Triangle>::deserialize(ref serialized)?))
    }
}

/// Structural equality of boxed triangles.
pub impl BoxedTrianglePartialEq of PartialEq<Box<Triangle>> {
    fn eq(lhs: @Box<Triangle>, rhs: @Box<Triangle>) -> bool {
        (*lhs).unbox() == (*rhs).unbox()
    }
    fn ne(lhs: @Box<Triangle>, rhs: @Box<Triangle>) -> bool {
        !Self::eq(lhs, rhs)
    }
}

/// `2^32`: lifts a raw `Fixed` to the Q64.64 scale of `cross_wide`.
const RAW_ONE: i128 = 0x100000000;

/// `-1`, `0` or `1`.
#[inline(always)]
fn sign(x: i128) -> i8 {
    if x > 0 {
        1
    } else if x < 0 {
        -1
    } else {
        0
    }
}

/// Twice the signed area, exact (raw Q64.64): positive for a counter-clockwise triangle.
#[inline(always)]
pub(crate) fn signed_area2_wide(a: Vec2, b: Vec2, c: Vec2) -> i128 {
    let ab = b - a;
    let ac = c - a;
    cross_wide(ab.x, ab.y, ac.x, ac.y)
}

#[inline(always)]
fn orientation_of(area2: i128, epsilon: Fixed) -> TriangleOrientation {
    let eps: i128 = epsilon.raw.into() * RAW_ONE;
    if area2 > eps {
        TriangleOrientation::CounterClockwise
    } else if area2 < -eps {
        TriangleOrientation::Clockwise
    } else {
        TriangleOrientation::Degenerate
    }
}

/// `v / 3` on the raw components, rounding toward zero.
#[inline(always)]
fn third(v: Vec2) -> Vec2 {
    Vec2 { x: Fixed { raw: v.x.raw / 3 }, y: Fixed { raw: v.y.raw / 3 } }
}

/// The outward unit normal of the core edge `p -> q` (zero for a zero-length edge).
#[inline(always)]
fn edge_normal(p: Vec2, q: Vec2) -> Vec2 {
    let e = q - p;
    match try_normalize2(e.y, -e.x) {
        Some((x, y)) => Vec2 { x, y },
        None => Vec2Trait::ZERO,
    }
}

/// The outward unit normal of edge `i` of the closed vertex list `pts` (`ccw` orientation).
#[inline(always)]
fn outward_normal(pts: Span<Vec2>, i: u32, ccw: bool) -> Vec2 {
    let n = edge_normal(*pts[i], *pts[i + 1]);
    if ccw {
        n
    } else {
        -n
    }
}

/// The counter-clockwise core polygon of `t` and whether `b` and `c` were swapped (clockwise
/// triangle). Unvalidated: a degenerate triangle keeps its collinear vertices and zero normals
/// for its zero-length edges. Normalisation rounds to nearest.
pub(crate) fn triangle_core(t: Triangle) -> (ConvexPolygon, bool) {
    let reversed = signed_area2_wide(t.a, t.b, t.c) < 0;
    let (p0, p1, p2) = if reversed {
        (t.a, t.c, t.b)
    } else {
        (t.a, t.b, t.c)
    };
    let o = Vec2Trait::ZERO;
    (
        ConvexPolygon {
            vertices: [p0, p1, p2, o, o, o, o, o],
            normals: [edge_normal(p0, p1), edge_normal(p1, p2), edge_normal(p2, p0), o, o, o, o, o],
            count: 3,
        },
        reversed,
    )
}

/// The triangle vertex id of core vertex `k` (`b` and `c` exchange places when `reversed`).
#[inline(always)]
pub(crate) fn core_vertex_id(k: u32, reversed: bool) -> u32 {
    if reversed && k != 0 {
        3 - k
    } else {
        k
    }
}

/// The triangle edge id (`i -> i + 1`) of core edge `k` when `reversed` (the core runs
/// `a, c, b`: its edges are `ca`, `cb`, `ba`, i.e. triangle edges 2, 1, 0).
#[inline(always)]
pub(crate) fn core_edge_id(k: u32, reversed: bool) -> u32 {
    if reversed {
        2 - k
    } else {
        k
    }
}

/// A support feature of the core polygon (packed ids `2 k`, `2 k + 1`) with the triangle's ids.
pub(crate) fn feature_to_triangle(f: PolygonalFeature, reversed: bool) -> PolygonalFeature {
    let [v0, v1] = f.vids;
    PolygonalFeature {
        vertices: f.vertices,
        vids: [
            FeatureIdTrait::vertex(core_vertex_id(v0.code() / 2, reversed)),
            FeatureIdTrait::vertex(core_vertex_id(v1.code() / 2, reversed)),
        ],
        fid: FeatureIdTrait::face(core_edge_id(f.fid.code() / 2, reversed)),
        num_vertices: f.num_vertices,
    }
}

#[generate_trait]
pub impl TriangleImpl of TriangleTrait {
    /// The triangle `a`, `b`, `c` (upstream `new`).
    #[inline(always)]
    fn new(a: Vec2, b: Vec2, c: Vec2) -> Triangle {
        Triangle { a, b, c }
    }

    /// The triangle of the three points (upstream `from_array`, a reinterpretation there).
    #[inline(always)]
    fn from_array(arr: [Vec2; 3]) -> Triangle {
        arr.into()
    }

    /// `[a, b, c]` (upstream `vertices`).
    #[inline(always)]
    fn vertices(self: Triangle) -> [Vec2; 3] {
        [self.a, self.b, self.c]
    }

    /// `[ab, bc, ca]` (upstream `edges`).
    fn edges(self: Triangle) -> [Segment; 3] {
        [
            Segment { a: self.a, b: self.b }, Segment { a: self.b, b: self.c },
            Segment { a: self.c, b: self.a },
        ]
    }

    /// `[b - a, c - b, a - c]` (upstream `edges_scaled_directions`).
    fn edges_scaled_directions(self: Triangle) -> [Vec2; 3] {
        [self.b - self.a, self.c - self.b, self.a - self.c]
    }

    /// The vertices multiplied component-wise by `scale` (floored products).
    fn scaled(self: Triangle, scale: Vec2) -> Triangle {
        Triangle { a: self.a * scale, b: self.b * scale, c: self.c * scale }
    }

    /// The vertices moved by `pose` (upstream `transformed`).
    fn transformed(self: Triangle, pose: Pose2) -> Triangle {
        Triangle {
            a: pose.transform_point(self.a),
            b: pose.transform_point(self.b),
            c: pose.transform_point(self.c),
        }
    }

    /// Exchanges `b` and `c`, flipping the orientation (upstream `reverse`).
    fn reverse(ref self: Triangle) {
        let b = self.b;
        self.b = self.c;
        self.c = b;
    }

    /// The edge opposite to the vertex with the smallest `dir . v` (first of `a`, `b`, `c` on
    /// ties, exact comparisons; upstream `local_support_edge_segment`).
    fn local_support_edge_segment(self: Triangle, dir: Vec2) -> Segment {
        let da = dot_wide(dir.x, dir.y, self.a.x, self.a.y);
        let db = dot_wide(dir.x, dir.y, self.b.x, self.b.y);
        let dc = dot_wide(dir.x, dir.y, self.c.x, self.c.y);
        if da <= db && da <= dc {
            Segment { a: self.b, b: self.c }
        } else if db <= dc {
            Segment { a: self.c, b: self.a }
        } else {
            Segment { a: self.a, b: self.b }
        }
    }

    /// The edge whose unit normal `(t.y, -t.x) / |t|` (`t` the edge direction) is most aligned
    /// with `dir`, as a feature with ids `Vertex(i)`, `Vertex(i + 1)`, `Face(i)` (upstream 2D
    /// `support_face`; first edge on ties, zero-length edges skipped). The normal is outward
    /// for a counter-clockwise triangle only, as upstream.
    fn support_face(self: Triangle, dir: Vec2) -> PolygonalFeature {
        let pts = [self.a, self.b, self.c];
        let [t0, t1, t2] = Self::edges_scaled_directions(self);
        let mut best: u32 = 0;
        let mut best_dot = -MAX;
        let mut i: u32 = 0;
        for t in [t0, t1, t2].span() {
            if let Some((x, y)) = try_normalize2((*t).y, -(*t).x) {
                let d = dot2(x, dir.x, y, dir.y);
                if d > best_dot {
                    best = i;
                    best_dot = d;
                }
            }
            i += 1;
        }
        let next = if best == 2 {
            0
        } else {
            best + 1
        };
        PolygonalFeature {
            vertices: [*pts.span()[best], *pts.span()[next]],
            vids: [FeatureIdTrait::vertex(best), FeatureIdTrait::vertex(next)],
            fid: FeatureIdTrait::face(best),
            num_vertices: 2,
        }
    }

    /// `(min, max)` of `v . dir` over the vertices (floored dot products; upstream
    /// `extents_on_dir`, same comparison order).
    fn extents_on_dir(self: Triangle, dir: Vec2) -> (Fixed, Fixed) {
        let a = self.a.dot(dir);
        let b = self.b.dot(dir);
        let c = self.c.dot(dir);
        if a > b {
            if b > c {
                (c, a)
            } else if a > c {
                (b, a)
            } else {
                (b, c)
            }
        } else if a > c {
            (c, b)
        } else if b > c {
            (a, b)
        } else {
            (a, c)
        }
    }

    /// `|cross(b - a, c - a)| / 2`, exact then floored once (upstream `area`).
    fn area(self: Triangle) -> Fixed {
        let area2 = signed_area2_wide(self.a, self.b, self.c);
        let abs = if area2 < 0 {
            -area2
        } else {
            area2
        };
        Fixed { raw: (abs / 0x200000000).try_into().unwrap() }
    }

    /// The angular inertia of the unit-density triangle about `a` (upstream
    /// `unit_angular_inertia`, from Box2D): the six products summed exactly, floored once, then
    /// divided by 6 toward zero.
    fn unit_angular_inertia(self: Triangle) -> Fixed {
        let e1 = self.b - self.a;
        let e2 = self.c - self.a;
        let sum = wide_mul(e1.x, e1.x)
            .add(wide_mul(e2.x, e1.x))
            .add(wide_mul(e2.x, e2.x))
            .add(wide_mul(e1.y, e1.y))
            .add(wide_mul(e2.y, e1.y))
            .add(wide_mul(e2.y, e2.y))
            .narrow();
        Fixed { raw: sum.raw / 6 }
    }

    /// `(a + b + c) / 3`, the sum exact and each component divided toward zero (upstream
    /// `center`, which scales each vertex by the rounded `1 / 3` first).
    fn center(self: Triangle) -> Vec2 {
        third(self.a + self.b + self.c)
    }

    /// The sum of the three floored edge lengths (upstream `perimeter`).
    fn perimeter(self: Triangle) -> Fixed {
        distance2(self.b.x, self.b.y, self.a.x, self.a.y)
            + distance2(self.c.x, self.c.y, self.b.x, self.b.y)
            + distance2(self.a.x, self.a.y, self.c.x, self.c.y)
    }

    /// The circumscribed circle `(center, radius)` (upstream `circumcircle`). A degenerate
    /// (collinear, exactly) triangle answers the midpoint of its longest edge and half its length
    /// (upstream's tie order). Otherwise the centre is `c + (ux, uy)` with `ux = (|a'|^2 b'.y -
    /// |b'|^2 a'.y) / D`, `uy = (|b'|^2 a'.x - |a'|^2 b'.x) / D`, `D = 2 cross(a', b')` (`a' = a -
    /// c`, `b' = b - c`): the squared lengths floor, the numerators are exact, the quotients
    /// round to nearest; the radius is the floored distance to `a`.
    /// #### Panics
    /// * `'Option::unwrap failed.'` when the centre leaves the scalar range (a nearly degenerate
    ///   triangle).
    fn circumcircle(self: Triangle) -> (Vec2, Fixed) {
        let a = self.a - self.c;
        let b = self.b - self.c;
        let cross = cross_wide(a.x, a.y, b.x, b.y);
        if cross == 0 {
            let c = self.a - self.b;
            let na = norm2_sq_wide(a.x, a.y);
            let nb = norm2_sq_wide(b.x, b.y);
            let nc = norm2_sq_wide(c.x, c.y);
            return if nc >= na && nc >= nb {
                (self.a.midpoint(self.b), norm2(c.x, c.y) / FixedTrait::from_int(2))
            } else if na >= nb && na >= nc {
                (self.a.midpoint(self.c), norm2(a.x, a.y) / FixedTrait::from_int(2))
            } else {
                (self.b.midpoint(self.c), norm2(b.x, b.y) / FixedTrait::from_int(2))
            };
        }
        let na = a.length_squared();
        let nb = b.length_squared();
        // `na b.y - nb a.y` and `nb a.x - na b.x`, exact.
        let num_x = cross_wide(na, nb, a.y, b.y);
        let num_y = cross_wide(nb, na, b.x, a.x);
        let den = 2 * cross;
        let ux = crate::ray::quotient::div_wide(num_x, den).unwrap();
        let uy = crate::ray::quotient::div_wide(num_y, den).unwrap();
        let center = self.c + Vec2 { x: ux, y: uy };
        (center, distance2(center.x, center.y, self.a.x, self.a.y))
    }

    /// Whether `p` is inside the triangle, boundary included, whatever its orientation: no two
    /// of the exact edge side tests have strictly opposite signs (upstream 2D `contains_point`).
    fn contains_point(self: Triangle, p: Vec2) -> bool {
        let ab = self.b - self.a;
        let bc = self.c - self.b;
        let ca = self.a - self.c;
        let pa = p - self.a;
        let pb = p - self.b;
        let pc = p - self.c;
        let s1 = sign(cross_wide(ab.x, ab.y, pa.x, pa.y));
        let s2 = sign(cross_wide(bc.x, bc.y, pb.x, pb.y));
        let s3 = sign(cross_wide(ca.x, ca.y, pc.x, pc.y));
        s1 * s2 >= 0 && s1 * s3 >= 0 && s2 * s3 >= 0
    }

    /// The orientation from the exact doubled signed area compared with `epsilon` (upstream 2D
    /// `orientation`: `area2 > epsilon` counter-clockwise, `< -epsilon` clockwise).
    fn orientation(self: Triangle, epsilon: Fixed) -> TriangleOrientation {
        orientation_of(signed_area2_wide(self.a, self.b, self.c), epsilon)
    }

    /// The orientation of `a`, `b`, `c` (upstream `orientation2d`).
    fn orientation2d(a: Vec2, b: Vec2, c: Vec2, epsilon: Fixed) -> TriangleOrientation {
        orientation_of(signed_area2_wide(a, b, c), epsilon)
    }

    /// The vertex `i + 1` whose angle between `p_i - p_(i+1)` and `p_(i+2) - p_(i+1)` has the
    /// smallest `|cos|`, returned as `i` (upstream `angle_closest_to_90`: first on ties, the
    /// unit directions rounded to nearest; a zero-length edge never wins).
    fn angle_closest_to_90(self: Triangle) -> u32 {
        let pts = [self.a, self.b, self.c, self.a, self.b].span();
        let mut best_cos = FixedTrait::from_int(2);
        let mut selected = 0;
        let mut i = 0;
        while i != 3 {
            let p1 = *pts[i + 1];
            let d1 = *pts[i] - p1;
            let d2 = *pts[i + 2] - p1;
            if let Some((x1, y1)) = try_normalize2(d1.x, d1.y) {
                if let Some((x2, y2)) = try_normalize2(d2.x, d2.y) {
                    let cos_abs = dot2(x1, x2, y1, y2).abs();
                    if cos_abs < best_cos {
                        best_cos = cos_abs;
                        selected = i;
                    }
                }
            }
            i += 1;
        }
        selected
    }

    /// The vertex with the greatest `v . dir` (exact; upstream's `>` comparison order: `c`
    /// wins ties against `a` and `b`, `b` wins against `a`). `dir` need not be unit.
    fn local_support_point(self: Triangle, dir: Vec2) -> Vec2 {
        let d1 = dot_wide(self.a.x, self.a.y, dir.x, dir.y);
        let d2 = dot_wide(self.b.x, self.b.y, dir.x, dir.y);
        let d3 = dot_wide(self.c.x, self.c.y, dir.x, dir.y);
        if d1 > d2 {
            if d1 > d3 {
                self.a
            } else {
                self.c
            }
        } else if d2 > d3 {
            self.b
        } else {
            self.c
        }
    }

    /// Exact bounds of the three vertices (upstream `local_aabb`).
    fn compute_local_aabb(self: Triangle) -> Aabb {
        Aabb { mins: self.a.min(self.b).min(self.c), maxs: self.a.max(self.b).max(self.c) }
    }

    /// The bounds of the transformed vertices (upstream `aabb`: `transformed(pose).local_aabb()`).
    fn compute_aabb(self: Triangle, pose: Pose2) -> Aabb {
        Self::compute_local_aabb(Self::transformed(self, pose))
    }

    /// Upstream name of [`TriangleTrait::compute_local_aabb`].
    #[inline(always)]
    fn local_aabb(self: Triangle) -> Aabb {
        Self::compute_local_aabb(self)
    }

    /// Upstream name of [`TriangleTrait::compute_aabb`].
    #[inline(always)]
    fn aabb(self: Triangle, pose: Pose2) -> Aabb {
        Self::compute_aabb(self, pose)
    }

    /// Point-cloud sphere (upstream): centre [`TriangleTrait::center`], radius the largest
    /// floored distance to a vertex (the farthest chosen on exact squared distances).
    fn local_bounding_sphere(self: Triangle) -> BoundingSphere {
        let center = Self::center(self);
        let da = self.a - center;
        let db = self.b - center;
        let dc = self.c - center;
        let (sa, sb, sc) = (
            norm2_sq_wide(da.x, da.y), norm2_sq_wide(db.x, db.y), norm2_sq_wide(dc.x, dc.y),
        );
        let far = if sb > sa && sb >= sc {
            db
        } else if sc > sa && sc > sb {
            dc
        } else {
            da
        };
        BoundingSphere { center, radius: norm2(far.x, far.y) }
    }

    /// [`TriangleTrait::local_bounding_sphere`] placed at `pose`.
    fn bounding_sphere(self: Triangle, pose: Pose2) -> BoundingSphere {
        Self::local_bounding_sphere(self).transform_by(pose)
    }

    /// Uniform-density mass properties (upstream `MassProperties::from_triangle`).
    fn mass_properties(self: Triangle, density: Fixed) -> MassProperties {
        MassPropertiesTrait::from_triangle(density, self.a, self.b, self.c)
    }

    /// The core polygon's feature normal mapped from a triangle feature: the outward normal of
    /// edge `i` for `Face(i)`, the normalised sum of the two edge normals around vertex `i` for
    /// `Vertex(i)`; `None` for an unknown feature or a degenerate triangle.
    fn feature_normal(self: Triangle, feature: FeatureId) -> Option<Vec2> {
        let area2 = signed_area2_wide(self.a, self.b, self.c);
        if area2 == 0 || feature.is_unknown() {
            return None;
        }
        let ccw = area2 > 0;
        let pts = [self.a, self.b, self.c, self.a].span();
        let code = feature.code();
        if feature.is_face() {
            Some(outward_normal(pts, code, ccw))
        } else {
            let prev = if code == 0 {
                2
            } else {
                code - 1
            };
            let s = outward_normal(pts, prev, ccw) + outward_normal(pts, code, ccw);
            match try_normalize2(s.x, s.y) {
                Some((x, y)) => Some(Vec2 { x, y }),
                None => None,
            }
        }
    }
}

#[cfg(test)]
mod tests;
