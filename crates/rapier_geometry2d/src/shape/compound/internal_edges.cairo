//! The one-off pass of `CompoundFlags::FIX_INTERNAL_EDGES` (Parry 0.31 `shape/compound.rs`,
//! `part_outlines`, `weld_corners` and the 2D `compute_pseudo_normals`; lot CE): the normal cones
//! every part may report contacts in, built from the outline of the union rather than from each
//! part's own faces.
//!
//! An edge two parts share is a cut, so it is dropped, and each surviving edge opens towards the
//! edge next to it along the union's outline, which a cut may have left in another part. That
//! closes the corner where a surface runs into a cut, the ledge a body would otherwise catch on,
//! without closing one the cut merely passes through.
//!
//! # Deviations
//!
//! * Welding is absolute: two corners weld when `max(|dx|, |dy|) <= weld_tolerance` (a `Fixed`,
//!   `DEFAULT_WELD_TOLERANCE` = 4 raw). Upstream's tolerance is relative, `tolerance * f64::EPSILON
//!   *
//!   max(|c1|_inf, |c2|_inf)` with `tolerance` = 4 ULPs; Q32.32 has an absolute resolution, and its
//!   rounding error is absolute too (ADR 0001, CE's weld entry).
//! * The weld scans every pair of corners (upstream: a BVH of the corners, same pairs), and the
//!   shared-edge and corner maps are `Felt252Dict`s keyed by vertex ids (upstream `HashMap`s).
//! * Signed areas and turns are exact wide sums and cross products; the face normals and the
//!   halfway limits are `try_normalize` / `normalize_or` of `Fixed` sums.

use core::dict::{Felt252Dict, Felt252DictTrait};
use fixed::{Fixed, FixedTrait};
use glam_core::{Vec2, Vec2Trait};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::point::wide2::cross_wide;
use crate::shape::{ConvexPolygonTrait, Shape};
use super::pseudo_normals::{CompoundEdgeCone, CompoundPseudoNormals, turns_clockwise};

/// A corner claimed by more than one boundary edge (upstream's `None` slot).
const MANY: u32 = 0xffff_ffff;
/// `2^32`: the edge key `min * 2^32 + max`.
const KEY_SHIFT: felt252 = 0x1_0000_0000;

/// The corners of a polygonal part in its own frame (upstream: `ConvexPolygon::points`,
/// `Cuboid::to_polyline`, the triangle's `a, b, c`); `None` for a part with no straight sides.
fn part_corners(shape: Shape) -> Option<Array<Vec2>> {
    match shape {
        Shape::Cuboid(c) => {
            let he = c.half_extents;
            Some(
                array![
                    Vec2 { x: -he.x, y: -he.y }, Vec2 { x: he.x, y: -he.y }, he,
                    Vec2 { x: -he.x, y: he.y },
                ],
            )
        },
        Shape::ConvexPolygon(p) => {
            let p = p.unbox();
            let vertices = p.vertices().span();
            let mut out = array![];
            let mut i: u32 = 0;
            let count: u32 = p.count.into();
            while i != count {
                out.append(*vertices.at(i));
                i += 1;
            }
            Some(out)
        },
        Shape::Triangle(t) => {
            let t = t.unbox();
            Some(array![t.a, t.b, t.c])
        },
        _ => None,
    }
}

/// `2 * signed area` of a closed outline, exact.
fn signed_area(outline: Span<Vec2>) -> i128 {
    let n = outline.len();
    let mut sum: i128 = 0;
    let mut i: u32 = 0;
    while i != n {
        let a = *outline.at(i);
        let b = *outline.at(if i + 1 == n {
            0
        } else {
            i + 1
        });
        sum += cross_wide(a.x, a.y, b.x, b.y);
        i += 1;
    }
    sum
}

/// The parts' outlines in the compound's frame, wound counter-clockwise (upstream
/// `part_outlines`); `None` for a part with no straight sides.
fn part_outlines(shapes: Span<(Pose2, Shape)>) -> Array<Option<Span<Vec2>>> {
    let mut out = array![];
    for part in shapes {
        let (pose, shape) = *part;
        match part_corners(shape) {
            Some(corners) => {
                let mut outline = array![];
                for corner in corners {
                    outline.append(pose.transform_point(corner));
                }
                let outline = outline.span();
                if signed_area(outline) < 0 {
                    let mut reversed = array![];
                    let mut i = outline.len();
                    while i != 0 {
                        i -= 1;
                        reversed.append(*outline.at(i));
                    }
                    out.append(Some(reversed.span()));
                } else {
                    out.append(Some(outline));
                }
            },
            None => out.append(None),
        }
    }
    out
}

/// The root of `i` in the union-find `parent` (upstream's `find`, with path halving).
fn find(ref parent: Felt252Dict<u32>, i: u32) -> u32 {
    let mut i = i;
    loop {
        let p = parent.get(i.into());
        if p == i {
            break i;
        }
        let grand = parent.get(p.into());
        parent.insert(i.into(), grand);
        i = grand;
    }
}

/// One vertex id per corner (upstream `weld_corners`): corners within `tolerance` of each other
/// (Chebyshev distance, absolute, see the module documentation) share an id, chains included; the
/// ids are dense, numbered by the first corner of each group.
pub fn weld_corners(corners: Span<Vec2>, tolerance: Fixed) -> Span<u32> {
    let n = corners.len();
    let mut parent: Felt252Dict<u32> = Default::default();
    let mut i: u32 = 0;
    while i != n {
        parent.insert(i.into(), i);
        i += 1;
    }
    let mut i: u32 = 0;
    while i != n {
        let a = *corners.at(i);
        let mut j = i + 1;
        while j != n {
            let b = *corners.at(j);
            if (a.x - b.x).abs() <= tolerance && (a.y - b.y).abs() <= tolerance {
                let (ra, rb) = (find(ref parent, i), find(ref parent, j));
                parent.insert(ra.into(), rb);
            }
            j += 1;
        }
        i += 1;
    }
    // Number the roots so the ids are dense (`MANY` marks a root not numbered yet).
    let mut ids: Felt252Dict<u32> = Default::default();
    let mut next: u32 = 0;
    let mut out = array![];
    let mut i: u32 = 0;
    while i != n {
        let root = find(ref parent, i);
        let id = ids.get(root.into());
        if id == 0 {
            next += 1;
            ids.insert(root.into(), next);
            out.append(next - 1);
        } else {
            out.append(id - 1);
        }
        i += 1;
    }
    out.span()
}

/// The undirected key of the edge `(a, b)`.
#[inline(always)]
fn edge_key(a: u32, b: u32) -> felt252 {
    if a <= b {
        a.into() * KEY_SHIFT + b.into()
    } else {
        b.into() * KEY_SHIFT + a.into()
    }
}

/// The index after `i` around a loop of `n`.
#[inline(always)]
fn next_index(i: u32, n: u32) -> u32 {
    if i + 1 == n {
        0
    } else {
        i + 1
    }
}

/// The outward normal of the counter-clockwise edge `a -> b` (upstream `ccw_face_normal`), `None`
/// for a degenerate edge.
fn ccw_face_normal(a: Vec2, b: Vec2) -> Option<Vec2> {
    let ab = b - a;
    Vec2 { x: ab.y, y: -ab.x }.try_normalize()
}

/// Records `normal` (index `k` of the boundary normals) at `vertex` in `slots`: the first claim
/// stores `k + 1`, a second one [`MANY`].
fn claim(ref slots: Felt252Dict<u32>, vertex: u32, k: u32) {
    let slot = slots.get(vertex.into());
    slots.insert(vertex.into(), if slot == 0 {
        k + 1
    } else {
        MANY
    });
}

/// The one boundary normal claimed at `vertex`, `None` when none or several are.
fn claimed(ref slots: Felt252Dict<u32>, normals: Span<Vec2>, vertex: u32) -> Option<Vec2> {
    let slot = slots.get(vertex.into());
    if slot == 0 || slot == MANY {
        None
    } else {
        Some(*normals.at(slot - 1))
    }
}

/// Halfway from `face` to `neighbour`, or `face` (upstream `halfway_to`).
fn halfway_to(face: Vec2, neighbour: Option<Vec2>) -> Vec2 {
    match neighbour {
        Some(n) => (face + n).normalize_or(face),
        None => face,
    }
}

/// Upstream 2D `compute_pseudo_normals`: one entry per part, `None` for a part with no straight
/// sides, else the cones of its boundary edges in the part's frame (see the module documentation).
/// #### Panics
/// * `'i64_sub Overflow'` / `'i64_sub Underflow'` when two corners differ by `2^31` or more.
pub fn compute_pseudo_normals(
    shapes: Span<(Pose2, Shape)>, weld_tolerance: Fixed,
) -> Span<Option<CompoundPseudoNormals>> {
    let outlines = part_outlines(shapes).span();
    let mut corners = array![];
    for outline in outlines {
        if let Some(o) = outline {
            for c in *o {
                corners.append(*c);
            }
        }
    }
    let welded = weld_corners(corners.span(), weld_tolerance);
    let mut vertex_ids: Array<Option<Span<u32>>> = array![];
    let mut next: u32 = 0;
    for outline in outlines {
        match outline {
            Some(o) => {
                vertex_ids.append(Some(welded.slice(next, o.len())));
                next += o.len();
            },
            None => vertex_ids.append(None),
        }
    }
    let vertex_ids = vertex_ids.span();

    let mut sharing: Felt252Dict<u32> = Default::default();
    for ids in vertex_ids {
        if let Some(ids) = ids {
            let n = ids.len();
            let mut i: u32 = 0;
            while i != n {
                let key = edge_key(*ids.at(i), *ids.at(next_index(i, n)));
                sharing.insert(key, sharing.get(key) + 1);
                i += 1;
            }
        }
    }

    // Per part, the outward normal of every outline edge, `None` where a sibling part covers it;
    // all the boundary normals in one array, indexed by the corner maps.
    let mut boundary: Array<Option<Span<Option<Vec2>>>> = array![];
    let mut normals: Array<Vec2> = array![];
    let mut arriving: Felt252Dict<u32> = Default::default();
    let mut leaving: Felt252Dict<u32> = Default::default();
    let mut p: u32 = 0;
    for outline in outlines {
        match (*outline, *vertex_ids.at(p)) {
            (
                Some(o), Some(ids),
            ) => {
                let n = o.len();
                let mut part = array![];
                let mut i: u32 = 0;
                while i != n {
                    let j = next_index(i, n);
                    let (a, b) = (*ids.at(i), *ids.at(j));
                    let face = if sharing.get(edge_key(a, b)) > 1 {
                        None
                    } else {
                        ccw_face_normal(*o.at(i), *o.at(j))
                    };
                    if let Some(f) = face {
                        let k = normals.len();
                        normals.append(f);
                        claim(ref arriving, b, k);
                        claim(ref leaving, a, k);
                    }
                    part.append(face);
                    i += 1;
                }
                boundary.append(Some(part.span()));
            },
            _ => boundary.append(None),
        }
        p += 1;
    }
    let normals = normals.span();

    let mut out = array![];
    let mut p: u32 = 0;
    for part in shapes {
        let (pose, _) = *part;
        match (*vertex_ids.at(p), *boundary.at(p)) {
            (
                Some(ids), Some(faces),
            ) => {
                let n = ids.len();
                let mut edges = array![];
                let mut i: u32 = 0;
                while i != n {
                    if let Some(face) = *faces.at(i) {
                        let (a, b) = (*ids.at(i), *ids.at(next_index(i, n)));
                        // A concave corner has no outward range for its two edges to share, so
                        // the cone stops on this edge's own normal, as where the surface runs into
                        // a cut. Walking counter-clockwise turns the normals the same way.
                        let clockwise = match claimed(ref arriving, normals, a) {
                            Some(previous) => if turns_clockwise(previous, face) {
                                None
                            } else {
                                Some(previous)
                            },
                            None => None,
                        };
                        let counter_clockwise = match claimed(ref leaving, normals, b) {
                            Some(following) => if turns_clockwise(face, following) {
                                None
                            } else {
                                Some(following)
                            },
                            None => None,
                        };
                        // Cones are stated in the part's frame, where the narrow phase applies
                        // them.
                        let r = pose.rotation;
                        edges
                            .append(
                                CompoundEdgeCone {
                                    face: r.inverse_rotate(face),
                                    clockwise_limit: r.inverse_rotate(halfway_to(face, clockwise)),
                                    counter_clockwise_limit: r
                                        .inverse_rotate(halfway_to(face, counter_clockwise)),
                                },
                            );
                    }
                    i += 1;
                }
                out.append(Some(CompoundPseudoNormals { boundary_edges: edges.span() }));
            },
            _ => out.append(None),
        }
        p += 1;
    }
    out.span()
}
