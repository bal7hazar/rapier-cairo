//! Rejected candidates of the dispatcher, kept for the `gas_*` ranking (test builds only).

use fixed::Fixed;
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::contact::{ContactManifold, ContactManifoldTrait};
use crate::contact_generators::ball_ball::{
    contact_manifold_ball_ball, contact_manifold_ball_ball_shapes,
};
use crate::contact_generators::capsule_capsule::{
    contact_manifold_capsule_capsule, contact_manifold_capsule_capsule_shapes,
};
use crate::contact_generators::convex_ball::{
    contact_manifold_ball_convex, contact_manifold_convex_ball, contact_manifold_convex_ball_shapes,
};
use crate::contact_generators::cuboid_capsule::{
    contact_manifold_cuboid_capsule, contact_manifold_cuboid_capsule_shapes,
};
use crate::contact_generators::cuboid_cuboid::{
    contact_manifold_cuboid_cuboid, contact_manifold_cuboid_cuboid_shapes,
};
use crate::contact_generators::cuboid_segment::{
    contact_manifold_cuboid_segment, contact_manifold_cuboid_segment_shapes,
};
use crate::contact_generators::halfspace_pfm::{
    contact_manifold_halfspace_pfm, contact_manifold_halfspace_pfm_shapes,
};
use crate::contact_generators::pfm_pfm::contact_manifold_pfm_pfm;
use crate::shape::{Ball, Capsule, Cuboid, HalfSpace, Segment, Shape};
use super::contact_manifold;

/// GG's original winner: one typed `#[inline(always)]` match and direct generator calls.
///
/// This remains the cheapest form when fully inlined into a caller. Behind an outlined caller, the
/// loop-free match is charged as one function, i.e. the most expensive arm on every pair; see
/// [`contact_manifold_plain_outlined`].
#[inline(always)]
pub fn contact_manifold_plain(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    match (shape1, shape2) {
        (
            Shape::Ball(ball1), Shape::Ball(ball2),
        ) => {
            contact_manifold_ball_ball(pos12, ball1, ball2, prediction, ref manifold);
            true
        },
        (
            Shape::Cuboid(cuboid1), Shape::Cuboid(cuboid2),
        ) => {
            contact_manifold_cuboid_cuboid(pos12, cuboid1, cuboid2, prediction, ref manifold);
            true
        },
        (
            Shape::Capsule(capsule1), Shape::Capsule(capsule2),
        ) => {
            contact_manifold_capsule_capsule(pos12, capsule1, capsule2, prediction, ref manifold);
            true
        },
        (Shape::Ball(ball1), Shape::Triangle(_)) | (Shape::Ball(ball1), Shape::RoundCuboid(_)) |
        (Shape::Ball(ball1), Shape::RoundTriangle(_)) |
        (
            Shape::Ball(ball1), Shape::RoundConvexPolygon(_),
        ) => {
            contact_manifold_ball_convex(pos12, ball1, shape2, prediction, ref manifold);
            true
        },
        (
            Shape::Ball(ball1), _,
        ) => {
            contact_manifold_ball_convex(pos12, ball1, shape2, prediction, ref manifold);
            true
        },
        (Shape::Triangle(_), Shape::Ball(ball2)) | (Shape::RoundCuboid(_), Shape::Ball(ball2)) |
        (Shape::RoundTriangle(_), Shape::Ball(ball2)) |
        (
            Shape::RoundConvexPolygon(_), Shape::Ball(ball2),
        ) => {
            contact_manifold_convex_ball(pos12, shape1, ball2, prediction, ref manifold);
            true
        },
        (
            _, Shape::Ball(ball2),
        ) => {
            contact_manifold_convex_ball(pos12, shape1, ball2, prediction, ref manifold);
            true
        },
        (
            Shape::Cuboid(cuboid1), Shape::Capsule(capsule2),
        ) => {
            contact_manifold_cuboid_capsule(pos12, cuboid1, capsule2, prediction, ref manifold);
            true
        },
        (
            Shape::Capsule(_), Shape::Cuboid(_),
        ) => contact_manifold_cuboid_capsule_shapes(
            pos12, shape1, shape2, prediction, ref manifold,
        ),
        (
            Shape::Cuboid(cuboid1), Shape::Segment(segment2),
        ) => {
            contact_manifold_cuboid_segment(pos12, cuboid1, segment2, prediction, ref manifold);
            true
        },
        (
            Shape::Segment(_), Shape::Cuboid(_),
        ) => contact_manifold_cuboid_segment_shapes(
            pos12, shape1, shape2, prediction, ref manifold,
        ),
        (Shape::HalfSpace(halfspace1), Shape::ConvexPolygon(_)) |
        (
            Shape::HalfSpace(halfspace1), Shape::Cuboid(_),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12, halfspace1, shape2, prediction, ref manifold, false,
            );
            true
        },
        (
            Shape::HalfSpace(halfspace1), Shape::Segment(_),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12, halfspace1, shape2, prediction, ref manifold, false,
            );
            true
        },
        (
            Shape::HalfSpace(halfspace1), Shape::Capsule(_),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12, halfspace1, shape2, prediction, ref manifold, false,
            );
            true
        },
        (Shape::ConvexPolygon(_), Shape::HalfSpace(halfspace2)) |
        (
            Shape::Cuboid(_), Shape::HalfSpace(halfspace2),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12.inverse(), halfspace2, shape1, prediction, ref manifold, true,
            );
            true
        },
        (
            Shape::Segment(_), Shape::HalfSpace(halfspace2),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12.inverse(), halfspace2, shape1, prediction, ref manifold, true,
            );
            true
        },
        (
            Shape::Capsule(_), Shape::HalfSpace(halfspace2),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12.inverse(), halfspace2, shape1, prediction, ref manifold, true,
            );
            true
        },
        (Shape::Triangle(_), _) | (Shape::RoundCuboid(_), _) | (Shape::RoundTriangle(_), _) |
        (
            Shape::RoundConvexPolygon(_), _,
        ) => {
            contact_manifold_pfm_pfm(pos12, shape1, shape2, prediction, ref manifold);
            true
        },
        (
            _, Shape::Triangle(x2),
        ) => {
            contact_manifold_pfm_pfm(pos12, shape1, Shape::Triangle(x2), prediction, ref manifold);
            true
        },
        (
            _, Shape::RoundCuboid(x2),
        ) => {
            contact_manifold_pfm_pfm(
                pos12, shape1, Shape::RoundCuboid(x2), prediction, ref manifold,
            );
            true
        },
        (
            _, Shape::RoundTriangle(x2),
        ) => {
            contact_manifold_pfm_pfm(
                pos12, shape1, Shape::RoundTriangle(x2), prediction, ref manifold,
            );
            true
        },
        (
            _, Shape::RoundConvexPolygon(x2),
        ) => {
            contact_manifold_pfm_pfm(
                pos12, shape1, Shape::RoundConvexPolygon(x2), prediction, ref manifold,
            );
            true
        },
        _ => {
            manifold.clear();
            false
        },
    }
}

/// The metered winner behind a call: each generator is still charged only when its one-iteration
/// loop body runs.
#[inline(never)]
pub fn contact_manifold_outlined(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    contact_manifold(pos12, shape1, shape2, prediction, ref manifold)
}

/// GG's plain typed match behind a call: the whole `match` is charged as one function, i.e. the
/// most expensive arm on every pair.
#[inline(never)]
pub fn contact_manifold_plain_outlined(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    contact_manifold_plain(pos12, shape1, shape2, prediction, ref manifold)
}

/// The winner with one `#[inline(never)]` helper per arm (AGENTS.md section 7): same path
/// sensitivity, plus one call per pair.
#[inline(always)]
pub fn contact_manifold_helpers(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    match (shape1, shape2) {
        (
            Shape::Ball(ball1), Shape::Ball(ball2),
        ) => {
            ball_ball(pos12, ball1, ball2, prediction, ref manifold);
            true
        },
        (
            Shape::Cuboid(cuboid1), Shape::Cuboid(cuboid2),
        ) => {
            cuboid_cuboid(pos12, cuboid1, cuboid2, prediction, ref manifold);
            true
        },
        (
            Shape::Capsule(capsule1), Shape::Capsule(capsule2),
        ) => {
            capsule_capsule(pos12, capsule1, capsule2, prediction, ref manifold);
            true
        },
        (Shape::Ball(ball1), Shape::Triangle(_)) | (Shape::Ball(ball1), Shape::RoundCuboid(_)) |
        (Shape::Ball(ball1), Shape::RoundTriangle(_)) |
        (
            Shape::Ball(ball1), Shape::RoundConvexPolygon(_),
        ) => {
            ball_convex(pos12, ball1, shape2, prediction, ref manifold);
            true
        },
        (
            Shape::Ball(ball1), _,
        ) => {
            ball_convex(pos12, ball1, shape2, prediction, ref manifold);
            true
        },
        (Shape::Triangle(_), Shape::Ball(ball2)) | (Shape::RoundCuboid(_), Shape::Ball(ball2)) |
        (Shape::RoundTriangle(_), Shape::Ball(ball2)) |
        (
            Shape::RoundConvexPolygon(_), Shape::Ball(ball2),
        ) => {
            convex_ball(pos12, shape1, ball2, prediction, ref manifold);
            true
        },
        (
            _, Shape::Ball(ball2),
        ) => {
            convex_ball(pos12, shape1, ball2, prediction, ref manifold);
            true
        },
        (
            Shape::Cuboid(cuboid1), Shape::Capsule(capsule2),
        ) => {
            cuboid_capsule(pos12, cuboid1, capsule2, prediction, ref manifold);
            true
        },
        (
            Shape::Capsule(_), Shape::Cuboid(_),
        ) => capsule_cuboid(pos12, shape1, shape2, prediction, ref manifold),
        (
            Shape::Cuboid(cuboid1), Shape::Segment(segment2),
        ) => {
            cuboid_segment(pos12, cuboid1, segment2, prediction, ref manifold);
            true
        },
        (
            Shape::Segment(_), Shape::Cuboid(_),
        ) => segment_cuboid(pos12, shape1, shape2, prediction, ref manifold),
        (Shape::HalfSpace(halfspace1), Shape::ConvexPolygon(_)) |
        (
            Shape::HalfSpace(halfspace1), Shape::Cuboid(_),
        ) => {
            halfspace_pfm_helper(pos12, halfspace1, shape2, prediction, ref manifold);
            true
        },
        (
            Shape::HalfSpace(halfspace1), Shape::Segment(_),
        ) => {
            halfspace_pfm_helper(pos12, halfspace1, shape2, prediction, ref manifold);
            true
        },
        (
            Shape::HalfSpace(halfspace1), Shape::Capsule(_),
        ) => {
            halfspace_pfm_helper(pos12, halfspace1, shape2, prediction, ref manifold);
            true
        },
        (Shape::ConvexPolygon(_), Shape::HalfSpace(halfspace2)) |
        (
            Shape::Cuboid(_), Shape::HalfSpace(halfspace2),
        ) => {
            pfm_halfspace(pos12, shape1, halfspace2, prediction, ref manifold);
            true
        },
        (
            Shape::Segment(_), Shape::HalfSpace(halfspace2),
        ) => {
            pfm_halfspace(pos12, shape1, halfspace2, prediction, ref manifold);
            true
        },
        (
            Shape::Capsule(_), Shape::HalfSpace(halfspace2),
        ) => {
            pfm_halfspace(pos12, shape1, halfspace2, prediction, ref manifold);
            true
        },
        (Shape::Triangle(_), _) | (Shape::RoundCuboid(_), _) | (Shape::RoundTriangle(_), _) |
        (
            Shape::RoundConvexPolygon(_), _,
        ) => {
            contact_manifold_pfm_pfm(pos12, shape1, shape2, prediction, ref manifold);
            true
        },
        (
            _, Shape::Triangle(x2),
        ) => {
            contact_manifold_pfm_pfm(pos12, shape1, Shape::Triangle(x2), prediction, ref manifold);
            true
        },
        (
            _, Shape::RoundCuboid(x2),
        ) => {
            contact_manifold_pfm_pfm(
                pos12, shape1, Shape::RoundCuboid(x2), prediction, ref manifold,
            );
            true
        },
        (
            _, Shape::RoundTriangle(x2),
        ) => {
            contact_manifold_pfm_pfm(
                pos12, shape1, Shape::RoundTriangle(x2), prediction, ref manifold,
            );
            true
        },
        (
            _, Shape::RoundConvexPolygon(x2),
        ) => {
            contact_manifold_pfm_pfm(
                pos12, shape1, Shape::RoundConvexPolygon(x2), prediction, ref manifold,
            );
            true
        },
        _ => {
            manifold.clear();
            false
        },
    }
}

#[inline(never)]
fn ball_ball(
    pos12: Pose2, ball1: Ball, ball2: Ball, prediction: Fixed, ref manifold: ContactManifold,
) {
    contact_manifold_ball_ball(pos12, ball1, ball2, prediction, ref manifold);
}

#[inline(never)]
fn cuboid_cuboid(
    pos12: Pose2,
    cuboid1: Cuboid,
    cuboid2: Cuboid,
    prediction: Fixed,
    ref manifold: ContactManifold,
) {
    contact_manifold_cuboid_cuboid(pos12, cuboid1, cuboid2, prediction, ref manifold);
}

#[inline(never)]
fn capsule_capsule(
    pos12: Pose2,
    capsule1: Capsule,
    capsule2: Capsule,
    prediction: Fixed,
    ref manifold: ContactManifold,
) {
    contact_manifold_capsule_capsule(pos12, capsule1, capsule2, prediction, ref manifold);
}

#[inline(never)]
fn ball_convex(
    pos12: Pose2, ball1: Ball, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) {
    contact_manifold_ball_convex(pos12, ball1, shape2, prediction, ref manifold);
}

#[inline(never)]
fn convex_ball(
    pos12: Pose2, shape1: Shape, ball2: Ball, prediction: Fixed, ref manifold: ContactManifold,
) {
    contact_manifold_convex_ball(pos12, shape1, ball2, prediction, ref manifold);
}

#[inline(never)]
fn cuboid_capsule(
    pos12: Pose2,
    cuboid1: Cuboid,
    capsule2: Capsule,
    prediction: Fixed,
    ref manifold: ContactManifold,
) {
    contact_manifold_cuboid_capsule(pos12, cuboid1, capsule2, prediction, ref manifold);
}

/// Capsule–cuboid: only the `Shape` wrapper of `cuboid_capsule` reaches the flipped
/// generator.
#[inline(never)]
fn capsule_cuboid(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    contact_manifold_cuboid_capsule_shapes(pos12, shape1, shape2, prediction, ref manifold)
}

#[inline(never)]
fn cuboid_segment(
    pos12: Pose2,
    cuboid1: Cuboid,
    segment2: Segment,
    prediction: Fixed,
    ref manifold: ContactManifold,
) {
    contact_manifold_cuboid_segment(pos12, cuboid1, segment2, prediction, ref manifold);
}

/// Segment–cuboid: only the `Shape` wrapper of `cuboid_segment` reaches the flipped
/// generator.
#[inline(never)]
fn segment_cuboid(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    contact_manifold_cuboid_segment_shapes(pos12, shape1, shape2, prediction, ref manifold)
}

#[inline(never)]
fn halfspace_pfm_helper(
    pos12: Pose2,
    halfspace1: HalfSpace,
    pfm2: Shape,
    prediction: Fixed,
    ref manifold: ContactManifold,
) {
    contact_manifold_halfspace_pfm(pos12, halfspace1, pfm2, prediction, ref manifold, false);
}

/// Cuboid / segment / capsule first, half-space second: upstream's `flipped = true`.
#[inline(never)]
fn pfm_halfspace(
    pos12: Pose2,
    pfm1: Shape,
    halfspace2: HalfSpace,
    prediction: Fixed,
    ref manifold: ContactManifold,
) {
    contact_manifold_halfspace_pfm(
        pos12.inverse(), halfspace2, pfm1, prediction, ref manifold, true,
    );
}

/// Upstream's shape: the `*_shapes` wrappers tried in priority order, each one re-matching the
/// enum (inlined like the winner). Also the reference the tests compare the dispatcher with.
#[inline(always)]
pub fn contact_manifold_shapes_chain(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    if contact_manifold_ball_ball_shapes(pos12, shape1, shape2, prediction, ref manifold)
        || contact_manifold_cuboid_cuboid_shapes(pos12, shape1, shape2, prediction, ref manifold)
        || contact_manifold_capsule_capsule_shapes(pos12, shape1, shape2, prediction, ref manifold)
        || contact_manifold_convex_ball_shapes(pos12, shape1, shape2, prediction, ref manifold)
        || contact_manifold_cuboid_capsule_shapes(pos12, shape1, shape2, prediction, ref manifold)
        || contact_manifold_cuboid_segment_shapes(pos12, shape1, shape2, prediction, ref manifold)
        || contact_manifold_halfspace_pfm_shapes(pos12, shape1, shape2, prediction, ref manifold) {
        true
    } else {
        manifold.clear();
        false
    }
}
use crate::manifold::ManifoldTrait;
use super::{
    contact_manifold_polygon_capsule, contact_manifold_polygon_cuboid,
    contact_manifold_polygon_polygon, contact_manifold_polygon_segment,
};

// CP2 rejected fallback table: 1 extra step per persistent cuboid pair.
#[inline(always)]
pub fn contact_manifold_step_fallback(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    match (shape1, shape2) {
        (
            Shape::Ball(ball1), Shape::Ball(ball2),
        ) => {
            contact_manifold_ball_ball(pos12, ball1, ball2, prediction, ref manifold);
            true
        },
        (
            Shape::Cuboid(cuboid1), Shape::Cuboid(cuboid2),
        ) => {
            if !manifold.try_update_contacts(pos12) {
                contact_manifold_cuboid_cuboid(pos12, cuboid1, cuboid2, prediction, ref manifold);
            }
            true
        },
        (
            Shape::Capsule(capsule1), Shape::Capsule(capsule2),
        ) => {
            contact_manifold_capsule_capsule(pos12, capsule1, capsule2, prediction, ref manifold);
            true
        },
        (Shape::Ball(ball1), Shape::Triangle(_)) | (Shape::Ball(ball1), Shape::RoundCuboid(_)) |
        (Shape::Ball(ball1), Shape::RoundTriangle(_)) |
        (
            Shape::Ball(ball1), Shape::RoundConvexPolygon(_),
        ) => {
            contact_manifold_ball_convex(pos12, ball1, shape2, prediction, ref manifold);
            true
        },
        (
            Shape::Ball(ball1), _,
        ) => {
            contact_manifold_ball_convex(pos12, ball1, shape2, prediction, ref manifold);
            true
        },
        (Shape::Triangle(_), Shape::Ball(ball2)) | (Shape::RoundCuboid(_), Shape::Ball(ball2)) |
        (Shape::RoundTriangle(_), Shape::Ball(ball2)) |
        (
            Shape::RoundConvexPolygon(_), Shape::Ball(ball2),
        ) => {
            contact_manifold_convex_ball(pos12, shape1, ball2, prediction, ref manifold);
            true
        },
        (
            _, Shape::Ball(ball2),
        ) => {
            contact_manifold_convex_ball(pos12, shape1, ball2, prediction, ref manifold);
            true
        },
        (
            Shape::Cuboid(cuboid1), Shape::Capsule(capsule2),
        ) => {
            if !manifold.try_update_contacts(pos12) {
                contact_manifold_cuboid_capsule(pos12, cuboid1, capsule2, prediction, ref manifold);
            }
            true
        },
        (
            Shape::Capsule(_), Shape::Cuboid(_),
        ) => {
            if manifold.try_update_contacts(pos12) {
                return true;
            }
            contact_manifold_cuboid_capsule_shapes(pos12, shape1, shape2, prediction, ref manifold)
        },
        (
            Shape::Cuboid(cuboid1), Shape::Segment(segment2),
        ) => {
            if !manifold.try_update_contacts(pos12) {
                contact_manifold_cuboid_segment(pos12, cuboid1, segment2, prediction, ref manifold);
            }
            true
        },
        (
            Shape::Segment(_), Shape::Cuboid(_),
        ) => {
            if manifold.try_update_contacts(pos12) {
                return true;
            }
            contact_manifold_cuboid_segment_shapes(pos12, shape1, shape2, prediction, ref manifold)
        },
        (Shape::HalfSpace(halfspace1), Shape::ConvexPolygon(_)) |
        (Shape::HalfSpace(halfspace1), Shape::Cuboid(_)) |
        (Shape::HalfSpace(halfspace1), Shape::Segment(_)) |
        (
            Shape::HalfSpace(halfspace1), Shape::Capsule(_),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12, halfspace1, shape2, prediction, ref manifold, false,
            );
            true
        },
        (Shape::ConvexPolygon(_), Shape::HalfSpace(halfspace2)) |
        (Shape::Cuboid(_), Shape::HalfSpace(halfspace2)) |
        (Shape::Segment(_), Shape::HalfSpace(halfspace2)) |
        (
            Shape::Capsule(_), Shape::HalfSpace(halfspace2),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12.inverse(), halfspace2, shape1, prediction, ref manifold, true,
            );
            true
        },
        (Shape::Triangle(_), _) | (Shape::RoundCuboid(_), _) | (Shape::RoundTriangle(_), _) |
        (
            Shape::RoundConvexPolygon(_), _,
        ) => {
            contact_manifold_pfm_pfm(pos12, shape1, shape2, prediction, ref manifold);
            true
        },
        (
            _, Shape::Triangle(x2),
        ) => {
            contact_manifold_pfm_pfm(pos12, shape1, Shape::Triangle(x2), prediction, ref manifold);
            true
        },
        (
            _, Shape::RoundCuboid(x2),
        ) => {
            contact_manifold_pfm_pfm(
                pos12, shape1, Shape::RoundCuboid(x2), prediction, ref manifold,
            );
            true
        },
        (
            _, Shape::RoundTriangle(x2),
        ) => {
            contact_manifold_pfm_pfm(
                pos12, shape1, Shape::RoundTriangle(x2), prediction, ref manifold,
            );
            true
        },
        (
            _, Shape::RoundConvexPolygon(x2),
        ) => {
            contact_manifold_pfm_pfm(
                pos12, shape1, Shape::RoundConvexPolygon(x2), prediction, ref manifold,
            );
            true
        },
        _ => polygon_pair(pos12, shape1, shape2, prediction, ref manifold),
    }
}

// Preserve the original match layout for existing pairs. New pairs use their own metered table.
#[inline(never)]
fn polygon_pair(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    match (shape1, shape2) {
        (
            Shape::ConvexPolygon(a), Shape::ConvexPolygon(b),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_polygon_polygon(
                    pos12, a.unbox(), b.unbox(), prediction, ref manifold,
                );
                pending = false;
            }
            true
        },
        (
            Shape::ConvexPolygon(a), Shape::Cuboid(b),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_polygon_cuboid(
                    pos12, a.unbox(), b, prediction, false, ref manifold,
                );
                pending = false;
            }
            true
        },
        (
            Shape::Cuboid(b), Shape::ConvexPolygon(a),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_polygon_cuboid(
                    pos12.inverse(), a.unbox(), b, prediction, true, ref manifold,
                );
                pending = false;
            }
            true
        },
        (
            Shape::ConvexPolygon(a), Shape::Segment(b),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_polygon_segment(
                    pos12, a.unbox(), b, prediction, false, ref manifold,
                );
                pending = false;
            }
            true
        },
        (
            Shape::Segment(b), Shape::ConvexPolygon(a),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_polygon_segment(
                    pos12.inverse(), a.unbox(), b, prediction, true, ref manifold,
                );
                pending = false;
            }
            true
        },
        (
            Shape::ConvexPolygon(a), Shape::Capsule(b),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_polygon_capsule(
                    pos12, a.unbox(), b, prediction, false, ref manifold,
                );
                pending = false;
            }
            true
        },
        (
            Shape::Capsule(b), Shape::ConvexPolygon(a),
        ) => {
            let mut pending = true;
            while pending {
                contact_manifold_polygon_capsule(
                    pos12.inverse(), a.unbox(), b, prediction, true, ref manifold,
                );
                pending = false;
            }
            true
        },
        _ => {
            manifold.clear();
            false
        },
    }
}

/// Boxed-argument candidate retained after measuring no improvement on P3 scenes.
pub mod boxed;
