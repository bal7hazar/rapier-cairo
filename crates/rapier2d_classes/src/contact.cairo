//! The contact generation of the basic shapes in two declared classes, one per shape-pair family:
//! the pairs with a ball (`ball_ball`, `convex_ball`) and the others (`cuboid_cuboid`,
//! `polygon_polygon`, `halfspace_pfm`). The arms are those of
//! `rapier_geometry2d::dispatch::basic::contact_manifold_step_basic` (the dispatcher of
//! `BasicStepConfig`), through the generators' public entries: same results.
//!
//! Only a manifold's geometry crosses ([`ManifoldGeometry`], 29 felts): its solver data, which no
//! generator reads, stays in the caller (CS3: 2,519 Cairo steps per call against 3,466 for the
//! whole manifold).

use rapier2d::pipeline::config::ContactDispatcher;
use rapier2d::prelude::{Fixed, Pose2, Pose2Trait, Shape, Vec2};
use rapier_geometry2d::contact::{
    ContactManifold, ContactManifoldData, ContactManifoldTrait, TrackedContact,
};
use rapier_geometry2d::contact_generators::ball_ball::contact_manifold_ball_ball;
use rapier_geometry2d::contact_generators::convex_ball::{
    contact_manifold_ball_convex, contact_manifold_convex_ball,
};
use rapier_geometry2d::contact_generators::cuboid_cuboid::contact_manifold_cuboid_cuboid;
use rapier_geometry2d::contact_generators::halfspace_pfm::contact_manifold_halfspace_pfm;
use rapier_geometry2d::contact_generators::polygon_polygon::{
    contact_manifold_polygon_cuboid, contact_manifold_polygon_polygon,
};
use rapier_geometry2d::dispatch::basic::errors::UNSUPPORTED;
use starknet::SyscallResultTrait;
use starknet::syscalls::library_call_syscall;
use crate::hashes::{ClassHashes, errors};

/// What a contact generator reads and writes of a manifold: [`ContactManifold`] without its
/// solver data (`data`, which only the narrow phase writes, after the generator).
#[derive(Copy, Drop, Serde)]
pub struct ManifoldGeometry {
    pub points: [TrackedContact; 2],
    pub num_points: u8,
    pub local_n1: Vec2,
    pub local_n2: Vec2,
    pub subshape1: u32,
    pub subshape2: u32,
}

/// `manifold`'s geometry.
pub fn geometry(manifold: @ContactManifold) -> ManifoldGeometry {
    ManifoldGeometry {
        points: *manifold.points,
        num_points: *manifold.num_points,
        local_n1: *manifold.local_n1,
        local_n2: *manifold.local_n2,
        subshape1: *manifold.subshape1,
        subshape2: *manifold.subshape2,
    }
}

/// The manifold of `geometry` with the solver data `data`.
pub fn with_geometry(geometry: ManifoldGeometry, data: ContactManifoldData) -> ContactManifold {
    let ManifoldGeometry {
        points, num_points, local_n1, local_n2, subshape1, subshape2,
    } = geometry;
    ContactManifold { points, num_points, local_n1, local_n2, subshape1, subshape2, data }
}

/// The pairs with a ball: `contact_manifold_step_basic`'s ball arms. Always `true` (supported).
///
/// # Panics
/// `UNSUPPORTED` on a pair without a ball or with a shape that is not basic.
#[inline(always)]
pub fn contact_manifold_ball_family(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    match (shape1, shape2) {
        (
            Shape::Ball(ball1), Shape::Ball(ball2),
        ) => { contact_manifold_ball_ball(pos12, ball1, ball2, prediction, ref manifold); },
        (Shape::Ball(ball1), Shape::Cuboid(_)) | (Shape::Ball(ball1), Shape::ConvexPolygon(_)) |
        (
            Shape::Ball(ball1), Shape::HalfSpace(_),
        ) => { contact_manifold_ball_convex(pos12, ball1, shape2, prediction, ref manifold); },
        (Shape::Cuboid(_), Shape::Ball(ball2)) | (Shape::ConvexPolygon(_), Shape::Ball(ball2)) |
        (
            Shape::HalfSpace(_), Shape::Ball(ball2),
        ) => { contact_manifold_convex_ball(pos12, shape1, ball2, prediction, ref manifold); },
        _ => core::panic_with_felt252(UNSUPPORTED),
    }
    true
}

/// The pairs of cuboids, convex polygons and half-spaces: `contact_manifold_step_basic`'s other
/// arms (a half-space pair is unsupported: `false`, manifold cleared).
///
/// # Panics
/// `UNSUPPORTED` on a pair with a ball or with a shape that is not basic.
#[inline(always)]
pub fn contact_manifold_polygon_family(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    match (shape1, shape2) {
        (
            Shape::Cuboid(cuboid1), Shape::Cuboid(cuboid2),
        ) => { contact_manifold_cuboid_cuboid(pos12, cuboid1, cuboid2, prediction, ref manifold); },
        (Shape::HalfSpace(halfspace1), Shape::ConvexPolygon(_)) |
        (
            Shape::HalfSpace(halfspace1), Shape::Cuboid(_),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12, halfspace1, shape2, prediction, ref manifold, false,
            );
        },
        (Shape::ConvexPolygon(_), Shape::HalfSpace(halfspace2)) |
        (
            Shape::Cuboid(_), Shape::HalfSpace(halfspace2),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12.inverse(), halfspace2, shape1, prediction, ref manifold, true,
            );
        },
        (
            Shape::ConvexPolygon(a), Shape::ConvexPolygon(b),
        ) => {
            contact_manifold_polygon_polygon(pos12, a.unbox(), b.unbox(), prediction, ref manifold);
        },
        (
            Shape::ConvexPolygon(a), Shape::Cuboid(b),
        ) => {
            contact_manifold_polygon_cuboid(pos12, a.unbox(), b, prediction, false, ref manifold);
        },
        (
            Shape::Cuboid(b), Shape::ConvexPolygon(a),
        ) => {
            contact_manifold_polygon_cuboid(
                pos12.inverse(), a.unbox(), b, prediction, true, ref manifold,
            );
        },
        (Shape::HalfSpace(_), Shape::HalfSpace(_)) => {
            manifold.clear();
            return false;
        },
        _ => core::panic_with_felt252(UNSUPPORTED),
    }
    true
}

/// The contact dispatcher of a caller class: each pair's manifold geometry computed by the class
/// of its family, `ContactBallClass` (a pair with a ball, at `H::contact_ball()`) or
/// `ContactPolygonClass` (at `H::contact_polygon()`); the solver data stays in the caller.
///
/// # Panics
/// `errors::DECODE` when the class returns something else than its result; as the class (a shape
/// that is not basic: `UNSUPPORTED`).
pub impl FamilyDispatcher<impl H: ClassHashes> of ContactDispatcher {
    fn contact_manifold(
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) -> bool {
        let class_hash = match (shape1, shape2) {
            (Shape::Ball(_), _) | (_, Shape::Ball(_)) => H::contact_ball(),
            _ => H::contact_polygon(),
        };
        let mut calldata = array![];
        pos12.serialize(ref calldata);
        shape1.serialize(ref calldata);
        shape2.serialize(ref calldata);
        prediction.serialize(ref calldata);
        geometry(@manifold).serialize(ref calldata);
        let mut ret = library_call_syscall(
            class_hash, selector!("contact_geometry"), calldata.span(),
        )
            .unwrap_syscall();
        let (supported, out): (bool, ManifoldGeometry) = Serde::deserialize(ref ret)
            .expect(errors::DECODE);
        manifold = with_geometry(out, manifold.data);
        supported
    }
}

/// The contact generation of the pairs with a ball.
#[starknet::contract]
pub mod ContactBallClass {
    use rapier2d::prelude::{Fixed, Pose2, Shape};
    use super::{ManifoldGeometry, contact_manifold_ball_family, with_geometry};

    #[storage]
    struct Storage {}

    /// [`contact_manifold_ball_family`] on the manifold of `geometry` (default solver data, which
    /// no generator reads): whether the pair is supported, and the updated geometry.
    #[external(v0)]
    fn contact_geometry(
        self: @ContractState,
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        geometry: ManifoldGeometry,
    ) -> (bool, ManifoldGeometry) {
        let mut manifold = with_geometry(geometry, Default::default());
        let supported = contact_manifold_ball_family(
            pos12, shape1, shape2, prediction, ref manifold,
        );
        (supported, super::geometry(@manifold))
    }
}

/// The contact generation of the pairs of cuboids, convex polygons and half-spaces.
#[starknet::contract]
pub mod ContactPolygonClass {
    use rapier2d::prelude::{Fixed, Pose2, Shape};
    use super::{ManifoldGeometry, contact_manifold_polygon_family, with_geometry};

    #[storage]
    struct Storage {}

    /// [`contact_manifold_polygon_family`] on the manifold of `geometry`: whether the pair is
    /// supported, and the updated geometry.
    #[external(v0)]
    fn contact_geometry(
        self: @ContractState,
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        geometry: ManifoldGeometry,
    ) -> (bool, ManifoldGeometry) {
        let mut manifold = with_geometry(geometry, Default::default());
        let supported = contact_manifold_polygon_family(
            pos12, shape1, shape2, prediction, ref manifold,
        );
        (supported, super::geometry(@manifold))
    }
}
