//! The contact generation of the game's shapes in two declared classes, one per shape-pair
//! family (work package CS3): the pairs with a ball (`ball_ball`, `convex_ball`) and the others
//! (`cuboid_cuboid`, `polygon_polygon`, `halfspace_pfm`). [`FamilyDispatcher`] sends each pair
//! to the class of its family; the arms are those of
//! `rapier_geometry2d::dispatch::basic::contact_manifold_step_basic`, through the generators'
//! public entries (same results; `tests/split.cairo` checks the digest on pile10).
//!
//! * `ContactBallClass`: [`contact_manifold_ball_family`] behind `contact_geometry`.
//! * `ContactPolygonClass`: [`contact_manifold_polygon_family`] behind `contact_geometry`.
//! * `FamilyStep`: the caller class (`BasicGameStep` with [`FamilyStepConfig`]).

use rapier2d::pipeline::config::{ContactDispatcher, NoComposites, NoJoints, NoSensors, StepConfig};
use rapier2d::prelude::{Fixed, Pose2, Pose2Trait, Shape};
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldTrait};
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
use crate::split::{ManifoldGeometry, class_at, errors, geometry, with_geometry};

/// Storage slots of the two class hashes.
pub const BALL_CLASS_SLOT: felt252 = selector!("contact_ball_class");
pub const POLYGON_CLASS_SLOT: felt252 = selector!("contact_polygon_class");

/// The pairs with a ball: `contact_manifold_step_basic`'s ball arms.
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

/// Each pair's manifold geometry computed by the class of its family (`ContactBallClass` or
/// `ContactPolygonClass`, class hashes at [`BALL_CLASS_SLOT`] / [`POLYGON_CLASS_SLOT`]).
pub impl FamilyDispatcher of ContactDispatcher {
    fn contact_manifold(
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) -> bool {
        let slot = match (shape1, shape2) {
            (Shape::Ball(_), _) | (_, Shape::Ball(_)) => BALL_CLASS_SLOT,
            _ => POLYGON_CLASS_SLOT,
        };
        let mut calldata = array![];
        pos12.serialize(ref calldata);
        shape1.serialize(ref calldata);
        shape2.serialize(ref calldata);
        prediction.serialize(ref calldata);
        geometry(@manifold).serialize(ref calldata);
        let mut ret = library_call_syscall(
            class_at(slot), selector!("contact_geometry"), calldata.span(),
        )
            .unwrap_syscall();
        let (supported, out): (bool, ManifoldGeometry) = Serde::deserialize(ref ret)
            .expect(errors::DECODE);
        manifold = with_geometry(out, manifold.data);
        supported
    }
}

/// `BasicStepConfig` with the contact generation in the two family classes.
pub impl FamilyStepConfig of StepConfig {
    impl Dispatcher = FamilyDispatcher;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
}

#[starknet::contract]
pub mod ContactBallClass {
    use rapier2d::prelude::{Fixed, Pose2, Shape};
    use crate::split::{ManifoldGeometry, with_geometry};
    use super::contact_manifold_ball_family;

    #[storage]
    struct Storage {}

    /// `ContactClass::contact_geometry` on the pairs with a ball.
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
        (supported, crate::split::geometry(@manifold))
    }
}

#[starknet::contract]
pub mod ContactPolygonClass {
    use rapier2d::prelude::{Fixed, Pose2, Shape};
    use crate::split::{ManifoldGeometry, with_geometry};
    use super::contact_manifold_polygon_family;

    #[storage]
    struct Storage {}

    /// `ContactClass::contact_geometry` on the pairs without a ball.
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
        (supported, crate::split::geometry(@manifold))
    }
}
