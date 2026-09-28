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
/// The geometry that crosses (CS4), defined with the batched narrow phase (CS5).
pub use rapier2d::pipeline::stages::narrow::{
    ContactBatch, ContactJob, ManifoldGeometry, geometry, with_geometry,
};
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
use starknet::syscalls::library_call_syscall;
use starknet::{ClassHash, SyscallResultTrait};
use crate::hashes::{ClassHashes, errors};
use crate::narrow::wire::{put_job, put_result, take_job, take_result};

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

/// A pair with a ball goes to `ContactBallClass`, any other to `ContactPolygonClass`.
#[inline(always)]
fn ball_family(shape1: Shape, shape2: Shape) -> bool {
    match (shape1, shape2) {
        (Shape::Ball(_), _) | (_, Shape::Ball(_)) => true,
        _ => false,
    }
}

/// One library call of the family class at `class_hash` on `jobs` (not empty).
fn family_call(
    class_hash: ClassHash, prediction: Fixed, jobs: Span<ContactJob>,
) -> Span<(bool, ManifoldGeometry)> {
    let mut calldata = array![];
    prediction.serialize(ref calldata);
    jobs.serialize(ref calldata);
    let mut ret = library_call_syscall(class_hash, selector!("contact_batch"), calldata.span())
        .unwrap_syscall();
    Serde::deserialize(ref ret).expect(errors::DECODE)
}

/// The contact generation of a step's batch (the narrow phase's `BatchedNarrowPhase`): one call
/// of `ContactBallClass` (at `H::contact_ball()`) with the pairs with a ball and one of
/// `ContactPolygonClass` (at `H::contact_polygon()`) with the others, each skipped when it has no
/// pair; only the manifolds' geometry crosses, the solver data stays in the caller.
///
/// # Panics
/// `errors::DECODE` when a class returns something else than its result; as the classes (a shape
/// that is not basic: `UNSUPPORTED`).
pub impl FamilyBatch<impl H: ClassHashes> of ContactBatch {
    fn contact_geometries(
        prediction: Fixed, jobs: Span<ContactJob>,
    ) -> Span<(bool, ManifoldGeometry)> {
        family_geometries(H::contact_ball(), H::contact_polygon(), prediction, jobs)
    }
}

/// [`FamilyBatch`] with the family classes at `contact_ball` and `contact_polygon` (the hashes a
/// declared class receives, CS6: `crate::narrow`).
pub fn family_geometries(
    contact_ball: ClassHash, contact_polygon: ClassHash, prediction: Fixed, jobs: Span<ContactJob>,
) -> Span<(bool, ManifoldGeometry)> {
    let mut ball = array![];
    let mut polygon = array![];
    let mut families = array![];
    for job in jobs {
        let is_ball = ball_family(*job.shape1, *job.shape2);
        if is_ball {
            ball.append(*job);
        } else {
            polygon.append(*job);
        }
        families.append(is_ball);
    }
    if polygon.is_empty() {
        return family_call(contact_ball, prediction, ball.span());
    }
    if ball.is_empty() {
        return family_call(contact_polygon, prediction, polygon.span());
    }
    let mut ball = family_call(contact_ball, prediction, ball.span());
    let mut polygon = family_call(contact_polygon, prediction, polygon.span());
    let mut out = array![];
    for is_ball in families {
        out
            .append(
                if is_ball {
                    *ball.pop_front().unwrap()
                } else {
                    *polygon.pop_front().unwrap()
                },
            );
    }
    out.span()
}

/// [`family_geometries`] with the polygon family run here (CX2: the class that calls it is the
/// polygon family's): one call of the ball family class at `contact_ball` with the pairs with a
/// ball, skipped when there is none.
pub fn family_local_polygon(
    contact_ball: ClassHash, prediction: Fixed, jobs: Span<ContactJob>,
) -> Span<(bool, ManifoldGeometry)> {
    let mut ball = array![];
    for job in jobs {
        if ball_family(*job.shape1, *job.shape2) {
            ball.append(*job);
        }
    }
    let mut ball = if ball.is_empty() {
        array![].span()
    } else {
        family_call(contact_ball, prediction, ball.span())
    };
    let mut out = array![];
    for job in jobs {
        if ball_family(*job.shape1, *job.shape2) {
            out.append(*ball.pop_front().unwrap());
        } else {
            let mut manifold = with_geometry(*job.geometry, Default::default());
            let supported = contact_manifold_polygon_family(
                *job.pos12, *job.shape1, *job.shape2, prediction, ref manifold,
            );
            out.append((supported, geometry(@manifold)));
        }
    }
    out.span()
}

/// One library call of `contact_packed` with the job wire `calldata` (the prediction, then the
/// jobs): its result felts.
fn packed_call(class_hash: ClassHash, calldata: Array<felt252>) -> Span<felt252> {
    let mut ret = library_call_syscall(class_hash, selector!("contact_packed"), calldata.span())
        .unwrap_syscall();
    let _ = ret.pop_front();
    ret
}

/// [`family_geometries`] through the compact wires (CX2, `crate::narrow::wire`): the jobs and
/// the results packed.
pub fn family_packed(
    contact_ball: ClassHash, contact_polygon: ClassHash, prediction: Fixed, jobs: Span<ContactJob>,
) -> Span<(bool, ManifoldGeometry)> {
    let mut ball = array![];
    let mut polygon = array![];
    prediction.serialize(ref ball);
    prediction.serialize(ref polygon);
    let mut families = array![];
    let mut balls = false;
    let mut polygons = false;
    for job in jobs {
        let is_ball = ball_family(*job.shape1, *job.shape2);
        if is_ball {
            put_job(ref ball, job);
            balls = true;
        } else {
            put_job(ref polygon, job);
            polygons = true;
        }
        families.append(is_ball);
    }
    let mut ball = if balls {
        packed_call(contact_ball, ball)
    } else {
        array![].span()
    };
    let mut polygon = if polygons {
        packed_call(contact_polygon, polygon)
    } else {
        array![].span()
    };
    let mut out = array![];
    for is_ball in families {
        out.append(if is_ball {
            take_result(ref ball)
        } else {
            take_result(ref polygon)
        });
    }
    out.span()
}

/// [`contact_manifold_ball_family`] on each job of a job wire: the result wire.
pub fn ball_packed(prediction: Fixed, jobs: Span<felt252>) -> Span<felt252> {
    let mut jobs = jobs;
    let mut out = array![];
    while !jobs.is_empty() {
        let job = take_job(ref jobs);
        let mut manifold = with_geometry(job.geometry, Default::default());
        let supported = contact_manifold_ball_family(
            job.pos12, job.shape1, job.shape2, prediction, ref manifold,
        );
        put_result(ref out, supported, @geometry(@manifold));
    }
    out.span()
}

/// [`contact_manifold_polygon_family`] on each job of a job wire: the result wire.
pub fn polygon_packed(prediction: Fixed, jobs: Span<felt252>) -> Span<felt252> {
    let mut jobs = jobs;
    let mut out = array![];
    while !jobs.is_empty() {
        let job = take_job(ref jobs);
        let mut manifold = with_geometry(job.geometry, Default::default());
        let supported = contact_manifold_polygon_family(
            job.pos12, job.shape1, job.shape2, prediction, ref manifold,
        );
        put_result(ref out, supported, @geometry(@manifold));
    }
    out.span()
}

/// [`contact_manifold_ball_family`] on each job of a batch (default solver data, which no
/// generator reads).
pub fn ball_batch(prediction: Fixed, jobs: Span<ContactJob>) -> Span<(bool, ManifoldGeometry)> {
    let mut out = array![];
    for job in jobs {
        let mut manifold = with_geometry(*job.geometry, Default::default());
        let supported = contact_manifold_ball_family(
            *job.pos12, *job.shape1, *job.shape2, prediction, ref manifold,
        );
        out.append((supported, geometry(@manifold)));
    }
    out.span()
}

/// [`contact_manifold_polygon_family`] on each job of a batch.
pub fn polygon_batch(prediction: Fixed, jobs: Span<ContactJob>) -> Span<(bool, ManifoldGeometry)> {
    let mut out = array![];
    for job in jobs {
        let mut manifold = with_geometry(*job.geometry, Default::default());
        let supported = contact_manifold_polygon_family(
            *job.pos12, *job.shape1, *job.shape2, prediction, ref manifold,
        );
        out.append((supported, geometry(@manifold)));
    }
    out.span()
}

/// The contact generation of the pairs with a ball.
#[starknet::contract]
pub mod ContactBallClass {
    use rapier2d::prelude::{Fixed, Pose2, Shape};
    use crate::narrow::wire::Wire;
    use super::{ContactJob, ManifoldGeometry, contact_manifold_ball_family, with_geometry};

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
    /// [`super::ball_batch`]: a step's pairs with a ball at once.
    #[external(v0)]
    fn contact_batch(
        self: @ContractState, prediction: Fixed, jobs: Span<ContactJob>,
    ) -> Span<(bool, ManifoldGeometry)> {
        super::ball_batch(prediction, jobs)
    }

    /// [`super::ball_packed`]: a step's pairs with a ball on the compact wires (CX2).
    #[external(v0)]
    fn contact_packed(self: @ContractState, prediction: Fixed, jobs: Wire) -> Span<felt252> {
        super::ball_packed(prediction, jobs.felts)
    }
}

/// The contact generation of the pairs of cuboids, convex polygons and half-spaces.
#[starknet::contract]
pub mod ContactPolygonClass {
    use rapier2d::prelude::{Fixed, Pose2, Shape};
    use crate::narrow::wire::Wire;
    use super::{ContactJob, ManifoldGeometry, contact_manifold_polygon_family, with_geometry};

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
    /// [`super::polygon_batch`]: a step's pairs without a ball at once.
    #[external(v0)]
    fn contact_batch(
        self: @ContractState, prediction: Fixed, jobs: Span<ContactJob>,
    ) -> Span<(bool, ManifoldGeometry)> {
        super::polygon_batch(prediction, jobs)
    }

    /// [`super::polygon_packed`]: a step's pairs without a ball on the compact wires (CX2).
    #[external(v0)]
    fn contact_packed(self: @ContractState, prediction: Fixed, jobs: Wire) -> Span<felt252> {
        super::polygon_packed(prediction, jobs.felts)
    }
}
