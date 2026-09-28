//! The batched narrow phase (CS5): the pair loop of `compute_contacts_from_scratch_with` in two
//! passes, so that the contact generation of every pair of the step is handed to a
//! [`ContactBatch`] at once instead of one dispatcher call per pair.
//!
//! 1. The pairs that reach the dispatcher (both colliders solid, not filtered) are collected in
//!    pair order, each as a [`ContactJob`]: the pose of collider 2 in collider 1's frame, the two
//!    shapes and the geometry of the previous step's manifold of the pair (found by the same
//!    sorted walk over the previous pairs, read only), or a default one.
//! 2. [`ContactBatch::contact_geometries`] updates every job's geometry.
//! 3. The pair loop runs as before, the dispatcher call replaced by the next batch result (the
//!    new geometry with the previous manifold's solver data, which no contact generator reads):
//!    same carry-over, filters, solver data, events and pair list.
//!
//! The previous manifold a job carries is the one the loop would hand the dispatcher (the pair of
//! the same key, same generations, not an intersection pair), so the results are those of
//! `PairLoopNarrowPhase` with the batch's dispatcher. No composite pair is compiled (as
//! `NoComposites`).
//!
//! [`InProcessBatch`] runs the batch in the calling program (the batched loop measured against the
//! per-pair loop); `rapier2d_classes::FamilyBatch` library-calls one class per shape-pair family.

use fixed::Fixed;
use glam::Vec2;
use rapier_core::collider::CollisionEventFlagsTrait;
use rapier_dynamics2d::collider_set::ColliderSet;
use rapier_dynamics2d::events::{
    CollisionEvent, PairEventStatusTrait, START_EVENT_EMITTED, started, stopped,
};
use rapier_dynamics2d::narrow_phase::strategies::IntersectionStrategy;
use rapier_dynamics2d::narrow_phase::strategies::errors::COMPOSITE;
use rapier_dynamics2d::narrow_phase::{
    ContactDispatcher, ContactPair, ContactPairTrait, NarrowPhase, PairCollider, dropped_event,
    events_on, key_before, pair_filtered, pair_pose, solver_data_supported,
};
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldData, TrackedContact};
use rapier_geometry2d::shape::{Shape, ShapeTrait};
use rapier_math::pose2::Pose2;
use super::NarrowPhaseStage;

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
#[inline(always)]
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
#[inline(always)]
pub fn with_geometry(geometry: ManifoldGeometry, data: ContactManifoldData) -> ContactManifold {
    let ManifoldGeometry {
        points, num_points, local_n1, local_n2, subshape1, subshape2,
    } = geometry;
    ContactManifold { points, num_points, local_n1, local_n2, subshape1, subshape2, data }
}

/// What the contact generation of one pair reads: the pose of shape 2 in shape 1's frame, the
/// shapes and the geometry of the pair's previous manifold (a default one for a new pair).
#[derive(Copy, Drop, Serde)]
pub struct ContactJob {
    pub pos12: Pose2,
    pub shape1: Shape,
    pub shape2: Shape,
    pub geometry: ManifoldGeometry,
}

/// The contact generation of a batch of pairs.
pub trait ContactBatch {
    /// For each job of `jobs`, in order: whether its pair of shapes is supported, and its
    /// manifold's geometry as `ContactDispatcher::contact_manifold` updates it (at `prediction`).
    fn contact_geometries(
        prediction: Fixed, jobs: Span<ContactJob>,
    ) -> Span<(bool, ManifoldGeometry)>;
}

/// The batch in process: `D::contact_manifold` on each job (default solver data, which no
/// contact generator reads).
pub impl InProcessBatch<impl D: ContactDispatcher> of ContactBatch {
    fn contact_geometries(
        prediction: Fixed, jobs: Span<ContactJob>,
    ) -> Span<(bool, ManifoldGeometry)> {
        let mut out = array![];
        for job in jobs {
            let mut manifold = with_geometry(*job.geometry, Default::default());
            let supported = D::contact_manifold(
                *job.pos12, *job.shape1, *job.shape2, prediction, ref manifold,
            );
            out.append((supported, geometry(@manifold)));
        }
        out.span()
    }
}

/// The pair loop with the contact generation batched by `B` and the sensor pairs handled by `S`
/// (see the module documentation).
///
/// # Panics
/// As `PairLoopNarrowPhase`; `'Narrow phase: no composites'` when a pair with a composite shape
/// reaches the contact generation.
pub impl BatchedNarrowPhase<
    impl B: ContactBatch, impl S: IntersectionStrategy,
> of NarrowPhaseStage {
    #[inline(always)]
    fn compute_contacts(
        ref narrow_phase: NarrowPhase,
        prediction: Fixed,
        scratch: Span<PairCollider>,
        pairs: Span<(u32, u32)>,
        ref colliders: ColliderSet,
    ) -> Array<CollisionEvent> {
        compute_contacts_batched::<
            B, S,
        >(ref narrow_phase, prediction, scratch, pairs, ref colliders)
    }
}

/// The jobs of the pairs of `pairs` that reach the contact generation, in pair order (pass 1).
fn contact_jobs(
    previous: Span<ContactPair>, scratch: Span<PairCollider>, pairs: Span<(u32, u32)>,
) -> Array<ContactJob> {
    let mut jobs = array![];
    let mut cursor: u32 = 0;
    for (i, j) in pairs {
        let co1 = *scratch.at(*i);
        let co2 = *scratch.at(*j);
        if !(co1.solid && co2.solid) || pair_filtered(co1, co2) {
            continue;
        }
        let h1 = co1.handle;
        let h2 = co2.handle;
        let mut previous_geometry = geometry(@Default::default());
        while let Some(boxed) = previous.get(cursor) {
            let head = boxed.unbox();
            let a1 = *head.collider1;
            let a2 = *head.collider2;
            if key_before(a1, a2, h1, h2) {
                cursor += 1;
                continue;
            }
            if a1.index == h1.index && a2.index == h2.index {
                cursor += 1;
                if a1 == h1 && a2 == h2 && !(*head.event_status).is_intersection_pair() {
                    previous_geometry = geometry(head.manifold);
                }
            }
            break;
        }
        jobs
            .append(
                ContactJob {
                    pos12: pair_pose(co1, co2),
                    shape1: co1.shape,
                    shape2: co2.shape,
                    geometry: previous_geometry,
                },
            );
    }
    jobs
}

/// `compute_contacts_from_scratch_with` with the contact generation of every pair done by `B`
/// before the loop (see the module documentation).
fn compute_contacts_batched<impl B: ContactBatch, impl S: IntersectionStrategy>(
    ref narrow_phase: NarrowPhase,
    prediction: Fixed,
    scratch: Span<PairCollider>,
    pairs: Span<(u32, u32)>,
    ref colliders: ColliderSet,
) -> Array<CollisionEvent> {
    let previous = narrow_phase.pairs.span();
    let jobs = contact_jobs(previous, scratch, pairs);
    let mut results = if jobs.is_empty() {
        array![].span()
    } else {
        B::contact_geometries(prediction, jobs.span())
    };
    let fresh_pair = ContactPairTrait::new(Default::default(), Default::default());
    let fresh = BoxTrait::new(@fresh_pair);
    let mut cursor: u32 = 0;
    let mut current = array![];
    let mut events = array![];
    let mut transitions = array![];
    for (i, j) in pairs {
        let co1 = *scratch.at(*i);
        let co2 = *scratch.at(*j);
        if !(co1.solid && co2.solid) {
            if (co1.solid || co1.sensor) && (co2.solid || co2.sensor) {
                S::intersection_pair(
                    co1,
                    co2,
                    previous,
                    ref cursor,
                    ref colliders,
                    ref events,
                    ref transitions,
                    ref current,
                );
            }
            continue;
        }
        let h1 = co1.handle;
        let h2 = co2.handle;
        let mut found = fresh;
        while let Some(boxed) = previous.get(cursor) {
            let head = boxed.unbox();
            let a1 = *head.collider1;
            let a2 = *head.collider2;
            if key_before(a1, a2, h1, h2) {
                dropped_event(a1, a2, *head.event_status, ref colliders, ref events);
                cursor += 1;
                continue;
            }
            if a1.index == h1.index && a2.index == h2.index {
                cursor += 1;
                if a1 == h1 && a2 == h2 && !(*head.event_status).is_intersection_pair() {
                    found = boxed;
                } else {
                    dropped_event(a1, a2, *head.event_status, ref colliders, ref events);
                }
            }
            break;
        }
        let status = *found.unbox().event_status;
        let had_contact = *found.unbox().manifold.data.num_solver_contacts != 0;
        if pair_filtered(co1, co2) {
            let mut event_status = status;
            if had_contact && events_on(co1, co2) {
                event_status.bits = event_status.bits & 252;
                transitions.append(stopped(h1, h2, CollisionEventFlagsTrait::empty()));
            }
            current
                .append(
                    ContactPair {
                        collider1: h1, collider2: h2, manifold: Default::default(), event_status,
                    },
                );
            continue;
        }
        let (supported, new_geometry) = *results.pop_front().unwrap();
        let mut manifold = with_geometry(new_geometry, *found.unbox().manifold.data);
        if !supported {
            assert(!co1.shape.is_composite() && !co2.shape.is_composite(), COMPOSITE);
            manifold.num_points = 0;
        }
        let manifold = solver_data_supported(prediction, co1, co2, manifold);
        let has_contact = manifold.data.num_solver_contacts != 0;
        let mut event_status = status;
        if has_contact != had_contact && events_on(co1, co2) {
            if has_contact {
                event_status.bits = event_status.bits | START_EVENT_EMITTED.bits;
                transitions.append(started(h1, h2));
            } else {
                event_status.bits = event_status.bits & 252;
                transitions.append(stopped(h1, h2, CollisionEventFlagsTrait::empty()));
            }
        }
        current.append(ContactPair { collider1: h1, collider2: h2, manifold, event_status });
    }
    while let Some(boxed) = previous.get(cursor) {
        let head = boxed.unbox();
        dropped_event(
            *head.collider1, *head.collider2, *head.event_status, ref colliders, ref events,
        );
        cursor += 1;
    }
    events.append_span(transitions.span());
    narrow_phase.pairs = current;
    events
}
