//! The compact wires of the narrow phase's crossings (CX2): the caller's pair loop call
//! ([`super::LibraryCallNarrowPhase`]) and the family calls it makes.
//!
//! A wire is a flat run of felts, each holding three 64-bit *lanes* (`a + b · 2^64 + c · 2^128`):
//! a `Fixed` is its raw value plus `2^63`, two `u32` share a lane (`low + high · 2^32`), a
//! handle is `index + generation · 2^32`. Decoding is total: every felt decodes (to garbage when
//! it was not written by the matching encoder), so a wire carries no tag and no length but the
//! record counts. What crosses:
//!
//! * a pair collider: 5 felts (handle, parent, flags and events; both interaction groups,
//!   collision types and dominance; pose, parent's centre of mass, friction and restitution),
//!   then the shape by the basic codec and the one-way cone when there is one;
//! * a previous pair: 10 felts, its handles, event status, solver contact count, user data and
//!   manifold geometry: the pair loop rewrites every other solver-data field
//!   (`solver_data_supported`), so they stay in the caller;
//! * a contact job: 10 felts (the pose of shape 2 in shape 1's frame, the previous geometry),
//!   then both shapes by the basic codec; a result: the geometry and the `supported` flag,
//!   9 felts;
//! * a current pair: the handles, the whole manifold and the event status, 17 felts.
//!
//! [`Wire`] is the last argument of a declared class's entry point: its `Serde` takes the rest of
//! the calldata as it is (no copy).

#[feature("bounded-int-utils")]
use core::internal::bounded_int::{self, BoundedInt, DivRemHelper, SubHelper, UnitInt};
use rapier2d::pipeline::stages::narrow::{ContactJob, ManifoldGeometry};
use rapier2d::prelude::{Fixed, Handle, Pose2, Rot2, Shape, Vec2};
use rapier2d::world::basic_state::{deserialize_basic_shape, serialize_basic_shape};
use rapier_core::collider::{ActiveCollisionTypes, ActiveEvents, CoefficientCombineRule};
use rapier_core::interaction_groups::{Group, InteractionGroups, InteractionTestMode};
use rapier_core::rigid_body::RigidBodyType;
use rapier_dynamics2d::collider::components::OneWayPlatform;
use rapier_dynamics2d::events::PairEventStatus;
use rapier_dynamics2d::narrow_phase::{ContactPair, PairCollider};
use rapier_geometry2d::contact::{
    ContactData, ContactManifold, ContactManifoldData, SolverContact, SolverFlags, TrackedContact,
};
use rapier_geometry2d::feature_id::FeatureId;
use crate::hashes::errors;

/// The rest of a calldata, taken as it is by its `Serde` (the last argument of an entry point).
#[derive(Copy, Drop)]
pub struct Wire {
    pub felts: Span<felt252>,
}

/// Writes the felts; reading takes every remaining felt.
pub impl WireSerde of Serde<Wire> {
    fn serialize(self: @Wire, ref output: Array<felt252>) {
        output.append_span(*self.felts);
    }

    fn deserialize(ref serialized: Span<felt252>) -> Option<Wire> {
        let felts = serialized;
        serialized = array![].span();
        Some(Wire { felts })
    }
}

const B63: felt252 = 0x8000000000000000;
const B15: felt252 = 0x8000;
const T32: felt252 = 0x100000000;
const T64: felt252 = 0x10000000000000000;
const T128: felt252 = 0x100000000000000000000000000000000;
const U_B63: UnitInt<0x8000000000000000> = 0x8000000000000000;
const NZ_T64: NonZero<UnitInt<0x10000000000000000>> = 0x10000000000000000;
const NZ_T32: NonZero<UnitInt<0x100000000>> = 0x100000000;
const NZ_2: NonZero<u32> = 2;
const NZ_4: NonZero<u32> = 4;
const NZ_8: NonZero<u32> = 8;
const NZ_256: NonZero<u32> = 256;

/// A 64-bit lane.
pub type Lane = BoundedInt<0, 0xffffffffffffffff>;
type Word = BoundedInt<0, 0xffffffff>;

impl DivRemWide of DivRemHelper<u128, UnitInt<0x10000000000000000>> {
    type DivT = Lane;
    type RemT = Lane;
}

impl DivRemLane of DivRemHelper<Lane, UnitInt<0x100000000>> {
    type DivT = Word;
    type RemT = Word;
}

impl SubBias of SubHelper<Lane, UnitInt<0x8000000000000000>> {
    type Result = BoundedInt<-0x8000000000000000, 0x7fffffffffffffff>;
}

/// The lane of a `Fixed`.
#[inline(always)]
pub fn fixed_lane(value: Fixed) -> felt252 {
    value.raw.into() + B63
}

/// The `Fixed` of a lane.
#[inline(always)]
pub fn lane_fixed(lane: Lane) -> Fixed {
    Fixed { raw: bounded_int::upcast(bounded_int::sub(lane, U_B63)) }
}

/// The lane of two words.
#[inline(always)]
pub fn words_lane(low: u32, high: u32) -> felt252 {
    low.into() + high.into() * T32
}

/// The words of a lane, `(low, high)`.
#[inline(always)]
pub fn lane_words(lane: Lane) -> (u32, u32) {
    let (high, low) = bounded_int::div_rem(lane, NZ_T32);
    (bounded_int::upcast(low), bounded_int::upcast(high))
}

/// The lane of a handle.
#[inline(always)]
pub fn handle_lane(handle: Handle) -> felt252 {
    words_lane(handle.index, handle.generation)
}

#[inline(always)]
fn lane_handle(lane: Lane) -> Handle {
    let (index, generation) = lane_words(lane);
    Handle { index, generation }
}

/// Appends the felt of three lanes.
pub fn put(ref out: Array<felt252>, a: felt252, b: felt252, c: felt252) {
    out.append(a + b * T64 + c * T128);
}

/// The three lanes of the next felt.
///
/// # Panics
/// `errors::DECODE` when the wire is exhausted.
pub fn take(ref wire: Span<felt252>) -> (Lane, Lane, Lane) {
    let value: u256 = (*wire.pop_front().expect(errors::DECODE)).into();
    let (b, a) = bounded_int::div_rem(value.low, NZ_T64);
    let (_, c) = bounded_int::div_rem(value.high, NZ_T64);
    (a, b, c)
}

/// Three `Fixed` in one felt.
#[inline(always)]
fn put_fixed(ref out: Array<felt252>, a: Fixed, b: Fixed, c: Fixed) {
    put(ref out, fixed_lane(a), fixed_lane(b), fixed_lane(c));
}

#[inline(always)]
fn take_fixed(ref wire: Span<felt252>) -> (Fixed, Fixed, Fixed) {
    let (a, b, c) = take(ref wire);
    (lane_fixed(a), lane_fixed(b), lane_fixed(c))
}

fn small<T, +TryInto<u32, T>>(word: u32) -> T {
    word.try_into().expect(errors::DECODE)
}

// --- Manifold geometry -------------------------------------------------------------------------

fn put_point(ref out: Array<felt252>, point: TrackedContact) {
    put_fixed(ref out, point.local_p1.x, point.local_p1.y, point.local_p2.x);
    put_fixed(ref out, point.local_p2.y, point.dist, point.data.impulse);
    put_fixed(
        ref out,
        point.data.tangent_impulse,
        point.data.warmstart_impulse,
        point.data.warmstart_tangent_impulse,
    );
}

fn take_point(ref wire: Span<felt252>, fids: Lane) -> TrackedContact {
    let (p1x, p1y, p2x) = take_fixed(ref wire);
    let (p2y, dist, impulse) = take_fixed(ref wire);
    let (tangent_impulse, warmstart_impulse, warmstart_tangent_impulse) = take_fixed(ref wire);
    let (fid1, fid2) = lane_words(fids);
    TrackedContact {
        local_p1: Vec2 { x: p1x, y: p1y },
        local_p2: Vec2 { x: p2x, y: p2y },
        dist,
        fid1: FeatureId { packed: fid1 },
        fid2: FeatureId { packed: fid2 },
        data: ContactData {
            impulse, tangent_impulse, warmstart_impulse, warmstart_tangent_impulse,
        },
    }
}

/// A manifold's geometry in 9 felts, the last lane `extra` (a lane the record chooses).
pub fn put_geometry(ref out: Array<felt252>, geometry: @ManifoldGeometry, extra: felt252) {
    let [p, q] = *geometry.points;
    put(
        ref out,
        words_lane(*geometry.subshape1, *geometry.subshape2),
        words_lane(p.fid1.packed, p.fid2.packed),
        words_lane(q.fid1.packed, q.fid2.packed),
    );
    put_point(ref out, p);
    put_point(ref out, q);
    let n1 = *geometry.local_n1;
    let n2 = *geometry.local_n2;
    put_fixed(ref out, n1.x, n1.y, n2.x);
    put(ref out, fixed_lane(n2.y), (*geometry.num_points).into(), extra);
}

/// The geometry of [`put_geometry`] and its `extra` lane.
///
/// # Panics
/// `errors::DECODE` when the wire is exhausted or the point count is not a `u8`.
pub fn take_geometry(ref wire: Span<felt252>) -> (ManifoldGeometry, Lane) {
    let (subshapes, fids0, fids1) = take(ref wire);
    let p = take_point(ref wire, fids0);
    let q = take_point(ref wire, fids1);
    let (n1x, n1y, n2x) = take_fixed(ref wire);
    let (n2y, num_points, extra) = take(ref wire);
    let (subshape1, subshape2) = lane_words(subshapes);
    let num_points: u64 = bounded_int::upcast(num_points);
    (
        ManifoldGeometry {
            points: [p, q],
            num_points: num_points.try_into().expect(errors::DECODE),
            local_n1: Vec2 { x: n1x, y: n1y },
            local_n2: Vec2 { x: n2x, y: lane_fixed(n2y) },
            subshape1,
            subshape2,
        },
        extra,
    )
}

// --- Pair colliders (caller -> NarrowPhaseClass) -----------------------------------------------

#[inline(always)]
fn rule_code(rule: CoefficientCombineRule) -> u32 {
    match rule {
        CoefficientCombineRule::Average => 0,
        CoefficientCombineRule::Min => 1,
        CoefficientCombineRule::Multiply => 2,
        CoefficientCombineRule::Max => 3,
        CoefficientCombineRule::ClampedSum => 4,
        CoefficientCombineRule::GeometricMean => 5,
    }
}

#[inline(always)]
fn rule_of(code: u32) -> CoefficientCombineRule {
    match code {
        0 => CoefficientCombineRule::Average,
        1 => CoefficientCombineRule::Min,
        2 => CoefficientCombineRule::Multiply,
        3 => CoefficientCombineRule::Max,
        4 => CoefficientCombineRule::ClampedSum,
        _ => CoefficientCombineRule::GeometricMean,
    }
}

#[inline(always)]
fn type_code(body_type: RigidBodyType) -> u32 {
    match body_type {
        RigidBodyType::Dynamic => 0,
        RigidBodyType::Fixed => 1,
        RigidBodyType::KinematicPositionBased => 2,
        RigidBodyType::KinematicVelocityBased => 3,
    }
}

#[inline(always)]
fn type_of(code: u32) -> RigidBodyType {
    match code {
        0 => RigidBodyType::Dynamic,
        1 => RigidBodyType::Fixed,
        2 => RigidBodyType::KinematicPositionBased,
        _ => RigidBodyType::KinematicVelocityBased,
    }
}

#[inline(always)]
fn mode_code(mode: InteractionTestMode) -> u32 {
    match mode {
        InteractionTestMode::And => 0,
        InteractionTestMode::Or => 1,
    }
}

#[inline(always)]
fn mode_of(code: u32) -> InteractionTestMode {
    if code == 0 {
        InteractionTestMode::And
    } else {
        InteractionTestMode::Or
    }
}

#[inline(always)]
fn groups_lane(groups: InteractionGroups) -> felt252 {
    words_lane(groups.memberships.bits, groups.filter.bits)
}

#[inline(always)]
fn lane_groups(lane: Lane, mode: u32) -> InteractionGroups {
    let (memberships, filter) = lane_words(lane);
    InteractionGroups {
        memberships: Group { bits: memberships },
        filter: Group { bits: filter },
        test_mode: mode_of(mode),
    }
}

#[inline(always)]
fn bit(value: bool) -> u32 {
    if value {
        1
    } else {
        0
    }
}

/// Appends the record of a pair collider (see the module documentation).
///
/// # Panics
/// `'State: not a basic shape'` on another shape.
pub fn put_collider(ref out: Array<felt252>, collider: @PairCollider) {
    let co = *collider;
    let one_way = co.one_way.unbox();
    let (body, has_body) = match co.body {
        Some(body) => (handle_lane(body), 1),
        None => (0, 0),
    };
    let flags = bit(co.solid)
        + bit(co.sensor) * 2
        + has_body * 4
        + bit(one_way.is_some()) * 8
        + mode_code(co.collision_groups.test_mode) * 16
        + mode_code(co.solver_groups.test_mode) * 32
        + type_code(co.body_type) * 64
        + rule_code(co.friction_combine_rule) * 256
        + rule_code(co.restitution_combine_rule) * 2048;
    put(ref out, handle_lane(co.handle), body, words_lane(flags, co.active_events.bits));
    put(
        ref out,
        groups_lane(co.collision_groups),
        groups_lane(co.solver_groups),
        co.active_collision_types.bits.into() + (co.dominance.into() + B15) * T32,
    );
    put_fixed(ref out, co.pose.translation.x, co.pose.translation.y, co.pose.rotation.re);
    put_fixed(ref out, co.pose.rotation.im, co.world_com.x, co.world_com.y);
    put(ref out, fixed_lane(co.friction), fixed_lane(co.restitution), 0);
    serialize_basic_shape(@co.shape, ref out);
    if let Some(platform) = one_way {
        platform.serialize(ref out);
    }
}

/// The pair collider of [`put_collider`].
///
/// # Panics
/// `errors::DECODE` when the wire is exhausted or malformed; `'State: not a basic shape'` on
/// another shape tag.
pub fn take_collider(ref wire: Span<felt252>) -> PairCollider {
    let (handle, body, flags_events) = take(ref wire);
    let (flags, events) = lane_words(flags_events);
    let (flags, solid) = DivRem::div_rem(flags, NZ_2);
    let (flags, sensor) = DivRem::div_rem(flags, NZ_2);
    let (flags, has_body) = DivRem::div_rem(flags, NZ_2);
    let (flags, has_one_way) = DivRem::div_rem(flags, NZ_2);
    let (flags, collision_mode) = DivRem::div_rem(flags, NZ_2);
    let (flags, solver_mode) = DivRem::div_rem(flags, NZ_2);
    let (flags, body_type) = DivRem::div_rem(flags, NZ_4);
    let (restitution_rule, friction_rule) = DivRem::div_rem(flags, NZ_8);
    let (collision, solver, types_dominance) = take(ref wire);
    let (types, dominance) = lane_words(types_dominance);
    let dominance: felt252 = dominance.into() - B15;
    let (tx, ty, re) = take_fixed(ref wire);
    let (im, comx, comy) = take_fixed(ref wire);
    let (friction, restitution, _) = take_fixed(ref wire);
    let shape: Shape = deserialize_basic_shape(ref wire).expect(errors::DECODE);
    let one_way: Option<OneWayPlatform> = if has_one_way == 0 {
        None
    } else {
        Some(Serde::deserialize(ref wire).expect(errors::DECODE))
    };
    PairCollider {
        handle: lane_handle(handle),
        solid: solid != 0,
        sensor: sensor != 0,
        shape,
        pose: Pose2 { translation: Vec2 { x: tx, y: ty }, rotation: Rot2 { re, im } },
        friction,
        restitution,
        friction_combine_rule: rule_of(friction_rule),
        restitution_combine_rule: rule_of(restitution_rule),
        active_collision_types: ActiveCollisionTypes { bits: small(types) },
        collision_groups: lane_groups(collision, collision_mode),
        solver_groups: lane_groups(solver, solver_mode),
        active_events: ActiveEvents { bits: events },
        one_way: BoxTrait::new(one_way),
        body: if has_body == 0 {
            None
        } else {
            Some(lane_handle(body))
        },
        body_type: type_of(body_type),
        world_com: Vec2 { x: comx, y: comy },
        dominance: dominance.try_into().expect(errors::DECODE),
    }
}

// --- Previous pairs (caller -> NarrowPhaseClass) -----------------------------------------------

/// Appends the record of a previous pair: all the pair loop reads of it (see the module
/// documentation).
pub fn put_previous(ref out: Array<felt252>, pair: @ContactPair) {
    let data = pair.manifold.data;
    put(
        ref out,
        handle_lane(*pair.collider1),
        handle_lane(*pair.collider2),
        words_lane(
            (*pair.event_status.bits).into() + (*data.num_solver_contacts).into() * 256,
            *data.user_data,
        ),
    );
    put_geometry(ref out, @crate::contact::geometry(pair.manifold), 0);
}

/// The previous pair of [`put_previous`]: its solver data is the default one but the solver
/// contact count and the user data (the pair loop reads nothing else of it).
///
/// # Panics
/// `errors::DECODE` when the wire is exhausted or malformed.
pub fn take_previous(ref wire: Span<felt252>) -> ContactPair {
    let (collider1, collider2, status) = take(ref wire);
    let (status, user_data) = lane_words(status);
    let (count, status) = DivRem::div_rem(status, NZ_256);
    let (geometry, _) = take_geometry(ref wire);
    let mut data: ContactManifoldData = Default::default();
    data.num_solver_contacts = small(count);
    data.user_data = user_data;
    ContactPair {
        collider1: lane_handle(collider1),
        collider2: lane_handle(collider2),
        manifold: crate::contact::with_geometry(geometry, data),
        event_status: PairEventStatus { bits: small(status) },
    }
}

// --- Contact jobs and results (NarrowPhaseClass <-> family classes) ----------------------------

/// Appends the record of a contact job.
///
/// # Panics
/// `'State: not a basic shape'` on another shape.
pub fn put_job(ref out: Array<felt252>, job: @ContactJob) {
    let pos12 = *job.pos12;
    put_fixed(ref out, pos12.translation.x, pos12.translation.y, pos12.rotation.re);
    put_geometry(ref out, job.geometry, fixed_lane(pos12.rotation.im));
    serialize_basic_shape(job.shape1, ref out);
    serialize_basic_shape(job.shape2, ref out);
}

/// The contact job of [`put_job`].
///
/// # Panics
/// `errors::DECODE` when the wire is exhausted or malformed; `'State: not a basic shape'` on
/// another shape tag.
pub fn take_job(ref wire: Span<felt252>) -> ContactJob {
    let (tx, ty, re) = take_fixed(ref wire);
    let (geometry, im) = take_geometry(ref wire);
    let shape1 = deserialize_basic_shape(ref wire).expect(errors::DECODE);
    let shape2 = deserialize_basic_shape(ref wire).expect(errors::DECODE);
    ContactJob {
        pos12: Pose2 {
            translation: Vec2 { x: tx, y: ty }, rotation: Rot2 { re, im: lane_fixed(im) },
        },
        shape1,
        shape2,
        geometry,
    }
}

/// Appends the record of a result.
pub fn put_result(ref out: Array<felt252>, supported: bool, geometry: @ManifoldGeometry) {
    put_geometry(ref out, geometry, bit(supported).into());
}

/// The result of [`put_result`].
pub fn take_result(ref wire: Span<felt252>) -> (bool, ManifoldGeometry) {
    let (geometry, supported) = take_geometry(ref wire);
    let supported: u64 = bounded_int::upcast(supported);
    (supported != 0, geometry)
}

// --- Current pairs (NarrowPhaseClass -> caller) ------------------------------------------------

#[inline(always)]
fn body_lane(body: Option<Handle>) -> (felt252, u32) {
    match body {
        Some(handle) => (handle_lane(handle), 1),
        None => (0, 0),
    }
}

#[inline(always)]
fn lane_body(lane: Lane, some: u32) -> Option<Handle> {
    if some == 0 {
        None
    } else {
        Some(lane_handle(lane))
    }
}

/// Appends the record of a new pair: the whole pair.
pub fn put_current(ref out: Array<felt252>, pair: @ContactPair) {
    let data = *pair.manifold.data;
    let (body1, some1) = body_lane(data.rigid_body1);
    let (body2, some2) = body_lane(data.rigid_body2);
    let status = (*pair.event_status.bits).into()
        + data.num_solver_contacts.into() * 256
        + some1 * 65536
        + some2 * 131072;
    put(
        ref out,
        handle_lane(*pair.collider1),
        handle_lane(*pair.collider2),
        words_lane(status, data.user_data),
    );
    let dominance: felt252 = data.relative_dominance.into() + B15;
    put_geometry(
        ref out,
        @crate::contact::geometry(pair.manifold),
        data.solver_flags.bits.into() + dominance * T32,
    );
    put(ref out, body1, body2, fixed_lane(data.normal.x));
    put_fixed(ref out, data.normal.y, data.friction, data.restitution);
    let [c, d] = data.solver_contacts;
    put_fixed(ref out, c.anchor1.x, c.anchor1.y, c.anchor2.x);
    put_fixed(ref out, c.anchor2.y, c.dist, c.tangent_velocity.x);
    put_fixed(ref out, d.anchor1.x, d.anchor1.y, d.anchor2.x);
    put_fixed(ref out, d.anchor2.y, d.dist, d.tangent_velocity.x);
    put(
        ref out,
        fixed_lane(c.tangent_velocity.y),
        fixed_lane(d.tangent_velocity.y),
        words_lane(c.contact_id, d.contact_id),
    );
}

/// The pair of [`put_current`].
///
/// # Panics
/// `errors::DECODE` when the wire is exhausted or malformed.
pub fn take_current(ref wire: Span<felt252>) -> ContactPair {
    let (collider1, collider2, status) = take(ref wire);
    let (status, user_data) = lane_words(status);
    let (rest, status) = DivRem::div_rem(status, NZ_256);
    let (bodies, count) = DivRem::div_rem(rest, NZ_256);
    let (some2, some1) = DivRem::div_rem(bodies, NZ_2);
    let (geometry, flags_dominance) = take_geometry(ref wire);
    let (flags, dominance) = lane_words(flags_dominance);
    let dominance: felt252 = dominance.into() - B15;
    let (body1, body2, nx) = take(ref wire);
    let (ny, friction, restitution) = take_fixed(ref wire);
    let (c1x, c1y, c2x) = take_fixed(ref wire);
    let (c2y, cdist, ctx) = take_fixed(ref wire);
    let (d1x, d1y, d2x) = take_fixed(ref wire);
    let (d2y, ddist, dtx) = take_fixed(ref wire);
    let (cty, dty, ids) = take(ref wire);
    let (cid, did) = lane_words(ids);
    let data = ContactManifoldData {
        rigid_body1: lane_body(body1, some1),
        rigid_body2: lane_body(body2, some2),
        solver_flags: SolverFlags { bits: flags },
        normal: Vec2 { x: lane_fixed(nx), y: ny },
        solver_contacts: [
            SolverContact {
                anchor1: Vec2 { x: c1x, y: c1y },
                anchor2: Vec2 { x: c2x, y: c2y },
                dist: cdist,
                tangent_velocity: Vec2 { x: ctx, y: lane_fixed(cty) },
                contact_id: cid,
            },
            SolverContact {
                anchor1: Vec2 { x: d1x, y: d1y },
                anchor2: Vec2 { x: d2x, y: d2y },
                dist: ddist,
                tangent_velocity: Vec2 { x: dtx, y: lane_fixed(dty) },
                contact_id: did,
            },
        ],
        num_solver_contacts: small(count),
        relative_dominance: dominance.try_into().expect(errors::DECODE),
        user_data,
        friction,
        restitution,
    };
    let manifold: ContactManifold = crate::contact::with_geometry(geometry, data);
    ContactPair {
        collider1: lane_handle(collider1),
        collider2: lane_handle(collider2),
        manifold,
        event_status: PairEventStatus { bits: small(status) },
    }
}

#[cfg(test)]
mod tests {
    use rapier2d::pipeline::stages::narrow::{ContactJob, ManifoldGeometry};
    use rapier2d::prelude::{Fixed, Handle, Pose2, Rot2, Shape, Vec2};
    use rapier_core::collider::{ActiveCollisionTypes, ActiveEvents, CoefficientCombineRule};
    use rapier_core::interaction_groups::{Group, InteractionGroups, InteractionTestMode};
    use rapier_core::rigid_body::RigidBodyType;
    use rapier_dynamics2d::collider::components::OneWayPlatform;
    use rapier_dynamics2d::events::PairEventStatus;
    use rapier_dynamics2d::narrow_phase::{ContactPair, PairCollider};
    use rapier_geometry2d::contact::{
        ContactData, ContactManifold, ContactManifoldData, SolverContact, SolverFlags,
        TrackedContact,
    };
    use rapier_geometry2d::feature_id::FeatureId;
    use rapier_geometry2d::shape::{Ball, Cuboid, HalfSpace};
    use super::{
        put_collider, put_current, put_job, put_previous, put_result, take_collider, take_current,
        take_job, take_previous, take_result,
    };

    const MIN: i64 = -0x8000000000000000;
    const MAX: i64 = 0x7fffffffffffffff;

    fn f(raw: i64) -> Fixed {
        Fixed { raw }
    }

    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: f(x), y: f(y) }
    }

    fn point(seed: i64) -> TrackedContact {
        TrackedContact {
            local_p1: v(seed, -seed),
            local_p2: v(MIN, MAX),
            dist: f(-3 * seed),
            fid1: FeatureId { packed: 0xffffffff },
            fid2: FeatureId { packed: 7 },
            data: ContactData {
                impulse: f(seed + 1),
                tangent_impulse: f(-seed - 2),
                warmstart_impulse: f(MAX - seed),
                warmstart_tangent_impulse: f(MIN + seed),
            },
        }
    }

    fn contact(seed: i64, id: u32) -> SolverContact {
        SolverContact {
            anchor1: v(seed, MIN),
            anchor2: v(-seed, MAX),
            dist: f(seed * 5),
            tangent_velocity: v(-7, seed),
            contact_id: id,
        }
    }

    fn pairs() -> Array<ContactPair> {
        let manifold = ContactManifold {
            points: [point(12345), point(987654321)],
            num_points: 2,
            local_n1: v(MAX, -1),
            local_n2: v(MIN, 1),
            subshape1: 0xffffffff,
            subshape2: 3,
            data: ContactManifoldData {
                rigid_body1: Some(Handle { index: 0xffffffff, generation: 0xffffffff }),
                rigid_body2: None,
                solver_flags: SolverFlags { bits: 0xffffffff },
                normal: v(-4, 4),
                solver_contacts: [contact(99, 0x80000001), contact(-98, 0xffffffff)],
                num_solver_contacts: 255,
                relative_dominance: -32768,
                user_data: 0xfffffffe,
                friction: f(MIN),
                restitution: f(MAX),
            },
        };
        let mut other = manifold;
        other.data.rigid_body1 = None;
        other.data.rigid_body2 = Some(Handle { index: 3, generation: 0 });
        other.data.relative_dominance = 32767;
        other.data.num_solver_contacts = 0;
        other.num_points = 0;
        array![
            ContactPair {
                collider1: Handle { index: 0, generation: 0xffffffff },
                collider2: Handle { index: 0xffffffff, generation: 1 },
                manifold,
                event_status: PairEventStatus { bits: 255 },
            },
            ContactPair {
                collider1: Handle { index: 5, generation: 6 },
                collider2: Handle { index: 7, generation: 8 },
                manifold: other,
                event_status: PairEventStatus { bits: 0 },
            },
            ContactPair {
                collider1: Default::default(),
                collider2: Default::default(),
                manifold: Default::default(),
                event_status: Default::default(),
            },
        ]
    }

    fn colliders() -> Array<PairCollider> {
        let groups = InteractionGroups {
            memberships: Group { bits: 0xffffffff },
            filter: Group { bits: 0x80000000 },
            test_mode: InteractionTestMode::Or,
        };
        let a = PairCollider {
            handle: Handle { index: 0xffffffff, generation: 0xffffffff },
            solid: true,
            sensor: false,
            shape: Shape::Cuboid(Cuboid { half_extents: v(MAX, 1) }),
            pose: Pose2 { translation: v(MIN, -5), rotation: Rot2 { re: f(MAX), im: f(-1) } },
            friction: f(-2),
            restitution: f(MAX),
            friction_combine_rule: CoefficientCombineRule::GeometricMean,
            restitution_combine_rule: CoefficientCombineRule::ClampedSum,
            active_collision_types: ActiveCollisionTypes { bits: 0xffff },
            collision_groups: groups,
            solver_groups: Default::default(),
            active_events: ActiveEvents { bits: 0xffffffff },
            one_way: BoxTrait::new(Some(OneWayPlatform { local_up: v(0, 1), cos_allowed_angle: f(-9) })),
            body: Some(Handle { index: 0xffffffff, generation: 0 }),
            body_type: RigidBodyType::KinematicVelocityBased,
            world_com: v(MIN, MAX),
            dominance: -32768,
        };
        let mut b = a;
        b.solid = false;
        b.sensor = true;
        b.shape = Shape::Ball(Ball { radius: f(3) });
        b.friction_combine_rule = CoefficientCombineRule::Average;
        b.restitution_combine_rule = CoefficientCombineRule::Max;
        b.collision_groups = Default::default();
        b.solver_groups = groups;
        b.one_way = BoxTrait::new(None);
        b.body = None;
        b.body_type = RigidBodyType::Dynamic;
        b.dominance = 32767;
        let mut c = b;
        c.shape = Shape::HalfSpace(HalfSpace { normal: v(0, -1) });
        c.body_type = RigidBodyType::Fixed;
        c.friction_combine_rule = CoefficientCombineRule::Min;
        c.restitution_combine_rule = CoefficientCombineRule::Multiply;
        c.dominance = 0;
        let mut d = c;
        d.body_type = RigidBodyType::KinematicPositionBased;
        array![a, b, c, d]
    }

    fn felts<T, +Serde<T>, +Drop<T>>(value: @T) -> Array<felt252> {
        let mut out = array![];
        value.serialize(ref out);
        out
    }

    #[test]
    fn test_current_round_trip() {
        let mut wire = array![];
        for pair in pairs().span() {
            put_current(ref wire, pair);
        }
        assert_eq!(wire.len(), 3 * 17);
        let mut wire = wire.span();
        for pair in pairs() {
            assert_eq!(take_current(ref wire), pair);
        }
        assert!(wire.is_empty());
    }

    #[test]
    fn test_previous_round_trip() {
        let mut wire = array![];
        for pair in pairs().span() {
            put_previous(ref wire, pair);
        }
        assert_eq!(wire.len(), 3 * 10);
        let mut wire = wire.span();
        for pair in pairs() {
            let mut expected = pair;
            let mut data: ContactManifoldData = Default::default();
            data.num_solver_contacts = pair.manifold.data.num_solver_contacts;
            data.user_data = pair.manifold.data.user_data;
            expected.manifold.data = data;
            assert_eq!(take_previous(ref wire), expected);
        }
        assert!(wire.is_empty());
    }

    #[test]
    fn test_collider_round_trip() {
        let mut wire = array![];
        for collider in colliders().span() {
            put_collider(ref wire, collider);
        }
        let mut wire = wire.span();
        for collider in colliders() {
            assert_eq!(take_collider(ref wire), collider);
        }
        assert!(wire.is_empty());
    }

    #[test]
    fn test_job_and_result_round_trip() {
        let shapes = array![
            Shape::Ball(Ball { radius: f(MAX) }), Shape::Cuboid(Cuboid { half_extents: v(1, 2) }),
        ];
        for pair in pairs() {
            let geometry: ManifoldGeometry = crate::contact::geometry(@pair.manifold);
            let job = ContactJob {
                pos12: Pose2 { translation: v(MIN, MAX), rotation: Rot2 { re: f(-1), im: f(MIN) } },
                shape1: *shapes[0],
                shape2: *shapes[1],
                geometry,
            };
            let mut wire = array![];
            put_job(ref wire, @job);
            put_result(ref wire, true, @geometry);
            put_result(ref wire, false, @geometry);
            let mut wire = wire.span();
            assert_eq!(felts(@take_job(ref wire)), felts(@job));
            let (supported, got) = take_result(ref wire);
            assert!(supported);
            assert_eq!(felts(@got), felts(@geometry));
            let (supported, got) = take_result(ref wire);
            assert!(!supported);
            assert_eq!(felts(@got), felts(@geometry));
            assert!(wire.is_empty());
        }
    }
}
