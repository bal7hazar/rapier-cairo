//! Sleeping (work package SL): per-step islands, the sleep and wake-up decisions and the sleep
//! timer (upstream `dynamics/island_manager/*`, `RigidBodyActivation::update_energy`,
//! `RigidBodyMassProps::recompute_max_extent`, `geometry::relative_pose_drift`).
//!
//! Upstream maintains persistent islands (eager merges, deferred splits, a single awake island).
//! Here an island is rebuilt every step (D9: only each body's `RigidBodyActivation` persists):
//! the connected components, through `rapier_core`'s [`UnionFind`], of the enabled non-fixed
//! bodies linked by a touching contact pair or an enabled joint; a fixed, disabled or missing
//! body never connects two islands. [`update_islands`] then applies upstream's two rules to
//! every island that has an awake member, walking the bodies in ascending slot:
//!
//! * **wake-up** — an island with both sleeping and awake members (a *mixed* island) wakes up
//!   strongly as a whole (`wake_up(true)` on every member: sleeping flags cleared, timers
//!   reset). This covers upstream's wake-up points inside the step in one rule: a contact that
//!   starts touching a sleeping body (`strong_wake_sleeping_side`), a joint linking it to an
//!   awake body (`wake_for_link`), a user wake-up of one body of a sleeping island (`RigidBody::
//!   wake_up` then `IslandManager::wake_up`, which strong-wakes the whole sleeping island, the
//!   target included), since an awake body touching or jointed to a sleeping one is exactly a
//!   mixed island. Upstream leaves the timer of an already awake body that starts touching a
//!   sleeping island alone (`wake_up(false)`); such a body is moving, its timer is zero anyway;
//! * **sleep** — when every member is eligible (`time_since_can_sleep >= time_until_sleep`, a
//!   just-woken member counting with a zero timer), every member goes to sleep (`RigidBody::
//!   sleep`: flag set, timer pinned, velocities zeroed, pose kept), before the solver runs, as
//!   upstream's `update_islands` does.
//!
//! The union-find is skipped when nothing can change: no awake member, or no eligible awake
//! member and no touching pair or enabled joint linking an awake member to a sleeping one (BT2:
//! then no island is mixed; [`SleepCensus`], counted by the user-changes walk that has
//! every body in hand: a walk of its own costs about 10k gas per body, the bodies being copied
//! out of their span). The slow path is an `#[inline(never)]` call behind an
//! `if` (shipped) rather than a one-iteration `while` (metered call, `alternatives::
//! update_islands_metered`): measured on `cuboid_stack(3)`, the metered form costs 7k more on
//! the fast path and 11k more on the slow one (`tests::gas_islands_*`); the dicts are runtime
//! costs, not part of the statically charged path.
//!
//! The timer ([`update_sleep_timer`]) is upstream's `update_energy`, run at the end of the step
//! on every body the step moved, with the velocities the solver left and the displacement of
//! the step (`position` before the update, `next_position` after): upstream runs it at the start
//! of the next step on the same velocities and on the pose difference with the pose it stored
//! one step earlier (`sleep_prev_pose`), so the timers are identical at the moment of the sleep
//! decision and no previous pose is persisted. The displacement rate is upstream's
//! [`relative_pose_drift`]: the translation length plus the rotation chord `2 sin(Δθ/2) ·
//! max_extent`, with [`max_extent`] the farthest collider point from the local centre of mass
//! (bounding spheres, upstream `recompute_max_extent`), recomputed with the mass properties from
//! the colliders.
//!
//! Cost of the timer (lot SC, `update_sleep_timer`, per moving body, Sierra gas | Cairo steps,
//! `timer_tests::gas_timer_*`): a fast-moving dynamic body (the translation of the step is at
//! least twice the default limit) costs 10.4k | 71 instead of 18.2k | 146; a body at rest
//! without turn 18.9k | 160 instead of 18.6k | 150, one that turns 38.6k | 343 instead of 35.3k |
//! 301. The verdicts are the reference's, only the arithmetic differs: `floor(sqrt(x² + y²)) < k`
//! is `x² + y² < k²` (a comparison of squares in `felt252`, one `u128` conversion), the angular
//! gate compares the wide square of the angular velocity, and the default configuration (the
//! default threshold, `length_unit = 1`, `dt = 1 / 60`) has its limit as a constant. What is not
//! a plain "moving" or "cannot sleep" verdict is behind a one-iteration `while`, so that the
//! bodies that do not need it are not charged for it. Candidates: `alternatives`.
//!
//! Divergences from upstream's persistent islands, all documented in the PR report:
//! * a body put to sleep by hand (`RigidBody::sleep`) in an island that has an awake member is
//!   woken up at the next step (upstream keeps it flagged asleep inside the awake island);
//! * the wake-up of a modified collider's contact partners happens at the next step's user
//!   changes (`pipeline::user_changes`), not inside the narrow phase, with the same result;
//! * kinematic bodies are island members and sleep when both velocities are exactly zero, as
//!   upstream's `update_energy`; a moving kinematic body keeps its island awake;
//! * a half-space collider on a dynamic body gives `max_extent = fixed::MAX` and the body never
//!   sleeps (upstream: an infinite extent, the same outcome through `inf`/`NaN` arithmetic);
//! * rounding: the translation length and the chord's square root floor, the chord's division
//!   rounds to nearest (`Fixed` semantics); upstream works in `f64`.
//!
//! Determinism: bodies, pairs and joints are walked in ascending slot / pair order; the dicts
//! (`UnionFind`, per-root flags) are only read at known keys, never iterated.

use core::dict::{Felt252Dict, Felt252DictTrait};
use fixed::{Fixed, FixedTrait, HALF, MAX, ONE, TWO, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_core::Handle;
use rapier_core::data::union_find::{UnionFind, UnionFindTrait};
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_core::rigid_body::activation::DEFAULT_NORMALIZED_LINEAR_THRESHOLD;
use rapier_core::rigid_body::{RigidBodyActivation, RigidBodyActivationTrait, RigidBodyType};
use rapier_dynamics2d::joint::{ImpulseJoint, JointEnabled};
use rapier_dynamics2d::narrow_phase::ContactPair;
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet, RigidBodySetTrait, RigidBodyTrait};
use rapier_geometry2d::shape::Shape;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::{Rot2, Rot2Trait};
use super::ordering::{BODY_AWAKE, BODY_SLEEPING, body_status, link_status};

#[cfg(test)]
pub(crate) mod alternatives;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod timer_tests;

/// `1e-6`, upstream's guard on `2 (1 + cos Δθ)` before the square root in the rotation chord.
const CHORD_EPSILON: Fixed = Fixed { raw: 4295 };

/// The default step (`IntegrationParameters::default().dt`, `1 / 60`) and the linear limit it
/// gives to the default threshold at `length_unit = 1`: `0.05 · dt`, floored twice.
const DEFAULT_DT: Fixed = Fixed { raw: 71582788 };
const DEFAULT_LIMIT: Fixed = Fixed { raw: 3579139 };

/// Whether a body takes part in islands: enabled and not fixed (upstream `ensure_body`).
#[inline(always)]
pub(crate) fn is_member(body: @RigidBody) -> bool {
    *body.enabled && *body.body_type != RigidBodyType::Fixed
}

/// What [`update_islands`] needs to know before building anything: how many island members
/// sleep, how many are awake, and whether an awake one is eligible for sleep.
#[derive(Copy, Drop, PartialEq, Debug, Default)]
pub struct SleepCensus {
    pub sleeping: u32,
    pub awake: u32,
    pub eligible: bool,
}

#[generate_trait]
pub impl SleepCensusImpl of SleepCensusTrait {
    /// Counts `body` (any body: non-members are ignored).
    #[inline(always)]
    fn count(ref self: SleepCensus, body: @RigidBody) {
        if is_member(body) {
            if *body.activation.sleeping {
                self.sleeping += 1;
            } else {
                self.awake += 1;
                if body.activation.is_eligible_for_sleep() {
                    self.eligible = true;
                }
            }
        }
    }

    /// The census of `entries` (a walk of its own; the fused step counts while it walks).
    fn taken(entries: Span<(Handle, RigidBody)>) -> SleepCensus {
        let mut census: SleepCensus = Default::default();
        for (_, body) in entries {
            census.count(body);
        }
        census
    }
}

/// Both bodies of a link are members: the link connects their islands.
#[inline(always)]
fn links(entries: Span<(Handle, RigidBody)>, body1: Option<Handle>, body2: Option<Handle>) -> bool {
    match (body1, body2) {
        (Some(h1), Some(h2)) => body_status(entries, h1) >= BODY_SLEEPING
            && body_status(entries, h2) >= BODY_SLEEPING,
        _ => false,
    }
}

/// The sleep and wake-up decisions of one step (see the module documentation), after the narrow
/// phase and before the solver. `pairs` are the step's contact pairs (dormant pairs of sleeping
/// bodies included, see `pipeline::sleeping`), `joints` every impulse joint, `entries` every
/// body in ascending slot as `user_changes_bodies` left them, `census` their [`SleepCensus`].
/// Every body whose activation or velocities change is written to `bodies`; returns the entries
/// with those bodies updated (the input span when nothing changed), whether any member body
/// sleeps after the update and whether a body was woken up (then the dormant pairs of the woken
/// bodies are the caller's to revive, `pipeline::step`). `dormant` are the pairs
/// `pipeline::sleeping::split_dormant` took out of `pairs`: they link the sleeping islands.
pub fn update_islands(
    ref bodies: RigidBodySet,
    pairs: Span<ContactPair>,
    dormant: Span<ContactPair>,
    joints: Span<(Handle, ImpulseJoint)>,
    entries: Span<(Handle, RigidBody)>,
    census: SleepCensus,
) -> (Span<(Handle, RigidBody)>, bool, bool) {
    // Nothing can change without an awake member, nor without an eligible awake member or a
    // mixed island (BT2: an island with an awake and a sleeping member has a link between an
    // awake and a sleeping member; dormant pairs never link an awake body).
    if census.awake != 0
        && (census.eligible
            || (census.sleeping != 0 && links_awake_to_sleeping(pairs, joints, entries))) {
        update_islands_slow(ref bodies, pairs, dormant, joints, entries)
    } else {
        (entries, census.sleeping != 0, false)
    }
}

/// Whether a touching pair of `pairs` or an enabled joint of `joints` links an awake member to
/// a sleeping one (their statuses in `entries`, ascending slot, possibly sparse: every body the
/// links reference). Without such a link no island is mixed, so the wake-up rule has nothing to
/// do (BT2: the union-find is skipped). Out of line: the fast path of [`update_islands`] stays
/// loop-free.
#[inline(never)]
pub(crate) fn links_awake_to_sleeping(
    pairs: Span<ContactPair>,
    joints: Span<(Handle, ImpulseJoint)>,
    entries: Span<(Handle, RigidBody)>,
) -> bool {
    for pair in pairs {
        if *pair.manifold.data.num_solver_contacts != 0 {
            let (s1, s2) = link_status(
                entries, *pair.manifold.data.rigid_body1, *pair.manifold.data.rigid_body2,
            );
            if crosses(s1, s2) {
                return true;
            }
        }
    }
    for (_, joint) in joints {
        if *joint.data.enabled == JointEnabled::Enabled {
            let (s1, s2) = link_status(entries, Some(*joint.body1), Some(*joint.body2));
            if crosses(s1, s2) {
                return true;
            }
        }
    }
    false
}

/// One side awake, the other asleep.
#[inline(always)]
fn crosses(s1: u8, s2: u8) -> bool {
    (s1 == BODY_AWAKE && s2 == BODY_SLEEPING) || (s1 == BODY_SLEEPING && s2 == BODY_AWAKE)
}

/// [`update_islands`] once the fast checks passed: the union-find and the two walks.
#[inline(never)]
pub(crate) fn update_islands_slow(
    ref bodies: RigidBodySet,
    pairs: Span<ContactPair>,
    dormant: Span<ContactPair>,
    joints: Span<(Handle, ImpulseJoint)>,
    entries: Span<(Handle, RigidBody)>,
) -> (Span<(Handle, RigidBody)>, bool, bool) {
    let mut forest: UnionFind = Default::default();
    link_pairs(ref forest, pairs, entries);
    link_pairs(ref forest, dormant, entries);
    for (_, joint) in joints {
        if *joint.data.enabled == JointEnabled::Enabled
            && links(entries, Some(*joint.body1), Some(*joint.body2)) {
            let _ = forest.union(*joint.body1.index, *joint.body2.index);
        }
    }
    // Per island (keyed by its root): has it an awake member, a sleeping member, an awake member
    // that is not eligible, a member whose `time_until_sleep` is positive (a strong wake-up
    // leaves it ineligible)?
    let mut has_awake: Felt252Dict<bool> = Default::default();
    let mut has_sleeping: Felt252Dict<bool> = Default::default();
    let mut blocked: Felt252Dict<bool> = Default::default();
    let mut positive_ttl: Felt252Dict<bool> = Default::default();
    let mut roots = array![];
    for (handle, body) in entries {
        let mut root = 0;
        if is_member(body) {
            root = forest.find(*handle.index);
            let key: felt252 = root.into();
            if *body.activation.sleeping {
                if !has_sleeping.get(key) {
                    has_sleeping.insert(key, true);
                }
            } else {
                if !has_awake.get(key) {
                    has_awake.insert(key, true);
                }
                if !body.activation.is_eligible_for_sleep() && !blocked.get(key) {
                    blocked.insert(key, true);
                }
            }
            if *body.activation.time_until_sleep > ZERO && !positive_ttl.get(key) {
                positive_ttl.insert(key, true);
            }
        }
        roots.append(root);
    }
    let mut roots = roots.span();
    let mut out = array![];
    let mut rebuilt = false;
    let mut position: u32 = 0;
    let mut asleep_after: u32 = 0;
    let mut woken = false;
    for entry in entries {
        let (handle, body) = entry;
        let key: felt252 = (*roots.pop_front().unwrap()).into();
        if is_member(body) && has_awake.get(key) {
            let mut body = *body;
            let mut changed = false;
            let mixed = has_sleeping.get(key);
            if mixed {
                if body.activation.sleeping {
                    woken = true;
                }
                if body.activation.sleeping || body.activation.time_since_can_sleep != ZERO {
                    body.wake_up(true);
                    changed = true;
                }
            }
            let sleeps = if mixed {
                !positive_ttl.get(key)
            } else {
                !blocked.get(key)
            };
            if sleeps {
                body.sleep();
                changed = true;
            }
            if changed {
                let _ = bodies.set(*handle, body);
                if !rebuilt {
                    out.append_span(entries.slice(0, position));
                    rebuilt = true;
                }
                out.append((*handle, body));
            } else if rebuilt {
                out.append(*entry);
            }
            if body.activation.sleeping {
                asleep_after += 1;
            }
        } else {
            if rebuilt {
                out.append(*entry);
            }
            if is_member(body) && *body.activation.sleeping {
                asleep_after += 1;
            }
        }
        position += 1;
    }
    if rebuilt {
        (out.span(), asleep_after != 0, woken)
    } else {
        (entries, asleep_after != 0, woken)
    }
}

/// Unions the bodies of every touching pair of `pairs` that links two members.
fn link_pairs(ref forest: UnionFind, pairs: Span<ContactPair>, entries: Span<(Handle, RigidBody)>) {
    for pair in pairs {
        if *pair.manifold.data.num_solver_contacts != 0 {
            let body1 = *pair.manifold.data.rigid_body1;
            let body2 = *pair.manifold.data.rigid_body2;
            if links(entries, body1, body2) {
                let _ = forest.union(body1.unwrap().index, body2.unwrap().index);
            }
        }
    }
}

/// Upstream's `update_body_energy` for one body the step moved: the still-time counter grows by
/// `dt` when the body passes the motion gates of its type (`RigidBodyActivationTrait::
/// can_sleep`) and resets otherwise. `previous` is the pose before the step's position update,
/// `body.pos.position` the pose after it and `body.vels` the velocities the solver left. The
/// farthest-point displacement is [`relative_pose_drift`] with the body's `max_extent`.
///
/// Exact and bit-identical to upstream's gates on the values (the drift, then `dynamic_gate`),
/// with the work spread over three paths, cheapest first (lot SC, `alternatives::
/// update_sleep_timer_v1` is the straightforward form):
/// * a dynamic body that cannot sleep (negative threshold) stops at that test;
/// * a dynamic body with the default threshold, `length_unit` and `dt` whose translation reaches
///   `2 · DEFAULT_LIMIT` cannot sleep either (the drift is at least the translation, and the gate
///   wants half of it below the limit): [`translation_near`] decides that in `felt252` arithmetic,
///   with one integer conversion and no square root, the case of every fast-moving body;
/// * every other dynamic body goes through [`dynamic_gate_slow`], behind a one-iteration `while`
///   that only the bodies taking it are charged for (AGENTS.md §7); its arguments are the eleven
///   values it reads (the whole body as a captured variable costs 4k more,
///   `alternatives::update_sleep_timer_captured`).
///
/// A sleeping body is never passed here (the step skips it).
///
/// # Panics
/// `'Fixed: overflow'` from the chord's products, and from `angvel²` for a body without collider.
/// The values are those of the reference except in two ranges where it panics and this one
/// answers: a translation of `2^31` units or more in one step (its length overflows `Fixed`), and
/// `|angvel| ≥ 46 000` rad/s for a body with colliders (the wide square does not overflow).
#[inline(always)]
pub fn update_sleep_timer(ref body: RigidBody, previous: Pose2, params: IntegrationParameters) {
    // `RigidBodyActivationTrait::update_energy` arm by arm, inlined: an outlined call is
    // charged its dearest path (the drift's square roots and division) for every body.
    let can_sleep = match body.body_type {
        RigidBodyType::Dynamic => {
            let position = body.pos.position;
            let dx: felt252 = position.translation.x.raw.into() - previous.translation.x.raw.into();
            let dy: felt252 = position.translation.y.raw.into() - previous.translation.y.raw.into();
            if body.activation.time_since_can_sleep == ZERO
                && is_default_configuration(
                    body.activation.normalized_linear_threshold, params.length_unit, params.dt,
                )
                && !translation_near(dx, dy) {
                return;
            }
            let mut can_sleep = false;
            let mut pending = true;
            let mut near = false;
            if is_default_configuration(
                body.activation.normalized_linear_threshold, params.length_unit, params.dt,
            ) {
                if translation_near(dx, dy) {
                    near = true;
                } else {
                    pending = false;
                }
            } else if body.activation.normalized_linear_threshold < ZERO {
                // Cannot sleep.
                pending = false;
            }
            let threshold = body.activation.normalized_linear_threshold;
            let angular_threshold = body.activation.angular_threshold;
            let angvel = body.vels.angvel;
            let max_extent = body.mprops.max_extent;
            if pending {
                while pending {
                    can_sleep =
                        dynamic_gate_slow(
                            threshold,
                            angular_threshold,
                            angvel,
                            max_extent,
                            dx,
                            dy,
                            position.rotation,
                            previous.rotation,
                            params.length_unit,
                            params.dt,
                            near,
                        );
                    pending = false;
                }
            }
            can_sleep
        },
        RigidBodyType::KinematicPositionBased |
        RigidBodyType::KinematicVelocityBased => body.vels.linvel == Vec2Trait::ZERO
            && body.vels.angvel == ZERO,
        RigidBodyType::Fixed => true,
    };
    body.activation.update_timer(can_sleep, params.dt);
}

/// `2^64` and `2^128` as field elements: the weights of [`is_default_configuration`].
const TWO_POW_64: felt252 = 0x10000000000000000;
const TWO_POW_128: felt252 = 0x100000000000000000000000000000000;

/// `threshold == DEFAULT_NORMALIZED_LINEAR_THRESHOLD && length_unit == ONE && dt == DEFAULT_DT` in
/// one test: the three raw differences (each below `2^64` in magnitude) weighted by `1`, `2^64`
/// and `2^128` add up to zero, in the field or not (the sum stays far below the field size), only
/// when each is zero.
#[inline(always)]
fn is_default_configuration(threshold: Fixed, length_unit: Fixed, dt: Fixed) -> bool {
    let differences: felt252 = (threshold.raw.into()
        - DEFAULT_NORMALIZED_LINEAR_THRESHOLD.raw.into())
        + (length_unit.raw.into() - ONE.raw.into()) * TWO_POW_64
        + (dt.raw.into() - DEFAULT_DT.raw.into()) * TWO_POW_128;
    differences == 0
}

/// `(2 · DEFAULT_LIMIT)² - 1`: the squared raw translation below which a body with the default
/// configuration may still sleep (`length < 2 · limit`, see [`translation_below`]), minus one.
const NEAR_SQ_MINUS_ONE: felt252 = 51240943925283;

/// Whether the translation `(dx, dy)` (raw) is below `2 · DEFAULT_LIMIT`, i.e. `dx² + dy² <
/// (2 · DEFAULT_LIMIT)²`, in `felt252` (exact: the squares stay far below the field size): the
/// difference to the bound is a `u128` exactly when it is not negative.
#[inline(always)]
pub(crate) fn translation_near(dx: felt252, dy: felt252) -> bool {
    let room: Option<u128> = (NEAR_SQ_MINUS_ONE - dx * dx - dy * dy).try_into();
    room.is_some()
}

/// The gate of a dynamic body the fast tests of [`update_sleep_timer`] did not decide, from the
/// raw translation `(dx, dy)` of the step and the rotations before and after. `near`: the
/// translation is known to be below `2 · DEFAULT_LIMIT` (default configuration); `threshold` is
/// not negative. Upstream's `dynamic_gate` on the drift `|Δt| + chord` (see
/// [`relative_pose_drift`]), with the limit test done without the square root of `|Δt|`
/// ([`translation_below`]): only the chord of a turn ([`rotation_chord`]) needs square roots.
#[inline(always)]
fn dynamic_gate_slow(
    threshold: Fixed,
    angular_threshold: Fixed,
    angvel: Fixed,
    max_extent: Fixed,
    dx: felt252,
    dy: felt252,
    rotation: Rot2,
    previous_rotation: Rot2,
    length_unit: Fixed,
    dt: Fixed,
    near: bool,
) -> bool {
    let activation = RigidBodyActivation {
        normalized_linear_threshold: threshold,
        angular_threshold,
        time_until_sleep: ZERO,
        time_since_can_sleep: ZERO,
        sleeping: false,
    };
    let angular_ok = if max_extent > ZERO {
        activation.angular_gate_wide(angvel)
    } else {
        let sq_angvel = if angvel == ZERO {
            ZERO
        } else {
            angvel * angvel
        };
        activation.angular_gate(sq_angvel, max_extent)
    };
    if !angular_ok {
        return false;
    }
    let limit = if near {
        DEFAULT_LIMIT
    } else {
        activation.linear_limit(length_unit, dt)
    };
    if max_extent == MAX {
        // A half-space collider: the drift is `MAX`.
        MAX * HALF < limit
    } else if near && rotation == previous_rotation {
        true
    } else {
        translation_below(dx, dy, limit, rotation_chord(previous_rotation, rotation, max_extent))
    }
}

/// `(length(dx, dy) + chord) * HALF < limit` for a non-negative `chord`, without the square root:
/// `length` is `floor(sqrt(dx² + dy²))` on the raw values and the sum floors when halved, so the
/// gate holds exactly when `length < k` with `k = 2 · limit - chord`, that is when `k > 0` and
/// `dx² + dy² < k²`. In `felt252` (exact: `k ≤ 2^64`, so `k² ≤ 2^128`); a non-positive
/// `limit`
/// gives a non-positive `k`, so the gate never holds, as `drift / 2 < limit` never does.
#[inline(always)]
pub fn translation_below(dx: felt252, dy: felt252, limit: Fixed, chord: Fixed) -> bool {
    let k: felt252 = limit.raw.into() * 2 - chord.raw.into();
    let positive: Option<u64> = (k - 1).try_into();
    let room: Option<u128> = (k * k - 1 - dx * dx - dy * dy).try_into();
    positive.is_some() && room.is_some()
}

/// Upstream `relative_pose_drift`: how far the farthest point of a body of extent `max_extent`
/// may have moved from `base` to `cur`: `|Δt| + 2 sin(Δθ/2) · max_extent`, with `sin(Δθ/2) =
/// |sin Δθ| / sqrt(2 (1 + cos Δθ))` (`1` when the denominator is below `1e-6`, a half turn).
/// The chord is skipped when the rotation did not change (`sin Δθ = 0`, `cos Δθ > 0`).
/// Returns `fixed::MAX` for `max_extent = MAX` (a half-space collider): the body never sleeps.
/// Rounding: lengths and the square root floor, the division rounds to nearest. Inlined so
/// that a body that did not turn pays neither the complex product nor the second square root
/// and the division.
#[inline(always)]
pub fn relative_pose_drift(base: Pose2, cur: Pose2, max_extent: Fixed) -> Fixed {
    if max_extent == MAX {
        return MAX;
    }
    let trans = (cur.translation - base.translation).length();
    // Same rotation: `Δθ = 0`, no chord (skips the complex product of the common case).
    if cur.rotation == base.rotation {
        return trans;
    }
    let delta = cur.rotation.mul(base.rotation.inverse());
    if delta.im == ZERO && delta.re > ZERO {
        return trans;
    }
    trans + chord(delta, max_extent)
}

/// The rotation term of [`relative_pose_drift`] (`max_extent` is not `MAX`): zero when the rotation
/// did not change, else `chord(cur_rotation · base_rotation⁻¹)`.
#[inline(always)]
fn rotation_chord(base_rotation: Rot2, cur_rotation: Rot2, max_extent: Fixed) -> Fixed {
    if cur_rotation == base_rotation {
        return ZERO;
    }
    let delta = cur_rotation.mul(base_rotation.inverse());
    if delta.im == ZERO && delta.re > ZERO {
        return ZERO;
    }
    chord(delta, max_extent)
}

/// `2 sin(Δθ/2) · max_extent` for the relative rotation `delta`, with `sin(Δθ/2) = |sin Δθ|
/// /
/// sqrt(2 (1 + cos Δθ))`, `1` when the denominator is below `1e-6` (a half turn).
#[inline(always)]
fn chord(delta: Rot2, max_extent: Fixed) -> Fixed {
    let denom = TWO * (ONE + delta.re);
    let half_sin = if denom > CHORD_EPSILON {
        delta.im.abs() / denom.sqrt()
    } else {
        ONE
    };
    TWO * half_sin * max_extent
}

/// Upstream `recompute_max_extent`: the largest `|pos_wrt_parent · center − local_com| +
/// radius` over the local bounding spheres of the given enabled colliders `(shape,
/// pos_wrt_parent)`; `0` without collider, `fixed::MAX` as soon as one is a half-space.
pub fn max_extent(local_com: Vec2, colliders: Span<(Shape, Pose2)>) -> Fixed {
    let mut max = ZERO;
    for (shape, pos_wrt_parent) in colliders {
        let (center, radius) = local_bounding_sphere(*shape);
        if radius == MAX {
            return MAX;
        }
        let extent = (pos_wrt_parent.transform_point(center) - local_com).length() + radius;
        if extent > max {
            max = extent;
        }
    }
    max
}

/// Parry's `compute_local_bounding_sphere` as `(center, radius)`: a ball is its own sphere; a
/// cuboid's radius is the length of its half extents; a capsule is centred on its segment's
/// midpoint with `radius + half height`; a segment's sphere is the point cloud's (midpoint,
/// distance to an end); a half-space is unbounded (`fixed::MAX`). Lengths floor.
pub fn local_bounding_sphere(shape: Shape) -> (Vec2, Fixed) {
    match shape {
        Shape::Ball(ball) => (Vec2Trait::ZERO, ball.radius),
        Shape::Cuboid(cuboid) => (Vec2Trait::ZERO, cuboid.half_extents.length()),
        Shape::Capsule(capsule) => {
            let (a, b) = (capsule.segment.a, capsule.segment.b);
            (a.midpoint(b), capsule.radius + (b - a).length() * HALF)
        },
        Shape::Segment(segment) => {
            let center = segment.a.midpoint(segment.b);
            (center, (segment.a - center).length())
        },
        Shape::HalfSpace(_) => (Vec2Trait::ZERO, MAX),
        Shape::ConvexPolygon(p) => {
            // Defer polygon-only work in this outlined match.
            let mut result = (Vec2Trait::ZERO, ZERO);
            let mut pending = true;
            while pending {
                result =
                    rapier_geometry2d::shape::ConvexPolygonTrait::compute_local_bounding_sphere(
                        p.unbox(),
                    );
                pending = false;
            }
            result
        },
    }
}
