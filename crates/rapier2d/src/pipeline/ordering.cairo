//! D8 solve order of the step's touching manifolds (moved out of `crate::pipeline` by work
//! package CL, file budget): pairs of two non-fixed bodies first, then pairs with a fixed body or
//! none, each in pair order, and the scatter of the solved manifolds back into the pairs. Also
//! the per-body status lookup ([`body_status`]) the D8 flag and the sleeping stages (SL) share.

use rapier_core::Handle;
use rapier_core::rigid_body::RigidBodyType;
use rapier_dynamics2d::narrow_phase::ContactPair;
use rapier_dynamics2d::rigid_body_set::RigidBody;
use rapier_geometry2d::contact::ContactManifold;

/// The manifolds of the pairs that have at least one solver contact, in pair order. The solver
/// input is [`solve_order`] of this list.
pub fn touching_manifolds(pairs: Span<ContactPair>) -> Array<ContactManifold> {
    let mut out = array![];
    for pair in pairs {
        if *pair.manifold.data.num_solver_contacts != 0 {
            out.append(*pair.manifold);
        }
    }
    out
}

/// [`body_status`]: the manifold or joint side has no body (a standalone collider).
pub const BODY_NONE: u8 = 0;
/// [`body_status`]: a fixed body.
pub const BODY_FIXED: u8 = 1;
/// [`body_status`]: a disabled non-fixed body (not an island member, not moved by the step).
pub const BODY_DISABLED: u8 = 2;
/// [`body_status`]: an enabled non-fixed body that sleeps.
pub const BODY_SLEEPING: u8 = 3;
/// [`body_status`]: an enabled non-fixed body that is awake, or a handle `entries` lacks.
pub const BODY_AWAKE: u8 = 4;

/// D8: whether a manifold between `body1` and `body2` goes to the second solve group: it has a
/// fixed body or no body. Upstream colours a touching pair with the lowest free colour when both
/// bodies are non-fixed (kinematic included: `RigidBody::is_fixed` is `body_type == Fixed`) and
/// with the highest free colour otherwise ("fixed geometry the final say each sweep"), and solves
/// the colours in ascending order. `entries` holds every body in ascending arena slot.
#[inline(always)]
pub(crate) fn fixed_last_flag(
    entries: Span<(Handle, RigidBody)>, body1: Option<Handle>, body2: Option<Handle>,
) -> bool {
    let (s1, s2) = link_status(entries, body1, body2);
    fixed_last_of(s1, s2)
}

/// [`fixed_last_flag`] on two [`body_status`] codes: either side is fixed or has no body.
#[inline(always)]
pub(crate) fn fixed_last_of(s1: u8, s2: u8) -> bool {
    s1 <= BODY_FIXED || s2 <= BODY_FIXED
}

/// SL: a link (contact pair, joint) is dormant when neither side is moved by the step and one
/// of them sleeps: both sides are `BODY_NONE`, `BODY_FIXED` or `BODY_SLEEPING`, at least one
/// `BODY_SLEEPING`. Its pair is neither regenerated nor solved (upstream: a pair without an
/// awake body is not updated and the sleeping body is not in the active set).
#[inline(always)]
pub(crate) fn dormant_of(s1: u8, s2: u8) -> bool {
    (s1 == BODY_SLEEPING || s2 == BODY_SLEEPING)
        && s1 != BODY_AWAKE
        && s2 != BODY_AWAKE
        && s1 != BODY_DISABLED
        && s2 != BODY_DISABLED
}

/// The [`body_status`] codes of the two sides of a link; `None` is [`BODY_NONE`].
#[inline(always)]
pub(crate) fn link_status(
    entries: Span<(Handle, RigidBody)>, body1: Option<Handle>, body2: Option<Handle>,
) -> (u8, u8) {
    let s1 = match body1 {
        Some(h) => body_status(entries, h),
        None => BODY_NONE,
    };
    let s2 = match body2 {
        Some(h) => body_status(entries, h),
        None => BODY_NONE,
    };
    (s1, s2)
}

/// The status code (`BODY_*`) of the body of `handle`; a handle `entries` lacks counts as an
/// awake body (as the pre-SL lookup counted it non-fixed). The position of a body in `entries`
/// is at most its arena slot (equal while no body was removed), so the lookup starts at the
/// slot and walks down over the holes.
pub(crate) fn body_status(entries: Span<(Handle, RigidBody)>, handle: Handle) -> u8 {
    if entries.is_empty() {
        return BODY_AWAKE;
    }
    let mut position = handle.index;
    if position >= entries.len() {
        position = entries.len() - 1;
    }
    let mut status = BODY_AWAKE;
    loop {
        let (candidate, body) = entries.at(position);
        if candidate.index == @handle.index {
            status =
                if *body.body_type == RigidBodyType::Fixed {
                    BODY_FIXED
                } else if !*body.enabled {
                    BODY_DISABLED
                } else if *body.activation.sleeping {
                    BODY_SLEEPING
                } else {
                    BODY_AWAKE
                };
            break;
        }
        if candidate.index < @handle.index || position == 0 {
            break;
        }
        position -= 1;
    }
    status
}

/// The stable partition of `manifolds` (pair order): the entries flagged `false` (in order), then
/// those flagged `true`.
fn partition(manifolds: Span<ContactManifold>, flags: Span<bool>) -> Array<ContactManifold> {
    let mut first = array![];
    let mut last = array![];
    let mut next = flags;
    for manifold in manifolds {
        if *next.pop_front().unwrap() {
            last.append(*manifold);
        } else {
            first.append(*manifold);
        }
    }
    first.append_span(last.span());
    first
}

/// D8: the touching manifolds of `pairs` (as [`touching_manifolds`]) as the stable partition of
/// the ascending pair order into the manifolds between two non-fixed bodies, then the manifolds
/// with a fixed body or no body (see [`fixed_last_flag`]). `entries` holds every body
/// (`user_changes_bodies`, ascending arena slot). Returns the manifolds in solve order and, per
/// touching pair in pair order, whether it went to the second group (the argument of
/// [`scatter_touching`]).
pub fn solve_order(
    pairs: Span<ContactPair>, entries: Span<(Handle, RigidBody)>,
) -> (Array<ContactManifold>, Array<bool>) {
    let mut first = array![];
    let mut last = array![];
    let mut flags = array![];
    for pair in pairs {
        if *pair.manifold.data.num_solver_contacts != 0 {
            let manifold = *pair.manifold;
            let fixed_last = fixed_last_flag(
                entries, manifold.data.rigid_body1, manifold.data.rigid_body2,
            );
            flags.append(fixed_last);
            if fixed_last {
                last.append(manifold);
            } else {
                first.append(manifold);
            }
        }
    }
    first.append_span(last.span());
    (first, flags)
}

/// `pairs` with the manifold of each touching pair replaced by its solved manifold. `solved` is
/// the output of `solve_island` on [`solve_order`]'s manifolds and `last` its flags: the pairs
/// flagged `false` take `solved` from the front in pair order, the others from the point where
/// that group ends.
pub fn scatter_touching(
    pairs: Span<ContactPair>, solved: Span<ContactManifold>, last: Span<bool>,
) -> Array<ContactPair> {
    let mut n_first = 0;
    for fixed_last in last {
        if !*fixed_last {
            n_first += 1;
        }
    }
    scatter_touching_split(pairs, solved, last, n_first)
}

/// [`scatter_touching`] with the size of the first group given.
pub fn scatter_touching_split(
    pairs: Span<ContactPair>, solved: Span<ContactManifold>, last: Span<bool>, n_first: u32,
) -> Array<ContactPair> {
    let mut first = solved.slice(0, n_first);
    let mut rest = solved.slice(n_first, solved.len() - n_first);
    let mut flags = last;
    let mut out = array![];
    for pair in pairs {
        let mut pair = *pair;
        if pair.manifold.data.num_solver_contacts != 0 {
            pair
                .manifold =
                    if *flags.pop_front().unwrap() {
                        *rest.pop_front().unwrap()
                    } else {
                        *first.pop_front().unwrap()
                    };
        }
        out.append(pair);
    }
    out
}
