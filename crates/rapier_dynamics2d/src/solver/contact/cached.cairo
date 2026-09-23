//! Cached products preserve `(dir * im) * impulse`: no reassociation or changed rounding.
//! Direction and inverse masses are frozen by contact generation for the entire frame.
use super::*;
use super::super::body::BodyPair;

/// Frame-constant `direction * inverse_mass` for both endpoints, floored once per component.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub(crate) struct WeightedPair {
    pub first: Vec2,
    pub second: Vec2,
}
/// Normal/tangent weighted directions; retain the exact constraint order until frame end.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub(crate) struct Directions {
    pub normal: WeightedPair,
    pub tangent: WeightedPair,
}
/// Cache the unchanged inner products of `(dir * im) * impulse`, in constraint order.
/// Products floor; fixed overflow propagates. Inputs must remain frozen for the frame.
pub(crate) fn prepare(mut cs: Span<ContactConstraint>) -> Array<Directions> {
    let mut out = array![];
    while let Some(c) = cs.pop_front() {
        let t = tangent(*c.dir1);
        out
            .append(
                Directions {
                    normal: WeightedPair { first: *c.dir1 * *c.im1, second: *c.dir1 * *c.im2 },
                    tangent: WeightedPair { first: t * *c.im1, second: t * *c.im2 },
                },
            );
    }
    out
}
/// Apply an impulse with preweighted directions and inertia-weighted angular Jacobians.
/// Preserves the original multiplication/addition order; products floor, overflow panics.
pub(crate) fn apply(
    _dir: Vec2,
    weighted: WeightedPair,
    ig1: Fixed,
    ig2: Fixed,
    impulse: Fixed,
    ref v1: SolverVel,
    ref v2: SolverVel,
) {
    v1.linear = v1.linear + weighted.first * Vec2 { x: impulse, y: impulse };
    v2.linear = v2.linear + weighted.second * Vec2 { x: -impulse, y: -impulse };
    v1.angular += ig1 * impulse;
    v2.angular += ig2 * impulse;
}

/// Solve one normal row, mutating its impulse and both velocities. Cache must match `dir`.
/// Uses the original Q32.32 operations and rounding; all intermediates must fit Fixed.
pub(crate) fn solve_normal(
    ref p: ContactConstraintNormalPart,
    dir: Vec2,
    weighted: WeightedPair,
    ref v1: SolverVel,
    ref v2: SolverVel,
) {
    let dv = jv(dir, p.gcross1, p.gcross2, v1, v2) + p.rhs;
    let new_impulse = p.cfm_factor * max(ZERO, p.impulse - p.r * dv);
    let delta = new_impulse - p.impulse;
    p.impulse = new_impulse;
    apply(dir, weighted, p.ii_gcross1, p.ii_gcross2, delta, ref v1, ref v2);
}

/// Solve one tangent row with the supplied friction limit and matching direction cache.
/// Preserves Q32.32 rounding and overflow panics, updating the row and both velocities.
pub(crate) fn solve_tangent(
    ref p: ContactConstraintTangentPart,
    dir: Vec2,
    weighted: WeightedPair,
    limit: Fixed,
    ref v1: SolverVel,
    ref v2: SolverVel,
) {
    let dv = jv(dir, p.gcross1, p.gcross2, v1, v2) + p.rhs;
    let new_impulse = min(limit, max(-limit, p.impulse - p.r * dv));
    let delta = new_impulse - p.impulse;
    p.impulse = new_impulse;
    apply(dir, weighted, p.ii_gcross1, p.ii_gcross2, delta, ref v1, ref v2);
}


/// Apply normal then tangent warm starts using the matching frame cache.
/// Zero impulses are an exact no-op; otherwise products floor and overflow panics.
/// Only velocities change; WORLD bodies must be gathered as the identity default.
pub(crate) fn warmstart_sparse(c: ContactConstraint, directions: Directions, ref pair: BodyPair) {
    if c.num_elements == 0 {
        return;
    }
    let [a, b] = c.elements;
    if a.normal_part.impulse == ZERO
        && a.tangent_part.impulse == ZERO
        && (c.num_elements == 1
            || (b.normal_part.impulse == ZERO && b.tangent_part.impulse == ZERO)) {
        return;
    }
    let mut pending = true;
    while pending {
        let mut v1 = velocity(pair.first);
        let mut v2 = velocity(pair.second);
        let [a, b] = c.elements;
        apply(
            c.dir1,
            directions.normal,
            a.normal_part.ii_gcross1,
            a.normal_part.ii_gcross2,
            a.normal_part.impulse,
            ref v1,
            ref v2,
        );
        if c.num_elements == 2 {
            apply(
                c.dir1,
                directions.normal,
                b.normal_part.ii_gcross1,
                b.normal_part.ii_gcross2,
                b.normal_part.impulse,
                ref v1,
                ref v2,
            );
        }
        apply(
            tangent(c.dir1),
            directions.tangent,
            a.tangent_part.ii_gcross1,
            a.tangent_part.ii_gcross2,
            a.tangent_part.impulse,
            ref v1,
            ref v2,
        );
        if c.num_elements == 2 {
            apply(
                tangent(c.dir1),
                directions.tangent,
                b.tangent_part.ii_gcross1,
                b.tangent_part.ii_gcross2,
                b.tangent_part.impulse,
                ref v1,
                ref v2,
            );
        }
        pair.first.linvel = v1.linear;
        pair.first.angvel = v1.angular;
        pair.second.linvel = v2.linear;
        pair.second.angvel = v2.angular;
        pending = false;
    }
}

#[cfg(test)]
pub(crate) mod alternatives;

pub(crate) mod zero;
