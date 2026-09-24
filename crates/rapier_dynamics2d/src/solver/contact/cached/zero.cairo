//! Exact zero-state fast path. With zero velocities, row rhs and impulses, every computed
//! delta and every velocity increment is exactly zero. No arithmetic is reassociated.
use super::{*, solve_normal, solve_tangent};

#[inline(always)]
fn row_zero(e: ContactConstraintElement) -> bool {
    e.normal_part.rhs == ZERO
        && e.normal_part.impulse == ZERO
        && e.tangent_part.rhs == ZERO
        && e.tangent_part.impulse == ZERO
}
#[inline(always)]
fn idle(c: ContactConstraint, pair: BodyPair) -> bool {
    let [a, b] = c.elements;
    pair.first.linvel == Default::default()
        && pair.second.linvel == Default::default()
        && pair.first.angvel == ZERO
        && pair.second.angvel == ZERO
        && row_zero(a)
        && (c.num_elements == 1 || row_zero(b))
}
/// Normal-only sweep with matching cached directions. The proven all-zero state is inert.
/// Otherwise preserves scalar operation order, floor rounding and fixed overflow panics.
pub(crate) fn solve_normal_only(
    ref c: ContactConstraint, directions: Directions, ref pair: BodyPair,
) {
    if c.num_elements == 0 {
        return;
    }
    if idle(c, pair) {
        return;
    }
    let mut pending = true;
    while pending {
        let mut v1 = velocity(pair.first);
        let mut v2 = velocity(pair.second);
        let [mut a, mut b] = c.elements;
        solve_normal(ref a.normal_part, c.dir1, directions.normal, ref v1, ref v2);
        if c.num_elements == 2 {
            solve_normal(ref b.normal_part, c.dir1, directions.normal, ref v1, ref v2);
        }
        c.elements = [a, b];
        pair.first.linvel = v1.linear;
        pair.first.angvel = v1.angular;
        pair.second.linvel = v2.linear;
        pair.second.angvel = v2.angular;
        pending = false;
    }
}

/// Normal-then-tangent sweep with matching frame cache and gathered endpoint bodies.
/// The all-zero state is inert; otherwise rounding, range and panics match the array API.
pub(crate) fn solve_both(ref c: ContactConstraint, directions: Directions, ref pair: BodyPair) {
    if c.num_elements == 0 {
        return;
    }
    if idle(c, pair) {
        return;
    }
    let mut pending = true;
    while pending {
        let mut v1 = velocity(pair.first);
        let mut v2 = velocity(pair.second);
        let [mut a, mut b] = c.elements;
        solve_normal(ref a.normal_part, c.dir1, directions.normal, ref v1, ref v2);
        if c.num_elements == 2 {
            solve_normal(ref b.normal_part, c.dir1, directions.normal, ref v1, ref v2);
        }
        let t = tangent(c.dir1);
        solve_tangent(
            ref a.tangent_part,
            t,
            directions.tangent,
            c.limit * a.normal_part.impulse,
            ref v1,
            ref v2,
        );
        if c.num_elements == 2 {
            solve_tangent(
                ref b.tangent_part,
                t,
                directions.tangent,
                c.limit * b.normal_part.impulse,
                ref v1,
                ref v2,
            );
        }
        c.elements = [a, b];
        pair.first.linvel = v1.linear;
        pair.first.angvel = v1.angular;
        pair.second.linvel = v2.linear;
        pair.second.angvel = v2.angular;
        pending = false;
    }
}
