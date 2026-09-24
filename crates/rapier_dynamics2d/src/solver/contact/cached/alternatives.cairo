//! Cached kernels without zero-state guards, retained for comparison.
use super::{*, apply, solve_normal, solve_tangent};

pub(crate) fn solve_normal_only(
    ref c: ContactConstraint, directions: Directions, ref pair: BodyPair,
) {
    if c.num_elements == 0 {
        return;
    }
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
}

pub(crate) fn solve_both(ref c: ContactConstraint, directions: Directions, ref pair: BodyPair) {
    if c.num_elements == 0 {
        return;
    }
    let mut v1 = velocity(pair.first);
    let mut v2 = velocity(pair.second);
    let [mut a, mut b] = c.elements;
    solve_normal(ref a.normal_part, c.dir1, directions.normal, ref v1, ref v2);
    if c.num_elements == 2 {
        solve_normal(ref b.normal_part, c.dir1, directions.normal, ref v1, ref v2);
    }
    let t = tangent(c.dir1);
    solve_tangent(
        ref a.tangent_part, t, directions.tangent, c.limit * a.normal_part.impulse, ref v1, ref v2,
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
}

pub(crate) fn warmstart(c: ContactConstraint, directions: Directions, ref pair: BodyPair) {
    if c.num_elements == 0 {
        return;
    }
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
}
