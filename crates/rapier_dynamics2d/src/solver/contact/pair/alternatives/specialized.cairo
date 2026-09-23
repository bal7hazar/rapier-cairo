//! Statically selected normal/friction kernels, with the original arithmetic and row order.
use crate::solver::body::BodyPair;
use crate::solver::contact::*;

pub(crate) fn solve_normal_only(ref c: ContactConstraint, ref pair: BodyPair) {
    if c.num_elements == 0 {
        return;
    }
    let mut v1 = velocity(pair.first);
    let mut v2 = velocity(pair.second);
    let [mut a, mut b] = c.elements;
    solve_normal(ref a.normal_part, c.dir1, c.im1, c.im2, ref v1, ref v2);
    if c.num_elements == 2 {
        solve_normal(ref b.normal_part, c.dir1, c.im1, c.im2, ref v1, ref v2);
    }
    c.elements = [a, b];
    pair.first.linvel = v1.linear;
    pair.first.angvel = v1.angular;
    pair.second.linvel = v2.linear;
    pair.second.angvel = v2.angular;
}

pub(crate) fn solve_both(ref c: ContactConstraint, ref pair: BodyPair) {
    if c.num_elements == 0 {
        return;
    }
    let mut v1 = velocity(pair.first);
    let mut v2 = velocity(pair.second);
    let [mut a, mut b] = c.elements;
    solve_normal(ref a.normal_part, c.dir1, c.im1, c.im2, ref v1, ref v2);
    if c.num_elements == 2 {
        solve_normal(ref b.normal_part, c.dir1, c.im1, c.im2, ref v1, ref v2);
    }
    let t = tangent(c.dir1);
    solve_tangent(
        ref a.tangent_part, t, c.im1, c.im2, c.limit * a.normal_part.impulse, ref v1, ref v2,
    );
    if c.num_elements == 2 {
        solve_tangent(
            ref b.tangent_part, t, c.im1, c.im2, c.limit * b.normal_part.impulse, ref v1, ref v2,
        );
    }
    c.elements = [a, b];
    pair.first.linvel = v1.linear;
    pair.first.angvel = v1.angular;
    pair.second.linvel = v2.linear;
    pair.second.angvel = v2.angular;
}
