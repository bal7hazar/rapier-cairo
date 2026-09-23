//! Direct two-body kernels for island sweeps. Arithmetic and row order match the array API.
//! Gathered bodies include the identity WORLD body; only velocities are scattered.
use crate::solver::body::BodyPair;
use crate::solver::joint::*;

pub(crate) fn warmstart(constraint: JointConstraint, ref pair: BodyPair) {
    if constraint.num_rows == 0 {
        return;
    }
    let mut v1 = velocity(pair.first);
    let mut v2 = velocity(pair.second);
    let [a, b, c] = constraint.rows;
    apply(a, a.impulse, constraint.im1, constraint.im2, ref v1, ref v2);
    if constraint.num_rows >= 2 {
        apply(b, b.impulse, constraint.im1, constraint.im2, ref v1, ref v2);
    }
    if constraint.num_rows == 3 {
        apply(c, c.impulse, constraint.im1, constraint.im2, ref v1, ref v2);
    }
    pair.first.linvel = v1.linear;
    pair.first.angvel = v1.angular;
    pair.second.linvel = v2.linear;
    pair.second.angvel = v2.angular;
}

pub(crate) fn solve(ref constraint: JointConstraint, ref pair: BodyPair, biased: bool) {
    if constraint.num_rows == 0 {
        return;
    }
    if !biased {
        constraint.remove_bias();
    }
    let mut v1 = velocity(pair.first);
    let mut v2 = velocity(pair.second);
    let [mut a, mut b, mut c] = constraint.rows;
    solve_row(ref a, constraint.im1, constraint.im2, ref v1, ref v2);
    if constraint.num_rows >= 2 {
        solve_row(ref b, constraint.im1, constraint.im2, ref v1, ref v2);
    }
    if constraint.num_rows == 3 {
        solve_row(ref c, constraint.im1, constraint.im2, ref v1, ref v2);
    }
    constraint.rows = [a, b, c];
    pair.first.linvel = v1.linear;
    pair.first.angvel = v1.angular;
    pair.second.linvel = v2.linear;
    pair.second.angvel = v2.angular;
}


#[cfg(test)]
pub(crate) mod alternatives;
