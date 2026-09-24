//! Rejected dispatch candidates: outlined early return and single-iteration metering.
use super::*;

pub fn solve_early_return(ref c: JointConstraint, ref bodies: Array<SolverBody>, biased: bool) {
    let mut pending = c.num_rows == 4;
    while pending {
        super::solve(ref c, ref bodies, biased, false);
        return;
    }
    if c.num_rows == 0 {
        return;
    }
    if !biased {
        c.remove_bias();
    }
    let mut v1 = velocity(read(bodies.span(), c.solver_vel1));
    let mut v2 = velocity(read(bodies.span(), c.solver_vel2));
    let [mut a, mut b, mut d] = c.rows;
    solve_row(ref a, c.im1, c.im2, ref v1, ref v2);
    if c.num_rows >= 2 {
        solve_row(ref b, c.im1, c.im2, ref v1, ref v2);
    }
    if c.num_rows == 3 {
        solve_row(ref d, c.im1, c.im2, ref v1, ref v2);
    }
    c.rows = [a, b, d];
    scatter(ref bodies, c.solver_vel1, v1, c.solver_vel2, v2);
}

pub fn solve_metered(ref c: JointConstraint, ref bodies: Array<SolverBody>, biased: bool) {
    let mut pending = c.num_rows == 4;
    while pending {
        super::solve(ref c, ref bodies, biased, false);
        pending = false;
    }
    if c.num_rows == 0 || c.num_rows == 4 {
        return;
    }
    if !biased {
        c.remove_bias();
    }
    let mut v1 = velocity(read(bodies.span(), c.solver_vel1));
    let mut v2 = velocity(read(bodies.span(), c.solver_vel2));
    let [mut a, mut b, mut d] = c.rows;
    solve_row(ref a, c.im1, c.im2, ref v1, ref v2);
    if c.num_rows >= 2 {
        solve_row(ref b, c.im1, c.im2, ref v1, ref v2);
    }
    if c.num_rows == 3 {
        solve_row(ref d, c.im1, c.im2, ref v1, ref v2);
    }
    c.rows = [a, b, d];
    scatter(ref bodies, c.solver_vel1, v1, c.solver_vel2, v2);
}
