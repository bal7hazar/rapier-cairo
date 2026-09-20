//! Rejected candidate: gather/scatter each row, preserving identical normal-then-tangent order.
use super::super::body::{SolverBody, read, scatter, velocity};
use super::{ContactConstraint, solve_normal, solve_tangent, tangent};

pub fn solve(
    ref c: ContactConstraint, ref bodies: Array<SolverBody>, normal: bool, friction: bool,
) {
    if c.num_elements == 0 || (!normal && !friction) {
        return;
    }
    let [mut a, mut b] = c.elements;
    let mut pass: u8 = 0;
    while pass != 2 {
        let enabled = if pass == 0 {
            normal
        } else {
            friction
        };
        if enabled {
            let mut i: u8 = 0;
            while i != c.num_elements {
                let mut e = if i == 0 {
                    a
                } else {
                    b
                };
                let mut v1 = velocity(read(bodies.span(), c.solver_vel1));
                let mut v2 = velocity(read(bodies.span(), c.solver_vel2));
                if pass == 0 {
                    solve_normal(ref e.normal_part, c.dir1, c.im1, c.im2, ref v1, ref v2);
                } else {
                    solve_tangent(
                        ref e.tangent_part,
                        tangent(c.dir1),
                        c.im1,
                        c.im2,
                        c.limit * e.normal_part.impulse,
                        ref v1,
                        ref v2,
                    );
                }
                scatter(ref bodies, c.solver_vel1, v1, c.solver_vel2, v2);
                if i == 0 {
                    a = e;
                } else {
                    b = e;
                }
                i += 1;
            }
        }
        pass += 1;
    }
    c.elements = [a, b];
}
