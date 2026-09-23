//! Rejected candidates: gather/scatter each row, preserving identical normal-then-tangent order;
//! two-stage midpoint halving; world-frame local anchors.
use fixed::HALF;
use fixed::wide::{WideNarrow, WideSub, dot2, wide_from, wide_mul};
use glam::Vec2;
use rapier_geometry2d::contact::SolverContact;
use rapier_math::pose2::{Pose2, Pose2Trait};
use super::super::body::{SolverBody, read, scatter, velocity};
use super::{ContactConstraint, solve_normal, solve_tangent, tangent};

/// Same value as `super::midpoint` (nested floors by 2^32 then 2 equal one floor by 2^33):
/// narrow the Q64.64 sum, then a separate `* HALF` product.
pub fn midpoint_two_stage(sc: SolverContact, dir: Vec2, com1: Vec2, com2: Vec2) -> Vec2 {
    let wp1 = com1 + sc.anchor1;
    let wp2 = com2 + sc.anchor2;
    let d = wp1 - wp2;
    let shift = dot2(d.x, dir.x, d.y, dir.y) - sc.dist;
    let s = wp1 + wp2;
    Vec2 {
        x: wide_from(s.x).sub(wide_mul(dir.x, shift)).narrow() * HALF,
        y: wide_from(s.y).sub(wide_mul(dir.y, shift)).narrow() * HALF,
    }
}

/// Local anchor from the world midpoint, as upstream writes it: `pose^-1 * point`. Handles the
/// world body without a branch but pays the translation subtraction again.
pub fn local_world_frame(pose: Pose2, point: Vec2) -> Vec2 {
    pose.inverse_transform_point(point)
}

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
