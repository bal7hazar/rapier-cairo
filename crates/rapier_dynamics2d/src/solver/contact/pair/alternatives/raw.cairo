//! Direct loop-free kernels retained for gas reranking.
use crate::solver::body::BodyPair;
use crate::solver::contact::*;

pub(crate) fn warmstart(c: ContactConstraint, ref pair: BodyPair) {
    if c.num_elements == 0 {
        return;
    }
    let mut v1 = velocity(pair.first);
    let mut v2 = velocity(pair.second);
    let [a, b] = c.elements;
    warm_normal(a.normal_part, c, ref v1, ref v2);
    if c.num_elements == 2 {
        warm_normal(b.normal_part, c, ref v1, ref v2);
    }
    warm_tangent(a.tangent_part, c, ref v1, ref v2);
    if c.num_elements == 2 {
        warm_tangent(b.tangent_part, c, ref v1, ref v2);
    }
    pair.first.linvel = v1.linear;
    pair.first.angvel = v1.angular;
    pair.second.linvel = v2.linear;
    pair.second.angvel = v2.angular;
}

pub(crate) fn solve(
    ref c: ContactConstraint, ref pair: BodyPair, solve_restitution: bool, solve_friction: bool,
) {
    if c.num_elements == 0 || (!solve_restitution && !solve_friction) {
        return;
    }
    let mut v1 = velocity(pair.first);
    let mut v2 = velocity(pair.second);
    let [mut a, mut b] = c.elements;
    if solve_restitution {
        solve_normal(ref a.normal_part, c.dir1, c.im1, c.im2, ref v1, ref v2);
        if c.num_elements == 2 {
            solve_normal(ref b.normal_part, c.dir1, c.im1, c.im2, ref v1, ref v2);
        }
    }
    if solve_friction {
        let t = tangent(c.dir1);
        solve_tangent(
            ref a.tangent_part, t, c.im1, c.im2, c.limit * a.normal_part.impulse, ref v1, ref v2,
        );
        if c.num_elements == 2 {
            solve_tangent(
                ref b.tangent_part,
                t,
                c.im1,
                c.im2,
                c.limit * b.normal_part.impulse,
                ref v1,
                ref v2,
            );
        }
    }
    c.elements = [a, b];
    pair.first.linvel = v1.linear;
    pair.first.angvel = v1.angular;
    pair.second.linvel = v2.linear;
    pair.second.angvel = v2.angular;
}

pub(crate) fn apply_restitution(ref c: ContactConstraint, ref pair: BodyPair) {
    if c.num_elements == 0 {
        return;
    }
    let [mut a, mut b] = c.elements;
    if a.restitution_seed >= ZERO && (c.num_elements == 1 || b.restitution_seed >= ZERO) {
        return;
    }
    let mut v1 = velocity(pair.first);
    let mut v2 = velocity(pair.second);
    bounce(ref a, c.dir1, c.im1, c.im2, ref v1, ref v2);
    if c.num_elements == 2 {
        bounce(ref b, c.dir1, c.im1, c.im2, ref v1, ref v2);
    }
    c.elements = [a, b];
    pair.first.linvel = v1.linear;
    pair.first.angvel = v1.angular;
    pair.second.linvel = v2.linear;
    pair.second.angvel = v2.angular;
}

pub(crate) fn update(ref c: ContactConstraint, params: IntegrationParameters, pair: BodyPair) {
    if c.num_elements == 0 {
        return;
    }
    assert(params.warmstart_coefficient >= ZERO, errors::NEGATIVE);
    let p1 = pair.first.position;
    let p2 = pair.second.position;
    let cap = params.max_corrective_velocity();
    let [mut a, mut b] = c.elements;
    update_element(ref a, c, p1, p2, params.warmstart_coefficient, cap);
    if c.num_elements == 2 {
        update_element(ref b, c, p1, p2, params.warmstart_coefficient, cap);
    }
    c.elements = [a, b];
    c.cfm_factor = c.soft_cfm_factor;
}

pub(crate) fn update_rhs_wo_bias(ref c: ContactConstraint, pair: BodyPair) {
    if c.num_elements == 0 {
        return;
    }
    let p1 = pair.first.position;
    let p2 = pair.second.position;
    let [mut a, mut b] = c.elements;
    refresh_unbiased(ref a, c, p1, p2);
    if c.num_elements == 2 {
        refresh_unbiased(ref b, c, p1, p2);
    }
    c.elements = [a, b];
    c.remove_bias();
}
