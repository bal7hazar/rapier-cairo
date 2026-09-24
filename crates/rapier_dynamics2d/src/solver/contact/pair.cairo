//! Direct two-body kernels for island sweeps. Arithmetic and row order match the array API.
//! Gathered bodies include the identity WORLD body; only velocities are scattered.
use super::*;
use super::super::body::BodyPair;

/// Refresh/bank the active rows using current poses and frozen coefficients.
/// Requires nonnegative warmstart coefficient; rounding and panics match array `update`.
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


/// Refresh speculative slack from current poses and strip bias/softness.
/// Same products, rounding and overflow panics as the array API; preserves body values.
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


#[cfg(test)]
pub(crate) mod alternatives;

/// Final bounce for eligible new contacts; bodies must match constraint endpoint order.
/// Rounding/overflow match the array API. The computing path is metered only when entered.
pub(crate) fn apply_restitution_metered(ref c: ContactConstraint, ref pair: BodyPair) {
    if c.num_elements == 0 {
        return;
    }
    let [mut a, mut b] = c.elements;
    if a.restitution_seed >= ZERO && (c.num_elements == 1 || b.restitution_seed >= ZERO) {
        return;
    }
    let mut pending = true;
    while pending {
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
        pending = false;
    }
}
