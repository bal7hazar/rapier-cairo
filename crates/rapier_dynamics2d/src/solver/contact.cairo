//! Scalar soft contacts on frozen mock manifolds. Call `generate` once per frame, then each
//! substep: add forces, `update`, `warmstart`, biased `solve`, integrate poses,
//! `update_rhs_wo_bias`, unbiased `solve`. After all substeps call `apply_restitution`, then
//! `writeback_impulses`. DF supplies the driver. Every sweep visits normals in element order,
//! then tangents in element order. `solve_restitution` means solve normal rows (upstream name).
//!
//! All arithmetic is checked Q32.32: fused Jacobian dots floor once, scalar products floor,
//! divisions round to nearest. Overflow panics come from fixed/core; masses and coefficients must
//! be nonnegative and all intermediates representable. Normals must be unit. Masses, dt, softness,
//! manifold membership and body-array ordering are frozen until the next `generate`.
mod element;
pub(crate) mod pair;
mod set;
use core::num::traits::DivRem;
pub use element::{
    ContactConstraintElement, ContactConstraintNormalPart, ContactConstraintNormalPartTrait,
    ContactConstraintTangentPart, ContactConstraintTangentPartTrait,
};
use element::{apply, bounce, coefficients, dot, jv, max, min, solve_normal, solve_tangent, tangent};
use fixed::wide::{WideMul, WideNarrow, WideSub, dot2, wide_from, wide_mul};
use fixed::{Fixed, HALF, ONE, ZERO};
use glam::Vec2;
use rapier_core::data::handle::Handle;
use rapier_core::integration_parameters::spring::SpringCoefficientsTrait;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_geometry2d::contact::{
    ContactManifold, ContactManifoldTrait, NEW_CONTACT_BIT, SolverContact,
};
use rapier_math::math_ext::inv;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
pub use set::{ContactConstraintsSet, ContactConstraintsSetTrait};
use super::body::{SolverBody, SolverVel, WORLD, read, scatter, velocity};

/// Invalid solver inputs, checked during generation (off the hot solve path).
pub mod errors {
    pub const COUNT: felt252 = 'Contact: invalid count';
    pub const CONTACT_ID: felt252 = 'Contact: invalid point id';
    pub const BODY: felt252 = 'Contact: missing body';
    pub const SAME_BODY: felt252 = 'Contact: same body';
    pub const NEGATIVE: felt252 = 'Contact: negative input';
}

/// One manifold, at most two active elements. `solver_vel*` are dense body-array indices;
/// `0xffffffff` is the immovable world. `manifold_id` indexes the original manifold span.
/// `limit` is the coefficient of friction; each tangent's actual limit uses its normal impulse.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct ContactConstraint {
    pub solver_vel1: u32,
    pub solver_vel2: u32,
    pub dir1: Vec2,
    pub im1: Vec2,
    pub im2: Vec2,
    pub cfm_factor: Fixed,
    pub limit: Fixed,
    pub elements: [ContactConstraintElement; 2],
    pub num_elements: u8,
    pub manifold_id: u32,
    /// Hoisted substep reciprocal and spring coefficients; no divisions in update or solve.
    pub inv_dt: Fixed,
    pub erp_inv_dt: Fixed,
    pub soft_cfm_factor: Fixed,
}

#[generate_trait]
pub impl ContactConstraintImpl of ContactConstraintTrait {
    /// Build one manifold with id zero; sets assign its actual id. Empty/disabled manifolds
    /// are no-ops. Resolves full generational handles in `bodies`; absent handles are world.
    /// `dt` is the substep duration, >= 0 (zero has reciprocal zero). Checks counts, point ids,
    /// distinct bodies and nonnegative materials/masses; panics with the `errors` constants.
    /// Both rows' lever arms and both local anchors use upstream's common midpoint of the two
    /// witnesses (`midpoint`), frozen until the next generate. Warm starts are zeroed for NEW
    /// contacts.
    fn generate(
        manifold: ContactManifold,
        bodies: Span<SolverBody>,
        params: IntegrationParameters,
        dt: Fixed,
    ) -> ContactConstraint {
        assert(
            manifold.num_points <= 2 && manifold.data.num_solver_contacts <= manifold.num_points,
            errors::COUNT,
        );
        let (_, enabled) = DivRem::div_rem(manifold.data.solver_flags.bits, 2);
        if manifold.data.num_solver_contacts == 0 || enabled == 0 {
            return Default::default();
        }
        assert(
            dt >= ZERO && manifold.data.friction >= ZERO && manifold.data.restitution >= ZERO,
            errors::NEGATIVE,
        );
        let raw1 = resolve(bodies, manifold.data.rigid_body1);
        let raw2 = resolve(bodies, manifold.data.rigid_body2);
        assert(raw1 == WORLD || raw2 == WORLD || raw1 != raw2, errors::SAME_BODY);
        let original1 = read(bodies, raw1);
        let original2 = read(bodies, raw2);
        validate_mass(original1);
        validate_mass(original2);
        // Zero inverse mass does not imply WORLD: kinematic endpoints retain their
        // velocity and substep pose, including when initially stationary. Fixed parents
        // already dominate through effective_group; absent parents resolve to WORLD.
        let id1 = if manifold.data.relative_dominance > 0 {
            WORLD
        } else {
            raw1
        };
        let id2 = if manifold.data.relative_dominance < 0 {
            WORLD
        } else {
            raw2
        };
        let b1 = read(bodies, id1);
        let b2 = read(bodies, id2);
        let spring = if id1 == WORLD || id2 == WORLD {
            params.static_contact_softness
        } else {
            params.contact_softness
        };
        let soft = spring.coefficients(dt);
        let mut result = ContactConstraint {
            solver_vel1: id1,
            solver_vel2: id2,
            dir1: -manifold.data.normal,
            im1: b1.im,
            im2: b2.im,
            cfm_factor: soft.cfm_factor,
            limit: manifold.data.friction,
            num_elements: manifold.data.num_solver_contacts,
            inv_dt: inv(dt),
            erp_inv_dt: soft.erp_inv_dt,
            soft_cfm_factor: soft.cfm_factor,
            ..Default::default(),
        };
        let [sc0, sc1] = manifold.data.solver_contacts;
        let e0 = generate_element(
            sc0, manifold, result.dir1, original1, original2, b1, b2, id1 == WORLD, id2 == WORLD,
        );
        let e1 = if result.num_elements == 2 {
            let e = generate_element(
                sc1,
                manifold,
                result.dir1,
                original1,
                original2,
                b1,
                b2,
                id1 == WORLD,
                id2 == WORLD,
            );
            assert(e.contact_id != e0.contact_id, errors::CONTACT_ID);
            e
        } else {
            Default::default()
        };
        result.elements = [e0, e1];
        result
    }

    /// Refresh distances/biases and bank previous impulses before warm-start scaling.
    /// `manifold` identifies this frame's unchanged contact membership; no geometry rebuild.
    /// Uses cached dt/softness; `params` supplies warmstart coefficient and corrective cap.
    /// Requires nonnegative warmstart coefficient; fixed arithmetic may overflow.
    fn update(
        ref self: ContactConstraint,
        params: IntegrationParameters,
        bodies: Span<SolverBody>,
        _manifold: ContactManifold,
    ) {
        if self.num_elements == 0 {
            return;
        }
        assert(params.warmstart_coefficient >= ZERO, errors::NEGATIVE);
        let p1 = read(bodies, self.solver_vel1).position;
        let p2 = read(bodies, self.solver_vel2).position;
        let cap = params.max_corrective_velocity();
        let [mut a, mut b] = self.elements;
        update_element(ref a, self, p1, p2, params.warmstart_coefficient, cap);
        if self.num_elements == 2 {
            update_element(ref b, self, p1, p2, params.warmstart_coefficient, cap);
        }
        self.elements = [a, b];
        self.cfm_factor = self.soft_cfm_factor;
    }

    /// Reapply the scaled normal then tangent impulses, with one gather/scatter per manifold.
    /// Body array order must match generation; arithmetic overflow panics.
    fn warmstart(self: ContactConstraint, ref bodies: Array<SolverBody>) {
        if self.num_elements == 0 {
            return;
        }
        let mut v1 = velocity(read(bodies.span(), self.solver_vel1));
        let mut v2 = velocity(read(bodies.span(), self.solver_vel2));
        let [a, b] = self.elements;
        warm_normal(a.normal_part, self, ref v1, ref v2);
        if self.num_elements == 2 {
            warm_normal(b.normal_part, self, ref v1, ref v2);
        }
        warm_tangent(a.tangent_part, self, ref v1, ref v2);
        if self.num_elements == 2 {
            warm_tangent(b.tangent_part, self, ref v1, ref v2);
        }
        scatter(ref bodies, self.solver_vel1, v1, self.solver_vel2, v2);
    }

    /// Sequential normal and/or friction sweep. Uses current rhs (biased after `update`, rigid
    /// after `update_rhs_wo_bias`). Zero divisions; one body gather/scatter for both points.
    /// Fixed products/dots floor; overflow panics. Does not perform the final bounce pass.
    fn solve(
        ref self: ContactConstraint,
        ref bodies: Array<SolverBody>,
        solve_restitution: bool,
        solve_friction: bool,
    ) {
        if self.num_elements == 0 || (!solve_restitution && !solve_friction) {
            return;
        }
        let mut v1 = velocity(read(bodies.span(), self.solver_vel1));
        let mut v2 = velocity(read(bodies.span(), self.solver_vel2));
        let [mut a, mut b] = self.elements;
        if solve_restitution {
            solve_normal(ref a.normal_part, self.dir1, self.im1, self.im2, ref v1, ref v2);
            if self.num_elements == 2 {
                solve_normal(ref b.normal_part, self.dir1, self.im1, self.im2, ref v1, ref v2);
            }
        }
        if solve_friction {
            let t = tangent(self.dir1);
            solve_tangent(
                ref a.tangent_part,
                t,
                self.im1,
                self.im2,
                self.limit * a.normal_part.impulse,
                ref v1,
                ref v2,
            );
            if self.num_elements == 2 {
                solve_tangent(
                    ref b.tangent_part,
                    t,
                    self.im1,
                    self.im2,
                    self.limit * b.normal_part.impulse,
                    ref v1,
                    ref v2,
                );
            }
        }
        self.elements = [a, b];
        scatter(ref bodies, self.solver_vel1, v1, self.solver_vel2, v2);
    }

    /// Strip penetration/tangent bias and softness using cached unbiased rhs. Exact copy;
    /// when poses changed, use `update_rhs_wo_bias` instead so speculative slack is refreshed.
    fn remove_bias(ref self: ContactConstraint) {
        let [mut a, mut b] = self.elements;
        strip(ref a);
        if self.num_elements == 2 {
            strip(ref b);
        }
        self.elements = [a, b];
        self.cfm_factor = ONE;
    }

    /// Recompute speculative slack from the CURRENT poses, then strip bias/softness.
    /// Call after position integration, before the relax sweep; cached substep reciprocal.
    /// Products floor; pose/dot overflow panics.
    fn update_rhs_wo_bias(ref self: ContactConstraint, bodies: Span<SolverBody>) {
        if self.num_elements == 0 {
            return;
        }
        let p1 = read(bodies, self.solver_vel1).position;
        let p2 = read(bodies, self.solver_vel2).position;
        let [mut a, mut b] = self.elements;
        refresh_unbiased(ref a, self, p1, p2);
        if self.num_elements == 2 {
            refresh_unbiased(ref b, self, p1, p2);
        }
        self.elements = [a, b];
        self.remove_bias();
    }

    /// Final rigid restitution sweep. Only NEW, approaching contacts with a positive total
    /// normal impulse this frame are eligible. Call once after all substeps; overflow panics.
    fn apply_restitution(ref self: ContactConstraint, ref bodies: Array<SolverBody>) {
        if self.num_elements == 0 {
            return;
        }
        let [mut a, mut b] = self.elements;
        if a.restitution_seed >= ZERO && (self.num_elements == 1 || b.restitution_seed >= ZERO) {
            return;
        }
        let mut v1 = velocity(read(bodies.span(), self.solver_vel1));
        let mut v2 = velocity(read(bodies.span(), self.solver_vel2));
        bounce(ref a, self.dir1, self.im1, self.im2, ref v1, ref v2);
        if self.num_elements == 2 {
            bounce(ref b, self.dir1, self.im1, self.im2, ref v1, ref v2);
        }
        self.elements = [a, b];
        scatter(ref bodies, self.solver_vel1, v1, self.solver_vel2, v2);
    }

    /// Persist full-frame totals and final-substep warm starts into the original tracked point
    /// ids, preserving other fields. Array order/count must match generation; out-of-range id
    /// panics with `Contact: invalid point id`. Totals can panic on overflow.
    fn writeback_impulses(self: ContactConstraint, ref manifolds: Array<ContactManifold>) {
        if self.num_elements == 0 {
            return;
        }
        assert(self.manifold_id < manifolds.len(), errors::CONTACT_ID);
        let mut out = array![];
        let mut id = 0;
        while let Some(mut m) = manifolds.pop_front() {
            if id == self.manifold_id {
                writeback(self, ref m);
            }
            out.append(m);
            id += 1;
        }
        manifolds = out;
    }
}

fn resolve(mut bodies: Span<SolverBody>, handle: Option<Handle>) -> u32 {
    let Some(h) = handle else {
        return WORLD;
    };
    let mut id = 0;
    while let Some(b) = bodies.pop_front() {
        if *b.handle == h {
            return id;
        }
        id += 1;
    }
    core::panic_with_felt252(errors::BODY)
}
fn validate_mass(b: SolverBody) {
    assert(b.im.x >= ZERO && b.im.y >= ZERO && b.ii >= ZERO, errors::NEGATIVE);
}

fn generate_element(
    sc: SolverContact,
    m: ContactManifold,
    dir: Vec2,
    original1: SolverBody,
    original2: SolverBody,
    b1: SolverBody,
    b2: SolverBody,
    world1: bool,
    world2: bool,
) -> ContactConstraintElement {
    let is_new = sc.contact_id >= NEW_CONTACT_BIT;
    let cid = if is_new {
        sc.contact_id - NEW_CONTACT_BIT
    } else {
        sc.contact_id
    };
    assert(cid < m.num_points.into(), errors::CONTACT_ID);
    let data = m.point(cid.try_into().unwrap()).data;
    let ni = if is_new {
        ZERO
    } else {
        data.warmstart_impulse
    };
    let ti = if is_new {
        ZERO
    } else {
        data.warmstart_tangent_impulse
    };
    assert(ni >= ZERO, errors::NEGATIVE);
    let point = midpoint(sc, dir, original1.position.translation, original2.position.translation);
    let dp1 = point - original1.position.translation;
    let dp2 = point - original2.position.translation;
    let (g1, g2, ig1, ig2, r) = coefficients(dir, dp1, dp2, b1, b2);
    let n = ContactConstraintNormalPart {
        gcross1: g1,
        gcross2: g2,
        ii_gcross1: ig1,
        ii_gcross2: ig2,
        r,
        impulse: ni,
        impulse_accumulator: -ni,
        ..Default::default(),
    };
    let seed = if is_new {
        m.data.restitution * jv(dir, g1, g2, velocity(b1), velocity(b2))
    } else {
        ZERO
    };
    let (g1, g2, ig1, ig2, r) = coefficients(tangent(dir), dp1, dp2, b1, b2);
    let t = ContactConstraintTangentPart {
        gcross1: g1,
        gcross2: g2,
        ii_gcross1: ig1,
        ii_gcross2: ig2,
        r,
        impulse: ti,
        impulse_accumulator: -ti,
        ..Default::default(),
    };
    // Both local anchors freeze the same world point, so the base separation is `sc.dist`.
    ContactConstraintElement {
        normal_part: n,
        tangent_part: t,
        local_p1: local_anchor(b1, world1, point, dp1),
        local_p2: local_anchor(b2, world2, point, dp2),
        dist: sc.dist,
        restitution_seed: seed,
        contact_id: cid.try_into().unwrap(),
        tangent_velocity: sc.tangent_velocity,
    }
}

/// Upstream's frozen solver point (`pair_update.rs`, "Localize solver contacts"): the first
/// witness slides along the normal until the pair is exactly `sc.dist` apart, then both witnesses
/// meet halfway. `com*` are the original centres of mass the anchors are relative to. One floor
/// per component: `floor((wp1 + wp2 - dir * shift) * HALF)`, `shift` itself floored once.
pub(crate) fn midpoint(sc: SolverContact, dir: Vec2, com1: Vec2, com2: Vec2) -> Vec2 {
    let wp1 = com1 + sc.anchor1;
    let wp2 = com2 + sc.anchor2;
    let d = wp1 - wp2;
    let shift = dot2(d.x, dir.x, d.y, dir.y) - sc.dist;
    let s = wp1 + wp2;
    Vec2 {
        x: wide_from(s.x).sub(wide_mul(dir.x, shift)).mul(HALF).narrow(),
        y: wide_from(s.y).sub(wide_mul(dir.y, shift)).mul(HALF).narrow(),
    }
}

/// The frozen point in the solver body's frame: world coordinates for the world (identity
/// pose), else `R^T * dp`, equal to `inverse_transform_point(point)` because a non-world solver
/// body is the original one (translation = centre of mass) and saves the subtraction.
#[inline(always)]
fn local_anchor(b: SolverBody, world: bool, point: Vec2, dp: Vec2) -> Vec2 {
    if world {
        point
    } else {
        b.position.rotation.inverse_rotate(dp)
    }
}

fn update_element(
    ref e: ContactConstraintElement,
    c: ContactConstraint,
    p1: Pose2,
    p2: Pose2,
    warm: Fixed,
    cap: Fixed,
) {
    let dp = p1.transform_point(e.local_p1) - p2.transform_point(e.local_p2);
    let dist = e.dist + dot(dp, c.dir1);
    e.normal_part.rhs_wo_bias = max(ZERO, dist) * c.inv_dt;
    e.normal_part.rhs = e.normal_part.rhs_wo_bias + min(ZERO, max(-cap, dist * c.erp_inv_dt));
    e.normal_part.cfm_factor = if dist > ZERO {
        ONE
    } else {
        c.soft_cfm_factor
    };
    e.normal_part.impulse_accumulator += e.normal_part.impulse;
    e.normal_part.impulse = e.normal_part.impulse * warm;
    e.tangent_part.impulse_accumulator += e.tangent_part.impulse;
    e.tangent_part.impulse = e.tangent_part.impulse * warm;
    e.tangent_part.rhs = e.tangent_part.rhs_wo_bias + dot(dp, tangent(c.dir1)) * c.inv_dt;
}
fn refresh_unbiased(ref e: ContactConstraintElement, c: ContactConstraint, p1: Pose2, p2: Pose2) {
    let dp = p1.transform_point(e.local_p1) - p2.transform_point(e.local_p2);
    e.normal_part.rhs_wo_bias = max(ZERO, e.dist + dot(dp, c.dir1)) * c.inv_dt;
}
fn strip(ref e: ContactConstraintElement) {
    e.normal_part.rhs = e.normal_part.rhs_wo_bias;
    e.normal_part.cfm_factor = ONE;
    e.tangent_part.rhs = e.tangent_part.rhs_wo_bias;
}
fn warm_normal(
    p: ContactConstraintNormalPart, c: ContactConstraint, ref v1: SolverVel, ref v2: SolverVel,
) {
    apply(c.dir1, c.im1, c.im2, p.ii_gcross1, p.ii_gcross2, p.impulse, ref v1, ref v2);
}
fn warm_tangent(
    p: ContactConstraintTangentPart, c: ContactConstraint, ref v1: SolverVel, ref v2: SolverVel,
) {
    apply(tangent(c.dir1), c.im1, c.im2, p.ii_gcross1, p.ii_gcross2, p.impulse, ref v1, ref v2);
}
pub(crate) fn writeback(c: ContactConstraint, ref m: ContactManifold) {
    let [a, b] = c.elements;
    write_point(a, ref m);
    if c.num_elements == 2 {
        write_point(b, ref m);
    }
}
fn write_point(e: ContactConstraintElement, ref m: ContactManifold) {
    let [mut p0, mut p1] = m.points;
    let mut p = if e.contact_id == 0 {
        p0
    } else {
        p1
    };
    p.data.impulse = e.normal_part.total_impulse();
    p.data.tangent_impulse = e.tangent_part.total_impulse();
    p.data.warmstart_impulse = e.normal_part.impulse;
    p.data.warmstart_tangent_impulse = e.tangent_part.impulse;
    if e.contact_id == 0 {
        p0 = p;
    } else {
        p1 = p;
    }
    m.points = [p0, p1];
}

#[cfg(test)]
mod alternatives;
#[cfg(test)]
mod benches;

pub(crate) mod cached;
#[cfg(test)]
mod checks;
#[cfg(test)]
mod fixtures;
