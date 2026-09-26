//! Direct generation of the split constraints (BT3): each manifold goes straight to its
//! `Frozen` / `Hot` pair, without the intermediate `ContactConstraint` and its two elements
//! (`contact::generate_cached` then `push`, kept as `alternatives::generate_via_constraints`).
//! Same checks in the same order, same operations on the same operands (`contact::midpoint`,
//! `element::coefficients` and `contact::local_anchor` written out on the scalars they read),
//! same values: the constraint-set path stays the reference of the split tests.
//!
//! What the loop carries is kept small (Cairo steps are mostly copies here): endpoints resolve
//! through a slot index instead of scanning the bodies, a body contributes only the scalars
//! generation reads (not four `SolverBody` copies), the manifold is read through its snapshot,
//! and the softness cache holds the two spring configurations instead of the whole
//! `IntegrationParameters`.
use core::dict::{Felt252Dict, Felt252DictTrait};
use core::num::traits::DivRem;
use fixed::wide::{WideMul, WideNarrow, WideSub, dot2, dot4, mul_sub, wide_from, wide_mul};
use fixed::{Fixed, HALF, ZERO};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_core::integration_parameters::spring::{SpringCoefficients, SpringCoefficientsTrait};
use rapier_geometry2d::contact::{ContactManifold, NEW_CONTACT_BIT, SolverContact, TrackedContact};
use rapier_math::math_ext::inv;
use rapier_math::rot2::{Rot2, Rot2Trait};
use super::super::super::super::body::{SolverBody, SolverVel, WORLD};
use super::super::super::super::contact::element::{jv, tangent};
use super::super::super::super::contact::errors;
use super::{Bank, BankPoint, Frozen, FrozenPoint, Hot, HotPoint, Row, State, Weights};

/// `ContactConstraintsSetTrait::generate` then the split of every active constraint, in one
/// pass, with the step's constants computed once: same constraints, checks and panics as
/// `contact::generate_cached` with its `contact::SoftCache`.
pub(crate) fn generate(
    mut manifolds: Span<ContactManifold>,
    bodies: Span<SolverBody>,
    params: IntegrationParameters,
    dt: Fixed,
) -> (Array<Frozen>, State) {
    let inv_dt = inv(dt);
    let mut soft = Soft {
        dynamic: params.contact_softness,
        fixed: params.static_contact_softness,
        dt,
        dynamic_pair: None,
        fixed_pair: None,
    };
    let mut frozen = array![];
    let mut hot = array![];
    let mut bank = array![];
    let mut index = body_index(bodies);
    let mut id = 0;
    while let Some(m) = manifolds.pop_front() {
        split_manifold(
            m, bodies, dt, inv_dt, ref soft, ref index, id, ref frozen, ref hot, ref bank,
        );
        id += 1;
    }
    (frozen, State { hot, bank })
}

/// `contact::SoftCache` without the parameters: the softness `(erp_inv_dt, cfm_factor)` of the
/// dynamic and of the static contacts, computed at first use.
#[derive(Copy, Drop)]
struct Soft {
    dynamic: SpringCoefficients,
    fixed: SpringCoefficients,
    dt: Fixed,
    dynamic_pair: Option<(Fixed, Fixed)>,
    fixed_pair: Option<(Fixed, Fixed)>,
}

#[generate_trait]
impl SoftImpl of SoftTrait {
    /// `SoftCache::get`: `static_contact_softness` (a world endpoint) or `contact_softness`.
    #[inline(always)]
    fn get(ref self: Soft, world: bool) -> (Fixed, Fixed) {
        if world {
            if let Some(pair) = self.fixed_pair {
                return pair;
            }
            let c = self.fixed.coefficients(self.dt);
            self.fixed_pair = Some((c.erp_inv_dt, c.cfm_factor));
            (c.erp_inv_dt, c.cfm_factor)
        } else {
            if let Some(pair) = self.dynamic_pair {
                return pair;
            }
            let c = self.dynamic.coefficients(self.dt);
            self.dynamic_pair = Some((c.erp_inv_dt, c.cfm_factor));
            (c.erp_inv_dt, c.cfm_factor)
        }
    }
}

/// What generation reads from a solver endpoint: the constraint's body (`WORLD`: zero masses,
/// identity rotation, zero velocity) and the original body's centre of mass.
#[derive(Copy, Drop)]
struct End {
    im: Vec2,
    ii: Fixed,
    rotation: Rot2,
    vel: SolverVel,
    com: Vec2,
    world: bool,
}

/// The endpoint of dense id `raw` (`WORLD` for none), `WORLD` as the solver body when
/// `dominated`. Checks the original body's masses first (`contact::validate_mass`).
#[inline(always)]
fn end(bodies: Span<SolverBody>, raw: u32, dominated: bool) -> End {
    if raw == WORLD {
        return End {
            im: Default::default(),
            ii: ZERO,
            rotation: Default::default(),
            vel: Default::default(),
            com: Default::default(),
            world: true,
        };
    }
    let b = bodies.at(raw);
    let im = *b.im;
    let ii = *b.ii;
    assert(im.x >= ZERO && im.y >= ZERO && ii >= ZERO, errors::NEGATIVE);
    let com = *b.position.translation;
    if dominated {
        return End {
            im: Default::default(),
            ii: ZERO,
            rotation: Default::default(),
            vel: Default::default(),
            com,
            world: true,
        };
    }
    End {
        im,
        ii,
        rotation: *b.position.rotation,
        vel: SolverVel { linear: *b.linvel, angular: *b.angvel },
        com,
        world: false,
    }
}

/// `generate_cached` + `push` for one manifold: appends nothing for an inactive one.
#[inline(always)]
fn split_manifold(
    m: @ContactManifold,
    bodies: Span<SolverBody>,
    dt: Fixed,
    inv_dt: Fixed,
    ref soft: Soft,
    ref index: Felt252Dict<u32>,
    manifold_id: u32,
    ref frozen: Array<Frozen>,
    ref hot: Array<Hot>,
    ref bank: Array<Bank>,
) {
    let count = *m.data.num_solver_contacts;
    let num_points = *m.num_points;
    assert(num_points <= 2 && count <= num_points, errors::COUNT);
    let (_, enabled) = DivRem::div_rem(*m.data.solver_flags.bits, 2);
    if count == 0 || enabled == 0 {
        return;
    }
    let (friction, restitution) = (*m.data.friction, *m.data.restitution);
    assert(dt >= ZERO && friction >= ZERO && restitution >= ZERO, errors::NEGATIVE);
    let raw1 = resolve(ref index, bodies, *m.data.rigid_body1);
    let raw2 = resolve(ref index, bodies, *m.data.rigid_body2);
    assert(raw1 == WORLD || raw2 == WORLD || raw1 != raw2, errors::SAME_BODY);
    let dominance = *m.data.relative_dominance;
    let e1 = end(bodies, raw1, dominance > 0);
    let e2 = end(bodies, raw2, dominance < 0);
    let i = if e1.world {
        WORLD
    } else {
        raw1
    };
    let j = if e2.world {
        WORLD
    } else {
        raw2
    };
    let (erp_inv_dt, soft_cfm) = soft.get(e1.world || e2.world);
    let dir = -*m.data.normal;
    let t = tangent(dir);
    let im_sum = e1.im + e2.im;
    let [sc0, sc1] = *m.data.solver_contacts;
    let [p0, p1] = *m.points;
    let (fa, ha, ba, cid0) = split_point(
        sc0, num_points, p0, p1, restitution, dir, t, im_sum, e1, e2,
    );
    let (fb, hb, bb) = if count == 2 {
        let (fb, hb, bb, cid1) = split_point(
            sc1, num_points, p0, p1, restitution, dir, t, im_sum, e1, e2,
        );
        assert(cid1 != cid0, errors::CONTACT_ID);
        (fb, hb, bb)
    } else {
        (Default::default(), Default::default(), Default::default())
    };
    // Floored products first, then negated (the negation of `dir * im2` exactly).
    let (wn2, wt2) = (dir * e2.im, t * e2.im);
    frozen
        .append(
            Frozen {
                i,
                j,
                dir,
                t,
                wn: Weights { first: dir * e1.im, neg_second: -wn2 },
                wt: Weights { first: t * e1.im, neg_second: -wt2 },
                limit: friction,
                count,
                manifold_id,
                inv_dt,
                erp_inv_dt,
                soft_cfm,
                a: fa,
                b: fb,
            },
        );
    hot.append(Hot { a: ha, b: hb });
    bank.append(Bank { a: ba, b: bb });
}

/// Dense id + 1 of each body, by handle slot (BT3: `contact::resolve` scans the bodies for
/// each endpoint of each manifold). Slots are unique in a body set.
fn body_index(mut bodies: Span<SolverBody>) -> Felt252Dict<u32> {
    let mut index: Felt252Dict<u32> = Default::default();
    let mut id: u32 = 1;
    while let Some(b) = bodies.pop_front() {
        index.insert((*b.handle.index).into(), id);
        id += 1;
    }
    index
}

/// `contact::resolve` through the index: the dense id of `handle` (same slot and generation),
/// `WORLD` for `None`; panics with `errors::BODY` when the body is absent.
#[inline(always)]
fn resolve(ref index: Felt252Dict<u32>, bodies: Span<SolverBody>, handle: Option<Handle>) -> u32 {
    let Some(h) = handle else {
        return WORLD;
    };
    let id = index.get(h.index.into());
    assert(id != 0 && *bodies.at(id - 1).handle == h, errors::BODY);
    id - 1
}

/// `element::coefficients` on the scalars it reads (`im_sum = b1.im + b2.im`): lever-arm
/// crosses, inertia-weighted crosses and the inverse projected mass (`inv`, zero for zero).
#[inline(always)]
fn coefficients(
    dir: Vec2, a1: Vec2, a2: Vec2, im_sum: Vec2, ii1: Fixed, ii2: Fixed,
) -> (Fixed, Fixed, Fixed, Fixed, Fixed) {
    let g1 = mul_sub(a1.x, dir.y, a1.y, dir.x);
    let g2 = mul_sub(a2.x, -dir.y, a2.y, -dir.x);
    let ig1 = ii1 * g1;
    let ig2 = ii2 * g2;
    let mass_dir = im_sum * dir;
    let k = dot4(dir.x, mass_dir.x, dir.y, mass_dir.y, g1, ig1, g2, ig2);
    (g1, g2, ig1, ig2, inv(k))
}

/// `contact::midpoint`: the first witness slid along the normal until the pair is exactly
/// `sc.dist` apart, both witnesses meeting halfway; one floor per component.
#[inline(always)]
fn midpoint(sc: SolverContact, dir: Vec2, com1: Vec2, com2: Vec2) -> Vec2 {
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

/// `contact::local_anchor`: the world point for a world endpoint, else `R^T * dp`.
#[inline(always)]
fn local_anchor(e: End, point: Vec2, dp: Vec2) -> Vec2 {
    if e.world {
        point
    } else {
        e.rotation.inverse_rotate(dp)
    }
}

/// `generate_element` then `frozen_point` / `hot_point`; also returns the tracked point id.
#[inline(always)]
fn split_point(
    sc: SolverContact,
    num_points: u8,
    p0: TrackedContact,
    p1: TrackedContact,
    restitution: Fixed,
    dir: Vec2,
    t: Vec2,
    im_sum: Vec2,
    e1: End,
    e2: End,
) -> (FrozenPoint, HotPoint, BankPoint, u8) {
    let is_new = sc.contact_id >= NEW_CONTACT_BIT;
    let cid = if is_new {
        sc.contact_id - NEW_CONTACT_BIT
    } else {
        sc.contact_id
    };
    assert(cid < num_points.into(), errors::CONTACT_ID);
    let (ni, ti) = if is_new {
        (ZERO, ZERO)
    } else if cid == 0 {
        (p0.data.warmstart_impulse, p0.data.warmstart_tangent_impulse)
    } else {
        (p1.data.warmstart_impulse, p1.data.warmstart_tangent_impulse)
    };
    assert(ni >= ZERO, errors::NEGATIVE);
    let point = midpoint(sc, dir, e1.com, e2.com);
    let dp1 = point - e1.com;
    let dp2 = point - e2.com;
    let (g1, g2, ig1, ig2, r) = coefficients(dir, dp1, dp2, im_sum, e1.ii, e2.ii);
    let seed = if is_new {
        restitution * jv(dir, g1, g2, e1.vel, e2.vel)
    } else {
        ZERO
    };
    let (tg1, tg2, tig1, tig2, tr) = coefficients(t, dp1, dp2, im_sum, e1.ii, e2.ii);
    let cid: u8 = cid.try_into().unwrap();
    (
        FrozenPoint {
            n: Row { g1, g2, ig1, ig2, r },
            t: Row { g1: tg1, g2: tg2, ig1: tig1, ig2: tig2, r: tr },
            local_p1: local_anchor(e1, point, dp1),
            local_p2: local_anchor(e2, point, dp2),
            dist: sc.dist,
            t_rhs_wo_bias: ZERO,
            seed,
            contact_id: cid,
        },
        HotPoint { impulse: ni, rhs: ZERO, cfm: ZERO, t_impulse: ti, t_rhs: ZERO },
        BankPoint { acc: -ni, t_acc: -ti, dist: ZERO, t_dist: ZERO },
        cid,
    )
}
