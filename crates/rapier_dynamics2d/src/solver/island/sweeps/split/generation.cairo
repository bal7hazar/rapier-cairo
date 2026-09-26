//! Direct generation of the split constraints (BT3): each manifold goes straight to its
//! `Frozen` / `Hot` pair, without the intermediate `ContactConstraint` and its two elements
//! (`contact::generate_cached` then `push`, kept as `alternatives::generate_via_constraints`).
//! Same checks in the same order, same kernels (`midpoint`, `coefficients`, `local_anchor`),
//! same values: the constraint-set path stays the reference of the split tests.
use core::dict::{Felt252Dict, Felt252DictTrait};
use core::num::traits::DivRem;
use fixed::{Fixed, ZERO};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldTrait, NEW_CONTACT_BIT};
use super::super::super::super::body::{SolverBody, WORLD, read, velocity};
use super::super::super::super::contact::element::{coefficients, jv, tangent};
use super::super::super::super::contact::{
    SoftCache, SoftCacheTrait, errors, local_anchor, midpoint, validate_mass,
};
use super::{Frozen, FrozenPoint, Hot, HotPoint, Row, Weights};

/// `ContactConstraintsSetTrait::generate` then the split of every active constraint, in one
/// pass, with the step's constants computed once (`contact::SoftCache`): same constraints,
/// checks and panics as `contact::generate_cached`.
pub(crate) fn generate(
    mut manifolds: Span<ContactManifold>,
    bodies: Span<SolverBody>,
    params: IntegrationParameters,
    dt: Fixed,
) -> (Array<Frozen>, Array<Hot>) {
    let mut cache = SoftCacheTrait::new(params, dt);
    let mut frozen = array![];
    let mut hot = array![];
    let mut index = body_index(bodies);
    let mut id = 0;
    while let Some(m) = manifolds.pop_front() {
        split_manifold(*m, bodies, dt, ref cache, ref index, id, ref frozen, ref hot);
        id += 1;
    }
    (frozen, hot)
}

/// `generate_cached` + `push` for one manifold: appends nothing for an inactive one.
#[inline(always)]
fn split_manifold(
    m: ContactManifold,
    bodies: Span<SolverBody>,
    dt: Fixed,
    ref cache: SoftCache,
    ref index: Felt252Dict<u32>,
    manifold_id: u32,
    ref frozen: Array<Frozen>,
    ref hot: Array<Hot>,
) {
    let count = m.data.num_solver_contacts;
    assert(m.num_points <= 2 && count <= m.num_points, errors::COUNT);
    let (_, enabled) = DivRem::div_rem(m.data.solver_flags.bits, 2);
    if count == 0 || enabled == 0 {
        return;
    }
    assert(dt >= ZERO && m.data.friction >= ZERO && m.data.restitution >= ZERO, errors::NEGATIVE);
    let raw1 = resolve(ref index, bodies, m.data.rigid_body1);
    let raw2 = resolve(ref index, bodies, m.data.rigid_body2);
    assert(raw1 == WORLD || raw2 == WORLD || raw1 != raw2, errors::SAME_BODY);
    let original1 = read(bodies, raw1);
    let original2 = read(bodies, raw2);
    validate_mass(original1);
    validate_mass(original2);
    let i = if m.data.relative_dominance > 0 {
        WORLD
    } else {
        raw1
    };
    let j = if m.data.relative_dominance < 0 {
        WORLD
    } else {
        raw2
    };
    let b1 = if i == raw1 {
        original1
    } else {
        Default::default()
    };
    let b2 = if j == raw2 {
        original2
    } else {
        Default::default()
    };
    let soft = cache.get(i == WORLD || j == WORLD);
    let dir = -m.data.normal;
    let t = tangent(dir);
    let [sc0, sc1] = m.data.solver_contacts;
    let ends = Ends {
        b1, b2, com1: original1.position.translation, com2: original2.position.translation,
    };
    let (fa, ha, cid0) = split_point(sc0, m, dir, t, ends, i == WORLD, j == WORLD);
    let (fb, hb) = if count == 2 {
        let (fb, hb, cid1) = split_point(sc1, m, dir, t, ends, i == WORLD, j == WORLD);
        assert(cid1 != cid0, errors::CONTACT_ID);
        (fb, hb)
    } else {
        (Default::default(), Default::default())
    };
    // Floored products first, then negated (the negation of `dir * im2` exactly).
    let (wn2, wt2) = (dir * b2.im, t * b2.im);
    frozen
        .append(
            Frozen {
                i,
                j,
                dir,
                t,
                wn: Weights { first: dir * b1.im, neg_second: -wn2 },
                wt: Weights { first: t * b1.im, neg_second: -wt2 },
                limit: m.data.friction,
                count,
                manifold_id,
                inv_dt: cache.inv_dt,
                erp_inv_dt: soft.erp_inv_dt,
                soft_cfm: soft.cfm_factor,
                a: fa,
                b: fb,
            },
        );
    hot.append(Hot { a: ha, b: hb });
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

/// The solver bodies of a constraint (`WORLD` reads as the default body) and the original
/// centres of mass the frozen point is built from.
#[derive(Copy, Drop)]
struct Ends {
    b1: SolverBody,
    b2: SolverBody,
    com1: Vec2,
    com2: Vec2,
}

/// `generate_element` then `frozen_point` / `hot_point`; also returns the tracked point id.
#[inline(always)]
fn split_point(
    sc: rapier_geometry2d::contact::SolverContact,
    m: ContactManifold,
    dir: Vec2,
    t: Vec2,
    e: Ends,
    world1: bool,
    world2: bool,
) -> (FrozenPoint, HotPoint, u8) {
    let is_new = sc.contact_id >= NEW_CONTACT_BIT;
    let cid = if is_new {
        sc.contact_id - NEW_CONTACT_BIT
    } else {
        sc.contact_id
    };
    assert(cid < m.num_points.into(), errors::CONTACT_ID);
    let data = m.point(cid.try_into().unwrap()).data;
    let (ni, ti) = if is_new {
        (ZERO, ZERO)
    } else {
        (data.warmstart_impulse, data.warmstart_tangent_impulse)
    };
    assert(ni >= ZERO, errors::NEGATIVE);
    let point = midpoint(sc, dir, e.com1, e.com2);
    let dp1 = point - e.com1;
    let dp2 = point - e.com2;
    let (g1, g2, ig1, ig2, r) = coefficients(dir, dp1, dp2, e.b1, e.b2);
    let seed = if is_new {
        m.data.restitution * jv(dir, g1, g2, velocity(e.b1), velocity(e.b2))
    } else {
        ZERO
    };
    let (tg1, tg2, tig1, tig2, tr) = coefficients(t, dp1, dp2, e.b1, e.b2);
    let cid: u8 = cid.try_into().unwrap();
    (
        FrozenPoint {
            n: Row { g1, g2, ig1, ig2, r },
            t: Row { g1: tg1, g2: tg2, ig1: tig1, ig2: tig2, r: tr },
            local_p1: local_anchor(e.b1, world1, point, dp1),
            local_p2: local_anchor(e.b2, world2, point, dp2),
            dist: sc.dist,
            t_rhs_wo_bias: ZERO,
            seed,
            contact_id: cid,
        },
        HotPoint {
            impulse: ni,
            rhs: ZERO,
            cfm: ZERO,
            t_impulse: ti,
            t_rhs: ZERO,
            acc: -ni,
            t_acc: -ti,
            dist: ZERO,
            t_dist: ZERO,
        },
        cid,
    )
}
