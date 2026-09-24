//! Lifecycle, input-contract and numerical regression checks.
#[cfg(test)]
mod tests {
    use fixed::{FixedTrait, HALF, MAX, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_core::data::handle::Handle;
    use rapier_core::integration_parameters::IntegrationParametersTrait;
    use rapier_geometry2d::contact::{ContactManifoldTrait, NEW_CONTACT_BIT, SolverContact};
    use rapier_math::math_ext::gcross_vv;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use super::super::fixtures::{fixture, prepared};
    use super::super::{
        ContactConstraintNormalPartTrait, ContactConstraintTrait, alternatives, midpoint,
    };

    fn raw(x: i64, y: i64) -> Vec2 {
        Vec2 { x: FixedTrait::from_raw(x), y: FixedTrait::from_raw(y) }
    }

    // Upstream `pair_update.rs`: shift = (wp2 - wp1).n - dist, p1 = wp1 + n shift, point =
    // (p1 + wp2) / 2. Normal (0, 1), so dir1 = (0, -1). Rows: wp1, wp2 (world), dist, point.
    #[test]
    fn test_midpoint_matches_upstream_localization() {
        let h = 2147483648_i64;
        let one = 2 * h;
        let mut cases = array![
            ((0, 0), (0, one), one, (0, h)), // consistent pair: plain midpoint
            ((0, 0), (858993459, one), one, (429496729, h)), // tangential offset halves (floor)
            ((0, 0), (0, one), h, (0, 3 * h / 2)), // dist wins: witness 1 slides to 0.5
            ((one, -h), (one, h), -one, (one, one)) // penetration: shift = 2
        ]
            .span();
        let dir = raw(0, -one);
        let com2 = raw(h, -one);
        while let Some(((x1, y1), (x2, y2), dist, (px, py))) = cases.pop_front() {
            let wp2 = raw(*x2, *y2);
            let sc = SolverContact {
                anchor1: raw(*x1, *y1),
                anchor2: wp2 - com2,
                dist: FixedTrait::from_raw(*dist),
                ..Default::default(),
            };
            let expected = raw(*px, *py);
            assert_eq!(midpoint(sc, dir, Default::default(), com2), expected);
            assert_eq!(
                alternatives::midpoint_two_stage(sc, dir, Default::default(), com2), expected,
            );
        }
    }

    // Both local anchors freeze the common midpoint; the base separation is the contact's.
    #[test]
    fn test_generated_anchors_share_the_midpoint() {
        let (mut m, mut bs, p) = fixture(1);
        let mut b = bs.pop_front().unwrap();
        b
            .position =
                Pose2 {
                    translation: raw(858993459, 4294967296),
                    rotation: Rot2 {
                        re: FixedTrait::from_raw(3719550787), im: FixedTrait::from_raw(2147483648),
                    },
                };
        bs.append(b);
        let [mut sc, sc1] = m.data.solver_contacts;
        sc.anchor1 = raw(0, 21474836);
        sc.anchor2 = raw(-2147483648, -4294967296);
        sc.dist = FixedTrait::from_raw(-21474836);
        m.data.solver_contacts = [sc, sc1];
        let c = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
        let [e, _] = c.elements;
        let point = midpoint(sc, c.dir1, Default::default(), b.position.translation);
        assert_eq!(e.local_p1, point);
        let back = b.position.transform_point(e.local_p2) - point;
        assert!(back.x.abs() <= FixedTrait::from_raw(4) && back.y.abs() <= FixedTrait::from_raw(4));
        assert_eq!(e.dist, sc.dist);
        let arm = point - b.position.translation;
        assert_eq!(e.normal_part.gcross2, gcross_vv(arm.x, arm.y, -c.dir1.x, -c.dir1.y));
    }

    #[test]
    #[fuzzer(runs: 32, seed: 20260922)]
    fn fuzz_midpoint_candidates_match(ax: i16, ay: i16, bx: i16, by: i16, dist: i16) {
        let sc = SolverContact {
            anchor1: raw(ax.into() * 65537, ay.into() * 65539),
            anchor2: raw(bx.into() * 65541, by.into() * 65543),
            dist: FixedTrait::from_raw(dist.into() * 4099),
            ..Default::default(),
        };
        let dir = raw(-2147483648, -3719550787);
        let com1 = raw(ay.into() * 131071, 7);
        let com2 = raw(-3, bx.into() * 131073);
        assert_eq!(
            midpoint(sc, dir, com1, com2), alternatives::midpoint_two_stage(sc, dir, com1, com2),
        );
    }

    #[test]
    fn gas_baseline() {
        let _ = rapier_testing::opaque(ONE);
    }

    #[test]
    fn test_empty_disabled_and_zero_mass() {
        let (mut m, bs, p) = fixture(0);
        let mut c = ContactConstraintTrait::generate(m, bs.span(), p, ZERO);
        let mut empty = array![];
        c.update(p, empty.span(), m);
        c.warmstart(ref empty);
        c.solve(ref empty, true, true);
        c.update_rhs_wo_bias(empty.span());
        c.apply_restitution(ref empty);
        let mut ms = array![];
        c.writeback_impulses(ref ms);
        assert_eq!(empty.len(), 0);
        m.num_points = 1;
        m.data.num_solver_contacts = 1;
        m.data.solver_flags.bits = 2;
        assert_eq!(ContactConstraintTrait::generate(m, empty.span(), p, ZERO).num_elements, 0);
        m.data.solver_flags.bits = 1;
        m.data.rigid_body2 = None;
        let c = ContactConstraintTrait::generate(m, empty.span(), p, p.substep_dt());
        let [a, _] = c.elements;
        assert_eq!(a.normal_part.r, ZERO);
        assert_eq!(a.tangent_part.r, ZERO);
    }

    #[test]
    fn test_new_contact_gate_and_banked_impulses() {
        let (mut m, bs, p) = fixture(1);
        let [mut pt, spare] = m.points;
        pt.data.warmstart_impulse = TWO;
        pt.data.warmstart_tangent_impulse = -ONE;
        m.points = [pt, spare];
        m.data.restitution = ONE;
        let mut cases = array![false, true].span();
        while let Some(new_contact) = cases.pop_front() {
            let [mut sc, sc1] = m.data.solver_contacts;
            sc.contact_id = if *new_contact {
                NEW_CONTACT_BIT
            } else {
                0
            };
            m.data.solver_contacts = [sc, sc1];
            let mut c = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
            let [a, _] = c.elements;
            assert_eq!(a.normal_part.total_impulse(), ZERO);
            assert_eq!(a.normal_part.impulse, if *new_contact {
                ZERO
            } else {
                TWO
            });
            assert_eq!(a.restitution_seed < ZERO, *new_contact);
            c.update(p, bs.span(), m);
            let [a, _] = c.elements;
            assert_eq!(a.normal_part.impulse_accumulator, ZERO);
            c.update(p, bs.span(), m);
            let [b, _] = c.elements;
            assert_eq!(b.normal_part.impulse_accumulator, a.normal_part.impulse);
        }
    }

    #[test]
    fn test_refresh_rhs_after_integration_and_static_softness() {
        let (m, mut bs, p) = fixture(1);
        let mut c = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
        assert_eq!(c.soft_cfm_factor, p.static_contact_softness_coefficients().cfm_factor);
        c.update(p, bs.span(), m);
        let [a, _] = c.elements;
        assert!(a.normal_part.rhs < ZERO);
        let mut b = bs.pop_front().unwrap();
        b.position.translation.y = ONE;
        bs.append(b);
        c.update_rhs_wo_bias(bs.span());
        let [a, _] = c.elements;
        assert!(a.normal_part.rhs > ONE);
        assert_eq!(a.normal_part.cfm_factor, ONE);
        assert_eq!(a.tangent_part.rhs, ZERO);
        c.update(p, bs.span(), m);
        let [a, _] = c.elements;
        assert_eq!(a.normal_part.cfm_factor, ONE);
        assert_eq!(a.normal_part.rhs, a.normal_part.rhs_wo_bias);
    }

    #[test]
    fn test_writeback_uses_contact_ids_and_preserves_manifolds() {
        let (mut m, bs, p) = fixture(2);
        let [mut sc0, mut sc1] = m.data.solver_contacts;
        sc0.contact_id = 1;
        sc1.contact_id = 0;
        m.data.solver_contacts = [sc0, sc1];
        let mut c = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
        let [mut a, mut b] = c.elements;
        a.normal_part.impulse = ONE;
        a.normal_part.impulse_accumulator = TWO;
        b.normal_part.impulse = HALF;
        c.elements = [a, b];
        c.manifold_id = 1;
        let mut ms = array![m, m];
        c.writeback_impulses(ref ms);
        assert_eq!(*ms.at(0), m);
        assert_eq!(ms.at(1).point(1).data.impulse, ONE + TWO);
        assert_eq!(ms.at(1).point(0).data.warmstart_impulse, HALF);
        assert_eq!(*ms.at(1).data, m.data);
    }

    #[test]
    fn test_generation_handles_dominance_and_directional_mass() {
        let (mut m, mut bs, p) = fixture(1);
        let mut b2 = bs.pop_front().unwrap();
        let mut b1 = b2;
        b1.handle = Handle { index: 99, generation: 7 };
        b1.im = Vec2 { x: ZERO, y: TWO };
        bs = array![b2, b1];
        m.data.rigid_body1 = Some(b1.handle);
        let c = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
        assert_eq!(c.solver_vel1, 1);
        assert_eq!(c.solver_vel2, 0);
        assert_eq!(c.soft_cfm_factor, p.contact_softness_coefficients().cfm_factor);
        m.data.relative_dominance = 1;
        let c = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
        assert_eq!(c.solver_vel1, 0xffffffff);
        assert_eq!(c.solver_vel2, 0);
        m.data.relative_dominance = -1;
        let c = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
        assert_eq!(c.solver_vel1, 1);
        assert_eq!(c.solver_vel2, 0xffffffff);
        b2.im = Default::default();
        b2.ii = ZERO;
        let c = ContactConstraintTrait::generate(m, array![b1, b2].span(), p, p.substep_dt());
        assert_eq!(c.solver_vel2, 0xffffffff);
    }

    /// Equal-dominance zero-mass endpoints remain real, even at rest. This also
    /// selects upstream's two-body softness instead of the world-contact spring.
    #[test]
    fn test_kinematic_endpoints_keep_identity_and_velocity() {
        let (mut m, mut bs, p) = fixture(1);
        let dynamic = bs.pop_front().unwrap();
        let mut kine = dynamic;
        kine.handle.index = 99;
        kine.im = Default::default();
        kine.ii = ZERO;
        m.data.rigid_body1 = Some(kine.handle);
        for speed in array![ZERO, ONE, -ONE] {
            kine.linvel = Vec2 { x: speed, y: speed };
            kine.angvel = speed;
            let bs = array![kine, dynamic];
            let c = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
            assert_eq!(c.solver_vel1, 0);
            assert_eq!(c.solver_vel2, 1);
            assert_eq!(c.im1, Default::default());
            assert_eq!(c.soft_cfm_factor, p.contact_softness_coefficients().cfm_factor);
            let mut swapped = m;
            swapped.data.rigid_body1 = m.data.rigid_body2;
            swapped.data.rigid_body2 = m.data.rigid_body1;
            let c = ContactConstraintTrait::generate(swapped, bs.span(), p, p.substep_dt());
            assert_eq!(c.solver_vel1, 1);
            assert_eq!(c.solver_vel2, 0);
        }
    }

    #[test]
    fn test_separating_contact_does_not_bounce_or_attract() {
        let (mut m, mut bs, p) = fixture(1);
        let mut body = bs.pop_front().unwrap();
        body.linvel.y = ONE;
        bs.append(body);
        m.data.restitution = ONE;
        let [mut sc, sc1] = m.data.solver_contacts;
        sc.contact_id = NEW_CONTACT_BIT;
        sc.dist = ONE;
        m.data.solver_contacts = [sc, sc1];
        let mut c = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
        c.update(p, bs.span(), m);
        c.solve(ref bs, true, true);
        c.apply_restitution(ref bs);
        assert_eq!(*bs.at(0), body);
        let [a, _] = c.elements;
        assert_eq!(a.normal_part.impulse, ZERO);
    }

    #[test]
    #[fuzzer(runs: 32, seed: 20260920)]
    fn fuzz_gathered_matches_direct(vx: i16, vy: i16, warm: u8) {
        let mut counts = array![1_u8, 2_u8].span();
        while let Some(n) = counts.pop_front() {
            let mut flags = array![(true, true), (true, false), (false, true), (false, false)]
                .span();
            while let Some(flags) = flags.pop_front() {
                let (mut c, mut bs) = prepared(*n);
                let mut b = bs.pop_front().unwrap();
                b.linvel.x = FixedTrait::from_raw(vx.into() * 1048576);
                b.linvel.y = FixedTrait::from_raw(vy.into() * 1048576);
                bs.append(b);
                let [mut a, mut a1] = c.elements;
                a.normal_part.impulse = FixedTrait::from_raw(warm.into() * 1048576);
                a1.normal_part.impulse = a.normal_part.impulse;
                c.elements = [a, a1];
                let mut direct = c;
                let mut ds = array![b];
                let (normal, friction) = *flags;
                c.solve(ref bs, normal, friction);
                alternatives::solve(ref direct, ref ds, normal, friction);
                assert_eq!(c, direct);
                assert_eq!(*bs.at(0), *ds.at(0));
            }
        }
    }

    #[test]
    #[fuzzer(runs: 32, seed: 20260920)]
    fn fuzz_rigid_contact_does_not_add_energy(vx: i16, vy: i16) {
        let (mut c, mut bs) = prepared(1);
        let mut b = bs.pop_front().unwrap();
        b.linvel.x = FixedTrait::from_raw(vx.into() * 1048576);
        b.linvel.y = FixedTrait::from_raw(vy.into() * 1048576);
        bs.append(b);
        let before = b.linvel.x * b.linvel.x + b.linvel.y * b.linvel.y;
        c.remove_bias();
        c.solve(ref bs, true, true);
        let a = *bs.at(0);
        let after = a.linvel.x * a.linvel.x + a.linvel.y * a.linvel.y + a.angvel * a.angvel;
        assert!(after <= before + FixedTrait::from_raw(128));
    }

    #[test]
    #[fuzzer(runs: 32, seed: 20260920)]
    fn fuzz_two_dynamic_bodies_match_direct(vx: i16, vy: i16) {
        let (mut m, mut bs, p) = fixture(2);
        let mut other = *bs.at(0);
        other.handle.index = 19;
        other
            .linvel =
                Vec2 {
                    x: FixedTrait::from_raw(vx.into() * 1048576),
                    y: FixedTrait::from_raw(vy.into() * 1048576),
                };
        other.angvel = HALF;
        m.data.rigid_body1 = Some(other.handle);
        bs.append(other);
        let mut c = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
        c.update(p, bs.span(), m);
        let mut ds = array![*bs.at(0), *bs.at(1)];
        let mut direct = c;
        c.warmstart(ref bs);
        direct.warmstart(ref ds);
        c.solve(ref bs, true, true);
        alternatives::solve(ref direct, ref ds, true, true);
        assert_eq!(c, direct);
        assert_eq!(*bs.at(0), *ds.at(0));
        assert_eq!(*bs.at(1), *ds.at(1));
        // Equal inverse masses: signed linear impulses cancel within the floor budget.
        let total = *bs.at(0).linvel + *bs.at(1).linvel;
        assert!((total.x - HALF - other.linvel.x).abs() < FixedTrait::from_raw(16));
        assert!((total.y + ONE - other.linvel.y).abs() < FixedTrait::from_raw(16));
    }

    #[test]
    #[should_panic(expected: ('Contact: invalid count',))]
    fn test_invalid_count() {
        let (mut m, bs, p) = fixture(1);
        m.num_points = 3;
        let _ = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
    }
    #[test]
    #[should_panic(expected: ('Contact: invalid point id',))]
    fn test_invalid_point_id() {
        let (mut m, bs, p) = fixture(1);
        let [mut sc, sc1] = m.data.solver_contacts;
        sc.contact_id = 7;
        m.data.solver_contacts = [sc, sc1];
        let _ = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
    }
    #[test]
    #[should_panic(expected: ('Contact: missing body',))]
    fn test_stale_generation_rejected() {
        let (mut m, bs, p) = fixture(1);
        m.data.rigid_body2 = Some(Handle { index: 7, generation: 2 });
        let _ = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
    }
    #[test]
    #[should_panic(expected: ('Contact: same body',))]
    fn test_same_body_rejected() {
        let (mut m, bs, p) = fixture(1);
        m.data.rigid_body1 = m.data.rigid_body2;
        let _ = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
    }
    #[test]
    #[should_panic(expected: ('Contact: negative input',))]
    fn test_negative_material() {
        let (mut m, bs, p) = fixture(1);
        m.data.friction = -ONE;
        let _ = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
    }
    #[test]
    #[should_panic(expected: ('Fixed: overflow',))]
    fn test_effective_mass_overflow_is_checked() {
        let (m, mut bs, p) = fixture(1);
        let mut b = bs.pop_front().unwrap();
        b.ii = MAX;
        let mut m = m;
        let [mut sc, sc1] = m.data.solver_contacts;
        // Both witnesses at x = 2, so the common-midpoint lever arm is 2 as well.
        sc.anchor1.x = TWO;
        sc.anchor2.x = TWO;
        m.data.solver_contacts = [sc, sc1];
        bs.append(b);
        let _ = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
    }
}
