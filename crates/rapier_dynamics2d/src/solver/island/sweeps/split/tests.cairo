//! The split sweeps against the constraint-set sweeps they replace (`sweeps::contact` on a
//! `DenseBodies` store), stage by stage, on a three-body stack with a world endpoint, a
//! one-point manifold, warm starts and approaching NEW contacts (restitution bounces). Gas and
//! step probes per stage on the same scene, one `gas_baseline`.
use fixed::{Fixed, FixedTrait, HALF, ONE};
use glam_core::Vec2;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_geometry2d::contact::{ContactManifold, NEW_CONTACT_BIT};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::{Rot2, Rot2Trait};
use rapier_testing::opaque;
use super::generation::{generate, probe_round_trip, probe_round_trip_via_pose};
use super::super::contact::contacts as reference;
use super::super::super::fixtures::stack;
use super::super::super::super::body::SolverBody;
use super::super::super::super::body_store::{BodyStep, DenseBodies, DenseBodiesTrait};
use super::super::super::super::contact::element::{separation, tangent};
use super::super::super::super::contact::{ContactConstraintsSetTrait, cached};
use super::super::super::{add_forces, damp, integrate};
use super::{SweepBodies, SweepBodiesTrait, alternatives, contacts, writeback};

/// The stack of three with velocities from `(vx, vy, spin)`, restitution on the world manifold
/// (whose contacts are NEW), warm starts `warm` on the second manifold (tracked point ids), and
/// one solver contact on the third.
fn scene(
    vx: i16, vy: i16, spin: i16, warm: u8,
) -> (Array<SolverBody>, Array<BodyStep>, Array<ContactManifold>, IntegrationParameters) {
    let (bs, steps, ms) = stack(3);
    let mut bodies = array![];
    let mut i: i64 = 0;
    for b in bs {
        let mut b = b;
        b.linvel.x = FixedTrait::from_raw(vx.into() * 1048573 + i * 77);
        b.linvel.y = FixedTrait::from_raw(vy.into() * 1048577 - i * 131);
        b.angvel = FixedTrait::from_raw(spin.into() * 524287 + i);
        bodies.append(b);
        i += 1;
    }
    let mut manifolds = array![];
    let mut k = 0;
    for m in ms {
        let mut m = m;
        if k == 0 {
            m.data.restitution = HALF;
        } else if k == 1 {
            let [mut a, mut b] = m.data.solver_contacts;
            a.contact_id = 0;
            b.contact_id = 1;
            m.data.solver_contacts = [a, b];
            let [mut p0, mut p1] = m.points;
            p0.data.warmstart_impulse = FixedTrait::from_raw(warm.into() * 16777213);
            p1.data.warmstart_impulse = FixedTrait::from_raw(warm.into() * 8388617);
            p0.data.warmstart_tangent_impulse = FixedTrait::from_raw(-(warm.into() * 4194301));
            m.points = [p0, p1];
        } else {
            m.data.num_solver_contacts = 1;
        }
        manifolds.append(m);
        k += 1;
    }
    (bodies, steps, manifolds, Default::default())
}

/// Every stage of one substep and the final bounce, plus forces, integration, damping and
/// writeback, through both implementations; bodies and manifolds must match exactly.
fn check(vx: i16, vy: i16, spin: i16, warm: u8) {
    let (bs, steps, ms, p) = scene(vx, vy, spin, warm);
    let dt = p.substep_dt();
    let mut cs = ContactConstraintsSetTrait::generate(ms.span(), bs.span(), p, dt);
    let directions = cached::prepare(cs.constraints.span());
    let mut dense: DenseBodies = DenseBodiesTrait::new(bs.span());
    let (frozen, mut state) = generate(ms.span(), bs.span(), p, dt);
    let mut sb: SweepBodies = DenseBodiesTrait::new(bs.span());
    let (max_lin, max_ang) = (p.max_linear_velocity(), Fixed { raw: 3373259426 } * p.inv_dt());
    add_forces(ref dense, steps.span());
    sb.add_forces(steps.span());
    // Stage 5 (the next substep's update from the separations cached by stage 2) against the
    // reference's recomputing update.
    for stage in array![0_u8, 1, 2, 3, 5, 1, 4] {
        if stage == 2 {
            integrate(ref dense, steps.span(), dt, max_lin, max_ang);
            sb.integrate(steps.span(), dt, max_lin, max_ang);
        }
        let reference_stage = if stage == 5 {
            0
        } else {
            stage
        };
        reference(ref cs, ref dense, ms.span(), p, reference_stage, directions.span());
        contacts(ref state, frozen.span(), ref sb, p, stage);
        let mut i = 0;
        while i != 3 {
            assert_eq!(sb.get(i), dense.get(i));
            i += 1;
        }
    }
    damp(ref dense, steps.span(), p.dt);
    sb.damp(steps.span(), p.dt);
    let mut expected = ms.clone();
    cs.writeback_impulses(ref expected);
    let mut actual = ms;
    writeback(frozen.span(), @state, ref actual);
    assert_eq!(actual, expected);
    let mut out: DenseBodies = DenseBodiesTrait::new(bs.span());
    sb.finish(ref out);
    let mut i = 0;
    while i != 3 {
        assert_eq!(out.get(i), dense.get(i));
        i += 1;
    }
}

#[test]
fn test_split_matches_constraint_set_sweeps() {
    // (vx, vy, spin, warm): resting, falling onto the stack (bounces), sliding and spinning,
    // rising (no bounce), and a cold start.
    let cases = array![
        (0_i16, 0_i16, 0_i16, 0_u8), (0, -2000, 0, 3), (1500, -700, 900, 9), (-800, 1200, -300, 1),
        (300, -3000, 20, 0),
    ];
    for (vx, vy, spin, warm) in cases {
        check(vx, vy, spin, warm);
    }
}

#[test]
#[fuzzer(runs: 24, seed: 925)]
fn fuzz_split_matches_constraint_set_sweeps(vx: i16, vy: i16, spin: i16, warm: u8) {
    check(vx / 8, vy / 8, spin / 8, warm);
}

/// Probe of the split path: generation only (`stage == 5`; `8`: BT1's generation through the
/// constraints, `alternatives::generate_via_constraints`), or generation then one stage.
fn probe(stage: u8) {
    let (bs, _, ms, p) = scene(opaque(0), opaque(-2000), opaque(900), opaque(3));
    let (frozen, mut state) = if stage == 8 {
        alternatives::generate_via_constraints(ms.span(), bs.span(), p, p.substep_dt())
    } else {
        generate(ms.span(), bs.span(), p, p.substep_dt())
    };
    let mut sb: SweepBodies = DenseBodiesTrait::new(bs.span());
    if stage < 5 {
        contacts(ref state, frozen.span(), ref sb, p, stage);
    } else if stage == 7 {
        alternatives::biased_metered(ref state, frozen.span(), ref sb);
    } else if stage == 6 {
        let mut ms = ms;
        writeback(frozen.span(), @state, ref ms);
        let _ = opaque(ms.span());
    }
    let _ = opaque((state.hot.span(), state.bank.span(), sb.get(opaque(1))));
}

#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}
#[test]
fn gas_generate_stack3() {
    probe(5);
}
#[test]
fn gas_generate_via_constraints_stack3() {
    probe(8);
}
#[test]
fn gas_update_stack3() {
    probe(0);
}
#[test]
fn gas_biased_stack3() {
    probe(1);
}
#[test]
fn gas_biased_metered_stack3() {
    probe(7);
}
#[test]
fn gas_refresh_stack3() {
    probe(2);
}
#[test]
fn gas_relax_stack3() {
    probe(3);
}
#[test]
fn gas_restitution_stack3() {
    probe(4);
}
#[test]
fn gas_writeback_stack3() {
    probe(6);
}

/// SF1: the anchors' round trip at the build-time poses, winner (`element::separation`, fused
/// since FU1 P2) and rejected candidate (`Pose2Trait::transform_point` and two dots), on
/// a ground contact and a contact between two turned bodies. Sierra gas charges both paths of the
/// world test alike; the Cairo steps (`--tracked-resource cairo-steps`) tell them apart.
fn round_trip_inputs(ground: bool) -> (bool, Vec2, Rot2, Vec2, Rot2, Vec2, Vec2, Vec2) {
    let r = |x: i64| FixedTrait::from_raw(x);
    let v = |x: i64, y: i64| Vec2 { x: r(x), y: r(y) };
    opaque(
        (
            ground,
            v(858993459, 4294967296),
            Rot2 { re: r(4294967295), im: r(103173) },
            v(787410671, 3436551409),
            Rot2 { re: r(3719550787), im: r(2147483648) },
            v(-643944242, 1),
            v(-1431655765, -4366550085),
            v(-2147483648, -3719550787),
        ),
    )
}
fn round_trip_winner(ground: bool) {
    let (world1, com1, rot1, com2, rot2, lp1, lp2, dir) = round_trip_inputs(ground);
    let _ = opaque(probe_round_trip(world1, com1, rot1, com2, rot2, lp1, lp2, dir));
}
fn round_trip_via_pose(ground: bool) {
    let (world1, com1, rot1, com2, rot2, lp1, lp2, dir) = round_trip_inputs(ground);
    let _ = opaque(probe_round_trip_via_pose(world1, com1, rot1, com2, rot2, lp1, lp2, dir));
}
/// SF1 guard (FU1): generation stores `sc.dist - n0`, with `n0` the round trip's separation at
/// the build-time poses; the refresh at those same poses must give back `sc.dist` exactly. Two
/// turned bodies and a world endpoint (identity pose). On each case the unfused round trip
/// (`via_pose`, the floored transforms of the SF1 defect) gives a different `n0`, and its
/// refresh misses `sc.dist`: the test fails if generation and the refresh ever stop sharing one
/// function.
#[test]
fn test_round_trip_refresh_gives_back_sc_dist() {
    let r = |x: i64| FixedTrait::from_raw(x);
    let v = |x: i64, y: i64| Vec2 { x: r(x), y: r(y) };
    // (world1, (com1, rot1), (com2, rot2), (local_p1, local_p2), (dir, sc.dist)), raw Q32.32
    let cases: Array<
        (bool, (i64, i64, i64, i64), (i64, i64, i64, i64), (i64, i64, i64, i64), (i64, i64, i64)),
    > =
        array![
        (
            false,
            (86681562512, 10638180417, 4075662797, 1354886280),
            (87793668870, 1283825251, 4294205569, 80886329),
            (240153109, 1035787950, 1348825199, -1117869483),
            (698347296, 4237812540, 17896493),
        ),
        (
            false,
            (84725315082, 3929854785, 4290557034, 194587792),
            (82269371634, 6710922483, 3781636006, 2036166296),
            (1163228109, 818967996, 645849116, 360520365),
            (1805897462, 3896854940, 2534490),
        ),
        (
            false,
            (87681174376, 11243278573, 3855207134, -1893177758),
            (83138775798, 819135423, 3782330654, 2034875645),
            (-1428304577, -1798682204, -1892662456, -1956244431),
            (531346061, -4261973186, -57803519),
        ),
        (
            false,
            (75190180043, 2518922471, 4286671530, -266816917),
            (84241766109, 14790616619, 4231009595, 738445581),
            (239061850, -1568754814, -353767510, -2027604178),
            (4228524271, -752546717, 85635270),
        ),
        (
            true,
            (0, 0, 4294967296, 0),
            (76823452448, 4102699863, 4044947469, -1444002785),
            (77375978100, -83197677, -284879743, 1984647005),
            (-1136450432, -4141886586, 64129517),
        ),
        (
            true,
            (0, 0, 4294967296, 0),
            (76491114215, 1577149209, 3987756581, -1595161912),
            (76444898374, 47662421, -698182315, 620715286),
            (3708665895, 2166227446, 54138614),
        ),
    ];
    for (world1, a, b, l, d) in cases {
        let ((c1x, c1y, re1, im1), (c2x, c2y, re2, im2)) = (a, b);
        let ((l1x, l1y, l2x, l2y), (dx, dy, sc)) = (l, d);
        let (com1, rot1) = (v(c1x, c1y), Rot2 { re: r(re1), im: r(im1) });
        let (com2, rot2) = (v(c2x, c2y), Rot2 { re: r(re2), im: r(im2) });
        let (lp1, lp2, dir, sc) = (v(l1x, l1y), v(l2x, l2y), v(dx, dy), r(sc));
        let pose1 = if world1 {
            Default::default()
        } else {
            Pose2 { translation: com1, rotation: rot1 }
        };
        let pose2 = Pose2 { translation: com2, rotation: rot2 };
        let (n0, _) = probe_round_trip(world1, com1, rot1, com2, rot2, lp1, lp2, dir);
        let (dist, _) = separation(pose1, lp1, pose2, lp2, dir, tangent(dir), sc - n0);
        assert_eq!(dist, sc);
        let (n0_pose, _) = probe_round_trip_via_pose(world1, com1, rot1, com2, rot2, lp1, lp2, dir);
        let (missed, _) = separation(pose1, lp1, pose2, lp2, dir, tangent(dir), sc - n0_pose);
        assert_ne!(missed, sc);
    }
}

#[test]
fn gas_round_trip_ground() {
    round_trip_winner(true);
}
#[test]
fn gas_round_trip_ground_via_pose() {
    round_trip_via_pose(true);
}
#[test]
fn gas_round_trip_bodies() {
    round_trip_winner(false);
}
#[test]
fn gas_round_trip_bodies_via_pose() {
    round_trip_via_pose(false);
}
#[test]
#[fuzzer(runs: 32, seed: 20260927)]
fn fuzz_round_trip_candidates_close(ax: i16, ay: i16, bx: i16, by: i16, turn: i16, ground: bool) {
    let r = |x: i64| FixedTrait::from_raw(x);
    let v = |x: i64, y: i64| Vec2 { x: r(x), y: r(y) };
    let rot = Rot2Trait::from_cos_sin(r(4294967296), r(turn.into() * 131071));
    let (com1, com2) = (v(bx.into() * 131071, 7), v(-3, ay.into() * 131073));
    let lp1 = v(ax.into() * 65537, ay.into() * 65539);
    let lp2 = v(bx.into() * 65541, by.into() * 65543);
    let dir = rot.rotate(v(0, -4294967296));
    // FU1 P2: the fused winner is within one ulp of the exact separations, the floored candidate
    // within 2.3 (`docs/research/fused-rescales.md` §4), so they differ by at most 3 raw.
    let (n, t) = probe_round_trip(ground, com1, rot, com2, rot, lp1, lp2, dir);
    let (n_pose, t_pose) = probe_round_trip_via_pose(ground, com1, rot, com2, rot, lp1, lp2, dir);
    let three = r(3);
    assert!(n.abs_diff_eq(n_pose, three) && t.abs_diff_eq(t_pose, three));
}
