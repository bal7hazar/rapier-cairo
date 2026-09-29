//! CC1 nonlinear shape casts against Parry f64 0.30.2 (family `nonlinear_shape_casts`), with
//! `stop_at_penetration` true and false; one test per fixture part (step budget).
//!
//! Bands (raw Q32.32 units): time of impact `TOI` = 2^18 (~6e-5; measured maximum 181k on the
//! grazing regime, where the approach speed along the normal is small): upstream bisects to
//! f64's `eps_tol`, the port to `TOLERANCE` = 5120 ulp of distance, so its time is off by up to
//! `TOLERANCE / approach speed` (always on the early side). Normals `NORMAL` = 2^24 (~4e-3): the
//! reported normal is the separating direction of the last advancement, taken at slightly
//! different times on each side. Starts in contact answer `t = 0` exactly.
//!
//! Status: equal, except `Converged` against `Failed` for an impact after the start (both end
//! when the bisection range collapses onto a slightly penetrating time; which one is reached is
//! decided by the last rounding) and `PenetratingOrWithinTargetDist` against `Converged` for an
//! exactly touching start (GJK's closest points answer either within its tolerance).
//!
//! Documented deviations (not compared): the coincident / crossing segments (`segment_segment/
//! touching`, `/penetrating`: upstream's closest points of crossing segments are `~1e-17` apart,
//! just above f64's `DEFAULT_EPSILON` or not, so it answers a hit or none by rounding noise; the
//! port answers `Failed` at `t = 0`), and `polygon_capsule/touching` without stopping at
//! penetration, where upstream's contact query (GJK + EPA) fails and its `??` answers no hit.
use fixed::Fixed;
use glam_core::Vec2;
use rapier_geometry2d::query::nonlinear_shape_cast::{NonlinearRigidMotion, cast_shapes_nonlinear};
use rapier_golden::compare::abs_diff;
use rapier_golden::generated::nonlinear_shape_casts;
use rapier_golden::generated::nonlinear_shape_casts::{
    part0, part1, part10, part11, part2, part3, part4, part5, part6, part7, part8, part9,
};
use rapier_golden::types::{NonlinearMotionRaw, NonlinearShapeCastCase, ShapeCastHitRaw, Vec2Raw};
use super::sh1_contacts_golden::shape;
use super::shape_casts_golden::{pose, status_index, v};

fn motion(m: NonlinearMotionRaw) -> NonlinearRigidMotion {
    NonlinearRigidMotion {
        start: pose(m.start),
        local_center: v(m.local_center),
        linvel: v(m.linvel),
        angvel: Fixed { raw: m.angvel },
    }
}
fn vdiff(a: Vec2, b: Vec2Raw) -> u64 {
    let (dx, dy) = (abs_diff(a.x.raw, b.x), abs_diff(a.y.raw, b.y));
    if dx > dy {
        dx
    } else {
        dy
    }
}

const TOI: u64 = 0x40000;
const NORMAL: u64 = 0x1000000;

/// Compares every case of one part; `first` is the index of its first case in the family (the
/// regime is `index % 5`: hit, miss, touching, penetrating, grazing). Returns the number of
/// mismatches, each printed.
fn check(cases: Span<NonlinearShapeCastCase>, first: u32) -> u32 {
    let t0 = Fixed { raw: nonlinear_shape_casts::START_TIME };
    let t1 = Fixed { raw: nonlinear_shape_casts::END_TIME };
    let mut index = first;
    let mut bad = 0;
    for c in cases {
        let (s1, s2) = (shape(*c.shape1), shape(*c.shape2));
        let regime = index % 5;
        index += 1;
        let coincident = *c.id == 'segment_segment/touching'
            || *c.id == 'segment_segment/penetrating';
        let answers = (*c.answers).span();
        let mut k = 0;
        while k != 2 {
            let expected: ShapeCastHitRaw = *answers[k];
            let got = cast_shapes_nonlinear(
                motion(*c.motion1), s1, motion(*c.motion2), s2, t0, t1, k == 0,
            );
            k += 1;
            let Some(got) = got else {
                if *c.supported {
                    println!("{} unsupported", *c.id);
                    bad += 1;
                }
                continue;
            };
            if coincident {
                continue;
            }
            match (got, expected.some) {
                (None, false) => {},
                (None, true) => {
                    println!("{} stop {} port miss", *c.id, k == 1);
                    bad += 1;
                },
                (
                    Some(h), false,
                ) => {
                    // Documented: upstream's contact query fails on the exact touching start of
                    // `polygon_capsule` (directional mode only).
                    if !(*c.id == 'polygon_capsule/touching' && k == 2) {
                        println!("{} stop {} port hit {:?}", *c.id, k == 1, h);
                        bad += 1;
                    }
                },
                (
                    Some(h), true,
                ) => {
                    let (got_status, status) = (status_index(h.status), expected.status);
                    let at_start = h.time_of_impact.raw == 0;
                    let status_ok = got_status == status
                        || (!at_start && got_status + status == 3 && got_status * status == 2)
                        || (at_start && regime == 2 && got_status
                            + status == 4 && got_status * status == 3);
                    if abs_diff(h.time_of_impact.raw, expected.toi) > TOI
                        || !status_ok
                        || (at_start != (expected.toi == 0)) {
                        println!(
                            "{} stop {} toi {} vs {} status {} vs {}",
                            *c.id,
                            k == 1,
                            h.time_of_impact.raw,
                            expected.toi,
                            got_status,
                            status,
                        );
                        bad += 1;
                    } else if !at_start && vdiff(h.normal1, expected.normal1) > NORMAL {
                        println!(
                            "{} stop {} normal {:?} vs {:?}",
                            *c.id,
                            k == 1,
                            h.normal1,
                            expected.normal1,
                        );
                        bad += 1;
                    }
                },
            }
        }
    }
    bad
}

#[test]
fn test_nonlinear_part0() {
    assert_eq!(check(part0::cases(), 0), 0);
}
#[test]
fn test_nonlinear_part1() {
    assert_eq!(check(part1::cases(), 12), 0);
}
#[test]
fn test_nonlinear_part2() {
    assert_eq!(check(part2::cases(), 24), 0);
}
#[test]
fn test_nonlinear_part3() {
    assert_eq!(check(part3::cases(), 36), 0);
}
#[test]
fn test_nonlinear_part4() {
    assert_eq!(check(part4::cases(), 48), 0);
}
#[test]
fn test_nonlinear_part5() {
    assert_eq!(check(part5::cases(), 60), 0);
}
#[test]
fn test_nonlinear_part6() {
    assert_eq!(check(part6::cases(), 72), 0);
}
#[test]
fn test_nonlinear_part7() {
    assert_eq!(check(part7::cases(), 84), 0);
}
#[test]
fn test_nonlinear_part8() {
    assert_eq!(check(part8::cases(), 96), 0);
}
#[test]
fn test_nonlinear_part9() {
    assert_eq!(check(part9::cases(), 108), 0);
}
#[test]
fn test_nonlinear_part10() {
    assert_eq!(check(part10::cases(), 120), 0);
}
#[test]
fn test_nonlinear_part11() {
    assert_eq!(check(part11::cases(), 132), 0);
}
