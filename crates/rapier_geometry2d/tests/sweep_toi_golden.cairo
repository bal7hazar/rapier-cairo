//! CC1 swept time of impact against Parry f64 0.30.2 (family `sweep_toi`): the proxy of shape 1
//! standing still, the proxy of shape 2 swept (linearly, turning, missing, overlapping, grazing).
//!
//! Bands (raw Q32.32 units, measured maxima in parentheses): statuses equal; fraction `FRACTION`
//! = 2^17 (59773: upstream's root finder accepts any time where the distance is within
//! `linear_slop / 4` of the target, and the port's exact distance kernel steers it through other
//! iterates than GJK's); normal `NORMAL` = 2^10 (243); impact point `POINT` = 2^17 (53495).
use fixed::Fixed;
use glam::Vec2;
use rapier_geometry2d::query::sweep::{
    SweepToiStatus, SweepTrait, ToiProxyTrait, sweep_time_of_impact,
};
use rapier_golden::compare::abs_diff;
use rapier_golden::generated::sweep_toi;
use rapier_golden::generated::sweep_toi::{
    part0, part1, part2, part3, part4, part5, part6, part7, part8,
};
use rapier_golden::types::{SweepToiCase, Vec2Raw};

const FRACTION: u64 = 0x20000;
const NORMAL: u64 = 0x400;
const POINT: u64 = 0x20000;
use super::sh1_contacts_golden::shape;
use super::shape_casts_golden::{pose, v};

fn status_index(s: SweepToiStatus) -> u8 {
    match s {
        SweepToiStatus::Overlapped => 0,
        SweepToiStatus::Hit => 1,
        SweepToiStatus::Separated => 2,
        SweepToiStatus::Failed => 3,
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

fn run(c: SweepToiCase) -> (u8, u64, u64, u64) {
    let p1 = ToiProxyTrait::from_shape(shape(c.shape1)).unwrap();
    let p2 = ToiProxyTrait::from_shape(shape(c.shape2)).unwrap();
    let s1 = SweepTrait::constant(pose(c.pose1), Vec2 { x: Fixed { raw: 0 }, y: Fixed { raw: 0 } });
    let s2 = SweepTrait::from_poses(pose(c.start2), pose(c.end2), v(c.local_center2));
    let r = sweep_time_of_impact(
        p1,
        s1,
        p2,
        s2,
        Fixed { raw: sweep_toi::MAX_FRACTION },
        Fixed { raw: sweep_toi::LINEAR_SLOP },
    );
    (
        status_index(r.status),
        abs_diff(r.fraction.raw, c.fraction),
        vdiff(r.normal, c.normal),
        vdiff(r.point, c.point),
    )
}

fn check(cases: Span<SweepToiCase>) {
    for c in cases {
        let (status, fraction, normal, point) = run(*c);
        assert_eq!(status, *c.status, "{}", *c.id);
        assert!(fraction <= FRACTION && normal <= NORMAL && point <= POINT, "{}", *c.id);
    }
}

#[test]
fn test_sweep_toi_parts_0_2() {
    check(part0::cases());
    check(part1::cases());
    check(part2::cases());
}

#[test]
fn test_sweep_toi_parts_3_5() {
    check(part3::cases());
    check(part4::cases());
    check(part5::cases());
}

#[test]
fn test_sweep_toi_parts_6_8() {
    check(part6::cases());
    check(part7::cases());
    check(part8::cases());
}
