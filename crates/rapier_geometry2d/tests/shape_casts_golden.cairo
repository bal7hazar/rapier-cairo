//! CC1 linear shape casts against Parry f64 0.30.2 (family `shape_casts`): every case under the
//! five option sets of `shape_casts::OPTIONS`.
//!
//! Bands (raw Q32.32 units, measured maxima in parentheses):
//!
//! * time of impact: `EXACT_TOI` = 4 for the analytic pairs (ball–ball, half-space; 1),
//!   `GJK_TOI` = 256 where upstream runs GJK (104: GJK stops within its tolerance, the port's
//!   Minkowski ray cast is exact);
//! * normals: `NORMAL` = 2^16 outside the ambiguous regimes; witnesses of a converged impact at
//!   `t > 0` within `SURFACE` = 4096 of their shape's surface (a face–face impact reports one
//!   point of the contact family, as GJK does, so they are not compared to upstream's);
//! * hit / miss and status: equal, except the documented cases below.
//!
//! CN1's start contacts (the cases after [`CC1_CASES`], `test_shape_cast_starts_golden`): shape 2
//! `2^-16` (or exactly 0) from a face of shape 1, into, along and away from it, every answer
//! through the contact geometry at the start: hit / miss, status equal, `normal1` equal
//! (`START_NORMAL` = 0), `normal2` within `START_NORMAL2` (the rotation into frame 2), time of
//! impact within `GJK_TOI`. Exceptions: the near-corner cases (`CORNER_NORMAL`); a `t = 0` answer
//! without contact geometry (the cast's direction, as CC1's ambiguous regimes: not compared); and
//! a ball exactly touching the floor and moving along or away from it with `stop_at_penetration`,
//! where upstream's GJK misses and the port answers `t = 0` (`UPSTREAM_TANGENT_MISSES` = 6).
//!
//! Ambiguous regimes: `touching` and `grazing` (first contact corner against corner or face on
//! face, where the normal is any member of a family) compare the hit, time and status only.
//! Documented deviations:
//!
//! * upstream answers **no hit** for four exactly touching pairs (`ball_cuboid`, `cuboid_ball`,
//!   `polygon_capsule`, `rtri_ball`) under every option set that reports the contact geometry:
//!   its contact query (GJK + EPA) fails on the exact touching configuration and the `?` of
//!   `cast_shapes_support_map_support_map` turns that into `None`. The port answers the hit at
//!   `t = 0` (`UPSTREAM_CONTACT_FAILURES` = 16 answers);
//! * two coincident segments (`segment_segment/touching`) have no defined contact normal: the
//!   port reports `-y`, upstream `+y`, so `pass_through` drops or keeps the start on opposite
//!   sides (1 answer).
use fixed::Fixed;
use glam::Vec2;
use rapier_geometry2d::point::PointQuery;
use rapier_geometry2d::query::shape_cast::{ShapeCastOptions, ShapeCastStatus, cast_shapes};
use rapier_geometry2d::shape::Shape;
use rapier_golden::compare::abs_diff;
use rapier_golden::generated::shape_casts;
use rapier_golden::types::{PoseRaw, ShapeCastHitRaw, ShapeCastOptionsRaw, Vec2Raw};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use super::sh1_contacts_golden::shape;

pub fn v(p: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: p.x }, y: Fixed { raw: p.y } }
}
pub fn pose(p: PoseRaw) -> Pose2 {
    Pose2 {
        translation: v(p.translation),
        rotation: Rot2 { re: Fixed { raw: p.rotation.re }, im: Fixed { raw: p.rotation.im } },
    }
}
fn options(o: ShapeCastOptionsRaw) -> ShapeCastOptions {
    ShapeCastOptions {
        max_time_of_impact: Fixed { raw: o.max_time_of_impact },
        target_distance: Fixed { raw: o.target_distance },
        stop_at_penetration: o.stop_at_penetration,
        compute_impact_geometry_on_penetration: o.compute_impact_geometry_on_penetration,
    }
}
pub fn status_index(s: ShapeCastStatus) -> u8 {
    match s {
        ShapeCastStatus::OutOfIterations => 0,
        ShapeCastStatus::Converged => 1,
        ShapeCastStatus::Failed => 2,
        ShapeCastStatus::PenetratingOrWithinTargetDist => 3,
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
/// Distance (max component, raw) from `p` to the surface of `s`.
pub fn off_surface(s: Shape, p: Vec2) -> u64 {
    let proj = s.project_local_point(p, false);
    vdiff(proj.point, Vec2Raw { x: p.x.raw, y: p.y.raw })
}

/// `true` for the ambiguous regimes (`touching`, `grazing`) of the case at `index`.
fn ambiguous(index: u32) -> bool {
    let regime = index % 5;
    regime == 2 || regime == 4
}

const EXACT_TOI: u64 = 4;
const GJK_TOI: u64 = 256;
const NORMAL: u64 = 0x10000;
const SURFACE: u64 = 4096;
const UPSTREAM_CONTACT_FAILURES: u32 = 16;
const SEGMENT_NORMAL_SIGN: u32 = 1;
/// CC1's cases (28 pairs, five regimes); CN1's start contacts follow.
const CC1_CASES: u32 = 140;
/// CN1: the start normals of face contacts, exact as upstream's (raw units; measured maximum 0).
/// CN1: `normal2` is `normal1` turned into frame 2, one rounding of the rotation (measured: 1).
const START_NORMAL2: u64 = 4;
/// CN1: a box whose corner is 3 or 12 ulps past the end of the floor's face, `2^-16` above it: no
/// face contact (the slab test's slack scales with the height), the normal of the corner pair. Its
/// normalisation (`support_map::core_witness`) keeps the relative precision of a `2^-16` length,
/// so `y` rounds to 1 where upstream's is 1 - tilt^2 / 2 (measured: 4 at 3 ulps, 72 at 12).
const CORNER_NORMAL: u64 = 128;
/// CN1 answers where upstream's GJK misses an exactly touching ball not moving into the face.
const UPSTREAM_TANGENT_MISSES: u32 = 6;
const START_NORMAL: u64 = 0;

#[test]
fn test_shape_casts_golden() {
    let opts = shape_casts::OPTIONS.span();
    let mut index: u32 = 0;
    let (mut contact_failures, mut segment_sign, mut compared): (u32, u32, u32) = (0, 0, 0);
    for c in shape_casts::cases().slice(0, CC1_CASES) {
        let (s1, s2) = (shape(*c.shape1), shape(*c.shape2));
        let answers = (*c.answers).span();
        let mut k = 0;
        while k != 5 {
            let expected: ShapeCastHitRaw = *answers[k];
            let got = cast_shapes(
                pose(*c.pos1), v(*c.vel1), s1, pose(*c.pos2), v(*c.vel2), s2, options(*opts[k]),
            );
            let Some(got) = got else {
                assert!(!*c.supported, "{} opt {} unsupported", *c.id, k);
                k += 1;
                continue;
            };
            assert!(*c.supported, "{} opt {} supported", *c.id, k);
            match (got, expected.some) {
                (None, false) => {},
                (None, true) => panic!("{} opt {}: port miss", *c.id, k),
                (
                    Some(h), false,
                ) => {
                    // Documented: upstream's failed contact query, or the coincident segments.
                    assert!(index % 5 == 2 && h.time_of_impact.raw == 0, "{} opt {}", *c.id, k);
                    if *c.id == 'segment_segment/touching' {
                        segment_sign += 1;
                    } else {
                        contact_failures += 1;
                    }
                },
                (
                    Some(h), true,
                ) => {
                    compared += 1;
                    let band = if *c.iterative {
                        GJK_TOI
                    } else {
                        EXACT_TOI
                    };
                    assert!(
                        abs_diff(h.time_of_impact.raw, expected.toi) <= band,
                        "{} opt {} toi {:?} {}",
                        *c.id,
                        k,
                        h.time_of_impact,
                        expected.toi,
                    );
                    assert_eq!(status_index(h.status), expected.status, "{} opt {}", *c.id, k);
                    if !ambiguous(index) {
                        assert!(
                            vdiff(h.normal1, expected.normal1) <= NORMAL
                                && vdiff(h.normal2, expected.normal2) <= NORMAL,
                            "{} opt {} normal {:?}",
                            *c.id,
                            k,
                            h,
                        );
                        if h.status == ShapeCastStatus::Converged && h.time_of_impact.raw > 0 {
                            assert!(
                                off_surface(s1, h.witness1) <= SURFACE
                                    && off_surface(s2, h.witness2) <= SURFACE,
                                "{} opt {} witnesses {:?}",
                                *c.id,
                                k,
                                h,
                            );
                        }
                    }
                },
            }
            k += 1;
        }
        index += 1;
    }
    assert_eq!(contact_failures, UPSTREAM_CONTACT_FAILURES);
    assert_eq!(segment_sign, SEGMENT_NORMAL_SIGN);
    assert!(compared > 300);
}

/// The near-corner cases of CN1 (`CORNER_NORMAL`).
fn corner(id: felt252) -> bool {
    let ids = array![
        'start_corner_in/into', 'start_corner_in/along', 'start_corner_in/away',
        'start_corner_out/into', 'start_corner_out/along', 'start_corner_out/away',
    ];
    let mut found = false;
    for i in ids {
        if i == id {
            found = true;
        }
    }
    found
}

#[test]
fn test_shape_cast_starts_golden() {
    let opts = shape_casts::OPTIONS.span();
    let all = shape_casts::cases();
    let (mut hits, mut worst, mut worst2, mut tangent_misses): (u32, u64, u64, u32) = (0, 0, 0, 0);
    let mut worst_corner: u64 = 0;
    for c in all.slice(CC1_CASES, all.len() - CC1_CASES) {
        let (s1, s2) = (shape(*c.shape1), shape(*c.shape2));
        let answers = (*c.answers).span();
        let mut k = 0;
        while k != 5 {
            let expected: ShapeCastHitRaw = *answers[k];
            let got = cast_shapes(
                pose(*c.pos1), v(*c.vel1), s1, pose(*c.pos2), v(*c.vel2), s2, options(*opts[k]),
            )
                .unwrap();
            if got.is_some() != expected.some {
                // Documented: upstream's GJK misses a ball exactly touching the face and not
                // moving into it; the port answers the start at `t = 0` (with the exact normal).
                let h = got.expect('start: port miss');
                assert!(h.time_of_impact.raw == 0, "{} opt {} hit {:?}", *c.id, k, h);
                println!("{} opt {}: port hit at t = 0, upstream miss", *c.id, k);
                tangent_misses += 1;
                k += 1;
                continue;
            }
            if let Some(h) = got {
                hits += 1;
                assert!(
                    abs_diff(h.time_of_impact.raw, expected.toi) <= GJK_TOI,
                    "{} opt {} toi {:?}",
                    *c.id,
                    k,
                    h.time_of_impact,
                );
                assert_eq!(status_index(h.status), expected.status, "{} opt {}", *c.id, k);
                // Without contact geometry, a `t = 0` answer reports the cast's own direction
                // (CC1's ambiguous regime), not the start contact: not compared.
                if k == 4 && expected.toi == 0 {
                    k += 1;
                    continue;
                }
                let d1 = vdiff(h.normal1, expected.normal1);
                let d2 = vdiff(h.normal2, expected.normal2);
                println!("{} opt {}: normal1 {} normal2 {} ulps", *c.id, k, d1, d2);
                if corner(*c.id) {
                    assert!(
                        d1 <= CORNER_NORMAL && d2 <= CORNER_NORMAL, "{} opt {} {:?}", *c.id, k, h,
                    );
                    if d1 > worst_corner {
                        worst_corner = d1;
                    }
                    if d2 > worst_corner {
                        worst_corner = d2;
                    }
                } else {
                    assert!(
                        d1 <= START_NORMAL && d2 <= START_NORMAL2,
                        "{} opt {} normal {:?}",
                        *c.id,
                        k,
                        h,
                    );
                    if d1 > worst {
                        worst = d1;
                    }
                    if d2 > worst2 {
                        worst2 = d2;
                    }
                }
            }
            k += 1;
        }
    }
    assert!(hits > 40);
    assert_eq!(tangent_misses, UPSTREAM_TANGENT_MISSES);
    println!("shape cast starts: {} hits, worst normal1 {} normal2 {} ulps", hits, worst, worst2);
}
