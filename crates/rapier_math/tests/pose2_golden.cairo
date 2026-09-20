//! End-to-end comparisons with Parry/glamx f64 pose fixtures; no input renormalization.
use fixed::{Fixed, ZERO};
use glam::Vec2;
use rapier_golden::compare::{vec2_within, within};
use rapier_golden::types::{PoseRaw, RotRaw, Vec2Raw};
use rapier_golden::{aabb, pose2, scenes};
use rapier_math::consts::{ONE_SQ_RAW, UNIT_TOL_SQ_RAW};
use rapier_math::math_ext::norm2_sq_wide;
use rapier_math::pose2::{IDENTITY, Pose2, Pose2Trait};
use rapier_math::rot2::{Rot2, Rot2Trait};
use rapier_testing::opaque;

fn vector(v: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: v.x }, y: Fixed { raw: v.y } }
}
fn rotation(r: RotRaw) -> Rot2 {
    Rot2 { re: Fixed { raw: r.re }, im: Fixed { raw: r.im } }
}
fn pose(p: PoseRaw) -> Pose2 {
    Pose2Trait::new(vector(p.translation), rotation(p.rotation))
}
fn raw(v: Vec2) -> Vec2Raw {
    Vec2Raw { x: v.x.raw, y: v.y.raw }
}
fn magnitude(v: Vec2) -> u64 {
    let x: i128 = v.x.raw.into();
    let y: i128 = v.y.raw.into();
    let sum = (if x < 0 {
        -x
    } else {
        x
    }) + (if y < 0 {
        -y
    } else {
        y
    });
    (sum / 4294967296 + 1).try_into().unwrap()
}
fn check_rot(r: Rot2, expected: RotRaw, tolerance: u64) {
    assert!(within(r.re.raw, expected.re, tolerance));
    assert!(within(r.im.raw, expected.im, tolerance));
}
fn check_pose(p: Pose2, expected: PoseRaw, tolerance: u64) {
    check_rot(p.rotation, expected.rotation, 2);
    assert!(vec2_within(raw(p.translation), expected.translation, tolerance));
}

#[test]
fn test_all_pose_pairs() {
    for c in pose2::cases() {
        let a = pose(*c.a);
        let b = pose(*c.b);
        let tolerance = 4 + 2 * (magnitude(a.translation) + magnitude(b.translation));
        check_pose(a * b, *c.mul, tolerance);
        check_pose(a.inverse(), *c.inverse, tolerance);
        check_pose(a.inv_mul(b), *c.inv_mul, tolerance);
        check_rot(a.rotation * b.rotation, *c.rot_mul, 2);
        check_rot(a.rotation.inverse(), *c.rot_inverse, 0);
        let same = a.inv_mul(a);
        assert!(same.rotation.is_unit(), "self relative rotation {}", *c.id);
        assert!(vec2_within(raw(same.translation), raw(IDENTITY.translation), 2));
        assert!(within(same.rotation.re.raw, 4294967296, 2));
        assert_eq!(same.rotation.im, ZERO);
        // Direct pos12 matches composed inverse within its additional one rounding.
        let composed = a.inverse() * b;
        assert!(vec2_within(raw(a.inv_mul(b).translation), raw(composed.translation), 2));
    }
}

#[test]
fn test_all_point_and_vector_transforms() {
    for c in pose2::cases() {
        let a = pose(*c.a);
        let mut i = 0;
        while i != 3 {
            let p = vector(*c.points.span().at(i));
            let tol = 4 + 2 * (magnitude(p) + magnitude(a.translation));
            assert!(vec2_within(raw(a.transform_point(p)), *c.transform_point.span().at(i), tol));
            assert!(
                vec2_within(
                    raw(a.inverse_transform_point(p)), *c.inverse_transform_point.span().at(i), tol,
                ),
            );
            assert!(vec2_within(raw(a.transform_vector(p)), *c.transform_vector.span().at(i), tol));
            assert!(
                vec2_within(
                    raw(a.inverse_transform_vector(p)),
                    *c.inverse_transform_vector.span().at(i),
                    tol,
                ),
            );
            assert!(vec2_within(raw(a.inverse_transform_point(a.transform_point(p))), raw(p), tol));
            assert!(
                vec2_within(raw(a.inverse_transform_vector(a.transform_vector(p))), raw(p), tol),
            );
            i += 1;
        }
    }
}

#[test]
fn test_aabb_pose_round_trips() {
    for c in aabb::cases() {
        let p = pose(*c.pose);
        assert!(p.rotation.is_unit());
        // Exercise the poses and world-space bounds consumed by the geometry layer.
        for v in array![vector(*c.mins), vector(*c.maxs)].span() {
            let tol = 4 + 2 * (magnitude(*v) + magnitude(p.translation));
            assert!(
                vec2_within(raw(p.transform_point(p.inverse_transform_point(*v))), raw(*v), tol),
            );
        }
    }
}

#[test]
fn test_scene_sample_rotations_and_linearized_substeps_stay_unit() {
    for c in scenes::cases() {
        let mut s = 0;
        while s != 22 {
            let sample = *c.samples.span().at(s);
            let mut k: u32 = 0;
            while k != *c.num_dynamic {
                let state = *sample.states.span().at(k.try_into().unwrap());
                let r = rotation(state.rotation);
                assert!(r.is_unit(), "scene {} step {}", *c.id, sample.step);
                let w = Fixed { raw: state.angvel };
                let dt = Fixed { raw: *c.dt / 4 };
                let mut advanced = r;
                let mut substep = 0;
                while substep != 4 {
                    advanced = advanced.integrate(w, dt);
                    assert!(advanced.is_unit());
                    substep += 1;
                }
                k += 1;
            }
            s += 1;
        }
    }
}

#[test]
fn test_chain_golden_checkpoints() {
    for c in pose2::chain_cases() {
        let step = rotation(*c.step);
        let mut r = rapier_math::rot2::IDENTITY;
        let mut n = 0;
        for sample in c.samples.span() {
            while n != *sample.steps {
                r = r * step;
                n += 1;
            }
            // Each fused complex product adds < sqrt(2) ulps of vector error. The
            // snapped step has norm 1 +/- 2^-32; 2*n covers all four checkpoints.
            check_rot(r, *sample.rotation, 2 * n.into());
            let drift = norm2_sq_wide(r.re, r.im) - ONE_SQ_RAW;
            let signed_q32: i64 = (drift / 4294967296).try_into().unwrap();
            assert!(within(signed_q32, *sample.drift, 4 * n.into()));
        }
    }
}

/// Deterministic study: k=0 means never normalize. k=16 deliberately ends eight steps
/// after the last normalization at 992. Report both endpoint and maximum post-update drift.
#[test]
fn test_drift_study() {
    for c in pose2::chain_cases() {
        for k in array![0_u32, 1, 4, 16].span() {
            let step = rotation(*c.step);
            let mut r = rapier_math::rot2::IDENTITY;
            let mut n: u32 = 0;
            let mut since: u32 = 0;
            let mut peak: i128 = 0;
            while n != 1000 {
                r = r * step;
                n += 1;
                since += 1;
                if *k != 0 && since == *k {
                    r = r.renormalize();
                    since = 0;
                }
                let drift = norm2_sq_wide(r.re, r.im) - ONE_SQ_RAW;
                let abs = if drift < 0 {
                    -drift
                } else {
                    drift
                };
                if abs > peak {
                    peak = abs;
                }
                if *k == 1 {
                    assert!(abs <= UNIT_TOL_SQ_RAW);
                }
            }
            let drift = norm2_sq_wide(r.re, r.im) - ONE_SQ_RAW;
            let abs = if drift < 0 {
                -drift
            } else {
                drift
            };
            let upstream = *c.samples.span().at(3).drift;
            println!(
                "drift {} k={} signed_q32={} abs_q64={} peak_q64={} upstream_q32={}",
                *c.id,
                *k,
                drift / 4294967296,
                abs,
                peak,
                upstream,
            );
        }
    }
}

#[test]
fn gas_baseline() {}

#[test]
fn gas_relative_pose_golden() {
    let c = pose2::PAIR_SAME_POSE;
    let result = opaque(pose(c.a)).inv_mul(opaque(pose(c.b)));
    assert_eq!(result.translation, IDENTITY.translation);
    assert!(result.rotation.is_unit());
}
