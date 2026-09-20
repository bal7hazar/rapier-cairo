//! Sanity checks of the `pose2` fixtures against closed-form results, computed here with plain
//! integer arithmetic on the raw Q32.32 values. They guard the harness (wrong operand order,
//! wrong sign, wrong unit), not the port.

use rapier_golden::compare::{vec2_within, within};
use rapier_golden::pose2;
use rapier_golden::types::{PoseRaw, RotRaw, Vec2Raw};

const ONE: i128 = 0x100000000;

/// Q32.32 product, truncated toward zero.
fn mul(a: i128, b: i128) -> i128 {
    a * b / ONE
}

fn narrow(value: i128) -> i64 {
    value.try_into().unwrap()
}

/// `|x| + |y|` in whole units, rounded up: scales the tolerance with the magnitude, since the
/// norm error of a rotation (`1 ± 2^-32`) multiplies whatever it is applied to.
fn magnitude(v: Vec2Raw) -> u64 {
    let x: i128 = v.x.into();
    let y: i128 = v.y.into();
    let sum = (if x < 0 {
        -x
    } else {
        x
    }) + (if y < 0 {
        -y
    } else {
        y
    });
    (sum / ONE + 1).try_into().unwrap()
}

/// `r · v`.
fn rotate(r: RotRaw, v: Vec2Raw) -> Vec2Raw {
    let (re, im): (i128, i128) = (r.re.into(), r.im.into());
    let (x, y): (i128, i128) = (v.x.into(), v.y.into());
    Vec2Raw { x: narrow(mul(re, x) - mul(im, y)), y: narrow(mul(im, x) + mul(re, y)) }
}

/// `r⁻¹ · v` with `r⁻¹ = (re, -im)`.
fn rotate_inverse(r: RotRaw, v: Vec2Raw) -> Vec2Raw {
    rotate(RotRaw { re: r.re, im: -r.im }, v)
}

fn add(a: Vec2Raw, b: Vec2Raw) -> Vec2Raw {
    Vec2Raw { x: a.x + b.x, y: a.y + b.y }
}

fn sub(a: Vec2Raw, b: Vec2Raw) -> Vec2Raw {
    Vec2Raw { x: a.x - b.x, y: a.y - b.y }
}

/// Complex product `a · b`.
fn rot_product(a: RotRaw, b: RotRaw) -> RotRaw {
    let (are, aim): (i128, i128) = (a.re.into(), a.im.into());
    let (bre, bim): (i128, i128) = (b.re.into(), b.im.into());
    RotRaw { re: narrow(mul(are, bre) - mul(aim, bim)), im: narrow(mul(aim, bre) + mul(are, bim)) }
}

fn rot_within(actual: RotRaw, expected: RotRaw, tolerance: u64) -> bool {
    within(actual.re, expected.re, tolerance) && within(actual.im, expected.im, tolerance)
}

#[test]
fn test_tables_have_the_expected_sizes() {
    assert_eq!(pose2::cases().len(), 7);
    assert_eq!(pose2::chain_cases().len(), 3);
}

#[test]
fn test_rotation_product_is_the_plain_complex_product() {
    for case in pose2::cases() {
        let expected = rot_product(*case.a.rotation, *case.b.rotation);
        assert!(rot_within(*case.rot_mul, expected, 2), "rot_mul of {}", *case.id);
        // `Pose * Pose` composes the very same rotations.
        assert!(rot_within(*case.mul.rotation, expected, 2), "mul.rotation of {}", *case.id);
    }
}

#[test]
fn test_rotation_inverse_is_the_conjugate() {
    for case in pose2::cases() {
        let expected = RotRaw { re: *case.a.rotation.re, im: -*case.a.rotation.im };
        assert_eq!(*case.rot_inverse, expected, "{}", *case.id);
        assert_eq!(*case.inverse.rotation, expected, "{}", *case.id);
    }
}

#[test]
fn test_mul_translation_is_a_rotated_b_plus_a() {
    for case in pose2::cases() {
        let expected = add(rotate(*case.a.rotation, *case.b.translation), *case.a.translation);
        let tolerance = 4 + 2 * magnitude(*case.b.translation);
        assert!(vec2_within(*case.mul.translation, expected, tolerance), "mul of {}", *case.id);
    }
}

#[test]
fn test_inverse_translation_is_minus_the_rotated_translation() {
    for case in pose2::cases() {
        let expected = rotate_inverse(
            *case.a.rotation, Vec2Raw { x: -*case.a.translation.x, y: -*case.a.translation.y },
        );
        let tolerance = 4 + 2 * magnitude(*case.a.translation);
        assert!(
            vec2_within(*case.inverse.translation, expected, tolerance), "inverse of {}", *case.id,
        );
    }
}

#[test]
fn test_inv_mul_is_the_relative_pose() {
    for case in pose2::cases() {
        // pos12: rotation conj(a) · b, translation conj(a) · (t_b - t_a).
        let rotation = rot_product(
            RotRaw { re: *case.a.rotation.re, im: -*case.a.rotation.im }, *case.b.rotation,
        );
        assert!(
            rot_within(*case.inv_mul.rotation, rotation, 2), "inv_mul.rotation of {}", *case.id,
        );
        let delta = sub(*case.b.translation, *case.a.translation);
        let expected = rotate_inverse(*case.a.rotation, delta);
        let tolerance = 4 + 2 * magnitude(delta);
        assert!(
            vec2_within(*case.inv_mul.translation, expected, tolerance),
            "inv_mul.translation of {}",
            *case.id,
        );
    }
}

#[test]
fn test_inv_mul_of_equal_poses_is_the_identity() {
    let case = pose2::PAIR_SAME_POSE;
    assert_eq!(case.a, case.b);
    assert!(vec2_within(case.inv_mul.translation, Vec2Raw { x: 0, y: 0 }, 2));
    // conj(r) · r = |r|² = 1 ± 2^-31 for a rotation whose norm is 1 ± 2^-32.
    assert!(rot_within(case.inv_mul.rotation, RotRaw { re: 0x100000000, im: 0 }, 2));
}

#[test]
fn test_mul_by_the_snapped_inverse_is_close_to_the_identity() {
    let case = pose2::PAIR_POSE_TIMES_INVERSE;
    assert!(vec2_within(case.mul.translation, Vec2Raw { x: 0, y: 0 }, 4));
    assert!(rot_within(case.mul.rotation, RotRaw { re: 0x100000000, im: 0 }, 2));
}

#[test]
fn test_point_and_vector_transforms() {
    for case in pose2::cases() {
        let [p0, p1, p2] = *case.points;
        let points = array![p0, p1, p2];
        let [t0, t1, t2] = *case.transform_point;
        let transformed = array![t0, t1, t2];
        let [i0, i1, i2] = *case.inverse_transform_point;
        let inverse_transformed = array![i0, i1, i2];
        let [v0, v1, v2] = *case.transform_vector;
        let vectors = array![v0, v1, v2];
        let [w0, w1, w2] = *case.inverse_transform_vector;
        let inverse_vectors = array![w0, w1, w2];
        let mut k = 0;
        while k != 3 {
            let p = *points.at(k);
            let tolerance = 4 + 2 * (magnitude(p) + magnitude(*case.a.translation));
            // a · p = R p + t.
            let forward = add(rotate(*case.a.rotation, p), *case.a.translation);
            assert!(
                vec2_within(*transformed.at(k), forward, tolerance),
                "transform_point of {}",
                *case.id,
            );
            // a⁻¹ · p = R⁻¹ (p - t).
            let backward = rotate_inverse(*case.a.rotation, sub(p, *case.a.translation));
            assert!(
                vec2_within(*inverse_transformed.at(k), backward, tolerance),
                "inverse_transform_point of {}",
                *case.id,
            );
            // Vectors ignore the translation.
            assert!(
                vec2_within(*vectors.at(k), rotate(*case.a.rotation, p), tolerance),
                "transform_vector of {}",
                *case.id,
            );
            assert!(
                vec2_within(*inverse_vectors.at(k), rotate_inverse(*case.a.rotation, p), tolerance),
                "inverse_transform_vector of {}",
                *case.id,
            );
            k += 1;
        };
    }
}

#[test]
fn test_transform_point_then_inverse_returns_the_point() {
    for case in pose2::cases() {
        let [p0, _, _] = *case.points;
        let [t0, _, _] = *case.transform_point;
        // Rotating back with the conjugate multiplies by |r|² = 1 ± 2^-31 instead of 1.
        let back = rotate_inverse(*case.a.rotation, sub(t0, *case.a.translation));
        let tolerance = 4 + 2 * (magnitude(t0) + magnitude(*case.a.translation));
        assert!(vec2_within(back, p0, tolerance), "{}", *case.id);
    }
}

#[test]
fn test_chain_steps_have_a_norm_within_one_ulp_of_one() {
    for case in pose2::chain_cases() {
        let re: i128 = (*case.step.re).into();
        let im: i128 = (*case.step.im).into();
        // (re² + im²) in 2^-64 units, compared with 2^64: an error of k ulps is k · 2^33.
        let norm = re * re + im * im;
        let one = ONE * ONE;
        let error = if norm > one {
            norm - one
        } else {
            one - norm
        };
        assert!(error <= ONE, "step of {}", *case.id);
    }
}

#[test]
fn test_chain_drift_is_the_norm_minus_one() {
    for case in pose2::chain_cases() {
        for sample in case.samples.span() {
            let re: i128 = (*sample.rotation.re).into();
            let im: i128 = (*sample.rotation.im).into();
            let norm = narrow(mul(re, re) + mul(im, im));
            // Both are rounded from `f64`, the check recomputes the norm from rounded parts.
            assert!(within(norm, *sample.norm_squared, 4), "norm of {}", *case.id);
            assert!(
                within(*sample.norm_squared - 0x100000000, *sample.drift, 1),
                "drift of {}",
                *case.id,
            );
        }
    }
}

#[test]
fn test_chain_drift_grows_linearly_with_the_number_of_steps() {
    for case in pose2::chain_cases() {
        let [_, s10, s100, s1000] = *case.samples;
        assert_eq!((s10.steps, s100.steps, s1000.steps), (10, 100, 1000));
        // (1 + e)^n - 1 = n e up to n² e²: the quadratic term is far below one ulp here.
        assert!(within(s1000.drift, 10 * s100.drift, 16), "10 -> 100 of {}", *case.id);
        assert!(within(s100.drift, 10 * s10.drift, 16), "100 -> 1000 of {}", *case.id);
        assert!(s1000.drift != 0, "the drift of a 1000-step chain is measurable ({})", *case.id);
    }
}

#[test]
fn test_identity_first_pose_is_neutral() {
    let case = pose2::PAIR_IDENTITY_TRANSLATED;
    assert_eq!(case.mul, case.b);
    assert_eq!(case.inv_mul, case.b);
    let PoseRaw { translation, rotation } = case.inverse;
    assert_eq!((translation.x, translation.y, rotation.re, rotation.im), (0, 0, 0x100000000, 0));
}
