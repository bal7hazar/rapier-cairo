//! Unit tests of the character controller's pure parts (options, dims, hit classification and
//! decomposition) and the `gas_*` probes of the rejected formulations. Scenes are in
//! `tests_scenes`; the golden comparison in `tests/control_golden.cairo`.

use fixed::{FRAC_PI_2, FRAC_PI_4, Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
use glam_core::vec2::Vec2;
use rapier_geometry2d::query::{ShapeCastHit, ShapeCastStatus};
use rapier_geometry2d::shape::{Ball, CapsuleTrait, CuboidTrait, Shape};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use super::alternatives::{hit_info_cosine, manifolds_metered};
use super::contacts::manifolds;
use super::motion::{compute_hit_info, decompose_hit, handle_slopes, subtract_hit};
use super::{
    CharacterAutostep, CharacterLength, CharacterLengthTrait, DEFAULT_NUDGE, DEFAULT_OFFSET,
    DEFAULT_SNAP, EffectiveCharacterMovement, HitDecompositionTrait, KinematicCharacterController,
    compute_dims, try_normalize_and_get_length,
};

fn ratio(n: i64, d: i64) -> Fixed {
    FixedTrait::from_ratio(n, d)
}

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

/// cos 30 degrees (Q32.32).
fn cos30() -> Fixed {
    Fixed { raw: 3719550787 }
}

fn hit(normal: Vec2) -> ShapeCastHit {
    ShapeCastHit {
        time_of_impact: ZERO,
        witness1: v(ZERO, ZERO),
        witness2: v(ZERO, ZERO),
        normal1: normal,
        normal2: -normal,
        status: ShapeCastStatus::Converged,
    }
}

#[test]
fn test_defaults() {
    let c: KinematicCharacterController = Default::default();
    assert_eq!(c.up, v(ZERO, ONE));
    assert_eq!(c.offset, CharacterLength::Relative(DEFAULT_OFFSET));
    assert!(c.slide && c.autostep.is_none());
    assert_eq!((c.max_slope_climb_angle, c.min_slope_slide_angle), (FRAC_PI_4, FRAC_PI_4));
    assert_eq!(c.snap_to_ground, Some(CharacterLength::Relative(DEFAULT_SNAP)));
    assert_eq!(c.normal_nudge_factor, DEFAULT_NUDGE);
    let a: CharacterAutostep = Default::default();
    assert_eq!(a.max_height, CharacterLength::Relative(ratio(1, 4)));
    assert_eq!(a.min_width, CharacterLength::Relative(HALF));
    assert!(a.include_dynamic_bodies);
}

#[test]
fn test_character_length() {
    let rel = CharacterLength::Relative(ratio(1, 4));
    let abs = CharacterLength::Absolute(ratio(3, 2));
    assert_eq!((rel.eval(TWO), abs.eval(TWO)), (HALF, ratio(3, 2)));
    let double = |x: Fixed| x * TWO;
    assert_eq!(abs.map_absolute(double), CharacterLength::Absolute(ratio(3, 1)));
    assert_eq!(rel.map_absolute(double), rel);
    let double = |x: Fixed| x * TWO;
    assert_eq!(rel.map_relative(double), CharacterLength::Relative(HALF));
    assert_eq!(abs.map_relative(|x: Fixed| x + ONE), abs);
}

/// `(up, shape, side, height)`.
#[test]
fn test_compute_dims() {
    let up_y = v(ZERO, ONE);
    let up_x = v(ONE, ZERO);
    let cases: Array<(Vec2, Shape, Fixed, Fixed)> = array![
        (up_y, Shape::Ball(Ball { radius: HALF }), ONE, ONE),
        (up_y, Shape::Cuboid(CuboidTrait::new(v(ratio(3, 10), HALF))), ratio(3, 10) * TWO, ONE),
        (up_x, Shape::Cuboid(CuboidTrait::new(v(ratio(3, 10), HALF))), ONE, ratio(3, 10) * TWO),
        (up_y, Shape::Capsule(CapsuleTrait::new(v(ZERO, -HALF), v(ZERO, HALF), HALF)), ONE, TWO),
    ];
    for (up, shape, side, height) in cases {
        let dims = compute_dims(up, shape);
        assert!((dims.x - side).abs() <= Fixed { raw: 2 }, "side {:?}", dims);
        assert_eq!(dims.y, height);
    }
}

#[test]
fn test_try_normalize_and_get_length() {
    assert!(try_normalize_and_get_length(v(ZERO, ZERO), Fixed { raw: 42950 }).is_none());
    assert!(
        try_normalize_and_get_length(v(Fixed { raw: 42950 }, ZERO), Fixed { raw: 42950 }).is_none(),
    );
    let (dir, len) = try_normalize_and_get_length(v(ratio(3, 1), ratio(4, 1)), Fixed { raw: 42950 })
        .unwrap();
    assert_eq!(len, ratio(5, 1));
    assert!(
        (dir.x - ratio(3, 5)).abs() <= Fixed { raw: 2 }
            && (dir.y - ratio(4, 5)).abs() <= Fixed { raw: 2 },
    );
}

/// Upstream's signed floor angle: `(normal, is_wall, is_nonslip_slope)` with `up = +Y`, climb and
/// slide limits at pi / 4. A surface leaning right of `up` (normal `x > 0`) has a negative angle.
#[test]
fn test_hit_info_signed_angle() {
    let c: KinematicCharacterController = Default::default();
    let cases: Array<(Vec2, bool, bool)> = array![
        (v(ZERO, ONE), false, true), (v(-cos30(), HALF), true, false),
        (v(cos30(), HALF), false, true), (v(-HALF, cos30()), false, true),
        (v(HALF, cos30()), false, true), (v(-ONE, ZERO), true, false), (v(ONE, ZERO), false, true),
        (v(-HALF, -cos30()), false, false),
    ];
    for (n, wall, nonslip) in cases {
        let info = compute_hit_info(c, hit(n));
        assert!(info.is_wall == wall && info.is_nonslip_slope == nonslip, "normal {:?}", n);
        let alt = hit_info_cosine(c, hit(n));
        assert!(alt.is_wall == wall && alt.is_nonslip_slope == nonslip, "cosine {:?}", n);
    }
    // A vertical climb limit: only surfaces past the vertical are walls.
    let steep = KinematicCharacterController {
        max_slope_climb_angle: FRAC_PI_2 + ratio(1, 10), ..c,
    };
    assert!(!compute_hit_info(steep, hit(v(-ONE, ZERO))).is_wall);
}

#[test]
fn test_decompose_hit() {
    let up = hit(v(ZERO, ONE));
    // Into the surface: the penetration part is dropped.
    let d = decompose_hit(v(ONE, -ONE), up);
    assert_eq!(
        (d.normal_part, d.horizontal_tangent, d.vertical_tangent),
        (v(ZERO, ZERO), v(ZERO, ZERO), v(ONE, ZERO)),
    );
    assert_eq!(d.unconstrained_slide_part(), v(ONE, ZERO));
    // Away from it: kept as the normal part.
    let d = decompose_hit(v(ONE, HALF), up);
    assert_eq!((d.normal_part, d.vertical_tangent), (v(ZERO, HALF), v(ONE, ZERO)));
    assert_eq!(d.unconstrained_slide_part(), v(ONE, HALF));
}

#[test]
fn test_subtract_hit() {
    let up = hit(v(ZERO, ONE));
    let out = subtract_hit(v(ONE, -ONE), up);
    assert_eq!(out.x, ONE);
    assert_eq!(out.y, Fixed { raw: 42950 });
    assert_eq!(subtract_hit(v(ONE, ONE), up), v(ONE, ONE));
}

/// `handle_slopes` on a flat floor and on a wall: `(normal, input, remaining, allowed,
/// sliding)`, the nudge `1e-4` along the normal included.
#[test]
fn test_handle_slopes() {
    let c: KinematicCharacterController = Default::default();
    let nudge = DEFAULT_NUDGE;
    let cases: Array<(Vec2, Vec2, Vec2, Vec2, bool)> = array![
        // Walking on the floor: the fall is removed, the walk kept.
        (v(ZERO, ONE), v(ONE, -HALF), v(ONE, -HALF), v(ONE, nudge), true),
        // Into a wall without climbing intent: the climb along the wall is removed.
        (v(-ONE, ZERO), v(ONE, ZERO), v(ONE, HALF), v(-nudge, ZERO), false),
        // Into a wall with climbing intent: slide up.
        (v(-ONE, ZERO), v(ONE, ONE), v(ONE, HALF), v(-nudge, HALF), true),
    ];
    for (n, input, remaining, allowed, sliding) in cases {
        let mut result = EffectiveCharacterMovement {
            translation: v(ZERO, ZERO), grounded: false, is_sliding_down_slope: false,
        };
        let info = compute_hit_info(c, hit(n));
        let got = handle_slopes(c, info, input, remaining, c.normal_nudge_factor, ref result);
        assert_eq!(got, allowed);
        assert_eq!(result.is_sliding_down_slope, sliding);
    }
}

/// The two manifold formulations agree.
#[test]
fn test_manifolds_metered_agrees() {
    let ball = Shape::Ball(Ball { radius: HALF });
    let ground = Shape::Cuboid(CuboidTrait::new(v(ratio(10, 1), HALF)));
    let pos12 = Pose2Trait::new(v(ratio(1, 10), -ratio(101, 100)), Rot2 { re: ONE, im: ZERO });
    let a = manifolds(pos12, ball, ground, ratio(6, 100));
    let b = manifolds_metered(pos12, ball, ground, ratio(6, 100));
    assert_eq!(a.len(), 1);
    assert_eq!(a, b);
}

// ---------------------------------------------------------------------------------------------
// Gas probes (subtract `gas_baseline`).

#[test]
fn gas_baseline() {
    let _ = opaque(hit(v(-cos30(), HALF)));
    let _: KinematicCharacterController = opaque(Default::default());
}

#[test]
fn gas_hit_info_angle_to() {
    let h = opaque(hit(v(-cos30(), HALF)));
    let c: KinematicCharacterController = opaque(Default::default());
    let _ = opaque(compute_hit_info(c, h).is_wall);
}

#[test]
fn gas_hit_info_cosine() {
    let h = opaque(hit(v(-cos30(), HALF)));
    let c: KinematicCharacterController = opaque(Default::default());
    let _ = opaque(hit_info_cosine(c, h).is_wall);
}

#[test]
fn gas_manifolds_metered() {
    let _ = opaque(hit(v(-cos30(), HALF)));
    let _: KinematicCharacterController = opaque(Default::default());
    let ball = opaque(Shape::Ball(Ball { radius: HALF }));
    let ground = opaque(Shape::Cuboid(CuboidTrait::new(v(ratio(10, 1), HALF))));
    let pos12: Pose2 = opaque(
        Pose2Trait::new(v(ratio(1, 10), -ratio(101, 100)), Rot2 { re: ONE, im: ZERO }),
    );
    let _ = opaque(manifolds_metered(pos12, ball, ground, ratio(6, 100)));
}

#[test]
fn gas_manifolds_step() {
    let _ = opaque(hit(v(-cos30(), HALF)));
    let _: KinematicCharacterController = opaque(Default::default());
    let ball = opaque(Shape::Ball(Ball { radius: HALF }));
    let ground = opaque(Shape::Cuboid(CuboidTrait::new(v(ratio(10, 1), HALF))));
    let pos12: Pose2 = opaque(
        Pose2Trait::new(v(ratio(1, 10), -ratio(101, 100)), Rot2 { re: ONE, im: ZERO }),
    );
    let _ = opaque(manifolds(pos12, ball, ground, ratio(6, 100)));
}
