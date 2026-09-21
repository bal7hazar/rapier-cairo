//! Sanity checks of the golden fixtures against closed-form results, computed here with plain
//! integer arithmetic on the raw Q32.32 values. They guard the harness (wrong field, wrong unit,
//! wrong sign), not the port.

use rapier_golden::compare::{vec2_within, within};
use rapier_golden::types::{ManifoldCase, ShapeRaw, Vec2Raw};
use rapier_golden::{aabb, contact_manifolds, integration_parameters, mass_properties};

const ONE: i128 = 0x100000000;
/// round(π · 2^32)
const PI: i128 = 13493037705;

/// Q32.32 product, truncated toward zero.
fn mul(a: i128, b: i128) -> i128 {
    a * b / ONE
}

fn narrow(value: i128) -> i64 {
    value.try_into().unwrap()
}

/// `n · n` for a raw vector; 2^32 for a unit vector.
fn norm_squared(v: Vec2Raw) -> i64 {
    narrow(mul(v.x.into(), v.x.into()) + mul(v.y.into(), v.y.into()))
}

#[test]
fn test_tables_have_the_expected_sizes() {
    assert_eq!(integration_parameters::cases().len(), 2);
    assert_eq!(mass_properties::cases().len(), 11);
    assert_eq!(mass_properties::body_cases().len(), 1);
    assert_eq!(aabb::cases().len(), 32);
    // 12 pairs x 6 regimes + 7 extra degenerate cases + 8 flipped-order cases.
    assert_eq!(contact_manifolds::cases().len(), 87);
}

#[test]
fn test_ball_mass_is_density_pi_r_squared() {
    let mut checked = 0_u32;
    for case in mass_properties::cases() {
        if let ShapeRaw::Ball(radius) = *case.shape {
            let r: i128 = radius.into();
            let density: i128 = (*case.density).into();
            let mass = mul(mul(mul(PI, r), r), density);
            // Three truncating products plus the rounding of π and of the radius itself.
            assert!(within(*case.expected.mass, narrow(mass), 16), "mass of {}", *case.id);
            // Disc: I = m r² / 2.
            let inertia = mul(mul(mass, r), r) / 2;
            assert!(
                within(*case.expected.principal_inertia, narrow(inertia), 16),
                "inertia of {}",
                *case.id,
            );
            checked += 1;
        }
    }
    assert_eq!(checked, 4);
}

#[test]
fn test_cuboid_mass_and_inertia() {
    let mut checked = 0_u32;
    for case in mass_properties::cases() {
        if let ShapeRaw::Cuboid(half_extents) = *case.shape {
            let hx: i128 = half_extents.x.into();
            let hy: i128 = half_extents.y.into();
            let density: i128 = (*case.density).into();
            let mass = 4 * mul(mul(hx, hy), density);
            assert!(within(*case.expected.mass, narrow(mass), 16), "mass of {}", *case.id);
            // Rectangle: I = m (hx² + hy²) / 3.
            let inertia = mul(mass, mul(hx, hx) + mul(hy, hy)) / 3;
            assert!(
                within(*case.expected.principal_inertia, narrow(inertia), 16),
                "inertia of {}",
                *case.id,
            );
            assert_eq!(*case.expected.local_com, Vec2Raw { x: 0, y: 0 });
            checked += 1;
        }
    }
    assert_eq!(checked, 3);
}

#[test]
fn test_mass_times_inverse_mass_is_one() {
    for case in mass_properties::cases() {
        let mass: i128 = (*case.expected.mass).into();
        let inv_mass: i128 = (*case.expected.inv_mass).into();
        // Each factor is rounded to ±0.5 ulp, which the product scales by the other factor.
        let tolerance: u64 = ((mass + inv_mass) / ONE + 2).try_into().unwrap();
        assert!(within(narrow(mul(mass, inv_mass)), narrow(ONE), tolerance), "{}", *case.id);
    }
}

#[test]
fn test_ball_aabb_is_centre_plus_minus_radius() {
    let mut checked = 0_u32;
    for case in aabb::cases() {
        if let ShapeRaw::Ball(radius) = *case.shape {
            let centre = *case.pose.translation;
            let mins = Vec2Raw { x: centre.x - radius, y: centre.y - radius };
            let maxs = Vec2Raw { x: centre.x + radius, y: centre.y + radius };
            assert!(vec2_within(*case.mins, mins, 1), "mins of {}", *case.id);
            assert!(vec2_within(*case.maxs, maxs, 1), "maxs of {}", *case.id);
            checked += 1;
        }
    }
    assert_eq!(checked, 8);
}

#[test]
fn test_aabb_mins_are_below_maxs() {
    for case in aabb::cases() {
        assert!(*case.mins.x < *case.maxs.x, "x order of {}", *case.id);
        assert!(*case.mins.y < *case.maxs.y, "y order of {}", *case.id);
    }
}

#[test]
fn test_cuboid_aabb_under_quarter_turn_swaps_the_half_extents() {
    let case = aabb::CUBOID_ROT90;
    let ShapeRaw::Cuboid(half_extents) = case.shape else {
        panic!("not a cuboid")
    };
    let centre = case.pose.translation;
    let mins = Vec2Raw { x: centre.x - half_extents.y, y: centre.y - half_extents.x };
    let maxs = Vec2Raw { x: centre.x + half_extents.y, y: centre.y + half_extents.x };
    assert_eq!(case.mins, mins);
    assert_eq!(case.maxs, maxs);
}

#[test]
fn test_separated_regime_has_no_contact_point() {
    let separated = [
        contact_manifolds::BALL_BALL_SEPARATED, contact_manifolds::BALL_CUBOID_SEPARATED,
        contact_manifolds::BALL_CAPSULE_SEPARATED, contact_manifolds::CUBOID_CUBOID_SEPARATED,
        contact_manifolds::CUBOID_CAPSULE_SEPARATED, contact_manifolds::CAPSULE_CAPSULE_SEPARATED,
        contact_manifolds::HALFSPACE_BALL_SEPARATED, contact_manifolds::HALFSPACE_CUBOID_SEPARATED,
        contact_manifolds::SEGMENT_BALL_SEPARATED, contact_manifolds::HALFSPACE_CAPSULE_SEPARATED,
        contact_manifolds::HALFSPACE_SEGMENT_SEPARATED, contact_manifolds::CUBOID_SEGMENT_SEPARATED,
    ];
    for case in separated.span() {
        assert_eq!(*case.num_points, 0, "{}", *case.id);
    }
}

#[test]
fn test_within_prediction_regime_has_a_positive_distance_below_prediction() {
    let within_prediction = [
        contact_manifolds::BALL_BALL_WITHIN_PRED, contact_manifolds::BALL_CUBOID_WITHIN_PRED,
        contact_manifolds::BALL_CAPSULE_WITHIN_PRED, contact_manifolds::CUBOID_CUBOID_WITHIN_PRED,
        contact_manifolds::CUBOID_CAPSULE_WITHIN_PRED,
        contact_manifolds::CAPSULE_CAPSULE_WITHIN_PRED,
        contact_manifolds::HALFSPACE_BALL_WITHIN_PRED,
        contact_manifolds::HALFSPACE_CUBOID_WITHIN_PRED,
        contact_manifolds::SEGMENT_BALL_WITHIN_PRED,
        contact_manifolds::HALFSPACE_CAPSULE_WITHIN_PRED,
        contact_manifolds::HALFSPACE_SEGMENT_WITHIN_PRED,
        contact_manifolds::CUBOID_SEGMENT_WITHIN_PRED,
    ];
    for case in within_prediction.span() {
        assert!(*case.num_points >= 1, "{}", *case.id);
        let [first, _] = *case.points;
        assert!(first.dist > 0, "{}", *case.id);
        assert!(first.dist < contact_manifolds::PREDICTION, "{}", *case.id);
    }
}

#[test]
fn test_touching_regime_has_a_zero_distance() {
    let touching = [
        contact_manifolds::BALL_BALL_TOUCHING, contact_manifolds::BALL_CUBOID_TOUCHING,
        contact_manifolds::BALL_CAPSULE_TOUCHING, contact_manifolds::CUBOID_CUBOID_TOUCHING,
        contact_manifolds::CUBOID_CAPSULE_TOUCHING, contact_manifolds::CAPSULE_CAPSULE_TOUCHING,
        contact_manifolds::HALFSPACE_BALL_TOUCHING, contact_manifolds::HALFSPACE_CUBOID_TOUCHING,
        contact_manifolds::SEGMENT_BALL_TOUCHING, contact_manifolds::HALFSPACE_CAPSULE_TOUCHING,
        contact_manifolds::HALFSPACE_SEGMENT_TOUCHING, contact_manifolds::CUBOID_SEGMENT_TOUCHING,
    ];
    for case in touching.span() {
        assert!(*case.num_points >= 1, "{}", *case.id);
        let [first, _] = *case.points;
        assert!(within(first.dist, 0, 4), "{}", *case.id);
    }
}

#[test]
fn test_shallow_regime_has_a_contact_point() {
    let shallow = [
        contact_manifolds::BALL_BALL_SHALLOW, contact_manifolds::BALL_CUBOID_SHALLOW,
        contact_manifolds::BALL_CAPSULE_SHALLOW, contact_manifolds::CUBOID_CUBOID_SHALLOW,
        contact_manifolds::CUBOID_CAPSULE_SHALLOW, contact_manifolds::CAPSULE_CAPSULE_SHALLOW,
        contact_manifolds::HALFSPACE_BALL_SHALLOW, contact_manifolds::HALFSPACE_CUBOID_SHALLOW,
        contact_manifolds::SEGMENT_BALL_SHALLOW, contact_manifolds::HALFSPACE_CAPSULE_SHALLOW,
        contact_manifolds::HALFSPACE_SEGMENT_SHALLOW, contact_manifolds::CUBOID_SEGMENT_SHALLOW,
        contact_manifolds::CUBOID_BALL_SHALLOW, contact_manifolds::CAPSULE_BALL_SHALLOW,
        contact_manifolds::BALL_HALFSPACE_SHALLOW, contact_manifolds::BALL_SEGMENT_SHALLOW,
        contact_manifolds::CAPSULE_CUBOID_SHALLOW, contact_manifolds::CAPSULE_HALFSPACE_SHALLOW,
        contact_manifolds::SEGMENT_HALFSPACE_SHALLOW, contact_manifolds::SEGMENT_CUBOID_SHALLOW,
    ];
    for case in shallow.span() {
        assert!(*case.num_points >= 1, "{}", *case.id);
    }
}

#[test]
fn test_deep_regime_has_a_contact_point() {
    let deep = [
        contact_manifolds::BALL_BALL_DEEP, contact_manifolds::BALL_CUBOID_DEEP,
        contact_manifolds::BALL_CAPSULE_DEEP, contact_manifolds::CUBOID_CUBOID_DEEP,
        contact_manifolds::CUBOID_CAPSULE_DEEP, contact_manifolds::CAPSULE_CAPSULE_DEEP,
        contact_manifolds::HALFSPACE_BALL_DEEP, contact_manifolds::HALFSPACE_CUBOID_DEEP,
        contact_manifolds::SEGMENT_BALL_DEEP, contact_manifolds::HALFSPACE_CAPSULE_DEEP,
        contact_manifolds::HALFSPACE_SEGMENT_DEEP, contact_manifolds::CUBOID_SEGMENT_DEEP,
    ];
    for case in deep.span() {
        assert!(*case.num_points >= 1, "{}", *case.id);
    }
}

/// `local_n2 = -R12⁻¹ · local_n1`, with `R12⁻¹ = (re, -im)`.
fn assert_normals_are_consistent(case: @ManifoldCase) {
    let re: i128 = (*case.pos12.rotation.re).into();
    let im: i128 = (*case.pos12.rotation.im).into();
    let n1 = *case.local_n1;
    let expected = Vec2Raw {
        x: narrow(-(mul(re, n1.x.into()) + mul(im, n1.y.into()))),
        y: narrow(-(mul(re, n1.y.into()) - mul(im, n1.x.into()))),
    };
    assert!(vec2_within(*case.local_n2, expected, 4), "n2 of {}", *case.id);
}

#[test]
fn test_contact_normals_are_unit_and_opposite() {
    let mut with_contact = 0_u32;
    for case in contact_manifolds::cases() {
        if *case.num_points != 0 {
            assert!(within(norm_squared(*case.local_n1), narrow(ONE), 4), "|n1| of {}", *case.id);
            assert!(within(norm_squared(*case.local_n2), narrow(ONE), 4), "|n2| of {}", *case.id);
            assert_normals_are_consistent(case);
            with_contact += 1;
        }
    }
    // Everything but the 12 separated cases.
    assert_eq!(with_contact, 75);
}

#[test]
fn test_unused_contact_points_are_zeroed() {
    for case in contact_manifolds::cases() {
        let [first, second] = *case.points;
        if *case.num_points < 2 {
            assert_eq!(second.dist, 0, "{}", *case.id);
            assert_eq!(second.fid1, 0, "{}", *case.id);
        }
        if *case.num_points < 1 {
            assert_eq!(first.dist, 0, "{}", *case.id);
            assert_eq!(first.fid1, 0, "{}", *case.id);
        }
    }
}

#[test]
fn test_integration_defaults() {
    let defaults = integration_parameters::DEFAULTS;
    assert_eq!(defaults.num_solver_iterations, 4);
    assert_eq!(defaults.length_unit, narrow(ONE));
    assert_eq!(defaults.warmstart_coefficient, narrow(ONE));
    // 0.02, the default prediction distance, is what the manifold family uses.
    assert_eq!(defaults.normalized_prediction_distance, contact_manifolds::PREDICTION);
    assert_eq!(defaults.contact_softness.natural_frequency, narrow(30 * ONE));
    assert_eq!(defaults.static_contact_softness.natural_frequency, narrow(60 * ONE));
}

#[test]
fn test_integration_derived_quantities_are_consistent() {
    for case in integration_parameters::cases() {
        let iterations: i64 = (*case.num_solver_iterations).into();
        assert!(within(*case.substep_dt * iterations, *case.dt, 4), "substep of {}", *case.id);
        // dt * inv_dt = 1; inv_dt = 60 amplifies the rounding of dt.
        let one = mul((*case.dt).into(), (*case.inv_dt).into());
        assert!(within(narrow(one), narrow(ONE), 64), "inv_dt of {}", *case.id);
        // cfm_factor = 1 / (1 + cfm_coeff)
        let one = mul((*case.contact.cfm_factor).into(), ONE + (*case.contact.cfm_coeff).into());
        assert!(within(narrow(one), narrow(ONE), 4), "cfm of {}", *case.id);
        // erp = substep_dt * erp_inv_dt
        let erp = mul((*case.substep_dt).into(), (*case.contact.erp_inv_dt).into());
        assert!(within(narrow(erp), *case.contact.erp, 16), "erp of {}", *case.id);
    }
}
