//! Cuboid manifolds: persistent contacts, two-way SAT, then feature clipping.
//! First-axis ties, clipping order and f32 feature identifiers follow Parry.
use fixed::Fixed;
use glam::Vec2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::contact::{ContactManifold, ContactManifoldTrait};
use crate::manifold::ManifoldTrait;
use crate::polygonal_feature::PolygonalFeatureTrait;
use crate::sat::cuboid_cuboid_find_local_separating_normal_oneway;
use crate::shape::{Cuboid, CuboidTrait, Shape, ShapeTrait};

/// Updates `manifold` for `cuboid2` placed at `pos12` in `cuboid1`'s frame.
/// Tries persistence first, then clears separated pairs (`separation > prediction`).
/// Keeps all clipped points, including distances beyond prediction, and transfers
/// impulses by feature ids. Ties select shape 1's SAT normal.
/// Requires nonnegative half extents, a unit rotation and representable Q32.32
/// intermediates; coordinates bounded by 8192 suffice for clipping projections.
/// Products and ratios floor; SAT normalization rounds nearest. Panics on fixed
/// overflow or `Clip: projection range`, as the underlying SAT/clipping kernels.
#[inline(never)]
pub fn contact_manifold_cuboid_cuboid(
    pos12: Pose2,
    cuboid1: Cuboid,
    cuboid2: Cuboid,
    prediction: Fixed,
    ref manifold: ContactManifold,
) {
    if manifold.try_update_contacts(pos12) {
        return;
    }
    let pos21 = pos12.inverse();
    let (sep1, axis1) = cuboid_cuboid_find_local_separating_normal_oneway(cuboid1, cuboid2, pos12);
    if sep1 > prediction {
        manifold.clear();
        return;
    }
    let (sep2, axis2) = cuboid_cuboid_find_local_separating_normal_oneway(cuboid2, cuboid1, pos21);
    if sep2 > prediction {
        manifold.clear();
        return;
    }
    let normal = if sep2 > sep1 {
        pos12.transform_vector(-axis2)
    } else {
        axis1
    };
    generate_contacts(pos12, pos21, cuboid1, cuboid2, normal, ref manifold);
}

/// Dispatches only a cuboid/cuboid pair, returning `true` even when separated.
/// Returns `false` without changing `manifold` for any other shape combination.
/// Arguments, rounding, valid ranges and panics are those of the typed generator.
pub fn contact_manifold_cuboid_cuboid_shapes(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    if let (Some(cuboid1), Some(cuboid2)) = (shape1.as_cuboid(), shape2.as_cuboid()) {
        contact_manifold_cuboid_cuboid(pos12, cuboid1, cuboid2, prediction, ref manifold);
        true
    } else {
        false
    }
}

#[inline(never)]
fn generate_contacts(
    pos12: Pose2,
    pos21: Pose2,
    cuboid1: Cuboid,
    cuboid2: Cuboid,
    normal: Vec2,
    ref manifold: ContactManifold,
) {
    let old = manifold;
    manifold.clear();
    let local_n2 = pos21.transform_vector(-normal);
    // contacts transforms feature 2, clips in frame 1, and restores local anchors.
    PolygonalFeatureTrait::contacts(
        pos12,
        pos21,
        normal,
        local_n2,
        cuboid1.support_feature(normal),
        cuboid2.support_feature(local_n2),
        ref manifold,
        false,
    );
    manifold.local_n1 = normal;
    manifold.local_n2 = local_n2;
    manifold.match_contacts(@old);
}

#[cfg(test)]
mod alternatives {
    use super::{
        ContactManifold, ContactManifoldTrait, Cuboid, Fixed, ManifoldTrait, Pose2, Pose2Trait,
        cuboid_cuboid_find_local_separating_normal_oneway, generate_contacts,
    };

    // The brief calls this upstream order, but the read-only upstream clone already
    // exits after sep1. Sierra gas ties on all six regimes; retain upstream order.
    #[inline(never)]
    pub fn eager_sat(
        pos12: Pose2,
        cuboid1: Cuboid,
        cuboid2: Cuboid,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) {
        if manifold.try_update_contacts(pos12) {
            return;
        }
        let pos21 = pos12.inverse();
        let (sep1, axis1) = cuboid_cuboid_find_local_separating_normal_oneway(
            cuboid1, cuboid2, pos12,
        );
        let (sep2, axis2) = cuboid_cuboid_find_local_separating_normal_oneway(
            cuboid2, cuboid1, pos21,
        );
        if sep1 > prediction || sep2 > prediction {
            manifold.clear();
            return;
        }
        let normal = if sep2 > sep1 {
            pos12.transform_vector(-axis2)
        } else {
            axis1
        };
        generate_contacts(pos12, pos21, cuboid1, cuboid2, normal, ref manifold);
    }
    // Rejected: splitting persistence from regeneration costs more Sierra gas.
    #[inline(never)]
    pub fn split_short(
        pos12: Pose2,
        cuboid1: Cuboid,
        cuboid2: Cuboid,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) {
        if manifold.try_update_contacts(pos12) {
            return;
        }
        regenerate_short(pos12, cuboid1, cuboid2, prediction, ref manifold);
    }

    #[inline(never)]
    fn regenerate_short(
        pos12: Pose2,
        cuboid1: Cuboid,
        cuboid2: Cuboid,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) {
        let pos21 = pos12.inverse();
        let (sep1, axis1) = cuboid_cuboid_find_local_separating_normal_oneway(
            cuboid1, cuboid2, pos12,
        );
        if sep1 > prediction {
            manifold.clear();
            return;
        }
        let (sep2, axis2) = cuboid_cuboid_find_local_separating_normal_oneway(
            cuboid2, cuboid1, pos21,
        );
        if sep2 > prediction {
            manifold.clear();
            return;
        }
        let normal = if sep2 > sep1 {
            pos12.transform_vector(-axis2)
        } else {
            axis1
        };
        generate_contacts(pos12, pos21, cuboid1, cuboid2, normal, ref manifold);
    }
}

#[cfg(test)]
mod tests {
    use fixed::{ONE, TWO, ZERO};
    use rapier_golden::contact_manifolds;
    use rapier_golden::types::{ManifoldCase, ShapeRaw};
    use rapier_math::pose2::IDENTITY;
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::contact::ContactData;
    use crate::shape::Ball;
    use super::*;

    fn inputs(c: ManifoldCase) -> (Pose2, Cuboid, Cuboid, Fixed) {
        let ShapeRaw::Cuboid(h1) = c.shape1 else {
            panic!("cuboid fixture");
        };
        let ShapeRaw::Cuboid(h2) = c.shape2 else {
            panic!("cuboid fixture");
        };
        (
            Pose2 {
                translation: Vec2 {
                    x: Fixed { raw: c.pos12.translation.x },
                    y: Fixed { raw: c.pos12.translation.y },
                },
                rotation: Rot2 {
                    re: Fixed { raw: c.pos12.rotation.re }, im: Fixed { raw: c.pos12.rotation.im },
                },
            },
            Cuboid { half_extents: Vec2 { x: Fixed { raw: h1.x }, y: Fixed { raw: h1.y } } },
            Cuboid { half_extents: Vec2 { x: Fixed { raw: h2.x }, y: Fixed { raw: h2.y } } },
            Fixed { raw: contact_manifolds::PREDICTION },
        )
    }
    fn compare(p: Pose2, a: Cuboid, b: Cuboid, prediction: Fixed, initial: ContactManifold) {
        let mut short = initial;
        let mut eager = initial;
        contact_manifold_cuboid_cuboid(p, a, b, prediction, ref short);
        alternatives::eager_sat(p, a, b, prediction, ref eager);
        assert_eq!(short, eager);
        let mut split = initial;
        alternatives::split_short(p, a, b, prediction, ref split);
        assert_eq!(short, split);
    }
    #[test]
    fn test_candidates_all_goldens_both_orders() {
        let mut count = 0;
        for c in contact_manifolds::cases() {
            if let (ShapeRaw::Cuboid(_), ShapeRaw::Cuboid(_)) = (*c.shape1, *c.shape2) {
                let (p, a, b, prediction) = inputs(*c);
                compare(p, a, b, prediction, Default::default());
                compare(p.inverse(), b, a, prediction, Default::default());
                count += 1;
            }
        }
        assert_eq!(count, 8);
    }
    #[test]
    fn test_persistence_and_regeneration_transfer_impulses() {
        let (mut p, a, b, prediction) = inputs(contact_manifolds::CUBOID_CUBOID_WITHIN_PRED);
        let mut m: ContactManifold = Default::default();
        contact_manifold_cuboid_cuboid(p, a, b, prediction, ref m);
        let [mut first, mut second] = m.points;
        first
            .data =
                ContactData {
                    impulse: ONE,
                    tangent_impulse: TWO,
                    warmstart_impulse: TWO,
                    warmstart_tangent_impulse: ONE,
                };
        second.data = ContactData { impulse: TWO, ..first.data };
        m.points = [first, second];
        let old = m;
        // Tangential motion changes regenerated anchors, but persistence must keep them.
        p.translation.y = p.translation.y + Fixed { raw: 1000 };
        p.translation.x = p.translation.x + Fixed { raw: 1000 };
        contact_manifold_cuboid_cuboid(p, a, b, ZERO, ref m);
        assert_eq!(m.num_points, 2);
        for i in array![0_u8, 1].span() {
            let now = m.point(*i);
            let before = old.point(*i);
            assert_eq!(now.local_p1, before.local_p1);
            assert_eq!(now.local_p2, before.local_p2);
            assert_eq!((now.fid1, now.fid2), (before.fid1, before.fid2));
            assert_eq!(now.data, before.data);
            assert_eq!(now.dist, before.dist + Fixed { raw: 1000 });
        }
        compare(p, a, b, ZERO, old);
        // Exceed the persistence tangent tolerance: anchors change, fids keep data.
        p.translation.y = p.translation.y + Fixed { raw: 42949673 };
        contact_manifold_cuboid_cuboid(p, a, b, prediction, ref m);
        assert_ne!(m.point(0).local_p1, old.point(0).local_p1);
        assert_eq!(m.point(0).data, first.data);
        assert_eq!(m.point(1).data, second.data);
        compare(p, a, b, prediction, old);
    }
    #[test]
    fn test_dispatch_and_clear_preserve_metadata() {
        let (p, a, b, prediction) = inputs(contact_manifolds::CUBOID_CUBOID_TOUCHING);
        let mut m: ContactManifold = Default::default();
        m.subshape1 = 4;
        m.data.user_data = 42;
        assert!(
            contact_manifold_cuboid_cuboid_shapes(
                p, Shape::Cuboid(a), Shape::Cuboid(b), prediction, ref m,
            ),
        );
        let old = m;
        let ball = Shape::Ball(Ball { radius: ONE });
        for (s1, s2) in array![(ball, Shape::Cuboid(b)), (Shape::Cuboid(a), ball), (ball, ball)]
            .span() {
            assert!(!contact_manifold_cuboid_cuboid_shapes(p, *s1, *s2, prediction, ref m));
            assert_eq!(m, old);
        }
        let far = Pose2 { translation: Vec2 { x: TWO + TWO, y: ONE }, ..IDENTITY };
        assert!(
            contact_manifold_cuboid_cuboid_shapes(
                far, Shape::Cuboid(a), Shape::Cuboid(b), prediction, ref m,
            ),
        );
        assert_eq!(m.num_points, 0);
        assert_eq!(m.points, old.points);
        assert_eq!(m.local_n1, old.local_n1);
        assert_eq!(m.local_n2, old.local_n2);
        assert_eq!(m.data, old.data);
        assert_eq!(m.subshape1, 4);
    }
    #[test]
    fn test_prediction_boundary_and_degenerate_extents() {
        let a = Cuboid { half_extents: Vec2 { x: ONE, y: ONE } };
        for (x, prediction, count) in array![
            (TWO, ZERO, 2_u8), (TWO, Fixed { raw: -1 }, 0), (TWO + ONE, ONE, 2),
            (TWO + ONE, Fixed { raw: ONE.raw - 1 }, 0),
        ]
            .span() {
            let p = Pose2 { translation: Vec2 { x: *x, y: ZERO }, ..IDENTITY };
            let mut m = Default::default();
            contact_manifold_cuboid_cuboid(p, a, a, *prediction, ref m);
            assert_eq!(m.num_points, *count);
            compare(p, a, a, *prediction, Default::default());
        }
        for h in array![ZERO, Fixed { raw: 1 }, Fixed { raw: 17592186044416 }].span() {
            let c = Cuboid { half_extents: Vec2 { x: *h, y: *h } };
            compare(IDENTITY, c, c, ZERO, Default::default());
            let mut m = Default::default();
            contact_manifold_cuboid_cuboid(IDENTITY, c, c, ZERO, ref m);
            assert_eq!(m.num_points, 2);
            let normal = if *h == ZERO {
                Vec2 { x: Fixed { raw: 3037000500 }, y: Fixed { raw: 3037000500 } }
            } else {
                Vec2 { x: ONE, y: ZERO }
            };
            assert_eq!(m.local_n1, normal);
            assert_eq!(m.point(0).dist, -(*h + *h));
        }
    }
    #[test]
    #[fuzzer(runs: 64, seed: 20260921)]
    fn fuzz_candidates_equivalent(x: i16, y: i16, h: u8, rotated: bool) {
        let (mut p, a, mut b, prediction) = inputs(contact_manifolds::CUBOID_CUBOID_SHALLOW);
        p
            .translation =
                Vec2 { x: Fixed { raw: x.into() * 1048576 }, y: Fixed { raw: y.into() * 1048576 } };
        if !rotated {
            p.rotation = IDENTITY.rotation;
        }
        b.half_extents.x = Fixed { raw: h.into() * 16777216 };
        compare(p, a, b, prediction, Default::default());
        compare(p.inverse(), b, a, prediction, Default::default());
    }
    #[test]
    #[should_panic(expected: 'i64_sub Underflow')]
    fn test_unrepresentable_separation_panics() {
        let a = Cuboid { half_extents: Vec2 { x: fixed::MAX, y: ONE } };
        let mut m = Default::default();
        contact_manifold_cuboid_cuboid(IDENTITY, a, a, ZERO, ref m);
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque(inputs(contact_manifolds::CUBOID_CUBOID_TOUCHING));
    }
    #[test]
    fn gas_separated_short() {
        let (p, a, b, prediction) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_SEPARATED));
        let mut m = Default::default();
        contact_manifold_cuboid_cuboid(p, a, b, prediction, ref m);
        let _ = opaque(m);
    }
    #[test]
    fn gas_separated_eager() {
        let (p, a, b, prediction) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_SEPARATED));
        let mut m = Default::default();
        alternatives::eager_sat(p, a, b, prediction, ref m);
        let _ = opaque(m);
    }
    #[test]
    fn gas_within_pred_short() {
        let (p, a, b, prediction) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_WITHIN_PRED));
        let mut m = Default::default();
        contact_manifold_cuboid_cuboid(p, a, b, prediction, ref m);
        let _ = opaque(m);
    }
    #[test]
    fn gas_within_pred_eager() {
        let (p, a, b, prediction) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_WITHIN_PRED));
        let mut m = Default::default();
        alternatives::eager_sat(p, a, b, prediction, ref m);
        let _ = opaque(m);
    }
    #[test]
    fn gas_touching_short() {
        let (p, a, b, prediction) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_TOUCHING));
        let mut m = Default::default();
        contact_manifold_cuboid_cuboid(p, a, b, prediction, ref m);
        let _ = opaque(m);
    }
    #[test]
    fn gas_touching_eager() {
        let (p, a, b, prediction) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_TOUCHING));
        let mut m = Default::default();
        alternatives::eager_sat(p, a, b, prediction, ref m);
        let _ = opaque(m);
    }
    #[test]
    fn gas_shallow_short() {
        let (p, a, b, prediction) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_SHALLOW));
        let mut m = Default::default();
        contact_manifold_cuboid_cuboid(p, a, b, prediction, ref m);
        let _ = opaque(m);
    }
    #[test]
    fn gas_shallow_eager() {
        let (p, a, b, prediction) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_SHALLOW));
        let mut m = Default::default();
        alternatives::eager_sat(p, a, b, prediction, ref m);
        let _ = opaque(m);
    }
    #[test]
    fn gas_deep_short() {
        let (p, a, b, prediction) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_DEEP));
        let mut m = Default::default();
        contact_manifold_cuboid_cuboid(p, a, b, prediction, ref m);
        let _ = opaque(m);
    }
    #[test]
    fn gas_deep_eager() {
        let (p, a, b, prediction) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_DEEP));
        let mut m = Default::default();
        alternatives::eager_sat(p, a, b, prediction, ref m);
        let _ = opaque(m);
    }
    #[test]
    fn gas_degenerate_short() {
        let (p, a, b, prediction) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_DEGENERATE));
        let mut m = Default::default();
        contact_manifold_cuboid_cuboid(p, a, b, prediction, ref m);
        let _ = opaque(m);
    }
    #[test]
    fn gas_degenerate_eager() {
        let (p, a, b, prediction) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_DEGENERATE));
        let mut m = Default::default();
        alternatives::eager_sat(p, a, b, prediction, ref m);
        let _ = opaque(m);
    }
    #[test]
    fn gas_shapes() {
        let (p, a, b, prediction) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_TOUCHING));
        let mut m = Default::default();
        let _ = contact_manifold_cuboid_cuboid_shapes(
            p, Shape::Cuboid(a), Shape::Cuboid(b), prediction, ref m,
        );
        let _ = opaque(m);
    }
    #[test]
    fn gas_shapes_unsupported() {
        let (p, a, _, prediction) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_TOUCHING));
        let mut m = Default::default();
        let _ = contact_manifold_cuboid_cuboid_shapes(
            p, Shape::Cuboid(a), Shape::Ball(Ball { radius: ONE }), prediction, ref m,
        );
        let _ = opaque(m);
    }
    #[test]
    fn gas_fast_empty() {
        let (p, _, _, _) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_TOUCHING));
        let mut m: ContactManifold = Default::default();
        let _ = opaque(m.try_update_contacts(p));
    }
    fn cached() -> ContactManifold {
        use crate::contact::TrackedContact;
        use crate::feature_id::FeatureId;
        let p = TrackedContact {
            local_p1: Vec2 { x: ONE, y: Fixed { raw: -2147483648 } },
            local_p2: Vec2 { x: Fixed { raw: -2147483648 }, y: Fixed { raw: -2147483648 } },
            fid1: FeatureId { packed: 0x40000002 },
            fid2: FeatureId { packed: 0xc000003d },
            ..Default::default(),
        };
        ContactManifold {
            points: [
                p,
                TrackedContact {
                    local_p1: Vec2 { y: Fixed { raw: 2147483648 }, ..p.local_p1 },
                    local_p2: Vec2 { y: Fixed { raw: 2147483648 }, ..p.local_p2 },
                    fid1: FeatureId { packed: 0x40000000 },
                    ..p,
                },
            ],
            num_points: 2,
            local_n1: Vec2 { x: ONE, y: ZERO },
            local_n2: Vec2 { x: -ONE, y: ZERO },
            ..Default::default(),
        }
    }
    #[test]
    fn gas_fast_setup() {
        let _ = opaque(inputs(contact_manifolds::CUBOID_CUBOID_TOUCHING));
        let _ = opaque(cached());
    }
    #[test]
    fn gas_fast_hit() {
        let (p, _, _, _) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_TOUCHING));
        let mut m = opaque(cached());
        let _ = opaque(m.try_update_contacts(p));
    }
    #[test]
    fn gas_generator_fast_hit() {
        let (p, a, b, prediction) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_TOUCHING));
        let mut m = opaque(cached());
        contact_manifold_cuboid_cuboid(p, a, b, prediction, ref m);
        let _ = opaque(m);
    }
    #[test]
    fn gas_sat_one() {
        let (p, a, b, _) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_TOUCHING));
        let _ = opaque(cuboid_cuboid_find_local_separating_normal_oneway(a, b, p));
    }
    #[test]
    fn gas_sat_two() {
        let (p, a, b, _) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_TOUCHING));
        let q = p.inverse();
        let _ = opaque(cuboid_cuboid_find_local_separating_normal_oneway(a, b, p));
        let _ = opaque(cuboid_cuboid_find_local_separating_normal_oneway(b, a, q));
    }
    #[test]
    fn gas_clipping_and_matching() {
        let (p, a, b, _) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_TOUCHING));
        let mut m = Default::default();
        // The fixture has an identity rotation; its inverse is exact negation.
        let q = Pose2 { translation: -p.translation, ..IDENTITY };
        generate_contacts(p, q, a, b, Vec2 { x: ONE, y: ZERO }, ref m);
        let _ = opaque(m);
    }
    #[test]
    fn test_second_sat_rejects_and_negative_translation() {
        let (mut p, a, b, prediction) = inputs(contact_manifolds::CUBOID_CUBOID_DEEP);
        p.translation = Vec2 { x: Fixed { raw: 6442450944 }, y: ONE };
        let (sep1, _) = cuboid_cuboid_find_local_separating_normal_oneway(a, b, p);
        let (sep2, _) = cuboid_cuboid_find_local_separating_normal_oneway(b, a, p.inverse());
        assert!(sep1 < ZERO);
        assert!(sep2 > prediction);
        let mut m = Default::default();
        contact_manifold_cuboid_cuboid(p, a, b, prediction, ref m);
        assert_eq!(m.num_points, 0);
        compare(p, a, b, prediction, Default::default());
        p.translation = -p.translation;
        contact_manifold_cuboid_cuboid(p, a, b, prediction, ref m);
        assert_eq!(m.num_points, 0);
    }
    #[test]
    fn gas_split_fast_hit() {
        let (p, a, b, prediction) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_TOUCHING));
        let mut m = opaque(cached());
        alternatives::split_short(p, a, b, prediction, ref m);
        let _ = opaque(m);
    }
    #[test]
    fn gas_split_touching() {
        let (p, a, b, prediction) = opaque(inputs(contact_manifolds::CUBOID_CUBOID_TOUCHING));
        let mut m = Default::default();
        alternatives::split_short(p, a, b, prediction, ref m);
        let _ = opaque(m);
    }
}
