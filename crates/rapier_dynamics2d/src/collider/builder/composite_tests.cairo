use fixed::{FixedTrait, HALF, ONE, TWO, ZERO};
use glam_core::Vec2;
use rapier_geometry2d::mass::{MassProperties, MassPropertiesTrait};
use rapier_geometry2d::shape::{
    CompoundTrait, HeightFieldTrait, PolylineFlagsTrait, PolylineTrait, Shape, ShapeTrait,
};
use rapier_testing::opaque;
use super::ColliderBuilderTrait;

fn v(x: i32, y: i32) -> Vec2 {
    Vec2 { x: FixedTrait::from_int(x), y: FixedTrait::from_int(y) }
}

#[test]
fn test_composite_constructors() {
    let vertices = array![v(-2, 0), v(0, 1), v(2, 0)].span();
    let polyline = ColliderBuilderTrait::polyline(vertices, None).build();
    assert_eq!(polyline.shape.as_polyline().unwrap(), PolylineTrait::new(vertices, None));
    assert_eq!(polyline.shape.as_polyline().unwrap().num_segments(), 2);
    // No mass, as upstream.
    let zero: MassProperties = Default::default();
    assert_eq!(polyline.shape.mass_properties(ONE), zero);
    let indices = array![[0, 1], [1, 2]].span();
    let flagged = ColliderBuilderTrait::polyline_with_flags(
        vertices, Some(indices), PolylineFlagsTrait::empty(),
    )
        .build();
    assert_eq!(flagged.shape, polyline.shape);
    let oriented = ColliderBuilderTrait::oriented_polyline(vertices, Some(indices)).build();
    assert!(oriented.shape.as_polyline().unwrap().is_oriented());
    let heights = array![ZERO, ONE, HALF, ZERO].span();
    let ground = ColliderBuilderTrait::heightfield(heights, v(6, 2)).build();
    let h = ground.shape.as_heightfield().unwrap();
    assert_eq!(h, HeightFieldTrait::new(heights, v(6, 2)));
    assert_eq!(h.num_cells(), 3);
    assert_eq!(ground.shape.mass_properties(TWO), zero);
    assert!(!ground.shape.is_convex());
    assert!(match ground.shape {
        Shape::HeightField(_) => true,
        _ => false,
    });
}

#[test]
fn gas_polyline_10() {
    let mut vertices = array![];
    let mut i: i32 = 0;
    while i != 11 {
        vertices.append(v(i, i % 2));
        i += 1;
    }
    let _ = ColliderBuilderTrait::polyline(opaque(vertices.span()), None);
}

#[test]
fn gas_heightfield_10() {
    let heights = array![ZERO, ONE, ZERO, ONE, ZERO, ONE, ZERO, ONE, ZERO, ONE, ZERO].span();
    let _ = ColliderBuilderTrait::heightfield(opaque(heights), opaque(v(10, 1)));
}

#[test]
fn gas_baseline() {}

#[test]
fn test_compound_constructor() {
    let bar: Shape = rapier_geometry2d::shape::CuboidTrait::new(v(1, 1)).into();
    let pose = rapier_math::pose2::Pose2 { translation: v(1, 2), ..Default::default() };
    let parts = array![(Default::default(), bar), (pose, bar)].span();
    let c = ColliderBuilderTrait::compound(parts).build();
    let compound = c.shape.as_compound().unwrap();
    assert_eq!(compound.shapes(), parts);
    // The sum of the parts' mass properties, as upstream's `from_compound`.
    let expected = bar.mass_properties(TWO) + bar.mass_properties(TWO).transform_by(pose);
    assert_eq!(c.shape.mass_properties(TWO), expected);
    assert!(c.shape.is_composite() && !c.shape.is_convex());
}

#[test]
#[should_panic(expected: 'Compound: no part')]
fn test_compound_constructor_empty() {
    let _ = ColliderBuilderTrait::compound(array![].span());
}

#[test]
fn gas_compound_2() {
    let bar: Shape = rapier_geometry2d::shape::CuboidTrait::new(v(1, 1)).into();
    let pose = rapier_math::pose2::Pose2 { translation: v(1, 2), ..Default::default() };
    let _ = ColliderBuilderTrait::compound(
        opaque(array![(Default::default(), bar), (pose, bar)].span()),
    );
}
