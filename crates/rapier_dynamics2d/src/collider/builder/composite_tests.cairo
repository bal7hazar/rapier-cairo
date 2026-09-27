use fixed::{FixedTrait, HALF, ONE, TWO, ZERO};
use glam::Vec2;
use rapier_geometry2d::mass::MassProperties;
use rapier_geometry2d::shape::{
    HeightFieldTrait, PolylineFlagsTrait, PolylineTrait, Shape, ShapeTrait,
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
