//! Area and centre of mass of a convex polygon given by its vertices (Parry
//! `mass_properties/mass_properties_convex_polygon.rs`; PX3).

use fixed::{Fixed, ZERO};
use glam_core::{Vec2, Vec2Trait};
use crate::point::cross_wide;

/// Errors of the polygon mass helpers.
pub mod errors {
    /// An empty vertex list (upstream panics on `unwrap`).
    pub const EMPTY: felt252 = 'Mass: empty polygon';
    /// A triangle area that does not fit the scalar range.
    pub const AREA_OVERFLOW: felt252 = 'Mass: area overflow';
}

/// `x / d` on the raw, truncated toward zero.
#[inline(always)]
fn div_raw(x: Fixed, d: i64) -> Fixed {
    Fixed { raw: x.raw / d }
}

/// The area and the centre of mass of the convex polygon `convex_polygon` (upstream
/// `convex_polygon_area_and_center_of_mass`): the fan of triangles from the geometric centre
/// (the mean of the vertices), each triangle weighted by its area; for a zero area the geometric
/// centre. Vertices in either orientation; the fan follows the given order and closes on the
/// first vertex.
///
/// The arithmetic is that of `MassPropertiesTrait::from_convex_polygon` for a `ConvexPolygon`
/// (which this function does not change): raw divisions truncate toward zero, each triangle area
/// is one exact wide cross product halved (toward zero), and each area-weighted product floors.
/// #### Panics
/// * `'Mass: empty polygon'` for no vertex.
/// * `'Mass: area overflow'` if a triangle area leaves the scalar range.
/// * `'Fixed: overflow'` / `'i64_add Overflow'` if a sum or a product leaves the scalar range.
/// #### Deviations
/// * Fixed-point rounding as above, within a few ulps of upstream's `f64`.
pub fn convex_polygon_area_and_center_of_mass(convex_polygon: Span<Vec2>) -> (Fixed, Vec2) {
    let n = convex_polygon.len();
    assert(n != 0, errors::EMPTY);
    let mut sum = Vec2Trait::ZERO;
    for p in convex_polygon {
        sum = sum + *p;
    }
    let count: i64 = n.into();
    let geometric_center = Vec2 { x: div_raw(sum.x, count), y: div_raw(sum.y, count) };
    let mut weighted = Vec2Trait::ZERO;
    let mut area_sum = ZERO;
    let mut i = 0;
    while i != n {
        let a = *convex_polygon.at(i);
        let b = *convex_polygon.at(if i + 1 == n {
            0
        } else {
            i + 1
        });
        let e = b - a;
        let d = geometric_center - a;
        let twice: i128 = cross_wide(e.x, e.y, d.x, d.y);
        let twice = if twice < 0 {
            -twice
        } else {
            twice
        };
        let part = Fixed { raw: (twice / 0x200000000).try_into().expect(errors::AREA_OVERFLOW) };
        let s = a + b + geometric_center;
        let center = Vec2 { x: div_raw(s.x, 3), y: div_raw(s.y, 3) };
        weighted = weighted + center.mul_scalar(part);
        area_sum = area_sum + part;
        i += 1;
    }
    if area_sum == ZERO {
        (area_sum, geometric_center)
    } else {
        (area_sum, Vec2 { x: weighted.x / area_sum, y: weighted.y / area_sum })
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, ONE, ZERO};
    use glam_core::Vec2;
    use rapier_testing::opaque;
    use crate::mass::MassPropertiesTrait;
    use crate::shape::{ConvexPolygon, ConvexPolygonTrait};
    use super::convex_polygon_area_and_center_of_mass;

    fn f(n: i64) -> Fixed {
        Fixed { raw: n * 0x1_0000_0000 }
    }

    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: f(x), y: f(y) }
    }

    #[test]
    fn test_area_and_center_table() {
        // The centre truncates toward zero on the raw, as `from_convex_polygon`: one ulp below.
        let half = Fixed { raw: 0x7fff_ffff };
        let square = array![v(0, 0), v(1, 0), v(1, 1), v(0, 1)];
        assert_eq!(
            convex_polygon_area_and_center_of_mass(square.span()), (ONE, Vec2 { x: half, y: half }),
        );
        // The orientation does not change the area.
        let clockwise = array![v(0, 0), v(0, 2), v(2, 2), v(2, 0)];
        assert_eq!(convex_polygon_area_and_center_of_mass(clockwise.span()), (f(4), v(1, 1)));
        // Collinear vertices: zero area, geometric centre.
        let line = array![v(0, 0), v(1, 0), v(2, 0)];
        assert_eq!(convex_polygon_area_and_center_of_mass(line.span()), (ZERO, v(1, 0)));
        // A single point and two points are degenerate polygons too.
        assert_eq!(convex_polygon_area_and_center_of_mass(array![v(3, 4)].span()), (ZERO, v(3, 4)));
        assert_eq!(
            convex_polygon_area_and_center_of_mass(array![v(0, 0), v(2, 2)].span()),
            (ZERO, v(1, 1)),
        );
    }

    #[test]
    fn test_agrees_with_the_polygon_mass_properties() {
        // The port's `ConvexPolygon` mass properties use the same fan: same centre, same mass.
        let pts = array![v(-1, -1), v(1, -1), v(1, 1), v(-1, 1)];
        let polygon: ConvexPolygon = ConvexPolygonTrait::from_convex_polyline(pts.span()).unwrap();
        let (area, com) = convex_polygon_area_and_center_of_mass(pts.span());
        let props = polygon.mass_properties(ONE);
        assert_eq!(props.local_com, com);
        assert_eq!(props.mass(), area);
    }

    #[test]
    #[should_panic(expected: ('Mass: empty polygon',))]
    fn test_empty_polygon_panics() {
        let _ = convex_polygon_area_and_center_of_mass(array![].span());
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_convex_polygon_area_and_center_of_mass() {
        let pts = array![v(-1, -1), v(1, -1), v(2, 1), v(0, 2), v(-1, 1)];
        let _ = convex_polygon_area_and_center_of_mass(opaque(pts).span());
    }

    #[test]
    fn gas_convex_polygon_area_and_center_of_mass_triangle() {
        let pts = array![v(0, 0), v(3, 0), v(0, 3)];
        let _ = convex_polygon_area_and_center_of_mass(opaque(pts).span());
    }
}
