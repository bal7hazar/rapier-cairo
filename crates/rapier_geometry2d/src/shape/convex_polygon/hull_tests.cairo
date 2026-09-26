//! `from_convex_hull` and `offsetted`.

use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_testing::opaque;
use super::{ConvexPolygon, ConvexPolygonTrait};

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn i(n: i32) -> Fixed {
    FixedTrait::from_int(n)
}

fn square() -> ConvexPolygon {
    ConvexPolygonTrait::from_convex_polyline(
        [v(-ONE, -ONE), v(ONE, -ONE), v(ONE, ONE), v(-ONE, ONE)].span(),
    )
        .unwrap()
}

/// `(0, 0), (1, 1), (2, 4), ...` up to `n` points: a strictly convex chain, so all are on the hull.
fn parabola(n: i32) -> Array<Vec2> {
    let mut points = array![];
    let mut k = 0;
    while k != n {
        points.append(v(i(k), i(k * k)));
        k += 1;
    }
    points
}

#[test]
fn test_hull_drops_interior_edge_and_duplicate_points() {
    // Scrambled square corners, edge midpoints, the centre and a duplicated corner.
    let points = array![
        v(ONE, ONE), v(ZERO, ZERO), v(-ONE, ONE), v(ZERO, -ONE), v(ONE, -ONE), v(-ONE, ZERO),
        v(ONE, ONE), v(-ONE, -ONE), v(ONE, ZERO), v(ZERO, ONE), v(HALF, -HALF),
    ];
    assert_eq!(ConvexPolygonTrait::from_convex_hull(points.span()), Some(square()));
    // Start at the lowest x, then the lowest y: the same square whatever the input order.
    let reversed = array![v(-ONE, ONE), v(ONE, ONE), v(ONE, -ONE), v(-ONE, -ONE)];
    assert_eq!(ConvexPolygonTrait::from_convex_hull(reversed.span()), Some(square()));
    let triangle = array![v(ONE, ZERO), v(-ONE, -ONE), v(ZERO, ZERO), v(-ONE, ONE), v(-HALF, ZERO)];
    let hull = ConvexPolygonTrait::from_convex_hull(triangle.span()).unwrap();
    assert_eq!(hull.count(), 3);
    assert_eq!(
        (hull.vertex(0), hull.vertex(1), hull.vertex(2)),
        (v(-ONE, -ONE), v(ONE, ZERO), v(-ONE, ONE)),
    );
}

#[test]
fn test_hull_is_none_when_degenerate_or_too_large() {
    let line = array![v(ZERO, ZERO), v(ONE, ONE), v(TWO, TWO), v(HALF, HALF)];
    let vertical = array![v(ONE, -ONE), v(ONE, ONE), v(ONE, ZERO)];
    let same = array![v(ONE, ONE), v(ONE, ONE), v(ONE, ONE)];
    let two = array![v(ZERO, ZERO), v(ONE, ZERO)];
    let cases = array![line, vertical, same, two, array![], parabola(9)];
    for points in cases {
        assert_eq!(ConvexPolygonTrait::from_convex_hull(points.span()), None);
    }
    // Eight vertices is the capacity.
    let polygon = ConvexPolygonTrait::from_convex_hull(parabola(8).span()).unwrap();
    assert_eq!(polygon.count(), 8);
    assert_eq!((polygon.vertex(0), polygon.vertex(7)), (v(ZERO, ZERO), v(i(7), i(49))));
}

#[test]
fn test_offsetted_moves_every_edge_by_the_amount() {
    let grown = square().offsetted(HALF);
    let expected = v(i(3) / TWO, i(3) / TWO);
    assert_eq!(grown.count(), 4);
    assert_eq!((grown.vertex(0), grown.vertex(2)), (-expected, expected));
    assert_eq!(grown.normals(), square().normals());
    assert_eq!(square().offsetted(ZERO), square());
    // A skewed triangle: each edge, measured along its unit normal, moves by `amount` (a few ulps).
    let triangle = ConvexPolygonTrait::from_convex_polyline(
        [v(-ONE, -ONE), v(TWO, -ONE), v(-ONE, ONE)].span(),
    )
        .unwrap();
    let grown = triangle.offsetted(HALF);
    let mut k = 0;
    while k != 3 {
        let next = triangle.next(k);
        let moved = (grown.vertex(next) - triangle.vertex(next)).dot(triangle.normal(k));
        let error = if moved > HALF {
            moved - HALF
        } else {
            HALF - moved
        };
        assert!(error.raw < 64);
        k += 1;
    }
}

#[test]
#[should_panic(expected: ('Polygon: negative offset',))]
fn test_offsetted_negative_panics() {
    let _ = square().offsetted(-HALF);
}

#[test]
fn gas_baseline() {}

#[test]
fn gas_from_convex_hull_square() {
    let points = array![
        v(ONE, ONE), v(ZERO, ZERO), v(-ONE, ONE), v(ZERO, -ONE), v(ONE, -ONE), v(-ONE, -ONE),
    ];
    let _ = ConvexPolygonTrait::from_convex_hull(opaque(points).span());
}

#[test]
fn gas_offsetted() {
    let _ = opaque(square()).offsetted(opaque(HALF));
}
