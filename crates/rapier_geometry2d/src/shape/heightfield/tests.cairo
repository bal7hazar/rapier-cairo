use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
use glam_core::Vec2;
use rapier_testing::opaque;
use crate::aabb::Aabb;
use crate::shape::segment::SegmentTrait;
use super::{HeightField, HeightFieldTrait};

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn int(x: i32) -> Fixed {
    FixedTrait::from_int(x)
}

/// Heights `0, 1, 0.5, 0` over `x in [-3, 3]`, `y` doubled.
fn hf() -> HeightField {
    HeightFieldTrait::new(array![ZERO, ONE, HALF, ZERO].span(), v(int(6), TWO))
}

#[test]
fn test_cells_and_accessors() {
    let h = hf();
    assert_eq!(h.num_cells(), 3);
    assert_eq!(h.scale(), v(int(6), TWO));
    assert_eq!(h.local_aabb(), Aabb { mins: v(int(-3), ZERO), maxs: v(int(3), TWO) });
    assert_eq!(h.cell_width(), TWO);
    assert_eq!(h.unit_cell_width(), FixedTrait::from_raw(1431655765));
    assert_eq!(h.start_x(), int(-3));
    assert_eq!(h.x_at(3), int(3));
    assert_eq!(h.segment_at(0), Some(SegmentTrait::new(v(int(-3), ZERO), v(int(-1), TWO))));
    assert_eq!(h.segment_at(1), Some(SegmentTrait::new(v(int(-1), TWO), v(ONE, ONE))));
    assert_eq!(h.segment_at(3), None);
    assert_eq!(h.segments().len(), 3);
    let zero: crate::mass::MassProperties = Default::default();
    assert_eq!(h.mass_properties(ONE), zero);
}

#[test]
fn test_cell_at_point_and_height() {
    let h = hf();
    // (x, expected cell)
    let table = array![
        (int(-3), Some(0)), (int(-2), Some(0)), (int(-1), Some(1)), (ZERO, Some(1)),
        (int(3), Some(2)), (int(4), None), (int(-4), None),
    ];
    for (x, cell) in table {
        assert_eq!(h.cell_at_point(v(x, int(7))), cell);
    }
    // The height of the cell's line at `pt.x` (see the module documentation).
    assert_eq!(h.height_at_point(v(int(-2), int(5))), Some(ONE));
    assert_eq!(h.height_at_point(v(ZERO, ZERO)), Some(ONE + HALF));
    assert_eq!(h.height_at_point(v(int(5), ZERO)), None);
}

#[test]
fn test_removed_cells_and_scale() {
    let mut h = hf();
    h.set_segment_removed(1, true);
    assert!(h.is_segment_removed(1));
    assert_eq!(h.segment_at(1), None);
    assert_eq!(h.segments().len(), 2);
    assert_eq!(h.cells_statuses(), array![true, false, true].span());
    h.set_segment_removed(1, false);
    assert_eq!(h.segments().len(), 3);
    let s = hf().scaled(v(HALF, TWO));
    assert_eq!(s.scale(), v(int(3), int(4)));
    assert_eq!(s.local_aabb(), Aabb { mins: v(-(ONE + HALF), ZERO), maxs: v(ONE + HALF, int(4)) });
}

#[test]
fn test_elements_in_local_aabb() {
    let h = hf();
    // (box, cells)
    let table = array![
        (Aabb { mins: v(int(-10), int(-10)), maxs: v(int(10), int(10)) }, array![0, 1, 2]),
        (Aabb { mins: v(int(-2), ZERO), maxs: v(int(-2), ZERO) }, array![0]),
        // Above every height of cells 0 and 2 but cell 1 reaches y = 2.
        (Aabb { mins: v(int(-3), int(3)), maxs: v(int(3), int(4)) }, array![]),
        (Aabb { mins: v(ZERO, ONE + HALF), maxs: v(HALF, int(3)) }, array![1]),
        (Aabb { mins: v(int(5), ZERO), maxs: v(int(6), ONE) }, array![]),
        (Aabb { mins: v(int(-1), ZERO), maxs: v(ONE, ONE) }, array![1]),
    ];
    for (b, cells) in table {
        let mut got = array![];
        for (i, _) in h.elements_in_local_aabb(b) {
            got.append(i);
        }
        assert_eq!(got, cells);
    }
    assert_eq!(
        h
            .unclamped_elements_range_in_local_aabb(
                Aabb { mins: v(int(-7), ZERO), maxs: v(ZERO, ZERO) },
            ),
        (-2, 2),
    );
}

#[test]
#[should_panic(expected: 'HeightField: < 2 heights')]
fn test_too_few_heights() {
    let _ = HeightFieldTrait::new(array![ONE].span(), v(ONE, ONE));
}

#[test]
#[should_panic(expected: 'HeightField: scale.x <= 0')]
fn test_negative_scale() {
    let _ = HeightFieldTrait::new(array![ONE, ONE].span(), v(-ONE, ONE));
}

#[test]
fn gas_baseline() {}

fn ground(n: u32) -> HeightField {
    let mut heights = array![];
    let mut i: u32 = 0;
    while i != n + 1 {
        heights.append(if i % 2 == 0 {
            ZERO
        } else {
            HALF
        });
        i += 1;
    }
    let w: i32 = n.try_into().unwrap();
    HeightFieldTrait::new(heights.span(), v(int(w), ONE))
}

#[test]
fn gas_elements_in_local_aabb_50() {
    let h = ground(50);
    let _ = opaque(@h)
        .elements_in_local_aabb(opaque(Aabb { mins: v(HALF, ZERO), maxs: v(ONE, HALF) }));
}

#[test]
fn gas_segment_at() {
    let h = ground(10);
    let _ = opaque(@h).segment_at(opaque(4));
}
