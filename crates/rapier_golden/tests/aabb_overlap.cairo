//! Sanity checks of the `aabb_overlap` fixtures: the pair lists are recomputed here with a
//! closed-interval integer test (the inputs are exact Q32.32 numbers, so the comparison is exact)
//! and must match the lists recorded from upstream, including their order.

use rapier_golden::aabb_overlap;
use rapier_golden::types::{AabbOverlapCase, OverlapBoxRaw};

/// One raw unit, 2^-32.
const ULP: i64 = 1;
const ONE: i64 = 0x100000000;

/// Closed convention: touching boxes overlap.
fn overlaps(a: @OverlapBoxRaw, b: @OverlapBoxRaw) -> bool {
    *a.mins.x <= *b.maxs.x
        && *b.mins.x <= *a.maxs.x
        && *a.mins.y <= *b.maxs.y
        && *b.mins.y <= *a.maxs.y
}

fn has_pair(case: @AabbOverlapCase, i: u32, j: u32) -> bool {
    let pairs = case.pairs.span();
    let mut k = 0;
    let mut found = false;
    while k != *case.num_pairs {
        let pair = pairs.at(k);
        if *pair.i == i && *pair.j == j {
            found = true;
        }
        k += 1;
    }
    found
}

#[test]
fn test_tables_have_the_expected_sizes() {
    let cases = aabb_overlap::cases();
    assert_eq!(cases.len(), 5);
    let boxes = array![12_u32, 8, 8, 20, 32];
    let pairs = array![29_u32, 13, 16, 8, 18];
    let mut i = 0;
    while i != 5 {
        assert_eq!(*cases.at(i).num_aabbs, *boxes.at(i), "boxes of {}", *cases.at(i).id);
        assert_eq!(*cases.at(i).num_pairs, *pairs.at(i), "pairs of {}", *cases.at(i).id);
        i += 1;
    }
}

#[test]
fn test_pairs_are_the_closed_interval_overlaps_in_lexicographic_order() {
    for case in aabb_overlap::cases() {
        let boxes = case.aabbs.span();
        let pairs = case.pairs.span();
        let n = *case.num_aabbs;
        let mut k = 0;
        let mut i = 0;
        while i != n {
            let mut j = i + 1;
            while j != n {
                if overlaps(boxes.at(i), boxes.at(j)) {
                    assert!(k < *case.num_pairs, "missing pair ({}, {}) in {}", i, j, *case.id);
                    let pair = pairs.at(k);
                    assert_eq!((*pair.i, *pair.j), (i, j), "pair {} of {}", k, *case.id);
                    k += 1;
                }
                j += 1;
            }
            i += 1;
        }
        assert_eq!(k, *case.num_pairs, "pair count of {}", *case.id);
    }
}

#[test]
fn test_overlap_is_symmetric() {
    for case in aabb_overlap::cases() {
        let boxes = case.aabbs.span();
        let n = *case.num_aabbs;
        let mut i = 0;
        while i != n {
            let mut j = 0;
            while j != n {
                assert_eq!(
                    overlaps(boxes.at(i), boxes.at(j)),
                    overlaps(boxes.at(j), boxes.at(i)),
                    "{} vs {} in {}",
                    i,
                    j,
                    *case.id,
                );
                j += 1;
            }
            i += 1;
        }
    }
}

#[test]
fn test_every_box_overlaps_itself() {
    for case in aabb_overlap::cases() {
        let boxes = case.aabbs.span();
        let mut i = 0;
        while i != *case.num_aabbs {
            assert!(overlaps(boxes.at(i), boxes.at(i)), "box {} of {}", i, *case.id);
            i += 1;
        }
    }
}

#[test]
fn test_static_flag_marks_pairs_the_broad_phase_drops() {
    for case in aabb_overlap::cases() {
        let boxes = case.aabbs.span();
        let pairs = case.pairs.span();
        let mut k = 0;
        while k != *case.num_pairs {
            let pair = pairs.at(k);
            let both = *boxes.at(*pair.i).is_static && *boxes.at(*pair.j).is_static;
            assert_eq!(*pair.both_static, both, "pair {} of {}", k, *case.id);
            assert!(*pair.i < *pair.j, "pair {} of {} is ordered", k, *case.id);
            k += 1;
        }
    }
}

#[test]
fn test_boundaries_are_decided_to_the_last_raw_unit() {
    let case = aabb_overlap::SET_ULP_BOUNDARY;
    let boxes = case.aabbs.span();
    // Box 0 is [0, 1]², box 2 starts exactly at its right edge, box 1 one raw unit beyond it and
    // box 3 one raw unit inside it.
    assert_eq!(*boxes.at(0).maxs.x, ONE);
    assert_eq!(*boxes.at(1).mins.x, ONE + ULP);
    assert_eq!(*boxes.at(2).mins.x, ONE);
    assert_eq!(*boxes.at(3).mins.x, ONE - ULP);
    assert!(!has_pair(@case, 0, 1), "gap of one ulp along x");
    assert!(has_pair(@case, 0, 2), "exact touch along x");
    assert!(has_pair(@case, 0, 3), "overlap of one ulp along x");
    assert!(!has_pair(@case, 0, 4), "gap of one ulp along y");
    assert!(has_pair(@case, 0, 5), "exact touch along y");
    assert!(!has_pair(@case, 0, 6), "gap of one ulp on both axes");
    assert!(has_pair(@case, 0, 7), "corner to corner");
}

#[test]
fn test_grid_of_unit_boxes_touches_along_edges_and_corners() {
    let case = aabb_overlap::SET_GRID_TOUCHING;
    // 4 x 3 boxes: 9 + 8 edge contacts and 2 * 6 diagonal (corner) contacts.
    assert_eq!(case.num_pairs, 29);
    assert!(has_pair(@case, 0, 1), "left-right neighbours");
    assert!(has_pair(@case, 0, 4), "bottom-top neighbours");
    assert!(has_pair(@case, 0, 5), "diagonal neighbours touch at a corner");
    assert!(!has_pair(@case, 0, 2), "two boxes apart");
    assert!(!has_pair(@case, 0, 10), "far apart");
}

#[test]
fn test_merged_box_bounds_every_box() {
    for case in aabb_overlap::cases() {
        let boxes = case.aabbs.span();
        let mut min_x = *boxes.at(0).mins.x;
        let mut min_y = *boxes.at(0).mins.y;
        let mut max_x = *boxes.at(0).maxs.x;
        let mut max_y = *boxes.at(0).maxs.y;
        let mut i = 1;
        while i != *case.num_aabbs {
            let b = boxes.at(i);
            if *b.mins.x < min_x {
                min_x = *b.mins.x;
            }
            if *b.mins.y < min_y {
                min_y = *b.mins.y;
            }
            if *b.maxs.x > max_x {
                max_x = *b.maxs.x;
            }
            if *b.maxs.y > max_y {
                max_y = *b.maxs.y;
            }
            i += 1;
        }
        assert_eq!((*case.merged_mins.x, *case.merged_mins.y), (min_x, min_y), "{}", *case.id);
        assert_eq!((*case.merged_maxs.x, *case.merged_maxs.y), (max_x, max_y), "{}", *case.id);
    }
}
