//! Family `aabb_overlap`: the set of overlapping pairs of a list of AABBs, as decided by
//! `parry::bounding_volume::BoundingVolume::intersects` (closed boundaries: touching AABBs
//! overlap), plus `BoundingVolume::merged` over the whole list.
//!
//! Every coordinate is an exact Q32.32 number, so the decision is exact in both engines: a port
//! must reproduce the pair list bit for bit, no tolerance involved.

use crate::q::{jqvec, jvec, QVec, Q};
use rapier2d_f64::parry::bounding_volume::{Aabb, BoundingVolume};
use serde_json::{json, Value};

/// One raw Q32.32 unit, 2^-32.
const ULP: i64 = 1;
const ONE: i64 = 1 << 32;

#[derive(Copy, Clone)]
struct Box2 {
    mins: QVec,
    maxs: QVec,
    is_static: bool,
}

fn raw_box(min_x: i64, min_y: i64, max_x: i64, max_y: i64, is_static: bool) -> Box2 {
    Box2 {
        mins: QVec {
            x: Q(min_x),
            y: Q(min_y),
        },
        maxs: QVec {
            x: Q(max_x),
            y: Q(max_y),
        },
        is_static,
    }
}

/// Box from decimal corners (all exactly representable in the sets below).
fn bx(min_x: f64, min_y: f64, max_x: f64, max_y: f64, is_static: bool) -> Box2 {
    Box2 {
        mins: QVec::snap(min_x, min_y),
        maxs: QVec::snap(max_x, max_y),
        is_static,
    }
}

struct Set {
    name: &'static str,
    note: &'static str,
    boxes: Vec<Box2>,
}

/// Deterministic pseudo-random generator (PCG-style LCG); the sets must not depend on anything
/// but this source file.
struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.0 >> 33
    }

    /// Uniform integer in `lo..hi`.
    fn range(&mut self, lo: u64, hi: u64) -> u64 {
        lo + self.next() % (hi - lo)
    }
}

/// `count` boxes on a 1/16 grid inside `[0, extent)²`, side between `min_side/16` and
/// `max_side/16`; every `static_every`-th box is static. Sixteenths make exact edge contacts
/// likely.
fn random_boxes(
    seed: u64,
    count: usize,
    extent: u64,
    min_side: u64,
    max_side: u64,
    static_every: usize,
) -> Vec<Box2> {
    let mut rng = Lcg(seed);
    let sixteenth = ONE / 16;
    (0..count)
        .map(|i| {
            let x = rng.range(0, extent * 16) as i64;
            let y = rng.range(0, extent * 16) as i64;
            let w = rng.range(min_side, max_side) as i64;
            let h = rng.range(min_side, max_side) as i64;
            raw_box(
                x * sixteenth,
                y * sixteenth,
                (x + w) * sixteenth,
                (y + h) * sixteenth,
                i % static_every == 0,
            )
        })
        .collect()
}

fn sets() -> Vec<Set> {
    // 4 x 3 unit boxes sharing edges and corners; the bottom row is static.
    let grid = (0..12)
        .map(|i| {
            let (c, r) = ((i % 4) as f64, (i / 4) as f64);
            bx(c, r, c + 1.0, r + 1.0, i < 4)
        })
        .collect();

    let nested = vec![
        bx(0.0, 0.0, 8.0, 8.0, true),
        bx(1.0, 1.0, 7.0, 7.0, false),
        bx(2.0, 2.0, 6.0, 6.0, false),
        bx(3.0, 3.0, 5.0, 5.0, false),
        bx(3.0, 3.0, 5.0, 5.0, false),
        bx(8.0, 0.0, 9.0, 8.0, true),
        bx(8.0, 8.0, 9.0, 9.0, false),
        bx(10.0, 10.0, 11.0, 11.0, false),
    ];

    // Boxes one raw unit (2^-32) apart, exactly touching, and overlapping by one raw unit.
    let u = ULP;
    let ulp = vec![
        raw_box(0, 0, ONE, ONE, true),
        raw_box(ONE + u, 0, 2 * ONE, ONE, false),
        raw_box(ONE, 0, 2 * ONE, ONE, false),
        raw_box(ONE - u, 0, 2 * ONE, ONE, false),
        raw_box(0, ONE + u, ONE, 2 * ONE, true),
        raw_box(0, ONE, ONE, 2 * ONE, false),
        raw_box(ONE + u, ONE + u, 2 * ONE, 2 * ONE, false),
        raw_box(ONE, ONE, 2 * ONE, 2 * ONE, false),
    ];

    let mut mixed = random_boxes(0x5eed_0001, 20, 8, 4, 32, 3);
    // Force the boundary cases the random sets only hit by luck.
    mixed[1] = Box2 {
        mins: QVec {
            x: mixed[0].maxs.x,
            y: mixed[0].mins.y,
        },
        maxs: QVec {
            x: Q(mixed[0].maxs.x.0 + ONE),
            y: mixed[0].maxs.y,
        },
        is_static: false,
    };
    mixed[2] = Box2 {
        mins: QVec {
            x: Q(mixed[0].mins.x.0 + ONE / 16),
            y: Q(mixed[0].mins.y.0 + ONE / 16),
        },
        maxs: QVec {
            x: Q(mixed[0].maxs.x.0 - ONE / 16),
            y: Q(mixed[0].maxs.y.0 - ONE / 16),
        },
        is_static: false,
    };

    let random = random_boxes(0x5eed_0002, 32, 12, 4, 40, 4);

    vec![
        Set {
            name: "grid_touching",
            note: "4x3 unit boxes sharing edges and corners; bottom row static",
            boxes: grid,
        },
        Set {
            name: "nested",
            note: "nested, identical and edge/corner-touching boxes around a static 8x8 box",
            boxes: nested,
        },
        Set {
            name: "ulp_boundary",
            note: "gaps and overlaps of exactly one raw unit (2^-32) along x, y and diagonally",
            boxes: ulp,
        },
        Set {
            name: "mixed_20",
            note: "20 boxes on a 1/16 grid, every third static; boxes 1 and 2 forced to touch / nest in box 0",
            boxes: mixed,
        },
        Set {
            name: "random_32",
            note: "32 boxes on a 1/16 grid, every fourth static",
            boxes: random,
        },
    ]
}

fn aabb(b: &Box2) -> Aabb {
    Aabb::new(b.mins.v(), b.maxs.v())
}

fn set_json(set: &Set) -> Value {
    let aabbs: Vec<Aabb> = set.boxes.iter().map(aabb).collect();
    let mut pairs = Vec::new();
    for i in 0..aabbs.len() {
        for j in (i + 1)..aabbs.len() {
            let ij = aabbs[i].intersects(&aabbs[j]);
            assert_eq!(ij, aabbs[j].intersects(&aabbs[i]), "asymmetric intersects");
            if ij {
                pairs.push(json!({
                    "i": i,
                    "j": j,
                    "both_static": set.boxes[i].is_static && set.boxes[j].is_static,
                }));
            }
        }
    }
    let merged = aabbs[1..].iter().fold(aabbs[0], |acc, b| acc.merged(b));

    json!({
        "id": format!("set/{}", set.name),
        "note": set.note,
        "aabbs": set.boxes.iter().map(|b| json!({
            "mins": jqvec(b.mins),
            "maxs": jqvec(b.maxs),
            "static": b.is_static,
        })).collect::<Vec<_>>(),
        "expected": {
            "num_pairs": pairs.len(),
            "pairs": pairs,
            "merged": { "mins": jvec(merged.mins), "maxs": jvec(merged.maxs) },
        },
    })
}

pub fn generate() -> Value {
    json!({
        "family": "aabb_overlap",
        "convention": "closed: a.mins <= b.maxs && a.maxs >= b.mins on every axis, so exactly touching AABBs overlap",
        "cases": sets().iter().map(set_json).collect::<Vec<_>>(),
    })
}
