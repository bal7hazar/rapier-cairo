//! RG2 (#189's second cause): candidates for `ShapeSerde`'s tag dispatch. SH1 (tags `6..=9`), SH2a
//! (`10..=11`) and SH2b (`12`) turned the derived six-arm `Serde<Shape>` into a thirteen-arm one,
//! and PLAN.md logged a `WorldState` round trip as +676 Cairo steps dearer than at `0.1.0-alpha.4`.
//!
//! Measured here, by swapping each candidate into the shipped `ShapeSerde` and re-running
//! `crates/rapier2d/tests/game_path.cairo`'s `steps_game_serde_trips` (`--tracked-resource
//! cairo-steps`, exact steps, not Sierra gas): `mixed` (the pre-RG2 shape: the six old arms inlined
//! next to three call arms for the SH2a / SH2b payloads only), `two_level` (six old arms plus one
//! wildcard, out of line, mirroring `deserialize`'s existing split) and the shipped `flat` (every
//! SH1 / SH2a / SH2b variant its own `#[inline(never)]` helper, none of them sharing a match) all
//! land on **the same 2,953,098 steps** — identical to the pre-RG2 shipped code. Only removing
//! every out-of-line call (`single_match`, every arm inlined) regresses (2,953,143, +45). None
//! recovers `0.1.0-alpha.4`'s cost for the six old shapes alone (2,589 steps on a dedicated
//! six-shape round trip vs 2,679..2,685 at `HEAD`, checked both ways: reordering `Shape`'s variant
//! declaration to restore the old shapes' pre-SH2a/SH2b discriminants made no difference to that
//! number either, and regressed the full game probe by +131 steps because `compute_aabb` /
//! `mass_properties` also dispatch on that order). The remaining gap looks structural (more
//! variants in the `Shape` enum itself), not a `ShapeSerde` dispatch-shape problem; see
//! `REPORT.md`'s Escalations.
//!
//! Sierra gas cannot see any of this: `Serde<Shape>::serialize` / `::deserialize` are loop-free, so
//! Sierra charges every call the worst branch (the heaviest new shape) regardless of which tag is
//! actually taken (GF2). The `gas_*` probes below only bound worst-case growth; the decision was
//! made on Cairo steps, reported in `REPORT.md`.

use fixed::{Fixed, HALF, ONE, ZERO};
use glam::Vec2;
use rapier_math::pose2::Pose2Trait;
use rapier_testing::opaque;
use crate::shape::compound::{BoxedCompoundSerde, CompoundTrait};
use crate::shape::convex_polygon::BoxedConvexPolygonSerde;
use crate::shape::heightfield::{BoxedHeightFieldSerde, HeightFieldTrait};
use crate::shape::polyline::{BoxedPolylineSerde, PolylineTrait};
use crate::shape::round_shape::{
    BoxedRoundConvexPolygonShapeSerde, BoxedRoundTriangleSerde, RoundConvexPolygonShapeTrait,
    RoundShapeTrait,
};
use crate::shape::triangle::{BoxedTriangleSerde, TriangleTrait};
use super::{
    BallTrait, CapsuleTrait, ConvexPolygonTrait, CuboidTrait, HalfSpaceTrait, SegmentTrait, Shape,
    ShapeSerde,
};

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn ball() -> Shape {
    Shape::Ball(BallTrait::new(HALF))
}
fn cuboid() -> Shape {
    Shape::Cuboid(CuboidTrait::new(v(ONE, ONE)))
}
fn capsule() -> Shape {
    Shape::Capsule(CapsuleTrait::new(v(ZERO, -ONE), v(ZERO, ONE), HALF))
}
fn segment() -> Shape {
    Shape::Segment(SegmentTrait::new(v(-ONE, ZERO), v(ONE, ONE)))
}
fn halfspace() -> Shape {
    Shape::HalfSpace(HalfSpaceTrait::new(v(ZERO, ONE)))
}
fn triangle_points() -> Span<Vec2> {
    array![v(-ONE, -ONE), v(ONE, -ONE), v(ZERO, ONE)].span()
}
fn polygon() -> Shape {
    Shape::ConvexPolygon(
        BoxTrait::new(ConvexPolygonTrait::from_convex_polyline(triangle_points()).unwrap()),
    )
}
fn triangle() -> Shape {
    Shape::Triangle(BoxTrait::new(TriangleTrait::new(v(-ONE, -ONE), v(ONE, -ONE), v(ZERO, ONE))))
}
fn round_cuboid() -> Shape {
    Shape::RoundCuboid(RoundShapeTrait::new(CuboidTrait::new(v(ONE, ONE)), HALF))
}
fn round_triangle() -> Shape {
    Shape::RoundTriangle(
        BoxTrait::new(
            RoundShapeTrait::new(
                TriangleTrait::new(v(-ONE, -ONE), v(ONE, -ONE), v(ZERO, ONE)), HALF,
            ),
        ),
    )
}
fn round_convex_polygon() -> Shape {
    let p = ConvexPolygonTrait::from_convex_polyline(triangle_points()).unwrap();
    Shape::RoundConvexPolygon(
        BoxTrait::new(RoundConvexPolygonShapeTrait::new(RoundShapeTrait::new(p, HALF))),
    )
}
fn polyline() -> Shape {
    Shape::Polyline(
        BoxTrait::new(
            PolylineTrait::new(array![v(ZERO, ZERO), v(ONE, ZERO), v(ONE, ONE)].span(), None),
        ),
    )
}
fn heightfield() -> Shape {
    Shape::HeightField(
        BoxTrait::new(HeightFieldTrait::new(array![ZERO, ONE, ZERO].span(), v(ONE, ONE))),
    )
}
fn compound() -> Shape {
    Shape::Compound(
        BoxTrait::new(CompoundTrait::new(array![(Pose2Trait::IDENTITY, cuboid())].span())),
    )
}

/// The six shapes the closed set had at `0.1.0-alpha.4` (tags `0..=5`); what the game's colliders
/// hold (`crates/rapier2d/tests/game_path.cairo`).
fn old_shapes() -> Span<Shape> {
    array![ball(), cuboid(), capsule(), segment(), halfspace(), polygon()].span()
}

/// The SH1 / SH2a / SH2b shapes (tags `6..=12`).
fn new_shapes() -> Span<Shape> {
    array![
        triangle(), round_cuboid(), round_triangle(), round_convex_polygon(), polyline(),
        heightfield(), compound(),
    ]
        .span()
}

/// One serialize / deserialize round trip of every shape in `shapes`, through `candidate`, as
/// `WorldState` does for its colliders' shapes: every shape appended to one felt array, then read
/// back in order.
fn round_trip(shapes: Span<Shape>, candidate: Candidate) -> Array<Shape> {
    let mut out: Array<felt252> = array![];
    for shape in shapes {
        match candidate {
            Candidate::Mixed => mixed_serialize(shape, ref out),
            Candidate::TwoLevel => two_level_serialize(shape, ref out),
            Candidate::SingleMatch => single_match_serialize(shape, ref out),
        }
    }
    let mut span = out.span();
    let mut result = array![];
    let n = shapes.len();
    let mut i = 0;
    while i != n {
        let shape = single_match_deserialize(ref span);
        result.append(shape.unwrap());
        i += 1;
    }
    result
}

#[derive(Copy, Drop)]
enum Candidate {
    Mixed,
    TwoLevel,
    SingleMatch,
}

/// The pre-RG2 shipped dispatch: the top match is on `Shape` itself, with the six old arms inlined
/// next to three call arms for the SH2a / SH2b payloads only (the SH1 payloads stayed inlined).
/// Tied with the shipped `flat` on `steps_game_serde_trips` (see the module doc); kept because the
/// ranking can flip between compiler versions (AGENTS §2).
fn mixed_serialize(shape: @Shape, ref output: Array<felt252>) {
    match shape {
        Shape::Ball(x) => {
            Serde::serialize(@0, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::Triangle(x) => {
            Serde::serialize(@6, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::RoundCuboid(x) => {
            Serde::serialize(@7, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::RoundTriangle(x) => {
            Serde::serialize(@8, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::RoundConvexPolygon(x) => {
            Serde::serialize(@9, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::Polyline(x) => mixed_serialize_polyline(x, ref output),
        Shape::HeightField(x) => mixed_serialize_heightfield(x, ref output),
        Shape::Compound(x) => mixed_serialize_compound(x, ref output),
        Shape::Cuboid(x) => {
            Serde::serialize(@1, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::Capsule(x) => {
            Serde::serialize(@2, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::Segment(x) => {
            Serde::serialize(@3, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::HalfSpace(x) => {
            Serde::serialize(@4, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::ConvexPolygon(x) => {
            Serde::serialize(@5, ref output);
            Serde::serialize(x, ref output);
        },
    }
}

#[inline(never)]
fn mixed_serialize_polyline(x: @Box<crate::shape::polyline::Polyline>, ref output: Array<felt252>) {
    Serde::serialize(@10, ref output);
    Serde::serialize(x, ref output);
}
#[inline(never)]
fn mixed_serialize_heightfield(
    x: @Box<crate::shape::heightfield::HeightField>, ref output: Array<felt252>,
) {
    Serde::serialize(@11, ref output);
    Serde::serialize(x, ref output);
}
#[inline(never)]
fn mixed_serialize_compound(x: @Box<crate::shape::compound::Compound>, ref output: Array<felt252>) {
    Serde::serialize(@12, ref output);
    Serde::serialize(x, ref output);
}

/// `serialize`'s top match keeps exactly the six old arms it had at `0.1.0-alpha.4` (the derived
/// shape) plus one wildcard, out of line, as `deserialize` (unchanged everywhere) already does on
/// the tag side. Tied with the shipped `flat` on `steps_game_serde_trips`.
fn two_level_serialize(shape: @Shape, ref output: Array<felt252>) {
    match shape {
        Shape::Ball(x) => {
            Serde::serialize(@0, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::Cuboid(x) => {
            Serde::serialize(@1, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::Capsule(x) => {
            Serde::serialize(@2, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::Segment(x) => {
            Serde::serialize(@3, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::HalfSpace(x) => {
            Serde::serialize(@4, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::ConvexPolygon(x) => {
            Serde::serialize(@5, ref output);
            Serde::serialize(x, ref output);
        },
        _ => two_level_serialize_new(shape, ref output),
    }
}

#[inline(never)]
fn two_level_serialize_new(shape: @Shape, ref output: Array<felt252>) {
    match shape {
        Shape::Triangle(x) => {
            Serde::serialize(@6, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::RoundCuboid(x) => {
            Serde::serialize(@7, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::RoundTriangle(x) => {
            Serde::serialize(@8, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::RoundConvexPolygon(x) => {
            Serde::serialize(@9, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::Polyline(x) => {
            Serde::serialize(@10, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::HeightField(x) => {
            Serde::serialize(@11, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::Compound(x) => {
            Serde::serialize(@12, ref output);
            Serde::serialize(x, ref output);
        },
        _ => {},
    }
}

/// Every arm inlined, none out of line: the fully-derived shape a thirteen-variant enum would get.
/// Rejected: +45 Cairo steps on `steps_game_serde_trips` relative to the shipped `flat` / `mixed` /
/// `two_level` (all three of which tie). Its `deserialize` doubles as the reference for all three
/// candidates: they serialize differently but agree on the wire, so one `match` on the tag reads
/// any of them back.
fn single_match_serialize(shape: @Shape, ref output: Array<felt252>) {
    match shape {
        Shape::Ball(x) => {
            Serde::serialize(@0, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::Triangle(x) => {
            Serde::serialize(@6, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::RoundCuboid(x) => {
            Serde::serialize(@7, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::RoundTriangle(x) => {
            Serde::serialize(@8, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::RoundConvexPolygon(x) => {
            Serde::serialize(@9, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::Polyline(x) => {
            Serde::serialize(@10, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::HeightField(x) => {
            Serde::serialize(@11, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::Compound(x) => {
            Serde::serialize(@12, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::Cuboid(x) => {
            Serde::serialize(@1, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::Capsule(x) => {
            Serde::serialize(@2, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::Segment(x) => {
            Serde::serialize(@3, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::HalfSpace(x) => {
            Serde::serialize(@4, ref output);
            Serde::serialize(x, ref output);
        },
        Shape::ConvexPolygon(x) => {
            Serde::serialize(@5, ref output);
            Serde::serialize(x, ref output);
        },
    }
}

fn single_match_deserialize(ref serialized: Span<felt252>) -> Option<Shape> {
    let idx: felt252 = Serde::deserialize(ref serialized)?;
    Some(
        match idx {
            0 => Shape::Ball(Serde::deserialize(ref serialized)?),
            1 => Shape::Cuboid(Serde::deserialize(ref serialized)?),
            2 => Shape::Capsule(Serde::deserialize(ref serialized)?),
            3 => Shape::Segment(Serde::deserialize(ref serialized)?),
            4 => Shape::HalfSpace(Serde::deserialize(ref serialized)?),
            5 => Shape::ConvexPolygon(Serde::deserialize(ref serialized)?),
            6 => Shape::Triangle(Serde::deserialize(ref serialized)?),
            7 => Shape::RoundCuboid(Serde::deserialize(ref serialized)?),
            8 => Shape::RoundTriangle(Serde::deserialize(ref serialized)?),
            9 => Shape::RoundConvexPolygon(Serde::deserialize(ref serialized)?),
            10 => Shape::Polyline(Serde::deserialize(ref serialized)?),
            11 => Shape::HeightField(Serde::deserialize(ref serialized)?),
            12 => Shape::Compound(Serde::deserialize(ref serialized)?),
            _ => { return None; },
        },
    )
}

/// The three candidates agree on every shape of the closed set, old and new, and their bytes match
/// the shipped `ShapeSerde`.
#[test]
fn test_candidates_agree_and_match_shipped() {
    let mut shapes: Array<Shape> = array![];
    for shape in old_shapes() {
        shapes.append(*shape);
    }
    for shape in new_shapes() {
        shapes.append(*shape);
    }
    let shapes = shapes.span();
    for shape in shapes {
        let mut shipped: Array<felt252> = array![];
        Serde::serialize(shape, ref shipped);
        let mut mixed: Array<felt252> = array![];
        mixed_serialize(shape, ref mixed);
        let mut two_level: Array<felt252> = array![];
        two_level_serialize(shape, ref two_level);
        let mut single_match: Array<felt252> = array![];
        single_match_serialize(shape, ref single_match);
        assert_eq!(shipped, mixed);
        assert_eq!(shipped, two_level);
        assert_eq!(shipped, single_match);
    }
    assert_eq!(round_trip(shapes, Candidate::Mixed).span(), shapes);
    assert_eq!(round_trip(shapes, Candidate::TwoLevel).span(), shapes);
    assert_eq!(round_trip(shapes, Candidate::SingleMatch).span(), shapes);
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}
#[test]
fn gas_round_trip_old_shipped() {
    let shapes = opaque(old_shapes());
    let mut out: Array<felt252> = array![];
    for shape in shapes {
        ShapeSerde::serialize(shape, ref out);
    }
    let mut span = out.span();
    let n = shapes.len();
    let mut i = 0;
    while i != n {
        let _ = Serde::<Shape>::deserialize(ref span).unwrap();
        i += 1;
    }
}
#[test]
fn gas_round_trip_old_mixed() {
    let _ = round_trip(opaque(old_shapes()), Candidate::Mixed);
}
#[test]
fn gas_round_trip_old_two_level() {
    let _ = round_trip(opaque(old_shapes()), Candidate::TwoLevel);
}
#[test]
fn gas_round_trip_old_single_match() {
    let _ = round_trip(opaque(old_shapes()), Candidate::SingleMatch);
}
#[test]
fn gas_round_trip_new_shipped() {
    let shapes = opaque(new_shapes());
    let mut out: Array<felt252> = array![];
    for shape in shapes {
        ShapeSerde::serialize(shape, ref out);
    }
    let mut span = out.span();
    let n = shapes.len();
    let mut i = 0;
    while i != n {
        let _ = Serde::<Shape>::deserialize(ref span).unwrap();
        i += 1;
    }
}
#[test]
fn gas_round_trip_new_mixed() {
    let _ = round_trip(opaque(new_shapes()), Candidate::Mixed);
}
#[test]
fn gas_round_trip_new_two_level() {
    let _ = round_trip(opaque(new_shapes()), Candidate::TwoLevel);
}
#[test]
fn gas_round_trip_new_single_match() {
    let _ = round_trip(opaque(new_shapes()), Candidate::SingleMatch);
}
