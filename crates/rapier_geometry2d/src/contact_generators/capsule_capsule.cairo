//! Capsule–capsule contact manifold (Parry `contact_manifolds_capsule_capsule.rs`, 2D branch).
//!
//! The closest points of the two core segments give the first contact; when the segments are
//! nearly parallel (`|cos| >= cos(π/8)`) and nearly perpendicular to the normal
//! (`|sin| < sin(π/8)`), a second contact comes from clipping the two segments along the normal.
//! Radii are applied last: each point moves by its radius along its normal, `dist` loses both.
//!
//! Fixed-point choices:
//! * **One normalisation.** Only the normal is normalised (`try_normalize2_and_length`, whose
//!   length is the distance). The parallelism tests use the unnormalised directions `d1`, `d2`
//!   and compare squares wide: `(d1.d2)^2 >= cos^2(π/8) |d1|^2 |d2|^2` and
//!   `(d1.n)^2 < sin^2(π/8) |d1|^2`.
//! * **Exact coincidence test.** Upstream's `length_squared > ε·100` (second point distinct from
//!   the first) is the wide squared distance against `100 · DEFAULT_EPSILON` lifted to Q64.64.
//! * `local_p2` of the first point is read off `capsule2`'s own segment with the barycentric
//!   location of the closest point, instead of transforming it back through `pos12`.
use core::num::traits::{DivRem, WideMul};
use fixed::wide::{dot2, norm2_squared};
use fixed::{Fixed, ONE, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::math_ext::norm2::{norm2_sq_wide, sq_wide};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use rapier_math::try_normalize2_and_length;
use crate::clip::{ClippingPoints, clip_segment_segment_with_normal};
use crate::closest_points::closest_points_segment_segment_with_locations;
use crate::contact::{ContactManifold, ContactManifoldTrait, TrackedContact};
use crate::feature_id::FeatureIdTrait;
use crate::manifold::ManifoldTrait;
use crate::point::SegmentPointLocation;
use crate::point::segment::segment_point_at;
use crate::shape::{Capsule, Segment, Shape};

/// `cos^2(π/8) = (2 + √2) / 4` as a raw Q32.32.
const COS_SQ_FRAC_PI_8_RAW: u128 = 3665983898;
/// `sin^2(π/8) = (2 - √2) / 4` as a raw Q32.32.
const SIN_SQ_FRAC_PI_8_RAW: i64 = 628983398;
/// `100 · DEFAULT_EPSILON` (`100 · 2^-23`) lifted to the raw Q64.64 scale of `norm2_sq_wide`.
const SECOND_POINT_EPS_SQ_RAW: i128 = 219902325555200;
const Q32: NonZero<u128> = 0x1_0000_0000;

/// Upstream's capsule feature code of a segment location: `2 v` for vertex `v`, `1` inside.
#[inline(always)]
fn location_code(loc: SegmentPointLocation) -> u32 {
    match loc {
        SegmentPointLocation::OnVertex(v) => v * 2,
        SegmentPointLocation::OnEdge(_) => 1,
    }
}

/// Upstream's near-parallel test on the unnormalised directions, squared and wide.
///
/// A direction whose narrowed squared length is 0 (shorter than `2^-16`) has no direction: the
/// test fails, as upstream's `direction()` does for a zero-length segment.
fn nearly_parallel(d1: Vec2, d2: Vec2, n: Vec2) -> bool {
    let a = norm2_squared(d1.x, d1.y);
    let e = norm2_squared(d2.x, d2.y);
    if a.raw == 0 || e.raw == 0 {
        return false;
    }
    let t = dot2(d1.x, n.x, d1.y, n.y);
    if sq_wide(t) >= a.raw.wide_mul(SIN_SQ_FRAC_PI_8_RAW) {
        return false;
    }
    let b = dot2(d1.x, d2.x, d1.y, d2.y);
    let ae: u128 = a.raw.wide_mul(e.raw).try_into().unwrap();
    let (q, _) = DivRem::div_rem(ae, Q32);
    let b2: u128 = sq_wide(b).try_into().unwrap();
    b2 >= q * COS_SQ_FRAC_PI_8_RAW
}

/// The core contact (before radii) and the frame data the second point needs.
#[derive(Copy, Drop)]
struct Core {
    contact: TrackedContact,
    seg2: Segment,
    n1: Vec2,
    n2: Vec2,
}

/// First contact from the closest points of the core segments, or `None` beyond
/// `prediction + r1 + r2`.
#[inline(always)]
fn first_point(
    pos12: Pose2, capsule1: Capsule, capsule2: Capsule, prediction: Fixed,
) -> Option<Core> {
    let seg1 = capsule1.segment;
    let local_seg2 = capsule2.segment;
    let seg2 = Segment {
        a: pos12.transform_point(local_seg2.a), b: pos12.transform_point(local_seg2.b),
    };
    let (loc1, loc2) = closest_points_segment_segment_with_locations(seg1, seg2);
    let p1 = segment_point_at(seg1, loc1);
    let d = segment_point_at(seg2, loc2) - p1;
    let (n1, dist) = match try_normalize2_and_length(d.x, d.y, ZERO) {
        Some((x, y, len)) => (Vec2 { x, y }, len),
        None => (Vec2 { x: ZERO, y: ONE }, ZERO),
    };
    if dist > prediction + capsule1.radius + capsule2.radius {
        return None;
    }
    let contact = TrackedContact {
        local_p1: p1,
        local_p2: segment_point_at(local_seg2, loc2),
        dist,
        fid1: FeatureIdTrait::face(location_code(loc1)),
        fid2: FeatureIdTrait::face(location_code(loc2)),
        data: Default::default(),
    };
    Some(Core { contact, seg2, n1, n2: pos12.rotation.inverse_rotate(-n1) })
}

/// Turns a clipping point into a contact (upstream's `TrackedContact::new` of `clip_a`/`clip_b`).
#[inline(always)]
fn clipped(pos12: Pose2, c: ClippingPoints, n1: Vec2) -> TrackedContact {
    let (p1, p2, f1, f2) = c;
    let d = p2 - p1;
    TrackedContact {
        local_p1: p1,
        local_p2: pos12.inverse_transform_point(p2),
        dist: dot2(d.x, n1.x, d.y, n1.y),
        fid1: FeatureIdTrait::face(f1),
        fid2: FeatureIdTrait::face(f2),
        data: Default::default(),
    }
}

/// Upstream: `clip_a` unless it coincides with the first point, then `clip_b`.
#[inline(always)]
fn pick(
    pos12: Pose2, ca: ClippingPoints, cb: ClippingPoints, p1: Vec2, n1: Vec2,
) -> TrackedContact {
    let (a1, _, _, _) = ca;
    let d = a1 - p1;
    if norm2_sq_wide(d.x, d.y) > SECOND_POINT_EPS_SQ_RAW {
        clipped(pos12, ca, n1)
    } else {
        clipped(pos12, cb, n1)
    }
}

/// Second contact of near-parallel capsules: upstream's `clip_segment_segment_with_normal`.
#[inline(always)]
fn second_point(pos12: Pose2, seg1: Segment, core: Core) -> Option<TrackedContact> {
    let seg2 = core.seg2;
    if !nearly_parallel(seg1.b - seg1.a, seg2.b - seg2.a, core.n1) {
        return None;
    }
    match clip_segment_segment_with_normal((seg1.a, seg1.b), (seg2.a, seg2.b), core.n1) {
        Some((ca, cb)) => Some(pick(pos12, ca, cb, core.contact.local_p1, core.n1)),
        None => None,
    }
}

/// Applies the radii and writes the points and normals (upstream's final loop).
#[inline(always)]
fn finish(
    ref manifold: ContactManifold,
    old: @ContactManifold,
    core: Core,
    second: Option<TrackedContact>,
    r1: Fixed,
    r2: Fixed,
) {
    let o1 = core.n1.mul_scalar(r1);
    let o2 = core.n2.mul_scalar(r2);
    let radii = r1 + r2;
    let mut c0 = core.contact;
    c0.local_p1 = c0.local_p1 + o1;
    c0.local_p2 = c0.local_p2 + o2;
    c0.dist = c0.dist - radii;
    manifold.local_n1 = core.n1;
    manifold.local_n2 = core.n2;
    match second {
        Some(mut c1) => {
            c1.local_p1 = c1.local_p1 + o1;
            c1.local_p2 = c1.local_p2 + o2;
            c1.dist = c1.dist - radii;
            manifold.points = [c0, c1];
            manifold.num_points = 2;
        },
        None => {
            let [_, p1] = manifold.points;
            manifold.points = [c0, p1];
            manifold.num_points = 1;
        },
    }
    manifold.match_contacts(old);
}

/// Contact manifold between two capsules (Parry `contact_manifold_capsule_capsule`, 2D).
///
/// `pos12` is the pose of `capsule2` in the frame of `capsule1` (unit rotation). Points are kept
/// when the core distance is at most `prediction + r1 + r2`; otherwise the manifold is cleared
/// (normals untouched). Crossing core segments give the `+Y` fallback normal and `dist = -r1-r2`.
/// Feature ids are `face(0|1|2)`: vertex `a`, interior, vertex `b` of each core segment.
/// `ContactData` is carried over from the previous points with the same ids.
/// #### Panics
/// * As `closest_points_segment_segment_with_locations` and `clip_segment_segment_with_normal`
///   (coordinates beyond their documented ranges).
pub fn contact_manifold_capsule_capsule(
    pos12: Pose2,
    capsule1: Capsule,
    capsule2: Capsule,
    prediction: Fixed,
    ref manifold: ContactManifold,
) {
    let old = manifold;
    manifold.clear();
    if let Some(core) = first_point(pos12, capsule1, capsule2, prediction) {
        let second = second_point(pos12, capsule1.segment, core);
        finish(ref manifold, @old, core, second, capsule1.radius, capsule2.radius);
    }
}

/// [`contact_manifold_capsule_capsule`] for two [`Shape`]s: `true` when both are capsules
/// (the manifold is updated), `false` otherwise (the manifold is untouched).
pub fn contact_manifold_capsule_capsule_shapes(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    if let Shape::Capsule(capsule1) = shape1 {
        if let Shape::Capsule(capsule2) = shape2 {
            contact_manifold_capsule_capsule(pos12, capsule1, capsule2, prediction, ref manifold);
            return true;
        }
    }
    false
}

/// Rejected candidates, kept for the `gas_*` ranking.
#[cfg(test)]
pub mod alternatives {
    use fixed::Fixed;
    use rapier_math::pose2::Pose2;
    use crate::clip::clip_segment_segment_with_features;
    use crate::contact::{ContactManifold, ContactManifoldTrait, TrackedContact};
    use crate::shape::{Capsule, Segment};
    use super::{Core, finish, first_point, nearly_parallel, pick};

    /// Variant (b): the second point from the plain clip along segment 1's direction
    /// (`clip_segment_segment_with_features`), same selection rule.
    fn second_point_features(pos12: Pose2, seg1: Segment, core: Core) -> Option<TrackedContact> {
        let seg2 = core.seg2;
        if !nearly_parallel(seg1.b - seg1.a, seg2.b - seg2.a, core.n1) {
            return None;
        }
        match clip_segment_segment_with_features((seg1.a, seg1.b), (seg2.a, seg2.b)) {
            Some((ca, cb)) => Some(pick(pos12, ca, cb, core.contact.local_p1, core.n1)),
            None => None,
        }
    }

    pub fn contact_manifold_capsule_capsule_clip_features(
        pos12: Pose2,
        capsule1: Capsule,
        capsule2: Capsule,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) {
        let old = manifold;
        manifold.clear();
        if let Some(core) = first_point(pos12, capsule1, capsule2, prediction) {
            let second = second_point_features(pos12, capsule1.segment, core);
            finish(ref manifold, @old, core, second, capsule1.radius, capsule2.radius);
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
    use glam::Vec2;
    use rapier_golden::contact_manifolds::{
        CAPSULE_CAPSULE_DEEP, CAPSULE_CAPSULE_DEGENERATE, CAPSULE_CAPSULE_SEPARATED,
        CAPSULE_CAPSULE_SHALLOW, CAPSULE_CAPSULE_TOUCHING, CAPSULE_CAPSULE_WITHIN_PRED, PREDICTION,
    };
    use rapier_golden::types::{ManifoldCase, ShapeRaw, Vec2Raw};
    use rapier_math::pose2::{IDENTITY, Pose2};
    use rapier_math::rot2::{Rot2, Rot2Trait};
    use rapier_testing::opaque;
    use crate::contact::{ContactData, ContactManifold, ContactManifoldTrait};
    use crate::feature_id::FeatureIdTrait;
    use crate::shape::{Ball, Capsule, CapsuleTrait, Shape};
    use super::alternatives::contact_manifold_capsule_capsule_clip_features;
    use super::{contact_manifold_capsule_capsule, contact_manifold_capsule_capsule_shapes};

    fn f(n: i64, d: i64) -> Fixed {
        FixedTrait::from_raw(n * 0x1_0000_0000 / d)
    }
    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }
    fn at(x: Fixed, y: Fixed) -> Pose2 {
        Pose2 { translation: v(x, y), ..IDENTITY }
    }
    fn quarter(x: Fixed, y: Fixed) -> Pose2 {
        Pose2 { translation: v(x, y), rotation: Rot2 { re: ZERO, im: ONE } }
    }
    fn close(a: Vec2, b: Vec2) -> bool {
        (a.x - b.x).abs().raw <= 0x10000 && (a.y - b.y).abs().raw <= 0x10000
    }
    fn vr(r: Vec2Raw) -> Vec2 {
        v(FixedTrait::from_raw(r.x), FixedTrait::from_raw(r.y))
    }
    fn capsule_of(s: ShapeRaw) -> Capsule {
        match s {
            ShapeRaw::Capsule(c) => CapsuleTrait::new(
                vr(c.a), vr(c.b), FixedTrait::from_raw(c.radius),
            ),
            _ => panic!("not a capsule"),
        }
    }
    fn input(c: ManifoldCase) -> (Pose2, Capsule, Capsule) {
        let p = c.pos12;
        let pose = Pose2 {
            translation: vr(p.translation),
            rotation: Rot2 {
                re: FixedTrait::from_raw(p.rotation.re), im: FixedTrait::from_raw(p.rotation.im),
            },
        };
        (pose, capsule_of(c.shape1), capsule_of(c.shape2))
    }
    fn run(pos12: Pose2, c1: Capsule, c2: Capsule, prediction: Fixed) -> ContactManifold {
        let mut m: ContactManifold = Default::default();
        contact_manifold_capsule_capsule(pos12, c1, c2, prediction, ref m);
        m
    }

    /// `(pose, capsule2 half height, expected points, dist of point 0, n1)`: capsule 1 is
    /// `new_y(1/2, 1/4)`, capsule 2 `new_y(h, 1/4)` (or `new_x` when `h < 0`).
    #[test]
    fn test_regimes_table() {
        let c1 = CapsuleTrait::new_y(HALF, f(1, 4));
        let p = f(1, 10);
        let cases: Span<(Pose2, Capsule, u8, Fixed, Vec2)> = array![
            // Parallel stacked side by side, overlapping: two points, 0.1 apart.
            (at(f(6, 10), f(1, 5)), c1, 2, p, v(ONE, ZERO)),
            // Crossing cores (a plus sign): +Y fallback, one point, dist = -r1 - r2.
            (quarter(ZERO, ZERO), c1, 1, -HALF, v(ZERO, ONE)),
            // T shape: capsule 2 horizontal above capsule 1, touching its end cap.
            (at(ZERO, ONE), CapsuleTrait::new_x(HALF, f(1, 4)), 1, ZERO, v(ZERO, ONE)),
            // Separated within prediction, parallel and aligned end to end: one point.
            (at(ZERO, f(3, 2) + f(1, 20)), c1, 1, f(1, 20), v(ZERO, ONE)),
            // Beyond prediction: no point.
            (at(f(2, 1), ZERO), c1, 0, ZERO, v(ZERO, ZERO)),
            // Zero-length capsule 2 (a ball): no second point.
            (at(f(4, 10), ZERO), CapsuleTrait::new_y(ZERO, f(1, 4)), 1, -p, v(ONE, ZERO)),
        ]
            .span();
        for (pose, c2, n, dist, normal) in cases {
            let m = run(*pose, c1, *c2, p);
            assert_eq!(m.num_points, *n);
            if *n != 0 {
                assert!((m.point(0).dist - *dist).abs().raw <= 2);
                assert_eq!(m.local_n1, *normal);
                assert_eq!(m.local_n2, pose.rotation.inverse_rotate(-*normal));
            }
        }
    }

    #[test]
    fn test_parallel_points_ids_and_radius_offsets() {
        let c = CapsuleTrait::new_y(HALF, f(1, 4));
        let m = run(at(f(6, 10), f(1, 4)), c, c, f(1, 10));
        // First: seg1 interior at y = -1/4 against seg2's vertex a; second: seg1's b.
        assert_eq!(m.point(0).local_p1, v(f(1, 4), f(-1, 4)));
        assert_eq!(m.point(0).local_p2, v(f(-1, 4), f(-1, 2)));
        assert_eq!(m.point(0).fid1, FeatureIdTrait::face(1));
        assert_eq!(m.point(0).fid2, FeatureIdTrait::face(0));
        assert_eq!(m.point(1).local_p1, v(f(1, 4), HALF));
        assert_eq!(m.point(1).fid1, FeatureIdTrait::face(2));
        assert_eq!(m.point(1).fid2, FeatureIdTrait::face(1));
    }

    #[test]
    fn test_warm_start_and_shapes_dispatch() {
        let c = CapsuleTrait::new_y(HALF, f(1, 4));
        let pose = at(f(6, 10), f(1, 5));
        let mut m = run(pose, c, c, ONE);
        let data = ContactData { impulse: ONE, ..Default::default() };
        let [p0, p1] = m.points;
        m.points = [p0, super::TrackedContact { data, ..p1 }];
        let s = Shape::Capsule(c);
        assert!(contact_manifold_capsule_capsule_shapes(pose, s, s, ONE, ref m));
        assert_eq!(m.point(1).data, data);
        assert_eq!(m.point(0).data, Default::default());
        let before = m;
        let ball = Shape::Ball(Ball { radius: ONE });
        assert!(!contact_manifold_capsule_capsule_shapes(pose, s, ball, ZERO, ref m));
        assert!(!contact_manifold_capsule_capsule_shapes(pose, ball, s, ZERO, ref m));
        assert_eq!(m, before);
    }

    /// The two second-point constructions agree on every golden input and on rotated stacks.
    #[test]
    fn test_second_point_variants_agree() {
        let pred = FixedTrait::from_raw(PREDICTION);
        let goldens = array![
            CAPSULE_CAPSULE_WITHIN_PRED, CAPSULE_CAPSULE_TOUCHING, CAPSULE_CAPSULE_DEEP,
            CAPSULE_CAPSULE_SHALLOW, CAPSULE_CAPSULE_DEGENERATE,
        ];
        for c in goldens.span() {
            let (pose, c1, c2) = input(*c);
            let mut m: ContactManifold = Default::default();
            contact_manifold_capsule_capsule_clip_features(pose, c1, c2, pred, ref m);
            assert_eq!(m, run(pose, c1, c2, pred));
        }
    }

    #[fuzzer(runs: 64, seed: 7)]
    #[test]
    fn fuzz_second_point_variants(x: i16, y: i16, im: i16, h: u16, flip: bool) {
        let c1 = CapsuleTrait::new_y(HALF, f(1, 4));
        // Half heights >= 1/8: for much shorter cores the two projection axes can pick different
        // clip points (seen at h ~ 0.0014), so the variants are only compared on this domain.
        let c2 = CapsuleTrait::new_y(
            FixedTrait::from_raw(h.into() * 0x1_0000 + 0x2000_0000), f(1, 5),
        );
        // Near-identity or near-half-turn rotations keep the cores near-parallel.
        let re = if flip {
            -ONE
        } else {
            ONE
        };
        let pose = Pose2 {
            translation: v(
                FixedTrait::from_raw(x.into() * 0x1_0000),
                FixedTrait::from_raw(y.into() * 0x1_0000),
            ),
            rotation: Rot2 { re, im: FixedTrait::from_raw(im.into()) },
        };
        let mut a: ContactManifold = Default::default();
        contact_manifold_capsule_capsule_clip_features(pose, c1, c2, ONE, ref a);
        let b = run(pose, c1, c2, ONE);
        // Same count, first point and ids. The second point is not bit-identical: (b) projects
        // on segment 1's direction, (a) on the normal's tangent, which differ by the small
        // angle between the cores (measured up to ~5e3 ulps here).
        assert_eq!(a.num_points, b.num_points);
        assert_eq!(a.point(0), b.point(0));
        if a.num_points == 2 {
            let (p, q) = (a.point(1), b.point(1));
            assert_eq!((p.fid1, p.fid2), (q.fid1, q.fid2));
            assert!(close(p.local_p1, q.local_p1) && close(p.local_p2, q.local_p2));
            assert!((p.dist - q.dist).abs().raw <= 0x10000);
        }
    }

    fn probe(c: ManifoldCase) {
        let (pose, c1, c2) = input(opaque(c));
        let _ = run(pose, c1, c2, FixedTrait::from_raw(PREDICTION));
    }
    fn probe_alt(c: ManifoldCase) {
        let (pose, c1, c2) = input(opaque(c));
        let mut m: ContactManifold = Default::default();
        contact_manifold_capsule_capsule_clip_features(
            pose, c1, c2, FixedTrait::from_raw(PREDICTION), ref m,
        );
    }

    #[test]
    fn gas_baseline() {
        let (pose, c1, c2) = input(opaque(CAPSULE_CAPSULE_DEEP));
        let _ = (pose, c1, c2);
        let _: ContactManifold = Default::default();
    }
    #[test]
    fn gas_capsule_capsule_separated() {
        probe(CAPSULE_CAPSULE_SEPARATED);
    }
    #[test]
    fn gas_capsule_capsule_within_pred() {
        probe(CAPSULE_CAPSULE_WITHIN_PRED);
    }
    #[test]
    fn gas_capsule_capsule_touching() {
        probe(CAPSULE_CAPSULE_TOUCHING);
    }
    #[test]
    fn gas_capsule_capsule_shallow() {
        probe(CAPSULE_CAPSULE_SHALLOW);
    }
    #[test]
    fn gas_capsule_capsule_deep() {
        probe(CAPSULE_CAPSULE_DEEP);
    }
    #[test]
    fn gas_capsule_capsule_degenerate() {
        probe(CAPSULE_CAPSULE_DEGENERATE);
    }
    #[test]
    fn gas_capsule_capsule_clip_features_within_pred() {
        probe_alt(CAPSULE_CAPSULE_WITHIN_PRED);
    }
    #[test]
    fn gas_capsule_capsule_clip_features_deep() {
        probe_alt(CAPSULE_CAPSULE_DEEP);
    }
    #[test]
    fn gas_capsule_capsule_shapes() {
        let (pose, c1, c2) = input(opaque(CAPSULE_CAPSULE_DEEP));
        let mut m: ContactManifold = Default::default();
        let _ = contact_manifold_capsule_capsule_shapes(
            pose, Shape::Capsule(c1), Shape::Capsule(c2), FixedTrait::from_raw(PREDICTION), ref m,
        );
    }
}
