//! Contact-manifold persistence helpers (Parry `ContactManifold` methods).
//!
//! The 2D manifold has at most two points, so matching stays fully unrolled: no dictionaries, no
//! allocation and no dependence on iteration order. Compound subshape poses are deferred; `pos12`
//! is always the relative pose of shape 2 in shape 1's frame.

use fixed::wide::dot2;
use fixed::{Fixed, ZERO};
use glam::Vec2;
use rapier_math::consts::{COS_1_DEGREES, DIST_SQ_THRESHOLD_RAW};
use rapier_math::math_ext::norm2::{norm2_sq_wide, sq_wide};
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::contact::{ContactData, ContactManifold, TrackedContact};

#[generate_trait]
pub impl ManifoldImpl of ManifoldTrait {
    /// Attempts to refresh the stored points for `pos12` using Parry's default tolerances.
    ///
    /// The normal may rotate by less than 1 degree (`COS_1_DEGREES`) and each reprojected point
    /// may move by at most `1e-3` (`DIST_SQ_THRESHOLD_RAW`, compared wide). On success only
    /// `TrackedContact::dist` is updated; `local_p1`, `local_p2`, feature ids and data are kept.
    /// Returns `false` for an empty manifold or when full regeneration is required.
    fn try_update_contacts(ref self: ContactManifold, pos12: Pose2) -> bool {
        self.try_update_contacts_eps(pos12, COS_1_DEGREES, DIST_SQ_THRESHOLD_RAW)
    }

    /// Attempts to refresh the stored points for `pos12` with caller-provided tolerances.
    ///
    /// `angle_cos_tol` is compared to `-local_n1 . (pos12.rotation * local_n2)`. `dist_sq_tol`
    /// is a raw Q64.64 squared-distance threshold so sub-ulp squared tolerances never underflow.
    /// Panics only as the underlying fixed-point pose transforms and wide products do.
    fn try_update_contacts_eps(
        ref self: ContactManifold, pos12: Pose2, angle_cos_tol: Fixed, dist_sq_tol: i128,
    ) -> bool {
        try_update_contacts_eps_fused(ref self, pos12, angle_cos_tol, dist_sq_tol)
    }

    /// Transfers `ContactData` from `old` where `(fid1, fid2)` match.
    ///
    /// Unknown feature ids (`packed == 0`) never match, even when both sides are unknown.
    /// Unmatched contacts are left untouched; generators are expected to create them with default
    /// data before calling this method.
    fn match_contacts(ref self: ContactManifold, old: @ContactManifold) {
        let old_num = *old.num_points;
        let [old0, old1] = *old.points;
        let [mut p0, mut p1] = self.points;
        if self.num_points != 0 {
            p0.data = match_data_by_id(p0, old_num, old0, old1);
        }
        if self.num_points > 1 {
            p1.data = match_data_by_id(p1, old_num, old0, old1);
        }
        self.points = [p0, p1];
    }

    /// Transfers `ContactData` from `old` when both local anchors are within `dist_threshold`.
    ///
    /// This is the fallback for shape pairs without stable feature ids. The threshold is squared
    /// wide (`sq_wide`) and the comparison is strict, matching upstream.
    fn match_contacts_using_positions(
        ref self: ContactManifold, old: @ContactManifold, dist_threshold: Fixed,
    ) {
        let threshold = sq_wide(dist_threshold);
        let old_num = *old.num_points;
        let [old0, old1] = *old.points;
        let [mut p0, mut p1] = self.points;
        if self.num_points != 0 {
            p0.data = match_data_by_position(p0, old_num, old0, old1, threshold);
        }
        if self.num_points > 1 {
            p1.data = match_data_by_position(p1, old_num, old0, old1, threshold);
        }
        self.points = [p0, p1];
    }

    /// Returns the index of the contact with the smallest signed distance.
    ///
    /// `None` means the manifold is empty. Ties keep the first point, preserving generation order.
    fn find_deepest_contact(self: ContactManifold) -> Option<u8> {
        if self.num_points == 0 {
            None
        } else if self.num_points == 1 {
            Some(0)
        } else {
            let [p0, p1] = self.points;
            if p1.dist < p0.dist {
                Some(1)
            } else {
                Some(0)
            }
        }
    }

    /// Returns the largest signed distance among stored contacts, or zero for an empty manifold.
    ///
    /// Points beyond prediction are not filtered here; this reports what the manifold stores.
    fn max_dist(self: ContactManifold) -> Fixed {
        if self.num_points == 0 {
            ZERO
        } else if self.num_points == 1 {
            let [p0, _] = self.points;
            p0.dist
        } else {
            let [p0, p1] = self.points;
            if p1.dist > p0.dist {
                p1.dist
            } else {
                p0.dist
            }
        }
    }
}

fn try_update_contacts_eps_fused(
    ref manifold: ContactManifold, pos12: Pose2, angle_cos_tol: Fixed, dist_sq_tol: i128,
) -> bool {
    if manifold.num_points == 0 {
        return false;
    }
    if !normal_matches(manifold.local_n1, manifold.local_n2, pos12, angle_cos_tol) {
        return false;
    }

    let [p0, p1] = manifold.points;
    let (ok0, u0) = update_candidate(p0, pos12, manifold.local_n1, dist_sq_tol);
    if !ok0 {
        return false;
    }
    if manifold.num_points == 1 {
        manifold.points = [u0, p1];
        return true;
    }

    let (ok1, u1) = update_candidate(p1, pos12, manifold.local_n1, dist_sq_tol);
    if !ok1 {
        return false;
    }
    manifold.points = [u0, u1];
    true
}

fn normal_matches(local_n1: Vec2, local_n2: Vec2, pos12: Pose2, angle_cos_tol: Fixed) -> bool {
    let n2_in_1 = pos12.transform_vector(local_n2);
    -dot2(local_n1.x, n2_in_1.x, local_n1.y, n2_in_1.y) >= angle_cos_tol
}

fn update_candidate(
    mut pt: TrackedContact, pos12: Pose2, local_n1: Vec2, dist_sq_tol: i128,
) -> (bool, TrackedContact) {
    let local_p2 = pos12.transform_point(pt.local_p2);
    let dpt = local_p2 - pt.local_p1;
    let dist = dot2(dpt.x, local_n1.x, dpt.y, local_n1.y);

    if sign_switched(dist, pt.dist) {
        return (false, pt);
    }

    let new_p1 = Vec2 { x: local_p2.x - local_n1.x * dist, y: local_p2.y - local_n1.y * dist };
    let delta = new_p1 - pt.local_p1;
    if norm2_sq_wide(delta.x, delta.y) > dist_sq_tol {
        return (false, pt);
    }

    pt.dist = dist;
    (true, pt)
}

fn sign_switched(a: Fixed, b: Fixed) -> bool {
    (a.raw < 0 && b.raw > 0) || (a.raw > 0 && b.raw < 0)
}

fn known_pair(pt: TrackedContact) -> bool {
    pt.fid1.packed != 0 && pt.fid2.packed != 0
}

fn same_ids(a: TrackedContact, b: TrackedContact) -> bool {
    a.fid1 == b.fid1 && a.fid2 == b.fid2
}

fn match_data_by_id(
    contact: TrackedContact, old_num: u8, old0: TrackedContact, old1: TrackedContact,
) -> ContactData {
    if known_pair(contact) && old_num != 0 && same_ids(contact, old0) {
        old0.data
    } else if known_pair(contact) && old_num > 1 && same_ids(contact, old1) {
        old1.data
    } else {
        contact.data
    }
}

fn close_positions(a: TrackedContact, b: TrackedContact, threshold_sq_raw: i128) -> bool {
    let dp1 = a.local_p1 - b.local_p1;
    let dp2 = a.local_p2 - b.local_p2;
    norm2_sq_wide(dp1.x, dp1.y) < threshold_sq_raw && norm2_sq_wide(dp2.x, dp2.y) < threshold_sq_raw
}

fn match_data_by_position(
    contact: TrackedContact,
    old_num: u8,
    old0: TrackedContact,
    old1: TrackedContact,
    threshold_sq_raw: i128,
) -> ContactData {
    if old_num != 0 && close_positions(contact, old0, threshold_sq_raw) {
        old0.data
    } else if old_num > 1 && close_positions(contact, old1, threshold_sq_raw) {
        old1.data
    } else {
        contact.data
    }
}

#[cfg(test)]
mod alternatives {
    use fixed::wide::dot2;
    use fixed::{Fixed, ZERO};
    use rapier_math::math_ext::norm2::norm2_sq_wide;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use crate::contact::{ContactManifold, TrackedContact};
    use super::normal_matches;

    /// Direct upstream shape: validate all points, then refresh separations in a second pass.
    pub fn try_update_contacts_eps_direct(
        ref manifold: ContactManifold, pos12: Pose2, angle_cos_tol: Fixed, dist_sq_tol: i128,
    ) -> bool {
        if manifold.num_points == 0 {
            return false;
        }
        if !normal_matches(manifold.local_n1, manifold.local_n2, pos12, angle_cos_tol) {
            return false;
        }

        let mut i = 0_u8;
        while i != manifold.num_points {
            let pt = point(manifold, i);
            let local_p2 = pos12.transform_point(pt.local_p2);
            let dpt = local_p2 - pt.local_p1;
            let dist = dot2(dpt.x, manifold.local_n1.x, dpt.y, manifold.local_n1.y);
            if dist * pt.dist < ZERO {
                return false;
            }
            let new_p1 = local_p2
                - glam::Vec2 { x: manifold.local_n1.x * dist, y: manifold.local_n1.y * dist };
            let delta = new_p1 - pt.local_p1;
            if norm2_sq_wide(delta.x, delta.y) > dist_sq_tol {
                return false;
            }
            i += 1;
        }

        let [mut p0, mut p1] = manifold.points;
        if manifold.num_points != 0 {
            p0.dist = separation(p0, pos12, manifold.local_n1);
        }
        if manifold.num_points > 1 {
            p1.dist = separation(p1, pos12, manifold.local_n1);
        }
        manifold.points = [p0, p1];
        true
    }

    fn point(manifold: ContactManifold, i: u8) -> TrackedContact {
        let [p0, p1] = manifold.points;
        if i == 0 {
            p0
        } else {
            p1
        }
    }

    fn separation(pt: TrackedContact, pos12: Pose2, local_n1: glam::Vec2) -> Fixed {
        let local_p2 = pos12.transform_point(pt.local_p2);
        let dpt = local_p2 - pt.local_p1;
        dot2(dpt.x, local_n1.x, dpt.y, local_n1.y)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, ONE, ZERO};
    use glam::Vec2;
    use rapier_math::consts::{COS_1_DEGREES, DIST_SQ_THRESHOLD_RAW};
    use rapier_math::pose2::Pose2;
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::contact::{ContactData, ContactManifold, TrackedContact};
    use crate::feature_id::{FEATURE_UNKNOWN, FeatureIdTrait};
    use super::{ManifoldTrait, alternatives};

    const COS_0_5: Fixed = Fixed { raw: 4294803757 };
    const SIN_0_5: Fixed = Fixed { raw: 37480185 };
    const COS_2: Fixed = Fixed { raw: 4292350918 };
    const SIN_2: Fixed = Fixed { raw: 149892197 };
    const ONE_MILLI: Fixed = Fixed { raw: 4294967 };
    const HALF_MILLI: Fixed = Fixed { raw: 2147484 };
    const TWO_MILLI: Fixed = Fixed { raw: 8589935 };

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn pose(tx: Fixed, ty: Fixed, re: Fixed, im: Fixed) -> Pose2 {
        Pose2 { translation: v(tx, ty), rotation: Rot2 { re, im } }
    }

    fn data(raw: i64) -> ContactData {
        ContactData { impulse: Fixed { raw }, ..Default::default() }
    }

    fn contact(code: u32, dist: Fixed, p1: Vec2, p2: Vec2, impulse: i64) -> TrackedContact {
        TrackedContact {
            local_p1: p1,
            local_p2: p2,
            dist,
            fid1: FeatureIdTrait::face(code),
            fid2: FeatureIdTrait::vertex(code + 10),
            data: data(impulse),
        }
    }

    fn manifold(p0: TrackedContact, p1: TrackedContact, n: u8) -> ContactManifold {
        ContactManifold {
            points: [p0, p1],
            num_points: n,
            local_n1: v(ONE, ZERO),
            local_n2: v(-ONE, ZERO),
            ..Default::default(),
        }
    }

    fn first(m: ContactManifold) -> TrackedContact {
        let [p0, _] = m.points;
        p0
    }

    fn second(m: ContactManifold) -> TrackedContact {
        let [_, p1] = m.points;
        p1
    }

    #[test]
    fn test_try_update_angle_and_point_thresholds() {
        let c = contact(1, ZERO, v(ZERO, ZERO), v(ZERO, ZERO), 0);
        let mut empty: ContactManifold = Default::default();
        assert!(!empty.try_update_contacts(Default::default()));

        let ok_rot = pose(ZERO, ZERO, COS_0_5, SIN_0_5);
        let bad_rot = pose(ZERO, ZERO, COS_2, SIN_2);
        assert!(manifold(c, Default::default(), 1).try_update_contacts(ok_rot));
        assert!(!manifold(c, Default::default(), 1).try_update_contacts(bad_rot));

        let mut small = manifold(c, Default::default(), 1);
        assert!(small.try_update_contacts(pose(ZERO, HALF_MILLI, ONE, ZERO)));
        assert_eq!(first(small).dist, ZERO);

        let mut large = manifold(c, Default::default(), 1);
        assert!(!large.try_update_contacts(pose(ZERO, TWO_MILLI, ONE, ZERO)));

        let mut sep = manifold(c, Default::default(), 1);
        assert!(sep.try_update_contacts(pose(ONE_MILLI, ZERO, ONE, ZERO)));
        assert_eq!(first(sep).dist, ONE_MILLI);
    }

    #[test]
    fn test_try_update_rejects_sign_flip() {
        let c = contact(1, -ONE_MILLI, v(ZERO, ZERO), v(ZERO, ZERO), 0);
        let mut m = manifold(c, Default::default(), 1);
        assert!(!m.try_update_contacts(pose(ONE_MILLI, ZERO, ONE, ZERO)));
        assert_eq!(first(m).dist, -ONE_MILLI);
    }

    #[test]
    fn test_match_contacts_by_ids() {
        let old0 = contact(1, ZERO, v(ZERO, ZERO), v(ZERO, ZERO), 10);
        let old1 = contact(2, ZERO, v(ONE, ZERO), v(ONE, ZERO), 20);
        let old = manifold(old0, old1, 2);

        let mut swapped = manifold(
            contact(2, ZERO, v(ZERO, ZERO), v(ZERO, ZERO), 0),
            contact(1, ZERO, v(ZERO, ZERO), v(ZERO, ZERO), 0),
            2,
        );
        swapped.match_contacts(@old);
        assert_eq!(first(swapped).data.impulse.raw, 20);
        assert_eq!(second(swapped).data.impulse.raw, 10);

        let mut one = manifold(contact(1, ZERO, v(ZERO, ZERO), v(ZERO, ZERO), 0), old1, 1);
        one.match_contacts(@old);
        assert_eq!(first(one).data.impulse.raw, 10);

        let mut none = manifold(contact(7, ZERO, v(ZERO, ZERO), v(ZERO, ZERO), 0), old1, 1);
        none.match_contacts(@old);
        assert_eq!(first(none).data.impulse.raw, 0);
    }

    #[test]
    fn test_unknown_ids_never_match_and_positions_can_match() {
        let unknown = TrackedContact {
            fid1: FEATURE_UNKNOWN, fid2: FEATURE_UNKNOWN, data: data(33), ..Default::default(),
        };
        let old = manifold(unknown, Default::default(), 1);
        let mut by_id = manifold(
            TrackedContact { data: data(0), ..unknown }, Default::default(), 1,
        );
        by_id.match_contacts(@old);
        assert_eq!(first(by_id).data.impulse.raw, 0);

        let mut by_pos = by_id;
        by_pos.match_contacts_using_positions(@old, ONE_MILLI);
        assert_eq!(first(by_pos).data.impulse.raw, 33);
    }

    #[test]
    fn test_empty_deepest_and_max_dist() {
        let empty: ContactManifold = Default::default();
        assert_eq!(empty.find_deepest_contact(), None);
        assert_eq!(empty.max_dist(), ZERO);

        let m = manifold(
            contact(1, FixedTrait::from_raw(-3), v(ZERO, ZERO), v(ZERO, ZERO), 0),
            contact(2, FixedTrait::from_raw(5), v(ZERO, ZERO), v(ZERO, ZERO), 0),
            2,
        );
        assert_eq!(m.find_deepest_contact(), Some(0));
        assert_eq!(m.max_dist(), FixedTrait::from_raw(5));
    }

    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_try_update_candidates_equivalent(tx: i16, ty: i16, two_points: bool) {
        let p0 = contact(1, ZERO, v(ZERO, ZERO), v(ZERO, ZERO), 0);
        let p1 = contact(2, ZERO, v(ONE, ZERO), v(ONE, ZERO), 0);
        let n = if two_points {
            2
        } else {
            1
        };
        let delta = pose(Fixed { raw: tx.into() }, Fixed { raw: ty.into() }, ONE, ZERO);
        let mut a = manifold(p0, p1, n);
        let mut b = a;
        let ra = a.try_update_contacts_eps(delta, COS_1_DEGREES, DIST_SQ_THRESHOLD_RAW);
        let rb = alternatives::try_update_contacts_eps_direct(
            ref b, delta, COS_1_DEGREES, DIST_SQ_THRESHOLD_RAW,
        );
        assert_eq!(ra, rb);
        assert_eq!(a, b);
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }

    #[test]
    fn gas_try_update_contacts() {
        let c = contact(1, ZERO, v(ZERO, ZERO), v(ZERO, ZERO), 0);
        assert!(manifold(c, Default::default(), 1).try_update_contacts(opaque(Default::default())));
    }

    #[test]
    fn gas_try_update_contacts_eps_fused() {
        let c = contact(1, ZERO, v(ZERO, ZERO), v(ZERO, ZERO), 0);
        let mut m = manifold(c, Default::default(), 1);
        assert!(
            m
                .try_update_contacts_eps(
                    opaque(Default::default()),
                    opaque(COS_1_DEGREES),
                    opaque(DIST_SQ_THRESHOLD_RAW),
                ),
        );
    }

    #[test]
    fn gas_try_update_contacts_eps_direct() {
        let c = contact(1, ZERO, v(ZERO, ZERO), v(ZERO, ZERO), 0);
        let mut m = manifold(c, Default::default(), 1);
        assert!(
            alternatives::try_update_contacts_eps_direct(
                ref m,
                opaque(Default::default()),
                opaque(COS_1_DEGREES),
                opaque(DIST_SQ_THRESHOLD_RAW),
            ),
        );
    }

    #[test]
    fn gas_match_contacts() {
        let old = manifold(
            contact(1, ZERO, v(ZERO, ZERO), v(ZERO, ZERO), 10),
            contact(2, ZERO, v(ONE, ZERO), v(ONE, ZERO), 20),
            2,
        );
        let mut m = manifold(
            contact(2, ZERO, v(ZERO, ZERO), v(ZERO, ZERO), 0),
            contact(1, ZERO, v(ZERO, ZERO), v(ZERO, ZERO), 0),
            2,
        );
        m.match_contacts(opaque(@old));
    }

    #[test]
    fn gas_match_contacts_using_positions() {
        let old = manifold(
            contact(1, ZERO, v(ZERO, ZERO), v(ZERO, ZERO), 10), Default::default(), 1,
        );
        let mut m = manifold(
            contact(2, ZERO, v(ZERO, ZERO), v(ZERO, ZERO), 0), Default::default(), 1,
        );
        m.match_contacts_using_positions(opaque(@old), opaque(ONE_MILLI));
    }

    #[test]
    fn gas_find_deepest_contact() {
        let m = manifold(
            contact(1, -ONE_MILLI, v(ZERO, ZERO), v(ZERO, ZERO), 0),
            contact(2, ZERO, v(ZERO, ZERO), v(ZERO, ZERO), 0),
            2,
        );
        assert_eq!(opaque(m).find_deepest_contact(), Some(0));
    }

    #[test]
    fn gas_max_dist() {
        let m = manifold(
            contact(1, -ONE_MILLI, v(ZERO, ZERO), v(ZERO, ZERO), 0),
            contact(2, TWO_MILLI, v(ZERO, ZERO), v(ZERO, ZERO), 0),
            2,
        );
        assert_eq!(opaque(m).max_dist(), TWO_MILLI);
    }
}
