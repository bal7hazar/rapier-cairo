//! Rejected formulations of the character controller (see `super`'s candidates): the slope
//! classification by cosine comparison, the manifolds through the metered dispatcher.

use fixed::{Fixed, FixedTrait, TrigTrait, ZERO};
use glam_core::vec2::Vec2Trait;
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldTrait};
use rapier_geometry2d::dispatch::composite::contact_manifolds_composite;
use rapier_geometry2d::dispatch::contact_manifold;
use rapier_geometry2d::query::ShapeCastHit;
use rapier_geometry2d::shape::{Shape, ShapeTrait};
use rapier_math::pose2::Pose2;
use super::{HitInfo, KinematicCharacterController};

/// `angle >= theta` for glam's signed `angle = sign(perp) * acos(c)`, `theta` in `[0, pi]`,
/// without the angle: `c <= cos(theta)` on the positive side, `theta == 0 && c >= 1` on the
/// negative one.
fn angle_ge(perp: Fixed, dot: Fixed, norm: Fixed, theta: Fixed) -> bool {
    if perp >= ZERO {
        dot <= theta.cos() * norm
    } else {
        theta == ZERO && dot >= norm
    }
}

/// `angle <= theta` for the same angle, `theta` in `[0, pi]`.
fn angle_le(perp: Fixed, dot: Fixed, norm: Fixed, theta: Fixed) -> bool {
    if perp >= ZERO {
        dot >= theta.cos() * norm
    } else {
        true
    }
}

/// `compute_hit_info` by cosine comparison (thresholds in `[0, pi]`).
pub fn hit_info_cosine(controller: KinematicCharacterController, toi: ShapeCastHit) -> HitInfo {
    let up = controller.up;
    let n = toi.normal1;
    let perp = up.perp_dot(n);
    let dot = up.dot(n);
    let norm = (up.length_squared() * n.length_squared()).sqrt();
    let is_ceiling = dot < ZERO;
    let is_wall = !is_ceiling && angle_ge(perp, dot, norm, controller.max_slope_climb_angle);
    let is_nonslip_slope = angle_le(perp, dot, norm, controller.min_slope_slide_angle);
    HitInfo { toi, is_wall, is_nonslip_slope }
}

/// `contacts::manifolds` through the metered `dispatch::contact_manifold`.
pub fn manifolds_metered(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed,
) -> Array<ContactManifold> {
    if shape1.is_composite() || shape2.is_composite() {
        return contact_manifolds_composite(pos12, shape1, shape2, prediction, [].span())
            .unwrap_or_default();
    }
    let mut manifold = ContactManifoldTrait::new();
    if contact_manifold(pos12, shape1, shape2, prediction, ref manifold) {
        array![manifold]
    } else {
        array![]
    }
}
