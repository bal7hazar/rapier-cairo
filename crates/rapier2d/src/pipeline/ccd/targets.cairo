//! The targets of the bullets and of the substep splitter (upstream queries its broad-phase tree
//! with each fast collider's swept box): the colliders whose broad-phase box meets `region`, the
//! union of the swept boxes of the pass.
//!
//! When the world's active set (BT2) describes the world (valid, sets unmodified since the step
//! filled it), its static proxies already hold the boxes of every fixed, sleeping and parentless
//! collider, loosened by half the prediction distance as upstream's tree: only the colliders
//! whose box meets `region` are read, plus the colliders of the awake bodies. Otherwise every
//! collider is read (`sweeps::collect_targets`). Both give the same targets, in another order
//! (statics then awake colliders, against ascending slot).

use fixed::{Fixed, HALF};
use rapier_core::Handle;
use rapier_core::collider::ColliderEnabled;
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::rigid_body_set::{RigidBodySet, RigidBodySetTrait};
use rapier_geometry2d::aabb::{Aabb, AabbTrait};
use rapier_geometry2d::broad_phase::BroadPhaseProxy;
use super::sweeps::{FastCollider, Target, collect_targets, contains, target_with_parent};

/// What the targets read from the world's active set (BT2): its validity, the prediction
/// distance of its static proxies, the static proxies and the awake colliders.
#[derive(Copy, Drop)]
pub struct ActiveView {
    pub valid: bool,
    pub prediction: Fixed,
    pub statics: Span<BroadPhaseProxy>,
    /// The awake colliders, each with its parent's position in `bodies`.
    pub awake: Span<(Handle, u32)>,
    /// The awake bodies.
    pub bodies: Span<Handle>,
}

/// The union of the swept boxes of `fast` (`None` when empty).
pub fn swept_region(fast: Span<Span<FastCollider>>) -> Option<Aabb> {
    let mut region: Option<Aabb> = None;
    for colliders in fast {
        for fc in *colliders {
            region =
                Some(match region {
                    Some(r) => r.merged(*fc.swept_aabb),
                    None => *fc.swept_aabb,
                });
        }
    }
    region
}

/// The targets meeting `region` (see the module documentation); `bullets` are the handles of the
/// bullet bodies, `margin` the loosening of the target boxes (half the prediction distance).
pub fn targets_near(
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    active: ActiveView,
    bullets: Span<Handle>,
    margin: Fixed,
    region: Aabb,
) -> Array<Target> {
    let mut out = array![];
    if active.valid && active.prediction
        * HALF == margin && !bodies.is_modified() && !colliders.is_modified() {
        for proxy in active.statics {
            if proxy.aabb.intersects(region) {
                if let Some(collider) = colliders.get(*proxy.collider) {
                    if collider.flags.enabled == ColliderEnabled::Enabled {
                        let mut target = target_with_parent(
                            ref bodies, *proxy.collider, collider, bullets, margin,
                        );
                        target.aabb = *proxy.aabb;
                        out.append(target);
                    }
                }
            }
        }
        for entry in active.awake {
            let (handle, parent) = *entry;
            // A bullet's collider is never a bullet's target: skipped without a read.
            if let Some(body) = active.bodies.get(parent) {
                if contains(bullets, *body.unbox()) {
                    continue;
                }
            }
            if let Some(collider) = colliders.get(handle) {
                if collider.flags.enabled == ColliderEnabled::Enabled {
                    let target = target_with_parent(ref bodies, handle, collider, bullets, margin);
                    if target.aabb.intersects(region) {
                        out.append(target);
                    }
                }
            }
        }
        return out;
    }
    for target in collect_targets(ref bodies, ref colliders, bullets, margin, false) {
        if target.aabb.intersects(region) {
            out.append(target);
        }
    }
    out
}
