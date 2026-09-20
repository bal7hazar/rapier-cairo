//! Mass properties of a shape (Parry's `MassProperties`, 2D).
//!
//! Frozen in `docs/interfaces/geometry-dynamics.md` §4. The constructors per shape, `transform_by`
//! and the combination of two properties are work package GB; only the type is fixed here.
//! `inv_mass == 0` means infinite mass, and `rapier_math::math_ext::inv` (`inv(0) = 0`) is what
//! makes fixed bodies work without special cases.

use fixed::Fixed;
use glam::vec2::Vec2;

/// Centre of mass in the shape's local frame, inverse mass, inverse principal angular inertia
/// (a scalar in 2D).
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct MassProperties {
    pub local_com: Vec2,
    pub inv_mass: Fixed,
    pub inv_principal_inertia: Fixed,
}
