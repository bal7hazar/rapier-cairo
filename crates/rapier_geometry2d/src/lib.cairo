//! 2D collision geometry for rapier.cairo: the subset of Parry that Rapier's step consumes.
//!
//! The types shared with `rapier_dynamics2d` are frozen in `docs/interfaces/geometry-dynamics.md`
//! and live in [`contact`], [`feature_id`] and [`mass`]; the algorithms (shapes, AABB, SAT,
//! clipping, manifold generators, broad phase) are wave-3 work packages.

pub mod contact;
pub mod feature_id;
pub mod mass;
