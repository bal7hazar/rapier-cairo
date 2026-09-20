//! Temporary GB representations. Replace imports with `crate::shape` after GB lands.
use fixed::Fixed;
use glam::Vec2;
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Cuboid {
    pub half_extents: Vec2,
}
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Segment {
    pub a: Vec2,
    pub b: Vec2,
}
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Capsule {
    pub segment: Segment,
    pub radius: Fixed,
}
