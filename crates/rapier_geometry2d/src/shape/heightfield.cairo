//! The 2D `HeightField` (Parry `shape/heightfield2.rs`, work package SH2a): `n` heights sampled
//! at regular abscissae over `[-scale.x / 2, scale.x / 2]`, each consecutive pair a cell
//! (a segment), each cell enabled or removed ([`HeightFieldCellStatus`], `true` = enabled).
//!
//! # Fixed point
//!
//! Upstream computes a cell's abscissa as `(-0.5 + i / (n - 1)) * scale.x` with three roundings;
//! the port floors the exact rational `scale.x * (2 i - (n - 1)) / (2 (n - 1))` once, so shared
//! vertices of neighbouring cells are identical and the ends are exactly `±scale.x / 2`. The cell
//! range of a box (`map_elements_in_local_aabb`) is the same floor / ceil of exact rationals.
//!
//! # Deviations
//!
//! * `scale.x` must be positive (upstream divides by it and silently mirrors a negative one).
//! * The height filter of `map_elements_in_local_aabb` compares the scaled heights with the box
//!   (upstream divides the box by `scale.y`, which swaps its bounds for a negative `scale.y`).
//! * `height_at_point` answers the height of the cell's line at `pt.x`; upstream answers
//!   `seg.a.y + (height - pt.y)` (its line-line parameter), a height only when `pt.y == seg.a.y`.
//! * `set_scale` recomputes the box from the heights (upstream scales it by `new / old`).

use fixed::{Fixed, HALF, MAX, MIN, ONE, ZERO};
use glam::Vec2;
use rapier_math::pose2::Pose2;
use crate::aabb::bounding_volume::{BoundingSphere, BoundingSphereTrait};
use crate::aabb::{Aabb, AabbTrait};
use crate::mass::MassProperties;
use crate::shape::segment::{Segment, SegmentTrait};

/// Failure modes of [`HeightFieldTrait`].
pub mod errors {
    /// Fewer than two heights.
    pub const TOO_FEW_HEIGHTS: felt252 = 'HeightField: < 2 heights';
    /// `scale.x <= 0`.
    pub const SCALE_X: felt252 = 'HeightField: scale.x <= 0';
    /// A cell index is not below `num_cells`.
    pub const CELL_INDEX: felt252 = 'HeightField: cell index';
}

/// The status of a cell (upstream `HeightFieldCellStatus`, 2D): `true` when the cell exists,
/// `false` when it was removed.
pub type HeightFieldCellStatus = bool;

/// A 2D heightfield (upstream `HeightField`, 2D).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct HeightField {
    heights: Span<Fixed>,
    status: Span<HeightFieldCellStatus>,
    scale: Vec2,
    aabb: Aabb,
}

/// `Serde` of the boxed variant payload.
pub impl BoxedHeightFieldSerde of Serde<Box<HeightField>> {
    fn serialize(self: @Box<HeightField>, ref output: Array<felt252>) {
        Serde::serialize(@(*self).unbox(), ref output);
    }
    fn deserialize(ref serialized: Span<felt252>) -> Option<Box<HeightField>> {
        Some(BoxTrait::new(Serde::deserialize(ref serialized)?))
    }
}

pub impl BoxedHeightFieldPartialEq of PartialEq<Box<HeightField>> {
    fn eq(lhs: @Box<HeightField>, rhs: @Box<HeightField>) -> bool {
        (*lhs).unbox() == (*rhs).unbox()
    }
    fn ne(lhs: @Box<HeightField>, rhs: @Box<HeightField>) -> bool {
        !Self::eq(lhs, rhs)
    }
}

/// `floor(num / den)` for `den > 0`.
#[inline(always)]
fn floor_div(num: i128, den: i128) -> i128 {
    let q = num / den;
    if num < 0 && q * den != num {
        q - 1
    } else {
        q
    }
}

/// `ceil(num / den)` for `den > 0`.
#[inline(always)]
fn ceil_div(num: i128, den: i128) -> i128 {
    -floor_div(-num, den)
}

/// The box of `heights` at `scale` (upstream `new`): `x` in `±scale.x / 2`, `y` between the
/// scaled extreme heights.
fn heights_aabb(heights: Span<Fixed>, scale: Vec2) -> Aabb {
    let mut min = MAX;
    let mut max = MIN;
    for h in heights {
        if *h < min {
            min = *h;
        }
        if *h > max {
            max = *h;
        }
    }
    let half_x = scale.x * HALF;
    Aabb { mins: Vec2 { x: -half_x, y: min * scale.y }, maxs: Vec2 { x: half_x, y: max * scale.y } }
}

#[generate_trait]
pub impl HeightFieldImpl of HeightFieldTrait {
    /// A heightfield with every cell enabled (upstream `new`).
    /// #### Panics
    /// * [`errors::TOO_FEW_HEIGHTS`] for fewer than two heights, [`errors::SCALE_X`] for
    ///   `scale.x <= 0`.
    fn new(heights: Span<Fixed>, scale: Vec2) -> HeightField {
        assert(heights.len() > 1, errors::TOO_FEW_HEIGHTS);
        assert(scale.x > ZERO, errors::SCALE_X);
        let mut status = array![];
        let mut i: u32 = 1;
        while i != heights.len() {
            status.append(true);
            i += 1;
        }
        HeightField { heights, status: status.span(), scale, aabb: heights_aabb(heights, scale) }
    }

    /// The number of cells, `heights.len() - 1` (upstream `num_cells`).
    #[inline(always)]
    fn num_cells(self: @HeightField) -> u32 {
        self.heights.len() - 1
    }

    /// The heights (upstream `heights`).
    #[inline(always)]
    fn heights(self: @HeightField) -> Span<Fixed> {
        *self.heights
    }

    /// The status of every cell (`true` = enabled).
    #[inline(always)]
    fn cells_statuses(self: @HeightField) -> Span<HeightFieldCellStatus> {
        *self.status
    }

    /// The scale (upstream `scale`).
    #[inline(always)]
    fn scale(self: @HeightField) -> Vec2 {
        *self.scale
    }

    /// Replaces the scale (upstream `set_scale`), the box recomputed from the heights.
    /// #### Panics
    /// * [`errors::SCALE_X`] for `new_scale.x <= 0`.
    fn set_scale(ref self: HeightField, new_scale: Vec2) {
        assert(new_scale.x > ZERO, errors::SCALE_X);
        self.scale = new_scale;
        self.aabb = heights_aabb(self.heights, new_scale);
    }

    /// The heightfield with its scale multiplied component-wise by `scale` (upstream `scaled`).
    /// #### Panics
    /// * As [`HeightFieldTrait::set_scale`].
    fn scaled(self: @HeightField, scale: Vec2) -> HeightField {
        let mut out = *self;
        out.set_scale(*self.scale * scale);
        out
    }

    /// The local box (upstream `root_aabb`).
    #[inline(always)]
    fn root_aabb(self: @HeightField) -> Aabb {
        *self.aabb
    }

    /// The local box (upstream `local_aabb`).
    #[inline(always)]
    fn local_aabb(self: @HeightField) -> Aabb {
        *self.aabb
    }

    /// The local box moved by `pose` (upstream `aabb`).
    #[inline(always)]
    fn aabb(self: @HeightField, pose: Pose2) -> Aabb {
        self.aabb.transform_by(pose)
    }

    /// Upstream name of [`HeightFieldTrait::local_aabb`] on `Shape`.
    #[inline(always)]
    fn compute_local_aabb(self: @HeightField) -> Aabb {
        *self.aabb
    }

    /// Upstream name of [`HeightFieldTrait::aabb`] on `Shape`.
    #[inline(always)]
    fn compute_aabb(self: @HeightField, pose: Pose2) -> Aabb {
        self.aabb(pose)
    }

    /// The bounding sphere of the local box (upstream `local_bounding_sphere`).
    fn local_bounding_sphere(self: @HeightField) -> BoundingSphere {
        self.aabb.bounding_sphere()
    }

    /// [`HeightFieldTrait::local_bounding_sphere`] placed at `pose` (upstream `bounding_sphere`).
    fn bounding_sphere(self: @HeightField, pose: Pose2) -> BoundingSphere {
        self.local_bounding_sphere().transform_by(pose)
    }

    /// Zero: a heightfield has no area (upstream `Shape::mass_properties`).
    #[inline(always)]
    fn mass_properties(self: @HeightField, density: Fixed) -> MassProperties {
        Default::default()
    }

    /// The width of a cell, `scale.x / (n - 1)` floored (upstream `cell_width`).
    fn cell_width(self: @HeightField) -> Fixed {
        let cells: i128 = self.num_cells().into();
        let raw: i128 = (*self.scale).x.raw.into();
        Fixed { raw: floor_div(raw, cells).try_into().unwrap() }
    }

    /// The width of a cell before scaling, `1 / (n - 1)` floored (upstream `unit_cell_width`).
    fn unit_cell_width(self: @HeightField) -> Fixed {
        let cells: i128 = self.num_cells().into();
        let one: i128 = ONE.raw.into();
        Fixed { raw: floor_div(one, cells).try_into().unwrap() }
    }

    /// The abscissa of the first height, `-scale.x / 2` (upstream `start_x`).
    #[inline(always)]
    fn start_x(self: @HeightField) -> Fixed {
        self.x_at(0)
    }

    /// The abscissa of height `i`: `scale.x * (2 i - (n - 1)) / (2 (n - 1))`, floored.
    fn x_at(self: @HeightField, i: u32) -> Fixed {
        let cells: i128 = self.num_cells().into();
        let i: i128 = i.into();
        let raw: i128 = (*self.scale).x.raw.into();
        Fixed { raw: floor_div(raw * (2 * i - cells), 2 * cells).try_into().unwrap() }
    }

    /// The cell below or above `pt` (upstream `cell_at_point`): `None` when `pt.x` is outside
    /// `±scale.x / 2`; the last cell for `pt.x == scale.x / 2`.
    fn cell_at_point(self: @HeightField, pt: Vec2) -> Option<u32> {
        let cells: i128 = self.num_cells().into();
        let sx: i128 = (*self.scale).x.raw.into();
        let px: i128 = pt.x.raw.into();
        let offset = 2 * px + sx;
        if offset < 0 || offset > 2 * sx {
            return None;
        }
        let cell = floor_div(offset * cells, 2 * sx);
        let cell = if cell > cells - 1 {
            cells - 1
        } else {
            cell
        };
        Some(cell.try_into().unwrap())
    }

    /// The height of the cell below or above `pt`, at `pt.x` (see the module documentation for
    /// the deviation); `None` outside the heightfield or on a removed cell.
    fn height_at_point(self: @HeightField, pt: Vec2) -> Option<Fixed> {
        let cell = self.cell_at_point(pt)?;
        let seg = self.segment_at(cell)?;
        let dx = seg.b.x - seg.a.x;
        if dx == ZERO {
            return Some(seg.a.y);
        }
        let px: i128 = (pt.x - seg.a.x).raw.into();
        let dy: i128 = (seg.b.y - seg.a.y).raw.into();
        let num = px * dy;
        let den: i128 = dx.raw.into();
        Some(seg.a.y + Fixed { raw: floor_div(num, den).try_into().unwrap() })
    }

    /// The segment of cell `i` whatever its status.
    /// #### Panics
    /// * [`errors::CELL_INDEX`] when `i >= num_cells`.
    fn cell_segment(self: @HeightField, i: u32) -> Segment {
        assert(i < self.num_cells(), errors::CELL_INDEX);
        let sy = (*self.scale).y;
        SegmentTrait::new(
            Vec2 { x: self.x_at(i), y: *self.heights.at(i) * sy },
            Vec2 { x: self.x_at(i + 1), y: *self.heights.at(i + 1) * sy },
        )
    }

    /// The segment of cell `i`, `None` when `i >= num_cells` or the cell is removed (upstream
    /// `segment_at`).
    fn segment_at(self: @HeightField, i: u32) -> Option<Segment> {
        if i >= self.num_cells() || !*self.status.at(i) {
            return None;
        }
        Some(self.cell_segment(i))
    }

    /// The segments of the enabled cells, ascending (upstream `segments`).
    fn segments(self: @HeightField) -> Array<Segment> {
        let mut out = array![];
        let mut i: u32 = 0;
        while i != self.num_cells() {
            if *self.status.at(i) {
                out.append(self.cell_segment(i));
            }
            i += 1;
        }
        out
    }

    /// Removes or restores cell `i` (upstream `set_segment_removed`).
    /// #### Panics
    /// * [`errors::CELL_INDEX`] when `i >= num_cells`.
    fn set_segment_removed(ref self: HeightField, i: u32, removed: bool) {
        assert(i < self.num_cells(), errors::CELL_INDEX);
        let mut status = array![];
        let mut k: u32 = 0;
        for s in self.status {
            status.append(if k == i {
                !removed
            } else {
                *s
            });
            k += 1;
        }
        self.status = status.span();
    }

    /// `true` when cell `i` is removed (upstream `is_segment_removed`).
    #[inline(always)]
    fn is_segment_removed(self: @HeightField, i: u32) -> bool {
        !*self.status.at(i)
    }

    /// The cells whose abscissa range meets the one of `aabb`, unclamped (upstream
    /// `unclamped_elements_range_in_local_aabb`): `(start, end)`, `end` excluded.
    fn unclamped_elements_range_in_local_aabb(self: @HeightField, aabb: Aabb) -> (i64, i64) {
        let cells: i128 = self.num_cells().into();
        let sx: i128 = (*self.scale).x.raw.into();
        let mins_x: i128 = aabb.mins.x.raw.into();
        let maxs_x: i128 = aabb.maxs.x.raw.into();
        let lo = (2 * mins_x + sx) * cells;
        let hi = (2 * maxs_x + sx) * cells;
        (floor_div(lo, 2 * sx).try_into().unwrap(), ceil_div(hi, 2 * sx).try_into().unwrap())
    }

    /// The enabled cells whose segment may meet `aabb` (upstream `map_elements_in_local_aabb`):
    /// the cells of its abscissa range (clamped), minus those with both end points above or
    /// both below the box. Ascending `(cell, segment)`.
    fn elements_in_local_aabb(self: @HeightField, aabb: Aabb) -> Array<(u32, Segment)> {
        let mut out = array![];
        let half: i128 = (*self.scale).x.raw.into();
        let (lo_x, hi_x): (i128, i128) = (aabb.mins.x.raw.into(), aabb.maxs.x.raw.into());
        // Outside the abscissa range `±scale.x / 2` (compared doubled, exactly).
        if 2 * hi_x < -half || 2 * lo_x > half {
            return out;
        }
        let cells: i64 = self.num_cells().into();
        let (start, end) = self.unclamped_elements_range_in_local_aabb(aabb);
        let start = if start < 0 {
            0
        } else if start > cells - 1 {
            cells - 1
        } else {
            start
        };
        let end = if end < 0 {
            0
        } else if end > cells {
            cells
        } else {
            end
        };
        let mut i: u32 = start.try_into().unwrap();
        let end: u32 = end.try_into().unwrap();
        while i < end {
            if *self.status.at(i) {
                let seg = self.cell_segment(i);
                let above = seg.a.y > aabb.maxs.y && seg.b.y > aabb.maxs.y;
                let below = seg.a.y < aabb.mins.y && seg.b.y < aabb.mins.y;
                if !above && !below {
                    out.append((i, seg));
                }
            }
            i += 1;
        }
        out
    }
}

#[cfg(test)]
mod tests;
