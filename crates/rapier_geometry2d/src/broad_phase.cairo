//! Stateless 2D broad phase over AABBs.
//!
//! `find_pairs` returns index pairs `(i, j)` with `i < j`, in ascending lexicographic order.
//! Static-static pairs are skipped; group filtering and add/remove pair events are deferred to the
//! narrow phase packages. The measured dispatch is tail scan below 32 proxies, x strips for
//! 32–63, and a 2D grid from 64. The cell is a power of two derived from the proxies (median
//! of five sampled extents, `scale`): the cost is the same in a world scaled by any factor.
//! Boxes wider than two cells (grounds) are paired in one scan each instead of being inserted.
//! Crowded cells fall back to a scan;
//! the emitted-pair density chooses direct append for dense sets and the tail scan otherwise.
//! All scratch is per call. Cells use floor division over the full signed raw range and large
//! AABBs never expand into unbounded cell counts. Proxies are inserted in descending index
//! order so pairs come out as rows that only need replaying in reverse (no global sort).

use rapier_core::data::handle::Handle;
use crate::aabb::Aabb;

/// One collider proxy consumed by the stateless broad phase.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct BroadPhaseProxy {
    /// Collider handle carried for later stages; ordering here is by proxy index.
    pub collider: Handle,
    /// World-space bounds of the collider.
    pub aabb: Aabb,
    /// Whether the collider belongs to a static/fixed body.
    pub is_static: bool,
}

/// Finds every overlapping non static-static proxy pair.
///
/// The returned pairs are proxy indices, not handles, and are sorted by `(i, j)`.
#[inline(always)]
pub fn find_pairs(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    if proxies.len() < 32 {
        find_pairs_tail_static_split(proxies)
    } else if proxies.len() < 64 {
        strip::find_pairs_strip(proxies)
    } else {
        grid::find_pairs_grid(proxies)
    }
}

#[inline(always)]
fn append_pair(ref pairs: Array<(u32, u32)>, i: u32, j: u32) {
    let mut pending = true;
    while pending {
        pairs.append((i, j));
        pending = false;
    }
}

#[inline(always)]
fn overlaps_fields_flat(
    a_min_x: i64,
    a_max_x: i64,
    a_min_y: i64,
    a_max_y: i64,
    b_min_x: i64,
    b_max_x: i64,
    b_min_y: i64,
    b_max_y: i64,
) -> bool {
    a_min_x <= b_max_x && b_min_x <= a_max_x && a_min_y <= b_max_y && b_min_y <= a_max_y
}

#[inline(always)]
fn overlaps_raw(a: Aabb, b: Aabb) -> bool {
    a.mins.x.raw <= b.maxs.x.raw
        && b.mins.x.raw <= a.maxs.x.raw
        && a.mins.y.raw <= b.maxs.y.raw
        && b.mins.y.raw <= a.maxs.y.raw
}

#[inline(always)]
fn pair_allowed(a: BroadPhaseProxy, b: BroadPhaseProxy) -> bool {
    !(a.is_static && b.is_static)
}

/// Brute-force candidate: scan `i < j`, skip static-static, test both axes raw.
///
/// This is O(n²), allocation-free except for the output, and naturally produces sorted pairs.
fn find_pairs_brute(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    let n = proxies.len();
    let mut pairs = array![];
    let mut i = 0;
    while i != n {
        let a = *proxies.at(i);
        let mut j = i + 1;
        while j != n {
            let b = *proxies.at(j);
            if pair_allowed(a, b) && overlaps_raw(a.aabb, b.aabb) {
                pairs.append((i, j));
            }
            j += 1;
        }
        i += 1;
    }
    pairs
}

/// Tail-scan candidate: hoist `a`, walk the `j` tail with `pop_front`, and meter append only.
fn find_pairs_tail_static_split(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    let n = proxies.len();
    let mut pairs = array![];
    if n == 0 {
        return pairs;
    }
    let end = n - 1;
    let mut i = 0;
    while i != end {
        let a = *proxies.at(i);
        let a_aabb = a.aabb;
        let a_min_x = a_aabb.mins.x.raw;
        let a_max_x = a_aabb.maxs.x.raw;
        let a_min_y = a_aabb.mins.y.raw;
        let a_max_y = a_aabb.maxs.y.raw;
        let mut j = i + 1;
        let mut tail = proxies.slice(j, n - j);
        if a.is_static {
            while let Option::Some(value) = tail.pop_front() {
                let b = *value;
                let b_aabb = b.aabb;
                if !b.is_static
                    && overlaps_fields_flat(
                        a_min_x,
                        a_max_x,
                        a_min_y,
                        a_max_y,
                        b_aabb.mins.x.raw,
                        b_aabb.maxs.x.raw,
                        b_aabb.mins.y.raw,
                        b_aabb.maxs.y.raw,
                    ) {
                    pairs.append((i, j));
                }
                j += 1;
            }
        } else {
            while let Option::Some(value) = tail.pop_front() {
                let b = *value;
                let b_aabb = b.aabb;
                if overlaps_fields_flat(
                    a_min_x,
                    a_max_x,
                    a_min_y,
                    a_max_y,
                    b_aabb.mins.x.raw,
                    b_aabb.maxs.x.raw,
                    b_aabb.mins.y.raw,
                    b_aabb.maxs.y.raw,
                ) {
                    append_pair(ref pairs, i, j);
                }
                j += 1;
            }
        }
        i += 1;
    }
    pairs
}
#[cfg(test)]
pub mod alternatives;
#[cfg(test)]
mod benches;
mod grid;
#[cfg(test)]
mod merge_sorted;

mod ordering;
/// `ColliderPair`: two collider handles.
pub mod pair;
pub use pair::{ColliderPair, ColliderPairDefault, ColliderPairImpl, ColliderPairTrait};
mod scale;
/// BT2: pairs of the awake proxies against kept static ones.
pub mod sparse;
pub use sparse::find_pairs_sparse;
#[cfg(test)]
mod scale_benches;
#[cfg(test)]
mod scale_tests;
mod strip;
#[cfg(test)]
mod tests;

// A dense processed set pays less for direct append than for the tail scan's metered append.
// `index` is the number of proxies processed before the crowded one.
// Compare in u64: no overflow for any u32 proxy or output length. This changes only cost.
fn crowded_fallback(proxies: Span<BroadPhaseProxy>, emitted: u32, index: u32) -> Array<(u32, u32)> {
    let emitted: u64 = emitted.into();
    let index: u64 = index.into();
    if emitted * 4 > index * (index + 1) {
        find_pairs_brute(proxies)
    } else {
        find_pairs_tail_static_split(proxies)
    }
}
