//! The live-pair list helpers of the sparse step (`super::sparse_step`): the pairs of the
//! narrow phase's list that are not dormant, kept by position (BT2, BT4), and with the dormant
//! pairs kept apart (WS3: [`same_keys`], [`live_positions`]).

use rapier_dynamics2d::narrow_phase::{ContactPair, key_before};

/// `live` (ascending key) merged into `dormant` (ascending key, disjoint keys), with the
/// positions of the `live` pairs in the result.
pub(crate) fn merge_live(
    live: Span<ContactPair>, dormant: Span<ContactPair>,
) -> (Array<ContactPair>, Array<u32>) {
    let mut live = live;
    let mut dormant = dormant;
    let mut out = array![];
    let mut positions = array![];
    while let Some(a) = live.pop_front() {
        while let Some(d) = dormant.get(0) {
            let d = d.unbox();
            if key_before(*d.collider1, *d.collider2, *a.collider1, *a.collider2) {
                out.append(*d);
                dormant.pop_front().unwrap();
            } else {
                break;
            }
        }
        positions.append(out.len());
        out.append(*a);
    }
    out.append_span(dormant);
    (out, positions)
}

/// Whether `live` has the keys of the pairs of `pairs` at `positions` (ascending), in order, and
/// whether it has their values too (BT4: the previous step's live pairs are read in place, not
/// kept as a copy). The value comparison stops at the first difference or touching pair; a
/// `false` second answer only costs a copy of the list.
pub(crate) fn compare_live(
    pairs: Span<ContactPair>, positions: Span<u32>, live: Span<ContactPair>,
) -> (bool, bool) {
    if live.len() != positions.len() {
        return (false, false);
    }
    let mut live = live;
    let mut unchanged = true;
    for position in positions {
        let now = live.pop_front().unwrap();
        let before = pairs.at(*position);
        if before.collider1 != now.collider1 || before.collider2 != now.collider2 {
            return (false, false);
        }
        // A touching pair got new impulses from the solver: not worth comparing (a pair found
        // equal is copied all the same).
        if unchanged && (*now.manifold.data.num_solver_contacts != 0 || before != now) {
            unchanged = false;
        }
    }
    (true, unchanged)
}

/// `pairs` with the pair at `positions[k]` replaced by `live[k]` (same count, ascending
/// positions): one copy of the list.
pub(crate) fn write_live(
    pairs: Span<ContactPair>, positions: Span<u32>, live: Span<ContactPair>,
) -> Array<ContactPair> {
    let mut out = array![];
    let mut positions = positions;
    let mut live = live;
    let mut next = match positions.pop_front() {
        Some(p) => *p,
        None => NO_POSITION,
    };
    let mut k: u32 = 0;
    for pair in pairs {
        if k == next {
            out.append(*live.pop_front().unwrap());
            next = match positions.pop_front() {
                Some(p) => *p,
                None => NO_POSITION,
            };
        } else {
            out.append(*pair);
        }
        k += 1;
    }
    out
}

/// Whether `before` and `now` (ascending key) have the same keys, in order (WS3: then the live
/// pairs keep their positions in the whole list).
pub(crate) fn same_keys(before: Span<ContactPair>, now: Span<ContactPair>) -> bool {
    if before.len() != now.len() {
        return false;
    }
    let mut now = now;
    for pair in before {
        let other = now.pop_front().unwrap();
        if pair.collider1 != other.collider1 || pair.collider2 != other.collider2 {
            return false;
        }
    }
    true
}

/// The positions of the `live` pairs in the ascending merge of `live` and `dormant` (disjoint
/// keys): [`merge_live`]'s positions without the merged list (WS3).
pub(crate) fn live_positions(live: Span<ContactPair>, dormant: Span<ContactPair>) -> Array<u32> {
    let mut dormant = dormant;
    let mut positions = array![];
    let mut k: u32 = 0;
    for a in live {
        while let Some(d) = dormant.get(0) {
            let d = d.unbox();
            if key_before(*d.collider1, *d.collider2, *a.collider1, *a.collider2) {
                k += 1;
                dormant.pop_front().unwrap();
            } else {
                break;
            }
        }
        positions.append(k);
        k += 1;
    }
    positions
}

/// Past every position of a pair list.
const NO_POSITION: u32 = 0xffffffff;

/// `pairs` split by `positions` (ascending): the pairs at those positions, and the others.
pub(crate) fn split_at_positions(
    pairs: Span<ContactPair>, positions: Span<u32>,
) -> (Array<ContactPair>, Array<ContactPair>) {
    let mut live = array![];
    let mut dormant = array![];
    let mut positions = positions;
    let mut k: u32 = 0;
    for pair in pairs {
        let is_live = match positions.get(0) {
            Some(p) => *p.unbox() == k,
            None => false,
        };
        if is_live {
            positions.pop_front().unwrap();
            live.append(*pair);
        } else {
            dormant.append(*pair);
        }
        k += 1;
    }
    (live, dormant)
}
