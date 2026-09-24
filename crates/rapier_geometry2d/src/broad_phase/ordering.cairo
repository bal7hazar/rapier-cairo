use super::{BroadPhaseProxy, overlaps_raw, pair_allowed};

#[inline(always)]
fn pair_before(a: (u32, u32), b: (u32, u32)) -> bool {
    let (ai, aj) = a;
    let (bi, bj) = b;
    ai < bi || (ai == bi && aj < bj)
}

fn merge_pair_runs(
    src: Span<(u32, u32)>, left: u32, mid: u32, right: u32, ref dst: Array<(u32, u32)>,
) {
    let mut i = left;
    let mut j = mid;
    while i != mid && j != right {
        let a = *src.at(i);
        let b = *src.at(j);
        if pair_before(a, b) {
            dst.append(a);
            i += 1;
        } else {
            dst.append(b);
            j += 1;
        }
    }
    while i != mid {
        dst.append(*src.at(i));
        i += 1;
    }
    while j != right {
        dst.append(*src.at(j));
        j += 1;
    }
}

/// Restores ascending lexicographic order without depending on dict traversal order.
pub(crate) fn merge_sort_pairs(mut src: Array<(u32, u32)>) -> Array<(u32, u32)> {
    let n = src.len();
    let mut width = 1;
    while width < n {
        let span = src.span();
        let mut dst = array![];
        let mut left = 0;
        while left != n {
            let mut mid = left + width;
            if mid > n {
                mid = n;
            }
            let mut right = mid + width;
            if right > n {
                right = n;
            }
            merge_pair_runs(span, left, mid, right, ref dst);
            left = right;
        }
        src = dst;
        width = width * 2;
    }
    src
}

/// Row `i` of a large proxy: every later proxy, in ascending `j`, so the row is sorted.
pub(crate) fn large_row(
    proxies: Span<BroadPhaseProxy>, i: u32, a: BroadPhaseProxy, ref pairs: Array<(u32, u32)>,
) {
    let mut j = i + 1;
    let mut tail = proxies.slice(j, proxies.len() - j);
    while let Option::Some(b) = tail.pop_front() {
        let b = *b;
        if pair_allowed(a, b) && overlaps_raw(a.aabb, b.aabb) {
            pairs.append((i, j));
        }
        j += 1;
    }
}

/// Tests a small proxy against the large proxies already seen (all later, stored descending).
pub(crate) fn large_partners(
    proxies: Span<BroadPhaseProxy>,
    i: u32,
    a: BroadPhaseProxy,
    mut large: Span<u32>,
    ref pairs: Array<(u32, u32)>,
) {
    while let Option::Some(l) = large.pop_back() {
        let j = *l;
        let b = *proxies.at(j);
        if pair_allowed(a, b) && overlaps_raw(a.aabb, b.aabb) {
            pairs.append((i, j));
        }
    }
}

/// `pairs` holds rows `(i, j)` emitted for descending `i`, each row contiguous. Replays the rows
/// from the last one (smallest `i`) and sorts a row by `j` only when it arrived out of order,
/// so the bookkeeping is paid per emitted pair, never per proxy.
pub(crate) fn rows_ascending(pairs: Array<(u32, u32)>) -> Array<(u32, u32)> {
    let src = pairs.span();
    let mut out = array![];
    let mut end = src.len();
    while end != 0 {
        let (i, mut next_j) = *src.at(end - 1);
        let mut start = end - 1;
        let mut sorted = true;
        while start != 0 {
            let (pi, pj) = *src.at(start - 1);
            if pi != i {
                break;
            }
            if pj > next_j {
                sorted = false;
            }
            next_j = pj;
            start -= 1;
        }
        let row = src.slice(start, end - start);
        if sorted {
            for p in row {
                out.append(*p);
            }
        } else {
            let mut tmp = array![];
            for p in row {
                tmp.append(*p);
            }
            for p in merge_sort_pairs(tmp) {
                out.append(p);
            }
        }
        end = start;
    }
    out
}
