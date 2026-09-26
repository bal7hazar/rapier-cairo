//! Sparse broad phase (work package BT2): the pairs of the awake proxies of a world whose other
//! proxies are static and kept from one step to the next (`rapier2d::pipeline::active_set`).
//!
//! [`find_pairs_sparse`] returns exactly the pairs [`super::find_pairs`] returns on the merge of
//! both lists by collider slot (every overlapping pair with at least one dynamic proxy, closed
//! intervals, `(i, j)` ascending), without walking a static row against static proxies: a
//! static row only visits the dynamic proxies after it, a dynamic row the whole tail (both lists
//! merged by slot). Cost: `O(S · D + D · (S + D))` overlap tests for `S` static and `D` dynamic
//! proxies, nothing when `D = 0`.

use super::BroadPhaseProxy;

/// The overlap test of `find_pairs` (closed intervals, raw comparisons).
#[inline(always)]
fn overlaps(a: @BroadPhaseProxy, b: @BroadPhaseProxy) -> bool {
    let a = a.aabb;
    let b = b.aabb;
    a.mins.x.raw <= b.maxs.x.raw
        && b.mins.x.raw <= a.maxs.x.raw
        && a.mins.y.raw <= b.maxs.y.raw
        && b.mins.y.raw <= a.maxs.y.raw
}

/// Every overlapping pair between a proxy of `dynamic` and a proxy of `statics` or another of
/// `dynamic`, in the order `find_pairs` gives them on the merge of both lists by collider slot.
/// A pair is `(a, b)`, indices into the concatenation `statics ++ dynamic` (a dynamic proxy `k`
/// is `statics.len() + k`), `a` the proxy of the lower collider slot.
///
/// Both lists must be in ascending collider slot (`collider.index`) with no slot in both; the
/// `is_static` flags are not read (membership decides). Static-static pairs are never tested.
pub fn find_pairs_sparse(
    statics: Span<BroadPhaseProxy>, dynamic: Span<BroadPhaseProxy>,
) -> Array<(u32, u32)> {
    let n_s = statics.len();
    let n_d = dynamic.len();
    let mut pairs = array![];
    let mut si: u32 = 0;
    let mut di: u32 = 0;
    while di != n_d {
        let d = dynamic.at(di);
        let d_slot = *d.collider.index;
        // Static rows below `d`: their partners are the dynamic proxies from `d` on.
        while si != n_s {
            let a = statics.at(si);
            if *a.collider.index > d_slot {
                break;
            }
            let mut dj = di;
            let mut tail = dynamic.slice(di, n_d - di);
            while let Some(b) = tail.pop_front() {
                if overlaps(a, b) {
                    pairs.append((si, n_s + dj));
                }
                dj += 1;
            }
            si += 1;
        }
        // Row `d`: the static proxies from `si` on and the dynamic ones after `d`, by slot.
        let mut sj = si;
        let mut dj = di + 1;
        loop {
            let take_static = if sj == n_s {
                if dj == n_d {
                    break;
                }
                false
            } else if dj == n_d {
                true
            } else {
                *statics.at(sj).collider.index < *dynamic.at(dj).collider.index
            };
            if take_static {
                if overlaps(d, statics.at(sj)) {
                    pairs.append((n_s + di, sj));
                }
                sj += 1;
            } else {
                if overlaps(d, dynamic.at(dj)) {
                    pairs.append((n_s + di, n_s + dj));
                }
                dj += 1;
            }
        }
        di += 1;
    }
    pairs
}

#[cfg(test)]
mod tests {
    use fixed::Fixed;
    use glam::Vec2;
    use rapier_core::data::handle::HandleTrait;
    use rapier_testing::opaque;
    use crate::aabb::AabbTrait;
    use super::find_pairs_sparse;
    use super::super::{BroadPhaseProxy, find_pairs};

    fn proxy(slot: u32, x: i64, y: i64, w: i64, h: i64, is_static: bool) -> BroadPhaseProxy {
        let one: i64 = 0x100000000;
        BroadPhaseProxy {
            collider: HandleTrait::new(slot, 0),
            aabb: AabbTrait::new(
                Vec2 { x: Fixed { raw: x * one }, y: Fixed { raw: y * one } },
                Vec2 { x: Fixed { raw: (x + w) * one }, y: Fixed { raw: (y + h) * one } },
            ),
            is_static,
        }
    }

    fn position(i: u32, n_s: u32, static_at: Span<u32>, dynamic_at: Span<u32>) -> u32 {
        if i < n_s {
            *static_at.at(i)
        } else {
            *dynamic_at.at(i - n_s)
        }
    }

    /// `find_pairs` on `all` (ascending slot) against `find_pairs_sparse` on its split, the
    /// sparse indices mapped back to positions in `all`.
    fn assert_equivalent(all: Span<BroadPhaseProxy>) {
        let mut statics = array![];
        let mut dynamic = array![];
        let mut static_at = array![];
        let mut dynamic_at = array![];
        let mut k: u32 = 0;
        for p in all {
            if *p.is_static {
                statics.append(*p);
                static_at.append(k);
            } else {
                dynamic.append(*p);
                dynamic_at.append(k);
            }
            k += 1;
        }
        let n_s = statics.len();
        let expected = find_pairs(all);
        let actual = find_pairs_sparse(statics.span(), dynamic.span());
        assert_eq!(actual.len(), expected.len());
        let mut i = 0;
        while i != expected.len() {
            let (a, b) = *actual.at(i);
            let a = position(a, n_s, static_at.span(), dynamic_at.span());
            let b = position(b, n_s, static_at.span(), dynamic_at.span());
            assert_eq!((a, b), *expected.at(i));
            i += 1;
        }
    }

    /// Worlds of 0–70 proxies (tail scan, strips and grid on the reference side), static every
    /// `k`-th proxy, sizes up to 7 cells, a ground spanning everything first or last.
    #[test]
    #[fuzzer(runs: 48, seed: 20260925)]
    fn fuzz_sparse_matches_find_pairs(seed: u16) {
        let s: u32 = seed.into();
        let n = s % 71;
        let every = 1 + s % 5;
        let ground_first = s % 2 == 0;
        let mut all = array![];
        let mut slot: u32 = 0;
        if ground_first {
            all.append(proxy(slot, -1000, -2, 2000, 2, true));
            slot += 1;
        }
        let mut i: u32 = 0;
        while i != n {
            let x: i64 = ((s + i * 17) % 29).into();
            let y: i64 = ((s + i * 31) % 13).into();
            let w: i64 = (1 + (s + i) % 7).into();
            let h: i64 = (1 + (s + 3 * i) % 3).into();
            all.append(proxy(slot, x - 14, y - 2, w, h, (i + s) % every == 0));
            slot += 1;
            i += 1;
        }
        if !ground_first {
            all.append(proxy(slot, -1000, -2, 2000, 2, true));
        }
        assert_equivalent(all.span());
    }

    /// Edge cases: empty lists, all static, all dynamic, touching boundaries (closed intervals).
    #[test]
    fn test_sparse_edge_cases() {
        assert_eq!(find_pairs_sparse(array![].span(), array![].span()).len(), 0);
        let statics = array![proxy(0, 0, 0, 1, 1, true), proxy(1, 0, 0, 1, 1, true)];
        assert_eq!(find_pairs_sparse(statics.span(), array![].span()).len(), 0);
        let cases = array![
            array![proxy(0, 0, 0, 1, 1, false), proxy(1, 1, 1, 1, 1, false)],
            array![proxy(0, 0, 0, 1, 1, true), proxy(1, 1, 0, 1, 1, false)],
            array![proxy(0, 0, 0, 1, 1, false), proxy(1, 2, 0, 1, 1, true)],
            array![
                proxy(0, 0, 0, 4, 4, true), proxy(1, 1, 1, 1, 1, false), proxy(2, 0, 0, 4, 4, true),
                proxy(3, 1, 1, 1, 1, false),
            ],
        ];
        for case in cases.span() {
            assert_equivalent(case.span());
        }
    }

    fn level(dynamic_slot: u32, n: u32) -> (Array<BroadPhaseProxy>, Array<BroadPhaseProxy>) {
        let mut statics = array![];
        let mut dynamic = array![];
        let mut i: u32 = 0;
        while i != n {
            let x: i64 = (i % 5).into();
            let y: i64 = (i / 5).into();
            if i == dynamic_slot {
                dynamic.append(proxy(i, 40, 20, 1, 1, false));
            } else {
                statics.append(proxy(i, x * 2, y * 2, 2, 2, true));
            }
            i += 1;
        }
        (statics, dynamic)
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(1_u32);
    }

    /// A level-10 flight: 13 static blocks and the ground, one awake pebble in the air (last
    /// slot), against `find_pairs` on the same 15 proxies.
    #[test]
    fn gas_sparse_flight15() {
        let (statics, dynamic) = level(14, 15);
        let _ = find_pairs_sparse(opaque(statics.span()), opaque(dynamic.span()));
    }

    #[test]
    fn gas_find_pairs_flight15() {
        let (statics, dynamic) = level(14, 15);
        let mut all = statics;
        all.append_span(dynamic.span());
        let _ = find_pairs(opaque(all.span()));
    }

    /// Everything asleep: no dynamic proxy.
    #[test]
    fn gas_sparse_asleep15() {
        let (statics, _) = level(99, 15);
        let _ = find_pairs_sparse(opaque(statics.span()), opaque(array![].span()));
    }
}
