//! `ColliderPair` (upstream `geometry/broad_phase_pair_event.rs`): two collider handles.
//!
//! The stateless broad phase returns proxy index pairs (`find_pairs`); this is the handle-level
//! pair upstream's broad-phase events and narrow-phase keys carry, for callers that name a pair
//! by its colliders.

use rapier_core::data::handle::Handle;

/// A pair of collider handles. `Default` is [`ColliderPairTrait::zero`], as upstream.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ColliderPair {
    /// The first collider of the pair.
    pub collider1: Handle,
    /// The second collider of the pair.
    pub collider2: Handle,
}

/// Upstream `Default`: [`ColliderPairTrait::zero`].
pub impl ColliderPairDefault of Default<ColliderPair> {
    #[inline(always)]
    fn default() -> ColliderPair {
        ColliderPairTrait::zero()
    }
}

#[generate_trait]
pub impl ColliderPairImpl of ColliderPairTrait {
    /// The pair `(collider1, collider2)`, in that order.
    #[inline(always)]
    fn new(collider1: Handle, collider2: Handle) -> ColliderPair {
        ColliderPair { collider1, collider2 }
    }

    /// The pair with its two handles exchanged.
    #[inline(always)]
    fn swap(self: ColliderPair) -> ColliderPair {
        ColliderPair { collider1: self.collider2, collider2: self.collider1 }
    }

    /// Two artificial handles `(0, 0)`, not guaranteed to resolve.
    #[inline(always)]
    fn zero() -> ColliderPair {
        let zero = Handle { index: 0, generation: 0 };
        ColliderPair { collider1: zero, collider2: zero }
    }
}

#[cfg(test)]
mod tests {
    use rapier_core::data::handle::Handle;
    use rapier_testing::opaque;
    use super::{ColliderPair, ColliderPairTrait};

    fn h(index: u32, generation: u32) -> Handle {
        Handle { index, generation }
    }

    #[test]
    fn test_pair_helpers() {
        // (pair, swapped)
        let cases = array![
            (ColliderPairTrait::new(h(1, 2), h(3, 4)), ColliderPairTrait::new(h(3, 4), h(1, 2))),
            (ColliderPairTrait::zero(), ColliderPairTrait::zero()),
            (ColliderPairTrait::new(h(5, 0), h(5, 0)), ColliderPairTrait::new(h(5, 0), h(5, 0))),
        ];
        for (pair, swapped) in cases {
            assert_eq!(pair.swap(), swapped);
            assert_eq!(pair.swap().swap(), pair);
        }
        let pair = ColliderPairTrait::new(h(1, 2), h(3, 4));
        assert_eq!((pair.collider1, pair.collider2), (h(1, 2), h(3, 4)));
        let default: ColliderPair = Default::default();
        assert_eq!(default, ColliderPair { collider1: h(0, 0), collider2: h(0, 0) });
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(1_u32);
    }

    #[test]
    fn gas_new() {
        let _ = opaque(ColliderPairTrait::new(opaque(h(1, 2)), h(3, 4)));
    }

    #[test]
    fn gas_swap() {
        let _ = opaque(opaque(ColliderPairTrait::new(h(1, 2), h(3, 4))).swap());
    }
}
