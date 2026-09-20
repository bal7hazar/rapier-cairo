//! Packed feature identifiers (Parry's `PackedFeatureId`).
//!
//! A feature id names the vertex or face of a shape that produced a contact point; the narrow
//! phase matches the points of two consecutive manifolds by comparing `(fid1, fid2)` pairs, so the
//! id is kept as a single `u32` and compared as one felt. Layout: a 2-bit header in the top bits
//! (`01` vertex, `11` face, `00` unknown; `10` edge is 3D-only and not used here) and a 30-bit
//! code below. The split is done with `DivRem` by `2^30`, never with shifts.
//!
//! Semantics follow the **f32** build of Parry: the f64 build's cuboid ids are broken upstream
//! (bit 31 of the float is read as a sign) and the golden fixtures carry f32 ids.

use core::num::traits::DivRem;

/// `2^30`: the value of the lowest header bit.
const CODE_SPAN: u32 = 0x4000_0000;
const CODE_SPAN_NZ: NonZero<u32> = 0x4000_0000;
const HEADER_VERTEX: u32 = 1;
const HEADER_FACE: u32 = 3;

pub mod errors {
    pub const CODE_TOO_LARGE: felt252 = 'FeatureId: code >= 2^30';
}

/// A packed feature id; `packed == 0` is the unknown feature.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct FeatureId {
    pub packed: u32,
}

/// The unknown feature (upstream `PackedFeatureId::UNKNOWN`).
pub const FEATURE_UNKNOWN: FeatureId = FeatureId { packed: 0 };

#[generate_trait]
pub impl FeatureIdImpl of FeatureIdTrait {
    /// Vertex feature `code` (`code < 2^30`).
    ///
    /// # Panics
    /// * `'FeatureId: code >= 2^30'`.
    fn vertex(code: u32) -> FeatureId {
        assert(code < CODE_SPAN, errors::CODE_TOO_LARGE);
        FeatureId { packed: HEADER_VERTEX * CODE_SPAN + code }
    }

    /// Face feature `code` (`code < 2^30`). In 2D a face is an edge of the polygon.
    ///
    /// # Panics
    /// * `'FeatureId: code >= 2^30'`.
    fn face(code: u32) -> FeatureId {
        assert(code < CODE_SPAN, errors::CODE_TOO_LARGE);
        FeatureId { packed: HEADER_FACE * CODE_SPAN + code }
    }

    /// The 2-bit header (`0` unknown, `1` vertex, `3` face) and the 30-bit code.
    fn split(self: FeatureId) -> (u32, u32) {
        DivRem::div_rem(self.packed, CODE_SPAN_NZ)
    }

    fn is_vertex(self: FeatureId) -> bool {
        let (header, _) = self.split();
        header == HEADER_VERTEX
    }

    fn is_face(self: FeatureId) -> bool {
        let (header, _) = self.split();
        header == HEADER_FACE
    }

    fn is_unknown(self: FeatureId) -> bool {
        self.packed == 0
    }

    /// The shape-specific code, without the header.
    fn code(self: FeatureId) -> u32 {
        let (_, code) = self.split();
        code
    }
}

#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use super::{FEATURE_UNKNOWN, FeatureId, FeatureIdTrait};

    #[test]
    fn test_pack_unpack_round_trip() {
        let cases: Span<(u32, bool)> = array![
            (0, true), (1, false), (7, true), (0x3fff_ffff, false),
        ]
            .span();
        for (code, vertex) in cases {
            let id = if *vertex {
                FeatureIdTrait::vertex(*code)
            } else {
                FeatureIdTrait::face(*code)
            };
            assert_eq!(id.code(), *code);
            assert_eq!(id.is_vertex(), *vertex);
            assert_eq!(id.is_face(), !*vertex);
            assert!(!id.is_unknown());
        }
    }

    #[test]
    fn test_unknown_and_upstream_encoding() {
        assert!(FEATURE_UNKNOWN.is_unknown());
        assert!(!FEATURE_UNKNOWN.is_vertex());
        assert!(!FEATURE_UNKNOWN.is_face());
        // Upstream: vertex(10).0 == 0x4000_000a, face(5).0 == 0xC000_0005.
        assert_eq!(FeatureIdTrait::vertex(10), FeatureId { packed: 0x4000_000a });
        assert_eq!(FeatureIdTrait::face(5), FeatureId { packed: 0xC000_0005 });
    }

    #[test]
    #[should_panic(expected: 'FeatureId: code >= 2^30')]
    fn test_code_too_large_panics() {
        FeatureIdTrait::vertex(0x4000_0000);
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(10_u32);
    }

    #[test]
    fn gas_vertex() {
        let _ = FeatureIdTrait::vertex(opaque(10_u32));
    }

    #[test]
    fn gas_is_face() {
        let _ = FeatureId { packed: opaque(0xC000_0005_u32) }.is_face();
    }

    #[test]
    fn gas_eq() {
        let a = FeatureId { packed: opaque(0xC000_0005_u32) };
        let b = FeatureId { packed: opaque(0x4000_0005_u32) };
        let _ = a == b;
    }
}
