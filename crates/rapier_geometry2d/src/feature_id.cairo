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
    pub const NOT_A_VERTEX: felt252 = 'FeatureId: not a vertex';
    pub const NOT_A_FACE: felt252 = 'FeatureId: not a face';
}

/// A packed feature id; `packed == 0` is the unknown feature.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct FeatureId {
    pub packed: u32,
}

/// The unknown feature (upstream `PackedFeatureId::UNKNOWN`).
pub const FEATURE_UNKNOWN: FeatureId = FeatureId { packed: 0 };

/// Index of a sub-shape of a composite shape (upstream `SubShapeId`); `0` for a shape without
/// sub-shapes, which is every shape of the closed set.
pub type SubShapeId = u32;

/// The unpacked form of a [`FeatureId`] (upstream's `FeatureId` enum; the packed struct carries
/// the upstream name here because it is the frozen interface type). `Edge` is 3D-only.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub enum UnpackedFeatureId {
    Vertex: u32,
    Face: u32,
    #[default]
    Unknown,
}

/// `PackedFeatureId::from(FeatureId)`: packs a vertex or face code (`< 2^30`, as upstream).
/// #### Panics
/// * `'FeatureId: code >= 2^30'`.
pub impl UnpackedFeatureIdIntoFeatureId of Into<UnpackedFeatureId, FeatureId> {
    fn into(self: UnpackedFeatureId) -> FeatureId {
        match self {
            UnpackedFeatureId::Vertex(code) => FeatureIdTrait::vertex(code),
            UnpackedFeatureId::Face(code) => FeatureIdTrait::face(code),
            UnpackedFeatureId::Unknown => FEATURE_UNKNOWN,
        }
    }
}

#[generate_trait]
pub impl FeatureIdImpl of FeatureIdTrait {
    /// The unknown feature (upstream `PackedFeatureId::UNKNOWN`), same as [`FEATURE_UNKNOWN`].
    const UNKNOWN: FeatureId = FEATURE_UNKNOWN;

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

    /// Upstream `PackedFeatureId::unpack`: `Vertex(code)`, `Face(code)`, or `Unknown` for the
    /// zero id and the (3D-only) edge header.
    fn unpack(self: FeatureId) -> UnpackedFeatureId {
        let (header, code) = self.split();
        if header == HEADER_VERTEX {
            UnpackedFeatureId::Vertex(code)
        } else if header == HEADER_FACE {
            UnpackedFeatureId::Face(code)
        } else {
            UnpackedFeatureId::Unknown
        }
    }

    /// The code of a vertex feature (upstream `FeatureId::unwrap_vertex`).
    /// #### Panics
    /// * `'FeatureId: not a vertex'` for a face or the unknown feature.
    fn unwrap_vertex(self: FeatureId) -> u32 {
        let (header, code) = self.split();
        assert(header == HEADER_VERTEX, errors::NOT_A_VERTEX);
        code
    }

    /// The code of a face feature (upstream `FeatureId::unwrap_face`).
    /// #### Panics
    /// * `'FeatureId: not a face'` for a vertex or the unknown feature.
    fn unwrap_face(self: FeatureId) -> u32 {
        let (header, code) = self.split();
        assert(header == HEADER_FACE, errors::NOT_A_FACE);
        code
    }
}

#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use super::{FEATURE_UNKNOWN, FeatureId, FeatureIdTrait, UnpackedFeatureId};

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
    fn test_unpack_pack_and_unwrap() {
        // (packed, unpacked): the edge header (`0b10`, 3D-only) unpacks to `Unknown`, as upstream
        // 2D.
        let cases: Span<(u32, UnpackedFeatureId)> = array![
            (0x4000_000a, UnpackedFeatureId::Vertex(10)), (0xC000_0005, UnpackedFeatureId::Face(5)),
            (0, UnpackedFeatureId::Unknown), (0x8000_0003, UnpackedFeatureId::Unknown),
        ]
            .span();
        for (packed, unpacked) in cases {
            let id = FeatureId { packed: *packed };
            assert_eq!(id.unpack(), *unpacked);
            if *packed != 0x8000_0003 {
                let repacked: FeatureId = (*unpacked).into();
                assert_eq!(repacked, id);
            }
        }
        assert_eq!(FeatureIdTrait::UNKNOWN, FEATURE_UNKNOWN);
        assert_eq!(FeatureIdTrait::vertex(7).unwrap_vertex(), 7);
        assert_eq!(FeatureIdTrait::face(3).unwrap_face(), 3);
        let default: UnpackedFeatureId = Default::default();
        assert_eq!(default, UnpackedFeatureId::Unknown);
    }

    #[test]
    #[should_panic(expected: 'FeatureId: not a vertex')]
    fn test_unwrap_vertex_of_a_face_panics() {
        FeatureIdTrait::face(1).unwrap_vertex();
    }

    #[test]
    #[should_panic(expected: 'FeatureId: not a face')]
    fn test_unwrap_face_of_unknown_panics() {
        FEATURE_UNKNOWN.unwrap_face();
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

    #[test]
    fn gas_unpack() {
        let _ = FeatureId { packed: opaque(0xC000_0005_u32) }.unpack();
    }

    #[test]
    fn gas_unwrap_vertex() {
        let _ = FeatureId { packed: opaque(0x4000_0005_u32) }.unwrap_vertex();
    }

    #[test]
    fn gas_from_unpacked() {
        let _: FeatureId = opaque(UnpackedFeatureId::Face(5)).into();
    }
}
